/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Display Relay — Central (dongle) side
 * ======================================
 * When enabled, the central:
 *   1. Registers BT connection callbacks.
 *   2. On peripheral connect, discovers the battery relay GATT characteristic
 *      (layer data is multiplexed through it).
 *   3. Subscribes to ZMK battery and layer events.
 *   4. On every battery change, writes struct battery_relay_data to each
 *      connected peripheral that has been successfully discovered.
 *   5. On every layer change, writes the layer index to each
 *      connected peripheral via the battery relay characteristic.
 *   6. Caches battery levels and layer index so newly-discovered peripherals
 *      get the current state immediately after GATT discovery completes.
 *   7. Periodically re-broadcasts battery state for resilience against
 *      dropped BLE writes.
 *
 * IMPORTANT: All bt_gatt_write_without_response calls are serialized through
 * a dedicated writer thread.  This function can block waiting for BT TX
 * buffers — if called directly from the system work queue, it deadlocks
 * because the BT stack needs the system work queue for TX completion.
 *
 * This allows peripherals with displays (e.g. nice_view_gem) to show battery
 * levels for all keyboard splits and the current active layer.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

#include <zmk/events/battery_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/battery.h>
#include <zmk/keymap.h>

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* -------------------------------------------------------------------------
 * Cached state — sent to peripherals after GATT discovery completes and
 * periodically re-broadcast for resilience.
 * ---------------------------------------------------------------------- */

static uint8_t battery_cache[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];
static uint8_t layer_cache;

/* Diagnostic counters — read from dongle display for debugging */
volatile uint32_t relay_diag_conn_count;      /* relay_connected calls */
volatile uint32_t relay_diag_disc_start;      /* discovery attempts */
volatile uint32_t relay_diag_disc_ok;         /* discovery successes */
volatile uint32_t relay_diag_disc_fail;       /* discovery not-found */
volatile uint32_t relay_diag_disc_err;        /* bt_gatt_discover errors */
volatile uint32_t relay_diag_write_ok;        /* successful writes */
volatile uint32_t relay_diag_write_err;       /* failed writes */

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
static uint8_t dongle_battery_cache;
#endif

/* -------------------------------------------------------------------------
 * Per-peripheral relay state
 * ---------------------------------------------------------------------- */

struct peripheral_relay {
    struct bt_conn *conn;

    /* Battery relay GATT characteristic (also used for layer data) */
    uint16_t bat_char_handle;
    bool bat_ready;

    /* Layer relay — multiplexed through the battery relay characteristic.
     * layer_ready is set after discovery completes so layer writes can begin. */
    bool layer_ready;

    /* Discovery state */
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128 discover_uuid; /* copy kept alive for async discovery */
    struct k_work_delayable discovery_work;
    struct k_work_delayable watchdog_work; /* fires if discovery callback never comes */
    bool discovery_in_progress;      /* true between bt_gatt_discover and callback */
    uint8_t discover_retries;        /* remaining retries on transient errors */
};

/* Delay before starting GATT discovery after connection.
 * ZMK's split stack also does GATT discovery on connect;
 * starting a concurrent discovery causes -EBUSY.
 * 2000 ms gives ZMK time to complete its own setup first. */
#define RELAY_DISCOVERY_DELAY_MS 5000

/* Extra stagger per relay-slot index so peripherals that connect
 * simultaneously don't start GATT discovery at the same time. */
#define RELAY_DISCOVERY_STAGGER_MS 3000

/* How many times to retry bt_gatt_discover on transient errors. */
#define RELAY_DISCOVERY_MAX_RETRIES 5

/* Delay between discovery retries (ms). */
#define RELAY_DISCOVERY_RETRY_DELAY_MS 3000

/* Watchdog: if discovery callback never fires, retry after this (ms). */
#define RELAY_DISCOVERY_WATCHDOG_MS 8000

/* How often to re-broadcast cached battery state (ms). */
#define RELAY_PERIODIC_BROADCAST_MS 60000

static struct peripheral_relay relays[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

static struct peripheral_relay *get_relay_by_conn(struct bt_conn *conn) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == conn) {
            return &relays[i];
        }
    }
    return NULL;
}

static struct peripheral_relay *get_free_relay(void) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == NULL) {
            return &relays[i];
        }
    }
    return NULL;
}

static int get_relay_index(const struct peripheral_relay *relay) {
    return (int)(relay - relays);
}

/* -------------------------------------------------------------------------
 * Serialized write queue
 *
 * bt_gatt_write_without_response can block waiting for BT TX buffers.
 * If called from the system work queue, this deadlocks because the BT
 * stack needs the system work queue for TX completion callbacks.
 *
 * Solution: a message queue + dedicated writer thread.  Callers enqueue
 * write requests (non-blocking), and the writer thread sends them one
 * at a time.  Blocking on TX buffers is safe here because it doesn't
 * hold up the system work queue.
 * ---------------------------------------------------------------------- */

struct relay_write_msg {
    uint8_t relay_idx;    /* index into relays[] */
    struct battery_relay_data data;
};

#define RELAY_WRITE_QUEUE_SIZE 16
K_MSGQ_DEFINE(relay_write_msgq, sizeof(struct relay_write_msg), RELAY_WRITE_QUEUE_SIZE, 4);

#define RELAY_WRITER_STACK_SIZE 1024
#define RELAY_WRITER_PRIORITY   K_PRIO_PREEMPT(8)

K_THREAD_STACK_DEFINE(relay_writer_stack, RELAY_WRITER_STACK_SIZE);
static struct k_thread relay_writer_thread;

static void relay_writer_func(void *p1, void *p2, void *p3) {
    struct relay_write_msg msg;

    while (true) {
        /* Block until a write request is available */
        k_msgq_get(&relay_write_msgq, &msg, K_FOREVER);

        struct peripheral_relay *relay = &relays[msg.relay_idx];
        if (relay->conn == NULL) {
            continue;
        }

        /* This call may block waiting for TX buffers — that's OK
         * because we're on a dedicated thread, not the system work queue. */
        int err = bt_gatt_write_without_response(relay->conn, relay->bat_char_handle,
                                                  &msg.data, sizeof(msg.data), false);
        if (err) {
            relay_diag_write_err++;
            LOG_WRN("relay_writer: write failed: %d (relay %u)", err, msg.relay_idx);
        } else {
            relay_diag_write_ok++;
        }
    }
}

/* -------------------------------------------------------------------------
 * Write helpers — enqueue to writer thread instead of writing directly
 * ---------------------------------------------------------------------- */

static void write_battery_to_relay(struct peripheral_relay *relay, uint8_t source, uint8_t level) {
    if (!relay->bat_ready || relay->conn == NULL) {
        return;
    }
    struct relay_write_msg msg = {
        .relay_idx = (uint8_t)get_relay_index(relay),
        .data = { .source = source, .level = level },
    };
    if (k_msgq_put(&relay_write_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("relay: write queue full, dropping write (source=%u)", source);
    }
}

static void write_layer_to_relay(struct peripheral_relay *relay, uint8_t layer) {
    if (!relay->layer_ready || relay->conn == NULL) {
        return;
    }
    struct relay_write_msg msg = {
        .relay_idx = (uint8_t)get_relay_index(relay),
        .data = { .source = BATTERY_RELAY_SOURCE_LAYER, .level = layer },
    };
    if (k_msgq_put(&relay_write_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("relay: write queue full, dropping layer write");
    }
}

static void broadcast_battery(uint8_t source, uint8_t level) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        write_battery_to_relay(&relays[i], source, level);
    }
}

static void broadcast_layer(uint8_t layer) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        write_layer_to_relay(&relays[i], layer);
    }
}

/** Push all cached state to a single peripheral after discovery completes. */
static void push_cached_state(struct peripheral_relay *relay) {
    /* Push all cached battery levels */
    for (int i = 0; i < ARRAY_SIZE(battery_cache); i++) {
        if (battery_cache[i] > 0) {
            write_battery_to_relay(relay, (uint8_t)i, battery_cache[i]);
        }
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (dongle_battery_cache > 0) {
        write_battery_to_relay(relay, BATTERY_RELAY_SOURCE_DONGLE, dongle_battery_cache);
    }
#endif

    /* Push current layer */
    write_layer_to_relay(relay, layer_cache);
}

/* -------------------------------------------------------------------------
 * GATT discovery — battery characteristic only
 *
 * Layer data is multiplexed through the battery relay characteristic,
 * so no separate layer discovery is needed.  After battery discovery
 * completes we mark layer_ready and push cached state.
 * ---------------------------------------------------------------------- */

static uint8_t battery_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay, discover_params);
    relay->discovery_in_progress = false;
    k_work_cancel_delayable(&relay->watchdog_work);

    if (!attr) {
        if (!relay->bat_ready) {
            relay_diag_disc_fail++;
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
            /* Retry with delay if retries remain, otherwise give up */
            if (relay->discover_retries > 0) {
                relay->discover_retries--;
                k_work_schedule(&relay->discovery_work,
                                K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS));
            } else {
                LOG_ERR("battery_relay: giving up discovery on conn %p", (void *)conn);
            }
        } else {
            /* Discovery succeeded — proceed to mark layer_ready */
            k_work_schedule(&relay->discovery_work, K_NO_WAIT);
        }
        return BT_GATT_ITER_STOP;
    }

    relay_diag_disc_ok++;
    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready = true;
    LOG_INF("battery_relay: characteristic found, handle=%u", relay->bat_char_handle);

    /* Schedule via work queue so current GATT discovery fully completes first. */
    k_work_schedule(&relay->discovery_work, K_NO_WAIT);

    return BT_GATT_ITER_STOP;
}

static void start_battery_discovery(struct peripheral_relay *relay) {
    relay_diag_disc_start++;
    memcpy(&relay->discover_uuid, BATTERY_RELAY_CHAR_UUID, sizeof(relay->discover_uuid));

    relay->discover_params.uuid = &relay->discover_uuid.uuid;
    relay->discover_params.func = battery_discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    relay->discovery_in_progress = true;
    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        relay->discovery_in_progress = false;
        relay_diag_disc_err++;
        LOG_ERR("battery_relay: bt_gatt_discover failed: %d (retries left %u)",
                err, relay->discover_retries);
        /* Retry on transient errors */
        if (relay->discover_retries > 0 &&
            (err == -EBUSY || err == -ENOMEM || err == -EAGAIN)) {
            relay->discover_retries--;
            k_work_schedule(&relay->discovery_work,
                            K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS));
        }
    } else {
        /* Arm watchdog — if callback never fires, retry discovery */
        k_work_schedule(&relay->watchdog_work,
                        K_MSEC(RELAY_DISCOVERY_WATCHDOG_MS));
    }
}

static void discovery_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay, discovery_work);
    if (relay->conn == NULL) {
        return;
    }
    if (!relay->bat_ready) {
        start_battery_discovery(relay);
    } else if (!relay->layer_ready) {
        /* Layer is multiplexed through the battery characteristic,
         * so no separate discovery needed — just mark ready. */
        relay->layer_ready = true;
        push_cached_state(relay);
    }
}

static void watchdog_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay, watchdog_work);
    if (relay->conn == NULL || relay->bat_ready) {
        return;
    }
    /* Discovery callback never fired — retry if retries remain */
    LOG_WRN("battery_relay: watchdog fired, discovery callback never came (retries %u)",
            relay->discover_retries);
    relay->discovery_in_progress = false;
    if (relay->discover_retries > 0) {
        relay->discover_retries--;
        k_work_schedule(&relay->discovery_work, K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS));
    } else {
        LOG_ERR("battery_relay: giving up after watchdog exhausted retries");
    }
}

/* -------------------------------------------------------------------------
 * Periodic re-broadcast of cached battery state
 * ---------------------------------------------------------------------- */

static void periodic_broadcast_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(periodic_broadcast_work, periodic_broadcast_handler);

static void periodic_broadcast_handler(struct k_work *work) {
    for (int src = 0; src < ARRAY_SIZE(battery_cache); src++) {
        if (battery_cache[src] > 0) {
            broadcast_battery((uint8_t)src, battery_cache[src]);
        }
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (dongle_battery_cache > 0) {
        broadcast_battery(BATTERY_RELAY_SOURCE_DONGLE, dongle_battery_cache);
    }
#endif

    /* Rebroadcast layer — BLE write-without-response can drop packets,
     * so periodic resend ensures peripherals stay in sync. */
    broadcast_layer(layer_cache);

    /* Reschedule */
    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
}

/* -------------------------------------------------------------------------
 * BT connection callbacks
 * ---------------------------------------------------------------------- */

static void relay_connected(struct bt_conn *conn, uint8_t conn_err) {
    if (conn_err) {
        return;
    }

    /* Only handle LE connections (split peripherals are always LE) */
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) {
        return;
    }
    /* Only handle central-role connections (to split peripherals).
     * Skip host BLE connections where the dongle is a peripheral. */
    if (info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }

    relay_diag_conn_count++;

    struct peripheral_relay *relay = get_free_relay();
    if (!relay) {
        LOG_WRN("battery_relay: no free relay slot for new connection");
        return;
    }

    relay->conn = bt_conn_ref(conn);
    relay->bat_ready = false;
    relay->bat_char_handle = 0;
    relay->layer_ready = false;
    relay->discovery_in_progress = false;
    relay->discover_retries = RELAY_DISCOVERY_MAX_RETRIES;

    k_work_init_delayable(&relay->discovery_work, discovery_work_handler);
    k_work_init_delayable(&relay->watchdog_work, watchdog_work_handler);

    /* Stagger discovery start per relay index so that peripherals
     * connecting at the same time don't start concurrent GATT
     * discoveries that compete for ATT buffers. */
    uint32_t delay = RELAY_DISCOVERY_DELAY_MS +
                     (uint32_t)get_relay_index(relay) * RELAY_DISCOVERY_STAGGER_MS;
    k_work_schedule(&relay->discovery_work, K_MSEC(delay));

    /* Start the periodic rebroadcast timer the first time a peripheral connects.
     * k_work_schedule is a no-op if the work is already pending, so calling it on
     * every connection is safe and ensures the timer is always running. */
    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) {
        return;
    }

    LOG_DBG("relay: peripheral disconnected (reason %u), clearing slot", reason);
    k_work_cancel_delayable(&relay->discovery_work);
    k_work_cancel_delayable(&relay->watchdog_work);
    bt_conn_unref(relay->conn);
    relay->conn = NULL;
    relay->bat_ready = false;
    relay->bat_char_handle = 0;
    relay->layer_ready = false;
    relay->discovery_in_progress = false;
}

BT_CONN_CB_DEFINE(battery_relay_conn_cb) = {
    .connected = relay_connected,
    .disconnected = relay_disconnected,
};

/* -------------------------------------------------------------------------
 * ZMK event listener — battery + layer changes
 * ---------------------------------------------------------------------- */

static int relay_central_event_handler(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev) {
        LOG_INF("relay_central: peripheral battery event source=%u level=%u",
                periph_ev->source, periph_ev->state_of_charge);
        /* Cache before broadcasting */
        if (periph_ev->source < ARRAY_SIZE(battery_cache)) {
            battery_cache[periph_ev->source] = periph_ev->state_of_charge;
        }
        broadcast_battery(periph_ev->source, periph_ev->state_of_charge);
        return ZMK_EV_EVENT_BUBBLE;
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    const struct zmk_battery_state_changed *bat_ev = as_zmk_battery_state_changed(eh);
    if (bat_ev) {
        dongle_battery_cache = bat_ev->state_of_charge;
        broadcast_battery(BATTERY_RELAY_SOURCE_DONGLE, bat_ev->state_of_charge);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    const struct zmk_layer_state_changed *layer_ev = as_zmk_layer_state_changed(eh);
    if (layer_ev) {
        uint8_t highest = zmk_keymap_highest_layer_active();
        layer_cache = highest;
        broadcast_layer(highest);
        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(battery_relay_central, relay_central_event_handler);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_peripheral_battery_state_changed);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_layer_state_changed);

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
ZMK_SUBSCRIPTION(battery_relay_central, zmk_battery_state_changed);
#endif

/* -------------------------------------------------------------------------
 * Writer thread initialization
 * ---------------------------------------------------------------------- */

static int relay_writer_init(void) {
    k_thread_create(&relay_writer_thread, relay_writer_stack,
                    K_THREAD_STACK_SIZEOF(relay_writer_stack),
                    relay_writer_func, NULL, NULL, NULL,
                    RELAY_WRITER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&relay_writer_thread, "relay_writer");
    return 0;
}

SYS_INIT(relay_writer_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
