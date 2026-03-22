/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Display Relay — Central (dongle) side
 * ======================================
 * When enabled, the central:
 *   1. Registers BT connection callbacks.
 *   2. On peripheral connect, discovers the battery relay and layer relay
 *      GATT characteristics after a long delay (30 s) to avoid interfering
 *      with ZMK's split stack setup.
 *   3. Subscribes to ZMK battery and layer events, caching values locally.
 *   4. Periodically writes cached battery and layer state to discovered
 *      peripherals (every 60 s).  Event-driven writes are intentionally
 *      avoided — every GATT write consumes a shared BLE TX buffer that
 *      ZMK also needs for HID reports, trackpoint data, etc.
 *
 * IMPORTANT: All BLE GATT operations (discovery, writes) run on a dedicated
 * relay_work_q — NEVER on the system work queue.  bt_gatt_write_without_response
 * can block waiting for TX buffers; if that happens on the system work queue
 * it deadlocks the BLE stack (which needs the work queue to free TX buffers).
 *
 * This allows peripherals with displays (e.g. nice_view_gem) to show battery
 * levels for all keyboard splits and the current active layer.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

#include <zmk/split/central.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/battery.h>
#include <zmk/keymap.h>

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* -------------------------------------------------------------------------
 * Dedicated work queue for relay GATT operations.
 *
 * bt_gatt_write_without_response() can block waiting for BLE TX credits.
 * Running on the system work queue would deadlock the BLE stack, so all
 * relay writes and discovery are dispatched here instead.
 * ---------------------------------------------------------------------- */

#define RELAY_WORK_Q_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(relay_work_q_stack, RELAY_WORK_Q_STACK_SIZE);
static struct k_work_q relay_work_q;

/* -------------------------------------------------------------------------
 * Cached state — updated by ZMK event listeners, written to peripherals
 * only during the periodic broadcast (never inline from event handlers).
 * ---------------------------------------------------------------------- */

static uint8_t battery_cache[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];
static uint8_t layer_cache;

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
static uint8_t dongle_battery_cache;
#endif

/* -------------------------------------------------------------------------
 * Per-peripheral relay state
 * ---------------------------------------------------------------------- */

struct peripheral_relay {
    struct bt_conn *conn;

    /* Battery relay GATT characteristic */
    uint16_t bat_char_handle;
    bool bat_ready;          /* characteristic found and usable for writes */
    bool bat_discovery_done; /* battery discovery has been attempted */

    /* Layer relay GATT characteristic */
    uint16_t layer_char_handle;
    bool layer_ready;

    /* Discovery state */
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128 discover_uuid; /* copy kept alive for async discovery */
    struct k_work_delayable discovery_work;
    uint8_t discovery_retries;
};

/* Delay before starting GATT discovery after connection.
 * ZMK's split stack does GATT discovery, notification subscription,
 * security establishment, and connection parameter negotiation on
 * connect.  On reconnection after sleep these can take significantly
 * longer.  30 s ensures ZMK's split stack is fully stable before we
 * touch the ATT channel (Zephyr allows only one outstanding ATT
 * request per connection — our discovery blocks everything else). */
#define RELAY_DISCOVERY_DELAY_MS 30000

/* How often to write cached battery state to peripherals (ms).
 * Battery changes slowly — periodic broadcast is sufficient. */
#define RELAY_PERIODIC_BROADCAST_MS 60000

/* Retry parameters for GATT discovery when it fails (e.g. -EBUSY). */
#define MAX_DISCOVERY_RETRIES 5
#define DISCOVERY_RETRY_BASE_MS 2000

/* Delay between battery discovery completing and layer discovery starting.
 * This gap lets ZMK's split stack (or any other ATT client) perform
 * operations on the connection without being blocked by our discovery. */
#define RELAY_INTER_DISCOVERY_DELAY_MS 5000

/* Delay between individual GATT writes during periodic broadcast.
 * Spaces out writes to avoid exhausting BLE TX buffers in a burst. */
#define RELAY_WRITE_SPACING_MS 100

/* Minimum interval between layer broadcasts (ms).  Momentary layer taps
 * generate rapid activate/deactivate pairs; debouncing avoids flooding
 * the BLE TX buffers which can starve ZMK's split communication. */
#define LAYER_BROADCAST_DEBOUNCE_MS 100

static struct peripheral_relay relays[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

/* Debounced layer broadcast — runs on relay_work_q */
static void layer_broadcast_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(layer_broadcast_work, layer_broadcast_work_handler);

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

/* -------------------------------------------------------------------------
 * Write helpers — only called from relay_work_q context
 * ---------------------------------------------------------------------- */

static void write_battery_to_relay(struct peripheral_relay *relay, uint8_t source, uint8_t level) {
    if (!relay->bat_ready || relay->conn == NULL) {
        return;
    }
    struct battery_relay_data data = { .source = source, .level = level };
    int err = bt_gatt_write_without_response(relay->conn, relay->bat_char_handle,
                                             &data, sizeof(data), false);
    if (err) {
        LOG_WRN("battery_relay: write failed: %d", err);
        if (err == -ENOTCONN) {
            relay->bat_ready = false;
            relay->layer_ready = false;
        }
    }
}

static void write_layer_to_relay(struct peripheral_relay *relay, uint8_t layer) {
    if (!relay->layer_ready || relay->conn == NULL) {
        return;
    }
    struct layer_relay_data data = { .layer = layer };
    int err = bt_gatt_write_without_response(relay->conn, relay->layer_char_handle,
                                             &data, sizeof(data), false);
    if (err) {
        LOG_WRN("layer_relay: write failed: %d", err);
        if (err == -ENOTCONN) {
            relay->bat_ready = false;
            relay->layer_ready = false;
        }
    }
}

/* Debounced layer broadcast: writes the latest cached layer to all relays.
 * Runs on relay_work_q — safe to block on TX credits without affecting ZMK. */
static void layer_broadcast_work_handler(struct k_work *work) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        write_layer_to_relay(&relays[i], layer_cache);
    }
}

/* -------------------------------------------------------------------------
 * GATT discovery — battery first, then layer (chained)
 *
 * IMPORTANT: Discovery callbacks run in BT RX thread context.  Never call
 * bt_gatt_discover() directly from within a callback — always defer to
 * relay_work_q via discovery_work to avoid re-entering the GATT stack.
 * ---------------------------------------------------------------------- */

static void start_layer_discovery(struct peripheral_relay *relay);

/** Schedule a discovery retry with exponential backoff. */
static void schedule_discovery_retry(struct peripheral_relay *relay, const char *phase) {
    if (relay->discovery_retries < MAX_DISCOVERY_RETRIES) {
        uint32_t delay = DISCOVERY_RETRY_BASE_MS *
                         (1U << MIN(relay->discovery_retries, 3));
        relay->discovery_retries++;
        LOG_INF("%s: retry %u/%d in %u ms", phase,
                relay->discovery_retries, MAX_DISCOVERY_RETRIES, delay);
        k_work_schedule_for_queue(&relay_work_q, &relay->discovery_work,
                                  K_MSEC(delay));
    } else {
        LOG_ERR("%s: giving up after %d retries", phase, MAX_DISCOVERY_RETRIES);
    }
}

static uint8_t layer_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay, discover_params);

    if (!attr) {
        if (!relay->layer_ready) {
            LOG_DBG("layer_relay: characteristic not found on conn %p", (void *)conn);
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->layer_char_handle = chrc->value_handle;
    relay->layer_ready = true;
    LOG_INF("layer_relay: characteristic found, handle=%u", relay->layer_char_handle);

    return BT_GATT_ITER_STOP;
}

static void start_layer_discovery(struct peripheral_relay *relay) {
    if (relay->conn == NULL) {
        return;
    }

    memcpy(&relay->discover_uuid, LAYER_RELAY_CHAR_UUID, sizeof(relay->discover_uuid));

    relay->discover_params.uuid = &relay->discover_uuid.uuid;
    relay->discover_params.func = layer_discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        LOG_WRN("layer_relay: bt_gatt_discover failed: %d", err);
        schedule_discovery_retry(relay, "layer_relay");
    }
}

static uint8_t battery_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay, discover_params);

    if (!attr) {
        if (!relay->bat_ready) {
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
        }
        /* Mark battery discovery as attempted.  Always defer next step
         * via work queue — never call bt_gatt_discover from a callback.
         *
         * If battery relay was not found, skip layer discovery — the
         * peripheral doesn't have relay services. */
        relay->bat_discovery_done = true;
        if (relay->bat_ready) {
            k_work_schedule_for_queue(&relay_work_q, &relay->discovery_work,
                                      K_MSEC(RELAY_INTER_DISCOVERY_DELAY_MS));
        } else {
            LOG_DBG("battery_relay: skipping layer discovery — no battery "
                     "relay on this peripheral");
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready = true;
    relay->bat_discovery_done = true;
    LOG_INF("battery_relay: characteristic found, handle=%u", relay->bat_char_handle);

    /* Chain: discover layer relay characteristic next.
     * Schedule via work queue with a generous delay so the ATT channel
     * is free for ZMK's split stack between our discovery phases. */
    k_work_schedule_for_queue(&relay_work_q, &relay->discovery_work,
                              K_MSEC(RELAY_INTER_DISCOVERY_DELAY_MS));

    return BT_GATT_ITER_STOP;
}

static void start_battery_discovery(struct peripheral_relay *relay) {
    if (relay->conn == NULL) {
        return;
    }

    memcpy(&relay->discover_uuid, BATTERY_RELAY_CHAR_UUID, sizeof(relay->discover_uuid));

    relay->discover_params.uuid = &relay->discover_uuid.uuid;
    relay->discover_params.func = battery_discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        LOG_WRN("battery_relay: bt_gatt_discover failed: %d", err);
        schedule_discovery_retry(relay, "battery_relay");
    }
}

static void discovery_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay, discovery_work);
    if (relay->conn == NULL) {
        return;
    }
    if (!relay->bat_discovery_done) {
        start_battery_discovery(relay);
    } else if (!relay->layer_ready) {
        start_layer_discovery(relay);
    }
}

/* -------------------------------------------------------------------------
 * Periodic broadcast — writes cached battery + layer state to peripherals.
 *
 * Battery is only delivered here (every 60 s).  Layer is also sent here
 * as a fallback, but primarily delivered by the debounced layer_broadcast_work.
 * Writes are spaced by RELAY_WRITE_SPACING_MS to avoid exhausting BLE TX
 * buffers that ZMK needs for HID reports / trackpoint data.
 * ---------------------------------------------------------------------- */

static void periodic_broadcast_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(periodic_broadcast_work, periodic_broadcast_handler);

static void periodic_broadcast_handler(struct k_work *work) {
    for (int r = 0; r < ARRAY_SIZE(relays); r++) {
        struct peripheral_relay *relay = &relays[r];

        /* Write cached battery levels */
        for (int src = 0; src < ARRAY_SIZE(battery_cache); src++) {
            if (battery_cache[src] > 0) {
                write_battery_to_relay(relay, (uint8_t)src, battery_cache[src]);
                k_msleep(RELAY_WRITE_SPACING_MS);
            }
        }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
        if (dongle_battery_cache > 0) {
            write_battery_to_relay(relay, BATTERY_RELAY_SOURCE_DONGLE,
                                   dongle_battery_cache);
            k_msleep(RELAY_WRITE_SPACING_MS);
        }
#endif

        /* Write cached layer */
        write_layer_to_relay(relay, layer_cache);
        k_msleep(RELAY_WRITE_SPACING_MS);
    }

    /* Reschedule */
    k_work_schedule_for_queue(&relay_work_q, &periodic_broadcast_work,
                              K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
}

/* -------------------------------------------------------------------------
 * BT connection callbacks
 * ---------------------------------------------------------------------- */

static void relay_connected(struct bt_conn *conn, uint8_t conn_err) {
    if (conn_err) {
        return;
    }

    /* Only handle connections where we are the central (split peripherals).
     * Ignore connections where we are the peripheral (e.g. BLE HID to host). */
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) {
        return;
    }
    if (info.role != BT_HCI_ROLE_CENTRAL) {
        return;
    }

    struct peripheral_relay *relay = get_free_relay();
    if (!relay) {
        LOG_WRN("battery_relay: no free relay slot for new connection");
        return;
    }

    relay->conn = bt_conn_ref(conn);
    relay->bat_ready = false;
    relay->bat_discovery_done = false;
    relay->bat_char_handle = 0;
    relay->layer_ready = false;
    relay->layer_char_handle = 0;
    relay->discovery_retries = 0;

    k_work_init_delayable(&relay->discovery_work, discovery_work_handler);
    k_work_schedule_for_queue(&relay_work_q, &relay->discovery_work,
                              K_MSEC(RELAY_DISCOVERY_DELAY_MS));
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) {
        return;
    }

    LOG_DBG("relay: peripheral disconnected (reason %u), clearing slot", reason);
    k_work_cancel_delayable(&relay->discovery_work);
    bt_conn_unref(relay->conn);
    relay->conn = NULL;
    relay->bat_ready = false;
    relay->bat_discovery_done = false;
    relay->bat_char_handle = 0;
    relay->layer_ready = false;
    relay->layer_char_handle = 0;
    relay->discovery_retries = 0;
}

BT_CONN_CB_DEFINE(battery_relay_conn_cb) = {
    .connected = relay_connected,
    .disconnected = relay_disconnected,
};

/* -------------------------------------------------------------------------
 * ZMK event listener
 *
 * Battery: cache only — periodic broadcast handles delivery (60 s).
 * Layer:   cache + debounced write — layer changes are user-visible and
 *          tied to keypresses, so they need prompt delivery.  The write
 *          is deferred to relay_work_q so ZMK's event pipeline never
 *          blocks on BLE TX.  One small write per layer change is
 *          acceptable TX buffer pressure.
 * ---------------------------------------------------------------------- */

static int relay_central_event_handler(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev) {
        if (periph_ev->source < ARRAY_SIZE(battery_cache)) {
            battery_cache[periph_ev->source] = periph_ev->state_of_charge;
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    const struct zmk_battery_state_changed *bat_ev = as_zmk_battery_state_changed(eh);
    if (bat_ev) {
        dongle_battery_cache = bat_ev->state_of_charge;
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    const struct zmk_layer_state_changed *layer_ev = as_zmk_layer_state_changed(eh);
    if (layer_ev) {
        layer_cache = zmk_keymap_highest_layer_active();
        /* Debounce: rapid layer activate/deactivate pairs produce one write */
        k_work_schedule_for_queue(&relay_work_q, &layer_broadcast_work,
                                  K_MSEC(LAYER_BROADCAST_DEBOUNCE_MS));
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
 * Init — start dedicated work queue and periodic rebroadcast timer
 * ---------------------------------------------------------------------- */

static int relay_central_init(void) {
    k_work_queue_start(&relay_work_q, relay_work_q_stack,
                       K_THREAD_STACK_SIZEOF(relay_work_q_stack),
                       K_PRIO_PREEMPT(10), /* low priority — never starve ZMK */
                       NULL);

    k_work_schedule_for_queue(&relay_work_q, &periodic_broadcast_work,
                              K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
    return 0;
}

SYS_INIT(relay_central_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
