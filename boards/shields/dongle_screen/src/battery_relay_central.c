/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Display Relay — Central (dongle) side
 * ======================================
 * When enabled, the central:
 *   1. Registers BT connection callbacks to track peripheral connections.
 *   2. Subscribes to ZMK battery and layer events, caching values locally.
 *   3. Periodically (every 60 s) attempts GATT discovery on any connected
 *      peripheral whose relay handle is not yet known, then writes
 *      cached state to all discovered peripherals.
 *
 * DESIGN PRINCIPLE: Never initiate GATT operations at connection time.
 * ZMK's split stack does extensive GATT work on connect (service discovery,
 * HID notification subscription, security, connection parameter negotiation).
 * Zephyr allows only one outstanding ATT request per connection — any GATT
 * client operation we start can collide with ZMK's and break key forwarding.
 *
 * Instead, all discovery and writes happen lazily from the periodic handler
 * on a dedicated low-priority work queue.  If discovery fails (-EBUSY
 * because ZMK is doing a battery read), we simply try again next cycle.
 *
 * Both battery and layer data are multiplexed through the single battery
 * relay GATT characteristic.  Layer data uses source=BATTERY_RELAY_SOURCE_LAYER
 * with the layer index in the level field.  This avoids needing a separate
 * layer characteristic and eliminates a second GATT discovery operation.
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

    /* Battery relay GATT characteristic — also used for layer data */
    uint16_t bat_char_handle;
    bool bat_ready;          /* characteristic found and usable for writes */
    bool bat_discovery_done; /* discovery has been attempted (won't retry) */

    /* Discovery state — used by the periodic handler, one relay at a time */
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128 discover_uuid; /* copy kept alive for async discovery */

    /* Set true while an async bt_gatt_discover is in flight */
    bool discovery_in_flight;
};

/* How often to run the periodic handler (ms).  Each cycle:
 * 1. Attempts discovery on ONE undiscovered peripheral (if any).
 * 2. Writes cached battery + layer state to all discovered peripherals.
 * Layer changes are also sent promptly via debounced event-driven writes. */
#define RELAY_PERIODIC_MS 60000

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
 *
 * All data (battery AND layer) goes through the single battery relay
 * characteristic.  Layer data is encoded as source=BATTERY_RELAY_SOURCE_LAYER,
 * level=layer_index.
 * ---------------------------------------------------------------------- */

static void write_to_relay(struct peripheral_relay *relay, uint8_t source, uint8_t level) {
    if (!relay->bat_ready || relay->conn == NULL) {
        return;
    }
    struct battery_relay_data data = { .source = source, .level = level };
    int err = bt_gatt_write_without_response(relay->conn, relay->bat_char_handle,
                                             &data, sizeof(data), false);
    if (err) {
        LOG_WRN("relay: write failed (source=%u): %d", source, err);
        if (err == -ENOTCONN) {
            relay->bat_ready = false;
        }
    }
}

/* Debounced layer broadcast: writes the latest cached layer to all relays.
 * Runs on relay_work_q — safe to block on TX credits without affecting ZMK. */
static void layer_broadcast_work_handler(struct k_work *work) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        write_to_relay(&relays[i], BATTERY_RELAY_SOURCE_LAYER, layer_cache);
    }
}

/* -------------------------------------------------------------------------
 * GATT discovery — called from periodic handler, one relay at a time.
 *
 * Only ONE discovery is needed per peripheral: the battery relay
 * characteristic.  Layer data is multiplexed through the same characteristic,
 * so no separate layer discovery is required.
 *
 * Discovery callbacks run in BT RX thread context.  They just record the
 * result and clear the in_flight flag — no further GATT calls are made
 * from within callbacks.
 * ---------------------------------------------------------------------- */

static uint8_t battery_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay, discover_params);
    relay->discovery_in_flight = false;
    relay->bat_discovery_done = true;

    if (!attr) {
        LOG_DBG("relay: characteristic not found on conn %p", (void *)conn);
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready = true;
    LOG_INF("relay: characteristic found, handle=%u", relay->bat_char_handle);

    return BT_GATT_ITER_STOP;
}

/** Try discovery on a relay that needs it.
 *  Returns true if a discovery was started (caller should not start another). */
static bool try_discovery_step(struct peripheral_relay *relay) {
    if (relay->conn == NULL || relay->discovery_in_flight || relay->bat_discovery_done) {
        return false;
    }

    memcpy(&relay->discover_uuid, BATTERY_RELAY_CHAR_UUID, sizeof(relay->discover_uuid));
    relay->discover_params.uuid = &relay->discover_uuid.uuid;
    relay->discover_params.func = battery_discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        LOG_DBG("relay: discovery returned %d, will retry next cycle", err);
        return false;
    }
    relay->discovery_in_flight = true;
    return true;
}

/* -------------------------------------------------------------------------
 * Periodic handler — discovery + broadcast, all in one place.
 *
 * Runs on relay_work_q every RELAY_PERIODIC_MS.  Each cycle:
 * 1. Tries ONE discovery on the first relay that needs it (at most
 *    one bt_gatt_discover per cycle to minimize ATT channel disruption).
 * 2. Writes cached battery + layer state to all discovered relays with
 *    spacing between writes to avoid TX buffer exhaustion.
 * ---------------------------------------------------------------------- */

static void periodic_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(periodic_work, periodic_handler);

static void periodic_handler(struct k_work *work) {
    /* Step 1: Try discovery on one undiscovered relay */
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (try_discovery_step(&relays[i])) {
            break; /* At most one discovery per cycle */
        }
    }

    /* Step 2: Write cached state to all discovered relays */
    for (int r = 0; r < ARRAY_SIZE(relays); r++) {
        struct peripheral_relay *relay = &relays[r];

        for (int src = 0; src < ARRAY_SIZE(battery_cache); src++) {
            if (battery_cache[src] > 0) {
                write_to_relay(relay, (uint8_t)src, battery_cache[src]);
                k_msleep(RELAY_WRITE_SPACING_MS);
            }
        }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
        if (dongle_battery_cache > 0) {
            write_to_relay(relay, BATTERY_RELAY_SOURCE_DONGLE, dongle_battery_cache);
            k_msleep(RELAY_WRITE_SPACING_MS);
        }
#endif

        write_to_relay(relay, BATTERY_RELAY_SOURCE_LAYER, layer_cache);
        k_msleep(RELAY_WRITE_SPACING_MS);
    }

    /* Reschedule */
    k_work_schedule_for_queue(&relay_work_q, &periodic_work,
                              K_MSEC(RELAY_PERIODIC_MS));
}

/* -------------------------------------------------------------------------
 * BT connection callbacks — just track connections, NO GATT operations.
 * ---------------------------------------------------------------------- */

static void relay_connected(struct bt_conn *conn, uint8_t conn_err) {
    if (conn_err) {
        return;
    }

    /* Only handle connections where we are the central (split peripherals). */
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) {
        return;
    }
    if (info.role != BT_HCI_ROLE_CENTRAL) {
        return;
    }

    struct peripheral_relay *relay = get_free_relay();
    if (!relay) {
        LOG_WRN("relay: no free relay slot for new connection");
        return;
    }

    relay->conn = bt_conn_ref(conn);
    relay->bat_ready = false;
    relay->bat_discovery_done = false;
    relay->bat_char_handle = 0;
    relay->discovery_in_flight = false;

    LOG_INF("relay: peripheral connected, will discover in next periodic cycle");
    /* NO discovery scheduled here — the periodic handler will pick it up */
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) {
        return;
    }

    LOG_DBG("relay: peripheral disconnected (reason %u), clearing slot", reason);
    bt_conn_unref(relay->conn);
    relay->conn = NULL;
    relay->bat_ready = false;
    relay->bat_discovery_done = false;
    relay->bat_char_handle = 0;
    relay->discovery_in_flight = false;
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
 * Init — start dedicated work queue and periodic handler
 * ---------------------------------------------------------------------- */

static int relay_central_init(void) {
    k_work_queue_start(&relay_work_q, relay_work_q_stack,
                       K_THREAD_STACK_SIZEOF(relay_work_q_stack),
                       K_PRIO_PREEMPT(10), /* low priority — never starve ZMK */
                       NULL);

    /* First periodic cycle at 60 s — gives ZMK's split stack plenty of
     * time to fully establish connections, security, and subscriptions
     * on all peripherals before we attempt any GATT operations. */
    k_work_schedule_for_queue(&relay_work_q, &periodic_work,
                              K_MSEC(RELAY_PERIODIC_MS));
    return 0;
}

SYS_INIT(relay_central_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
