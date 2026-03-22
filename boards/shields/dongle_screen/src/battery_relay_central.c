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
 *      GATT characteristics (sequentially to avoid EBUSY).
 *   3. Subscribes to ZMK battery and layer events.
 *   4. On every battery change, writes struct battery_relay_data to each
 *      connected peripheral that has been successfully discovered.
 *   5. On every layer change, writes struct layer_relay_data to each
 *      connected peripheral that has a layer relay characteristic.
 *   6. Caches battery levels and layer index so newly-discovered peripherals
 *      get the current state immediately after GATT discovery completes.
 *   7. Periodically re-broadcasts battery state for resilience against
 *      dropped BLE writes.
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
 * Cached state — sent to peripherals after GATT discovery completes and
 * periodically re-broadcast for resilience.
 * ---------------------------------------------------------------------- */

static uint8_t battery_cache[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];
static uint8_t layer_cache;

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

    /* Deferred push after discovery completes — walks through cached
     * battery sources one at a time with RELAY_WRITE_SPACING_MS between
     * writes, then sends the layer as the final step. */
    struct k_work_delayable push_work;
    uint8_t push_index; /* next battery_cache index to push, or
                         * ARRAY_SIZE(battery_cache) = push layer,
                         * > ARRAY_SIZE(battery_cache) = done */
};

/* Delay before starting GATT discovery after connection.
 * ZMK's split stack also does GATT discovery on connect;
 * starting a concurrent discovery causes -EBUSY.
 *
 * After sleep/wake reconnection ZMK's split stack may take longer to
 * fully re-subscribe to HID notifications (security re-establishment,
 * service-changed handling, etc.).  Zephyr only allows one outstanding
 * ATT request per connection — our discovery blocks ALL other ATT
 * operations on that link.  10 s gives ZMK ample headroom. */
#define RELAY_DISCOVERY_DELAY_MS 10000

/* How often to re-broadcast cached battery state (ms). */
#define RELAY_PERIODIC_BROADCAST_MS 60000

/* Retry parameters for GATT discovery when it fails (e.g. -EBUSY). */
#define MAX_DISCOVERY_RETRIES 5
#define DISCOVERY_RETRY_BASE_MS 1000

/* Delay between battery discovery completing and layer discovery starting.
 * This gap lets ZMK's split stack (or any other ATT client) perform
 * operations on the connection without being blocked by our discovery. */
#define RELAY_INTER_DISCOVERY_DELAY_MS 2000

/* Delay between individual GATT writes when pushing cached state.
 * Rapid-fire bt_gatt_write_without_response calls can exhaust BLE TX
 * buffers and starve ZMK's split HID communication. */
#define RELAY_WRITE_SPACING_MS 50

/* Minimum interval between layer broadcasts (ms).  Momentary layer taps
 * generate rapid activate/deactivate pairs; debouncing avoids flooding
 * the BLE TX buffers which can starve ZMK's split communication. */
#define LAYER_BROADCAST_DEBOUNCE_MS 100

static struct peripheral_relay relays[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

/* Debounced layer broadcast work */
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
 * Write helpers
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

/**
 * Push cached state to a single peripheral one write at a time.
 *
 * Each invocation sends at most one GATT write, then reschedules itself
 * after RELAY_WRITE_SPACING_MS.  This avoids a burst of rapid-fire
 * bt_gatt_write_without_response() calls that can exhaust BLE TX buffers
 * and starve ZMK's split HID communication.
 */
static void push_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay, push_work);

    if (relay->conn == NULL) {
        return;
    }

    /* Walk through battery_cache entries, skipping empty ones */
    while (relay->push_index < ARRAY_SIZE(battery_cache)) {
        uint8_t idx = relay->push_index++;
        if (battery_cache[idx] > 0) {
            write_battery_to_relay(relay, idx, battery_cache[idx]);
            /* Reschedule for next entry after a spacing delay */
            k_work_schedule(&relay->push_work, K_MSEC(RELAY_WRITE_SPACING_MS));
            return;
        }
    }

    /* All battery entries done — push current layer as the final write */
    if (relay->push_index == ARRAY_SIZE(battery_cache)) {
        relay->push_index++;
        write_layer_to_relay(relay, layer_cache);
    }
}

/** Schedule a deferred push of cached state (avoids writing during discovery). */
static void schedule_push_cached_state(struct peripheral_relay *relay) {
    relay->push_index = 0;
    k_work_schedule(&relay->push_work, K_MSEC(200));
}

/* Debounced layer broadcast: writes the latest cached layer to all relays. */
static void layer_broadcast_work_handler(struct k_work *work) {
    broadcast_layer(layer_cache);
}

/* -------------------------------------------------------------------------
 * GATT discovery — battery first, then layer (chained)
 *
 * IMPORTANT: Discovery callbacks run in BT RX thread context.  Never call
 * bt_gatt_discover() directly from within a callback — always defer to the
 * system work queue via discovery_work to avoid re-entering the GATT stack.
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
        k_work_schedule(&relay->discovery_work, K_MSEC(delay));
    } else {
        LOG_ERR("%s: giving up after %d retries, pushing available state",
                phase, MAX_DISCOVERY_RETRIES);
        /* Push whatever state we have so battery info still works
         * even if layer discovery never succeeded. */
        schedule_push_cached_state(relay);
    }
}

static uint8_t layer_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay, discover_params);

    if (!attr) {
        if (!relay->layer_ready) {
            LOG_DBG("layer_relay: characteristic not found on conn %p", (void *)conn);
        }
        /* Discovery complete — schedule deferred push of cached state */
        schedule_push_cached_state(relay);
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->layer_char_handle = chrc->value_handle;
    relay->layer_ready = true;
    LOG_INF("layer_relay: characteristic found, handle=%u", relay->layer_char_handle);

    /* Discovery complete — schedule deferred push of cached state */
    schedule_push_cached_state(relay);

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
        /* Mark battery discovery as attempted so work handler proceeds
         * to layer discovery.  Always defer via work queue — never call
         * bt_gatt_discover from inside a GATT callback.
         *
         * If battery relay was not found, skip layer discovery entirely —
         * if the peripheral doesn't have the battery relay service it
         * won't have the layer relay either, and a full GATT database
         * scan for a non-existent characteristic needlessly occupies the
         * ATT channel, potentially blocking ZMK's split HID operations. */
        relay->bat_discovery_done = true;
        if (relay->bat_ready) {
            k_work_schedule(&relay->discovery_work,
                            K_MSEC(RELAY_INTER_DISCOVERY_DELAY_MS));
        } else {
            LOG_DBG("battery_relay: skipping layer discovery — no battery "
                     "relay on this peripheral");
            schedule_push_cached_state(relay);
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready = true;
    relay->bat_discovery_done = true;
    LOG_INF("battery_relay: characteristic found, handle=%u", relay->bat_char_handle);

    /* Chain: discover layer relay characteristic next.
     * Schedule via work queue so current GATT discovery fully completes
     * first, with a generous delay so the ATT channel is free for ZMK's
     * split stack to perform any pending operations. */
    k_work_schedule(&relay->discovery_work,
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
    k_work_init_delayable(&relay->push_work, push_work_handler);
    k_work_schedule(&relay->discovery_work, K_MSEC(RELAY_DISCOVERY_DELAY_MS));
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) {
        return;
    }

    LOG_DBG("relay: peripheral disconnected (reason %u), clearing slot", reason);
    k_work_cancel_delayable(&relay->discovery_work);
    k_work_cancel_delayable(&relay->push_work);
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
 * ZMK event listener — battery + layer changes
 * ---------------------------------------------------------------------- */

static int relay_central_event_handler(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev) {
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
        broadcast_battery(BATTERY_RELAY_SOURCE_DONGLE, bat_ev->state_of_charge);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    const struct zmk_layer_state_changed *layer_ev = as_zmk_layer_state_changed(eh);
    if (layer_ev) {
        uint8_t highest = zmk_keymap_highest_layer_active();
        layer_cache = highest;
        /* Debounce: schedule broadcast after a short delay so rapid
         * layer activate/deactivate pairs only produce one BLE write. */
        k_work_schedule(&layer_broadcast_work, K_MSEC(LAYER_BROADCAST_DEBOUNCE_MS));
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
 * Init — start periodic rebroadcast timer
 * ---------------------------------------------------------------------- */

static int relay_central_init(void) {
    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
    return 0;
}

SYS_INIT(relay_central_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
