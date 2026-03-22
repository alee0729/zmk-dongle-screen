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
#include <zmk/events/peripheral_battery_state_changed.h>
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
    bool bat_ready;

    /* Layer relay GATT characteristic */
    uint16_t layer_char_handle;
    bool layer_ready;

    /* Discovery state */
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128 discover_uuid; /* copy kept alive for async discovery */
    struct k_work_delayable discovery_work;
};

/* Delay before starting GATT discovery after connection.
 * ZMK's split stack also does GATT discovery on connect;
 * starting a concurrent discovery causes -EBUSY.
 * 500 ms gives ZMK time to complete its own setup first. */
#define RELAY_DISCOVERY_DELAY_MS 500

/* How often to re-broadcast cached battery state (ms). */
#define RELAY_PERIODIC_BROADCAST_MS 60000

static struct peripheral_relay relays[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

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

    /* Push current layer */
    write_layer_to_relay(relay, layer_cache);
}

/* -------------------------------------------------------------------------
 * GATT discovery — battery first, then layer (chained)
 * ---------------------------------------------------------------------- */

static void start_layer_discovery(struct peripheral_relay *relay);

static uint8_t layer_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay, discover_params);

    if (!attr) {
        if (!relay->layer_ready) {
            LOG_DBG("layer_relay: characteristic not found on conn %p", (void *)conn);
        }
        /* Discovery complete — push cached state to this peripheral */
        push_cached_state(relay);
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->layer_char_handle = chrc->value_handle;
    relay->layer_ready = true;
    LOG_INF("layer_relay: characteristic found, handle=%u", relay->layer_char_handle);

    return BT_GATT_ITER_STOP;
}

static void start_layer_discovery(struct peripheral_relay *relay) {
    memcpy(&relay->discover_uuid, LAYER_RELAY_CHAR_UUID, sizeof(relay->discover_uuid));

    relay->discover_params.uuid = &relay->discover_uuid.uuid;
    relay->discover_params.func = layer_discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        LOG_ERR("layer_relay: bt_gatt_discover failed: %d", err);
        /* Still push what we have (battery handles may be valid) */
        push_cached_state(relay);
    }
}

static uint8_t battery_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay, discover_params);

    if (!attr) {
        if (!relay->bat_ready) {
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
        }
        /* Chain: now discover layer relay characteristic */
        start_layer_discovery(relay);
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready = true;
    LOG_INF("battery_relay: characteristic found, handle=%u", relay->bat_char_handle);

    return BT_GATT_ITER_STOP;
}

static void start_battery_discovery(struct peripheral_relay *relay) {
    memcpy(&relay->discover_uuid, BATTERY_RELAY_CHAR_UUID, sizeof(relay->discover_uuid));

    relay->discover_params.uuid = &relay->discover_uuid.uuid;
    relay->discover_params.func = battery_discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        LOG_ERR("battery_relay: bt_gatt_discover failed: %d", err);
    }
}

static void discovery_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay, discovery_work);
    if (relay->conn == NULL) {
        return;
    }
    start_battery_discovery(relay);
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

    /* Only handle LE connections (split peripherals are always LE) */
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) {
        return;
    }

    struct peripheral_relay *relay = get_free_relay();
    if (!relay) {
        LOG_WRN("battery_relay: no free relay slot for new connection");
        return;
    }

    relay->conn = bt_conn_ref(conn);
    relay->bat_ready = false;
    relay->bat_char_handle = 0;
    relay->layer_ready = false;
    relay->layer_char_handle = 0;

    k_work_init_delayable(&relay->discovery_work, discovery_work_handler);
    k_work_schedule(&relay->discovery_work, K_MSEC(RELAY_DISCOVERY_DELAY_MS));
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
    relay->bat_char_handle = 0;
    relay->layer_ready = false;
    relay->layer_char_handle = 0;
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
 * Init — start periodic rebroadcast timer
 * ---------------------------------------------------------------------- */

static int relay_central_init(void) {
    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
    return 0;
}

SYS_INIT(relay_central_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
