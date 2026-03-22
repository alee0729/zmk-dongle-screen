/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Battery Relay — Central (dongle) side
 * ======================================
 * When enabled, the central:
 *   1. Registers BT connection callbacks.
 *   2. On peripheral connect, discovers the battery relay GATT characteristic.
 *   3. Subscribes to ZMK battery events.
 *   4. On every battery change, writes struct battery_relay_data to each
 *      connected peripheral that has been successfully discovered.
 *
 * This allows peripherals running the dongle_screen shield to display battery
 * levels for all keyboard splits, not just their own.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

#include <zmk/split/central.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/battery.h>

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* -------------------------------------------------------------------------
 * Per-peripheral relay state
 * ---------------------------------------------------------------------- */

struct peripheral_relay {
    struct bt_conn *conn;
    uint16_t char_handle;
    bool ready;
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128 char_uuid; /* copy kept alive for async discovery */
    struct k_work_delayable discovery_work; /* deferred to avoid racing ZMK split init */
};

/* Delay before starting battery relay GATT discovery after connection.
 * ZMK's split stack and input-split service also do GATT discovery on connect;
 * starting a concurrent discovery causes -EBUSY and delays/breaks both.
 * 500 ms gives ZMK time to complete its own setup first. */
#define BATTERY_RELAY_DISCOVERY_DELAY_MS 500

static void discovery_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay, discovery_work);
    if (relay->conn == NULL) {
        return;
    }
    start_discovery(relay);
}

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

static void broadcast_battery(uint8_t source, uint8_t level) {
    struct battery_relay_data data = {
        .source = source,
        .level = level,
    };

    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (!relays[i].ready || relays[i].conn == NULL) {
            continue;
        }
        int err = bt_gatt_write_without_response(relays[i].conn, relays[i].char_handle,
                                                 &data, sizeof(data), false);
        if (err) {
            LOG_WRN("battery_relay: write to peripheral %d failed: %d", i, err);
        }
    }
}

/* -------------------------------------------------------------------------
 * GATT discovery
 * ---------------------------------------------------------------------- */

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay,
                                                   discover_params);

    if (!attr) {
        /* Discovery finished — log whether we found the characteristic */
        if (!relay->ready) {
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->char_handle = chrc->value_handle;
    relay->ready = true;
    LOG_INF("battery_relay: characteristic found, handle=%u", relay->char_handle);

    return BT_GATT_ITER_STOP;
}

static void start_discovery(struct peripheral_relay *relay) {
    /* Copy the UUID so it stays alive for the duration of async discovery */
    memcpy(&relay->char_uuid, BATTERY_RELAY_CHAR_UUID, sizeof(relay->char_uuid));

    relay->discover_params.uuid = &relay->char_uuid.uuid;
    relay->discover_params.func = discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        LOG_ERR("battery_relay: bt_gatt_discover failed: %d", err);
    }
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
    relay->ready = false;
    relay->char_handle = 0;

    k_work_init_delayable(&relay->discovery_work, discovery_work_handler);
    k_work_schedule(&relay->discovery_work, K_MSEC(BATTERY_RELAY_DISCOVERY_DELAY_MS));
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) {
        return;
    }

    LOG_DBG("battery_relay: peripheral disconnected (reason %u), clearing slot", reason);
    k_work_cancel_delayable(&relay->discovery_work);
    bt_conn_unref(relay->conn);
    relay->conn = NULL;
    relay->ready = false;
    relay->char_handle = 0;
}

BT_CONN_CB_DEFINE(battery_relay_conn_cb) = {
    .connected = relay_connected,
    .disconnected = relay_disconnected,
};

/* -------------------------------------------------------------------------
 * ZMK event listener — broadcast battery changes to all peripherals
 * ---------------------------------------------------------------------- */

static int battery_relay_central_event_handler(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev) {
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

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(battery_relay_central, battery_relay_central_event_handler);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_peripheral_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
ZMK_SUBSCRIPTION(battery_relay_central, zmk_battery_state_changed);
#endif
struct peripheral_relay {
    struct bt_conn *conn;
    uint16_t char_handle;
    bool ready;
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128 char_uuid; /* copy kept alive for async discovery */
};

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

static void broadcast_battery(uint8_t source, uint8_t level) {
    struct battery_relay_data data = {
        .source = source,
        .level = level,
    };

    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (!relays[i].ready || relays[i].conn == NULL) {
            continue;
        }
        int err = bt_gatt_write_without_response(relays[i].conn, relays[i].char_handle,
                                                 &data, sizeof(data), false);
        if (err) {
            LOG_WRN("battery_relay: write to peripheral %d failed: %d", i, err);
        }
    }
}

/* -------------------------------------------------------------------------
 * GATT discovery
 * ---------------------------------------------------------------------- */

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay,
                                                   discover_params);

    if (!attr) {
        /* Discovery finished — log whether we found the characteristic */
        if (!relay->ready) {
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->char_handle = chrc->value_handle;
    relay->ready = true;
    LOG_INF("battery_relay: characteristic found, handle=%u", relay->char_handle);

    return BT_GATT_ITER_STOP;
}

static void start_discovery(struct peripheral_relay *relay) {
    /* Copy the UUID so it stays alive for the duration of async discovery */
    memcpy(&relay->char_uuid, BATTERY_RELAY_CHAR_UUID, sizeof(relay->char_uuid));

    relay->discover_params.uuid = &relay->char_uuid.uuid;
    relay->discover_params.func = discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        LOG_ERR("battery_relay: bt_gatt_discover failed: %d", err);
    }
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
    relay->ready = false;
    relay->char_handle = 0;

    start_discovery(relay);
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) {
        return;
    }

    LOG_DBG("battery_relay: peripheral disconnected (reason %u), clearing slot", reason);
    bt_conn_unref(relay->conn);
    relay->conn = NULL;
    relay->ready = false;
    relay->char_handle = 0;
}

BT_CONN_CB_DEFINE(battery_relay_conn_cb) = {
    .connected = relay_connected,
    .disconnected = relay_disconnected,
};

/* -------------------------------------------------------------------------
 * ZMK event listener — broadcast battery changes to all peripherals
 * ---------------------------------------------------------------------- */

static int battery_relay_central_event_handler(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev) {
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

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(battery_relay_central, battery_relay_central_event_handler);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_peripheral_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
ZMK_SUBSCRIPTION(battery_relay_central, zmk_battery_state_changed);
#endif
