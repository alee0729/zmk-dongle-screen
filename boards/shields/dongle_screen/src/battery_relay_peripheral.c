/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Battery Relay — Peripheral side
 * =================================
 * Registers a GATT service that the central (dongle) writes battery data to.
 * Each write delivers a struct battery_relay_data {source, level}.  The handler
 * stores the value in a local cache and re-raises a
 * zmk_peripheral_battery_state_changed event so the battery_status display
 * widget redraws without any further modification.
 *
 * The battery_status widget already subscribes to
 * zmk_peripheral_battery_state_changed unconditionally, so raising this event
 * here is enough to drive the display on a peripheral build.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * On peripheral builds zmk/split/central.h is not available, so the
 * zmk_peripheral_battery_state_changed event struct is not declared.
 * Provide a local declaration + implementation so we can raise it here
 * and have the battery_status widget (which subscribes unconditionally)
 * pick it up.
 */
struct zmk_peripheral_battery_state_changed {
    uint8_t source;
    uint8_t state_of_charge;
};

ZMK_EVENT_DECLARE(zmk_peripheral_battery_state_changed);
ZMK_EVENT_IMPL(zmk_peripheral_battery_state_changed);

/* -------------------------------------------------------------------------
 * Relay cache
 *
 * Indexed by data.source.  BATTERY_RELAY_SOURCE_DONGLE (0xFF) is ignored
 * here because SOURCE_OFFSET == 0 on peripheral builds and there is no slot
 * for the dongle battery.
 * ---------------------------------------------------------------------- */

static uint8_t relay_cache[CONFIG_DONGLE_SCREEN_BATTERY_RELAY_SOURCE_COUNT];
static uint8_t relayed_layer;

uint8_t battery_relay_get_layer(void) {
    return relayed_layer;
}

/* -------------------------------------------------------------------------
 * GATT write handler
 * ---------------------------------------------------------------------- */

static ssize_t battery_relay_write_cb(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len,
                                       uint16_t offset, uint8_t flags) {
    if (len != sizeof(struct battery_relay_data)) {
        LOG_WRN("battery_relay: unexpected write length %u (expected %zu)",
                len, sizeof(struct battery_relay_data));
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const struct battery_relay_data *data = buf;
    LOG_INF("relay_periph: write received source=%u level=%u", data->source, data->level);

    /* Ignore the dongle's own battery (no slot for it on peripheral display) */
    if (data->source == BATTERY_RELAY_SOURCE_DONGLE) {
        return len;
    }

    /* Layer data is multiplexed through this characteristic.
     * source=0xFE means level contains the active layer index.
     * Store it and raise zmk_layer_state_changed so the layer_status
     * widget redraws with the relayed value. */
    if (data->source == BATTERY_RELAY_SOURCE_LAYER) {
        if (data->level != relayed_layer) {
            relayed_layer = data->level;
            LOG_DBG("relay: layer=%u", relayed_layer);
            ZMK_EVENT_RAISE(new_zmk_layer_state_changed(
                (struct zmk_layer_state_changed){
                    .layer = relayed_layer,
                    .state = true,
                    .timestamp = k_uptime_get(),
                }));
        }
        return len;
    }

    if (data->source >= CONFIG_DONGLE_SCREEN_BATTERY_RELAY_SOURCE_COUNT) {
        LOG_WRN("battery_relay: source %u out of range (max %d)",
                data->source, CONFIG_DONGLE_SCREEN_BATTERY_RELAY_SOURCE_COUNT - 1);
        return len;
    }

    relay_cache[data->source] = data->level;

    LOG_DBG("battery_relay: source=%u level=%u%%", data->source, data->level);

    /*
     * Re-raise as zmk_peripheral_battery_state_changed so the battery_status
     * widget redraws — it already subscribes to this event unconditionally.
     */
    ZMK_EVENT_RAISE(new_zmk_peripheral_battery_state_changed(
        (struct zmk_peripheral_battery_state_changed){
            .source = data->source,
            .state_of_charge = data->level,
        }));

    return len;
}

/* -------------------------------------------------------------------------
 * GATT service definition
 * ---------------------------------------------------------------------- */

BT_GATT_SERVICE_DEFINE(battery_relay_svc,
    BT_GATT_PRIMARY_SERVICE(BATTERY_RELAY_SERVICE_UUID),
    BT_GATT_CHARACTERISTIC(BATTERY_RELAY_CHAR_UUID,
        BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, battery_relay_write_cb, NULL),
);
