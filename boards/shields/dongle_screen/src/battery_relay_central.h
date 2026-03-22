/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/bluetooth/uuid.h>

/*
 * 128-bit UUIDs for the battery relay GATT service.
 *
 * The central (dongle) discovers these on each peripheral and writes
 * battery level updates so the peripheral's display can show all splits'
 * battery levels.
 *
 * Service UUID : 6e400010-b5a3-f393-e0a9-e50e24dcca9e
 * Characteristic: 6e400011-b5a3-f393-e0a9-e50e24dcca9e
 */
#define BATTERY_RELAY_SERVICE_UUID_INIT \
    BT_UUID_128_ENCODE(0x6e400010, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

#define BATTERY_RELAY_CHAR_UUID_INIT \
    BT_UUID_128_ENCODE(0x6e400011, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

#define BATTERY_RELAY_SERVICE_UUID BT_UUID_DECLARE_128(BATTERY_RELAY_SERVICE_UUID_INIT)
#define BATTERY_RELAY_CHAR_UUID    BT_UUID_DECLARE_128(BATTERY_RELAY_CHAR_UUID_INIT)


/* source value used to identify the dongle's own battery */
#define BATTERY_RELAY_SOURCE_DONGLE 0xFF

/* source value used to carry layer data through the battery relay characteristic.
 * When source == BATTERY_RELAY_SOURCE_LAYER, the level field contains the
 * highest active layer index instead of a battery percentage. */
#define BATTERY_RELAY_SOURCE_LAYER 0xFE

/**
 * @brief Payload written by the central to each peripheral's relay characteristic.
 *
 * @param source  0-based peripheral index as seen by the central,
 *                BATTERY_RELAY_SOURCE_DONGLE (0xFF) for the dongle itself,
 *                or BATTERY_RELAY_SOURCE_LAYER (0xFE) for layer data.
 * @param level   Battery state-of-charge (0-100 %) for battery sources,
 *                or highest active layer index for BATTERY_RELAY_SOURCE_LAYER.
 */
struct battery_relay_data {
    uint8_t source;
    uint8_t level;
} __packed;
