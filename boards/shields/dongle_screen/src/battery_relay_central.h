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
 * Service UUID : 7b1a2e3c-4d5f-6a7b-8c9d-aebfc0d1e2f3
 * Characteristic: 7b1a2e3c-4d5f-6a7b-8c9d-aebfc0d1e2f4
 */
#define BATTERY_RELAY_SERVICE_UUID_INIT \
    BT_UUID_128_ENCODE(0x7b1a2e3c, 0x4d5f, 0x6a7b, 0x8c9d, 0xaebfc0d1e2f3)

#define BATTERY_RELAY_CHAR_UUID_INIT \
    BT_UUID_128_ENCODE(0x7b1a2e3c, 0x4d5f, 0x6a7b, 0x8c9d, 0xaebfc0d1e2f4)

#define BATTERY_RELAY_SERVICE_UUID BT_UUID_DECLARE_128(BATTERY_RELAY_SERVICE_UUID_INIT)
#define BATTERY_RELAY_CHAR_UUID    BT_UUID_DECLARE_128(BATTERY_RELAY_CHAR_UUID_INIT)

/* source value used to identify the dongle's own battery */
#define BATTERY_RELAY_SOURCE_DONGLE 0xFF

/**
 * @brief Payload written by the central to each peripheral's relay characteristic.
 *
 * @param source  0-based peripheral index as seen by the central, or
 *                BATTERY_RELAY_SOURCE_DONGLE (0xFF) for the dongle itself.
 * @param level   Battery state-of-charge, 0-100 %.
 */
struct battery_relay_data {
    uint8_t source;
    uint8_t level;
} __packed;
