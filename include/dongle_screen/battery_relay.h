/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Public API for the battery/layer relay module.
 * Any display module (dongle_screen, nice_view_gem, etc.) can include this
 * header to access relayed state on peripheral builds.
 */

#pragma once

#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_DONGLE_SCREEN_BATTERY_RELAY) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

/** Return the last layer index relayed from the central (dongle). */
uint8_t battery_relay_get_layer(void);

#endif
