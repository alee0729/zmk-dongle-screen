/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Battery Relay — Central (dongle) side
 * ======================================
 * When enabled, the central:
 *   1. Subscribes to ZMK activity-state events.
 *   2. On peripheral connect, defers GATT discovery until the keyboard is idle
 *      to avoid stealing BLE connection events from the split HID protocol.
 *   3. Caches battery levels; on every ACTIVE→IDLE transition, sends all
 *      cached (dirty) levels to each connected peripheral that has completed
 *      discovery via a dedicated writer thread.
 *   4. The writer thread re-checks keyboard_active before each GATT write
 *      and drops the message if the user has resumed typing in the meantime.
 *   5. On IDLE→ACTIVE transition, purges the write queue and re-marks all
 *      cached values dirty so the next idle will re-send them.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

#include <zmk/split/central.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/battery.h>
#include <zmk/activity.h>

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */

/** Initial delay before first discovery attempt after connect. */
#define RELAY_DISCOVERY_INITIAL_DELAY_MS   500
/** Retry interval when discovery is deferred because keyboard is active. */
#define RELAY_DISCOVERY_RETRY_DELAY_MS     3000
/** Extra stagger per relay index so two peripherals don't discover together. */
#define RELAY_DISCOVERY_DEFER_STAGGER_MS   1000
/** Maximum number of battery write messages in flight. */
#define RELAY_WRITE_MSGQ_DEPTH             16

/* -------------------------------------------------------------------------
 * Activity state
 * ---------------------------------------------------------------------- */

/**
 * Tracks whether the keyboard is currently active (being typed on).
 * Defaults to true so we never send ATT traffic immediately at startup.
 * Updated by zmk_activity_state_changed events.
 */
static volatile bool keyboard_active = true;

/* -------------------------------------------------------------------------
 * Battery cache and dirty flags
 * ---------------------------------------------------------------------- */

/** Most-recently received battery level per peripheral source index. */
static uint8_t battery_cache[CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_COUNT];
/** Whether the cached value has not yet been successfully enqueued. */
static bool battery_dirty[CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
static uint8_t dongle_battery_cache;
static bool dongle_battery_dirty;
#endif

/* -------------------------------------------------------------------------
 * Writer message queue
 * ---------------------------------------------------------------------- */

struct relay_write_msg {
    int relay_idx;
    struct battery_relay_data data;
};

K_MSGQ_DEFINE(relay_write_msgq, sizeof(struct relay_write_msg),
              RELAY_WRITE_MSGQ_DEPTH, 4);

/* -------------------------------------------------------------------------
 * Per-peripheral relay state
 * ---------------------------------------------------------------------- */

struct peripheral_relay {
    struct bt_conn *conn;
    uint16_t char_handle;
    bool ready;
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128 char_uuid; /* copy kept alive for async discovery */
    struct k_work_delayable discovery_work;
};

static struct peripheral_relay relays[CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

static struct peripheral_relay *get_relay_by_conn(struct bt_conn *conn) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == conn) return &relays[i];
    }
    return NULL;
}

static struct peripheral_relay *get_free_relay(void) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == NULL) return &relays[i];
    }
    return NULL;
}

static int get_relay_index(const struct peripheral_relay *relay) {
    return (int)(relay - relays);
}

/* -------------------------------------------------------------------------
 * Forward declarations
 * ---------------------------------------------------------------------- */

static void start_discovery(struct peripheral_relay *relay);
static void flush_dirty(void);
static void mark_all_dirty(void);

/* -------------------------------------------------------------------------
 * Writer thread — serialises GATT writes, drops them if keyboard is active
 * ---------------------------------------------------------------------- */

static void relay_writer_func(void *p1, void *p2, void *p3) {
    struct relay_write_msg msg;
    while (true) {
        k_msgq_get(&relay_write_msgq, &msg, K_FOREVER);

        /* Drop write if keyboard became active while message was queued */
        if (keyboard_active) {
            LOG_DBG("relay_writer: dropping write (keyboard active)");
            continue;
        }

        struct peripheral_relay *relay = &relays[msg.relay_idx];
        if (relay->conn == NULL) continue;

        int err = bt_gatt_write_without_response(relay->conn, relay->char_handle,
                                                 &msg.data, sizeof(msg.data), false);
        if (err) {
            LOG_WRN("battery_relay: write to peripheral %d failed: %d",
                    msg.relay_idx, err);
        }
    }
}

K_THREAD_DEFINE(relay_writer_tid, 1024, relay_writer_func, NULL, NULL, NULL,
                K_PRIO_PREEMPT(7), 0, 0);

/* -------------------------------------------------------------------------
 * Write helpers
 * ---------------------------------------------------------------------- */

/**
 * Enqueue a write to one peripheral relay slot.
 * Returns true if the message was successfully enqueued.
 */
static bool write_battery_to_relay(int relay_idx, uint8_t source, uint8_t level) {
    struct peripheral_relay *relay = &relays[relay_idx];
    if (!relay->ready || relay->conn == NULL) return false;

    struct relay_write_msg msg = {
        .relay_idx = relay_idx,
        .data = { .source = source, .level = level },
    };
    return k_msgq_put(&relay_write_msgq, &msg, K_NO_WAIT) == 0;
}

/**
 * Broadcast a battery value to all connected+discovered peripherals.
 * Returns true if at least one write was enqueued.
 */
static bool broadcast_battery(uint8_t source, uint8_t level) {
    bool any_enqueued = false;
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (write_battery_to_relay(i, source, level)) any_enqueued = true;
    }
    return any_enqueued;
}

/**
 * Mark all non-zero cached values dirty so they will be re-sent at the next
 * idle flush.
 */
static void mark_all_dirty(void) {
    for (int i = 0; i < ARRAY_SIZE(battery_dirty); i++) {
        if (battery_cache[i] > 0) battery_dirty[i] = true;
    }
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (dongle_battery_cache > 0) dongle_battery_dirty = true;
#endif
}

/**
 * Send all dirty cached battery values.
 * Only clears the dirty flag if the write was successfully enqueued, so
 * that a value is never silently lost when all relays are still discovering.
 */
static void flush_dirty(void) {
    for (int i = 0; i < ARRAY_SIZE(battery_dirty); i++) {
        if (battery_dirty[i] && battery_cache[i] > 0) {
            if (broadcast_battery((uint8_t)i, battery_cache[i])) {
                battery_dirty[i] = false;
            }
        }
    }
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (dongle_battery_dirty && dongle_battery_cache > 0) {
        if (broadcast_battery(BATTERY_RELAY_SOURCE_DONGLE, dongle_battery_cache)) {
            dongle_battery_dirty = false;
        }
    }
#endif
}

/* -------------------------------------------------------------------------
 * GATT discovery
 * ---------------------------------------------------------------------- */

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay,
                                                   discover_params);

    if (!attr) {
        if (!relay->ready) {
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->char_handle = chrc->value_handle;
    relay->ready = true;
    LOG_INF("battery_relay: characteristic found on peripheral %d, handle=%u",
            get_relay_index(relay), relay->char_handle);

    /* Peripheral just became ready — send any cached battery values now if idle */
    if (!keyboard_active) {
        flush_dirty();
    }

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

static void discovery_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay,
                                                   discovery_work);

    if (relay->conn == NULL) return;

    if (!relay->ready) {
        if (keyboard_active) {
            /* Defer until idle; stagger multiple relays to avoid concurrent discovers */
            uint32_t delay = RELAY_DISCOVERY_RETRY_DELAY_MS +
                             (uint32_t)get_relay_index(relay) * RELAY_DISCOVERY_DEFER_STAGGER_MS;
            LOG_DBG("battery_relay: deferring discovery for peripheral %d (%u ms)",
                    get_relay_index(relay), delay);
            k_work_schedule(&relay->discovery_work, K_MSEC(delay));
            return;
        }
        start_discovery(relay);
    } else {
        /* Already discovered (e.g. re-triggered after reconnect) — flush cache */
        mark_all_dirty();
        if (!keyboard_active) flush_dirty();
    }
}

/* -------------------------------------------------------------------------
 * BT connection callbacks
 * ---------------------------------------------------------------------- */

static void relay_connected(struct bt_conn *conn, uint8_t conn_err) {
    if (conn_err) return;

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) return;

    struct peripheral_relay *relay = get_free_relay();
    if (!relay) {
        LOG_WRN("battery_relay: no free relay slot for new connection");
        return;
    }

    relay->conn = bt_conn_ref(conn);
    relay->ready = false;
    relay->char_handle = 0;

    /* Delay initial discovery; defer further if keyboard is active */
    k_work_schedule(&relay->discovery_work,
                    K_MSEC(RELAY_DISCOVERY_INITIAL_DELAY_MS));
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) return;

    LOG_DBG("battery_relay: peripheral %d disconnected (reason %u)",
            get_relay_index(relay), reason);
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
 * ZMK event listener — battery changes + activity state
 * ---------------------------------------------------------------------- */

static int battery_relay_central_event_handler(const zmk_event_t *eh) {
    /* Peripheral battery update */
    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev) {
        uint8_t src = (uint8_t)periph_ev->source;
        uint8_t lvl = periph_ev->state_of_charge;
        if (src < ARRAY_SIZE(battery_cache)) {
            battery_cache[src] = lvl;
            battery_dirty[src] = true;
        }
        if (!keyboard_active) flush_dirty();
        return ZMK_EV_EVENT_BUBBLE;
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    /* Dongle battery update */
    const struct zmk_battery_state_changed *bat_ev = as_zmk_battery_state_changed(eh);
    if (bat_ev) {
        dongle_battery_cache = bat_ev->state_of_charge;
        dongle_battery_dirty = true;
        if (!keyboard_active) flush_dirty();
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    /* Activity state change */
    const struct zmk_activity_state_changed *activity_ev =
        as_zmk_activity_state_changed(eh);
    if (activity_ev) {
        bool was_active = keyboard_active;
        keyboard_active = (activity_ev->state == ZMK_ACTIVITY_ACTIVE);

        if (!was_active && keyboard_active) {
            /*
             * Resuming typing: purge any writes that were queued during the
             * idle window but haven't been sent yet, then re-mark everything
             * dirty so the next idle will re-send them.
             */
            k_msgq_purge(&relay_write_msgq);
            mark_all_dirty();
            LOG_DBG("relay: purged write queue on ACTIVE transition");
        }

        if (was_active && !keyboard_active) {
            /* Became idle — send all pending battery values */
            flush_dirty();
        }

        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(battery_relay_central, battery_relay_central_event_handler);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_peripheral_battery_state_changed);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_activity_state_changed);

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
ZMK_SUBSCRIPTION(battery_relay_central, zmk_battery_state_changed);
#endif

/* -------------------------------------------------------------------------
 * Initialise delayable work items (must be done at runtime)
 * ---------------------------------------------------------------------- */

static int battery_relay_central_init(void) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        k_work_init_delayable(&relays[i].discovery_work, discovery_work_handler);
    }
    return 0;
}

SYS_INIT(battery_relay_central_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
