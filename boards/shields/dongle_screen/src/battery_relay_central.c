/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Display Relay — Central (dongle) side
 * ======================================
 * When enabled, the central:
 *   1. Registers BT connection callbacks.
 *   2. On peripheral connect, discovers the battery relay GATT characteristic.
 *   3. Subscribes to ZMK battery and activity-state events.
 *   4. On every battery change, updates a dirty flag but does NOT write
 *      immediately. Writes are deferred until the keyboard becomes idle/sleep.
 *   5. On ACTIVE→IDLE or ACTIVE→SLEEP transition, flushes all dirty battery
 *      values to every connected peripheral via GATT write-without-response.
 *   6. Periodically marks state dirty so stale displays resync on next idle.
 *
 * WHY IDLE-GATING:
 *   The nRF52840 BLE connection between dongle and each shield is a shared
 *   radio resource.  Both the split keyboard HID protocol (peripheral→central
 *   keystrokes) and relay writes (central→peripheral display data) share the
 *   same connection events and ATT bearer.  Any write during a typing burst
 *   can steal a connection event slot needed for a keystroke, causing a
 *   missed key.  Battery levels change only every few minutes, so deferring
 *   writes to idle periods costs nothing perceptible in display latency.
 *
 * IMPORTANT: All bt_gatt_write_without_response calls are serialized through
 * a dedicated writer thread.  This function can block waiting for BT TX
 * buffers — if called directly from the system work queue, it deadlocks
 * because the BT stack needs the system work queue for TX completion.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

#include <zmk/events/battery_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/battery.h>
/* Layer relay disabled: #include <zmk/events/layer_state_changed.h> */
/* Layer relay disabled: #include <zmk/keymap.h> */

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* -------------------------------------------------------------------------
 * Cached state — updated on every event, written to peripherals only when
 * keyboard becomes idle/sleep.
 * ---------------------------------------------------------------------- */

static uint8_t battery_cache[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

/* Dirty flags: set when a cache value changes (or on reconnect/periodic).
 * Cleared by flush_dirty() after the value is broadcast to all peripherals. */
static bool battery_dirty[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

/* True when the keyboard is actively in use.  Writes are suppressed while
 * active to avoid competing with HID split protocol for BLE TX bandwidth. */
static bool keyboard_active = true; /* conservative default — wait for first IDLE event */

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
static uint8_t dongle_battery_cache;
static bool dongle_battery_dirty;
#endif

/* Diagnostic counters */
volatile uint32_t relay_diag_conn_count;
volatile uint32_t relay_diag_disc_start;
volatile uint32_t relay_diag_disc_ok;
volatile uint32_t relay_diag_disc_fail;
volatile uint32_t relay_diag_disc_err;
volatile uint32_t relay_diag_write_ok;
volatile uint32_t relay_diag_write_err;

/* -------------------------------------------------------------------------
 * Per-peripheral relay state
 * ---------------------------------------------------------------------- */

struct peripheral_relay {
    struct bt_conn *conn;
    uint16_t bat_char_handle;
    bool bat_ready;
    /* Layer relay disabled: bool layer_ready; */
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128 discover_uuid;
    struct k_work_delayable discovery_work;
    struct k_work_delayable watchdog_work;
    bool discovery_in_progress;
    uint8_t discover_retries;
};

#define RELAY_DISCOVERY_DELAY_MS       5000
#define RELAY_DISCOVERY_STAGGER_MS     3000
#define RELAY_DISCOVERY_MAX_RETRIES    5
#define RELAY_DISCOVERY_RETRY_DELAY_MS 3000
#define RELAY_DISCOVERY_WATCHDOG_MS    8000
#define RELAY_PERIODIC_BROADCAST_MS    60000

static struct peripheral_relay relays[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

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
 * Serialized write queue + dedicated writer thread
 * ---------------------------------------------------------------------- */

struct relay_write_msg {
    uint8_t relay_idx;
    struct battery_relay_data data;
};

#define RELAY_WRITE_QUEUE_SIZE 16
K_MSGQ_DEFINE(relay_write_msgq, sizeof(struct relay_write_msg), RELAY_WRITE_QUEUE_SIZE, 4);

#define RELAY_WRITER_STACK_SIZE 1024
#define RELAY_WRITER_PRIORITY   K_PRIO_PREEMPT(8)

K_THREAD_STACK_DEFINE(relay_writer_stack, RELAY_WRITER_STACK_SIZE);
static struct k_thread relay_writer_thread;

static void relay_writer_func(void *p1, void *p2, void *p3) {
    struct relay_write_msg msg;
    while (true) {
        k_msgq_get(&relay_write_msgq, &msg, K_FOREVER);
        struct peripheral_relay *relay = &relays[msg.relay_idx];
        if (relay->conn == NULL) continue;
        int err = bt_gatt_write_without_response(relay->conn, relay->bat_char_handle,
                                                  &msg.data, sizeof(msg.data), false);
        if (err) {
            relay_diag_write_err++;
            LOG_WRN("relay_writer: write failed: %d (relay %u)", err, msg.relay_idx);
        } else {
            relay_diag_write_ok++;
        }
    }
}

/* -------------------------------------------------------------------------
 * Write helpers
 * ---------------------------------------------------------------------- */

static void write_battery_to_relay(struct peripheral_relay *relay, uint8_t source, uint8_t level) {
    if (!relay->bat_ready || relay->conn == NULL) return;
    struct relay_write_msg msg = {
        .relay_idx = (uint8_t)get_relay_index(relay),
        .data = { .source = source, .level = level },
    };
    if (k_msgq_put(&relay_write_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("relay: write queue full, dropping write (source=%u)", source);
    }
}

static void broadcast_battery(uint8_t source, uint8_t level) {
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        write_battery_to_relay(&relays[i], source, level);
    }
}

/* Layer relay disabled:
 * static void write_layer_to_relay(...) { ... }
 * static void broadcast_layer(uint8_t layer) { ... }
 */

/* -------------------------------------------------------------------------
 * Idle-gated flush
 *
 * Called only when keyboard_active transitions false (IDLE or SLEEP).
 * Sends all pending dirty battery values to connected peripherals.
 * ---------------------------------------------------------------------- */

static void flush_dirty(void) {
    for (int i = 0; i < ARRAY_SIZE(battery_dirty); i++) {
        if (battery_dirty[i] && battery_cache[i] > 0) {
            battery_dirty[i] = false;
            broadcast_battery((uint8_t)i, battery_cache[i]);
        }
    }
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (dongle_battery_dirty && dongle_battery_cache > 0) {
        dongle_battery_dirty = false;
        broadcast_battery(BATTERY_RELAY_SOURCE_DONGLE, dongle_battery_cache);
    }
#endif
    LOG_DBG("relay: flushed dirty battery state to peripherals");
}

/* Mark all cached values dirty so they will be re-sent on the next flush.
 * Used after peripheral discovery and for periodic resilience resyncs. */
static void mark_all_dirty(void) {
    for (int i = 0; i < ARRAY_SIZE(battery_cache); i++) {
        if (battery_cache[i] > 0) battery_dirty[i] = true;
    }
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (dongle_battery_cache > 0) dongle_battery_dirty = true;
#endif
}

/* -------------------------------------------------------------------------
 * GATT discovery
 * ---------------------------------------------------------------------- */

static uint8_t battery_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params) {
    struct peripheral_relay *relay = CONTAINER_OF(params, struct peripheral_relay, discover_params);
    relay->discovery_in_progress = false;
    k_work_cancel_delayable(&relay->watchdog_work);

    if (!attr) {
        if (!relay->bat_ready) {
            relay_diag_disc_fail++;
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
            if (relay->discover_retries > 0) {
                relay->discover_retries--;
                k_work_schedule(&relay->discovery_work, K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS));
            } else {
                LOG_ERR("battery_relay: giving up discovery on conn %p", (void *)conn);
            }
        } else {
            k_work_schedule(&relay->discovery_work, K_NO_WAIT);
        }
        return BT_GATT_ITER_STOP;
    }

    relay_diag_disc_ok++;
    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready = true;
    LOG_INF("battery_relay: characteristic found, handle=%u", relay->bat_char_handle);

    k_work_schedule(&relay->discovery_work, K_NO_WAIT);
    return BT_GATT_ITER_STOP;
}

static void start_battery_discovery(struct peripheral_relay *relay) {
    relay_diag_disc_start++;
    memcpy(&relay->discover_uuid, BATTERY_RELAY_CHAR_UUID, sizeof(relay->discover_uuid));

    relay->discover_params.uuid = &relay->discover_uuid.uuid;
    relay->discover_params.func = battery_discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    relay->discovery_in_progress = true;
    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        relay->discovery_in_progress = false;
        relay_diag_disc_err++;
        LOG_ERR("battery_relay: bt_gatt_discover failed: %d (retries left %u)",
                err, relay->discover_retries);
        if (relay->discover_retries > 0 &&
            (err == -EBUSY || err == -ENOMEM || err == -EAGAIN)) {
            relay->discover_retries--;
            k_work_schedule(&relay->discovery_work, K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS));
        }
    } else {
        k_work_schedule(&relay->watchdog_work, K_MSEC(RELAY_DISCOVERY_WATCHDOG_MS));
    }
}

static void discovery_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay, discovery_work);
    if (relay->conn == NULL) return;

    if (!relay->bat_ready) {
        start_battery_discovery(relay);
    } else {
        /* Discovery complete — mark cached state dirty and flush if idle.
         * If keyboard is active, writes will be deferred to next IDLE transition.
         * This prevents discovery-triggered writes from competing with keystrokes. */
        mark_all_dirty();
        if (!keyboard_active) {
            flush_dirty();
        }
    }
}

static void watchdog_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay = CONTAINER_OF(dwork, struct peripheral_relay, watchdog_work);
    if (relay->conn == NULL || relay->bat_ready) return;
    LOG_WRN("battery_relay: watchdog fired, discovery callback never came (retries %u)",
            relay->discover_retries);
    relay->discovery_in_progress = false;
    if (relay->discover_retries > 0) {
        relay->discover_retries--;
        k_work_schedule(&relay->discovery_work, K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS));
    } else {
        LOG_ERR("battery_relay: giving up after watchdog exhausted retries");
    }
}

/* -------------------------------------------------------------------------
 * Periodic resilience rebroadcast
 *
 * Marks all cached values dirty every RELAY_PERIODIC_BROADCAST_MS.
 * The flush only happens if the keyboard is idle at that moment, or on
 * the next ACTIVE→IDLE transition.
 * ---------------------------------------------------------------------- */

static void periodic_broadcast_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(periodic_broadcast_work, periodic_broadcast_handler);

static void periodic_broadcast_handler(struct k_work *work) {
    mark_all_dirty();
    if (!keyboard_active) {
        flush_dirty();
    }
    /* Reschedule regardless — dirty flags persist until flushed */
    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
}

/* -------------------------------------------------------------------------
 * BT connection callbacks
 * ---------------------------------------------------------------------- */

static void relay_connected(struct bt_conn *conn, uint8_t conn_err) {
    if (conn_err) return;

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) return;
    if (info.role != BT_CONN_ROLE_CENTRAL) return;

    relay_diag_conn_count++;

    struct peripheral_relay *relay = get_free_relay();
    if (!relay) {
        LOG_WRN("battery_relay: no free relay slot for new connection");
        return;
    }

    relay->conn = bt_conn_ref(conn);
    relay->bat_ready = false;
    relay->bat_char_handle = 0;
    relay->discovery_in_progress = false;
    relay->discover_retries = RELAY_DISCOVERY_MAX_RETRIES;

    k_work_init_delayable(&relay->discovery_work, discovery_work_handler);
    k_work_init_delayable(&relay->watchdog_work, watchdog_work_handler);

    uint32_t delay = RELAY_DISCOVERY_DELAY_MS +
                     (uint32_t)get_relay_index(relay) * RELAY_DISCOVERY_STAGGER_MS;
    k_work_schedule(&relay->discovery_work, K_MSEC(delay));

    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) return;

    LOG_DBG("relay: peripheral disconnected (reason %u), clearing slot", reason);
    k_work_cancel_delayable(&relay->discovery_work);
    k_work_cancel_delayable(&relay->watchdog_work);
    bt_conn_unref(relay->conn);
    relay->conn = NULL;
    relay->bat_ready = false;
    relay->bat_char_handle = 0;
    relay->discovery_in_progress = false;

    /* Mark everything dirty so the reconnecting peripheral gets a full
     * state push on its next discovery completion. */
    mark_all_dirty();
}

BT_CONN_CB_DEFINE(battery_relay_conn_cb) = {
    .connected = relay_connected,
    .disconnected = relay_disconnected,
};

/* -------------------------------------------------------------------------
 * ZMK event listener — battery changes + activity state
 * ---------------------------------------------------------------------- */

static int relay_central_event_handler(const zmk_event_t *eh) {
    /* Activity state — the idle gate */
    const struct zmk_activity_state_changed *activity_ev = as_zmk_activity_state_changed(eh);
    if (activity_ev) {
        bool was_active = keyboard_active;
        keyboard_active = (activity_ev->state == ZMK_ACTIVITY_ACTIVE);
        LOG_DBG("relay: activity state → %s",
                keyboard_active ? "ACTIVE" : "IDLE/SLEEP");
        if (was_active && !keyboard_active) {
            /* Just transitioned from active typing to idle/sleep.
             * Safe to send relay writes now. */
            flush_dirty();
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Peripheral battery level changed */
    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev) {
        uint8_t source = periph_ev->source;
        uint8_t level  = periph_ev->state_of_charge;
        if (source < ARRAY_SIZE(battery_cache) && battery_cache[source] == level) {
            return ZMK_EV_EVENT_BUBBLE; /* unchanged — skip */
        }
        if (source < ARRAY_SIZE(battery_cache)) {
            battery_cache[source] = level;
            battery_dirty[source] = true;
        }
        LOG_DBG("relay: battery[%u]=%u%% dirty (keyboard_active=%d)",
                source, level, keyboard_active);
        if (!keyboard_active) {
            flush_dirty();
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    const struct zmk_battery_state_changed *bat_ev = as_zmk_battery_state_changed(eh);
    if (bat_ev) {
        uint8_t level = bat_ev->state_of_charge;
        if (dongle_battery_cache == level) return ZMK_EV_EVENT_BUBBLE;
        dongle_battery_cache = level;
        dongle_battery_dirty = true;
        if (!keyboard_active) flush_dirty();
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    /* Layer relay disabled:
     * const struct zmk_layer_state_changed *layer_ev = ...;
     */

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(battery_relay_central, relay_central_event_handler);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_activity_state_changed);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_peripheral_battery_state_changed);
/* Layer relay disabled: ZMK_SUBSCRIPTION(battery_relay_central, zmk_layer_state_changed); */

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
ZMK_SUBSCRIPTION(battery_relay_central, zmk_battery_state_changed);
#endif

/* -------------------------------------------------------------------------
 * Writer thread initialization
 * ---------------------------------------------------------------------- */

static int relay_writer_init(void) {
    k_thread_create(&relay_writer_thread, relay_writer_stack,
                    K_THREAD_STACK_SIZEOF(relay_writer_stack),
                    relay_writer_func, NULL, NULL, NULL,
                    RELAY_WRITER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&relay_writer_thread, "relay_writer");
    return 0;
}

SYS_INIT(relay_writer_init, APPLICATION, 99);
