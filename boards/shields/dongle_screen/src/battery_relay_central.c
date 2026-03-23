/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Display Relay — Central (dongle) side
 * ======================================
 * When enabled, the central:
 *   1. Subscribes to ZMK battery and layer events, caching values locally.
 *   2. Periodically scans for connected peripherals using
 *      bt_conn_foreach, attempts GATT discovery on any whose relay handle
 *      is not yet known, then writes cached state to all discovered ones.
 *
 * DESIGN PRINCIPLE: Zero interference with ZMK's split stack.
 * ZMK's split stack does extensive GATT work on connect (service discovery,
 * HID notification subscription, security, connection parameter negotiation).
 * Even calling bt_conn_ref() on a connection being set up can interfere
 * with the second peripheral's connection — holding a ref on a connection
 * whose setup fails prevents the connection object from being freed,
 * exhausting the tiny BLE connection pool.
 *
 * Therefore, connections are only captured (bt_conn_ref) AFTER all expected
 * peripherals are connected.  During the waiting phase, we only count
 * connections — no refs, no pointer storage, no GATT calls.
 *
 * Both battery and layer data are multiplexed through the single battery
 * relay GATT characteristic.  Layer data uses source=BATTERY_RELAY_SOURCE_LAYER
 * with the layer index in the level field.
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

    /* Timestamp (uptime ms) when this connection was added to the relay.
     * Discovery is deferred until RELAY_CONN_SETTLE_MS after this time
     * to avoid interfering with ZMK's split stack GATT setup. */
    int64_t conn_added_at;
};

/* Periodic handler intervals (ms).
 * Fast interval is used while any relay still needs discovery.
 * Slow interval is used once all relays are ready (battery refresh). */
#define RELAY_FAST_MS  5000
#define RELAY_SLOW_MS 30000

/* Initial delay before the first periodic cycle (ms).
 * Gives ZMK's split stack a head-start on establishing connections. */
#define RELAY_INITIAL_DELAY_MS 5000

/* Delay between individual GATT writes during periodic broadcast.
 * Spaces out writes to avoid exhausting BLE TX buffers in a burst. */
#define RELAY_WRITE_SPACING_MS 100

/* Minimum interval between layer broadcasts (ms).  Momentary layer taps
 * generate rapid activate/deactivate pairs; debouncing avoids flooding
 * the BLE TX buffers which can starve ZMK's split communication. */
#define LAYER_BROADCAST_DEBOUNCE_MS 100

/* Time (ms) to wait after connections are captured before attempting
 * GATT discovery.  ZMK's split stack performs its own GATT operations on
 * connect (service discovery, HID subscription, security negotiation).
 * This cooldown ensures we only discover after ZMK has finished. */
#define RELAY_CONN_SETTLE_MS 5000

/* Maximum time (ms) to wait for all expected peripherals before proceeding
 * with discovery on whichever are connected.  Prevents indefinite waiting
 * if a peripheral is intentionally off or unreachable. */
#define RELAY_ALL_CONNECT_TIMEOUT_MS 30000

static struct peripheral_relay relays[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

/* True once we've captured (bt_conn_ref'd) the connections into the relay
 * array.  Before this, we only count connections — zero interaction. */
static bool connections_captured;

/* Timestamp when we first saw at least one central connection.
 * Used for the all-connect timeout.  0 = no connection seen yet. */
static int64_t first_conn_seen_at;

/* Timestamp when connections were captured into the relay array.
 * Used for the per-connection settle time before GATT discovery. */
static int64_t connections_captured_at;

/* Forward declarations for work items used across sections */
static void periodic_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(periodic_work, periodic_handler);

/* Debounced layer broadcast — runs on relay_work_q */
static void layer_broadcast_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(layer_broadcast_work, layer_broadcast_work_handler);

/* -------------------------------------------------------------------------
 * Connection counting — lightweight, no bt_conn_ref, no pointer storage.
 *
 * Used during the waiting phase to check how many peripherals are
 * connected without interfering with ZMK's connection management.
 * ---------------------------------------------------------------------- */

static void conn_count_cb(struct bt_conn *conn, void *user_data) {
    int *count = user_data;
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) < 0) {
        return;
    }
    if (info.role != BT_HCI_ROLE_CENTRAL) {
        return;
    }
    (*count)++;
}

static int count_central_connections(void) {
    int count = 0;
    bt_conn_foreach(BT_CONN_TYPE_LE, conn_count_cb, &count);
    return count;
}

/* -------------------------------------------------------------------------
 * Connection capture — called ONCE after all peripherals are connected
 * (or timeout).  This is the only place bt_conn_ref is called.
 * ---------------------------------------------------------------------- */

struct conn_scan_data {
    struct bt_conn *active[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];
    int count;
};

static void conn_scan_cb(struct bt_conn *conn, void *user_data) {
    struct conn_scan_data *scan = user_data;
    struct bt_conn_info info;

    if (scan->count >= ZMK_SPLIT_BLE_PERIPHERAL_COUNT) {
        return;
    }

    if (bt_conn_get_info(conn, &info) < 0) {
        return;
    }

    /* Only interested in connections where we are the central */
    if (info.role != BT_HCI_ROLE_CENTRAL) {
        return;
    }

    scan->active[scan->count++] = conn;
}

/** Capture all current central connections into the relay array.
 *  Called once after all peripherals are connected (or timeout). */
static void capture_connections(void) {
    struct conn_scan_data scan = { .count = 0 };
    bt_conn_foreach(BT_CONN_TYPE_LE, conn_scan_cb, &scan);

    int64_t now = k_uptime_get();
    for (int s = 0; s < scan.count; s++) {
        for (int i = 0; i < ARRAY_SIZE(relays); i++) {
            if (relays[i].conn == NULL) {
                relays[i].conn = bt_conn_ref(scan.active[s]);
                relays[i].bat_ready = false;
                relays[i].bat_discovery_done = false;
                relays[i].bat_char_handle = 0;
                relays[i].discovery_in_flight = false;
                relays[i].conn_added_at = now;
                LOG_INF("relay: captured peripheral connection, slot %d", i);
                break;
            }
        }
    }

    connections_captured = true;
    connections_captured_at = now;
    LOG_INF("relay: captured %d connections", scan.count);
}

/** Synchronize relay array with currently active BLE connections.
 *  Called from periodic handler AFTER connections have been captured.
 *  Detects disconnected peripherals and picks up reconnections. */
static void sync_relay_connections(void) {
    struct conn_scan_data scan = { .count = 0 };

    bt_conn_foreach(BT_CONN_TYPE_LE, conn_scan_cb, &scan);

    /* Remove relay entries for connections that are no longer active */
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == NULL) {
            continue;
        }

        bool still_active = false;
        for (int s = 0; s < scan.count; s++) {
            if (relays[i].conn == scan.active[s]) {
                still_active = true;
                break;
            }
        }

        if (!still_active) {
            LOG_INF("relay: peripheral disconnected, clearing slot %d", i);
            bt_conn_unref(relays[i].conn);
            relays[i].conn = NULL;
            relays[i].bat_ready = false;
            relays[i].bat_discovery_done = false;
            relays[i].bat_char_handle = 0;
            relays[i].discovery_in_flight = false;
        }
    }

    /* Add new connections that aren't in the relay array yet */
    for (int s = 0; s < scan.count; s++) {
        bool found = false;
        for (int i = 0; i < ARRAY_SIZE(relays); i++) {
            if (relays[i].conn == scan.active[s]) {
                found = true;
                break;
            }
        }

        if (!found) {
            for (int i = 0; i < ARRAY_SIZE(relays); i++) {
                if (relays[i].conn == NULL) {
                    relays[i].conn = bt_conn_ref(scan.active[s]);
                    relays[i].bat_ready = false;
                    relays[i].bat_discovery_done = false;
                    relays[i].bat_char_handle = 0;
                    relays[i].discovery_in_flight = false;
                    relays[i].conn_added_at = k_uptime_get();
                    LOG_INF("relay: found new peripheral connection, slot %d", i);
                    break;
                }
            }
        }
    }
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
    LOG_INF("relay: writing source=%u level=%u to handle=%u", source, level, relay->bat_char_handle);
    struct battery_relay_data data = { .source = source, .level = level };
    int err = bt_gatt_write_without_response(relay->conn, relay->bat_char_handle,
                                             &data, sizeof(data), false);
    if (err) {
        LOG_WRN("relay: write failed (source=%u): %d", source, err);
        if (err == -ENOTCONN) {
            relay->bat_ready = false;
            /* Reschedule periodic handler quickly so sync_relay_connections()
             * detects the disconnect and cleans up the stale slot, rather
             * than waiting up to RELAY_SLOW_MS (30 s). */
            k_work_reschedule_for_queue(&relay_work_q, &periodic_work,
                                        K_MSEC(RELAY_FAST_MS));
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

    if (!attr) {
        LOG_WRN("relay: characteristic not found on conn %p, will retry", (void *)conn);
        /* Do NOT set bat_discovery_done — allow retry next cycle */
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready = true;
    relay->bat_discovery_done = true;
    LOG_INF("relay: characteristic found, handle=%u", relay->bat_char_handle);

    return BT_GATT_ITER_STOP;
}

/** Try discovery on a relay that needs it.
 *  Returns true if a discovery was started (caller should not start another). */
static bool try_discovery_step(struct peripheral_relay *relay) {
    if (relay->conn == NULL || relay->discovery_in_flight || relay->bat_discovery_done) {
        return false;
    }

    /* Wait for ZMK's split stack to finish its own GATT setup before
     * we start ours.  Attempting discovery too early can interfere with
     * ZMK's service discovery / security negotiation. */
    if (k_uptime_get() - relay->conn_added_at < RELAY_CONN_SETTLE_MS) {
        LOG_DBG("relay: conn %p still settling, deferring discovery", (void *)relay->conn);
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
 * Periodic handler — connection scan + discovery + broadcast.
 *
 * Two phases:
 *
 * WAITING PHASE (connections_captured == false):
 *   Only counts central connections via bt_conn_foreach — no bt_conn_ref,
 *   no pointer storage, no GATT calls.  This ensures zero interference
 *   with ZMK's scanning and connection setup.  Transitions to active
 *   phase when all expected peripherals are connected OR timeout.
 *
 * ACTIVE PHASE (connections_captured == true):
 *   Syncs relay array, runs GATT discovery, writes cached state.
 *   Runs every 5 s while discovering, 30 s once all relays are ready.
 * ---------------------------------------------------------------------- */

static void periodic_handler(struct k_work *work) {

    /* ---- WAITING PHASE: hands off, just count ---- */
    if (!connections_captured) {
        int count = count_central_connections();

        if (count > 0 && first_conn_seen_at == 0) {
            first_conn_seen_at = k_uptime_get();
        }

        bool all_connected = (count >= ZMK_SPLIT_BLE_PERIPHERAL_COUNT);
        bool timed_out = (first_conn_seen_at > 0) &&
                         (k_uptime_get() - first_conn_seen_at > RELAY_ALL_CONNECT_TIMEOUT_MS);

        if (count > 0 && !all_connected && !timed_out) {
            LOG_INF("relay: waiting for peripherals: %d/%d connected "
                    "(%lld s until timeout)",
                    count, ZMK_SPLIT_BLE_PERIPHERAL_COUNT,
                    (RELAY_ALL_CONNECT_TIMEOUT_MS -
                     (k_uptime_get() - first_conn_seen_at)) / 1000);
            k_work_schedule_for_queue(&relay_work_q, &periodic_work,
                                      K_MSEC(RELAY_FAST_MS));
            return;
        }

        if (count == 0) {
            /* No connections yet — keep polling */
            k_work_schedule_for_queue(&relay_work_q, &periodic_work,
                                      K_MSEC(RELAY_FAST_MS));
            return;
        }

        /* All connected (or timeout) — capture connections now */
        LOG_INF("relay: %d peripherals connected, capturing connections", count);
        capture_connections();

        /* Give ZMK's GATT setup time to finish on the last connection */
        k_work_schedule_for_queue(&relay_work_q, &periodic_work,
                                  K_MSEC(RELAY_CONN_SETTLE_MS));
        return;
    }

    /* ---- ACTIVE PHASE: sync, discover, write ---- */

    /* Sync relay array with current connections (detect disconnects,
     * pick up reconnections) */
    sync_relay_connections();

    /* Count connection state */
    int connected = 0;
    int ready = 0;
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn != NULL) {
            connected++;
            if (relays[i].bat_ready) {
                ready++;
            }
        }
    }

    /* Try discovery on undiscovered relays (one at a time) */
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == NULL) {
            continue;
        }
        if (!relays[i].bat_ready && try_discovery_step(&relays[i])) {
            LOG_INF("relay: started discovery on slot %d", i);
            break; /* one discovery at a time */
        }
    }

    bool all_ready = (connected >= ZMK_SPLIT_BLE_PERIPHERAL_COUNT) &&
                     (ready == connected);

    /* Log relay state for diagnostics */
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn != NULL) {
            LOG_INF("relay: slot %d: ready=%d disc_done=%d handle=%u",
                    i, relays[i].bat_ready, relays[i].bat_discovery_done,
                    relays[i].bat_char_handle);
        }
    }

    /* Write cached state to all discovered relays */
    for (int r = 0; r < ARRAY_SIZE(relays); r++) {
        struct peripheral_relay *relay = &relays[r];

        for (int src = 0; src < ARRAY_SIZE(battery_cache); src++) {
            write_to_relay(relay, (uint8_t)src, battery_cache[src]);
            k_msleep(RELAY_WRITE_SPACING_MS);
        }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
        write_to_relay(relay, BATTERY_RELAY_SOURCE_DONGLE, dongle_battery_cache);
        k_msleep(RELAY_WRITE_SPACING_MS);
#endif

        write_to_relay(relay, BATTERY_RELAY_SOURCE_LAYER, layer_cache);
        k_msleep(RELAY_WRITE_SPACING_MS);
    }

    /* Reschedule: fast while discovering, slow once all relays are ready */
    int next_ms = all_ready ? RELAY_SLOW_MS : RELAY_FAST_MS;
    k_work_schedule_for_queue(&relay_work_q, &periodic_work, K_MSEC(next_ms));
}

/* -------------------------------------------------------------------------
 * ZMK event listener
 *
 * Battery: cache only — periodic broadcast handles delivery (30 s).
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
    LOG_INF("relay: central init, initial_delay=%d fast=%d slow=%d settle=%d",
            RELAY_INITIAL_DELAY_MS, RELAY_FAST_MS, RELAY_SLOW_MS, RELAY_CONN_SETTLE_MS);
    k_work_queue_start(&relay_work_q, relay_work_q_stack,
                       K_THREAD_STACK_SIZEOF(relay_work_q_stack),
                       K_PRIO_PREEMPT(10), /* low priority — never starve ZMK */
                       NULL);

    /* Start polling for connections.  During the waiting phase, we only
     * COUNT connections (no bt_conn_ref, no GATT) to ensure zero
     * interference with ZMK's scanning and connection setup. */
    k_work_schedule_for_queue(&relay_work_q, &periodic_work,
                              K_MSEC(RELAY_INITIAL_DELAY_MS));
    return 0;
}

SYS_INIT(relay_central_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
