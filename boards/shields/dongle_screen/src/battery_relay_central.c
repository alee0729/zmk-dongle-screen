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
 *   3. Subscribes to ZMK battery and keypress events.
 *   4. On every battery change, writes struct battery_relay_data to each
 *      connected peripheral that has been successfully discovered.
 *      - Left battery (source 0) and dongle battery relay without an idle gate.
 *      - Right battery (sources > 0) is gated: only relayed after both shields
 *        have been idle (no keypresses) for RIGHT_BAT_IDLE_THRESHOLD_MS (30 s).
 *        This prevents GATT write-without-response PDUs from competing with
 *        HID notifications sent the other way during active typing, fully
 *        unblocking keystroke transmission on both shields.
 *   5. Caches battery levels so newly-discovered peripherals get current state
 *      after GATT discovery completes.
 *   6. Periodically re-broadcasts cached battery state for resilience.
 *
 * Idle tracking
 * -------------
 * Uses zmk_position_state_changed (fires on every key press from either
 * shield) to record last_active_ms.  This is more reliable than the
 * transition-based zmk_activity_state_changed, which only fires on
 * IDLE->ACTIVE edges and can be suppressed by continuous touchpad/sensor
 * events keeping the ZMK activity timer alive.
 * On each key press: update last_active_ms, cancel any pending flush,
 * reschedule flush for last_active_ms + RIGHT_BAT_IDLE_THRESHOLD_MS.
 *
 * Dirty-flag invariant
 * --------------------
 * bat_dirty[src] is set whenever new battery data arrives and is cleared ONLY
 * after bt_gatt_write_without_response() returns 0.  Going back to ACTIVE
 * (a keypress) cancels the scheduled idle flush but never clears dirty flags,
 * so the next idle window will retry all pending writes (fixes Issues 1 & 2).
 *
 * Discovery stagger
 * -----------------
 * All bt_gatt_discover retries use:
 *   RELAY_DISCOVERY_RETRY_DELAY_MS + idx * RELAY_DISCOVERY_STAGGER_MS
 * so that both peripherals never fire bt_gatt_discover simultaneously even
 * when they defer to the same idle window (fixes Issue 3).
 *
 * Layer relay is temporarily disabled (code left in place but commented out)
 * while battery relay timing is being stabilised.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

#include <zmk/events/battery_state_changed.h>
#include <zmk/events/position_state_changed.h>
/* Layer relay DISABLED: #include <zmk/events/layer_state_changed.h> */
#include <zmk/event_manager.h>
#include <zmk/battery.h>
/* Layer relay DISABLED: #include <zmk/keymap.h> */

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* -------------------------------------------------------------------------
 * Idle threshold for right-battery relay
 *
 * Right-battery data (source index > 0) is only written to the relay
 * characteristic when no key has been pressed on either shield for at least
 * this many milliseconds.  Keeps GATT write-without-response PDUs off the
 * air during active typing.
 * ---------------------------------------------------------------------- */

#define RIGHT_BAT_IDLE_THRESHOLD_MS 30000

/*
 * Updated on every key-press event (zmk_position_state_changed, state=true).
 * Initialised to 0 so that at power-on both_shields_idle_enough() becomes
 * true 30 s after boot if no keys are pressed.
 */
static int64_t last_active_ms = 0;

static bool both_shields_idle_enough(void)
{
    return (k_uptime_get() - last_active_ms) >= RIGHT_BAT_IDLE_THRESHOLD_MS;
}

/* -------------------------------------------------------------------------
 * Cached state
 * ---------------------------------------------------------------------- */

static uint8_t battery_cache[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];
/* Layer relay DISABLED: static uint8_t layer_cache; */

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
static uint8_t dongle_battery_cache;
#endif

/* -------------------------------------------------------------------------
 * Per-peripheral relay state
 * ---------------------------------------------------------------------- */

struct peripheral_relay {
    struct bt_conn *conn;

    uint16_t bat_char_handle;
    bool     bat_ready;

    /* Layer relay DISABLED: bool layer_ready; */

    /*
     * Dirty flags — one per peripheral-battery source index, plus dongle.
     * Set when new data arrives or a write fails.
     * Cleared ONLY after bt_gatt_write_without_response() returns 0.
     * Keypresses cancel the flush work but never clear these flags.
     */
    bool bat_dirty[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    bool dongle_bat_dirty;
#endif

    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128              discover_uuid;
    struct k_work_delayable         discovery_work;
    uint8_t                         discover_retries;
};

#define RELAY_DISCOVERY_DELAY_MS       2000
#define RELAY_DISCOVERY_STAGGER_MS     2000
#define RELAY_DISCOVERY_MAX_RETRIES    5
#define RELAY_DISCOVERY_RETRY_DELAY_MS 2000
#define RELAY_PERIODIC_BROADCAST_MS    60000

/* Retry delay when flush_dirty leaves writes outstanding (e.g. bat_ready
 * false or -ENOBUFS).  Short enough to recover quickly, long enough not
 * to hammer the BLE stack. */
#define RELAY_FLUSH_RETRY_MS           5000

static struct peripheral_relay relays[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

static struct peripheral_relay *get_relay_by_conn(struct bt_conn *conn)
{
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == conn) return &relays[i];
    }
    return NULL;
}

static struct peripheral_relay *get_free_relay(void)
{
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == NULL) return &relays[i];
    }
    return NULL;
}

static int get_relay_index(const struct peripheral_relay *relay)
{
    return (int)(relay - relays);
}

/* -------------------------------------------------------------------------
 * Write helper — returns 0 on success, negative errno on failure.
 * ---------------------------------------------------------------------- */

static int write_battery_to_relay(struct peripheral_relay *relay,
                                   uint8_t source, uint8_t level)
{
    if (!relay->bat_ready || relay->conn == NULL) {
        return -ENOTCONN;
    }
    struct battery_relay_data data = { .source = source, .level = level };
    int err = bt_gatt_write_without_response(relay->conn, relay->bat_char_handle,
                                             &data, sizeof(data), false);
    if (err) {
        LOG_WRN("battery_relay: write src=%u failed: %d", source, err);
    }
    return err;
}

/*
 * Layer relay DISABLED:
 *
 * static void write_layer_to_relay(struct peripheral_relay *relay, uint8_t layer) { ... }
 * static void broadcast_layer(uint8_t layer) { ... }
 */

/* -------------------------------------------------------------------------
 * flush_dirty — push pending battery data to a single relay.
 *
 * Dirty flag cleared ONLY on successful write.
 * Right-battery sources (index > 0) skipped when not idle.
 * Returns true if any right-battery dirty flags remain after the flush.
 * ---------------------------------------------------------------------- */

static bool flush_dirty(struct peripheral_relay *relay)
{
    bool right_still_dirty = false;

    if (!relay->bat_ready || relay->conn == NULL) {
        /* Can't write yet; check if there is right-battery work pending */
        for (int src = 1; src < (int)ARRAY_SIZE(battery_cache); src++) {
            if (relay->bat_dirty[src]) {
                right_still_dirty = true;
                break;
            }
        }
        return right_still_dirty;
    }

    for (int src = 0; src < (int)ARRAY_SIZE(battery_cache); src++) {
        if (!relay->bat_dirty[src]) continue;
        if (src > 0 && !both_shields_idle_enough()) {
            right_still_dirty = true;
            continue;
        }
        if (write_battery_to_relay(relay, (uint8_t)src, battery_cache[src]) == 0) {
            relay->bat_dirty[src] = false;
        } else {
            if (src > 0) right_still_dirty = true;
        }
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (relay->dongle_bat_dirty) {
        if (write_battery_to_relay(relay, BATTERY_RELAY_SOURCE_DONGLE,
                                   dongle_battery_cache) == 0) {
            relay->dongle_bat_dirty = false;
        }
    }
#endif

    return right_still_dirty;
}

/* -------------------------------------------------------------------------
 * Idle-flush work
 * ---------------------------------------------------------------------- */

static void right_bat_idle_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(right_bat_idle_work, right_bat_idle_work_handler);

static void right_bat_idle_work_handler(struct k_work *work)
{
    if (!both_shields_idle_enough()) {
        return;
    }
    LOG_DBG("battery_relay: idle threshold reached, flushing right battery");

    bool any_remaining = false;
    for (int i = 0; i < (int)ARRAY_SIZE(relays); i++) {
        if (flush_dirty(&relays[i])) {
            any_remaining = true;
        }
    }

    /*
     * If any dirty flags remain (bat_ready was false or write returned
     * -ENOBUFS), retry after a short delay rather than waiting for the
     * next 60-s periodic.
     */
    if (any_remaining) {
        LOG_DBG("battery_relay: writes incomplete, retrying in %d ms",
                RELAY_FLUSH_RETRY_MS);
        k_work_schedule(&right_bat_idle_work, K_MSEC(RELAY_FLUSH_RETRY_MS));
    }
}

/*
 * Schedule right_bat_idle_work to fire at last_active_ms + threshold.
 * If that deadline is already past, fire immediately.
 * k_work_schedule is a no-op when the item is already pending at an earlier
 * deadline, which is correct here — keypresses reschedule explicitly.
 */
static void schedule_right_bat_idle_flush(void)
{
    int64_t deadline = last_active_ms + RIGHT_BAT_IDLE_THRESHOLD_MS;
    int64_t now      = k_uptime_get();
    int64_t delay    = deadline - now;

    if (delay <= 0) {
        k_work_schedule(&right_bat_idle_work, K_NO_WAIT);
    } else {
        k_work_schedule(&right_bat_idle_work, K_MSEC((uint32_t)delay));
    }
}

/* -------------------------------------------------------------------------
 * broadcast_battery
 *
 * Source 0 (left) and dongle battery written immediately.
 * Sources > 0 (right) deferred until idle threshold is met.
 * Any failure (write error or not-ready relay) marks dirty and ensures the
 * idle flush is scheduled.
 * ---------------------------------------------------------------------- */

static void broadcast_battery(uint8_t source, uint8_t level)
{
    bool is_right  = (source < (uint8_t)ARRAY_SIZE(battery_cache) && source > 0);
    bool idle_ok   = both_shields_idle_enough();
    bool any_dirty = false;

    for (int i = 0; i < (int)ARRAY_SIZE(relays); i++) {
        if (is_right && !idle_ok) {
            /* Not idle yet — defer */
            if (source < (uint8_t)ARRAY_SIZE(relays[i].bat_dirty)) {
                relays[i].bat_dirty[source] = true;
                any_dirty = true;
            }
            continue;
        }
        int err = write_battery_to_relay(&relays[i], source, level);
        if (err != 0 && source < (uint8_t)ARRAY_SIZE(relays[i].bat_dirty)) {
            relays[i].bat_dirty[source] = true;
            if (is_right) any_dirty = true;
        }
    }

    /*
     * Schedule idle flush whenever any right-battery dirty flag was set,
     * whether because of the idle gate or a failed live write.
     */
    if (is_right && any_dirty) {
        schedule_right_bat_idle_flush();
    }
}

/* -------------------------------------------------------------------------
 * push_cached_state — called once after GATT discovery succeeds.
 * ---------------------------------------------------------------------- */

static void push_cached_state(struct peripheral_relay *relay)
{
    for (int i = 0; i < (int)ARRAY_SIZE(battery_cache); i++) {
        if (battery_cache[i] > 0) {
            relay->bat_dirty[i] = true;
        }
    }
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (dongle_battery_cache > 0) {
        relay->dongle_bat_dirty = true;
    }
#endif

    if (flush_dirty(relay)) {
        /* Right-battery dirty remains — arm the idle flush */
        schedule_right_bat_idle_flush();
    }

    /* Layer relay DISABLED: write_layer_to_relay(relay, layer_cache); */
}

/* -------------------------------------------------------------------------
 * GATT discovery
 * ---------------------------------------------------------------------- */

static uint8_t battery_discover_func(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params)
{
    struct peripheral_relay *relay =
        CONTAINER_OF(params, struct peripheral_relay, discover_params);
    int idx = get_relay_index(relay);

    if (!attr) {
        if (!relay->bat_ready) {
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
            if (relay->discover_retries > 0) {
                relay->discover_retries--;
                k_work_schedule(&relay->discovery_work,
                                K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS +
                                       (uint32_t)idx * RELAY_DISCOVERY_STAGGER_MS));
            } else {
                LOG_ERR("battery_relay: giving up discovery on conn %p", (void *)conn);
            }
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready       = true;
    LOG_INF("battery_relay: characteristic found, handle=%u", relay->bat_char_handle);

    k_work_schedule(&relay->discovery_work, K_NO_WAIT);
    return BT_GATT_ITER_STOP;
}

static void start_battery_discovery(struct peripheral_relay *relay)
{
    int idx = get_relay_index(relay);

    memcpy(&relay->discover_uuid, BATTERY_RELAY_CHAR_UUID,
           sizeof(relay->discover_uuid));
    relay->discover_params.uuid         = &relay->discover_uuid.uuid;
    relay->discover_params.func         = battery_discover_func;
    relay->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    relay->discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    relay->discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(relay->conn, &relay->discover_params);
    if (err) {
        LOG_ERR("battery_relay: bt_gatt_discover failed: %d (retries left %u)",
                err, relay->discover_retries);
        if (relay->discover_retries > 0 &&
            (err == -EBUSY || err == -ENOMEM || err == -EAGAIN)) {
            relay->discover_retries--;
            k_work_schedule(&relay->discovery_work,
                            K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS +
                                   (uint32_t)idx * RELAY_DISCOVERY_STAGGER_MS));
        }
    }
}

static void discovery_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay *relay =
        CONTAINER_OF(dwork, struct peripheral_relay, discovery_work);

    if (relay->conn == NULL) return;

    if (!relay->bat_ready) {
        start_battery_discovery(relay);
    } else {
        push_cached_state(relay);
    }
}

/* -------------------------------------------------------------------------
 * Periodic re-broadcast
 * ---------------------------------------------------------------------- */

static void periodic_broadcast_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(periodic_broadcast_work, periodic_broadcast_handler);

static void periodic_broadcast_handler(struct k_work *work)
{
    for (int src = 0; src < (int)ARRAY_SIZE(battery_cache); src++) {
        if (battery_cache[src] > 0) {
            broadcast_battery((uint8_t)src, battery_cache[src]);
        }
    }
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (dongle_battery_cache > 0) {
        broadcast_battery(BATTERY_RELAY_SOURCE_DONGLE, dongle_battery_cache);
    }
#endif
    /* Layer relay DISABLED: broadcast_layer(layer_cache); */
    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
}

/* -------------------------------------------------------------------------
 * BT connection callbacks
 * ---------------------------------------------------------------------- */

static void relay_connected(struct bt_conn *conn, uint8_t conn_err)
{
    if (conn_err) return;

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) return;
    if (info.role != BT_CONN_ROLE_CENTRAL) return;

    struct peripheral_relay *relay = get_free_relay();
    if (!relay) {
        LOG_WRN("battery_relay: no free relay slot");
        return;
    }

    relay->conn             = bt_conn_ref(conn);
    relay->bat_ready        = false;
    relay->bat_char_handle  = 0;
    relay->discover_retries = RELAY_DISCOVERY_MAX_RETRIES;
    memset(relay->bat_dirty, 0, sizeof(relay->bat_dirty));
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    relay->dongle_bat_dirty = false;
#endif
    /* Layer relay DISABLED: relay->layer_ready = false; */

    k_work_init_delayable(&relay->discovery_work, discovery_work_handler);

    uint32_t delay = RELAY_DISCOVERY_DELAY_MS +
                     (uint32_t)get_relay_index(relay) * RELAY_DISCOVERY_STAGGER_MS;
    k_work_schedule(&relay->discovery_work, K_MSEC(delay));
    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
}

static void relay_disconnected(struct bt_conn *conn, uint8_t reason)
{
    struct peripheral_relay *relay = get_relay_by_conn(conn);
    if (!relay) return;

    LOG_DBG("relay: peripheral disconnected (reason %u)", reason);
    k_work_cancel_delayable(&relay->discovery_work);
    bt_conn_unref(relay->conn);
    relay->conn            = NULL;
    relay->bat_ready       = false;
    relay->bat_char_handle = 0;
    /* Layer relay DISABLED: relay->layer_ready = false; */
}

BT_CONN_CB_DEFINE(battery_relay_conn_cb) = {
    .connected    = relay_connected,
    .disconnected = relay_disconnected,
};

/* -------------------------------------------------------------------------
 * ZMK event listener
 * ---------------------------------------------------------------------- */

static int relay_central_event_handler(const zmk_event_t *eh)
{
    /* Key press from either shield — reset the idle clock */
    const struct zmk_position_state_changed *pos_ev =
        as_zmk_position_state_changed(eh);
    if (pos_ev) {
        if (pos_ev->state) { /* state=true means pressed, not released */
            last_active_ms = k_uptime_get();
            /*
             * Cancel any pending flush and reschedule for exactly
             * RIGHT_BAT_IDLE_THRESHOLD_MS from now.  This ensures both
             * shields are quiet for the full threshold before writing.
             * Dirty flags are NOT cleared — the rescheduled work will
             * flush them.
             */
            k_work_cancel_delayable(&right_bat_idle_work);
            k_work_schedule(&right_bat_idle_work,
                            K_MSEC(RIGHT_BAT_IDLE_THRESHOLD_MS));
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev) {
        LOG_INF("relay_central: peripheral battery source=%u level=%u",
                periph_ev->source, periph_ev->state_of_charge);
        if (periph_ev->source < (uint8_t)ARRAY_SIZE(battery_cache)) {
            battery_cache[periph_ev->source] = periph_ev->state_of_charge;
        }
        broadcast_battery(periph_ev->source, periph_ev->state_of_charge);
        return ZMK_EV_EVENT_BUBBLE;
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    const struct zmk_battery_state_changed *bat_ev = as_zmk_battery_state_changed(eh);
    if (bat_ev) {
        dongle_battery_cache = bat_ev->state_of_charge;
        broadcast_battery(BATTERY_RELAY_SOURCE_DONGLE, bat_ev->state_of_charge);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    /*
     * Layer relay DISABLED:
     *
     * const struct zmk_layer_state_changed *layer_ev =
     *     as_zmk_layer_state_changed(eh);
     * if (layer_ev) {
     *     uint8_t highest = zmk_keymap_highest_layer_active();
     *     layer_cache = highest;
     *     broadcast_layer(highest);
     *     return ZMK_EV_EVENT_BUBBLE;
     * }
     */

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(battery_relay_central, relay_central_event_handler);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_position_state_changed);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_peripheral_battery_state_changed);
/* Layer relay DISABLED:
 * ZMK_SUBSCRIPTION(battery_relay_central, zmk_layer_state_changed);
 */

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
ZMK_SUBSCRIPTION(battery_relay_central, zmk_battery_state_changed);
#endif
