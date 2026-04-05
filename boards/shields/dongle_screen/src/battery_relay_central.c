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
 *   3. Subscribes to ZMK battery events.
 *   4. On every battery change, writes struct battery_relay_data to each
 *      connected peripheral that has been successfully discovered.
 *   5. Caches battery levels so newly-discovered peripherals get current state
 *      after GATT discovery completes.
 *   6. Periodically re-broadcasts cached battery state for resilience.
 *
 * Write reliability
 * -----------------
 * bt_gatt_write_without_response is non-blocking and does not compete with
 * HID keystroke data.  HID notifications flow shield→dongle; relay writes flow
 * dongle→shield — opposite directions on separate TX queues.
 *
 * Dirty-flag invariant
 * --------------------
 * bat_dirty[src] is set whenever new battery data arrives or a write fails,
 * and is cleared ONLY after bt_gatt_write_without_response() returns 0.
 * A 5-second periodic retry work item (bat_relay_retry_work) flushes any
 * remaining dirty writes, handling transient -ENOBUFS failures automatically.
 *
 * Idle gate for right-battery relay
 * ----------------------------------
 * Source 0 (left battery) and BATTERY_RELAY_SOURCE_DONGLE (dongle battery)
 * are relayed immediately.  Source > 0 (right battery) is gated behind a
 * RIGHT_BAT_IDLE_THRESHOLD_MS (30 s) both-shields-idle window.  This prevents
 * GATT relay writes from competing with HID keystroke notifications during
 * active typing.  The right_bat_idle_work fires after the threshold and
 * flushes all pending dirty writes.
 *
 * When going ACTIVE, only the scheduled flush work is cancelled; dirty flags
 * are preserved so the next idle window will flush them.  This avoids the
 * IDLE→ACTIVE→IDLE cycle silently discarding data.
 *
 * Discovery recovery
 * ------------------
 * Initial GATT discovery is attempted 2 s after connection (+ stagger per
 * slot) with up to RELAY_DISCOVERY_MAX_RETRIES retries on transient errors
 * (-EBUSY / -EAGAIN / -ENOMEM returned by bt_gatt_discover itself).
 *
 * ZMK’s own split/battery GATT operations run concurrently and can cause
 * -EBUSY, exhausting the initial retries before our discovery ever starts.
 * The periodic retry work detects this and re-schedules discovery every
 * BAT_RELAY_REDISCOVER_DELAY_MS — BUT ONLY when the discovery callback
 * never reported “not found” (discovery_gave_up == false).  If the ATT bearer
 * already completed a discovery and the server replied “no such attribute”,
 * it means the relay service is genuinely absent on that peripheral (e.g. the
 * right shield has no display) and we stop trying so we don’t spam its ATT
 * channel and disrupt keystroke traffic.
 *
 * Discovery stagger
 * -----------------
 * All bt_gatt_discover retries use:
 *   RELAY_DISCOVERY_RETRY_DELAY_MS + idx * RELAY_DISCOVERY_STAGGER_MS
 * so that both peripherals never fire bt_gatt_discover simultaneously.
 *
 * Layer relay is temporarily disabled (code left in place but commented out)
 * while battery relay is being stabilised.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

#include <zmk/events/battery_state_changed.h>
#include <zmk/events/activity_state_changed.h>
/* Layer relay DISABLED: #include <zmk/events/layer_state_changed.h> */
#include <zmk/event_manager.h>
#include <zmk/battery.h>
/* Layer relay DISABLED: #include <zmk/keymap.h> */

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

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

    /*
     * Set when ATT discovery completed but the relay characteristic was not
     * found on this peripheral (i.e. the server responded “not found”).
     * The periodic retry work skips re-discovery for this relay to avoid
     * spamming the ATT channel of peripherals without the relay service
     * (e.g. the right shield).
     * Reset to false on each new connection.
     */
    bool     discovery_gave_up;

    /*
     * Set the first time battery_discover_func is invoked, i.e. the moment
     * bt_gatt_discover() returned 0 and ATT-level discovery actually ran.
     * Until this flag is set, all failures have been transient call-level
     * errors (-EBUSY/-ENOMEM) that never reached the ATT bearer.
     *
     * The periodic retry work uses this to decide whether to re-schedule
     * discovery:
     *   - false  →  ATT never ran (pure EBUSY); safe to retry, no ATT
     *               traffic generated yet.
     *   - true   →  ATT ran at least once; the callback's own retry/
     *               gave_up logic handles everything from here.
     * Reset to false on each new connection.
     */
    bool     discovery_att_started;

    /* Layer relay DISABLED: bool layer_ready; */

    /*
     * Dirty flags — one per peripheral-battery source index, plus dongle.
     * Set when new data arrives or a write fails.
     * Cleared ONLY after bt_gatt_write_without_response() returns 0.
     * Source > 0 flags are also subject to the idle gate.
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

#define RELAY_DISCOVERY_DELAY_MS       20000
#define RELAY_DISCOVERY_STAGGER_MS     5000
#define RELAY_DISCOVERY_MAX_RETRIES    5
#define RELAY_DISCOVERY_RETRY_DELAY_MS 5000
#define RELAY_PERIODIC_BROADCAST_MS    60000

/* How often the retry work polls for pending dirty writes. */
#define BAT_RELAY_RETRY_MS             30000

/*
 * How long to wait before re-attempting GATT discovery from the periodic
 * retry work, for relays where initial discovery exhausted EBUSY retries
 * without the ATT discovery ever completing.  Gives ZMK’s own GATT
 * operations (split-service setup, battery-level fetching) time to finish.
 */
#define BAT_RELAY_REDISCOVER_DELAY_MS  10000

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
 * static void write_layer_to_relay(...) { ... }
 * static void broadcast_layer(uint8_t layer) { ... }
 */

/* -------------------------------------------------------------------------
 * Idle gate for right-battery relay
 *
 * Source 0 (left battery) and dongle battery relay immediately.
 * Source > 0 (right battery) is held until both shields have been idle
 * for RIGHT_BAT_IDLE_THRESHOLD_MS to avoid competing with HID traffic.
 * ---------------------------------------------------------------------- */

static int64_t last_active_ms = 0;
#define RIGHT_BAT_IDLE_THRESHOLD_MS 30000

static bool both_shields_idle_enough(void)
{
    return (k_uptime_get() - last_active_ms) >= RIGHT_BAT_IDLE_THRESHOLD_MS;
}

/* Forward declarations needed before K_WORK_DELAYABLE_DEFINE */
static void right_bat_idle_work_handler(struct k_work *work);
static bool flush_dirty(struct peripheral_relay *relay);

K_WORK_DELAYABLE_DEFINE(right_bat_idle_work, right_bat_idle_work_handler);

/*
 * Schedule the right-battery idle flush to fire when the idle threshold
 * is reached.  If already past the threshold, fire immediately (delay=0).
 */
static void schedule_right_bat_idle_flush(void)
{
    int64_t now      = k_uptime_get();
    int64_t deadline = last_active_ms + RIGHT_BAT_IDLE_THRESHOLD_MS;
    int64_t delay    = deadline - now;
    if (delay < 0) delay = 0;
    k_work_schedule(&right_bat_idle_work, K_MSEC((uint32_t)delay));
}

/* -------------------------------------------------------------------------
 * flush_dirty — push all pending battery data to a single relay.
 *
 * Dirty flag cleared ONLY on successful write.
 * Source > 0 (right battery) is also subject to the idle gate here, so
 * that the periodic retry work and post-discovery push don’t bypass it.
 * Returns true if any dirty flags remain.
 * ---------------------------------------------------------------------- */

static bool flush_dirty(struct peripheral_relay *relay)
{
    bool still_dirty = false;

    if (!relay->bat_ready || relay->conn == NULL) {
        for (int src = 0; src < (int)ARRAY_SIZE(battery_cache); src++) {
            if (relay->bat_dirty[src]) { still_dirty = true; break; }
        }
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
        if (relay->dongle_bat_dirty) still_dirty = true;
#endif
        return still_dirty;
    }

    for (int src = 0; src < (int)ARRAY_SIZE(battery_cache); src++) {
        if (!relay->bat_dirty[src]) continue;
        /* Right-battery sources (src > 0) are idle-gated */
        if (src > 0 && !both_shields_idle_enough()) {
            still_dirty = true;
            continue;
        }
        if (write_battery_to_relay(relay, (uint8_t)src, battery_cache[src]) == 0) {
            relay->bat_dirty[src] = false;
        } else {
            still_dirty = true;
        }
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (relay->dongle_bat_dirty) {
        if (write_battery_to_relay(relay, BATTERY_RELAY_SOURCE_DONGLE,
                                   dongle_battery_cache) == 0) {
            relay->dongle_bat_dirty = false;
        } else {
            still_dirty = true;
        }
    }
#endif

    return still_dirty;
}

/* -------------------------------------------------------------------------
 * right_bat_idle_work handler
 *
 * Fires after the idle threshold; flushes dirty right-battery writes on
 * all relays.  Guard checks both_shields_idle_enough() in case the work
 * was scheduled but activity resumed before it fired.
 * ---------------------------------------------------------------------- */

static void right_bat_idle_work_handler(struct k_work *work)
{
    if (!both_shields_idle_enough()) return;
    for (int i = 0; i < (int)ARRAY_SIZE(relays); i++) {
        flush_dirty(&relays[i]);
    }
}

/* -------------------------------------------------------------------------
 * Periodic retry work
 *
 * Fires every BAT_RELAY_RETRY_MS.
 *
 * 1. For any relay that is connected, has no characteristic yet, and has not
 *    confirmed absence (discovery_gave_up == false), re-schedule discovery.
 *    This recovers from exhausted initial retries due to ZMK’s concurrent
 *    GATT operations at connection time.  discovery_gave_up prevents
 *    re-hammering peripherals that genuinely lack the relay service.
 *
 * 2. Flush any dirty battery writes on every relay.
 * ---------------------------------------------------------------------- */

static void bat_relay_retry_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(bat_relay_retry_work, bat_relay_retry_handler);

static void bat_relay_retry_handler(struct k_work *work)
{
    for (int i = 0; i < (int)ARRAY_SIZE(relays); i++) {
        struct peripheral_relay *relay = &relays[i];

        /*
         * Re-attempt GATT discovery ONLY when:
         *  - peripheral is connected and characteristic not yet found
         *  - ATT-level discovery has never started (!discovery_att_started)
         *    — i.e. all previous attempts returned -EBUSY/-ENOMEM before
         *    reaching the ATT bearer.  Once ATT starts, battery_discover_func
         *    owns the retry/gave_up cycle; we must not interfere or we risk
         *    flooding the right shield with extra ATT requests.
         *  - no discovery work already pending / running
         */
        if (relay->conn != NULL && !relay->bat_ready &&
            !relay->discovery_att_started &&
            !k_work_delayable_is_pending(&relay->discovery_work)) {
            LOG_INF("battery_relay[%d]: re-scheduling discovery from retry handler", i);
            relay->discover_retries = RELAY_DISCOVERY_MAX_RETRIES;
            k_work_schedule(&relay->discovery_work,
                            K_MSEC(BAT_RELAY_REDISCOVER_DELAY_MS +
                                   (uint32_t)i * RELAY_DISCOVERY_STAGGER_MS));
        }

        flush_dirty(relay);
    }
    k_work_schedule(&bat_relay_retry_work, K_MSEC(BAT_RELAY_RETRY_MS));
}

/* -------------------------------------------------------------------------
 * broadcast_battery — write to all relays, mark dirty on failure.
 *
 * Source 0 (left battery) and dongle battery relay immediately.
 * Source > 0 (right battery) is idle-gated: if both shields have not been
 * idle for RIGHT_BAT_IDLE_THRESHOLD_MS the write is deferred by marking
 * dirty and scheduling right_bat_idle_work.
 * ---------------------------------------------------------------------- */

static void broadcast_battery(uint8_t source, uint8_t level)
{
    for (int i = 0; i < (int)ARRAY_SIZE(relays); i++) {
        /* Non-left-battery peripheral sources need the idle gate */
        if (source > 0 && source != BATTERY_RELAY_SOURCE_DONGLE) {
            if (!both_shields_idle_enough()) {
                if (source < (uint8_t)ARRAY_SIZE(relays[i].bat_dirty)) {
                    relays[i].bat_dirty[source] = true;
                }
                schedule_right_bat_idle_flush();
                continue;
            }
        }
        int err = write_battery_to_relay(&relays[i], source, level);
        if (err != 0 && source < (uint8_t)ARRAY_SIZE(relays[i].bat_dirty)) {
            relays[i].bat_dirty[source] = true;
        }
    }
}

/* -------------------------------------------------------------------------
 * push_cached_state — called once after GATT discovery succeeds.
 * Marks all cached sources dirty and flushes immediately.
 * Source > 0 dirty flags will be held by flush_dirty until idle.
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

    flush_dirty(relay);

    /* Ensure src>0 dirty flags get flushed on next idle window */
    schedule_right_bat_idle_flush();

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

    /*
     * Mark that ATT-level discovery has run at least once.  From this point
     * the callback's own retry/gave_up logic controls everything, and the
     * periodic retry work will no longer re-schedule discovery for this relay.
     */
    relay->discovery_att_started = true;

    if (!attr) {
        if (!relay->bat_ready) {
            if (relay->discover_retries > 0) {
                relay->discover_retries--;
                k_work_schedule(&relay->discovery_work,
                                K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS +
                                       (uint32_t)idx * RELAY_DISCOVERY_STAGGER_MS));
            } else {
                /*
                 * ATT discovery completed and the server replied “not found”
                 * after all retries.  Mark gave_up so the periodic retry
                 * work does not repeatedly re-discover on this connection
                 * (which would spam ATT and disrupt keystroke traffic on
                 * peripherals without the relay service).
                 */
                LOG_INF("battery_relay[%d]: characteristic not found, stopping retries",
                        idx);
                relay->discovery_gave_up = true;
            }
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready       = true;
    LOG_INF("battery_relay[%d]: characteristic found, handle=%u",
            idx, relay->bat_char_handle);

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
        LOG_DBG("battery_relay[%d]: bt_gatt_discover err=%d (retries=%u)",
                idx, err, relay->discover_retries);
        if (relay->discover_retries > 0 &&
            (err == -EBUSY || err == -ENOMEM || err == -EAGAIN)) {
            relay->discover_retries--;
            k_work_schedule(&relay->discovery_work,
                            K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS +
                                   (uint32_t)idx * RELAY_DISCOVERY_STAGGER_MS));
        }
        /*
         * On non-transient errors or exhausted retries: discovery_gave_up
         * stays false so the periodic retry work will re-attempt later.
         * Only battery_discover_func sets discovery_gave_up — and only when
         * the ATT bearer has completed discovery and the server replied
         * "not found".  This prevents ATT spam on peripherals that genuinely
         * lack the relay service (e.g. the right shield) while still allowing
         * recovery for the left shield if the initial attempts hit EBUSY.
         */
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

    relay->conn                  = bt_conn_ref(conn);
    relay->bat_ready             = false;
    relay->bat_char_handle       = 0;
    relay->discovery_gave_up     = false;
    relay->discovery_att_started = false;
    relay->discover_retries      = RELAY_DISCOVERY_MAX_RETRIES;
    memset(relay->bat_dirty, 0, sizeof(relay->bat_dirty));
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    relay->dongle_bat_dirty = false;
#endif
    /* Layer relay DISABLED: relay->layer_ready = false; */

    k_work_init_delayable(&relay->discovery_work, discovery_work_handler);

    uint32_t delay = RELAY_DISCOVERY_DELAY_MS +
                     (uint32_t)get_relay_index(relay) * RELAY_DISCOVERY_STAGGER_MS;
    k_work_schedule(&relay->discovery_work, K_MSEC(delay));

    /* Start the background work items (no-op if already running). */
    k_work_schedule(&periodic_broadcast_work, K_MSEC(RELAY_PERIODIC_BROADCAST_MS));
    k_work_schedule(&bat_relay_retry_work, K_MSEC(BAT_RELAY_RETRY_MS));
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

    const struct zmk_activity_state_changed *act_ev = as_zmk_activity_state_changed(eh);
    if (act_ev) {
        if (act_ev->state == ZMK_ACTIVITY_ACTIVE) {
            last_active_ms = k_uptime_get();
            /*
             * Cancel the scheduled idle flush — dirty flags are preserved
             * intentionally so the next idle window can flush them.
             * Do NOT clear dirty flags here (fixes IDLE→ACTIVE purge issue).
             */
            k_work_cancel_delayable(&right_bat_idle_work);
        } else {
            /* IDLE or SLEEP: schedule flush after threshold */
            schedule_right_bat_idle_flush();
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

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
ZMK_SUBSCRIPTION(battery_relay_central, zmk_peripheral_battery_state_changed);
ZMK_SUBSCRIPTION(battery_relay_central, zmk_activity_state_changed);
/* Layer relay DISABLED:
 * ZMK_SUBSCRIPTION(battery_relay_central, zmk_layer_state_changed);
 */

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
ZMK_SUBSCRIPTION(battery_relay_central, zmk_battery_state_changed);
#endif
