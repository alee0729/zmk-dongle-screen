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
 *   3. Subscribes to ZMK battery and activity events.
 *   4. On every battery change, writes struct battery_relay_data to each
 *      connected peripheral that has been successfully discovered.
 *      - Left battery (source 0) and dongle battery relay without an idle gate.
 *      - Right battery (sources > 0) is gated: only relayed after both shields
 *        have been idle for RIGHT_BAT_IDLE_THRESHOLD_MS (30 s).  This prevents
 *        GATT write-without-response PDUs from competing with HID notifications
 *        sent the other way during active typing, fully unblocking keystroke
 *        transmission on both shields.
 *   5. Caches battery levels so newly-discovered peripherals get current state
 *      after GATT discovery completes.
 *   6. Periodically re-broadcasts cached battery state for resilience.
 *
 * Dirty-flag invariant
 * --------------------
 * bat_dirty[src] is set whenever new battery data arrives and is cleared ONLY
 * after bt_gatt_write_without_response() returns 0.  Going back to ACTIVE
 * cancels the scheduled idle flush but never clears dirty flags, so the next
 * idle window will retry all pending writes (fixes Issues 1 and 2).
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
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>
/* Layer relay DISABLED: #include <zmk/events/layer_state_changed.h> */
#include <zmk/event_manager.h>
#include <zmk/battery.h>
/* Layer relay DISABLED: #include <zmk/keymap.h> */

#include "battery_relay_central.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* -------------------------------------------------------------------------
 * Idle threshold for right-battery relay
 *
 * Right-battery data (source index > 0, i.e. anything other than the left
 * shield) is only written to the relay characteristic when both shields have
 * been idle for at least this many milliseconds.  This keeps GATT
 * write-without-response PDUs off the air during active typing and prevents
 * TX-queue contention with HID notifications going the other direction.
 * ---------------------------------------------------------------------- */

#define RIGHT_BAT_IDLE_THRESHOLD_MS 30000

/*
 * Tracks the k_uptime_get() value of the most recent ZMK_ACTIVITY_ACTIVE
 * event.  Initialised to 0 so that at power-on both_shields_idle_enough()
 * returns false for the first 30 s, giving the BT stack time to settle
 * before any battery writes are attempted.
 */
static int64_t last_active_ms = 0;

static bool both_shields_idle_enough(void)
{
    return (k_uptime_get() - last_active_ms) >= RIGHT_BAT_IDLE_THRESHOLD_MS;
}

/* -------------------------------------------------------------------------
 * Cached state — sent to peripherals after GATT discovery completes and
 * periodically re-broadcast for resilience.
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

    /* Battery relay GATT characteristic handle */
    uint16_t bat_char_handle;
    bool     bat_ready;

    /* Layer relay DISABLED: bool layer_ready; */

    /*
     * Dirty flags — one per peripheral-battery source index, plus dongle.
     *
     * Set when new data arrives (or when bat_ready is false at write time).
     * Cleared ONLY after bt_gatt_write_without_response() returns 0.
     * Going ACTIVE cancels the flush work item but never touches these flags,
     * so the next idle window will flush all accumulated data.
     */
    bool bat_dirty[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];
#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    bool dongle_bat_dirty;
#endif

    /* Discovery state */
    struct bt_gatt_discover_params discover_params;
    struct bt_uuid_128              discover_uuid;
    struct k_work_delayable         discovery_work;
    uint8_t                         discover_retries;
};

/*
 * Initial delay before starting GATT discovery after a peripheral connects.
 * ZMK's split stack also does its own GATT work on connect; 2 s gives it
 * time to finish before we start our discovery.
 */
#define RELAY_DISCOVERY_DELAY_MS       2000

/*
 * Per-slot stagger added on top of every discovery (re)schedule:
 *   base_delay_ms + relay_index * RELAY_DISCOVERY_STAGGER_MS
 * Ensures peripheral 0 and peripheral 1 never call bt_gatt_discover at the
 * same instant, even when both defer to the same idle window.
 */
#define RELAY_DISCOVERY_STAGGER_MS     2000

#define RELAY_DISCOVERY_MAX_RETRIES    5

/* Base delay between discovery retries; per-index stagger is added on top. */
#define RELAY_DISCOVERY_RETRY_DELAY_MS 2000

#define RELAY_PERIODIC_BROADCAST_MS    60000

static struct peripheral_relay relays[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

static struct peripheral_relay *get_relay_by_conn(struct bt_conn *conn)
{
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == conn) {
            return &relays[i];
        }
    }
    return NULL;
}

static struct peripheral_relay *get_free_relay(void)
{
    for (int i = 0; i < ARRAY_SIZE(relays); i++) {
        if (relays[i].conn == NULL) {
            return &relays[i];
        }
    }
    return NULL;
}

static int get_relay_index(const struct peripheral_relay *relay)
{
    return (int)(relay - relays);
}

/* -------------------------------------------------------------------------
 * Write helper
 *
 * Returns 0 on success, negative errno on failure.  Callers use the return
 * value to decide whether to (re-)set a dirty flag rather than assuming the
 * write succeeded.
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
 * static void write_layer_to_relay(struct peripheral_relay *relay, uint8_t layer)
 * {
 *     if (!relay->layer_ready || relay->conn == NULL) { return; }
 *     struct battery_relay_data data = {
 *         .source = BATTERY_RELAY_SOURCE_LAYER,
 *         .level  = layer,
 *     };
 *     bt_gatt_write_without_response(relay->conn, relay->bat_char_handle,
 *                                    &data, sizeof(data), false);
 * }
 *
 * static void broadcast_layer(uint8_t layer)
 * {
 *     for (int i = 0; i < ARRAY_SIZE(relays); i++) {
 *         write_layer_to_relay(&relays[i], layer);
 *     }
 * }
 */

/* -------------------------------------------------------------------------
 * flush_dirty — push pending battery data to a single relay.
 *
 * The dirty flag for each source is cleared ONLY after a successful write.
 * Right-battery sources (index > 0) are skipped unless the idle threshold
 * has been met; the right_bat_idle_work work item will retry them later.
 *
 * This function is safe to call at any time; it is a no-op if bat_ready is
 * false or the connection is gone.
 * ---------------------------------------------------------------------- */

static void flush_dirty(struct peripheral_relay *relay)
{
    if (!relay->bat_ready || relay->conn == NULL) {
        return;
    }

    for (int src = 0; src < (int)ARRAY_SIZE(battery_cache); src++) {
        if (!relay->bat_dirty[src]) {
            continue;
        }
        /* Right-battery sources require the idle threshold to be met */
        if (src > 0 && !both_shields_idle_enough()) {
            continue; /* keep dirty; right_bat_idle_work will retry */
        }
        if (write_battery_to_relay(relay, (uint8_t)src, battery_cache[src]) == 0) {
            relay->bat_dirty[src] = false;
        }
        /* On write failure the flag stays set; next flush will retry. */
    }

#if IS_ENABLED(CONFIG_ZMK_DONGLE_DISPLAY_DONGLE_BATTERY)
    if (relay->dongle_bat_dirty) {
        if (write_battery_to_relay(relay, BATTERY_RELAY_SOURCE_DONGLE,
                                   dongle_battery_cache) == 0) {
            relay->dongle_bat_dirty = false;
        }
    }
#endif
}

/* -------------------------------------------------------------------------
 * Idle-flush work — fires RIGHT_BAT_IDLE_THRESHOLD_MS after the most recent
 * ZMK_ACTIVITY_ACTIVE event, i.e. when both shields have been quiet long
 * enough.  Calls flush_dirty on every relay so right-battery dirty flags are
 * sent.
 * ---------------------------------------------------------------------- */

static void right_bat_idle_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(right_bat_idle_work, right_bat_idle_work_handler);

static void right_bat_idle_work_handler(struct k_work *work)
{
    if (!both_shields_idle_enough()) {
        /* Raced with a ZMK_ACTIVITY_ACTIVE event — nothing to do. */
        return;
    }
    LOG_DBG("battery_relay: idle threshold reached, flushing right battery");
    for (int i = 0; i < (int)ARRAY_SIZE(relays); i++) {
        flush_dirty(&relays[i]);
    }
}

/*
 * Schedule right_bat_idle_work to fire at last_active_ms + threshold.
 * If that deadline is already in the past, fire immediately.
 * k_work_schedule is a no-op when the item is already pending at an earlier
 * deadline, which is the correct behaviour here.
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
 * broadcast_battery — write to all relays; gate right battery on idle.
 *
 * Source 0 (left battery) and dongle battery are written immediately.
 * Sources > 0 (right battery) are deferred until both shields have been idle
 * for RIGHT_BAT_IDLE_THRESHOLD_MS.  Any relay that cannot receive right now
 * (bat_ready=false or write returns an error) gets its dirty flag set so
 * flush_dirty will retry on the next opportunity.
 * ---------------------------------------------------------------------- */

static void broadcast_battery(uint8_t source, uint8_t level)
{
    bool is_right = (source < (uint8_t)ARRAY_SIZE(battery_cache) && source > 0);
    bool idle_ok  = both_shields_idle_enough();

    for (int i = 0; i < (int)ARRAY_SIZE(relays); i++) {
        if (is_right && !idle_ok) {
            /* Defer: mark dirty so the idle flush will send it later */
            if (source < (uint8_t)ARRAY_SIZE(relays[i].bat_dirty)) {
                relays[i].bat_dirty[source] = true;
            }
            continue;
        }
        int err = write_battery_to_relay(&relays[i], source, level);
        if (err != 0 && source < (uint8_t)ARRAY_SIZE(relays[i].bat_dirty)) {
            relays[i].bat_dirty[source] = true;
        }
    }

    /* Ensure the idle-flush work item is scheduled if we deferred anything */
    if (is_right && !idle_ok) {
        schedule_right_bat_idle_flush();
    }
}

/* -------------------------------------------------------------------------
 * push_cached_state — called once after GATT discovery succeeds.
 *
 * Marks all cached sources dirty, then flushes whatever is currently
 * eligible (left battery + dongle battery go immediately; right battery only
 * if already past the idle threshold).  Any remaining dirty flags will be
 * cleared by the next right_bat_idle_work firing.
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

    /* Flush whatever is currently eligible */
    flush_dirty(relay);

    /* If any right-battery sources are still dirty, arm the idle flush */
    for (int i = 1; i < (int)ARRAY_SIZE(battery_cache); i++) {
        if (relay->bat_dirty[i]) {
            schedule_right_bat_idle_flush();
            break;
        }
    }

    /* Layer relay DISABLED: write_layer_to_relay(relay, layer_cache); */
}

/* -------------------------------------------------------------------------
 * GATT discovery — battery characteristic only
 * ---------------------------------------------------------------------- */

static uint8_t battery_discover_func(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params)
{
    struct peripheral_relay *relay =
        CONTAINER_OF(params, struct peripheral_relay, discover_params);
    int idx = get_relay_index(relay);

    if (!attr) {
        /* Discovery scan completed without finding the characteristic */
        if (!relay->bat_ready) {
            LOG_DBG("battery_relay: characteristic not found on conn %p", (void *)conn);
            if (relay->discover_retries > 0) {
                relay->discover_retries--;
                /*
                 * Always add per-index stagger so that both peripherals
                 * never schedule bt_gatt_discover at the same instant,
                 * even when they defer to the same idle window (Issue 3).
                 */
                k_work_schedule(&relay->discovery_work,
                                K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS +
                                       (uint32_t)idx * RELAY_DISCOVERY_STAGGER_MS));
            } else {
                LOG_ERR("battery_relay: giving up discovery on conn %p",
                        (void *)conn);
            }
        }
        /*
         * If bat_ready is already true we returned ITER_STOP when we found
         * the attribute and already scheduled push_cached_state; nothing
         * more to do here.
         */
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    relay->bat_char_handle = chrc->value_handle;
    relay->bat_ready       = true;
    LOG_INF("battery_relay: characteristic found, handle=%u",
            relay->bat_char_handle);

    /*
     * Schedule push_cached_state via the work queue so it runs outside the
     * BT callback context.  K_NO_WAIT fires on the next work-queue tick.
     */
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
            /*
             * Per-index stagger on every retry path, not just the
             * characteristic-not-found path, so that simultaneous -EBUSY
             * retries from both peripherals are also spread apart.
             */
            k_work_schedule(&relay->discovery_work,
                            K_MSEC(RELAY_DISCOVERY_RETRY_DELAY_MS +
                                   (uint32_t)idx * RELAY_DISCOVERY_STAGGER_MS));
        }
    }
}

static void discovery_work_handler(struct k_work *work)
{
    struct k_work_delayable  *dwork = k_work_delayable_from_work(work);
    struct peripheral_relay  *relay =
        CONTAINER_OF(dwork, struct peripheral_relay, discovery_work);

    if (relay->conn == NULL) {
        return;
    }

    if (!relay->bat_ready) {
        start_battery_discovery(relay);
    } else {
        /*
         * Battery characteristic found.  Push all cached state to this
         * relay: left battery + dongle battery go immediately; right battery
         * will be sent on the next idle window if the threshold hasn't been
         * reached yet.
         *
         * Layer relay DISABLED — no layer_ready step needed.
         */
        push_cached_state(relay);
    }
}

/* -------------------------------------------------------------------------
 * Periodic re-broadcast of cached battery state
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
    if (conn_err) {
        return;
    }

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) < 0 || info.type != BT_CONN_TYPE_LE) {
        return;
    }
    /* Only handle central-role connections (to split peripherals). */
    if (info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }

    struct peripheral_relay *relay = get_free_relay();
    if (!relay) {
        LOG_WRN("battery_relay: no free relay slot for new connection");
        return;
    }

    relay->conn            = bt_conn_ref(conn);
    relay->bat_ready       = false;
    relay->bat_char_handle = 0;
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
    if (!relay) {
        return;
    }

    LOG_DBG("relay: peripheral disconnected (reason %u), clearing slot", reason);
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
 * ZMK event listener — battery changes + activity state.
 * Layer events are DISABLED.
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

    const struct zmk_activity_state_changed *act_ev =
        as_zmk_activity_state_changed(eh);
    if (act_ev) {
        if (act_ev->state == ZMK_ACTIVITY_ACTIVE) {
            /*
             * Record the moment the system became active and cancel any
             * pending right-battery flush.  Dirty flags are intentionally
             * NOT cleared here — the next idle window will flush them.
             * This fixes Issue 2 (IDLE→ACTIVE purge losing data).
             */
            last_active_ms = k_uptime_get();
            k_work_cancel_delayable(&right_bat_idle_work);
        } else {
            /*
             * ZMK_ACTIVITY_IDLE or ZMK_ACTIVITY_SLEEP: schedule the
             * right-battery flush to fire at last_active_ms + threshold.
             */
            schedule_right_bat_idle_flush();
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

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
