/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * dabase_v2: TPS43 power rail policy.
 *
 * Ported from the TrackPoint-era policy (Exp81,
 * kb_zmk_ps2_mouse_trackpoint_driver_pr/src/power/tp_power_policy.c) with the
 * piece the IQS5xx needs and a nub did not: the pad loses its register
 * configuration when the rail is cut, so every wake re-runs the full init
 * sequence (EVENT_MODE off, gestures off, report rate, SETUP_COMPLETE).
 *
 *   ACTIVE  -> rail on, wait boot-mute-ms, reconfigure the pad, poll again
 *   IDLE    -> rail off (ZMK idle, CONFIG_ZMK_IDLE_TIMEOUT)
 *   SLEEP   -> same as IDLE
 *   USB     -> never cut: VBUS has no battery budget
 *
 * NOTE: pin isolation (applying the I2C sleep pinctrl so the internal
 * pull-ups cannot leak into the unpowered pad) is NOT in this revision: the
 * first attempt that touched pinctrl from here wedged the console. Re-add it
 * as a separate, isolated change once this base policy is proven.
 *
 * Why the rail GPIO is driven directly instead of ext_power_disable():
 * ext_power_generic_disable() schedules a settings save on every call, so an
 * idle cycle would churn the settings partition and can persist
 * "ext_power = off" across a reboot (it did, in the PS/2 era).
 */

#define DT_DRV_COMPAT dabase_tps43_rail

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>
#endif

#include "tps43.h"

LOG_MODULE_DECLARE(tps43, CONFIG_INPUT_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(dabase_tps43_rail) && IS_ENABLED(CONFIG_INPUT_TPS43)

#define RAIL_NODE DT_INST(0, dabase_tps43_rail)

static const struct gpio_dt_spec rail = GPIO_DT_SPEC_INST_GET(0, gpios);
static const struct device *const pad = DEVICE_DT_GET(DT_NODELABEL(tps43_trackpad));

static bool rail_powered;
static struct k_work_delayable boot_mute_work;

static void rail_cut(void) {
    if (!rail_powered) {
        return;
    }
    rail_powered = false;

    /* Tell the driver first so it stops polling, then cut the rail and take
     * the bus lines low-leak. */
    (void)tps43_rail_power(pad, false);
    (void)gpio_pin_set_dt(&rail, 0);

    LOG_INF("TP rail: off (idle)");
}

static void boot_mute_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!rail_powered) {
        return;
    }

    int err = tps43_rail_power(pad, true);
    if (err != 0) {
        LOG_ERR("TP rail: pad did not come back (%d)", err);
    }
}

static void rail_arm(void) {
    if (rail_powered) {
        return;
    }
    rail_powered = true;

    (void)gpio_pin_set_dt(&rail, 1);

    LOG_INF("TP rail: on (%d ms mute)", DT_INST_PROP(0, boot_mute_ms));
    k_work_reschedule(&boot_mute_work, K_MSEC(DT_INST_PROP(0, boot_mute_ms)));
}

static bool usb_powered(void) {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    return zmk_usb_is_powered();
#else
    return false;
#endif
}

static int rail_activity_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    if (ev == NULL) {
        return -ENOTSUP;
    }

    if (ev->state == ZMK_ACTIVITY_ACTIVE || usb_powered()) {
        rail_arm();
    } else {
        rail_cut();
    }

    return 0;
}

ZMK_LISTENER(tps43_rail_policy, rail_activity_listener);
ZMK_SUBSCRIPTION(tps43_rail_policy, zmk_activity_state_changed);

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
static int rail_usb_listener(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);

    if (ev == NULL) {
        return -ENOTSUP;
    }

    if (ev->conn_state != ZMK_USB_CONN_NONE) {
        /* Plugging in while idle raises no activity event. */
        rail_arm();
    } else if (zmk_activity_get_state() != ZMK_ACTIVITY_ACTIVE) {
        rail_cut();
    }

    return 0;
}

ZMK_LISTENER(tps43_rail_usb, rail_usb_listener);
ZMK_SUBSCRIPTION(tps43_rail_usb, zmk_usb_conn_state_changed);
#endif

static int rail_policy_init(void) {
    if (!gpio_is_ready_dt(&rail) || !device_is_ready(pad)) {
        LOG_ERR("TP rail: rail GPIO or pad device not ready");
        return -ENODEV;
    }

    k_work_init_delayable(&boot_mute_work, boot_mute_handler);

    /* The devicetree gpio-hog has already driven the rail high before the
     * pad driver probed; from here on this policy owns the level. */
    rail_powered = true;

    return 0;
}

SYS_INIT(rail_policy_init, APPLICATION, 92);

#endif /* rail node present && INPUT_TPS43 */
