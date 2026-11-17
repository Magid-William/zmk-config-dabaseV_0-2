/*
 * dabase_v2: app-side layer gate for the TPS43 trackpoint velocity mode.
 *
 * The driver lives in a Zephyr library that does not always carry ZMK's app
 * include directory (the battery half builds it without pointing support), so
 * it cannot include <zmk/keymap.h>. This app source does the keymap lookup and
 * pushes the result into the driver.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zmk/keymap.h>

#include "tps43.h"

#define TPS43_LAYER_GATE_POLL_MS 20

#if DT_HAS_COMPAT_STATUS_OKAY(azoteq_tps43)
static const struct device *const tp_dev = DEVICE_DT_GET_ONE(azoteq_tps43);
static struct k_work_delayable layer_gate_work;

static void layer_gate_work_handler(struct k_work *work) {
    if (device_is_ready(tp_dev)) {
        int32_t layer = tps43_trackpoint_layer(tp_dev);

        tps43_trackpoint_set_layer_active(
            tp_dev, layer < 0 || zmk_keymap_layer_active((uint8_t)layer));
    }

    k_work_reschedule(&layer_gate_work, K_MSEC(TPS43_LAYER_GATE_POLL_MS));
}

static int tps43_layer_gate_init(void) {
    k_work_init_delayable(&layer_gate_work, layer_gate_work_handler);
    k_work_reschedule(&layer_gate_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(tps43_layer_gate_init, APPLICATION, 0);
#endif
