#include <stdint.h>
#define DT_DRV_COMPAT azoteq_tps43

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "tps43.h"
#include <zmk/keymap.h>

#if IS_ENABLED(CONFIG_SHELL) && DT_HAS_COMPAT_STATUS_OKAY(azoteq_tps43)
#include <zephyr/shell/shell.h>
#define TPS43_TP_SHELL 1
#else
#define TPS43_TP_SHELL 0
#endif

LOG_MODULE_REGISTER(tps43, CONFIG_INPUT_LOG_LEVEL);
 
/**
 * @brief Ends communication window with trackpad
 * 
 * After each read of trackpad registers, it is necessary to end the communication window
 * by writing the special address 0xEEEE, which causes a NACK from the device.
 * This is a mandatory step according to the IQS5xx protocol.
 * 
 * @param dev Pointer to trackpad device
 */
static void tps43_end_communication_window(const struct device *dev) {
    const struct tps43_config *config = dev->config;
    uint8_t end_buf[2];

    sys_put_be16(TPS43_REG_END_COMM_WINDOW, end_buf);

    int ret = i2c_write_dt(&config->i2c_bus, end_buf, sizeof(end_buf));
    if (ret != 0 && ret != -EIO) {
        LOG_INF("End communication window write returned: %d (NACK expected)", ret);
    }
}

/**
 * @brief Reads a sequence of trackpad registers
 * 
 * Performs reading of multiple bytes from sequential trackpad registers,
 * starting from the specified address. Used for reading related registers,
 * such as gesture events (GESTURE_EVENTS_0 and GESTURE_EVENTS_1).
 * 
 * @param dev Pointer to trackpad device
 * @param reg Starting register address (16-bit)
 * @param val Pointer to buffer for data
 * @param len Number of bytes to read
 * @return 0 on success, negative error code on failure
 */
static int read_sequence_registers(const struct device *dev, uint16_t reg, void *val, size_t len) {
    const struct tps43_config *config = dev->config;
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)((reg >> 8) & 0xFF);
    addr_buf[1] = (uint8_t)(reg & 0xFF);

    return i2c_write_read_dt(&config->i2c_bus, addr_buf, 2, val, len);
}

/**
 * @brief Reads a 16-bit trackpad register via I2C
 * 
 * Performs reading of a 16-bit value from the specified trackpad register.
 * Data is interpreted as big-endian (MSB first).
 * 
 * @param dev Pointer to trackpad device
 * @param reg Register address (16-bit)
 * @param val Pointer to variable to store the read value
 * @return 0 on success, negative error code on failure
 */
static int tps43_i2c_read_reg16(const struct device *dev, uint16_t reg, uint16_t *val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[2];
    // forms 2-byte register address: (MSB, LSB)
    // MSB: shift right by 8 bits (0x2F00 -> 0x2F)
    // LSB: bitwise AND with mask - mask leaves only lower byte (0x2F00 -> 0x00)
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;
    
    // writes register address (reg_buf) and reads 2 bytes of data (into buffer buf)
    ret = i2c_write_read_dt(&config->i2c_bus, reg_buf, sizeof(reg_buf), buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Register 0x%04x read error: %d", reg, ret);
        return ret;
    }
    
    // converts big-endian data (MSB first) back to 16-bit value
    *val = (buf[0] << 8) | buf[1];
    return 0;
}

/**
 * @brief Writes a 16-bit value to trackpad register via I2C
 * 
 * Performs writing of a 16-bit value to the specified trackpad register.
 * Data is transmitted as big-endian (MSB first).
 * 
 * @param dev Pointer to trackpad device
 * @param reg Register address (16-bit)
 * @param val Value to write (16-bit)
 * @return 0 on success, negative error code on failure
 */
static __maybe_unused int tps43_i2c_write_reg16(const struct device *dev, uint16_t reg,
                                                uint16_t val)
{
    const struct tps43_config *config = dev->config;
    // forms 4-byte register address: (MSB, LSB, MSB_VALUE, LSB_VALUE)
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};
    int ret;
    
    ret = i2c_write_dt(&config->i2c_bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Register 0x%04x write error: %d", reg, ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Reads an 8-bit trackpad register via I2C
 * 
 * Performs reading of an 8-bit value from the specified trackpad register.
 * Used for reading most configuration and status registers.
 * 
 * @param dev Pointer to trackpad device
 * @param reg Register address (16-bit)
 * @param val Pointer to variable to store the read value
 * @param with_err Flag to log error or expected behavior
 * @return 0 on success, negative error code on failure
 */
static int tps43_i2c_read_reg8_w_err(const struct device *dev, uint16_t reg, uint8_t *val, bool with_err)
{
    const struct tps43_config *config = dev->config;
    // forms 2-byte register address: (MSB, LSB)
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;
    
    ret = i2c_write_read_dt(&config->i2c_bus, reg_buf, sizeof(reg_buf), val, 1);
    if (ret != 0) {
        if (!with_err) {
            LOG_INF("Expected completion of register 0x%04x read: %d", reg, ret);
        } else {
            LOG_ERR("Register 0x%04x read error: %d", reg, ret);
        }
        return ret;
    }
    return ret;
}

static inline int tps43_i2c_read_reg8(const struct device *dev, uint16_t reg, uint8_t *val)
{
    return tps43_i2c_read_reg8_w_err(dev, reg, val, true);
}

/**
 * @brief Writes an 8-bit value to trackpad register via I2C
 * 
 * Performs writing of an 8-bit value to the specified trackpad register.
 * Used for writing configuration and control registers.
 * 
 * @param dev Pointer to trackpad device
 * @param reg Register address (16-bit)
 * @param val Value to write (8-bit)
 * @return 0 on success, negative error code on failure
 */
static int tps43_i2c_write_reg8(const struct device *dev, uint16_t reg, uint8_t val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};
    int ret;
    
    ret = i2c_write_dt(&config->i2c_bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Register 0x%04x write error: %d", reg, ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Callback handler for RDY pin interrupt from trackpad
 * 
 * Called when the RDY (Ready) pin state of the trackpad changes,
 * signaling that new data is available for reading.
 * Schedules execution of work handler to read the data.
 * 
 * @param dev Pointer to trackpad device
 * @param cb Pointer to GPIO callback structure
 * @param pins Mask of pins that triggered the interrupt
 */
 static void tps43_rdy_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
     struct tps43_drv_data *drv_data = CONTAINER_OF(cb, struct tps43_drv_data, rdy_cb);
 
     k_work_submit(&drv_data->work);
 }

/* ======================================================================
 * dabase_v2 patch: RDY-less polling
 *
 * The dabase_v2 PCB does not route the TPS43 RDY line, so the interrupt
 * driven path above never fires. When the devicetree node has no
 * rdy-gpios the driver reads the same report window on a timer instead.
 * Two periods are used: fast while a finger is down, slower when idle, so
 * an untouched pad does not keep the bus and the CPU busy.
 * ====================================================================== */
#define TPS43_POLL_ACTIVE_MS CONFIG_INPUT_TPS43_POLL_INTERVAL_MS
#define TPS43_POLL_IDLE_MS CONFIG_INPUT_TPS43_POLL_IDLE_INTERVAL_MS

/* dabase_v2 patch: trackpoint-style velocity mode limits. */
#define TPS43_TP_MAX_DT_MS 30
#define TPS43_TP_MAX_PX_PER_REPORT 80

static void tps43_poll_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tps43_drv_data *drv_data = CONTAINER_OF(dwork, struct tps43_drv_data, poll_work);

    /* Suspend/resume cancels and restarts the poll, and an unconfigured
     * device would only produce I2C errors. */
    if (!drv_data->initialized || !drv_data->device_ready || drv_data->suspended ||
        drv_data->rail_off) {
        return;
    }

    k_work_submit(&drv_data->work);
}
 

/** Dump charging state */
static void tps43_dump_status(const struct device *dev) {

    uint8_t sys_info = 0;
    tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_0, &sys_info);
    LOG_INF("Charging state: 0x%02X", sys_info & TPS43_CHARGING_MODE_MASK); // for debugging charging mode
}

/** Force communication start with the trackpad.  See datasheet 8.8.2 */
static void tps43_force_communication(const struct device *dev) {
    // Do a bogus read where we don't care about a possible NACK
    uint8_t control_reg = 0;
    tps43_i2c_read_reg8_w_err(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg, false);
}

/**
 * @brief Internal function to put trackpad into suspend/resume mode
 * 
 * Controls the SYSTEM_CONTROL_1 register (0x0432), setting or clearing the SUSPEND bit.
 * In suspend mode, the trackpad enters a low power consumption state and does not process
 * touches until wake-up.
 * 
 * @param dev Pointer to trackpad device
 * @param suspend true - enter suspend, false - exit suspend
 * @param lock_held true if semaphore is already held (for internal use)
 * @return 0 on success, negative error code on failure
 */
static int tps43_set_suspend_internal(const struct device *dev, bool suspend, bool lock_held) {
    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret = 0;

    // dabase_v2: never touch the bus while the rail is cut
    if (drv_data->rail_off) {
        return 0;
    }

    // If power management is disabled, or device is already in the desired state, do nothing
    if (drv_data->suspended == suspend || !config->enable_power_management) {
        return 0;
    }

    // Acquire semaphore if not already held
    if (!lock_held) {
        if (k_sem_take(&drv_data->lock, K_MSEC(100)) != 0) {
            LOG_WRN("Failed to acquire semaphore for suspend/resume");
            return -EBUSY;
        }
    }

    // Disable RDY interrupts when entering suspend (before any I2C operations)
    // This prevents race condition when RDY fires between suspend attempt and flag setting
    if (suspend && config->rdy_gpio.port != NULL) {
        ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_DISABLE);
        if (ret == 0) {
            LOG_INF("RDY interrupts disabled before suspend");
        }
    }

    uint8_t control_reg = 0;
    
    // When exiting suspend, first transaction will return NACK (section 7.3.1)
    if (drv_data->suspended && !suspend) {
        tps43_force_communication(dev);
        k_sleep(K_MSEC(1)); // need at least 200uS before 2nd read
        LOG_INF("I2C Wake: device awakened from suspend");
        
        // After wake-up, read register again
        ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg);

    } else if (!drv_data->suspended) {
        tps43_force_communication(dev);
        tps43_dump_status(dev); // for debugging power management behavior

        // Read current value 
        ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg);
    }

    if (suspend) {
        control_reg |= TPS43_SUSPEND;
        LOG_INF("Entering suspend (low power consumption)");
    } else {
        control_reg &= ~TPS43_SUSPEND;
        LOG_INF("Exiting suspend");
    }

    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, control_reg);
    if (ret != 0) {
        if (ret == -EIO && suspend) {
            LOG_INF("Failed to write suspend, device already in suspend");
            drv_data->suspended = true;
            ret = 0;
            goto done;
        }
        LOG_ERR("SYSTEM_CONTROL_1 write error: %d", ret);
        goto done;
    }

    drv_data->suspended = suspend;

done:
    // Enable RDY interrupts after resume
    if (!suspend && config->rdy_gpio.port != NULL) {
        ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret == 0) {
            LOG_INF("RDY interrupts enabled");
        }
    }
    /* dabase_v2 patch: stop polling while suspended (it would keep waking
     * the CPU from sleep) and restart it on resume. */
    if (drv_data->polling) {
        if (drv_data->suspended) {
            k_work_cancel_delayable(&drv_data->poll_work);
        } else {
            k_work_reschedule(&drv_data->poll_work, K_MSEC(TPS43_POLL_ACTIVE_MS));
        }
    }
    tps43_end_communication_window(dev);
    if (!lock_held) {
        k_sem_give(&drv_data->lock);
    }
    return ret;
}

/** Common swipe handling for 1 and 3 finger variants */
static void tps43_handle_swipe(const struct device *dev, int16_t rel_x, int16_t rel_y) {
    if (rel_x < 0) {
        LOG_INF("swipe left - INPUT_BTN_WEST");
        input_report_key(dev, INPUT_BTN_WEST, 1, true, K_FOREVER);
        input_report_key(dev, INPUT_BTN_WEST, 0, true, K_FOREVER);
    }
    if (rel_x > 0) {
        LOG_INF("swipe right - INPUT_BTN_EAST");
        input_report_key(dev, INPUT_BTN_EAST, 1, true, K_FOREVER);
        input_report_key(dev, INPUT_BTN_EAST, 0, true, K_FOREVER);
    }
    if (rel_y < 0) {
        LOG_INF("swipe up - INPUT_BTN_NORTH");
        input_report_key(dev, INPUT_BTN_NORTH, 1, true, K_FOREVER);
        input_report_key(dev, INPUT_BTN_NORTH, 0, true, K_FOREVER);
    }
    if (rel_y > 0) {
        LOG_INF("swipe down - INPUT_BTN_SOUTH");
        input_report_key(dev, INPUT_BTN_SOUTH, 1, true, K_FOREVER);
        input_report_key(dev, INPUT_BTN_SOUTH, 0, true, K_FOREVER);
    }
}

/* dabase_v2 experiment: trackpoint-style velocity from the landing point.
 *
 * The pad's absolute coordinates are treated as a virtual joystick:
 *   - the first single-finger report stores the landing point (anchor)
 *   - the offset from the anchor sets direction and speed
 *   - a radial deadzone around the anchor emits nothing
 *   - speed saturates at tp_radius
 *   - lifting the finger (or a second finger) forgets the anchor
 * The circle is never clipped to the pad: landing near an edge simply means
 * less usable travel in that direction.
 */
static void tps43_trackpoint_forget(struct tps43_drv_data *drv_data) {
    drv_data->tp_anchor_valid = false;
    drv_data->tp_speed = 0;
    drv_data->tp_rem_x = 0;
    drv_data->tp_rem_y = 0;

    /* Do not carry remainders across a mode switch or a new contact. */
    drv_data->xy_x_remainder = 0;
    drv_data->xy_y_remainder = 0;
    drv_data->scroll_h_remainder = 0;
    drv_data->scroll_v_remainder = 0;
    drv_data->gate_x = 0;
    drv_data->gate_y = 0;
    drv_data->gate_run = 0;
}

static void tps43_trackpoint_diag(struct tps43_drv_data *drv_data, int32_t abs_x, int32_t abs_y,
                                  uint8_t num_fingers, bool is_touching) {
    if (!drv_data->tp_diag) {
        return;
    }

    int64_t now = k_uptime_get();
    int64_t period = is_touching ? 100 : 1000;
    if (now - drv_data->tp_last_diag_ms < period) {
        return;
    }
    drv_data->tp_last_diag_ms = now;

    int32_t dx = 0;
    int32_t dy = 0;
    if (drv_data->tp_anchor_valid) {
        dx = abs_x - drv_data->tp_anchor_x;
        dy = abs_y - drv_data->tp_anchor_y;
    }

    bool layer_active = drv_data->tp_layer < 0 ||
                        zmk_keymap_layer_active((uint8_t)drv_data->tp_layer);

    LOG_INF("tp diag ax=%d ay=%d n=%u touch=%d anchor=%d,%d off=%d,%d speed=%d layer=%d active=%d",
            (int)abs_x, (int)abs_y, num_fingers, is_touching ? 1 : 0,
            (int)drv_data->tp_anchor_x, (int)drv_data->tp_anchor_y,
            (int)dx, (int)dy, (int)drv_data->tp_speed, (int)drv_data->tp_layer,
            layer_active ? 1 : 0);
}

static void tps43_trackpoint_update(const struct device *dev, struct tps43_drv_data *drv_data,
                                    int32_t abs_x, int32_t abs_y, uint8_t num_fingers,
                                    bool is_touching) {
    int64_t now = k_uptime_get();

    if (!is_touching) {
        tps43_trackpoint_forget(drv_data);
        return;
    }

    /* The IQS5xx reports 0xFFFF/0xFFFF when it has no valid coordinate. */
    if (abs_x == 0xFFFF || abs_y == 0xFFFF) {
        drv_data->tp_speed = 0;
        return;
    }

    /* Only one finger drives the pointer. A real second finger drops the
     * anchor, so putting it back down one finger later starts a fresh one. */
    if (num_fingers > 1) {
        tps43_trackpoint_forget(drv_data);
        return;
    }

    /* A polled pad can return an empty window in the middle of a touch.
     * Hold the anchor through it instead of re-anchoring on the next one. */
    if (num_fingers == 0) {
        drv_data->tp_speed = 0;
        drv_data->tp_last_ms = now;
        return;
    }

    if (!drv_data->tp_anchor_valid) {
        drv_data->tp_anchor_x = abs_x;
        drv_data->tp_anchor_y = abs_y;
        drv_data->tp_abs_x = abs_x;
        drv_data->tp_abs_y = abs_y;
        drv_data->tp_offset_x = 0;
        drv_data->tp_offset_y = 0;
        drv_data->tp_anchor_valid = true;
        drv_data->tp_last_ms = now;
        drv_data->tp_speed = 0;
        drv_data->tp_rem_x = 0;
        drv_data->tp_rem_y = 0;
        return;
    }

    drv_data->tp_abs_x = abs_x;
    drv_data->tp_abs_y = abs_y;
    drv_data->tp_offset_x = abs_x - drv_data->tp_anchor_x;
    drv_data->tp_offset_y = abs_y - drv_data->tp_anchor_y;

    int64_t dt_ms = now - drv_data->tp_last_ms;
    drv_data->tp_last_ms = now;
    if (dt_ms < 1) {
        dt_ms = 1;
    }
    if (dt_ms > TPS43_TP_MAX_DT_MS) {
        dt_ms = TPS43_TP_MAX_DT_MS;
    }

    int32_t deadzone = drv_data->tp_deadzone;
    int32_t radius = drv_data->tp_radius;
    if (deadzone < 0) {
        deadzone = 0;
    }
    if (radius <= deadzone) {
        radius = deadzone + 1;
    }

    int64_t dx = drv_data->tp_offset_x;
    int64_t dy = drv_data->tp_offset_y;
    int64_t dist2 = dx * dx + dy * dy;

    if (dist2 <= (int64_t)deadzone * deadzone) {
        drv_data->tp_speed = 0;
        drv_data->tp_rem_x = 0;
        drv_data->tp_rem_y = 0;
        return;
    }

    float dist = sqrtf((float)dist2);
    float norm = (dist - (float)deadzone) / (float)(radius - deadzone);
    if (norm < 0.0f) {
        norm = 0.0f;
    } else if (norm > 1.0f) {
        norm = 1.0f;
    }

    float exponent = (float)drv_data->tp_exp_x100 / 100.0f;
    if (exponent < 0.05f) {
        exponent = 0.05f;
    }

    float speed = (float)drv_data->tp_max_speed * powf(norm, exponent);
    if (speed < 0.0f) {
        speed = 0.0f;
    }
    drv_data->tp_speed = (int32_t)(speed + 0.5f);

    /* Convert px/s to this report's share, carrying sub-pixel remainders so
     * speeds below one pixel per report still move the cursor. */
    float px = speed * ((float)dt_ms / 1000.0f);
    if (px > TPS43_TP_MAX_PX_PER_REPORT) {
        px = TPS43_TP_MAX_PX_PER_REPORT;
    }

    drv_data->tp_rem_x += (int32_t)lroundf((float)dx / dist * px * 1000.0f);
    drv_data->tp_rem_y += (int32_t)lroundf((float)dy / dist * px * 1000.0f);

    int32_t out_x = drv_data->tp_rem_x / 1000;
    int32_t out_y = drv_data->tp_rem_y / 1000;
    drv_data->tp_rem_x -= out_x * 1000;
    drv_data->tp_rem_y -= out_y * 1000;

    if (out_x != 0 || out_y != 0) {
        LOG_DBG("tp move dx=%d dy=%d off=%d,%d speed=%d",
                (int)out_x, (int)out_y, (int)drv_data->tp_offset_x,
                (int)drv_data->tp_offset_y, (int)drv_data->tp_speed);
        input_report_rel(dev, INPUT_REL_X, (int16_t)out_x, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, (int16_t)out_y, true, K_FOREVER);
    }
}

/**
 * @brief Main work handler for processing trackpad events
 * 
 * Executed when receiving an interrupt from the trackpad (RDY pin).
 * Reads and processes gesture events, cursor movement and scrolling,
 * converting them into input events for the ZMK system.
 * Also manages trackpad wake-up from suspend mode when activity is detected.
 * 
 * Protected by semaphore to prevent interruption by other I2C operations,
 * which ensures smooth cursor movement without interruptions.
 * 
 * @param work Pointer to work structure
 */
static void tps43_work_handler(struct k_work *work) {
    struct tps43_drv_data *drv_data = CONTAINER_OF(work, struct tps43_drv_data, work);
    const struct device *dev = drv_data->dev;
    const struct tps43_config *config = dev->config;
    bool is_scroll_active = false;
    bool is_zoom_active = false;
    bool is_drag_active = drv_data->drag_active;
    int ret;
    
    // dabase_v2: no I2C traffic while the pad's rail is cut
    if (drv_data->rail_off) {
        return;
    }

    // If device is in suspend, ignore interrupt (RDY should be disabled)
    if (drv_data->suspended) {
        LOG_WRN("RDY interrupt in suspend mode - ignoring");
        return;
    }
    
    // Acquire semaphore to protect all I2C operations from interruption
    // This prevents conflicts during simultaneous trackpad access
    k_sem_take(&drv_data->lock, K_FOREVER);

    /*
     * Read the whole contiguous block from GESTURE_EVENTS_0 through REL_Y in a
     * single I2C transaction instead of issuing several independent reads.
     * This is much cheaper on the bus. Byte offsets into the buffer:
     *   [0] GESTURE_EVENTS_0   (0x000D)
     *   [1] GESTURE_EVENTS_1   (0x000E)
     *   [2] SYSTEM_INFO_0      (0x000F)
     *   [3] SYSTEM_INFO_1      (0x0010)
     *   [4] NUM_FINGERS        (0x0011)
     *   [5..6] REL_X           (0x0012, big-endian)
     *   [7..8] REL_Y           (0x0014, big-endian)
     *   [9..10] ABS_X          (0x0016, big-endian)
     *   [11..12] ABS_Y         (0x0018, big-endian)
     *   [13..14] TOUCH_STRENGTH (0x001A)
     *   [15] TOUCH_AREA        (0x001C)
     */
    uint8_t touch_data[(TPS43_REG_TOUCH_AREA + 1) - TPS43_REG_GESTURE_EVENTS_0];
    ret = read_sequence_registers(dev, TPS43_REG_GESTURE_EVENTS_0, touch_data,
                                  sizeof(touch_data));
    /* dabase_v2 patch: without RDY the pad has no open communication window.
     * Datasheet 8.8.2 ("Force Communication"): while the IQS5xx is in a
     * low-power state the first addressing is NACKed on purpose; the master
     * must repeat it after at least 150 us and the pad then ACKs (clock
     * stretching covers the rest). Retry a few times before giving up. */
    for (int attempt = 0; ret < 0 && attempt < 3; attempt++) {
        k_busy_wait(250);
        ret = read_sequence_registers(dev, TPS43_REG_GESTURE_EVENTS_0, touch_data,
                                      sizeof(touch_data));
    }
    if (ret < 0) {
        /* dabase_v2 patch: polling means most attempts land between two
         * communication windows and are NACKed by design, so only complain
         * when the pad has been silent for a while. */
        drv_data->read_failures++;
        if (k_uptime_get() - drv_data->last_ok_ms > 2000 &&
            (drv_data->read_failures % 200) == 1) {
            LOG_ERR("Pad silent for %lld ms (%u consecutive failed reads)",
                    (long long)(k_uptime_get() - drv_data->last_ok_ms),
                    drv_data->read_failures);
        }
        goto done;
    }

    if (k_uptime_get() - drv_data->last_ok_ms > 2000) {
        LOG_INF("Pad answering again after %u failed reads (silent %lld ms)",
                drv_data->read_failures, (long long)(k_uptime_get() - drv_data->last_ok_ms));
    }
    drv_data->last_ok_ms = k_uptime_get();
    drv_data->read_failures = 0;

    const uint8_t gestures_events[2] = {touch_data[0], touch_data[1]};
    uint8_t num_fingers = touch_data[4];
    int16_t rel_x = (int16_t)((touch_data[5] << 8) | touch_data[6]);
    int16_t rel_y = (int16_t)((touch_data[7] << 8) | touch_data[8]);
    int32_t abs_x = (int32_t)((touch_data[9] << 8) | touch_data[10]);
    int32_t abs_y = (int32_t)((touch_data[11] << 8) | touch_data[12]);

#if CONFIG_INPUT_TPS43_JITTER_MIN_RUN > 1
    /* dabase_v2: a report with no movement ends a movement run. */
    if (rel_x == 0 && rel_y == 0) {
        drv_data->gate_run = 0;
    } else if (drv_data->gate_run < UINT8_MAX) {
        drv_data->gate_run++;
    }
#endif

#if IS_ENABLED(CONFIG_INPUT_TPS43_RAW_DIAG)
    /* dabase_v2 bring-up: show the raw report window when something moves,
     * plus a 2 s heartbeat so silent reads (all zeros) are visible too. */
    {
        static int64_t last_diag_ms;
        int64_t now = k_uptime_get();
        bool active = num_fingers || rel_x || rel_y || touch_data[0] || touch_data[1];

        if ((active && now - last_diag_ms >= 100) || now - last_diag_ms >= 2000) {
            last_diag_ms = now;
            LOG_INF("raw g=%02x/%02x s=%02x/%02x n=%u dx=%d dy=%d ax=%d ay=%d",
                    touch_data[0], touch_data[1], touch_data[2], touch_data[3],
                    num_fingers, rel_x, rel_y, (int)abs_x, (int)abs_y);
        }
    }
#endif

    bool is_touching = (num_fingers > 0);

    /* dabase_v2 patch: polling reads some windows before the pad has
     * refreshed its touch count, so a single empty window must not end a
     * touch (that would send a click release on every other poll). Require
     * a few consecutive empty windows before reporting the finger lift. */
    if (!is_touching && drv_data->touching) {
        if (++drv_data->touch_release_count < 3) {
            is_touching = true;
        }
    } else if (is_touching) {
        drv_data->touch_release_count = 0;
    }

#if IS_ENABLED(CONFIG_INPUT_TPS43_LAYER_KEEPALIVE_MS)
    /* dabase_v2: keep the temp layer (mouse buttons on the thumb cluster)
     * active while a finger is on the pad, not only while it moves. ZMK's
     * temp-layer processor re-arms its timer on every input event, so a
     * harmless zero REL_MISC event is enough; the HID mapper ignores MISC
     * and sync=false keeps it out of the mouse report. */
    if (is_touching && CONFIG_INPUT_TPS43_LAYER_KEEPALIVE_MS > 0 &&
        k_uptime_get() - drv_data->last_keepalive_ms >=
            CONFIG_INPUT_TPS43_LAYER_KEEPALIVE_MS) {
        drv_data->last_keepalive_ms = k_uptime_get();
        input_report_rel(dev, INPUT_REL_MISC, 0, false, K_FOREVER);
    }
#endif

    if (is_touching != drv_data->touching) {
        drv_data->touching = is_touching;
        /* Never carry noise between contacts. */
        drv_data->gate_x = 0;
        drv_data->gate_y = 0;
        drv_data->gate_run = 0;
        LOG_INF("Touch state changed: %s", is_touching ? "down" : "up");
#if IS_ENABLED(CONFIG_INPUT_TPS43_REPORT_TOUCH)
        /* ZMK's input listener maps BTN_TOUCH to mouse button 0, so
         * reporting it makes every finger contact a held left click (the
         * desktop selects text while the pointer moves). The pad's own
         * tap / press-and-hold gestures already emit BTN_0, so this is off
         * by default. */
        input_report_key(dev, INPUT_BTN_TOUCH, is_touching ? 1 : 0, true, K_FOREVER);
#endif
        if (!is_touching) {
            tps43_trackpoint_forget(drv_data);
        }
    }

    /* dabase_v2 experiment: absolute-position velocity mode. The raw relative
     * path below stays available by turning the mode off from the shell. */
    drv_data->tp_abs_x = abs_x;
    drv_data->tp_abs_y = abs_y;
    tps43_trackpoint_diag(drv_data, abs_x, abs_y, num_fingers, is_touching);

    bool tp_velocity = drv_data->tp_mode &&
                       (drv_data->tp_layer < 0 ||
                        zmk_keymap_layer_active((uint8_t)drv_data->tp_layer));
    if (tp_velocity) {
        tps43_trackpoint_update(dev, drv_data, abs_x, abs_y, num_fingers, is_touching);
        goto done;
    }
    if (drv_data->tp_anchor_valid) {
        /* Leaving the target layer mid-touch: drop the anchor and the
         * sub-pixel remainders so the raw path starts clean. */
        tps43_trackpoint_forget(drv_data);
    }

    if (gestures_events[0] != 0 || gestures_events[1] != 0) {
        LOG_DBG("Gestures: Single=0x%02X, Multi=0x%02X", gestures_events[0], gestures_events[1]);

        /* dabase_v2 patch: the pad keeps setting gesture flags in the report
         * window even when GESTURE_EVENT is cleared in SYSTEM_CONFIG_1, so act
         * only on the gestures the devicetree actually enables. With every
         * flag off this pad is a pure pointer (buttons come from the temp
         * layer on the keymap, scrolling from the scroll layer). */
        if (config->single_tap && (gestures_events[0] & TPS43_SINGLE_TAP)) {
            LOG_INF("Single tap -> LEFT BUTTON");
            input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
            input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
        }
        if (config->swipes &&
            (gestures_events[0] & (TPS43_SWIPE_UP | TPS43_SWIPE_DOWN | TPS43_SWIPE_LEFT |
                                   TPS43_SWIPE_RIGHT))) {
            LOG_INF("Single finger swipe");
            tps43_handle_swipe(dev, rel_x, rel_y);
        }
        if (config->two_finger_tap && (gestures_events[1] & TPS43_TWO_FINGER_TAP)) {
            LOG_INF("Two finger tap -> RIGHT BUTTON");
            input_report_key(dev, INPUT_BTN_1, 1, true, K_FOREVER);
            input_report_key(dev, INPUT_BTN_1, 0, true, K_FOREVER);
        }
        if (config->press_and_hold && (gestures_events[0] & TPS43_PRESS_AND_HOLD) &&
            (!(is_drag_active))) {
            LOG_INF("Press and hold detected - DRAG (HOLD LEFT BUTTON)");
            // set internal drag flag and press left mouse button
            is_drag_active = true;
            input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
        }
        if (config->scroll && (gestures_events[1] & TPS43_SCROLL)) {
            // set scroll flag for processing in tp_movement block
            is_scroll_active = true;
        }
        if (config->zoom && (gestures_events[1] & TPS43_ZOOM)) {
            // set zoom flag for processing in tp_movement block
            is_zoom_active = true;
        }
    }

    // Detect release of a 'press and hold' left press
    if (!config->press_and_hold && is_drag_active) {
        is_drag_active = false;
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
    } else if ((!(gestures_events[0] & TPS43_PRESS_AND_HOLD)) && is_drag_active) {
        LOG_INF("Press and hold end detected - RELEASE (RELEASE LEFT BUTTON)");
        // release drag flag and release left mouse button
        is_drag_active = false;
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);   // release + sync
    }

    if (rel_x != 0 || rel_y != 0) {
        // Handle three-finger swipes
        if (config->swipes) {
            if (num_fingers == 3) {
                LOG_INF("Three-finger movement - checking for swipe");
                tps43_handle_swipe(dev, rel_x, rel_y);
            }
        }

        if (is_scroll_active) {
#if IS_ENABLED(CONFIG_INPUT_TPS43_SCROLL_BOTH_AXES)
            /* dabase_v2 patch: report both wheel axes from the same window,
             * so a diagonal two-finger swipe scrolls diagonally instead of
             * being flattened onto whichever component is larger. */
            if (config->invert_scroll_x) {
                rel_x = -rel_x;
            }
            if (config->invert_scroll_y) {
                rel_y = -rel_y;
            }
            /* Keep the sub-count remainder: at scroll-sensitivity 30 a
             * cross-axis delta under 4 counts would otherwise truncate to
             * zero and small diagonal components would vanish entirely. */
            int32_t h = (int32_t)rel_x * config->scroll_sensitivity +
                        drv_data->scroll_h_remainder;
            int32_t v = (int32_t)rel_y * config->scroll_sensitivity +
                        drv_data->scroll_v_remainder;
            int16_t h_wheel = (int16_t)(h / 100);
            int16_t v_wheel = (int16_t)(v / 100);
            drv_data->scroll_h_remainder = (int16_t)(h - (int32_t)h_wheel * 100);
            drv_data->scroll_v_remainder = (int16_t)(v - (int32_t)v_wheel * 100);

#if IS_ENABLED(CONFIG_INPUT_TPS43_RAW_DIAG)
            {
                static int64_t last_scroll_diag;
                if (k_uptime_get() - last_scroll_diag >= 100) {
                    last_scroll_diag = k_uptime_get();
                    LOG_INF("scroll raw dx=%d dy=%d -> h=%d v=%d", rel_x, rel_y,
                            h_wheel, v_wheel);
                }
            }
#endif
            LOG_DBG("Scrolling h=%d v=%d", h_wheel, v_wheel);
            input_report_rel(dev, INPUT_REL_HWHEEL, h_wheel, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_WHEEL, v_wheel, true, K_FOREVER);
#else
            // Scroll processing: keep only dominant axis
            if (abs(rel_x) > abs(rel_y)) {
                // Horizontal scroll
                if (config->invert_scroll_x) {
                    rel_x = -rel_x;
                }
                int16_t wheel = (rel_x * config->scroll_sensitivity) / 100;
                LOG_DBG("Scrolling %d horizontally", wheel);
                input_report_rel(dev, INPUT_REL_HWHEEL, wheel, true, K_FOREVER);
            } else {
                // Vertical scroll
                if (config->invert_scroll_y) {
                    rel_y = -rel_y;
                }
                int16_t wheel = (rel_y * config->scroll_sensitivity) / 100;
                LOG_DBG("Scrolling %d vertically", wheel);
                input_report_rel(dev, INPUT_REL_WHEEL, wheel, true, K_FOREVER);
            }
#endif
            is_scroll_active = false;
        } else if (is_zoom_active) {
            // Zoom processing: the zoom amount comes in via rel_x
            int16_t zoom_delta = (rel_x * config->zoom_sensitivity) / 100;
            LOG_INF("Zooming %d, rel_x=%d", zoom_delta, rel_x);
            input_report_rel(dev, INPUT_REL_MISC, zoom_delta, true, K_FOREVER);
            is_zoom_active = false;
        } else {
            int16_t move_x = rel_x;
            int16_t move_y = rel_y;

#if CONFIG_INPUT_TPS43_JITTER_MIN_RUN > 1
            /* dabase_v2: isolated one-report blips are sensor noise, not a
             * finger movement. On the layer-6 scroll path the same counts
             * become full wheel ticks, so they cannot be left in. A real
             * movement keeps the run going, because the pad accumulates its
             * delta since the previous read. */
            bool run_ok = drv_data->gate_run >= CONFIG_INPUT_TPS43_JITTER_MIN_RUN;
            if (!run_ok) {
                drv_data->gate_x = 0;
                drv_data->gate_y = 0;
                move_x = 0;
                move_y = 0;
            }
#else
            const bool run_ok = true;
#endif

#if CONFIG_INPUT_TPS43_JITTER_GATE_COUNTS > 0
            if (run_ok) {
                /* Accumulate sub-threshold deltas per axis and only let them
                 * through once an axis reaches the threshold, so +1/-1 sensor
                 * noise nets to zero instead of driving the remainder
                 * accumulator into 1 px steps. */
                drv_data->gate_x += rel_x;
                drv_data->gate_y += rel_y;
                if (abs(drv_data->gate_x) < CONFIG_INPUT_TPS43_JITTER_GATE_COUNTS &&
                    abs(drv_data->gate_y) < CONFIG_INPUT_TPS43_JITTER_GATE_COUNTS) {
                    move_x = 0;
                    move_y = 0;
                } else {
                    move_x = drv_data->gate_x;
                    move_y = drv_data->gate_y;
                    drv_data->gate_x = 0;
                    drv_data->gate_y = 0;
                }
            }
#endif

            /* Normal cursor movement, scaled by `sensitivity` percent.
             * dabase_v2 patch: carry the sub-count remainder per axis, so
             * e.g. sensitivity 90 does not truncate slow movement to zero. */
            int32_t sx = (int32_t)move_x * config->sensitivity +
                         drv_data->xy_x_remainder;
            int32_t sy = (int32_t)move_y * config->sensitivity +
                         drv_data->xy_y_remainder;
            int32_t ox = CLAMP(sx / 100, INT16_MIN, INT16_MAX);
            int32_t oy = CLAMP(sy / 100, INT16_MIN, INT16_MAX);

            drv_data->xy_x_remainder = (int16_t)(sx - ox * 100);
            drv_data->xy_y_remainder = (int16_t)(sy - oy * 100);

            if (ox != 0 || oy != 0) {
                LOG_DBG("Sending movement: dx=%d, dy=%d", (int)ox, (int)oy);
                input_report_rel(dev, INPUT_REL_X, (int16_t)ox, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, (int16_t)oy, true, K_FOREVER);
            }
        }
    }

done:
    // Save for next call
    drv_data->drag_active = is_drag_active;
    tps43_end_communication_window(dev);
    
    // Release semaphore after completing all I2C operations
    k_sem_give(&drv_data->lock);

    /* dabase_v2 patch: poll again (fast while touched, slow while idle).
     * Runs on the system workqueue, so this read has already finished. */
    if (drv_data->polling && drv_data->device_ready && !drv_data->suspended) {
        k_work_reschedule(&drv_data->poll_work,
                          K_MSEC(drv_data->touching ? TPS43_POLL_ACTIVE_MS
                                                    : TPS43_POLL_IDLE_MS));
    }
}

/**
 * @brief Resets driver internal state values
 * 
 * Initializes all driver state flags to initial values.
 * Used during initialization and device reset.
 * 
 * @param dev Pointer to trackpad device
 * @return 0 on success
 */
static int tps43_reset_values(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;

    drv_data->device_ready = false;
    drv_data->initialized = false;
    drv_data->drag_active = false;

    /* dabase_v2 experiment: trackpoint-style velocity defaults. These are
     * only starting points; `tps43_tp` changes them live. */
    drv_data->tp_mode = IS_ENABLED(CONFIG_INPUT_TPS43_TRACKPOINT);
    drv_data->tp_anchor_valid = false;
    drv_data->tp_diag = false;
    drv_data->tp_layer = CONFIG_INPUT_TPS43_TRACKPOINT_LAYER;
    drv_data->tp_deadzone = CONFIG_INPUT_TPS43_TRACKPOINT_DEADZONE_RADIUS_COUNTS;
    drv_data->tp_radius = CONFIG_INPUT_TPS43_TRACKPOINT_RADIUS_COUNTS;
    drv_data->tp_max_speed = CONFIG_INPUT_TPS43_TRACKPOINT_MAX_SPEED;
    drv_data->tp_exp_x100 = CONFIG_INPUT_TPS43_TRACKPOINT_EXPONENT_X100;
    drv_data->tp_counts_per_mm = CONFIG_INPUT_TPS43_TRACKPOINT_COUNTS_PER_MM;
    drv_data->tp_anchor_x = 0;
    drv_data->tp_anchor_y = 0;
    drv_data->tp_abs_x = 0;
    drv_data->tp_abs_y = 0;
    drv_data->tp_offset_x = 0;
    drv_data->tp_offset_y = 0;
    drv_data->tp_speed = 0;
    drv_data->tp_rem_x = 0;
    drv_data->tp_rem_y = 0;
    drv_data->tp_last_ms = k_uptime_get();
    drv_data->tp_last_diag_ms = 0;

    LOG_INF("Values reset");
    return 0;
}

/**
 * @brief Configures trackpad system registers for operation
 * 
 * Sets up trackpad registers to track touch events, gestures and movement.
 * Enables necessary gestures (single tap, press and hold, scroll, two finger tap),
 * configures axis inversion and sets setup complete flag.
 * 
 * @param dev Pointer to trackpad device
 * @return 0 on success, negative error code on failure
 */
static int tps43_configure_device(const struct device *dev) {

    const struct tps43_config *config = dev->config;
    int ret;

    // write to TPS43_REG_SYSTEM_CONFIG_1 events to track  
    uint8_t events_to_track = TPS43_TP_EVENT;

    /* dabase_v2 patch: EVENT_MODE suppresses the communication window while
     * nothing happens on the pad, which is only workable when RDY is wired
     * (the pad raises RDY to announce a window). Without RDY, keep the
     * window available every conversion cycle so polling can read it. */
    if (config->rdy_gpio.port != NULL) {
        events_to_track |= TPS43_EVENT_MODE;
    }
    
    // Gestures (single_tap, press_and_hold, scroll, two_finger_tap)
    if (config->single_tap || config->press_and_hold || 
        config->scroll || config->two_finger_tap) {
        events_to_track |= TPS43_GESTURE_EVENT;
    }
    
    // Touch events for absolute coordinates
    events_to_track |= TPS43_TOUCH_EVENT;
    
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONFIG_1, events_to_track);
    if (ret != 0) {
        LOG_WRN("Events to track write error: %d", ret);
        return ret;
    }
    LOG_INF("Events configured: 0x%02X", events_to_track);

    // axis configuration
    uint8_t xy_config = 0;
    xy_config |= config->invert_x ? TPS43_FLIP_X : 0;
    xy_config |= config->invert_y ? TPS43_FLIP_Y : 0;
    xy_config |= config->switch_xy ? TPS43_SWITCH_XY_AXIS : 0;
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_XY_CONFIG_0, xy_config);
    if (ret != 0) {
        LOG_WRN("XY configuration write error: %d", ret);
        return ret;
    }

    // enable single gestures at hardware level
    if (config->single_tap || config->press_and_hold || config->swipes) {
        uint8_t single_gestures = 0;
        single_gestures |= config->single_tap ? TPS43_SINGLE_TAP : 0;
        single_gestures |= config->press_and_hold ? TPS43_PRESS_AND_HOLD : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_UP : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_DOWN : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_LEFT : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_RIGHT : 0;
        
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SINGLE_FINGER_GESTURES, single_gestures);
        if (ret != 0) {
            LOG_WRN("Single gestures configuration error: %d", ret);
            return ret;
        }
        LOG_INF("Single gestures enabled: 0x%02X", single_gestures);
    }

    // enable multi-gestures
    if (config->two_finger_tap || config->scroll || config->zoom) {
        uint8_t multi_gestures = 0;
        multi_gestures |= config->two_finger_tap ? TPS43_TWO_FINGER_TAP : 0;
        multi_gestures |= config->scroll ? TPS43_SCROLL : 0;
        multi_gestures |= config->zoom ? TPS43_ZOOM : 0;
        
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_MULTI_FINGER_GESTURES, multi_gestures);
        if (ret != 0) {
            LOG_WRN("Multi-gesture configuration error: %d", ret);
            return ret;
        }
        LOG_INF("Multi-gestures enabled: 0x%02X", multi_gestures);
    }

    // filter configuration
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_FILTER_SETTINGS, config->filter_settings);
    if (ret != 0) {
        LOG_WRN("Filter settings write error: %d", ret);
        return ret;
    }
    LOG_INF("Filter settings set: 0x%02X", config->filter_settings);

    // dynamic filter configuration (only if set in DT)
    if (config->filter_dynamic_bottom != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_XY_DYNAMIC_FILTER_BOTTOM,
                                   (uint8_t)config->filter_dynamic_bottom);
        if (ret != 0) {
            LOG_WRN("Dynamic filter bottom write error: %d", ret);
            return ret;
        }
        LOG_INF("Dynamic filter bottom set: 0x%02X", (uint8_t)config->filter_dynamic_bottom);
    }

    if (config->filter_dynamic_lower != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_XY_DYNAMIC_FILTER_LOWER,
                                   (uint8_t)config->filter_dynamic_lower);
        if (ret != 0) {
            LOG_WRN("Dynamic filter lower write error: %d", ret);
            return ret;
        }
        LOG_INF("Dynamic filter lower set: 0x%02X", (uint8_t)config->filter_dynamic_lower);
    }

    if (config->filter_dynamic_upper != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_XY_DYNAMIC_FILTER_UPPER,
                                    (uint16_t)config->filter_dynamic_upper);
        if (ret != 0) {
            LOG_WRN("Dynamic filter upper write error: %d", ret);
            return ret;
        }
        LOG_INF("Dynamic filter upper set: 0x%04X", (uint16_t)config->filter_dynamic_upper);
    }

    // resolution configuration (only if set in DT)
    if (config->x_resolution != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_X_RESOLUTION,
                                    (uint16_t)config->x_resolution);
        if (ret != 0) {
            LOG_WRN("X resolution write error: %d", ret);
            return ret;
        }
        LOG_INF("X resolution set: %d", (uint16_t)config->x_resolution);
    }

    if (config->y_resolution != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_Y_RESOLUTION,
                                    (uint16_t)config->y_resolution);
        if (ret != 0) {
            LOG_WRN("Y resolution write error: %d", ret);
            return ret;
        }
        LOG_INF("Y resolution set: %d", (uint16_t)config->y_resolution);
    }

    // swipe configuration (only if set in DT)
    if (config->swipe_initial_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SWIPE_INITIAL_DISTANCE,
                                    (uint16_t)config->swipe_initial_distance);
        if (ret != 0) {
            LOG_WRN("Swipe distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe initial distance set: %u px", (uint16_t)config->swipe_initial_distance);
    }

    if (config->swipe_initial_time != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SWIPE_INITIAL_TIME,
                                    (uint16_t)config->swipe_initial_time);
        if (ret != 0) {
            LOG_WRN("Swipe time write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe initial time set: %u ms", (uint16_t)config->swipe_initial_time);
    }

    if (config->swipe_angle != -1) {
        double angle_rad = config->swipe_angle * 3.141592653589793 / 180.0;
        uint8_t reg_val = (uint8_t)(64.0 * tan(angle_rad));
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SWIPE_ANGLE, reg_val);
        if (ret != 0) {
            LOG_WRN("Swipe angle write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe angle set: %d deg (reg: 0x%02X)", config->swipe_angle, reg_val);
    }

    if (config->swipe_consecutive_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SWIPE_CONSECUTIVE_DISTANCE,
                                    (uint16_t)config->swipe_consecutive_distance);
        if (ret != 0) {
            LOG_WRN("Swipe consecutive distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe consecutive distance set: %u px", (uint16_t)config->swipe_consecutive_distance);
    }

    if (config->swipe_consecutive_time != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SWIPE_CONSECUTIVE_TIME,
                                    (uint16_t)config->swipe_consecutive_time);
        if (ret != 0) {
            LOG_WRN("Swipe consecutive time write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe consecutive time set: %u ms", (uint16_t)config->swipe_consecutive_time);
    }

    // scroll configuration (only if set in DT)
    if (config->scroll_initial_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SCROLL_INITIAL_DISTANCE,
                                    (uint16_t)config->scroll_initial_distance);
        if (ret != 0) {
            LOG_WRN("Scroll initial distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Scroll initial distance set: %u px", (uint16_t)config->scroll_initial_distance);
    }

    if (config->scroll_angle != -1) {
        double angle_rad = config->scroll_angle * 3.141592653589793 / 180.0;
        uint8_t reg_val = (uint8_t)(64.0 * tan(angle_rad));
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SCROLL_ANGLE, reg_val);
        if (ret != 0) {
            LOG_WRN("Scroll angle write error: %d", ret);
            return ret;
        }
        LOG_INF("Scroll angle set: %d deg (reg: 0x%02X)", config->scroll_angle, reg_val);
    }

    // zoom configuration (only if set in DT)
    if (config->zoom_initial_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_ZOOM_INITIAL_DISTANCE,
                                    (uint16_t)config->zoom_initial_distance);
        if (ret != 0) {
            LOG_WRN("Zoom initial distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Zoom initial distance set: %u px", (uint16_t)config->zoom_initial_distance);
    }

    if (config->zoom_consecutive_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_ZOOM_CONSECUTIVE_DISTANCE,
                                    (uint16_t)config->zoom_consecutive_distance);
        if (ret != 0) {
            LOG_WRN("Zoom consecutive distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Zoom consecutive distance set: %u px", (uint16_t)config->zoom_consecutive_distance);
    }

    // tap/hold configuration (only if set in DT)
    if (config->tap_time != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_TAP_TIME,
                                    (uint16_t)config->tap_time);
        if (ret != 0) {
            LOG_WRN("Tap time write error: %d", ret);
            return ret;
        }
        LOG_INF("Tap time set: %u ms", (uint16_t)config->tap_time);
    }

    if (config->tap_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_TAP_DISTANCE,
                                    (uint16_t)config->tap_distance);
        if (ret != 0) {
            LOG_WRN("Tap distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Tap distance set: %u px", (uint16_t)config->tap_distance);
    }

    if (config->hold_time != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_HOLD_TIME,
                                    (uint16_t)config->hold_time);
        if (ret != 0) {
            LOG_WRN("Hold time write error: %d", ret);
            return ret;
        }
        LOG_INF("Hold time set: %u ms", (uint16_t)config->hold_time);
    }

    // ATI configuration (only if set in DT)
    if (config->ati_target != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_ATI_TARGET,
                                    (uint16_t)config->ati_target);
        if (ret != 0) {
            LOG_WRN("ATI target write error: %d", ret);
            return ret;
        }
        LOG_INF("ATI target set: %u", (uint16_t)config->ati_target);
    }

    if (config->ref_drift_limit != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_REF_DRIFT_LIMIT,
                                   (uint8_t)config->ref_drift_limit);
        if (ret != 0) {
            LOG_WRN("Ref drift limit write error: %d", ret);
            return ret;
        }
        LOG_INF("Ref drift limit set: %u", (uint8_t)config->ref_drift_limit);
    }

    if (config->reati_lower_limit != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_REATI_LOWER_LIMIT,
                                   (uint8_t)config->reati_lower_limit);
        if (ret != 0) {
            LOG_WRN("REATI lower limit write error: %d", ret);
            return ret;
        }
        LOG_INF("REATI lower limit set: %u", (uint8_t)config->reati_lower_limit);
    }

    if (config->reati_upper_limit != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_REATI_UPPER_LIMIT,
                                   (uint8_t)config->reati_upper_limit);
        if (ret != 0) {
            LOG_WRN("REATI upper limit write error: %d", ret);
            return ret;
        }
        LOG_INF("REATI upper limit set: %u", (uint8_t)config->reati_upper_limit);
    }

    if (config->max_count_limit != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_MAX_COUNT_LIMIT,
                                    (uint16_t)config->max_count_limit);
        if (ret != 0) {
            LOG_WRN("Max count limit write error: %d", ret);
            return ret;
        }
        LOG_INF("Max count limit set: %u", (uint16_t)config->max_count_limit);
    }

    if (config->ati_retry_time != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_ATI_RETRY_TIME,
                                   (uint8_t)config->ati_retry_time);
        if (ret != 0) {
            LOG_WRN("ATI retry time write error: %d", ret);
            return ret;
        }
        LOG_INF("ATI retry time set: %u s", (uint8_t)config->ati_retry_time);
    }

    // Report rate configuration (only if set in DT)
    if (config->report_rate_active != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_ACTIVE,
                                    (uint16_t)config->report_rate_active);
        if (ret != 0) {
            LOG_WRN("Report rate active write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate active set: %u ms", (uint16_t)config->report_rate_active);
    }

    if (config->report_rate_idle_touch != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_IDLE_TOUCH,
                                    (uint16_t)config->report_rate_idle_touch);
        if (ret != 0) {
            LOG_WRN("Report rate idle touch write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate idle touch set: %u ms", (uint16_t)config->report_rate_idle_touch);
    }

    if (config->report_rate_idle != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_IDLE,
                                    (uint16_t)config->report_rate_idle);
        if (ret != 0) {
            LOG_WRN("Report rate idle write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate idle set: %u ms", (uint16_t)config->report_rate_idle);
    }

    if (config->report_rate_lp1 != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_LP1,
                                    (uint16_t)config->report_rate_lp1);
        if (ret != 0) {
            LOG_WRN("Report rate LP1 write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate LP1 set: %u ms", (uint16_t)config->report_rate_lp1);
    }

    if (config->report_rate_lp2 != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_LP2,
                                    (uint16_t)config->report_rate_lp2);
        if (ret != 0) {
            LOG_WRN("Report rate LP2 write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate LP2 set: %u ms", (uint16_t)config->report_rate_lp2);
    }

    // Timeout configuration (only if set in DT)
    if (config->timeout_active != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_TIMEOUT_ACTIVE,
                                   (uint8_t)config->timeout_active);
        if (ret != 0) {
            LOG_WRN("Timeout active write error: %d", ret);
            return ret;
        }
        LOG_INF("Timeout active set: %u s", (uint8_t)config->timeout_active);
    }

    if (config->timeout_idle_touch != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_TIMEOUT_IDLE_TOUCH,
                                   (uint8_t)config->timeout_idle_touch);
        if (ret != 0) {
            LOG_WRN("Timeout idle touch write error: %d", ret);
            return ret;
        }
        LOG_INF("Timeout idle touch set: %u s", (uint8_t)config->timeout_idle_touch);
    }

    if (config->timeout_idle != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_TIMEOUT_IDLE,
                                   (uint8_t)config->timeout_idle);
        if (ret != 0) {
            LOG_WRN("Timeout idle write error: %d", ret);
            return ret;
        }
        LOG_INF("Timeout idle set: %u s", (uint8_t)config->timeout_idle);
    }

    if (config->timeout_lp1 != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_TIMEOUT_LP1,
                                   (uint8_t)config->timeout_lp1);
        if (ret != 0) {
            LOG_WRN("Timeout LP1 write error: %d", ret);
            return ret;
        }
        LOG_INF("Timeout LP1 set: %u s", (uint8_t)config->timeout_lp1);
    }

    if (config->ref_update_time != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_REF_UPDATE_TIME,
                                   (uint8_t)config->ref_update_time);
        if (ret != 0) {
            LOG_WRN("Ref update time write error: %d", ret);
            return ret;
        }
        LOG_INF("Ref update time set: %u s", (uint8_t)config->ref_update_time);
    }

    // set configuration complete flag
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONFIG_0, TPS43_SETUP_COMPLETE | TPS43_WDT_ENABLE | TPS43_REATI);
    if (ret != 0) {
        LOG_WRN("Setup complete flag write error: %d", ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Checks device reset state and performs reconfiguration
 * 
 * Waits for device readiness after reset, checks SHOW_RESET flag
 * and sends reset acknowledgment (ACK_RESET) when necessary.
 * Then performs full device configuration.
 * 
 * @param dev Pointer to trackpad device
 * @return 0 on success, negative error code on failure
 */
static int check_reset_and_reconfigure(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;
    int ret;
    uint8_t sys_info = 0;
    uint8_t wait_count = 0;
    const uint8_t max_wait_count = 50;

    // Wait for device readiness (the first few reads may fail until device is up)
    do {
        ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_0, &sys_info);
        if (ret < 0) {
            k_sleep(K_MSEC(100));
            wait_count++;
            if (wait_count >= max_wait_count) {
                LOG_ERR("Device not responding after %d ms", wait_count * 100);
                return -ETIMEDOUT;
            }
        }
    } while (ret < 0);
    
    LOG_INF("Device ready after %d ms", wait_count * 100);

    // after reset, set flag to acknowledge that reset was performed
    if (sys_info & TPS43_SHOW_RESET) {
        LOG_INF("SHOW_RESET detected, sending ACK_RESET and enable ATI");
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_0, TPS43_ACK_RESET | TPS43_AUTO_ATI);
        if (ret != 0) {
            LOG_ERR("ACK_RESET send error: %d", ret);
            return ret;
        }
        k_sleep(K_MSEC(10));
    }

    ret = tps43_configure_device(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration error: %d", ret);
        return ret;
    }

    drv_data->device_ready = true;
    
    return 0;
}

/**
 * @brief Public function to put trackpad into suspend/resume
 * 
 * @param dev Pointer to trackpad device
 * @param suspend true - enter suspend, false - exit suspend
 * @return 0 on success, negative error code on failure
 */
static int tps43_set_suspend(const struct device *dev, bool suspend) {
    return tps43_set_suspend_internal(dev, suspend, false);
}

static void tps43_dump_registers(const struct device *dev) {
    uint8_t reg8;
    uint16_t reg16;
    int ret = 0;
    int read_ret;

    LOG_INF("Dumping TPS43 registers");

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_0, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_INFO_0 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_INFO_0, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_1, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_INFO_1 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_INFO_1, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_0, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_CONTROL_0 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_CONTROL_0, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_CONTROL_1 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_CONTROL_1, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONFIG_0, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_CONFIG_0 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_CONFIG_0, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONFIG_1, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_CONFIG_1 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_CONFIG_1, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_GLOBAL_ATI_C, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("GLOBAL_ATI_C (0x%04X): 0x%02X", TPS43_REG_GLOBAL_ATI_C, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_ATI_TARGET, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ATI_TARGET (0x%04X): 0x%04X", TPS43_REG_ATI_TARGET, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_REF_DRIFT_LIMIT, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REF_DRIFT_LIMIT (0x%04X): 0x%02X", TPS43_REG_REF_DRIFT_LIMIT, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_REATI_LOWER_LIMIT, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REATI_LOWER_LIMIT (0x%04X): 0x%02X", TPS43_REG_REATI_LOWER_LIMIT, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_REATI_UPPER_LIMIT, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REATI_UPPER_LIMIT (0x%04X): 0x%02X", TPS43_REG_REATI_UPPER_LIMIT, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_MAX_COUNT_LIMIT, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("MAX_COUNT_LIMIT (0x%04X): 0x%04X", TPS43_REG_MAX_COUNT_LIMIT, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_ATI_RETRY_TIME, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ATI_RETRY_TIME (0x%04X): 0x%02X", TPS43_REG_ATI_RETRY_TIME, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_ACTIVE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_ACTIVE (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_ACTIVE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_IDLE_TOUCH, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_IDLE_TOUCH (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_IDLE_TOUCH, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_IDLE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_IDLE (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_IDLE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_LP1, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_LP1 (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_LP1, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_LP2, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_LP2 (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_LP2, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_TIMEOUT_ACTIVE, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TIMEOUT_ACTIVE (0x%04X): 0x%02X", TPS43_REG_TIMEOUT_ACTIVE, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_TIMEOUT_IDLE_TOUCH, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TIMEOUT_IDLE_TOUCH (0x%04X): 0x%02X", TPS43_REG_TIMEOUT_IDLE_TOUCH, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_TIMEOUT_IDLE, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TIMEOUT_IDLE (0x%04X): 0x%02X", TPS43_REG_TIMEOUT_IDLE, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_TIMEOUT_LP1, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TIMEOUT_LP1 (0x%04X): 0x%02X", TPS43_REG_TIMEOUT_LP1, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_REF_UPDATE_TIME, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REF_UPDATE_TIME (0x%04X): 0x%02X", TPS43_REG_REF_UPDATE_TIME, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_XY_STATIC_BETA, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("XY_STATIC_BETA (0x%04X): 0x%02X", TPS43_REG_XY_STATIC_BETA, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_ALP_COUNT_BETA, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ALP_COUNT_BETA (0x%04X): 0x%02X", TPS43_REG_ALP_COUNT_BETA, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_ALP1_LTA_BETA, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ALP1_LTA_BETA (0x%04X): 0x%02X", TPS43_REG_ALP1_LTA_BETA, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_ALP2_LTA_BETA, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ALP2_LTA_BETA (0x%04X): 0x%02X", TPS43_REG_ALP2_LTA_BETA, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_XY_DYNAMIC_FILTER_BOTTOM, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("XY_DYNAMIC_FILTER_BOTTOM (0x%04X): 0x%02X", TPS43_REG_XY_DYNAMIC_FILTER_BOTTOM, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_XY_DYNAMIC_FILTER_LOWER, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("XY_DYNAMIC_FILTER_LOWER (0x%04X): 0x%02X", TPS43_REG_XY_DYNAMIC_FILTER_LOWER, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_XY_DYNAMIC_FILTER_UPPER, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("XY_DYNAMIC_FILTER_UPPER (0x%04X): 0x%04X", TPS43_REG_XY_DYNAMIC_FILTER_UPPER, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_X_RESOLUTION, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("X_RESOLUTION (0x%04X): 0x%04X", TPS43_REG_X_RESOLUTION, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_Y_RESOLUTION, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("Y_RESOLUTION (0x%04X): 0x%04X", TPS43_REG_Y_RESOLUTION, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_TAP_TIME, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TAP_TIME (0x%04X): %u ms", TPS43_REG_TAP_TIME, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_TAP_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TAP_DISTANCE (0x%04X): %u px", TPS43_REG_TAP_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_HOLD_TIME, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("HOLD_TIME (0x%04X): %u ms", TPS43_REG_HOLD_TIME, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SWIPE_INITIAL_TIME, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_INITIAL_TIME (0x%04X): %u ms", TPS43_REG_SWIPE_INITIAL_TIME, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SWIPE_INITIAL_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_INITIAL_DISTANCE (0x%04X): %u px", TPS43_REG_SWIPE_INITIAL_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SWIPE_CONSECUTIVE_TIME, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_CONSECUTIVE_TIME (0x%04X): %u ms", TPS43_REG_SWIPE_CONSECUTIVE_TIME, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SWIPE_CONSECUTIVE_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_CONSECUTIVE_DISTANCE (0x%04X): %u px", TPS43_REG_SWIPE_CONSECUTIVE_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SWIPE_ANGLE, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_ANGLE (0x%04X): 0x%02X", TPS43_REG_SWIPE_ANGLE, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SCROLL_INITIAL_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SCROLL_INITIAL_DISTANCE (0x%04X): %u px", TPS43_REG_SCROLL_INITIAL_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SCROLL_ANGLE, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SCROLL_ANGLE (0x%04X): 0x%02X", TPS43_REG_SCROLL_ANGLE, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_ZOOM_INITIAL_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ZOOM_INITIAL_DISTANCE (0x%04X): %u px", TPS43_REG_ZOOM_INITIAL_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_ZOOM_CONSECUTIVE_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ZOOM_CONSECUTIVE_DISTANCE (0x%04X): %u px", TPS43_REG_ZOOM_CONSECUTIVE_DISTANCE, reg16);
    }

    if (ret != 0) {
        LOG_WRN("error: some reads failed...");
    }

    tps43_end_communication_window(dev);
}

/**
 * @brief Initializes TPS43 trackpad driver
 * 
 * Performs full driver initialization: checks I2C bus availability,
 * performs hardware reset via GPIO RST (if connected), waits for device
 * readiness, configures trackpad registers and sets up GPIO RDY interrupts.
 * Also initializes power management system when necessary.
 * 
 * @param dev Pointer to trackpad device
 * @return 0 on success, negative error code on failure
 */
static int tps43_init(const struct device *dev) {

    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret;

    drv_data->dev = dev;

    LOG_INF("=== Azoteq tps43 driver for device %s ===", dev->name);
    
    // Check I2C bus
    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_ERR("I2C bus not available");
        return -ENODEV;
    }
    
    LOG_INF("I2C bus: %s", config->i2c_bus.bus->name);
    LOG_INF("I2C address: 0x%02x", config->i2c_bus.addr);

    ret = tps43_reset_values(dev);
    if (ret != 0) {
        LOG_ERR("Values reset error: %d", ret);
        return ret;
    }

    // GPIO reset via hardware RST
    if (config->rst_gpio.port) {
        ret = gpio_pin_configure_dt(&config->rst_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("RST GPIO configuration error: %d", ret);
            return ret;
        }
        
        gpio_pin_set_dt(&config->rst_gpio, 0);
        k_sleep(K_MSEC(10));
        gpio_pin_set_dt(&config->rst_gpio, 1);
        k_sleep(K_MSEC(610));
        
        LOG_INF("Hardware reset completed");
    }

    // check SHOW_RESET and configure
    ret = check_reset_and_reconfigure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration error: %d", ret);
        return ret;
    }

    // configure RDY interrupts only AFTER device configuration!
    if (config->rdy_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
        if (ret != 0) {
            LOG_WRN("RDY GPIO configuration error: %d", ret);
        } else {
            ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, 
                                                    GPIO_INT_EDGE_TO_ACTIVE);
            if (ret == 0) {
                gpio_init_callback(&drv_data->rdy_cb, tps43_rdy_callback, 
                                    BIT(config->rdy_gpio.pin));
                ret = gpio_add_callback(config->rdy_gpio.port, &drv_data->rdy_cb);
                if (ret == 0) {
                    LOG_INF("RDY interrupt configured");
                } else {
                    LOG_WRN("RDY callback add error: %d", ret);
                }
            }
        }
    }

    drv_data->initialized = true;
    drv_data->suspended = false;

    // Initialize semaphore to protect I2C operations
    // First parameter - initial count (1 = available)
    // Second parameter - maximum count (1 = binary semaphore)
    k_sem_init(&drv_data->lock, 1, 1);

    k_work_init(&drv_data->work, tps43_work_handler);
    k_work_init_delayable(&drv_data->poll_work, tps43_poll_work_handler);

    /* dabase_v2 patch: no RDY line on this PCB -> read on a timer. */
    if (config->rdy_gpio.port == NULL) {
        drv_data->polling = true;
        LOG_INF("RDY not wired: polling every %d ms (idle %d ms)",
                TPS43_POLL_ACTIVE_MS, TPS43_POLL_IDLE_MS);
        k_work_reschedule(&drv_data->poll_work, K_MSEC(TPS43_POLL_IDLE_MS));
    }

    tps43_dump_registers(dev);
    
    LOG_INF("TPS43 driver successfully initialized");
    return 0;
}

 
#define TPS43_INIT(inst)                                                                             \
    static struct tps43_drv_data tps43_##inst##_drvdata = {                                          \
        .device_ready = false,                                                                       \
        .initialized = false,                                                                        \
        .drag_active = false,                                                                        \
        .suspended = false,                                                                          \
    };                                                                                               \
                                                                                                     \
    static const struct tps43_config tps43_##inst##_config = {                                       \
        .i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                                       \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rdy_gpios, {0}),                                  \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {0}),                                  \
        .single_tap = DT_INST_PROP(inst, single_tap),                                                \
        .press_and_hold = DT_INST_PROP(inst, press_and_hold),                                        \
        .two_finger_tap = DT_INST_PROP(inst, two_finger_tap),                                        \
        .scroll = DT_INST_PROP(inst, scroll),                                                        \
        .zoom = DT_INST_PROP(inst, zoom),                                                            \
        .swipes = DT_INST_PROP(inst, swipes),                                                        \
        .invert_x = DT_INST_PROP(inst, invert_x),                                                    \
        .invert_y = DT_INST_PROP(inst, invert_y),                                                    \
        .switch_xy = DT_INST_PROP(inst, switch_xy),                                                  \
        .invert_scroll_x = DT_INST_PROP(inst, invert_scroll_x),                                      \
        .invert_scroll_y = DT_INST_PROP(inst, invert_scroll_y),                                      \
        .sensitivity = DT_INST_PROP_OR(inst, sensitivity, 100),                                      \
        .scroll_sensitivity = DT_INST_PROP_OR(inst, scroll_sensitivity, 100),                        \
        .zoom_sensitivity = DT_INST_PROP_OR(inst, zoom_sensitivity, 100),                            \
        .enable_power_management = DT_INST_PROP_OR(inst, enable_power_management, true),             \
        .idle_sleep = DT_INST_PROP_OR(inst, idle_sleep, false),                                      \
        .filter_settings = DT_INST_PROP_OR(inst, filter_settings, 0x0F),                             \
        .filter_dynamic_bottom = DT_INST_PROP_OR(inst, filter_dynamic_bottom, -1),                    \
        .filter_dynamic_lower = DT_INST_PROP_OR(inst, filter_dynamic_lower, -1),                     \
        .filter_dynamic_upper = DT_INST_PROP_OR(inst, filter_dynamic_upper, -1),                     \
        .x_resolution = DT_INST_PROP_OR(inst, x_resolution, -1),                                     \
        .y_resolution = DT_INST_PROP_OR(inst, y_resolution, -1),                                     \
        .swipe_initial_distance = DT_INST_PROP_OR(inst, swipe_initial_distance, -1),                 \
        .swipe_initial_time = DT_INST_PROP_OR(inst, swipe_initial_time, -1),                         \
        .swipe_angle = DT_INST_PROP_OR(inst, swipe_angle, -1),                                       \
        .swipe_consecutive_distance = DT_INST_PROP_OR(inst, swipe_consecutive_distance, -1),         \
        .swipe_consecutive_time = DT_INST_PROP_OR(inst, swipe_consecutive_time, -1),                 \
        .scroll_initial_distance = DT_INST_PROP_OR(inst, scroll_initial_distance, -1),               \
        .scroll_angle = DT_INST_PROP_OR(inst, scroll_angle, -1),                                     \
        .zoom_initial_distance = DT_INST_PROP_OR(inst, zoom_initial_distance, -1),                   \
        .zoom_consecutive_distance = DT_INST_PROP_OR(inst, zoom_consecutive_distance, -1),           \
        .ati_target = DT_INST_PROP_OR(inst, ati_target, -1),                                         \
        .ref_drift_limit = DT_INST_PROP_OR(inst, ref_drift_limit, -1),                               \
        .reati_lower_limit = DT_INST_PROP_OR(inst, reati_lower_limit, -1),                           \
        .reati_upper_limit = DT_INST_PROP_OR(inst, reati_upper_limit, -1),                           \
        .max_count_limit = DT_INST_PROP_OR(inst, max_count_limit, -1),                               \
        .ati_retry_time = DT_INST_PROP_OR(inst, ati_retry_time, -1),                                 \
        .report_rate_active = DT_INST_PROP_OR(inst, report_rate_active, -1),                         \
        .report_rate_idle_touch = DT_INST_PROP_OR(inst, report_rate_idle_touch, -1),                 \
        .report_rate_idle = DT_INST_PROP_OR(inst, report_rate_idle, -1),                             \
        .report_rate_lp1 = DT_INST_PROP_OR(inst, report_rate_lp1, -1),                               \
        .report_rate_lp2 = DT_INST_PROP_OR(inst, report_rate_lp2, -1),                               \
        .timeout_active = DT_INST_PROP_OR(inst, timeout_active, -1),                                 \
        .timeout_idle_touch = DT_INST_PROP_OR(inst, timeout_idle_touch, -1),                         \
        .timeout_idle = DT_INST_PROP_OR(inst, timeout_idle, -1),                                     \
        .timeout_lp1 = DT_INST_PROP_OR(inst, timeout_lp1, -1),                                       \
        .ref_update_time = DT_INST_PROP_OR(inst, ref_update_time, -1),                               \
        .tap_time = DT_INST_PROP_OR(inst, tap_time, -1),                                             \
        .tap_distance = DT_INST_PROP_OR(inst, tap_distance, -1),                                     \
        .hold_time = DT_INST_PROP_OR(inst, hold_time, -1),                                           \
    };                                                                                               \
                                                                                                     \
    DEVICE_DT_INST_DEFINE(inst, tps43_init, NULL, &tps43_##inst##_drvdata, &tps43_##inst##_config,   \
                        POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);                              \
    BUILD_ASSERT(DT_INST_REG_ADDR(inst) == TPS43_I2C_ADDR, "I2C address mismatch");


DT_INST_FOREACH_STATUS_OKAY(TPS43_INIT)

#if TPS43_TP_SHELL
/* dabase_v2 experiment: live tuning over the USB/serial shell.
 *
 *   tps43_tp status
 *   tps43_tp mode on|off|toggle
 *   tps43_tp deadzone_mm 6        (diameter, matches how it is described)
 *   tps43_tp deadzone_counts 154  (radius, raw pad counts)
 *   tps43_tp radius_mm 10         (outer radius, full speed)
 *   tps43_tp radius_counts 512
 *   tps43_tp speed 1000           (px/s)
 *   tps43_tp exp 200              (x100)
 *   tps43_tp mm 51                (counts per mm)
 *   tps43_tp diag on|off|toggle
 *   tps43_tp reanchor
 */
static struct tps43_drv_data *tps43_tp_data(const struct shell *sh) {
    const struct device *dev = DEVICE_DT_GET_ONE(azoteq_tps43);

    if (!device_is_ready(dev)) {
        shell_error(sh, "tps43 is not ready");
        return NULL;
    }
    return dev->data;
}

static int tps43_tp_parse_long(const struct shell *sh, const char *s, long min, long max,
                               long *out) {
    char *end = NULL;
    long v = strtol(s, &end, 10);

    if (end == s || *end != '\0' || v < min || v > max) {
        shell_error(sh, "expected %ld..%ld, got '%s'", min, max, s);
        return -EINVAL;
    }
    *out = v;
    return 0;
}

static int tps43_tp_parse_mm(const struct shell *sh, const char *s, float min, float max,
                             float *out) {
    char *end = NULL;
    float v = strtof(s, &end);

    if (end == s || *end != '\0' || !(v >= min && v <= max)) {
        shell_error(sh, "value out of range, got '%s'", s);
        return -EINVAL;
    }
    *out = v;
    return 0;
}

static int tps43_tp_cmd_status(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);

    (void)argc;
    (void)argv;

    if (d == NULL) {
        return -ENODEV;
    }

    int cpm = d->tp_counts_per_mm > 0 ? d->tp_counts_per_mm : 1;
    int dz_r10 = (int)((d->tp_deadzone * 10) / cpm);
    int dz_d10 = (int)((d->tp_deadzone * 20) / cpm);
    int r10 = (int)((d->tp_radius * 10) / cpm);

    if (d->tp_layer < 0) {
        shell_print(sh, "trackpoint: %s  diag: %s  layer: all",
                    d->tp_mode ? "on" : "off", d->tp_diag ? "on" : "off");
    } else {
        bool layer_active = zmk_keymap_layer_active((uint8_t)d->tp_layer);

        shell_print(sh, "trackpoint: %s  diag: %s  layer: %d (%s)",
                    d->tp_mode ? "on" : "off", d->tp_diag ? "on" : "off",
                    (int)d->tp_layer, layer_active ? "active" : "inactive");
    }
    shell_print(sh, "  deadzone: %d counts (%d.%d mm radius, %d.%d mm diameter)",
                (int)d->tp_deadzone, dz_r10 / 10, dz_r10 % 10, dz_d10 / 10, dz_d10 % 10);
    shell_print(sh, "  full speed at %d counts (%d.%d mm radius)",
                (int)d->tp_radius, r10 / 10, r10 % 10);
    shell_print(sh, "  max %d px/s, exponent %d.%02d, scale %d counts/mm",
                (int)d->tp_max_speed, (int)(d->tp_exp_x100 / 100),
                (int)(d->tp_exp_x100 % 100), cpm);
    shell_print(sh, "  anchor: %s (%d,%d)",
                d->tp_anchor_valid ? "yes" : "no", (int)d->tp_anchor_x,
                (int)d->tp_anchor_y);
    shell_print(sh, "  abs: %d,%d  offset: %d,%d  speed: %d px/s",
                (int)d->tp_abs_x, (int)d->tp_abs_y, (int)d->tp_offset_x,
                (int)d->tp_offset_y, (int)d->tp_speed);
    shell_print(sh, "  touching: %s", d->touching ? "yes" : "no");
    return 0;
}

static int tps43_tp_cmd_mode(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);

    if (d == NULL) {
        return -ENODEV;
    }

    if (strcmp(argv[1], "on") == 0) {
        d->tp_mode = true;
    } else if (strcmp(argv[1], "off") == 0) {
        d->tp_mode = false;
    } else if (strcmp(argv[1], "toggle") == 0) {
        d->tp_mode = !d->tp_mode;
    } else {
        shell_error(sh, "use on, off or toggle");
        return -EINVAL;
    }

    tps43_trackpoint_forget(d);
    shell_print(sh, "trackpoint mode: %s", d->tp_mode ? "on" : "off");
    return 0;
}

static int tps43_tp_cmd_layer(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);
    long v;

    if (d == NULL) {
        return -ENODEV;
    }

    if (strcmp(argv[1], "off") == 0) {
        v = -1;
    } else if (tps43_tp_parse_long(sh, argv[1], -1, 31, &v) != 0) {
        return -EINVAL;
    }

    d->tp_layer = (int32_t)v;
    tps43_trackpoint_forget(d);
    if (d->tp_layer < 0) {
        shell_print(sh, "velocity mode: all layers");
    } else {
        shell_print(sh, "velocity mode: layer %d only", (int)d->tp_layer);
    }
    return 0;
}

static int tps43_tp_cmd_deadzone_mm(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);
    float mm;

    if (d == NULL) {
        return -ENODEV;
    }
    if (tps43_tp_parse_mm(sh, argv[1], 0.0f, 20.0f, &mm) != 0) {
        return -EINVAL;
    }

    int cpm = d->tp_counts_per_mm > 0 ? d->tp_counts_per_mm : 1;
    d->tp_deadzone = (int32_t)lroundf(mm * (float)cpm / 2.0f);

    if (d->tp_radius <= d->tp_deadzone) {
        d->tp_radius = d->tp_deadzone + 1;
    }
    tps43_trackpoint_forget(d);

    int d10 = (int)((d->tp_deadzone * 20) / cpm);
    shell_print(sh, "deadzone: %d counts (%d.%d mm diameter), full speed at %d counts",
                (int)d->tp_deadzone, d10 / 10, d10 % 10, (int)d->tp_radius);
    return 0;
}

static int tps43_tp_cmd_deadzone_counts(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);
    long v;

    if (d == NULL) {
        return -ENODEV;
    }
    if (tps43_tp_parse_long(sh, argv[1], 0, 1000, &v) != 0) {
        return -EINVAL;
    }

    d->tp_deadzone = (int32_t)v;
    if (d->tp_radius <= d->tp_deadzone) {
        d->tp_radius = d->tp_deadzone + 1;
    }
    tps43_trackpoint_forget(d);
    shell_print(sh, "deadzone radius: %d counts", (int)d->tp_deadzone);
    return 0;
}

static int tps43_tp_cmd_radius_mm(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);
    float mm;

    if (d == NULL) {
        return -ENODEV;
    }
    if (tps43_tp_parse_mm(sh, argv[1], 0.1f, 40.0f, &mm) != 0) {
        return -EINVAL;
    }

    int cpm = d->tp_counts_per_mm > 0 ? d->tp_counts_per_mm : 1;
    int32_t counts = (int32_t)lroundf(mm * (float)cpm);
    if (counts <= d->tp_deadzone) {
        shell_error(sh, "full-speed radius must be above deadzone (%d counts)",
                    (int)d->tp_deadzone);
        return -EINVAL;
    }

    d->tp_radius = counts;
    tps43_trackpoint_forget(d);
    shell_print(sh, "full speed at %d counts", (int)d->tp_radius);
    return 0;
}

static int tps43_tp_cmd_radius_counts(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);
    long v;

    if (d == NULL) {
        return -ENODEV;
    }
    if (tps43_tp_parse_long(sh, argv[1], 1, 4096, &v) != 0) {
        return -EINVAL;
    }
    if (v <= d->tp_deadzone) {
        shell_error(sh, "full-speed radius must be above deadzone (%d counts)",
                    (int)d->tp_deadzone);
        return -EINVAL;
    }

    d->tp_radius = (int32_t)v;
    tps43_trackpoint_forget(d);
    shell_print(sh, "full speed at %d counts", (int)d->tp_radius);
    return 0;
}

static int tps43_tp_cmd_speed(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);
    long v;

    if (d == NULL) {
        return -ENODEV;
    }
    if (tps43_tp_parse_long(sh, argv[1], 50, 5000, &v) != 0) {
        return -EINVAL;
    }

    d->tp_max_speed = (int32_t)v;
    shell_print(sh, "max speed: %d px/s", (int)d->tp_max_speed);
    return 0;
}

static int tps43_tp_cmd_exp(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);
    long v;

    if (d == NULL) {
        return -ENODEV;
    }
    if (tps43_tp_parse_long(sh, argv[1], 50, 500, &v) != 0) {
        return -EINVAL;
    }

    d->tp_exp_x100 = (int32_t)v;
    shell_print(sh, "exponent: %d.%02d", (int)(d->tp_exp_x100 / 100),
                (int)(d->tp_exp_x100 % 100));
    return 0;
}

static int tps43_tp_cmd_mm(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);
    long v;

    if (d == NULL) {
        return -ENODEV;
    }
    if (tps43_tp_parse_long(sh, argv[1], 1, 500, &v) != 0) {
        return -EINVAL;
    }

    d->tp_counts_per_mm = (int32_t)v;
    shell_print(sh, "scale: %d counts/mm (existing counts are unchanged)",
                (int)d->tp_counts_per_mm);
    return 0;
}

static int tps43_tp_cmd_diag(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);

    if (d == NULL) {
        return -ENODEV;
    }

    if (strcmp(argv[1], "on") == 0) {
        d->tp_diag = true;
    } else if (strcmp(argv[1], "off") == 0) {
        d->tp_diag = false;
    } else if (strcmp(argv[1], "toggle") == 0) {
        d->tp_diag = !d->tp_diag;
    } else {
        shell_error(sh, "use on, off or toggle");
        return -EINVAL;
    }

    shell_print(sh, "diag: %s", d->tp_diag ? "on" : "off");
    return 0;
}

static int tps43_tp_cmd_reanchor(const struct shell *sh, size_t argc, char **argv) {
    struct tps43_drv_data *d = tps43_tp_data(sh);

    (void)argc;
    (void)argv;

    if (d == NULL) {
        return -ENODEV;
    }

    tps43_trackpoint_forget(d);
    shell_print(sh, "anchor dropped; the next single-finger report re-anchors");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_tps43_tp,
    SHELL_CMD(status, NULL, "Show trackpoint state", tps43_tp_cmd_status),
    SHELL_CMD_ARG(mode, NULL, "on|off|toggle", tps43_tp_cmd_mode, 2, 0),
    SHELL_CMD_ARG(layer, NULL, "<layer>|off (-1 = all)", tps43_tp_cmd_layer, 2, 0),
    SHELL_CMD_ARG(deadzone_mm, NULL, "<diameter_mm> (0.0..20.0)",
                  tps43_tp_cmd_deadzone_mm, 2, 0),
    SHELL_CMD_ARG(deadzone_counts, NULL, "<radius_counts> (0..1000)",
                  tps43_tp_cmd_deadzone_counts, 2, 0),
    SHELL_CMD_ARG(radius_mm, NULL, "<outer_radius_mm> (0.1..40.0)",
                  tps43_tp_cmd_radius_mm, 2, 0),
    SHELL_CMD_ARG(radius_counts, NULL, "<outer_radius_counts> (1..4096)",
                  tps43_tp_cmd_radius_counts, 2, 0),
    SHELL_CMD_ARG(speed, NULL, "<px_per_s> (50..5000)", tps43_tp_cmd_speed, 2, 0),
    SHELL_CMD_ARG(exp, NULL, "<x100> (50..500)", tps43_tp_cmd_exp, 2, 0),
    SHELL_CMD_ARG(mm, NULL, "<counts_per_mm> (1..500)", tps43_tp_cmd_mm, 2, 0),
    SHELL_CMD_ARG(diag, NULL, "on|off|toggle", tps43_tp_cmd_diag, 2, 0),
    SHELL_CMD(reanchor, NULL, "Drop the anchor now", tps43_tp_cmd_reanchor),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(tps43_tp, &sub_tps43_tp, "TPS43 trackpoint tuning", NULL);
#endif /* TPS43_TP_SHELL */

/**
 * @brief Public function to manage trackpad sleep mode
 * 
 * This function is used by ZMK power management system (via tps43_idle_sleeper)
 * to put trackpad into sleep mode when keyboard transitions to idle/sleep state.
 * 
 * @param dev Pointer to trackpad device
 * @param sleep true - enter sleep mode, false - wake up
 * @return 0 on success, negative error code on failure
 */
int tps43_rail_power(const struct device *dev, bool on) {
    struct tps43_drv_data *drv_data = dev->data;

    if (!on) {
        if (drv_data->rail_off) {
            return 0;
        }
        /* Stop all traffic before the caller drops the rail. */
        drv_data->rail_off = true;
        drv_data->device_ready = false;
        drv_data->touching = false;
        drv_data->touch_release_count = 0;
        drv_data->gate_x = 0;
        drv_data->gate_y = 0;
        drv_data->gate_run = 0;
        drv_data->drag_active = false;
        k_work_cancel_delayable(&drv_data->poll_work);
        LOG_INF("rail off: polling stopped");
        return 0;
    }

    if (!drv_data->rail_off) {
        return 0;
    }
    drv_data->rail_off = false;

    /* The pad rebooted: registers are back to defaults (EVENT_MODE on,
     * gestures on, no report rate), so run the full init sequence again. */
    int ret = check_reset_and_reconfigure(dev);
    if (ret != 0) {
        LOG_ERR("rail on: reconfigure failed (%d)", ret);
        return ret;
    }

    drv_data->read_failures = 0;
    drv_data->last_ok_ms = k_uptime_get();
    drv_data->suspended = false;

    if (drv_data->polling) {
        k_work_reschedule(&drv_data->poll_work, K_MSEC(TPS43_POLL_IDLE_MS));
    }

    LOG_INF("rail on: pad reconfigured");
    return 0;
}

int tps43_set_sleep(const struct device *dev, bool sleep) {
    if (dev == NULL) {
        return -EINVAL;
    }
    return tps43_set_suspend(dev, sleep);
}
