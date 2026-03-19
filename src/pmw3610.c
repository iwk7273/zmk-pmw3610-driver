/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610_custom

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_CUSTOM_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10 + CONFIG_PMW3610_CUSTOM_INIT_POWER_UP_EXTRA_DELAY_MS, // >10ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200, // 150 us required, test shows too short,
                                       // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,  // 10 ms required in spec,
                                       // test shows too short,
                                       // especially when integrated with display,
                                       // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

#ifdef CONFIG_PMW3610_CUSTOM_TRACE
#define PMW3610_TRACE_BURST_FF_WARN_PCT 70U
#define PMW3610_TRACE_MIN_RATIO_SAMPLES 40U
#define PMW3610_TRACE_ANOMALY_END_ZERO_SAMPLES 24U
#define PMW3610_TRACE_IRQ_LOW_STICKY_MS 200

static void pmw3610_trace_reset_period(struct pixart_data *data) {
    data->trace_irq_count = 0U;
    data->trace_period_sample_count = 0U;
    data->trace_period_report_count = 0U;
    data->trace_period_raw_nonzero_count = 0U;
    data->trace_period_raw_neg11_count = 0U;
    data->trace_period_burst_xy_ff_count = 0U;
    data->trace_period_burst_all_ff_count = 0U;
    data->trace_period_irq_low_sticky_count = 0U;
    data->trace_period_same_delta_max = data->trace_same_delta_run;
}

static void pmw3610_trace_flush_same_delta(struct pixart_data *data) {
    if (!data->trace_have_last_delta) {
        return;
    }

    if (data->trace_same_delta_run > data->trace_period_same_delta_max) {
        data->trace_period_same_delta_max = data->trace_same_delta_run;
    }
    if (data->trace_same_delta_run > data->trace_same_delta_max) {
        data->trace_same_delta_max = data->trace_same_delta_run;
    }
}

static void pmw3610_trace_reset_window_counters(struct pixart_data *data) {
    data->trace_motion_count = 0U;
    data->trace_report_x_count = 0U;
    data->trace_report_y_count = 0U;
    data->trace_report_count = 0U;
    data->trace_raw_nonzero_count = 0U;
    data->trace_raw_neg11_count = 0U;
    data->trace_burst_xy_ff_count = 0U;
    data->trace_burst_all_ff_count = 0U;
    data->trace_irq_low_event_count = 0U;
    data->trace_irq_low_sticky_count = 0U;
    data->trace_irq_low_total_ms = 0U;

    data->trace_anomaly_active = false;
    data->trace_anomaly_start_ms = 0;
    data->trace_anomaly_zero_streak = 0U;

    data->trace_irq_low_active = false;
    data->trace_irq_low_sticky_marked = false;
    data->trace_irq_low_start_ms = 0;

    data->trace_have_last_delta = false;
    data->trace_last_rx = 0;
    data->trace_last_ry = 0;
    data->trace_same_delta_run = 0U;
    data->trace_same_delta_max = 0U;

    pmw3610_trace_reset_period(data);
}

static void pmw3610_trace_finish_window(struct pixart_data *data, const char *reason) {
    if (!data->trace_window_active) {
        return;
    }

    int64_t now = k_uptime_get();
    if (data->trace_anomaly_active) {
        int64_t active_ms = now - data->trace_anomaly_start_ms;
        LOG_INF("TRACE anomaly end id=%u reason=%s active_ms=%lld", data->trace_window_id, reason,
                (long long)active_ms);
        data->trace_anomaly_active = false;
    }

    pmw3610_trace_flush_same_delta(data);
    int64_t elapsed = now - data->trace_window_start_ms;
    LOG_INF("TRACE window end id=%u reason=%s elapsed_ms=%lld samples=%u reports=%u raw_nonzero=%u "
            "raw_neg11=%u burst_xy_ff=%u burst_all_ff=%u irq_low_events=%u irq_low_sticky=%u "
            "irq_low_total_ms=%llu same_delta_max=%u",
            data->trace_window_id, reason, (long long)elapsed, data->trace_motion_count,
            data->trace_report_count, data->trace_raw_nonzero_count, data->trace_raw_neg11_count,
            data->trace_burst_xy_ff_count, data->trace_burst_all_ff_count,
            data->trace_irq_low_event_count, data->trace_irq_low_sticky_count,
            (unsigned long long)data->trace_irq_low_total_ms, data->trace_same_delta_max);

    data->trace_window_active = false;
}

static bool pmw3610_trace_is_window_active(struct pixart_data *data) {
    if (!data->trace_window_active) {
        return false;
    }

    if (k_uptime_get() <= data->trace_window_until_ms) {
        return true;
    }

    pmw3610_trace_finish_window(data, "timeout");
    return false;
}

static void pmw3610_trace_start_window(struct pixart_data *data, const char *reason, int state) {
    int64_t now = k_uptime_get();
    if (data->trace_window_active) {
        pmw3610_trace_finish_window(data, "restart");
    }

    data->trace_window_active = true;
    data->trace_window_id++;
    data->trace_window_start_ms = now;
    data->trace_window_until_ms = now + CONFIG_PMW3610_CUSTOM_TRACE_WINDOW_MS;
    data->trace_last_summary_ms = now;
    pmw3610_trace_reset_window_counters(data);

    LOG_INF("TRACE window start id=%u reason=%s state=%d start_ms=%lld duration_ms=%d ready=%d "
            "data_ready=%d idx=%u",
            data->trace_window_id, reason, state, (long long)now,
            CONFIG_PMW3610_CUSTOM_TRACE_WINDOW_MS, data->ready, data->data_ready, data->data_index);
}

static void pmw3610_trace_log_activity_state(struct pixart_data *data, int state) {
    int64_t now = k_uptime_get();
    LOG_INF("TRACE activity state=%d now_ms=%lld ready=%d data_ready=%d idx=%u", state,
            (long long)now, data->ready, data->data_ready, data->data_index);
    pmw3610_trace_start_window(data, "activity_state", state);
}

static void pmw3610_trace_log_perf_transition(struct pixart_data *data, bool enabled, uint8_t before,
                                              uint8_t after, bool changed) {
    LOG_INF("TRACE performance enabled=%d before=0x%x after=0x%x changed=%d data_ready=%d idx=%u",
            enabled, before, after, changed, data->data_ready, data->data_index);
}

static void pmw3610_trace_sample_irq_level(struct pixart_data *data, const char *src) {
    if (!pmw3610_trace_is_window_active(data)) {
        return;
    }

    const struct pixart_config *config = data->dev->config;
    int level = gpio_pin_get_dt(&config->irq_gpio);
    if (level < 0) {
        return;
    }

    int64_t now = k_uptime_get();
    if (level == 0) {
        if (!data->trace_irq_low_active) {
            data->trace_irq_low_active = true;
            data->trace_irq_low_sticky_marked = false;
            data->trace_irq_low_start_ms = now;
            data->trace_irq_low_event_count++;
        } else if (!data->trace_irq_low_sticky_marked &&
                   now - data->trace_irq_low_start_ms >= PMW3610_TRACE_IRQ_LOW_STICKY_MS) {
            data->trace_irq_low_sticky_marked = true;
            data->trace_irq_low_sticky_count++;
            data->trace_period_irq_low_sticky_count++;
            LOG_INF("TRACE irq-low-sticky id=%u src=%s low_ms=%lld", data->trace_window_id, src,
                    (long long)(now - data->trace_irq_low_start_ms));
        }
        return;
    }

    if (!data->trace_irq_low_active) {
        return;
    }

    int64_t low_ms = now - data->trace_irq_low_start_ms;
    if (low_ms < 0) {
        low_ms = 0;
    }

    if (!data->trace_irq_low_sticky_marked && low_ms >= PMW3610_TRACE_IRQ_LOW_STICKY_MS) {
        data->trace_irq_low_sticky_count++;
        data->trace_period_irq_low_sticky_count++;
    }

    data->trace_irq_low_total_ms += (uint64_t)low_ms;
    data->trace_irq_low_active = false;
    data->trace_irq_low_sticky_marked = false;
}

static void pmw3610_trace_on_irq(struct pixart_data *data) {
    if (!pmw3610_trace_is_window_active(data)) {
        return;
    }

    data->trace_irq_count++;
    pmw3610_trace_sample_irq_level(data, "irq");
}

static void pmw3610_trace_log_motion_sample(struct pixart_data *data, int16_t raw_x, int16_t raw_y,
                                            int16_t adj_x, int16_t adj_y, bool ignored,
                                            const uint8_t *burst) {
    if (!pmw3610_trace_is_window_active(data)) {
        return;
    }

    bool raw_nonzero = (raw_x != 0) || (raw_y != 0);
    bool raw_neg11 = (raw_x == -1) && (raw_y == -1);
    bool burst_xy_ff = (burst[PMW3610_X_L_POS] == 0xFF) && (burst[PMW3610_Y_L_POS] == 0xFF) &&
                       (burst[PMW3610_XY_H_POS] == 0xFF);
    bool burst_all_ff = true;
    for (size_t i = 0; i < PMW3610_BURST_SIZE; i++) {
        if (burst[i] != 0xFF) {
            burst_all_ff = false;
            break;
        }
    }

    data->trace_motion_count++;
    data->trace_period_sample_count++;
    if (raw_nonzero) {
        data->trace_raw_nonzero_count++;
        data->trace_period_raw_nonzero_count++;
    }
    if (raw_neg11) {
        data->trace_raw_neg11_count++;
        data->trace_period_raw_neg11_count++;
    }
    if (burst_xy_ff) {
        data->trace_burst_xy_ff_count++;
        data->trace_period_burst_xy_ff_count++;
    }
    if (burst_all_ff) {
        data->trace_burst_all_ff_count++;
        data->trace_period_burst_all_ff_count++;
    }

    int64_t now = k_uptime_get();
    if (!data->trace_anomaly_active && raw_nonzero) {
        data->trace_anomaly_active = true;
        data->trace_anomaly_start_ms = now;
        data->trace_anomaly_zero_streak = 0U;
        LOG_INF("TRACE anomaly start id=%u reason=%s raw=%d/%d adj=%d/%d ignored=%d "
                "burst=%02x %02x %02x %02x",
                data->trace_window_id, raw_neg11 ? "raw_neg11" : "raw_nonzero", raw_x, raw_y, adj_x,
                adj_y, ignored, burst[0], burst[1], burst[2], burst[3]);
    } else if (data->trace_anomaly_active) {
        if (raw_nonzero) {
            data->trace_anomaly_zero_streak = 0U;
        } else {
            if (++data->trace_anomaly_zero_streak >= PMW3610_TRACE_ANOMALY_END_ZERO_SAMPLES) {
                int64_t active_ms = now - data->trace_anomaly_start_ms;
                LOG_INF("TRACE anomaly end id=%u reason=stabilized active_ms=%lld",
                        data->trace_window_id, (long long)active_ms);
                data->trace_anomaly_active = false;
                data->trace_anomaly_zero_streak = 0U;
            }
        }
    }
}

static void pmw3610_trace_log_report_emit(struct pixart_data *data, int16_t rx, int16_t ry) {
    if (!pmw3610_trace_is_window_active(data)) {
        return;
    }

    data->trace_report_count++;
    data->trace_period_report_count++;
    if (rx != 0) {
        data->trace_report_x_count++;
    }
    if (ry != 0) {
        data->trace_report_y_count++;
    }

    if (!data->trace_have_last_delta) {
        data->trace_have_last_delta = true;
        data->trace_last_rx = rx;
        data->trace_last_ry = ry;
        data->trace_same_delta_run = 1U;
        pmw3610_trace_flush_same_delta(data);
        return;
    }

    if (rx == data->trace_last_rx && ry == data->trace_last_ry) {
        data->trace_same_delta_run++;
        pmw3610_trace_flush_same_delta(data);
        return;
    }

    pmw3610_trace_flush_same_delta(data);
    data->trace_last_rx = rx;
    data->trace_last_ry = ry;
    data->trace_same_delta_run = 1U;
}

static void pmw3610_trace_maybe_report_irq_rate(struct pixart_data *data) {
    if (!pmw3610_trace_is_window_active(data)) {
        return;
    }

    pmw3610_trace_sample_irq_level(data, "work");

    int64_t now = k_uptime_get();
    int64_t span = now - data->trace_last_summary_ms;
    if (span < CONFIG_PMW3610_CUSTOM_TRACE_IRQ_REPORT_MS) {
        return;
    }

    pmw3610_trace_flush_same_delta(data);

    uint32_t samples = data->trace_period_sample_count;
    uint32_t samples_ps = (span > 0) ? (uint32_t)(((uint64_t)samples * 1000U) / (uint64_t)span) : 0U;
    uint32_t irqs = data->trace_irq_count;
    uint32_t irqs_ps = (span > 0) ? (uint32_t)(((uint64_t)irqs * 1000U) / (uint64_t)span) : 0U;
    uint32_t reports = data->trace_period_report_count;
    uint32_t reports_ps =
        (span > 0) ? (uint32_t)(((uint64_t)reports * 1000U) / (uint64_t)span) : 0U;
    uint32_t raw_neg11_pct =
        (samples > 0) ? (uint32_t)(((uint64_t)data->trace_period_raw_neg11_count * 100U) / samples)
                      : 0U;
    uint32_t burst_xy_ff_pct =
        (samples > 0)
            ? (uint32_t)(((uint64_t)data->trace_period_burst_xy_ff_count * 100U) / samples)
            : 0U;
    uint32_t burst_all_ff_pct =
        (samples > 0)
            ? (uint32_t)(((uint64_t)data->trace_period_burst_all_ff_count * 100U) / samples)
            : 0U;
    uint32_t current_irq_low_ms = 0U;
    if (data->trace_irq_low_active && now > data->trace_irq_low_start_ms) {
        current_irq_low_ms = (uint32_t)(now - data->trace_irq_low_start_ms);
    }

    LOG_INF("TRACE summary id=%u span_ms=%lld samples=%u(%u/s) irq=%u(%u/s) reports=%u(%u/s) "
            "raw_nonzero=%u raw_neg11=%u(%u%%) burst_xy_ff=%u(%u%%) burst_all_ff=%u(%u%%) "
            "same_delta_max=%u current_same=%u irq_low_ms=%u irq_low_sticky=%u",
            data->trace_window_id, (long long)span, samples, samples_ps, irqs, irqs_ps, reports,
            reports_ps, data->trace_period_raw_nonzero_count, data->trace_period_raw_neg11_count,
            raw_neg11_pct, data->trace_period_burst_xy_ff_count, burst_xy_ff_pct,
            data->trace_period_burst_all_ff_count, burst_all_ff_pct, data->trace_period_same_delta_max,
            data->trace_same_delta_run, current_irq_low_ms, data->trace_period_irq_low_sticky_count);

    if (samples >= PMW3610_TRACE_MIN_RATIO_SAMPLES &&
        burst_all_ff_pct >= PMW3610_TRACE_BURST_FF_WARN_PCT) {
        LOG_INF("TRACE burst-ff-high id=%u burst_all_ff_pct=%u samples=%u", data->trace_window_id,
                burst_all_ff_pct, samples);
    }

    data->trace_last_summary_ms = now;
    pmw3610_trace_reset_period(data);
}
#endif // CONFIG_PMW3610_CUSTOM_TRACE

//////// Function definitions //////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
	const struct pixart_config *cfg = dev->config;
	const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(addr), },
		{ .buf = value, .len = len, },
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
	return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
	const struct pixart_config *cfg = dev->config;
	uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
	const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };
	return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    int err = pmw3610_write_reg(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }
    
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return 0;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    // Convert CPI to register value
    uint8_t value = (cpi / 200);
    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    /* set the cpi */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    /* Write data */
    int err;
    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 8160; // 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_CUSTOM_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_CUSTOM_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_CUSTOM_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_CUSTOM_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
    const struct pixart_config *config = dev->config;
    struct pixart_data *data = dev->data;
    int err = 0;

    if (config->force_awake) {
        uint8_t value;
        err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
        if (err) {
            LOG_ERR("Can't read ref-performance %d", err);
            return err;
        }
        LOG_INF("Get performance register (reg value 0x%x)", value);

        // Set prefered RUN RATE        
        //   BIT 3:   VEL_RUNRATE    0x0: 8ms; 0x1 4ms;
        //   BIT 2:   POSHI_RUN_RATE 0x0: 8ms; 0x1 4ms;
        //   BIT 1-0: POSLO_RUN_RATE 0x0: 8ms; 0x1 4ms; 0x2 2ms; 0x4 Reserved
        uint8_t perf;
        if (config->force_awake_4ms_mode) {
            perf = 0x0d; // RUN RATE @ 4ms
        } else {
            // reset bit[3..0] to 0x0 (normal operation)
            perf = value & 0x0F; // RUN RATE @ 8ms
        }

        if (enabled) {
            perf |= 0xF0; // set bit[3..0] to 0xF (force awake)
        }
        bool perf_changed = perf != value;
        if (perf_changed) {
            data->data_index = 0;
            data->data_ready = (CONFIG_PMW3610_CUSTOM_IGNORE_FIRST_N == 0);
            err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
            if (err) {
                LOG_ERR("Can't write performance register %d", err);
                return err;
            }
            LOG_INF("Set performance register (reg value 0x%x)", perf);
        }
#ifdef CONFIG_PMW3610_CUSTOM_TRACE
        pmw3610_trace_log_perf_transition(data, enabled, value, perf, perf_changed);
#endif
        LOG_INF("%s performance mode", enabled ? "enable" : "disable");
    }

    return err;
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int pmw3610_async_init_power_up(const struct device *dev) {
	int ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    uint8_t product_id = 0x01;
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    int err = 0;
    const struct pixart_config *config = dev->config;

    // clear motion registers first (required in datasheet)
    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = pmw3610_read_reg(dev, reg, buf);
    }

    if (!err) {
        err = pmw3610_set_performance(dev, true);
    }

    if (!err) {
        err = pmw3610_set_cpi(dev, config->cpi);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                         CONFIG_PMW3610_CUSTOM_RUN_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                         CONFIG_PMW3610_CUSTOM_REST1_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                         CONFIG_PMW3610_CUSTOM_REST2_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                      CONFIG_PMW3610_CUSTOM_REST1_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                      CONFIG_PMW3610_CUSTOM_REST2_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
                                      CONFIG_PMW3610_CUSTOM_REST3_SAMPLE_TIME_MS);
    }

    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

  static void pmw3610_async_init(struct k_work *work) {
      struct k_work_delayable *work2 = (struct k_work_delayable *)work;
      struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
      const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        if (CONFIG_PMW3610_CUSTOM_INIT_RETRY_COUNT > 0 &&
            data->init_retry_count < CONFIG_PMW3610_CUSTOM_INIT_RETRY_COUNT) {
            data->init_retry_count++;
            LOG_WRN("PMW3610 init failed in step %d, retry %u/%u", data->async_init_step,
                    data->init_retry_count, CONFIG_PMW3610_CUSTOM_INIT_RETRY_COUNT);
            data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
            k_work_schedule(&data->init_work,
                            K_MSEC(async_init_delay[ASYNC_INIT_STEP_POWER_UP] +
                                   CONFIG_PMW3610_CUSTOM_INIT_RETRY_DELAY_MS));
        } else {
            LOG_ERR("PMW3610 initialization failed in step %d", data->async_init_step);
            LOG_ERR("PMW3610 init failed after %u retries", data->init_retry_count);
        }
        return;
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            data->init_retry_count = 0;
            data->data_index = 0;
            data->data_ready = (CONFIG_PMW3610_CUSTOM_IGNORE_FIRST_N == 0);
            data->last_data = k_uptime_get();
            LOG_INF("PMW3610 initialized");
            pmw3610_set_interrupt(dev, true);
            uint32_t pending_cpi = (uint32_t)atomic_get(&data->pending_cpi);
              if (pending_cpi != 0) {
                  int err = pmw3610_set_cpi(dev, pending_cpi);
                  if (err) {
                      LOG_WRN("Failed to apply pending CPI %u (%d)", pending_cpi, err);
                  } else {
                      atomic_set(&data->pending_cpi, 0);
                      LOG_INF("Applied pending CPI %u", pending_cpi);
                  }
              }
          } else {
              k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
          }
      }
  }

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    uint8_t buf[PMW3610_BURST_SIZE];

#if CONFIG_PMW3610_CUSTOM_REPORT_INTERVAL_MIN > 0 ||                                               \
    IS_ENABLED(CONFIG_PMW3610_CUSTOM_IGNORE_AFTER_REST) ||                                         \
    IS_ENABLED(CONFIG_PMW3610_CUSTOM_ANTI_WARP)
    int64_t now = k_uptime_get();
#endif
#if IS_ENABLED(CONFIG_PMW3610_CUSTOM_IGNORE_AFTER_REST) ||                                          \
    IS_ENABLED(CONFIG_PMW3610_CUSTOM_ANTI_WARP)
    int64_t passed = now - (int64_t)data->last_data;
#endif

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    static int64_t dx = 0;
    static int64_t dy = 0;

#if CONFIG_PMW3610_CUSTOM_REPORT_INTERVAL_MIN > 0
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
#endif

	int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
    if (err) {
        return err;
    }
    // LOG_HEXDUMP_DBG(buf, PMW3610_BURST_SIZE, "buf");

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

    int16_t raw_x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t raw_y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);
    int16_t x = raw_x;
    int16_t y = raw_y;
    LOG_DBG("x/y: %d/%d", x, y);

#if IS_ENABLED(CONFIG_PMW3610_CUSTOM_SWAP_XY)
    int16_t a = x;
    x = y;
    y = a;
#endif
#if IS_ENABLED(CONFIG_PMW3610_CUSTOM_INVERT_X)
    x = -x;
#endif
#if IS_ENABLED(CONFIG_PMW3610_CUSTOM_INVERT_Y)
    y = -y;
#endif

#if IS_ENABLED(CONFIG_PMW3610_CUSTOM_IGNORE_AFTER_REST)
    if (passed > CONFIG_PMW3610_CUSTOM_RUN_DOWNSHIFT_TIME_MS +
                     CONFIG_PMW3610_CUSTOM_REST1_DOWNSHIFT_TIME_MS +
                     CONFIG_PMW3610_CUSTOM_REST2_DOWNSHIFT_TIME_MS) {
        data->data_index = 0;
        data->data_ready = (CONFIG_PMW3610_CUSTOM_IGNORE_FIRST_N == 0);
    }
#endif

#if IS_ENABLED(CONFIG_PMW3610_CUSTOM_ANTI_WARP)
    if (passed > CONFIG_PMW3610_CUSTOM_ANTI_WARP_INACTIVITY_MS &&
        (x > CONFIG_PMW3610_CUSTOM_ANTI_WARP_THRES || y > CONFIG_PMW3610_CUSTOM_ANTI_WARP_THRES) &&
        data->data_ready) {
        data->last_data = now;
        data->data_index = CONFIG_PMW3610_CUSTOM_IGNORE_FIRST_N / 2;
        data->data_ready = (CONFIG_PMW3610_CUSTOM_IGNORE_FIRST_N == 0);
        LOG_WRN("Discarded large movement after inactivity, likely warping");
    }
#endif

#ifdef CONFIG_PMW3610_CUSTOM_SMART_ALGORITHM
    int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) 
                    + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        pmw3610_write(dev, 0x32, 0x00);
        data->sw_smart_flag = false;
    }
    if (!data->sw_smart_flag && shutter > 45) {
        pmw3610_write(dev, 0x32, 0x80);
        data->sw_smart_flag = true;
    }
#endif

#if IS_ENABLED(CONFIG_PMW3610_CUSTOM_IGNORE_AFTER_REST) ||                                          \
    IS_ENABLED(CONFIG_PMW3610_CUSTOM_ANTI_WARP)
    data->last_data = now;
#endif

    bool ignored_sample = !data->data_ready;
#ifdef CONFIG_PMW3610_CUSTOM_TRACE
    pmw3610_trace_log_motion_sample(data, raw_x, raw_y, x, y, ignored_sample, buf);
#endif

    if (!data->data_ready) {
        if (++data->data_index >= CONFIG_PMW3610_CUSTOM_IGNORE_FIRST_N) {
            data->data_ready = true;
        }
        return 0;
    }

#if CONFIG_PMW3610_CUSTOM_REPORT_INTERVAL_MIN > 0
    // purge accumulated delta, if last sampled had not been reported on last report tick
    if (now - last_smp_time >= CONFIG_PMW3610_CUSTOM_REPORT_INTERVAL_MIN) {
        dx = 0;
        dy = 0;
    }
    last_smp_time = now;
#endif

    // accumulate delta until report in next iteration
    dx += x;
    dy += y;

#if CONFIG_PMW3610_CUSTOM_REPORT_INTERVAL_MIN > 0
    // strict to report inerval
    if (now - last_rpt_time < CONFIG_PMW3610_CUSTOM_REPORT_INTERVAL_MIN) {
        return 0;
    }
#endif

    // fetch report value
    int16_t rx = (int16_t)CLAMP(dx, INT16_MIN, INT16_MAX);
    int16_t ry = (int16_t)CLAMP(dy, INT16_MIN, INT16_MAX);
    bool have_x = rx != 0;
    bool have_y = ry != 0;

    if (have_x || have_y) {
#if CONFIG_PMW3610_CUSTOM_REPORT_INTERVAL_MIN > 0
        last_rpt_time = now;
#endif
        dx = 0;
        dy = 0;
#ifdef CONFIG_PMW3610_CUSTOM_TRACE
        pmw3610_trace_log_report_emit(data, rx, ry);
#endif
        if (have_x) {
            input_report(dev, config->evt_type, config->x_input_code, rx, !have_y, K_NO_WAIT);
        }
        if (have_y) {
            input_report(dev, config->evt_type, config->y_input_code, ry, true, K_NO_WAIT);
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
#ifdef CONFIG_PMW3610_CUSTOM_TRACE
    pmw3610_trace_on_irq(data);
#endif
    pmw3610_set_interrupt(dev, false);
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;
    pmw3610_report_data(dev);
#ifdef CONFIG_PMW3610_CUSTOM_TRACE
    pmw3610_trace_maybe_report_irq_rate(data);
#endif
    pmw3610_set_interrupt(dev, true);
}

static int pmw3610_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    return err;
}

  static int pmw3610_init(const struct device *dev) {
      struct pixart_data *data = dev->data;
      const struct pixart_config *config = dev->config;
      int err;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("%s is not ready", config->spi.bus->name);
		return -ENODEV;
	}

    // init device pointer
    data->dev = dev;
    atomic_set(&data->pending_cpi, 0);
    data->init_retry_count = 0;

    // init smart algorithm flag;
    data->sw_smart_flag = false;
    data->data_index = 0;
    data->data_ready = (CONFIG_PMW3610_CUSTOM_IGNORE_FIRST_N == 0);
    data->last_data = k_uptime_get();
#ifdef CONFIG_PMW3610_CUSTOM_TRACE
    data->trace_window_active = false;
    data->trace_window_id = 0U;
    data->trace_window_start_ms = 0;
    data->trace_window_until_ms = 0;
    data->trace_last_summary_ms = 0;
    pmw3610_trace_reset_window_counters(data);
#endif

    // init trigger handler work
    k_work_init(&data->trigger_work, pmw3610_work_callback);

    // init irq routine
    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // 3. other configs like cpi, downshift time, sample time etc.
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

  static int pmw3610_attr_set(const struct device *dev, enum sensor_channel chan,
                              enum sensor_attribute attr, const struct sensor_value *val) {
      struct pixart_data *data = dev->data;
      int err;

      if (unlikely(chan != SENSOR_CHAN_ALL)) {
          return -ENOTSUP;
      }

      if (unlikely(!data->ready)) {
          if ((uint32_t)attr == PMW3610_ATTR_CPI) {
              uint32_t cpi = PMW3610_SVALUE_TO_CPI(*val);
              atomic_set(&data->pending_cpi, (atomic_val_t)cpi);
              LOG_INF("Queued CPI %u", cpi);
              return 0;
          }
          LOG_DBG("Device is not initialized yet");
          return -EBUSY;
      }

    switch ((uint32_t)attr) {
    case PMW3610_ATTR_CPI:
        err = pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val));
        break;

    case PMW3610_ATTR_RUN_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST1_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST2_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST1_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST2_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST3_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .attr_set = pmw3610_attr_set,
};

// #if IS_ENABLED(CONFIG_PM_DEVICE)
// static int pmw3610_pm_action(const struct device *dev, enum pm_device_action action) {
//     switch (action) {
//     case PM_DEVICE_ACTION_SUSPEND:
//         return pmw3610_set_interrupt(dev, false);
//     case PM_DEVICE_ACTION_RESUME:
//         return pmw3610_set_interrupt(dev, true);
//     default:
//         return -ENOTSUP;
//     }
// }
// #endif // IS_ENABLED(CONFIG_PM_DEVICE)
// PM_DEVICE_DT_INST_DEFINE(n, pmw3610_pm_action);

#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
                        SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PMW3610_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
    static const struct pixart_config config##n = {                                                \
		.spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),		                               \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                                       \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                                             \
        .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                                     \
        .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                                     \
        .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                                       \
        .force_awake_4ms_mode = DT_PROP(DT_DRV_INST(n), force_awake_4ms_mode),                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_INPUT_PMW3610_CUSTOM_INIT_PRIORITY, &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)


#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610_custom, GET_PMW3610_DEV)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }

    bool enable = state_ev->state == ZMK_ACTIVITY_ACTIVE ? 1 : 0;
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        struct pixart_data *data = pmw3610_devs[i]->data;
#ifdef CONFIG_PMW3610_CUSTOM_TRACE
        pmw3610_trace_log_activity_state(data, state_ev->state);
#endif
        pmw3610_set_performance(pmw3610_devs[i], enable);
        if (!enable) {
            data->data_index = 0;
            data->data_ready = (CONFIG_PMW3610_CUSTOM_IGNORE_FIRST_N == 0);
            data->last_data = k_uptime_get();
        }
    }

    return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
