#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* device data structure */
struct pixart_data {
    const struct device          *dev;
    bool                         sw_smart_flag; // for pmw3610 smart algorithm

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work                trigger_work; // realtrigger job

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;
    uint8_t                      init_retry_count;

    bool                         ready; // whether init is finished successfully
    int                          err; // error code during async init
    atomic_t                     pending_cpi; // pending CPI before ready (0 = none)
    bool                         data_ready; // whether movement data can be reported
    uint8_t                      data_index; // count ignored events before reporting
    uint64_t                     last_data; // last motion sample timestamp (ms)

#ifdef CONFIG_PMW3610_CUSTOM_TRACE
    bool                         trace_window_active;
    uint32_t                     trace_window_id;
    int64_t                      trace_window_start_ms;
    int64_t                      trace_window_until_ms;
    int64_t                      trace_last_summary_ms;
    struct k_work_delayable      trace_window_work;

    /* Window totals */
    uint32_t                     trace_motion_count;
    uint32_t                     trace_report_x_count;
    uint32_t                     trace_report_y_count;
    uint32_t                     trace_report_count;
    uint32_t                     trace_raw_nonzero_count;
    uint32_t                     trace_raw_neg11_count;
    uint32_t                     trace_burst_xy_ff_count;
    uint32_t                     trace_burst_all_ff_count;
    uint32_t                     trace_irq_low_event_count;
    uint32_t                     trace_irq_low_sticky_count;
    uint64_t                     trace_irq_low_total_ms;

    /* Period counters for summary logs */
    uint32_t                     trace_irq_count;
    uint32_t                     trace_period_sample_count;
    uint32_t                     trace_period_report_count;
    uint32_t                     trace_period_raw_nonzero_count;
    uint32_t                     trace_period_raw_neg11_count;
    uint32_t                     trace_period_burst_xy_ff_count;
    uint32_t                     trace_period_burst_all_ff_count;
    uint32_t                     trace_period_irq_low_sticky_count;
    uint32_t                     trace_period_same_delta_max;

    /* Anomaly state */
    bool                         trace_anomaly_active;
    int64_t                      trace_anomaly_start_ms;
    uint32_t                     trace_anomaly_zero_streak;

    /* IRQ LOW stretch tracking */
    bool                         trace_irq_low_active;
    bool                         trace_irq_low_sticky_marked;
    int64_t                      trace_irq_low_start_ms;

    /* Same delta run tracking */
    bool                         trace_have_last_delta;
    int16_t                      trace_last_rx;
    int16_t                      trace_last_ry;
    uint32_t                     trace_same_delta_run;
    uint32_t                     trace_same_delta_max;
#endif
};

// device config data structure
struct pixart_config {
	struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
    bool force_awake;
    bool force_awake_4ms_mode;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
