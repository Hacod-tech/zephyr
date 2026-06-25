/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/sensor/max32664a.h>

#define MAX32664A_CMD_DELAY_MS        10
#define MAX32664A_AFE_ENABLE_DELAY_MS 40
#define MAX32664A_ALGO_ENABLE_DELAY_MS 40
#define MAX32664A_STARTUP_MS          1000

#define MAX32664A_STATUS_DATA_RDY     3

#define MAX32664A_AFE_IDX_MAX30101    0x03
#define MAX32664A_MAX30101_PART_ID    0x15
#define MAX32664A_REG_PART_ID         0xFF

#define MAX32664A_OUT_SENSOR_ALGO     0x03
#define MAX32664A_FIFO_SAMPLE_SIZE    22

#define MAX30101_REG_SPO2_CFG         0x0a
#define MAX30101_SPO2_ADC_RGE_SHIFT   5
#define MAX30101_SPO2_SR_SHIFT        2
#define MAX30101_SPO2_PW_SHIFT        0
#define MAX30101_SPO2_SR_MASK         GENMASK(4, 2)
#define MAX30101_SPO2_PW_MASK         GENMASK(1, 0)
#define MAX30101_SPO2_ADC_RGE_MASK    GENMASK(6, 5)

/** MaximFast mode 1 FIFO sample (sensor + algorithm, UG6806 p.51). */
struct max32664a_sample {
	uint32_t ir;
	uint32_t red;
	uint16_t led3;
	uint16_t led4;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t hr_tenths;
	uint8_t hr_confidence;
	uint16_t spo2_tenths;
	uint8_t algo_state;
};

struct max32664a_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec mfio_gpio;
	uint8_t fifo_threshold;
	bool use_agc;
	bool skip_accelerometer;
	uint16_t sample_rate_hz;
	uint8_t spo2_pw;
	uint8_t spo2_adc_rge;
};

struct max32664a_data {
	struct max32664a_sample sample;
	enum max32664a_algo_mode algo_mode;
	uint16_t sample_rate_hz;
	bool worker_running;

	struct k_thread thread;
	k_tid_t thread_id;
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MAX32664A_THREAD_STACK_SIZE);

	struct k_msgq sample_queue;
	uint8_t sample_queue_buf[CONFIG_MAX32664A_QUEUE_SIZE * sizeof(struct max32664a_sample)];

	uint8_t i2c_buf[(CONFIG_MAX32664A_SAMPLE_BUFFER_SIZE * MAX32664A_FIFO_SAMPLE_SIZE) + 1];
};

int max32664a_i2c_transmit(const struct device *dev, const uint8_t *tx_buf, uint8_t tx_len,
			   uint8_t *rx_buf, uint32_t rx_len, uint16_t delay_ms);

void max32664a_worker(const struct device *dev);
