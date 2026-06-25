/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "max32664a.h"

LOG_MODULE_DECLARE(maxim_max32664a, CONFIG_SENSOR_LOG_LEVEL);

static int hub_status(const struct device *dev, uint8_t *status)
{
	uint8_t tx[2] = { 0x00, 0x00 };
	uint8_t rx[2];
	int ret;

	ret = max32664a_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx),
				     MAX32664A_CMD_DELAY_MS);
	if (ret != 0) {
		return ret;
	}

	if (rx[0] != 0) {
		return -EIO;
	}

	*status = rx[1];
	return 0;
}

static int fifo_count(const struct device *dev, uint8_t *count)
{
	uint8_t tx[2] = { 0x12, 0x00 };
	uint8_t rx[2];
	int ret;

	ret = max32664a_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx),
				     MAX32664A_CMD_DELAY_MS);
	if (ret != 0) {
		return ret;
	}

	if (rx[0] != 0) {
		return -EIO;
	}

	*count = rx[1];
	return 0;
}

static void parse_sample(const uint8_t *raw, struct max32664a_sample *sample)
{
	sample->ir = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2];
	sample->red = ((uint32_t)raw[3] << 16) | ((uint32_t)raw[4] << 8) | raw[5];
	sample->led3 = ((uint16_t)raw[6] << 8) | raw[7];
	sample->led4 = ((uint16_t)raw[8] << 8) | raw[9];
	sample->accel_x = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
	sample->accel_y = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);
	sample->accel_z = (int16_t)(((uint16_t)raw[14] << 8) | raw[15]);
	sample->hr_tenths = ((uint16_t)raw[16] << 8) | raw[17];
	sample->hr_confidence = raw[18];
	sample->spo2_tenths = ((uint16_t)raw[19] << 8) | raw[20];
	sample->algo_state = raw[21];
}

static void push_sample(struct max32664a_data *data, const struct max32664a_sample *sample)
{
	while (k_msgq_put(&data->sample_queue, sample, K_NO_WAIT) != 0) {
		k_msgq_purge(&data->sample_queue);
	}
}

void max32664a_worker(const struct device *dev)
{
	struct max32664a_data *data = dev->data;
	uint8_t tx[2] = { 0x12, 0x01 };

	while (data->worker_running) {
		uint8_t status;
		uint8_t samples;
		int ret;

		ret = hub_status(dev, &status);
		if (ret != 0) {
			k_msleep(50);
			continue;
		}

		if ((status & BIT(MAX32664A_STATUS_DATA_RDY)) == 0) {
			k_msleep(20);
			continue;
		}

		ret = fifo_count(dev, &samples);
		if (ret != 0 || samples == 0) {
			k_msleep(20);
			continue;
		}

		if (samples > CONFIG_MAX32664A_SAMPLE_BUFFER_SIZE) {
			// LOG_WRN("FIFO overflow (%u), dropping", samples);
			samples = CONFIG_MAX32664A_SAMPLE_BUFFER_SIZE;
		}

		ret = max32664a_i2c_transmit(dev, tx, sizeof(tx), data->i2c_buf,
					     (samples * MAX32664A_FIFO_SAMPLE_SIZE) + 1,
					     MAX32664A_CMD_DELAY_MS);
		if (ret != 0 || data->i2c_buf[0] != 0) {
			k_msleep(20);
			continue;
		}

		for (uint8_t i = 0; i < samples; i++) {
			struct max32664a_sample sample;
			const uint8_t *raw = &data->i2c_buf[1 + (i * MAX32664A_FIFO_SAMPLE_SIZE)];

			parse_sample(raw, &sample);
			push_sample(data, &sample);
		}
	}
}
