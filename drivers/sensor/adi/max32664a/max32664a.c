/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAX32664A sensor hub driver for MaximFast algorithm with MAX30101 AFE.
 * Protocol per Maxim UG6806.
 */

#include "max32664a.h"

#define DT_DRV_COMPAT maxim_max32664a

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "max32664a driver enabled without any devices"
#endif

LOG_MODULE_REGISTER(maxim_max32664a, CONFIG_SENSOR_LOG_LEVEL);

int max32664a_i2c_transmit(const struct device *dev, const uint8_t *tx_buf, uint8_t tx_len,
			   uint8_t *rx_buf, uint32_t rx_len, uint16_t delay_ms)
{
	const struct max32664a_config *config = dev->config;

	if (i2c_write_dt(&config->i2c, tx_buf, tx_len) != 0) {
		return -EIO;
	}

	k_msleep(delay_ms);

	if (i2c_read_dt(&config->i2c, rx_buf, rx_len) != 0) {
		return -EIO;
	}

	k_msleep(MAX32664A_CMD_DELAY_MS);

	if (rx_buf[0] != 0) {
		return -EINVAL;
	}

	return 0;
}

static int hub_cmd(const struct device *dev, const uint8_t *tx, uint8_t tx_len, uint16_t delay_ms)
{
	uint8_t rx;

	return max32664a_i2c_transmit(dev, tx, tx_len, &rx, 1, delay_ms);
}

static int max30101_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	uint8_t tx[3] = { 0x41, MAX32664A_AFE_IDX_MAX30101, reg };
	uint8_t rx[2];
	int ret;

	ret = max32664a_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx),
				     MAX32664A_CMD_DELAY_MS);
	if (ret != 0) {
		return ret;
	}

	*val = rx[1];
	return 0;
}

static int max30101_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	uint8_t tx[4] = { 0x40, MAX32664A_AFE_IDX_MAX30101, reg, val };

	return hub_cmd(dev, tx, sizeof(tx), MAX32664A_CMD_DELAY_MS);
}

static int sample_rate_to_spo2_sr(uint16_t hz, uint8_t *sr)
{
	switch (hz) {
	case 50:
		*sr = 0;
		return 0;
	case 100:
		*sr = 1;
		return 0;
	case 200:
		*sr = 2;
		return 0;
	case 400:
		*sr = 3;
		return 0;
	case 800:
		*sr = 4;
		return 0;
	case 1000:
		*sr = 5;
		return 0;
	case 1600:
		*sr = 6;
		return 0;
	case 3200:
		*sr = 7;
		return 0;
	default:
		return -EINVAL;
	}
}

static int apply_max30101_sample_rate(const struct device *dev, uint16_t hz)
{
	const struct max32664a_config *config = dev->config;
	uint8_t sr;
	uint8_t reg;
	uint8_t new_reg;
	int ret;

	ret = sample_rate_to_spo2_sr(hz, &sr);
	if (ret != 0) {
		return ret;
	}

	ret = max30101_read_reg(dev, MAX30101_REG_SPO2_CFG, &reg);
	if (ret != 0) {
		return ret;
	}

	new_reg = (reg & ~MAX30101_SPO2_SR_MASK) | (sr << MAX30101_SPO2_SR_SHIFT);
	new_reg = (new_reg & ~MAX30101_SPO2_PW_MASK) |
		  ((config->spo2_pw & 0x03) << MAX30101_SPO2_PW_SHIFT);
	new_reg = (new_reg & ~MAX30101_SPO2_ADC_RGE_MASK) |
		  ((config->spo2_adc_rge & 0x03) << MAX30101_SPO2_ADC_RGE_SHIFT);

	if (new_reg == reg) {
		return 0;
	}

	return max30101_write_reg(dev, MAX30101_REG_SPO2_CFG, new_reg);
}

static int check_max30101(const struct device *dev)
{
	uint8_t tx[3] = { 0x41, MAX32664A_AFE_IDX_MAX30101, MAX32664A_REG_PART_ID };
	uint8_t rx[2];
	int ret;

	ret = max32664a_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx),
				     MAX32664A_CMD_DELAY_MS);
	if (ret != 0) {
		return ret;
	}

	if (rx[1] != MAX32664A_MAX30101_PART_ID) {
		LOG_ERR("MAX30101 PART_ID 0x%02x, expected 0x%02x", rx[1],
			MAX32664A_MAX30101_PART_ID);
		return -ENODEV;
	}

	LOG_DBG("MAX30101 PART_ID OK");
	return 0;
}

static int set_output_mode(const struct device *dev, uint8_t mode)
{
	uint8_t tx[3] = { 0x10, 0x00, mode };

	return hub_cmd(dev, tx, sizeof(tx), MAX32664A_CMD_DELAY_MS);
}

static int set_fifo_threshold(const struct device *dev, uint8_t threshold)
{
	uint8_t tx[3] = { 0x10, 0x01, threshold };

	return hub_cmd(dev, tx, sizeof(tx), MAX32664A_CMD_DELAY_MS);
}

static int afe_enable(const struct device *dev, bool enable)
{
	uint8_t tx[3] = { 0x44, MAX32664A_AFE_IDX_MAX30101, enable ? 0x01 : 0x00 };

	return hub_cmd(dev, tx, sizeof(tx), MAX32664A_AFE_ENABLE_DELAY_MS);
}

static int accel_enable(const struct device *dev, bool enable)
{
	uint8_t tx[4] = { 0x44, 0x04, enable ? 0x01 : 0x00, 0x00 };

	return hub_cmd(dev, tx, sizeof(tx), 20);
}

static int agc_enable(const struct device *dev, bool enable)
{
	uint8_t tx[3] = { 0x52, 0x00, enable ? 0x01 : 0x00 };

	return hub_cmd(dev, tx, sizeof(tx), 20);
}

static int maximfast_enable(const struct device *dev, bool enable)
{
	uint8_t tx[3] = { 0x52, 0x02, enable ? 0x01 : 0x00 };

	return hub_cmd(dev, tx, sizeof(tx), MAX32664A_ALGO_ENABLE_DELAY_MS);
}

static int enter_application_mode(const struct device *dev)
{
	const struct max32664a_config *config = dev->config;

	gpio_pin_set_dt(&config->reset_gpio, 0);
	k_msleep(20);

	gpio_pin_set_dt(&config->mfio_gpio, 1);
	k_msleep(20);

	gpio_pin_set_dt(&config->reset_gpio, 1);
	k_msleep(MAX32664A_STARTUP_MS);

	return 0;
}

static int verify_application_mode(const struct device *dev)
{
	uint8_t tx[2] = { 0x02, 0x00 };
	uint8_t rx[2];
	int ret;

	ret = max32664a_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx),
				     MAX32664A_CMD_DELAY_MS);
	if (ret != 0) {
		return ret;
	}

	if (rx[1] != 0x00) {
		LOG_ERR("Hub not in application mode (0x%02x)", rx[1]);
		return -EINVAL;
	}

	return 0;
}

static int start_worker(const struct device *dev)
{
	struct max32664a_data *data = dev->data;

	if (data->thread_id == NULL) {
		data->worker_running = true;
		data->thread_id = k_thread_create(
			&data->thread, data->thread_stack,
			K_THREAD_STACK_SIZEOF(data->thread_stack),
			(k_thread_entry_t)max32664a_worker, (void *)dev, NULL, NULL,
			K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
		k_thread_name_set(data->thread_id, "max32664a");
	} else {
		data->worker_running = true;
		k_thread_resume(data->thread_id);
	}

	return 0;
}

static int stop_worker(const struct device *dev)
{
	struct max32664a_data *data = dev->data;

	data->worker_running = false;
	if (data->thread_id != NULL) {
		k_thread_suspend(data->thread_id);
	}

	return 0;
}

static int start_maximfast(const struct device *dev)
{
	const struct max32664a_config *config = dev->config;
	struct max32664a_data *data = dev->data;
	int ret;

	ret = set_output_mode(dev, MAX32664A_OUT_SENSOR_ALGO);
	if (ret != 0) {
		return ret;
	}

	ret = set_fifo_threshold(dev, config->fifo_threshold);
	if (ret != 0) {
		return ret;
	}

	ret = apply_max30101_sample_rate(dev, data->sample_rate_hz);
	if (ret != 0) {
		LOG_ERR("Failed to set sample rate %u Hz", data->sample_rate_hz);
		return ret;
	}

	ret = afe_enable(dev, true);
	if (ret != 0) {
		return ret;
	}

	if (!config->skip_accelerometer) {
		ret = accel_enable(dev, true);
		if (ret != 0) {
			return ret;
		}
	}

	if (config->use_agc) {
		ret = agc_enable(dev, true);
		if (ret != 0) {
			return ret;
		}
	}

	ret = maximfast_enable(dev, true);
	if (ret != 0) {
		return ret;
	}

	data->algo_mode = MAX32664A_ALGO_MAXIMFAST;
	return start_worker(dev);
}

static int stop_maximfast(const struct device *dev)
{
	const struct max32664a_config *config = dev->config;
	struct max32664a_data *data = dev->data;
	int ret;

	ret = maximfast_enable(dev, false);
	if (ret != 0) {
		return ret;
	}

	ret = afe_enable(dev, false);
	if (ret != 0) {
		return ret;
	}

	if (!config->skip_accelerometer) {
		ret = accel_enable(dev, false);
		if (ret != 0) {
			return ret;
		}
	}

	ret = set_output_mode(dev, 0x00);
	if (ret != 0) {
		return ret;
	}

	stop_worker(dev);
	data->algo_mode = MAX32664A_ALGO_OFF;
	k_msgq_purge(&data->sample_queue);

	return 0;
}

static int max32664a_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct max32664a_data *data = dev->data;

	ARG_UNUSED(chan);

	if (data->algo_mode == MAX32664A_ALGO_OFF) {
		return -EAGAIN;
	}

	if (k_msgq_get(&data->sample_queue, &data->sample, K_NO_WAIT) != 0) {
		return -EAGAIN;
	}

	return 0;
}

static int max32664a_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	const struct max32664a_data *data = dev->data;

	switch ((int)chan) {
	case SENSOR_CHAN_IR:
		val->val1 = data->sample.ir;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_RED:
		val->val1 = data->sample.red;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_ACCEL_X:
		val->val1 = data->sample.accel_x;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		val->val1 = data->sample.accel_y;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		val->val1 = data->sample.accel_z;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_MAX32664A_HEARTRATE:
		val->val1 = data->sample.hr_tenths / 10;
		val->val2 = data->sample.hr_confidence;
		break;
	case SENSOR_CHAN_MAX32664A_SPO2:
		val->val1 = data->sample.spo2_tenths / 10;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int max32664a_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	struct max32664a_data *data = dev->data;

	ARG_UNUSED(chan);

	switch ((int)attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (data->algo_mode != MAX32664A_ALGO_OFF) {
			return -EBUSY;
		}
		if (val->val1 <= 0 || val->val2 != 0) {
			return -EINVAL;
		}
		{
			uint8_t sr;

			if (sample_rate_to_spo2_sr(val->val1, &sr) != 0) {
				return -EINVAL;
			}
		}
		data->sample_rate_hz = val->val1;
		return 0;
	case SENSOR_ATTR_MAX32664A_ALGO_MODE:
		break;
	default:
		return -ENOTSUP;
	}

	switch (val->val1) {
	case MAX32664A_ALGO_OFF:
		return stop_maximfast(dev);
	case MAX32664A_ALGO_MAXIMFAST:
		return start_maximfast(dev);
	default:
		return -EINVAL;
	}
}

static int max32664a_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	const struct max32664a_data *data = dev->data;

	ARG_UNUSED(chan);

	switch ((int)attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		val->val1 = data->sample_rate_hz;
		val->val2 = 0;
		return 0;
	case SENSOR_ATTR_MAX32664A_ALGO_MODE:
		val->val1 = data->algo_mode;
		val->val2 = 0;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, max32664a_api) = {
	.sample_fetch = max32664a_sample_fetch,
	.channel_get = max32664a_channel_get,
	.attr_set = max32664a_attr_set,
	.attr_get = max32664a_attr_get,
};

static int max32664a_init(const struct device *dev)
{
	const struct max32664a_config *config = dev->config;
	struct max32664a_data *data = dev->data;
	uint8_t tx[2] = { 0xFF, 0x03 };
	uint8_t rx[4];
	int ret;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->reset_gpio) || !gpio_is_ready_dt(&config->mfio_gpio)) {
		LOG_ERR("GPIO not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&config->mfio_gpio, GPIO_INPUT);

	enter_application_mode(dev);

	ret = verify_application_mode(dev);
	if (ret != 0) {
		return ret;
	}

	ret = max32664a_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx),
				     MAX32664A_CMD_DELAY_MS);
	if (ret == 0) {
		LOG_INF("Hub firmware %u.%u.%u", rx[1], rx[2], rx[3]);
	}

	ret = check_max30101(dev);
	if (ret != 0) {
		return ret;
	}

	k_msgq_init(&data->sample_queue, data->sample_queue_buf, sizeof(struct max32664a_sample),
		    CONFIG_MAX32664A_QUEUE_SIZE);

	data->algo_mode = MAX32664A_ALGO_OFF;
	data->sample_rate_hz = config->sample_rate_hz;
	data->worker_running = false;
	data->thread_id = NULL;

	return 0;
}

#define MAX32664A_DEFINE(inst)                                                                     \
	static struct max32664a_data max32664a_data_##inst;                                        \
                                                                                                   \
	static const struct max32664a_config max32664a_config_##inst = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                            \
		.mfio_gpio = GPIO_DT_SPEC_INST_GET(inst, mfio_gpios),                              \
		.fifo_threshold = DT_INST_PROP(inst, fifo_threshold),                              \
		.use_agc = DT_INST_PROP(inst, use_agc),                                            \
		.skip_accelerometer = DT_INST_PROP(inst, skip_accelerometer),                      \
		.sample_rate_hz = DT_INST_PROP(inst, sample_rate_hz),                              \
		.spo2_pw = DT_INST_ENUM_IDX(inst, led_pulse_width_us),                             \
		.spo2_adc_rge = DT_INST_ENUM_IDX(inst, adc_range_na),                              \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, max32664a_init, NULL, &max32664a_data_##inst,           \
				     &max32664a_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &max32664a_api);

DT_INST_FOREACH_STATUS_OKAY(MAX32664A_DEFINE)
