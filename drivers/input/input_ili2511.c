/*
 * Copyright (c) 2026 Hacod
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Protocol based on the Linux ili210x/ili251x driver (Ilitek I2C protocol v3.x).
 */

#define DT_DRV_COMPAT ilitek_ili2511

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ili2511, CONFIG_INPUT_LOG_LEVEL);

#define ILI2511_REG_TOUCHDATA  0x10
#define ILI2511_REG_PANEL_INFO 0x20
#define ILI2511_DATA_SIZE1     31
#define ILI2511_DATA_SIZE2     20
#define ILI2511_DATA_SIZE      (ILI2511_DATA_SIZE1 + ILI2511_DATA_SIZE2)
#define ILI2511_MAX_TOUCHES    10
#define ILI2511_POLL_PERIOD_MS 20
#define ILI2511_RESET_PULSE_MS 10
#define ILI2511_RESET_WAIT_MS  100
#define ILI2511_DEFAULT_MAX_XY 16384U

struct ili2511_config {
	struct input_touchscreen_common_config common;
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec irq;
	struct gpio_dt_spec rst;
};

struct ili2511_data {
	const struct device *dev;
	struct gpio_callback gpio_cb;
	struct k_work_delayable work;
	uint16_t max_x;
	uint16_t max_y;
	uint16_t last_x;
	uint16_t last_y;
	bool pressed;
};

INPUT_TOUCH_STRUCT_CHECK(struct ili2511_config);

static int ili2511_read_reg(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *buf, size_t len,
			    k_timeout_t delay)
{
	int ret;

	ret = i2c_write_dt(i2c, &reg, 1);
	if (ret < 0) {
		return ret;
	}

	if (!K_TIMEOUT_EQ(delay, K_NO_WAIT)) {
		k_sleep(delay);
	}

	return i2c_read_dt(i2c, buf, len);
}

static int ili2511_read_touch_data(const struct i2c_dt_spec *i2c, uint8_t *data)
{
	int ret;

	ret = ili2511_read_reg(i2c, ILI2511_REG_TOUCHDATA, data, ILI2511_DATA_SIZE1, K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}

	/* data[0] == 2 means a second packet with more finger slots follows */
	if (data[0] == 2) {
		ret = i2c_read_dt(i2c, data + ILI2511_DATA_SIZE1, ILI2511_DATA_SIZE2);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static bool ili2511_finger_coords(const uint8_t *touchdata, unsigned int finger, uint16_t *x,
				  uint16_t *y)
{
	uint16_t val;
	size_t off = 1U + (finger * 5U);

	if (off + 4U >= ILI2511_DATA_SIZE) {
		return false;
	}

	val = sys_get_be16(&touchdata[off]);
	if ((val & BIT(15)) == 0U) {
		return false;
	}

	*x = val & 0x3fffU;
	*y = sys_get_be16(&touchdata[off + 2U]);
	return true;
}

static uint16_t ili2511_scale(uint16_t raw, uint16_t max_raw, uint16_t screen)
{
	if (screen == 0U || max_raw == 0U) {
		return raw;
	}

	if (raw >= max_raw) {
		return screen > 0U ? screen - 1U : 0U;
	}

	return (uint16_t)(((uint32_t)raw * screen) / max_raw);
}

static void ili2511_report(const struct device *dev, const uint8_t *touchdata)
{
	const struct ili2511_config *cfg = dev->config;
	struct ili2511_data *data = dev->data;
	uint16_t raw_x = 0;
	uint16_t raw_y = 0;
	uint16_t x;
	uint16_t y;
	bool pressed = false;

	/* LVGL pointer path uses a single contact; report first active finger. */
	for (unsigned int i = 0; i < ILI2511_MAX_TOUCHES; i++) {
		if (ili2511_finger_coords(touchdata, i, &raw_x, &raw_y)) {
			pressed = true;
			break;
		}
	}

	if (pressed) {
		x = ili2511_scale(raw_x, data->max_x, cfg->common.screen_width);
		y = ili2511_scale(raw_y, data->max_y, cfg->common.screen_height);

		/* Skip duplicate pressed samples to avoid flooding the input queue. */
		if (data->pressed && x == data->last_x && y == data->last_y) {
			return;
		}

		data->last_x = x;
		data->last_y = y;
		input_touchscreen_report_pos(dev, x, y, K_NO_WAIT);
		input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_NO_WAIT);
		LOG_DBG("press raw=%u,%u -> %u,%u", raw_x, raw_y, x, y);
	} else if (data->pressed) {
		input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_NO_WAIT);
		LOG_DBG("release");
	}

	data->pressed = pressed;
}

static void ili2511_work_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	struct ili2511_data *data = CONTAINER_OF(dwork, struct ili2511_data, work);
	const struct device *dev = data->dev;
	const struct ili2511_config *cfg = dev->config;
	uint8_t touchdata[ILI2511_DATA_SIZE] = {0};
	int ret;

	ret = ili2511_read_touch_data(&cfg->i2c, touchdata);
	if (ret < 0) {
		LOG_ERR("touch data read failed: %d", ret);
		return;
	}

	ili2511_report(dev, touchdata);

	/* Keep sampling while a finger is down (matches Linux ili251x behaviour). */
	if (data->pressed) {
		(void)k_work_reschedule(&data->work, K_MSEC(ILI2511_POLL_PERIOD_MS));
	}
}

static void ili2511_irq_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct ili2511_data *data = CONTAINER_OF(cb, struct ili2511_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	(void)k_work_reschedule(&data->work, K_NO_WAIT);
}

static void ili2511_read_resolution(const struct device *dev)
{
	const struct ili2511_config *cfg = dev->config;
	struct ili2511_data *data = dev->data;
	uint8_t rs[10];
	uint16_t resx;
	uint16_t resy;
	int ret;

	data->max_x = cfg->common.screen_width;
	data->max_y = cfg->common.screen_height;

	ret = ili2511_read_reg(&cfg->i2c, ILI2511_REG_PANEL_INFO, rs, sizeof(rs), K_MSEC(5));
	if (ret < 0) {
		LOG_WRN("panel info read failed (%d), assuming 1:1 mapping", ret);
		return;
	}

	resx = sys_get_le16(&rs[0]);
	resy = sys_get_le16(&rs[2]);

	if (resx == 0U || resx == 0xffffU || resy == 0U || resy == 0xffffU) {
		LOG_WRN("invalid panel resolution %u x %u, using default %u", resx, resy,
			ILI2511_DEFAULT_MAX_XY);
		data->max_x = ILI2511_DEFAULT_MAX_XY;
		data->max_y = ILI2511_DEFAULT_MAX_XY;
		return;
	}

	data->max_x = resx;
	data->max_y = resy;
	LOG_INF("panel touch range %u x %u -> screen %u x %u", data->max_x, data->max_y,
		cfg->common.screen_width, cfg->common.screen_height);
}

static int ili2511_init(const struct device *dev)
{
	const struct ili2511_config *cfg = dev->config;
	struct ili2511_data *data = dev->data;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->irq) || !gpio_is_ready_dt(&cfg->rst)) {
		LOG_ERR("IRQ/RST GPIO not ready");
		return -ENODEV;
	}

	if (cfg->common.screen_width == 0U || cfg->common.screen_height == 0U) {
		LOG_ERR("screen-width/height must be set for coordinate scaling");
		return -EINVAL;
	}

	data->dev = dev;
	data->pressed = false;
	data->last_x = UINT16_MAX;
	data->last_y = UINT16_MAX;
	k_work_init_delayable(&data->work, ili2511_work_handler);

	ret = gpio_pin_configure_dt(&cfg->rst, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("RST configure failed: %d", ret);
		return ret;
	}

	k_msleep(ILI2511_RESET_PULSE_MS);
	ret = gpio_pin_set_dt(&cfg->rst, 0);
	if (ret < 0) {
		return ret;
	}
	k_msleep(ILI2511_RESET_WAIT_MS);

	ili2511_read_resolution(dev);

	ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("IRQ configure failed: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, ili2511_irq_handler, BIT(cfg->irq.pin));
	ret = gpio_add_callback(cfg->irq.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("IRQ callback failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_FALLING);
	if (ret < 0) {
		LOG_ERR("IRQ interrupt configure failed: %d", ret);
		return ret;
	}

	LOG_INF("ILI2511 ready on %s addr 0x%02x", cfg->i2c.bus->name, cfg->i2c.addr);
	return 0;
}

#define ILI2511_INIT(inst)                                                                         \
	static const struct ili2511_config ili2511_cfg_##inst = {                                  \
		.common = INPUT_TOUCH_DT_INST_COMMON_CONFIG_INIT(inst),                            \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                                     \
		.rst = GPIO_DT_SPEC_INST_GET(inst, rst_gpios),                                     \
	};                                                                                         \
	static struct ili2511_data ili2511_data_##inst;                                            \
	DEVICE_DT_INST_DEFINE(inst, ili2511_init, NULL, &ili2511_data_##inst,                      \
			      &ili2511_cfg_##inst, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ILI2511_INIT)
