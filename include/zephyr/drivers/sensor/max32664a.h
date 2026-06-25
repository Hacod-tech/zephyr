/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended sensor API for the MAX32664A biometric sensor hub (MaximFast + MAX30101).
 * @ingroup max32664a_interface
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664A_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664A_H_

/**
 * @defgroup max32664a_interface MAX32664A
 * @ingroup sensor_interface_ext
 * @brief Maxim Integrated MAX32664A finger HR/SpO2 sensor hub with MAX30101 AFE
 * @{
 */

#include <zephyr/device.h>

enum sensor_channel_max32664a {
	/** Heart rate in bpm (val1), confidence 0-100 (val2) */
	SENSOR_CHAN_MAX32664A_HEARTRATE = SENSOR_CHAN_PRIV_START,
	/** SpO2 in percent (val1), confidence 0-100 (val2) */
	SENSOR_CHAN_MAX32664A_SPO2,
};

enum sensor_attribute_max32664a {
	/** Enable/disable MaximFast algorithm (see max32664a_algo_mode) */
	SENSOR_ATTR_MAX32664A_ALGO_MODE = SENSOR_ATTR_PRIV_START,
};

/**
 * MAX30101 sample rate (Hz) is configured with the standard
 * @ref SENSOR_ATTR_SAMPLING_FREQUENCY attribute (50–3200 Hz).
 * Set it before starting MaximFast; returns -EBUSY while the algorithm runs.
 */

enum max32664a_algo_mode {
	MAX32664A_ALGO_OFF,
	/** MaximFast mode 1 report (UG6806 Table 8) */
	MAX32664A_ALGO_MAXIMFAST,
};

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664A_H_ */
