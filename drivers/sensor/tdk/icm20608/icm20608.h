/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20608_ICM20608_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20608_ICM20608_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#define ICM20608_REG_CHIP_ID 0x75
#define ICM20608_CHIP_ID     0xAE

#define ICM20608_REG_GYRO_CFG  0x1B
#define ICM20608_GYRO_FS_SHIFT 3

#define ICM20608_REG_ACCEL_CFG  0x1C
#define ICM20608_ACCEL_FS_SHIFT 3

#define ICM20608_REG_INT_EN 0x38
#define ICM20608_DRDY_EN    BIT(0)

#define ICM20608_REG_DATA_START 0x3B

#define ICM20608_REG_PWR_MGMT1 0x6B
#define ICM20608_SLEEP_EN      BIT(6)

/* measured in degrees/sec x10 to avoid floating point */
static const uint16_t icm20608_gyro_sensitivity_x10[] = {1310, 655, 328, 164};

struct icm20608_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_sensitivity_shift;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_sensitivity_x10;

#ifdef CONFIG_ICM20608_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;

	const struct sensor_trigger *data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_ICM20608_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ICM20608_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_ICM20608_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_ICM20608_TRIGGER */
};

struct icm20608_config {
	struct i2c_dt_spec i2c;
#ifdef CONFIG_ICM20608_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif /* CONFIG_ICM20608_TRIGGER */
};

#ifdef CONFIG_ICM20608_TRIGGER
int icm20608_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);

int icm20608_init_interrupt(const struct device *dev);
#endif

#endif /* __SENSOR_ICM20608__ */
