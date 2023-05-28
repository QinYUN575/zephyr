/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#define DT_DRV_COMPAT invensense_icm20608

#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "icm20608.h"

LOG_MODULE_REGISTER(ICM20608, LOG_LEVEL_INF);

/* see "Accelerometer Measurements" section from register map description */
static void icm20608_convert_accel(struct sensor_value *val, int16_t raw_val,
				  uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
static void icm20608_convert_gyro(struct sensor_value *val, int16_t raw_val,
				 uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
static inline void icm20608_convert_temp(struct sensor_value *val,
					int16_t raw_val)
{
	val->val1 = raw_val / 340 + 36;
	val->val2 = ((int64_t)(raw_val % 340) * 1000000) / 340 + 530000;

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int icm20608_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct icm20608_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20608_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		icm20608_convert_accel(val + 1, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		icm20608_convert_accel(val + 2, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm20608_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20608_convert_accel(val, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20608_convert_accel(val, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20608_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		icm20608_convert_gyro(val + 1, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		icm20608_convert_gyro(val + 2, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm20608_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20608_convert_gyro(val, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20608_convert_gyro(val, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	default: /* chan == SENSOR_CHAN_DIE_TEMP */
		icm20608_convert_temp(val, drv_data->temp);
	}

	return 0;
}

static int icm20608_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct icm20608_data *drv_data = dev->data;
	const struct icm20608_config *cfg = dev->config;
	int16_t buf[7];

	if (i2c_burst_read_dt(&cfg->i2c, ICM20608_REG_DATA_START, (uint8_t *)buf,
			      14) < 0) {
		LOG_ERR("Failed to read data sample.");
		return -EIO;
	}

	drv_data->accel_x = sys_be16_to_cpu(buf[0]);
	drv_data->accel_y = sys_be16_to_cpu(buf[1]);
	drv_data->accel_z = sys_be16_to_cpu(buf[2]);
	drv_data->temp = sys_be16_to_cpu(buf[3]);
	drv_data->gyro_x = sys_be16_to_cpu(buf[4]);
	drv_data->gyro_y = sys_be16_to_cpu(buf[5]);
	drv_data->gyro_z = sys_be16_to_cpu(buf[6]);

	return 0;
}

static const struct sensor_driver_api icm20608_driver_api = {
#if CONFIG_ICM20608_TRIGGER
	.trigger_set = icm20608_trigger_set,
#endif
	.sample_fetch = icm20608_sample_fetch,
	.channel_get = icm20608_channel_get,
};

int icm20608_init(const struct device *dev)
{
	struct icm20608_data *drv_data = dev->data;
	const struct icm20608_config *cfg = dev->config;
	uint8_t id, i;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	/* check chip ID */
	if (i2c_reg_read_byte_dt(&cfg->i2c, ICM20608_REG_CHIP_ID, &id) < 0) {
		LOG_ERR("Failed to read chip ID.");
		return -EIO;
	}
	printk("Chip ID: 0x%x\n", id);
	LOG_INF("Chip ID: 0x%x", id);

	if (id != ICM20608_CHIP_ID) {
		LOG_ERR("Invalid chip ID.");
		return -EINVAL;
	}

	/* wake up chip */
	if (i2c_reg_update_byte_dt(&cfg->i2c, ICM20608_REG_PWR_MGMT1,
				   ICM20608_SLEEP_EN, 0) < 0) {
		LOG_ERR("Failed to wake up chip.");
		return -EIO;
	}

	/* set accelerometer full-scale range */
	for (i = 0U; i < 4; i++) {
		if (BIT(i+1) == CONFIG_ICM20608_ACCEL_FS) {
			break;
		}
	}

	if (i == 4U) {
		LOG_ERR("Invalid value for accel full-scale range.");
		return -EINVAL;
	}

	if (i2c_reg_write_byte_dt(&cfg->i2c, ICM20608_REG_ACCEL_CFG,
				  i << ICM20608_ACCEL_FS_SHIFT) < 0) {
		LOG_ERR("Failed to write accel full-scale range.");
		return -EIO;
	}

	drv_data->accel_sensitivity_shift = 14 - i;

	/* set gyroscope full-scale range */
	for (i = 0U; i < 4; i++) {
		if (BIT(i) * 250 == CONFIG_ICM20608_GYRO_FS) {
			break;
		}
	}

	if (i == 4U) {
		LOG_ERR("Invalid value for gyro full-scale range.");
		return -EINVAL;
	}

	if (i2c_reg_write_byte_dt(&cfg->i2c, ICM20608_REG_GYRO_CFG,
				  i << ICM20608_GYRO_FS_SHIFT) < 0) {
		LOG_ERR("Failed to write gyro full-scale range.");
		return -EIO;
	}

	drv_data->gyro_sensitivity_x10 = icm20608_gyro_sensitivity_x10[i];

#ifdef CONFIG_ICM20608_TRIGGER
	if (cfg->int_gpio.port) {
		if (icm20608_init_interrupt(dev) < 0) {
			LOG_DBG("Failed to initialize interrupts.");
			return -EIO;
		}
	}
#endif

	return 0;
}

#define ICM20608_DEFINE(inst)									\
	static struct icm20608_data icm20608_data_##inst;						\
												\
	static const struct icm20608_config icm20608_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		IF_ENABLED(CONFIG_ICM20608_TRIGGER,						\
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),))	\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20608_init, NULL,					\
			      &icm20608_data_##inst, &icm20608_config_##inst,			\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,				\
			      &icm20608_driver_api);						\

DT_INST_FOREACH_STATUS_OKAY(ICM20608_DEFINE)
