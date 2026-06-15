/*
 *
 * Copyright (c) [2017] by InvenSense, Inc.
 * * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/** @defgroup Driver routines for reading temperature compensated data.
 *  @brief Routines for reading temperature compensated data
 *  @{
 */

/** @file inv_imu_driver_tc.h */

#ifndef _INV_IMU_DRIVER_TC_H_
#define _INV_IMU_DRIVER_TC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_defs.h"

#define INVALID_SENSOR_DATA ((int16_t)0x8000)

/** Float Sensor data from registers */
typedef struct {
	float accel_data[3];
	float gyro_data[3];
	int16_t temp_data;
} inv_imu_float_sensor_data_t;

typedef struct gyr_tc {
	float a_lfs;
	float a_hfs;
	float b_lfs;
	float b_hfs;
} gyr_tc_t;

typedef struct acc_tc {
	float a_lfs;
	float a_hfs;
	float a_hfs32;
	float b_lfs;
	float b_hfs;
	float b_hfs32;
} acc_tc_t;

typedef struct sensor_tc {
	float lfs;
	float hfs;
} sensor_tc_t;

#define IPREG_SYS1_REG_151 0xa497
typedef struct {
	uint8_t gyro_z_afsr_pulse_qual: 2;
	uint8_t gyro_y_afsr_pulse_qual: 2;
	uint8_t gyro_x_afsr_pulse_qual: 2;
	uint8_t gyro_temp_odr: 2;
} ipreg_sys1_reg_151_internal_t;

/*** Required for conversion to single slope */
#define IPREG_SYS2_REG_105 0xa569
typedef struct {
	uint8_t accel_x_tmid_off: 4;
	uint8_t accel_x_tmid_gain: 4;
} ipreg_sys2_reg_105_internal_t;

#define IPREG_SYS2_REG_106 0xa56a
typedef struct {
	uint8_t accel_y_tmid_off: 4;
	uint8_t accel_y_tmid_gain: 4;
} ipreg_sys2_reg_106_internal_t;

#define IPREG_SYS2_REG_107 0xa56b
typedef struct {
	uint8_t accel_z_tmid_off: 4;
	uint8_t accel_z_tmid_gain: 4;
} ipreg_sys2_reg_107_internal_t;
/******************************************/

#define IPREG_SYS2_REG_108 0xa56c
typedef struct {
	uint8_t resv_1: 1;
	uint8_t accel_dec1_freeze_trim: 3;
	uint8_t accel_32gee_fs: 1;
	uint8_t accel_temp_odr: 3;
} ipreg_sys2_reg_108_internal_t;

/** @brief Disable HW temperature compensation.
 *  @param[in] s      Pointer to device.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_disable_hw_tc(inv_imu_device_t *s);

/** @brief Initialize temperature compensation.
 *  @param[in] s      Pointer to device.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_init_tc(inv_imu_device_t *s);

/** @brief Configure accel mode and update TC configuration.
 *  @param[in] s           Pointer to transport structure.
 *  @param[in] accel_mode  The requested mode.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_set_tc_accel_mode(inv_imu_device_t *s, pwr_mgmt0_accel_mode_t accel_mode);

/** @brief Configure gyro mode and update TC configuration.
 *  @param[in] s          Pointer to transport structure.
 *  @param[in] gyro_mode  The requested mode.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_tc_gyro_mode(inv_imu_device_t *s, pwr_mgmt0_gyro_mode_t gyro_mode);

/** @brief Configure accel Output Data Rate and update TC configuration.
 *  @param[in] s          Pointer to device.
 *  @param[in] frequency  The requested frequency.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_tc_accel_frequency(inv_imu_device_t *s, const accel_config0_accel_odr_t frequency);

/** @brief Configure gyro Output Data Rate and update TC configuration.
 *  @param[in] s          Pointer to device.
 *  @param[in] frequency  The requested frequency.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_tc_gyro_frequency(inv_imu_device_t *s, const gyro_config0_gyro_odr_t frequency);

/** @brief Get current temperature compensated sensor data from the registers.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Current accel, gyro and temperature data from the registers.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_get_tc_register_data(inv_imu_device_t *s, inv_imu_float_sensor_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_TC_H_ */

/** @} */
