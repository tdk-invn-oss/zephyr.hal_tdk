/*
 * Copyright (c) 2017 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup Driver IMU driver high level functions
 *  @brief High-level function to setup an IMU device
 *  @ingroup  Driver
 *  @{
 */

/** @file inv_imu_driver_internal.h
 * High-level function to setup an IMU device
 */

#ifndef _INV_IMU_DRIVER_INTERNAL_H_
#define _INV_IMU_DRIVER_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"

#include "imu/inv_imu_defs_internal.h"

#include <stdint.h>
#include <string.h>

typedef struct {
	uint8_t gyro_ui_filt_ord_ind;
	uint8_t accel_ui_filt_ord_ind;
} inv_imu_chip_config_t;

typedef struct {
	uint16_t die_id;
	uint8_t lot_id[6];
	uint8_t wafer_id;
	uint8_t die_rev;
} inv_imu_manufacturing_info_t;

/** Sensor data from registers */
typedef union {
	struct {
		int16_t accel_data[3];
		int16_t gyro_data[3];
		int16_t temp_data;
	} sixteen_bits;
	struct {
		int32_t accel_data[3];
		int32_t gyro_data[3];
		int16_t temp_data;
	} twenty_bits;
} inv_imu_sensor_data_internal_t;

int icm566xx_read_chip_config(inv_imu_device_t *s, inv_imu_chip_config_t *cfg);

/** @brief Read the manufacturing info and fill the 'info' structure.
 *  @param[in] s      Pointer to device.
 *  @param[out] info  Manufacturing info structure.
 *  @return           0 on success, negative value on error.
 *  @warning This function will power-up the SRAM. For power consumption consideration, * you can
 * manually call `icm566xx_adv_power_down_sram` if you don't need to *           preserve SRAM
 * content.
 */
int inv_read_manufacturing_info(inv_imu_device_t *s, inv_imu_manufacturing_info_t *info);

/** @brief  Read onboard memory content and check it against memory pointed by input parameter
 *  @param[in] s          Pointer to device.
 *  @param[in] data       Expected data.
 *  @param[in] sram_addr  Address of the SRAM to read.
 *  @param[in] size       Size to compare.
 *  @return               0 on success, negative value on error.
 */
int icm566xx_check_memory(inv_imu_device_t *s, const uint8_t *data, uint32_t sram_addr,
			  uint32_t size);

/** @brief Configure default pad scenario
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_adv_set_default_pad_scenario(inv_imu_device_t *s);

/** @brief Make sure the eDMP image is the correct one.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_check_eDMP_image(void *s);

uint8_t inv_imu_read_otp(void *icm_driver, uint16_t address);
int icm566xx_write_otp(void *icm_driver, uint16_t address, uint8_t data, uint8_t soak);

/** @brief Read the UI sensor data length and set the inv_device data length field
 *  @param[in] s  Pointer to device. *  @param[in] sensor_data_length  Pointer to uint8_t for
 * holding sensor data length value. *  @return       0 on success, negative value on error.
 */
int icm566xx_get_sensor_data_length(inv_imu_device_t *s,
				    inv_imu_sensor_reg_len_t *sensor_data_length);

/** @brief Set the length (16 or 20-bits) for the UI sensor data
 *  @param[in] s           Pointer to device. *  @param[in] data_length Requested data length. *
 * @return                0 on success, negative value on error.
 */
int icm566xx_set_sensor_data_length(inv_imu_device_t *s, inv_imu_sensor_reg_len_t data_length);

/** @brief Get current sensor data from the registers.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Current accel, gyro and temperature data from the registers.
 *  @return           0 on success, negative value on error.
 */
int icm566xx_get_register_data_internal(inv_imu_device_t *s, inv_imu_sensor_data_internal_t *data);

/** @brief Read all registers containing data (temperature, accelerometer and gyroscope). * It will
 * then call `sensor_event_cb` function provided *         in the `inv_imu_device_t` for each
 * packet.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_adv_get_data_from_registers_internal(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_INTERNAL_H_ */

/** @} */
