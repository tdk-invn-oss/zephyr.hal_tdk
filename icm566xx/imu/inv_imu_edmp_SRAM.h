/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#ifndef _INV_IMU_EDMP_SRAM_H_
#define _INV_IMU_EDMP_SRAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "icm566xx/imu/inv_imu_driver.h"

typedef struct {
	/* Bump */
	int8_t bump_decim_rate;
	int8_t bump_decim_count;
	int16_t bump_th_ratio_Q8;
	int16_t bump_change_th_ratio_Q8;
	int16_t bump_conf_time;
	int16_t bump_debounce_time;
	uint16_t bump_min_bump_nb;
	int16_t bump_time;
	int16_t bump_change_th_offset_Q12;
	int16_t bump_th_offset_Q12;
} inv_imu_edmp_bump_parameters_t;

/** @brief  Init SRAM for APEX algorithm Bump.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_bump_init(inv_imu_device_t *s);

/** @brief  Get current Bump configuration settings.
 *  @param[in]  s           Pointer to device.
 *  @param[out] Bump_params  Pointer to Bump configuration structure, which will hold current Bump
 * configuration.
 *  @return                 0 on success, negative value on error.
 */
int icm566xx_edmp_bump_get_parameters(inv_imu_device_t *s,
				      inv_imu_edmp_bump_parameters_t *bump_params);

/** @brief  Set new Bump configuration settings.
 *  @param[in]  s           Pointer to device.
 *  @param[in]  Bump_params  Pointer to Bump configuration structure, which contains new Bump
 * configuration.
 *  @return                 0 on success, negative value on error.
 */
int icm566xx_edmp_bump_set_parameters(inv_imu_device_t *s,
				      const inv_imu_edmp_bump_parameters_t *bump_params);

/** @brief  Enable APEX algorithm Bump.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_bump(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Bump.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_bump(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm Bump.
 *  @param[in] s  Pointer to device.
 *  @param[in] bump_odr  New value for tap ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_bump_odr(inv_imu_device_t *s, uint16_t bump_odr);

/** @brief  Initialize APEX for use with SRAM algos
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_init_apex_sram(inv_imu_device_t *s);

/** @brief  Configure eDMP for use with SRAM algos
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_configure_sram(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* SWENG_EMD_MCU_FIRMWARE_VENICE_SOURCES_DRIVERS_IMU_INV_IMU_EDMP_SRAM_H_ */
