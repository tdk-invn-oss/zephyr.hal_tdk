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

/* Bump specific memory map */
#define EDMP_RAM_PRGM_BASE 0x5AC
#define EDMP_RAM_PRGM_SIZE 0x20C
#define EDMP_RAM_DATA_BASE 0x7B8
#define EDMP_RAM_DATA_SIZE 0x48

#define EDMP_bump_decim_rate        0x7B8
#define EDMP_bump_decim_rate_SIZE   0x1
#define EDMP_bump_decim_count       0x7B9
#define EDMP_bump_decim_count_SIZE  0x1
#define EDMP_bump_outInterrupt      0x7BA
#define EDMP_bump_outInterrupt_SIZE 0x1

#define EDMP_bump_state               0x7BC
#define EDMP_bump_state_SIZE          0x30
#define EDMP_bump_state_state_id      0x7DA
#define EDMP_bump_state_state_id_SIZE 0x1

#define EDMP_bump_config                           0x7EC
#define EDMP_bump_config_SIZE                      0x12
#define EDMP_bump_config_odr_bump                  0x7EC
#define EDMP_bump_config_odr_bump_SIZE             0x2
#define EDMP_bump_config_change_th_ratio_Q8        0x7EE
#define EDMP_bump_config_change_th_ratio_Q8_SIZE   0x2
#define EDMP_bump_config_change_th_offset_Q12      0x7F0
#define EDMP_bump_config_change_th_offset_Q12_SIZE 0x2
#define EDMP_bump_config_bump_th_ratio_Q8          0x7F2
#define EDMP_bump_config_bump_th_ratio_Q8_SIZE     0x2
#define EDMP_bump_config_bump_th_offset_Q12        0x7F4
#define EDMP_bump_config_bump_th_offset_Q12_SIZE   0x2
#define EDMP_bump_config_conf_time                 0x7F6
#define EDMP_bump_config_conf_time_SIZE            0x2
#define EDMP_bump_config_bump_time                 0x7F8
#define EDMP_bump_config_bump_time_SIZE            0x2
#define EDMP_bump_config_debounce_time             0x7FA
#define EDMP_bump_config_debounce_time_SIZE        0x2
#define EDMP_bump_config_min_bump_nb               0x7FC
#define EDMP_bump_config_min_bump_nb_SIZE          0x2

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
