/*
 * Copyright (c) 2017 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup EDMP EDMP
 *  @brief API to drive eDMP features.
 *  @{
 */

/** @file inv_imu_edmp.h */

#ifndef _INV_IMU_EDMP_LOWG_H_
#define _INV_IMU_EDMP_LOWG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_defs.h"

#include <stdint.h>
#include <string.h>

/** Registers to retrieve interrupts status for APEX. */
typedef struct {
	int_apex_status0_t int_apex_status0;
} int_apex_statusx_t;

/** Registers to configure interrupts for APEX. */
typedef struct {
	int_apex_config0_t int_apex_config0;
} int_apex_configx_t;

/** @brief APEX interrupts definition */
typedef struct {
	uint8_t INV_LOWG;
} inv_imu_edmp_int_state_t;

/** @brief IMU APEX inputs parameters definition
 *  @note Refer to the datasheet for details on how to configure these parameters.
 */
typedef struct {
	/* Freefall */
	uint16_t lowg_peak_th;
	uint16_t lowg_peak_th_hyst;
	uint16_t lowg_time_th;

	/* Power save */
	uint32_t power_save_time;
	uint8_t power_save_en;

} inv_imu_edmp_apex_parameters_t;

/** @brief Returns current EDMP parameters for APEX algorithms.
 *  @param[in] s   Pointer to device.
 *  @param[out] p  The current parameters read from registers.
 *  @return        0 on success, negative value on error.
 */
int icm566xx_edmp_get_lowg_parameters(inv_imu_device_t *s, inv_imu_edmp_apex_parameters_t *p);

/** @brief Configures EDMP parameters for APEX algorithms.
 *  @warning This function should be called only when all EDMP algorithms are disabled.
 *  @param[in] s  Pointer to device.
 *  @param[in] p  The requested input parameters.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_lowg_parameters(inv_imu_device_t *s, const inv_imu_edmp_apex_parameters_t *p);

/** @brief Retrieve interrupts configuration.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Configuration of each APEX interrupt.
 *  @return         0 on success, negative value on error.
 */
int icm566xx_edmp_get_config_int_lowg(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it);

/** @brief Configure APEX interrupt.
 *  @param[in] s   Pointer to device.
 *  @param[in] it  State of each APEX interrupt to configure.
 *  @return        0 on success, negative value on error.
 */
int icm566xx_edmp_set_config_int_lowg(inv_imu_device_t *s, const inv_imu_edmp_int_state_t *it);

/** @brief  Enable APEX algorithm Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_lowg(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_lowg(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm Freefall.
 *  @param[in] s  Pointer to device.
 *  @param[in] ff_odr  New value for freefall ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_lowg_odr(inv_imu_device_t *s, uint16_t ff_odr);

/** @brief Read APEX interrupt status.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Status of each APEX interrupt.
 *  @return         0 on success, negative value on error.
 */
int icm566xx_edmp_get_int_lowg_status(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_EDMP_LOWG_H_ */

/** @} */
