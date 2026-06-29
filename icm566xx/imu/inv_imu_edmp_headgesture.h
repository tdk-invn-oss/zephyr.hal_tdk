/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup EDMP EDMP
 *  @brief API to drive eDMP Head-Gesture features.
 *  @{
 */

/** @file inv_imu_edmp_headgesture.h */

#ifndef _INV_IMU_EDMP_HEADGESTURE_H_
#define _INV_IMU_EDMP_HEADGESTURE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "icm566xx/imu/inv_imu_driver.h"
#include "icm566xx/imu/inv_imu_edmp.h"
#include "icm566xx/imu/inv_imu_edmp_defs.h"

#include <stdint.h>
#include <string.h>

/** @brief APEX interrupts definition */
typedef struct {
	uint8_t INV_HEAD_GESTURE;
} inv_imu_edmp_headgesture_int_state_t;

/** @brief IMU Head-Gesture outputs definition
 */
typedef enum {
	INV_HEADGESTURE_UNCONCLUSIVE = 0,
	INV_HEADGESTURE_NO,
	INV_HEADGESTURE_YES,
} inv_imu_edmp_headgesture_outputs_t;

/** @brief IMU Head-Gesture inputs parameters definition
 */
typedef struct {
	int16_t mounting_matrix[9]; /* A 3x3 mounting-matrix applied on gyroscope data */
} inv_imu_edmp_headgesture_parameters_t;

/** @brief Initialize EDMP to load/execute Head-Gesture
 *  @param[in] s    Pointer to device.
 *  @return         0 on success, negative value on error.
 */
int icm566xx_edmp_headgesture_init(inv_imu_device_t *s);

/** @brief Returns current EDMP parameters for Head-Gesture algorithms.
 *  @param[in] s   Pointer to device.
 *  @param[out] p  The current parameters read from registers.
 *  @return        0 on success, negative value on error.
 */
int icm566xx_edmp_headgesture_get_parameters(inv_imu_device_t *s,
					     inv_imu_edmp_headgesture_parameters_t *p);

/** @brief Configures EDMP parameters for Head-Gesture algorithms.
 *  @warning This function should be called only when all EDMP algorithms are disabled.
 *  @param[in] s  Pointer to device.
 *  @param[in] p  The requested input parameters.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_headgesture_set_parameters(inv_imu_device_t *s,
					     const inv_imu_edmp_headgesture_parameters_t *p);

/** @brief  Enable Head-Gesture algorithm.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_headgesture_enable(inv_imu_device_t *s);

/** @brief  Disable Head-Gesture algorithm.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_headgesture_disable(inv_imu_device_t *s);

/** @brief Enable or disable interrupt head gesture source.
 *  @param[in] s    Pointer to device.
 *  @param[in] it   State of each interrupt
 *  @return         0 on success, negative value on error.
 */
int icm566xx_edmp_headgesture_set_config_int(inv_imu_device_t *s,
					     inv_imu_edmp_headgesture_int_state_t *it);

/** @brief Read head gesture interrupt status.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Status of each interrupt.
 *  @return         0 on success, negative value on error.
 */
int icm566xx_edmp_headgesture_get_int_status(inv_imu_device_t *s,
					     inv_imu_edmp_headgesture_int_state_t *it);

/** @brief  Retrieve Head-Gesture output prediction.
 *  @param[in]  s                  Pointer to device.
 *  @param[out] headgesture_output Prediction reported by the SRAM image.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_headgesture_get_output(inv_imu_device_t *s,
					 inv_imu_edmp_headgesture_outputs_t *headgesture_output);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_EDMP_LOWG_H_ */

/** @} */
