/*
 * Copyright (c) 2026 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#ifndef _INV_IMU_H_
#define _INV_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup IMU IMU
 *  @brief Describes IMU
 *  @{
 */

#include <zephyr/devicetree.h>

/** @file inv_imu.h */

#if DT_HAS_COMPAT_STATUS_OKAY(invensense_icm56622)
/* Device description ICM56622 */
/* Device ID */
#define ICM56622

/* Device description */
#define INV_IMU_STRING_ID          "ICM56622"
#define INV_IMU_WHOAMI             0xd3
#define INV_IMU_HIGH_FSR_SUPPORTED 0
#define INV_IMU_FSYNC_SUPPORTED    1
#define INV_IMU_INT2_PIN_SUPPORTED 1
#define INV_IMU_CLKIN_SUPPORTED    1
#define INV_IMU_AUX1_SUPPORTED     1
#define INV_IMU_LIMITED_APEX       1

#define FORMAT_SENSOR_DATA FORMAT_16BIT_REG_DATA
#define INVALID_REG_VALUE  INVALID_VALUE_FIFO
#define ACCEL_FSR_SETTING  ACCEL_CONFIG0_AP_ACCEL_FS_SEL_16_G
#define GYRO_FSR_SETTING   GYRO_CONFIG0_AP_GYRO_FS_SEL_2000_DPS
#define REG_SAT_VALUE      0x8000
#define ACCEL_FS_VALUE     16
#define GYRO_FS_VALUE      2000

typedef int16_t sensor_data_t;

#elif DT_HAS_COMPAT_STATUS_OKAY(invensense_icm56686)
/* Device description ICM56686 */
/* Device ID */
#define ICM56686

/* Device description */
#define INV_IMU_STRING_ID          "ICM56686"
#define INV_IMU_WHOAMI             0x08
#define INV_IMU_HIGH_FSR_SUPPORTED 1
#define INV_IMU_FSYNC_SUPPORTED    1
#define INV_IMU_INT2_PIN_SUPPORTED 1
#define INV_IMU_CLKIN_SUPPORTED    1
#define INV_IMU_AUX1_SUPPORTED     1
#define INV_IMU_LIMITED_APEX       0

#define FORMAT_SENSOR_DATA FORMAT_20BIT_REG_DATA
#define INVALID_REG_VALUE  INVALID_VALUE_FIFO_20
#define ACCEL_FSR_SETTING  ACCEL_CONFIG0_AP_ACCEL_FS_SEL_32_G
#define GYRO_FSR_SETTING   GYRO_CONFIG0_AP_GYRO_FS_SEL_4000_DPS
#define REG_SAT_VALUE      0x80000
#define ACCEL_FS_VALUE     32
#define GYRO_FS_VALUE      4000
typedef int32_t sensor_data_t;
#else
#error "No compatible Invensense IMU device enabled in devicetree"
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
