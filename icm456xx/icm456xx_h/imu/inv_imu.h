/*
 * Copyright (c) 2018 TDK Invensense
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

/** @file inv_imu.h */

/* Device description ICM45605 */
#define INV_ICM45605_STRING_ID         "ICM45605"
#define INV_ICM45605_WHOAMI            0xE5

/* Device description ICM45605S */
#define INV_ICM45605S_STRING_ID        "ICM45605S"
#define INV_ICM45605S_WHOAMI           0xE5

/* Device description ICM45686 */
#define INV_ICM45686_STRING_ID         "ICM45686"
#define INV_ICM45686_WHOAMI            0xE9

/* Device description ICM45686S */
#define INV_ICM45686S_STRING_ID        "ICM45686S"
#define INV_ICM45686S_WHOAMI           0xEE

#if CONFIG_USE_EMD_ICM45686
  #define INV_IMU_HIGH_FSR_SUPPORTED   1
#else /* ICM45605 */
  #define INV_IMU_HIGH_FSR_SUPPORTED   0
#endif

#define INV_IMU_FSYNC_SUPPORTED      1
#define INV_IMU_USE_BASIC_SMD        0
#define INV_IMU_INT2_PIN_SUPPORTED   1
#define INV_IMU_I2C_MASTER_SUPPORTED 1
#define INV_IMU_CLKIN_SUPPORTED      0

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
