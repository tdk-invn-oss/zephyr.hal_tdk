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

/* Device description ICM42670P */
#define INV_ICM42670P_STRING_ID "ICM42670P"
#define INV_ICM42670P_WHOAMI    0x67

/* Device description ICM42670S */
#define INV_ICM42670S_STRING_ID "ICM42670S"
#define INV_ICM42670S_WHOAMI    0x69

/* Device description ICM42370P */
#define INV_ICM42370P_STRING_ID "ICM42370P"
#define INV_ICM42370P_WHOAMI    0x0D

#define INV_IMU_REV            INV_IMU_REV_A
#define INV_IMU_HFSR_SUPPORTED 0

#if CONFIG_USE_EMD_ICM42670
#define INV_IMU_IS_GYRO_SUPPORTED 1
#else /* USE_EMD_ICM42370 */
#define INV_IMU_IS_GYRO_SUPPORTED 0
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
