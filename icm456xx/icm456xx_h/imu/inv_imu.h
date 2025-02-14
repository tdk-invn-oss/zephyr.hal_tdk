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

/* Device description ICM45687 */
#define INV_ICM45687_STRING_ID         "ICM45687"
#define INV_ICM45687_WHOAMI            0x85

#define INV_IMU_REV                     INV_IMU_REV_A
#define INV_IMU_HFSR_SUPPORTED          0

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
