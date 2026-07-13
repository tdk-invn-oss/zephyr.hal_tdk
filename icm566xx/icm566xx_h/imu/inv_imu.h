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

/** @file inv_imu.h */

#if CONFIG_USE_EMD_ICM56622
/* Device description ICM56622 */
#define SENSOR_DATA_TYPE   int16_t
#define FORMAT_SENSOR_DATA FORMAT_16BIT_REG_DATA
#define INV_IMU_WHOAMI     0xD3
#elif CONFIG_USE_EMD_ICM56686
/* Device description ICM56686 */
#define SENSOR_DATA_TYPE   int32_t
#define FORMAT_SENSOR_DATA FORMAT_20BIT_REG_DATA
#define INV_IMU_WHOAMI     0x08
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
