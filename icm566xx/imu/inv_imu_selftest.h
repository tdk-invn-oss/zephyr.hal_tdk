/*
 * Copyright (c) 2015 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup selftest Self-test
 *  @brief    API to execute self-test procedure.
 *  @{
 */

/** @file icm566xx_selftest.h */

#ifndef _INV_IMU_SELFTEST_H_
#define _INV_IMU_SELFTEST_H_

#include <stdint.h>

#include "icm566xx/imu/inv_imu_driver_advanced.h"

#ifdef __cplusplus
extern "C" {
#endif

/* STC status codes */
#define INV_IMU_ST_STATUS_SUCCESS 1  /**< Indicates test is successful */
#define INV_IMU_ST_STATUS_FAIL    -1 /**< Indicates test is failing */
#define INV_IMU_ST_STATUS_NOT_RUN 0  /**< Indicates test has not run */

/*
 * EDMP_STC_CONFIGPARAMS
 */
#define SELFTEST_INIT_EN_MASK      0x0001
#define SELFTEST_INIT_EN           0x0001
#define SELFTEST_INIT_DIS          0x0000
#define SELFTEST_ACCEL_EN_MASK     0x0002
#define SELFTEST_ACCEL_EN          0x0002
#define SELFTEST_ACCEL_DIS         0x0000
#define SELFTEST_GYRO_EN_MASK      0x0004
#define SELFTEST_GYRO_EN           0x0004
#define SELFTEST_GYRO_DIS          0x0000
#define SELFTEST_AVERAGE_TIME_MASK 0x0380
#define SELFTEST_ACCEL_THRESH_MASK 0x1C00
#define SELFTEST_GYRO_THRESH_MASK  0xE000

/*
 * EDMP_STC_RESULTS
 */
#define STC_RESULTS_ACCEL_X_MASK   0x0001
#define STC_RESULTS_ACCEL_Y_MASK   0x0002
#define STC_RESULTS_ACCEL_Z_MASK   0x0004
#define STC_RESULTS_GYRO_X_MASK    0x0008
#define STC_RESULTS_GYRO_Y_MASK    0x0010
#define STC_RESULTS_GYRO_Z_MASK    0x0020
#define STC_RESULTS_ST_STATUS_MASK 0x00C0

/* Averaging time used to perform self-test */
typedef enum {
	SELFTEST_AVG_TIME_10_MS = 0x0000,
	SELFTEST_AVG_TIME_20_MS = 0x0080,
	SELFTEST_AVG_TIME_40_MS = 0x0100,
	SELFTEST_AVG_TIME_80_MS = 0x0180,
	SELFTEST_AVG_TIME_160_MS = 0x0200,
	SELFTEST_AVG_TIME_320_MS = 0x0280,

	ST_DIRECT_AVG_TIME_10_MS = 10,
	ST_DIRECT_AVG_TIME_20_MS = 20,
	ST_DIRECT_AVG_TIME_40_MS = 40,
	ST_DIRECT_AVG_TIME_80_MS = 80,
	ST_DIRECT_AVG_TIME_160_MS = 160,
	ST_DIRECT_AVG_TIME_320_MS = 320
} selftest_average_time_t;

/* Tolerance between factory trim and accel self-test response */
typedef enum {
	SELFTEST_ACCEL_THRESHOLD_5_PERCENT = 0x0000,
	SELFTEST_ACCEL_THRESHOLD_10_PERCENT = 0x0400,
	SELFTEST_ACCEL_THRESHOLD_15_PERCENT = 0x0800,
	SELFTEST_ACCEL_THRESHOLD_20_PERCENT = 0x0c00,
	SELFTEST_ACCEL_THRESHOLD_25_PERCENT = 0x1000,
	SELFTEST_ACCEL_THRESHOLD_30_PERCENT = 0x1400,
	SELFTEST_ACCEL_THRESHOLD_40_PERCENT = 0x1800,
	SELFTEST_ACCEL_THRESHOLD_50_PERCENT = 0x1c00,

	ST_DIRECT_ACCEL_THRESHOLD_5_PERCENT = 5,
	ST_DIRECT_ACCEL_THRESHOLD_10_PERCENT = 10,
	ST_DIRECT_ACCEL_THRESHOLD_15_PERCENT = 15,
	ST_DIRECT_ACCEL_THRESHOLD_20_PERCENT = 20,
	ST_DIRECT_ACCEL_THRESHOLD_25_PERCENT = 25,
	ST_DIRECT_ACCEL_THRESHOLD_30_PERCENT = 30,
	ST_DIRECT_ACCEL_THRESHOLD_40_PERCENT = 40,
	ST_DIRECT_ACCEL_THRESHOLD_50_PERCENT = 50
} selftest_accel_threshold_t;

/* Tolerance between factory trim and gyro self-test response */
typedef enum {
	SELFTEST_GYRO_THRESHOLD_5_PERCENT = 0x0000,
	SELFTEST_GYRO_THRESHOLD_10_PERCENT = 0x2000,
	SELFTEST_GYRO_THRESHOLD_15_PERCENT = 0x4000,
	SELFTEST_GYRO_THRESHOLD_20_PERCENT = 0x6000,
	SELFTEST_GYRO_THRESHOLD_25_PERCENT = 0x8000,
	SELFTEST_GYRO_THRESHOLD_30_PERCENT = 0xa000,
	SELFTEST_GYRO_THRESHOLD_40_PERCENT = 0xc000,
	SELFTEST_GYRO_THRESHOLD_50_PERCENT = 0xe000,

	ST_DIRECT_GYRO_THRESHOLD_5_PERCENT = 5,
	ST_DIRECT_GYRO_THRESHOLD_10_PERCENT = 10,
	ST_DIRECT_GYRO_THRESHOLD_15_PERCENT = 15,
	ST_DIRECT_GYRO_THRESHOLD_20_PERCENT = 20,
	ST_DIRECT_GYRO_THRESHOLD_25_PERCENT = 25,
	ST_DIRECT_GYRO_THRESHOLD_30_PERCENT = 30,
	ST_DIRECT_GYRO_THRESHOLD_40_PERCENT = 40,
	ST_DIRECT_GYRO_THRESHOLD_50_PERCENT = 50
} selftest_gyro_threshold_t;

/** Self-Test parameters */
typedef struct {
	/** If set, enable accel self-test */
	uint8_t accel_en;

	/** If set, enable gyro self-test */
	uint8_t gyro_en;

	/** Averaging time used to perform self-test */
	selftest_average_time_t avg_time;

	/** Tolerance between factory trim and accel self-test response */
	selftest_accel_threshold_t accel_limit;

	/** Tolerance between factory trim and gyro self-test response */
	selftest_gyro_threshold_t gyro_limit;

	/** Mechanism for adding patches to self-test operations */
	uint32_t patch_settings;
} inv_imu_selftest_parameters_t;

/** Self-test outputs */
typedef struct {
	/** Global accel self-test status. 1 for success, 0 otherwise. */
	int8_t accel_status;

	/** Global gyro self-test status. 1 for success, 0 otherwise. */
	int8_t gyro_status;

	/** AX self-test status. 1 for success, 0 otherwise. */
	int8_t ax_status;

	/** AY self-test status. 1 for success, 0 otherwise. */
	int8_t ay_status;

	/** AZ self-test status. 1 for success, 0 otherwise. */
	int8_t az_status;

	/** GX self-test status. 1 for success, 0 otherwise. */
	int8_t gx_status;

	/** GY self-test status. 1 for success, 0 otherwise. */
	int8_t gy_status;

	/** GZ self-test status. 1 for success, 0 otherwise. */
	int8_t gz_status;
} inv_imu_selftest_output_t;

/** @brief Provide recommended parameters to execute self-test.
 *  @param[in] s          Pointer to device.
 *  @param[in] st_params  Structure filled with recommended params.
 *  @return               0 on success, negative value on error.
 */
int icm566xx_selftest_init_params(inv_imu_device_t *s, inv_imu_selftest_parameters_t *st_params);

/** @brief Provide recommended parameters to execute self-test.
 *  @param[in] s          Pointer to device.
 *  @param[in] st_params  Structure filled with recommended params.
 *  @return               0 on success, negative value on error.
 */
int icm566xx_st_direct_init_params(inv_imu_device_t *s, inv_imu_selftest_parameters_t *st_params);

/** @brief Perform hardware self-test for Accel and/or Gyro.
 *  @param[in] s            Pointer to device.
 *  @param[in] st_params    Self-test parameters to be used.
 *  @param[out] st_output   Output from Self-test operation.
 *  @return                 0 on success, negative value on error.
 */
int icm566xx_selftest(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
		      inv_imu_selftest_output_t *st_output);

/** @brief Perform host-based self-test for Accel and/or Gyro.
 *  @param[in] s            Pointer to device.
 *  @param[in] st_params    Self-test parameters to be used.
 *  @param[out] st_output   Output from Self-test operation.
 *  @return                 0 on success, negative value on error.
 */
int icm566xx_st_direct(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
		       inv_imu_selftest_output_t *st_output);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_SELFTEST_H_ */

/** @} */
