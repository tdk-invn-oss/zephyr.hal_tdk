/*
 * Copyright (c) 2015 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup selfcal Self-cal
 *  @brief    API to execute self-cal procedure.
 *  @{
 */

/** @file icm566xx_selfcal.h */

#ifndef _INV_IMU_SELFCAL_H_
#define _INV_IMU_SELFCAL_H_

#include <stdint.h>

#include "icm566xx/imu/inv_imu_driver_advanced.h"

#ifdef __cplusplus
extern "C" {
#endif

/* STC status codes */
#define INV_IMU_SC_STATUS_SUCCESS 1  /**< Indicates test is successful */
#define INV_IMU_SC_STATUS_FAIL    -1 /**< Indicates test is failing */
#define INV_IMU_SC_STATUS_NOT_RUN 0  /**< Indicates test has not run */

/* User gain values are stored as an integer value between 0 and 2048 */
#define USERGAIN_SCALE 2048

/* Moved over from inv_imu_defs.h */
#define SELFCAL_ACCEL_EN_MASK    0x0008
#define SELFCAL_ACCEL_EN         0x0008
#define SELFCAL_ACCEL_DIS        0x0000
#define SELFCAL_GYRO_EN_MASK     0x0010
#define SELFCAL_GYRO_EN          0x0010
#define SELFCAL_GYRO_DIS         0x0000
#define SELFCAL_GYRO_METHOD_MASK 0x0020
#define SELFCAL_SSOM_EN_MASK     0x0040
#define SELFCAL_SSOM_EN          0x0040
#define SELFCAL_SSOM_DIS         0x0000

typedef enum {
	SELFCAL_STEP_RESPONSE = 0x0000,
	SELFCAL_SC2V_ADC = 0x0020
} selfcal_gyro_method_t;

typedef enum {
	SELFCAL_PATCH_EN_ACCEL_AX_MEMS = 0x0010,
	SELFCAL_PATCH_EN_ACCEL_AX_CMOS = 0x0020,
	SELFCAL_PATCH_EN_ACCEL_AY_MEMS = 0x0040,
	SELFCAL_PATCH_EN_ACCEL_AY_CMOS = 0x0080,
	SELFCAL_PATCH_EN_ACCEL_AZ_MEMS1 = 0x0100,
	SELFCAL_PATCH_EN_ACCEL_AZ_MEMS2 = 0x0200,
	SELFCAL_PATCH_EN_GYRO2_PHASE1 = 0x0400,
	SELFCAL_PATCH_EN_GYRO2_PHASE2 = 0x0800
} sc_patch_params_t;
/* */

/* Moved over from inv_imu_edmp_defs.h */
#define EDMP_HOST_INT_SELF_CAL_DONE_POS 0x3
#define EDMP_HOST_INT_SELF_CAL_DONE_MSK 0x8
/* */

/* Moved over from inv_imu_edmp_memmap.h */
/* These should be used with caution as the addresses may change */
/*
 * stc_accel_sc_nout_meas_x
 * *       Accel measurements for X axis in s32.16 for accel self-cal
 *
 */
#define EDMP_STC_ACCEL_SC_NOUT_MEAS_X      0x48
#define EDMP_STC_ACCEL_SC_NOUT_MEAS_X_SIZE 4

/*
 * stc_accel_sc_nout_meas_y
 * *       Accel measurements for Y axis in s32.16 for accel self-cal
 *
 */
#define EDMP_STC_ACCEL_SC_NOUT_MEAS_Y      0x4c
#define EDMP_STC_ACCEL_SC_NOUT_MEAS_Y_SIZE 4

/*
 * stc_accel_sc_nout1_meas_z
 * *       Accel measurements for Z axis in s32.16 for accel self-cal
 *
 */
#define EDMP_STC_ACCEL_SC_NOUT1_MEAS_Z      0x50
#define EDMP_STC_ACCEL_SC_NOUT1_MEAS_Z_SIZE 4

/*
 * stc_accel_sc_nout2_meas_z
 * *       Accel measurements for Z axis in s32.16 for accel self-cal
 *
 */
#define EDMP_STC_ACCEL_SC_NOUT2_MEAS_Z      0x5c
#define EDMP_STC_ACCEL_SC_NOUT2_MEAS_Z_SIZE 4

/*
 * stc_accel_sc_cmos_meas_x
 * *       Accel measurements for X axis in s32.16 for accel self-cal
 *
 */
#define EDMP_STC_ACCEL_SC_CMOS_MEAS_X      0x54
#define EDMP_STC_ACCEL_SC_CMOS_MEAS_X_SIZE 4
/* */

/** Self-Cal parameters */
typedef struct {
	/** If set, enable accel self-cal */
	uint8_t accel_en;

	/** If set, enable gyro self-cal */
	uint8_t gyro_en;

	/** If set, enable SSOM for self-cal */
	uint8_t ssom_en;

	/** The accel slope values used in self-cal calculations */
	int32_t accel_slope[3];

	/** The accel bias values used in self-cal calculations */
	int32_t accel_bias[3];

	/** The gyro slope values used in self-cal calculations */
	int32_t gyro_slope[3];

	/** The gyro bias values used in self-cal calculations */
	int32_t gyro_bias[3];

	/** The method used to calculate the gyro calibration */
	selfcal_gyro_method_t gyro_method[3];

	/** Averaging time */
	selftest_average_time_t avg_time;

	/** Mechanism for adding patches to self-cal operations */
	uint32_t patch_settings;
} inv_imu_selfcal_parameters_t;

/** Self-cal outputs */
typedef struct {
	/** Global accel self-cal passed. 1 for success, 0 otherwise. */
	int8_t accel_status;

	/** Global gyro self-cal passed. 1 for success, 0 otherwise. */
	int8_t gyro_status;

	/** Calculated values for accel user gain fields. */
	uint16_t accel_gains[3];

	/** Calculated values for accel user gain fields. */
	uint16_t gyro_gains[3];
} inv_imu_selfcal_output_t;

/** @brief Provide recommended parameters to execute self-calib.
 *  @param[in] s           Pointer to device.
 *  @param[in] sc_params   Structure filled with recommended params.
 *  @return                0 on success, negative value on error.
 */
int icm566xx_selfcal_init_params(inv_imu_device_t *s, inv_imu_selfcal_parameters_t *sc_params);

/** @brief Perform hardware self-cal plus user gain calculation for Accel and/or Gyro.
 *  @param[in] s            Pointer to device.
 *  @param[in] sc_params    Self-cal parameters to be used.
 *  @param[out] sc_output   Output from Self-Cal operation
 *  @return                 0 on success, negative value on error.
 */
int icm566xx_selfcal(inv_imu_device_t *s, const inv_imu_selfcal_parameters_t *sc_params,
		     inv_imu_selfcal_output_t *sc_output);

/** @brief Set accel user gain values to the appropriate register fields.
 *  @param[in] s               Pointer to device.
 *  @param[in] accel_usergain  Accel user gain values.
 *  @return                    0 on success, negative value on error.
 */
int icm566xx_selfcal_set_accel_usergain(inv_imu_device_t *s, const uint16_t *accel_usergain);

/** @brief Set gyro user gain values to the appropriate register fields.
 *  @param[in] s              Pointer to device.
 *  @param[in] gyro_usergain  Gyro user gain values.
 *  @return                   0 on success, negative value on error.
 */
int icm566xx_selfcal_set_gyro_usergain(inv_imu_device_t *s, const uint16_t *gyro_usergain);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_SELFCAL_H_ */

/** @} */
