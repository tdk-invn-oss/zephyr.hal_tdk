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

#ifndef _INV_IMU_EDMP_APEX_H_
#define _INV_IMU_EDMP_APEX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "icm566xx/imu/inv_imu_driver.h"
#include "icm566xx/imu/inv_imu_edmp.h"
#include "icm566xx/imu/inv_imu_edmp_memmap.h"
#include "icm566xx/imu/inv_imu_edmp_defs.h"

#include <stdint.h>
#include <string.h>

/** Registers to retrieve interrupts status for APEX. */
typedef struct {
	int_apex_status0_t int_apex_status0;
	int_apex_status1_t int_apex_status1;
	int_apex_status2_t int_apex_status2;
} int_apex_statusx_t;

/** Registers to configure interrupts for APEX. */
typedef struct {
	int_apex_config0_t int_apex_config0;
	int_apex_config1_t int_apex_config1;
	int_apex_config2_t int_apex_config2;
} int_apex_configx_t;

/** @brief APEX interrupts definition */
typedef struct {
	uint8_t INV_TAP;
	uint8_t INV_HIGHG;
	uint8_t INV_LOWG;
	uint8_t INV_TILT_DET;
	uint8_t INV_STEP_CNT_OVFL;
	uint8_t INV_STEP_DET;
	uint8_t INV_FF;
	uint8_t INV_R2W;
	uint8_t INV_B2S;

	uint8_t INV_R2W_SLEEP;
	uint8_t INV_B2S_REV;
	uint8_t INV_SMD;
	uint8_t INV_SELF_TEST;
	uint8_t INV_SEC_AUTH;
	uint8_t INV_RSVD;

	uint8_t INV_SHAKE;
	uint8_t INV_NO_MOTION;
	uint8_t INV_MOTION;
	uint8_t INV_NO_FLAT;
	uint8_t INV_FLAT;
	uint8_t INV_BUMP;
} inv_imu_edmp_int_state_t;

/** @brief IMU APEX inputs parameters definition
 *  @note Refer to the datasheet for details on how to configure these parameters.
 */
typedef struct {
	/* Pedometer */
	uint32_t ped_amp_th;
	uint16_t ped_step_cnt_th;
	uint16_t ped_step_det_th;
	uint16_t ped_sb_timer_th;
	uint32_t ped_hi_en_th;
	uint8_t ped_sensitivity_mode;
	uint32_t ped_low_en_amp_th;
	uint16_t ped_odr;

	/* Tilt */
	uint16_t tilt_wait_time;
	int32_t tilt_angle;
	uint16_t tilt_odr;

	/* SMD */
	uint8_t smd_sensitivity;
	uint16_t smd_odr;

	/* R2W */
	uint32_t r2w_sleep_time_out;
	uint32_t r2w_sleep_gesture_delay;
	uint32_t r2w_mounting_matrix;
	uint32_t r2w_gravity_filter_gain;
	uint32_t r2w_motion_th_angle_cosine;
	uint32_t r2w_motion_th_timer_fast;
	uint32_t r2w_motion_th_timer_slow;
	uint32_t r2w_motion_prev_gravity_timeout;
	uint32_t r2w_last_gravity_motion_timer;
	uint32_t r2w_last_gravity_timeout;
	uint32_t r2w_gesture_validity_timeout;
	uint16_t r2w_odr;
	uint32_t r2w_sampling_period;

	/* Freefall */
	uint16_t lowg_peak_th;
	uint16_t lowg_peak_th_hyst;
	uint16_t lowg_time_th;
	uint16_t highg_peak_th;
	uint16_t highg_peak_th_hyst;
	uint16_t highg_time_th;
	uint32_t ff_min_duration;
	uint32_t ff_max_duration;
	uint32_t ff_debounce_duration;
	uint16_t ff_odr;

	/* Tap */
	uint8_t tap_min_jerk;
	uint16_t tap_tmax;
	uint8_t tap_tmin;
	uint8_t tap_max_peak_tol;
	uint8_t tap_smudge_reject_th;
	uint8_t tap_tavg;
	uint8_t tap_max_tap;
	uint8_t tap_min_tap;
	uint16_t tap_max_energy_primary;
	uint16_t tap_max_energy_secondary;
	uint8_t tap_axis_select_mask;
	uint16_t tap_odr;

	/* B2S */
	uint8_t b2s_mounting_matrix;
	uint32_t b2s_DevNormMax;
	uint32_t b2s_SinLimit;
	uint32_t b2s_FastMotionAgeLimit;
	uint32_t b2s_FastLimit;
	uint32_t b2s_FastMotionTimeLimit;
	uint32_t b2s_BringToSeeAgeLimit;
	uint32_t b2s_StaticLimit;
	uint32_t b2s_ThrCosAng;
	uint32_t b2s_RevB2sLatencyTh;
	uint16_t b2s_odr;

	/* Shake */
	uint16_t shake_thresh_static;
	uint16_t shake_minimal_static_duration;
	uint16_t shake_thresh_motion;
	uint16_t shake_thresh_shake_x;
	uint16_t shake_thresh_shake_y;
	uint16_t shake_thresh_shake_z;
	uint16_t shake_minimal_shake_duration;
	uint16_t shake_maximal_shake_duration;
	uint16_t shake_internal_duration;
	uint16_t shake_axis_detect_factor;
	uint16_t shake_axis_majority_th;
	uint16_t shake_odr;

	/* noMotion */
	uint16_t nomotion_thresh_static;
	uint16_t nomotion_minimal_static_duration;
	uint16_t nomotion_thresh_motion;
	uint16_t nomotion_odr;

	/* Flat */
	int32_t flat_cos_flat_angle_th;
	int32_t flat_ref_axis_x;
	int32_t flat_ref_axis_y;
	int32_t flat_ref_axis_z;
	uint16_t flat_wait_time;
	uint16_t flat_three_axis_conv_time;
	uint32_t flat_ref_axis_selection;
	uint16_t flat_isSymetrical;
	uint16_t flat_odr;

	/* Power save */
	uint32_t power_save_time;
	uint8_t power_save_en;

	/* 3 axis */
	uint16_t three_axis_odr;
	uint32_t three_axis_accel_only_gain;

} inv_imu_edmp_apex_parameters_t;

/** @brief Pedometer activity class */
typedef enum {
	INV_IMU_EDMP_UNKNOWN = 0,
	INV_IMU_EDMP_WALK = 1,
	INV_IMU_EDMP_RUN = 2,
} inv_imu_edmp_activity_class_t;

typedef enum {
	X_AXIS_SELECTION = 0,
	Y_AXIS_SELECTION = 1,
	Z_AXIS_SELECTION = 2,
	ADV_AXIS_SELECTION = 3
} inv_imu_flat_ref_axis_selection_e;

/** @brief Pedometer outputs */
typedef struct {
	/** @brief Number of steps. */
	uint16_t step_cnt;

	/** @brief Walk/Run cadency in number of samples.
	 *         Number of samples between two steps with u6.2 format (8-bits unsigned in Q2).
	 *         cadency (steps/s) = EDMP_ODR_HZ / (step_cadence * 0.25)
	 */
	uint8_t step_cadence;

	/** @brief Detected activity. */
	inv_imu_edmp_activity_class_t activity_class;
} inv_imu_edmp_pedometer_data_t;

/** @brief Tap number definition */
typedef enum {
	INV_IMU_EDMP_TAP_TRIPLE = 0x03,
	INV_IMU_EDMP_TAP_DOUBLE = 0x02,
	INV_IMU_EDMP_TAP_SINGLE = 0x01,
} inv_imu_edmp_tap_num_t;

/** @brief Tap axis definition */
typedef enum {
	INV_IMU_EDMP_TAP_AXIS_Z = 0x02,
	INV_IMU_EDMP_TAP_AXIS_Y = 0x01,
	INV_IMU_EDMP_TAP_AXIS_X = 0x00,
} inv_imu_edmp_tap_axis_t;

/** @brief Tap direction definition */
typedef enum {
	INV_IMU_EDMP_TAP_DIR_POSITIVE = 0x01,
	INV_IMU_EDMP_TAP_DIR_NEGATIVE = 0x00,
} inv_imu_edmp_tap_dir_t;

/** @brief Tap outputs */
typedef struct {
	inv_imu_edmp_tap_num_t num;
	inv_imu_edmp_tap_axis_t axis;
	inv_imu_edmp_tap_dir_t direction;
	uint8_t double_tap_timing;
	uint8_t triple_tap_timing;
} inv_imu_edmp_tap_data_t;

/*Flat Outputs*/
/** @brief Flat direction definition */
typedef enum {
	INIT_STATE = 0,
	FLAT_STATE = 1,
	NON_FLAT_STATE = 3
} inv_imu_edmp_flat_state_e;

/*No Motion Outputs*/
typedef enum {
	AXIS_X_MINUS = 0,
	AXIS_Y_MINUS = 1,
	AXIS_Z_MINUS = 2,
	AXIS_X_PLUS = 4,
	AXIS_Y_PLUS = 5,
	AXIS_Z_PLUS = 6
} inv_imu_edmp_nomtion_majoraxis_e;

typedef struct {
	inv_imu_edmp_nomtion_majoraxis_e majority_axis;
	/*will need to convert raw tilt angle its value in degree*/
	int16_t tilt_angle_raw_format;
	int8_t sign_angle;
} inv_imu_edmp_nomtion_data_t;

/** @brief Returns current EDMP parameters for APEX algorithms.
 *  @param[in] s   Pointer to device.
 *  @param[out] p  The current parameters read from registers.
 *  @return        0 on success, negative value on error.
 */
int icm566xx_edmp_get_apex_parameters(inv_imu_device_t *s, inv_imu_edmp_apex_parameters_t *p);

/** @brief Configures EDMP parameters for APEX algorithms.
 *  @warning This function should be called only when all EDMP algorithms are disabled.
 *  @param[in] s  Pointer to device.
 *  @param[in] p  The requested input parameters.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_apex_parameters(inv_imu_device_t *s, const inv_imu_edmp_apex_parameters_t *p);

/** @brief Retrieve interrupts configuration.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Configuration of each APEX interrupt.
 *  @return         0 on success, negative value on error.
 */
int icm566xx_edmp_get_config_int_apex(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it);

/** @brief Configure APEX interrupt.
 *  @param[in] s   Pointer to device.
 *  @param[in] it  State of each APEX interrupt to configure.
 *  @return        0 on success, negative value on error.
 */
int icm566xx_edmp_set_config_int_apex(inv_imu_device_t *s, const inv_imu_edmp_int_state_t *it);

/** @brief  Enable APEX algorithm Pedometer.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_pedometer(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Pedometer.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_pedometer(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm Pedometer.
 *  @param[in] s  Pointer to device.
 *  @param[in] ped_odr  New value for pedometer ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_pedometer_odr(inv_imu_device_t *s, uint16_t ped_odr);

/** @brief  Enable APEX algorithm Significant Motion Detection.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_smd(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Significant Motion Detection.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_smd(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm SMD.
 *  @param[in] s  Pointer to device.
 *  @param[in] smd_odr  New value for SMD ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_smd_odr(inv_imu_device_t *s, uint16_t smd_odr);

/** @brief  Enable APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_tilt(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_tilt(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @param[in] tilt_odr  New value for tilt ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_tilt_odr(inv_imu_device_t *s, uint16_t tilt_odr);

/** @brief  Enable APEX algorithm R2W.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_r2w(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm R2W.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_r2w(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm R2W.
 *  @param[in] s  Pointer to device.
 *  @param[in] r2w_odr  New value for R2W ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_r2w_odr(inv_imu_device_t *s, uint16_t r2w_odr);

/** @brief  Enable APEX algorithm B2S.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_b2s(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm B2S.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_b2s(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm B2S.
 *  @param[in] s  Pointer to device.
 *  @param[in] b2s_odr  New value for B2S ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_b2s_odr(inv_imu_device_t *s, uint16_t b2s_odr);

/** @brief  Enable APEX algorithm Tap.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_tap(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Tap.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_tap(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm Tap.
 *  @param[in] s  Pointer to device.
 *  @param[in] tap_odr  New value for tap ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_tap_odr(inv_imu_device_t *s, uint16_t tap_odr);

/** @brief  Enable APEX algorithm Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_ff(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_ff(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm Freefall.
 *  @param[in] s  Pointer to device.
 *  @param[in] ff_odr  New value for freefall ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_ff_odr(inv_imu_device_t *s, uint16_t ff_odr);

/** @brief  Enable APEX algorithm soft hard iron correlation.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_soft_iron_cor(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm soft hard iron correlation.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_soft_iron_cor(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm Shake.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_shake(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Shake.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_shake(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm Shake.
 *  @param[in] s  Pointer to device.
 *  @param[in] shake_odr  New value for shake ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_shake_odr(inv_imu_device_t *s, uint16_t shake_odr);

/** @brief  Enable APEX algorithm noMotion.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_noMotion(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm noMotion.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_noMotion(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm no-motion.
 *  @param[in] s  Pointer to device.
 *  @param[in] noMotion_odr  New value for no-motion ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_noMotion_odr(inv_imu_device_t *s, uint16_t noMotion_odr);

/** @brief  Enable APEX algorithm flat.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_flat(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm flat.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_flat(inv_imu_device_t *s);

/** @brief  Set ODR for APEX algorithm Flat.
 *  @param[in] s  Pointer to device.
 *  @param[in] flat_odr  New value for flat ODR
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_flat_odr(inv_imu_device_t *s, uint16_t flat_odr);

/** @brief  Enable APEX algorithm Tap.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */

/** @brief Configure EDMP Output Data Rate for compass data.
 *  @param[in] s  Pointer to device.
 *  @param[in] frequency The requested frequency.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_set_compass_frequency(inv_imu_device_t *s,
					const dmp_ext_sen_odr_cfg_ext_odr_t frequency);

/** @brief  Enable compass handling by eDMP as external sensor.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable_compass_es(inv_imu_device_t *s);

/** @brief  Disable compass handling by eDMP as external sensor.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable_compass_es(inv_imu_device_t *s);

/** @brief Read APEX interrupt status.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Status of each APEX interrupt.
 *  @return         0 on success, negative value on error.
 */
int icm566xx_edmp_get_int_apex_status(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it);

/** @brief Retrieve pedometer outputs.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Pedometer step count and activity data value.
 *  @return           0 on success, negative value on error.
 *  @retval           INV_IMU_ERROR_EDMP_BUF_EMPTY if step count buffer is empty.
 */
int icm566xx_edmp_get_pedometer_data(inv_imu_device_t *s, inv_imu_edmp_pedometer_data_t *data);

/** @brief Retrieve APEX free fall outputs and format them.
 *  @param[in] s                   Pointer to device.
 *  @param[out] freefall_duration  Duration in number of sample.
 *  @return                        0 on success, negative value on error.
 */
int icm566xx_edmp_get_ff_data(inv_imu_device_t *s, uint16_t *freefall_duration);

/** @brief Retrieve tap outputs.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Tap number and direction.
 *  @return           0 on success, negative value on error.
 */
int icm566xx_edmp_get_tap_data(inv_imu_device_t *s, inv_imu_edmp_tap_data_t *data);

/** @brief Retrieve noMotion outputs.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Flat Status, timer,quat(x/y/z), cos_flat_angle.
 *  @return           0 on success, negative value on error.
 */
int icm566xx_edmp_get_nomotion_data(inv_imu_device_t *s, inv_imu_edmp_nomtion_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_EDMP_APEX_H_ */

/** @} */
