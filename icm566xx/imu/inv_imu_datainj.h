/*
 * Copyright (c) 2016 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#ifndef _INV_IMU_DATA_INJ_H_
#define _INV_IMU_DATA_INJ_H_

#include "icm566xx/imu/inv_imu_driver_advanced.h"

typedef enum {
	/* APEX features */
	INV_DATA_INJ_FEATURE_R2W_MM_0,  /* RTW 25Hz with APEX mounting-matrix = [ 1  0  0;  0  1  0;
					   0  0  1] */
	INV_DATA_INJ_FEATURE_R2W_MM_1,  /* RTW 25Hz with APEX mounting-matrix = [ 1  0  0;  0 -1  0;
					   0  0 -1] */
	INV_DATA_INJ_FEATURE_R2W_MM_2,  /* RTW 25Hz with APEX mounting-matrix = [-1  0  0;  0  1  0;
					   0  0 -1] */
	INV_DATA_INJ_FEATURE_R2W_MM_3,  /* RTW 25Hz with APEX mounting-matrix = [-1  0  0;  0 -1  0;
					   0  0  1] */
	INV_DATA_INJ_FEATURE_R2W_MM_4,  /* RTW 25Hz with APEX mounting-matrix = [ 0  1  0;  1  0  0;
					   0  0 -1] */
	INV_DATA_INJ_FEATURE_R2W_MM_5,  /* RTW 25Hz with APEX mounting-matrix = [ 0  1  0; -1  0  0;
					   0  0  1] */
	INV_DATA_INJ_FEATURE_R2W_MM_6,  /* RTW 25Hz with APEX mounting-matrix = [ 0 -1  0;  1  0  0;
					   0  0  1] */
	INV_DATA_INJ_FEATURE_R2W_MM_7,  /* RTW 25Hz with APEX mounting-matrix = [ 0 -1  0; -1  0  0;
					   0  0 -1] */
	INV_DATA_INJ_FEATURE_R2W_50HZ,  /* RTW 50Hz with APEX mounting-matrix = [ 1  0  0;  0  1  0;
					   0  0  1] */
	INV_DATA_INJ_FEATURE_R2W_100HZ, /* RTW 100Hz with APEX mounting-matrix = [ 1  0  0;  0  1 0;
					   0  0  1] */

	INV_DATA_INJ_FEATURE_SMD_SENSIVITY_0,
	INV_DATA_INJ_FEATURE_SMD_SENSIVITY_1,
	INV_DATA_INJ_FEATURE_SMD_SENSIVITY_2,
	INV_DATA_INJ_FEATURE_SMD_SENSIVITY_3,
	INV_DATA_INJ_FEATURE_SMD_SENSIVITY_4,

	INV_DATA_INJ_FEATURE_TILT,

	INV_DATA_INJ_FEATURE_PEDO_25HZ_MODE0,
	INV_DATA_INJ_FEATURE_PEDO_25HZ_MODE1,
	INV_DATA_INJ_FEATURE_PEDO_50HZ_MODE0,
	INV_DATA_INJ_FEATURE_PEDO_50HZ_MODE1,
	INV_DATA_INJ_FEATURE_PEDO_OVRF,

	/* ST features */
	INV_DATA_INJ_FEATURE_SELF_TEST_16SAMPLES_AG,
	INV_DATA_INJ_FEATURE_SELF_TEST_200SAMPLES_A,
	INV_DATA_INJ_FEATURE_SELF_TEST_200SAMPLES_G,
	INV_DATA_INJ_FEATURE_SELF_TEST_16SAMPLES_AG_HIGH_FSR,

	INV_DATA_INJ_FEATURE_LOAD_NEW_DMP_IMAGE,

	INV_DATA_INJ_FEATURE_FF,

	INV_DATA_INJ_FEATURE_TAP,

	INV_DATA_INJ_FEATURE_PEDOMETER,
	INV_DATA_INJ_FEATURE_R2W,
	INV_DATA_INJ_FEATURE_BASICSMD,
	INV_DATA_INJ_FEATURE_NOMOTION, /*31 */
	INV_DATA_INJ_FEATURE_SHAKE,    /*32 */
	INV_DATA_INJ_FEATURE_B2S,
	INV_DATA_INJ_FEATURE_FLAT,
	/*the below are deprecated for Venice. To be removed */
	INV_DATA_INJ_FEATURE_TILTRAM,
	INV_DATA_INJ_FEATURE_IRONCAL,
	INV_DATA_INJ_FEATURE_BUMP

} inv_imu_data_inj_feature_t;

union var_types_t {
	long data_long;
	short data_short;
	float data_float;
	unsigned char data_uchar[4];
	unsigned short data_ushort;
	unsigned long data_ulong;
};

typedef union {
	uint8_t data[28];
	struct {
		uint16_t dmp_exec_time;
		uint16_t step_count;
		uint8_t step_cadence;
		uint8_t activity_class;
		uint8_t int_status;
		uint8_t padding;
		int32_t state_3axis_accel_wx;
		int32_t state_3axis_accel_wy;
		int32_t state_3axis_accel_wz;
	} pedo;
	struct {
		uint16_t dmp_exec_time;
		uint16_t duration;
		uint16_t state;
		uint16_t lowg_state;
		uint16_t lowg_cntr;
		uint16_t highg_state;
		uint16_t highg_cntr;
		uint8_t lowg_event;
		uint8_t highg_event;
		uint8_t int_status;
	} ff;
	struct {
		uint16_t dmp_exec_time;
		uint8_t int_status;
		uint8_t reserved;
		uint32_t angle;
		int32_t quat[4];
	} tilt;
	struct {
		uint16_t dmp_exec_time;
		uint8_t type;
		uint8_t axis;
		uint8_t dir;
		uint8_t dble_timing;
		uint8_t trple_timing;
		uint8_t it;
		uint8_t int_status;
	} tap;
	struct {
		uint16_t dmp_exec_time;
		uint8_t int_status;
	} smd;
	struct {
		uint16_t dmp_exec_time;
		uint8_t smd_flag;
		uint8_t smd_wom_flag;
	} basicsmd;
	struct {
		uint16_t dmp_exec_time;
		uint8_t st_status1;
		uint8_t st_status2;
		uint8_t reserved[3];
	} st;
	struct {
		uint16_t dmp_exec_time;
		uint8_t wake_event;
		uint8_t sleep_event;
	} r2w;
	struct {
		uint16_t dmp_exec_time;
		uint8_t b2s_event;
		uint8_t b2s_rev_event;
	} b2s;
	struct {
		uint16_t dmp_exec_time;
		int16_t cal_mag[3];
	} ironcal;
	struct {
		uint16_t dmp_exec_time;
		uint16_t shake_axis;
		uint16_t shake_C1;
		uint16_t shake_candidate;
		uint16_t shake_general_shake_timer;
		uint16_t shake_stabilityCount;
		uint16_t shake_previous_state;
		uint8_t candidate_x;
		uint8_t candidate_y;
		uint8_t candidate_z;
		uint8_t shake_outInterrupt;
	} shake;
	struct {
		uint16_t dmp_exec_time;
		int16_t noMotion_tilt_angle_raw_format;
		int16_t noMotion_x_axis_ref;
		int16_t noMotion_y_axis_ref;
		int8_t noMotion_sign_angle;
		uint8_t noMotion_majority_axis;
		uint8_t int_status;
	} noMotion;
	struct {
		uint16_t dmp_exec_time;
		uint16_t flat_init_timer;
		uint16_t flat_timer;
		uint16_t padding;
		int32_t flat_quat_wb_x;
		int32_t flat_quat_wb_y;
		int32_t flat_quat_wb_z;
		int32_t cos_flat_angle;
		uint8_t flat_event;
		uint8_t no_flat_event;
		uint8_t flat_state;
	} flat;
	struct {
		uint16_t dmp_exec_time;
		uint8_t bump_event;
	} bump;
} inv_imu_data_inj_output;

typedef struct {
	uint32_t sample_count;
	uint32_t dmp_exec_time_min;
	uint32_t dmp_exec_time_max;
	float dmp_exec_time_mean;
} inv_imu_data_inj_stats;

typedef struct {
	uint16_t low_g_state;
	uint16_t low_g_cntr;
	uint16_t low_g_low_peak_thres;
	uint16_t low_g_low_peak_thres_hyst;
	uint16_t low_g_low_time_thres;
	uint16_t low_g_accel_ODR;
} inv_imu_edmp_low_g_intern;

/** @brief 25 (stabilisation time at start) + 200 + 25 (stabilisation time after ST on) + 200 = 450
 */
#define DATA_INJ_SELF_TEST_A_200_TOTAL_SAMPLE_COUNT  (450)
/** @brief 100 (stabilisation time at start) + 200 + 50 (stabilisation time after ST on) + 200 = 550
 */
#define DATA_INJ_SELF_TEST_G_200_TOTAL_SAMPLE_COUNT  (550)
/** @brief 100 (stabilisation time at start) + 200 + 50 (stabilisation time after ST on) + 200 = 550
 */
#define DATA_INJ_SELF_TEST_AG_200_TOTAL_SAMPLE_COUNT (550)
/** @brief 25 (stabilisation time at start) + 16 + 25 (stabilisation time after ST on) + 16 = 82 */
#define DATA_INJ_SELF_TEST_A_16_TOTAL_SAMPLE_COUNT   (82)
/** @brief 100 (stabilisation time at start) + 16 + 50 (stabilisation time after ST on) + 16 = 182
 */
#define DATA_INJ_SELF_TEST_G_16_TOTAL_SAMPLE_COUNT   (182)
/** @brief 100 (stabilisation time at start) + 16 + 50 (stabilisation time after ST on) + 16 = 182
 */
#define DATA_INJ_SELF_TEST_AG_16_TOTAL_SAMPLE_COUNT  (182)
/** @brief Max number of samples required for self-test data injection */
#define DATA_INJ_SELF_TEST_MAX_SAMPLE_COUNT          (DATA_INJ_SELF_TEST_AG_200_TOTAL_SAMPLE_COUNT)

uint32_t icm566xx_data_inj_inject(inv_imu_device_t *self, inv_imu_data_inj_feature_t feature,
				  int32_t sensor_cli_test_data[6], uint16_t param, uint32_t idx,
				  inv_imu_data_inj_output *out);
void icm566xx_data_inj_reset(inv_imu_device_t *self);
void icm566xx_data_inj_get_stats(inv_imu_data_inj_stats *stats);

/** @brief Set accel scaling factor so that data are injected in format expected by edmp which
 * differs from IV data format. Indeed, IV accel data are +/- FSRgee on 16bits but edmp expects only
 * 1gee Q16. So it comes:
 * - with  2gee FSR, IV scale is 1gee Q14 => scale factor is lshift by 2
 * - with  4gee FSR, IV scale is 1gee Q13 => scale factor is lshift by 3
 * - with  8gee FSR, IV scale is 1gee Q12 => scale factor is lshift by 4
 * - with 16gee FSR, IV scale is 1gee Q11 => scale factor is lshift by 5
 * - with 32gee FSR, IV scale is 1gee Q10 => scale factor is lshift by 6
 * * given the definition of ACCEL_CONFIG0_SREG_UI_ACCEL_FS_SEL:
 *  4 : 2 gee
 *  3 : 4 gee
 *  2 : 8 gee
 *  1 : 16 gee
 *  0 : 32 gee
 * then it comes that scale factor is:
 *   6 - ACCEL_CONFIG0_SREG_UI_ACCEL_FS_SEL
 * * @return scale factor on success, negative value on error
 */
void icm566xx_data_inj_set_accel_scale_factor(inv_imu_device_t *self);

/** @brief Returns scale factor to be applied on IV accel data to feed edmp with expected data
 * format. Scale factor is returned as an arithmetic shift to be applied on each of the 3 accel
 * data.
 * @warning Scale factor depends on accel FSR so it is processed based on actual FSR value at the
 * time this function is called. Thus, this function should be called after accel FSR is set.
 *
 * @return scale factor on success, negative value on error
 */
int icm566xx_data_inj_get_accel_scale_factor(void);

int icm566xx_data_inj_force_ped_odr_25Hz(inv_imu_device_t *self);

/** @brief Get current time.
 *  @param[in] self  Pointer to device.
 *  @return       The current time in us.
 */
extern uint64_t inv_imu_data_inj_get_time_us(inv_imu_device_t *self);

#endif /* _INV_IMU_DATA_INJ_H_ */
