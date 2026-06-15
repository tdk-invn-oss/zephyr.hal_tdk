/*
 *
 * Copyright (c) [2017] by InvenSense, Inc.
 * * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_apex.h"

int inv_imu_edmp_get_apex_parameters(inv_imu_device_t *s, inv_imu_edmp_apex_parameters_t *p)
{
	int status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	/* Pedometer */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_AMP_TH, (uint8_t *)&p->ped_amp_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_CNT_TH, (uint8_t *)&p->ped_step_cnt_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_DET_TH, (uint8_t *)&p->ped_step_det_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_SB_TIMER_TH, (uint8_t *)&p->ped_sb_timer_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_HI_EN_TH, (uint8_t *)&p->ped_hi_en_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_SENSITIVITY_MODE,
					 (uint8_t *)&p->ped_sensitivity_mode);
	status |=
		INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_LOW_EN_AMP_TH, (uint8_t *)&p->ped_low_en_amp_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_PED, (uint8_t *)&p->ped_odr);

	/* Tilt */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TILT_WAIT_TIME, (uint8_t *)&p->tilt_wait_time);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TILT_ANGLE_TH, (uint8_t *)&p->tilt_angle);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_TILT, (uint8_t *)&p->tilt_odr);

	/* SMD */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SMD_SENSITIVITY, (uint8_t *)&p->smd_sensitivity);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_SMD, (uint8_t *)&p->smd_odr);

	/* R2W */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_SLEEP_TIME_OUT,
					 (uint8_t *)&p->r2w_sleep_time_out);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_SLEEP_GESTURE_DELAY,
					 (uint8_t *)&p->r2w_sleep_gesture_delay);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOUNTING_MATRIX,
					 (uint8_t *)&p->r2w_mounting_matrix);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_GRAVITY_FILTER_GAIN,
					 (uint8_t *)&p->r2w_gravity_filter_gain);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_ANGLE_COSINE,
					 (uint8_t *)&p->r2w_motion_th_angle_cosine);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_TIMER_FAST,
					 (uint8_t *)&p->r2w_motion_th_timer_fast);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_TIMER_SLOW,
					 (uint8_t *)&p->r2w_motion_th_timer_slow);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOTION_PREV_GRAVITY_TIMEOUT,
					 (uint8_t *)&p->r2w_motion_prev_gravity_timeout);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_LAST_GRAVITY_MOTION_TIMER,
					 (uint8_t *)&p->r2w_last_gravity_motion_timer);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_LAST_GRAVITY_TIMEOUT,
					 (uint8_t *)&p->r2w_last_gravity_timeout);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_GESTURE_VALIDITY_TIMEOUT,
					 (uint8_t *)&p->r2w_gesture_validity_timeout);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_R2W, (uint8_t *)&p->r2w_odr);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_SAMPLING_PERIOD,
					 (uint8_t *)&p->r2w_sampling_period);

	/* Freefall */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH, (uint8_t *)&p->lowg_peak_th);
	status |=
		INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH_HYST, (uint8_t *)&p->lowg_peak_th_hyst);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_TIME_TH, (uint8_t *)&p->lowg_time_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH, (uint8_t *)&p->highg_peak_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH_HYST,
					 (uint8_t *)&p->highg_peak_th_hyst);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_TIME_TH, (uint8_t *)&p->highg_time_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_MIN_DURATION, (uint8_t *)&p->ff_min_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_MAX_DURATION, (uint8_t *)&p->ff_max_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DEBOUNCE_DURATION,
					 (uint8_t *)&p->ff_debounce_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_FF, (uint8_t *)&p->ff_odr);

	/* Tap */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MIN_JERK, (uint8_t *)&p->tap_min_jerk);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TMAX, (uint8_t *)&p->tap_tmax);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TMIN, (uint8_t *)&p->tap_tmin);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MAX_PEAK_TOL, (uint8_t *)&p->tap_max_peak_tol);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_SMUDGE_REJECT_THR,
					 (uint8_t *)&p->tap_smudge_reject_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TAVG, (uint8_t *)&p->tap_tavg);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MAX_TAP, (uint8_t *)&p->tap_max_tap);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MIN_TAP, (uint8_t *)&p->tap_min_tap);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MAX_ENERGY_PRIMARY_AXIS,
					 (uint8_t *)&p->tap_max_energy_primary);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MAX_ENERGY_SECONDARY_AXIS,
					 (uint8_t *)&p->tap_max_energy_secondary);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_AXIS_SELECT_MASK,
					 (uint8_t *)&p->tap_axis_select_mask);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_TAP, (uint8_t *)&p->tap_odr);

	/* B2S */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_MOUNTING_MATRIX,
					 (uint8_t *)&p->b2s_mounting_matrix);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_DEV_NORM_MAX, (uint8_t *)&p->b2s_DevNormMax);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SIN_LIMIT, (uint8_t *)&p->b2s_SinLimit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_FAST_MOTION_AGE_LIMIT,
					 (uint8_t *)&p->b2s_FastMotionAgeLimit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_FAST_LIMIT, (uint8_t *)&p->b2s_FastLimit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_FAST_MOTION_TIME_LIMIT,
					 (uint8_t *)&p->b2s_FastMotionTimeLimit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_AGE_LIMIT,
					 (uint8_t *)&p->b2s_BringToSeeAgeLimit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_STATIC_LIMIT, (uint8_t *)&p->b2s_StaticLimit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_THR_COS_ANGLE, (uint8_t *)&p->b2s_ThrCosAng);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_REVB2S_LATENCY_THR,
					 (uint8_t *)&p->b2s_RevB2sLatencyTh);

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_B2S, (uint8_t *)&p->b2s_odr);

	/* shake */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_THR_STATIC,
					 (uint8_t *)&p->shake_thresh_static);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_MIN_STATIC_DURATION,
					 (uint8_t *)&p->shake_minimal_static_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_THR_MOTION,
					 (uint8_t *)&p->shake_thresh_motion);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_THR_SHAKE_X,
					 (uint8_t *)&p->shake_thresh_shake_x);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_THR_SHAKE_Y,
					 (uint8_t *)&p->shake_thresh_shake_y);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_THR_SHAKE_Z,
					 (uint8_t *)&p->shake_thresh_shake_z);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_MIN_SHAKE_DURATION,
					 (uint8_t *)&p->shake_minimal_shake_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_MAX_SHAKE_DURATION,
					 (uint8_t *)&p->shake_maximal_shake_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_INTERNAL_DURATION,
					 (uint8_t *)&p->shake_internal_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_AXIS_DETECTION_FACTOR,
					 (uint8_t *)&p->shake_axis_detect_factor);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SHAKE_AXIS_MAJORITY_TH,
					 (uint8_t *)&p->shake_axis_majority_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_SHAKE, (uint8_t *)&p->shake_odr);

	/* noMotion */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_NOMOTION_THR_STATIC,
					 (uint8_t *)&p->nomotion_thresh_static);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_NOMOTION_MIN_STATIC_DURATION,
					 (uint8_t *)&p->nomotion_minimal_static_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_NOMOTION_THR_MOTION,
					 (uint8_t *)&p->nomotion_thresh_motion);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_NOMOTION, (uint8_t *)&p->nomotion_odr);

	/* Flat */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FLAT_WAIT_TIME, (uint8_t *)&p->flat_wait_time);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FLAT_COS_FLAT_ANGLE_TH,
					 (uint8_t *)&p->flat_cos_flat_angle_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FLAT_REF_AXIS_SELECTION,
					 (uint8_t *)&p->flat_ref_axis_selection);
	status |=
		INV_IMU_READ_EDMP_SRAM(s, EDMP_FLAT_ISSYMETRICAL, (uint8_t *)&p->flat_isSymetrical);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FLAT_THREE_AXIS_CONV_TIME,
					 (uint8_t *)&p->flat_three_axis_conv_time);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FLAT_REF_AXIS_X, (uint8_t *)&p->flat_ref_axis_x);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FLAT_REF_AXIS_Y, (uint8_t *)&p->flat_ref_axis_y);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FLAT_REF_AXIS_Z, (uint8_t *)&p->flat_ref_axis_z);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_FLAT, (uint8_t *)&p->flat_odr);

	/* Three Axis */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ODR_3AXIS, (uint8_t *)&p->three_axis_odr);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ACCEL_ONLY_GAIN_FAST_CONV,
					 (uint8_t *)&p->three_axis_accel_only_gain);

	/* Power save */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_POWER_SAVE_TIME, (uint8_t *)&p->power_save_time);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	p->power_save_en = edmp_apex_en1.power_save_en ? INV_IMU_ENABLE : INV_IMU_DISABLE;

	return status;
}

int inv_imu_edmp_set_apex_parameters(inv_imu_device_t *s, const inv_imu_edmp_apex_parameters_t *p)
{
	int status = INV_IMU_OK;
	edmp_apex_enx_t cfg;

#if 0
	/* DMP cannot be configured if it is running, hence make sure all APEX algorithms are off */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 2, (uint8_t *)&cfg);
	if (cfg.edmp_apex_en0.pedo_en || cfg.edmp_apex_en0.tilt_en || cfg.edmp_apex_en0.ff_en ||
	    cfg.edmp_apex_en0.smd_en || cfg.edmp_apex_en0.tap_en || cfg.edmp_apex_en0.r2w_en || cfg.edmp_apex_en0.shake_en || cfg.edmp_apex_en0.nomotion_en ||
		cfg.edmp_apex_en1.flat_en)
		return INV_IMU_ERROR;
#endif
	/* Pedometer */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_AMP_TH, (uint8_t *)&p->ped_amp_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_STEP_CNT_TH, (uint8_t *)&p->ped_step_cnt_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_PREV_STEP_CNT_TH,
					  (uint8_t *)&p->ped_step_cnt_th); /* same as step_cnt_th */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_STEP_DET_TH, (uint8_t *)&p->ped_step_det_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_SB_TIMER_TH, (uint8_t *)&p->ped_sb_timer_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_HI_EN_TH, (uint8_t *)&p->ped_hi_en_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_SENSITIVITY_MODE,
					  (uint8_t *)&p->ped_sensitivity_mode);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_LOW_EN_AMP_TH,
					  (uint8_t *)&p->ped_low_en_amp_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_PED, (uint8_t *)&p->ped_odr);

	/* Tilt */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TILT_WAIT_TIME, (uint8_t *)&p->tilt_wait_time);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TILT_ANGLE_TH, (uint8_t *)&p->tilt_angle);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_TILT, (uint8_t *)&p->tilt_odr);

	/* SMD */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SMD_SENSITIVITY, (uint8_t *)&p->smd_sensitivity);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_SMD, (uint8_t *)&p->smd_odr);

	/* R2W */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_SLEEP_TIME_OUT,
					  (uint8_t *)&p->r2w_sleep_time_out);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_SLEEP_GESTURE_DELAY,
					  (uint8_t *)&p->r2w_sleep_gesture_delay);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOUNTING_MATRIX,
					  (uint8_t *)&p->r2w_mounting_matrix);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_GRAVITY_FILTER_GAIN,
					  (uint8_t *)&p->r2w_gravity_filter_gain);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_ANGLE_COSINE,
					  (uint8_t *)&p->r2w_motion_th_angle_cosine);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_TIMER_FAST,
					  (uint8_t *)&p->r2w_motion_th_timer_fast);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_TIMER_SLOW,
					  (uint8_t *)&p->r2w_motion_th_timer_slow);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOTION_PREV_GRAVITY_TIMEOUT,
					  (uint8_t *)&p->r2w_motion_prev_gravity_timeout);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_LAST_GRAVITY_MOTION_TIMER,
					  (uint8_t *)&p->r2w_last_gravity_motion_timer);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_LAST_GRAVITY_TIMEOUT,
					  (uint8_t *)&p->r2w_last_gravity_timeout);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_GESTURE_VALIDITY_TIMEOUT,
					  (uint8_t *)&p->r2w_gesture_validity_timeout);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_R2W, (uint8_t *)&p->r2w_odr);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_SAMPLING_PERIOD,
					  (uint8_t *)&p->r2w_sampling_period);

	/* Free Fall */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH, (uint8_t *)&p->lowg_peak_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH_HYST,
					  (uint8_t *)&p->lowg_peak_th_hyst);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_TIME_TH, (uint8_t *)&p->lowg_time_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH, (uint8_t *)&p->highg_peak_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH_HYST,
					  (uint8_t *)&p->highg_peak_th_hyst);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_TIME_TH, (uint8_t *)&p->highg_time_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_MIN_DURATION, (uint8_t *)&p->ff_min_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_MAX_DURATION, (uint8_t *)&p->ff_max_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_DEBOUNCE_DURATION,
					  (uint8_t *)&p->ff_debounce_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_FF, (uint8_t *)&p->ff_odr);

	/* Tap */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MIN_JERK, (uint8_t *)&p->tap_min_jerk);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TMAX, (uint8_t *)&p->tap_tmax);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TMIN, (uint8_t *)&p->tap_tmin);
	status |=
		INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MAX_PEAK_TOL, (uint8_t *)&p->tap_max_peak_tol);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_SMUDGE_REJECT_THR,
					  (uint8_t *)&p->tap_smudge_reject_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TAVG, (uint8_t *)&p->tap_tavg);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_TAP, (uint8_t *)&p->tap_odr);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MAX_TAP, (uint8_t *)&p->tap_max_tap);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MIN_TAP, (uint8_t *)&p->tap_min_tap);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MAX_ENERGY_PRIMARY_AXIS,
					  (uint8_t *)&p->tap_max_energy_primary);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MAX_ENERGY_SECONDARY_AXIS,
					  (uint8_t *)&p->tap_max_energy_secondary);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_AXIS_SELECT_MASK,
					  (uint8_t *)&p->tap_axis_select_mask);

	/* B2S */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_MOUNTING_MATRIX,
					  (uint8_t *)&p->b2s_mounting_matrix);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_DEV_NORM_MAX, (uint8_t *)&p->b2s_DevNormMax);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SIN_LIMIT, (uint8_t *)&p->b2s_SinLimit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_FAST_MOTION_AGE_LIMIT,
					  (uint8_t *)&p->b2s_FastMotionAgeLimit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_FAST_LIMIT, (uint8_t *)&p->b2s_FastLimit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_FAST_MOTION_TIME_LIMIT,
					  (uint8_t *)&p->b2s_FastMotionTimeLimit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_AGE_LIMIT,
					  (uint8_t *)&p->b2s_BringToSeeAgeLimit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_STATIC_LIMIT, (uint8_t *)&p->b2s_StaticLimit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_THR_COS_ANGLE, (uint8_t *)&p->b2s_ThrCosAng);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_REVB2S_LATENCY_THR,
					  (uint8_t *)&p->b2s_RevB2sLatencyTh);

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_B2S, (uint8_t *)&p->b2s_odr);

	/* shake */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_THR_STATIC,
					  (uint8_t *)&p->shake_thresh_static);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_MIN_STATIC_DURATION,
					  (uint8_t *)&p->shake_minimal_static_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_THR_MOTION,
					  (uint8_t *)&p->shake_thresh_motion);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_THR_SHAKE_X,
					  (uint8_t *)&p->shake_thresh_shake_x);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_THR_SHAKE_Y,
					  (uint8_t *)&p->shake_thresh_shake_y);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_THR_SHAKE_Z,
					  (uint8_t *)&p->shake_thresh_shake_z);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_MIN_SHAKE_DURATION,
					  (uint8_t *)&p->shake_minimal_shake_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_MAX_SHAKE_DURATION,
					  (uint8_t *)&p->shake_maximal_shake_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_INTERNAL_DURATION,
					  (uint8_t *)&p->shake_internal_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_AXIS_DETECTION_FACTOR,
					  (uint8_t *)&p->shake_axis_detect_factor);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SHAKE_AXIS_MAJORITY_TH,
					  (uint8_t *)&p->shake_axis_majority_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_SHAKE, (uint8_t *)&p->shake_odr);

	/* noMotion */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_NOMOTION_THR_STATIC,
					  (uint8_t *)&p->nomotion_thresh_static);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_NOMOTION_MIN_STATIC_DURATION,
					  (uint8_t *)&p->nomotion_minimal_static_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_NOMOTION_THR_MOTION,
					  (uint8_t *)&p->nomotion_thresh_motion);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_NOMOTION, (uint8_t *)&p->nomotion_odr);

	/* Flat */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FLAT_WAIT_TIME, (uint8_t *)&p->flat_wait_time);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FLAT_COS_FLAT_ANGLE_TH,
					  (uint8_t *)&p->flat_cos_flat_angle_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FLAT_REF_AXIS_SELECTION,
					  (uint8_t *)&p->flat_ref_axis_selection);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FLAT_ISSYMETRICAL,
					  (uint8_t *)&p->flat_isSymetrical);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FLAT_THREE_AXIS_CONV_TIME,
					  (uint8_t *)&p->flat_three_axis_conv_time);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FLAT_REF_AXIS_X, (uint8_t *)&p->flat_ref_axis_x);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FLAT_REF_AXIS_Y, (uint8_t *)&p->flat_ref_axis_y);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FLAT_REF_AXIS_Z, (uint8_t *)&p->flat_ref_axis_z);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_FLAT, (uint8_t *)&p->flat_odr);

	/* Three Axis */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_3AXIS, (uint8_t *)&p->three_axis_odr);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ACCEL_ONLY_GAIN_FAST_CONV,
					  (uint8_t *)&p->three_axis_accel_only_gain);

	/* Power save */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_POWER_SAVE_TIME, (uint8_t *)&p->power_save_time);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&cfg.edmp_apex_en1);
	cfg.edmp_apex_en1.power_save_en = p->power_save_en;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&cfg.edmp_apex_en1);

	return status;
}

int inv_imu_edmp_get_config_int_apex(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it)
{
	int status = INV_IMU_OK;
	int_apex_configx_t cfg;

	status |= inv_imu_read_reg(s, INT_APEX_CONFIG0, 3, (uint8_t *)&cfg);
	/* INT_APEX_CONFIG0 */
	it->INV_TAP = !cfg.int_apex_config0.int_status_mask_pin_tap_detect;
	it->INV_HIGHG = !cfg.int_apex_config0.int_status_mask_pin_high_g_det;
	it->INV_LOWG = !cfg.int_apex_config0.int_status_mask_pin_low_g_det;
	it->INV_TILT_DET = !cfg.int_apex_config0.int_status_mask_pin_tilt_det;
	it->INV_STEP_CNT_OVFL = !cfg.int_apex_config0.int_status_mask_pin_step_cnt_ovfl;
	it->INV_STEP_DET = !cfg.int_apex_config0.int_status_mask_pin_step_det;
	it->INV_FF = !cfg.int_apex_config0.int_status_mask_pin_ff_det;
	it->INV_R2W = !cfg.int_apex_config0.int_status_mask_pin_r2w_wake_det;

	/* INT_APEX_CONFIG1 */
	it->INV_R2W_SLEEP = !cfg.int_apex_config1.int_status_mask_pin_r2w_sleep_det;
	it->INV_SMD = !cfg.int_apex_config1.int_status_mask_pin_smd_det;

	it->INV_SELF_TEST = !cfg.int_apex_config1.int_status_mask_pin_selftest_done;
	it->INV_SEC_AUTH = !cfg.int_apex_config1.int_status_mask_pin_sa_done;
	it->INV_BUMP =
		!cfg.int_apex_config1
			 .int_status_mask_pin_sa_done; /*bump shares the int status with sa_done */
	it->INV_B2S = !cfg.int_apex_config1.int_status_mask_pin_b2s_det;
	it->INV_B2S_REV = !cfg.int_apex_config1.int_status_mask_pin_revb2s_det;
	it->INV_SHAKE = !cfg.int_apex_config1.int_status_mask_pin_shake_det;

	/* INT_APEX_CONFIG2 */
	it->INV_MOTION = !cfg.int_apex_config2.int_status_mask_pin_motion_det;
	it->INV_NO_MOTION = !cfg.int_apex_config2.int_status_mask_pin_nomotion_det;
	it->INV_FLAT = !cfg.int_apex_config2.int_status_mask_pin_flat_det;
	it->INV_NO_FLAT = !cfg.int_apex_config2.int_status_mask_pin_noflat_det;

	return status;
}

int inv_imu_edmp_set_config_int_apex(inv_imu_device_t *s, const inv_imu_edmp_int_state_t *it)
{
	int status = INV_IMU_OK;
	int_apex_configx_t cfg = {0};

	/* INT_APEX_CONFIG0 */
	cfg.int_apex_config0.int_status_mask_pin_tap_detect = !it->INV_TAP;
	cfg.int_apex_config0.int_status_mask_pin_high_g_det = !it->INV_HIGHG;
	cfg.int_apex_config0.int_status_mask_pin_low_g_det = !it->INV_LOWG;
	cfg.int_apex_config0.int_status_mask_pin_tilt_det = !it->INV_TILT_DET;
	cfg.int_apex_config0.int_status_mask_pin_step_cnt_ovfl = !it->INV_STEP_CNT_OVFL;
	cfg.int_apex_config0.int_status_mask_pin_step_det = !it->INV_STEP_DET;
	cfg.int_apex_config0.int_status_mask_pin_ff_det = !it->INV_FF;
	cfg.int_apex_config0.int_status_mask_pin_r2w_wake_det = !it->INV_R2W;

	/* INT_APEX_CONFIG1 */
	cfg.int_apex_config1.int_status_mask_pin_r2w_sleep_det = !it->INV_R2W_SLEEP;

	cfg.int_apex_config1.int_status_mask_pin_smd_det = !it->INV_SMD;
	cfg.int_apex_config1.int_status_mask_pin_selftest_done = !it->INV_SELF_TEST;
	cfg.int_apex_config1.int_status_mask_pin_sa_done = !it->INV_SEC_AUTH;
	cfg.int_apex_config1.int_status_mask_pin_sa_done =
		!it->INV_BUMP; /*Bump shares the interrupt bit with sa_done */
	cfg.int_apex_config1.int_status_mask_pin_b2s_det = !it->INV_B2S;
	cfg.int_apex_config1.int_status_mask_pin_revb2s_det = !it->INV_B2S_REV;
	cfg.int_apex_config1.int_status_mask_pin_shake_det = !it->INV_SHAKE;
	/* INT_APEX_CONFIG2 */
	cfg.int_apex_config2.int_status_mask_pin_motion_det = !it->INV_MOTION;
	cfg.int_apex_config2.int_status_mask_pin_nomotion_det = !it->INV_NO_MOTION;
	cfg.int_apex_config2.int_status_mask_pin_flat_det = !it->INV_FLAT;
	cfg.int_apex_config2.int_status_mask_pin_noflat_det = !it->INV_NO_FLAT;

	status |= inv_imu_write_reg(s, INT_APEX_CONFIG0, 3, (uint8_t *)&cfg);

	return status;
}

int inv_imu_edmp_enable_pedometer(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	/* Make sure pedometer is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.pedo_en) {
		return status;
	}

	apex_buffer_mgmt_t apex_buffer_mgmt;

	status |= inv_imu_read_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);
	apex_buffer_mgmt.step_count_host_rptr = 2;
	apex_buffer_mgmt.step_count_edmp_wptr = 0;
	status |= inv_imu_write_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);

	/* Enable Pedometer */
	edmp_apex_en0.pedo_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_pedometer(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.pedo_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_pedometer_odr(inv_imu_device_t *s, uint16_t ped_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_PED, (uint8_t *)&ped_odr);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_3AXIS, (uint8_t *)&ped_odr);
	return status;
}

int inv_imu_edmp_enable_smd(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.smd_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_smd(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.smd_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_smd_odr(inv_imu_device_t *s, uint16_t smd_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_SMD, (uint8_t *)&smd_odr);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_PED, (uint8_t *)&smd_odr);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_3AXIS, (uint8_t *)&smd_odr);
	return status;
}

int inv_imu_edmp_enable_tilt(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tilt_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_tilt(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tilt_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_tilt_odr(inv_imu_device_t *s, uint16_t tilt_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_TILT, (uint8_t *)&tilt_odr);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_3AXIS, (uint8_t *)&tilt_odr);
	return status;
}

int inv_imu_edmp_enable_r2w(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.r2w_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_r2w(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.r2w_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_r2w_odr(inv_imu_device_t *s, uint16_t r2w_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_R2W, (uint8_t *)&r2w_odr);
	return status;
}

int inv_imu_edmp_enable_b2s(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	fifo_config0_t fifo_config0;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&fifo_config0);
	if (fifo_config0.fifo_depth == FIFO_CONFIG0_FIFO_DEPTH_MAX) {
		return INV_IMU_ERROR;
	}

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.b2s_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_disable_b2s(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.b2s_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_set_b2s_odr(inv_imu_device_t *s, uint16_t b2s_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_B2S, (uint8_t *)&b2s_odr);
	return status;
}

int inv_imu_edmp_enable_tap(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tap_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_tap(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tap_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_tap_odr(inv_imu_device_t *s, uint16_t smd_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_TAP, (uint8_t *)&smd_odr);

	return status;
}

int inv_imu_edmp_enable_ff(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	/* Make sure freefall is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.ff_en) {
		return status;
	}

	apex_buffer_mgmt_t apex_buffer_mgmt;

	status |= inv_imu_read_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);
	apex_buffer_mgmt.ff_duration_host_rptr = 0;
	apex_buffer_mgmt.ff_duration_edmp_wptr = 0;
	status |= inv_imu_write_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);

	/* Enable freefall */
	edmp_apex_en0.ff_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_ff(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.ff_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_ff_odr(inv_imu_device_t *s, uint16_t ff_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_FF, (uint8_t *)&ff_odr);

	return status;
}

int inv_imu_edmp_enable_soft_iron_cor(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	return status;
}

int inv_imu_edmp_disable_soft_iron_cor(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	return status;
}

int inv_imu_edmp_enable_shake(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.shake_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_shake(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.shake_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_shake_odr(inv_imu_device_t *s, uint16_t shake_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_SHAKE, (uint8_t *)&shake_odr);
	return status;
}

int inv_imu_edmp_enable_noMotion(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.nomotion_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_noMotion(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.nomotion_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_noMotion_odr(inv_imu_device_t *s, uint16_t noMotion_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_NOMOTION, (uint8_t *)&noMotion_odr);
	return status;
}

int inv_imu_edmp_enable_flat(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.flat_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_disable_flat(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.flat_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_set_flat_odr(inv_imu_device_t *s, uint16_t flat_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_FLAT, (uint8_t *)&flat_odr);
	return status;
}

int inv_imu_edmp_set_compass_frequency(inv_imu_device_t *s,
				       const dmp_ext_sen_odr_cfg_ext_odr_t frequency)
{
	int status = INV_IMU_OK;

	return status;
}

int inv_imu_edmp_enable_compass_es(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	return status;
}

int inv_imu_edmp_disable_compass_es(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	return status;
}

int inv_imu_edmp_get_int_apex_status(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it)
{
	int status = INV_IMU_OK;
	int_apex_statusx_t st;

	/* Read APEX interrupt status */
	status |= inv_imu_read_reg(s, INT_APEX_STATUS0, 3, (uint8_t *)&st);

	it->INV_TAP = st.int_apex_status0.int_status_tap_detect;
	it->INV_HIGHG = st.int_apex_status0.int_status_high_g_det;
	it->INV_LOWG = st.int_apex_status0.int_status_low_g_det;
	it->INV_TILT_DET = st.int_apex_status0.int_status_tilt_det;
	it->INV_STEP_CNT_OVFL = st.int_apex_status0.int_status_step_cnt_ovfl;
	it->INV_STEP_DET = st.int_apex_status0.int_status_step_det;
	it->INV_FF = st.int_apex_status0.int_status_ff_det;
	it->INV_R2W = st.int_apex_status0.int_status_r2w_wake_det;
	it->INV_SMD = st.int_apex_status1.int_status_smd_det;
	it->INV_R2W_SLEEP = st.int_apex_status1.int_status_r2w_sleep_det;
	it->INV_SELF_TEST = st.int_apex_status1.int_status_selftest_done;
	it->INV_SEC_AUTH = st.int_apex_status1.int_status_sa_done;
	it->INV_BUMP =
		st.int_apex_status1.int_status_sa_done; /*bump shares the int status with sa_done */
	it->INV_B2S = st.int_apex_status1.int_status_b2s_det;
	it->INV_B2S_REV = st.int_apex_status1.int_status_revb2s_det;
	it->INV_SHAKE = st.int_apex_status1.int_status_shake_det;
	it->INV_NO_MOTION = st.int_apex_status2.int_status_nomotion_det;
	it->INV_MOTION = st.int_apex_status2.int_status_motion_det;
	it->INV_NO_FLAT = st.int_apex_status2.int_status_noflat_det;
	it->INV_FLAT = st.int_apex_status2.int_status_flat_det;

	return status;
}

int inv_imu_edmp_get_pedometer_data(inv_imu_device_t *s, inv_imu_edmp_pedometer_data_t *ped_data)
{
	int status = INV_IMU_OK;
	uint8_t data[2];
	uint8_t retry = 0;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_ACTIVITY_CLASS, data);
	ped_data->activity_class = (inv_imu_edmp_activity_class_t)data[0];

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_CADENCE, data);
	ped_data->step_cadence = data[0];

	/*
	 * Always read BUF2 as we forced a buffer full
	 * configuration in `inv_imu_edmp_init_apex` function.
	 */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_CNT_BUF2, data);

	/*
	 * Read value multiple times in case the buffer was written while we were reading it.
	 * If we read the same value twice consecutively, it means we have a proper value.
	 */
	while (status == INV_IMU_OK) {
		uint8_t data_verif[2];
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_CNT_BUF2, data_verif);

		if ((data[0] == data_verif[0]) && (data[1] == data_verif[1])) {
			break;
		}

		data[0] = data_verif[0];
		data[1] = data_verif[1];

		retry++;

		if (retry > 5) {
			return INV_IMU_ERROR;
		}
	}
	ped_data->step_cnt = data[1] << 8 | data[0];

	return status;
}

int inv_imu_edmp_get_ff_data(inv_imu_device_t *s, uint16_t *freefall_duration)
{
	int status = INV_IMU_OK;
	uint8_t data[2];
	apex_buffer_mgmt_t apex_buffer_mgmt;
	uint8_t edmp_wptr, host_rptr;

	status |= inv_imu_read_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);
	host_rptr = apex_buffer_mgmt.ff_duration_host_rptr;
	edmp_wptr = apex_buffer_mgmt.ff_duration_edmp_wptr;

	if (host_rptr == edmp_wptr) {
		return INV_IMU_ERROR_EDMP_BUF_EMPTY; /* No data. */
	}

	if ((host_rptr & 0x1) == 0) {
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DURATION_BUF1, data);
	} else {
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DURATION_BUF2, data);
	}

	host_rptr = (host_rptr + 1) & 0x03;

	apex_buffer_mgmt.ff_duration_host_rptr = host_rptr;
	status |= inv_imu_write_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);

	*freefall_duration = (data[1] << 8) | data[0];

	return status;
}

int inv_imu_edmp_get_tap_data(inv_imu_device_t *s, inv_imu_edmp_tap_data_t *data)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_NUM, (uint8_t *)&(data->num));
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_AXIS, (uint8_t *)&(data->axis));
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_DIR, (uint8_t *)&(data->direction));
	if (data->num == INV_IMU_EDMP_TAP_DOUBLE) {
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_DOUBLE_TAP_TIMING,
						 (uint8_t *)&(data->double_tap_timing));
	} else {
		data->double_tap_timing = 0;
	}

	if (data->num == INV_IMU_EDMP_TAP_TRIPLE) {
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TRIPLE_TAP_TIMING,
						 (uint8_t *)&(data->triple_tap_timing));
	} else {
		data->triple_tap_timing = 0;
	}

	return status;
}

int inv_imu_edmp_get_nomotion_data(inv_imu_device_t *s, inv_imu_edmp_nomtion_data_t *data)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_NOMOTION_MAJORITY_AXIS,
					 (uint8_t *)&(data->majority_axis));
	status |=
		INV_IMU_READ_EDMP_SRAM(s, EDMP_NOMOTION_SIGN_ANGLE, (uint8_t *)&(data->sign_angle));
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_NOMOTION_TILT_ANGLE_RAW_FORMAT,
					 (uint8_t *)&(data->tilt_angle_raw_format));

	return status;
}
