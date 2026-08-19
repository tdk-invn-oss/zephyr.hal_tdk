/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#ifndef __INV_IMU_EDMP_MEMMAP_H__
#define __INV_IMU_EDMP_MEMMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * power_save_time
 *
 * The time (in sample number) after which eDMP goes in power save mode.
 * Unit: time in sample number
 * Range: [0 - 2147483648]
 * Default value: 6400
 * Format: int32_t
 */
#define EDMP_POWER_SAVE_TIME      0xa8
#define EDMP_POWER_SAVE_TIME_SIZE 4

/*
 * odr_3axis
 *
 * Specify ODR of Three-Axis
 * Unit: Hertz
 * Default value: 50
 * Format: int16_t
 */
#define EDMP_ODR_3AXIS      0x30
#define EDMP_ODR_3AXIS_SIZE 2

/*
 * threeAxis_accel_only_gain
 *
 * Gain value to handle smoothing of 3axis reactivity
 * Unit: N/A
 * Default: 67108864 = q30(0.0625)
 * Format: int32_t
 */
#define EDMP_THREEAXIS_ACCEL_ONLY_GAIN      0x1ec
#define EDMP_THREEAXIS_ACCEL_ONLY_GAIN_SIZE 4

/*
 * odr_ped
 *
 * Specify ODR of feature Pedometer
 * Unit: Hertz
 * Default value: 50
 * Format: int16_t
 */
#define EDMP_ODR_PED      0x34
#define EDMP_ODR_PED_SIZE 2

/*
 * ped_amp_th
 *
 * Threshold on the absolute value of the filtered accelerometer, above which a valid step will be
 * counted. Low values increase detection sensitivity: reduce miss-detection. High values reduce
 * detection sensitivity: reduce false-positive. Unit: LSB with 1 LSB = 1 g/2^25 Range: [1006632 -
 * 3019898] Default value: 2080374 (corresponding to about 62mgee) Format: int32_t
 */
#define EDMP_PED_AMP_TH      0x388
#define EDMP_PED_AMP_TH_SIZE 4

/*
 * ped_step_cnt_th
 *
 * Minimum number of steps needed to be buffered before starting to increment step count in real
 * time. Low values reduce latency but increase false positives. High values increase step count
 * accuracy but increase latency Unit: number of steps Range: [0-15] Default value: 5 Format:
 * int16_t
 */
#define EDMP_PED_STEP_CNT_TH      0x37e
#define EDMP_PED_STEP_CNT_TH_SIZE 2

/*
 * ped_prev_step_cnt_th
 *
 * Intermediate number of steps needed to be buffered waiting the internal counting reached the
 * ped_step_cnt_th (when ped_prev_step_cnt_th is lower than ped_step_cnt_th). Note: as soon as
 * ped_step_cnt_th is reached ped_prev_step_cnt_th value is set to ped_step_cnt_th. Unit: number of
 * steps Range: [0-15] Default value: 5 Recommendation: set same value as ped_step_cnt_th. Format:
 * int16_t
 */
#define EDMP_PED_PREV_STEP_CNT_TH      0x2fc
#define EDMP_PED_PREV_STEP_CNT_TH_SIZE 2

/*
 * ped_step_det_th
 *
 * Minimum number of steps that must be detected before step event is signaled.
 * Low values reduce latency but increase false positives.
 * High values increase step event validity but increase latency.
 * Unit: number of steps
 * Range: [0-7]
 * Default value: 2
 * Format: int16_t
 */
#define EDMP_PED_STEP_DET_TH      0x380
#define EDMP_PED_STEP_DET_TH_SIZE 2

/*
 * ped_sb_timer_th
 *
 * Maximum permitted time between two consecutive steps.
 * While in the step buffer state, the step buffer count resets to 0 if a new step isn't detected
 * for this amount of time (user is considered to have "stopped walking") Unit: time in samples
 * number Range: [0 - 225] Default value: 150 for ODR = 50 Hz (corresponding to 3s) Recommendation:
 * it is linked to stop and go use case to avoid seeing the impact of pedo_step_cnt_thr and
 * pedo_step_det_thr Format: int16_t
 */
#define EDMP_PED_SB_TIMER_TH      0x382
#define EDMP_PED_SB_TIMER_TH_SIZE 2

/*
 * ped_hi_en_th
 *
 * Threshold on signal's energy in frequency band corresponding to walking/running.
 * Unit: LSB with 1 LSB = 1 g/2^25
 * Range: [2949120 - 5210112]
 * Default value: 3506176 (corresponding to about 104mgee)
 * Recommendation: if some walking steps are classified as running, higher the value, if some
 * running steps are classified as walking, lower the value Format: int32_t
 */
#define EDMP_PED_HI_EN_TH      0x38c
#define EDMP_PED_HI_EN_TH_SIZE 4

/*
 * ped_sensitivity_mode
 *
 * Pedometer sensitivity mode.
 * Slow walk mode improves slow walk detection (less than 1 Hz) but the number of false positives
 * may increase Unit: no unit Range: 0: Normal 1: Slow walk Default value: 0 Format: uint8_t
 */
#define EDMP_PED_SENSITIVITY_MODE      0x390
#define EDMP_PED_SENSITIVITY_MODE_SIZE 1

/*
 * ped_low_en_amp_th
 *
 * Threshold on signal amplitude on filtered accelerometer to identify a valid step in slow walk
 * mode. Used to increase step detection for slow walk use case only. Unit: LSB with 1 LSB = 1
 * g/2^25 Range: [1006632 - 3523215] Default value: 2684354 (corresponding to about 80mgee) Format:
 * int32_t
 */
#define EDMP_PED_LOW_EN_AMP_TH      0x384
#define EDMP_PED_LOW_EN_AMP_TH_SIZE 4

/*
 * ped_step_cnt_buf1
 *
 * Number of steps done since the last init of the pedometer feature.
 * Filled in alternatively with ped_step_cnt_buf2.
 * Unit: number of steps
 * Format: uint16_t
 */
#define EDMP_PED_STEP_CNT_BUF1      0x7c
#define EDMP_PED_STEP_CNT_BUF1_SIZE 2

/*
 * ped_step_cnt_buf2
 *
 * Number of steps done since the last init of the pedometer feature.
 * Filled in alternatively with ped_step_cnt_buf1.
 * Unit: number of steps
 * Format: uint16_t
 */
#define EDMP_PED_STEP_CNT_BUF2      0x7e
#define EDMP_PED_STEP_CNT_BUF2_SIZE 2

/*
 * ped_step_cadence
 *
 * Instant step cadence measured by the algorithm
 * Unit: 4*number of samples between two consecutive steps.
 * Cadency (step/s) = (ped_step_cadence / 4) / (pedometer_ODR).
 * Format: uint8_t
 */
#define EDMP_PED_STEP_CADENCE      0x79
#define EDMP_PED_STEP_CADENCE_SIZE 1

/*
 * ped_activity_class
 *
 * Activity classification of step detected
 * Unit: Enum: unknown (0), walk(1), run(2)
 * Format: uint8_t
 */
#define EDMP_PED_ACTIVITY_CLASS      0x7a
#define EDMP_PED_ACTIVITY_CLASS_SIZE 1

/*
 * odr_tilt
 *
 * Specify ODR of feature Tilt
 * Unit: Hertz
 * Default value: 50
 * Format: int16_t
 */
#define EDMP_ODR_TILT      0x32
#define EDMP_ODR_TILT_SIZE 2

/*
 * tilt_wait_time
 *
 * Minimum duration for which the device should be tilted before signaling event.
 * Unit: in sample number - ODR dependent
 * Default value: 200 (corresponding to 4s at 50Hz)
 * Range: [25 - 1000]
 * Format: int16_t
 */
#define EDMP_TILT_WAIT_TIME      0x214
#define EDMP_TILT_WAIT_TIME_SIZE 2

/*
 * tilt_angle_th
 *
 * The tilt angle threshold to trig tilt event
 * Unit: cosine value of angle in q30
 * Default value: 879557810 (corresponding to 35 degree)
 * Format: int32_t
 */
#define EDMP_TILT_ANGLE_TH      0x218
#define EDMP_TILT_ANGLE_TH_SIZE 4

/*
 * tilt_reset_en
 *
 * Set 1 to reset tilt prior to any further tilt processing on next sensor data
 * Format: uint8_t
 */
#define EDMP_TILT_RESET_EN      0x63
#define EDMP_TILT_RESET_EN_SIZE 1

/*
 * quat_reset_en
 *
 * Set 1 to force reset 3-axis quaternion when next tilt reset is done
 * This is applicable only if tilt_reset_en is also set to 1
 * Format: uint8_t
 */
#define EDMP_QUAT_RESET_EN      0x62
#define EDMP_QUAT_RESET_EN_SIZE 1

/*
 * odr_smd
 *
 * Specify ODR of feature Significant-Motion-Detection
 * Unit: Hertz
 * Default value: 50
 * Format: int16_t
 */
#define EDMP_ODR_SMD      0x36
#define EDMP_ODR_SMD_SIZE 2

/*
 * smd_sensitivity
 *
 * Parameter to tune SMD algorithm robustness to rejection, ranging from 0 to 4 (values higher than
 * 4 are reserved). Low values increase detection rate but increase false positives. High values
 * reduce false positives but reduce detection rate (especially for transport use cases). Unit:
 * integer Range: [0 - 4] Default value: 0 Format: uint8_t
 */
#define EDMP_SMD_SENSITIVITY      0x3aa
#define EDMP_SMD_SENSITIVITY_SIZE 1

/*
 * odr_r2w
 *
 * Specify ODR of feature Raise-to-Wake
 * Unit: Hertz
 * Default value: 25
 * Format: int16_t
 */
#define EDMP_ODR_R2W      0x2e
#define EDMP_ODR_R2W_SIZE 2

/*
 * r2w_sleep_time_out
 *
 * Defines the duration after wake event to report sleep event regardless if position changed or
 * not. Unit: Time in ms, not ODR dependent Default value: 640 (corresponding to 0.64s) Range: [100
 * - 10000] Format: int32_t
 */
#define EDMP_R2W_SLEEP_TIME_OUT      0x16c
#define EDMP_R2W_SLEEP_TIME_OUT_SIZE 4

/*
 * r2w_sleep_gesture_delay
 *
 * Defines the minimal duration of sleep position before triggering the sleep event
 * Unit: Time in ms, not ODR dependent
 * Default value: 96 (corresponding to 0.096s)
 * Range: [0 - 256]
 * Format: int32_t
 */
#define EDMP_R2W_SLEEP_GESTURE_DELAY      0x170
#define EDMP_R2W_SLEEP_GESTURE_DELAY_SIZE 4

/*
 * r2w_mounting_matrix
 *
 * Mounting matrix to rotate data from chip frame to device frame
 * Range: 3 lower bits only are used [b2 b1 b0]:
 * - when b2 = 1 swap X and Y
 * - when b1 = 1 flip X sign
 * - when b0 = 1 flip Y sign
 * Default value: 0 (corresponding to chip frame aligned with android frame)
 * Format: int32_t
 */
#define EDMP_R2W_MOUNTING_MATRIX      0x174
#define EDMP_R2W_MOUNTING_MATRIX_SIZE 4

/*
 * r2w_sampling_period
 *
 * Sampling period corresponding to sensor ODR
 * Unit: Time in ms
 * Default value is 40 (corresponding to 40ms for 25Hz)
 * Range: [10 - 40]
 * Format: int32_t
 */
#define EDMP_R2W_SAMPLING_PERIOD      0x178
#define EDMP_R2W_SAMPLING_PERIOD_SIZE 4

/*
 * r2w_gravity_filter_gain
 *
 * Gain used to filtered the accelerometer to obtain an estimation of the gravity (low-pass filter),
 * Defined: forgetting factor = Gain * SAMPLING_PERIOD / (40 * 32)
 * Unit: N/A
 * Range: [2-16]
 * Default value: is 8 at 25Hz and 6 for 50 and 100Hz
 * Format: int32_t
 */
#define EDMP_R2W_GRAVITY_FILTER_GAIN      0x17c
#define EDMP_R2W_GRAVITY_FILTER_GAIN_SIZE 4

/*
 * r2w_motion_thr_angle_cosine
 *
 * Set the minimal angle that needed to be applied to the device in order to detect R2W
 * Unit: cosine value of angle in q30
 * Default: 1046221864 (corresponding to an angle of 13 degree)
 * Range: [130856211 - 1069655912] (corresponding to an angle between 5 and 85 degree)
 * Format: int32_t
 */
#define EDMP_R2W_MOTION_THR_ANGLE_COSINE      0x180
#define EDMP_R2W_MOTION_THR_ANGLE_COSINE_SIZE 4

/*
 * r2w_motion_thr_timer_fast
 *
 * Timer relative to the rapidity of the algorithm to trigger wake up when the orientation before
 * motion is Y axis up (with less than 30 degree of inclination) Unit: Time in ms (not ODR
 * dependent) Default value: 240 Range: [100 - 500] Format: int32_t
 */
#define EDMP_R2W_MOTION_THR_TIMER_FAST      0x184
#define EDMP_R2W_MOTION_THR_TIMER_FAST_SIZE 4

/*
 * r2w_motion_thr_timer_slow
 *
 * Timer relative to the rapidity of the algorithm to trigger wake up when the orientation before
 * motion is over 30 degree on the Y axis Unit: Time in ms (not ODR dependent) Default value: 500
 * Range: [240- 1000]
 * Format: int32_t
 */
#define EDMP_R2W_MOTION_THR_TIMER_SLOW      0x188
#define EDMP_R2W_MOTION_THR_TIMER_SLOW_SIZE 4

/*
 * r2w_motion_prev_gravity_timeout
 *
 * Time delay to update internal value of previous gravity when no motion is detected.
 * Longer time enables detection motion during slower gesture.
 * Unit: Time in ms (not ODR dependent)
 * Default value: 300
 * Range: [100- 1000]
 * Format: int32_t
 */
#define EDMP_R2W_MOTION_PREV_GRAVITY_TIMEOUT      0x18c
#define EDMP_R2W_MOTION_PREV_GRAVITY_TIMEOUT_SIZE 4

/*
 * r2w_last_gravity_motion_timer
 *
 * Time delay to update the current gravity estimator when no motion is detected
 * Unit: Time in ms (not ODR dependent)
 * Default value: 480
 * Range: [100- 1000]
 * Format: int32_t
 */
#define EDMP_R2W_LAST_GRAVITY_MOTION_TIMER      0x190
#define EDMP_R2W_LAST_GRAVITY_MOTION_TIMER_SIZE 4

/*
 * r2w_last_gravity_timeout
 *
 * Time delay to update gravity in case motion is detected continuously, forces to update gravity
 * estimator even if the device is not stable Unit: Time in ms (not ODR dependent) Default value:
 * 2600 Range: [1000- 10000] Format: int32_t
 */
#define EDMP_R2W_LAST_GRAVITY_TIMEOUT      0x194
#define EDMP_R2W_LAST_GRAVITY_TIMEOUT_SIZE 4

/*
 * r2w_gesture_validity_timeout
 *
 * Gesture validity timeout, if gesture is not completed in this timeout limit, gesture is invalid
 * Unit: Time in ms (not ODR dependent)
 * Default value: 240
 * Range: [100- 2000]
 * Format: int32_t
 */
#define EDMP_R2W_GESTURE_VALIDITY_TIMEOUT      0x198
#define EDMP_R2W_GESTURE_VALIDITY_TIMEOUT_SIZE 4

/*
 * odr_ff
 *
 * Specify ODR of feature Free-Fall
 * Unit: Hertz
 * Default value: 400
 * Format: int16_t
 */
#define EDMP_ODR_FF      0x2a
#define EDMP_ODR_FF_SIZE 2

/*
 * lowg_peak_th
 *
 * If the sum of absolute accelerometer axes falls below lowg_peak_th, this triggers the start of
 * LowG state Unit: in LSB, with 1 LBS = 1g / 2^12 Default value: 2048 (corresponding to 0.5g)
 * Range: [128 - 4096]
 * Format: int16_t
 */
#define EDMP_LOWG_PEAK_TH      0xda
#define EDMP_LOWG_PEAK_TH_SIZE 2

/*
 * lowg_peak_th_hyst
 *
 * Hysteresis added to the threshold to confirm the triggering of the LowG state
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Default value: 128 (corresponding to 31.2mg)
 * Range: [128 - 1024]
 * Format: int16_t
 */
#define EDMP_LOWG_PEAK_TH_HYST      0xdc
#define EDMP_LOWG_PEAK_TH_HYST_SIZE 2

/*
 * lowg_time_th
 *
 * The number of samples device should stay below (lowg_peak_th + lowg_peak_th_hyst) before LowG
 * state is triggered Unit: in sample number - ODR dependent Default value: 13 (corresponding to
 * 32ms at 400Hz) Range: [1 - 300] Format: int16_t
 */
#define EDMP_LOWG_TIME_TH      0xde
#define EDMP_LOWG_TIME_TH_SIZE 2

/*
 * highg_peak_th
 *
 * Any of the absolute accelerometer axis surpasses high_peak_thres will trigger the start of HighG
 * state Unit: in LSB, with 1 LBS = 1g / 2^12 Default value: 29696 (corresponding to 7.25g) Range:
 * [1024 - 32768]
 */
#define EDMP_HIGHG_PEAK_TH      0xce
#define EDMP_HIGHG_PEAK_TH_SIZE 2

/*
 * highg_peak_th_hyst
 *
 * A hysteresis high_peak_thres_hyst is subtracted from the threshold (high_peak_thres) after the
 * initial threshold is met Unit: in LSB, with 1 LBS = 1g / 2^12 Default value: 640  (corresponding
 * to 0.1563g) Range: [128 - 1024] Format: int16_t
 */
#define EDMP_HIGHG_PEAK_TH_HYST      0xd0
#define EDMP_HIGHG_PEAK_TH_HYST_SIZE 2

/*
 * highg_time_th
 *
 * The number of samples the device should stay above (highg_peak_th - highg_peak_th_hyst) before
 * HighG state is triggered Unit: in sample number - ODR dependent Default value: 1 (corresponding
 * to 2.5ms at 400Hz) Range: [1 - 300] Format: int16_t
 */
#define EDMP_HIGHG_TIME_TH      0xd2
#define EDMP_HIGHG_TIME_TH_SIZE 2

/*
 * ff_min_duration
 *
 * Minimum duration between a LowG and HighG event to trigger freefall. Shorter freefalls are
 * ignored. Unit: in sample number, ODR dependent Default value: 57 (corresponding to 142ms at
 * 400Hz) Range: [4 - 420] For information, it is corresponding to approximately a fall of 10cm
 * Format: int32_t
 */
#define EDMP_FF_MIN_DURATION      0xbc
#define EDMP_FF_MIN_DURATION_SIZE 4

/*
 * ff_max_duration
 *
 * Maximum duration after a LowG event to wait HighG event before reset freefall algorithm. Longer
 * freefalls are ignored. Unit: in sample number, ODR dependent Default value: 285 (corresponding to
 * 712ms at 400Hz) Range: [12 - 1040] For information, it is corresponding to approximately a fall
 * of 2.5 meters Format: int32_t
 */
#define EDMP_FF_MAX_DURATION      0xc0
#define EDMP_FF_MAX_DURATION_SIZE 4

/*
 * ff_debounce_duration
 *
 * Wait duration after freefall event detected (to avoid false freefall event due to bounces).
 * Unit: in sample number, ODR dependent
 * Default value: 800 (corresponding to 2s at 400Hz)
 * Range: [75 - 3000]
 * Format: int32_t
 */
#define EDMP_FF_DEBOUNCE_DURATION      0xc4
#define EDMP_FF_DEBOUNCE_DURATION_SIZE 4

/*
 * ff_duration_buf1
 *
 * Duration of the freefall.
 * Filled in alternatively with ff_duration_buf2.
 * Unit: time in sample number
 * Format: int16_t
 */
#define EDMP_FF_DURATION_BUF1      0x88
#define EDMP_FF_DURATION_BUF1_SIZE 2

/*
 * ff_duration_buf2
 *
 * Duration of the freefall
 * Filled in alternatively with ff_duration_buf1
 * Unit: time in sample number
 * Format: int16_t
 */
#define EDMP_FF_DURATION_BUF2      0x8a
#define EDMP_FF_DURATION_BUF2_SIZE 2

/*
 * odr_tap
 *
 * Specify ODR of feature Tap
 * Unit: Hertz
 * Default value: 400
 * Format: int16_t
 */
#define EDMP_ODR_TAP      0x2c
#define EDMP_ODR_TAP_SIZE 2

/*
 * tap_tmax
 *
 * Time window since the jerk maximum value, used to observe a single-tap, double-tap
 * triple-tap or rejection of the tap event.
 * Unit: in sample number, ODR dependent
 * Default: 198 at 400Hz (corresponding to 0.495s)
 * Format: int16_t
 */
#define EDMP_TAP_TMAX      0xe4
#define EDMP_TAP_TMAX_SIZE 2

/*
 * tap_tmin
 *
 * Single tap window, sub-window within the tap_tmax to detect single-tap event
 * Unit: in sample number, ODR dependent
 * Default value: 66 at 400Hz (corresponding to 0.1650s)
 * Format: uint8_t
 */
#define EDMP_TAP_TMIN      0xe6
#define EDMP_TAP_TMIN_SIZE 1

/*
 * tap_min_jerk
 *
 * The minimal value of jerk to be considered as a tap candidate
 * Unit: in LSB with 1 LSB = 1g / 2^4
 * Default value: 18 (corresponding to 1.125 g)
 * Format: uint8_t
 */
#define EDMP_TAP_MIN_JERK      0xe7
#define EDMP_TAP_MIN_JERK_SIZE 1

/*
 * tap_smudge_reject_thr
 *
 * Maximum time spent over tap_max_peak_tol before rejecting the tap candidate, at the end of
 * tap_tmin window. If internal counter of number of samples over the smudge threshold is below the
 * tap_smudge_reject_th, single-tap is detected. Unit: in sample number, ODR dependent Default
 * value: 34 at 400Hz, representing 0.085 s Format: uint8_t
 */
#define EDMP_TAP_SMUDGE_REJECT_THR      0xe8
#define EDMP_TAP_SMUDGE_REJECT_THR_SIZE 1

/*
 * tap_max_peak_tol
 *
 * Maximum peak tolerance is the percentage of pulse amplitude to get the
 * smudge threshold for rejection
 * Unit: N/A
 * Range: [1 (12.5%) - 2 (25.0%) - 3 (37.5%) - 4 (50%)]
 * Default value: 2 (corresponding to 25 %)
 * Format: uint8_t
 */
#define EDMP_TAP_MAX_PEAK_TOL      0xe9
#define EDMP_TAP_MAX_PEAK_TOL_SIZE 1

/*
 * tap_tavg
 *
 * Energy measurement window size to determine the tap axis associated with the 1st tap
 * Unit: in sample number, ODR dependent
 * Use value limited to [1 - 2 - 4 - 8]
 * Default value: 8
 * Format: uint8_t
 */
#define EDMP_TAP_TAVG      0xea
#define EDMP_TAP_TAVG_SIZE 1

/*
 * tap_max_tap
 *
 * Maximal number of tap quantity detected to be valid
 * Unit: in enum
 * Use value limited to [1 - 2 - 3] (corresponding to [single, double, triple])
 * Default value: 2
 * Format: uint8_t
 */
#define EDMP_TAP_MAX_TAP      0xec
#define EDMP_TAP_MAX_TAP_SIZE 1

/*
 * tap_min_tap
 *
 * Minimal number of tap quantity detected to be valid
 * Unit: in enum
 * Use value limited to [1 - 2 - 3] (corresponding to [single, double, triple])
 * Default value: 2
 * Format: uint8_t
 */
#define EDMP_TAP_MIN_TAP      0xed
#define EDMP_TAP_MIN_TAP_SIZE 1

/*
 * tap_axis_select_mask
 *
 * Mask used to select the reported axis and direction of the tap feature
 * Unit: bit mask: 1 for +X, 2 for -X, 4 for +Y, 8 for -Y, 16 for +Z and 32 for -Z
 * Range: [1 - 63] * Default value: 32 (algorithm will detect only -Z tap, also called backtap)
 * Note: Use value 63 to detect tap for all axes and all directions
 * Format: uint8_t
 */
#define EDMP_TAP_AXIS_SELECT_MASK      0xf2
#define EDMP_TAP_AXIS_SELECT_MASK_SIZE 1

/*
 * tap_max_energy_primary_axis
 *
 * Threshold for maximal energy on tap primary axis which is used to check validity of the tap
 * detection Unit: N/A Range: [1000 - 10000] Default value: 3000 Note: Use 0 to disable this check
 * and it is only valid for tap directional rejection Format: int16_t
 */
#define EDMP_TAP_MAX_ENERGY_PRIMARY_AXIS      0xee
#define EDMP_TAP_MAX_ENERGY_PRIMARY_AXIS_SIZE 2

/*
 * tap_max_energy_secondary_axis
 *
 * Threshold for maximal energy on tap secondary axis which is used to check validity of the tap
 * detection Unit: N/A Range: [200 - 1000] Default value: 500 Note: Use 0 to disable this check and
 * it is only valid for tap directional rejection Format: int16_t
 */
#define EDMP_TAP_MAX_ENERGY_SECONDARY_AXIS      0xf0
#define EDMP_TAP_MAX_ENERGY_SECONDARY_AXIS_SIZE 2

/*
 * tap_num
 *
 * Indicate the type of the last tap event
 * Unit: N/A
 * Range: [0: no tap, 1: single tap, 2: double tap, 3: triple tap]
 * Format: uint8_t
 */
#define EDMP_TAP_NUM      0x8e
#define EDMP_TAP_NUM_SIZE 1

/*
 * tap_axis
 *
 * Indicate the axis of the tap in the device frame
 * Unit: N/A
 * Range: [0: ax, 1: ay, 2: az]
 * Format: uint8_t
 */
#define EDMP_TAP_AXIS      0x8f
#define EDMP_TAP_AXIS_SIZE 1

/*
 * tap_dir
 *
 * Indicate the direction of the tap axis in the device frame
 * Unit: N/A
 * Range: [0: positive, 1: negative]
 * Format: uint8_t
 */
#define EDMP_TAP_DIR      0x90
#define EDMP_TAP_DIR_SIZE 1

/*
 * double_tap_timing
 *
 * Number of samples between the first and second pulse detected in the detection window, only valid
 * for double tap events Unit: time in sample number Format: uint8_t
 */
#define EDMP_DOUBLE_TAP_TIMING      0x91
#define EDMP_DOUBLE_TAP_TIMING_SIZE 1

/*
 * triple_tap_timing
 *
 * Number of samples between the first and third pulse detected in the detection window, only valid
 * for triple tap events Unit: time in sample number Format: uint8_t
 */
#define EDMP_TRIPLE_TAP_TIMING      0x92
#define EDMP_TRIPLE_TAP_TIMING_SIZE 1

/*
 * odr_b2s
 *
 * Specify ODR of feature bring-to-see
 * Unit: Hertz
 * Default value: 50
 * Format: int16_t
 */
#define EDMP_ODR_B2S      0x38
#define EDMP_ODR_B2S_SIZE 2

/*
 * b2s_mounting_matrix
 *
 * Mounting matrix to apply to the accelerometer data before bring-to-see computation, as a bit-mask
 * combination of operations. 3bits are used [b2 b1 b0]: Range: 3 lower bits only are used [b2 b1
 * b0]:
 * - when b2 = 1 swap X and Y, flip Z
 * - when b1 = 1 flip X sign, flip Z
 * - when b0 = 1 flip Y sign, flip Z
 * Default: 0 being identity matrix
 * Format: uint8_t
 */
#define EDMP_B2S_MOUNTING_MATRIX      0x42c
#define EDMP_B2S_MOUNTING_MATRIX_SIZE 1

/*
 * b2s_dev_norm_max
 *
 * Hysteresis added or removed to norm estimate and Y axis constrains value for reverse bring-to-see
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Range: [1 - 2048]
 * Default value: 700 (corresponding to 0.1709g)
 * Format: int32_t
 */
#define EDMP_B2S_DEV_NORM_MAX      0x430
#define EDMP_B2S_DEV_NORM_MAX_SIZE 4

/*
 * b2s_sin_limit
 *
 * Maximum threshold on absolute value of X axis in bring-to-see position.
 * Link to the sinus value of inclination angle on X axis
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Range: [300 - 3000]
 * Default value: 2048 (corresponding to 30 degree)
 * Format: int32_t
 */
#define EDMP_B2S_SIN_LIMIT      0x434
#define EDMP_B2S_SIN_LIMIT_SIZE 4

/*
 * b2s_fast_motion_age_limit
 *
 * Time limit between last "Fast motion" and bring-to-see position
 * Unit: in sample number - ODR dependent
 * Default value: 20 (corresponding to 400ms at 50Hz)
 * Format: int32_t
 */
#define EDMP_B2S_FAST_MOTION_AGE_LIMIT      0x438
#define EDMP_B2S_FAST_MOTION_AGE_LIMIT_SIZE 4

/*
 * b2s_fast_limit
 *
 * Threshold of minimal motion to be detected as "Fast motion"
 * Unit: no unit, filtered data
 * Range: [300 - 3000]
 * Default value: 200
 * Format: int32_t
 */
#define EDMP_B2S_FAST_LIMIT      0x43c
#define EDMP_B2S_FAST_LIMIT_SIZE 4

/*
 * b2s_fast_motion_time_limit
 *
 * Minimum time where the criterion is above the threshold  to be classified as "Fast motion"
 * Unit: in sample number - ODR dependent
 * Default value: 4 (corresponding to 80ms at 50Hz)
 * Format: int32_t
 */
#define EDMP_B2S_FAST_MOTION_TIME_LIMIT      0x440
#define EDMP_B2S_FAST_MOTION_TIME_LIMIT_SIZE 4

/*
 * b2s_age_limit
 *
 * Minimum time between two bring-to-see events
 * Unit: in sample number - ODR dependent
 * Default value: 50 (corresponding to 1s at 50Hz)
 * Format: int32_t
 */
#define EDMP_B2S_AGE_LIMIT      0x444
#define EDMP_B2S_AGE_LIMIT_SIZE 4

/*
 * b2s_static_limit
 *
 * Threshold to determine static phase required after the gesture bring-to-see to validate it
 * Unit: no unit, filtered data
 * Default value: 1400
 * Format: int32_t
 */
#define EDMP_B2S_STATIC_LIMIT      0x448
#define EDMP_B2S_STATIC_LIMIT_SIZE 4

/*
 * b2s_thr_cos_angle
 *
 * Reverse bring-to-see threshold, condition satisfied when moving away from bring-to-see
 * orientation position by more than threshold angle corresponding to the cosine angle Unit: cosine
 * value of angle in q30 Default value: 1057429273 (corresponding to 10 degree) Format: int32_t
 */
#define EDMP_B2S_THR_COS_ANGLE      0x44c
#define EDMP_B2S_THR_COS_ANGLE_SIZE 4

/*
 * b2s_revb2s_latency_thr
 *
 * Condition of reverse bring-to-see should be maintained at least during b2s_revb2s_latency_thr
 * Unit: in sample number - ODR dependent
 * Default value: 25 (corresponding to 0.5s at 50Hz)
 * Format: int32_t
 */
#define EDMP_B2S_REVB2S_LATENCY_THR      0x450
#define EDMP_B2S_REVB2S_LATENCY_THR_SIZE 4

/*
 * odr_noMotion
 *
 * Specify ODR of feature No-Motion
 * Unit: Hertz
 * Default value: 50
 * Format: int16_t
 */
#define EDMP_ODR_NOMOTION      0x3c
#define EDMP_ODR_NOMOTION_SIZE 2

/*
 * nomotion_thr_static
 *
 * Amplitude threshold of the static criteria for each axis
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Default value: 80 (corresponding to 19 mg - relative to sensor noise)
 * Format: int16_t
 */
#define EDMP_NOMOTION_THR_STATIC      0x522
#define EDMP_NOMOTION_THR_STATIC_SIZE 2

/*
 * nomotion_min_static_duration
 *
 * Minimal duration of static detection before triggering the status no motion
 * Unit: in sample number - ODR dependent
 * Default value: 150 (corresponding to 3s at 50Hz)
 * Format: int16_t
 */
#define EDMP_NOMOTION_MIN_STATIC_DURATION      0x524
#define EDMP_NOMOTION_MIN_STATIC_DURATION_SIZE 2

/*
 * nomotion_thr_motion
 *
 * Amplitude threshold of the motion criteria for each axis
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Default value: 819 (corresponding to 200mg)
 * Format: int16_t
 */
#define EDMP_NOMOTION_THR_MOTION      0x526
#define EDMP_NOMOTION_THR_MOTION_SIZE 2

/*
 * noMotion_majority_axis
 *
 * Value representing which axis has the majority of nomotion detection: * 0: -x, 1: -y, 2: -z, 3:
 * N/A, 4: +x, 5: +y, 6: +z Format: uint8_t
 */
#define EDMP_NOMOTION_MAJORITY_AXIS      0x95
#define EDMP_NOMOTION_MAJORITY_AXIS_SIZE 1

/*
 * noMotion_sign_angle
 *
 * Sign to be applied to tilt angle to determined if the device point up or down:
 * -1: negative, 0: N/A, 1: positive
 * Format: int8_t
 */
#define EDMP_NOMOTION_SIGN_ANGLE      0x96
#define EDMP_NOMOTION_SIGN_ANGLE_SIZE 1

/*
 * noMotion_x_axis_ref
 *
 * x axis reference in q12 of the static position
 * Format: int16_t
 */
#define EDMP_NOMOTION_X_AXIS_REF      0x98
#define EDMP_NOMOTION_X_AXIS_REF_SIZE 2

/*
 * noMotion_y_axis_ref
 *
 * y axis reference in q12 of the static position
 * Format: int16_t
 */
#define EDMP_NOMOTION_Y_AXIS_REF      0x9a
#define EDMP_NOMOTION_Y_AXIS_REF_SIZE 2

/*
 * noMotion_tilt_angle_raw_format
 *
 * Tilt angle in raw format, to be convert in degree by using this formula:
 * sign_angle*57.2958f*acosf(tilt_angle_raw_format/4096.0f)
 * Format: int16_t
 */
#define EDMP_NOMOTION_TILT_ANGLE_RAW_FORMAT      0x9c
#define EDMP_NOMOTION_TILT_ANGLE_RAW_FORMAT_SIZE 2

/*
 * odr_shake
 *
 * Specify ODR of feature Shake
 * Unit: Hertz
 * Default value: 50
 * Format: int16_t
 */
#define EDMP_ODR_SHAKE      0x3a
#define EDMP_ODR_SHAKE_SIZE 2

/*
 * shake_axis_majority_th
 *
 * Axis detection threshold added before comparison between two axis to select the majority axis in
 * done using this formula `majority_axis_nb > minority_axis_nb + shake_axis_majority_th` Unit: no
 * unit, integer value Default value: is 1 Format: int16_t
 */
#define EDMP_SHAKE_AXIS_MAJORITY_TH      0x500
#define EDMP_SHAKE_AXIS_MAJORITY_TH_SIZE 2

/*
 * shake_thr_static
 *
 * Threshold under which the internal detection of static phase is done, common to all axes.
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Default value: 2000 (corresponding to 488mg)
 * Recommendation: can be set lower to be more restrictive on static situation,
 * can be set higher to allow the static detection when user in movement (like medium walking,
 * running) Format: int16_t
 */
#define EDMP_SHAKE_THR_STATIC      0x502
#define EDMP_SHAKE_THR_STATIC_SIZE 2

/*
 * shake_min_static_duration
 *
 * Time threshold to define the minimal duration of static phase to be valid
 * Unit: in sample number, warning ODR dependency
 * Default value: 25 (corresponding to 0.5s at 50Hz)
 * Recommendation: use range value between [0.5s - 1.5s]
 * Format: int16_t
 */
#define EDMP_SHAKE_MIN_STATIC_DURATION      0x504
#define EDMP_SHAKE_MIN_STATIC_DURATION_SIZE 2

/*
 * shake_thr_motion
 *
 * Threshold to get out of the static phase and detect any motion (not specific to shake), common to
 * all axes. Unit: in LSB, with 1 LBS = 1g / 2^12 Default value: 2400 (corresponding to 585mg)
 * Recommendation: can be set lower to be more restrictive on motion situation,
 * can be set higher to allow keep in the static phase when user in movement (like medium walking,
 * running) Format: int16_t
 */
#define EDMP_SHAKE_THR_MOTION      0x506
#define EDMP_SHAKE_THR_MOTION_SIZE 2

/*
 * shake_thr_shake_x
 *
 * Threshold to be reached to classify x axis as shake, it is linked to
 * the amplitude of the signal on x axis during the shake estimation window
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Default value: 2400 (corresponding to 585mg)
 * Recommendation: use lower value to detect shake with lower intensity
 * Format: int16_t
 */
#define EDMP_SHAKE_THR_SHAKE_X      0x508
#define EDMP_SHAKE_THR_SHAKE_X_SIZE 2

/*
 * shake_thr_shake_y
 *
 * Threshold to be reached to classify y axis as shake, it is linked to
 * the amplitude of the signal on y axis during the shake estimation window
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Default value: 2400 (corresponding to 585mg)
 * Recommendation: use lower value to detect shake with lower intensity
 * Format: int16_t
 */
#define EDMP_SHAKE_THR_SHAKE_Y      0x50a
#define EDMP_SHAKE_THR_SHAKE_Y_SIZE 2

/*
 * shake_thr_shake_z
 *
 * Threshold to be reached to classify z axis as shake, it is linked to
 * the amplitude of the signal on z axis during the shake estimation window
 * Unit: in LSB, with 1 LBS = 1g / 2^12
 * Default value: 2400 (corresponding to 585mg)
 * Recommendation: use lower value to detect shake with lower intensity
 * Format: int16_t
 */
#define EDMP_SHAKE_THR_SHAKE_Z      0x50c
#define EDMP_SHAKE_THR_SHAKE_Z_SIZE 2

/*
 * shake_min_shake_duration
 *
 * Time threshold to define the minimal duration of shake to be valid
 * Unit: in sample number, warning ODR dependency
 * Default value: 24 (corresponding to 480ms at 50Hz)
 * Recommendation: set the value as a multiple of internal_duration parameter
 * Format: int16_t
 */
#define EDMP_SHAKE_MIN_SHAKE_DURATION      0x50e
#define EDMP_SHAKE_MIN_SHAKE_DURATION_SIZE 2

/*
 * shake_max_shake_duration
 *
 * Time threshold to define the maximal duration of shake to be valid
 * Unit: in sample number, warning ODR dependency
 * Default value: 72 (corresponding to 1.44s at 50Hz)
 * Recommendation: set the value as a multiple of internal_duration parameter
 * Format: int16_t
 */
#define EDMP_SHAKE_MAX_SHAKE_DURATION      0x510
#define EDMP_SHAKE_MAX_SHAKE_DURATION_SIZE 2

/*
 * shake_internal_duration
 *
 * Internal time window for feature extraction to classify shake in progress once motion is
 * internally detected Unit: in sample number, warning ODR dependency Default value: 8,
 * corresponding to 160ms at 50Hz Recommendation: use range [8-16] Format: int16_t
 */
#define EDMP_SHAKE_INTERNAL_DURATION      0x512
#define EDMP_SHAKE_INTERNAL_DURATION_SIZE 2

/*
 * shake_axis_detection_factor
 *
 * Axis detection factor to consider an axis as major one
 * Formula is major_axis_feature*axis_detection_factor > minor_axis_feature
 * Unit: no unit, with a q12 coding value
 * Default value: 2048 (corresponding to value 0.5)
 * Format: int16_t
 */
#define EDMP_SHAKE_AXIS_DETECTION_FACTOR      0x514
#define EDMP_SHAKE_AXIS_DETECTION_FACTOR_SIZE 2

/*
 * shake_outaxis
 *
 * Axis on which the shake as been identified:
 * 0: undefined axis,
 * 1: X axis shake,
 * 2: Y axis shake,
 * 3: Z axis shake.
 * Format: uint8_t
 */
#define EDMP_SHAKE_OUTAXIS      0x93
#define EDMP_SHAKE_OUTAXIS_SIZE 1

/*
 * odr_flat
 *
 * Specify ODR of feature Flat
 * Unit: Hertz
 * Default value: 50
 * Format: int16_t
 */
#define EDMP_ODR_FLAT      0x3e
#define EDMP_ODR_FLAT_SIZE 2

/*
 * accel_only_gain_fast_conv
 *
 * Gain value to handle smoothing of 3axis reactivity
 * Format: q30 in int32_t
 * Unit: N/A
 * Default: 100663296 (corresponding to 0.09375)
 */
#define EDMP_ACCEL_ONLY_GAIN_FAST_CONV      0x554
#define EDMP_ACCEL_ONLY_GAIN_FAST_CONV_SIZE 4

/*
 * flat_ref_axis_selection
 *
 * Specify which axis to consider as reference and set ref_axis in consequence:
 * * 0 -> X axis (1,0,0)
 * * 1 -> Y axis (0,1,0)
 * * 2 -> Z axis (0,0,1)
 * * 3 -> Advanced (a 3d vector must be provided by the user)
 * Default value: 2 (Z axis corresponding to vertical in android reference)
 * Format: int16_t
 */
#define EDMP_FLAT_REF_AXIS_SELECTION      0x574
#define EDMP_FLAT_REF_AXIS_SELECTION_SIZE 2

/*
 * flat_isSymmetrical
 *
 * Flag defining whether behaviour is symmetrical or asymmetrical around flat_ref_axis_selection
 * Unit: enum
 * Range: [0: asymmetrical, 1: symmetrical]
 * Default value: 1, symmetrical
 * Format: int16_t
 */
#define EDMP_FLAT_ISSYMMETRICAL      0x576
#define EDMP_FLAT_ISSYMMETRICAL_SIZE 2

/*
 * flat_three_axis_conv_time
 *
 * Specify convergence time (in samples) for convergence of orientation estimation
 * Unit: in sample number - ODR dependent
 * Algorithm detection is disable during initialization
 * Default value: 50 (corresponding to 1s at 50Hz)
 * Format: int16_t
 */
#define EDMP_FLAT_THREE_AXIS_CONV_TIME      0x578
#define EDMP_FLAT_THREE_AXIS_CONV_TIME_SIZE 2

/*
 * flat_wait_time
 *
 * Wait time after the flat/no flat angle is reached and maintained to trigger the flat/no flat
 * event Unit: in sample number - ODR dependent Default value: 10 (corresponding to 200ms at 50Hz)
 * Format: int16_t
 */
#define EDMP_FLAT_WAIT_TIME      0x57a
#define EDMP_FLAT_WAIT_TIME_SIZE 2

/*
 * flat_cos_flat_angle_th
 *
 * Tilt angle with reference vector to detect flat / no flat condition
 * Unit: cosine value of angle in q30
 * Default value: 1057429273 (corresponding to 10 degree)
 * Format: int32_t
 */
#define EDMP_FLAT_COS_FLAT_ANGLE_TH      0x57c
#define EDMP_FLAT_COS_FLAT_ANGLE_TH_SIZE 4

/*
 * flat_ref_axis_x
 *
 * Reference value for x axis
 * Unit: in LSB, with 1 LBS = 1g / 2^30
 * Default value: 0
 * Format: int32_t
 */
#define EDMP_FLAT_REF_AXIS_X      0x580
#define EDMP_FLAT_REF_AXIS_X_SIZE 4

/*
 * flat_ref_axis_y
 *
 * Reference value for y axis
 * Unit: in LSB, with 1 LBS = 1g / 2^30
 * Default value: 0
 * Format: int32_t
 */
#define EDMP_FLAT_REF_AXIS_Y      0x584
#define EDMP_FLAT_REF_AXIS_Y_SIZE 4

/*
 * flat_ref_axis_z
 *
 * Reference value for z axis
 * Unit: in LSB, with 1 LBS = 1g / 2^30
 * Default value: 1073741824 (corresponding to 1g)
 * Format: int32_t
 */
#define EDMP_FLAT_REF_AXIS_Z      0x588
#define EDMP_FLAT_REF_AXIS_Z_SIZE 4

/*
 * stc_results
 *
 * bit0:    accel_x_self_test_result -- 0/1: passed/failed
 * bit1:    accel_y_self_test_result -- 0/1: passed/failed
 * bit2:    accel_z_self_test_result -- 0/1: passed/failed
 * bit3:    gyro_x_self_test_result  -- 0/1: passed/failed
 * bit4:    gyro_y_self_test_result  -- 0/1: passed/failed
 * bit5:    gyro_z_self_test_result  -- 0/1: passed/failed
 * bit6~7:  self_test_status         -- 0/1/2: done/in-progress/error
 * bit8~31: unused
 */
#define EDMP_STC_RESULTS      0x44
#define EDMP_STC_RESULTS_SIZE 4

/*
 * stc_configParams
 *
 * bit0:     self_test_init_en         -- 0/1: Disabled/Enabled
 * bit1:     accel_self_test_en        -- 0/1: Disabled/Enabled
 * bit2:     gyro_self_test_en         -- 0/1: Disabled/Enabled
 * bit3~6:   unused
 * bit7~9:   self_test_average_time    -- 0/1/2/3/4/5: 10/20/40/80/160/320 ms -> 8/16/32/64/128/256
 * samples @ 800Hz bit10~12: accel_self_test_threshold -- 0/1/2/3/4/5/6/7: 5/10/15/20/25/30/40/50
 * bit13~15: gyro_self_test_threshold  -- 0/1/2/3/4/5/6/7: 5/10/15/20/25/30/40/50
 * bit16~31: unused
 */
#define EDMP_STC_CONFIGPARAMS      0x38
#define EDMP_STC_CONFIGPARAMS_SIZE 4

/*
 * stc_patch_en
 *
 * bit0:    patch_en_self_test_accel_phase1 -- 0/1: disabled/enabled
 * bit1:    patch_en_self_test_accel_phase2 -- 0/1: disabled/enabled
 * bit2:    patch_en_self_test_gyro1_phase1 -- 0/1: disabled/enabled
 * bit3:    patch_en_self_test_gyro1_phase2 -- 0/1: disabled/enabled
 * bit4~31: unused
 */
#define EDMP_STC_PATCH_EN      0x3c
#define EDMP_STC_PATCH_EN_SIZE 4

/*
 * stc_debug_en
 *
 * Debug capability of self-test feature. Must be set to 0 at anytime when self-test is requested.
 */
#define EDMP_STC_DEBUG_EN      0x40
#define EDMP_STC_DEBUG_EN_SIZE 4

#ifdef __cplusplus
}
#endif

#endif /* __INV_IMU_EDMP_MEMMAP_H__ */
