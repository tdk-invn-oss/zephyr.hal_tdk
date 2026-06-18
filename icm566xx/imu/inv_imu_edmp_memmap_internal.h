/*
 * Copyright (c) 2021 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#ifndef __INV_IMU_EDMP_MEMMAP_INTERNAL_H__
#define __INV_IMU_EDMP_MEMMAP_INTERNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * apex_dataMem
 *
 */
#define EDMP_APEX_DATAMEM      0x0
#define EDMP_APEX_DATAMEM_SIZE 1452

/*
 * apex_otpData
 *
 */
#define EDMP_APEX_OTPDATA      0x0
#define EDMP_APEX_OTPDATA_SIZE 38

/*
 * apex_otpData_gyro_x_str_ft
 *
 */
#define EDMP_APEX_OTPDATA_GYRO_X_STR_FT      0x0
#define EDMP_APEX_OTPDATA_GYRO_X_STR_FT_SIZE 2

/*
 * apex_otpData_gyro_y_str_ft
 *
 */
#define EDMP_APEX_OTPDATA_GYRO_Y_STR_FT      0x2
#define EDMP_APEX_OTPDATA_GYRO_Y_STR_FT_SIZE 2

/*
 * apex_otpData_gyro_z_str_ft
 *
 */
#define EDMP_APEX_OTPDATA_GYRO_Z_STR_FT      0x4
#define EDMP_APEX_OTPDATA_GYRO_Z_STR_FT_SIZE 2

/*
 * apex_otpData_gyro_x_cmos_gain_ft
 *
 */
#define EDMP_APEX_OTPDATA_GYRO_X_CMOS_GAIN_FT      0x6
#define EDMP_APEX_OTPDATA_GYRO_X_CMOS_GAIN_FT_SIZE 2

/*
 * apex_otpData_gyro_y_cmos_gain_ft
 *
 */
#define EDMP_APEX_OTPDATA_GYRO_Y_CMOS_GAIN_FT      0x8
#define EDMP_APEX_OTPDATA_GYRO_Y_CMOS_GAIN_FT_SIZE 2

/*
 * apex_otpData_gyro_z_cmos_gain_ft
 *
 */
#define EDMP_APEX_OTPDATA_GYRO_Z_CMOS_GAIN_FT      0xa
#define EDMP_APEX_OTPDATA_GYRO_Z_CMOS_GAIN_FT_SIZE 2

/*
 * apex_otpData_accel_x_str_ft
 *
 */
#define EDMP_APEX_OTPDATA_ACCEL_X_STR_FT      0xc
#define EDMP_APEX_OTPDATA_ACCEL_X_STR_FT_SIZE 2

/*
 * apex_otpData_accel_y_str_ft
 *
 */
#define EDMP_APEX_OTPDATA_ACCEL_Y_STR_FT      0xe
#define EDMP_APEX_OTPDATA_ACCEL_Y_STR_FT_SIZE 2

/*
 * apex_otpData_accel_z_str_ft
 *
 */
#define EDMP_APEX_OTPDATA_ACCEL_Z_STR_FT      0x10
#define EDMP_APEX_OTPDATA_ACCEL_Z_STR_FT_SIZE 2

/*
 * apex_otpData_accel_x_sc_nout_ft
 *
 */
#define EDMP_APEX_OTPDATA_ACCEL_X_SC_NOUT_FT      0x12
#define EDMP_APEX_OTPDATA_ACCEL_X_SC_NOUT_FT_SIZE 2

/*
 * apex_otpData_accel_y_sc_nout_ft
 *
 */
#define EDMP_APEX_OTPDATA_ACCEL_Y_SC_NOUT_FT      0x14
#define EDMP_APEX_OTPDATA_ACCEL_Y_SC_NOUT_FT_SIZE 2

/*
 * apex_otpData_accel_z_sc_nout_ft
 *
 */
#define EDMP_APEX_OTPDATA_ACCEL_Z_SC_NOUT_FT      0x16
#define EDMP_APEX_OTPDATA_ACCEL_Z_SC_NOUT_FT_SIZE 2

/*
 * apex_otpData_self_cal_ft_ver
 *
 */
#define EDMP_APEX_OTPDATA_SELF_CAL_FT_VER      0x18
#define EDMP_APEX_OTPDATA_SELF_CAL_FT_VER_SIZE 1

/*
 * apex_otpData_manu_id
 *
 */
#define EDMP_APEX_OTPDATA_MANU_ID      0x19
#define EDMP_APEX_OTPDATA_MANU_ID_SIZE 1

/*
 * apex_otpData_chip_id
 *
 */
#define EDMP_APEX_OTPDATA_CHIP_ID      0x1a
#define EDMP_APEX_OTPDATA_CHIP_ID_SIZE 1

/*
 * apex_otpData_rev_id
 *
 */
#define EDMP_APEX_OTPDATA_REV_ID      0x1b
#define EDMP_APEX_OTPDATA_REV_ID_SIZE 1

/*
 * apex_otpData_asic_wafer_id
 *
 */
#define EDMP_APEX_OTPDATA_ASIC_WAFER_ID      0x1c
#define EDMP_APEX_OTPDATA_ASIC_WAFER_ID_SIZE 1

/*
 * apex_otpData_asic_die_id_0
 *
 */
#define EDMP_APEX_OTPDATA_ASIC_DIE_ID_0      0x1d
#define EDMP_APEX_OTPDATA_ASIC_DIE_ID_0_SIZE 1

/*
 * apex_otpData_asic_die_id_1
 *
 */
#define EDMP_APEX_OTPDATA_ASIC_DIE_ID_1      0x1e
#define EDMP_APEX_OTPDATA_ASIC_DIE_ID_1_SIZE 1

/*
 * apex_otpData_mems_wafer_id
 *
 */
#define EDMP_APEX_OTPDATA_MEMS_WAFER_ID      0x1f
#define EDMP_APEX_OTPDATA_MEMS_WAFER_ID_SIZE 1

/*
 * apex_otpData_mems_die_id_0
 *
 */
#define EDMP_APEX_OTPDATA_MEMS_DIE_ID_0      0x20
#define EDMP_APEX_OTPDATA_MEMS_DIE_ID_0_SIZE 1

/*
 * apex_otpData_mems_die_id_1
 *
 */
#define EDMP_APEX_OTPDATA_MEMS_DIE_ID_1      0x21
#define EDMP_APEX_OTPDATA_MEMS_DIE_ID_1_SIZE 1

/*
 * apex_otpData_sw_rcosc_trim
 *
 */
#define EDMP_APEX_OTPDATA_SW_RCOSC_TRIM      0x22
#define EDMP_APEX_OTPDATA_SW_RCOSC_TRIM_SIZE 1

/*
 * apex_otpData_sw_pll_trim
 *
 */
#define EDMP_APEX_OTPDATA_SW_PLL_TRIM      0x23
#define EDMP_APEX_OTPDATA_SW_PLL_TRIM_SIZE 1

/*
 * apex_otpData_sa_key_last_byte
 *
 */
#define EDMP_APEX_OTPDATA_SA_KEY_LAST_BYTE      0x24
#define EDMP_APEX_OTPDATA_SA_KEY_LAST_BYTE_SIZE 1

/*
 * apex_interf
 *
 */
#define EDMP_APEX_INTERF      0x28
#define EDMP_APEX_INTERF_SIZE 120

/*
 * apex_interf_odr_edmp
 *
 */
#define EDMP_APEX_INTERF_ODR_EDMP      0x28
#define EDMP_APEX_INTERF_ODR_EDMP_SIZE 2

/*
 * apex_interf_odr_ff
 *
 */
#define EDMP_APEX_INTERF_ODR_FF      0x2a
#define EDMP_APEX_INTERF_ODR_FF_SIZE 2

/*
 * apex_interf_odr_tap
 *
 */
#define EDMP_APEX_INTERF_ODR_TAP      0x2c
#define EDMP_APEX_INTERF_ODR_TAP_SIZE 2

/*
 * apex_interf_odr_r2w
 *
 */
#define EDMP_APEX_INTERF_ODR_R2W      0x2e
#define EDMP_APEX_INTERF_ODR_R2W_SIZE 2

/*
 * apex_interf_odr_3axis
 *
 */
#define EDMP_APEX_INTERF_ODR_3AXIS      0x30
#define EDMP_APEX_INTERF_ODR_3AXIS_SIZE 2

/*
 * apex_interf_odr_tilt
 *
 */
#define EDMP_APEX_INTERF_ODR_TILT      0x32
#define EDMP_APEX_INTERF_ODR_TILT_SIZE 2

/*
 * apex_interf_odr_ped
 *
 */
#define EDMP_APEX_INTERF_ODR_PED      0x34
#define EDMP_APEX_INTERF_ODR_PED_SIZE 2

/*
 * apex_interf_odr_smd
 *
 */
#define EDMP_APEX_INTERF_ODR_SMD      0x36
#define EDMP_APEX_INTERF_ODR_SMD_SIZE 2

/*
 * apex_interf_odr_b2s
 *
 */
#define EDMP_APEX_INTERF_ODR_B2S      0x38
#define EDMP_APEX_INTERF_ODR_B2S_SIZE 2

/*
 * apex_interf_odr_shake
 *
 */
#define EDMP_APEX_INTERF_ODR_SHAKE      0x3a
#define EDMP_APEX_INTERF_ODR_SHAKE_SIZE 2

/*
 * apex_interf_odr_noMotion
 *
 */
#define EDMP_APEX_INTERF_ODR_NOMOTION      0x3c
#define EDMP_APEX_INTERF_ODR_NOMOTION_SIZE 2

/*
 * apex_interf_odr_flat
 *
 */
#define EDMP_APEX_INTERF_ODR_FLAT      0x3e
#define EDMP_APEX_INTERF_ODR_FLAT_SIZE 2

/*
 * apex_interf_gyro_4000dps
 *
 */
#define EDMP_APEX_INTERF_GYRO_4000DPS      0x40
#define EDMP_APEX_INTERF_GYRO_4000DPS_SIZE 12

/*
 * apex_interf_temperature
 *
 */
#define EDMP_APEX_INTERF_TEMPERATURE      0x4c
#define EDMP_APEX_INTERF_TEMPERATURE_SIZE 4

/*
 * apex_interf_accel_scaled
 *
 */
#define EDMP_APEX_INTERF_ACCEL_SCALED      0x50
#define EDMP_APEX_INTERF_ACCEL_SCALED_SIZE 12

/*
 * apex_interf_racc_q25
 *
 */
#define EDMP_APEX_INTERF_RACC_Q25      0x50
#define EDMP_APEX_INTERF_RACC_Q25_SIZE 12

/*
 * apex_interf_acc_8g
 *
 */
#define EDMP_APEX_INTERF_ACC_8G      0x5c
#define EDMP_APEX_INTERF_ACC_8G_SIZE 6

/*
 * apex_interf_racc_q12
 *
 */
#define EDMP_APEX_INTERF_RACC_Q12      0x5c
#define EDMP_APEX_INTERF_RACC_Q12_SIZE 6

/*
 * apex_interf_quat_reset
 *
 */
#define EDMP_APEX_INTERF_QUAT_RESET      0x62
#define EDMP_APEX_INTERF_QUAT_RESET_SIZE 1

/*
 * apex_interf_tilt_reset
 *
 */
#define EDMP_APEX_INTERF_TILT_RESET      0x63
#define EDMP_APEX_INTERF_TILT_RESET_SIZE 1

/*
 * apex_interf_quat
 *
 */
#define EDMP_APEX_INTERF_QUAT      0x64
#define EDMP_APEX_INTERF_QUAT_SIZE 16

/*
 * apex_interf_accel_wz
 *
 */
#define EDMP_APEX_INTERF_ACCEL_WZ      0x74
#define EDMP_APEX_INTERF_ACCEL_WZ_SIZE 4

/*
 * apex_interf_pedo_outLock
 *
 */
#define EDMP_APEX_INTERF_PEDO_OUTLOCK      0x78
#define EDMP_APEX_INTERF_PEDO_OUTLOCK_SIZE 1

/*
 * apex_interf_cadence_q2_u8
 *
 */
#define EDMP_APEX_INTERF_CADENCE_Q2_U8      0x79
#define EDMP_APEX_INTERF_CADENCE_Q2_U8_SIZE 1

/*
 * apex_interf_activity_class_u2
 *
 */
#define EDMP_APEX_INTERF_ACTIVITY_CLASS_U2      0x7a
#define EDMP_APEX_INTERF_ACTIVITY_CLASS_U2_SIZE 1

/*
 * apex_interf_step_count_edmp_wptr
 *
 */
#define EDMP_APEX_INTERF_STEP_COUNT_EDMP_WPTR      0x7b
#define EDMP_APEX_INTERF_STEP_COUNT_EDMP_WPTR_SIZE 1

/*
 * apex_interf_step_count_u16
 *
 */
#define EDMP_APEX_INTERF_STEP_COUNT_U16      0x7c
#define EDMP_APEX_INTERF_STEP_COUNT_U16_SIZE 2

/*
 * apex_interf_step_count_u16_output_buf2
 *
 */
#define EDMP_APEX_INTERF_STEP_COUNT_U16_OUTPUT_BUF2      0x7e
#define EDMP_APEX_INTERF_STEP_COUNT_U16_OUTPUT_BUF2_SIZE 2

/*
 * apex_interf_ped_ER1_y1
 *
 */
#define EDMP_APEX_INTERF_PED_ER1_Y1      0x80
#define EDMP_APEX_INTERF_PED_ER1_Y1_SIZE 4

/*
 * apex_interf_ped_ER2_y1
 *
 */
#define EDMP_APEX_INTERF_PED_ER2_Y1      0x84
#define EDMP_APEX_INTERF_PED_ER2_Y1_SIZE 4

/*
 * apex_interf_free_fall_duration_samples_output_buf1
 *
 */
#define EDMP_APEX_INTERF_FREE_FALL_DURATION_SAMPLES_OUTPUT_BUF1      0x88
#define EDMP_APEX_INTERF_FREE_FALL_DURATION_SAMPLES_OUTPUT_BUF1_SIZE 2

/*
 * apex_interf_free_fall_duration_samples_output_buf2
 *
 */
#define EDMP_APEX_INTERF_FREE_FALL_DURATION_SAMPLES_OUTPUT_BUF2      0x8a
#define EDMP_APEX_INTERF_FREE_FALL_DURATION_SAMPLES_OUTPUT_BUF2_SIZE 2

/*
 * apex_interf_free_fall_duration_edmp_wptr
 *
 */
#define EDMP_APEX_INTERF_FREE_FALL_DURATION_EDMP_WPTR      0x8c
#define EDMP_APEX_INTERF_FREE_FALL_DURATION_EDMP_WPTR_SIZE 1

/*
 * apex_interf_tap_outLock
 *
 */
#define EDMP_APEX_INTERF_TAP_OUTLOCK      0x8d
#define EDMP_APEX_INTERF_TAP_OUTLOCK_SIZE 1

/*
 * apex_interf_tap
 *
 */
#define EDMP_APEX_INTERF_TAP      0x8e
#define EDMP_APEX_INTERF_TAP_SIZE 1

/*
 * apex_interf_tap_axis
 *
 */
#define EDMP_APEX_INTERF_TAP_AXIS      0x8f
#define EDMP_APEX_INTERF_TAP_AXIS_SIZE 1

/*
 * apex_interf_tap_direction
 *
 */
#define EDMP_APEX_INTERF_TAP_DIRECTION      0x90
#define EDMP_APEX_INTERF_TAP_DIRECTION_SIZE 1

/*
 * apex_interf_double_tap_timing
 *
 */
#define EDMP_APEX_INTERF_DOUBLE_TAP_TIMING      0x91
#define EDMP_APEX_INTERF_DOUBLE_TAP_TIMING_SIZE 1

/*
 * apex_interf_triple_tap_timing
 *
 */
#define EDMP_APEX_INTERF_TRIPLE_TAP_TIMING      0x92
#define EDMP_APEX_INTERF_TRIPLE_TAP_TIMING_SIZE 1

/*
 * apex_interf_shake_outaxis
 *
 */
#define EDMP_APEX_INTERF_SHAKE_OUTAXIS      0x93
#define EDMP_APEX_INTERF_SHAKE_OUTAXIS_SIZE 1

/*
 * apex_interf_noMotion_outLock
 *
 */
#define EDMP_APEX_INTERF_NOMOTION_OUTLOCK      0x94
#define EDMP_APEX_INTERF_NOMOTION_OUTLOCK_SIZE 1

/*
 * apex_interf_noMotion_majority_axis
 *
 */
#define EDMP_APEX_INTERF_NOMOTION_MAJORITY_AXIS      0x95
#define EDMP_APEX_INTERF_NOMOTION_MAJORITY_AXIS_SIZE 1

/*
 * apex_interf_noMotion_sign_angle
 *
 */
#define EDMP_APEX_INTERF_NOMOTION_SIGN_ANGLE      0x96
#define EDMP_APEX_INTERF_NOMOTION_SIGN_ANGLE_SIZE 1

/*
 * apex_interf_noMotion_x_axis_ref
 *
 */
#define EDMP_APEX_INTERF_NOMOTION_X_AXIS_REF      0x98
#define EDMP_APEX_INTERF_NOMOTION_X_AXIS_REF_SIZE 2

/*
 * apex_interf_noMotion_y_axis_ref
 *
 */
#define EDMP_APEX_INTERF_NOMOTION_Y_AXIS_REF      0x9a
#define EDMP_APEX_INTERF_NOMOTION_Y_AXIS_REF_SIZE 2

/*
 * apex_interf_noMotion_tilt_angle_raw_format
 *
 */
#define EDMP_APEX_INTERF_NOMOTION_TILT_ANGLE_RAW_FORMAT      0x9c
#define EDMP_APEX_INTERF_NOMOTION_TILT_ANGLE_RAW_FORMAT_SIZE 2

/*
 * apex_state_wom
 *
 */
#define EDMP_APEX_STATE_WOM      0xa0
#define EDMP_APEX_STATE_WOM_SIZE 12

/*
 * apex_state_wom_womHW
 *
 */
#define EDMP_APEX_STATE_WOM_WOMHW      0xa0
#define EDMP_APEX_STATE_WOM_WOMHW_SIZE 1

/*
 * apex_state_wom_counter_elapsed
 *
 */
#define EDMP_APEX_STATE_WOM_COUNTER_ELAPSED      0xa1
#define EDMP_APEX_STATE_WOM_COUNTER_ELAPSED_SIZE 1

/*
 * apex_state_wom_counter
 *
 */
#define EDMP_APEX_STATE_WOM_COUNTER      0xa4
#define EDMP_APEX_STATE_WOM_COUNTER_SIZE 4

/*
 * apex_state_wom_time_threshold
 *
 */
#define EDMP_APEX_STATE_WOM_TIME_THRESHOLD      0xa8
#define EDMP_APEX_STATE_WOM_TIME_THRESHOLD_SIZE 4

/*
 * apex_state_ff
 *
 */
#define EDMP_APEX_STATE_FF      0xac
#define EDMP_APEX_STATE_FF_SIZE 52

/*
 * apex_state_ff_decim_rate
 *
 */
#define EDMP_APEX_STATE_FF_DECIM_RATE      0xac
#define EDMP_APEX_STATE_FF_DECIM_RATE_SIZE 1

/*
 * apex_state_ff_decim_count
 *
 */
#define EDMP_APEX_STATE_FF_DECIM_COUNT      0xad
#define EDMP_APEX_STATE_FF_DECIM_COUNT_SIZE 1

/*
 * apex_state_ff_state
 *
 */
#define EDMP_APEX_STATE_FF_STATE      0xae
#define EDMP_APEX_STATE_FF_STATE_SIZE 1

/*
 * apex_state_ff_lowg_event
 *
 */
#define EDMP_APEX_STATE_FF_LOWG_EVENT      0xaf
#define EDMP_APEX_STATE_FF_LOWG_EVENT_SIZE 1

/*
 * apex_state_ff_highg_event
 *
 */
#define EDMP_APEX_STATE_FF_HIGHG_EVENT      0xb0
#define EDMP_APEX_STATE_FF_HIGHG_EVENT_SIZE 1

/*
 * apex_state_ff_interrupt
 *
 */
#define EDMP_APEX_STATE_FF_INTERRUPT      0xb1
#define EDMP_APEX_STATE_FF_INTERRUPT_SIZE 1

/*
 * apex_state_ff_timer
 *
 */
#define EDMP_APEX_STATE_FF_TIMER      0xb4
#define EDMP_APEX_STATE_FF_TIMER_SIZE 4

/*
 * apex_state_ff_ff_duration_samples
 *
 */
#define EDMP_APEX_STATE_FF_FF_DURATION_SAMPLES      0xb8
#define EDMP_APEX_STATE_FF_FF_DURATION_SAMPLES_SIZE 4

/*
 * apex_state_ff_min_duration_thr
 *
 */
#define EDMP_APEX_STATE_FF_MIN_DURATION_THR      0xbc
#define EDMP_APEX_STATE_FF_MIN_DURATION_THR_SIZE 4

/*
 * apex_state_ff_max_duration_thr
 *
 */
#define EDMP_APEX_STATE_FF_MAX_DURATION_THR      0xc0
#define EDMP_APEX_STATE_FF_MAX_DURATION_THR_SIZE 4

/*
 * apex_state_ff_debounce_thr
 *
 */
#define EDMP_APEX_STATE_FF_DEBOUNCE_THR      0xc4
#define EDMP_APEX_STATE_FF_DEBOUNCE_THR_SIZE 4

/*
 * apex_state_ff_high_g
 *
 */
#define EDMP_APEX_STATE_FF_HIGH_G      0xc8
#define EDMP_APEX_STATE_FF_HIGH_G_SIZE 12

/*
 * apex_state_ff_high_g_state
 *
 */
#define EDMP_APEX_STATE_FF_HIGH_G_STATE      0xc8
#define EDMP_APEX_STATE_FF_HIGH_G_STATE_SIZE 2

/*
 * apex_state_ff_high_g_cntr
 *
 */
#define EDMP_APEX_STATE_FF_HIGH_G_CNTR      0xca
#define EDMP_APEX_STATE_FF_HIGH_G_CNTR_SIZE 2

/*
 * apex_state_ff_high_g_accel_ODR
 *
 */
#define EDMP_APEX_STATE_FF_HIGH_G_ACCEL_ODR      0xcc
#define EDMP_APEX_STATE_FF_HIGH_G_ACCEL_ODR_SIZE 2

/*
 * apex_state_ff_high_g_high_peak_thres
 *
 */
#define EDMP_APEX_STATE_FF_HIGH_G_HIGH_PEAK_THRES      0xce
#define EDMP_APEX_STATE_FF_HIGH_G_HIGH_PEAK_THRES_SIZE 2

/*
 * apex_state_ff_high_g_high_peak_thres_hyst
 *
 */
#define EDMP_APEX_STATE_FF_HIGH_G_HIGH_PEAK_THRES_HYST      0xd0
#define EDMP_APEX_STATE_FF_HIGH_G_HIGH_PEAK_THRES_HYST_SIZE 2

/*
 * apex_state_ff_high_g_high_time_thres
 *
 */
#define EDMP_APEX_STATE_FF_HIGH_G_HIGH_TIME_THRES      0xd2
#define EDMP_APEX_STATE_FF_HIGH_G_HIGH_TIME_THRES_SIZE 2

/*
 * apex_state_ff_low_g
 *
 */
#define EDMP_APEX_STATE_FF_LOW_G      0xd4
#define EDMP_APEX_STATE_FF_LOW_G_SIZE 12

/*
 * apex_state_ff_low_g_state
 *
 */
#define EDMP_APEX_STATE_FF_LOW_G_STATE      0xd4
#define EDMP_APEX_STATE_FF_LOW_G_STATE_SIZE 2

/*
 * apex_state_ff_low_g_cntr
 *
 */
#define EDMP_APEX_STATE_FF_LOW_G_CNTR      0xd6
#define EDMP_APEX_STATE_FF_LOW_G_CNTR_SIZE 2

/*
 * apex_state_ff_low_g_accel_ODR
 *
 */
#define EDMP_APEX_STATE_FF_LOW_G_ACCEL_ODR      0xd8
#define EDMP_APEX_STATE_FF_LOW_G_ACCEL_ODR_SIZE 2

/*
 * apex_state_ff_low_g_low_peak_thres
 *
 */
#define EDMP_APEX_STATE_FF_LOW_G_LOW_PEAK_THRES      0xda
#define EDMP_APEX_STATE_FF_LOW_G_LOW_PEAK_THRES_SIZE 2

/*
 * apex_state_ff_low_g_low_peak_thres_hyst
 *
 */
#define EDMP_APEX_STATE_FF_LOW_G_LOW_PEAK_THRES_HYST      0xdc
#define EDMP_APEX_STATE_FF_LOW_G_LOW_PEAK_THRES_HYST_SIZE 2

/*
 * apex_state_ff_low_g_low_time_thres
 *
 */
#define EDMP_APEX_STATE_FF_LOW_G_LOW_TIME_THRES      0xde
#define EDMP_APEX_STATE_FF_LOW_G_LOW_TIME_THRES_SIZE 2

/*
 * apex_state_tap
 *
 */
#define EDMP_APEX_STATE_TAP      0xe0
#define EDMP_APEX_STATE_TAP_SIZE 52

/*
 * apex_state_tap_decim_rate
 *
 */
#define EDMP_APEX_STATE_TAP_DECIM_RATE      0xe0
#define EDMP_APEX_STATE_TAP_DECIM_RATE_SIZE 1

/*
 * apex_state_tap_decim_count
 *
 */
#define EDMP_APEX_STATE_TAP_DECIM_COUNT      0xe1
#define EDMP_APEX_STATE_TAP_DECIM_COUNT_SIZE 1

/*
 * apex_state_tap_firstSampleFlag
 *
 */
#define EDMP_APEX_STATE_TAP_FIRSTSAMPLEFLAG      0xe2
#define EDMP_APEX_STATE_TAP_FIRSTSAMPLEFLAG_SIZE 1

/*
 * apex_state_tap_interrupt
 *
 */
#define EDMP_APEX_STATE_TAP_INTERRUPT      0xe3
#define EDMP_APEX_STATE_TAP_INTERRUPT_SIZE 1

/*
 * apex_state_tap_params
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS      0xe4
#define EDMP_APEX_STATE_TAP_PARAMS_SIZE 16

/*
 * apex_state_tap_params_Tmax_u9
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_TMAX_U9      0xe4
#define EDMP_APEX_STATE_TAP_PARAMS_TMAX_U9_SIZE 2

/*
 * apex_state_tap_params_Tmin_u8
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_TMIN_U8      0xe6
#define EDMP_APEX_STATE_TAP_PARAMS_TMIN_U8_SIZE 1

/*
 * apex_state_tap_params_minJerkThreshold_u6
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_MINJERKTHRESHOLD_U6      0xe7
#define EDMP_APEX_STATE_TAP_PARAMS_MINJERKTHRESHOLD_U6_SIZE 1

/*
 * apex_state_tap_params_smudgeRejectThreshold_u7
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_SMUDGEREJECTTHRESHOLD_U7      0xe8
#define EDMP_APEX_STATE_TAP_PARAMS_SMUDGEREJECTTHRESHOLD_U7_SIZE 1

/*
 * apex_state_tap_params_maxPickTolerance_u2
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_MAXPICKTOLERANCE_U2      0xe9
#define EDMP_APEX_STATE_TAP_PARAMS_MAXPICKTOLERANCE_U2_SIZE 1

/*
 * apex_state_tap_params_Tavg_u3
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_TAVG_U3      0xea
#define EDMP_APEX_STATE_TAP_PARAMS_TAVG_U3_SIZE 1

/*
 * apex_state_tap_params_accel_odr
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_ACCEL_ODR      0xeb
#define EDMP_APEX_STATE_TAP_PARAMS_ACCEL_ODR_SIZE 1

/*
 * apex_state_tap_params_max_tap
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_MAX_TAP      0xec
#define EDMP_APEX_STATE_TAP_PARAMS_MAX_TAP_SIZE 1

/*
 * apex_state_tap_params_min_tap
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_MIN_TAP      0xed
#define EDMP_APEX_STATE_TAP_PARAMS_MIN_TAP_SIZE 1

/*
 * apex_state_tap_params_max_energy_primary_axis
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_MAX_ENERGY_PRIMARY_AXIS      0xee
#define EDMP_APEX_STATE_TAP_PARAMS_MAX_ENERGY_PRIMARY_AXIS_SIZE 2

/*
 * apex_state_tap_params_max_energy_secondary_axis
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_MAX_ENERGY_SECONDARY_AXIS      0xf0
#define EDMP_APEX_STATE_TAP_PARAMS_MAX_ENERGY_SECONDARY_AXIS_SIZE 2

/*
 * apex_state_tap_params_axis_select_mask
 *
 */
#define EDMP_APEX_STATE_TAP_PARAMS_AXIS_SELECT_MASK      0xf2
#define EDMP_APEX_STATE_TAP_PARAMS_AXIS_SELECT_MASK_SIZE 1

/*
 * apex_state_tap_states
 *
 */
#define EDMP_APEX_STATE_TAP_STATES      0xf4
#define EDMP_APEX_STATE_TAP_STATES_SIZE 24

/*
 * apex_state_tap_states_inout_Tmax_window_u1
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TMAX_WINDOW_U1      0xf4
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TMAX_WINDOW_U1_SIZE 1

/*
 * apex_state_tap_states_inout_Tmin_window_u1
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TMIN_WINDOW_U1      0xf5
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TMIN_WINDOW_U1_SIZE 1

/*
 * apex_state_tap_states_inout_Tavg_window_u1
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TAVG_WINDOW_U1      0xf6
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TAVG_WINDOW_U1_SIZE 1

/*
 * apex_state_tap_states_stateOut_detectJerk_u1
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_STATEOUT_DETECTJERK_U1      0xf7
#define EDMP_APEX_STATE_TAP_STATES_STATEOUT_DETECTJERK_U1_SIZE 1

/*
 * apex_state_tap_states_inout_jerkNormL1_u7
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_JERKNORML1_U7      0xf8
#define EDMP_APEX_STATE_TAP_STATES_INOUT_JERKNORML1_U7_SIZE 1

/*
 * apex_state_tap_states_inout_jerkNormL1_u7_ref
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_JERKNORML1_U7_REF      0xf9
#define EDMP_APEX_STATE_TAP_STATES_INOUT_JERKNORML1_U7_REF_SIZE 1

/*
 * apex_state_tap_states_inout_countTmin_u8
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_COUNTTMIN_U8      0xfa
#define EDMP_APEX_STATE_TAP_STATES_INOUT_COUNTTMIN_U8_SIZE 1

/*
 * apex_state_tap_states_inout_y_n_1_u7
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_Y_N_1_U7      0xfb
#define EDMP_APEX_STATE_TAP_STATES_INOUT_Y_N_1_U7_SIZE 1

/*
 * apex_state_tap_states_inout_countTavg_u3
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_COUNTTAVG_U3      0xfc
#define EDMP_APEX_STATE_TAP_STATES_INOUT_COUNTTAVG_U3_SIZE 1

/*
 * apex_state_tap_states_inout_tap_direction_u1
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TAP_DIRECTION_U1      0xfd
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TAP_DIRECTION_U1_SIZE 3

/*
 * apex_state_tap_states_inout_Acc_z1_s11
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_ACC_Z1_S11      0x100
#define EDMP_APEX_STATE_TAP_STATES_INOUT_ACC_Z1_S11_SIZE 6

/*
 * apex_state_tap_states_inout_tap_energy_u13
 *
 */
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TAP_ENERGY_U13      0x106
#define EDMP_APEX_STATE_TAP_STATES_INOUT_TAP_ENERGY_U13_SIZE 6

/*
 * apex_state_tap_pickCounter
 *
 */
#define EDMP_APEX_STATE_TAP_PICKCOUNTER      0x10c
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_SIZE 8

/*
 * apex_state_tap_pickCounter_state_count
 *
 */
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_STATE_COUNT      0x10c
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_STATE_COUNT_SIZE 2

/*
 * apex_state_tap_pickCounter_state_sampleCounter
 *
 */
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_STATE_SAMPLECOUNTER      0x10e
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_STATE_SAMPLECOUNTER_SIZE 2

/*
 * apex_state_tap_pickCounter_second_tap_count
 *
 */
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_SECOND_TAP_COUNT      0x110
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_SECOND_TAP_COUNT_SIZE 2

/*
 * apex_state_tap_pickCounter_third_tap_count
 *
 */
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_THIRD_TAP_COUNT      0x112
#define EDMP_APEX_STATE_TAP_PICKCOUNTER_THIRD_TAP_COUNT_SIZE 2

/*
 * apex_state_r2w
 *
 */
#define EDMP_APEX_STATE_R2W      0x114
#define EDMP_APEX_STATE_R2W_SIZE 172

/*
 * apex_state_r2w_decim_rate
 *
 */
#define EDMP_APEX_STATE_R2W_DECIM_RATE      0x114
#define EDMP_APEX_STATE_R2W_DECIM_RATE_SIZE 1

/*
 * apex_state_r2w_decim_count
 *
 */
#define EDMP_APEX_STATE_R2W_DECIM_COUNT      0x115
#define EDMP_APEX_STATE_R2W_DECIM_COUNT_SIZE 1

/*
 * apex_state_r2w_fire_wake_event
 *
 */
#define EDMP_APEX_STATE_R2W_FIRE_WAKE_EVENT      0x116
#define EDMP_APEX_STATE_R2W_FIRE_WAKE_EVENT_SIZE 1

/*
 * apex_state_r2w_fire_sleep_event
 *
 */
#define EDMP_APEX_STATE_R2W_FIRE_SLEEP_EVENT      0x117
#define EDMP_APEX_STATE_R2W_FIRE_SLEEP_EVENT_SIZE 1

/*
 * apex_state_r2w_state
 *
 */
#define EDMP_APEX_STATE_R2W_STATE      0x118
#define EDMP_APEX_STATE_R2W_STATE_SIZE 84

/*
 * apex_state_r2w_state_first_timer
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_FIRST_TIMER      0x118
#define EDMP_APEX_STATE_R2W_STATE_FIRST_TIMER_SIZE 4

/*
 * apex_state_r2w_state_gravity
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY      0x11c
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_SIZE 12

/*
 * apex_state_r2w_state_status
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_STATUS      0x128
#define EDMP_APEX_STATE_R2W_STATE_STATUS_SIZE 4

/*
 * apex_state_r2w_state_status_counter
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_STATUS_COUNTER      0x12c
#define EDMP_APEX_STATE_R2W_STATE_STATUS_COUNTER_SIZE 4

/*
 * apex_state_r2w_state_gravity_prev
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_PREV      0x130
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_PREV_SIZE 12

/*
 * apex_state_r2w_state_update_prev
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_UPDATE_PREV      0x13c
#define EDMP_APEX_STATE_R2W_STATE_UPDATE_PREV_SIZE 4

/*
 * apex_state_r2w_state_motion_counter
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_MOTION_COUNTER      0x140
#define EDMP_APEX_STATE_R2W_STATE_MOTION_COUNTER_SIZE 4

/*
 * apex_state_r2w_state_update_last
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_UPDATE_LAST      0x144
#define EDMP_APEX_STATE_R2W_STATE_UPDATE_LAST_SIZE 4

/*
 * apex_state_r2w_state_gravity_filter_gain
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_FILTER_GAIN      0x148
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_FILTER_GAIN_SIZE 4

/*
 * apex_state_r2w_state_cdt_gesture
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_CDT_GESTURE      0x14c
#define EDMP_APEX_STATE_R2W_STATE_CDT_GESTURE_SIZE 4

/*
 * apex_state_r2w_state_gravity_last_x
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_LAST_X      0x150
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_LAST_X_SIZE 4

/*
 * apex_state_r2w_state_gravity_last_y
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_LAST_Y      0x154
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_LAST_Y_SIZE 4

/*
 * apex_state_r2w_state_gravity_last_z
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_LAST_Z      0x158
#define EDMP_APEX_STATE_R2W_STATE_GRAVITY_LAST_Z_SIZE 4

/*
 * apex_state_r2w_state_update_last_counter
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_UPDATE_LAST_COUNTER      0x15c
#define EDMP_APEX_STATE_R2W_STATE_UPDATE_LAST_COUNTER_SIZE 4

/*
 * apex_state_r2w_state_enable_wake_event
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_ENABLE_WAKE_EVENT      0x160
#define EDMP_APEX_STATE_R2W_STATE_ENABLE_WAKE_EVENT_SIZE 4

/*
 * apex_state_r2w_state_enable_sleep_event
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_ENABLE_SLEEP_EVENT      0x164
#define EDMP_APEX_STATE_R2W_STATE_ENABLE_SLEEP_EVENT_SIZE 4

/*
 * apex_state_r2w_state_sleep_time_out
 *
 */
#define EDMP_APEX_STATE_R2W_STATE_SLEEP_TIME_OUT      0x168
#define EDMP_APEX_STATE_R2W_STATE_SLEEP_TIME_OUT_SIZE 4

/*
 * apex_state_r2w_param
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM      0x16c
#define EDMP_APEX_STATE_R2W_PARAM_SIZE 84

/*
 * apex_state_r2w_param_sleep_time_out
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_SLEEP_TIME_OUT      0x16c
#define EDMP_APEX_STATE_R2W_PARAM_SLEEP_TIME_OUT_SIZE 4

/*
 * apex_state_r2w_param_sleep_gesture_delay
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_SLEEP_GESTURE_DELAY      0x170
#define EDMP_APEX_STATE_R2W_PARAM_SLEEP_GESTURE_DELAY_SIZE 4

/*
 * apex_state_r2w_param_mounting_matrix
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_MOUNTING_MATRIX      0x174
#define EDMP_APEX_STATE_R2W_PARAM_MOUNTING_MATRIX_SIZE 4

/*
 * apex_state_r2w_param_sampling_period
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_SAMPLING_PERIOD      0x178
#define EDMP_APEX_STATE_R2W_PARAM_SAMPLING_PERIOD_SIZE 4

/*
 * apex_state_r2w_param_gravity_filter_gain
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_GRAVITY_FILTER_GAIN      0x17c
#define EDMP_APEX_STATE_R2W_PARAM_GRAVITY_FILTER_GAIN_SIZE 4

/*
 * apex_state_r2w_param_motion_thr_angle_cosine
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_MOTION_THR_ANGLE_COSINE      0x180
#define EDMP_APEX_STATE_R2W_PARAM_MOTION_THR_ANGLE_COSINE_SIZE 4

/*
 * apex_state_r2w_param_motion_thr_timer_fast
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_MOTION_THR_TIMER_FAST      0x184
#define EDMP_APEX_STATE_R2W_PARAM_MOTION_THR_TIMER_FAST_SIZE 4

/*
 * apex_state_r2w_param_motion_thr_timer_slow
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_MOTION_THR_TIMER_SLOW      0x188
#define EDMP_APEX_STATE_R2W_PARAM_MOTION_THR_TIMER_SLOW_SIZE 4

/*
 * apex_state_r2w_param_motion_prev_gravity_timeout
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_MOTION_PREV_GRAVITY_TIMEOUT      0x18c
#define EDMP_APEX_STATE_R2W_PARAM_MOTION_PREV_GRAVITY_TIMEOUT_SIZE 4

/*
 * apex_state_r2w_param_last_gravity_motion_timer
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_LAST_GRAVITY_MOTION_TIMER      0x190
#define EDMP_APEX_STATE_R2W_PARAM_LAST_GRAVITY_MOTION_TIMER_SIZE 4

/*
 * apex_state_r2w_param_last_gravity_timeout
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_LAST_GRAVITY_TIMEOUT      0x194
#define EDMP_APEX_STATE_R2W_PARAM_LAST_GRAVITY_TIMEOUT_SIZE 4

/*
 * apex_state_r2w_param_gesture_validity_timeout
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_GESTURE_VALIDITY_TIMEOUT      0x198
#define EDMP_APEX_STATE_R2W_PARAM_GESTURE_VALIDITY_TIMEOUT_SIZE 4

/*
 * apex_state_r2w_param_ACCEL_SAT
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_ACCEL_SAT      0x19c
#define EDMP_APEX_STATE_R2W_PARAM_ACCEL_SAT_SIZE 4

/*
 * apex_state_r2w_param_thr_cos20deg
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS20DEG      0x1a0
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS20DEG_SIZE 4

/*
 * apex_state_r2w_param_thr_cos30deg
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS30DEG      0x1a4
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS30DEG_SIZE 4

/*
 * apex_state_r2w_param_thr_cos80deg
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS80DEG      0x1a8
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS80DEG_SIZE 4

/*
 * apex_state_r2w_param_thr_cos70deg
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS70DEG      0x1ac
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS70DEG_SIZE 4

/*
 * apex_state_r2w_param_thr_cos60deg
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS60DEG      0x1b0
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS60DEG_SIZE 4

/*
 * apex_state_r2w_param_thr_cos45deg
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS45DEG      0x1b4
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS45DEG_SIZE 4

/*
 * apex_state_r2w_param_thr_cos83deg
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS83DEG      0x1b8
#define EDMP_APEX_STATE_R2W_PARAM_THR_COS83DEG_SIZE 4

/*
 * apex_state_r2w_param_const_gravity_estimator
 *
 */
#define EDMP_APEX_STATE_R2W_PARAM_CONST_GRAVITY_ESTIMATOR      0x1bc
#define EDMP_APEX_STATE_R2W_PARAM_CONST_GRAVITY_ESTIMATOR_SIZE 4

/*
 * apex_state_3axis
 *
 */
#define EDMP_APEX_STATE_3AXIS      0x1c0
#define EDMP_APEX_STATE_3AXIS_SIZE 48

/*
 * apex_state_3axis_decim_rate
 *
 */
#define EDMP_APEX_STATE_3AXIS_DECIM_RATE      0x1c0
#define EDMP_APEX_STATE_3AXIS_DECIM_RATE_SIZE 1

/*
 * apex_state_3axis_decim_count
 *
 */
#define EDMP_APEX_STATE_3AXIS_DECIM_COUNT      0x1c1
#define EDMP_APEX_STATE_3AXIS_DECIM_COUNT_SIZE 1

/*
 * apex_state_3axis_init
 *
 */
#define EDMP_APEX_STATE_3AXIS_INIT      0x1c2
#define EDMP_APEX_STATE_3AXIS_INIT_SIZE 2

/*
 * apex_state_3axis_quat
 *
 */
#define EDMP_APEX_STATE_3AXIS_QUAT      0x1c4
#define EDMP_APEX_STATE_3AXIS_QUAT_SIZE 16

/*
 * apex_state_3axis_gain
 *
 */
#define EDMP_APEX_STATE_3AXIS_GAIN      0x1d4
#define EDMP_APEX_STATE_3AXIS_GAIN_SIZE 4

/*
 * apex_state_3axis_qfeedbk
 *
 */
#define EDMP_APEX_STATE_3AXIS_QFEEDBK      0x1d8
#define EDMP_APEX_STATE_3AXIS_QFEEDBK_SIZE 8

/*
 * apex_state_3axis_accel_wx
 *
 */
#define EDMP_APEX_STATE_3AXIS_ACCEL_WX      0x1e0
#define EDMP_APEX_STATE_3AXIS_ACCEL_WX_SIZE 4

/*
 * apex_state_3axis_accel_wy
 *
 */
#define EDMP_APEX_STATE_3AXIS_ACCEL_WY      0x1e4
#define EDMP_APEX_STATE_3AXIS_ACCEL_WY_SIZE 4

/*
 * apex_state_3axis_accel_wz
 *
 */
#define EDMP_APEX_STATE_3AXIS_ACCEL_WZ      0x1e8
#define EDMP_APEX_STATE_3AXIS_ACCEL_WZ_SIZE 4

/*
 * apex_state_3axis_accel_only_gain
 *
 */
#define EDMP_APEX_STATE_3AXIS_ACCEL_ONLY_GAIN      0x1ec
#define EDMP_APEX_STATE_3AXIS_ACCEL_ONLY_GAIN_SIZE 4

/*
 * apex_state_tilt
 *
 */
#define EDMP_APEX_STATE_TILT      0x1f0
#define EDMP_APEX_STATE_TILT_SIZE 44

/*
 * apex_state_tilt_decim_rate
 *
 */
#define EDMP_APEX_STATE_TILT_DECIM_RATE      0x1f0
#define EDMP_APEX_STATE_TILT_DECIM_RATE_SIZE 1

/*
 * apex_state_tilt_decim_count
 *
 */
#define EDMP_APEX_STATE_TILT_DECIM_COUNT      0x1f1
#define EDMP_APEX_STATE_TILT_DECIM_COUNT_SIZE 1

/*
 * apex_state_tilt_isInit
 *
 */
#define EDMP_APEX_STATE_TILT_ISINIT      0x1f2
#define EDMP_APEX_STATE_TILT_ISINIT_SIZE 1

/*
 * apex_state_tilt_outInterrupt
 *
 */
#define EDMP_APEX_STATE_TILT_OUTINTERRUPT      0x1f3
#define EDMP_APEX_STATE_TILT_OUTINTERRUPT_SIZE 1

/*
 * apex_state_tilt_timer
 *
 */
#define EDMP_APEX_STATE_TILT_TIMER      0x1f4
#define EDMP_APEX_STATE_TILT_TIMER_SIZE 4

/*
 * apex_state_tilt_quat_wb_x
 *
 */
#define EDMP_APEX_STATE_TILT_QUAT_WB_X      0x1f8
#define EDMP_APEX_STATE_TILT_QUAT_WB_X_SIZE 4

/*
 * apex_state_tilt_quat_wb_y
 *
 */
#define EDMP_APEX_STATE_TILT_QUAT_WB_Y      0x1fc
#define EDMP_APEX_STATE_TILT_QUAT_WB_Y_SIZE 4

/*
 * apex_state_tilt_quat_wb_z
 *
 */
#define EDMP_APEX_STATE_TILT_QUAT_WB_Z      0x200
#define EDMP_APEX_STATE_TILT_QUAT_WB_Z_SIZE 4

/*
 * apex_state_tilt_tilt_angle
 *
 */
#define EDMP_APEX_STATE_TILT_TILT_ANGLE      0x204
#define EDMP_APEX_STATE_TILT_TILT_ANGLE_SIZE 4

/*
 * apex_state_tilt_quat_ref_x
 *
 */
#define EDMP_APEX_STATE_TILT_QUAT_REF_X      0x208
#define EDMP_APEX_STATE_TILT_QUAT_REF_X_SIZE 4

/*
 * apex_state_tilt_quat_ref_y
 *
 */
#define EDMP_APEX_STATE_TILT_QUAT_REF_Y      0x20c
#define EDMP_APEX_STATE_TILT_QUAT_REF_Y_SIZE 4

/*
 * apex_state_tilt_quat_ref_z
 *
 */
#define EDMP_APEX_STATE_TILT_QUAT_REF_Z      0x210
#define EDMP_APEX_STATE_TILT_QUAT_REF_Z_SIZE 4

/*
 * apex_state_tilt_wait_time
 *
 */
#define EDMP_APEX_STATE_TILT_WAIT_TIME      0x214
#define EDMP_APEX_STATE_TILT_WAIT_TIME_SIZE 2

/*
 * apex_state_tilt_tilt_angle_th
 *
 */
#define EDMP_APEX_STATE_TILT_TILT_ANGLE_TH      0x218
#define EDMP_APEX_STATE_TILT_TILT_ANGLE_TH_SIZE 4

/*
 * apex_state_ped
 *
 */
#define EDMP_APEX_STATE_PED      0x21c
#define EDMP_APEX_STATE_PED_SIZE 388

/*
 * apex_state_ped_decim_rate
 *
 */
#define EDMP_APEX_STATE_PED_DECIM_RATE      0x21c
#define EDMP_APEX_STATE_PED_DECIM_RATE_SIZE 1

/*
 * apex_state_ped_decim_count
 *
 */
#define EDMP_APEX_STATE_PED_DECIM_COUNT      0x21d
#define EDMP_APEX_STATE_PED_DECIM_COUNT_SIZE 1

/*
 * apex_state_ped_step_event_interrupt
 *
 */
#define EDMP_APEX_STATE_PED_STEP_EVENT_INTERRUPT      0x21e
#define EDMP_APEX_STATE_PED_STEP_EVENT_INTERRUPT_SIZE 1

/*
 * apex_state_ped_step_count_overflow_interrupt
 *
 */
#define EDMP_APEX_STATE_PED_STEP_COUNT_OVERFLOW_INTERRUPT      0x21f
#define EDMP_APEX_STATE_PED_STEP_COUNT_OVERFLOW_INTERRUPT_SIZE 1

/*
 * apex_state_ped_step_event
 *
 */
#define EDMP_APEX_STATE_PED_STEP_EVENT      0x220
#define EDMP_APEX_STATE_PED_STEP_EVENT_SIZE 1

/*
 * apex_state_ped_step_event_buffer
 *
 */
#define EDMP_APEX_STATE_PED_STEP_EVENT_BUFFER      0x221
#define EDMP_APEX_STATE_PED_STEP_EVENT_BUFFER_SIZE 1

/*
 * apex_state_ped_prev_step_count
 *
 */
#define EDMP_APEX_STATE_PED_PREV_STEP_COUNT      0x222
#define EDMP_APEX_STATE_PED_PREV_STEP_COUNT_SIZE 2

/*
 * apex_state_ped_step_counter
 *
 */
#define EDMP_APEX_STATE_PED_STEP_COUNTER      0x224
#define EDMP_APEX_STATE_PED_STEP_COUNTER_SIZE 4

/*
 * apex_state_ped_step_feature
 *
 */
#define EDMP_APEX_STATE_PED_STEP_FEATURE      0x228
#define EDMP_APEX_STATE_PED_STEP_FEATURE_SIZE 4

/*
 * apex_state_ped_step_counter_prev
 *
 */
#define EDMP_APEX_STATE_PED_STEP_COUNTER_PREV      0x22c
#define EDMP_APEX_STATE_PED_STEP_COUNTER_PREV_SIZE 4

/*
 * apex_state_ped_step_duration
 *
 */
#define EDMP_APEX_STATE_PED_STEP_DURATION      0x230
#define EDMP_APEX_STATE_PED_STEP_DURATION_SIZE 4

/*
 * apex_state_ped_dtc_n_hat
 *
 */
#define EDMP_APEX_STATE_PED_DTC_N_HAT      0x234
#define EDMP_APEX_STATE_PED_DTC_N_HAT_SIZE 4

/*
 * apex_state_ped_dtc_fast_n_hat
 *
 */
#define EDMP_APEX_STATE_PED_DTC_FAST_N_HAT      0x238
#define EDMP_APEX_STATE_PED_DTC_FAST_N_HAT_SIZE 4

/*
 * apex_state_ped_step_count_est
 *
 */
#define EDMP_APEX_STATE_PED_STEP_COUNT_EST      0x23c
#define EDMP_APEX_STATE_PED_STEP_COUNT_EST_SIZE 4

/*
 * apex_state_ped_time_counter
 *
 */
#define EDMP_APEX_STATE_PED_TIME_COUNTER      0x240
#define EDMP_APEX_STATE_PED_TIME_COUNTER_SIZE 4

/*
 * apex_state_ped_xold
 *
 */
#define EDMP_APEX_STATE_PED_XOLD      0x244
#define EDMP_APEX_STATE_PED_XOLD_SIZE 4

/*
 * apex_state_ped_x
 *
 */
#define EDMP_APEX_STATE_PED_X      0x248
#define EDMP_APEX_STATE_PED_X_SIZE 4

/*
 * apex_state_ped_x4
 *
 */
#define EDMP_APEX_STATE_PED_X4      0x24c
#define EDMP_APEX_STATE_PED_X4_SIZE 4

/*
 * apex_state_ped_x3
 *
 */
#define EDMP_APEX_STATE_PED_X3      0x250
#define EDMP_APEX_STATE_PED_X3_SIZE 4

/*
 * apex_state_ped_x2
 *
 */
#define EDMP_APEX_STATE_PED_X2      0x254
#define EDMP_APEX_STATE_PED_X2_SIZE 4

/*
 * apex_state_ped_x1
 *
 */
#define EDMP_APEX_STATE_PED_X1      0x258
#define EDMP_APEX_STATE_PED_X1_SIZE 4

/*
 * apex_state_ped_y4
 *
 */
#define EDMP_APEX_STATE_PED_Y4      0x25c
#define EDMP_APEX_STATE_PED_Y4_SIZE 4

/*
 * apex_state_ped_y3
 *
 */
#define EDMP_APEX_STATE_PED_Y3      0x260
#define EDMP_APEX_STATE_PED_Y3_SIZE 4

/*
 * apex_state_ped_y2
 *
 */
#define EDMP_APEX_STATE_PED_Y2      0x264
#define EDMP_APEX_STATE_PED_Y2_SIZE 4

/*
 * apex_state_ped_y1
 *
 */
#define EDMP_APEX_STATE_PED_Y1      0x268
#define EDMP_APEX_STATE_PED_Y1_SIZE 4

/*
 * apex_state_ped_ER1_y4
 *
 */
#define EDMP_APEX_STATE_PED_ER1_Y4      0x26c
#define EDMP_APEX_STATE_PED_ER1_Y4_SIZE 4

/*
 * apex_state_ped_ER1_y3
 *
 */
#define EDMP_APEX_STATE_PED_ER1_Y3      0x270
#define EDMP_APEX_STATE_PED_ER1_Y3_SIZE 4

/*
 * apex_state_ped_ER1_y2
 *
 */
#define EDMP_APEX_STATE_PED_ER1_Y2      0x274
#define EDMP_APEX_STATE_PED_ER1_Y2_SIZE 4

/*
 * apex_state_ped_ER1_y1
 *
 */
#define EDMP_APEX_STATE_PED_ER1_Y1      0x278
#define EDMP_APEX_STATE_PED_ER1_Y1_SIZE 4

/*
 * apex_state_ped_ER2_y4
 *
 */
#define EDMP_APEX_STATE_PED_ER2_Y4      0x27c
#define EDMP_APEX_STATE_PED_ER2_Y4_SIZE 4

/*
 * apex_state_ped_ER2_y3
 *
 */
#define EDMP_APEX_STATE_PED_ER2_Y3      0x280
#define EDMP_APEX_STATE_PED_ER2_Y3_SIZE 4

/*
 * apex_state_ped_ER2_y2
 *
 */
#define EDMP_APEX_STATE_PED_ER2_Y2      0x284
#define EDMP_APEX_STATE_PED_ER2_Y2_SIZE 4

/*
 * apex_state_ped_ER2_y1
 *
 */
#define EDMP_APEX_STATE_PED_ER2_Y1      0x288
#define EDMP_APEX_STATE_PED_ER2_Y1_SIZE 4

/*
 * apex_state_ped_y4_low
 *
 */
#define EDMP_APEX_STATE_PED_Y4_LOW      0x28c
#define EDMP_APEX_STATE_PED_Y4_LOW_SIZE 4

/*
 * apex_state_ped_y3_low
 *
 */
#define EDMP_APEX_STATE_PED_Y3_LOW      0x290
#define EDMP_APEX_STATE_PED_Y3_LOW_SIZE 4

/*
 * apex_state_ped_y2_low
 *
 */
#define EDMP_APEX_STATE_PED_Y2_LOW      0x294
#define EDMP_APEX_STATE_PED_Y2_LOW_SIZE 4

/*
 * apex_state_ped_y1_low
 *
 */
#define EDMP_APEX_STATE_PED_Y1_LOW      0x298
#define EDMP_APEX_STATE_PED_Y1_LOW_SIZE 4

/*
 * apex_state_ped_e1
 *
 */
#define EDMP_APEX_STATE_PED_E1      0x29c
#define EDMP_APEX_STATE_PED_E1_SIZE 4

/*
 * apex_state_ped_e2
 *
 */
#define EDMP_APEX_STATE_PED_E2      0x2a0
#define EDMP_APEX_STATE_PED_E2_SIZE 4

/*
 * apex_state_ped_e1_mean
 *
 */
#define EDMP_APEX_STATE_PED_E1_MEAN      0x2a4
#define EDMP_APEX_STATE_PED_E1_MEAN_SIZE 4

/*
 * apex_state_ped_e2_mean
 *
 */
#define EDMP_APEX_STATE_PED_E2_MEAN      0x2a8
#define EDMP_APEX_STATE_PED_E2_MEAN_SIZE 4

/*
 * apex_state_ped_step_buffer
 *
 */
#define EDMP_APEX_STATE_PED_STEP_BUFFER      0x2ac
#define EDMP_APEX_STATE_PED_STEP_BUFFER_SIZE 4

/*
 * apex_state_ped_step_buffer2
 *
 */
#define EDMP_APEX_STATE_PED_STEP_BUFFER2      0x2b0
#define EDMP_APEX_STATE_PED_STEP_BUFFER2_SIZE 4

/*
 * apex_state_ped_timer
 *
 */
#define EDMP_APEX_STATE_PED_TIMER      0x2b4
#define EDMP_APEX_STATE_PED_TIMER_SIZE 2

/*
 * apex_state_ped_sb_timer
 *
 */
#define EDMP_APEX_STATE_PED_SB_TIMER      0x2b6
#define EDMP_APEX_STATE_PED_SB_TIMER_SIZE 2

/*
 * apex_state_ped_min
 *
 */
#define EDMP_APEX_STATE_PED_MIN      0x2b8
#define EDMP_APEX_STATE_PED_MIN_SIZE 4

/*
 * apex_state_ped_max
 *
 */
#define EDMP_APEX_STATE_PED_MAX      0x2bc
#define EDMP_APEX_STATE_PED_MAX_SIZE 4

/*
 * apex_state_ped_current_time
 *
 */
#define EDMP_APEX_STATE_PED_CURRENT_TIME      0x2c0
#define EDMP_APEX_STATE_PED_CURRENT_TIME_SIZE 4

/*
 * apex_state_ped_hid_time
 *
 */
#define EDMP_APEX_STATE_PED_HID_TIME      0x2c4
#define EDMP_APEX_STATE_PED_HID_TIME_SIZE 4

/*
 * apex_state_ped_cadence
 *
 */
#define EDMP_APEX_STATE_PED_CADENCE      0x2c8
#define EDMP_APEX_STATE_PED_CADENCE_SIZE 4

/*
 * apex_state_ped_diff
 *
 */
#define EDMP_APEX_STATE_PED_DIFF      0x2cc
#define EDMP_APEX_STATE_PED_DIFF_SIZE 4

/*
 * apex_state_ped_incr
 *
 */
#define EDMP_APEX_STATE_PED_INCR      0x2d0
#define EDMP_APEX_STATE_PED_INCR_SIZE 4

/*
 * apex_state_ped_mode_counter
 *
 */
#define EDMP_APEX_STATE_PED_MODE_COUNTER      0x2d4
#define EDMP_APEX_STATE_PED_MODE_COUNTER_SIZE 2

/*
 * apex_state_ped_drive_state
 *
 */
#define EDMP_APEX_STATE_PED_DRIVE_STATE      0x2d6
#define EDMP_APEX_STATE_PED_DRIVE_STATE_SIZE 2

/*
 * apex_state_ped_sb_duration
 *
 */
#define EDMP_APEX_STATE_PED_SB_DURATION      0x2d8
#define EDMP_APEX_STATE_PED_SB_DURATION_SIZE 4

/*
 * apex_state_ped_y_in_1_avg
 *
 */
#define EDMP_APEX_STATE_PED_Y_IN_1_AVG      0x2dc
#define EDMP_APEX_STATE_PED_Y_IN_1_AVG_SIZE 4

/*
 * apex_state_ped_y_in_low_1_avg
 *
 */
#define EDMP_APEX_STATE_PED_Y_IN_LOW_1_AVG      0x2e0
#define EDMP_APEX_STATE_PED_Y_IN_LOW_1_AVG_SIZE 4

/*
 * apex_state_ped_y_in_1_avg_running
 *
 */
#define EDMP_APEX_STATE_PED_Y_IN_1_AVG_RUNNING      0x2e4
#define EDMP_APEX_STATE_PED_Y_IN_1_AVG_RUNNING_SIZE 4

/*
 * apex_state_ped_y_in_low_1_avg_running
 *
 */
#define EDMP_APEX_STATE_PED_Y_IN_LOW_1_AVG_RUNNING      0x2e8
#define EDMP_APEX_STATE_PED_Y_IN_LOW_1_AVG_RUNNING_SIZE 4

/*
 * apex_state_ped_flag_band_select
 *
 */
#define EDMP_APEX_STATE_PED_FLAG_BAND_SELECT      0x2ec
#define EDMP_APEX_STATE_PED_FLAG_BAND_SELECT_SIZE 2

/*
 * apex_state_ped_flag_band_select_p
 *
 */
#define EDMP_APEX_STATE_PED_FLAG_BAND_SELECT_P      0x2ee
#define EDMP_APEX_STATE_PED_FLAG_BAND_SELECT_P_SIZE 2

/*
 * apex_state_ped_step_counter_transition
 *
 */
#define EDMP_APEX_STATE_PED_STEP_COUNTER_TRANSITION      0x2f0
#define EDMP_APEX_STATE_PED_STEP_COUNTER_TRANSITION_SIZE 2

/*
 * apex_state_ped_activity_class
 *
 */
#define EDMP_APEX_STATE_PED_ACTIVITY_CLASS      0x2f2
#define EDMP_APEX_STATE_PED_ACTIVITY_CLASS_SIZE 2

/*
 * apex_state_ped_y1_select
 *
 */
#define EDMP_APEX_STATE_PED_Y1_SELECT      0x2f4
#define EDMP_APEX_STATE_PED_Y1_SELECT_SIZE 4

/*
 * apex_state_ped_y2_select
 *
 */
#define EDMP_APEX_STATE_PED_Y2_SELECT      0x2f8
#define EDMP_APEX_STATE_PED_Y2_SELECT_SIZE 4

/*
 * apex_state_ped_previous_sb_threshold
 *
 */
#define EDMP_APEX_STATE_PED_PREVIOUS_SB_THRESHOLD      0x2fc
#define EDMP_APEX_STATE_PED_PREVIOUS_SB_THRESHOLD_SIZE 2

/*
 * apex_state_ped_temp_step_counter
 *
 */
#define EDMP_APEX_STATE_PED_TEMP_STEP_COUNTER      0x2fe
#define EDMP_APEX_STATE_PED_TEMP_STEP_COUNTER_SIZE 2

/*
 * apex_state_ped_step_duration_upper_threshold
 *
 */
#define EDMP_APEX_STATE_PED_STEP_DURATION_UPPER_THRESHOLD      0x300
#define EDMP_APEX_STATE_PED_STEP_DURATION_UPPER_THRESHOLD_SIZE 4

/*
 * apex_state_ped_alpha_dtc
 *
 */
#define EDMP_APEX_STATE_PED_ALPHA_DTC      0x304
#define EDMP_APEX_STATE_PED_ALPHA_DTC_SIZE 4

/*
 * apex_state_ped_alpha_dtc_fast
 *
 */
#define EDMP_APEX_STATE_PED_ALPHA_DTC_FAST      0x308
#define EDMP_APEX_STATE_PED_ALPHA_DTC_FAST_SIZE 4

/*
 * apex_state_ped_B4
 *
 */
#define EDMP_APEX_STATE_PED_B4      0x30c
#define EDMP_APEX_STATE_PED_B4_SIZE 4

/*
 * apex_state_ped_A4
 *
 */
#define EDMP_APEX_STATE_PED_A4      0x310
#define EDMP_APEX_STATE_PED_A4_SIZE 4

/*
 * apex_state_ped_A3
 *
 */
#define EDMP_APEX_STATE_PED_A3      0x314
#define EDMP_APEX_STATE_PED_A3_SIZE 4

/*
 * apex_state_ped_A2
 *
 */
#define EDMP_APEX_STATE_PED_A2      0x318
#define EDMP_APEX_STATE_PED_A2_SIZE 4

/*
 * apex_state_ped_A1
 *
 */
#define EDMP_APEX_STATE_PED_A1      0x31c
#define EDMP_APEX_STATE_PED_A1_SIZE 4

/*
 * apex_state_ped_ER1_B4
 *
 */
#define EDMP_APEX_STATE_PED_ER1_B4      0x320
#define EDMP_APEX_STATE_PED_ER1_B4_SIZE 4

/*
 * apex_state_ped_ER1_A4
 *
 */
#define EDMP_APEX_STATE_PED_ER1_A4      0x324
#define EDMP_APEX_STATE_PED_ER1_A4_SIZE 4

/*
 * apex_state_ped_ER1_A3
 *
 */
#define EDMP_APEX_STATE_PED_ER1_A3      0x328
#define EDMP_APEX_STATE_PED_ER1_A3_SIZE 4

/*
 * apex_state_ped_ER1_A2
 *
 */
#define EDMP_APEX_STATE_PED_ER1_A2      0x32c
#define EDMP_APEX_STATE_PED_ER1_A2_SIZE 4

/*
 * apex_state_ped_ER1_A1
 *
 */
#define EDMP_APEX_STATE_PED_ER1_A1      0x330
#define EDMP_APEX_STATE_PED_ER1_A1_SIZE 4

/*
 * apex_state_ped_ER2_A3
 *
 */
#define EDMP_APEX_STATE_PED_ER2_A3      0x334
#define EDMP_APEX_STATE_PED_ER2_A3_SIZE 4

/*
 * apex_state_ped_ER2_A2
 *
 */
#define EDMP_APEX_STATE_PED_ER2_A2      0x338
#define EDMP_APEX_STATE_PED_ER2_A2_SIZE 4

/*
 * apex_state_ped_ER2_A1
 *
 */
#define EDMP_APEX_STATE_PED_ER2_A1      0x33c
#define EDMP_APEX_STATE_PED_ER2_A1_SIZE 4

/*
 * apex_state_ped_B4_low
 *
 */
#define EDMP_APEX_STATE_PED_B4_LOW      0x340
#define EDMP_APEX_STATE_PED_B4_LOW_SIZE 4

/*
 * apex_state_ped_A4_low
 *
 */
#define EDMP_APEX_STATE_PED_A4_LOW      0x344
#define EDMP_APEX_STATE_PED_A4_LOW_SIZE 4

/*
 * apex_state_ped_A3_low
 *
 */
#define EDMP_APEX_STATE_PED_A3_LOW      0x348
#define EDMP_APEX_STATE_PED_A3_LOW_SIZE 4

/*
 * apex_state_ped_A2_low
 *
 */
#define EDMP_APEX_STATE_PED_A2_LOW      0x34c
#define EDMP_APEX_STATE_PED_A2_LOW_SIZE 4

/*
 * apex_state_ped_A1_low
 *
 */
#define EDMP_APEX_STATE_PED_A1_LOW      0x350
#define EDMP_APEX_STATE_PED_A1_LOW_SIZE 4

/*
 * apex_state_ped_er_alpha
 *
 */
#define EDMP_APEX_STATE_PED_ER_ALPHA      0x354
#define EDMP_APEX_STATE_PED_ER_ALPHA_SIZE 4

/*
 * apex_state_ped_er_ratio_thres0
 *
 */
#define EDMP_APEX_STATE_PED_ER_RATIO_THRES0      0x358
#define EDMP_APEX_STATE_PED_ER_RATIO_THRES0_SIZE 4

/*
 * apex_state_ped_er_ratio_thres1
 *
 */
#define EDMP_APEX_STATE_PED_ER_RATIO_THRES1      0x35c
#define EDMP_APEX_STATE_PED_ER_RATIO_THRES1_SIZE 4

/*
 * apex_state_ped_alpha
 *
 */
#define EDMP_APEX_STATE_PED_ALPHA      0x360
#define EDMP_APEX_STATE_PED_ALPHA_SIZE 4

/*
 * apex_state_ped_delta
 *
 */
#define EDMP_APEX_STATE_PED_DELTA      0x364
#define EDMP_APEX_STATE_PED_DELTA_SIZE 4

/*
 * apex_state_ped_delta2
 *
 */
#define EDMP_APEX_STATE_PED_DELTA2      0x368
#define EDMP_APEX_STATE_PED_DELTA2_SIZE 2

/*
 * apex_state_ped_mode_window
 *
 */
#define EDMP_APEX_STATE_PED_MODE_WINDOW      0x36a
#define EDMP_APEX_STATE_PED_MODE_WINDOW_SIZE 2

/*
 * apex_state_ped_sb_duration_th
 *
 */
#define EDMP_APEX_STATE_PED_SB_DURATION_TH      0x36c
#define EDMP_APEX_STATE_PED_SB_DURATION_TH_SIZE 4

/*
 * apex_state_ped_alpha_a
 *
 */
#define EDMP_APEX_STATE_PED_ALPHA_A      0x370
#define EDMP_APEX_STATE_PED_ALPHA_A_SIZE 4

/*
 * apex_state_ped_y_ratio
 *
 */
#define EDMP_APEX_STATE_PED_Y_RATIO      0x374
#define EDMP_APEX_STATE_PED_Y_RATIO_SIZE 4

/*
 * apex_state_ped_step_counter_transition_th
 *
 */
#define EDMP_APEX_STATE_PED_STEP_COUNTER_TRANSITION_TH      0x378
#define EDMP_APEX_STATE_PED_STEP_COUNTER_TRANSITION_TH_SIZE 2

/*
 * apex_state_ped_param_max_time
 *
 */
#define EDMP_APEX_STATE_PED_PARAM_MAX_TIME      0x37a
#define EDMP_APEX_STATE_PED_PARAM_MAX_TIME_SIZE 2

/*
 * apex_state_ped_param_min_time
 *
 */
#define EDMP_APEX_STATE_PED_PARAM_MIN_TIME      0x37c
#define EDMP_APEX_STATE_PED_PARAM_MIN_TIME_SIZE 2

/*
 * apex_state_ped_sb_threshold
 *
 */
#define EDMP_APEX_STATE_PED_SB_THRESHOLD      0x37e
#define EDMP_APEX_STATE_PED_SB_THRESHOLD_SIZE 2

/*
 * apex_state_ped_sb_threshold2
 *
 */
#define EDMP_APEX_STATE_PED_SB_THRESHOLD2      0x380
#define EDMP_APEX_STATE_PED_SB_THRESHOLD2_SIZE 2

/*
 * apex_state_ped_sb_timer_threshold
 *
 */
#define EDMP_APEX_STATE_PED_SB_TIMER_THRESHOLD      0x382
#define EDMP_APEX_STATE_PED_SB_TIMER_THRESHOLD_SIZE 2

/*
 * apex_state_ped_Low_energy_amp_th
 *
 */
#define EDMP_APEX_STATE_PED_LOW_ENERGY_AMP_TH      0x384
#define EDMP_APEX_STATE_PED_LOW_ENERGY_AMP_TH_SIZE 4

/*
 * apex_state_ped_peak_threshold
 *
 */
#define EDMP_APEX_STATE_PED_PEAK_THRESHOLD      0x388
#define EDMP_APEX_STATE_PED_PEAK_THRESHOLD_SIZE 4

/*
 * apex_state_ped_e1_mean_const
 *
 */
#define EDMP_APEX_STATE_PED_E1_MEAN_CONST      0x38c
#define EDMP_APEX_STATE_PED_E1_MEAN_CONST_SIZE 4

/*
 * apex_state_ped_sensitivity_mode
 *
 */
#define EDMP_APEX_STATE_PED_SENSITIVITY_MODE      0x390
#define EDMP_APEX_STATE_PED_SENSITIVITY_MODE_SIZE 1

/*
 * apex_state_ped_min_time
 *
 */
#define EDMP_APEX_STATE_PED_MIN_TIME      0x392
#define EDMP_APEX_STATE_PED_MIN_TIME_SIZE 2

/*
 * apex_state_ped_max_time
 *
 */
#define EDMP_APEX_STATE_PED_MAX_TIME      0x394
#define EDMP_APEX_STATE_PED_MAX_TIME_SIZE 2

/*
 * apex_state_ped_cadency_thresh_high
 *
 */
#define EDMP_APEX_STATE_PED_CADENCY_THRESH_HIGH      0x398
#define EDMP_APEX_STATE_PED_CADENCY_THRESH_HIGH_SIZE 4

/*
 * apex_state_ped_cadency_thresh_low
 *
 */
#define EDMP_APEX_STATE_PED_CADENCY_THRESH_LOW      0x39c
#define EDMP_APEX_STATE_PED_CADENCY_THRESH_LOW_SIZE 4

/*
 * apex_state_authen
 *
 */
#define EDMP_APEX_STATE_AUTHEN      0x21c
#define EDMP_APEX_STATE_AUTHEN_SIZE 68

/*
 * apex_state_authen_inCipher
 *
 */
#define EDMP_APEX_STATE_AUTHEN_INCIPHER      0x21c
#define EDMP_APEX_STATE_AUTHEN_INCIPHER_SIZE 4

/*
 * apex_state_authen_outCipher
 *
 */
#define EDMP_APEX_STATE_AUTHEN_OUTCIPHER      0x220
#define EDMP_APEX_STATE_AUTHEN_OUTCIPHER_SIZE 4

/*
 * apex_state_authen_in_cipher
 *
 */
#define EDMP_APEX_STATE_AUTHEN_IN_CIPHER      0x224
#define EDMP_APEX_STATE_AUTHEN_IN_CIPHER_SIZE 4

/*
 * apex_state_authen_otp_key
 *
 */
#define EDMP_APEX_STATE_AUTHEN_OTP_KEY      0x228
#define EDMP_APEX_STATE_AUTHEN_OTP_KEY_SIZE 10

/*
 * apex_state_authen_crc
 *
 */
#define EDMP_APEX_STATE_AUTHEN_CRC      0x234
#define EDMP_APEX_STATE_AUTHEN_CRC_SIZE 4

/*
 * apex_state_authen_K0
 *
 */
#define EDMP_APEX_STATE_AUTHEN_K0      0x238
#define EDMP_APEX_STATE_AUTHEN_K0_SIZE 12

/*
 * apex_state_authen_to_hash
 *
 */
#define EDMP_APEX_STATE_AUTHEN_TO_HASH      0x244
#define EDMP_APEX_STATE_AUTHEN_TO_HASH_SIZE 12

/*
 * apex_state_authen_crc32_BE
 *
 */
#define EDMP_APEX_STATE_AUTHEN_CRC32_BE      0x250
#define EDMP_APEX_STATE_AUTHEN_CRC32_BE_SIZE 4

/*
 * apex_state_authen_K0_ipad_BE
 *
 */
#define EDMP_APEX_STATE_AUTHEN_K0_IPAD_BE      0x254
#define EDMP_APEX_STATE_AUTHEN_K0_IPAD_BE_SIZE 4

/*
 * apex_state_authen_K0_opad_BE
 *
 */
#define EDMP_APEX_STATE_AUTHEN_K0_OPAD_BE      0x258
#define EDMP_APEX_STATE_AUTHEN_K0_OPAD_BE_SIZE 4

/*
 * apex_state_authen_reset_value
 *
 */
#define EDMP_APEX_STATE_AUTHEN_RESET_VALUE      0x25c
#define EDMP_APEX_STATE_AUTHEN_RESET_VALUE_SIZE 4

/*
 * apex_state_smd
 *
 */
#define EDMP_APEX_STATE_SMD      0x3a0
#define EDMP_APEX_STATE_SMD_SIZE 132

/*
 * apex_state_smd_er_alpha1
 *
 */
#define EDMP_APEX_STATE_SMD_ER_ALPHA1      0x3a0
#define EDMP_APEX_STATE_SMD_ER_ALPHA1_SIZE 4

/*
 * apex_state_smd_er_alpha3
 *
 */
#define EDMP_APEX_STATE_SMD_ER_ALPHA3      0x3a4
#define EDMP_APEX_STATE_SMD_ER_ALPHA3_SIZE 4

/*
 * apex_state_smd_mode_window
 *
 */
#define EDMP_APEX_STATE_SMD_MODE_WINDOW      0x3a8
#define EDMP_APEX_STATE_SMD_MODE_WINDOW_SIZE 2

/*
 * apex_state_smd_sensitivity
 *
 */
#define EDMP_APEX_STATE_SMD_SENSITIVITY      0x3aa
#define EDMP_APEX_STATE_SMD_SENSITIVITY_SIZE 1

/*
 * apex_state_smd_decim_rate
 *
 */
#define EDMP_APEX_STATE_SMD_DECIM_RATE      0x3ab
#define EDMP_APEX_STATE_SMD_DECIM_RATE_SIZE 1

/*
 * apex_state_smd_decim_count
 *
 */
#define EDMP_APEX_STATE_SMD_DECIM_COUNT      0x3ac
#define EDMP_APEX_STATE_SMD_DECIM_COUNT_SIZE 1

/*
 * apex_state_smd_SMD_flag
 *
 */
#define EDMP_APEX_STATE_SMD_SMD_FLAG      0x3ad
#define EDMP_APEX_STATE_SMD_SMD_FLAG_SIZE 1

/*
 * apex_state_smd_mode_counter
 *
 */
#define EDMP_APEX_STATE_SMD_MODE_COUNTER      0x3ae
#define EDMP_APEX_STATE_SMD_MODE_COUNTER_SIZE 2

/*
 * apex_state_smd_lowVarianceCounter
 *
 */
#define EDMP_APEX_STATE_SMD_LOWVARIANCECOUNTER      0x3b0
#define EDMP_APEX_STATE_SMD_LOWVARIANCECOUNTER_SIZE 4

/*
 * apex_state_smd_SMDDelayCounter
 *
 */
#define EDMP_APEX_STATE_SMD_SMDDELAYCOUNTER      0x3b4
#define EDMP_APEX_STATE_SMD_SMDDELAYCOUNTER_SIZE 4

/*
 * apex_state_smd_e1_short
 *
 */
#define EDMP_APEX_STATE_SMD_E1_SHORT      0x3b8
#define EDMP_APEX_STATE_SMD_E1_SHORT_SIZE 4

/*
 * apex_state_smd_e2_short
 *
 */
#define EDMP_APEX_STATE_SMD_E2_SHORT      0x3bc
#define EDMP_APEX_STATE_SMD_E2_SHORT_SIZE 4

/*
 * apex_state_smd_var_run_temp
 *
 */
#define EDMP_APEX_STATE_SMD_VAR_RUN_TEMP      0x3c0
#define EDMP_APEX_STATE_SMD_VAR_RUN_TEMP_SIZE 4

/*
 * apex_state_smd_z_mean
 *
 */
#define EDMP_APEX_STATE_SMD_Z_MEAN      0x3c4
#define EDMP_APEX_STATE_SMD_Z_MEAN_SIZE 4

/*
 * apex_state_smd_z_incr
 *
 */
#define EDMP_APEX_STATE_SMD_Z_INCR      0x3c8
#define EDMP_APEX_STATE_SMD_Z_INCR_SIZE 4

/*
 * apex_state_smd_state_m_f
 *
 */
#define EDMP_APEX_STATE_SMD_STATE_M_F      0x3cc
#define EDMP_APEX_STATE_SMD_STATE_M_F_SIZE 4

/*
 * apex_state_smd_quat_wb1
 *
 */
#define EDMP_APEX_STATE_SMD_QUAT_WB1      0x3d0
#define EDMP_APEX_STATE_SMD_QUAT_WB1_SIZE 4

/*
 * apex_state_smd_quat_wb2
 *
 */
#define EDMP_APEX_STATE_SMD_QUAT_WB2      0x3d4
#define EDMP_APEX_STATE_SMD_QUAT_WB2_SIZE 4

/*
 * apex_state_smd_quat_wb3
 *
 */
#define EDMP_APEX_STATE_SMD_QUAT_WB3      0x3d8
#define EDMP_APEX_STATE_SMD_QUAT_WB3_SIZE 4

/*
 * apex_state_smd_motion_flag
 *
 */
#define EDMP_APEX_STATE_SMD_MOTION_FLAG      0x3dc
#define EDMP_APEX_STATE_SMD_MOTION_FLAG_SIZE 4

/*
 * apex_state_smd_walk_candidate
 *
 */
#define EDMP_APEX_STATE_SMD_WALK_CANDIDATE      0x3e0
#define EDMP_APEX_STATE_SMD_WALK_CANDIDATE_SIZE 4

/*
 * apex_state_smd_tilt_angle
 *
 */
#define EDMP_APEX_STATE_SMD_TILT_ANGLE      0x3e4
#define EDMP_APEX_STATE_SMD_TILT_ANGLE_SIZE 4

/*
 * apex_state_smd_prev_quat_wb1
 *
 */
#define EDMP_APEX_STATE_SMD_PREV_QUAT_WB1      0x3e8
#define EDMP_APEX_STATE_SMD_PREV_QUAT_WB1_SIZE 4

/*
 * apex_state_smd_prev_quat_wb2
 *
 */
#define EDMP_APEX_STATE_SMD_PREV_QUAT_WB2      0x3ec
#define EDMP_APEX_STATE_SMD_PREV_QUAT_WB2_SIZE 4

/*
 * apex_state_smd_prev_quat_wb3
 *
 */
#define EDMP_APEX_STATE_SMD_PREV_QUAT_WB3      0x3f0
#define EDMP_APEX_STATE_SMD_PREV_QUAT_WB3_SIZE 4

/*
 * apex_state_smd_TILT_init_flag
 *
 */
#define EDMP_APEX_STATE_SMD_TILT_INIT_FLAG      0x3f4
#define EDMP_APEX_STATE_SMD_TILT_INIT_FLAG_SIZE 4

/*
 * apex_state_smd_vibration
 *
 */
#define EDMP_APEX_STATE_SMD_VIBRATION      0x3f8
#define EDMP_APEX_STATE_SMD_VIBRATION_SIZE 44

/*
 * apex_state_smd_vibration_vibration_detection_ratio
 *
 */
#define EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION_RATIO      0x3f8
#define EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION_RATIO_SIZE 4

/*
 * apex_state_smd_vibration_vibration_magnitude_threshold
 *
 */
#define EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_MAGNITUDE_THRESHOLD      0x3fc
#define EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_MAGNITUDE_THRESHOLD_SIZE 4

/*
 * apex_state_smd_vibration_k
 *
 */
#define EDMP_APEX_STATE_SMD_VIBRATION_K      0x400
#define EDMP_APEX_STATE_SMD_VIBRATION_K_SIZE 4

/*
 * apex_state_smd_vibration_k_1
 *
 */
#define EDMP_APEX_STATE_SMD_VIBRATION_K_1      0x404
#define EDMP_APEX_STATE_SMD_VIBRATION_K_1_SIZE 4

/*
 * apex_state_smd_vibration_accel_q25_prev
 *
 */
#define EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV      0x408
#define EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV_SIZE 12

/*
 * apex_state_smd_vibration_accel_delta_lpf
 *
 */
#define EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF      0x414
#define EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF_SIZE 12

/*
 * apex_state_smd_vibration_vibration_detection
 *
 */
#define EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION      0x420
#define EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION_SIZE 4

/*
 * apex_state_b2s
 *
 */
#define EDMP_APEX_STATE_B2S      0x424
#define EDMP_APEX_STATE_B2S_SIZE 184

/*
 * apex_state_b2s_decim_count
 *
 */
#define EDMP_APEX_STATE_B2S_DECIM_COUNT      0x424
#define EDMP_APEX_STATE_B2S_DECIM_COUNT_SIZE 1

/*
 * apex_state_b2s_decim_rate
 *
 */
#define EDMP_APEX_STATE_B2S_DECIM_RATE      0x425
#define EDMP_APEX_STATE_B2S_DECIM_RATE_SIZE 1

/*
 * apex_state_b2s_acc_1g_q12
 *
 */
#define EDMP_APEX_STATE_B2S_ACC_1G_Q12      0x426
#define EDMP_APEX_STATE_B2S_ACC_1G_Q12_SIZE 6

/*
 * apex_state_b2s_setting
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING      0x42c
#define EDMP_APEX_STATE_B2S_SETTING_SIZE 52

/*
 * apex_state_b2s_setting_mounting_matrix
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_MOUNTING_MATRIX      0x42c
#define EDMP_APEX_STATE_B2S_SETTING_MOUNTING_MATRIX_SIZE 1

/*
 * apex_state_b2s_setting_lOneGValue
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_LONEGVALUE      0x42e
#define EDMP_APEX_STATE_B2S_SETTING_LONEGVALUE_SIZE 2

/*
 * apex_state_b2s_setting_DevNormMax
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_DEVNORMMAX      0x430
#define EDMP_APEX_STATE_B2S_SETTING_DEVNORMMAX_SIZE 4

/*
 * apex_state_b2s_setting_SinLimit
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_SINLIMIT      0x434
#define EDMP_APEX_STATE_B2S_SETTING_SINLIMIT_SIZE 4

/*
 * apex_state_b2s_setting_FastMotionAgeLimit
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_FASTMOTIONAGELIMIT      0x438
#define EDMP_APEX_STATE_B2S_SETTING_FASTMOTIONAGELIMIT_SIZE 4

/*
 * apex_state_b2s_setting_FastLimit
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_FASTLIMIT      0x43c
#define EDMP_APEX_STATE_B2S_SETTING_FASTLIMIT_SIZE 4

/*
 * apex_state_b2s_setting_FastMotionTimeLimit
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_FASTMOTIONTIMELIMIT      0x440
#define EDMP_APEX_STATE_B2S_SETTING_FASTMOTIONTIMELIMIT_SIZE 4

/*
 * apex_state_b2s_setting_BringToSeeAgeLimit
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_BRINGTOSEEAGELIMIT      0x444
#define EDMP_APEX_STATE_B2S_SETTING_BRINGTOSEEAGELIMIT_SIZE 4

/*
 * apex_state_b2s_setting_StaticLimit
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_STATICLIMIT      0x448
#define EDMP_APEX_STATE_B2S_SETTING_STATICLIMIT_SIZE 4

/*
 * apex_state_b2s_setting_ThrCosAng
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_THRCOSANG      0x44c
#define EDMP_APEX_STATE_B2S_SETTING_THRCOSANG_SIZE 4

/*
 * apex_state_b2s_setting_RevB2sLatencyTh
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_REVB2SLATENCYTH      0x450
#define EDMP_APEX_STATE_B2S_SETTING_REVB2SLATENCYTH_SIZE 4

/*
 * apex_state_b2s_setting_ReductedNormThreshold
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_REDUCTEDNORMTHRESHOLD      0x454
#define EDMP_APEX_STATE_B2S_SETTING_REDUCTEDNORMTHRESHOLD_SIZE 4

/*
 * apex_state_b2s_setting_LimitInf
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_LIMITINF      0x458
#define EDMP_APEX_STATE_B2S_SETTING_LIMITINF_SIZE 4

/*
 * apex_state_b2s_setting_LimitSup
 *
 */
#define EDMP_APEX_STATE_B2S_SETTING_LIMITSUP      0x45c
#define EDMP_APEX_STATE_B2S_SETTING_LIMITSUP_SIZE 4

/*
 * apex_state_b2s_state
 *
 */
#define EDMP_APEX_STATE_B2S_STATE      0x460
#define EDMP_APEX_STATE_B2S_STATE_SIZE 124

/*
 * apex_state_b2s_state_acc_prec
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_ACC_PREC      0x460
#define EDMP_APEX_STATE_B2S_STATE_ACC_PREC_SIZE 48

/*
 * apex_state_b2s_state_SampleCounter
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_SAMPLECOUNTER      0x490
#define EDMP_APEX_STATE_B2S_STATE_SAMPLECOUNTER_SIZE 4

/*
 * apex_state_b2s_state_Bts
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS      0x494
#define EDMP_APEX_STATE_B2S_STATE_BTS_SIZE 20

/*
 * apex_state_b2s_state_Bts_B2S_Event
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS_B2S_EVENT      0x494
#define EDMP_APEX_STATE_B2S_STATE_BTS_B2S_EVENT_SIZE 1

/*
 * apex_state_b2s_state_Bts_revB2S_Event
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS_REVB2S_EVENT      0x495
#define EDMP_APEX_STATE_B2S_STATE_BTS_REVB2S_EVENT_SIZE 1

/*
 * apex_state_b2s_state_Bts_GoodEventCounter
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS_GOODEVENTCOUNTER      0x496
#define EDMP_APEX_STATE_B2S_STATE_BTS_GOODEVENTCOUNTER_SIZE 2

/*
 * apex_state_b2s_state_Bts_RevB2SEventCounter
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS_REVB2SEVENTCOUNTER      0x498
#define EDMP_APEX_STATE_B2S_STATE_BTS_REVB2SEVENTCOUNTER_SIZE 2

/*
 * apex_state_b2s_state_Bts_RevB2sLatencyCount
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS_REVB2SLATENCYCOUNT      0x49a
#define EDMP_APEX_STATE_B2S_STATE_BTS_REVB2SLATENCYCOUNT_SIZE 2

/*
 * apex_state_b2s_state_Bts_FastMotionAge
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS_FASTMOTIONAGE      0x49c
#define EDMP_APEX_STATE_B2S_STATE_BTS_FASTMOTIONAGE_SIZE 4

/*
 * apex_state_b2s_state_Bts_FastTime
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS_FASTTIME      0x4a0
#define EDMP_APEX_STATE_B2S_STATE_BTS_FASTTIME_SIZE 4

/*
 * apex_state_b2s_state_Bts_BringToSeeAge
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_BTS_BRINGTOSEEAGE      0x4a4
#define EDMP_APEX_STATE_B2S_STATE_BTS_BRINGTOSEEAGE_SIZE 4

/*
 * apex_state_b2s_state_uiDeviation
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_UIDEVIATION      0x4a8
#define EDMP_APEX_STATE_B2S_STATE_UIDEVIATION_SIZE 4

/*
 * apex_state_b2s_state_iNorm2
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_INORM2      0x4ac
#define EDMP_APEX_STATE_B2S_STATE_INORM2_SIZE 4

/*
 * apex_state_b2s_state_DevByAxe
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_DEVBYAXE      0x4b0
#define EDMP_APEX_STATE_B2S_STATE_DEVBYAXE_SIZE 12

/*
 * apex_state_b2s_state_orientation
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_ORIENTATION      0x4bc
#define EDMP_APEX_STATE_B2S_STATE_ORIENTATION_SIZE 12

/*
 * apex_state_b2s_state_orientation_ref
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_ORIENTATION_REF      0x4c8
#define EDMP_APEX_STATE_B2S_STATE_ORIENTATION_REF_SIZE 12

/*
 * apex_state_b2s_state_reject_interrupt
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_REJECT_INTERRUPT      0x4d4
#define EDMP_APEX_STATE_B2S_STATE_REJECT_INTERRUPT_SIZE 2

/*
 * apex_state_b2s_state_update_ref
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_UPDATE_REF      0x4d6
#define EDMP_APEX_STATE_B2S_STATE_UPDATE_REF_SIZE 2

/*
 * apex_state_b2s_state_enable_revb2s
 *
 */
#define EDMP_APEX_STATE_B2S_STATE_ENABLE_REVB2S      0x4d8
#define EDMP_APEX_STATE_B2S_STATE_ENABLE_REVB2S_SIZE 2

/*
 * apex_state_shake
 *
 */
#define EDMP_APEX_STATE_SHAKE      0x4dc
#define EDMP_APEX_STATE_SHAKE_SIZE 58

/*
 * apex_state_shake_decim_rate
 *
 */
#define EDMP_APEX_STATE_SHAKE_DECIM_RATE      0x4dc
#define EDMP_APEX_STATE_SHAKE_DECIM_RATE_SIZE 1

/*
 * apex_state_shake_decim_count
 *
 */
#define EDMP_APEX_STATE_SHAKE_DECIM_COUNT      0x4dd
#define EDMP_APEX_STATE_SHAKE_DECIM_COUNT_SIZE 1

/*
 * apex_state_shake_outInterrupt
 *
 */
#define EDMP_APEX_STATE_SHAKE_OUTINTERRUPT      0x4de
#define EDMP_APEX_STATE_SHAKE_OUTINTERRUPT_SIZE 1

/*
 * apex_state_shake_shake_candidate_x
 *
 */
#define EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_X      0x4df
#define EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_X_SIZE 1

/*
 * apex_state_shake_shake_candidate_y
 *
 */
#define EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_Y      0x4e0
#define EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_Y_SIZE 1

/*
 * apex_state_shake_shake_candidate_z
 *
 */
#define EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_Z      0x4e1
#define EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_Z_SIZE 1

/*
 * apex_state_shake_previous_state
 *
 */
#define EDMP_APEX_STATE_SHAKE_PREVIOUS_STATE      0x4e2
#define EDMP_APEX_STATE_SHAKE_PREVIOUS_STATE_SIZE 1

/*
 * apex_state_shake_still
 *
 */
#define EDMP_APEX_STATE_SHAKE_STILL      0x4e4
#define EDMP_APEX_STATE_SHAKE_STILL_SIZE 6

/*
 * apex_state_shake_stabilityCount
 *
 */
#define EDMP_APEX_STATE_SHAKE_STABILITYCOUNT      0x4ea
#define EDMP_APEX_STATE_SHAKE_STABILITYCOUNT_SIZE 2

/*
 * apex_state_shake_C1
 *
 */
#define EDMP_APEX_STATE_SHAKE_C1      0x4ec
#define EDMP_APEX_STATE_SHAKE_C1_SIZE 2

/*
 * apex_state_shake_general_shake_timer
 *
 */
#define EDMP_APEX_STATE_SHAKE_GENERAL_SHAKE_TIMER      0x4ee
#define EDMP_APEX_STATE_SHAKE_GENERAL_SHAKE_TIMER_SIZE 2

/*
 * apex_state_shake_shake_candidate
 *
 */
#define EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE      0x4f0
#define EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_SIZE 2

/*
 * apex_state_shake_timer
 *
 */
#define EDMP_APEX_STATE_SHAKE_TIMER      0x4f2
#define EDMP_APEX_STATE_SHAKE_TIMER_SIZE 2

/*
 * apex_state_shake_min_value
 *
 */
#define EDMP_APEX_STATE_SHAKE_MIN_VALUE      0x4f4
#define EDMP_APEX_STATE_SHAKE_MIN_VALUE_SIZE 6

/*
 * apex_state_shake_max_value
 *
 */
#define EDMP_APEX_STATE_SHAKE_MAX_VALUE      0x4fa
#define EDMP_APEX_STATE_SHAKE_MAX_VALUE_SIZE 6

/*
 * apex_state_shake_config
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG      0x500
#define EDMP_APEX_STATE_SHAKE_CONFIG_SIZE 22

/*
 * apex_state_shake_config_shake_axis_majority_th
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_SHAKE_AXIS_MAJORITY_TH      0x500
#define EDMP_APEX_STATE_SHAKE_CONFIG_SHAKE_AXIS_MAJORITY_TH_SIZE 2

/*
 * apex_state_shake_config_thresh_static
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_STATIC      0x502
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_STATIC_SIZE 2

/*
 * apex_state_shake_config_minimal_static_duration
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_MINIMAL_STATIC_DURATION      0x504
#define EDMP_APEX_STATE_SHAKE_CONFIG_MINIMAL_STATIC_DURATION_SIZE 2

/*
 * apex_state_shake_config_thresh_motion
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_MOTION      0x506
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_MOTION_SIZE 2

/*
 * apex_state_shake_config_thresh_shake_x
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_SHAKE_X      0x508
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_SHAKE_X_SIZE 2

/*
 * apex_state_shake_config_thresh_shake_y
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_SHAKE_Y      0x50a
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_SHAKE_Y_SIZE 2

/*
 * apex_state_shake_config_thresh_shake_z
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_SHAKE_Z      0x50c
#define EDMP_APEX_STATE_SHAKE_CONFIG_THRESH_SHAKE_Z_SIZE 2

/*
 * apex_state_shake_config_minimal_shake_duration
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_MINIMAL_SHAKE_DURATION      0x50e
#define EDMP_APEX_STATE_SHAKE_CONFIG_MINIMAL_SHAKE_DURATION_SIZE 2

/*
 * apex_state_shake_config_maximal_shake_duration
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_MAXIMAL_SHAKE_DURATION      0x510
#define EDMP_APEX_STATE_SHAKE_CONFIG_MAXIMAL_SHAKE_DURATION_SIZE 2

/*
 * apex_state_shake_config_internal_duration
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_INTERNAL_DURATION      0x512
#define EDMP_APEX_STATE_SHAKE_CONFIG_INTERNAL_DURATION_SIZE 2

/*
 * apex_state_shake_config_axis_detection_factor
 *
 */
#define EDMP_APEX_STATE_SHAKE_CONFIG_AXIS_DETECTION_FACTOR      0x514
#define EDMP_APEX_STATE_SHAKE_CONFIG_AXIS_DETECTION_FACTOR_SIZE 2

/*
 * apex_state_noMotion
 *
 */
#define EDMP_APEX_STATE_NOMOTION      0x516
#define EDMP_APEX_STATE_NOMOTION_SIZE 18

/*
 * apex_state_noMotion_decim_rate
 *
 */
#define EDMP_APEX_STATE_NOMOTION_DECIM_RATE      0x516
#define EDMP_APEX_STATE_NOMOTION_DECIM_RATE_SIZE 1

/*
 * apex_state_noMotion_decim_count
 *
 */
#define EDMP_APEX_STATE_NOMOTION_DECIM_COUNT      0x517
#define EDMP_APEX_STATE_NOMOTION_DECIM_COUNT_SIZE 1

/*
 * apex_state_noMotion_outInterrupt
 *
 */
#define EDMP_APEX_STATE_NOMOTION_OUTINTERRUPT      0x518
#define EDMP_APEX_STATE_NOMOTION_OUTINTERRUPT_SIZE 1

/*
 * apex_state_noMotion_previous_state
 *
 */
#define EDMP_APEX_STATE_NOMOTION_PREVIOUS_STATE      0x519
#define EDMP_APEX_STATE_NOMOTION_PREVIOUS_STATE_SIZE 1

/*
 * apex_state_noMotion_still
 *
 */
#define EDMP_APEX_STATE_NOMOTION_STILL      0x51a
#define EDMP_APEX_STATE_NOMOTION_STILL_SIZE 6

/*
 * apex_state_noMotion_stabilityCount
 *
 */
#define EDMP_APEX_STATE_NOMOTION_STABILITYCOUNT      0x520
#define EDMP_APEX_STATE_NOMOTION_STABILITYCOUNT_SIZE 2

/*
 * apex_state_noMotion_config
 *
 */
#define EDMP_APEX_STATE_NOMOTION_CONFIG      0x522
#define EDMP_APEX_STATE_NOMOTION_CONFIG_SIZE 6

/*
 * apex_state_noMotion_config_thresh_static
 *
 */
#define EDMP_APEX_STATE_NOMOTION_CONFIG_THRESH_STATIC      0x522
#define EDMP_APEX_STATE_NOMOTION_CONFIG_THRESH_STATIC_SIZE 2

/*
 * apex_state_noMotion_config_minimal_static_duration
 *
 */
#define EDMP_APEX_STATE_NOMOTION_CONFIG_MINIMAL_STATIC_DURATION      0x524
#define EDMP_APEX_STATE_NOMOTION_CONFIG_MINIMAL_STATIC_DURATION_SIZE 2

/*
 * apex_state_noMotion_config_thresh_motion
 *
 */
#define EDMP_APEX_STATE_NOMOTION_CONFIG_THRESH_MOTION      0x526
#define EDMP_APEX_STATE_NOMOTION_CONFIG_THRESH_MOTION_SIZE 2

/*
 * apex_state_3axisB
 *
 */
#define EDMP_APEX_STATE_3AXISB      0x528
#define EDMP_APEX_STATE_3AXISB_SIZE 48

/*
 * apex_state_3axisB_decim_rate
 *
 */
#define EDMP_APEX_STATE_3AXISB_DECIM_RATE      0x528
#define EDMP_APEX_STATE_3AXISB_DECIM_RATE_SIZE 1

/*
 * apex_state_3axisB_decim_count
 *
 */
#define EDMP_APEX_STATE_3AXISB_DECIM_COUNT      0x529
#define EDMP_APEX_STATE_3AXISB_DECIM_COUNT_SIZE 1

/*
 * apex_state_3axisB_init
 *
 */
#define EDMP_APEX_STATE_3AXISB_INIT      0x52a
#define EDMP_APEX_STATE_3AXISB_INIT_SIZE 2

/*
 * apex_state_3axisB_quat
 *
 */
#define EDMP_APEX_STATE_3AXISB_QUAT      0x52c
#define EDMP_APEX_STATE_3AXISB_QUAT_SIZE 16

/*
 * apex_state_3axisB_gain
 *
 */
#define EDMP_APEX_STATE_3AXISB_GAIN      0x53c
#define EDMP_APEX_STATE_3AXISB_GAIN_SIZE 4

/*
 * apex_state_3axisB_qfeedbk
 *
 */
#define EDMP_APEX_STATE_3AXISB_QFEEDBK      0x540
#define EDMP_APEX_STATE_3AXISB_QFEEDBK_SIZE 8

/*
 * apex_state_3axisB_accel_wx
 *
 */
#define EDMP_APEX_STATE_3AXISB_ACCEL_WX      0x548
#define EDMP_APEX_STATE_3AXISB_ACCEL_WX_SIZE 4

/*
 * apex_state_3axisB_accel_wy
 *
 */
#define EDMP_APEX_STATE_3AXISB_ACCEL_WY      0x54c
#define EDMP_APEX_STATE_3AXISB_ACCEL_WY_SIZE 4

/*
 * apex_state_3axisB_accel_wz
 *
 */
#define EDMP_APEX_STATE_3AXISB_ACCEL_WZ      0x550
#define EDMP_APEX_STATE_3AXISB_ACCEL_WZ_SIZE 4

/*
 * apex_state_3axisB_accel_only_gain
 *
 */
#define EDMP_APEX_STATE_3AXISB_ACCEL_ONLY_GAIN      0x554
#define EDMP_APEX_STATE_3AXISB_ACCEL_ONLY_GAIN_SIZE 4

/*
 * apex_state_flat
 *
 */
#define EDMP_APEX_STATE_FLAT      0x558
#define EDMP_APEX_STATE_FLAT_SIZE 52

/*
 * apex_state_flat_decim_rate
 *
 */
#define EDMP_APEX_STATE_FLAT_DECIM_RATE      0x558
#define EDMP_APEX_STATE_FLAT_DECIM_RATE_SIZE 1

/*
 * apex_state_flat_decim_count
 *
 */
#define EDMP_APEX_STATE_FLAT_DECIM_COUNT      0x559
#define EDMP_APEX_STATE_FLAT_DECIM_COUNT_SIZE 1

/*
 * apex_state_flat_flatInterrupt
 *
 */
#define EDMP_APEX_STATE_FLAT_FLATINTERRUPT      0x55a
#define EDMP_APEX_STATE_FLAT_FLATINTERRUPT_SIZE 1

/*
 * apex_state_flat_noFlatInterrupt
 *
 */
#define EDMP_APEX_STATE_FLAT_NOFLATINTERRUPT      0x55b
#define EDMP_APEX_STATE_FLAT_NOFLATINTERRUPT_SIZE 1

/*
 * apex_state_flat_state
 *
 */
#define EDMP_APEX_STATE_FLAT_STATE      0x55c
#define EDMP_APEX_STATE_FLAT_STATE_SIZE 1

/*
 * apex_state_flat_init_timer
 *
 */
#define EDMP_APEX_STATE_FLAT_INIT_TIMER      0x55e
#define EDMP_APEX_STATE_FLAT_INIT_TIMER_SIZE 2

/*
 * apex_state_flat_timer
 *
 */
#define EDMP_APEX_STATE_FLAT_TIMER      0x560
#define EDMP_APEX_STATE_FLAT_TIMER_SIZE 2

/*
 * apex_state_flat_quat_wb_x
 *
 */
#define EDMP_APEX_STATE_FLAT_QUAT_WB_X      0x564
#define EDMP_APEX_STATE_FLAT_QUAT_WB_X_SIZE 4

/*
 * apex_state_flat_quat_wb_y
 *
 */
#define EDMP_APEX_STATE_FLAT_QUAT_WB_Y      0x568
#define EDMP_APEX_STATE_FLAT_QUAT_WB_Y_SIZE 4

/*
 * apex_state_flat_quat_wb_z
 *
 */
#define EDMP_APEX_STATE_FLAT_QUAT_WB_Z      0x56c
#define EDMP_APEX_STATE_FLAT_QUAT_WB_Z_SIZE 4

/*
 * apex_state_flat_cos_flat_angle
 *
 */
#define EDMP_APEX_STATE_FLAT_COS_FLAT_ANGLE      0x570
#define EDMP_APEX_STATE_FLAT_COS_FLAT_ANGLE_SIZE 4

/*
 * apex_state_flat_config
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG      0x574
#define EDMP_APEX_STATE_FLAT_CONFIG_SIZE 24

/*
 * apex_state_flat_config_ref_axis_selection
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG_REF_AXIS_SELECTION      0x574
#define EDMP_APEX_STATE_FLAT_CONFIG_REF_AXIS_SELECTION_SIZE 2

/*
 * apex_state_flat_config_isSymetrical
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG_ISSYMETRICAL      0x576
#define EDMP_APEX_STATE_FLAT_CONFIG_ISSYMETRICAL_SIZE 2

/*
 * apex_state_flat_config_three_axis_conv_time
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG_THREE_AXIS_CONV_TIME      0x578
#define EDMP_APEX_STATE_FLAT_CONFIG_THREE_AXIS_CONV_TIME_SIZE 2

/*
 * apex_state_flat_config_wait_time
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG_WAIT_TIME      0x57a
#define EDMP_APEX_STATE_FLAT_CONFIG_WAIT_TIME_SIZE 2

/*
 * apex_state_flat_config_cos_flat_angle_th
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG_COS_FLAT_ANGLE_TH      0x57c
#define EDMP_APEX_STATE_FLAT_CONFIG_COS_FLAT_ANGLE_TH_SIZE 4

/*
 * apex_state_flat_config_ref_axis_x
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG_REF_AXIS_X      0x580
#define EDMP_APEX_STATE_FLAT_CONFIG_REF_AXIS_X_SIZE 4

/*
 * apex_state_flat_config_ref_axis_y
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG_REF_AXIS_Y      0x584
#define EDMP_APEX_STATE_FLAT_CONFIG_REF_AXIS_Y_SIZE 4

/*
 * apex_state_flat_config_ref_axis_z
 *
 */
#define EDMP_APEX_STATE_FLAT_CONFIG_REF_AXIS_Z      0x588
#define EDMP_APEX_STATE_FLAT_CONFIG_REF_AXIS_Z_SIZE 4

/*
 * apex_features_en0
 *
 */
#define EDMP_APEX_FEATURES_EN0      0x58c
#define EDMP_APEX_FEATURES_EN0_SIZE 1

/*
 * apex_features_en1
 *
 */
#define EDMP_APEX_FEATURES_EN1      0x58d
#define EDMP_APEX_FEATURES_EN1_SIZE 1

/*
 * apex_edmp_odr
 *
 */
#define EDMP_APEX_EDMP_ODR      0x58e
#define EDMP_APEX_EDMP_ODR_SIZE 1

/*
 * apex_int_status_register
 *
 */
#define EDMP_APEX_INT_STATUS_REGISTER      0x58f
#define EDMP_APEX_INT_STATUS_REGISTER_SIZE 1

/*
 * apex_edmp2host_int_reg0
 *
 */
#define EDMP_APEX_EDMP2HOST_INT_REG0      0x590
#define EDMP_APEX_EDMP2HOST_INT_REG0_SIZE 1

/*
 * apex_edmp2host_int_reg1
 *
 */
#define EDMP_APEX_EDMP2HOST_INT_REG1      0x591
#define EDMP_APEX_EDMP2HOST_INT_REG1_SIZE 1

/*
 * apex_edmp2host_int_reg2
 *
 */
#define EDMP_APEX_EDMP2HOST_INT_REG2      0x592
#define EDMP_APEX_EDMP2HOST_INT_REG2_SIZE 1

/*
 * apex_tmpVars
 *
 */
#define EDMP_APEX_TMPVARS      0x594
#define EDMP_APEX_TMPVARS_SIZE 16

/*
 * apex_debugVars
 *
 */
#define EDMP_APEX_DEBUGVARS      0x5a4
#define EDMP_APEX_DEBUGVARS_SIZE 8

/*
 * selfTestCal_dataMem
 *
 */
#define EDMP_SELFTESTCAL_DATAMEM      0x0
#define EDMP_SELFTESTCAL_DATAMEM_SIZE 392

/*
 * selfTestCal_otpData
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA      0x0
#define EDMP_SELFTESTCAL_OTPDATA_SIZE 38

/*
 * selfTestCal_otpData_gyro_x_str_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_X_STR_FT      0x0
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_X_STR_FT_SIZE 2

/*
 * selfTestCal_otpData_gyro_y_str_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_Y_STR_FT      0x2
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_Y_STR_FT_SIZE 2

/*
 * selfTestCal_otpData_gyro_z_str_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_Z_STR_FT      0x4
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_Z_STR_FT_SIZE 2

/*
 * selfTestCal_otpData_gyro_x_cmos_gain_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_X_CMOS_GAIN_FT      0x6
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_X_CMOS_GAIN_FT_SIZE 2

/*
 * selfTestCal_otpData_gyro_y_cmos_gain_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_Y_CMOS_GAIN_FT      0x8
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_Y_CMOS_GAIN_FT_SIZE 2

/*
 * selfTestCal_otpData_gyro_z_cmos_gain_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_Z_CMOS_GAIN_FT      0xa
#define EDMP_SELFTESTCAL_OTPDATA_GYRO_Z_CMOS_GAIN_FT_SIZE 2

/*
 * selfTestCal_otpData_accel_x_str_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_X_STR_FT      0xc
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_X_STR_FT_SIZE 2

/*
 * selfTestCal_otpData_accel_y_str_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_Y_STR_FT      0xe
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_Y_STR_FT_SIZE 2

/*
 * selfTestCal_otpData_accel_z_str_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_Z_STR_FT      0x10
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_Z_STR_FT_SIZE 2

/*
 * selfTestCal_otpData_accel_x_sc_nout_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_X_SC_NOUT_FT      0x12
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_X_SC_NOUT_FT_SIZE 2

/*
 * selfTestCal_otpData_accel_y_sc_nout_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_Y_SC_NOUT_FT      0x14
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_Y_SC_NOUT_FT_SIZE 2

/*
 * selfTestCal_otpData_accel_z_sc_nout_ft
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_Z_SC_NOUT_FT      0x16
#define EDMP_SELFTESTCAL_OTPDATA_ACCEL_Z_SC_NOUT_FT_SIZE 2

/*
 * selfTestCal_otpData_self_cal_ft_ver
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_SELF_CAL_FT_VER      0x18
#define EDMP_SELFTESTCAL_OTPDATA_SELF_CAL_FT_VER_SIZE 1

/*
 * selfTestCal_otpData_manu_id
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_MANU_ID      0x19
#define EDMP_SELFTESTCAL_OTPDATA_MANU_ID_SIZE 1

/*
 * selfTestCal_otpData_chip_id
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_CHIP_ID      0x1a
#define EDMP_SELFTESTCAL_OTPDATA_CHIP_ID_SIZE 1

/*
 * selfTestCal_otpData_rev_id
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_REV_ID      0x1b
#define EDMP_SELFTESTCAL_OTPDATA_REV_ID_SIZE 1

/*
 * selfTestCal_otpData_asic_wafer_id
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ASIC_WAFER_ID      0x1c
#define EDMP_SELFTESTCAL_OTPDATA_ASIC_WAFER_ID_SIZE 1

/*
 * selfTestCal_otpData_asic_die_id_0
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ASIC_DIE_ID_0      0x1d
#define EDMP_SELFTESTCAL_OTPDATA_ASIC_DIE_ID_0_SIZE 1

/*
 * selfTestCal_otpData_asic_die_id_1
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_ASIC_DIE_ID_1      0x1e
#define EDMP_SELFTESTCAL_OTPDATA_ASIC_DIE_ID_1_SIZE 1

/*
 * selfTestCal_otpData_mems_wafer_id
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_MEMS_WAFER_ID      0x1f
#define EDMP_SELFTESTCAL_OTPDATA_MEMS_WAFER_ID_SIZE 1

/*
 * selfTestCal_otpData_mems_die_id_0
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_MEMS_DIE_ID_0      0x20
#define EDMP_SELFTESTCAL_OTPDATA_MEMS_DIE_ID_0_SIZE 1

/*
 * selfTestCal_otpData_mems_die_id_1
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_MEMS_DIE_ID_1      0x21
#define EDMP_SELFTESTCAL_OTPDATA_MEMS_DIE_ID_1_SIZE 1

/*
 * selfTestCal_otpData_sw_rcosc_trim
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_SW_RCOSC_TRIM      0x22
#define EDMP_SELFTESTCAL_OTPDATA_SW_RCOSC_TRIM_SIZE 1

/*
 * selfTestCal_otpData_sw_pll_trim
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_SW_PLL_TRIM      0x23
#define EDMP_SELFTESTCAL_OTPDATA_SW_PLL_TRIM_SIZE 1

/*
 * selfTestCal_otpData_sa_key_last_byte
 *
 */
#define EDMP_SELFTESTCAL_OTPDATA_SA_KEY_LAST_BYTE      0x24
#define EDMP_SELFTESTCAL_OTPDATA_SA_KEY_LAST_BYTE_SIZE 1

/*
 * selfTestCal_debugVars
 *
 */
#define EDMP_SELFTESTCAL_DEBUGVARS      0x28
#define EDMP_SELFTESTCAL_DEBUGVARS_SIZE 16

/*
 * selfTestCal_state_selfTestCal
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL      0x38
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_SIZE 336

/*
 * selfTestCal_state_selfTestCal_configParams
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_CONFIGPARAMS      0x38
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_CONFIGPARAMS_SIZE 4

/*
 * selfTestCal_state_selfTestCal_patch_en
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_PATCH_EN      0x3c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_PATCH_EN_SIZE 4

/*
 * selfTestCal_state_selfTestCal_debug_stop_point
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_DEBUG_STOP_POINT      0x40
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_DEBUG_STOP_POINT_SIZE 4

/*
 * selfTestCal_state_selfTestCal_results
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_RESULTS      0x44
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_RESULTS_SIZE 4

/*
 * selfTestCal_state_selfTestCal_accel_measurent1
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_MEASURENT1      0x48
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_MEASURENT1_SIZE 12

/*
 * selfTestCal_state_selfTestCal_accel_measurent2
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_MEASURENT2      0x54
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_MEASURENT2_SIZE 12

/*
 * selfTestCal_state_selfTestCal_gyro_measurent
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_MEASURENT      0x60
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_MEASURENT_SIZE 12

/*
 * selfTestCal_state_selfTestCal_accelData
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCELDATA      0x6c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCELDATA_SIZE 12

/*
 * selfTestCal_state_selfTestCal_gyroData
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRODATA      0x78
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRODATA_SIZE 12

/*
 * selfTestCal_state_selfTestCal_sample_numb
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_SAMPLE_NUMB      0x84
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_SAMPLE_NUMB_SIZE 4

/*
 * selfTestCal_state_selfTestCal_rShiftToAverage
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_RSHIFTTOAVERAGE      0x88
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_RSHIFTTOAVERAGE_SIZE 4

/*
 * selfTestCal_state_selfTestCal_test_threshold_accel
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_TEST_THRESHOLD_ACCEL      0x8c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_TEST_THRESHOLD_ACCEL_SIZE 4

/*
 * selfTestCal_state_selfTestCal_test_threshold_gyro
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_TEST_THRESHOLD_GYRO      0x90
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_TEST_THRESHOLD_GYRO_SIZE 4

/*
 * selfTestCal_state_selfTestCal_phase
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_PHASE      0x94
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_PHASE_SIZE 4

/*
 * selfTestCal_state_selfTestCal_sample_count_accel
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_SAMPLE_COUNT_ACCEL      0x98
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_SAMPLE_COUNT_ACCEL_SIZE 4

/*
 * selfTestCal_state_selfTestCal_sample_count_gyro
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_SAMPLE_COUNT_GYRO      0x9c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_SAMPLE_COUNT_GYRO_SIZE 4

/*
 * selfTestCal_state_selfTestCal_accel_out_phase1
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_OUT_PHASE1      0xa0
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_OUT_PHASE1_SIZE 12

/*
 * selfTestCal_state_selfTestCal_accel_out_phase2
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_OUT_PHASE2      0xac
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_OUT_PHASE2_SIZE 12

/*
 * selfTestCal_state_selfTestCal_accel_accumulated
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ACCUMULATED      0xb8
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ACCUMULATED_SIZE 24

/*
 * selfTestCal_state_selfTestCal_gyro_out_phase1
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_OUT_PHASE1      0xd0
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_OUT_PHASE1_SIZE 12

/*
 * selfTestCal_state_selfTestCal_gyro_out_phase2
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_OUT_PHASE2      0xdc
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_OUT_PHASE2_SIZE 12

/*
 * selfTestCal_state_selfTestCal_gyro_accumulated
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ACCUMULATED      0xe8
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ACCUMULATED_SIZE 24

/*
 * selfTestCal_state_selfTestCal_edmp_odr_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_EDMP_ODR_REG_VAL      0x100
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_EDMP_ODR_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY      0x101
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_SIZE 6

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_mode_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_MODE_REG_VAL      0x101
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_MODE_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_odr_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_ODR_REG_VAL      0x102
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_ODR_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_x_tmid_gain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_X_TMID_GAIN_REG_VAL   \
	0x103
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_X_TMID_GAIN_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_x_tmid_off_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_X_TMID_OFF_REG_VAL    \
	0x103
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_X_TMID_OFF_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_y_tmid_gain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_Y_TMID_GAIN_REG_VAL   \
	0x104
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_Y_TMID_GAIN_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_y_tmid_off_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_Y_TMID_OFF_REG_VAL    \
	0x104
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_Y_TMID_OFF_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_z_tmid_gain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_Z_TMID_GAIN_REG_VAL   \
	0x105
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_Z_TMID_GAIN_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_z_tmid_off_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_Z_TMID_OFF_REG_VAL    \
	0x105
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_Z_TMID_OFF_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_lpf_bypass_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_LPF_BYPASS_REG_VAL    \
	0x106
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_LPF_BYPASS_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_ui_3rd_ord_sel_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_UI_3RD_ORD_SEL_REG_VAL \
	0x106
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_UI_3RD_ORD_SEL_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_datapath_settings_cpy_accel_ui_lpfbw_sel_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_UI_LPFBW_SEL_REG_VAL  \
	0x106
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_DATAPATH_SETTINGS_CPY_ACCEL_UI_LPFBW_SEL_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY      0x107
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_SIZE 7

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_mode_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_MODE_REG_VAL      0x107
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_MODE_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_odr_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_ODR_REG_VAL      0x108
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_ODR_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_ap_gyro_fs_sel_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_AP_GYRO_FS_SEL_REG_VAL      0x108
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_AP_GYRO_FS_SEL_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_x_tmid_gain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_X_TMID_GAIN_REG_VAL 0x109
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_X_TMID_GAIN_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_x_tmid_off_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_X_TMID_OFF_REG_VAL      0x109
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_X_TMID_OFF_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_y_tmid_gain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_Y_TMID_GAIN_REG_VAL 0x10a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_Y_TMID_GAIN_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_y_tmid_off_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_Y_TMID_OFF_REG_VAL      0x10a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_Y_TMID_OFF_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_z_tmid_gain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_Z_TMID_GAIN_REG_VAL 0x10b
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_Z_TMID_GAIN_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_z_tmid_off_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_Z_TMID_OFF_REG_VAL      0x10b
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_Z_TMID_OFF_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_lpf_bypass_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_LPF_BYPASS_REG_VAL      0x10c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_LPF_BYPASS_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_ui_3rd_ord_sel_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_UI_3RD_ORD_SEL_REG_VAL  \
	0x10d
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_UI_3RD_ORD_SEL_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_datapath_settings_cpy_gyro_ui_lpfbw_sel_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_UI_LPFBW_SEL_REG_VAL    \
	0x10d
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_DATAPATH_SETTINGS_CPY_GYRO_UI_LPFBW_SEL_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY      0x10e
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_SIZE 14

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_accel_testbuffer_guardband_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_TESTBUFFER_GUARDBAND_TRIM_D2D_REG_VAL \
	0x10e
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_TESTBUFFER_GUARDBAND_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_ax_dmd_inv_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AX_DMD_INV_TRIM_D2D_REG_VAL   \
	0x10f
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AX_DMD_INV_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_ax_ft_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AX_FT_TRIM_D2D_REG_VAL      0x110
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AX_FT_TRIM_D2D_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_ax_drv_phase_trim_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AX_DRV_PHASE_TRIM_D2A_REG_VAL \
	0x110
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AX_DRV_PHASE_TRIM_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_axy_2x_speed_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AXY_2X_SPEED_TRIM_D2D_REG_VAL \
	0x110
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AXY_2X_SPEED_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_ay_dmd_inv_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AY_DMD_INV_TRIM_D2D_REG_VAL   \
	0x111
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AY_DMD_INV_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_ay_drv_phase_trim_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AY_DRV_PHASE_TRIM_D2A_REG_VAL \
	0x111
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AY_DRV_PHASE_TRIM_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_az_2x_speed_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_2X_SPEED_TRIM_D2D_REG_VAL  \
	0x112
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_2X_SPEED_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_az_dmd_inv_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_DMD_INV_TRIM_D2D_REG_VAL   \
	0x112
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_DMD_INV_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_ay_ft_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AY_FT_TRIM_D2D_REG_VAL      0x113
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AY_FT_TRIM_D2D_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_az_drv_phase_trim_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_DRV_PHASE_TRIM_D2A_REG_VAL \
	0x113
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_DRV_PHASE_TRIM_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_az_mems_gain_trim_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_MEMS_GAIN_TRIM_D2A_REG_VAL \
	0x113
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_MEMS_GAIN_TRIM_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_az_ft_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_FT_TRIM_D2D_REG_VAL      0x114
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_FT_TRIM_D2D_REG_VAL_SIZE 1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_en_ana_st_vref_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_EN_ANA_ST_VREF_D2A_REG_VAL    \
	0x115
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_EN_ANA_ST_VREF_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_accel_test_cfg_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_TEST_CFG_D2A_REG_VAL    \
	0x116
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_TEST_CFG_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_accel_testbuffer_gain_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_TESTBUFFER_GAIN_D2A_REG_VAL \
	0x116
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_TESTBUFFER_GAIN_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_accel_drv_mode_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_DRV_MODE_D2A_REG_VAL    \
	0x116
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_DRV_MODE_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_accel_xyz_sel_0_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_XYZ_SEL_0_D2D_REG_VAL   \
	0x117
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_XYZ_SEL_0_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_accel_xyz_sel_1_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_XYZ_SEL_1_D2D_REG_VAL   \
	0x117
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_XYZ_SEL_1_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_accel_xyz_sel_2_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_XYZ_SEL_2_D2D_REG_VAL   \
	0x117
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_XYZ_SEL_2_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_accel_gap_test_en_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_GAP_TEST_EN_D2A_REG_VAL \
	0x118
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_ACCEL_GAP_TEST_EN_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_ax_drive_reset_off_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AX_DRIVE_RESET_OFF_D2A_REG_VAL \
	0x118
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AX_DRIVE_RESET_OFF_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_ay_drive_reset_off_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AY_DRIVE_RESET_OFF_D2A_REG_VAL \
	0x118
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AY_DRIVE_RESET_OFF_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_az_drive_reset_off_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_DRIVE_RESET_OFF_D2A_REG_VAL \
	0x118
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_AZ_DRIVE_RESET_OFF_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_en_accel_testbuffer_higain_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_EN_ACCEL_TESTBUFFER_HIGAIN_D2A_REG_VAL \
	0x119
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_EN_ACCEL_TESTBUFFER_HIGAIN_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_en_accel_testbuffer_sw_pm1_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_EN_ACCEL_TESTBUFFER_SW_PM1_D2A_REG_VAL \
	0x119
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_EN_ACCEL_TESTBUFFER_SW_PM1_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_en_accel_testbuffer_sw_pm2_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_EN_ACCEL_TESTBUFFER_SW_PM2_D2A_REG_VAL \
	0x119
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_EN_ACCEL_TESTBUFFER_SW_PM2_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_pd_accel_c2v_b_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_ACCEL_C2V_B_D2A_REG_VAL    \
	0x119
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_ACCEL_C2V_B_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_pd_accel_testbuffer_b_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_ACCEL_TESTBUFFER_B_D2A_REG_VAL \
	0x11a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_ACCEL_TESTBUFFER_B_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_pd_accel_testbuffer_hv_b_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_ACCEL_TESTBUFFER_HV_B_D2A_REG_VAL \
	0x11a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_ACCEL_TESTBUFFER_HV_B_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_pd_ax_drive_b_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_AX_DRIVE_B_D2A_REG_VAL 0x11a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_AX_DRIVE_B_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_pd_ay_drive_b_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_AY_DRIVE_B_D2A_REG_VAL 0x11a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_AY_DRIVE_B_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_pd_az_drive_b_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_AZ_DRIVE_B_D2A_REG_VAL 0x11a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_PD_AZ_DRIVE_B_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_rst_accel_arr_b_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_RST_ACCEL_ARR_B_D2D_REG_VAL   \
	0x11a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_RST_ACCEL_ARR_B_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_rst_accel_testbuffer_b_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_RST_ACCEL_TESTBUFFER_B_D2A_REG_VAL \
	0x11b
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_RST_ACCEL_TESTBUFFER_B_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_analog_settings_cpy_rst_accel_testbuffer_dmd_b_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_RST_ACCEL_TESTBUFFER_DMD_B_D2A_REG_VAL \
	0x11b
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_ANALOG_SETTINGS_CPY_RST_ACCEL_TESTBUFFER_DMD_B_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_analog_settings_cpy
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY      0x11c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_SIZE 5

/*
 * selfTestCal_state_selfTestCal_gyro_analog_settings_cpy_gz_sc2v_ft_trim_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GZ_SC2V_FT_TRIM_D2A_REG_VAL    \
	0x11c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GZ_SC2V_FT_TRIM_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_analog_settings_cpy_gx_sc2v_ft_trim_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GX_SC2V_FT_TRIM_D2A_REG_VAL    \
	0x11d
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GX_SC2V_FT_TRIM_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_analog_settings_cpy_gy_sc2v_ft_trim_d2a_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GY_SC2V_FT_TRIM_D2A_REG_VAL    \
	0x11e
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GY_SC2V_FT_TRIM_D2A_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_analog_settings_cpy_gx_dmd_phase_quadrant_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GX_DMD_PHASE_QUADRANT_TRIM_D2D_REG_VAL \
	0x11f
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GX_DMD_PHASE_QUADRANT_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_analog_settings_cpy_gy_dmd_phase_quadrant_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GY_DMD_PHASE_QUADRANT_TRIM_D2D_REG_VAL \
	0x11f
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GY_DMD_PHASE_QUADRANT_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_gyro_analog_settings_cpy_gz_dmd_phase_quadrant_trim_d2d_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GZ_DMD_PHASE_QUADRANT_TRIM_D2D_REG_VAL \
	0x120
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_ANALOG_SETTINGS_CPY_GZ_DMD_PHASE_QUADRANT_TRIM_D2D_REG_VAL_SIZE \
	1

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY      0x122
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_SIZE 24

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_x_mems_off_hfs_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_X_MEMS_OFF_HFS_REG_VAL  \
	0x122
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_X_MEMS_OFF_HFS_REG_VAL_SIZE \
	2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_y_mems_off_hfs_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Y_MEMS_OFF_HFS_REG_VAL  \
	0x124
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Y_MEMS_OFF_HFS_REG_VAL_SIZE \
	2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_z_mems_off_hfs_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Z_MEMS_OFF_HFS_REG_VAL  \
	0x126
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Z_MEMS_OFF_HFS_REG_VAL_SIZE \
	2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_x_gain_hfs_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_X_GAIN_HFS_REG_VAL      0x128
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_X_GAIN_HFS_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_y_gain_hfs_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Y_GAIN_HFS_REG_VAL      0x12a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Y_GAIN_HFS_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_z_gain_hfs_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Z_GAIN_HFS_REG_VAL      0x12c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Z_GAIN_HFS_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_x_offuser_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_X_OFFUSER_REG_VAL      0x12e
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_X_OFFUSER_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_y_offuser_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Y_OFFUSER_REG_VAL      0x130
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Y_OFFUSER_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_z_offuser_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Z_OFFUSER_REG_VAL      0x132
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Z_OFFUSER_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_x_usergain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_X_USERGAIN_REG_VAL      0x134
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_X_USERGAIN_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_y_usergain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Y_USERGAIN_REG_VAL      0x136
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Y_USERGAIN_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_accel_GOS_gain_offset_cpy_accel_z_usergain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Z_USERGAIN_REG_VAL      0x138
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_ACCEL_GOS_GAIN_OFFSET_CPY_ACCEL_Z_USERGAIN_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_gyro_user_gain_offset_cpy
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY      0x13a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_SIZE 12

/*
 * selfTestCal_state_selfTestCal_gyro_user_gain_offset_cpy_gyro_x_offuser_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_X_OFFUSER_REG_VAL      0x13a
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_X_OFFUSER_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_gyro_user_gain_offset_cpy_gyro_y_offuser_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_Y_OFFUSER_REG_VAL      0x13c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_Y_OFFUSER_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_gyro_user_gain_offset_cpy_gyro_z_offuser_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_Z_OFFUSER_REG_VAL      0x13e
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_Z_OFFUSER_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_gyro_user_gain_offset_cpy_gyro_x_usergain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_X_USERGAIN_REG_VAL      0x140
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_X_USERGAIN_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_gyro_user_gain_offset_cpy_gyro_y_usergain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_Y_USERGAIN_REG_VAL      0x142
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_Y_USERGAIN_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_gyro_user_gain_offset_cpy_gyro_z_usergain_reg_val
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_Z_USERGAIN_REG_VAL      0x144
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_GYRO_USER_GAIN_OFFSET_CPY_GYRO_Z_USERGAIN_REG_VAL_SIZE 2

/*
 * selfTestCal_state_selfTestCal_input_ssom
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INPUT_SSOM      0x148
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INPUT_SSOM_SIZE 16

/*
 * selfTestCal_state_selfTestCal_input_ssom_sensor_id
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INPUT_SSOM_SENSOR_ID      0x148
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INPUT_SSOM_SENSOR_ID_SIZE 4

/*
 * selfTestCal_state_selfTestCal_input_ssom_data
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INPUT_SSOM_DATA      0x14c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INPUT_SSOM_DATA_SIZE 12

/*
 * selfTestCal_state_selfTestCal_state_ssom
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM      0x158
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_SIZE 36

/*
 * selfTestCal_state_selfTestCal_state_ssom_ssom_trigger_th
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_SSOM_TRIGGER_TH      0x158
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_SSOM_TRIGGER_TH_SIZE 8

/*
 * selfTestCal_state_selfTestCal_state_ssom_minState
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_MINSTATE      0x160
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_MINSTATE_SIZE 12

/*
 * selfTestCal_state_selfTestCal_state_ssom_maxState
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_MAXSTATE      0x16c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_MAXSTATE_SIZE 12

/*
 * selfTestCal_state_selfTestCal_state_ssom_motion_status
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_MOTION_STATUS      0x178
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_STATE_SSOM_MOTION_STATUS_SIZE 4

/*
 * selfTestCal_state_selfTestCal_interrupt_count
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INTERRUPT_COUNT      0x17c
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INTERRUPT_COUNT_SIZE 4

/*
 * selfTestCal_state_selfTestCal_edmp2host_int_reg1
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_EDMP2HOST_INT_REG1      0x180
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_EDMP2HOST_INT_REG1_SIZE 1

/*
 * selfTestCal_state_selfTestCal_int_status_register
 *
 */
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INT_STATUS_REGISTER      0x181
#define EDMP_SELFTESTCAL_STATE_SELFTESTCAL_INT_STATUS_REGISTER_SIZE 1

#ifdef __cplusplus
}
#endif

#endif /* __INV_IMU_EDMP_MEMMAP_INTERNAL_H__ */
