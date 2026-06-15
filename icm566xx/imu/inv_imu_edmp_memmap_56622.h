/*
 *
 * Copyright (c) [2021] by InvenSense, Inc.
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

#ifndef __INV_IMU_EDMP_MEMMAP_H__
#define __INV_IMU_EDMP_MEMMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * power_save_time
 * *       The time (in sample number) after which eDMP goes in power save mode.
 *       Unit: time in sample number
 *       Range: [0 - 4294967295]
 *       Default value: 6400
 *       Format: uint32_t
 *
 */
#define EDMP_POWER_SAVE_TIME      0xa8
#define EDMP_POWER_SAVE_TIME_SIZE 4

/*
 * odr_ff
 * *       Specify ODR of feature Free-Fall
 *       Unit: Hertz
 *       Default value: 400
 *       Format: int16_t
 *
 */
#define EDMP_ODR_FF      0x2a
#define EDMP_ODR_FF_SIZE 2

/*
 * lowg_peak_th
 * *       Sum of absolute accelerometer axis falls lowg_peak_th will potentially trigger the start
 * of LowG state Unit: in LSB, with 1 LBS = 1g / 2^12 Default value: 2048 (corresponding to 0.5g)
 *       Recommended range value: [128 - 4096]
 *       Format: int16_t
 *
 */
#define EDMP_LOWG_PEAK_TH      0xda
#define EDMP_LOWG_PEAK_TH_SIZE 2

/*
 * lowg_peak_th_hyst
 * *       Hysteresis added to the threshold to confirm the triggering of the LowG state
 *       Unit: in LSB, with 1 LBS = 1g / 2^12
 *       Default value: 128 (corresponding to 31.2mg)
 *       Recommended range value: [128 - 1024]
 *       Format: int16_t
 *
 */
#define EDMP_LOWG_PEAK_TH_HYST      0xdc
#define EDMP_LOWG_PEAK_TH_HYST_SIZE 2

/*
 * lowg_time_th
 * *       The number of samples device should stay below (lowg_peak_th + lowg_peak_th_hyst) before
 * LowG state is triggered Unit: in sample number - ODR dependent Default value: 13 (corresponding
 * to 32ms at 400Hz) Recommended range value: [1 - 300] Format: int16_t
 *
 */
#define EDMP_LOWG_TIME_TH      0xde
#define EDMP_LOWG_TIME_TH_SIZE 2

/*
 * stc_results
 * *       bit0:		accel_x_self_test_result	-- 0/1: passed/failed
 *       bit1:		accel_y_self_test_result	-- 0/1: passed/failed
 *       bit2:		accel_z_self_test_result	-- 0/1: passed/failed
 *       bit3:		gyro_x_self_test_result		-- 0/1: passed/failed
 *       bit4:		gyro_y_self_test_result		-- 0/1: passed/failed
 *       bit5:		gyro_z_self_test_result		-- 0/1: passed/failed
 *       bit6~7:	self_test_status            -- 0/1/2: done/in-progress/error
 *       bit8~31:	unused
 *
 */
#define EDMP_STC_RESULTS      0x44
#define EDMP_STC_RESULTS_SIZE 4

/*
 * stc_gain_gx
 * *       Gyro measurements for X axis in s32.16 for gyro self-test Gain
 *
 */
#define EDMP_STC_GAIN_GX      0x60
#define EDMP_STC_GAIN_GX_SIZE 4

/*
 * stc_gain_gy
 * *       Gyro measurements for Y axis in s32.16 for gyro self-test Gain
 *
 */
#define EDMP_STC_GAIN_GY      0x64
#define EDMP_STC_GAIN_GY_SIZE 4

/*
 * stc_gain_gz
 * *       Gyro measurements for Z axis in s32.16 for gyro self-test Gain
 *
 */
#define EDMP_STC_GAIN_GZ      0x68
#define EDMP_STC_GAIN_GZ_SIZE 4

/*
 * stc_configParams
 * *       bit0:		self_testCal_init_en        -- 0/1: Disabled/Enabled
 *       bit1:		accel_self_test_en          -- 0/1: Disabled/Enabled
 *       bit2:		gyro_self_test_en           -- 0/1: Disabled/Enabled
 *       bit7~9:   self_test_average_time    -- 0/1/2/3/4/5: 10/20/40/80/160/320 ms ->
 * 8/16/32/64/128/256 samples @ 800Hz bit10~12:	accel_self_test_threshold	-- 0/1/2/3/4/5/6/7:
 * 5/10/15/20/25/30/40/50 bit13~15:	gyro_self_test_threshold	-- 0/1/2/3/4/5/6/7:
 * 5/10/15/20/25/30/40/50 bit16~31:	unused
 *
 */
#define EDMP_STC_CONFIGPARAMS      0x38
#define EDMP_STC_CONFIGPARAMS_SIZE 4

/*
 * stc_patch_en
 * *       bit0:		patch_en_self_test_accel_phase1   -- 0/1: disabled/enabled
 *       bit1:		patch_en_self_test_accel_phase2   -- 0/1: disabled/enabled
 *       bit2:		patch_en_self_test_gyro1_phase1   -- 0/1: disabled/enabled
 *       bit3:		patch_en_self_test_gyro1_phase2   -- 0/1: disabled/enabled
 *       bit12~31:	unused
 *
 */
#define EDMP_STC_PATCH_EN      0x3c
#define EDMP_STC_PATCH_EN_SIZE 4

/*
 * stc_debug_en
 * *       Debug capability of self-test feature. Must be set to 0 at anytime when self-test is
 * requested.
 *
 */
#define EDMP_STC_DEBUG_EN      0x40
#define EDMP_STC_DEBUG_EN_SIZE 4

/*
 * ssom_trigger_th_accel
 * *       Threshold for delta accel data to considered a changing status
 *
 */
#define EDMP_SSOM_TRIGGER_TH_ACCEL      0x158
#define EDMP_SSOM_TRIGGER_TH_ACCEL_SIZE 4

/*
 * ssom_trigger_th_gyro
 * *       Threshold for delta gyro data to considered a changing status
 *
 */
#define EDMP_SSOM_TRIGGER_TH_GYRO      0x15c
#define EDMP_SSOM_TRIGGER_TH_GYRO_SIZE 4

#ifdef __cplusplus
}
#endif

#endif /* __INV_IMU_EDMP_MEMMAP_H__ */
