/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#ifndef __INV_IMU_EDMP_SELFTEST_MEMMAP_H__
#define __INV_IMU_EDMP_SELFTEST_MEMMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

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

#endif /* __INV_IMU_EDMP_SELFTEST_MEMMAP_H__ */
