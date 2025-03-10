/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#ifndef __INV_IMU_EDMP_DEFS_H__
#define __INV_IMU_EDMP_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define EDMP_RAM_BASE	0x0
#define EDMP_ROM_BASE	0x4000
#define EDMP_ROM_DATA_SIZE	0x4C0
#define APEX_FEATURE_STACK_END	0x500
#define EDMP_RAM_FEATURE_PRGM_RAM_BASE	0x500
#define EDMP_HOST_INT_TAP_DET_POS	0x0
#define EDMP_HOST_INT_HIGHG_DET_POS	0x1
#define EDMP_HOST_INT_LOWG_DET_POS	0x2
#define EDMP_HOST_INT_TILT_DET_POS	0x3
#define EDMP_HOST_INT_STEP_CNT_OVFL_DET_POS	0x4
#define EDMP_HOST_INT_STEP_DET_POS	0x5
#define EDMP_HOST_INT_FF_DET_POS	0x6
#define EDMP_HOST_INT_R2W_WAKE_DET_POS	0x7
#define EDMP_HOST_INT_B2S_DET_POS	0x7
#define EDMP_HOST_INT_REVB2S_DET_POS	0x0
#define EDMP_HOST_INT_R2W_SLEEP_DET_POS	0x0
#define EDMP_HOST_INT_SMD_DET_POS	0x1
#define EDMP_HOST_INT_SELF_TEST_DONE_POS	0x2
#define EDMP_HOST_INT_SA_DONE_POS	0x4
#define EDMP_HOST_INT_BASIC_SMD_DET_POS	0x5
#define RAM_PATCHES_PRGM_RAM_BASE	0x500


#ifdef __cplusplus
}
#endif

#endif // __INV_IMU_EDMP_DEFS_H__
