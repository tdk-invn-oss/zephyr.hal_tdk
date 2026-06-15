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

#ifndef __INV_IMU_EDMP_DEFS_H__
#define __INV_IMU_EDMP_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define EDMP_ROM_ONLY_SP_START_ADDR         0x600
#define EDMP_ROM_RAM_SP_START_ADDR          0x900
#define EDMP_RAM_PRGM_BASE                  0x5AC
#define RAM_INSTR_PRGM_BASE                 0x1B0
#define RAM_INSTR_SP_START_ADDR             0x400
#define RAM_INSTR_FIFO_SIZE                 0x100
#define DEFAULT_ODR_HZ_BUMP                 0x320
#define DEFAULT_ODR_HZ_EDMP                 0x190
#define DEFAULT_ODR_HZ_FF                   0x190
#define DEFAULT_ODR_HZ_TAP                  0x190
#define DEFAULT_ODR_HZ_3AXIS                0x32
#define DEFAULT_ODR_HZ_TILT                 0x32
#define DEFAULT_ODR_HZ_PEDO                 0x32
#define DEFAULT_ODR_HZ_SMD                  0x32
#define DEFAULT_ODR_HZ_B2S                  0x32
#define DEFAULT_ODR_HZ_SHAKE                0x32
#define DEFAULT_ODR_HZ_NOMOTION             0x32
#define DEFAULT_ODR_HZ_FLAT                 0x32
#define DEFAULT_ODR_HZ_R2W                  0x19
#define EDMP_HOST_INT_TAP_DET_POS           0x0
#define EDMP_HOST_INT_TAP_DET_MSK           0x1
#define EDMP_HOST_INT_HIGHG_DET_POS         0x1
#define EDMP_HOST_INT_HIGHG_DET_MSK         0x2
#define EDMP_HOST_INT_LOWG_DET_POS          0x2
#define EDMP_HOST_INT_LOWG_DET_MSK          0x4
#define EDMP_HOST_INT_TILT_DET_POS          0x3
#define EDMP_HOST_INT_TILT_DET_MSK          0x8
#define EDMP_HOST_INT_STEP_CNT_OVFL_DET_POS 0x4
#define EDMP_HOST_INT_STEP_CNT_OVFL_DET_MSK 0x10
#define EDMP_HOST_INT_STEP_DET_POS          0x5
#define EDMP_HOST_INT_STEP_DET_MSK          0x20
#define EDMP_HOST_INT_FF_DET_POS            0x6
#define EDMP_HOST_INT_FF_DET_MSK            0x40
#define EDMP_HOST_INT_R2W_WAKE_DET_POS      0x7
#define EDMP_HOST_INT_R2W_WAKE_DET_MSK      0x80
#define EDMP_HOST_INT_R2W_SLEEP_DET_POS     0x0
#define EDMP_HOST_INT_R2W_SLEEP_DET_MSK     0x1
#define EDMP_HOST_INT_SMD_DET_POS           0x1
#define EDMP_HOST_INT_SMD_DET_MSK           0x2
#define EDMP_HOST_INT_SELF_TEST_DONE_POS    0x2
#define EDMP_HOST_INT_SELF_TEST_DONE_MSK    0x4
#define EDMP_HOST_INT_SA_DONE_POS           0x4
#define EDMP_HOST_INT_SA_DONE_MSK           0x10
#define EDMP_HOST_INT_BUMP_DET_POS          0x4
#define EDMP_HOST_INT_BUMP_DET_MSK          0x10
#define EDMP_HOST_INT_SHAKE_DET_POS         0x5
#define EDMP_HOST_INT_SHAKE_DET_MSK         0x20
#define EDMP_HOST_INT_B2S_DET_POS           0x6
#define EDMP_HOST_INT_B2S_DET_MSK           0x40
#define EDMP_HOST_INT_REVB2S_DET_POS        0x7
#define EDMP_HOST_INT_REVB2S_DET_MSK        0x80
#define EDMP_HOST_INT_NOMOTION_DET_POS      0x0
#define EDMP_HOST_INT_NOMOTION_DET_MSK      0x1
#define EDMP_HOST_INT_MOTION_DET_POS        0x1
#define EDMP_HOST_INT_MOTION_DET_MSK        0x2
#define EDMP_HOST_INT_NOFLAT_DET_POS        0x2
#define EDMP_HOST_INT_NOFLAT_DET_MSK        0x4
#define EDMP_HOST_INT_FLAT_DET_POS          0x3
#define EDMP_HOST_INT_FLAT_DET_MSK          0x8
#define PATCH_PRGM_BASE_SELF_TEST_CAL       0x200

#ifdef __cplusplus
}
#endif

#endif /* __INV_IMU_EDMP_DEFS_H__ */
