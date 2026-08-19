/*
 *
 * Copyright (c) [2024] by InvenSense, Inc.
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

#ifndef __INV_IMU_EDMP_APEX_LOWG_MEMMAP_H__
#define __INV_IMU_EDMP_APEX_LOWG_MEMMAP_H__

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
 * odr_lowg
 * *       Specify ODR of feature Free-Fall
 *       Unit: Hertz
 *       Default value: 400
 *       Format: int16_t
 *
 */
#define EDMP_ODR_LOWG      0x2a
#define EDMP_ODR_LOWG_SIZE 2

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

#ifdef __cplusplus
}
#endif

#endif /* __INV_IMU_EDMP_APEX_LOWG_MEMMAP_H__ */
