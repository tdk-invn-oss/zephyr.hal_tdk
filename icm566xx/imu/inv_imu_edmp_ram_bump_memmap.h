/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#ifndef __INV_IMU_EDMP_RAM_BUMP_MEMMAP_H__
#define __INV_IMU_EDMP_RAM_BUMP_MEMMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * bump_decim_rate
 *
 * Decimation rate for the bump feature
 * Format: int8_t
 *
 */
#define EDMP_BUMP_DECIM_RATE      0x7b8
#define EDMP_BUMP_DECIM_RATE_SIZE 1

/*
 * bump_decim_count
 *
 * Decimation current counter for the bump feature
 * Format: int8_t
 *
 */
#define EDMP_BUMP_DECIM_COUNT      0x7b9
#define EDMP_BUMP_DECIM_COUNT_SIZE 1

/*
 * bump_out_interrupt
 *
 * Bump feature interrupt
 * Format: uint8_t
 *
 */
#define EDMP_BUMP_OUT_INTERRUPT      0x7ba
#define EDMP_BUMP_OUT_INTERRUPT_SIZE 1

/*
 * bump_config_bump_th_ratio
 *
 * Threshold ratio above which a bump candidate is considered
 * Unit in LSB with 1 LSB = 1 / 2^8
 * Default value: 400, corresponding to 1.5625
 * Format: int16_t
 *
 */
#define EDMP_BUMP_CONFIG_BUMP_TH_RATIO      0x7f2
#define EDMP_BUMP_CONFIG_BUMP_TH_RATIO_SIZE 2

/*
 * bump_config_change_th_ratio
 *
 * Threshold ratio between the noise estimation w.r.t to the bump jerk, above which a change is
 * detected Unit in LSB with 1 LSB = 1 / 2^8 Default value: 380, corresponding to 1.484 Format:
 * int16_t
 *
 */
#define EDMP_BUMP_CONFIG_CHANGE_TH_RATIO      0x7ee
#define EDMP_BUMP_CONFIG_CHANGE_TH_RATIO_SIZE 2

/*
 * bump_config_conf_time
 *
 * Confusion time waited before starting to validate a new bump candidate
 * Unit in sample number, dependent on the configured ODR * Default value: 10, corresponding to 12.5
 * ms for 800 Hz Format: int16_t
 *
 */
#define EDMP_BUMP_CONFIG_CONF_TIME      0x7f6
#define EDMP_BUMP_CONFIG_CONF_TIME_SIZE 2

/*
 * bump_config_debounce_time
 *
 * Wait time after a bump to allow re-estimate of the vibration amplitude
 * Unit is sample number, dependent on the configured ODR
 * Default value: 800, corresponding to 1s for 800 Hz
 * Format: int16_t
 *
 */
#define EDMP_BUMP_CONFIG_DEBOUNCE_TIME      0x7fa
#define EDMP_BUMP_CONFIG_DEBOUNCE_TIME_SIZE 2

/*
 * bump_config_min_bump_nb
 *
 * Minimal bump duration (nb of bump above bump_config_bump_th_ratio_Q8) to validate the new bump
 * candidate Unit is sample number, dependent on the configured ODR Default value: 10, corresponding
 * to 12.5 ms for 800 Hz Format: uint16_t
 *
 */
#define EDMP_BUMP_CONFIG_MIN_BUMP_NB      0x7fc
#define EDMP_BUMP_CONFIG_MIN_BUMP_NB_SIZE 2

/*
 * bump_config_bump_time
 *
 * Duration of the validation phase for a new bump candidate
 * Unit is sample number, dependent on the configured ODR
 * Default value: 150, corresponding to 187.5 ms for 800 Hz
 * Format: int16_t
 *
 */
#define EDMP_BUMP_CONFIG_BUMP_TIME      0x7f8
#define EDMP_BUMP_CONFIG_BUMP_TIME_SIZE 2

/*
 * bump_config_change_th_offset
 *
 * Threshold offset between noise estimation w.r.t to the bump jerk above which a change is detected
 * Unit in LSB with 1 LBS = 1g / 2^12
 * Default value: 100, corresponding to 0.0244 g
 * Format: int16_t
 *
 */
#define EDMP_BUMP_CONFIG_CHANGE_TH_OFFSET      0x7f0
#define EDMP_BUMP_CONFIG_CHANGE_TH_OFFSET_SIZE 2

/*
 * bump_config_bump_th_offset
 *
 * Threshold offset above which a bump candidate is considered
 * Unit in LSB with 1 LBS = 1g / 2^12
 * Default value: 450, corresponding to 0.1099 g
 * Format: int16_t
 *
 */
#define EDMP_BUMP_CONFIG_BUMP_TH_OFFSET      0x7f4
#define EDMP_BUMP_CONFIG_BUMP_TH_OFFSET_SIZE 2

/*
 * bump_config_odr
 *
 * ODR at which the bump feature must run, both bump_decim_rate and bump_decim_count are dependent
 * on this value. Format: int16_t
 *
 */
#define EDMP_BUMP_CONFIG_ODR      0x7ec
#define EDMP_BUMP_CONFIG_ODR_SIZE 2

#ifdef __cplusplus
}
#endif

#endif /* __INV_IMU_EDMP_RAM_BUMP_MEMMAP_H__ */
