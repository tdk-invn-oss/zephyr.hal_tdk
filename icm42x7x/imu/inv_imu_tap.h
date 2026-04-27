/*
 *
 * Copyright (c) [2025] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/** @defgroup Tap Tap
 *  @brief High-level functions to drive Tap feature
 *  @{
 */

/** @file inv_imu_tap.h */

#ifndef _INV_IMU_TAP_H_
#define _INV_IMU_TAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"

/* TAP_EN bit location */
#define APEX_CONFIG1_TAP_EN_POS  APEX_CONFIG1_PED_ENABLE_POS
#define APEX_CONFIG1_TAP_EN_MASK (0x1 << APEX_CONFIG1_TAP_EN_POS)

/* Single Tap replaces Low-G */
#define INT_SOURCE6_SINGLE_TAP_INT1_EN_POS  INT_SOURCE6_LOWG_INT1_EN_POS
#define INT_SOURCE6_SINGLE_TAP_INT1_EN_MASK (1 << INT_SOURCE6_SINGLE_TAP_INT1_EN_POS)
#define INT_SOURCE7_SINGLE_TAP_INT2_EN_POS  INT_SOURCE7_LOWG_INT2_EN_POS
#define INT_SOURCE7_SINGLE_TAP_INT2_EN_MASK (1 << INT_SOURCE7_SINGLE_TAP_INT2_EN_POS)
/* Double Tap replaces Free-fall */
#define INT_SOURCE6_DOUBLE_TAP_INT1_EN_POS  INT_SOURCE6_FF_INT1_EN_POS
#define INT_SOURCE6_DOUBLE_TAP_INT1_EN_MASK (1 << INT_SOURCE6_DOUBLE_TAP_INT1_EN_POS)
#define INT_SOURCE7_DOUBLE_TAP_INT2_EN_POS  INT_SOURCE7_FF_INT2_EN_POS
#define INT_SOURCE7_DOUBLE_TAP_INT2_EN_MASK (1 << INT_SOURCE7_DOUBLE_TAP_INT2_EN_POS)
/* Triple Tap replaces Tilt */
#define INT_SOURCE6_TRIPLE_TAP_INT1_EN_POS  INT_SOURCE6_TILT_DET_INT1_EN_POS
#define INT_SOURCE6_TRIPLE_TAP_INT1_EN_MASK (1 << INT_SOURCE6_TRIPLE_TAP_INT1_EN_POS)
#define INT_SOURCE7_TRIPLE_TAP_INT2_EN_POS  INT_SOURCE7_TILT_DET_INT2_EN_POS
#define INT_SOURCE7_TRIPLE_TAP_INT2_EN_MASK (1 << INT_SOURCE7_TRIPLE_TAP_INT2_EN_POS)

#define INT_STATUS3_SINGLE_TAP_INT_POS  INT_STATUS3_LOWG_DET_INT_POS
#define INT_STATUS3_SINGLE_TAP_INT_MASK (1 << INT_STATUS3_SINGLE_TAP_INT_POS)
#define INT_STATUS3_DOUBLE_TAP_INT_POS  INT_STATUS3_FF_DET_INT_POS
#define INT_STATUS3_DOUBLE_TAP_INT_MASK (1 << INT_STATUS3_DOUBLE_TAP_INT_POS)
#define INT_STATUS3_TRIPLE_TAP_INT_POS  INT_STATUS3_TILT_DET_INT_POS
#define INT_STATUS3_TRIPLE_TAP_INT_MASK (1 << INT_STATUS3_TRIPLE_TAP_INT_POS)

/** @brief Tap axis definition */
typedef enum {
	INV_IMU_DMP_TAP_AXIS_Z = 0x02,
	INV_IMU_DMP_TAP_AXIS_Y = 0x01,
	INV_IMU_DMP_TAP_AXIS_X = 0x00,
} inv_imu_tap_axis_t;

/** @brief Tap direction definition */
typedef enum {
	INV_IMU_DMP_TAP_DIR_POSITIVE = 0x01,
	INV_IMU_DMP_TAP_DIR_NEGATIVE = 0x00,
} inv_imu_tap_dir_t;

/** APEX TAP outputs */
typedef struct {
	uint8_t  tap_num; /**< 0: no tap, 1: single tap, 2: double tap, 3: triple tap */
	uint8_t  tap_axis; /**< 0: ax, 1: ay, 2: az */
	uint8_t  tap_direction; /**< 0: positive, 1: negative */
	uint16_t double_tap_timing; /**< the sample count of second pulse in detection window */
	uint16_t triple_tap_timing; /**< the sample count of third pulse in detection window */
} inv_imu_tap_event_t;

/** Interrupt definition */
typedef struct {
	inv_imu_interrupt_value INV_SINGLE_TAP_DET;
	inv_imu_interrupt_value INV_DOUBLE_TAP_DET;
	inv_imu_interrupt_value INV_TRIPLE_TAP_DET;
} inv_imu_tap_interrupt_parameter_t;

/** @brief Tap inputs parameters definition
 */
typedef struct inv_imu_tap_parameters {
	/**
	 * State of power-save mode of dmp which skips computation in absence of movements.
	 * Unit is enum value [0 : 1]
	 * Default value is: 0
	 */
	APEX_CONFIG0_DMP_POWER_SAVE_t power_save;
	/**
	 * Duration for which no movements must be detected before dmp stops processing
	 * and enters power-save mode. Only meaningful if `power_save` is enabled.
	 * Unit is enum value [0 : 15] which will be converted to a number of samples
	 * Default value is: 0
	 */
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_t power_save_time;
	/**
	 * Time window since the jerk maximum value, used to observe a single-tap, double-tap,
	 * triple-tap or rejection of the tap event.
	 * Unit is sample count, ODR dependent
	 * Default value is: 198 at 400Hz, corresponding to 0.495s
	 */
	uint16_t tap_tmax;
	/**
	 * Single tap window, sub-window within the `tap_tmax` to detect tap event.
	 * Unit is sample count, ODR dependent
	 * Default value is: 66 at 400Hz, corresponding to 0.1650s
	 */
	uint8_t tap_tmin;
	/**
	 * Maximal number of tap quantity detected to be valid.
	 * Unit is enum value {1, 2, 3} corresponding to {single, double, triple}
	 * Default value is: 3 (triple tap)
	 */
	uint8_t tap_max;
	/**
	 * Minimal number of tap quantity detected to be valid.
	 * Unit is enum [1 - 2 - 3] corresponding to [single, double, triple]
	 * Default value is: 1 (single tap)
	 */
	uint8_t tap_min;
	/**
	 * Discrete value representing the percentage of pulse amplitude
	 * that define the smudge threshold value of tap rejection.
	 * Unit is enum {1, 2, 3, 4} corresponding to {12.5%, 25%, 37.5%, 50%}
	 * Default value is: 2 (corresponding to 25%)
	 */
	uint8_t tap_max_peak_tol;
	/**
	 * Energy measurement window size used to determine the tap axis
	 * associated with the 1st tap.
	 * Unit is sample count, ODR dependent
	 * Default value is: 8
	 * Use value limited to {1, 2, 4, 8}
	 */
	uint8_t tap_tavg;
	/**
	 * The minimal value of jerk to be considered as a tap candidate.
	 * Unit is LSB with 1 LSB = 1g / 2^12 (of the jerk value)
	 * Default value is: 4608 corresponding to 1.125 g
	 */
	uint16_t tap_min_jerk_threshold;
	/**
	 * Maximum time spent over the threshold defined through `tap_max_peak_tol`
	 * before rejecting the tap candidate. At the end of `tap_tmin` window,
	 * if the internal counter of samples over the smudge threshold is below
	 * the tap_smudge_reject_th, single-tap is detected.
	 * Unit is sample number, ODR dependent 
	 * Default value is: 34 at 400Hz, corresponding to 0.085 s
	 */
	uint8_t tap_smudge_rejection;
} inv_imu_tap_parameters_t;

/** @brief Load DMP tap image in DMP SRAM and set start address .
 *  @param[inout] s  Pointer to device.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_tap_load_dmp(inv_imu_device_t *s);

/** @brief Configure which tap interrupt source can trigger INT1.
 *  @param[in] s   Pointer to device.
 *  @param[in] it  Structure with the corresponding state to manage INT1.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_tap_set_config_int1(inv_imu_device_t *s, const inv_imu_tap_interrupt_parameter_t *it);

/** @brief Retrieve tap interrupts INT1 configuration.
 *  @param[in] s   Pointer to device.
 *  @param[out] it Structure with the corresponding state to manage INT1.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_tap_get_config_int1(inv_imu_device_t *s, inv_imu_tap_interrupt_parameter_t *it);

/** @brief Configure which tap interrupt source can trigger INT2.
 *  @param[in] s   Pointer to device.
 *  @param[in] it  Structure with the corresponding state to manage INT2.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_tap_set_config_int2(inv_imu_device_t *s, const inv_imu_tap_interrupt_parameter_t *it);

/** @brief Retrieve tap interrupts INT2 configuration.
 *  @param[in] s   Pointer to device.
 *  @param[out] it Structure with the corresponding state to manage INT2.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_tap_get_config_int2(inv_imu_device_t *s, inv_imu_tap_interrupt_parameter_t *it);

/** @brief Enable tap algorithm to run in DMP.
 *  @param[in] s   Pointer to device.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_tap_enable(inv_imu_device_t *s);

/** @brief Disable tap algorithm to run in DMP.
 *  @param[in] s   Pointer to device.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_tap_disable(inv_imu_device_t *s);

/** @brief  Retrieve TAP outputs and format them
 *  @param[in] s         Pointer to device.
 *  @param[out] tap_out  TAP event, axis and direction.
 *  @return              0 on success, negative value on error.
 */
int inv_imu_tap_get_data(inv_imu_device_t *s, inv_imu_tap_event_t *tap_out);

/** @brief Fill the Tap parameters structure with the default values.
 *  @param[in]  s      Pointer to device.
 *  @param[out] params Default input parameters. See @sa inv_imu_tap_parameters_t 
 *  @return            0 on success, negative value on error.
 */
int inv_imu_tap_init_parameters_struct(inv_imu_device_t *s, inv_imu_tap_parameters_t *params);

/** @brief Configure DMP parameters for Tap algorithms.
 *  @param[in]  s      Pointer to device.
 *  @param[in] params The requested input parameters. See @sa inv_imu_faceorientation_parameters_t
 *  @warning Tap must be disabled to reconfigure its parameters.
 *  @warning After calling inv_imu_tap_configure_parameters, it is mandatory to call 
 *           inv_imu_tap_load_dmp() to take the new Tap parameters into account.
 *  @return 0 on success, negative value on error.
 */
int inv_imu_tap_configure_parameters(inv_imu_device_t *s, const inv_imu_tap_parameters_t *params);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_TAP_H_ */

/** @} */
