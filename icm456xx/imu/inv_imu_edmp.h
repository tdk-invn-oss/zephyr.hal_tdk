/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
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

/** @defgroup EDMP EDMP
 *  @brief API to drive eDMP features.
 *  @{
 */

/** @file inv_imu_edmp.h */

#ifndef _INV_IMU_EDMP_H_
#define _INV_IMU_EDMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_edmp_memmap.h"

#include "sif_feature_extract_ir.h"
#include "sif_classifier_ir.h"

#include <stdint.h>
#include <string.h>

/** @brief Writes in EDMP SRAM
 *  @param[in] s     Pointer to device.
 *  @param[in] name  Name of the parameter.
 *  @param[in] val   Value to be written.
 *  @return          0 on success, negative value on error.
 */
#define INV_IMU_WRITE_EDMP_SRAM(s, name, val)                                                      \
	inv_imu_write_sram(s, (uint32_t)name, name##_SIZE, val)

/** @brief Reads in EDMP SRAM
 *  @param[in] s     Pointer to device.
 *  @param[in] name  Name of the parameter.
 *  @param[in] val   Value to be read.
 *  @return          0 on success, negative value on error.
 */
#define INV_IMU_READ_EDMP_SRAM(s, name, val) inv_imu_read_sram(s, (uint32_t)name, name##_SIZE, val)

/** @brief EDMP input interrupt lines definition */
typedef enum { INV_IMU_EDMP_INT0 = 0, INV_IMU_EDMP_INT1, INV_IMU_EDMP_INT2 } inv_imu_edmp_int_t;

/** @brief APEX interrupts definition */
typedef struct {
	uint8_t INV_TAP;
	uint8_t INV_HIGHG;
	uint8_t INV_LOWG;
	uint8_t INV_SIF;
	uint8_t INV_AID_HUMAN;
	uint8_t INV_AID_DEVICE;
	uint8_t INV_FF;
	uint8_t INV_B2S;

	uint8_t INV_B2S_REV;
	uint8_t INV_VVD;
	uint8_t INV_SELF_TEST;
	uint8_t INV_SEC_AUTH;
} inv_imu_edmp_int_state_t;

/** Registers to retrieve interrupts status for APEX. */
typedef struct {
	int_apex_status0_t int_apex_status0;
	int_apex_status1_t int_apex_status1;
} int_apex_statusx_t;

/** Registers to configure interrupts for APEX. */
typedef struct {
	int_apex_config0_t int_apex_config0;
	int_apex_config1_t int_apex_config1;
} int_apex_configx_t;

/** Registers to enable APEX features. */
typedef struct {
	edmp_apex_en0_t edmp_apex_en0;
	edmp_apex_en1_t edmp_apex_en1;
} edmp_apex_enx_t;

struct aid_parameters_s {
	uint32_t aid_update_win;
	uint32_t aid_alert_timer;
	uint8_t  aid_output_enable;
	uint8_t  aid_disable_multiple_interrupt;
};

/** @brief IMU APEX inputs parameters definition
 *  @note Refer to the datasheet for details on how to configure these parameters.
 */
typedef struct {
	/* AID (motion/noMotion) */
	struct aid_parameters_s aid_human;
	struct aid_parameters_s aid_device;

	/* Freefall */
	uint16_t lowg_peak_th;
	uint16_t lowg_peak_th_hyst;
	uint16_t lowg_time_th;
	uint16_t highg_peak_th;
	uint16_t highg_peak_th_hyst;
	uint16_t highg_time_th;
	uint32_t ff_min_duration;
	uint32_t ff_max_duration;
	uint32_t ff_debounce_duration;

	/* Tap */
	uint16_t tap_min_jerk;
	uint16_t tap_tmax;
	uint8_t  tap_tmin;
	uint8_t  tap_max_peak_tol;
	uint8_t  tap_smudge_reject_th;
	uint8_t  tap_tavg;
	uint8_t  tap_odr;
	uint8_t  tap_max;
	uint8_t  tap_min;

	/* B2S */
	uint8_t  b2s_mounting_matrix;
	int32_t  b2s_dev_norm_max;
	int32_t  b2s_sin_limit;
	int32_t  b2s_fast_limit;
	int32_t  b2s_static_limit;
	int32_t  b2s_thr_cos_ang;
	int32_t  b2s_rev_x_limit;
	uint32_t b2s_sin_flat_angle;
	uint32_t b2s_timer_flat_reject;
	uint16_t b2s_fast_motion_age_limit;
	uint16_t b2s_fast_motion_time_limit;
	uint16_t b2s_age_limit;
	uint16_t b2s_rev_latency_th;

	/* Power save */
	uint32_t power_save_time;
	uint8_t  power_save_en;

} inv_imu_edmp_apex_parameters_t;

/** @brief Tap number definition */
typedef enum {
	INV_IMU_EDMP_TAP_TRIPLE = 0x03,
	INV_IMU_EDMP_TAP_DOUBLE = 0x02,
	INV_IMU_EDMP_TAP_SINGLE = 0x01,
} inv_imu_edmp_tap_num_t;

/** @brief Tap axis definition */
typedef enum {
	INV_IMU_EDMP_TAP_AXIS_Z = 0x02,
	INV_IMU_EDMP_TAP_AXIS_Y = 0x01,
	INV_IMU_EDMP_TAP_AXIS_X = 0x00,
} inv_imu_edmp_tap_axis_t;

/** @brief Tap direction definition */
typedef enum {
	INV_IMU_EDMP_TAP_DIR_POSITIVE = 0x01,
	INV_IMU_EDMP_TAP_DIR_NEGATIVE = 0x00,
} inv_imu_edmp_tap_dir_t;

/** @brief Tap outputs */
typedef struct {
	inv_imu_edmp_tap_num_t  num;
	inv_imu_edmp_tap_axis_t axis;
	inv_imu_edmp_tap_dir_t  direction;
	uint16_t                double_tap_timing;
	uint16_t                triple_tap_timing;
} inv_imu_edmp_tap_data_t;

/** @brief IMU GAF inputs parameters definition
 *  @note Refer to the datasheet for details on how to configure these parameters.
 */
typedef struct {
	int8_t   clock_variation;
	uint32_t pdr_us;
	int32_t  stationary_angle_enable;
	int32_t  stationary_angle_duration_us;
	int32_t  stationary_angle_threshold_deg_q16;
	int32_t  gyr_cal_stationary_duration_us;
	int32_t  gyr_cal_threshold_metric1;
	int32_t  strict_gyr_cal_threshold_metric1;
	int32_t  gyr_cal_threshold_metric2;
	int32_t  strict_gyr_cal_threshold_metric2;
	int32_t  gyr_cal_sample_num_log2;
	int32_t  gyr_bias_reject_th;
	int32_t  acc_filtering_Npoints_log2;
	int32_t  acc_square_sin_angle_motion_detect_th;
	int32_t  golden_bias_timer;
	int32_t  golden_bias_temperature_validity;
	int32_t  fus_high_speed_drift;
	int32_t  fus_low_speed_drift_roll_pitch;
	int32_t  fus_measurement_covariance_acc;
	int32_t  fus_acceleration_rejection;
} inv_imu_edmp_gaf_parameters_t;

/** @brief IMU GAF outputs definition
 */
typedef struct {
	/** 6-axis (accel and gyro fusion) {w, x, y, z} quaternion */
	int16_t grv_quat_q14[4];

	/** Gyro bias {x, y, z} (1 dps = 1<<12)*/
	int16_t gyr_bias_q12[3];

	/** Gyro accuracy, from 0 (non calibrated) to 3 (well calibrated) */
	int8_t gyr_accuracy_flag;

	/** Stationary detection based on gyro data: 0 is for motion detected, 1 and 2 are for no motion, -1 is for not estimated flag */
	int8_t stationary_flag;
} inv_imu_edmp_gaf_outputs_t;

/** @brief SIF and VVD decision tree
 */
typedef struct {
	/* Pointer to array of [MAX_NODE_SIZE] */
	int16_t *decisionTreeThresholds;
	/* Pointer to array of [MAX_NODE_SIZE] */
	uint8_t *decisionTreeFeatureIDs;
	/* Pointer to array of [MAX_NODE_SIZE] */
	uint8_t *decisionTreeNextNodeRight;
	/* Pointer to array of [MAX_FEATURE_IDS] */
	uint8_t *decisionTreeThresholdsShift;
} inv_imu_edmp_decision_tree_t;

/** @brief SIF configurations from user.
 */
typedef struct {
	/*======================== Common Settings ========================*/
	int32_t wind_size_sample;
	int32_t inv_data_wind;

	/*======================== Time domain Features ========================*/
	int32_t acc_hyst_thr_q16;
	int32_t acc_min_peak_distance;
	int32_t acc_min_peak_height_q16;

	int32_t acc_t_config_num;

	/* Pointer to array of [ACC_T_FILTERS_NUM_MAX][MAX_FLT_COEF] */
	int32_t *acc_t_filtbna_q28;
	/* Pointer to array of [ACC_T_CONFIG_NUM_MAX][ACC_T_CONFIG_NUM_MAX] */
	uint8_t *acc_temporal_feas_config;

	/*======================== Decision Tree Params ========================*/
	int32_t                      node_size;
	inv_imu_edmp_decision_tree_t tree;

	/*======================== Algo ODR ========================*/
	uint32_t sif_odr;
} inv_imu_edmp_sif_user_config_t;

/** @brief IMU VVD inputs parameters definition
 *  @note Refer to the datasheet for details on how to configure these parameters.
 */
typedef struct {
	int32_t delayBeforeHighVVDTrig;
	int32_t delayBeforeLowVVDTrig;
	int32_t nbSamplesBeforeDecision;
	int32_t bestAxis[3];
	int32_t dynEnvVVDThresh;
} inv_imu_edmp_vvd_parameters_t;

/** @brief Configure EDMP Output Data Rate.
 *  @warning Accel frequency must be higher or equal to EDMP frequency.
 *  @warning If inv_imu_edmp_init_apex() was already called, application should call
 *          `inv_imu_edmp_recompute_apex_decimation()` afterwards if APEX algorithms are to be run.
 *  @param[in] s         Pointer to device.
 *  @param[in] frequency The requested frequency.
 *  @return              0 on success, negative value on error.
 */
int inv_imu_edmp_set_frequency(inv_imu_device_t *s, const dmp_ext_sen_odr_cfg_apex_odr_t frequency);

/** @brief Initialize EDMP APEX algorithms. This function should be called before
 *         calling any other function (except for `inv_imu_edmp_set_frequency`).
 *  @warning This function will reset all interrupt masks previously set with
 *           `inv_imu_edmp_unmask_int_src` and exit with `EDMP_INT_SRC_ACCEL_DRDY_MASK|EDMP_INT_SRC_GYRO_DRDY_MASK` unmasked on
 *           `INV_IMU_EDMP_INT0`.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error or if EDMP is enabled.
 */
int inv_imu_edmp_init_apex(inv_imu_device_t *s);

/** @brief Recompute EDMP APEX algorithms internal decimator based on new EDMP output Data Rate
           configured with `inv_imu_edmp_set_frequency`.
 *  @warning It is up to application level to save/restore previously configured APEX parameters,
 *           if any, with `inv_imu_edmp_set_apex_parameters`.
 *  @warning EDMP must be disabled before calling this function.
 *  @warning This function will reset all interrupt masks previously set with
 *           `inv_imu_edmp_unmask_int_src` and exit with `EDMP_INT_SRC_ACCEL_DRDY_MASK|EDMP_INT_SRC_GYRO_DRDY_MASK` unmasked on
 *           `INV_IMU_EDMP_INT0`.
 *  @param[in] s         Pointer to device.
 *  @return              0 on success, negative value on error or if EDMP is enabled.
 */
int inv_imu_edmp_recompute_apex_decimation(inv_imu_device_t *s);

/** @brief Returns current EDMP parameters for APEX algorithms.
 *  @param[in] s   Pointer to device.
 *  @param[out] p  The current parameters read from registers.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_edmp_get_apex_parameters(inv_imu_device_t *s, inv_imu_edmp_apex_parameters_t *p);

/** @brief Configures EDMP parameters for APEX algorithms.
 *  @warning This function should be called only when all EDMP algorithms are disabled.
 *  @param[in] s  Pointer to device.
 *  @param[in] p  The requested input parameters.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_set_apex_parameters(inv_imu_device_t *s, const inv_imu_edmp_apex_parameters_t *p);

/** @brief  Get current GAF configuration settings.
 *  @param[in]  s           Pointer to device.
 *  @param[out] gaf_params  GAF parameters.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_get_gaf_parameters(inv_imu_device_t *s, inv_imu_edmp_gaf_parameters_t *gaf_params);

/** @brief  Set current GAF configuration settings.
 *  @param[in]  s           Pointer to device.
 *  @param[in]  gaf_params  GAF parameters.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_set_gaf_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_gaf_parameters_t *gaf_params);

/** @brief  Get previously computed gyro bias so it can start from non-zero
 *  @param[in] s        Pointer to device.
 *  @param[out] gyr_bias_q12 Gyroscope biases in Q12 read from RAM in eDMP (FIFO output format)
 *  @param[out] gyr_bias_temperature Chip temperature at which gyroscope bias have been computed in Q16
 *  @param[out] accuracy Gyroscope calibration accuracy read from calibration algorithm in eDMP
 *  @return             0 on success, negative value on error.
 *  @warning            This should be called after disabling eDMP to be sure of array consistency.
 */
int inv_imu_edmp_get_gaf_gyr_bias(inv_imu_device_t *s, int16_t gyr_bias_q12[3],
                                  int32_t *gyr_bias_temperature, uint8_t *accuracy);

/** @brief  Set previously computed gyro bias so it can start from non-zero
 *  @param[in] s        Pointer to device.
 *  @param[in] gyr_bias_q12 Gyroscope biases to be applied in Q12 (FIFO format)
 *  @param[in] gyr_bias_temperature Chip temperature at which gyroscope bias have been computed in Q16
 *  @param[in] accuracy Gyroscope accuracy to be applied
 *  @return             0 on success, negative value on error.
 *  @warning            This must be called before inv_imu_edmp_set_gaf_parameters().
 */
int inv_imu_edmp_set_gaf_gyr_bias(inv_imu_device_t *s, const int16_t gyr_bias_q12[3],
                                  const int32_t gyr_bias_temperature, const uint8_t accuracy);

/** @brief Set previously computed accel bias so it can start from non-zero
 *  @param[in] s            Pointer to device.
 *  @param[in] acc_bias_q16 Accelerometer biases to be applied in Q16
 *  @return                 0 on success, negative value on error.
 *  @warning                This must be called before inv_imu_edmp_set_gaf_parameters().
 */
int inv_imu_edmp_set_gaf_acc_bias(inv_imu_device_t *s, const int32_t acc_bias_q16[3]);

/** @brief  Initialize SIF configuration with user settings.
 *  @param[in]  s    Pointer to device.
 *  @param[out] cfg  SIF parameters.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_edmp_set_sif_model(inv_imu_device_t *s, const inv_imu_edmp_sif_user_config_t *cfg);

/** @brief  Run on-demand service to request SIF set PDR on-demand feature.
 *  @param[in]  s                Pointer to device.
 *  @param[in]  pdr50_or_100hz   1 to request SIF PDR 100Hz, 0 to request SIF PDR 50Hz (default).
 *  @return                      0 on success, negative value on error.
 */
int inv_imu_edmp_set_sif_pdr(inv_imu_device_t *s, uint8_t pdr50_or_100hz);

/** @brief  Set current VVD machine learning model.
 *  @param[in] s                           Pointer to device.
 *  @return                                0 on success, negative value on error.
 */
int inv_imu_edmp_set_vvd_model(inv_imu_device_t *s);

/** @brief  Get current VVD configuration settings.
 *  @param[in]  s           Pointer to device.
 *  @param[out] vvd_params  VVD parameters.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_get_vvd_parameters(inv_imu_device_t *s, inv_imu_edmp_vvd_parameters_t *vvd_params);

/** @brief  Set current VVD configuration settings.
 *  @param[in]  s           Pointer to device.
 *  @param[in]  vvd_params  VVD parameters.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_set_vvd_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_vvd_parameters_t *vvd_params);

/** @brief  Apply a mounting-matrix at EDMP level, on all input data (axis remapping)
 *  @param[in] s                Pointer to device.
 *  @param[in] mounting_matrix  Mounting-matrix composed of -1/0/1 values.
 *  @return                     0 on success, negative value on error.
 */
int inv_imu_edmp_set_mounting_matrix(inv_imu_device_t *s, const int8_t mounting_matrix[9]);

/** @brief Retrieve interrupts configuration.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Configuration of each APEX interrupt.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_edmp_get_config_int_apex(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it);

/** @brief Configure APEX interrupt.
 *  @param[in] s   Pointer to device.
 *  @param[in] it  State of each APEX interrupt to configure.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_edmp_set_config_int_apex(inv_imu_device_t *s, const inv_imu_edmp_int_state_t *it);

/** @brief  Enable EDMP.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_enable(inv_imu_device_t *s);

/** @brief  Disable EDMP.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable(inv_imu_device_t *s);

/** @brief  Enable Activity/Inactivity Detection algorithm.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_aid(inv_imu_device_t *s);

/** @brief  Disable Activity/Inactivity Detection algorithm.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_aid(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm Tap.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_tap(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Tap.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_tap(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_ff(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_ff(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm B2S. 
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_b2s(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm B2S.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_b2s(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm GAF.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_enable_gaf(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm GAF.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_gaf(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm SIF.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_enable_sif(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm SIF.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_sif(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm VVD.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_enable_vvd(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm VVD.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_vvd(inv_imu_device_t *s);

/** @brief Read APEX interrupt status.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Status of each APEX interrupt.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_edmp_get_int_apex_status(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it);

/** @brief Retrieve APEX free fall outputs and format them.
 *  @param[in] s                   Pointer to device.
 *  @param[out] freefall_duration  Duration in number of sample.
 *  @return                        0 on success, negative value on error.
 */
int inv_imu_edmp_get_ff_data(inv_imu_device_t *s, uint16_t *freefall_duration);

/** @brief Retrieve tap outputs.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Tap number and direction.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_edmp_get_tap_data(inv_imu_device_t *s, inv_imu_edmp_tap_data_t *data);

/** @brief  Retrieve the current output state of the AID human instance
 *  @param[in] s              Pointer to device.
 *  @param[out] output_state  AID human instance current output state
 *  @return                   0 on success, negative value on error.
 */
int inv_imu_edmp_get_aid_data_human(inv_imu_device_t *s, uint8_t *output_state);

/** @brief  Retrieve the current output state of the AID device instance
 *  @param[in] s              Pointer to device.
 *  @param[out] output_state  AID device instance current output state
 *  @return                   0 on success, negative value on error.
 */
int inv_imu_edmp_get_aid_data_device(inv_imu_device_t *s, uint8_t *output_state);

/** @brief  Build GAF output value based on FIFO frame. 
 *  @param[in]  s    Pointer to device.
 *  @param[in]  es0  FIFO frame field of ES data, typically es0 field of @sa inv_imu_sensor_event_t.
 *  @param[in]  es1  FIFO frame field of ES data, typically es1 field of @sa inv_imu_sensor_event_t.
 *  @param[out] out  Pointer to GAF output structure, which will hold GAF latest values computed.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_edmp_gaf_decode_fifo(inv_imu_device_t *s, const uint8_t es0[9], const uint8_t es1[6],
                                 inv_imu_edmp_gaf_outputs_t *const out);

/** @brief  Get SIF output value - class index. 
 *  @param[in]  s            Pointer to device.
 *  @param[out] class_index  predicted class index.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_edmp_get_sif_class_index(inv_imu_device_t *s, int16_t *class_index);

/** @brief  Mask requested interrupt sources for edmp interrupt line passed in parameter.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line number that should be configured.
 *  @param[in] int_mask     Interrupt sources to mask.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_mask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
                              uint8_t int_mask);

/** @brief  Unmask requested interrupt sources for edmp interrupt line passed in parameter.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line number that should be configured.
 *  @param[in] int_mask     Interrupt sources to unmask. 
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_unmask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
                                uint8_t int_mask);

/** @brief  Setup EDMP to execute code in ROM.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_configure(inv_imu_device_t *s);

/** @brief Run EDMP using the on-demand mechanism.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_run_ondemand(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb);

/** @brief Request dynamic environment threshold parameter update of VVD without re-init.
 *  @param[in]  s                   Pointer to device.
 *  @param[in]  new_dyn_env_thresh  new VVD parameter to be applied.
 *  @return                         0 on success, negative value on error.
 *  @note The return value marks the fact that request was sent, inv_imu_edmp_check_vvd_thresh should be used to validate the actual update inside edmp.
 *  @sa inv_imu_edmp_check_vvd_thresh
 */
int inv_imu_edmp_request_vvd_update_thresh(inv_imu_device_t *s, const int32_t new_dyn_env_thresh);

/** @brief Verify dynamic environment threshold parameter update status.
 *  @param[in]  s                   Pointer to device.
 *  @param[out] dyn_thresh_status   Current status of dynamic update, can be NULL if caller wishes to ignore this info, will be 0 until last update didn't complete.
 *  @return                         0 on success, negative value on error.
 *  @sa inv_imu_vvd_request_update_dyn_thresh
 */
int inv_imu_edmp_check_vvd_thresh(inv_imu_device_t *s, uint8_t *const dyn_thresh_status);

/** @brief  Run EDMP using the on-demand mechanism to execute memset value in RAM range [start_addr, start_addr+size-1].
 *  @param[in] s           Pointer to device.
 *  @param[in] start_addr  RAM start address for which memset is to be done.
 *  @param[in] size        Size in bytes of RAM area to be set.
 *  @param[in] value       Value to be written to RAM area.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_edmp_write_ram_area(inv_imu_device_t *s, uint16_t start_addr, uint16_t size,
                                uint8_t value);

/** @brief Wait until EDMP idle bit is set (means EDMP execution is completed).
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_wait_for_idle(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_EDMP_H_ */

/** @} */
