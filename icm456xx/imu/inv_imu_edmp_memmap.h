/*
 * ________________________________________________________________________________________________________
 * Copyright © 2021 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 * ________________________________________________________________________________________________________
 */

#ifndef __INV_IMU_EDMP_MEMMAP_H__
#define __INV_IMU_EDMP_MEMMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* use_case_bitmask
 *
 * ISR1 init request service ID depending on system usecase
 * Set bit 0 for usecase which does not require VVD
 * Set bit 1 for usecase which does not require GAF
 * Set bit 2 for usecase which does not require SIF
 * Set bit 3 for usecase which does not require Free-Fall
 * Set bit 4 for usecase which does not require TAP
 * Set bit 5 for usecase which does not require B2S
 * Set bit 6 for usecase which does not require AID for human
 * Set bit 7 for usecase which does not require AID for device
 * Default: 0 (initialize all algo)
 */
#define EDMP_USE_CASE_BITMASK                                   0x5
#define EDMP_USE_CASE_BITMASK_SIZE                              1

/* ondemand_dynamic_service_request
 *
 * ISR1 on-demand service request ID
 * Set bit 1 to request set GAF parameters
 * Set bit 2 to request set GAF bias
 * Set bit 3 to request eDMP reconfiguration for GAF PDR 100Hz
 * Set bit 4 to request eDMP reconfiguration for GAF PDR 50Hz
 * Set bit 5 to request eDMP reconfiguration for ML PDR 100Hz
 * Set bit 6 to request eDMP reconfiguration for ML PDR 50Hz
 * Set bit 7 to request VVD dynamic on-the-fly parameterization
 * Default: 0 (no ISR1 service requested)
 */
#define EDMP_ONDEMAND_DYNAMIC_SERVICE_REQUEST                   0x4
#define EDMP_ONDEMAND_DYNAMIC_SERVICE_REQUEST_SIZE              1

/* ondemand_static_service_request
 *
 * ISR2 on-demand service request ID
 * Set bit 0 to request memset service
 * Default: 0 (no ISR2 service requested)
 */
#define EDMP_ONDEMAND_STATIC_SERVICE_REQUEST                    0x138
#define EDMP_ONDEMAND_STATIC_SERVICE_REQUEST_SIZE               4

/* ondemand_memset_addr
 *
 * Start address of RAM area to be written by EDMP when requested by memset static service.
 */
#define EDMP_ONDEMAND_MEMSET_ADDR                               0x13c
#define EDMP_ONDEMAND_MEMSET_ADDR_SIZE                          2

/* ondemand_memset_size
 *
 * Length in bytes of RAM area to be written by EDMP when requested by memset static service.
 */
#define EDMP_ONDEMAND_MEMSET_SIZE                               0x140
#define EDMP_ONDEMAND_MEMSET_SIZE_SIZE                          2

/* ondemand_memset_value
 *
 * Value to be written by EDMP to RAM area when requested by memset static service.
 */
#define EDMP_ONDEMAND_MEMSET_VALUE                              0x13e
#define EDMP_ONDEMAND_MEMSET_VALUE_SIZE                         1

/* dmp_odr_last_init
 *
 * Encoded eDMP ODR value effective during last eDMP init request
 * 0x1 for eDMP ODR 25Hz
 * 0x2 for eDMP ODR 50Hz
 * 0x8 for eDMP ODR 100Hz
 * 0x80 for eDMP ODR 200Hz
 * 0x8000 for eDMP ODR 400Hz
 * 0x80000000 for eDMP ODR 800Hz
 */
#define EDMP_DMP_ODR_LAST_INIT                                  0x1b4
#define EDMP_DMP_ODR_LAST_INIT_SIZE                             4

/* global_mounting_matrix
 *
 * A q14 3x3 matrix applied to input accel and gyro data
 * Default:Identity matrix being
 * 0x4000   0     0  
 *   0   0x4000   0 
 *   0     0   0x4000
 */
#define EDMP_GLOBAL_MOUNTING_MATRIX                             0x330
#define EDMP_GLOBAL_MOUNTING_MATRIX_SIZE                        18

/* lowg_peak_th
 *
 * Threshold for accel values below which low-g state is detected.
 * Unit: g in q12
 * Range: [128 - 4096]
 * Default: 2048
 */
#define EDMP_LOWG_PEAK_TH                                       0xcc
#define EDMP_LOWG_PEAK_TH_SIZE                                  2

/* lowg_peak_th_hyst
 *
 * Hysteresis value added to the low-g threshold after accel values get below threshold.
 * Unit: g in q12
 * Range: [128 - 1024]
 * Default: 128
 */
#define EDMP_LOWG_PEAK_TH_HYST                                  0xce
#define EDMP_LOWG_PEAK_TH_HYST_SIZE                             2

/* lowg_time_th
 *
 * Number of samples required to enter low-g state.
 * Unit: time in samples number
 * Range: [1 - 300]
 * Default: 13 (set for default ODR = 800 Hz, equivalent to 16 ms)
 */
#define EDMP_LOWG_TIME_TH                                       0xd0
#define EDMP_LOWG_TIME_TH_SIZE                                  2

/* highg_peak_th
 *
 * Threshold for accel values above which high-g state is detected.
 * Unit: g in q12
 * Range: [1024 - 32768]
 * Default: 29696
 */
#define EDMP_HIGHG_PEAK_TH                                      0xc0
#define EDMP_HIGHG_PEAK_TH_SIZE                                 2

/* highg_peak_th_hyst
 *
 * Hysteresis value subtracted from the high-g threshold after exceeding it.
 * Unit: g in q12
 * Range: [128 - 1024]
 * Default: 640
 */
#define EDMP_HIGHG_PEAK_TH_HYST                                 0xc2
#define EDMP_HIGHG_PEAK_TH_HYST_SIZE                            2

/* highg_time_th
 *
 * The number of samples device should stay above (highg_peak_th + highg_peak_th_hyst) before HighG state is triggered.
 * Unit: time in samples number
 * Range: [1-300]
 * Default: 1 (set for default ODR = 800 Hz, equivalent to 1.25 ms)
 */
#define EDMP_HIGHG_TIME_TH                                      0xc4
#define EDMP_HIGHG_TIME_TH_SIZE                                 2

/* ff_min_duration
 *
 * Minimum freefall duration. Shorter freefalls are ignored.
 * Unit: time in samples number
 * Range: [4 - 420]
 * Default: 57 (set for default ODR = 400 Hz, equivalent to 142 ms)
 */
#define EDMP_FF_MIN_DURATION                                    0xb0
#define EDMP_FF_MIN_DURATION_SIZE                               4

/* ff_max_duration
 *
 * Maximum freefall duration. Longer freefalls are ignored.
 * Unit: time in samples number
 * Range: [12 - 1040]
 * Default: 285 (set for default ODR = 400 Hz, equivalent to 712 ms)
 */
#define EDMP_FF_MAX_DURATION                                    0xb4
#define EDMP_FF_MAX_DURATION_SIZE                               4

/* ff_debounce_duration
 *
 * Period after a freefall is signaled during which a new freefall will not be detected. Prevents false detection due to bounces.
 * Unit: time in samples number
 * Range: [75 - 3000]
 * Default: 800 (set for default ODR = 800 Hz, equivalent to 1 s)
 */
#define EDMP_FF_DEBOUNCE_DURATION                               0xb8
#define EDMP_FF_DEBOUNCE_DURATION_SIZE                          4

/* ff_duration
 *
 * Duration of the freefall.
 * Unit: number of samples. Freefall duration in seconds / ACCEL_ODR_Hz
 */
#define EDMP_FF_DURATION                                        0xaa
#define EDMP_FF_DURATION_SIZE                                   2

/* aid_win_human
 *
 * The window time in number of samples to wait for continuous WOM before triggering AID
 * Unit: time in sample number
 * Range: [25 - 500] 
 * Default: 150 (3sec at 50Hz)
 * Recommended value at 50Hz = 150
 * Recommended value at 25Hz = 75
 */
#define EDMP_AID_WIN_HUMAN                                      0x300
#define EDMP_AID_WIN_HUMAN_SIZE                                 4

/* aid_alert_human
 *
 * The window time in number of samples before AID alert trigger after WOM stop reported motion
 * Unit: time in sample number
 * Range: [150 - 2147483648] 
 * Default: 90000 (30min at 50Hz)
 * Recommended value at 50Hz = 90000
 * Recommended value at 25Hz = 45000
 */
#define EDMP_AID_ALERT_HUMAN                                    0x304
#define EDMP_AID_ALERT_HUMAN_SIZE                               4

/* aid_en_output_human
 *
 * Bitmask controlling which output are enabled from AID
 * No Unit: bit 0 for enable activity detection, bit 1 for inactivity detection, bit 3 for alert
 * Default: 7 (all output enabled)
 */
#define EDMP_AID_EN_OUTPUT_HUMAN                                0x2fc
#define EDMP_AID_EN_OUTPUT_HUMAN_SIZE                           1

/* aid_dis_multi_output_human
 *
 * Option to disable output after each internal decision of the algorithm
 * No Unit: 1 to disable repetition of same state, 0 to enable repetition of same state after aid_win_human
 * Default: 0 (repetition enabled)
 */
#define EDMP_AID_DIS_MULTI_OUTPUT_HUMAN                         0x2fd
#define EDMP_AID_DIS_MULTI_OUTPUT_HUMAN_SIZE                    1

/* aid_win_device
 *
 * The window time in number of samples to wait for continuous WOM before triggering AID
 * Unit: time in sample number
 * Range: [25 - 500] 
 * Default: 150 (3sec at 50Hz)
 * Recommended value at 50Hz = 150
 * Recommended value at 25Hz = 75
 */
#define EDMP_AID_WIN_DEVICE                                     0x324
#define EDMP_AID_WIN_DEVICE_SIZE                                4

/* aid_alert_device
 *
 * The window time in number of samples before AID alert trigger after WOM stop reported motion
 * Unit: time in sample number
 * Range: [150 - 2147483648] 
 * Default: 90000 (30min at 50Hz)
 * Recommended value at 50Hz = 90000
 * Recommended value at 25Hz = 45000
 */
#define EDMP_AID_ALERT_DEVICE                                   0x328
#define EDMP_AID_ALERT_DEVICE_SIZE                              4

/* aid_en_output_device
 *
 * Bitmask controlling which output are enabled from AID
 * No Unit: bit 0 for enable activity detection, bit 1 for inactivity detection, bit 3 for alert
 * Default: 7 (all output enabled)
 */
#define EDMP_AID_EN_OUTPUT_DEVICE                               0x320
#define EDMP_AID_EN_OUTPUT_DEVICE_SIZE                          1

/* aid_dis_multi_output_device
 *
 * Option to disable output after each internal decision of the algorithm
 * No Unit: 1 to disable repetition of same state, 0 to enable repetition of same state after aid_win_device
 * Default: 0 (repetition enabled)
 */
#define EDMP_AID_DIS_MULTI_OUTPUT_DEVICE                        0x321
#define EDMP_AID_DIS_MULTI_OUTPUT_DEVICE_SIZE                   1

/* aid_human_output_state
 *
 * Decision taken by the AID algorithm for human instance
 * No unit: 1 when activity is detected, 2 when inactivity is detected, 6 when inactivity and sedentary alert are detected
 */
#define EDMP_AID_HUMAN_OUTPUT_STATE                             0x309
#define EDMP_AID_HUMAN_OUTPUT_STATE_SIZE                        1

/* aid_device_output_state
 *
 * Decision taken by the AID algorithm for device instance
 * No unit: 1 when activity is detected, 2 when inactivity is detected, 6 when inactivity and sedentary alert are detected
 */
#define EDMP_AID_DEVICE_OUTPUT_STATE                            0x32d
#define EDMP_AID_DEVICE_OUTPUT_STATE_SIZE                       1

/* gaf_pdr_partition
 *
 * GAF partition reconfiguration in case GAF PDR is not 50 Hz or 100 Hz.
 */
#define EDMP_GAF_PDR_PARTITION                                  0x1a0
#define EDMP_GAF_PDR_PARTITION_SIZE                             4

/* gaf_init_status
 *
 * GAF initialization status. Set to 1 by eDMP once GAF initialization is done.
 * Default: 0 (GAF initialization not performed)
 */
#define EDMP_GAF_INIT_STATUS                                    0x6b6
#define EDMP_GAF_INIT_STATUS_SIZE                               1

/* gaf_config_acc_odr_us
 *
 * GAF accelerometer input sensor data rate.
 * Range: {1250;2500;5000;10000;20000}
 * Unit: us
 * Default: 10000
 */
#define EDMP_GAF_CONFIG_ACC_ODR_US                              0x344
#define EDMP_GAF_CONFIG_ACC_ODR_US_SIZE                         4

/* gaf_config_gyr_odr_us
 *
 * GAF gyroscope input sensor data rate.
 * Range: {1250;2500;5000;10000;20000}
 * Unit: us
 * Default: 10000
 */
#define EDMP_GAF_CONFIG_GYR_ODR_US                              0x348
#define EDMP_GAF_CONFIG_GYR_ODR_US_SIZE                         4

/* gaf_config_acc_pdr_us
 *
 * GAF accelerometer processing data rate, sensor data are averaged if sensor is faster than GAF accelerometer PDR.
 * Range: {1250;2500;5000;10000;20000}
 * Unit: us
 * Default: 10000
 */
#define EDMP_GAF_CONFIG_ACC_PDR_US                              0x34c
#define EDMP_GAF_CONFIG_ACC_PDR_US_SIZE                         4

/* gaf_config_gyr_pdr_us
 *
 * GAF gyroscope processing data rate, sensor data are averaged if sensor is faster than GAF gyroscope PDR. Corresponds to GAF output data rate.
 * Range: {1250;2500;5000;10000;20000}
 * Unit: us
 * Default: 10000
 */
#define EDMP_GAF_CONFIG_GYR_PDR_US                              0x350
#define EDMP_GAF_CONFIG_GYR_PDR_US_SIZE                         4

/* gaf_config_stationary_angle_duration_us
 *
 * Duration of stationary detection.
 * Unit: us
 * Default: 500000 (0.5seconds)
 */
#define EDMP_GAF_CONFIG_STATIONARY_ANGLE_DURATION_US            0x35c
#define EDMP_GAF_CONFIG_STATIONARY_ANGLE_DURATION_US_SIZE       4

/* gaf_config_stationary_angle_threshold_deg_q16
 *
 * Threshold on angular deviation for stationary detection.
 * Unit: degree s32q16
 * Default: 65536 (1 degree)
 */
#define EDMP_GAF_CONFIG_STATIONARY_ANGLE_THRESHOLD_DEG_Q16      0x360
#define EDMP_GAF_CONFIG_STATIONARY_ANGLE_THRESHOLD_DEG_Q16_SIZE 4

/* gaf_config_fus_low_speed_drift_roll_pitch
 *
 * Gyroscope integration error related to bias precision. Higher value increases accel roll/pitch correction in steady state.
 * Unit: LSB
 * Default : 20
 */
#define EDMP_GAF_CONFIG_FUS_LOW_SPEED_DRIFT_ROLL_PITCH          0x364
#define EDMP_GAF_CONFIG_FUS_LOW_SPEED_DRIFT_ROLL_PITCH_SIZE     4

/* gaf_config_pll_clock_variation
 *
 * Error on the clock in same format as reg SW_PLL1_TRIM. Calculated as (actual_clk - target_clk) / target_clk * (2^7 - 1) / 5 * 100.
 * Default : 0
 */
#define EDMP_GAF_CONFIG_PLL_CLOCK_VARIATION                     0x36c
#define EDMP_GAF_CONFIG_PLL_CLOCK_VARIATION_SIZE                1

/* gaf_config_stationary_angle_enable
 *
 * Enable/disable stop integration of stationary angle.
 * Default : 0
 */
#define EDMP_GAF_CONFIG_STATIONARY_ANGLE_ENABLE                 0x368
#define EDMP_GAF_CONFIG_STATIONARY_ANGLE_ENABLE_SIZE            4

/* gaf_config_loose_gyr_cal_sample_num_log2
 *
 * Gyroscope calibration number of samples used to estimate metric1 and metric2, and minimum gyroscope calibration duration.
 * Unit: number of samples in log2
 * Range: [6 - 8]
 * Default: 7 (so 2^7 = 128 samples)
 */
#define EDMP_GAF_CONFIG_LOOSE_GYR_CAL_SAMPLE_NUM_LOG2           0x5c8
#define EDMP_GAF_CONFIG_LOOSE_GYR_CAL_SAMPLE_NUM_LOG2_SIZE      4

/* gaf_config_golden_bias_timer
 *
 * Validity timer of the strict bias in sample number.
 * Default: 1440000 (so 8 hours at 50Hz)
 */
#define EDMP_GAF_CONFIG_GOLDEN_BIAS_TIMER                       0x5c0
#define EDMP_GAF_CONFIG_GOLDEN_BIAS_TIMER_SIZE                  4

/* gaf_config_gyr_dt_us
 *
 * Gyro ODR corrected with PLL clock correction.
 * Unit: us
 * Default: 20000
 */
#define EDMP_GAF_CONFIG_GYR_DT_US                               0x400
#define EDMP_GAF_CONFIG_GYR_DT_US_SIZE                          4

/* gaf_config_fus_high_speed_drift
 *
 * Percentage of error (covering gyroscope sensitivity, timestamp and quantization) on gyroscope integration.
 * Unit: LSB with 2^15 LSB = 1% error
 * Default : 262144 (so 8% error)
 */
#define EDMP_GAF_CONFIG_FUS_HIGH_SPEED_DRIFT                    0x420
#define EDMP_GAF_CONFIG_FUS_HIGH_SPEED_DRIFT_SIZE               4

/* gaf_config_loose_gyr_cal_stationary_duration_us
 *
 * Duration for no motion gyroscope bias calibration.
 * Unit: us
 * Default value: 500000
 */
#define EDMP_GAF_CONFIG_LOOSE_GYR_CAL_STATIONARY_DURATION_US    0x4cc
#define EDMP_GAF_CONFIG_LOOSE_GYR_CAL_STATIONARY_DURATION_US_SIZE 4

/* gaf_config_loose_gyr_cal_threshold_metric1
 *
 * Stationary detection threshold of 1st metric for the loose bias calibration. Threshold is compare against absolute value of delta between current gyro value and last gyro value in an analysis window.
 * Unit: 2000dps = 2^20
 * Default value: 1200 (so 2.28dps)
 */
#define EDMP_GAF_CONFIG_LOOSE_GYR_CAL_THRESHOLD_METRIC1         0x4ec
#define EDMP_GAF_CONFIG_LOOSE_GYR_CAL_THRESHOLD_METRIC1_SIZE    4

/* gaf_config_loose_gyr_cal_threshold_metric2
 *
 * Stationary detection threshold of 2st metric for the loose bias calibration.
 * Default value: 60000 (no unit).
 */
#define EDMP_GAF_CONFIG_LOOSE_GYR_CAL_THRESHOLD_METRIC2         0x4dc
#define EDMP_GAF_CONFIG_LOOSE_GYR_CAL_THRESHOLD_METRIC2_SIZE    4

/* gaf_config_strict_gyr_cal_threshold_metric1
 *
 * Stationary detection threshold of 1st metric for the strict bias calibration. Threshold is compare against absolute value of delta between current gyro value and last gyro value in an analysis window.
 * Unit: 2000dps = 2^20
 * Default value: 300 (so 0.57dps)
 */
#define EDMP_GAF_CONFIG_STRICT_GYR_CAL_THRESHOLD_METRIC1        0x5b8
#define EDMP_GAF_CONFIG_STRICT_GYR_CAL_THRESHOLD_METRIC1_SIZE   4

/* gaf_config_strict_gyr_cal_threshold_metric2
 *
 * Stationary detection threshold of 2st metric for the strict bias calibration.
 * Default value: 400 (no unit).
 */
#define EDMP_GAF_CONFIG_STRICT_GYR_CAL_THRESHOLD_METRIC2        0x5bc
#define EDMP_GAF_CONFIG_STRICT_GYR_CAL_THRESHOLD_METRIC2_SIZE   4

/* gaf_config_gyr_bias_reject_th
 *
 * Gyro bias rejection threshold above which device is considered as moving. Threshold is compare against absolute value of delta between current estimated gyro bias and last estimated gyro bias in an analysis window.
 * Unit: 2000dps = 2^30.
 * Default value: 3650722 (so 6.8dps)
 */
#define EDMP_GAF_CONFIG_GYR_BIAS_REJECT_TH                      0x4e0
#define EDMP_GAF_CONFIG_GYR_BIAS_REJECT_TH_SIZE                 4

/* gaf_config_acc_filtering_Npoints_log2
 *
 * Accelerometer filtering mean value.
 * Unit: sample number in log2
 * Default: 5 (so 32 samples)
 */
#define EDMP_GAF_CONFIG_ACC_FILTERING_NPOINTS_LOG2              0x5b4
#define EDMP_GAF_CONFIG_ACC_FILTERING_NPOINTS_LOG2_SIZE         4

/* gaf_config_acc_square_sin_angle_motion_detect_th
 *
 * Square of sinus on angle threshold to reject gyroscope calibration.
 * Unit: (sin(theta)2)*225
 * Default: 4318 (so 0.65 degree)
 */
#define EDMP_GAF_CONFIG_ACC_SQUARE_SIN_ANGLE_MOTION_DETECT_TH   0x5b0
#define EDMP_GAF_CONFIG_ACC_SQUARE_SIN_ANGLE_MOTION_DETECT_TH_SIZE 4

/* gaf_config_golden_bias_temperature_validity
 *
 * Validity temperature variation of the strict bias.
 * Unit: degree C s32q16
 * Default: 983040 (so 15 degree C)
 */
#define EDMP_GAF_CONFIG_GOLDEN_BIAS_TEMPERATURE_VALIDITY        0x5c4
#define EDMP_GAF_CONFIG_GOLDEN_BIAS_TEMPERATURE_VALIDITY_SIZE   4

/* gaf_config_fus_measurement_covariance_acc
 *
 * Accelerometer measurement covariance.
 * Unit: g^2 s32q15
 * Default: 32768 (so 0.5g^2)
 */
#define EDMP_GAF_CONFIG_FUS_MEASUREMENT_COVARIANCE_ACC          0x408
#define EDMP_GAF_CONFIG_FUS_MEASUREMENT_COVARIANCE_ACC_SIZE     4

/* gaf_config_fus_acceleration_rejection
 *
 * Linear acceleration rejection.
 * Unit: m/s^2 s32q30
 * Range: [0 - 1073741824]
 * Default: 1073741824 (so 1.0, being maximum rejection)
 */
#define EDMP_GAF_CONFIG_FUS_ACCELERATION_REJECTION              0x41c
#define EDMP_GAF_CONFIG_FUS_ACCELERATION_REJECTION_SIZE         4

/* gaf_saved_acc_bias_1g_q16
 *
 * Externally computed accelerometer biases to feed inside the fusion (one value per axis).
 * Unit: gee in s32q16
 * Default: 0;0;0
 */
#define EDMP_GAF_SAVED_ACC_BIAS_1G_Q16                          0x66c
#define EDMP_GAF_SAVED_ACC_BIAS_1G_Q16_SIZE                     12

/* gaf_saved_acc_accuracy
 *
 * Accelerometer accuracy from 0 (non calibrated) to 3 (well calibrated).
 * Range : [0 - 3]
 * Default: 0
 */
#define EDMP_GAF_SAVED_ACC_ACCURACY                             0x6b5
#define EDMP_GAF_SAVED_ACC_ACCURACY_SIZE                        1

/* gaf_saved_gyr_bias_dps_q12
 *
 * Previous gyroscope bias to start with (one value per axis).
 * Unit: dps in s32q12
 * Default: 0;0;0
 */
#define EDMP_GAF_SAVED_GYR_BIAS_DPS_Q12                         0x5cc
#define EDMP_GAF_SAVED_GYR_BIAS_DPS_Q12_SIZE                    12

/* gaf_saved_gyr_accuracy
 *
 * Gyroscope accuracy from 0 (non calibrated) to 3 (well calibrated).
 * Range : [0 - 3]
 * Default: 0
 */
#define EDMP_GAF_SAVED_GYR_ACCURACY                             0x4bc
#define EDMP_GAF_SAVED_GYR_ACCURACY_SIZE                        4

/* gaf_saved_gyr_bias_temperature_deg_q16
 *
 * Previously stored temperature when gyro bias was estimated.
 * Unit: degree C in s32q16
 * Default: 
 */
#define EDMP_GAF_SAVED_GYR_BIAS_TEMPERATURE_DEG_Q16             0x5a0
#define EDMP_GAF_SAVED_GYR_BIAS_TEMPERATURE_DEG_Q16_SIZE        4

/* sif_time_feas_config
 *
 * Structure member of the SIF configuration generated with TDK SIF software tools. It shall be loaded as-is based on TDK recommendation.
 */
#define EDMP_SIF_TIME_FEAS_CONFIG                               0x97c
#define EDMP_SIF_TIME_FEAS_CONFIG_SIZE                          2

/* sif_filter
 *
 * Structure member of the SIF filter configuration generated with TDK SIF software tools. It shall be loaded as-is based on TDK recommendation.
 */
#define EDMP_SIF_FILTER                                         0x9fc
#define EDMP_SIF_FILTER_SIZE                                    80

/* sif_cconfig
 *
 * Structure member of the SIF filter configuration generated with TDK SIF software tools. It shall be loaded as-is based on TDK recommendation.
 */
#define EDMP_SIF_CCONFIG                                        0x9d4
#define EDMP_SIF_CCONFIG_SIZE                                   12

/* sif_tconfig
 *
 * Structure member of the SIF filter configuration generated with TDK SIF software tools. It shall be loaded as-is based on TDK recommendation.
 */
#define EDMP_SIF_TCONFIG                                        0x9e0
#define EDMP_SIF_TCONFIG_SIZE                                   24

/* sif_tree_thresholds
 *
 * Structure member of the SIF model Decision Tree generated with TDK SIF software tools. It shall be loaded as-is based on TDK recommendation.
 */
#define EDMP_SIF_TREE_THRESHOLDS                                0x138c
#define EDMP_SIF_TREE_THRESHOLDS_SIZE                           2

/* sif_tree_featureids
 *
 * Structure member of the SIF model Decision Tree generated with TDK SIF software tools. It shall be loaded as-is based on TDK recommendation.
 */
#define EDMP_SIF_TREE_FEATUREIDS                                0x158a
#define EDMP_SIF_TREE_FEATUREIDS_SIZE                           1

/* sif_tree_nextnoderight
 *
 * Structure member of the SIF model Decision Tree generated with TDK SIF software tools. It shall be loaded as-is based on TDK recommendation.
 */
#define EDMP_SIF_TREE_NEXTNODERIGHT                             0x1689
#define EDMP_SIF_TREE_NEXTNODERIGHT_SIZE                        1

/* sif_tree_thresholdsshift
 *
 * Structure member of the SIF model Decision Tree generated with TDK SIF software tools. It shall be loaded as-is based on TDK recommendation.
 */
#define EDMP_SIF_TREE_THRESHOLDSSHIFT                           0x1788
#define EDMP_SIF_TREE_THRESHOLDSSHIFT_SIZE                      1

/* sif_class_index
 *
 * Index of SIF predicted gesture class.
 */
#define EDMP_SIF_CLASS_INDEX                                    0x994
#define EDMP_SIF_CLASS_INDEX_SIZE                               2

/* b2s_mounting_matrix
 *
 * Mounting matrix to apply the accelerometer data before bring-to-see computation, as a bit-mask combination of operations.
 * bit0: flip Y, flip Z
 * bit1: flip X, flip Z
 * bit2: swap X/Y, flip Z
 * Range: [0 - 7]
 * Default: 0 being identity matrix
 */
#define EDMP_B2S_MOUNTING_MATRIX                                0x2df
#define EDMP_B2S_MOUNTING_MATRIX_SIZE                           1

/* b2s_one_g_value
 *
 * One gee value to be used as reference to trigger Bring-To-See.
 * Unit: 1gee = 2^12
 * Default: 4096
 */
#define EDMP_B2S_ONE_G_VALUE                                    0x204
#define EDMP_B2S_ONE_G_VALUE_SIZE                               4

/* b2s_settings_dev_norm_max
 *
 * Hysteresis added or removed to norm estimate and Y axis constrains value for RevB2S.
 * Unit: LSB, with 1 LBS = 1g / 2^12
 * Range: [1 - 2048]
 * Default: 700 (corresponding to 0.1709g)
 */
#define EDMP_B2S_SETTINGS_DEV_NORM_MAX                          0x208
#define EDMP_B2S_SETTINGS_DEV_NORM_MAX_SIZE                     4

/* b2s_settings_sin_limit
 *
 * Maximum threshold on absolute value of X axis in b2s position. Link to the sinus value of inclination angle on X axis.
 * Unit: LSB, with 1 LBS = 1g / 2^12
 * Range: [300 - 3000]
 * Default: 2048 (corresponding to 30deg)
 */
#define EDMP_B2S_SETTINGS_SIN_LIMIT                             0x20c
#define EDMP_B2S_SETTINGS_SIN_LIMIT_SIZE                        4

/* b2s_settings_fast_limit
 *
 * Threshold of minimal motion to be detected as "Fast motion".
 * Unit: No unit, filtered data
 * Range: [300 - 3000]
 * Default: 200
 */
#define EDMP_B2S_SETTINGS_FAST_LIMIT                            0x210
#define EDMP_B2S_SETTINGS_FAST_LIMIT_SIZE                       4

/* b2s_settings_static_limit
 *
 * Threshold to determine static phase required after the gesture B2S to validate it.
 * Unit: No unit, filtered data
 * Default: 1400
 */
#define EDMP_B2S_SETTINGS_STATIC_LIMIT                          0x214
#define EDMP_B2S_SETTINGS_STATIC_LIMIT_SIZE                     4

/* b2s_settings_thr_cos_ang
 *
 * RevB2S threshold, condition satisfied when moving away from bring2see orientation position by more than threshold angle corresponding to the cosine angle.
 * Unit: cosine value of angle in q30
 * Default: 1057429273 (so 10deg)
 */
#define EDMP_B2S_SETTINGS_THR_COS_ANG                           0x218
#define EDMP_B2S_SETTINGS_THR_COS_ANG_SIZE                      4

/* b2s_settings_limit_inf
 *
 * Lower bound limit of the last sample's norm considered for b2s detection, in s32q24. Value corresponding to the squared norm value of 0.687g2 is: 11532816
 * Unit: s32q24
 */
#define EDMP_B2S_SETTINGS_LIMIT_INF                             0x21c
#define EDMP_B2S_SETTINGS_LIMIT_INF_SIZE                        4

/* b2s_settings_limit_sup
 *
 * Higher bound limit of the last sample's norm considered for b2s detection, in s32q24. Value corresponding to the squared norm value of 1.371g2 is: 23001616
 * Unit: s32q24
 */
#define EDMP_B2S_SETTINGS_LIMIT_SUP                             0x220
#define EDMP_B2S_SETTINGS_LIMIT_SUP_SIZE                        4

/* b2s_settings_rev_x_limit
 *
 * Condition of reverse bring2see on X axis value to ensure Rev-B2S when arm points down.
 * Unit: LSB, with 1 LBS = 1g / 2^12
 * Default: 3680 (corresponding to 0.8984g)
 */
#define EDMP_B2S_SETTINGS_REV_X_LIMIT                           0x224
#define EDMP_B2S_SETTINGS_REV_X_LIMIT_SIZE                      4

/* b2s_settings_sin_flat_angle
 *
 * Condition to detect the flat position and reject B2S interrupt at return to rest position.
 * Unit: LSB, with 1 LBS = 1g / 2^12
 * Default: 2048 (corresponding to 30Â°)
 */
#define EDMP_B2S_SETTINGS_SIN_FLAT_ANGLE                        0x228
#define EDMP_B2S_SETTINGS_SIN_FLAT_ANGLE_SIZE                   4

/* b2s_settings_timer_flat_reject
 *
 * A timer to disable flat rejection when age of the last no-flat B2S detection is over the timer.
 * Unit: sample number - ODR dependent
 * Default: 350 (corresponding to 7s at 50Hz)
 */
#define EDMP_B2S_SETTINGS_TIMER_FLAT_REJECT                     0x22c
#define EDMP_B2S_SETTINGS_TIMER_FLAT_REJECT_SIZE                4

/* b2s_settings_fast_motion_age_limit
 *
 * Time limit between last "Fast motion" and b2s position.
 * Unit: sample number - ODR dependent
 * Default: 20 (corresponding to 400ms at 50Hz)
 */
#define EDMP_B2S_SETTINGS_FAST_MOTION_AGE_LIMIT                 0x230
#define EDMP_B2S_SETTINGS_FAST_MOTION_AGE_LIMIT_SIZE            2

/* b2s_settings_fast_motion_time_limit
 *
 * Minimum time where the criterion is above the threshold  to be classified as "Fast motion".
 * Unit: sample number - ODR dependent
 * Default: 4 (corresponding to 80ms at 50Hz)
 */
#define EDMP_B2S_SETTINGS_FAST_MOTION_TIME_LIMIT                0x232
#define EDMP_B2S_SETTINGS_FAST_MOTION_TIME_LIMIT_SIZE           2

/* b2s_settings_age_limit
 *
 * Minimum time between 2 event b2s.
 * Unit: sample number - ODR dependent
 * Default: 50 (corresponding to 1s at 50Hz)
 */
#define EDMP_B2S_SETTINGS_AGE_LIMIT                             0x234
#define EDMP_B2S_SETTINGS_AGE_LIMIT_SIZE                        2

/* b2s_settings_rev_latency_th
 *
 * Condition of RevB2S should be maintained at least during RevB2sLatencyTh.
 * Unit: sample number - ODR dependent
 * Default: 25 (corresponding to 0.5s at 50Hz)
 */
#define EDMP_B2S_SETTINGS_REV_LATENCY_TH                        0x236
#define EDMP_B2S_SETTINGS_REV_LATENCY_TH_SIZE                   2

/* tap_min_jerk
 *
 * The minimal value of jerk to be considered as a tap candidate.
 * Unit: LSB with 1 LSB = 1g / 2^12 (of the jerk value)
 * Range: [0 - 16384]
 * Default: 4608 (equivalent to 1.125 g) 
 */
#define EDMP_TAP_MIN_JERK                                       0x1c4
#define EDMP_TAP_MIN_JERK_SIZE                                  2

/* tap_tmax
 *
 * Size of the analysis window to detect tap events (single, double or triple tap)
 * Unit: time in sample number
 * Range: [49 - 496]
 * Default : 198 (set for default ODR = 400 Hz, equivalent to 0.495 s)
 */
#define EDMP_TAP_TMAX                                           0x1c6
#define EDMP_TAP_TMAX_SIZE                                      2

/* tap_tmin
 *
 * Single tap window, sub-windows within Tmax to detect single-tap event.
 * Unit: time in sample number
 * Range: [24 - 184]
 * Default: 66 (set for default ODR = 400 Hz, equivalent to 0.165 s)
 */
#define EDMP_TAP_TMIN                                           0x1c8
#define EDMP_TAP_TMIN_SIZE                                      1

/* tap_max_peak_tol
 *
 * Maximum peak tolerance is the percentage of pulse amplitude to get the smudge threshold for rejection.
 * Unit: N/A
 * Range: [1 (12.5%) 2 (25.0%) 3 (37.5%) 4 (50.0 %)]
 * Default: 2
 */
#define EDMP_TAP_MAX_PEAK_TOL                                   0x1ca
#define EDMP_TAP_MAX_PEAK_TOL_SIZE                              1

/* tap_smudge_reject_thr
 *
 * max acceptable number of samples (jerk value) over tap_max_peak_tol the during the Tmin window. Over this value, Tap event is rejected
 * unit: time in number of samples
 * range: [13 - 92]
 * Default: 34 (set for default ODR = 400 Hz, equivalent to 0.085 s)
 */
#define EDMP_TAP_SMUDGE_REJECT_THR                              0x1c9
#define EDMP_TAP_SMUDGE_REJECT_THR_SIZE                         1

/* tap_tavg
 *
 * Energy measurement window size to determine the tap axis associated with the 1st tap.
 * Unit: time in sample number
 * Range: [1 ; 2 ; 4 ; 8]
 * Default: 8
 * 
 */
#define EDMP_TAP_TAVG                                           0x1cb
#define EDMP_TAP_TAVG_SIZE                                      1

/* tap_odr
 *
 * TAP execution ODR:
 * 0: 200Hz, 1: 400Hz, 2: 800Hz.
 * Unit: N/A
 * Range: [0 ; 1 ; 2]
 * Default: 1 (400Hz)
 * 
 */
#define EDMP_TAP_ODR                                            0x1cc
#define EDMP_TAP_ODR_SIZE                                       1

/* tap_max
 *
 * Maximal number of tap quantity detected to be valid.
 * Unit: N/A
 * Range: [1 (single) ; 2 (double) ; 3 (triple)]
 * Default: 2 (double tap)
 * 
 */
#define EDMP_TAP_MAX                                            0x1cd
#define EDMP_TAP_MAX_SIZE                                       1

/* tap_min
 *
 * Minimal number of tap quantity detected to be valid.
 * Unit: N/A
 * Range: [1 (single) ; 2 (double) ; 3 (triple)]
 * Default: 2 (double tap)
 * 
 */
#define EDMP_TAP_MIN                                            0x1ce
#define EDMP_TAP_MIN_SIZE                                       1

/* tap_num
 *
 * type of the last reported TAP event:
 * 0: no tap, 1: single tap, 2: double tap, 3:triple tap
 */
#define EDMP_TAP_NUM                                            0x6
#define EDMP_TAP_NUM_SIZE                                       1

/* tap_axis
 *
 * Indicate the axis of the tap in the device frame
 * 0: ax, 1: ay, 2: az
 */
#define EDMP_TAP_AXIS                                           0x7
#define EDMP_TAP_AXIS_SIZE                                      1

/* tap_dir
 *
 * Indicate the direction of the tap in the device frame
 * 0: positive, 1: negative
 */
#define EDMP_TAP_DIR                                            0x8
#define EDMP_TAP_DIR_SIZE                                       1

/* double_tap_timing
 *
 * In case of double tap, indicate the sample count between the two detected pulses. Double tap timing in seconds is double_tap_timing / TAP_ODR_Hz.
 */
#define EDMP_DOUBLE_TAP_TIMING                                  0x1f4
#define EDMP_DOUBLE_TAP_TIMING_SIZE                             2

/* triple_tap_timing
 *
 * In case of triple tap, indicate the sample count between the first and third detected pulses. Triple tap timing in seconds is triple_tap_timing / TAP_ODR_Hz.
 */
#define EDMP_TRIPLE_TAP_TIMING                                  0x1f6
#define EDMP_TRIPLE_TAP_TIMING_SIZE                             2

/* power_save_time
 *
 * Time of inactivity after which eDMP goes in power save mode.
 * Unit: time in sample number
 * Range: [0 - 4294967295] 
 * Default value: 6400
 */
#define EDMP_POWER_SAVE_TIME                                    0x3c
#define EDMP_POWER_SAVE_TIME_SIZE                               4

/* stc_results
 *
 * Results/status from self-test run
 * bit0: AX Self-test result (0:pass 1:fail)
 * bit1: AY Self-test result (0:pass 1:fail)
 * bit2: AZ Self-test result (0:pass 1:fail)
 * bit3: GX Self-test result (0:pass 1:fail)
 * bit4: GY Self-test result (0:pass 1:fail)
 * bit5: GZ Self-test result (0:pass 1:fail)
 * bit6~7: Self-test status (0:Done 1:InProgress)
 */
#define EDMP_STC_RESULTS                                        0x190
#define EDMP_STC_RESULTS_SIZE                                   4

/* stc_configParams
 *
 * Self-test input parameters
 * bit0: If set, enable self-test init, must be set when any of accel or gyro self-test enable bit is set (bits 2:1)
 * bit1: If set, enable accel self-test 
 * bit2: If set, enable gyro self-test
 * bit3~6: Unused
 * bit7~9: Averaging time used to perform self-test (0/1/2/3/4/5: 10/20/40/80/160/320 ms)
 * bit10~12: Tolerance between factory trim and accel self-test response (0/1/2/3/4/5/6/7: 5/10/15/20/25/30/40/50%)
 * bit13~15: Tolerance between factory trim and gyro self-test response (0/1/2/3/4/5/6/7: 5/10/15/20/25/30/40/50%)
 */
#define EDMP_STC_CONFIGPARAMS                                   0x188
#define EDMP_STC_CONFIGPARAMS_SIZE                              4

/* stc_debug_en
 *
 * Debug capability of self-test feature. Must be set to 0 at anytime when self-test is requested.
 */
#define EDMP_STC_DEBUG_EN                                       0x18c
#define EDMP_STC_DEBUG_EN_SIZE                                  4

/* vvd_tree_thresholds
 *
 * Structure member of the VVD model Decision Tree. It shall be loaded with VVD model provided by TDK based on TDK recommendation.
 */
#define EDMP_VVD_TREE_THRESHOLDS                                0x744
#define EDMP_VVD_TREE_THRESHOLDS_SIZE                           2

/* vvd_tree_featureids
 *
 * Structure member of the VVD model Decision Tree. It shall be loaded with VVD model provided by TDK based on TDK recommendation
 */
#define EDMP_VVD_TREE_FEATUREIDS                                0x7c2
#define EDMP_VVD_TREE_FEATUREIDS_SIZE                           1

/* vvd_tree_nextnoderight
 *
 * Structure member of the VVD model Decision Tree. It shall be loaded with VVD model provided by TDK based on TDK recommendation
 */
#define EDMP_VVD_TREE_NEXTNODERIGHT                             0x801
#define EDMP_VVD_TREE_NEXTNODERIGHT_SIZE                        1

/* vvd_tree_thresholdsshift
 *
 * Structure member of the VVD model Decision Tree. It shall be loaded with VVD model provided by TDK based on TDK recommendation
 */
#define EDMP_VVD_TREE_THRESHOLDSSHIFT                           0x840
#define EDMP_VVD_TREE_THRESHOLDSSHIFT_SIZE                      1

/* vvd_params_delay_high
 *
 * Number of additional successive Vocal Vibration positive decisions to wait for before raising a VVD event interrupt. Increasing this tuning parameter will also increase Vocal Vibration Detection latency. Depending on the keyword, values suggested for KWS application are 0 or 1. For Transparency mode or music pause application, values suggested are either 1 or 2.
 * Range: {0, 1, 2, 3}
 * Default: 1
 */
#define EDMP_VVD_PARAMS_DELAY_HIGH                              0x960
#define EDMP_VVD_PARAMS_DELAY_HIGH_SIZE                         4

/* vvd_params_delay_low
 *
 * Number of additional successive Vocal Vibration negative decisions to wait for after a VVD event before resetting VVD state to 0. This can be used to avoid VVD rebounds or for applications where a continuous VVD flag is needed.
 * Default: 0
 */
#define EDMP_VVD_PARAMS_DELAY_LOW                               0x964
#define EDMP_VVD_PARAMS_DELAY_LOW_SIZE                          4

/* vvd_params_sample_cnt
 *
 * Number of samples having an inference, i.e. on which features are computed.
 * Default: 31
 */
#define EDMP_VVD_PARAMS_SAMPLE_CNT                              0x968
#define EDMP_VVD_PARAMS_SAMPLE_CNT_SIZE                         4

/* vvd_params_best_axis
 *
 * Allows to project on X/Y/Z axis based on IMU orientation inside earbud according to following equation: bestAxisCoordinate = bestAxis[0] * x + bestAxis[1] * y + bestAxis[2] * z
 * Unit: s32q12
 * Default: {0, 0, 2^12} to achieve best speech SNR on axis perpendicular to TWS user's ear
 */
#define EDMP_VVD_PARAMS_BEST_AXIS                               0x96c
#define EDMP_VVD_PARAMS_BEST_AXIS_SIZE                          12

/* vvd_params_thresh
 *
 * Dynamic energy threshold to trigger VVD in Q8.
 * Unit: s32q8
 * Default: 1920
 */
#define EDMP_VVD_PARAMS_THRESH                                  0x978
#define EDMP_VVD_PARAMS_THRESH_SIZE                             4

/* vvd_dynamic_thresh
 *
 * Value to be applied to vvd_params_thresh configuration when VVD dynamic on-the-fly parametrization is requested in ISR1.
 */
#define EDMP_VVD_DYNAMIC_THRESH                                 0x34
#define EDMP_VVD_DYNAMIC_THRESH_SIZE                            4

#ifdef __cplusplus
}
#endif

#endif // __INV_IMU_EDMP_MEMMAP_H__
