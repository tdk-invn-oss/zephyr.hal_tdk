/*
 * Copyright (c) 2026 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup DriverIctDefs ICT Definitions
 *  @brief    Registers and driver-related definitions and descriptions
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICT_DEFS_H_
#define _INV_ICT_DEFS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Register Map 
 */

/* Manufacturer ID */
#define INV_ICT_MANU_ID_REG 0x00

/* Chip ID */
#define INV_ICT_CHIP_ID_REG 0x01

/* Chip Config */
#define INV_ICT_CHIP_CONFIG_REG               0x03
#define INV_ICT_CHIP_CONFIG_REG_TEMP_SEL_MASK 0x08
#define INV_ICT_CHIP_CONFIG_REG_TEMP_SEL_POS  3

/* Mode Control */
#define INV_ICT_MODE_CTRL_REG                 0x04
#define INV_ICT_MODE_CTRL_REG_ODR_MASK        0x70
#define INV_ICT_MODE_CTRL_REG_ODR_POS         4
#define INV_ICT_MODE_CTRL_REG_FIFO_FLUSH_MASK 0x08
#define INV_ICT_MODE_CTRL_REG_FIFO_FLUSH_POS  3
#define INV_ICT_MODE_CTRL_REG_MODE_MASK       0x07
#define INV_ICT_MODE_CTRL_REG_MODE_POS        0

/* Mode Status */
#define INV_ICT_MODE_STATUS_REG                    0x05
#define INV_ICT_MODE_STATUS_REG_SELFTEST_FAIL_MASK 0x08
#define INV_ICT_MODE_STATUS_REG_SELFTEST_FAIL_POS  3
#define INV_ICT_MODE_STATUS_REG_STATUS_MASK        0x07
#define INV_ICT_MODE_STATUS_REG_STATUS_POS         0

/* Status */
#define INV_ICT_STATUS_REG                 0x06
#define INV_ICT_STATUS_REG_FIFO_THS_MASK   0x04
#define INV_ICT_STATUS_REG_FIFO_THS_POS    2
#define INV_ICT_STATUS_REG_FIFO_FULL_MASK  0x02
#define INV_ICT_STATUS_REG_FIFO_FULL_POS   1
#define INV_ICT_STATUS_REG_DATA_READY_MASK 0x01
#define INV_ICT_STATUS_REG_DATA_READY_POS  0

/* Frame Counter */
#define INV_ICT_FRAME_CNT_REG                    0x07
#define INV_ICT_FRAME_CNT_REG_FRAME_COUNTER_MASK 0x07
#define INV_ICT_FRAME_CNT_REG_FRAME_COUNTER_POS  0

/* Sensor data */
#define INV_ICT_TEMP_DATA_LSB  0x08 /* Temp data LSB */
#define INV_ICT_TEMP_DATA_MSB  0x09 /* Temp data MSB */
#define INV_ICT_MAG_DATA_X_LSB 0x0a /* Mag X axis LSB */
#define INV_ICT_MAG_DATA_X_MSB 0x0b /* Mag X axis MSB */
#define INV_ICT_MAG_DATA_Y_LSB 0x0c /* Mag Y axis LSB */
#define INV_ICT_MAG_DATA_Y_MSB 0x0d /* Mag Y axis MSB */
#define INV_ICT_MAG_DATA_Z_LSB 0x0e /* Mag Z axis LSB */
#define INV_ICT_MAG_DATA_Z_MSB 0x0f /* Mag Z axis MSB */

/* FIFO count */
#define INV_ICT_FIFO_COUNT_REG            0x10
#define INV_ICT_FIFO_COUNT_REG_LEVEL_MASK 0x1F
#define INV_ICT_FIFO_COUNT_REG_LEVEL_POS  0

/* FIFO data */
#define INV_ICT_FIFO_DATA_REG 0x11

/* FIFO config */
#define INV_ICT_FIFO_CONFIG_REG                       0x12
#define INV_ICT_FIFO_CONFIG_REG_THRESHOLD_MASK        0xF0
#define INV_ICT_FIFO_CONFIG_REG_THRESHOLD_POS         4
#define INV_ICT_FIFO_CONFIG_REG_HEADER_ENABLE_MASK    0x08
#define INV_ICT_FIFO_CONFIG_REG_HEADER_ENABLE_POS     3
#define INV_ICT_FIFO_CONFIG_REG_TIMESTAMP_ENABLE_MASK 0x04
#define INV_ICT_FIFO_CONFIG_REG_TIMESTAMP_ENABLE_POS  2
#define INV_ICT_FIFO_CONFIG_REG_MODE_MASK             0x02
#define INV_ICT_FIFO_CONFIG_REG_MODE_POS              1
#define INV_ICT_FIFO_CONFIG_REG_ENABLE_MASK           0x01
#define INV_ICT_FIFO_CONFIG_REG_ENABLE_POS            0

/* DSP config */
#define INV_ICT_DSP_CONFIG_REG                    0x13
#define INV_ICT_DSP_CONFIG_REG_FILTER_ENABLE_MASK 0x08
#define INV_ICT_DSP_CONFIG_REG_FILTER_ENABLE_POS  3
#define INV_ICT_DSP_CONFIG_REG_LNM_ODR_MASK       0x07
#define INV_ICT_DSP_CONFIG_REG_LNM_ODR_POS        0

/* LFO accuracy after trimming */
#define INV_ICT_LFO_FREQUENCY_ERROR_WS_REG 0x2B

/* Conf Calib Sys Cfg */
#define INV_ICT_CONF_CALIB_REG_ICT153XX      0x21
#define INV_ICT_CONF_CALIB_REG_ICT253XX      0x31
#define INV_ICT_CONF_CALIB_REG_SLEEP_EN_MASK 0x40
#define INV_ICT_CONF_CALIB_REG_SLEEP_EN_POS  6

/* Conf Calib App Mode */
#define INV_ICT_CONF_CALIB_APP_MODE_REG 0x71

/* Sequencer Control */
#define INV_ICT_SEQUENCER_CTRL_REG                    0x7c
#define INV_ICT_SEQUENCER_CTRL_REG_SOFT_RESET_MASK    0x80
#define INV_ICT_SEQUENCER_CTRL_REG_SOFT_RESET_POS     7
#define INV_ICT_SEQUENCER_CTRL_REG_ATE_TEST_MODE_MASK 0x01
#define INV_ICT_SEQUENCER_CTRL_REG_ATE_TEST_MODE_POS  0

/* Global Lock */
#define INV_ICT_GLOBAL_LOCK_REG 0x7f

/*
 * Definitions
 */

/** Chip ID */
#define ICT1531X_WHOAMI 0x45
#define ICT253XX_WHOAMI 0x46

/** Error/Success codes for driver */
#define INV_ICT_OK              0 /**< Success */
#define INV_ICT_ERROR           -1 /**< Unspecified error */
#define INV_ICT_ERROR_TRANSPORT -3 /**< Error occurred at transport level */
#define INV_ICT_ERROR_TIMEOUT   -4 /**< Action did not complete in the expected time window */
#define INV_ICT_ERROR_BAD_ARG   -11 /**< Invalid argument provided */

/** Operational modes */
typedef enum {
	INV_ICT_MODE_CTRL_REG_MODE_SLEEP       = 0 << INV_ICT_MODE_CTRL_REG_MODE_POS,
	INV_ICT_MODE_CTRL_REG_MODE_PULSED      = 1 << INV_ICT_MODE_CTRL_REG_MODE_POS,
	INV_ICT_MODE_CTRL_REG_MODE_SINGLE_SHOT = 2 << INV_ICT_MODE_CTRL_REG_MODE_POS,
	INV_ICT_MODE_CTRL_REG_MODE_MRM         = 3 << INV_ICT_MODE_CTRL_REG_MODE_POS,
	/* Only available for ICT-253XX devices */
	INV_ICT_MODE_CTRL_REG_MODE_SELFTEST = 4 << INV_ICT_MODE_CTRL_REG_MODE_POS,
} inv_ict_mode_t;

/** ODR for Pulse mode */
typedef enum {
	INV_ICT_MODE_CTRL_REG_ODR_100_HZ = 0 << INV_ICT_MODE_CTRL_REG_ODR_POS,
	INV_ICT_MODE_CTRL_REG_ODR_50_HZ  = 1 << INV_ICT_MODE_CTRL_REG_ODR_POS,
	INV_ICT_MODE_CTRL_REG_ODR_20_HZ  = 2 << INV_ICT_MODE_CTRL_REG_ODR_POS,
	INV_ICT_MODE_CTRL_REG_ODR_10_HZ  = 3 << INV_ICT_MODE_CTRL_REG_ODR_POS,
	INV_ICT_MODE_CTRL_REG_ODR_5_HZ   = 4 << INV_ICT_MODE_CTRL_REG_ODR_POS,
	INV_ICT_MODE_CTRL_REG_ODR_320_HZ = 5 << INV_ICT_MODE_CTRL_REG_ODR_POS,
	INV_ICT_MODE_CTRL_REG_ODR_200_HZ = 6 << INV_ICT_MODE_CTRL_REG_ODR_POS,
} inv_ict_odr_t;

/** ODR for Pulse mode in Low Noise Setting */
typedef enum {
	INV_ICT_DSP_CONFIG_REG_LNM_ODR_200_HZ  = 0 << INV_ICT_DSP_CONFIG_REG_LNM_ODR_POS,
	INV_ICT_DSP_CONFIG_REG_LNM_ODR_100_HZ  = 1 << INV_ICT_DSP_CONFIG_REG_LNM_ODR_POS,
	INV_ICT_DSP_CONFIG_REG_LNM_ODR_50_HZ   = 2 << INV_ICT_DSP_CONFIG_REG_LNM_ODR_POS,
	INV_ICT_DSP_CONFIG_REG_LNM_ODR_25_HZ   = 3 << INV_ICT_DSP_CONFIG_REG_LNM_ODR_POS,
	INV_ICT_DSP_CONFIG_REG_LNM_ODR_12_5_HZ = 4 << INV_ICT_DSP_CONFIG_REG_LNM_ODR_POS,
	INV_ICT_DSP_CONFIG_REG_LNM_ODR_6_25_HZ = 5 << INV_ICT_DSP_CONFIG_REG_LNM_ODR_POS,
} inv_ict_odr_ln_t;

/** Temperature sensor acquisition mode */
typedef enum {
	INV_ICT_CHIP_CONFIG_REG_TEMP_SEL_FILTERED = 0 << INV_ICT_CHIP_CONFIG_REG_TEMP_SEL_POS,
	INV_ICT_CHIP_CONFIG_REG_TEMP_SEL_RAW      = 1 << INV_ICT_CHIP_CONFIG_REG_TEMP_SEL_POS,
} inv_ict_chip_config_temp_sel_t;

/** FIFO configuration */
typedef struct {
	uint8_t threshold; /* Programmable level at which interrupt will be asserted */
	uint8_t header_enable; /**< Saturation indicator (first 3 LSb). 1: enabled, 0: disabled */
	uint8_t timestamp_enable; /**< Header content. 1: timestamp, 0: temperature */
	uint8_t mode; /**< FIFO mode. 0: Stop-on-Full, 1: Streaming */
	uint8_t enable; /**< Set to 1 to enable FIFO */
} inv_ict_fifo_config_t;

/** @brief One frame of FIFO header+data */
typedef struct {
	union {
		int16_t  temperature;
		uint16_t timestamp;
	} header;
	int16_t mag[3];
} inv_ict_fifo_data_t;

/** Size of the FIFO frames */
#define INV_ICT_FIFO_FRAME_SIZE_B 8

/** FSR defines */
typedef enum {
	INV_ICT_CONF_CALIB_APP_MODE_REG_4_9MT = 0,
	INV_ICT_CONF_CALIB_APP_MODE_REG_2_4MT = 1
} inv_ict_conf_calib_app_mode_t;

/** Mag FSR  */
#define INV_ICT_4_9MT_FSR 4900 /* uT */
#define INV_ICT_2_4MT_FSR 2400 /* uT */

/** Mag sensitivity */
#define INV_ICT_4_9MT_SENSITIVITY 155 /* nT/LSB */
#define INV_ICT_2_4MT_SENSITIVITY 75 /* nT/LSB */

/** Temperature */
#define INV_ICT_TEMPERATURE_OFFSET           25 /* 25 C */
#define INV_ICT_TEMPERATURE_SENSITIVITY_X100 625 /* 6.25 mC/LSB, multiplied by 100 to avoid float */

/** Timestamp resolution */
#define INV_ICT_FIFO_TIMESTAMP_RESOLUTION_NS_LSB 18750 /* 18.75 us/LSB */

/** Error/Success codes for self-test */
#define INV_ICT_SELFTEST_SUCCESS        0
#define INV_ICT_SELFTEST_ERROR_FAIL     -1 /* For ICT-253XX devices */
#define INV_ICT_SELFTEST_ERROR_PP_WIN_X (1 << 0) /* For ICT-153XX devices */
#define INV_ICT_SELFTEST_ERROR_PP_WIN_Y (1 << 1) /* For ICT-153XX devices */
#define INV_ICT_SELFTEST_ERROR_PP_WIN_Z (1 << 2) /* For ICT-153XX devices */

/** Defines how many samples we collect in each window for self-test 
 *  For ICT-153XX devices */
#define NUMBER_OF_SAMPLES_FOR_SELFTEST 100

/** ICT self-test status */
typedef struct {
	int status; /* Overall self-test status */
	/* Only available for ICT-153XX devices */
	int16_t max[3];
	int16_t min[3];
	int32_t pp[3];
} inv_ict_selftest_status_t;

/** Idle mode */
typedef enum {
	INV_ICT_CONF_CALIB_REG_SLEEP_MODE   = 0 << INV_ICT_CONF_CALIB_REG_SLEEP_EN_POS,
	INV_ICT_CONF_CALIB_REG_STANDBY_MODE = 1 << INV_ICT_CONF_CALIB_REG_SLEEP_EN_POS,
} inv_ict_conf_calib_idle_mode_t;
#ifdef __cplusplus
}
#endif

#endif /* _INV_ICT_DEFS_H_ */

/** @} */
