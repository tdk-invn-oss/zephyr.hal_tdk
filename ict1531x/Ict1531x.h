/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup DriverIct1531x Ict1531x driver
 *  @brief    Low-level driver for Ict1531x devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICT1531X_H_
#define _INV_ICT1531X_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Device identification */
#define ICT1531X_WHOAMI 0x45 /* Chip ID */

/* Register definitions */
/* Manufacturer ID. Reset value = 0xe7 */
#define ICT1531X_MANUF_ID_REG 0x00

/* Chip ID. Reset value = 0x45 */
#define ICT1531X_CHIP_ID_REG 0x01

/* Chip Config */
#define ICT1531X_CHIP_CONFIG_REG               0x03
#define ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_MASK 0x08
#define ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_POS  3

/* Mode Control */
#define ICT1531X_MODE_CTRL_REG           0x04
#define ICT1531X_MODE_CTRL_REG_ODR_MASK  0x70
#define ICT1531X_MODE_CTRL_REG_ODR_POS   4
#define ICT1531X_MODE_CTRL_REG_MODE_MASK 0x03
#define ICT1531X_MODE_CTRL_REG_MODE_POS  0

/* Mode Status */
#define ICT1531X_MODE_STATUS_REG             0x05
#define ICT1531X_MODE_STATUS_REG_STATUS_MASK 0x03
#define ICT1531X_MODE_STATUS_REG_STATUS_POS  0

/* Status Register */
#define ICT1531X_STATUS_REG                 0x06
#define ICT1531X_STATUS_REG_DATA_READY_MASK 0x01
#define ICT1531X_STATUS_REG_DATA_READY_POS  0

/* Frame Counter */
#define ICT1531X_FRAME_CNT_REG                    0x07
#define ICT1531X_FRAME_CNT_REG_FRAME_COUNTER_MASK 0x07
#define ICT1531X_FRAME_CNT_REG_FRAME_COUNTER_POS  0

/* Sensor data */
#define ICT1531X_TEMP_DATA_LSB  0x08 /* Temp data LSB */
#define ICT1531X_TEMP_DATA_MSB  0x09 /* Temp data MSB */
#define ICT1531X_MAG_DATA_X_LSB 0x0a /* Mag X axis LSB */
#define ICT1531X_MAG_DATA_X_MSB 0x0b /* Mag X axis MSB */
#define ICT1531X_MAG_DATA_Y_LSB 0x0c /* Mag Y axis LSB */
#define ICT1531X_MAG_DATA_Y_MSB 0x0d /* Mag Y axis MSB */
#define ICT1531X_MAG_DATA_Z_LSB 0x0e /* Mag Z axis LSB */
#define ICT1531X_MAG_DATA_Z_MSB 0x0f /* Mag Z axis MSB */

/* Configuration calibration */
#define ICT1531X_CONF_CALIB_REG 0x21

/* Sequencer Control */
#define ICT1531X_SEQUENCER_CTRL_REG                 0x7c
#define ICT1531X_SEQUENCER_CTRL_REG_SOFT_RESET_MASK 0x80
#define ICT1531X_SEQUENCER_CTRL_REG_SOFT_RESET_POS  7

/* Global Lock */
#define ICT1531X_GLOBAL_LOCK_REG 0x7f

/** @brief Error/Success codes for driver */
#define ICT1531X_OK              0 /**< Success */
#define ICT1531X_ERROR           -1 /**< Unspecified error */
#define ICT1531X_ERROR_TRANSPORT -3 /**< Error occurred at transport level */
#define ICT1531X_ERROR_TIMEOUT   -4 /**< Action did not complete in the expected time window */
#define ICT1531X_ERROR_BAD_ARG   -11 /**< Invalid argument provided */

/** @brief Error/Success codes for self-test */
#define ICT1531X_SELFTEST_SUCCESS        0
#define ICT1531X_SELFTEST_ERROR_PP_WIN_X (1 << 0)
#define ICT1531X_SELFTEST_ERROR_PP_WIN_Y (1 << 1)
#define ICT1531X_SELFTEST_ERROR_PP_WIN_Z (1 << 2)

/** @brief Operational modes */
typedef enum {
	ICT1531X_MODE_CTRL_REG_MODE_SLEEP       = 0,
	ICT1531X_MODE_CTRL_REG_MODE_PULSED      = 1,
	ICT1531X_MODE_CTRL_REG_MODE_SINGLE_SHOT = 2,
	ICT1531X_MODE_CTRL_REG_MODE_MRM         = 3
} inv_ict1531x_mode_t;

/** @brief Temperature sensor acquisition mode */
typedef enum {
	ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_FILTERED = 0 << ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_POS,
	ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_RAW      = 1 << ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_POS
} inv_ict1531x_chip_config_temp_sel_t;

/** @brief Function pointer to read register(s).
 *  @param[in] context  Pointer to context.
 *  @param[in] reg      Register address to be read.
 *  @param[out] buf     Output data from the register.
 *  @param[in] len      Number of byte to be read.
 *  @return             0 on success, negative value on error.
 */
typedef int (*inv_ict1531x_read_reg_t)(void *context, uint8_t reg, uint8_t *buf, uint32_t len);

/** @brief Function pointer to write register(s).
 *  @param[in] context  Pointer to context.
 *  @param[in] reg      Register address to be written.
 *  @param[in] buf      Input data to write.
 *  @param[in] len      Number of byte to be written.
 *  @return             0 on success, negative value on error.
 */
typedef int (*inv_ict1531x_write_reg_t)(void *context, uint8_t reg, const uint8_t *buf,
                                        uint32_t len);

/** @brief Ict1531x serial interface */
typedef struct inv_ict1531x_serif {
	void *                   context;
	inv_ict1531x_read_reg_t  read_reg;
	inv_ict1531x_write_reg_t write_reg;
	uint32_t                 max_read;
	uint32_t                 max_write;
	uint8_t is_first_transaction; /* indicates if the upcoming transaction will be the first one */
} inv_ict1531x_serif_t;

/** @brief ICT1531X driver states definition */
typedef struct inv_ict1531x {
	inv_ict1531x_serif_t serif;
} inv_ict1531x_t;

/** @brief Defines how many samples we collect in each window for self-test */
#define NUMBER_OF_SAMPLES_FOR_SELFTEST 100

/** @brief ICT1531X self-test status */
typedef struct inv_ict1531x_selftest_status {
	int     status; /* Overall self-test status */
	int16_t max[3];
	int16_t min[3];
	int32_t pp[3];
} inv_ict1531x_selftest_status_t;

/** @brief Hook for low-level system time() function to be implemented by upper layer
 *  @return monotonic timestamp in us
 *  @details
 *  When running self-tests, this function is used to measure a duration.
 *  It can also be used as a way of getting the current time.
 */
extern uint64_t inv_ict1531x_get_time_us(void);

/** @brief Function to read register(s).
 *  @param[in] serif  Pointer to serial interface object.
 *  @param[in] reg    Register address to be read.
 *  @param[out] buf   Output data from the register.
 *  @param[in] len    Number of byte to be read.
 *  @return           0 on success, negative value on error.
 */
int inv_ict1531x_read_reg(inv_ict1531x_serif_t *serif, uint8_t reg, uint8_t *buf, uint32_t len);
/** @brief Function to write register(s).
 *  @param[in] serif  Pointer to serial interface object.
 *  @param[in] reg    Register address to be written.
 *  @param[in] buf    Input data to write.
 *  @param[in] len    Number of byte to be written.
 *  @return           0 on success, negative value on error.
 */
int inv_ict1531x_write_reg(inv_ict1531x_serif_t *serif, uint8_t reg, const uint8_t *buf,
                           uint32_t len);

/** @brief Reset and initialize driver states
 *  @param[in] s      Pointer to device.
 *  @param[in] serif  Handle to serial interface object.
 */
void inv_ict1531x_reset_states(inv_ict1531x_t *s, const struct inv_ict1531x_serif *serif);

/** @brief Report data ready status for sensor data
 *  @param[in] s       Pointer to device.
 *  @param[in] status  Data ready status bit (1: A new set of data is available, 0: No new set of data is available)
 *  @return            0 on success, negative value on error.
 */
int inv_ict1531x_get_data_ready_status(inv_ict1531x_t *s, int *status);

/** @brief Read sensor data
 *  @param[in] s              Pointer to device.
 *  @param[out] mag_data_lsb  Mag data.
 *  @param[out] temp_data_lsb Temperature data.
 *  @return                   0 on success, negative value on error.
 */
int inv_ict1531x_poll_data(inv_ict1531x_t *s, int16_t mag_data_lsb[3], int16_t *temp_data_lsb);

/** @brief return WHOAMI value
 *  @param[in] s        Pointer to device.
 *  @param[out] whoami  WHOAMI of the device.
 *  @return             0 on success, negative value on error.
 */
int inv_ict1531x_get_whoami(inv_ict1531x_t *s, uint8_t *whoami);

/** @brief Perform a soft reset of the device
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_ict1531x_soft_reset(inv_ict1531x_t *s);

/** @brief Set the compass to different mode
 *  @param[in] s     Pointer to device.
 *  @param[in] mode  Mode to be set.
 *  @return          0 on success, negative value on error.
 */
int inv_ict1531x_set_mode(inv_ict1531x_t *s, inv_ict1531x_mode_t mode);

/** @brief return current mode status
 *  @param[in] s      Pointer to device.
 *  @param[out] mode  Current mode.
 *  @return           0 on success, negative value on error.
 */
int inv_ict1531x_get_mode(inv_ict1531x_t *s, inv_ict1531x_mode_t *mode);

/** @brief Set temperature mode.
 *         Must be called when the sensor is in Standby/Sleep mode.
 *  @param[in] s          Pointer to device.
 *  @param[in] temp_mode  Mode to apply to temperature sensor.
 *  @return               0 on success, negative value on error.
 */
int inv_ict1531x_set_temperature_mode(inv_ict1531x_t *                    s,
                                      inv_ict1531x_chip_config_temp_sel_t temp_mode);

/** @brief Trigger magnetic reset mode
 * @param[in] s Pointer to device.
 * @return 0 in case of success, negative value on error
 */
int inv_ict1531x_set_mrm(inv_ict1531x_t *s);

/** @brief Lock protected registers
 * @param[in] s           Pointer to device.
 * @param[in] lock			0=unlock, 1=lock
 * @return 0 in case of success, negative value on error
 */
int inv_ict1531x_global_lock(inv_ict1531x_t *s, uint8_t lock);

/** @brief Perform self-test
 * @param[in] s           Pointer to device.
 * @param[out] st_status  Status of the self-test (see `ICT1531X_SELFTEST_*` defines).
 * @return 0 in case of success, negative value on error.
 * @note The return code doesn't indicate the status of the self-test. It indicates if the
 *       function completed its execution or not. The self-test status should be read in 
 *       `st_status` if the function returned 0.
 */
int inv_ict1531x_selftest(inv_ict1531x_t *s, inv_ict1531x_selftest_status_t *st_status);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICT1531X_H_ */

/** @} */
