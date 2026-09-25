/*
 * Copyright (c) 2026 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup DriverIct ICT driver
 *  @brief    Low-level driver for ICT devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICT_DRIVER_H_
#define _INV_ICT_DRIVER_H_

#include "ict/inv_ict_defs.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Function pointer to read register(s).
 *  @param[in] context  Pointer to context.
 *  @param[in] reg      Register address to be read.
 *  @param[out] buf     Output data from the register.
 *  @param[in] len      Number of byte to be read.
 *  @return             0 on success, negative value on error.
 */
typedef int (*inv_ict_read_reg_t)(void *context, uint8_t reg, uint8_t *buf, uint32_t len);

/** @brief Function pointer to write register(s).
 *  @param[in] context  Pointer to context.
 *  @param[in] reg      Register address to be written.
 *  @param[in] buf      Input data to write.
 *  @param[in] len      Number of byte to be written.
 *  @return             0 on success, negative value on error.
 */
typedef int (*inv_ict_write_reg_t)(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

/** @brief Function pointer to sleep.
 *  @param[in] us  Time to sleep in microseconds.
 */
typedef void (*inv_ict_sleep_us_t)(uint32_t us);

/** @brief ICT serial interface */
typedef struct {
	void *              context;
	inv_ict_read_reg_t  read_reg; /**< Function pointer to read register(s). */
	inv_ict_write_reg_t write_reg; /**< Function pointer to write register(s). */
	inv_ict_sleep_us_t  sleep_us; /**< Function pointer to sleep. */
	uint32_t            max_read; /* Maximum length that can be read through serial interface */
	uint32_t            max_write; /* Maximum length that can be written through serial interface */
	uint8_t             is_first_transaction; /* Indicate if next serial access is the first one */
} inv_ict_serif_t;

/** @brief Possible ID for ICT devices */
typedef enum {
	ICT1531X,
	ICT25324,
	ICT25349,
} inv_ict_id_t;

/** @brief Macro to check if the device ID matches an ICT-153XX device */
#define IS_ICT153XX_DEVICE(id) (id == ICT1531X)

/** @brief Macro to check if the device ID matches an ICT-253XX device */
#define IS_ICT253XX_DEVICE(id) ((id == ICT25324) || (id == ICT25349))

/** @brief ICT driver states definition */
typedef struct {
	inv_ict_serif_t serif;
	inv_ict_id_t    id;
} inv_ict_device_t;

/** @brief Initialize ICT driver
 *  @param[in] s      Pointer to device.
 *  @param[in] serif  Handle to serial interface object.
 *  @return            0 on success, negative value on error.
 */
int inv_ict_init(inv_ict_device_t *s, const inv_ict_serif_t *serif);

/** @brief return ID of the device
 *  @param[in] s    Pointer to device.
 *  @param[out] id  Device ID.
 *  @return         0 on success, negative value on error.
 */
int inv_ict_get_id(inv_ict_device_t *s, inv_ict_id_t *id);

/** @brief Perform a soft reset of the device
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_ict_soft_reset(inv_ict_device_t *s);

/** @brief Report data ready status for sensor data
 *  @param[in] s       Pointer to device.
 *  @param[in] status  Data ready status bit (1: A new set of data is available, 0: No new set of data is available)
 *  @return            0 on success, negative value on error.
 */
int inv_ict_get_data_ready_status(inv_ict_device_t *s, int *status);

/** @brief Report FIFO Threshold status
 *  @param[in] s       Pointer to device.
 *  @param[in] status  Status bitmask (bit0: DRDY, bit1: FIFO Full, bit2: FIFO threshold)
 *  @return            0 on success, negative value on error.
 */
int inv_ict_get_status(inv_ict_device_t *s, int *status);

/** @brief Read sensor data
 *  @param[in] s              Pointer to device.
 *  @param[out] mag_data_lsb  Mag data.
 *  @param[out] temp_data_lsb Temperature data.
 *  @return                   0 on success, negative value on error.
 */
int inv_ict_poll_data(inv_ict_device_t *s, int16_t mag_data_lsb[3], int16_t *temp_data_lsb);

/** @brief Set the compass to different mode
 *  @param[in] s     Pointer to device.
 *  @param[in] mode  Mode to be set.
 *  @return          0 on success, negative value on error.
 */
int inv_ict_set_mode(inv_ict_device_t *s, inv_ict_mode_t mode);

/** @brief return current mode status
 *  @param[in] s      Pointer to device.
 *  @param[out] mode  Current mode.
 *  @return           0 on success, negative value on error.
 */
int inv_ict_get_mode(inv_ict_device_t *s, inv_ict_mode_t *mode);

/** @brief Set the low noise setting
 *  @param[in] s      Pointer to device.
 *  @param[in] ln_en  1: enable low noise, 0: disable low noise.
 *  @return           0 on success, negative value on error.
 */
int inv_ict_set_low_noise_setting(inv_ict_device_t *s, int ln_en);

/** @brief Set ODR for Pulse mode.
 *  @param[in] s    Pointer to device.
 *  @param[in] odr  Expected ODR.
 *  @return         0 on success, negative value on error.
 */
int inv_ict_set_odr(inv_ict_device_t *s, inv_ict_odr_t odr);

/** @brief Set ODR for Pulse mode with Low Noise Setting.
 *  @param[in] s    Pointer to device.
 *  @param[in] odr  Expected ODR in LN.
 *  @return         0 on success, negative value on error.
 */
int inv_ict_set_odr_ln(inv_ict_device_t *s, inv_ict_odr_ln_t odr);

/** @brief Convert ODR in us.
 *  @param[in] s        Pointer to device.
 *  @param[in] odr      ODR bitfield.
 *  @param[out] odr_us  ODR in us.
 *  @return             0 on success, negative value on error.
 */
int inv_ict_odr_to_us(inv_ict_device_t *s, const inv_ict_odr_t odr, uint32_t *odr_us);

/** @brief Convert ODR LNS in us.
 *  @param[in] s        Pointer to device.
 *  @param[in] odr_ln   ODR LNS bitfield.
 *  @param[out] odr_us  ODR in us.
 *  @return             0 on success, negative value on error.
 */
int inv_ict_odr_ln_to_us(inv_ict_device_t *s, const inv_ict_odr_ln_t odr_ln, uint32_t *odr_us);

/** @brief Set temperature mode.
 *         Must be called when the sensor is in Standby/Sleep mode.
 *  @param[in] s          Pointer to device.
 *  @param[in] temp_mode  Mode to apply to temperature sensor.
 *  @return               0 on success, negative value on error.
 */
int inv_ict_set_temperature_mode(inv_ict_device_t *s, inv_ict_chip_config_temp_sel_t temp_mode);

/** @brief Set FIFO configuration.
 *         Only available for ICT-253XX devices.
 *         Must be called when the sensor is in Standby/Sleep mode.
 *  @param[in] s       Pointer to device.
 *  @param[in] config  FIFO configuration.
 *  @return            0 on success, negative value on error.
 */
int inv_ict_set_fifo_config(inv_ict_device_t *s, inv_ict_fifo_config_t config);

/** @brief Flush FIFO.
 *         Only available for ICT-253XX devices.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_ict_flush_fifo(inv_ict_device_t *s);

/** @brief Get FIFO frame count.
 *         Only available for ICT-253XX devices.
 *  @param[in] s             Pointer to device.
 *  @param[out] frame_count  The number of frames in the FIFO.
 *  @return                  0 on success, negative value on error.
 */
int inv_ict_get_frame_count(inv_ict_device_t *s, uint8_t *frame_count);

/** @brief Get one FIFO frame data.
 *         Only available for ICT-253XX devices.
 *  @param[in] s          Pointer to device.
 *  @param[out] data      Pointer to a buffer (must be large enough to fit nb_frames frames).
 *  @param[in] nb_frames  Number of frames to be read.
 *  @return               0 on success, negative value on error.
 */
int inv_ict_get_fifo_frames(inv_ict_device_t *s, inv_ict_fifo_data_t *data,
                            const uint8_t nb_frames);

/** @brief Trigger magnetic reset mode
 * @param[in] s Pointer to device.
 * @return 0 in case of success, negative value on error
 */
int inv_ict_set_mrm(inv_ict_device_t *s);

/** @brief Lock protected registers
 * @param[in] s           Pointer to device.
 * @param[in] lock			0=unlock, 1=lock
 * @return 0 in case of success, negative value on error
 */
int inv_ict_global_lock(inv_ict_device_t *s, uint8_t lock);

/** @brief Perform self-test
 * @param[in] s           Pointer to device.
 * @param[out] st_status  Status of the self-test (see `INV_ICT_SELFTEST_*` defines).
 * @return 0 in case of success, negative value on error.
 * @note The return code doesn't indicate the status of the self-test. It indicates if the
 *       function completed its execution or not. The self-test status should be read in 
 *       `st_status` if the function returned 0.
 */
int inv_ict_selftest(inv_ict_device_t *s, inv_ict_selftest_status_t *st_status);

/** @brief Get sensor FSR
 * @param[in] s     Pointer to device.
 * @param[out] fsr  FSR of current device in uT.
 * @return 0 in case of success, negative value on error.
 */
int inv_ict_get_fsr_ut(inv_ict_device_t *s, uint32_t *fsr);

/** @brief Get sensor sensitivity
 * @param[in] s             Pointer to device.
 * @param[out] sensitivity  Sensitivity of sensor in nT/LSB.
 * @return 0 in case of success, negative value on error.
 */
int inv_ict_get_sensitivity_nt_per_lsb(inv_ict_device_t *s, uint32_t *sensitivity);

/** @brief Function to read register(s).
 *  @param[in] serif  Pointer to serial interface object.
 *  @param[in] reg    Register address to be read.
 *  @param[out] buf   Output data from the register.
 *  @param[in] len    Number of byte to be read.
 *  @return           0 on success, negative value on error.
 */
int inv_ict_read_reg(inv_ict_serif_t *s, uint8_t reg, uint8_t *buf, uint32_t len);

/** @brief Function to write register(s).
 *  @param[in] serif  Pointer to serial interface object.
 *  @param[in] reg    Register address to be written.
 *  @param[in] buf    Input data to write.
 *  @param[in] len    Number of byte to be written.
 *  @return           0 on success, negative value on error.
 */
int inv_ict_write_reg(inv_ict_serif_t *s, uint8_t reg, const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICT_DRIVER_H_ */

/** @} */
