/*
 * Copyright (c) 2018 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup Transport Transport
 *  @brief    Abstraction layer to access device's registers
 *  @{
 */

/** @file  inv_imu_transport.h */

#ifndef _INV_IMU_TRANSPORT_H_
#define _INV_IMU_TRANSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "icm42x7x/imu/inv_imu_defs.h"

#include "Invn/InvError.h"

/* Available serial interface type. */
#define UI_I2C  0 /**< identifies I2C interface. */
#define UI_SPI4 1 /**< identifies 4-wire SPI interface. */
#define UI_SPI3 2 /**< identifies 3-wire SPI interface. */

/** @brief Serif type definition.
 *  @deprecated Kept for retro-compatibility. Replaced with `uint32_t` type
 *              in `struct inv_imu_serif` struct.
 */
typedef uint32_t SERIAL_IF_TYPE_t;

/** Serial interface definition */
typedef struct inv_imu_serif {
	void *context;
	int (*read_reg)(struct inv_imu_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len);
	int (*write_reg)(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *buf, uint32_t len);
	uint32_t max_read;
	uint32_t max_write;
	uint32_t serif_type;
} inv_imu_serif_t;

/** Transport interface definition. */
typedef struct inv_imu_transport {
	/** Serial interface object. 
	 *  @warning Must be the first object in this structure. 
	 */
	inv_imu_serif_t serif;

	/** Contains mirrored values of some IP registers. */
	struct register_cache {
		uint8_t pwr_mgmt0_reg;
#if INV_IMU_IS_GYRO_SUPPORTED
		uint8_t gyro_config0_reg;
#endif
		uint8_t accel_config0_reg;
		uint8_t tmst_config1_reg;
	} register_cache;

	/** Internal counter for MCLK requests. */
	uint8_t need_mclk_cnt;
} inv_imu_transport_t;

/** @brief Init cache variable.
 *  @param[in] t  Pointer to transport (as void * so it can be called from any module).
 *  @return       0 on success, negative value on error.
 */
int inv_imu_init_transport(void *t);

/** @brief Reads data from a register on IMU.
 *  @param[in] t     Pointer to transport (as void * so it can be called from any module).
 *  @param[in] reg   Register address to be read.
 *  @param[in] len   Number of byte to be read.
 *  @param[out] buf  Output data from the register.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_read_reg(void *t, uint32_t reg, uint32_t len, uint8_t *buf);

/** @brief Writes data to a register on IMU.
 *  @param[in] t    Pointer to transport (as void * so it can be called from any module).
 *  @param[in] reg  Register address to be written.
 *  @param[in] len  Number of byte to be written.
 *  @param[in] buf  Input data to write.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_write_reg(void *t, uint32_t reg, uint32_t len, const uint8_t *buf);

/** @brief Enable MCLK.
 *  @param[in] t  Pointer to transport (as void * so it can be called from any module).
 *  @return       0 on success, negative value on error.
 */
int inv_imu_switch_on_mclk(void *t);

/** @brief Disable MCLK.
 *  @param[in] t  Pointer to transport (as void * so it can be called from any module).
 *  @return       0 on success, negative value on error.
 */
int inv_imu_switch_off_mclk(void *t);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_TRANSPORT_H_ */

/** @} */
