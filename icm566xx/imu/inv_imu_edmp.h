/*
 * Copyright (c) 2017 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
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
#if (INV_IMU_WHOAMI == 0xD3) &&                                                                    \
	defined(CUSTOMER_EXAMPLE) /* Only use this for the 56622 example apps */
#include "imu/inv_imu_edmp_memmap_56622.h"
#else
#include "imu/inv_imu_edmp_memmap.h"
#endif
#include "imu/inv_imu_edmp_defs.h"

#include <stdint.h>
#include <string.h>

#define EDMP_ROM_START_ADDR_IRQ0 0x4000
#define EDMP_ROM_START_ADDR_IRQ1 0x4004
#define EDMP_ROM_START_ADDR_IRQ2 0x4008

#define EDMP_RAM_BASE 0x0000
#define EDMP_RAM_SIZE 0x05BC /* EDMP Data */

/** @brief Writes in EDMP SRAM
 *  @param[in] s     Pointer to device.
 *  @param[in] name  Name of the parameter.
 *  @param[in] val   Value to be written.
 *  @return          0 on success, negative value on error.
 */
#define INV_IMU_WRITE_EDMP_SRAM(s, name, val)                                                      \
	icm566xx_write_sram(s, (uint32_t)name, name##_SIZE, val)

/** @brief Reads in EDMP SRAM
 *  @param[in] s     Pointer to device.
 *  @param[in] name  Name of the parameter.
 *  @param[in] val   Value to be read.
 *  @return          0 on success, negative value on error.
 */
#define INV_IMU_READ_EDMP_SRAM(s, name, val) icm566xx_read_sram(s, (uint32_t)name, name##_SIZE, val)

/** @brief EDMP input interrupt lines definition */
typedef enum {
	INV_IMU_EDMP_INT0 = 0,
	INV_IMU_EDMP_INT1,
	INV_IMU_EDMP_INT2
} inv_imu_edmp_int_t;

/** Registers to enable APEX features. */
typedef struct {
	edmp_apex_en0_t edmp_apex_en0;
	edmp_apex_en1_t edmp_apex_en1;
} edmp_apex_enx_t;

/** @brief Clear the SRAM used by eDMP for running APEX algorithms.
 *  @param[in] s         Pointer to device.
 *  @return              0 on success, negative value on error.
 */
int icm566xx_edmp_clear_apex_sram(inv_imu_device_t *s);

/** @brief Configure EDMP Output Data Rate for APEX algorithms.
 *  @warning Accel frequency must be higher or equal to EDMP frequency.
 *  @param[in] s         Pointer to device.
 *  @param[in] frequency The requested frequency.
 *  @return              0 on success, negative value on error.
 */
int icm566xx_edmp_set_frequency(inv_imu_device_t *s,
				const dmp_ext_sen_odr_cfg_apex_odr_t frequency);

/** @brief Get EDMP Output Data Rate for APEX algorithms.
 *  @param[in] s          Pointer to device.
 *  @param[out] frequency The current frequency.
 *  @return               0 on success, negative value on error.
 */
int icm566xx_edmp_get_frequency(inv_imu_device_t *s, dmp_ext_sen_odr_cfg_apex_odr_t *frequency);

/** @brief Initialize EDMP APEX algorithms. This function should be called before
 *         calling any other function (expect for `icm566xx_edmp_set_frequency`).
 *  @warning This function will power-up the SRAM. For power consumption consideration,
 *           you can manually call `icm566xx_adv_power_down_sram` if you don't need to * preserve
 * SRAM content.
 *  @warning This function requires the EDMP ODR to be set before being called.
 *           Make sure to call `icm566xx_edmp_set_frequency` before this one.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_init_apex(inv_imu_device_t *s);

/** @brief Initialize EDMP APEX algorithms. This function should be called before
 *         calling any other function (expect for `icm566xx_edmp_set_frequency`).
 *  @warning This function requires the EDMP ODR to be set before being called.
 *           Make sure to call `icm566xx_edmp_set_frequency` before this one.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_init_apex_save_sram(inv_imu_device_t *s);

/** @brief  Enable EDMP.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_enable(inv_imu_device_t *s);

/** @brief  Disable EDMP.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_disable(inv_imu_device_t *s);

/** @brief  Mask requested interrupt sources for edmp interrupt line passed in parameter.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line number that should be configured.
 *  @param[in] int_mask     Interrupt sources to mask.
 *  @return                 0 on success, negative value on error.
 */
int icm566xx_edmp_mask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
			       uint8_t int_mask);

/** @brief  Unmask requested interrupt sources for edmp interrupt line passed in parameter.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line number that should be configured.
 *  @param[in] int_mask     Interrupt sources to unmask. *  @return                 0 on success,
 * negative value on error.
 */
int icm566xx_edmp_unmask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
				 uint8_t int_mask);

/** @brief  Setup EDMP to execute code in ROM.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_configure(inv_imu_device_t *s);

/** @brief Run EDMP using the on-demand mechanism.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line.
 *  @return                 0 on success, negative value on error.
 */
int icm566xx_edmp_run_ondemand(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb);

/** @brief Wait until EDMP idle bit is set (means EDMP execution is completed).
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int icm566xx_edmp_wait_for_idle(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_EDMP_H_ */

/** @} */
