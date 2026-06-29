/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#include "icm566xx/imu/inv_imu_edmp.h"
#include "icm566xx/imu/inv_imu_edmp_headgesture.h"
#include "imu/inv_imu_edmp_ram_hdgst_defs.h"
#include "imu/inv_imu_edmp_ram_headgesture_memmap.h"

int icm566xx_edmp_headgesture_init(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	fifo_sram_sleep_t fifo_sram_sleep;
	static uint8_t ram_img[] = {
#include "imu/edmp_ram_hdgst_image.h"
	};
	uint16_t start_addr[] = {EDMP_RAM_HDGST_PRGM_BASE, EDMP_RAM_HDGST_PRGM_BASE + 4,
				 EDMP_RAM_HDGST_PRGM_BASE + 8};
	/* Only 8 MSB of SP address is written to register */
	uint8_t stack_addr = (uint8_t)(EDMP_RAM_HDGST_STACK_END >> 8);

	/* Set Start address for 3 edmp IRQ handlers */
	status |= icm566xx_write_reg(s, EDMP_PRGRM_IRQ0_0, sizeof(start_addr),
				     (uint8_t *)&start_addr[0]);

	/* Set Stack pointer start address */
	status |= icm566xx_write_reg(s, EDMP_SP_START_ADDR, sizeof(stack_addr),
				     (uint8_t *)&stack_addr);

	/* Make sure SRAM is up and running */
	status |= icm566xx_read_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	fifo_sram_sleep.fifo_gsleep_shared_sram = 1;
	status |= icm566xx_write_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);

	/* Write SRAM image (code+data) */
	status |= icm566xx_write_sram(s, EDMP_RAM_HDGST_DATA_BASE, sizeof(ram_img), ram_img);

	return status;
}

int icm566xx_edmp_headgesture_get_parameters(inv_imu_device_t *s,
					     inv_imu_edmp_headgesture_parameters_t *p)
{
	int status = INV_IMU_OK;
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HEADGESTURE_MOUNTING_MATRIX,
					 (uint8_t *)p->mounting_matrix);
	return status;
}

int icm566xx_edmp_headgesture_set_parameters(inv_imu_device_t *s,
					     const inv_imu_edmp_headgesture_parameters_t *p)
{
	int status = INV_IMU_OK;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HEADGESTURE_MOUNTING_MATRIX,
					  (uint8_t *)p->mounting_matrix);
	return status;
}

int icm566xx_edmp_headgesture_enable(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	/* Make sure Head-Gesture is not already enabled to prevent messing up pointers */
	status |= icm566xx_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.ff_en) {
		return status;
	}

	/* Enable Head-Gesture */
	edmp_apex_en0.ff_en = INV_IMU_ENABLE;
	status |= icm566xx_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int icm566xx_edmp_headgesture_disable(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= icm566xx_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	/* Disable Head-Gesture */
	edmp_apex_en0.ff_en = INV_IMU_DISABLE;
	status |= icm566xx_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int icm566xx_edmp_headgesture_set_config_int(inv_imu_device_t *s,
					     inv_imu_edmp_headgesture_int_state_t *it)
{
	int status = INV_IMU_OK;
	int_apex_config0_t int_apex_config0 = {0};

	int_apex_config0.int_status_mask_pin_low_g_det = !it->INV_HEAD_GESTURE;
	status |= icm566xx_write_reg(s, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);

	return status;
}

int icm566xx_edmp_headgesture_get_int_status(inv_imu_device_t *s,
					     inv_imu_edmp_headgesture_int_state_t *it)
{
	int status = INV_IMU_OK;
	int_apex_status0_t int_apex_status0;

	/* Read APEX interrupt status */
	status |= icm566xx_read_reg(s, INT_APEX_STATUS0, 1, (uint8_t *)&int_apex_status0);
	it->INV_HEAD_GESTURE = int_apex_status0.int_status_low_g_det;
	return status;
}

int icm566xx_edmp_headgesture_get_output(inv_imu_device_t *s,
					 inv_imu_edmp_headgesture_outputs_t *headgesture_output)
{
	int status = INV_IMU_OK;
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HEADGESTURE_OUTPUT_STATE,
					 (uint8_t *)headgesture_output);
	return status;
}
