/*
 *
 * Copyright (c) [2017] by InvenSense, Inc.
 * * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_SRAM.h"
#include "imu/inv_imu_edmp_defs.h"

static int dmp_getDecimationRate(inv_imu_device_t *s, uint16_t featureOdr, int8_t *decimationRate)
{
	uint16_t tmp = featureOdr;
	dmp_ext_sen_odr_cfg_apex_odr_t frequency;
	uint16_t sensorOdr = 0;
	inv_imu_edmp_get_frequency(s, &frequency);

	switch (frequency) {
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_25_HZ:
		sensorOdr = 25;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ:
		sensorOdr = 50;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ:
		sensorOdr = 100;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ:
		sensorOdr = 200;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ:
		sensorOdr = 400;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ:
		sensorOdr = 800;
		break;
	default:
		return INV_IMU_ERROR;
	}
	while (tmp < sensorOdr) {
		tmp += featureOdr;
		(*decimationRate)++;
	}
	return INV_IMU_OK;
}

int inv_imu_edmp_bump_init(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	static uint8_t ram_img[] = {
#include "edmp_ram_extended_features_image.h"
	};

	{
		/* Clear SRAM */
		uint8_t value = 0;
		for (uint32_t i = 0; i < (sizeof(ram_img)); i++) {
			status |=
				inv_imu_write_sram(s, (uint32_t)EDMP_RAM_PRGM_BASE + i, 1, &value);
		}
	}
	status |= inv_imu_write_sram(s, (uint32_t)EDMP_RAM_PRGM_BASE, sizeof(ram_img), ram_img);
	return status;
}

int inv_imu_edmp_bump_get_parameters(inv_imu_device_t *s,
				     inv_imu_edmp_bump_parameters_t *bump_params)
{
	int status = INV_IMU_OK;

	/* Bump */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_bump_config_bump_th_ratio_Q8,
					 (uint8_t *)&bump_params->bump_th_ratio_Q8);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_bump_config_change_th_ratio_Q8,
					 (uint8_t *)&bump_params->bump_change_th_ratio_Q8);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_bump_config_conf_time,
					 (uint8_t *)&bump_params->bump_conf_time);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_bump_config_debounce_time,
					 (uint8_t *)&bump_params->bump_debounce_time);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_bump_config_min_bump_nb,
					 (uint8_t *)&bump_params->bump_min_bump_nb);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_bump_config_bump_time,
					 (uint8_t *)&bump_params->bump_time);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_bump_config_change_th_offset_Q12,
					 (uint8_t *)&bump_params->bump_change_th_offset_Q12);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_bump_config_bump_th_offset_Q12,
					 (uint8_t *)&bump_params->bump_th_offset_Q12);

	return status;
}

int inv_imu_edmp_bump_set_parameters(inv_imu_device_t *s,
				     const inv_imu_edmp_bump_parameters_t *bump_params)
{
	int status = INV_IMU_OK;

	/* Bump */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_bump_th_ratio_Q8,
					  (uint8_t *)&bump_params->bump_th_ratio_Q8);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_change_th_ratio_Q8,
					  (uint8_t *)&bump_params->bump_change_th_ratio_Q8);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_conf_time,
					  (uint8_t *)&bump_params->bump_conf_time);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_debounce_time,
					  (uint8_t *)&bump_params->bump_debounce_time);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_min_bump_nb,
					  (uint8_t *)&bump_params->bump_min_bump_nb);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_bump_time,
					  (uint8_t *)&bump_params->bump_time);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_change_th_offset_Q12,
					  (uint8_t *)&bump_params->bump_change_th_offset_Q12);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_bump_th_offset_Q12,
					  (uint8_t *)&bump_params->bump_th_offset_Q12);

	return status;
}

int inv_imu_edmp_enable_bump(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;
	fifo_config0_t fifo_config0;

	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&fifo_config0);
	if (fifo_config0.fifo_depth > FIFO_CONFIG0_FIFO_DEPTH_APEX_WITH_RAM_FEATURE) {
		return INV_IMU_ERROR;
	}

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.reserved1 = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_disable_bump(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.reserved1 = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_set_bump_odr(inv_imu_device_t *s, uint16_t bump_odr)
{
	int status = INV_IMU_OK;
	int8_t decim_rate = 0;
	inv_imu_edmp_bump_parameters_t bump_params;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_config_odr_bump, (uint8_t *)&bump_odr);
	status |= dmp_getDecimationRate(s, bump_odr, &decim_rate);
	if (status) {
		return status;
	}

	bump_params.bump_decim_rate = decim_rate;
	bump_params.bump_decim_count = bump_params.bump_decim_rate;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_decim_rate,
					  (uint8_t *)&bump_params.bump_decim_rate);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_bump_decim_count,
					  (uint8_t *)&bump_params.bump_decim_count);

	return status;
}

int inv_imu_edmp_configure_sram(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	uint16_t start_addr[] = {EDMP_ROM_START_ADDR_IRQ0, EDMP_ROM_START_ADDR_IRQ1,
				 EDMP_ROM_START_ADDR_IRQ2};
	/* Only 8 MSB of SP address is written to register */
	uint8_t stack_addr = (uint8_t)(EDMP_ROM_RAM_SP_START_ADDR >> 8);

	/* Set Start address for 3 edmp IRQ handlers */
	status |= inv_imu_write_reg(s, EDMP_PRGRM_IRQ0_0, sizeof(start_addr),
				    (uint8_t *)&start_addr[0]);

	/* Set Stack pointer start address */
	status |= inv_imu_write_reg(s, EDMP_SP_START_ADDR, sizeof(stack_addr),
				    (uint8_t *)&stack_addr);

	return status;
}

int inv_imu_edmp_init_apex_sram(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	fifo_sram_sleep_t fifo_sram_sleep;
	uint8_t value;

	/* Same impl as inv_imu_adv_power_up_sram, duplicated here to prevent dependency */
	status |= inv_imu_read_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	fifo_sram_sleep.fifo_gsleep_shared_sram = 1;
	status |= inv_imu_write_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);

	/* Clear SRAM */
	value = 0;
	for (int i = 0; i < EDMP_RAM_SIZE; i++) {
		status |= inv_imu_write_sram(s, (uint32_t)EDMP_RAM_BASE + i, 1, &value);
	}

	/* Configure DMP address registers */
	status |= inv_imu_edmp_configure_sram(s);
	status |= inv_imu_edmp_init_apex_save_sram(s);
	return status;
}
