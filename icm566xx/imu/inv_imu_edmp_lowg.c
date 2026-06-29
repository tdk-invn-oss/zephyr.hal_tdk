/*
 * Copyright (c) 2017 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#include "icm566xx/imu/inv_imu_edmp.h"
#include "icm566xx/imu/inv_imu_edmp_lowg.h"

int icm566xx_edmp_get_lowg_parameters(inv_imu_device_t *s, inv_imu_edmp_apex_parameters_t *p)
{
	int status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	/* Freefall */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH, (uint8_t *)&p->lowg_peak_th);
	status |=
		INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH_HYST, (uint8_t *)&p->lowg_peak_th_hyst);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_TIME_TH, (uint8_t *)&p->lowg_time_th);

	/* Power save */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_POWER_SAVE_TIME, (uint8_t *)&p->power_save_time);
	status |= icm566xx_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	p->power_save_en = edmp_apex_en1.power_save_en ? INV_IMU_ENABLE : INV_IMU_DISABLE;

	return status;
}

int icm566xx_edmp_set_lowg_parameters(inv_imu_device_t *s, const inv_imu_edmp_apex_parameters_t *p)
{
	int status = INV_IMU_OK;
	edmp_apex_enx_t cfg;

	/* Free Fall */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH, (uint8_t *)&p->lowg_peak_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH_HYST,
					  (uint8_t *)&p->lowg_peak_th_hyst);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_TIME_TH, (uint8_t *)&p->lowg_time_th);

	/* Power save */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_POWER_SAVE_TIME, (uint8_t *)&p->power_save_time);
	status |= icm566xx_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&cfg.edmp_apex_en1);
	cfg.edmp_apex_en1.power_save_en = p->power_save_en;
	status |= icm566xx_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&cfg.edmp_apex_en1);

	return status;
}

int icm566xx_edmp_get_config_int_lowg(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it)
{
	int status = INV_IMU_OK;
	int_apex_configx_t cfg;

	status |= icm566xx_read_reg(s, INT_APEX_CONFIG0, 1, (uint8_t *)&cfg);
	/* INT_APEX_CONFIG0 */
	it->INV_LOWG = !cfg.int_apex_config0.int_status_mask_pin_low_g_det;

	return status;
}

int icm566xx_edmp_set_config_int_lowg(inv_imu_device_t *s, const inv_imu_edmp_int_state_t *it)
{
	int status = INV_IMU_OK;
	int_apex_configx_t cfg = {0};

	/* INT_APEX_CONFIG0 */
	cfg.int_apex_config0.int_status_mask_pin_low_g_det = !it->INV_LOWG;
	status |= icm566xx_write_reg(s, INT_APEX_CONFIG0, 1, (uint8_t *)&cfg);

	return status;
}

int icm566xx_edmp_enable_lowg(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	/* Make sure freefall is not already enabled to prevent messing up pointers */
	status |= icm566xx_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.ff_en) {
		return status;
	}

	/* Enable freefall */
	edmp_apex_en0.ff_en = INV_IMU_ENABLE;
	status |= icm566xx_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int icm566xx_edmp_disable_lowg(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= icm566xx_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.ff_en = INV_IMU_DISABLE;
	status |= icm566xx_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int icm566xx_edmp_set_lowg_odr(inv_imu_device_t *s, uint16_t ff_odr)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ODR_FF, (uint8_t *)&ff_odr);

	return status;
}

int icm566xx_edmp_get_int_lowg_status(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it)
{
	int status = INV_IMU_OK;
	int_apex_statusx_t st;

	/* Read APEX interrupt status */
	status |= icm566xx_read_reg(s, INT_APEX_STATUS0, 1, (uint8_t *)&st);
	it->INV_LOWG = st.int_apex_status0.int_status_low_g_det;
	return status;
}
