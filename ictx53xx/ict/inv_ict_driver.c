/*
 * Copyright (c) 2026 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#include "ict/inv_ict_driver.h"

#include <string.h>

int inv_ict_init(inv_ict_device_t *s, const inv_ict_serif_t *serif)
{
	int          rc = INV_ICT_OK;
	inv_ict_id_t id;

	memset(s, 0, sizeof(*s));
	s->serif = *serif;

	/* 
	 * After device power-up, the user may receive NACK for the very first I2C transaction.
	 * The user should perform one retry on the very first I2C transaction if it receives a NACK.
	 *
	 * Set `is_first_transaction` to 1 to indicate to serial layer to retry first transaction if
	 * it gets NACKed.
	 */
	s->serif.is_first_transaction = 1;

	/* Initialize device ID */
	rc |= inv_ict_get_id(s, &id);

	if (rc)
		return rc;
	else
		s->id = id;

	/* Reset device */
	rc |= inv_ict_soft_reset(s);

	return rc;
}

int inv_ict_get_id(inv_ict_device_t *s, inv_ict_id_t *id)
{
	int     rc = INV_ICT_OK;
	uint8_t whoami;

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_CHIP_ID_REG, &whoami, 1);

	if (rc)
		return rc;

	if (whoami == ICT1531X_WHOAMI) {
		*id = ICT1531X;
	} else if (whoami == ICT253XX_WHOAMI) {
		uint8_t fsr_config;

		/* Read CONF_CALIB_APP_MODE to distinguish which ICT is used */
		rc |= inv_ict_read_reg(&s->serif, INV_ICT_CONF_CALIB_APP_MODE_REG, &fsr_config, 1);

		*id = (fsr_config == INV_ICT_CONF_CALIB_APP_MODE_REG_4_9MT) ? ICT25349 : ICT25324;
	} else {
		return INV_ICT_ERROR; /* Unknown whaomi */
	}

	return rc;
}

int inv_ict_soft_reset(inv_ict_device_t *s)
{
	int     rc = INV_ICT_OK;
	uint8_t data;

	/* Unlock protected register */
	rc |= inv_ict_global_lock(s, 0);

	/* Perform soft-reset */
	rc |= inv_ict_read_reg(&s->serif, INV_ICT_SEQUENCER_CTRL_REG, &data, 1);
	data &= ~INV_ICT_SEQUENCER_CTRL_REG_SOFT_RESET_MASK;
	data |= 1 << INV_ICT_SEQUENCER_CTRL_REG_SOFT_RESET_POS;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_SEQUENCER_CTRL_REG, &data, 1);

	/* Wait 5ms */
	s->serif.sleep_us(5000);

	/* Lock protected register */
	rc |= inv_ict_global_lock(s, 1);

	/* Run MRM procedure after SW reset */
	rc |= inv_ict_set_mrm(s);

	return rc;
}

int inv_ict_get_data_ready_status(inv_ict_device_t *s, int *status)
{
	int     rc = INV_ICT_OK;
	uint8_t data;

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_STATUS_REG, &data, 1);
	*status = data & INV_ICT_STATUS_REG_DATA_READY_MASK;

	return rc;
}

int inv_ict_get_status(inv_ict_device_t *s, int *status)
{
	int     rc = INV_ICT_OK;
	uint8_t data;

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_STATUS_REG, &data, 1);
	*status = (data & (INV_ICT_STATUS_REG_FIFO_THS_MASK | INV_ICT_STATUS_REG_FIFO_FULL_MASK |
	                   INV_ICT_STATUS_REG_DATA_READY_MASK));

	return rc;
}

int inv_ict_poll_data(inv_ict_device_t *s, int16_t mag_data_lsb[3], int16_t *temp_data_lsb)
{
	int     rc      = INV_ICT_OK;
	uint8_t data[8] = { 0 };

	/* Reading the last data register (mag_data_z_msb) clears the data_ready bit to '0' */
	rc |= inv_ict_read_reg(&s->serif, INV_ICT_TEMP_DATA_LSB, data, 8);
	*temp_data_lsb  = (((int16_t)data[1]) << 8) | data[0];
	mag_data_lsb[0] = (((int16_t)data[3]) << 8) | data[2];
	mag_data_lsb[1] = (((int16_t)data[5]) << 8) | data[4];
	mag_data_lsb[2] = (((int16_t)data[7]) << 8) | data[6];

	return rc;
}

int inv_ict_set_mode(inv_ict_device_t *s, inv_ict_mode_t mode)
{
	int            rc = INV_ICT_OK;
	uint8_t        data;
	inv_ict_mode_t cur_mode;
	int            retry = 10;

	rc |= inv_ict_get_mode(s, &cur_mode);

	/* Requested mode already matches with current mode */
	if (cur_mode == mode)
		return rc;

	/* New mode can be applied only if device is currently in SLEEP mode */
	if (cur_mode != INV_ICT_MODE_CTRL_REG_MODE_SLEEP && mode != INV_ICT_MODE_CTRL_REG_MODE_SLEEP)
		return INV_ICT_ERROR;

	/* Apply new mode */
	rc |= inv_ict_read_reg(&s->serif, INV_ICT_MODE_CTRL_REG, &data, 1);
	data &= ~INV_ICT_MODE_CTRL_REG_MODE_MASK;
	data |= mode;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_MODE_CTRL_REG, &data, 1);

	/* If SLEEP was selected, wait for it to be applied */
	if (mode == INV_ICT_MODE_CTRL_REG_MODE_SLEEP) {
		do {
			rc |= inv_ict_get_mode(s, &cur_mode);

			if (cur_mode == mode)
				break; /* Exit the loop */

			/* Decrement timeout */
			s->serif.sleep_us(1000);
			--retry;
			if (retry == 0)
				return INV_ICT_ERROR_TIMEOUT;

		} while (rc == 0);
	}

	return rc;
}

int inv_ict_get_mode(inv_ict_device_t *s, inv_ict_mode_t *cur_mode)
{
	int     rc = INV_ICT_OK;
	uint8_t data;

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_MODE_STATUS_REG, &data, 1);
	*cur_mode = (inv_ict_mode_t)(data & INV_ICT_MODE_STATUS_REG_STATUS_MASK);

	return rc;
}

int inv_ict_set_low_noise_setting(inv_ict_device_t *s, int ln_en)
{
	int            rc = INV_ICT_OK;
	uint8_t        data;
	inv_ict_mode_t cur_mode;

	if (IS_ICT153XX_DEVICE(s->id))
		return INV_ICT_ERROR; /* ICT-153XX doesn't support LNS */

	rc |= inv_ict_get_mode(s, &cur_mode);
	if (cur_mode != INV_ICT_MODE_CTRL_REG_MODE_SLEEP)
		return INV_ICT_ERROR; /* LNS shall only be applied in SLEEP mode */

	if (ln_en) {
		/* Enabling LNS requires to set ODR to 200 Hz */
		rc |= inv_ict_set_odr(s, INV_ICT_MODE_CTRL_REG_ODR_200_HZ);
	} else {
		/* Disabling LNS shall be done only if LNS ODR is set to 200 Hz */
		rc |= inv_ict_set_odr_ln(s, INV_ICT_DSP_CONFIG_REG_LNM_ODR_200_HZ);
	}

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_DSP_CONFIG_REG, &data, 1);
	data &= ~INV_ICT_DSP_CONFIG_REG_FILTER_ENABLE_MASK;
	data |= (uint8_t)ln_en << INV_ICT_DSP_CONFIG_REG_FILTER_ENABLE_POS;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_DSP_CONFIG_REG, &data, 1);

	return rc;
}

int inv_ict_set_odr(inv_ict_device_t *s, inv_ict_odr_t odr)
{
	int     rc = INV_ICT_OK;
	uint8_t data;

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_MODE_CTRL_REG, &data, 1);
	data &= ~INV_ICT_MODE_CTRL_REG_ODR_MASK;
	data |= (uint8_t)odr;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_MODE_CTRL_REG, &data, 1);

	return rc;
}

int inv_ict_set_odr_ln(inv_ict_device_t *s, inv_ict_odr_ln_t odr)
{
	int     rc = INV_ICT_OK;
	uint8_t data;

	if (IS_ICT153XX_DEVICE(s->id))
		return INV_ICT_ERROR; /* ICT-153XX doesn't support LNS */

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_DSP_CONFIG_REG, &data, 1);
	data &= ~INV_ICT_DSP_CONFIG_REG_LNM_ODR_MASK;
	data |= (uint8_t)odr;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_DSP_CONFIG_REG, &data, 1);

	return rc;
}

int inv_ict_odr_to_us(inv_ict_device_t *s, const inv_ict_odr_t odr, uint32_t *odr_us)
{
	int rc = INV_ICT_OK;

	switch (odr) {
	case INV_ICT_MODE_CTRL_REG_ODR_100_HZ:
		*odr_us = 10000;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_50_HZ:
		*odr_us = 20000;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_20_HZ:
		*odr_us = 50000;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_10_HZ:
		*odr_us = 100000;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_5_HZ:
		*odr_us = 200000;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_320_HZ:
		*odr_us = 3125;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_200_HZ:
		*odr_us = 5000;
		break;
	default:
		return INV_ICT_ERROR_BAD_ARG; /* Unknown ODR */
	}
	return rc;
}

int inv_ict_odr_ln_to_us(inv_ict_device_t *s, const inv_ict_odr_ln_t odr_ln, uint32_t *odr_us)
{
	int rc = INV_ICT_OK;

	switch (odr_ln) {
	case INV_ICT_DSP_CONFIG_REG_LNM_ODR_200_HZ:
		*odr_us = 5000;
		break;
	case INV_ICT_DSP_CONFIG_REG_LNM_ODR_100_HZ:
		*odr_us = 10000;
		break;
	case INV_ICT_DSP_CONFIG_REG_LNM_ODR_50_HZ:
		*odr_us = 20000;
		break;
	case INV_ICT_DSP_CONFIG_REG_LNM_ODR_25_HZ:
		*odr_us = 40000;
		break;
	case INV_ICT_DSP_CONFIG_REG_LNM_ODR_12_5_HZ:
		*odr_us = 80000;
		break;
	case INV_ICT_DSP_CONFIG_REG_LNM_ODR_6_25_HZ:
		*odr_us = 160000;
		break;
	default:
		return INV_ICT_ERROR_BAD_ARG; /* Unknown ODR */
	}
	return rc;
}

int inv_ict_set_temperature_mode(inv_ict_device_t *s, inv_ict_chip_config_temp_sel_t temp_mode)
{
	int            rc = INV_ICT_OK;
	uint8_t        data;
	inv_ict_mode_t cur_mode;

	/* Save current operational mode */
	rc |= inv_ict_get_mode(s, &cur_mode);
	if (cur_mode != INV_ICT_MODE_CTRL_REG_MODE_SLEEP)
		return INV_ICT_ERROR; /* Temperature configuration shall only be applied in SLEEP mode */

	/* Apply temperature mode */
	rc |= inv_ict_read_reg(&s->serif, INV_ICT_CHIP_CONFIG_REG, &data, 1);
	data &= ~INV_ICT_CHIP_CONFIG_REG_TEMP_SEL_MASK;
	data |= temp_mode;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_CHIP_CONFIG_REG, &data, 1);

	return rc;
}

int inv_ict_set_fifo_config(inv_ict_device_t *s, inv_ict_fifo_config_t config)
{
	int            rc   = INV_ICT_OK;
	uint8_t        data = 0;
	inv_ict_mode_t cur_mode;

	if (IS_ICT153XX_DEVICE(s->id))
		return INV_ICT_ERROR; /* ICT-153XX doesn't support FIFO */

	rc |= inv_ict_get_mode(s, &cur_mode);
	if (cur_mode != INV_ICT_MODE_CTRL_REG_MODE_SLEEP)
		return INV_ICT_ERROR; /* FIFO configuration shall only be applied in SLEEP mode */

	/* Apply FIFO configuration */
	data |= config.threshold << INV_ICT_FIFO_CONFIG_REG_THRESHOLD_POS;
	data |= config.header_enable << INV_ICT_FIFO_CONFIG_REG_HEADER_ENABLE_POS;
	data |= config.timestamp_enable << INV_ICT_FIFO_CONFIG_REG_TIMESTAMP_ENABLE_POS;
	data |= config.mode << INV_ICT_FIFO_CONFIG_REG_MODE_POS;
	data |= config.enable << INV_ICT_FIFO_CONFIG_REG_ENABLE_POS;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_FIFO_CONFIG_REG, &data, 1);

	return rc;
}

int inv_ict_flush_fifo(inv_ict_device_t *s)
{
	int     rc = INV_ICT_OK;
	uint8_t data;
	int     retry = 10;

	if (IS_ICT153XX_DEVICE(s->id))
		return INV_ICT_ERROR; /* ICT-153XX doesn't support FIFO */

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_MODE_CTRL_REG, &data, 1);
	data &= ~INV_ICT_MODE_CTRL_REG_FIFO_FLUSH_MASK;
	data |= 1 << INV_ICT_MODE_CTRL_REG_FIFO_FLUSH_POS;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_MODE_CTRL_REG, &data, 1);

	/* Wait for bit to be cleared, indicating flush is completed */
	do {
		rc |= inv_ict_read_reg(&s->serif, INV_ICT_MODE_CTRL_REG, &data, 1);

		if ((data & INV_ICT_MODE_CTRL_REG_FIFO_FLUSH_MASK) == 0)
			break; /* Exit the loop */

		/* Decrement timeout */
		s->serif.sleep_us(100);
		--retry;
		if (retry == 0)
			return INV_ICT_ERROR_TIMEOUT;

	} while (rc == 0);

	return rc;
}

int inv_ict_get_frame_count(inv_ict_device_t *s, uint8_t *frame_count)
{
	int     rc = INV_ICT_OK;
	uint8_t data;

	if (IS_ICT153XX_DEVICE(s->id))
		return INV_ICT_ERROR; /* ICT-153XX doesn't support FIFO */

	rc |= inv_ict_read_reg(&s->serif, INV_ICT_FIFO_COUNT_REG, &data, 1);
	*frame_count = (data & INV_ICT_FIFO_COUNT_REG_LEVEL_MASK) >> INV_ICT_FIFO_COUNT_REG_LEVEL_POS;

	return rc;
}

int inv_ict_get_fifo_frames(inv_ict_device_t *s, inv_ict_fifo_data_t *data, uint8_t nb_frames)
{
	int      rc  = INV_ICT_OK;
	uint32_t len = nb_frames * INV_ICT_FIFO_FRAME_SIZE_B;
	uint32_t max_burst =
	    (s->serif.max_read / INV_ICT_FIFO_FRAME_SIZE_B) * INV_ICT_FIFO_FRAME_SIZE_B;
	uint32_t offset = 0;

	if (IS_ICT153XX_DEVICE(s->id))
		return INV_ICT_ERROR; /* ICT-153XX doesn't support FIFO */

	if (max_burst == 0)
		return INV_ICT_ERROR_TRANSPORT; /* `max_read` must be at least 8 B for FIFO reads. */

	while (offset < len) {
		uint32_t burst_len = len - offset;

		if (burst_len > max_burst)
			burst_len = max_burst;

		rc |= inv_ict_read_reg(&s->serif, INV_ICT_FIFO_DATA_REG, ((uint8_t *)data) + offset,
		                       burst_len);

		offset += burst_len;
	}

	return rc;
}

int inv_ict_set_mrm(inv_ict_device_t *s)
{
	int rc = INV_ICT_OK;
	int retry = 10;

	rc |= inv_ict_set_mode(s, INV_ICT_MODE_CTRL_REG_MODE_MRM);
	s->serif.sleep_us(10);

	/* Force sleep mode at end of MRM operation for ICT153xx devices only */
	if (IS_ICT153XX_DEVICE(s->id)) {
		rc |= inv_ict_set_mode(s, INV_ICT_MODE_CTRL_REG_MODE_SLEEP);
	}

	/* Wait for mode to reset to SLEEP indicating MRM is completed */
	do {
		inv_ict_mode_t cur_mode;

		rc |= inv_ict_get_mode(s, &cur_mode);

		if (cur_mode == INV_ICT_MODE_CTRL_REG_MODE_SLEEP)
			break; /* Exit the loop */

		/* Decrement timeout */
		s->serif.sleep_us(100);
		--retry;
		if (retry == 0)
			return INV_ICT_ERROR_TIMEOUT;
	} while (rc == 0);

	return rc;
}

int inv_ict_global_lock(inv_ict_device_t *s, uint8_t lock)
{
	int     rc = INV_ICT_OK;
	uint8_t data;

	/* 
	 * Unlock the protected registers:
	 * conf_calib.sys_cfg @ 0x21 and sequencer_ctrl @ 0x7C
	 */
	if (!lock)
		data = 0xCA;
	else
		data = 0;

	rc |= inv_ict_write_reg(&s->serif, INV_ICT_GLOBAL_LOCK_REG, &data, 1);

	return rc;
}

static int get_stats_over_time_window(inv_ict_device_t *s, int16_t max[3], int16_t min[3])
{
	int     rc = INV_ICT_OK;
	int16_t data[3];
	int16_t temp;

	/* Init statistics */
	max[0] = INT16_MIN;
	max[1] = INT16_MIN;
	max[2] = INT16_MIN;
	min[0] = INT16_MAX;
	min[1] = INT16_MAX;
	min[2] = INT16_MAX;

	for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_SELFTEST; i++) {
		/* Go to single-shot mode */
		rc |= inv_ict_set_mode(s, INV_ICT_MODE_CTRL_REG_MODE_SINGLE_SHOT);
		s->serif.sleep_us(4000);

		/* Read ict253xx data */
		rc |= inv_ict_poll_data(s, data, &temp);

		for (int j = 0; j < 3; j++) {
			/* min */
			if (data[j] < min[j])
				min[j] = data[j];

			/* max*/
			if (data[j] > max[j])
				max[j] = data[j];
		}
	}

	return rc;
}

static int inv_ict_selftest_on_host(inv_ict_device_t *s, inv_ict_selftest_status_t *st_status)
{
	int rc = INV_ICT_OK;

	if (!IS_ICT153XX_DEVICE(s->id))
		return INV_ICT_ERROR; /* This method applies for ICT-153XX only */

	st_status->status = INV_ICT_SELFTEST_SUCCESS;

	s->serif.sleep_us(2000);

	rc |= inv_ict_set_mrm(s);

	s->serif.sleep_us(10);

	rc |= get_stats_over_time_window(s, st_status->max, st_status->min);

	/* Compute peak-to-peak */
	for (int i = 0; i < 3; i++) {
		st_status->pp[i] = st_status->max[i] - st_status->min[i];

		if (st_status->pp[i] < 1)
			st_status->status |= 1 << i;
	}

	return rc;
}

static int inv_ict_configure_idle_mode(inv_ict_device_t *s, inv_ict_conf_calib_idle_mode_t cfg)
{
	int     rc = INV_ICT_OK;
	uint8_t data;
	uint8_t conf_calib_addr;

	if (IS_ICT153XX_DEVICE(s->id))
		conf_calib_addr = INV_ICT_CONF_CALIB_REG_ICT153XX;
	else
		conf_calib_addr = INV_ICT_CONF_CALIB_REG_ICT253XX;

	/* Unlock protected register */
	rc |= inv_ict_global_lock(s, 0);

	/* Enable Test Mode */
	rc |= inv_ict_read_reg(&s->serif, INV_ICT_SEQUENCER_CTRL_REG, &data, 1);
	data &= ~INV_ICT_SEQUENCER_CTRL_REG_ATE_TEST_MODE_MASK;
	data |= 1 << INV_ICT_SEQUENCER_CTRL_REG_ATE_TEST_MODE_POS;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_SEQUENCER_CTRL_REG, &data, 1);
	s->serif.sleep_us(1000);

	/* Set Sleep/Standby mode */
	rc |= inv_ict_read_reg(&s->serif, conf_calib_addr, &data, 1);
	data &= ~INV_ICT_CONF_CALIB_REG_SLEEP_EN_MASK;
	data |= cfg;
	rc |= inv_ict_write_reg(&s->serif, conf_calib_addr, &data, 1);

	/* Disable Test Mode */
	rc |= inv_ict_read_reg(&s->serif, INV_ICT_SEQUENCER_CTRL_REG, &data, 1);
	data &= ~INV_ICT_SEQUENCER_CTRL_REG_ATE_TEST_MODE_MASK;
	data |= 0 << INV_ICT_SEQUENCER_CTRL_REG_ATE_TEST_MODE_POS;
	rc |= inv_ict_write_reg(&s->serif, INV_ICT_SEQUENCER_CTRL_REG, &data, 1);
	s->serif.sleep_us(10000);

	/* Lock protected register */
	rc |= inv_ict_global_lock(s, 1);

	return rc;
}

static int inv_ict_selftest_on_chip(inv_ict_device_t *s, inv_ict_selftest_status_t *st_status)
{
	int            rc = INV_ICT_OK;
	inv_ict_mode_t mode;

	if (IS_ICT153XX_DEVICE(s->id))
		return INV_ICT_ERROR; /* This method doesn't apply for ICT-153XX */

	/* Configure in STANDBY mode */
	rc |= inv_ict_configure_idle_mode(s, INV_ICT_CONF_CALIB_REG_STANDBY_MODE);

	rc |= inv_ict_set_mode(s, INV_ICT_MODE_CTRL_REG_MODE_SELFTEST);

	s->serif.sleep_us(10000);

	rc |= inv_ict_get_mode(s, &mode);

	if (mode != INV_ICT_MODE_CTRL_REG_MODE_SLEEP) {
		/* 
		 * Device should automatically go back to sleep mode.
		 * Raise an error if that isn't the case. 
		 */
		rc = INV_ICT_ERROR;
	} else {
		uint8_t data;

		/* Read Selftest_fail */
		rc |= inv_ict_read_reg(&s->serif, INV_ICT_MODE_STATUS_REG, &data, 1);
		st_status->status = (data & INV_ICT_MODE_STATUS_REG_SELFTEST_FAIL_MASK) ?
                                INV_ICT_SELFTEST_ERROR_FAIL :
                                INV_ICT_SELFTEST_SUCCESS;
	}

	/* Configure in SLEEP mode */
	rc |= inv_ict_configure_idle_mode(s, INV_ICT_CONF_CALIB_REG_SLEEP_MODE);

	return rc;
}

int inv_ict_selftest(inv_ict_device_t *s, inv_ict_selftest_status_t *st_status)
{
	int            rc = INV_ICT_OK;
	inv_ict_mode_t cur_mode;

	rc |= inv_ict_get_mode(s, &cur_mode);
	if (cur_mode != INV_ICT_MODE_CTRL_REG_MODE_SLEEP)
		return INV_ICT_ERROR; /* Self-test shall be triggered from SLEEP mode */

	if (IS_ICT153XX_DEVICE(s->id)) {
		rc |= inv_ict_selftest_on_host(s, st_status);
	} else if (IS_ICT253XX_DEVICE(s->id)) {
		rc |= inv_ict_selftest_on_chip(s, st_status);
	} else {
		rc = INV_ICT_ERROR; /* Unknown ID */
	}

	return rc;
}

int inv_ict_get_fsr_ut(inv_ict_device_t *s, uint32_t *fsr)
{
	int rc = INV_ICT_OK;

	switch (s->id) {
	case ICT1531X:
	case ICT25324:
		*fsr = INV_ICT_2_4MT_FSR;
		break;
	case ICT25349:
		*fsr = INV_ICT_4_9MT_FSR;
		break;
	default:
		return INV_ICT_ERROR; /* Unknown ID */
	}

	return rc;
}

int inv_ict_get_sensitivity_nt_per_lsb(inv_ict_device_t *s, uint32_t *sensitivity)
{
	int rc = INV_ICT_OK;

	switch (s->id) {
	case ICT1531X:
	case ICT25324:
		*sensitivity = INV_ICT_2_4MT_SENSITIVITY;
		break;
	case ICT25349:
		*sensitivity = INV_ICT_4_9MT_SENSITIVITY;
		break;
	default:
		return INV_ICT_ERROR; /* Unknown ID */
	}

	return rc;
}

int inv_ict_read_reg(inv_ict_serif_t *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	int rc = INV_ICT_OK;

	if (!serif)
		return INV_ICT_ERROR;

	if (len > serif->max_read)
		return INV_ICT_ERROR_BAD_ARG;

	rc = serif->read_reg(serif->context, reg, buf, len);

	/* 
	 * After device power-up, the user may receive NACK for the very first I2C transaction.
	 * The user should perform one retry on the very first I2C transaction if it receives a NACK.
	 */
	if (serif->is_first_transaction) {
		serif->is_first_transaction = 0;
		if (rc != 0)
			rc = serif->read_reg(serif->context, reg, buf, len);
	}

	if (rc != 0)
		return INV_ICT_ERROR_TRANSPORT;

	return 0;
}

int inv_ict_write_reg(inv_ict_serif_t *serif, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	int rc = INV_ICT_OK;

	if (!serif)
		return INV_ICT_ERROR;

	if (len > serif->max_write)
		return INV_ICT_ERROR_BAD_ARG;

	rc = serif->write_reg(serif->context, reg, buf, len);

	/* 
	 * After device power-up, the user may receive NACK for the very first I2C transaction.
	 * The user should perform one retry on the very first I2C transaction if it receives a NACK.
	 */
	if (serif->is_first_transaction) {
		serif->is_first_transaction = 0;
		if (rc != 0)
			rc = serif->write_reg(serif->context, reg, buf, len);
	}

	if (rc != 0)
		return INV_ICT_ERROR_TRANSPORT;

	return 0;
}
