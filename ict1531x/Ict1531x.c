/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#include "Ict1531x.h"

#include <string.h>

static void wait_us(uint32_t us)
{
	uint64_t tt1, tt2;

	tt1 = inv_ict1531x_get_time_us();
	while (1) {
		tt2 = inv_ict1531x_get_time_us();
		if (tt2 > (tt1 + us))
			break;
	}
}

int inv_ict1531x_read_reg(inv_ict1531x_serif_t *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	int rc = ICT1531X_OK;

	if (!serif)
		return ICT1531X_ERROR;

	if (len > serif->max_read)
		return ICT1531X_ERROR_BAD_ARG;

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
		return ICT1531X_ERROR_TRANSPORT;

	return 0;
}

int inv_ict1531x_write_reg(inv_ict1531x_serif_t *serif, uint8_t reg, const uint8_t *buf,
                           uint32_t len)
{
	int rc = ICT1531X_OK;

	if (!serif)
		return ICT1531X_ERROR;

	if (len > serif->max_write)
		return ICT1531X_ERROR_BAD_ARG;

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
		return ICT1531X_ERROR_TRANSPORT;

	return 0;
}

void inv_ict1531x_reset_states(inv_ict1531x_t *s, const struct inv_ict1531x_serif *serif)
{
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
}

int inv_ict1531x_get_data_ready_status(inv_ict1531x_t *s, int *status)
{
	int     rc = ICT1531X_OK;
	uint8_t data;

	rc |= inv_ict1531x_read_reg(&s->serif, ICT1531X_STATUS_REG, &data, 1);
	*status = data & ICT1531X_STATUS_REG_DATA_READY_MASK;

	return rc;
}

int inv_ict1531x_poll_data(inv_ict1531x_t *s, int16_t mag_data_lsb[3], int16_t *temp_data_lsb)
{
	int     rc      = ICT1531X_OK;
	uint8_t data[8] = { 0 };

	/* Reading the last data register (mag_data_z_msb) clears the data_ready bit to '0' */
	rc |= inv_ict1531x_read_reg(&s->serif, ICT1531X_TEMP_DATA_LSB, data, 8);
	*temp_data_lsb  = (((int16_t)data[1]) << 8) | data[0];
	mag_data_lsb[0] = (((int16_t)data[3]) << 8) | data[2];
	mag_data_lsb[1] = (((int16_t)data[5]) << 8) | data[4];
	mag_data_lsb[2] = (((int16_t)data[7]) << 8) | data[6];

	return rc;
}

int inv_ict1531x_get_whoami(inv_ict1531x_t *s, uint8_t *whoami)
{
	int rc = ICT1531X_OK;
	rc |= inv_ict1531x_read_reg(&s->serif, ICT1531X_CHIP_ID_REG, whoami, 1);
	return rc;
}

int inv_ict1531x_soft_reset(inv_ict1531x_t *s)
{
	int     rc = ICT1531X_OK;
	uint8_t data;

	/* Unlock protected register */
	rc |= inv_ict1531x_global_lock(s, 0);

	/* Perform soft-reset */
	rc |= inv_ict1531x_read_reg(&s->serif, ICT1531X_SEQUENCER_CTRL_REG, &data, 1);
	data &= ~ICT1531X_SEQUENCER_CTRL_REG_SOFT_RESET_MASK;
	data |= 1 << ICT1531X_SEQUENCER_CTRL_REG_SOFT_RESET_POS;
	rc |= inv_ict1531x_write_reg(&s->serif, ICT1531X_SEQUENCER_CTRL_REG, &data, 1);

	/* Lock protected register */
	rc |= inv_ict1531x_global_lock(s, 1);

	/* Run MRM procedure after SW reset */
	rc |= inv_ict1531x_set_mrm(s);

	return rc;
}

int inv_ict1531x_set_mode(inv_ict1531x_t *s, inv_ict1531x_mode_t mode)
{
	int                 rc = ICT1531X_OK;
	uint8_t             data;
	inv_ict1531x_mode_t cur_mode;

	rc |= inv_ict1531x_get_mode(s, &cur_mode);

	/* Requested mode already matches with current mode */
	if (cur_mode == mode)
		return rc;

	/* 
	 * If current mode is different from STANDBY, it needs to transition to STANDBY before 
	 * setting the new mode 
	 */
	if (cur_mode != ICT1531X_MODE_CTRL_REG_MODE_SLEEP) {
		uint8_t data;

		/* Set STANDBY mode */
		rc |= inv_ict1531x_read_reg(&s->serif, ICT1531X_MODE_CTRL_REG, &data, 1);
		data &= ~ICT1531X_MODE_CTRL_REG_MODE_MASK;
		data |= ICT1531X_MODE_CTRL_REG_MODE_SLEEP;
		rc |= inv_ict1531x_write_reg(&s->serif, ICT1531X_MODE_CTRL_REG, &data, 1);

		/* Wait for STANDBY to be applied */
		do {
			rc |= inv_ict1531x_get_mode(s, &cur_mode);
		} while ((rc == 0) && (cur_mode != ICT1531X_MODE_CTRL_REG_MODE_SLEEP));
	}

	/* if expected mode is STANDBY, nothing else to do */
	if (mode == ICT1531X_MODE_CTRL_REG_MODE_SLEEP)
		return rc;

	/* Apply new mode */
	rc |= inv_ict1531x_read_reg(&s->serif, ICT1531X_MODE_CTRL_REG, &data, 1);
	data &= ~ICT1531X_MODE_CTRL_REG_MODE_MASK;
	data |= mode;
	rc |= inv_ict1531x_write_reg(&s->serif, ICT1531X_MODE_CTRL_REG, &data, 1);

	return rc;
}

int inv_ict1531x_get_mode(inv_ict1531x_t *s, inv_ict1531x_mode_t *cur_mode)
{
	int     rc = ICT1531X_OK;
	uint8_t data;

	rc |= inv_ict1531x_read_reg(&s->serif, ICT1531X_MODE_STATUS_REG, &data, 1);
	*cur_mode = (inv_ict1531x_mode_t)(data & ICT1531X_MODE_CTRL_REG_MODE_MASK);

	return rc;
}

int inv_ict1531x_set_temperature_mode(inv_ict1531x_t *                    s,
                                      inv_ict1531x_chip_config_temp_sel_t temp_mode)
{
	int                 rc = ICT1531X_OK;
	uint8_t             data;
	inv_ict1531x_mode_t cur_mode;

	/* Save current operational mode */
	rc |= inv_ict1531x_get_mode(s, &cur_mode);

	/* The user can select the raw temperature output only in Sleep Mode. */
	rc |= inv_ict1531x_set_mode(s, ICT1531X_MODE_CTRL_REG_MODE_SLEEP);

	/* Apply temperature mode */
	rc |= inv_ict1531x_read_reg(&s->serif, ICT1531X_MODE_STATUS_REG, &data, 1);
	data &= ~ICT1531X_CHIP_CONFIG_REG_TEMP_SEL_MASK;
	data |= temp_mode;
	rc |= inv_ict1531x_write_reg(&s->serif, ICT1531X_CHIP_CONFIG_REG, &data, 1);

	/* Reset initial operational mode */
	rc |= inv_ict1531x_set_mode(s, cur_mode);

	return rc;
}

int inv_ict1531x_set_mrm(inv_ict1531x_t *s)
{
	int                 rc = ICT1531X_OK;
	inv_ict1531x_mode_t cur_mode;

	/* Save current operational mode */
	rc |= inv_ict1531x_get_mode(s, &cur_mode);

	/* Set MRM mode */
	rc |= inv_ict1531x_set_mode(s, ICT1531X_MODE_CTRL_REG_MODE_MRM);
	wait_us(10);

	/* Reset initial operational mode */
	rc |= inv_ict1531x_set_mode(s, cur_mode);

	return rc;
}

int inv_ict1531x_global_lock(inv_ict1531x_t *s, uint8_t lock)
{
	int     rc = ICT1531X_OK;
	uint8_t data;

	/* 
	 * Unlock the protected registers:
	 * conf_calib.sys_cfg @ 0x21 and sequencer_ctrl @ 0x7C
	 */
	if (!lock)
		data = 0xCA;
	else
		data = 0;

	rc |= inv_ict1531x_write_reg(&s->serif, ICT1531X_GLOBAL_LOCK_REG, &data, 1);

	return rc;
}

static int get_stats_over_time_window(inv_ict1531x_t *s, int16_t max[3], int16_t min[3])
{
	int     rc = ICT1531X_OK;
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
		rc |= inv_ict1531x_set_mode(s, ICT1531X_MODE_CTRL_REG_MODE_SINGLE_SHOT);
		wait_us(4000);

		/* Read ict1531x data */
		rc |= inv_ict1531x_poll_data(s, data, &temp);

		/* Go back to sleep mode */
		rc |= inv_ict1531x_set_mode(s, ICT1531X_MODE_CTRL_REG_MODE_SLEEP);

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

int inv_ict1531x_selftest(inv_ict1531x_t *s, inv_ict1531x_selftest_status_t *st_status)
{
	int rc = ICT1531X_OK;

	st_status->status = ICT1531X_SELFTEST_SUCCESS;

	wait_us(2000);

	rc |= inv_ict1531x_set_mrm(s);

	wait_us(10);

	rc |= get_stats_over_time_window(s, st_status->max, st_status->min);

	/* Compute peak-to-peak */
	for (int i = 0; i < 3; i++) {
		st_status->pp[i] = st_status->max[i] - st_status->min[i];

		if (st_status->pp[i] < 1)
			st_status->status |= 1 << i;
	}

	return rc;
}
