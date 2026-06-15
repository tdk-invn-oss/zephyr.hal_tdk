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

#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_driver_tc.h"

#define TREF 20.0

float gyro_tc_coefficient[3] = {-14.0, -14.0, -4.0};
float accel_tc_coefficient[3] = {-19.0, -15.0, -16.0};

/* The range of ODRs for the gyro_temp field */
int16_t gyro_temp_odr_values[] = {3200, 1600, 1600, 800};

/* The range of ODRs for the accel_temp field */
int16_t accel_temp_odr_values[] = {800, 800, 400, 400, 200, 200, 100, 100};

/* The range of LN accel/gyro ODRs */
int16_t accel_gyro_odr_values[] = {6400, 6400, 6400, 6400, 3200, 1600, 800,
				   400,  200,  100,  50,   25,   12};

/*
 * The user offset field {gyro/accel}_{x/y/z}_offuser need to be set to 0x0000.
 * This only needs to be done at startup or after a soft reset.
 */
int inv_imu_init_tc(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	uint16_t zero16 = 0x0000;

	status |=
		inv_imu_write_reg(s, IPREG_SYS1_REG_2, 2, (uint8_t *)&zero16); /* gyro_x_offuser */
	status |=
		inv_imu_write_reg(s, IPREG_SYS1_REG_4, 2, (uint8_t *)&zero16); /* gyro_y_offuser */
	status |= inv_imu_write_reg(s, IPREG_SYS1_REG_6, 2,
				    (uint8_t *)&zero16); /* gyro_z_offuser	 */
	status |= inv_imu_write_reg(s, IPREG_SYS2_REG_12, 2,
				    (uint8_t *)&zero16); /* accel_x_offuser */
	status |= inv_imu_write_reg(s, IPREG_SYS2_REG_14, 2,
				    (uint8_t *)&zero16); /* accel_y_offuser */
	status |= inv_imu_write_reg(s, IPREG_SYS2_REG_16, 2,
				    (uint8_t *)&zero16); /* accel_z_offuser */

	return status;
}

/*
 * log2 except that 0 is returned for log2(0)
 */
static int16_t log2_tc(int16_t value)
{
	int16_t counter = -1;

	/* Treat 0 as a special value and return 0 instead of -1 */
	if (value == 0) {
		return 0;
	}
	while (value > 0) {
		counter++;
		value >>= 1;
	}
	return counter;
}

/*
 * The tmp_lpf_cfg field is defined as follows:
 *		Temperature sensor. Low-Pass filter BW selection (LNM)
 *			0: LPF bypass --Note: this setting leads to 1/2 Temp Data Rate
 *			1: 1/4 Temp Data Rate
 *			2: 1/6 Temp Data Rate
 *			3: 1/10 Temp Data Rate
 *			4: 1/20 Temp Data Rate
 *			5: 1/50 Temp Data Rate
 *			6,7: 1/100 Temp Data Rate
 * The calculated value N that is passed into this routine corresponds to the
 * denominators of the values above.
 * This routine finds the value, 0-7, that has the denominator that is closest
 * to N.
 */
static uint8_t calculate_lpf_cfg(int16_t N)
{
	/* These values are the midpoints of the tmp_lpf_cfg ranges */
	uint8_t tmp_lpf_cfg_range[] = {
		3,  5,  8,
		15, 35, 75}; /*Changed 1st value to 3 in accordance with note on setting 0*/
	uint8_t i;

	for (i = 0; i < sizeof(tmp_lpf_cfg_range); i++) {
		if (N < tmp_lpf_cfg_range[i]) {
			return i;
		}
	}
	return i; /* Return the maximum */
}

/*
 * Calculate new values for the fields tmp_lpf_cfg and tmp_dec_cfg
 * based on the max and min ODRs of the senors that are enabled in
 * LN mode.
 */
static int update_tc_configuration(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	pwr_mgmt0_t pwr_mgmt0;
	accel_config0_t accel_config0;
	gyro_config0_t gyro_config0;
	ipreg_sys2_reg_117_t sys2_reg_117;

	uint8_t max_odr_idx = 0x0C; /* 12.5 Hz	- Minimum LN ODR */
	uint8_t min_odr_idx = 0x03; /* 6400 Hz	- Maximum LN ODR */
	int16_t max_odr, min_odr;
	int16_t accel_temp_odr, gyro_temp_odr;
	int16_t tmp_dec_cfg, N;
	int8_t tmp_lpf_cfg;
	ipreg_sys1_reg_151_internal_t sys1_reg_151;
	ipreg_sys2_reg_108_internal_t sys2_reg_108;

	/* Get the sensor mode LN/LP/Off for accel and gyro */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);

	/* Get the max and min ODR of the sensors enabled in LN mode */
	if (pwr_mgmt0.accel_mode == PWR_MGMT0_ACCEL_MODE_LN) {
		status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
		if (accel_config0.accel_odr < max_odr_idx) {
			max_odr_idx = accel_config0.accel_odr;
		}
		if (accel_config0.accel_odr > min_odr_idx) {
			min_odr_idx = accel_config0.accel_odr;
		}
	}
	if (pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_LN) {
		status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
		if (gyro_config0.gyro_odr < max_odr_idx) {
			max_odr_idx = gyro_config0.gyro_odr;
		}
		if (gyro_config0.gyro_odr > min_odr_idx) {
			min_odr_idx = gyro_config0.gyro_odr;
		}
	}
	min_odr = accel_gyro_odr_values[min_odr_idx];
	max_odr = accel_gyro_odr_values[max_odr_idx];

	/* If the min and max weren't changed, no sensors are in LN mode, so return */
	if (min_odr > max_odr) {
		return status;
	}

	/* Get the temperature ODR values for accel and gyro */
	status |= inv_imu_read_reg(s, IPREG_SYS1_REG_151, 1, (uint8_t *)&sys1_reg_151);
	status |= inv_imu_read_reg(s, IPREG_SYS2_REG_108, 1, (uint8_t *)&sys2_reg_108);
	accel_temp_odr = accel_temp_odr_values[sys2_reg_108.accel_temp_odr];
	gyro_temp_odr = gyro_temp_odr_values[sys1_reg_151.gyro_temp_odr];

	/* If the gyro is in LN mode, use the gyro as the reference */
	if (pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_LN) {
		tmp_dec_cfg = log2_tc(gyro_temp_odr / max_odr);
		N = (gyro_temp_odr / (1 << tmp_dec_cfg)) / (min_odr / 2);
	}
	/* Otherwise, if the accel is in LN mode, use it instead */
	else if (pwr_mgmt0.accel_mode == PWR_MGMT0_ACCEL_MODE_LN) {
		tmp_dec_cfg = log2_tc(accel_temp_odr / max_odr);
		N = (accel_temp_odr / (1 << tmp_dec_cfg)) / (min_odr / 2);
	} else {
		return status; /* Neither sensor is in LN mode, so return */
	}

	tmp_lpf_cfg = calculate_lpf_cfg(N);

	/* Store the calculated values */
	status |= inv_imu_read_reg(s, IPREG_SYS2_REG_117, 1, (uint8_t *)&sys2_reg_117);
	sys2_reg_117.tmp_dec_cfg = tmp_dec_cfg;
	sys2_reg_117.tmp_lpf_cfg = tmp_lpf_cfg;
	status |= inv_imu_write_reg(s, IPREG_SYS2_REG_117, 1, (uint8_t *)&sys2_reg_117);

	return status;
}

/*
 *********************************************************************************
 * The Temperature Compensation configuration depends upon which sensors are       *
 * enabled and what their ODRs are. As such, the configuration needs to be updated *
 * anytime there is a change to the sensor state or to their ODRs. These routines  *
 * are used in place of the basic routines for setting accel/gyro modes and ODRs.  *
 **********************************************************************************
 */

/*
 * Update the accel mode and recalculate the temperature compensation configuration
 */
int inv_imu_set_tc_accel_mode(inv_imu_device_t *s, pwr_mgmt0_accel_mode_t accel_mode)
{
	int status = INV_IMU_OK;

	status |= inv_imu_set_accel_mode(s, accel_mode);

	status |= update_tc_configuration(s);
	return status;
}

/*
 * Update the gyro mode and recalculate the temperature compensation configuration
 */
int inv_imu_set_tc_gyro_mode(inv_imu_device_t *s, pwr_mgmt0_gyro_mode_t gyro_mode)
{
	int status = INV_IMU_OK;

	status |= inv_imu_set_gyro_mode(s, gyro_mode);

	status |= update_tc_configuration(s);
	return status;
}

/*
 * Update the accel frequency and recalculate the temperature compensation configuration
 */
int inv_imu_set_tc_accel_frequency(inv_imu_device_t *s, accel_config0_accel_odr_t frequency)
{
	int status = INV_IMU_OK;

	status |= inv_imu_set_accel_frequency(s, frequency);

	status |= update_tc_configuration(s);
	return status;
}

/*
 * Update the gyro frequency and recalculate the temperature compensation configuration
 */
int inv_imu_set_tc_gyro_frequency(inv_imu_device_t *s, gyro_config0_gyro_odr_t frequency)
{
	int status = INV_IMU_OK;

	status |= inv_imu_set_gyro_frequency(s, frequency);

	status |= update_tc_configuration(s);
	return status;
}

/*
 * Read the sensor data from the registers and apply the temperature compensation
 * The compensation value is calculated using the following equation:
 *				G(t) = 1 + C * deltaT / 2^17.
 * In order to keep the calculation in fixed point, multiply by 2^16 (1 << 16).
 */
int inv_imu_get_tc_register_data(inv_imu_device_t *s, inv_imu_float_sensor_data_t *fl_data)
{
	int status = INV_IMU_OK;
	float current_temp;
	float delta_temp;
	float gain_compensation;
	inv_imu_sensor_data_t data;

	status |= inv_imu_get_register_data(s, &data);
	fl_data->temp_data = data.temp_data;

	/* Determine the current temperature based on the raw register data */
	current_temp = 25.0 + ((float)data.temp_data / 128);

	/* Find the difference between the current temperature and the reference temperature */
	delta_temp = current_temp - TREF;

	/* If there is valid accel data */
	if (data.accel_data[0] != INVALID_SENSOR_DATA ||
	    data.accel_data[1] != INVALID_SENSOR_DATA ||
	    data.accel_data[2] != INVALID_SENSOR_DATA) {
		for (int i = 0; i < 3; i++) {
			/* Determine the gain compensation based on the application note */
			gain_compensation = 1 + (accel_tc_coefficient[i] * delta_temp) / (1 << 17);
			fl_data->accel_data[i] = (float)data.accel_data[i] * gain_compensation;
		}
	}

	/* If there is valid gyro data */
	if (data.gyro_data[0] != INVALID_SENSOR_DATA || data.gyro_data[1] != INVALID_SENSOR_DATA ||
	    data.gyro_data[2] != INVALID_SENSOR_DATA) {
		for (int i = 0; i < 3; i++) {
			gain_compensation = 1 + (gyro_tc_coefficient[i] * delta_temp) / (1 << 17);
			fl_data->gyro_data[i] = (float)data.gyro_data[i] * gain_compensation;
		}
	}
	return status;
}

/*
******************************************************************************
* The following code needs to be implemented for parts that have hardware     *
* temperature compensation enabled in the trim. This code is used to disable  *
* the HW temperature compensation.                                             *
******************************************************************************
*/
#define NUM_ACCEL_TC_COEF 18
#define NUM_GYRO_TC_COEF  12

#define START_ACCEL_TC_GAIN 0xa557
#define START_GYRO_TC_GAIN  0xa480

/*
 * Get trimmed values for gyro_x_mems_offtc{a,b}_{hfs,lfs}.
 */
static void read_gyr_off_values(inv_imu_device_t *s, int index, int16_t *gyr_off_vals)
{
	uint16_t gyr_off_addrs[3] = {0xa44d, 0xa457, 0xa461};
	uint8_t gyr_off_temp[7];

	inv_imu_read_reg(s, gyr_off_addrs[index], sizeof(gyr_off_temp), (uint8_t *)gyr_off_temp);
	gyr_off_vals[0] = (gyr_off_temp[0] >> 2) | ((gyr_off_temp[1] & 0x7f) << 6);
	gyr_off_vals[1] =
		(gyr_off_temp[1] >> 7) | (gyr_off_temp[2] << 1) | (gyr_off_temp[3] & 0x0f) << 9;
	gyr_off_vals[2] =
		(gyr_off_temp[3] >> 4) | (gyr_off_temp[4] << 4) | (gyr_off_temp[5] & 0x01) << 12;
	gyr_off_vals[3] = (gyr_off_temp[5] >> 1) | (gyr_off_temp[6] & 0x3f) << 7;

	for (int j = 0; j < 4; j++) {
		gyr_off_vals[j] &= 0x1FFF;      /* This is only a 13 bit value */
		if (gyr_off_vals[j] & 0x1000) { /* If this should be negative */
			gyr_off_vals[j] |= 0xE000;
		}
	}
}

/*
 * Write compensated values for gyro_x_mems_offtc{a,b}_{hfs,lfs}.
 */
static void write_gyr_off_values(inv_imu_device_t *s, int index, int16_t *gyr_off_vals)
{
	uint16_t gyr_off_addrs[3] = {0xa44d, 0xa457, 0xa461}; /**/
	uint8_t gyr_off_temp[7];

	for (int j = 0; j < 4; j++) {
		gyr_off_vals[j] &= 0x1FFF; /* These are only 13 bit values */
	}

	inv_imu_read_reg(s, gyr_off_addrs[index], 1, gyr_off_temp);
	gyr_off_temp[0] = (gyr_off_vals[0] << 2) | (gyr_off_temp[0] & 0x03);
	gyr_off_temp[1] = (gyr_off_vals[0] >> 6) | (gyr_off_vals[1] << 7);
	gyr_off_temp[2] = (gyr_off_vals[1] >> 1);
	gyr_off_temp[3] = (gyr_off_vals[1] >> 9) | (gyr_off_vals[2] << 4);
	gyr_off_temp[4] = (gyr_off_vals[2] >> 4);
	gyr_off_temp[5] = (gyr_off_vals[2] >> 12) | (gyr_off_vals[3] << 1);
	gyr_off_temp[6] = (gyr_off_vals[3] >> 7);

	inv_imu_write_reg(s, gyr_off_addrs[index], sizeof(gyr_off_temp), (uint8_t *)gyr_off_temp);
}

/*
 * Get trimmed values for accel_x_mems_offtc{a,b}_{hfs,hfs32,lfs}
 */
static void read_acc_off_values(inv_imu_device_t *s, int index, int16_t *acc_off_vals)
{
	uint16_t acc_off_addrs[3] = {0xa53f, 0xa547, 0xa54f};
	uint8_t acc_off_temp[8];

	inv_imu_read_reg(s, acc_off_addrs[index], sizeof(acc_off_temp), (uint8_t *)acc_off_temp);
	acc_off_vals[0] = acc_off_temp[0] | ((acc_off_temp[1] & 0x03) << 8);
	acc_off_vals[1] = (acc_off_temp[1] >> 2) | ((acc_off_temp[2] & 0x0f) << 6);
	acc_off_vals[2] = (acc_off_temp[2] >> 4) | ((acc_off_temp[3] & 0x3f) << 4);

	acc_off_vals[3] = acc_off_temp[4] | ((acc_off_temp[5] & 0x03) << 8);
	acc_off_vals[4] = (acc_off_temp[5] >> 2) | ((acc_off_temp[6] & 0x0f) << 6);
	acc_off_vals[5] = (acc_off_temp[6] >> 4) | ((acc_off_temp[7] & 0x3f) << 4);

	for (int j = 0; j < 6; j++) {
		acc_off_vals[j] &= 0x3FF;      /* This is only a 10 bit value */
		if (acc_off_vals[j] & 0x200) { /* If this should be negative */
			acc_off_vals[j] |= 0xFC00;
		}
	}
}

/*
 * Write compensated values for accel_x_mems_offtc{a,b}_{hfs,hfs32,lfs}.
 */
static void write_acc_off_values(inv_imu_device_t *s, int index, int16_t *acc_off_vals)
{
	uint16_t acc_off_addrs[3] = {0xa53f, 0xa547, 0xa54f}; /**/
	uint8_t acc_off_temp[8];

	for (int j = 0; j < 6; j++) {
		acc_off_vals[j] &= 0x3FF; /* These are only 10 bit values */
	}

	acc_off_temp[0] = acc_off_vals[0];
	acc_off_temp[1] = (acc_off_vals[0] >> 8) | (acc_off_vals[1] << 2);
	acc_off_temp[2] = (acc_off_vals[1] >> 6) | (acc_off_vals[2] << 4);
	acc_off_temp[3] = (acc_off_vals[2] >> 4);

	acc_off_temp[4] = acc_off_vals[3];
	acc_off_temp[5] = (acc_off_vals[3] >> 8) | (acc_off_vals[4] << 2);
	acc_off_temp[6] = (acc_off_vals[4] >> 6) | (acc_off_vals[5] << 4);
	acc_off_temp[7] = (acc_off_vals[5] >> 4);

	inv_imu_write_reg(s, acc_off_addrs[index], sizeof(acc_off_temp), (uint8_t *)acc_off_temp);
}

/*
 * Adjust the temperature compensation offset values.
 * The formula for calculating the new values is: *                          offsetTCnew =
 *offsetTC-gainTC*(offset/gain)
 *
 * Where:
 * offsetTC is the pre-correction value in: *
 *gyro_{x,y,z}_mems_offtc{a,b}_{hfs,lfs}/2^11 accel_{x,y,z}_mems_offtc{a,b}_{hfs32,hfs,lfs}/2^15
 * gainTC is the pre-zeroing value in: *
 *gyro_{x,y,z}_gaintc{a,b}_{hfs,lfs}/2^17 accel_{x,y,z}_gaintc{a,b}_{hfs32,hfs,lfs}/2^17 offset is
 *the value stored in: *			gyro_{x,y,z}_mems_off_{hfs,lfs}/2^7
 *			accel_{x,y,z}_mems_off_hfs/2^13
 * gain is the value stored in:
 *       	gyro_{x,y,z}_gain/2^11
 *			accel_{x,y,z}_gain_hfs/2^11
 *
 */
static void adjust_tc_offset(inv_imu_device_t *s)
{
	uint16_t gyr_gain_addrs[3] = {0xa480, 0xa484, 0xa488};
	uint16_t acc_gain_addrs[3] = {0xa557, 0xa55d, 0xa563};
	uint16_t gyr_mems_off_addrs[3] = {0xa415, 0xa41d, 0xa425};
	uint16_t acc_mems_off_addrs[3] = {0xa500, 0xa502, 0xa504};
	uint16_t gyr_mems_gain_addrs[3] = {0xa468, 0xa470, 0xa478};
	uint16_t acc_mems_gain_addrs[3] = {0xa506, 0xa508, 0xa50a};
	int16_t gyr_off_vals[4];
	int16_t acc_off_vals[6];
	uint8_t gyr_mem_off_val[4];
	int8_t acc_mem_off_val[2];
	int8_t acc_gain_vals[6];
	int8_t gyr_gain_vals[4];
	gyr_tc_t gyr_offs_tc[3];
	acc_tc_t acc_offs_tc[3];
	gyr_tc_t gyr_gain_tc[3];
	acc_tc_t acc_gain_tc[3];
	sensor_tc_t gyr_mems_off_tc[3];
	sensor_tc_t acc_mems_off_tc[3];
	uint8_t reg_value12[2];
	int16_t reg_value16;
	float gyr_mems_gain_tc[3];
	float acc_mems_gain_tc[3];

	for (int i = 0; i < 3; i++) {
		read_gyr_off_values(s, i, gyr_off_vals);
		gyr_offs_tc[i].a_hfs = (float)gyr_off_vals[0] / (float)(1 << 11);
		gyr_offs_tc[i].a_lfs = (float)gyr_off_vals[1] / (float)(1 << 11);
		gyr_offs_tc[i].b_hfs = (float)gyr_off_vals[2] / (float)(1 << 11);
		gyr_offs_tc[i].b_lfs = (float)gyr_off_vals[3] / (float)(1 << 11);

		read_acc_off_values(s, i, acc_off_vals);
		acc_offs_tc[i].a_hfs = (float)acc_off_vals[0] / (float)(1 << 15);
		acc_offs_tc[i].a_hfs32 = (float)acc_off_vals[1] / (float)(1 << 15);
		acc_offs_tc[i].a_lfs = (float)acc_off_vals[2] / (float)(1 << 15);
		acc_offs_tc[i].b_hfs = (float)acc_off_vals[3] / (float)(1 << 15);
		acc_offs_tc[i].b_hfs32 = (float)acc_off_vals[4] / (float)(1 << 15);
		acc_offs_tc[i].b_lfs = (float)acc_off_vals[5] / (float)(1 << 15);

		inv_imu_read_reg(s, gyr_gain_addrs[i], sizeof(gyr_gain_vals),
				 (uint8_t *)gyr_gain_vals);
		gyr_gain_tc[i].a_hfs = (float)(gyr_gain_vals[0]) / (float)(1 << 17);
		gyr_gain_tc[i].a_lfs = (float)(gyr_gain_vals[1]) / (float)(1 << 17);
		gyr_gain_tc[i].b_hfs = (float)(gyr_gain_vals[2]) / (float)(1 << 17);
		gyr_gain_tc[i].b_lfs = (float)(gyr_gain_vals[3]) / (float)(1 << 17);

		inv_imu_read_reg(s, acc_gain_addrs[i], sizeof(acc_gain_vals),
				 (uint8_t *)acc_gain_vals);
		acc_gain_tc[i].a_hfs = (float)(acc_gain_vals[0]) / (float)(1 << 17);
		acc_gain_tc[i].a_hfs32 = (float)(acc_gain_vals[1]) / (float)(1 << 17);
		acc_gain_tc[i].a_lfs = (float)(acc_gain_vals[2]) / (float)(1 << 17);
		acc_gain_tc[i].b_hfs = (float)(acc_gain_vals[3]) / (float)(1 << 17);
		acc_gain_tc[i].b_hfs32 = (float)(acc_gain_vals[4]) / (float)(1 << 17);
		acc_gain_tc[i].b_lfs = (float)(acc_gain_vals[5]) / (float)(1 << 17);

		inv_imu_read_reg(s, gyr_mems_off_addrs[i], 4, (uint8_t *)gyr_mem_off_val);
		reg_value16 = (int16_t)gyr_mem_off_val[0] + (int16_t)(gyr_mem_off_val[1] << 8);
		gyr_mems_off_tc[i].hfs = (float)(reg_value16) / (float)(1 << 7);
		reg_value16 = (int16_t)gyr_mem_off_val[2] + (int16_t)(gyr_mem_off_val[3] << 8);
		gyr_mems_off_tc[i].lfs = (float)(reg_value16) / (float)(1 << 7);

		inv_imu_read_reg(s, acc_mems_off_addrs[i], 2, (uint8_t *)acc_mem_off_val);
		reg_value16 = (int16_t)acc_mem_off_val[0] + (int16_t)(acc_mem_off_val[1] << 8);
		acc_mems_off_tc[i].hfs = (float)(reg_value16) / (float)(1 << 13);

		inv_imu_read_reg(s, gyr_mems_gain_addrs[i], 2, (uint8_t *)reg_value12);
		reg_value16 =
			(int16_t)reg_value12[0] +
			(int16_t)((reg_value12[1] & 0x0F) << 8); /* This is only a 12 bit value */
		if (reg_value16 & 0x800) {                       /* If this should be negative */
			reg_value16 |= 0xF000;
		}
		gyr_mems_gain_tc[i] = (float)(reg_value16 & 0xFFF) / (float)(1 << 11);

		inv_imu_read_reg(s, acc_mems_gain_addrs[i], 2, (uint8_t *)reg_value12);
		reg_value16 =
			(int16_t)reg_value12[0] +
			(int16_t)((reg_value12[1] & 0x0F) << 8); /* This is only a 12 bit value */
		if (reg_value16 & 0x800) {                       /* If this should be negative */
			reg_value16 |= 0xF000;
		}
		acc_mems_gain_tc[i] = (float)(reg_value16 & 0xFFF) / (float)(1 << 11);
	}

	for (int i = 0; i < 3; i++) {
		gyr_offs_tc[i].a_hfs -=
			gyr_gain_tc[i].a_hfs * (gyr_mems_off_tc[i].hfs / gyr_mems_gain_tc[i]);
		gyr_offs_tc[i].a_lfs -=
			gyr_gain_tc[i].a_lfs * (gyr_mems_off_tc[i].lfs / gyr_mems_gain_tc[i]);
		gyr_offs_tc[i].b_hfs -=
			gyr_gain_tc[i].b_hfs * (gyr_mems_off_tc[i].hfs / gyr_mems_gain_tc[i]);
		gyr_offs_tc[i].b_lfs -=
			gyr_gain_tc[i].b_lfs * (gyr_mems_off_tc[i].lfs / gyr_mems_gain_tc[i]);

		acc_offs_tc[i].a_hfs -=
			acc_gain_tc[i].a_hfs * (acc_mems_off_tc[i].hfs / acc_mems_gain_tc[i]);
		acc_offs_tc[i].a_hfs32 -=
			acc_gain_tc[i].a_hfs32 * (acc_mems_off_tc[i].hfs / acc_mems_gain_tc[i]);
		acc_offs_tc[i].a_lfs -=
			acc_gain_tc[i].a_lfs * (acc_mems_off_tc[i].hfs / acc_mems_gain_tc[i]);
		acc_offs_tc[i].b_hfs -=
			acc_gain_tc[i].b_hfs * (acc_mems_off_tc[i].hfs / acc_mems_gain_tc[i]);
		acc_offs_tc[i].b_hfs32 -=
			acc_gain_tc[i].b_hfs32 * (acc_mems_off_tc[i].hfs / acc_mems_gain_tc[i]);
		acc_offs_tc[i].b_lfs -=
			acc_gain_tc[i].b_lfs * (acc_mems_off_tc[i].hfs / acc_mems_gain_tc[i]);
	}

	for (int i = 0; i < 3; i++) {
		float rounding; /* The calculated float values need to be rounded */
				/* to the nearest fixed point value. */
		rounding = gyr_offs_tc[i].a_hfs < 0 ? -0.5 : 0.5;
		gyr_off_vals[0] = (int16_t)((gyr_offs_tc[i].a_hfs * (1 << 11)) + rounding);
		rounding = gyr_offs_tc[i].a_lfs < 0 ? -0.5 : 0.5;
		gyr_off_vals[1] = (int16_t)((gyr_offs_tc[i].a_lfs * (1 << 11)) + rounding);
		rounding = gyr_offs_tc[i].b_hfs < 0 ? -0.5 : 0.5;
		gyr_off_vals[2] = (int16_t)((gyr_offs_tc[i].b_hfs * (1 << 11)) + rounding);
		rounding = gyr_offs_tc[i].b_lfs < 0 ? -0.5 : 0.5;
		gyr_off_vals[3] = (int16_t)((gyr_offs_tc[i].b_lfs * (1 << 11)) + rounding);
		write_gyr_off_values(s, i, gyr_off_vals);

		rounding = acc_offs_tc[i].a_hfs < 0 ? -0.5 : 0.5;
		acc_off_vals[0] = (int16_t)((acc_offs_tc[i].a_hfs * (1 << 15)) + rounding);
		rounding = acc_offs_tc[i].a_hfs32 < 0 ? -0.5 : 0.5;
		acc_off_vals[1] = (int16_t)((acc_offs_tc[i].a_hfs32 * (1 << 15)) + rounding);
		rounding = acc_offs_tc[i].a_lfs < 0 ? -0.5 : 0.5;
		acc_off_vals[2] = (int16_t)((acc_offs_tc[i].a_lfs * (1 << 15)) + rounding);
		rounding = acc_offs_tc[i].b_hfs < 0 ? -0.5 : 0.5;
		acc_off_vals[3] = (int16_t)((acc_offs_tc[i].b_hfs * (1 << 15)) + rounding);
		rounding = acc_offs_tc[i].b_hfs32 < 0 ? -0.5 : 0.5;
		acc_off_vals[4] = (int16_t)((acc_offs_tc[i].b_hfs32 * (1 << 15)) + rounding);
		rounding = acc_offs_tc[i].b_lfs < 0 ? -0.5 : 0.5;
		acc_off_vals[5] = (int16_t)((acc_offs_tc[i].b_lfs * (1 << 15)) + rounding);
		write_acc_off_values(s, i, acc_off_vals);
	}
}

/*
 * Disable the HW temperature compensation so it doesn't interfere
 * with the SW temperature compensation.
 */
int inv_imu_disable_hw_tc(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	uint8_t zero8 = 0x00;

	adjust_tc_offset(s);

	/* Zero out the automatic temperature compensation gain */
	for (int i = 0; i < NUM_ACCEL_TC_COEF; i++) {
		status |= inv_imu_write_reg(s, START_ACCEL_TC_GAIN + i, 1, &zero8);
	}
	for (int i = 0; i < NUM_GYRO_TC_COEF; i++) {
		status |= inv_imu_write_reg(s, START_GYRO_TC_GAIN + i, 1, &zero8);
	}

	return status;
}
