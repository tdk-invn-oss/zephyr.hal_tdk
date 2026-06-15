/*
 *
 * Copyright (c) [2015] by InvenSense, Inc.
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

#include "imu/inv_imu_selftest.h"
#include "imu/inv_imu_edmp.h"
#include "Invn/EmbUtils/Message.h" /* Andrew - Just for debug */

/* Static functions definition */
static int set_selftest_parameters(inv_imu_device_t *s,
				   const inv_imu_selftest_parameters_t *st_params);
static int run_internal_selftest(inv_imu_device_t *s);
static int get_selftest_output(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
			       inv_imu_selftest_output_t *st_output);

/* API implementation */

int inv_imu_selftest_init_params(inv_imu_device_t *s, inv_imu_selftest_parameters_t *st_params)
{
	int rc = INV_IMU_OK;

	st_params->accel_en = INV_IMU_ENABLE;
	st_params->gyro_en = INV_IMU_ENABLE;
	st_params->avg_time = SELFTEST_AVG_TIME_320_MS;
	st_params->accel_limit = SELFTEST_ACCEL_THRESHOLD_50_PERCENT;
	st_params->gyro_limit = SELFTEST_GYRO_THRESHOLD_50_PERCENT;
	st_params->patch_settings = 0;

	return rc;
}

int inv_imu_selftest(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
		     inv_imu_selftest_output_t *st_output)
{
	int rc = INV_IMU_OK;

	if (!(st_params->accel_en || st_params->gyro_en)) {
		return INV_IMU_ERROR;
	}

	rc |= inv_imu_adv_device_reset(s);

	inv_imu_sleep_us(s, 10000);

	/* Configure start addresses as we reset the device */
	rc |= inv_imu_edmp_configure(s);

	rc |= set_selftest_parameters(s, st_params);

	rc |= run_internal_selftest(s);

	rc |= get_selftest_output(s, st_params, st_output);

	rc |= inv_imu_adv_device_reset(s);

	return rc;
}

/* Static functions implementation */

static int set_selftest_parameters(inv_imu_device_t *s,
				   const inv_imu_selftest_parameters_t *st_params)
{
	int rc = INV_IMU_OK;
	uint32_t tmp_stc_params;
	int init_en;

	rc |= inv_imu_adv_power_up_sram(s);

	rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_CONFIGPARAMS, (uint8_t *)&tmp_stc_params);
	tmp_stc_params &= ~(SELFTESTCAL_INIT_EN_MASK | SELFTEST_ACCEL_EN_MASK |
			    SELFTEST_GYRO_EN_MASK | SELFTEST_UNUSED | SELFTEST_AVERAGE_TIME_MASK |
			    SELFTEST_ACCEL_THRESH_MASK | SELFTEST_GYRO_THRESH_MASK);

	init_en = (st_params->accel_en || st_params->gyro_en);
	tmp_stc_params |= (init_en ? SELFTESTCAL_INIT_EN : SELFTESTCAL_INIT_DIS);
	tmp_stc_params |= (st_params->accel_en ? SELFTEST_ACCEL_EN : SELFTEST_ACCEL_DIS);
	tmp_stc_params |= (st_params->gyro_en ? SELFTEST_GYRO_EN : SELFTEST_GYRO_DIS);
	tmp_stc_params |= (uint32_t)(st_params->accel_limit & SELFTEST_ACCEL_THRESH_MASK);
	tmp_stc_params |= (uint32_t)(st_params->gyro_limit & SELFTEST_GYRO_THRESH_MASK);
	tmp_stc_params |= (uint32_t)(st_params->avg_time & SELFTEST_AVERAGE_TIME_MASK);
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_CONFIGPARAMS, (uint8_t *)&tmp_stc_params);

	tmp_stc_params = 0; /* disable any selftest stop point */
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_DEBUG_EN, (uint8_t *)&tmp_stc_params);

	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_PATCH_EN,
				      (uint8_t *)&(st_params->patch_settings));

	return rc;
}

static int run_internal_selftest(inv_imu_device_t *s)
{
	int rc = INV_IMU_OK;
	reg_host_msg_t reg_host_msg;
	int_apex_config1_t int_apex_config1;
	int timeout_us = 3000000; /* 3 seconds */

	rc |= inv_imu_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.testopenable = INV_IMU_ENABLE;
	rc |= inv_imu_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	/* Enable desired interrupt */
	rc |= inv_imu_read_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
	int_apex_config1.int_status_mask_pin_selftest_done = 0;
	rc |= inv_imu_write_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

	/* Run EDMP */
	rc |= inv_imu_edmp_run_ondemand(s, INV_IMU_EDMP_INT2);

	/* Wait for the desired interrupt */
	while (1) {
		int_apex_status1_t int_apex_status1;
		rc |= inv_imu_read_reg(s, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);
		if (int_apex_status1.int_status_selftest_done) {
			break;
		}

		inv_imu_sleep_us(s, 100);
		timeout_us -= 100;

		if (timeout_us <= 0) {
			return INV_IMU_ERROR_TIMEOUT;
		}
	}

	/* Disable interrupt */
	rc |= inv_imu_read_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
	int_apex_config1.int_status_mask_pin_selftest_done = 1;
	rc |= inv_imu_write_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

	return rc;
}

static int get_selftest_output(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
			       inv_imu_selftest_output_t *st_output)
{
	int rc = INV_IMU_OK;
	uint32_t stc_results;

	/* Read STC results */
	rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_RESULTS, (uint8_t *)&stc_results);

	if (!st_params->accel_en) {
		st_output->accel_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->ax_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->ay_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->az_status = INV_IMU_ST_STATUS_NOT_RUN;
	} else {
		st_output->accel_status =
			((stc_results & (STC_RESULTS_ACCEL_X_MASK | STC_RESULTS_ACCEL_Y_MASK |
					 STC_RESULTS_ACCEL_Z_MASK | STC_RESULTS_ST_STATUS_MASK)) ==
			 0)
				? INV_IMU_ST_STATUS_SUCCESS
				: INV_IMU_ST_STATUS_FAIL;
		st_output->ax_status = ((stc_results & STC_RESULTS_ACCEL_X_MASK) == 0)
					       ? INV_IMU_ST_STATUS_SUCCESS
					       : INV_IMU_ST_STATUS_FAIL;
		st_output->ay_status = ((stc_results & STC_RESULTS_ACCEL_Y_MASK) == 0)
					       ? INV_IMU_ST_STATUS_SUCCESS
					       : INV_IMU_ST_STATUS_FAIL;
		st_output->az_status = ((stc_results & STC_RESULTS_ACCEL_Z_MASK) == 0)
					       ? INV_IMU_ST_STATUS_SUCCESS
					       : INV_IMU_ST_STATUS_FAIL;
	}

	if (!st_params->gyro_en) {
		st_output->gyro_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gx_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gy_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gz_status = INV_IMU_ST_STATUS_NOT_RUN;
	} else {
		st_output->gyro_status =
			((stc_results & (STC_RESULTS_GYRO_X_MASK | STC_RESULTS_GYRO_Y_MASK |
					 STC_RESULTS_GYRO_Z_MASK | STC_RESULTS_ST_STATUS_MASK)) ==
			 0)
				? INV_IMU_ST_STATUS_SUCCESS
				: INV_IMU_ST_STATUS_FAIL;
		st_output->gx_status = ((stc_results & STC_RESULTS_GYRO_X_MASK) == 0)
					       ? INV_IMU_ST_STATUS_SUCCESS
					       : INV_IMU_ST_STATUS_FAIL;
		st_output->gy_status = ((stc_results & STC_RESULTS_GYRO_Y_MASK) == 0)
					       ? INV_IMU_ST_STATUS_SUCCESS
					       : INV_IMU_ST_STATUS_FAIL;
		st_output->gz_status = ((stc_results & STC_RESULTS_GYRO_Z_MASK) == 0)
					       ? INV_IMU_ST_STATUS_SUCCESS
					       : INV_IMU_ST_STATUS_FAIL;
	}

	return rc;
}

/*
********************************************************************************************************
* Direct (host based) self-test *
********************************************************************************************************
*/

int inv_imu_st_direct_init_params(inv_imu_device_t *s, inv_imu_selftest_parameters_t *st_params)
{
	int rc = INV_IMU_OK;

	st_params->accel_en = INV_IMU_ENABLE;
	st_params->gyro_en = INV_IMU_ENABLE;
	st_params->avg_time = ST_DIRECT_AVG_TIME_320_MS;
	st_params->accel_limit = ST_DIRECT_ACCEL_THRESHOLD_5_PERCENT;
	st_params->gyro_limit = ST_DIRECT_GYRO_THRESHOLD_5_PERCENT;

	return rc;
}

/*
 * int set_st_direct_config: Setup the configuration required to run self-test.
 */
static int set_st_direct_config(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params)
{
	int rc = INV_IMU_OK;
	ipreg_sys1_reg_146_t sys1_reg146;
	ipreg_sys1_reg_148_t sys1_reg148;
	ipreg_sys1_reg_150_t sys1_reg150;
	ipreg_sys1_reg_155_t sys1_reg155;
	ipreg_sys1_reg_158_t sys1_reg158;
	ipreg_sys2_reg_105_t sys2_reg105;
	ipreg_sys2_reg_106_t sys2_reg106;
	ipreg_sys2_reg_107_t sys2_reg107;
	ipreg_sys2_reg_117_t sys2_reg117;
	ipreg_sys2_reg_112_t sys2_reg112;
	uint16_t sys1_reg0 = 0x0030;
	dmp_ext_sen_odr_cfg_t dmp_cfg;

	inv_imu_write_reg(s, IPREG_SYS1_REG_0, 2, (uint8_t *)&sys1_reg0);

	inv_imu_read_reg(s, IPREG_SYS2_REG_117, 1, (uint8_t *)&sys2_reg117);
	sys2_reg117.tmp_inj_en_gos = 1;
	inv_imu_write_reg(s, IPREG_SYS2_REG_117, 1, (uint8_t *)&sys2_reg117);

	inv_imu_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_cfg);
	dmp_cfg.apex_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ;
	inv_imu_write_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_cfg);

	if (st_params->gyro_en) {
		inv_imu_set_gyro_frequency(s, GYRO_CONFIG0_GYRO_ODR_800_HZ);
#if INV_IMU_HIGH_FSR_SUPPORTED
		inv_imu_set_gyro_fsr(s, GYRO_CONFIG0_AP_GYRO_FS_SEL_4000_DPS);
#else
		inv_imu_set_gyro_fsr(s, GYRO_CONFIG0_AP_GYRO_FS_SEL_2000_DPS);
#endif

		inv_imu_read_reg(s, IPREG_SYS1_REG_155, 1, (uint8_t *)&sys1_reg155);
		sys1_reg155.gyro_lpf_bypass = 0;
		inv_imu_write_reg(s, IPREG_SYS1_REG_155, 1, (uint8_t *)&sys1_reg155);

		inv_imu_read_reg(s, IPREG_SYS1_REG_158, 1, (uint8_t *)&sys1_reg158);
		sys1_reg158.gyro_ois_3rd_ord_sel = 0;
		sys1_reg158.gyro_ui_lpfbw_sel = 2;
		inv_imu_write_reg(s, IPREG_SYS1_REG_158, 1, (uint8_t *)&sys1_reg158);

		sys1_reg146.gyro_x_tmid_gain = sys1_reg146.gyro_x_tmid_off = 7;
		inv_imu_write_reg(s, IPREG_SYS1_REG_146, 1, (uint8_t *)&sys1_reg146);

		sys1_reg148.gyro_y_tmid_gain = sys1_reg148.gyro_y_tmid_off = 7;
		inv_imu_write_reg(s, IPREG_SYS1_REG_148, 1, (uint8_t *)&sys1_reg148);

		sys1_reg150.gyro_z_tmid_gain = sys1_reg150.gyro_z_tmid_off = 7;
		inv_imu_write_reg(s, IPREG_SYS1_REG_150, 1, (uint8_t *)&sys1_reg150);

		inv_imu_set_gyro_mode(s, PWR_MGMT0_GYRO_MODE_LN);
	}
	if (st_params->accel_en) {
		inv_imu_set_accel_frequency(s, ACCEL_CONFIG0_ACCEL_ODR_800_HZ);
#if INV_IMU_HIGH_FSR_SUPPORTED
		inv_imu_set_accel_fsr(s, ACCEL_CONFIG0_AP_ACCEL_FS_SEL_32_G);
#else
		inv_imu_set_accel_fsr(s, ACCEL_CONFIG0_AP_ACCEL_FS_SEL_16_G);
#endif

		inv_imu_read_reg(s, IPREG_SYS2_REG_112, 1, (uint8_t *)&sys2_reg112);
		sys2_reg112.accel_ui_3rd_ord_sel = 0;
		sys2_reg112.accel_lpf_bypass = 0;
		sys2_reg112.accel_ui_lpfbw_sel = 2;
		inv_imu_write_reg(s, IPREG_SYS2_REG_112, 1, (uint8_t *)&sys2_reg112);

		sys2_reg105.accel_x_tmid_gain = sys2_reg105.accel_x_tmid_off = 7;
		inv_imu_write_reg(s, IPREG_SYS2_REG_105, 1, (uint8_t *)&sys2_reg105);

		sys2_reg106.accel_y_tmid_gain = sys2_reg106.accel_y_tmid_off = 7;
		inv_imu_write_reg(s, IPREG_SYS2_REG_106, 1, (uint8_t *)&sys2_reg106);

		sys2_reg107.accel_z_tmid_gain = sys2_reg107.accel_z_tmid_off = 7;
		inv_imu_write_reg(s, IPREG_SYS2_REG_107, 1, (uint8_t *)&sys2_reg107);

		inv_imu_set_accel_mode(s, PWR_MGMT0_ACCEL_MODE_LN);
	}

	inv_imu_sleep_us(s, GYR_STARTUP_TIME_US);

	return rc;
}

/*
 * st_direct_phase1: Run the first phase of self-test -- Normal data collection
 */
static int st_direct_phase1(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
			    float phase1_out[])
{
#define ODR_INTERVAL 1250 /* The ODR time interval at 800 Hz is 1250 us */
	int rc = INV_IMU_OK;
	int64_t phase1_total[6] = {0, 0, 0, 0, 0, 0};
	int timeout;
	int num_samples = 0;
	inv_imu_sensor_data_t sensor_data;

	timeout = st_params->avg_time * 1000; /* Convert to us. */

	while (timeout > 0) { /* Collect accel and gyro data in one pass */
		rc |= inv_imu_get_register_data(s, &sensor_data);
		num_samples++;
		if (st_params->accel_en) { /* If accel enabled */
			phase1_total[0] += sensor_data.accel_data[0];
			phase1_total[1] += sensor_data.accel_data[1];
			phase1_total[2] += sensor_data.accel_data[2];
		}
		if (st_params->gyro_en) { /* If gyro enabled */
			phase1_total[3] += sensor_data.gyro_data[0];
			phase1_total[4] += sensor_data.gyro_data[1];
			phase1_total[5] += sensor_data.gyro_data[2];
		}
		inv_imu_sleep_us(s, ODR_INTERVAL);
		timeout -= ODR_INTERVAL;
	}

	for (int i = 0; i < 6; i++) {
		phase1_out[i] = (float)phase1_total[i] / (float)num_samples;
	}
	return rc;
}

/*
 * st_direct_phase2: Run the second phase of self-test -- Data collection in self-test mode.
 */
static int st_direct_phase2(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
			    float phase2_out[])
{
#define ODR_INTERVAL 1250 /* The ODR time interval at 800 Hz is 1250 us */
	int rc = INV_IMU_OK;
	int32_t phase2_total[6] = {0, 0, 0, 0, 0, 0};
	int timeout;
	int num_samples_gyro = 0;
	int num_samples_accel[3] = {0, 0, 0};
	ipreg_ana_reg_8_t ana_reg_8;
	uint8_t selftest_byte;
	selftest_t selftest;
	inv_imu_sensor_data_t sensor_data;

	/* Analog configuration */
	inv_imu_read_reg(s, IPREG_ANA_REG_8, 1, (uint8_t *)&ana_reg_8);
	ana_reg_8.pd_accel_cp45_st_b_d2a = 1;
	inv_imu_write_reg(s, IPREG_ANA_REG_8, 1, (uint8_t *)&ana_reg_8);
	inv_imu_sleep_us(s, 1000);
	ana_reg_8.pd_accel_st_b_d2a = 1;
	inv_imu_write_reg(s, IPREG_ANA_REG_8, 1, (uint8_t *)&ana_reg_8);
	inv_imu_sleep_us(s, 1000);
	if (st_params->accel_en) { /* Run the accel axes one at a time */
		for (int i = 0; i < 3; i++) {
			timeout = st_params->avg_time * 1000; /* Convert to us. */
			selftest_byte = (1 << i);             /* Cycle through the accel axes */
			inv_imu_write_reg(s, SELFTEST, 1, &selftest_byte);
			inv_imu_sleep_us(s, 10000);
			while (timeout > 0) { /* Collect the data for the active accel axis */
				rc |= inv_imu_get_register_data(s, &sensor_data);
				num_samples_accel[i]++;
				phase2_total[i] += sensor_data.accel_data[i];
				inv_imu_sleep_us(s, ODR_INTERVAL);
				timeout -= ODR_INTERVAL;
			}
		}
	}

	if (st_params->gyro_en) { /* Run all three gyro axes at once */
		inv_imu_sleep_us(s, 2000);
		selftest.en_ax_st = selftest.en_ay_st = selftest.en_az_st = 0;
		selftest.en_gx_st = selftest.en_gy_st = selftest.en_gz_st = 1;
		inv_imu_write_reg(s, SELFTEST, 1, (uint8_t *)&selftest);
		inv_imu_sleep_us(s, GYR_STARTUP_TIME_US);
		timeout = st_params->avg_time * 1000; /* Convert to us. */
		while (timeout > 0) {
			rc |= inv_imu_get_register_data(s, &sensor_data);
			num_samples_gyro++;
			phase2_total[3] += sensor_data.gyro_data[0];
			phase2_total[4] += sensor_data.gyro_data[1];
			phase2_total[5] += sensor_data.gyro_data[2];
			inv_imu_sleep_us(s, ODR_INTERVAL);
			timeout -= ODR_INTERVAL;
		}
	}

	for (int i = 0; i < 3; i++) {
		phase2_out[i] = (float)phase2_total[i] /
				(float)num_samples_accel[i]; /* Calculate accel data average */
		phase2_out[i + 3] = (float)phase2_total[i + 3] /
				    (float)num_samples_gyro; /* Calculate gyro data average */
	}
	return rc;
}

/*
 * get_eDMP_params: Load the OTP trim values that are required for self-test.
 */
static int get_eDMP_params(inv_imu_device_t *s, int16_t str_ft[])
{
	int rc = INV_IMU_OK;
	inv_imu_selftest_parameters_t st_params;

	rc |= inv_imu_edmp_configure(s); /* Configure start addresses as we reset the device */

	rc |= inv_imu_selftest_init_params(s, &st_params);
	st_params.gyro_en = 0;
	st_params.avg_time = SELFTEST_AVG_TIME_10_MS;
	rc |= set_selftest_parameters(s, &st_params);

	rc |= run_internal_selftest(s); /* Run internal selftest to load SRAM */
	inv_imu_read_sram(s, IMEM_SRAM_REG_12, 6,
			  (uint8_t *)&str_ft[0]); /* Read accel_{x/y/z}_str_ft */
	inv_imu_read_sram(s, IMEM_SRAM_REG_0, 6,
			  (uint8_t *)&str_ft[3]); /* Read agyro_{x/y/z}_str_ft */

	return rc;
}

/*
 * inv_imu_st_direct: Run the self-test operation on the host.
 */
int inv_imu_st_direct(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
		      inv_imu_selftest_output_t *st_output)
{
#if INV_IMU_20BIT_REG_DATA_SUPPORTED == 1
#define ACCEL_FSR      32      /* The accel FSR is 32g */
#define GYRO_FSR       4000    /* The gyro FSR is 4000 dps */
#define MAX_SATURATION 0x80000 /* The saturation value for 20-bit data */
#else
#define ACCEL_FSR      16     /* The accel FSR is 16g */
#define GYRO_FSR       2000   /* The gyro FSR is 2000 dps */
#define MAX_SATURATION 0x8000 /* The saturation value for 16-bit data */
#endif
	int rc = INV_IMU_OK;
	float phase1_out[6];
	float phase2_out[6];
	float st_response_float[6];
	int16_t str_ft[6];
	float str_ft_float[6];
	float acc_threshold, gyr_threshold;
	float acc_str_min[3], acc_str_max[3], gyr_str_min[3], gyr_str_max[3];

	if (!(st_params->accel_en ||
	      st_params->gyro_en)) { /* If neither sensor is enabled, don't run self-test */
		return INV_IMU_ERROR;
	}

	rc |= inv_imu_adv_device_reset(s); /* Soft reset to create a clean configuration */

	inv_imu_sleep_us(s, 10000);

	rc |= inv_imu_adv_power_up_sram(s); /* Power up the SRAM */

	inv_imu_sleep_us(s, 10000);

	rc |= get_eDMP_params(s, str_ft); /* Load and read str_ft values from SRAM */

	/* Convert str_ft values to floating point */
	for (int i = 0; i < 3; i++) {
		str_ft_float[i] = (float)str_ft[i] /
				  (float)(1 << 13); /* accel_{x/y/z} has a 2^13 resolution */
		str_ft_float[i + 3] = (float)str_ft[i + 3] /
				      (float)(1 << 5); /* gyro_{x/y/z} has a 2^5 resolution */
	}

	rc |= set_st_direct_config(s, st_params); /* Configure the part for running self-test */

	acc_threshold = (float)st_params->accel_limit / 100.0;
	gyr_threshold = (float)st_params->gyro_limit / 100.0;

	rc |= st_direct_phase1(s, st_params,
			       phase1_out); /* Run phase1, collect data in normal mode */

	rc |= st_direct_phase2(s, st_params,
			       phase2_out); /* Run phase2, collect data in selftest mode */

	/* Take the delta of phase 1 and phase 2 and convert to SI units */
	/* The accel fsr is 32g and the fsr of the gyro is 4000dps */
	for (int i = 0; i < 3; i++) {
		/* Convert delta of the accel data from the two phases into SI units */
		st_response_float[i] =
			((phase2_out[i] - phase1_out[i]) * ACCEL_FSR) / MAX_SATURATION;

		/* Convert delta of the gyro data from the two phases into SI units */
		st_response_float[i + 3] =
			((phase2_out[i + 3] - phase1_out[i + 3]) * GYRO_FSR) / MAX_SATURATION;
	}

	/* Calculate the min and max threshold values for accel and gyro */
	for (int i = 0; i < 3; i++) {
		acc_str_min[i] = (1.0 - acc_threshold) * str_ft_float[i];
		acc_str_max[i] = (1.0 + acc_threshold) * str_ft_float[i];
		gyr_str_min[i] = (1.0 - gyr_threshold) * str_ft_float[i + 3];
		gyr_str_max[i] = (1.0 + gyr_threshold) * str_ft_float[i + 3];
	}

	memset(st_output, 0, sizeof(inv_imu_selftest_output_t));

	if (st_params->accel_en) {
		st_output->ax_status = ((st_response_float[0] < acc_str_min[0]) ||
					(st_response_float[0] > acc_str_max[0]))
					       ? -1
					       : 1;
		st_output->ay_status = ((st_response_float[1] < acc_str_min[1]) ||
					(st_response_float[1] > acc_str_max[1]))
					       ? -1
					       : 1;
		st_output->az_status = ((st_response_float[2] < acc_str_min[2]) ||
					(st_response_float[2] > acc_str_max[2]))
					       ? -1
					       : 1;
		st_output->accel_status =
			((st_output->ax_status == 1) && (st_output->ay_status == 1) &&
			 (st_output->az_status == 1))
				? 1
				: -1;
	}
	if (st_params->gyro_en) {
		st_output->gx_status = ((st_response_float[3] < gyr_str_min[0]) ||
					(st_response_float[3] > gyr_str_max[0]))
					       ? -1
					       : 1;
		st_output->gy_status = ((st_response_float[4] < gyr_str_min[1]) ||
					(st_response_float[4] > gyr_str_max[1]))
					       ? -1
					       : 1;
		st_output->gz_status = ((st_response_float[5] < gyr_str_min[2]) ||
					(st_response_float[5] > gyr_str_max[2]))
					       ? -1
					       : 1;
		st_output->gyro_status =
			((st_output->gx_status == 1) && (st_output->gy_status == 1) &&
			 (st_output->gz_status == 1))
				? 1
				: -1;
	}
	rc |= inv_imu_adv_device_reset(s);

	return rc;
}
