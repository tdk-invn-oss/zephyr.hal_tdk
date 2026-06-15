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

#include "imu/inv_imu_selfcal.h"
#include "imu/inv_imu_edmp.h"

/* Gyro Self-Cal calculation Method */
/* The methods chosen has exhibited the best result for a given axis */
#define SC_GYRO_X_CALC_METHOD SELFCAL_STEP_RESPONSE
#define SC_GYRO_Y_CALC_METHOD SELFCAL_SC2V_ADC
#define SC_GYRO_Z_CALC_METHOD SELFCAL_STEP_RESPONSE

/*
 * The slope and bias values are calculated as floats and then multiplied by 65536 in order
 * to be used in the fixed point implementation.
 */
#define SC_AX_SLOPE 28532 /* = 0.43537 * 65536. X-axis slope for Accel Self-Cal calculation */
#define SC_AY_SLOPE 31770 /* = 0.48477 * 65536. Y-axis slope for Accel Self-Cal calculation */
#define SC_AZ_SLOPE 16528 /* = 0.2522 * 65536. Z-axis slope for Accel Self-Cal calculation */

#define SC_AX_BIAS -31 /* = -0.00047 * 65536. X-axis bias for Accel Self-Cal calculation */
#define SC_AY_BIAS -34 /* = -0.00052 * 65536. Y-axis bias for Accel Self-Cal calculation */
#define SC_AZ_BIAS -29 /* = -0.00045 * 65536. Z-axis bias for Accel Self-Cal calculation */

#define SC_GX_SLOPE 19930 /* = 0.304112 * 65536. X-axis slope for Gyro Self-Cal calculation */
#define SC_GY_SLOPE 64991 /* = 0.991678 * 65536. Y-axis slope for Gyro Self-Cal calculation */
#define SC_GZ_SLOPE 15986 /* = 0.243926 * 65536. Z-axis slope for Gyro Self-Cal calculation */

#define SC_GX_BIAS 77  /* =  0.001182 * 65536. X-axis bias for Gyro Self-Cal calculation */
#define SC_GY_BIAS -18 /* = -0.000279 * 65536. Y-axis bias for Gyro Self-Cal calculation */
#define SC_GZ_BIAS -5  /* = -0.000081 * 65536. Z-axis bias for Gyro Self-Cal calculation */

#define SCALE_FROM_FLOAT 65536 /* To only use fixed point, values are scaled up by 2^16 */

/* Self-cal internal output */
typedef struct {
	int32_t sc_accel_nout_meas_x;
	int32_t sc_accel_nout_meas_y;
	int32_t sc_accel_nout1_meas_z;
	int32_t sc_accel_nout2_meas_z;
	int32_t sc_accel_cmos_meas_x;
	int16_t sc_accel_nout_trim[3];
	int16_t sc_gyro_str_ft_trim[3];
	int32_t sc_gyro_step_response32[3];
	int32_t sc_gyro_gain32[3];
	int16_t sc_gyro_cmos_trim[3];
} inv_imu_selfcal_output_internal_t;

/* Static functions definition */
static int calc_accel_usergain(inv_imu_device_t *s, const inv_imu_selfcal_parameters_t *sc_params,
			       const inv_imu_selfcal_output_internal_t *sc_output_i,
			       uint16_t *accel_usergain);
static int calc_gyro_step_response_usergain(inv_imu_device_t *s,
					    const inv_imu_selfcal_parameters_t *sc_params,
					    const inv_imu_selfcal_output_internal_t *sc_output_i,
					    uint16_t *gyro_usergain);
static int calc_gyro_sc2v_adc_usergain(inv_imu_device_t *s,
				       const inv_imu_selfcal_parameters_t *sc_params,
				       const inv_imu_selfcal_output_internal_t *sc_output_i,
				       uint16_t *gyro_usergain);
static int set_selfcal_parameters(inv_imu_device_t *s,
				  const inv_imu_selfcal_parameters_t *sc_params,
				  const selfcal_gyro_method_t gyro_method);
static int run_internal_selfcal(inv_imu_device_t *s);
static int get_selfcal_output(inv_imu_device_t *s, const inv_imu_selfcal_parameters_t *sc_params,
			      inv_imu_selfcal_output_t *sc_output,
			      inv_imu_selfcal_output_internal_t *sc_output_i);
static int run_selfcal(inv_imu_device_t *s, const inv_imu_selfcal_parameters_t *sc_params,
		       const selfcal_gyro_method_t gyro_method, inv_imu_selfcal_output_t *sc_output,
		       inv_imu_selfcal_output_internal_t *sc_output_i);

/* API implementation */

int inv_imu_selfcal_init_params(inv_imu_device_t *s, inv_imu_selfcal_parameters_t *sc_params)
{
	int rc = INV_IMU_OK;

	sc_params->avg_time = SELFTEST_AVG_TIME_320_MS;
	sc_params->accel_en = INV_IMU_ENABLE;
	sc_params->gyro_en = INV_IMU_ENABLE;
	sc_params->ssom_en = INV_IMU_ENABLE;
	sc_params->gyro_method[0] = SC_GYRO_X_CALC_METHOD;
	sc_params->gyro_method[1] = SC_GYRO_Y_CALC_METHOD;
	sc_params->gyro_method[2] = SC_GYRO_Z_CALC_METHOD;
	sc_params->accel_slope[0] = SC_AX_SLOPE;
	sc_params->accel_slope[1] = SC_AY_SLOPE;
	sc_params->accel_slope[2] = SC_AZ_SLOPE;
	sc_params->accel_bias[0] = SC_AX_BIAS;
	sc_params->accel_bias[1] = SC_AY_BIAS;
	sc_params->accel_bias[2] = SC_AZ_BIAS;
	sc_params->gyro_slope[0] = SC_GX_SLOPE;
	sc_params->gyro_slope[1] = SC_GY_SLOPE;
	sc_params->gyro_slope[2] = SC_GZ_SLOPE;
	sc_params->gyro_bias[0] = SC_GX_BIAS;
	sc_params->gyro_bias[1] = SC_GY_BIAS;
	sc_params->gyro_bias[2] = SC_GZ_BIAS;
	sc_params->patch_settings = 0;

	return rc;
}

int inv_imu_selfcal(inv_imu_device_t *s, const inv_imu_selfcal_parameters_t *sc_params,
		    inv_imu_selfcal_output_t *sc_output)
{
	int rc = INV_IMU_OK;
	inv_imu_selfcal_output_internal_t sc_internal_output = {0};
	uint16_t temp_step_response_gains[3] = {0};
	uint16_t temp_sc2v_adc_gains[3] = {0};
	int i;

	/* If neither gyro or accel self-cal are enabled */
	if (!(sc_params->accel_en || sc_params->gyro_en)) {
		return INV_IMU_ERROR;
	}

	/* If the gyro is to be calibrated */
	if (sc_params->gyro_en) {
		/* If at least one axis uses Step Response */
		if ((sc_params->gyro_method[0] == SELFCAL_STEP_RESPONSE) ||
		    (sc_params->gyro_method[1] == SELFCAL_STEP_RESPONSE) ||
		    (sc_params->gyro_method[2] == SELFCAL_STEP_RESPONSE)) {
			rc |= run_selfcal(s, sc_params, SELFCAL_STEP_RESPONSE, sc_output,
					  &sc_internal_output);
			if (sc_output->gyro_status == INV_IMU_SC_STATUS_SUCCESS) {
				rc |= calc_gyro_step_response_usergain(s, sc_params,
								       &sc_internal_output,
								       temp_step_response_gains);
			} else {
				return INV_IMU_ERROR;
			}
		}

		/* If at least one axis uses SC2V ADC */
		if ((sc_params->gyro_method[0] == SELFCAL_SC2V_ADC) ||
		    (sc_params->gyro_method[1] == SELFCAL_SC2V_ADC) ||
		    (sc_params->gyro_method[2] == SELFCAL_SC2V_ADC)) {
			rc |= run_selfcal(s, sc_params, SELFCAL_SC2V_ADC, sc_output,
					  &sc_internal_output);
			if (sc_output->gyro_status == INV_IMU_SC_STATUS_SUCCESS) {
				rc |= calc_gyro_sc2v_adc_usergain(s, sc_params, &sc_internal_output,
								  temp_sc2v_adc_gains);
			} else {
				return INV_IMU_ERROR;
			}
		}

		/* For each axis get the gain calculation generated by the configured method */
		for (i = 0; i < 3; i++) {
			sc_output->gyro_gains[i] =
				(sc_params->gyro_method[i] == SELFCAL_STEP_RESPONSE)
					? temp_step_response_gains[i]
					: temp_sc2v_adc_gains[i];
		}
	}

	/* If the accel is to be calibrated */
	if (sc_params->accel_en) {
		/* Gyro SelfCal is not enabled so SelfCal has not yet been run */
		if (!sc_params->gyro_en) {
			rc |= run_selfcal(s, sc_params, SELFCAL_STEP_RESPONSE, sc_output,
					  &sc_internal_output);
		}
		if (sc_output->accel_status != INV_IMU_SC_STATUS_SUCCESS) {
			return INV_IMU_ERROR;
		}
		rc |= calc_accel_usergain(s, sc_params, &sc_internal_output,
					  sc_output->accel_gains);
	}

	return rc;
}

int inv_imu_stc_set_accel_usergain(inv_imu_device_t *s, const uint16_t *accel_usergain)
{
	int rc = INV_IMU_OK;
	int i;
	int reg_index;

	/*
	 * The accel user gain values are written to 12 bit fields in registers:
	 * IPREG_SYS2_REG_18, IPREG_SYS2_REG_20 and IPREG_SYS2_REG_22
	 */
	for (i = 0, reg_index = IPREG_SYS2_REG_18; i < 3; i++, reg_index += 2) {
		rc |= inv_imu_write_reg(s, reg_index, 2, (uint8_t *)&accel_usergain[i]);
	}

	return rc;
}

int inv_imu_stc_set_gyro_usergain(inv_imu_device_t *s, const uint16_t *gyro_usergain)
{
	int rc = INV_IMU_OK;
	int i;
	int reg_index;

	/*
	 * The gyro user gain values are written to 12 bit fields in registers:
	 * IPREG_SYS1_REG_8, IPREG_SYS1_REG_10 and IPREG_SYS1_REG_12
	 */
	for (i = 0, reg_index = IPREG_SYS1_REG_8; i < 3; i++, reg_index += 2) {
		rc |= inv_imu_write_reg(s, reg_index, 2, (uint8_t *)&gyro_usergain[i]);
	}

	return rc;
}

/* Static functions implementation */

static int calc_accel_usergain(inv_imu_device_t *s, const inv_imu_selfcal_parameters_t *sc_params,
			       const inv_imu_selfcal_output_internal_t *sc_output_i,
			       uint16_t *accel_usergain)
{
	int rc = INV_IMU_OK;
	int32_t accel_sc_nout32[3];
	int32_t accel_sc_nout1_z, accel_sc_nout2_z, accel_sc_cmos_x;
	int16_t accel_sc_nout_trim[3];
	int32_t accel_sc_nout_trim32[3];
	int64_t temp;
	int64_t calculated_gain[3];
	int i;

#if (INV_IMU_HIGH_FSR_SUPPORTED == 1)
	accel_sc_nout32[0] = sc_output_i->sc_accel_nout_meas_x << 1;
	accel_sc_nout32[1] = sc_output_i->sc_accel_nout_meas_y << 1;
	accel_sc_nout1_z = sc_output_i->sc_accel_nout1_meas_z << 1;
	accel_sc_nout2_z = sc_output_i->sc_accel_nout2_meas_z << 1;
	accel_sc_cmos_x = sc_output_i->sc_accel_cmos_meas_x << 1;
#else
	accel_sc_nout32[0] = sc_output_i->sc_accel_nout_meas_x;
	accel_sc_nout32[1] = sc_output_i->sc_accel_nout_meas_y;
	accel_sc_nout1_z = sc_output_i->sc_accel_nout1_meas_z;
	accel_sc_nout2_z = sc_output_i->sc_accel_nout2_meas_z;
	accel_sc_cmos_x = sc_output_i->sc_accel_cmos_meas_x;
#endif

	/* Protect against division by zero */
	if (accel_sc_cmos_x == 0) {
		return INV_IMU_ERROR;
	}

	temp = (int64_t)(accel_sc_nout1_z + accel_sc_nout2_z) *
	       (int64_t)(accel_sc_nout1_z + accel_sc_nout2_z);
	accel_sc_nout32[2] = temp / (int64_t)(4 * accel_sc_cmos_x);

	for (i = 0; i < 3; i++) {
		accel_sc_nout_trim[i] = sc_output_i->sc_accel_nout_trim[i];
		if (accel_sc_nout32[i] < 0) {
			accel_sc_nout32[i] = -accel_sc_nout32[i];
		}
		accel_sc_nout_trim32[i] = accel_sc_nout_trim[i] << 4;

		/* temp = (sc_nout32 * slope) + (sc_nout_trim32 * (SCALE_FROM_FLOAT + bias - slope))
		 */
		temp = SCALE_FROM_FLOAT;
		temp += (int64_t)sc_params->accel_bias[i] - (int64_t)sc_params->accel_slope[i];
		temp *= accel_sc_nout_trim32[i];
		temp += accel_sc_nout32[i] * (int64_t)sc_params->accel_slope[i];

		/* Protect against division by zero */
		if (temp == 0) {
			return INV_IMU_ERROR;
		}

		/* calculated_gain = (sc_nout_trim32 * SCALE_FROM_FLOAT * USERGAIN_SCALE) / temp */
		calculated_gain[i] = (uint64_t)accel_sc_nout_trim32[i];
		calculated_gain[i] *= SCALE_FROM_FLOAT * USERGAIN_SCALE;
		calculated_gain[i] /= temp;

		accel_usergain[i] = (uint16_t)calculated_gain[i];
	}

	return rc;
}

static int calc_gyro_step_response_usergain(inv_imu_device_t *s,
					    const inv_imu_selfcal_parameters_t *sc_params,
					    const inv_imu_selfcal_output_internal_t *sc_output_i,
					    uint16_t *gyro_usergain)
{
	int rc = INV_IMU_OK;
	int32_t gyro_step_response32[3];
	int16_t gyro_step_response16[3];
	int16_t gyro_str_ft_trim[3];
	int64_t temp;
	uint32_t calculated_gain[3];
	int i;

	for (i = 0; i < 3; i++) {
		gyro_str_ft_trim[i] = sc_output_i->sc_gyro_str_ft_trim[i];
		gyro_step_response32[i] = sc_output_i->sc_gyro_step_response32[i];
		gyro_step_response16[i] = (gyro_step_response32[i] >> 11);
		temp = ((gyro_step_response16[i] - gyro_str_ft_trim[i]) * sc_params->gyro_slope[i]);
		temp = ((temp / gyro_str_ft_trim[i])) + SCALE_FROM_FLOAT + sc_params->gyro_bias[i];
		calculated_gain[i] = (SCALE_FROM_FLOAT * USERGAIN_SCALE) / temp;
		gyro_usergain[i] = (uint16_t)(calculated_gain[i]);
	}

	return rc;
}

static int calc_gyro_sc2v_adc_usergain(inv_imu_device_t *s,
				       const inv_imu_selfcal_parameters_t *sc_params,
				       const inv_imu_selfcal_output_internal_t *sc_output_i,
				       uint16_t *gyro_usergain)
{
	int rc = INV_IMU_OK;
	int32_t gyro_gain32[3];
	int16_t gyro_gain16[3];
	int16_t gyro_cmos_trim[3];
	int64_t temp;
	uint32_t calculated_gain[3];
	int i;

	for (i = 0; i < 3; i++) {
		gyro_cmos_trim[i] = sc_output_i->sc_gyro_cmos_trim[i];
		gyro_gain32[i] = sc_output_i->sc_gyro_gain32[i];
		gyro_gain16[i] = (gyro_gain32[i] >> 13);
		temp = (gyro_gain16[i] - gyro_cmos_trim[i]) * sc_params->gyro_slope[i];
		temp = (temp / gyro_cmos_trim[i]) + (1 * SCALE_FROM_FLOAT) +
		       sc_params->gyro_bias[i];
		calculated_gain[i] = (SCALE_FROM_FLOAT * USERGAIN_SCALE) / temp;
		gyro_usergain[i] = (uint16_t)(calculated_gain[i]);
	}

	return rc;
}

static int set_selfcal_parameters(inv_imu_device_t *s,
				  const inv_imu_selfcal_parameters_t *sc_params,
				  const selfcal_gyro_method_t gyro_method)
{
	int rc = INV_IMU_OK;
	uint32_t tmp_stc_params;
	int init_en;

	rc |= inv_imu_adv_power_up_sram(s);

	rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_CONFIGPARAMS, (uint8_t *)&tmp_stc_params);
	tmp_stc_params &=
		~(SELFTESTCAL_INIT_EN_MASK | SELFTEST_ACCEL_EN_MASK | SELFTEST_GYRO_EN_MASK |
		  SELFCAL_ACCEL_EN_MASK | SELFCAL_GYRO_EN_MASK | SELFCAL_SSOM_EN_MASK |
		  SELFCAL_GYRO_METHOD_MASK | SELFTEST_AVERAGE_TIME_MASK |
		  SELFTEST_ACCEL_THRESH_MASK | SELFTEST_GYRO_THRESH_MASK);

	init_en = (sc_params->accel_en || sc_params->gyro_en);
	tmp_stc_params |= (init_en ? SELFTESTCAL_INIT_EN : SELFTESTCAL_INIT_DIS);
	tmp_stc_params |= (sc_params->accel_en ? SELFCAL_ACCEL_EN : SELFCAL_ACCEL_DIS);
	tmp_stc_params |= (sc_params->gyro_en ? SELFCAL_GYRO_EN : SELFCAL_GYRO_DIS);
	tmp_stc_params |= (sc_params->ssom_en ? SELFCAL_SSOM_EN : SELFCAL_SSOM_DIS);
	tmp_stc_params |= (uint32_t)(sc_params->avg_time & SELFTEST_AVERAGE_TIME_MASK);
	tmp_stc_params |= (uint32_t)(gyro_method & SELFCAL_GYRO_METHOD_MASK);
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_CONFIGPARAMS, (uint8_t *)&tmp_stc_params);

	tmp_stc_params = 1344; /* SSOM accel threshold : 20.50 mg in q16 */
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SSOM_TRIGGER_TH_ACCEL, (uint8_t *)&tmp_stc_params);
	tmp_stc_params = 52000; /* SSOM gyro threshold : 0.794 dps in q16 */
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SSOM_TRIGGER_TH_GYRO, (uint8_t *)&tmp_stc_params);

	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_PATCH_EN,
				      (uint8_t *)&(sc_params->patch_settings));

	return rc;
}

static int run_internal_selfcal(inv_imu_device_t *s)
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
	int_apex_config1.reserved = 0;
	rc |= inv_imu_write_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

	/* Run EDMP */
	rc |= inv_imu_edmp_run_ondemand(s, INV_IMU_EDMP_INT2);

	/* Wait for the desired interrupt */
	while (1) {
		int_apex_status1_t int_apex_status1;
		rc |= inv_imu_read_reg(s, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);
		if (int_apex_status1.reserved) {
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
	int_apex_config1.reserved = 1;
	rc |= inv_imu_write_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

	return rc;
}

static int get_selfcal_output(inv_imu_device_t *s, const inv_imu_selfcal_parameters_t *sc_params,
			      inv_imu_selfcal_output_t *sc_output,
			      inv_imu_selfcal_output_internal_t *sc_output_i)
{
	int rc = INV_IMU_OK;
	uint32_t stc_results;

	/* Read STC results */
	rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_RESULTS, (uint8_t *)&stc_results);

	if (sc_params->accel_en) {
		sc_output->accel_status = ((stc_results & STC_RESULTS_ACCEL_SC_MASK) == 0)
						  ? INV_IMU_SC_STATUS_SUCCESS
						  : INV_IMU_SC_STATUS_FAIL;

		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_ACCEL_SC_NOUT_MEAS_X,
					     (uint8_t *)&(sc_output_i->sc_accel_nout_meas_x));
		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_ACCEL_SC_NOUT_MEAS_Y,
					     (uint8_t *)&(sc_output_i->sc_accel_nout_meas_y));
		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_ACCEL_SC_NOUT1_MEAS_Z,
					     (uint8_t *)&(sc_output_i->sc_accel_nout1_meas_z));
		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_ACCEL_SC_NOUT2_MEAS_Z,
					     (uint8_t *)&(sc_output_i->sc_accel_nout2_meas_z));
		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_ACCEL_SC_CMOS_MEAS_X,
					     (uint8_t *)&(sc_output_i->sc_accel_cmos_meas_x));

		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_18, 2,
					(uint8_t *)&(sc_output_i->sc_accel_nout_trim[0]));
		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_20, 2,
					(uint8_t *)&(sc_output_i->sc_accel_nout_trim[1]));
		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_22, 2,
					(uint8_t *)&(sc_output_i->sc_accel_nout_trim[2]));

	} else {
		sc_output->accel_status = INV_IMU_SC_STATUS_NOT_RUN;
		sc_output_i->sc_accel_nout_meas_x = 0;
		sc_output_i->sc_accel_nout_meas_y = 0;
		sc_output_i->sc_accel_nout1_meas_z = 0;
		sc_output_i->sc_accel_nout2_meas_z = 0;
		sc_output_i->sc_accel_cmos_meas_x = 0;
		sc_output_i->sc_accel_nout_trim[0] = 0;
		sc_output_i->sc_accel_nout_trim[1] = 0;
		sc_output_i->sc_accel_nout_trim[2] = 0;
	}

	if (sc_params->gyro_en) {
		sc_output->gyro_status = ((stc_results & STC_RESULTS_GYRO_SC_MASK) == 0)
						 ? INV_IMU_SC_STATUS_SUCCESS
						 : INV_IMU_SC_STATUS_FAIL;

		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_0, 2,
					(uint8_t *)&(sc_output_i->sc_gyro_str_ft_trim[0]));
		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_2, 2,
					(uint8_t *)&(sc_output_i->sc_gyro_str_ft_trim[1]));
		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_4, 2,
					(uint8_t *)&(sc_output_i->sc_gyro_str_ft_trim[2]));

		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_GAIN_GX,
					     (uint8_t *)&(sc_output_i->sc_gyro_step_response32[0]));
		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_GAIN_GY,
					     (uint8_t *)&(sc_output_i->sc_gyro_step_response32[1]));
		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_GAIN_GZ,
					     (uint8_t *)&(sc_output_i->sc_gyro_step_response32[2]));

		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_6, 2,
					(uint8_t *)&(sc_output_i->sc_gyro_cmos_trim[0]));
		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_8, 2,
					(uint8_t *)&(sc_output_i->sc_gyro_cmos_trim[1]));
		rc |= inv_imu_read_sram(s, IMEM_SRAM_REG_10, 2,
					(uint8_t *)&(sc_output_i->sc_gyro_cmos_trim[2]));

		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_GAIN_GX,
					     (uint8_t *)&(sc_output_i->sc_gyro_gain32[0]));
		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_GAIN_GY,
					     (uint8_t *)&(sc_output_i->sc_gyro_gain32[1]));
		rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_GAIN_GZ,
					     (uint8_t *)&(sc_output_i->sc_gyro_gain32[2]));
	} else {
		sc_output->gyro_status = INV_IMU_SC_STATUS_NOT_RUN;
		sc_output_i->sc_gyro_str_ft_trim[0] = 0;
		sc_output_i->sc_gyro_str_ft_trim[1] = 0;
		sc_output_i->sc_gyro_str_ft_trim[2] = 0;
		sc_output_i->sc_gyro_step_response32[0] = 0;
		sc_output_i->sc_gyro_step_response32[1] = 0;
		sc_output_i->sc_gyro_step_response32[2] = 0;
		sc_output_i->sc_gyro_cmos_trim[0] = 0;
		sc_output_i->sc_gyro_cmos_trim[1] = 0;
		sc_output_i->sc_gyro_cmos_trim[2] = 0;
		sc_output_i->sc_gyro_gain32[0] = 0;
		sc_output_i->sc_gyro_gain32[1] = 0;
		sc_output_i->sc_gyro_gain32[2] = 0;
	}
	return rc;
}

static int run_selfcal(inv_imu_device_t *s, const inv_imu_selfcal_parameters_t *sc_params,
		       const selfcal_gyro_method_t gyro_method, inv_imu_selfcal_output_t *sc_output,
		       inv_imu_selfcal_output_internal_t *sc_output_i)
{
	int rc = INV_IMU_OK;

	rc |= inv_imu_adv_device_reset(s);

	inv_imu_sleep_us(s, 10000);

	/* Configure start addresses as we reset the device */
	rc |= inv_imu_edmp_configure(s);

	rc |= set_selfcal_parameters(s, sc_params, gyro_method);

	rc |= inv_imu_set_accel_fsr(s, ACCEL_CONFIG0_AP_ACCEL_FS_SEL_4_G);

	rc |= run_internal_selfcal(s);

	rc |= get_selfcal_output(s, sc_params, sc_output, sc_output_i);

	rc |= inv_imu_adv_device_reset(s);

	return rc;
}
