/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_defs.h"
#include "imu/inv_imu_edmp_vvd.h"
#include "imu/inv_imu_edmp_algo_defs.h"

#define EDMP_ROM_START_ADDR_IRQ0 EDMP_ROM_BASE
#define EDMP_ROM_START_ADDR_IRQ1 (EDMP_ROM_BASE + 0x04)
#define EDMP_ROM_START_ADDR_IRQ2 (EDMP_ROM_BASE + 0x08)

static int check_dmp_odr_decimation(inv_imu_device_t *s);
static int dynamic_service_request(inv_imu_device_t *s, uint8_t service_id);

int inv_imu_edmp_set_frequency(inv_imu_device_t *s, const dmp_ext_sen_odr_cfg_apex_odr_t frequency)
{
	int                   status = INV_IMU_OK;
	dmp_ext_sen_odr_cfg_t dmp_ext_sen_odr_cfg;

	status |= inv_imu_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);
	dmp_ext_sen_odr_cfg.apex_odr = frequency;
	status |= inv_imu_write_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);

	return status;
}

int inv_imu_edmp_init_apex(inv_imu_device_t *s)
{
	int               status = INV_IMU_OK;
	fifo_sram_sleep_t fifo_sram_sleep;

	/* Configure DMP address registers */
	status |= inv_imu_edmp_configure(s);

	/* Same impl as inv_imu_adv_power_up_sram, duplicated here to prevent dependency */
	status |= inv_imu_read_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	fifo_sram_sleep.fifo_gsleep_shared_sram = 0x03;
	status |= inv_imu_write_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);

	/* Clear SRAM, take advantage of internal memset EDMP service which is faster
	 * than writing each RAM byte through serial interface transaction
	 */
	status |= inv_imu_edmp_write_ram_area(s, EDMP_RAM_BASE, EDMP_ROM_DATA_SIZE, 0);

	/* Init will eventually depend on the usecase, it can now proceed */
	status |= inv_imu_edmp_recompute_apex_decimation(s);

	return status;
}

int inv_imu_edmp_recompute_apex_decimation(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	uint8_t         value;
	edmp_apex_en0_t save_edmp_apex_en0;
	edmp_apex_en1_t save_edmp_apex_en1;
	edmp_apex_en0_t edmp_apex_en0 = { 0 };
	edmp_apex_en1_t edmp_apex_en1 = { 0 };
	reg_host_msg_t  reg_host_msg;

	/*
	 * Check that DMP is turned OFF before requesting init APEX and save DMP enabled bits before
	 * requesting init procedure
	 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);
	if (save_edmp_apex_en1.edmp_enable)
		return INV_IMU_ERROR;

	/*
	 * Make sure that all DMP interrupts are masked by default, to not trigger unexpected algorithm
	 *  execution when initialization is done if any sensor is running
	 */
	value = 0x3F;
	status |= inv_imu_write_reg(s, STATUS_MASK_PIN_0_7, 1, &value);
	status |= inv_imu_write_reg(s, STATUS_MASK_PIN_8_15, 1, &value);
	status |= inv_imu_write_reg(s, STATUS_MASK_PIN_16_23, 1, &value);

	/* Trigger EDMP with on-demand mode */
	status |= inv_imu_edmp_unmask_int_src(s, INV_IMU_EDMP_INT1, EDMP_INT_SRC_ON_DEMAND_MASK);

	/*
	 * Request to execute init procedure, make sure init is the only feature enabled
	 * (overwrite previously saved config)
	 */
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en1.init_en     = INV_IMU_ENABLE;
	edmp_apex_en1.edmp_enable = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	status |= inv_imu_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.edmp_on_demand_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	/* Wait 200 us to give enough time for EMDP to start running */
	inv_imu_sleep_us(s, 200);

	/* Wait for DMP execution to complete */
	status |= inv_imu_edmp_wait_for_idle(s);

	/* Reset states */
	status |= inv_imu_edmp_mask_int_src(s, INV_IMU_EDMP_INT1, EDMP_INT_SRC_ON_DEMAND_MASK);

	/* Restore original DMP state, with DMP necessarily disabled as it was checked at the beginning of this function */
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);

	status |= inv_imu_edmp_unmask_int_src(
	    s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK | EDMP_INT_SRC_GYRO_DRDY_MASK);

	return status;
}

static int dynamic_service_request(inv_imu_device_t *s, uint8_t service_id)
{
	int status = INV_IMU_OK;

	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ONDEMAND_DYNAMIC_SERVICE_REQUEST, (uint8_t *)&service_id);

	/* Trigger EDMP with on-demand mode */
	status |= inv_imu_edmp_run_ondemand(s, INV_IMU_EDMP_INT1);

	/* Wait 200 us to give enough time for EMDP to start running */
	inv_imu_sleep_us(s, 200);

	/* Wait for DMP execution to complete */
	status |= inv_imu_edmp_wait_for_idle(s);

	return status;
}

int inv_imu_edmp_get_apex_parameters(inv_imu_device_t *s, inv_imu_edmp_apex_parameters_t *p)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	/* AID */
	/* -- human instance -- */
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_WIN_HUMAN, (uint8_t *)&p->aid_human.aid_update_win);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_ALERT_HUMAN, (uint8_t *)&p->aid_human.aid_alert_timer);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_EN_OUTPUT_HUMAN,
	                                 (uint8_t *)&p->aid_human.aid_output_enable);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_DIS_MULTI_OUTPUT_HUMAN,
	                                 (uint8_t *)&p->aid_human.aid_disable_multiple_interrupt);
	/* -- device instance -- */
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_WIN_DEVICE, (uint8_t *)&p->aid_device.aid_update_win);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_ALERT_DEVICE, (uint8_t *)&p->aid_device.aid_alert_timer);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_EN_OUTPUT_DEVICE,
	                                 (uint8_t *)&p->aid_device.aid_output_enable);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_DIS_MULTI_OUTPUT_DEVICE,
	                                 (uint8_t *)&p->aid_device.aid_disable_multiple_interrupt);

	/* Freefall */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH, (uint8_t *)&p->lowg_peak_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH_HYST, (uint8_t *)&p->lowg_peak_th_hyst);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_TIME_TH, (uint8_t *)&p->lowg_time_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH, (uint8_t *)&p->highg_peak_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH_HYST, (uint8_t *)&p->highg_peak_th_hyst);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_TIME_TH, (uint8_t *)&p->highg_time_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_MIN_DURATION, (uint8_t *)&p->ff_min_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_MAX_DURATION, (uint8_t *)&p->ff_max_duration);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DEBOUNCE_DURATION, (uint8_t *)&p->ff_debounce_duration);

	/* Tap */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MIN_JERK, (uint8_t *)&p->tap_min_jerk);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TMAX, (uint8_t *)&p->tap_tmax);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TMIN, (uint8_t *)&p->tap_tmin);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MAX, (uint8_t *)&p->tap_max);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MIN, (uint8_t *)&p->tap_min);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MAX_PEAK_TOL, (uint8_t *)&p->tap_max_peak_tol);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_SMUDGE_REJECT_THR, (uint8_t *)&p->tap_smudge_reject_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TAVG, (uint8_t *)&p->tap_tavg);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_ODR, (uint8_t *)&p->tap_odr);

	/* B2S */
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_MOUNTING_MATRIX, (uint8_t *)&p->b2s_mounting_matrix);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_DEV_NORM_MAX, (uint8_t *)&p->b2s_dev_norm_max);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_SIN_LIMIT, (uint8_t *)&p->b2s_sin_limit);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_FAST_LIMIT, (uint8_t *)&p->b2s_fast_limit);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_STATIC_LIMIT, (uint8_t *)&p->b2s_static_limit);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_THR_COS_ANG, (uint8_t *)&p->b2s_thr_cos_ang);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_REV_X_LIMIT, (uint8_t *)&p->b2s_rev_x_limit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_SIN_FLAT_ANGLE,
	                                 (uint8_t *)&p->b2s_sin_flat_angle);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_TIMER_FLAT_REJECT,
	                                 (uint8_t *)&p->b2s_timer_flat_reject);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_FAST_MOTION_AGE_LIMIT,
	                                 (uint8_t *)&p->b2s_fast_motion_age_limit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_FAST_MOTION_TIME_LIMIT,
	                                 (uint8_t *)&p->b2s_fast_motion_time_limit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_AGE_LIMIT, (uint8_t *)&p->b2s_age_limit);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_SETTINGS_REV_LATENCY_TH,
	                                 (uint8_t *)&p->b2s_rev_latency_th);

	/* Power save */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_POWER_SAVE_TIME, (uint8_t *)&p->power_save_time);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	p->power_save_en = edmp_apex_en1.power_save_en ? INV_IMU_ENABLE : INV_IMU_DISABLE;

	return status;
}

int inv_imu_edmp_set_apex_parameters(inv_imu_device_t *s, const inv_imu_edmp_apex_parameters_t *p)
{
	int             status = INV_IMU_OK;
	edmp_apex_enx_t cfg;
	int32_t         one_g_value;
	int32_t         b2s_limit_inf;
	int32_t         b2s_limit_sup;

	/* DMP cannot be configured if it is running, hence make sure all APEX algorithms are off */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 2, (uint8_t *)&cfg);
	if (cfg.edmp_apex_en0.b2s_en || cfg.edmp_apex_en0.ff_en || cfg.edmp_apex_en0.aid_en ||
	    cfg.edmp_apex_en0.tap_en)
		return INV_IMU_ERROR;

	/* AID */
	/* -- human instance -- */
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_AID_WIN_HUMAN, (uint8_t *)&p->aid_human.aid_update_win);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_AID_ALERT_HUMAN, (uint8_t *)&p->aid_human.aid_alert_timer);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_AID_EN_OUTPUT_HUMAN,
	                                  (uint8_t *)&p->aid_human.aid_output_enable);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_AID_DIS_MULTI_OUTPUT_HUMAN,
	                                  (uint8_t *)&p->aid_human.aid_disable_multiple_interrupt);
	/* -- device instance -- */
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_AID_WIN_DEVICE, (uint8_t *)&p->aid_device.aid_update_win);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_AID_ALERT_DEVICE,
	                                  (uint8_t *)&p->aid_device.aid_alert_timer);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_AID_EN_OUTPUT_DEVICE,
	                                  (uint8_t *)&p->aid_device.aid_output_enable);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_AID_DIS_MULTI_OUTPUT_DEVICE,
	                                  (uint8_t *)&p->aid_device.aid_disable_multiple_interrupt);

	/* Free Fall */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH, (uint8_t *)&p->lowg_peak_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH_HYST, (uint8_t *)&p->lowg_peak_th_hyst);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_TIME_TH, (uint8_t *)&p->lowg_time_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH, (uint8_t *)&p->highg_peak_th);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH_HYST, (uint8_t *)&p->highg_peak_th_hyst);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_TIME_TH, (uint8_t *)&p->highg_time_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_MIN_DURATION, (uint8_t *)&p->ff_min_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_MAX_DURATION, (uint8_t *)&p->ff_max_duration);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_DEBOUNCE_DURATION, (uint8_t *)&p->ff_debounce_duration);

	/* Tap */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MIN_JERK, (uint8_t *)&p->tap_min_jerk);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TMAX, (uint8_t *)&p->tap_tmax);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TMIN, (uint8_t *)&p->tap_tmin);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MAX, (uint8_t *)&p->tap_max);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MIN, (uint8_t *)&p->tap_min);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MAX_PEAK_TOL, (uint8_t *)&p->tap_max_peak_tol);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_SMUDGE_REJECT_THR, (uint8_t *)&p->tap_smudge_reject_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TAVG, (uint8_t *)&p->tap_tavg);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_ODR, (uint8_t *)&p->tap_odr);

	/* B2S */
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_MOUNTING_MATRIX, (uint8_t *)&p->b2s_mounting_matrix);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_DEV_NORM_MAX, (uint8_t *)&p->b2s_dev_norm_max);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_SIN_LIMIT, (uint8_t *)&p->b2s_sin_limit);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_FAST_LIMIT, (uint8_t *)&p->b2s_fast_limit);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_STATIC_LIMIT, (uint8_t *)&p->b2s_static_limit);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_THR_COS_ANG, (uint8_t *)&p->b2s_thr_cos_ang);
	/* Derive internal parameters */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_ONE_G_VALUE, (uint8_t *)&one_g_value);
	b2s_limit_inf = (one_g_value - p->b2s_dev_norm_max) * (one_g_value - p->b2s_dev_norm_max);
	b2s_limit_sup = (one_g_value + p->b2s_dev_norm_max) * (one_g_value + p->b2s_dev_norm_max);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_LIMIT_INF, (uint8_t *)&b2s_limit_inf);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_LIMIT_SUP, (uint8_t *)&b2s_limit_sup);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_REV_X_LIMIT, (uint8_t *)&p->b2s_rev_x_limit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_SIN_FLAT_ANGLE,
	                                  (uint8_t *)&p->b2s_sin_flat_angle);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_TIMER_FLAT_REJECT,
	                                  (uint8_t *)&p->b2s_timer_flat_reject);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_FAST_MOTION_AGE_LIMIT,
	                                  (uint8_t *)&p->b2s_fast_motion_age_limit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_FAST_MOTION_TIME_LIMIT,
	                                  (uint8_t *)&p->b2s_fast_motion_time_limit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_AGE_LIMIT, (uint8_t *)&p->b2s_age_limit);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_SETTINGS_REV_LATENCY_TH,
	                                  (uint8_t *)&p->b2s_rev_latency_th);

	/* Power save */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_POWER_SAVE_TIME, (uint8_t *)&p->power_save_time);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&cfg.edmp_apex_en1);
	cfg.edmp_apex_en1.power_save_en = p->power_save_en;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&cfg.edmp_apex_en1);

	return status;
}

int inv_imu_edmp_get_gaf_parameters(inv_imu_device_t *s, inv_imu_edmp_gaf_parameters_t *gaf_params)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_PLL_CLOCK_VARIATION,
	                                 (uint8_t *)&gaf_params->clock_variation);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_GYR_PDR_US, (uint8_t *)&gaf_params->pdr_us);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_STATIONARY_ANGLE_ENABLE,
	                                 (uint8_t *)&gaf_params->stationary_angle_enable);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_STATIONARY_ANGLE_DURATION_US,
	                                 (uint8_t *)&gaf_params->stationary_angle_duration_us);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_STATIONARY_ANGLE_THRESHOLD_DEG_Q16,
	                                 (uint8_t *)&gaf_params->stationary_angle_threshold_deg_q16);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_FUS_LOW_SPEED_DRIFT_ROLL_PITCH,
	                                 (uint8_t *)&gaf_params->fus_low_speed_drift_roll_pitch);

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_LOOSE_GYR_CAL_STATIONARY_DURATION_US,
	                                 (uint8_t *)&gaf_params->gyr_cal_stationary_duration_us);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_LOOSE_GYR_CAL_THRESHOLD_METRIC1,
	                                 (uint8_t *)&gaf_params->gyr_cal_threshold_metric1);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_LOOSE_GYR_CAL_THRESHOLD_METRIC2,
	                                 (uint8_t *)&gaf_params->gyr_cal_threshold_metric2);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_STRICT_GYR_CAL_THRESHOLD_METRIC1,
	                                 (uint8_t *)&gaf_params->strict_gyr_cal_threshold_metric1);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_STRICT_GYR_CAL_THRESHOLD_METRIC2,
	                                 (uint8_t *)&gaf_params->strict_gyr_cal_threshold_metric2);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_LOOSE_GYR_CAL_SAMPLE_NUM_LOG2,
	                                 (uint8_t *)&gaf_params->gyr_cal_sample_num_log2);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_GYR_BIAS_REJECT_TH,
	                                 (uint8_t *)&gaf_params->gyr_bias_reject_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_ACC_FILTERING_NPOINTS_LOG2,
	                                 (uint8_t *)&gaf_params->acc_filtering_Npoints_log2);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_ACC_SQUARE_SIN_ANGLE_MOTION_DETECT_TH,
	                                 (uint8_t *)&gaf_params->acc_square_sin_angle_motion_detect_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_GOLDEN_BIAS_TIMER,
	                                 (uint8_t *)&gaf_params->golden_bias_timer);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_GOLDEN_BIAS_TEMPERATURE_VALIDITY,
	                                 (uint8_t *)&gaf_params->golden_bias_temperature_validity);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_FUS_HIGH_SPEED_DRIFT,
	                                 (uint8_t *)&gaf_params->fus_high_speed_drift);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_FUS_MEASUREMENT_COVARIANCE_ACC,
	                                 (uint8_t *)&gaf_params->fus_measurement_covariance_acc);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_CONFIG_FUS_ACCELERATION_REJECTION,
	                                 (uint8_t *)&gaf_params->fus_acceleration_rejection);

	return status;
}

int inv_imu_edmp_set_gaf_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_gaf_parameters_t *gaf_params)
{
	int             status = INV_IMU_OK;
	accel_config0_t accel_config0;
	gyro_config0_t  gyro_config0;
	uint32_t        acc_odr_us;
	uint32_t        gyr_odr_us;
	EDMP_GAF_PDR_400HZ_DECLARE_ARRAY(partitions_gaf_pdr_400hz);

	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
	switch (accel_config0.accel_odr) {
	case ACCEL_CONFIG0_ACCEL_ODR_50_HZ:
		acc_odr_us = 20000;
		break;
	case ACCEL_CONFIG0_ACCEL_ODR_100_HZ:
		acc_odr_us = 10000;
		break;
	case ACCEL_CONFIG0_ACCEL_ODR_200_HZ:
		acc_odr_us = 5000;
		break;
	case ACCEL_CONFIG0_ACCEL_ODR_400_HZ:
		acc_odr_us = 2500;
		break;
	case ACCEL_CONFIG0_ACCEL_ODR_800_HZ:
		acc_odr_us = 1250;
		break;
	default:
		return INV_IMU_ERROR;
	}
	status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
	switch (gyro_config0.gyro_odr) {
	case GYRO_CONFIG0_GYRO_ODR_50_HZ:
		gyr_odr_us = 20000;
		break;
	case GYRO_CONFIG0_GYRO_ODR_100_HZ:
		gyr_odr_us = 10000;
		break;
	case GYRO_CONFIG0_GYRO_ODR_200_HZ:
		gyr_odr_us = 5000;
		break;
	case GYRO_CONFIG0_GYRO_ODR_400_HZ:
		gyr_odr_us = 2500;
		break;
	case GYRO_CONFIG0_GYRO_ODR_800_HZ:
		gyr_odr_us = 1250;
		break;
	default:
		return INV_IMU_ERROR;
	}

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_PLL_CLOCK_VARIATION,
	                                  (uint8_t *)&gaf_params->clock_variation);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_ACC_ODR_US, (uint8_t *)&acc_odr_us);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_ACC_PDR_US, (uint8_t *)&gaf_params->pdr_us);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_GYR_ODR_US, (uint8_t *)&gyr_odr_us);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_GYR_PDR_US, (uint8_t *)&gaf_params->pdr_us);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_STATIONARY_ANGLE_ENABLE,
	                                  (uint8_t *)&gaf_params->stationary_angle_enable);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_STATIONARY_ANGLE_DURATION_US,
	                                  (uint8_t *)&gaf_params->stationary_angle_duration_us);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_STATIONARY_ANGLE_THRESHOLD_DEG_Q16,
	                                  (uint8_t *)&gaf_params->stationary_angle_threshold_deg_q16);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_FUS_LOW_SPEED_DRIFT_ROLL_PITCH,
	                                  (uint8_t *)&gaf_params->fus_low_speed_drift_roll_pitch);

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_LOOSE_GYR_CAL_STATIONARY_DURATION_US,
	                                  (uint8_t *)&gaf_params->gyr_cal_stationary_duration_us);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_LOOSE_GYR_CAL_THRESHOLD_METRIC1,
	                                  (uint8_t *)&gaf_params->gyr_cal_threshold_metric1);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_LOOSE_GYR_CAL_THRESHOLD_METRIC2,
	                                  (uint8_t *)&gaf_params->gyr_cal_threshold_metric2);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_STRICT_GYR_CAL_THRESHOLD_METRIC1,
	                                  (uint8_t *)&gaf_params->strict_gyr_cal_threshold_metric1);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_STRICT_GYR_CAL_THRESHOLD_METRIC2,
	                                  (uint8_t *)&gaf_params->strict_gyr_cal_threshold_metric2);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_LOOSE_GYR_CAL_SAMPLE_NUM_LOG2,
	                                  (uint8_t *)&gaf_params->gyr_cal_sample_num_log2);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_GYR_BIAS_REJECT_TH,
	                                  (uint8_t *)&gaf_params->gyr_bias_reject_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_ACC_FILTERING_NPOINTS_LOG2,
	                                  (uint8_t *)&gaf_params->acc_filtering_Npoints_log2);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_ACC_SQUARE_SIN_ANGLE_MOTION_DETECT_TH,
	                            (uint8_t *)&gaf_params->acc_square_sin_angle_motion_detect_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_GOLDEN_BIAS_TIMER,
	                                  (uint8_t *)&gaf_params->golden_bias_timer);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_GOLDEN_BIAS_TEMPERATURE_VALIDITY,
	                                  (uint8_t *)&gaf_params->golden_bias_temperature_validity);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_FUS_HIGH_SPEED_DRIFT,
	                                  (uint8_t *)&gaf_params->fus_high_speed_drift);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_FUS_MEASUREMENT_COVARIANCE_ACC,
	                                  (uint8_t *)&gaf_params->fus_measurement_covariance_acc);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_CONFIG_FUS_ACCELERATION_REJECTION,
	                                  (uint8_t *)&gaf_params->fus_acceleration_rejection);

	/* Run on-demand service to compute complex parameters based on customer simplified parameters */
	/* Complex parameters first */
	status |= dynamic_service_request(s, EDMP_ONDEMAND_ENABLE_GAF_PARAM);
	/* Then bias */
	status |= dynamic_service_request(s, EDMP_ONDEMAND_ENABLE_GAF_BIAS);
	/* Reset states */
	status |= inv_imu_edmp_mask_int_src(s, INV_IMU_EDMP_INT1, EDMP_INT_SRC_ON_DEMAND_MASK);
	status |= inv_imu_edmp_disable(s);

	if (gaf_params->pdr_us == 2500) {
		status |= inv_imu_write_sram(
		    s, EDMP_GAF_PDR_PARTITION,
		    sizeof(partitions_gaf_pdr_400hz[DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ]),
		    (const uint8_t *)partitions_gaf_pdr_400hz[DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ]);
	}

	return status;
}

int inv_imu_edmp_get_gaf_gyr_bias(inv_imu_device_t *s, int16_t gyr_bias_q12[3],
                                  int32_t *gyr_bias_temperature, uint8_t *accuracy)
{
	const int64_t gyr_2000dps_q30_to_1dps_q12 = 8192000; /* 2000 << 12 */
	int32_t       gyr_bias[3];
	int           status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_SAVED_GYR_BIAS_DPS_Q12, (uint8_t *)gyr_bias);
	/* Convert from internal format to FIFO format, so it can be reapplied afterwards with inv_imu_edmp_set_gaf_gyr_bias() */
	for (uint8_t i = 0; i < 3; i++) {
		int64_t tmp     = (((int64_t)gyr_bias[i] * gyr_2000dps_q30_to_1dps_q12) + (1 << 29)) >> 30;
		gyr_bias_q12[i] = (int16_t)tmp;
	}
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_SAVED_GYR_ACCURACY, (uint8_t *)accuracy);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_SAVED_GYR_BIAS_TEMPERATURE_DEG_Q16,
	                                 (uint8_t *)gyr_bias_temperature);

	return status;
}

int inv_imu_edmp_set_gaf_gyr_bias(inv_imu_device_t *s, const int16_t gyr_bias_q12[3],
                                  const int32_t gyr_bias_temperature, const uint8_t accuracy)
{
	int status = INV_IMU_OK;
	/* convert 16bits gyro bias value to 32bits */
	int32_t gyr_bias[3] = { gyr_bias_q12[0], gyr_bias_q12[1], gyr_bias_q12[2] };

	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_SAVED_GYR_BIAS_DPS_Q12, (const uint8_t *)gyr_bias);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_SAVED_GYR_ACCURACY, (const uint8_t *)&accuracy);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_SAVED_GYR_BIAS_TEMPERATURE_DEG_Q16,
	                                  (uint8_t *)&gyr_bias_temperature);

	return status;
}

int inv_imu_edmp_set_gaf_acc_bias(inv_imu_device_t *s, const int32_t acc_bias_q16[3])
{
	const uint8_t accuracy = 3;
	int           status   = INV_IMU_OK;

	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_SAVED_ACC_BIAS_1G_Q16, (const uint8_t *)acc_bias_q16);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_SAVED_ACC_ACCURACY, (const uint8_t *)&accuracy);

	return status;
}

int inv_imu_edmp_set_sif_model(inv_imu_device_t *s, const inv_imu_edmp_sif_user_config_t *cfg)
{
	int                   status      = INV_IMU_OK;
	uint32_t              nb_features = 0;
	config_common_t       cconfig;
	config_time_t         tconfig;
	dmp_ext_sen_odr_cfg_t dmp_ext_sen_odr_cfg;

	status |= inv_imu_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);

	/* Clear SRAM, take advantage of internal memset EDMP service which is faster
	 * than writing each RAM byte through serial interface transaction
	 */
	status |= inv_imu_edmp_write_ram_area(s, EDMP_SIF_TIME_FEAS_CONFIG,
	                                      EDMP_ROM_DATA_SIZE - EDMP_SIF_TIME_FEAS_CONFIG, 0);

	/* Initialize the SIF time domain feature configuration matrix */
	for (uint8_t k = 0; k < cfg->acc_t_config_num; k++) {
		uint16_t time_feas_config = cfg->acc_temporal_feas_config[k * ACC_T_CONFIG_NUM_MAX + 1];
		for (uint8_t i = 2; i < ACC_T_CONFIG_NUM_MAX; i++) {
			if (cfg->acc_temporal_feas_config[k * ACC_T_CONFIG_NUM_MAX + i])
				time_feas_config |= 1 << i;
		}
		/* Load SIF data memory to EDMP RAM */
		status |=
		    inv_imu_write_sram(s, EDMP_SIF_TIME_FEAS_CONFIG + k * EDMP_SIF_TIME_FEAS_CONFIG_SIZE,
		                       EDMP_SIF_TIME_FEAS_CONFIG_SIZE, (uint8_t *)&time_feas_config);
	}

	/* Initialize the SIF feature extraction filter */
	for (int32_t k = 0; k < cfg->acc_t_config_num; k++) {
		filter_state_t sif_filter = { 0 };
		uint32_t       idx = MAX_FLT_COEF * cfg->acc_temporal_feas_config[k * ACC_T_CONFIG_NUM_MAX];
		SIF_Filter_Init(&sif_filter, MAX_FLT_COEF, (const int32_t *)&cfg->acc_t_filtbna_q28[idx]);
		/* Load SIF data memory to EDMP RAM */
		status |= inv_imu_write_sram(s, EDMP_SIF_FILTER + k * EDMP_SIF_FILTER_SIZE,
		                             EDMP_SIF_FILTER_SIZE, (uint8_t *)&sif_filter);
	}

	/* Initialize the SIF feature extraction configurations */
	SIF_CommonConfig_Init(&cconfig, (int32_t)cfg->wind_size_sample, (int32_t)cfg->acc_t_config_num);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SIF_CCONFIG, (uint8_t *)&cconfig);
	SIF_TemporalConfig_Init(&tconfig, (int32_t)cfg->inv_data_wind, (int32_t)cfg->acc_hyst_thr_q16,
	                        (int32_t)cfg->acc_min_peak_distance,
	                        (int32_t)cfg->acc_min_peak_height_q16);
	switch ((dmp_ext_sen_odr_cfg_apex_odr_t)dmp_ext_sen_odr_cfg.apex_odr) {
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ:
		tconfig.acc_odr_us = 20000;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ:
		tconfig.acc_odr_us = 10000;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ:
		tconfig.acc_odr_us = 5000;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ:
		tconfig.acc_odr_us = 2500;
		break;
	case DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ:
		tconfig.acc_odr_us = 1250;
		break;
	default:
		/* Set a value for acc_odr_us not to have random, but it does not matter since it is an error case (25 Hz not supported) */
		tconfig.acc_odr_us = 1000000 / cfg->sif_odr;
		status |= INV_IMU_ERROR_EDMP_ODR;
		break;
	}
	tconfig.sif_pdr_us = 1000000 / cfg->sif_odr;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SIF_TCONFIG, (uint8_t *)&tconfig);

	/* Initialize the SIF tree structure */
	for (int32_t i = 0; i < cfg->node_size; i++) {
		/* Load SIF data memory to EDMP RAM */
		status |=
		    inv_imu_write_sram(s, EDMP_SIF_TREE_THRESHOLDS + i * EDMP_SIF_TREE_THRESHOLDS_SIZE,
		                       EDMP_SIF_TREE_THRESHOLDS_SIZE,
		                       (uint8_t *)&cfg->tree.decisionTreeThresholds[i]);
		status |=
		    inv_imu_write_sram(s, EDMP_SIF_TREE_FEATUREIDS + i * EDMP_SIF_TREE_FEATUREIDS_SIZE,
		                       EDMP_SIF_TREE_FEATUREIDS_SIZE,
		                       (uint8_t *)&cfg->tree.decisionTreeFeatureIDs[i]);
		status |= inv_imu_write_sram(
		    s, EDMP_SIF_TREE_NEXTNODERIGHT + i * EDMP_SIF_TREE_NEXTNODERIGHT_SIZE,
		    EDMP_SIF_TREE_NEXTNODERIGHT_SIZE, (uint8_t *)&cfg->tree.decisionTreeNextNodeRight[i]);
		if (cfg->tree.decisionTreeFeatureIDs[i] > nb_features)
			nb_features = cfg->tree.decisionTreeFeatureIDs[i];
	}
	for (uint32_t i = 0; i < (nb_features + 1); i++) {
		status |= inv_imu_write_sram(
		    s, EDMP_SIF_TREE_THRESHOLDSSHIFT + i * EDMP_SIF_TREE_THRESHOLDSSHIFT_SIZE,
		    EDMP_SIF_TREE_THRESHOLDSSHIFT_SIZE,
		    (uint8_t *)&cfg->tree.decisionTreeThresholdsShift[i]);
	}

	return status;
}

int inv_imu_edmp_set_sif_pdr(inv_imu_device_t *s, uint8_t pdr50_or_100hz)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t save_edmp_apex_en0;
	edmp_apex_en1_t save_edmp_apex_en1;

	/*
	 * Check that DMP is turned OFF before requesting set SIF PDR service
	 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);
	if (save_edmp_apex_en1.edmp_enable)
		return INV_IMU_ERROR;

	// Run on-demand service to compute complex parameters based on requested SIF PDR
	if (pdr50_or_100hz)
		status |= dynamic_service_request(s, EDMP_ONDEMAND_ENABLE_ML_PDR_100HZ);
	else
		status |= dynamic_service_request(s, EDMP_ONDEMAND_ENABLE_ML_PDR_50HZ);

	/* Reset states */
	status |= inv_imu_edmp_mask_int_src(s, INV_IMU_EDMP_INT1, EDMP_INT_SRC_ON_DEMAND_MASK);

	/* Restore original DMP state, with DMP necessarily disabled as it was checked at the beginning of this function */
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);

	return status;
}

int inv_imu_edmp_set_vvd_model(inv_imu_device_t *s)
{
	int      status          = INV_IMU_OK;
	uint32_t number_features = 0;

	if (NODE_SIZE_VVD > EDMP_VVD_MAX_NODE_SIZE)
		return -1;

	/* Initialize the VVD tree structure */
	for (int32_t i = 0; i < NODE_SIZE_VVD; i++) {
		/* Load SIF data memory to EDMP RAM */
		status |=
		    inv_imu_write_sram(s, EDMP_VVD_TREE_THRESHOLDS + i * EDMP_VVD_TREE_THRESHOLDS_SIZE,
		                       EDMP_VVD_TREE_THRESHOLDS_SIZE,
		                       (uint8_t *)&decisionTreeThresholdsVVD[i]);
		status |=
		    inv_imu_write_sram(s, EDMP_VVD_TREE_FEATUREIDS + i * EDMP_VVD_TREE_FEATUREIDS_SIZE,
		                       EDMP_VVD_TREE_FEATUREIDS_SIZE,
		                       (uint8_t *)&decisionTreeFeatureIDsVVD[i]);
		status |= inv_imu_write_sram(
		    s, EDMP_VVD_TREE_NEXTNODERIGHT + i * EDMP_VVD_TREE_NEXTNODERIGHT_SIZE,
		    EDMP_VVD_TREE_NEXTNODERIGHT_SIZE, (uint8_t *)&decisionTreeNextNodeRightVVD[i]);
		if (decisionTreeFeatureIDsVVD[i] > number_features)
			number_features = decisionTreeFeatureIDsVVD[i];
	}

	if ((number_features + 1) > EDMP_VVD_MAX_FEATURE_IDS)
		return -1;

	for (uint32_t i = 0; i < (number_features + 1); i++) {
		status |= inv_imu_write_sram(
		    s, EDMP_VVD_TREE_THRESHOLDSSHIFT + i * EDMP_VVD_TREE_THRESHOLDSSHIFT_SIZE,
		    EDMP_VVD_TREE_THRESHOLDSSHIFT_SIZE, (uint8_t *)&decisionTreeThresholdsShiftVVD[i]);
	}

	return status;
}

int inv_imu_edmp_get_vvd_parameters(inv_imu_device_t *s, inv_imu_edmp_vvd_parameters_t *vvd_params)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_VVD_PARAMS_DELAY_HIGH,
	                                 (uint8_t *)&vvd_params->delayBeforeHighVVDTrig);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_VVD_PARAMS_DELAY_LOW,
	                                 (uint8_t *)&vvd_params->delayBeforeLowVVDTrig);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_VVD_PARAMS_SAMPLE_CNT,
	                                 (uint8_t *)&vvd_params->nbSamplesBeforeDecision);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_VVD_PARAMS_BEST_AXIS, (uint8_t *)vvd_params->bestAxis);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_VVD_PARAMS_THRESH, (uint8_t *)&vvd_params->dynEnvVVDThresh);

	return status;
}

int inv_imu_edmp_set_vvd_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_vvd_parameters_t *vvd_params)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_VVD_PARAMS_DELAY_HIGH,
	                                  (uint8_t *)&vvd_params->delayBeforeHighVVDTrig);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_VVD_PARAMS_DELAY_LOW,
	                                  (uint8_t *)&vvd_params->delayBeforeLowVVDTrig);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_VVD_PARAMS_SAMPLE_CNT,
	                                  (uint8_t *)&vvd_params->nbSamplesBeforeDecision);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_VVD_PARAMS_BEST_AXIS, (uint8_t *)vvd_params->bestAxis);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_VVD_PARAMS_THRESH, (uint8_t *)&vvd_params->dynEnvVVDThresh);

	return status;
}

int inv_imu_edmp_set_mounting_matrix(inv_imu_device_t *s, const int8_t mounting_matrix[9])
{
	int           status                  = INV_IMU_OK;
	const int16_t edmp_mounting_matrix[9] = {
		(int16_t)mounting_matrix[0] << 14, (int16_t)mounting_matrix[1] << 14,
		(int16_t)mounting_matrix[2] << 14, (int16_t)mounting_matrix[3] << 14,
		(int16_t)mounting_matrix[4] << 14, (int16_t)mounting_matrix[5] << 14,
		(int16_t)mounting_matrix[6] << 14, (int16_t)mounting_matrix[7] << 14,
		(int16_t)mounting_matrix[8] << 14,
	};
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GLOBAL_MOUNTING_MATRIX, (uint8_t *)&edmp_mounting_matrix);

	return status;
}

int inv_imu_edmp_get_config_int_apex(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it)
{
	int                status = INV_IMU_OK;
	int_apex_configx_t cfg;

	status |= inv_imu_read_reg(s, INT_APEX_CONFIG0, 2, (uint8_t *)&cfg);
	/* INT_APEX_CONFIG0 */
	it->INV_TAP        = !cfg.int_apex_config0.int_status_mask_pin_tap_detect;
	it->INV_HIGHG      = !cfg.int_apex_config0.int_status_mask_pin_high_g_det;
	it->INV_LOWG       = !cfg.int_apex_config0.int_status_mask_pin_low_g_det;
	it->INV_SIF        = !cfg.int_apex_config0.int_status_mask_pin_sif_det;
	it->INV_AID_HUMAN  = !cfg.int_apex_config0.int_status_mask_pin_aid_human_det;
	it->INV_AID_DEVICE = !cfg.int_apex_config0.int_status_mask_pin_aid_device_det;
	it->INV_FF         = !cfg.int_apex_config0.int_status_mask_pin_ff_det;
	it->INV_B2S        = !cfg.int_apex_config0.int_status_mask_pin_b2s_det;

	/* INT_APEX_CONFIG1 */
	it->INV_B2S_REV   = !cfg.int_apex_config1.int_status_mask_pin_b2s_rev_det;
	it->INV_VVD       = !cfg.int_apex_config1.int_status_mask_pin_vvd_det;
	it->INV_SELF_TEST = !cfg.int_apex_config1.int_status_mask_pin_selftest_done;
	it->INV_SEC_AUTH  = !cfg.int_apex_config1.int_status_mask_pin_sa_done;

	return status;
}

int inv_imu_edmp_set_config_int_apex(inv_imu_device_t *s, const inv_imu_edmp_int_state_t *it)
{
	int                status = INV_IMU_OK;
	int_apex_configx_t cfg    = { 0 };

	/* INT_APEX_CONFIG0 */
	cfg.int_apex_config0.int_status_mask_pin_tap_detect     = !it->INV_TAP;
	cfg.int_apex_config0.int_status_mask_pin_high_g_det     = !it->INV_HIGHG;
	cfg.int_apex_config0.int_status_mask_pin_low_g_det      = !it->INV_LOWG;
	cfg.int_apex_config0.int_status_mask_pin_sif_det        = !it->INV_SIF;
	cfg.int_apex_config0.int_status_mask_pin_aid_human_det  = !it->INV_AID_HUMAN;
	cfg.int_apex_config0.int_status_mask_pin_aid_device_det = !it->INV_AID_DEVICE;
	cfg.int_apex_config0.int_status_mask_pin_ff_det         = !it->INV_FF;
	cfg.int_apex_config0.int_status_mask_pin_b2s_det        = !it->INV_B2S;

	/* INT_APEX_CONFIG1 */
	cfg.int_apex_config1.int_status_mask_pin_b2s_rev_det   = !it->INV_B2S_REV;
	cfg.int_apex_config1.int_status_mask_pin_vvd_det       = !it->INV_VVD;
	cfg.int_apex_config1.int_status_mask_pin_selftest_done = !it->INV_SELF_TEST;
	cfg.int_apex_config1.int_status_mask_pin_sa_done       = !it->INV_SEC_AUTH;

	status |= inv_imu_write_reg(s, INT_APEX_CONFIG0, 2, (uint8_t *)&cfg);

	return status;
}

int inv_imu_edmp_enable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.edmp_enable = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_disable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.edmp_enable = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	/* eDMP will be disabled once current processing is done, wait for idle then */
	status |= inv_imu_edmp_wait_for_idle(s);

	return status;
}

/*
 * Check if inv_imu_edmp_set_frequency() was called without recomputing APEX decimation
 * thanks to inv_imu_edmp_recompute_apex_decimation().
 * This function will compare edmp decimation rate in IMU SRAM as computed at APEX init step vs the
 * DMP ODR currently written in IMU register.
 * Returns INV_IMU_ERROR_EDMP_ODR if inv_imu_edmp_recompute_apex_decimation() should have been
 * called.
 */
static int check_dmp_odr_decimation(inv_imu_device_t *s)
{
	int                            status = INV_IMU_OK;
	uint32_t                       dmp_decim_rate_from_sram;
	dmp_ext_sen_odr_cfg_t          dmp_ext_sen_odr_cfg;
	dmp_ext_sen_odr_cfg_apex_odr_t apex_odr;

	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_DMP_ODR_LAST_INIT, (uint8_t *)&dmp_decim_rate_from_sram);
	status |= inv_imu_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);
	apex_odr = (dmp_ext_sen_odr_cfg_apex_odr_t)dmp_ext_sen_odr_cfg.apex_odr;
	switch (dmp_decim_rate_from_sram) {
	case 0x80000000:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 0x8000:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 0x80:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 0x8:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 0x2:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 0x1:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_25_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	default:
		return INV_IMU_ERROR;
	}
}

int inv_imu_edmp_enable_aid(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.aid_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	return status;
}

int inv_imu_edmp_disable_aid(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.aid_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	return status;
}

int inv_imu_edmp_enable_tap(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tap_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_tap(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tap_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_enable_ff(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	/* Make sure freefall is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.ff_en)
		return status;

	/* Enable freefall */
	edmp_apex_en0.ff_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_ff(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.ff_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_enable_b2s(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	/* Make sure B2S is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.b2s_en)
		return status;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.b2s_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_b2s(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.b2s_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_enable_gaf(inv_imu_device_t *s)
{
	int             status   = INV_IMU_OK;
	uint8_t         gaf_init = 0;
	edmp_apex_en1_t edmp_apex_en1;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	/* Make sure GAF is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	if (edmp_apex_en1.gaf_en)
		return status;

	/*
	 * Force reinitialization of algorithm because user might have changed GAF parameters between
	 * call	to `inv_imu_init_gaf()` and call to `inv_imu_edmp_enable_gaf()`.
	 * If this is not done, new user parameters won't be applied.
	 */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_INIT_STATUS, (uint8_t *)&gaf_init);

	/* Enable GAF */
	edmp_apex_en1.gaf_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_disable_gaf(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.gaf_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_enable_sif(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	/* Make sure SIF is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.sif_en)
		return status;

	edmp_apex_en0.sif_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_sif(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.sif_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_enable_vvd(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	/* Make sure VVD is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.vvd_en)
		return status;

	/* Enable VVD */
	edmp_apex_en0.vvd_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_vvd(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.vvd_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_get_int_apex_status(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it)
{
	int                status = INV_IMU_OK;
	int_apex_statusx_t st;

	/* Read APEX interrupt status */
	status |= inv_imu_read_reg(s, INT_APEX_STATUS0, 2, (uint8_t *)&st);

	it->INV_TAP        = st.int_apex_status0.int_status_tap_det;
	it->INV_HIGHG      = st.int_apex_status0.int_status_high_g_det;
	it->INV_LOWG       = st.int_apex_status0.int_status_low_g_det;
	it->INV_SIF        = st.int_apex_status0.int_status_sif_det;
	it->INV_AID_HUMAN  = st.int_apex_status0.int_status_aid_human_det;
	it->INV_AID_DEVICE = st.int_apex_status0.int_status_aid_device_det;
	it->INV_FF         = st.int_apex_status0.int_status_ff_det;
	it->INV_B2S        = st.int_apex_status0.int_status_b2s_det;
	it->INV_B2S_REV    = st.int_apex_status1.int_status_b2s_rev_det;
	it->INV_VVD        = st.int_apex_status1.int_status_vvd_det;
	it->INV_SELF_TEST  = st.int_apex_status1.int_status_selftest_done;
	it->INV_SEC_AUTH   = st.int_apex_status1.int_status_sa_done;

	return status;
}

int inv_imu_edmp_get_ff_data(inv_imu_device_t *s, uint16_t *freefall_duration)
{
	int     status = INV_IMU_OK;
	uint8_t data1[2];
	uint8_t data2[2];

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DURATION, data1);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DURATION, data2);
	if ((data1[0] == data2[0]) && (data1[1] == data2[1])) {
		*freefall_duration = (data1[1] << 8) | data1[0];
		return status;
	} else {
		*freefall_duration = 0;
		return INV_IMU_ERROR_EDMP_RAM_KO;
	}
}

int inv_imu_edmp_get_tap_data(inv_imu_device_t *s, inv_imu_edmp_tap_data_t *data)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_NUM, (uint8_t *)&(data->num));
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_AXIS, (uint8_t *)&(data->axis));
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_DIR, (uint8_t *)&(data->direction));
	if (data->num == INV_IMU_EDMP_TAP_DOUBLE) {
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_DOUBLE_TAP_TIMING,
		                                 (uint8_t *)&(data->double_tap_timing));
	} else {
		data->double_tap_timing = 0;
	}

	if (data->num == INV_IMU_EDMP_TAP_TRIPLE) {
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TRIPLE_TAP_TIMING,
		                                 (uint8_t *)&(data->triple_tap_timing));
	} else {
		data->triple_tap_timing = 0;
	}

	return status;
}

int inv_imu_edmp_get_aid_data_human(inv_imu_device_t *s, uint8_t *output_state)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_HUMAN_OUTPUT_STATE, output_state);

	return status;
}
int inv_imu_edmp_get_aid_data_device(inv_imu_device_t *s, uint8_t *output_state)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_AID_DEVICE_OUTPUT_STATE, output_state);

	return status;
}

int inv_imu_edmp_gaf_decode_fifo(inv_imu_device_t *s, const uint8_t es0[9], const uint8_t es1[6],
                                 inv_imu_edmp_gaf_outputs_t *const out)
{
	int status = INV_IMU_OK;

	/* Decode quaternion based on ES0 bitfield of FIFO content */
	out->grv_quat_q14[0] = (int16_t)(((int16_t)es0[1] << 8) + ((int16_t)es0[0]));
	out->grv_quat_q14[1] = (int16_t)(((int16_t)es0[3] << 8) + ((int16_t)es0[2]));
	out->grv_quat_q14[2] = (int16_t)(((int16_t)es0[5] << 8) + ((int16_t)es0[4]));
	out->grv_quat_q14[3] = (int16_t)(((int16_t)es0[7] << 8) + ((int16_t)es0[6]));

	/* Decode gyro bias and accuracy flag based on ES1 bitfield of FIFO content */
	out->gyr_bias_q12[0]   = (int16_t)(((int16_t)es1[0] << 8) + ((int16_t)es0[8]));
	out->gyr_bias_q12[1]   = (int16_t)(((int16_t)es1[2] << 8) + ((int16_t)es1[1]));
	out->gyr_bias_q12[2]   = (int16_t)(((int16_t)es1[4] << 8) + ((int16_t)es1[3]));
	out->gyr_accuracy_flag = es1[5] & 0x03;

	/* Decode stationary flag value based on ES1 bitfield of FIFO content */
	switch (es1[5] & 0xfc) {
	case 0xfc:
		out->stationary_flag = -1;
		break;
	case 0x0:
		out->stationary_flag = 0;
		break;
	case 0x4:
		out->stationary_flag = 1;
		break;
	case 0x8:
		out->stationary_flag = 2;
		break;
	default:
		status = -1;
		break;
	}

	return status;
}

int inv_imu_edmp_get_sif_class_index(inv_imu_device_t *s, int16_t *class_index)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SIF_CLASS_INDEX, (uint8_t *)class_index);
	return status;
}

int inv_imu_edmp_mask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb, uint8_t int_mask)
{
	int      status = INV_IMU_OK;
	uint32_t reg_addr;
	uint8_t  reg;

	/*
	 * Interrupt mask register for EDMP interrupts are located in 3 consecutive
	 * registers starting with STATUS_MASK_PIN_0_7 for interrupt0.
	 */
	reg_addr = STATUS_MASK_PIN_0_7 + edmp_int_nb;

	/* Set bits passed in param to mask corresponding interrupts */
	status |= inv_imu_read_reg(s, reg_addr, 1, &reg);
	reg |= int_mask;
	status |= inv_imu_write_reg(s, reg_addr, 1, &reg);

	return status;
}

int inv_imu_edmp_unmask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
                                uint8_t int_mask)
{
	int      status = INV_IMU_OK;
	uint32_t reg_addr;
	uint8_t  reg;

	/*
	 * Interrupt mask register for EDMP interrupts are located in 3 consecutive
	 * registers starting with STATUS_MASK_PIN_0_7 for interrupt0.
	 */
	reg_addr = STATUS_MASK_PIN_0_7 + edmp_int_nb;

	/* Clear bits passed in param to unmask corresponding interrupts */
	status |= inv_imu_read_reg(s, reg_addr, 1, &reg);
	reg &= ~(int_mask);
	status |= inv_imu_write_reg(s, reg_addr, 1, &reg);

	return status;
}

int inv_imu_edmp_configure(inv_imu_device_t *s)
{
	int      status       = INV_IMU_OK;
	uint16_t start_addr[] = { EDMP_ROM_START_ADDR_IRQ0, EDMP_ROM_START_ADDR_IRQ1,
		                      EDMP_ROM_START_ADDR_IRQ2 };
	/* Only 8 MSB of SP address is written to register */
	uint8_t stack_addr = (uint8_t)(EDMP_APEX_FEATURE_STACK_END >> 8);

	/* Set Start address for 3 edmp IRQ handlers */
	status |=
	    inv_imu_write_reg(s, EDMP_PRGRM_IRQ0_0, sizeof(start_addr), (uint8_t *)&start_addr[0]);

	/* Set Stack pointer start address */
	status |= inv_imu_write_reg(s, EDMP_SP_START_ADDR, sizeof(stack_addr), (uint8_t *)&stack_addr);

	return status;
}

int inv_imu_edmp_run_ondemand(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb)
{
	int            status = INV_IMU_OK;
	reg_host_msg_t reg_host_msg;

	status |= inv_imu_edmp_unmask_int_src(s, edmp_int_nb, EDMP_INT_SRC_ON_DEMAND_MASK);

	status |= inv_imu_edmp_enable(s);

	status |= inv_imu_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.edmp_on_demand_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	return status;
}

int inv_imu_edmp_request_vvd_update_thresh(inv_imu_device_t *s, const int32_t new_dyn_env_thresh)
{
	int            status         = INV_IMU_OK;
	uint8_t        update_request = EDMP_ONDEMAND_ENABLE_VVD_DYN_PARAM;
	reg_host_msg_t reg_host_msg;

	/* First retrieve the current value of the dynamic service request mask to know if update is possible */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ONDEMAND_DYNAMIC_SERVICE_REQUEST, &update_request);
	if (0 != update_request) {
		status = INV_IMU_ERROR; /* update already on-going, skip */
	} else {
		/* Prepare SRAM area for the update (only one at any given time is possible) */
		update_request = EDMP_ONDEMAND_ENABLE_VVD_DYN_PARAM;
		status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_VVD_DYNAMIC_THRESH,
		                                  (const uint8_t *)&new_dyn_env_thresh);
		status |=
		    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ONDEMAND_DYNAMIC_SERVICE_REQUEST, &update_request);
		/* Enable and trigger on-demand on ISR1 */
		status |= inv_imu_edmp_unmask_int_src(s, INV_IMU_EDMP_INT1, EDMP_INT_SRC_ON_DEMAND_MASK);
		status |= inv_imu_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
		reg_host_msg.edmp_on_demand_en = INV_IMU_ENABLE;
		status |= inv_imu_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	}
	return status;
}

int inv_imu_edmp_check_vvd_thresh(inv_imu_device_t *s, uint8_t *const dyn_thresh_status)
{
	int     status         = INV_IMU_OK;
	uint8_t update_request = EDMP_ONDEMAND_ENABLE_VVD_DYN_PARAM;
	uint8_t update_status  = 0;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_ONDEMAND_DYNAMIC_SERVICE_REQUEST, &update_request);
	if (0 == update_request) {
		/* Mask on-demand interrupt on ISR1 after the update
		 * although it is more precautionary than mandatory
		 * since there was only one on-demand request. edmp
		 * will auto-clear `on_demand_dynamic_service_request`.
		 */
		status |= inv_imu_edmp_mask_int_src(s, INV_IMU_EDMP_INT1, EDMP_INT_SRC_ON_DEMAND_MASK);
		update_status = 1;
	}
	if (NULL != dyn_thresh_status) {
		*dyn_thresh_status = update_status;
	}
	return status;
}

int inv_imu_edmp_write_ram_area(inv_imu_device_t *s, uint16_t start_addr, uint16_t size,
                                uint8_t value)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t save_edmp_apex_en0;
	edmp_apex_en1_t save_edmp_apex_en1;
	uint32_t        edmp_service_id;

	/*
	 * Check that DMP is turned OFF before requesting memset service
	 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);
	if (save_edmp_apex_en1.edmp_enable)
		return INV_IMU_ERROR;

	/* Request memset(start_addr, val, size) */
	edmp_service_id = EDMP_ONDEMAND_ENABLE_MEMSET;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ONDEMAND_STATIC_SERVICE_REQUEST,
	                                  (uint8_t *)&edmp_service_id);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ONDEMAND_MEMSET_ADDR, (uint8_t *)&start_addr);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ONDEMAND_MEMSET_SIZE, (uint8_t *)&size);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ONDEMAND_MEMSET_VALUE, (uint8_t *)&value);
	status |= inv_imu_edmp_run_ondemand(s, INV_IMU_EDMP_INT2);

	/* Wait 200 us to give enough time for EMDP to start running */
	inv_imu_sleep_us(s, 200);

	/* Wait for DMP execution to complete */
	status |= inv_imu_edmp_wait_for_idle(s);

	/* Reset states */
	status |= inv_imu_edmp_mask_int_src(s, INV_IMU_EDMP_INT2, EDMP_INT_SRC_ON_DEMAND_MASK);

	/* Restore original DMP state, with DMP necessarily disabled as it was checked at the beginning of this function */
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);

	return status;
}

int inv_imu_edmp_wait_for_idle(inv_imu_device_t *s)
{
	int          status = INV_IMU_OK;
	ipreg_misc_t ipreg_misc;
	int          timeout_us = 1000000; /* 1 sec */

	/* Wait for idle == 1 (indicates EDMP is not running, e.g execution is completed) */
	while (status == INV_IMU_OK) {
		status |= inv_imu_read_reg(s, IPREG_MISC, 1, (uint8_t *)&ipreg_misc);
		if (ipreg_misc.edmp_idle != 0)
			break;

		inv_imu_sleep_us(s, 5);
		timeout_us -= 5;

		if (timeout_us <= 0)
			return INV_IMU_ERROR_TIMEOUT;
	}

	return status;
}
