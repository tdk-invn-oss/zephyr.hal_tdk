/*
 *
 * Copyright (c) [2016] by InvenSense, Inc.
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

/* InvenSense */
#include "inv_imu_transport.h"
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "inv_imu_datainj.h"
#include "imu/inv_imu_driver_internal.h"
#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_apex.h"
#include "imu/inv_imu_edmp_SRAM.h"
#include "imu/inv_imu_edmp_memmap_internal.h"
#include "imu/inv_imu_defs_internal.h"

/* board driver */
#include "uart.h"
#include "delay.h"
#include "gpio.h"
#include "i2c_master.h"
#include "spi_master.h"
#include "timer.h"

/* std */
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

/*
 * Uncomment next macro definition to enable some more traces for data-inj debug purpose.
 */
/*#define DAT_INJ_DEBUG_ON */

/*
 * data injection may be run in 2 different modes. The 2 modes differ on the run speed.
 *  - in DATA_INJ_RUN_MODE_REAL_TIME mode, accel is always on running at the ODR targeted by the
 * current *    test. Thus, DMP execution is triggered by accel at effective ODR. This implies data
 * inj driver should notify data and inject next sample within one accel ODR.
 *  - in DATA_INJ_RUN_MODE_FAST mode, accel is started and stopped at each sample. Thus, DMP
 * execution is triggered by data injection driver when it is ready.
 *
 * DATA_INJ_RUN_MODE_FAST is much faster than DATA_INJ_RUN_MODE_REAL_TIME and it avoids any accel
 * ODR miss and thus it ensures each sample is played once only. Nevertheless, real-time mode run is
 * recommended to mimic real system as much as possible.
 */
#define DATA_INJ_RUN_MODE_FAST      0
#define DATA_INJ_RUN_MODE_REAL_TIME 1
#define DATA_INJ_RUN_MODE           DATA_INJ_RUN_MODE_FAST

static inv_imu_data_inj_stats data_inj_stats;

static uint8_t accel_scale_factor;
/* keep step activity static as step count buffer is written only when value changes */
static inv_imu_edmp_pedometer_data_t step_activity;

/* Data injection features */
static int data_inj_run_freefall_one_sample(void *self, inv_imu_data_inj_feature_t feature,
					    int32_t test_data[3]);
static int data_inj_run_pedometer_one_sample(void *self, inv_imu_data_inj_feature_t feature,
					     int32_t test_data[6], uint16_t param);
static int data_inj_run_tilt_one_sample(void *self, int32_t test_data[6]);
static int data_inj_run_tap_one_sample(void *self, int32_t test_data[6]);
static int data_inj_run_r2w_one_sample(void *self, int32_t test_data[6]);
static int data_inj_run_b2s_one_sample(void *self, int32_t test_data[6]);
static int data_inj_run_smd_one_sample(void *self, inv_imu_data_inj_feature_t feature,
				       int32_t test_data[6]);
static int data_inj_run_shake_one_sample(void *self, int32_t test_data[6]);
static int data_inj_run_noMotion_one_sample(void *self, int32_t test_data[6]);
static int data_inj_run_flat_one_sample(void *self, int32_t test_data[6]);
static int data_inj_run_bump_one_sample(void *self, int32_t test_data[6]);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
static void enable_acc_ln(void *self);
#endif

/* IMU utils */
static uint32_t wait_dmp_idle(void *self);
static uint32_t run_dmp(void *self);
static void inhibit_real_acc_data(void *self);
static void inject_apex_data(void *self, int32_t test_data[6]);
static void disable_acc(void *self);
/*static void inject_selftest_data(void *self, const uint8_t st_ft[6], const int16_t
 * acc_gyr_data[6]); */
/*static uint8_t selftest_limit_to_bitfield(uint8_t limit); */
static void update_data_inj_stats(void *self, uint32_t dmp_exec_time);
static void get_dmp_data_s32(void *self, uint32_t row, uint32_t col, int32_t *out);

#ifdef DAT_INJ_DEBUG_ON
static void print_smd_vars(void *self);
static void print_apex_registers(void *self);
static void print_freefall_vars(void *self);
static void print_s4s_vars(void *self);
static void print_3axis_vars(void *self);
static void print_tilt_vars(void *self);
static void print_dbg_regs(void *self);
static void print_pedo_vars(void *self);

#define PRINT_APEX_REGISTERS(a) print_apex_registers(a);
#define PRINT_FREEFALL_VARS(a)  print_freefall_vars(a);
#define PRINT_S4S_VARS(a)       print_s4s_vars(a);
#define PRINT_3AXIS_VARS(a)     print_3axis_vars(a);
#define PRINT_TILT_VARS(a)      print_tilt_vars(a);
#define PRINT_DBG_REGS(a)       print_dbg_regs(a);
#define PRINT_PEDO_VARS(a)      print_pedo_vars(a);
#define PRINT_SMD_VARS(a)       print_smd_vars(a);

#else

#define PRINT_APEX_REGISTERS(a)
#define PRINT_FREEFALL_VARS(a)
#define PRINT_S4S_VARS(a)
#define PRINT_3AXIS_VARS(a)
#define PRINT_TILT_VARS(a)
#define PRINT_DBG_REGS(a)
#define PRINT_PEDO_VARS(a)
#define PRINT_SMD_VARS(a)
#define PRINT_BASICSMD_VARS(a)
#define PRINT_MAG_VARS(a)

#endif

/*
 * This function had been removed from driver, but I am too lazy to replace all occurrences on this
 * file, * so I just redefine it here as static. Sorry.
 */
static int inv_imu_read_dmp_ram(void *s, uint8_t *data, uint32_t sram_addr, uint32_t size)
{
	return inv_imu_read_sram(s, sram_addr, size, data);
}

void datainj_sensor_event_cb(inv_imu_sensor_event_t *event)
{
}

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
static void enable_acc_ln(void *self)
{
	pwr_mgmt0_t pwr_mgmt0;

	inv_imu_read_reg(self, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.accel_mode = PWR_MGMT0_ACCEL_MODE_LN;
	inv_imu_write_reg(self, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
}
#endif

uint32_t inv_imu_data_inj_inject(inv_imu_device_t *self, inv_imu_data_inj_feature_t feature,
				 int32_t sensor_cli_test_data[7], uint16_t param, uint32_t idx,
				 inv_imu_data_inj_output *out)
{
	int ret = 0;
	uint8_t reg;
	uint8_t *tmp;
	int_apex_status0_t int_apex_status0;
	int_apex_status1_t int_apex_status1;
	uint8_t regtraces;
	uint16_t apex_data16 = 0;
	uint32_t apex_data32 = 0;
	(void)get_dmp_data_s32;
#ifdef DAT_INJ_DEBUG_ON
	/* dummy use of the symbols to prevent compiler from yelling about unused symbols */
	(void)print_apex_registers;
	(void)print_freefall_vars;
	(void)print_s4s_vars;
	(void)print_3axis_vars;
	(void)print_tilt_vars;
	(void)print_dbg_regs;
	(void)print_pedo_vars;
	(void)print_smd_vars;
#endif
	memset(out, 0, sizeof(*out));

	switch (feature) {
	case INV_DATA_INJ_FEATURE_FF:
		ret = data_inj_run_freefall_one_sample(self, feature, sensor_cli_test_data);

		/* get freefall duration in case edmp updated it */
		if (inv_imu_edmp_get_ff_data(self, &apex_data16) == 0) {
			INV_MSG(INV_MSG_LEVEL_INFO, "Freefall duration = %d, count = %d",
				apex_data16, data_inj_stats.sample_count);
			INV_MSG(INV_MSG_LEVEL_INFO, "DEBUG DATA = %d, %d, %d",
				sensor_cli_test_data[0], sensor_cli_test_data[1],
				sensor_cli_test_data[2]);
		}

		/*
		 * Get FF internal data that can be checked against IV:
		 * state, lowg state and counter, highg state and counter, lowg/highg event flags,
		 * interrupt status
		 */
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->ff.state,
				     (uint32_t)EDMP_APEX_STATE_FF_STATE,
				     EDMP_APEX_STATE_FF_STATE_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->ff.lowg_state,
				     (uint32_t)EDMP_APEX_STATE_FF_LOW_G_STATE,
				     EDMP_APEX_STATE_FF_LOW_G_STATE_SIZE +
					     EDMP_APEX_STATE_FF_LOW_G_CNTR_SIZE);

		inv_imu_read_dmp_ram(self, (uint8_t *)&out->ff.highg_state,
				     (uint32_t)EDMP_APEX_STATE_FF_HIGH_G_STATE,
				     EDMP_APEX_STATE_FF_HIGH_G_STATE_SIZE +
					     EDMP_APEX_STATE_FF_HIGH_G_CNTR_SIZE);

		inv_imu_read_reg(self, INT_APEX_STATUS0, 1, (uint8_t *)&int_apex_status0);

		out->ff.lowg_event = int_apex_status0.int_status_low_g_det;
		out->ff.highg_event = int_apex_status0.int_status_high_g_det;

		out->ff.int_status = int_apex_status0.int_status_ff_det;
		out->ff.dmp_exec_time = ret;
		out->ff.duration = apex_data16;

		break;

	case INV_DATA_INJ_FEATURE_TILT:
		ret = data_inj_run_tilt_one_sample(self, &sensor_cli_test_data[0]);
		inv_imu_read_reg(self, INT_APEX_STATUS0, 1, (uint8_t *)&int_apex_status0);
		tmp = (uint8_t *)&int_apex_status0; /* beurk! */
		reg = *tmp;
		if (reg) {
			INV_MSG(INV_MSG_LEVEL_INFO, "Tilt reg = 0x%x, datacount = %d", reg,
				data_inj_stats.sample_count);
			inv_imu_read_reg(self, DMP_EXT_SEN_ODR_CFG, 1, &regtraces);
			INV_MSG(INV_MSG_LEVEL_INFO, "Tilt DMP ODR reg = 0x%x", regtraces);
			inv_imu_read_reg(self, ACCEL_CONFIG0, 1, &regtraces);
			INV_MSG(INV_MSG_LEVEL_INFO, "Tilt ACC ODR reg = 0x%x", regtraces);
			inv_imu_read_dmp_ram(self, (uint8_t *)&apex_data16,
					     (uint32_t)EDMP_TILT_WAIT_TIME,
					     EDMP_TILT_WAIT_TIME_SIZE);
			INV_MSG(INV_MSG_LEVEL_INFO, "Tilt wait_time = 0x%x", apex_data16);
			inv_imu_read_dmp_ram(self, (uint8_t *)&apex_data32,
					     (uint32_t)EDMP_TILT_ANGLE_TH, EDMP_TILT_ANGLE_TH_SIZE);
			INV_MSG(INV_MSG_LEVEL_INFO, "Tilt angle_th = %ld", apex_data32);
			inv_imu_read_dmp_ram(self, &regtraces, EDMP_APEX_STATE_TILT_DECIM_RATE,
					     EDMP_APEX_STATE_TILT_DECIM_RATE_SIZE);
			INV_MSG(INV_MSG_LEVEL_INFO, "Tilt decim rate = 0x%x", regtraces);
		}

		/*
		 * Get Tilt internal data that can be checked against IV:
		 * tilt angle, 3axis quaternion, interrupt status
		 */
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->tilt.angle,
				     (uint32_t)EDMP_APEX_STATE_TILT_TILT_ANGLE,
				     EDMP_APEX_STATE_TILT_TILT_ANGLE_SIZE);

		inv_imu_read_dmp_ram(self, (uint8_t *)&out->tilt.quat[0],
				     (uint32_t)EDMP_APEX_INTERF_QUAT, EDMP_APEX_INTERF_QUAT_SIZE);

		out->tilt.int_status = int_apex_status0.int_status_tilt_det;
		out->tilt.dmp_exec_time = ret;
		break;

	case INV_DATA_INJ_FEATURE_TAP:
		ret = data_inj_run_tap_one_sample(self, sensor_cli_test_data);
		inv_imu_read_reg(self, INT_APEX_STATUS0, 1, (uint8_t *)&int_apex_status0);
		out->tap.int_status = int_apex_status0.int_status_tap_detect;

		/*
		 * Get Tap internal data that can be checked against IV:
		 * tap, tap_axis, tap_direction, double_tap_timing, tap_interrupt
		 */
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->tap.type, (uint32_t)EDMP_TAP_NUM,
				     EDMP_TAP_NUM_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->tap.axis, (uint32_t)EDMP_TAP_AXIS,
				     EDMP_TAP_AXIS_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->tap.dir, (uint32_t)EDMP_TAP_DIR,
				     EDMP_TAP_DIR_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->tap.dble_timing,
				     (uint32_t)EDMP_DOUBLE_TAP_TIMING, EDMP_DOUBLE_TAP_TIMING_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->tap.trple_timing,
				     (uint32_t)EDMP_APEX_INTERF_TRIPLE_TAP_TIMING,
				     EDMP_APEX_INTERF_TRIPLE_TAP_TIMING_SIZE);

		/*Check interrupt from sram too to add more robustness */
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->tap.it,
				     (uint32_t)EDMP_APEX_STATE_TAP_INTERRUPT,
				     EDMP_APEX_STATE_TAP_INTERRUPT_SIZE);

		out->tap.dmp_exec_time = ret;
		break;

	case INV_DATA_INJ_FEATURE_PEDOMETER:
		ret = data_inj_run_pedometer_one_sample(self, feature, sensor_cli_test_data, param);
		out->pedo.dmp_exec_time = ret;

		inv_imu_read_reg(self, INT_APEX_STATUS0, 1, (uint8_t *)&int_apex_status0);
		out->pedo.int_status = int_apex_status0.int_status_step_cnt_ovfl << 1 |
				       int_apex_status0.int_status_step_det;

		/*
		 * Note: step_count buffer is updated only when its value changes. Thus,
		 * inv_imu_apex_get_data_activity		 * returns a negative value when it
		 * can't read step_count. In that case, datainj output structure is		 *
		 * updated with last known value for step count. step_cadence and activity_class can
		 * be read safely at each sample.
		 */
		inv_imu_edmp_get_pedometer_data(self, &step_activity);
		out->pedo.step_count = step_activity.step_cnt;
		out->pedo.step_cadence = step_activity.step_cadence;
		out->pedo.activity_class = step_activity.activity_class;

		/*
		 * Get pedometer internal data that can be checked against IV:
		 * 3axis_accel_wx, 3axis_accel_wy, 3axis_accel_wz
		 */
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->pedo.state_3axis_accel_wx,
				     (uint32_t)EDMP_APEX_STATE_3AXIS_ACCEL_WX,
				     EDMP_APEX_STATE_3AXIS_ACCEL_WX_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->pedo.state_3axis_accel_wy,
				     (uint32_t)EDMP_APEX_STATE_3AXIS_ACCEL_WY,
				     EDMP_APEX_STATE_3AXIS_ACCEL_WY_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->pedo.state_3axis_accel_wz,
				     (uint32_t)EDMP_APEX_STATE_3AXIS_ACCEL_WZ,
				     EDMP_APEX_STATE_3AXIS_ACCEL_WZ_SIZE);
		break;

	case INV_DATA_INJ_FEATURE_SMD_SENSIVITY_0:
	case INV_DATA_INJ_FEATURE_SMD_SENSIVITY_1:
	case INV_DATA_INJ_FEATURE_SMD_SENSIVITY_2:
	case INV_DATA_INJ_FEATURE_SMD_SENSIVITY_3:
	case INV_DATA_INJ_FEATURE_SMD_SENSIVITY_4:
		ret = data_inj_run_smd_one_sample(self, feature, sensor_cli_test_data);
		inv_imu_read_reg(self, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);
		out->smd.int_status = int_apex_status1.int_status_smd_det;
		tmp = (uint8_t *)&int_apex_status1; /* beurk! */
		reg = *tmp;
		if (reg) {
			INV_MSG(INV_MSG_LEVEL_INFO, "SMD int = 0x%x, datacount = %d", reg,
				data_inj_stats.sample_count);
		}
		out->smd.dmp_exec_time = ret;
		break;
	case INV_DATA_INJ_FEATURE_R2W:
		ret = data_inj_run_r2w_one_sample(self, sensor_cli_test_data);

		inv_imu_read_reg(self, INT_APEX_STATUS0, 1, (uint8_t *)&int_apex_status0);
		tmp = (uint8_t *)&int_apex_status0; /* beurk! */
		reg = *tmp;
		out->r2w.wake_event = int_apex_status0.int_status_r2w_wake_det;
		if (reg) {
			INV_MSG(INV_MSG_LEVEL_INFO, "R2W Wake int = 0x%x, datacount = %d", reg,
				data_inj_stats.sample_count);
		}

		inv_imu_read_reg(self, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);
		tmp = (uint8_t *)&int_apex_status1; /* beurk! */
		reg = *tmp;
		out->r2w.sleep_event = int_apex_status1.int_status_r2w_sleep_det;
		if (reg) {
			INV_MSG(INV_MSG_LEVEL_INFO, "R2W Sleep int = 0x%x, datacount = %d", reg,
				data_inj_stats.sample_count);
		}

		out->b2s.dmp_exec_time = ret;
		break;

	case INV_DATA_INJ_FEATURE_B2S:
		ret = data_inj_run_b2s_one_sample(self, sensor_cli_test_data);

		inv_imu_read_reg(self, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);
		tmp = (uint8_t *)&int_apex_status1; /* beurk! */
		reg = *tmp;
		out->b2s.b2s_rev_event = int_apex_status1.int_status_revb2s_det;
		out->b2s.b2s_event = int_apex_status1.int_status_b2s_det;
		if (reg) {
			INV_MSG(INV_MSG_LEVEL_INFO, "B2S int = 0x%x, datacount = %d", reg,
				data_inj_stats.sample_count);
		}

		out->smd.dmp_exec_time = ret;
		break;
	case INV_DATA_INJ_FEATURE_SHAKE:
		ret = data_inj_run_shake_one_sample(self, &sensor_cli_test_data[0]);
		{
			inv_imu_read_reg(self, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);
			out->shake.shake_outInterrupt = int_apex_status1.int_status_shake_det;
		}
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.shake_axis,
				     (uint32_t)EDMP_SHAKE_OUTAXIS, EDMP_SHAKE_OUTAXIS_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.shake_C1,
				     (uint32_t)EDMP_APEX_STATE_SHAKE_C1,
				     EDMP_APEX_STATE_SHAKE_C1_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.shake_candidate,
				     (uint32_t)EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE,
				     EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.shake_general_shake_timer,
				     (uint32_t)EDMP_APEX_STATE_SHAKE_GENERAL_SHAKE_TIMER,
				     EDMP_APEX_STATE_SHAKE_GENERAL_SHAKE_TIMER_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.shake_stabilityCount,
				     (uint32_t)EDMP_APEX_STATE_SHAKE_STABILITYCOUNT,
				     EDMP_APEX_STATE_SHAKE_STABILITYCOUNT_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.shake_previous_state,
				     (uint32_t)EDMP_APEX_STATE_SHAKE_PREVIOUS_STATE,
				     EDMP_APEX_STATE_SHAKE_PREVIOUS_STATE_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.candidate_x,
				     (uint32_t)EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_X,
				     EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_X_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.candidate_y,
				     (uint32_t)EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_Y,
				     EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_Y_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->shake.candidate_z,
				     (uint32_t)EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_Z,
				     EDMP_APEX_STATE_SHAKE_SHAKE_CANDIDATE_Z_SIZE);
		out->shake.dmp_exec_time = ret;
		break;

	case INV_DATA_INJ_FEATURE_NOMOTION:
		ret = data_inj_run_noMotion_one_sample(self, sensor_cli_test_data);
		{
			int_apex_status2_t int_apex_status2;
			inv_imu_read_reg(self, INT_APEX_STATUS2, 1, (uint8_t *)&int_apex_status2);
			out->noMotion.int_status = int_apex_status2.int_status_nomotion_det |
						   (int_apex_status2.int_status_motion_det << 1);
			if (out->noMotion.int_status) {
				INV_MSG(INV_MSG_LEVEL_INFO, "NM int = 0x%x, datacount = %d",
					int_apex_status2, data_inj_stats.sample_count);
			}
		}
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->noMotion.noMotion_majority_axis,
				     (uint32_t)EDMP_NOMOTION_MAJORITY_AXIS,
				     EDMP_NOMOTION_MAJORITY_AXIS_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->noMotion.noMotion_tilt_angle_raw_format,
				     (uint32_t)EDMP_NOMOTION_TILT_ANGLE_RAW_FORMAT,
				     EDMP_NOMOTION_TILT_ANGLE_RAW_FORMAT_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->noMotion.noMotion_sign_angle,
				     (uint32_t)EDMP_NOMOTION_SIGN_ANGLE,
				     EDMP_NOMOTION_SIGN_ANGLE_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->noMotion.noMotion_x_axis_ref,
				     (uint32_t)EDMP_NOMOTION_X_AXIS_REF,
				     EDMP_NOMOTION_X_AXIS_REF_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->noMotion.noMotion_y_axis_ref,
				     (uint32_t)EDMP_NOMOTION_Y_AXIS_REF,
				     EDMP_NOMOTION_Y_AXIS_REF_SIZE);

		out->noMotion.dmp_exec_time = ret;
		break;

	case INV_DATA_INJ_FEATURE_FLAT:
		ret = data_inj_run_flat_one_sample(self, sensor_cli_test_data);
		{
			int_apex_status2_t int_apex_status2;
			inv_imu_read_reg(self, INT_APEX_STATUS2, 1, (uint8_t *)&int_apex_status2);

			out->flat.flat_event = int_apex_status2.int_status_flat_det;
			out->flat.no_flat_event = int_apex_status2.int_status_noflat_det;
		}
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->flat.flat_state,
				     (uint32_t)EDMP_APEX_STATE_FLAT_STATE,
				     EDMP_APEX_STATE_FLAT_STATE_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->flat.flat_init_timer,
				     (uint32_t)EDMP_APEX_STATE_FLAT_INIT_TIMER,
				     EDMP_APEX_STATE_FLAT_INIT_TIMER_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->flat.flat_timer,
				     (uint32_t)EDMP_APEX_STATE_FLAT_TIMER,
				     EDMP_APEX_STATE_FLAT_TIMER_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->flat.flat_quat_wb_x,
				     (uint32_t)EDMP_APEX_STATE_FLAT_QUAT_WB_X,
				     EDMP_APEX_STATE_FLAT_QUAT_WB_X_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->flat.flat_quat_wb_y,
				     (uint32_t)EDMP_APEX_STATE_FLAT_QUAT_WB_Y,
				     EDMP_APEX_STATE_FLAT_QUAT_WB_Y_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->flat.flat_quat_wb_z,
				     (uint32_t)EDMP_APEX_STATE_FLAT_QUAT_WB_Z,
				     EDMP_APEX_STATE_FLAT_QUAT_WB_Z_SIZE);
		inv_imu_read_dmp_ram(self, (uint8_t *)&out->flat.cos_flat_angle,
				     (uint32_t)EDMP_APEX_STATE_FLAT_COS_FLAT_ANGLE,
				     EDMP_APEX_STATE_FLAT_COS_FLAT_ANGLE_SIZE);
		out->flat.dmp_exec_time = ret;
		break;

	case INV_DATA_INJ_FEATURE_BUMP:
		ret = data_inj_run_bump_one_sample(self, sensor_cli_test_data);
		{
			int_apex_status1_t int_apex_status1;
			inv_imu_read_reg(self, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);

			/* Double check IT and sram content */
			if (int_apex_status1.int_status_sa_done) {
				INV_MSG(INV_MSG_LEVEL_INFO, "bump int = 0x%x, datacount = %d",
					int_apex_status1, data_inj_stats.sample_count);
				inv_imu_read_dmp_ram(self, (uint8_t *)&out->bump.bump_event,
						     (uint32_t)EDMP_bump_outInterrupt,
						     EDMP_bump_outInterrupt_SIZE);
			}
		}

		out->bump.dmp_exec_time = ret;
		break;
	default:
		return uint32_t - 1;
	}

	return 1;
}

void inv_imu_data_inj_reset(inv_imu_device_t *self)
{
	memset(&data_inj_stats, 0, sizeof(data_inj_stats));

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
	disable_acc(self);
#else
	(void)self;
	(void)disable_acc;
#endif
}

void inv_imu_data_inj_get_stats(inv_imu_data_inj_stats *stats)
{
	memcpy(stats, &data_inj_stats, sizeof(data_inj_stats));
}

void inv_imu_data_inj_set_accel_scale_factor(inv_imu_device_t *self)
{
	accel_config0_t accel_config0;

	/* Read Accel FSR and deduce accel data scaling factor */
	inv_imu_read_reg(self, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
	accel_scale_factor = 6 - (accel_config0.ap_accel_fs_sel);
}

int inv_imu_data_inj_get_accel_scale_factor(void)
{
	return accel_scale_factor;
}

static int data_inj_run_freefall_one_sample(void *self, inv_imu_data_inj_feature_t feature,
					    int32_t test_data[3])
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config0_t int_apex_config0;

		inv_imu_read_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);
		int_apex_config0.int_status_mask_pin_ff_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif
		inv_imu_edmp_enable_ff(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
		enable_acc_ln(self);
#endif
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);
	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_pedometer_one_sample(void *self, inv_imu_data_inj_feature_t feature,
					     int32_t test_data[6], uint16_t param)
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config0_t int_apex_config0;

		memset(&step_activity, 0, sizeof(step_activity));

		inv_imu_read_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);
		int_apex_config0.int_status_mask_pin_step_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif
		inv_imu_edmp_enable_pedometer(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
		enable_acc_ln(self);
#endif
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);
	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_tilt_one_sample(void *self, int32_t test_data[6])
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config0_t int_apex_config0;

		inv_imu_read_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);
		int_apex_config0.int_status_mask_pin_tilt_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif
		inv_imu_edmp_enable_tilt(self);

		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);

		inject_apex_data(self, test_data);
#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
		enable_acc_ln(self);
#endif
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);

	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_tap_one_sample(void *self, int32_t test_data[6])
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config0_t int_apex_config0;

		inv_imu_read_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);
		int_apex_config0.int_status_mask_pin_tap_detect = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif
		inv_imu_edmp_enable_tap(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
		enable_acc_ln(self);
#endif
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);
	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_r2w_one_sample(void *self, int32_t test_data[6])
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config0_t int_apex_config0;
		int_apex_config1_t int_apex_config1;

		inv_imu_read_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);
		int_apex_config0.int_status_mask_pin_r2w_wake_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG0, 1, (uint8_t *)&int_apex_config0);

		inv_imu_read_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
		int_apex_config1.int_status_mask_pin_r2w_sleep_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif
		inv_imu_edmp_enable_r2w(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
		enable_acc_ln(self);
#endif
	} else {
		inject_apex_data(self, test_data);
	}
	dmp_exec_time = run_dmp(self);
	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_b2s_one_sample(void *self, int32_t test_data[6])
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config1_t int_apex_config1;

		inv_imu_read_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
		int_apex_config1.int_status_mask_pin_b2s_det = 0;
		int_apex_config1.int_status_mask_pin_revb2s_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif
		inv_imu_edmp_enable_b2s(self);

		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
		enable_acc_ln(self);
#endif
	} else {
		inject_apex_data(self, test_data);
	}
	dmp_exec_time = run_dmp(self);

	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_shake_one_sample(void *self, int32_t test_data[6])
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config1_t int_apex_config1;

		inv_imu_read_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
		int_apex_config1.int_status_mask_pin_shake_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif

		inv_imu_edmp_enable_shake(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);

	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_noMotion_one_sample(void *self, int32_t test_data[6])
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config2_t int_apex_config2;

		inv_imu_read_reg(self, INT_APEX_CONFIG2, 1, (uint8_t *)&int_apex_config2);
		int_apex_config2.int_status_mask_pin_nomotion_det = 0;
		int_apex_config2.int_status_mask_pin_motion_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG2, 1, (uint8_t *)&int_apex_config2);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif

		inv_imu_edmp_enable_noMotion(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);

	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_flat_one_sample(void *self, int32_t test_data[6])
{
	uint32_t dmp_exec_time;
	uint8_t algo_flag = (uint8_t)test_data[3];

	if (data_inj_stats.sample_count == 0) {
		int_apex_config2_t int_apex_config2;
		if (algo_flag) {
			inv_imu_read_reg(self, INT_APEX_CONFIG2, 1, (uint8_t *)&int_apex_config2);
			int_apex_config2.int_status_mask_pin_flat_det = 0;
			int_apex_config2.int_status_mask_pin_noflat_det = 0;
			inv_imu_write_reg(self, INT_APEX_CONFIG2, 1, (uint8_t *)&int_apex_config2);
		}

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif

		inv_imu_edmp_enable_flat(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);

	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_smd_one_sample(void *self, inv_imu_data_inj_feature_t feature,
				       int32_t test_data[6])
{
	uint32_t dmp_exec_time;

	if (data_inj_stats.sample_count == 0) {
		int_apex_config1_t int_apex_config1;

		inv_imu_read_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
		int_apex_config1.int_status_mask_pin_smd_det = 0;
		inv_imu_write_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif
		inv_imu_edmp_enable_smd(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_REAL_TIME)
		enable_acc_ln(self);
#endif
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);
	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static int data_inj_run_bump_one_sample(void *self, int32_t test_data[6])
{
	uint32_t dmp_exec_time;
	uint8_t algo_flag = (uint8_t)test_data[3];

	if (data_inj_stats.sample_count == 0) {
		int_apex_config1_t int_apex_config1;
		if (algo_flag) {
			inv_imu_read_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
			int_apex_config1.int_status_mask_pin_sa_done =
				0; /*bump detection shares SA DONE */
			inv_imu_write_reg(self, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
		}

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
		/* inv_imu_edmp_init_apex unmasked EDMP_INT_SRC_ACCEL_DRDY_MASK. Mask it now as we
		 * want to run edmp on-demand */
		inv_imu_edmp_mask_int_src(self, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);
#endif

		inv_imu_edmp_enable_bump(self);
		inv_imu_edmp_enable(self);
		inhibit_real_acc_data(self);
		inject_apex_data(self, test_data);
	} else {
		inject_apex_data(self, test_data);
	}

	dmp_exec_time = run_dmp(self);

	data_inj_stats.sample_count++;
	update_data_inj_stats(self, dmp_exec_time);

	return dmp_exec_time;
}

static uint32_t wait_dmp_idle(void *self)
{
	ipreg_misc_t ipreg_misc;
	uint64_t dmp_start_time;

	/* wait to make sure dmp_idle = 0 [firmware feature starts once dmp_idle goes high-to-low]
	 */
	while (1) {
		inv_imu_read_reg(self, IPREG_MISC, 1, (uint8_t *)&ipreg_misc);

		if (ipreg_misc.edmp_idle == 0) {
			break;
		}

		inv_delay_us(5);
	}

	dmp_start_time = inv_imu_data_inj_get_time_us(self);

	/*
	 * wait to make sure dmp_idle = 1 [dmp_idle low-to-high indicates that firmware feature	 *
	 * complete for one sample/dmp-odr-cycle]
	 */
	while (1) {
		inv_imu_read_reg(self, IPREG_MISC, 1, (uint8_t *)&ipreg_misc);

		if (ipreg_misc.edmp_idle != 0) {
			break;
		}

		inv_delay_us(5);
	}
	return uint32_t)(inv_imu_data_inj_get_time_us(self) - dmp_start_time;
}

static uint32_t run_dmp(void *self)
{
	uint32_t dmp_exec_time;

#if (DATA_INJ_RUN_MODE == DATA_INJ_RUN_MODE_FAST)
	inv_imu_edmp_run_ondemand(self, INV_IMU_EDMP_INT0);
#endif

	dmp_exec_time = wait_dmp_idle(self);

	return dmp_exec_time;
}

static void inhibit_real_acc_data(void *self)
{
	sreg_ctrl_internal_t sreg_ctrl;

	inv_imu_read_reg(self, SREG_CTRL, 1, (uint8_t *)&sreg_ctrl);
	sreg_ctrl.sreg_debug_data_inject_en = 1;
	inv_imu_write_reg(self, SREG_CTRL, 1, (uint8_t *)&sreg_ctrl);
}

static void inject_apex_data(void *self, int32_t test_data[6])
{
	int accel_scale_factor = inv_imu_data_inj_get_accel_scale_factor();

	/*INV_MSG(INV_MSG_LEVEL_INFO, "injecting acc [% 5d % 5d % 5d]", test_data[0], test_data[1],
	 * test_data[2]); */

	test_data[0] <<= accel_scale_factor;
	test_data[1] <<= accel_scale_factor;
	test_data[2] <<= accel_scale_factor;

	inv_imu_write_reg(self, 0x8000 /*ACCEL_X_0_IHREG_SREG*/, 4, (uint8_t *)&test_data[0]);
	inv_imu_write_reg(self, 0x8004 /*ACCEL_Y_0_IHREG_SREG*/, 4, (uint8_t *)&test_data[1]);
	inv_imu_write_reg(self, 0x8008 /*ACCEL_Z_0_IHREG_SREG*/, 4, (uint8_t *)&test_data[2]);
}
#if 0
static void inject_selftest_data(void *self, const uint8_t st_ft[6], const int16_t acc_gyr_data[6])
{
	uint8_t data[3+3+6+6] = {0}; /* FTA, FTG, ACC, GYR */

	data[0] =  st_ft[0];
	data[1] =  st_ft[1];
	data[2] =  st_ft[2];
	data[3] =  st_ft[3];
	data[4] =  st_ft[4];
	data[5] =  st_ft[5];
	data[6] =  (uint8_t)((acc_gyr_data[0] & 0xFF00) >> 8);
	data[7] =  (uint8_t)( acc_gyr_data[0] & 0x00FF);
	data[8] =  (uint8_t)((acc_gyr_data[1] & 0xFF00) >> 8);
	data[9] =  (uint8_t)( acc_gyr_data[1] & 0x00FF);
	data[10] = (uint8_t)((acc_gyr_data[2] & 0xFF00) >> 8);
	data[11] = (uint8_t)( acc_gyr_data[2] & 0x00FF);
	data[12] = (uint8_t)((acc_gyr_data[3] & 0xFF00) >> 8);
	data[13] = (uint8_t)( acc_gyr_data[3] & 0x00FF);
	data[14] = (uint8_t)((acc_gyr_data[4] & 0xFF00) >> 8);
	data[15] = (uint8_t)( acc_gyr_data[4] & 0x00FF);
	data[16] = (uint8_t)((acc_gyr_data[5] & 0xFF00) >> 8);
	data[17] = (uint8_t)( acc_gyr_data[5] & 0x00FF);

	inv_imu_write_sram(self, 0, 18, data);
}
#endif

static void disable_acc(void *self)
{
	pwr_mgmt0_t pwr_mgmt0;

	inv_imu_read_reg(self, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.accel_mode = PWR_MGMT0_ACCEL_MODE_OFF;
	inv_imu_write_reg(self, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
}

static void update_data_inj_stats(void *self, uint32_t dmp_exec_time)
{
	/* Process max, min and mean execution time */
	data_inj_stats.dmp_exec_time_min = ((data_inj_stats.dmp_exec_time_min == 0) ||
					    (dmp_exec_time < data_inj_stats.dmp_exec_time_min))
						   ? dmp_exec_time
						   : data_inj_stats.dmp_exec_time_min;
	data_inj_stats.dmp_exec_time_max = (dmp_exec_time > data_inj_stats.dmp_exec_time_max)
						   ? dmp_exec_time
						   : data_inj_stats.dmp_exec_time_max;
	data_inj_stats.dmp_exec_time_mean +=
		(float)((dmp_exec_time - data_inj_stats.dmp_exec_time_mean)) /
		data_inj_stats.sample_count;
}

static void get_dmp_data_s32(void *self, uint32_t row, uint32_t col, int32_t *out)
{
	uint8_t data[4];

	inv_imu_read_dmp_ram(self, data, (row * 16) + (col * 4), 4);

	*out = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]));
}

int inv_imu_data_inj_force_ped_odr_25Hz(inv_imu_device_t *self)
{
	int status = 0;
	int16_t data_16;
	int32_t data_32;

	status |= INV_IMU_READ_EDMP_SRAM(self, EDMP_APEX_STATE_PED_CADENCY_THRESH_HIGH,
					 (uint8_t *)&data_32);
	data_32 >>= 1;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_CADENCY_THRESH_HIGH,
					  (uint8_t *)&data_32);
	status |= INV_IMU_READ_EDMP_SRAM(self, EDMP_APEX_STATE_PED_CADENCY_THRESH_LOW,
					 (uint8_t *)&data_32);
	data_32 >>= 1;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_CADENCY_THRESH_LOW,
					  (uint8_t *)&data_32);

	data_16 = 27;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_PARAM_MAX_TIME,
					  (uint8_t *)&data_16);
	data_16 = 7;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_PARAM_MIN_TIME,
					  (uint8_t *)&data_16);

	/* Initial constants (1.5 - 2.6Hz) at 25Hz */
	data_32 = 4272031L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_B4, (uint8_t *)&data_32);
	data_32 = 181579747L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_A4, (uint8_t *)&data_32);
	data_32 = -699413007L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_A3, (uint8_t *)&data_32);
	data_32 = 1114447893L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_A2, (uint8_t *)&data_32);
	data_32 = -852029605L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_A1, (uint8_t *)&data_32);

	/* Initial constant for low-band filter (0.6 - 1.25 Hz) at 25Hz */
	data_32 = 1602672L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_B4_LOW, (uint8_t *)&data_32);
	data_32 = 213062324L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_A4_LOW, (uint8_t *)&data_32);
	data_32 = -879901458L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_A3_LOW, (uint8_t *)&data_32);
	data_32 = 1386967533L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_A2_LOW, (uint8_t *)&data_32);
	data_32 = -988028987L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_A1_LOW, (uint8_t *)&data_32);

	data_32 = 536870912L;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_ALPHA, (uint8_t *)&data_32);

	data_32 = 4;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_DELTA, (uint8_t *)&data_32);
	data_16 = 5;
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_DELTA2, (uint8_t *)&data_16);

	data_32 = 1310720L; /* 40 << 15 */
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_STEP_DURATION_UPPER_THRESHOLD,
					  (uint8_t *)&data_32);

	data_32 = 491520L; /* 15 << 15 */
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_DTC_N_HAT, (uint8_t *)&data_32);

	data_32 = 409600L; /* 12.5 << 15 */
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_DTC_FAST_N_HAT,
					  (uint8_t *)&data_32);

	data_32 = 42949672L; /* 0.02*(2^30) */
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_ALPHA_DTC, (uint8_t *)&data_32);

	data_32 = 357913942L; /* 1/3*(2^30) */
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_ALPHA_DTC_FAST,
					  (uint8_t *)&data_32);

	data_32 = 10737418L; /* 0.01*(2^30) */
	status |= INV_IMU_WRITE_EDMP_SRAM(self, EDMP_APEX_STATE_PED_ALPHA_A, (uint8_t *)&data_32);

	/*PRINT_PEDO_VARS(self); */

	return status;
}

#ifdef DAT_INJ_DEBUG_ON
static void print_apex_registers(void *self)
{
	uint8_t reg;
	inv_imu_read_reg(self, DMP_EXT_SEN_ODR_CFG, 1, &reg);
	INV_MSG(INV_MSG_LEVEL_DEBUG, "DMP_EXT_SEN_ODR_CFG = 0x%02x", reg);

#if 0
	uint8_t reg[7];

	inv_imu_read_reg(self, APEX_CONFIG0, 2, &reg[0]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG0 = 0x%02x", reg[0]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG1 = 0x%02x", reg[1]);

	/* Access config registers (CONFIG2-CONFIG11) */
	inv_imu_read_reg(self, APEX_CONFIG2_MREG_TOP1, 7, &reg[0]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG2_MREG_TOP1 = 0x%02x", reg[0]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG3_MREG_TOP1 = 0x%02x", reg[1]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG4_MREG_TOP1 = 0x%02x", reg[2]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG5_MREG_TOP1 = 0x%02x", reg[3]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG9_MREG_TOP1 = 0x%02x", reg[4]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG10_MREG_TOP1 = 0x%02x", reg[5]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG11_MREG_TOP1 = 0x%02x", reg[6]);

	inv_imu_read_reg(self, APEX_CONFIG12_MREG_TOP1, 1, &reg[0]);
	INV_MSG(INV_MSG_LEVEL_INFO, "APEX_CONFIG12_MREG_TOP1 = 0x%02x", reg[0]);
#endif
}

static void print_freefall_vars(void *self)
{
	union var_types_t dump;

	/* inputs */
	inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
			     (uint32_t)EDMP_APEX_INTERF_ACC_8G + 0, EDMP_APEX_INTERF_ACC_8G_SIZE);
	INV_MSG(INV_MSG_LEVEL_DEBUG, "interf->acc_8g[0] = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
			     (uint32_t)EDMP_APEX_INTERF_ACC_8G + 2, EDMP_APEX_INTERF_ACC_8G_SIZE);
	INV_MSG(INV_MSG_LEVEL_DEBUG, "interf->acc_8g[1] = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
			     (uint32_t)EDMP_APEX_INTERF_ACC_8G + 4, EDMP_APEX_INTERF_ACC_8G_SIZE);
	INV_MSG(INV_MSG_LEVEL_DEBUG, "interf->acc_8g[2] = %d", dump.data_short);

	inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_FREEFALL_STATE,
			     EDMP_APEX_STATE_FREEFALL_STATE_SIZE);
	INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->state = %d", dump.data_uchar[0]);
	inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_FREEFALL_TIMER,
			     EDMP_APEX_STATE_FREEFALL_TIMER_SIZE);
	INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->timer = %d", dump.data_ulong);

	if (dump.data_ulong >= 456) {
		/* params/states */
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_LOW_G_STATE,
				     EDMP_APEX_STATE_FREEFALL_LOW_G_STATE_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->low_g.state = %d", dump.data_short);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_LOW_G_CNTR,
				     EDMP_APEX_STATE_FREEFALL_LOW_G_CNTR_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->low_g.cntr = %d", dump.data_short);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_LOW_G_LOW_PEAK_THRES,
				     EDMP_APEX_STATE_FREEFALL_LOW_G_LOW_PEAK_THRES_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->low_g.low_peak_thres = %d",
			dump.data_short);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_LOW_G_LOW_PEAK_THRES_HYST,
				     EDMP_APEX_STATE_FREEFALL_LOW_G_LOW_PEAK_THRES_HYST_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->low_g.low_peak_thres_hyst = %d",
			dump.data_short);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_LOW_G_LOW_TIME_THRES,
				     EDMP_APEX_STATE_FREEFALL_LOW_G_LOW_TIME_THRES_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->low_g.low_time_thres = %d",
			dump.data_short);

		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_HIGH_G_STATE,
				     EDMP_APEX_STATE_FREEFALL_HIGH_G_STATE_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->high_g.state = %d", dump.data_short);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_HIGH_G_CNTR,
				     EDMP_APEX_STATE_FREEFALL_HIGH_G_CNTR_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->high_g.cntr = %d", dump.data_short);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_HIGH_G_HIGH_PEAK_THRES,
				     EDMP_APEX_STATE_FREEFALL_HIGH_G_HIGH_PEAK_THRES_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->high_g.high_peak_thres = %d",
			dump.data_short);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_HIGH_G_HIGH_PEAK_THRES_HYST,
				     EDMP_APEX_STATE_FREEFALL_HIGH_G_HIGH_PEAK_THRES_HYST_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->high_g.high_peak_thres_hyst = %d",
			dump.data_short);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_HIGH_G_HIGH_TIME_THRES,
				     EDMP_APEX_STATE_FREEFALL_HIGH_G_HIGH_TIME_THRES_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->high_g.high_time_thres = %d",
			dump.data_short);

		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_LOWG_EVENT,
				     EDMP_APEX_STATE_FREEFALL_LOWG_EVENT_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->lowg_event = %d", dump.data_uchar[0]);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_HIGHG_EVENT,
				     EDMP_APEX_STATE_FREEFALL_HIGHG_EVENT_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->highg_event = %d",
			dump.data_uchar[0]);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_MIN_DURATION_THR,
				     EDMP_APEX_STATE_FREEFALL_MIN_DURATION_THR_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->min_duration_thr = %d",
			dump.data_ulong);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_MAX_DURATION_THR,
				     EDMP_APEX_STATE_FREEFALL_MAX_DURATION_THR_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->max_duration_thr = %d",
			dump.data_ulong);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_DEBOUNCE_THR,
				     EDMP_APEX_STATE_FREEFALL_DEBOUNCE_THR_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->debounce_thr = %d", dump.data_ulong);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_STATE,
				     EDMP_APEX_STATE_FREEFALL_STATE_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->state = %d", dump.data_uchar[0]);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_STATE_FREEFALL_TIMER,
				     EDMP_APEX_STATE_FREEFALL_TIMER_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "state_freeFall->timer = %d", dump.data_ulong);

		/* outputs */
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_INTERF_FREE_FALL_INTERRUPT,
				     EDMP_APEX_INTERF_FREE_FALL_INTERRUPT_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "interf->free_fall_interrupt = %d", dump.data_long);
		inv_imu_read_dmp_ram(self, (uint8_t *)dump.data_uchar,
				     (uint32_t)EDMP_APEX_INTERF_FREE_FALL_DURATION_SAMPLES,
				     EDMP_APEX_INTERF_FREE_FALL_DURATION_SAMPLES_SIZE);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "interf->free_fall_duration_samples = %d",
			dump.data_long);
	}
}

void print_dbg_regs(void *self)
{
	uint8_t reg;

	inv_imu_read_reg(self, EDMP_APEX_EN0, 1, &reg);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_EN0 = 0x%02x", reg);
	inv_imu_read_reg(self, EDMP_APEX_EN1, 1, &reg);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_EN1 = 0x%02x", reg);
	inv_imu_read_reg(self, DMP_EXT_SEN_ODR_CFG, 1, &reg);
	INV_MSG(INV_MSG_LEVEL_INFO, "DMP_EXT_SEN_ODR_CFG = 0x%02x", reg);
}

static void print_s4s_vars(void *self)
{
#if 0
	union var_types_t dump;
	uint16_t          stc_tick_period;

	INV_MSG(INV_MSG_LEVEL_INFO, "stc_tick_period = %d", stc_tick_period);

	inv_imu_read_dmp_ram(self, dump.data_uchar,
	                     (uint32_t)EDMP_APEX_STATE_APEXInterpolator_pending_counter_reset,
	                     EDMP_APEX_STATE_APEXInterpolator_pending_counter_reset_size);
	INV_MSG(INV_MSG_LEVEL_INFO, "s4s_pending_counter_reset = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_APEXInterpolator_s4s_ratio,
	                     EDMP_APEX_STATE_APEXInterpolator_s4s_ratio_size);
	INV_MSG(INV_MSG_LEVEL_INFO, "s4s_ratio = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_APEXInterpolator_timestamp_in,
	                     EDMP_APEX_STATE_APEXInterpolator_timestamp_in_size);
	INV_MSG(INV_MSG_LEVEL_INFO, "s4s_timestamp_in = %d", dump.data_ulong);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_APEXInterpolator_timestamp_out,
	                     EDMP_APEX_STATE_APEXInterpolator_timestamp_out_size);
	INV_MSG(INV_MSG_LEVEL_INFO, "s4s_timestamp_out = %d", dump.data_ulong);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_APEXInterpolator_counter,
	                     EDMP_APEX_STATE_APEXInterpolator_counter_size);
	INV_MSG(INV_MSG_LEVEL_INFO, "s4s_counter = %d", dump.data_ulong);
#endif
}

static void print_3axis_vars(void *self)
{
	union var_types_t dump;
	INV_MSG(INV_MSG_LEVEL_INFO, "3 axis vars:");
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ODR_SENSORDATA,
			     EDMP_APEX_INTERF_ODR_SENSORDATA_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ODR_SENSORData = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_DECIM_RATE,
			     EDMP_APEX_STATE_3AXIS_DECIM_RATE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_decim_rate = %d", dump.data_uchar[0]);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_DECIM_COUNT,
			     EDMP_APEX_STATE_3AXIS_DECIM_COUNT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_decim_count = %d", dump.data_uchar[0]);

	/* params/states */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_GAIN,
			     EDMP_APEX_STATE_3AXIS_GAIN_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_gain = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_INIT,
			     EDMP_APEX_STATE_3AXIS_INIT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_init = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_DECIM_RATE,
			     EDMP_APEX_STATE_3AXIS_DECIM_RATE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_decim_rate = %d", dump.data_uchar[0]);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_DECIM_COUNT,
			     EDMP_APEX_STATE_3AXIS_DECIM_COUNT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_decim_count = %d", dump.data_uchar[0]);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_QFEEDBK + 0, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_qfeedbk[0] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_QFEEDBK + 4, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_qfeedbk[1] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_ACCEL_WX,
			     EDMP_APEX_STATE_3AXIS_ACCEL_WX_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_accel_wx = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_ACCEL_WY,
			     EDMP_APEX_STATE_3AXIS_ACCEL_WY_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_accel_wy = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_3AXIS_ACCEL_WZ,
			     EDMP_APEX_STATE_3AXIS_ACCEL_WZ_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_accel_wz = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_3AXIS_ACCEL_NORM_SQUARE,
			     EDMP_APEX_STATE_3AXIS_ACCEL_NORM_SQUARE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_3axis_accel_norm_square = 0x%08x", dump.data_long);

	{
		int i;
		for (i = 0; i < 4; i++) {
			inv_imu_read_dmp_ram(self, dump.data_uchar,
					     (uint32_t)(EDMP_APEX_TMPVARS + (i * sizeof(dump))),
					     sizeof(dump));
			INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_TMPVARS[%d] = 0x%08x", i,
				dump.data_long);
		}
	}
	INV_MSG(INV_MSG_LEVEL_INFO, "");

	inv_imu_sleep_us(self, 5000);
}

static void print_tilt_vars(void *self)
{
	union var_types_t dump;
	INV_MSG(INV_MSG_LEVEL_INFO, "Tilt vars:");
	/* inputs */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 0, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[0] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 4, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[1] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 8, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[2] = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 0, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[0] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 4, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[1] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 8, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[2] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 12, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[3] = 0x%08x", dump.data_long);

	/* params */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_WAIT_TIME,
			     EDMP_APEX_STATE_TILT_WAIT_TIME_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_WAIT_TIME = %d", dump.data_short);

	/* states */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_DECIM_RATE,
			     EDMP_APEX_STATE_TILT_DECIM_RATE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_DECIM_RATE = %d", dump.data_uchar[0]);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_DECIM_COUNT,
			     EDMP_APEX_STATE_TILT_DECIM_COUNT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_DECIM_COUNT = %d", dump.data_uchar[0]);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_ISINIT,
			     EDMP_APEX_STATE_TILT_ISINIT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_ISInit = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_QUAT_WB_X,
			     EDMP_APEX_STATE_TILT_QUAT_WB_X_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_QUAT_WB_X = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_QUAT_WB_Y,
			     EDMP_APEX_STATE_TILT_QUAT_WB_Y_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_QUAT_WB_Y = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_QUAT_WB_Z,
			     EDMP_APEX_STATE_TILT_QUAT_WB_Z_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_QUAT_WB_Z = %d", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_QUAT_REF_X,
			     EDMP_APEX_STATE_TILT_QUAT_REF_X_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_QUAT_REF_X = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_QUAT_REF_Y,
			     EDMP_APEX_STATE_TILT_QUAT_REF_Y_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_QUAT_REF_Y = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_QUAT_REF_Z,
			     EDMP_APEX_STATE_TILT_QUAT_REF_Z_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_QUAT_REF_Z = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_TILT_ANGLE,
			     EDMP_APEX_STATE_TILT_TILT_ANGLE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_TILT_ANGLE = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_TILT_TIMER,
			     EDMP_APEX_STATE_TILT_TIMER_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_TILT_TIMER = %d", dump.data_long);

	/* outputs */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_TILT_OUTINTERRUPT,
			     EDMP_APEX_INTERF_TILT_OUTINTERRUPT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_TILT_OUTInterrupt = %d", dump.data_short);

	INV_MSG(INV_MSG_LEVEL_INFO, "");
}

static void print_smd_vars(void *self)
{
	union var_types_t dump;

	/* inputs */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 0, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[0] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 4, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[1] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 8, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[2] = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 0, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[0] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 4, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[1] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 8, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[2] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 12, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[3] = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_ER1_Y1, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "ped_ER1_y1 = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_ER2_Y1, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "ped_ER2_y1 = 0x%08x", dump.data_long);

	/* params / states */
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION_RATIO,
			     EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION_RATIO_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO,
		"EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION_RATIO = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_MAGNITUDE_THRESHOLD,
			     EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_MAGNITUDE_THRESHOLD_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO,
		"EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_MAGNITUDE_THRESHOLD = 0x%08x",
		dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_K,
			     EDMP_APEX_STATE_SMD_VIBRATION_K_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_K = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_K_1,
			     EDMP_APEX_STATE_SMD_VIBRATION_K_1_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_K_1 = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV + 0,
			     EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV0 = 0x%08x",
		dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV + 4,
			     EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV1 = 0x%08x",
		dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV + 8,
			     EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_Q25_PREV2 = 0x%08x",
		dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF + 0,
			     EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF0 = 0x%08x",
		dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF + 4,
			     EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF1 = 0x%08x",
		dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF + 8,
			     EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF2 = 0x%08x",
		dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION,
			     EDMP_APEX_STATE_SMD_VIBRATION_VIBRATION_DETECTION_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VIBRATION_ACCEL_DELTA_LPF3 = 0x%08x",
		dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_ER_ALPHA1,
			     EDMP_APEX_STATE_SMD_ER_ALPHA1_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_ER_ALPHA1 = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_ER_ALPHA3,
			     EDMP_APEX_STATE_SMD_ER_ALPHA3_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_ER_ALPHA3 = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_MODE_WINDOW,
			     EDMP_APEX_STATE_SMD_MODE_WINDOW_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_MODE_WINDOW = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_SENSITIVITY,
			     EDMP_APEX_STATE_SMD_SENSITIVITY_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_SENSITIVITY = %x", dump.data_uchar[0]);

	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_SMD_LOWVARIANCECOUNTER,
			     EDMP_APEX_STATE_SMD_LOWVARIANCECOUNTER_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_LOWVarianceCounter = 0x%08x",
		dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_SMDDELAYCOUNTER,
			     EDMP_APEX_STATE_SMD_SMDDELAYCOUNTER_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_SMDDelayCounter = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_E1_SHORT,
			     EDMP_APEX_STATE_SMD_E1_SHORT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_E1_SHORT = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_E2_SHORT,
			     EDMP_APEX_STATE_SMD_E2_SHORT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_E2_SHORT = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_VAR_RUN_TEMP,
			     EDMP_APEX_STATE_SMD_VAR_RUN_TEMP_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_VAR_RUN_TEMP = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_Z_MEAN,
			     EDMP_APEX_STATE_SMD_Z_MEAN_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_Z_MEAN = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_Z_INCR,
			     EDMP_APEX_STATE_SMD_Z_INCR_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_Z_INCR = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_MODE_COUNTER,
			     EDMP_APEX_STATE_SMD_MODE_COUNTER_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_MODE_COUNTER = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_STATE_M_F,
			     EDMP_APEX_STATE_SMD_STATE_M_F_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_STATE_M_F = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_QUAT_WB1,
			     EDMP_APEX_STATE_SMD_QUAT_WB1_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_QUAT_WB1 = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_QUAT_WB2,
			     EDMP_APEX_STATE_SMD_QUAT_WB2_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_QUAT_WB2 = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_QUAT_WB3,
			     EDMP_APEX_STATE_SMD_QUAT_WB3_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_QUAT_WB3 = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_MOTION_FLAG,
			     EDMP_APEX_STATE_SMD_MOTION_FLAG_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_MOTION_FLAG = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_WALK_CANDIDATE,
			     EDMP_APEX_STATE_SMD_WALK_CANDIDATE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_WALK_CANDIDATE = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_TILT_ANGLE,
			     EDMP_APEX_STATE_SMD_TILT_ANGLE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_TILT_ANGLE = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_PREV_QUAT_WB1,
			     EDMP_APEX_STATE_SMD_PREV_QUAT_WB1_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_PREV_QUAT_WB1 = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_PREV_QUAT_WB2,
			     EDMP_APEX_STATE_SMD_PREV_QUAT_WB2_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_PREV_QUAT_WB2 = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_PREV_QUAT_WB3,
			     EDMP_APEX_STATE_SMD_PREV_QUAT_WB3_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_PREV_QUAT_WB3 = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_TILT_INIT_FLAG,
			     EDMP_APEX_STATE_SMD_TILT_INIT_FLAG_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_TILT_init_flag = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_DECIM_RATE,
			     EDMP_APEX_STATE_SMD_DECIM_RATE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_DECIM_RATE = %x", dump.data_uchar[0]);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_SMD_DECIM_COUNT,
			     EDMP_APEX_STATE_SMD_DECIM_COUNT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_STATE_SMD_DECIM_COUNT = %x", dump.data_uchar[0]);

	/* outputs */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_SMD_FLAG,
			     EDMP_APEX_INTERF_SMD_FLAG_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "SMD_flag = = 0x%08x", dump.data_long);
}

static void print_pedo_vars(void *self)
{
	union var_types_t dump;

	/* inputs */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 0, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[0] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 4, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[1] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_SCALED + 8, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_SCALED[2] = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 0, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[0] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 4, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[1] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 8, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[2] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_QUAT + 12, 4);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_QUAT[3] = 0x%08x", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ACCEL_WZ,
			     EDMP_APEX_INTERF_ACCEL_WZ_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ACCEL_WZ = 0x%08x", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_INTERF_ODR_SENSORDATA,
			     EDMP_APEX_INTERF_ODR_SENSORDATA_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "EDMP_APEX_INTERF_ODR_SENSORData = %d", dump.data_long);

	/* params / states */
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_DECIM_RATE,
			     EDMP_APEX_STATE_PED_DECIM_RATE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->decim_rate = %d", dump.data_uchar[0]);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_DECIM_COUNT,
			     EDMP_APEX_STATE_PED_DECIM_COUNT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->decim_count = %d", dump.data_uchar[0]);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_ODR_PED, EDMP_ODR_PED_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->working_frequency = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_SB_THRESHOLD,
			     EDMP_APEX_STATE_PED_SB_THRESHOLD_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->sb_threshold = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_SB_THRESHOLD2,
			     EDMP_APEX_STATE_PED_SB_THRESHOLD2_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->sb_threshold2 = %d", dump.data_short);

	inv_imu_read_dmp_ram(self, dump.data_uchar,
			     (uint32_t)EDMP_APEX_STATE_PED_SB_TIMER_THRESHOLD,
			     EDMP_APEX_STATE_PED_SB_TIMER_THRESHOLD_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->sb_timer_threshold = %d", dump.data_short);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_LOW_ENERGY_AMP_TH,
			     EDMP_APEX_STATE_PED_LOW_ENERGY_AMP_TH_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->ped_low_en_amp_th = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_SENSITIVITY_MODE,
			     EDMP_APEX_STATE_PED_SENSITIVITY_MODE_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->ped_sensitivity_mode = %d",
		dump.data_uchar[0]);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_PEAK_THRESHOLD,
			     EDMP_APEX_STATE_PED_PEAK_THRESHOLD_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->peak_threshold = %d", dump.data_long);

	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_STEP_DURATION,
			     EDMP_APEX_STATE_PED_STEP_DURATION_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->step_duration = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_DTC_N_HAT,
			     EDMP_APEX_STATE_PED_DTC_N_HAT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->dtc_n_hat = %d", dump.data_long);
	inv_imu_read_dmp_ram(self, dump.data_uchar, (uint32_t)EDMP_APEX_STATE_PED_DTC_FAST_N_HAT,
			     EDMP_APEX_STATE_PED_DTC_FAST_N_HAT_SIZE);
	INV_MSG(INV_MSG_LEVEL_INFO, "state_pedometer->dtc_fast_n_hat = %d\n", dump.data_long);

	inv_imu_sleep_us(self, 5000);
}

#endif
