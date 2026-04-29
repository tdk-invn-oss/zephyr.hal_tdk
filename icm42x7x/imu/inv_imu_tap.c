/*
 *
 * Copyright (c) [2025] by InvenSense, Inc.
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

#include "inv_imu_tap.h"
#include "icm42x7x/imu/inv_imu_extfunc.h"

/* Static function and variables definition */
static int read_dmp_memory(inv_imu_device_t *s, uint8_t *out, uint32_t bank, uint32_t offset,
                           uint32_t size);
static int read_and_check_dmp_memory(inv_imu_device_t *s, const uint8_t *data, uint32_t bank,
                                     uint32_t offset, uint32_t size);
static int write_dmp_memory(inv_imu_device_t *s, const uint8_t *data, uint32_t bank,
                            uint32_t offset, uint32_t size);
static int load_dmp_sram_code(inv_imu_device_t *s, const uint8_t *dmp_prog, const uint32_t offset,
                              const uint32_t size);

static uint8_t dmp_image[] = {
#include "dmp3Default_xian_triple_tap.txt"
};

#define DMP_TAP_IMG_LOAD_START_ADDRESS  0
#define DMP_TAP_IMG_FETCH_START_ADDRESS (DMP_TAP_IMG_LOAD_START_ADDRESS + (12 * 16))

/** @defgroup TapParams Tap parameters
 *  @brief Addresses where to find tap parameters in the image
 *  @sa inv_imu_tap_parameters_t
 *  @{
 */
/* Algorithm parameters */
#define TAP_PARAM_MIN_JERK_THR      (2 * 16 + 0)
#define TAP_PARAM_MIN               (2 * 16 + 8)
#define TAP_PARAM_MAX               (2 * 16 + 4)
#define TAP_PARAM_TMIN              (2 * 16 + 10)
#define TAP_PARAM_TMAX              (2 * 16 + 14)
#define TAP_PARAM_TAVG              (3 * 16 + 10)
#define TAP_PARAM_MAX_PEAK_TOL      (4 * 16 + 12)
#define TAP_PARAM_SMUDGE_REJECT_THR (5 * 16 + 8)
/* Power save parameters */
#define TAP_POWER_SAVE_ENABLE (11 * 16 + 8)
#define TAP_POWER_SAVE_TIME   (11 * 16 + 12)
/** @} */

/** @defgroup TapOutputs Tap outputs
 *  @brief Addresses where to find tap outputs in the image
 *  @sa inv_imu_tap_event_t
 *  @{
 */
#define DOUBLE_TAP_TIMING (4 * 16 + 6)
#define TRIPLE_TAP_TIMING (4 * 16 + 10)
#define TAP_NUM           (5 * 16 + 2)
#define TAP_AXIS          (6 * 16 + 0)
#define TAP_DIR           (8 * 16 + 0)
/** @} */

/* Address of DMP_CONFIG1 register where DMP start address is written */
#define DMP_CONFIG1_MREG1 0x2c

/* Global masks for all possible tap interrupts */
#define INT_SOURCE6_TAP_INT1_EN_MASK                                                               \
	(INT_SOURCE6_SINGLE_TAP_INT1_EN_MASK | INT_SOURCE6_DOUBLE_TAP_INT1_EN_MASK |                   \
	 INT_SOURCE6_TRIPLE_TAP_INT1_EN_MASK)
#define INT_SOURCE7_TAP_INT2_EN_MASK                                                               \
	(INT_SOURCE7_SINGLE_TAP_INT2_EN_MASK | INT_SOURCE7_DOUBLE_TAP_INT2_EN_MASK |                   \
	 INT_SOURCE7_TRIPLE_TAP_INT2_EN_MASK)

/* API implementation */
int inv_imu_tap_load_dmp(inv_imu_device_t *s)
{
	/* Load DMP tap image in DMP SRAM and set start address */
	return load_dmp_sram_code(s, dmp_image, DMP_TAP_IMG_LOAD_START_ADDRESS,
	                          sizeof(dmp_image) / sizeof(dmp_image[0]));
}

int inv_imu_tap_set_config_int1(inv_imu_device_t *s, const inv_imu_tap_interrupt_parameter_t *it)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, INT_SOURCE6_MREG1, 1, &data);
	data &= ~INT_SOURCE6_TAP_INT1_EN_MASK;
	data |= it->INV_SINGLE_TAP_DET << INT_SOURCE6_SINGLE_TAP_INT1_EN_POS;
	data |= it->INV_DOUBLE_TAP_DET << INT_SOURCE6_DOUBLE_TAP_INT1_EN_POS;
	data |= it->INV_TRIPLE_TAP_DET << INT_SOURCE6_TRIPLE_TAP_INT1_EN_POS;
	status |= inv_imu_write_reg(s, INT_SOURCE6_MREG1, 1, &data);

	return status;
}

int inv_imu_tap_get_config_int1(inv_imu_device_t *s, inv_imu_tap_interrupt_parameter_t *it)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, INT_SOURCE6_MREG1, 1, &data);
	data                   = data & INT_SOURCE6_TAP_INT1_EN_MASK;
	it->INV_SINGLE_TAP_DET = (inv_imu_interrupt_value)(data >> INT_SOURCE6_SINGLE_TAP_INT1_EN_POS);
	it->INV_DOUBLE_TAP_DET = (inv_imu_interrupt_value)(data >> INT_SOURCE6_DOUBLE_TAP_INT1_EN_POS);
	it->INV_TRIPLE_TAP_DET = (inv_imu_interrupt_value)(data >> INT_SOURCE6_TRIPLE_TAP_INT1_EN_POS);
	return status;
}

int inv_imu_tap_set_config_int2(inv_imu_device_t *s, const inv_imu_tap_interrupt_parameter_t *it)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, INT_SOURCE7_MREG1, 1, &data);
	data &= ~INT_SOURCE7_TAP_INT2_EN_MASK;
	data |= it->INV_SINGLE_TAP_DET << INT_SOURCE7_SINGLE_TAP_INT2_EN_POS;
	data |= it->INV_DOUBLE_TAP_DET << INT_SOURCE7_DOUBLE_TAP_INT2_EN_POS;
	data |= it->INV_TRIPLE_TAP_DET << INT_SOURCE7_TRIPLE_TAP_INT2_EN_POS;
	status |= inv_imu_write_reg(s, INT_SOURCE7_MREG1, 1, &data);

	return status;
}

int inv_imu_tap_get_config_int2(inv_imu_device_t *s, inv_imu_tap_interrupt_parameter_t *it)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, INT_SOURCE7_MREG1, 1, &data);
	data                   = data & INT_SOURCE6_TAP_INT1_EN_MASK;
	it->INV_SINGLE_TAP_DET = (inv_imu_interrupt_value)(data >> INT_SOURCE7_SINGLE_TAP_INT2_EN_POS);
	it->INV_DOUBLE_TAP_DET = (inv_imu_interrupt_value)(data >> INT_SOURCE7_DOUBLE_TAP_INT2_EN_POS);
	it->INV_TRIPLE_TAP_DET = (inv_imu_interrupt_value)(data >> INT_SOURCE7_TRIPLE_TAP_INT2_EN_POS);
	return status;
}

int inv_imu_tap_enable(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t data;
	status |= inv_imu_start_dmp(s);

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &data);
	data |= (uint8_t)APEX_CONFIG1_TAP_EN_MASK;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &data);

	return status;
}

int inv_imu_tap_disable(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &data);
	data &= (uint8_t)~APEX_CONFIG1_TAP_EN_MASK;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &data);

	return status;
}

int inv_imu_tap_init_parameters_struct(inv_imu_device_t *s, inv_imu_tap_parameters_t *params)
{
	int status = 0;

	/* Default parameters at DMP image loading time */
	params->power_save      = APEX_CONFIG0_DMP_POWER_SAVE_DIS;
	params->power_save_time = APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_4_S;
	/* We want to detect single, double and triple tap */
	params->tap_max                = 3;
	params->tap_min                = 1;
	params->tap_tmax               = 450; /*1.1s at 400 Hz*/
	params->tap_tmin               = 66; /*165ms at 400 Hz*/
	params->tap_max_peak_tol       = 2; /*25% peak amplitude*/
	params->tap_tavg               = 8; /*8 samples to measure energy*/
	params->tap_min_jerk_threshold = 4608; /*1.125 gee*/
	params->tap_smudge_rejection   = 34; /*85ms at 400 Hz*/

	return status;
}

int inv_imu_tap_configure_parameters(inv_imu_device_t *s, const inv_imu_tap_parameters_t *params)
{
	int      status = 0;
	uint8_t  data;
	uint32_t tmp;

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &data);

	/* DMP cannot be configured if it is running, hence make sure all APEX algorithms are off */
	if (data & (APEX_CONFIG1_SMD_ENABLE_MASK | APEX_CONFIG1_PED_ENABLE_MASK |
	            APEX_CONFIG1_TILT_ENABLE_MASK | APEX_CONFIG1_FF_ENABLE_MASK))
		return INV_ERROR;

	/* Tap parameters handling */
	dmp_image[TAP_PARAM_MIN_JERK_THR + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] =
	    (uint8_t)(params->tap_min_jerk_threshold >> 8);
	dmp_image[TAP_PARAM_MIN_JERK_THR + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] =
	    (uint8_t)params->tap_min_jerk_threshold;

	dmp_image[TAP_PARAM_MIN + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_PARAM_MIN + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] = params->tap_min;

	dmp_image[TAP_PARAM_MAX + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_PARAM_MAX + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] = params->tap_max;

	dmp_image[TAP_PARAM_TMIN + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_PARAM_TMIN + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] = params->tap_tmin;

	dmp_image[TAP_PARAM_TMAX + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] =
	    (uint8_t)(params->tap_tmax >> 8);
	dmp_image[TAP_PARAM_TMAX + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] = (uint8_t)params->tap_tmax;

	dmp_image[TAP_PARAM_TAVG + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_PARAM_TAVG + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] = params->tap_tavg;

	dmp_image[TAP_PARAM_MAX_PEAK_TOL + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_PARAM_MAX_PEAK_TOL + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] =
	    params->tap_max_peak_tol;

	dmp_image[TAP_PARAM_SMUDGE_REJECT_THR + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_PARAM_SMUDGE_REJECT_THR + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] =
	    params->tap_smudge_rejection;

	/* Power save mode handling */
	data = (uint8_t)params->power_save >> APEX_CONFIG0_DMP_POWER_SAVE_EN_POS;
	dmp_image[TAP_POWER_SAVE_ENABLE + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_POWER_SAVE_ENABLE + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_POWER_SAVE_ENABLE + 2 - DMP_TAP_IMG_LOAD_START_ADDRESS] = 0;
	dmp_image[TAP_POWER_SAVE_ENABLE + 3 - DMP_TAP_IMG_LOAD_START_ADDRESS] = data;

	tmp = (uint32_t)params->power_save_time * 4 /*enum to integer*/ * 400 /*seconds to sample*/;
	dmp_image[TAP_POWER_SAVE_TIME + 0 - DMP_TAP_IMG_LOAD_START_ADDRESS] = (uint8_t)(tmp >> 24);
	dmp_image[TAP_POWER_SAVE_TIME + 1 - DMP_TAP_IMG_LOAD_START_ADDRESS] = (uint8_t)(tmp >> 16);
	dmp_image[TAP_POWER_SAVE_TIME + 2 - DMP_TAP_IMG_LOAD_START_ADDRESS] = (uint8_t)(tmp >> 8);
	dmp_image[TAP_POWER_SAVE_TIME + 3 - DMP_TAP_IMG_LOAD_START_ADDRESS] = (uint8_t)tmp;

	return status;
}

int inv_imu_tap_get_data(inv_imu_device_t *s, inv_imu_tap_event_t *tap_out)
{
	int      status = 0;
	uint16_t tmp;

	status |= read_dmp_memory(s, (uint8_t *)&tmp, 0x50, TAP_NUM, 2u);
	tap_out->tap_num = (uint8_t)(tmp >> 8);
	status |= read_dmp_memory(s, (uint8_t *)&tmp, 0x50, TAP_AXIS, 2u);
	tap_out->tap_axis = (uint8_t)(tmp >> 8);
	status |= read_dmp_memory(s, (uint8_t *)&tmp, 0x50, TAP_DIR, 2u);
	tap_out->tap_direction = (uint8_t)(tmp >> 8);
	if (2u == tap_out->tap_num) {
		status |= read_dmp_memory(s, (uint8_t *)&tmp, 0x50, DOUBLE_TAP_TIMING, 2u);
		tap_out->double_tap_timing = ((tmp << 8) & 0xff00) | ((tmp >> 8) & 0xff);
	} else {
		/* Double tap timing only makes sense if it was an actual double tap */
		tap_out->double_tap_timing = 0;
	}
	if (3u == tap_out->tap_num) {
		status |= read_dmp_memory(s, (uint8_t *)&tmp, 0x50, TRIPLE_TAP_TIMING, 2u);
		tap_out->triple_tap_timing = ((tmp << 8) & 0xff00) | ((tmp >> 8) & 0xff);
	} else {
		/* Triple tap timing only makes sense if it was an actual double tap */
		tap_out->triple_tap_timing = 0;
	}

	return status;
}

/* Static functions implementation */
static int read_dmp_memory(inv_imu_device_t *s, uint8_t *out, uint32_t bank, uint32_t offset,
                           uint32_t size)
{
	uint8_t  memory_bank;
	uint8_t  dmp_memory_address;
	uint32_t i;
	int      status = 0;

	/* make sure mclk is on */
	inv_imu_switch_on_mclk(s);

	/* Read DMP memory and check it against memory pointed by input parameter */
	memory_bank        = bank + (offset / 256);
	dmp_memory_address = offset % 256;

	inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);
	inv_imu_sleep_us(4);
	inv_imu_write_reg(s, MADDR_R, 1, &dmp_memory_address);
	inv_imu_sleep_us(4);

	for (i = offset; i < size + offset; i++) {
		if (0 == (i % 256)) {
			memory_bank        = bank + (i / 256);
			dmp_memory_address = 0;
			inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);
			inv_imu_sleep_us(4);
			inv_imu_write_reg(s, MADDR_R, 1, &dmp_memory_address);
			inv_imu_sleep_us(4);
		}

		inv_imu_read_reg(s, M_R, 1, &out[i - offset]);
		inv_imu_sleep_us(4);
	}

	memory_bank = 0;
	inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);

	/* cancel mclk request */
	inv_imu_switch_off_mclk(s);

	return status;
}

static int read_and_check_dmp_memory(inv_imu_device_t *s, const uint8_t *data, uint32_t bank,
                                     uint32_t offset, uint32_t size)
{
	uint8_t  memory_bank;
	uint8_t  dmp_memory_address;
	uint32_t i;
	int      status = 0;

	/* make sure mclk is on */
	inv_imu_switch_on_mclk(s);

	/* Read DMP memory and check it against memory pointed by input parameter */
	memory_bank        = bank + (offset / 256);
	dmp_memory_address = offset % 256;

	inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);
	inv_imu_sleep_us(4);
	inv_imu_write_reg(s, MADDR_R, 1, &dmp_memory_address);
	inv_imu_sleep_us(4);

	for (i = offset; i < size + offset; i++) {
		uint8_t readByte;

		if (0 == (i % 256)) {
			memory_bank        = bank + (i / 256);
			dmp_memory_address = 0;
			inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);
			inv_imu_sleep_us(4);
			inv_imu_write_reg(s, MADDR_R, 1, &dmp_memory_address);
			inv_imu_sleep_us(4);
		}

		inv_imu_read_reg(s, M_R, 1, &readByte);
		inv_imu_sleep_us(4);
		if (readByte != data[i - offset]) {
			status = INV_ERROR_MEM;
			break;
		}
	}

	memory_bank = 0;
	inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);

	/* cancel mclk request */
	inv_imu_switch_off_mclk(s);

	return status;
}

static int write_dmp_memory(inv_imu_device_t *s, const uint8_t *data, uint32_t bank,
                            uint32_t offset, uint32_t size)
{
	uint8_t  memory_bank;
	uint8_t  dmp_memory_address;
	uint32_t i;
	int      status = 0;

	/* make sure mclk is on */
	inv_imu_switch_on_mclk(s);

	/* Write memory pointed by data into DMP memory */
	memory_bank        = bank + (offset / 256);
	dmp_memory_address = offset % 256;
	inv_imu_write_reg(s, BLK_SEL_W, 1, &memory_bank);
	inv_imu_sleep_us(4); /* wait 16 MCLK (4MHz) clock cycles */
	inv_imu_write_reg(s, MADDR_W, 1, &dmp_memory_address);
	inv_imu_sleep_us(4); /* wait 16 MCLK (4MHz) clock cycles */

	for (i = offset; i < size + offset; i++) {
		if (0 == (i % 256)) {
			memory_bank        = bank + (i / 256);
			dmp_memory_address = 0;
			inv_imu_write_reg(s, BLK_SEL_W, 1, &memory_bank);
			inv_imu_sleep_us(4);
			inv_imu_write_reg(s, MADDR_W, 1, &dmp_memory_address);
			inv_imu_sleep_us(4);
		}

		inv_imu_write_reg(s, M_W, 1, &data[i - offset]);
		inv_imu_sleep_us(4);
	}

	memory_bank = 0;
	inv_imu_write_reg(s, BLK_SEL_W, 1, &memory_bank);

	/* cancel mclk request */
	inv_imu_switch_off_mclk(s);

	status |= read_and_check_dmp_memory(s, data, bank, offset, size);

	return status;
}

static int load_dmp_sram_code(inv_imu_device_t *s, const uint8_t *dmp_prog, const uint32_t offset,
                              const uint32_t size)
{
	int status = 0;

	if (size + offset > 1280U)
		return INV_ERROR_SIZE;

	status |= write_dmp_memory(s, dmp_prog, 0x50, offset, size);

	if (!status) {
		uint8_t data;
		s->dmp_is_on = 1; /* set `dmp_is_on` to 1 to prevent driver from clearing SRAM */
		/* Set DMP code start address.
		 * Start address is hardcoded to ram address DMP_TAP_IMG_FETCH_START_ADDRESS. 
		 * But HW multiply value in DMP_CONFIG1_MREG1 by 32.
 		 */
		data = (DMP_TAP_IMG_FETCH_START_ADDRESS / 32);
		status |= inv_imu_write_reg(s, DMP_CONFIG1_MREG1, 1, &data);
	}

	return status;
}