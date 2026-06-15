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

#include "imu/inv_imu_driver_internal.h"
#include "imu/inv_imu_edmp_memmap_internal.h"

int inv_imu_read_chip_config(inv_imu_device_t *s, inv_imu_chip_config_t *cfg)
{
	int status = INV_IMU_OK;
	ipreg_sys1_reg_158_t ipreg_sys1_reg_158;
	ipreg_sys2_reg_112_t ipreg_sys2_reg_112;

	status |= inv_imu_read_reg(s, IPREG_SYS1_REG_158, 1, (uint8_t *)&ipreg_sys1_reg_158);
	cfg->gyro_ui_filt_ord_ind = ipreg_sys1_reg_158.gyro_ui_lpfbw_sel;
	status |= inv_imu_read_reg(s, IPREG_SYS2_REG_112, 1, (uint8_t *)&ipreg_sys2_reg_112);
	cfg->accel_ui_filt_ord_ind = ipreg_sys2_reg_112.accel_ui_lpfbw_sel;

	return status;
}

int inv_read_manufacturing_info(inv_imu_device_t *s, inv_imu_manufacturing_info_t *info)
{
	int status = INV_IMU_OK;
	int i;
	uint8_t manuf_info[OTP_MANUFACT_INFO_SIZE];

	status |= inv_imu_adv_power_up_sram(s);

	for (i = 0; i < OTP_MANUFACT_INFO_SIZE; i++) {
		manuf_info[i] = inv_imu_read_otp(s, OTP_MANUFACT_INFO_ADDR + i);
	}

	info->die_id = ((uint16_t)(manuf_info[1] & 0x7F) << 8) | manuf_info[0];
	info->lot_id[0] = manuf_info[2] & 0x3F;
	info->lot_id[1] = ((manuf_info[2] & 0xC0) >> 6) | ((manuf_info[3] & 0x0F) << 2);
	info->lot_id[2] = ((manuf_info[3] & 0xF0) >> 4) | ((manuf_info[4] & 0x03) << 4);
	info->lot_id[3] = (manuf_info[4] & 0xFC) >> 2;
	info->lot_id[4] = manuf_info[5] & 0x3F;
	info->lot_id[5] = ((manuf_info[5] & 0xC0) >> 6) | ((manuf_info[6] & 0x0F) << 2);
	info->wafer_id = ((manuf_info[6] & 0xF0) >> 4) | ((manuf_info[7] & 0x01) << 4);
	info->die_rev = (manuf_info[7] & 0xFC) >> 2;

	return status;
}

int inv_imu_check_memory(inv_imu_device_t *s, const uint8_t *data, uint32_t sram_addr,
			 uint32_t size)
{
	uint32_t i, j;
	uint8_t readBytes[4];

	for (i = 0; i < size; i = i + sizeof(readBytes)) {
		int status = inv_imu_read_sram(s, sram_addr + i, sizeof(readBytes), readBytes);
		/* Return error if read failed */
		if (status) {
			return -1;
		}

		for (j = 0; (j < sizeof(readBytes)) && (i + j < size); j++) {
			if (readBytes[j] != data[i + j]) {
				return -1;
			}
		}
	}
	return 0;
}

int inv_imu_adv_set_default_pad_scenario(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	ioc_pad_scenario_aux_ovrd_internal_t ioc_pad_scenario_aux_ovrd;
	ioc_pad_scenario_ovrd_internal_t ioc_pad_scenario_ovrd;

	status |= inv_imu_read_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1,
				   (uint8_t *)&ioc_pad_scenario_aux_ovrd);
	ioc_pad_scenario_aux_ovrd.aux1_enable_ovrd = 1;
	ioc_pad_scenario_aux_ovrd.aux1_enable_ovrd_val = 1;
	status |= inv_imu_write_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1,
				    (uint8_t *)&ioc_pad_scenario_aux_ovrd);

	status |= inv_imu_read_reg(s, IOC_PAD_SCENARIO_OVRD, 1, (uint8_t *)&ioc_pad_scenario_ovrd);
	ioc_pad_scenario_ovrd.pads_int2_cfg_ovrd_val = 0;
	ioc_pad_scenario_ovrd.pads_int2_cfg_ovrd = 1;
	status |= inv_imu_write_reg(s, IOC_PAD_SCENARIO_OVRD, 1, (uint8_t *)&ioc_pad_scenario_ovrd);

	return status;
}

uint8_t inv_imu_read_otp(void *icm_driver, uint16_t address)
{
	uint8_t data, read_value;
	uint16_t data16;

	data = 0x00;
	inv_imu_write_reg(icm_driver, OTP_CTRL, 1, &data);
	inv_imu_sleep_us(icm_driver, 350);

	data16 = 0x0000;
	inv_imu_write_reg(icm_driver, OTP_MRA_0, 2, (uint8_t *)&data16);
	inv_imu_write_reg(icm_driver, OTP_MRB_0, 2, (uint8_t *)&data16);
	inv_imu_write_reg(icm_driver, OTP_MR_0, 2, (uint8_t *)&data16);

	inv_imu_sleep_us(icm_driver, 1);

	inv_imu_read_reg(icm_driver, address, 1, &read_value);
	data = 0x06;
	inv_imu_write_reg(icm_driver, OTP_CTRL, 1, &data);

	return read_value;
}

/* Check ROM content is the expected one. If not, load the expected version. */
int inv_imu_check_eDMP_image(void *s)
{
	int inv_imu_adv_power_up_rom(inv_imu_device_t * s);
	int inv_imu_adv_power_down_rom(inv_imu_device_t * s);

	int rc = 0;
	static uint8_t ref[] = {
#include "imu/edmp_prgm_rom.h"
	};
	fpga_ctrl_t fpga_ctrl;

	/* Enable writing to ROM */
	inv_imu_read_reg(s, FPGA_CTRL, 1, (uint8_t *)&fpga_ctrl);
	fpga_ctrl.bram_wr_dis = 0;
	inv_imu_write_reg(s, FPGA_CTRL, 1, (uint8_t *)&fpga_ctrl);

	rc = inv_imu_check_memory(s, ref, (uint32_t)0x4000, sizeof(ref));

	if (rc < 0) {
		rc = inv_imu_write_sram(s, (uint32_t)0x4000, sizeof(ref), ref);
		if (rc < 0) {
			return rc;
		}
	}

	return rc;
}

/*write OTP byte address is physical address */
int inv_imu_write_otp(void *icm_driver, uint16_t address, uint8_t data, uint8_t soak)
{
	uint8_t i;
	uint8_t mask = 1;
	uint8_t error = 0;
	uint8_t data8;
	uint16_t data16;

	data8 = 0x00;
	inv_imu_write_reg(icm_driver, OTP_CTRL, 1, &data8);

	inv_imu_sleep_us(icm_driver, 350);

	/*commands for write READ_1 */
	data16 = 0x4021;
	inv_imu_write_reg(icm_driver, OTP_MRA_0, 2, (uint8_t *)&data16);
	data16 = 0x0018;
	inv_imu_write_reg(icm_driver, OTP_MRB_0, 2, (uint8_t *)&data16);
	data16 = 0x00A0;
	inv_imu_write_reg(icm_driver, OTP_MR_0, 2, (uint8_t *)&data16);

	inv_imu_sleep_us(icm_driver, 350);

	/*true byte */
	for (i = 0; i < 8; i++) {
		if (data & (mask << i)) {
			data8 = ~(mask << i);
			inv_imu_write_reg(icm_driver, address, 1, &data8);

			inv_imu_sleep_us(icm_driver, 500);
		}
	}

	/*check data */
	inv_imu_read_reg(icm_driver, address, 1, &i);
	if (i != data) {
		error = 1;
	}

	/*READ 2   */
	data16 = 0x4020;
	inv_imu_write_reg(icm_driver, OTP_MRA_0, 2, (uint8_t *)&data16);
	data16 = 0x4021;
	inv_imu_write_reg(icm_driver, OTP_MRB_0, 2, (uint8_t *)&data16);
	data16 = 0x0016;
	inv_imu_write_reg(icm_driver, OTP_MR_0, 2, (uint8_t *)&data16);

	inv_imu_read_reg(icm_driver, address, 1, &i);
	if (i != data) {
		error |= 0x4;
	}

	i = inv_imu_read_otp(icm_driver, address);

	if (i != data) {
		error |= 0x10;
	}

	/*redundant */
	data8 = 0x06;
	inv_imu_write_reg(icm_driver, OTP_CTRL, 1, &data8);
	return error;
}

int inv_imu_set_sensor_data_length(inv_imu_device_t *s, inv_imu_sensor_reg_len_t data_length)
{
	int status = INV_IMU_OK;
	sreg_ctrl_internal_t sreg_ctrl;

	status |= inv_imu_read_reg(s, SREG_CTRL, 1, (uint8_t *)&sreg_ctrl);
	sreg_ctrl.sreg_sifs_20bits_en = (uint8_t)data_length;
	status |= inv_imu_write_reg(s, SREG_CTRL, 1, (uint8_t *)&sreg_ctrl);

	return status;
}

int inv_imu_get_sensor_data_length(inv_imu_device_t *s,
				   inv_imu_sensor_reg_len_t *sensor_data_length)
{
	int status = INV_IMU_OK;
	sreg_ctrl_internal_t sreg_ctrl;

	status |= inv_imu_read_reg(s, SREG_CTRL, 1, (uint8_t *)&sreg_ctrl);
	if (!status) {
		*sensor_data_length = (inv_imu_sensor_reg_len_t)sreg_ctrl.sreg_sifs_20bits_en;
	}

	return status;
}

int inv_imu_get_register_data_internal(inv_imu_device_t *s, inv_imu_sensor_data_internal_t *data)
{
	int status = INV_IMU_OK;
	inv_imu_sensor_reg_len_t sensor_data_length = INV_IMU_20_BIT_DATA;

	status |= inv_imu_get_sensor_data_length(s, &sensor_data_length);

	if (sensor_data_length == INV_IMU_16_BIT_DATA) {
		status |= inv_imu_read_reg(s, ACCEL_DATA_X_0, sizeof(inv_imu_sensor_data_t),
					   (uint8_t *)data);

		/* Format accel data from sensor registers. */
		FORMAT_16_BITS_DATA(s->endianness_data,
				    (uint8_t *)&data->sixteen_bits.accel_data[0],
				    (uint16_t *)&data->sixteen_bits.accel_data[0]);
		FORMAT_16_BITS_DATA(s->endianness_data,
				    (uint8_t *)&data->sixteen_bits.accel_data[1],
				    (uint16_t *)&data->sixteen_bits.accel_data[1]);
		FORMAT_16_BITS_DATA(s->endianness_data,
				    (uint8_t *)&data->sixteen_bits.accel_data[2],
				    (uint16_t *)&data->sixteen_bits.accel_data[2]);
		/* Format gyro data from sensor registers. */
		FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->sixteen_bits.gyro_data[0],
				    (uint16_t *)&data->sixteen_bits.gyro_data[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->sixteen_bits.gyro_data[1],
				    (uint16_t *)&data->sixteen_bits.gyro_data[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->sixteen_bits.gyro_data[2],
				    (uint16_t *)&data->sixteen_bits.gyro_data[2]);
		/* Format temperature data from sensor registers. */
		FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->sixteen_bits.temp_data,
				    (uint16_t *)&data->sixteen_bits.temp_data);
	} else { /* 20-bit data */

		uint8_t high_order_data[ACCEL_DATA_SIZE + GYRO_DATA_SIZE + TEMP_DATA_SIZE];
		uint8_t low_order_data[EXT_DATA_SIZE];

		status |= inv_imu_read_reg(s, ACCEL_DATA_X_0, sizeof(high_order_data),
					   high_order_data);
		status |= inv_imu_read_reg(s, EXT_DATA_X, sizeof(low_order_data), low_order_data);

		/* Format accel data from sensor registers. */
		FORMAT_20_BITS_DATA(s->endianness_data, (uint8_t *)&high_order_data[0],
				    (low_order_data[0] & 0x0F), &data->twenty_bits.accel_data[0]);
		FORMAT_20_BITS_DATA(s->endianness_data, (uint8_t *)&high_order_data[2],
				    (low_order_data[1] & 0x0F), &data->twenty_bits.accel_data[1]);
		FORMAT_20_BITS_DATA(s->endianness_data, (uint8_t *)&high_order_data[4],
				    (low_order_data[2] & 0x0F), &data->twenty_bits.accel_data[2]);
		/* Format gyro data from sensor registers. */
		FORMAT_20_BITS_DATA(s->endianness_data, (uint8_t *)&high_order_data[6],
				    (low_order_data[0] >> 4), &data->twenty_bits.gyro_data[0]);
		FORMAT_20_BITS_DATA(s->endianness_data, (uint8_t *)&high_order_data[8],
				    (low_order_data[1] >> 4), &data->twenty_bits.gyro_data[1]);
		FORMAT_20_BITS_DATA(s->endianness_data, (uint8_t *)&high_order_data[10],
				    (low_order_data[2] >> 4), &data->twenty_bits.gyro_data[2]);

		/* Make sure that 20-bit negative data is negative when it's in an int32_t */
		for (int ii = 0; ii < 3; ii++) {
			if (data->twenty_bits.accel_data[ii] & 0x8000) {
				data->twenty_bits.accel_data[ii] |= 0xFFF00000;
			}
			if (data->twenty_bits.gyro_data[ii] & 0x8000) {
				data->twenty_bits.gyro_data[ii] |= 0xFFF00000;
			}
		}
		/* Format temperature data from sensor registers. */
		FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&high_order_data[12],
				    &data->twenty_bits.temp_data);
	}
	return status;
}

int inv_imu_adv_get_data_from_registers_internal(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	inv_imu_adv_var_t *e = (inv_imu_adv_var_t *)s->adv_var;
	inv_imu_sensor_event_t event = {0};
	uint8_t data[ACCEL_DATA_SIZE + GYRO_DATA_SIZE + TEMP_DATA_SIZE];
	uint8_t ext_data[3];
#if INV_IMU_FSYNC_SUPPORTED
	uint8_t fsync_tag_in_accel = 0;
	uint8_t fsync_tag_in_gyro = 0;
	uint8_t fsync_tag_in_temp = 0;
#endif
	inv_imu_sensor_reg_len_t sensor_data_length = INV_IMU_20_BIT_DATA;

	status |= inv_imu_get_sensor_data_length(s, &sensor_data_length);

	/* Read sensor data from registers. */
	status |= inv_imu_read_reg(s, ACCEL_DATA_X_0, sizeof(data), data);

	/* Read accel data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[0], (uint16_t *)&event.accel[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[2], (uint16_t *)&event.accel[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[4], (uint16_t *)&event.accel[2]);

	if ((event.accel[0] != INVALID_VALUE_FIFO) && (event.accel[1] != INVALID_VALUE_FIFO) &&
	    (event.accel[2] != INVALID_VALUE_FIFO)) {
		event.sensor_mask |= (1 << INV_SENSOR_ACCEL);
	}

	/* Gyro */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[6], (uint16_t *)&event.gyro[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[8], (uint16_t *)&event.gyro[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[10], (uint16_t *)&event.gyro[2]);
	if ((event.gyro[0] != INVALID_VALUE_FIFO) && (event.gyro[1] != INVALID_VALUE_FIFO) &&
	    (event.gyro[2] != INVALID_VALUE_FIFO)) {
		event.sensor_mask |= (1 << INV_SENSOR_GYRO);
	}

	/* Temperature */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[12], (uint16_t *)&event.temperature);
	if (event.temperature != INVALID_VALUE_FIFO) {
		event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
	}

	if (sensor_data_length == INV_IMU_20_BIT_DATA) {
		status |= inv_imu_read_reg(s, EXT_DATA_X, sizeof(ext_data), ext_data);
		event.accel_high_res[0] = ext_data[0] & 0x0F;
		event.accel_high_res[1] = ext_data[1] & 0x0F;
		event.accel_high_res[2] = ext_data[2] & 0x0F;
		event.gyro_high_res[0] = ext_data[0] >> 4;
		event.gyro_high_res[1] = ext_data[1] >> 4;
		event.gyro_high_res[2] = ext_data[2] >> 4;
	}

#if INV_IMU_FSYNC_SUPPORTED
	/*
	 * Check if fsync flag is set and then get FSYNC counter.
	 */
	switch (e->fsync_tag) {
	case FSYNC_CONFIG0_AP_FSYNC_ACCEL_X:
		fsync_tag_in_accel =
			(event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[0] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_ACCEL_Y:
		fsync_tag_in_accel =
			(event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[1] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_ACCEL_Z:
		fsync_tag_in_accel =
			(event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[2] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_GYRO_X:
		fsync_tag_in_gyro =
			(event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[0] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_GYRO_Y:
		fsync_tag_in_gyro =
			(event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[1] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_GYRO_Z:
		fsync_tag_in_gyro =
			(event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[2] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_TEMP:
		fsync_tag_in_temp = (event.sensor_mask & (1 << INV_SENSOR_TEMPERATURE)) &&
				    (event.temperature & 0x1);
		break;
	default:
		break;
	}

	/* Sensor data register is configured to expose fsync flag. If fsync flag is set, process
	 * fsync counter. */
	if (fsync_tag_in_accel || fsync_tag_in_gyro || fsync_tag_in_temp) {
		uint8_t fsync_count[2];

		/*
		 * Read 16bits fsync counter containing time elapsed between last FSYNC interrupt
		 * and last ODR event. Fsync delta time depends on data endianness as counter is
		 * read over 2 registers and timestamp resolution.
		 */
		status |= inv_imu_read_reg(s, TMST_FSYNC_DATA_0, 2, fsync_count);
		FORMAT_16_BITS_DATA(s->endianness_data, &fsync_count[0],
				    (uint16_t *)&event.timestamp_fsync);

		event.sensor_mask |= (1 << INV_SENSOR_FSYNC_EVENT);
	}
#endif

	/* call sensor event callback */
	if (e->sensor_event_cb) {
		e->sensor_event_cb(&event);
	}

	return status;
}
