/*
 * Copyright (c) 2017 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#include "icm566xx/imu/inv_imu_edmp.h"
#include "icm566xx/imu/inv_imu_edmp_defs.h"

#define EDMP_ROM_START_ADDR_IRQ0 (EDMP_ROM_BASE + 0x0000)
#define EDMP_ROM_START_ADDR_IRQ1 (EDMP_ROM_BASE + 0x0004)
#define EDMP_ROM_START_ADDR_IRQ2 (EDMP_ROM_BASE + 0x0008)

int icm566xx_edmp_clear_apex_sram(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	fifo_sram_sleep_t fifo_sram_sleep;
	uint8_t value;

	/* Same impl as icm566xx_adv_power_up_sram, duplicated here to prevent dependency */
	status |= icm566xx_read_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	fifo_sram_sleep.fifo_gsleep_shared_sram = 1;
	status |= icm566xx_write_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);

	/* Clear SRAM */
	value = 0;
	for (int i = 0; i < EDMP_RAM_SIZE; i++) {
		status |= icm566xx_write_sram(s, (uint32_t)EDMP_RAM_BASE + i, 1, &value);
	}

	return status;
}

int icm566xx_edmp_set_frequency(inv_imu_device_t *s, const dmp_ext_sen_odr_cfg_apex_odr_t frequency)
{
	int status = INV_IMU_OK;
	dmp_ext_sen_odr_cfg_t dmp_ext_sen_odr_cfg;

	status |= icm566xx_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);
	dmp_ext_sen_odr_cfg.apex_odr = (uint8_t)frequency;
	status |= icm566xx_write_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);

	return status;
}

int icm566xx_edmp_get_frequency(inv_imu_device_t *s, dmp_ext_sen_odr_cfg_apex_odr_t *frequency)
{
	int status = INV_IMU_OK;
	dmp_ext_sen_odr_cfg_t dmp_ext_sen_odr_cfg;

	status |= icm566xx_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);
	*frequency = (dmp_ext_sen_odr_cfg_apex_odr_t)dmp_ext_sen_odr_cfg.apex_odr;

	return status;
}

int icm566xx_edmp_recompute_apex_decimation(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	uint8_t value;
	reg_host_msg_t reg_host_msg;
	edmp_apex_en0_t save_edmp_apex_en0;
	edmp_apex_en1_t save_edmp_apex_en1;
	edmp_apex_en0_t edmp_apex_en0 = {0};
	edmp_apex_en1_t edmp_apex_en1 = {0};

	/*
	 * Check that DMP is turned OFF before requesting init APEX and save DMP enabled bits before
	 * requesting init procedure
	 */
	status |= icm566xx_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= icm566xx_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);
	if (save_edmp_apex_en1.edmp_enable) {
		return INV_IMU_ERROR;
	}

	/*
	 * Make sure that all DMP interrupts are masked by default, to not trigger unexpected
	 * algorithm execution when initialization is done if any sensor is running
	 */
	value = 0x3F;
	status |= icm566xx_write_reg(s, STATUS_MASK_PIN_0_7, 1, &value);
	status |= icm566xx_write_reg(s, STATUS_MASK_PIN_8_15, 1, &value);
	status |= icm566xx_write_reg(s, STATUS_MASK_PIN_16_23, 1, &value);

	/* Trigger EDMP with on-demand mode */
	status |= icm566xx_edmp_unmask_int_src(s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ON_DEMAND_MASK);

	/* Request to execute init procedure */
	status |= icm566xx_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en1.init_en = INV_IMU_ENABLE;
	edmp_apex_en1.edmp_enable = INV_IMU_ENABLE;
	status |= icm566xx_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	status |= icm566xx_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.edmp_on_demand_en = INV_IMU_ENABLE;
	status |= icm566xx_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	/* Wait 200 us to give enough time for EMDP to start running */
	icm566xx_sleep_us(s, 200);

	/* Wait for DMP execution to complete */
	status |= icm566xx_edmp_wait_for_idle(s);

	/* Reset states */
	status |= icm566xx_edmp_mask_int_src(s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ON_DEMAND_MASK);

	/* Restore original DMP state, with DMP necessarily disabled as it was checked at the
	 * beginning of this function */
	status |= icm566xx_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= icm566xx_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);

	status |= icm566xx_edmp_unmask_int_src(s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);

	return status;
}

int icm566xx_edmp_init_apex(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	/* Configure DMP address registers */
	status |= icm566xx_edmp_configure(s);

	status |= icm566xx_edmp_clear_apex_sram(s);

	status |= icm566xx_edmp_recompute_apex_decimation(s);
	return status;
}

int icm566xx_edmp_enable(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= icm566xx_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.edmp_enable = INV_IMU_ENABLE;
	status |= icm566xx_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int icm566xx_edmp_disable(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= icm566xx_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.edmp_enable = INV_IMU_DISABLE;
	status |= icm566xx_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int icm566xx_edmp_mask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
			       uint8_t int_mask)
{
	int status = INV_IMU_OK;
	uint32_t reg_addr;
	uint8_t reg;

	/*
	 * Interrupt mask register for EDMP interrupts are located in 3 consecutive
	 * registers starting with STATUS_MASK_PIN_0_7 for interrupt0.
	 */
	reg_addr = STATUS_MASK_PIN_0_7 + edmp_int_nb;

	/* Set bits passed in param to mask corresponding interrupts */
	status |= icm566xx_read_reg(s, reg_addr, 1, &reg);
	reg |= int_mask;
	status |= icm566xx_write_reg(s, reg_addr, 1, &reg);

	return status;
}

int icm566xx_edmp_unmask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
				 uint8_t int_mask)
{
	int status = INV_IMU_OK;
	uint32_t reg_addr;
	uint8_t reg;

	/*
	 * Interrupt mask register for EDMP interrupts are located in 3 consecutive
	 * registers starting with STATUS_MASK_PIN_0_7 for interrupt0.
	 */
	reg_addr = STATUS_MASK_PIN_0_7 + edmp_int_nb;

	/* Clear bits passed in param to unmask corresponding interrupts */
	status |= icm566xx_read_reg(s, reg_addr, 1, &reg);
	reg &= ~int_mask;
	status |= icm566xx_write_reg(s, reg_addr, 1, &reg);

	return status;
}

int icm566xx_edmp_configure(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	uint16_t start_addr[] = {EDMP_ROM_START_ADDR_IRQ0, EDMP_ROM_START_ADDR_IRQ1,
				 EDMP_ROM_START_ADDR_IRQ2};
	/* Only 8 MSB of SP address is written to register */
	uint8_t stack_addr = (uint8_t)(EDMP_ROM_ONLY_SP_START_ADDR >> 8);

	/* Set Start address for 3 edmp IRQ handlers */
	status |= icm566xx_write_reg(s, EDMP_PRGRM_IRQ0_0, sizeof(start_addr),
				     (uint8_t *)&start_addr[0]);

	/* Set Stack pointer start address */
	status |= icm566xx_write_reg(s, EDMP_SP_START_ADDR, sizeof(stack_addr), &stack_addr);

	return status;
}

int icm566xx_edmp_run_ondemand(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb)
{
	int status = INV_IMU_OK;
	reg_host_msg_t reg_host_msg;

	status |= icm566xx_edmp_unmask_int_src(s, edmp_int_nb, EDMP_INT_SRC_ON_DEMAND_MASK);

	status |= icm566xx_edmp_enable(s);

	status |= icm566xx_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.edmp_on_demand_en = INV_IMU_ENABLE;
	status |= icm566xx_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	return status;
}

int icm566xx_edmp_wait_for_idle(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
	ipreg_misc_t ipreg_misc;
	int timeout_us = 1000000; /* 1 sec */

	/* Wait for idle == 1 (indicates EDMP is not running, e.g execution is completed) */
	while (status == INV_IMU_OK) {
		status |= icm566xx_read_reg(s, IPREG_MISC, 1, (uint8_t *)&ipreg_misc);
		if (ipreg_misc.edmp_idle != 0) {
			break;
		}

		icm566xx_sleep_us(s, 5);
		timeout_us -= 5;

		if (timeout_us <= 0) {
			return INV_IMU_ERROR_TIMEOUT;
		}
	}

	return status;
}
