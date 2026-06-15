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

/** @defgroup auth Secure Authentication
 *  @brief Functions required to execute the authentication routine
 *  @{
 */

/** @file inv_imu_auth.h */

#ifndef _INV_IMU_AUTH_H_
#define _INV_IMU_AUTH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#if defined(__ICCARM__)
#define STATIC_INLINE _Pragma("inline=forced") static inline
#else
#define STATIC_INLINE static inline __attribute__((always_inline))
#endif

/** @brief Serial interface structure to be used for authentication
 */
typedef struct {
	int (*read_reg_auth)(uint8_t reg, uint8_t *buf, uint32_t len);
	int (*write_reg_auth)(uint8_t reg, const uint8_t *buf, uint32_t len);
	void (*sleep_us)(uint32_t us);
} inv_serif_auth_t;

/** @brief Execute authentication routine.
 *  @param[in] s  Serial interface structure required for authentication routine.
 *  @return       0 on success, negative value on error.
 */
STATIC_INLINE int inv_imu_authenticate_device(inv_serif_auth_t *s);

/*
 * PRIVATE CODE
 */

/* Secure Authentication parameters */
#define IV_SIZE_BYTES         4
#define HMAC_KEY_SIZE_BYTES   12
#define HMAC_KEY_OTP_BYTE_POS 9

/* Addresses of direct access registers */
#define ACCEL_DATA_X_0_DREG_BANK1        0x00
#define GYRO_DATA_X_0_DREG_BANK1         0x06
#define PWR_MGMT0_DREG_BANK1             0x14
#define ACCEL_CONFIG0_DREG_BANK1         0x1F
#define GYRO_CONFIG0_DREG_BANK1          0x20
#define FIFO_CONFIG0_DREG_BANK1          0x21
#define EDMP_APEX_EN1_DREG_BANK1         0x2E
#define INT_APEX_CONFIG1_DREG_BANK1      0x3E
#define INT_APEX_STATUS1_DREG_BANK1      0x41
#define REG_HOST_MSG_DREG_BANK1          0x73
#define IREG_ADDR_15_8_DREG_BANK1        0x7C
#define IREG_DATA_DREG_BANK1             0x7E
#define REG_MISC2_DREG_BANK1             0x7F
/* Addresses of indirect access registers */
#define STATUS_MASK_PIN_0_7_IPREG_TOP1   0xA26A
#define STATUS_MASK_PIN_8_15_IPREG_TOP1  0xA26B
#define STATUS_MASK_PIN_16_23_IPREG_TOP1 0xA26C
#define IPREG_MISC_IPREG_TOP1            0xA290
#define FIFO_SRAM_SLEEP_IPREG_TOP1       0xA2A7

/* Secure Authentication interface with EDMP (position in RAM) */
#define EDMP_SA_CIPHER_IN  0x21c
#define EDMP_SA_CIPHER_OUT 0x220

/*
 * Contains registers value that will be * modified during the procedure
 */
typedef struct {
	/* Direct access registers */
	uint8_t fifo_config0;
	uint8_t accel_config0;
	uint8_t gyro_config0;
	uint8_t int_apex_config1;
	uint8_t reg_host_msg;
	uint8_t edmp_apex_en1;
	uint8_t pwr_mgmt0;

	/* Indirect access registers */
	uint8_t fifo_sram_sleep;
	uint8_t status_mask_pin_0_7;
	uint8_t status_mask_pin_8_15;
	uint8_t status_mask_pin_16_23;
} device_settings_t;

/*
 * Static fonction definition
 */

/* Generate 4 Bytes long IV with random data (noise in sensor data) */
STATIC_INLINE int generate_4bytes_iv(inv_serif_auth_t *s, uint8_t iv[IV_SIZE_BYTES]);

/* Runs HMAC-CRC32 algorithm on EDMP */
STATIC_INLINE int run_secure_auth_on_edmp(inv_serif_auth_t *s, uint8_t iv[IV_SIZE_BYTES],
					  uint8_t cipher_out[IV_SIZE_BYTES]);

/* Runs HMAC-CRC32 algorithm on host */
STATIC_INLINE int run_hmac_crc32(uint8_t key[HMAC_KEY_SIZE_BYTES], uint8_t iv[IV_SIZE_BYTES],
				 uint8_t cipher_out[IV_SIZE_BYTES]);

/* CRC32B implementation without lookup table */
STATIC_INLINE int crc32b(uint8_t *message, uint8_t input_len, uint8_t data[IV_SIZE_BYTES]);

/* Reads into MCLK registers (indirect registers) */
STATIC_INLINE int read_mreg(inv_serif_auth_t *s, uint32_t reg, uint32_t len, uint8_t *buf);

/* Writes into MCLK registers (indirect registers) */
STATIC_INLINE int write_mreg(inv_serif_auth_t *s, uint32_t reg, uint32_t len, const uint8_t *buf);

/* Waits for indirect register access completion */
STATIC_INLINE int wait_for_ireg_access_completion(inv_serif_auth_t *s);

/* Runs EDMP with on-demande mode */
STATIC_INLINE int run_edmp_ondemand(inv_serif_auth_t *s);

/* Saves registers value to re-apply after executing the routine */
STATIC_INLINE int save_settings(inv_serif_auth_t *s, device_settings_t *settings);

/* Restores registers value */
STATIC_INLINE int recover_settings(inv_serif_auth_t *s, device_settings_t *settings);

/*
 * Runs secure authentication procedure
 */
STATIC_INLINE int inv_imu_authenticate_device(inv_serif_auth_t *s)
{
	int status = 0;
	device_settings_t settings;
	uint8_t iv[IV_SIZE_BYTES] = {0};
	uint8_t cipher_imu[IV_SIZE_BYTES] = {0};
	uint8_t cipher_host[IV_SIZE_BYTES] = {0};
	uint8_t key[HMAC_KEY_SIZE_BYTES] = {0x64, 0x87, 0xED, 0x51, 0x10,
					    0xB4, 0x61, 0x19, 0x51, 0x01 /* Depends on OTP */,
					    0x00, 0x00};

	/*
	 * Specify key for each device     * For FPGA = 0
	 * For ASIC = 1
	 */
	key[HMAC_KEY_OTP_BYTE_POS] = 0x01;

	/* Save registers values for those we need to modify during the procedure */
	status |= save_settings(s, &settings);

	/* generate a random 4-byte IV */
	status |= generate_4bytes_iv(s, iv);

	/* Execute HMAC-CRC32 on EDMP */
	status |= run_secure_auth_on_edmp(s, iv, cipher_imu);

	/* Execute HMAC-CRC32 on host software */
	status |= run_hmac_crc32(key, iv, cipher_host);

	/* Compare result between IMU and host routines */
	if (memcmp(cipher_host, cipher_imu, IV_SIZE_BYTES)) {
		status = -1; /* Produce error if it doesn't match */
	}

	/* Restore registers values */
	status |= recover_settings(s, &settings);

	return status;
}

/*
 * Static functions implementation
 */
STATIC_INLINE int generate_4bytes_iv(inv_serif_auth_t *s, uint8_t iv[IV_SIZE_BYTES])
{
	int status = 0;
	uint8_t data;
	uint8_t accel_data[6];
	uint8_t gyro_data[6];

	/* Set Accel and Gyro FSR to lowest available and ODR to 100 Hz */
	data = 0x49; /* FSR = 2 g, ODR = 100 Hz */
	status |= s->write_reg_auth(ACCEL_CONFIG0_DREG_BANK1, &data, 1);
	data = 0x89; /* FSR = 15.625 dps, ODR = 100 Hz */
	status |= s->write_reg_auth(GYRO_CONFIG0_DREG_BANK1, &data, 1);

	/* Enable accel and gyro */
	data = 0x0F; /* accel_mode = LN, gyro_mode = LN */
	status |= s->write_reg_auth(PWR_MGMT0_DREG_BANK1, &data, 1);

	/* Wait for sensor to be ready (gyro startup time is 70 ms + margin) */
	s->sleep_us(100000);

	/* Read accel and gyro data */
	status |= s->read_reg_auth(ACCEL_DATA_X_0_DREG_BANK1, &accel_data[0], 6);
	status |= s->read_reg_auth(GYRO_DATA_X_0_DREG_BANK1, &gyro_data[0], 6);

	/* Fill in IV */
	iv[0] = accel_data[0]; /* ACC X LSB */
	iv[1] = gyro_data[0];  /* GYR X LSB */
	iv[2] = gyro_data[2];  /* GYR Y LSB */
	iv[3] = gyro_data[4];  /* GYR Z LSB */

	return status;
}

STATIC_INLINE int run_secure_auth_on_edmp(inv_serif_auth_t *s, uint8_t iv[IV_SIZE_BYTES],
					  uint8_t cipher_out[IV_SIZE_BYTES])
{
	int status = 0;
	uint8_t data;
	int timeout_us = 20000 /* 20 ms */;

	/* Disable FIFO */
	data = 0x00;
	status |= s->write_reg_auth(FIFO_CONFIG0_DREG_BANK1, &data, 1);

	/* Make sure SRAM is awake */
	data = 0x03;
	status |= write_mreg(s, FIFO_SRAM_SLEEP_IPREG_TOP1, 1, &data);

	/* Copy IV into EDMP SRAM */
	status |= write_mreg(s, (uint32_t)EDMP_SA_CIPHER_IN, IV_SIZE_BYTES, iv);

	/* Enable SA Done interrupt */
	data = 0x10; /* int_status_mask_pin_sa_done = 1 */
	status |= s->write_reg_auth(INT_APEX_CONFIG1_DREG_BANK1, &data, 1);

	/* Start DMP Secure Authentication process */
	status |= s->read_reg_auth(REG_HOST_MSG_DREG_BANK1, &data, 1);
	data |= 0x10 /* sa_en = 1 */;
	status |= s->write_reg_auth(REG_HOST_MSG_DREG_BANK1, &data, 1);

	/* Run EDMP */
	status |= run_edmp_ondemand(s);

	/* Wait for SA interrupt*/
	do {
		status |= s->read_reg_auth(INT_APEX_STATUS1_DREG_BANK1, &data, 1);
		data &= 0x10 /* int_status_sa_done */;

		s->sleep_us(100);
		timeout_us -= 100;
		if (timeout_us < 0) {
			return -1;
		}

	} while (!status && !data);

	/* Read cipher output from EDMP */
	status |= read_mreg(s, (uint32_t)EDMP_SA_CIPHER_OUT, IV_SIZE_BYTES, cipher_out);

	/* Stop Secure Authentication process */
	status |= s->read_reg_auth(REG_HOST_MSG_DREG_BANK1, &data, 1);
	data &= ~0x10 /* sa_en = 0 */;
	status |= s->write_reg_auth(REG_HOST_MSG_DREG_BANK1, &data, 1);

	return status;
}

STATIC_INLINE int run_hmac_crc32(uint8_t key[HMAC_KEY_SIZE_BYTES], uint8_t iv[IV_SIZE_BYTES],
				 uint8_t cipher_out[IV_SIZE_BYTES])
{
	int status = 0;
	int i;
	uint32_t ipad = 0x36363636;
	uint32_t K0_ipad_BE;
	uint32_t opad = 0x5C5C5C5C;
	uint32_t K0_opad_BE;
	uint32_t crc32_BE;
	uint8_t to_hash[HMAC_KEY_SIZE_BYTES];
	uint8_t iv_swapped[IV_SIZE_BYTES];
	uint8_t cipher_swapped[IV_SIZE_BYTES];

	/* The byte order of the IV is swapped first */
	iv_swapped[3] = iv[0];
	iv_swapped[2] = iv[1];
	iv_swapped[1] = iv[2];
	iv_swapped[0] = iv[3];

	/* Key is longer CRC32 block size (4B) so let's hash the key using CRC32 */
	status |= crc32b(key, HMAC_KEY_SIZE_BYTES, cipher_swapped);

	/* process K0 ^ ipad and K0 ^opad */
	crc32_BE = (((uint32_t)cipher_swapped[3]) << 24) + (((uint32_t)cipher_swapped[2]) << 16) +
		   (((uint32_t)cipher_swapped[1]) << 8) + cipher_swapped[0];
	K0_ipad_BE = crc32_BE ^ ipad;
	K0_opad_BE = crc32_BE ^ opad;

	/* process CRC32(K0_ipad | input_msg) */
	memcpy(to_hash, &K0_ipad_BE, IV_SIZE_BYTES);
	memcpy(to_hash + IV_SIZE_BYTES, iv_swapped, IV_SIZE_BYTES);
	status |= crc32b(to_hash, IV_SIZE_BYTES + IV_SIZE_BYTES, cipher_swapped);

	/* process HMAC = CRC32(K0_opad | CRC32(K0_ipad | input_msg)) */
	memcpy(to_hash, &K0_opad_BE, IV_SIZE_BYTES);
	memcpy(to_hash + IV_SIZE_BYTES, cipher_swapped, IV_SIZE_BYTES);
	status |= crc32b(to_hash, IV_SIZE_BYTES + IV_SIZE_BYTES, cipher_swapped);

	/* Swap cipher to populate `cipher_out` */
	for (i = 0; i < IV_SIZE_BYTES; i++) {
		cipher_out[i] = cipher_swapped[IV_SIZE_BYTES - i - 1];
	}

	return status;
}

STATIC_INLINE int crc32b(uint8_t *message, uint8_t input_len, uint8_t data[IV_SIZE_BYTES])
{
	/*
	 * This is a basic CRC-32 calculation with some optimization but no
	 * table lookup. The byte reversal is avoided by shifting the crc reg
	 * right instead of left and by using a reversed 32-bit word to represent
	 * the polynomial.
	 */
	int status = 0;
	int i, j;
	uint32_t byte, crc, mask;
	uint32_t output;

	crc = 0xFFFFFFFF;
	for (i = 0; i < input_len; i++) {
		byte = message[i];
		crc = crc ^ byte;
		for (j = 0; j < 8; j++) {
			mask = -(crc & 1);
			crc = (crc >> 1) ^ (0xEDB88320 & mask);
		}
	}
	output = ~crc;

	data[0] = (uint8_t)((output >> 24) & 0xff);
	data[1] = (uint8_t)((output >> 16) & 0xff);
	data[2] = (uint8_t)((output >> 8) & 0xff);
	data[3] = (uint8_t)((output) & 0xff);

	return status;
}

STATIC_INLINE int read_mreg(inv_serif_auth_t *s, uint32_t reg, uint32_t len, uint8_t *buf)
{
	int status = 0;
	uint8_t data[2];

	/* Write address first */
	data[0] = (reg & 0xFF00) >> 8;
	data[1] = reg & 0xFF;
	status |= wait_for_ireg_access_completion(s);
	status |= s->write_reg_auth(IREG_ADDR_15_8_DREG_BANK1, data, 2);

	/* Read all bytes one by one */
	for (uint32_t i = 0; i < len; i++) {
		status |= wait_for_ireg_access_completion(s);
		status |= s->read_reg_auth(IREG_DATA_DREG_BANK1, &buf[i], 1);
	}

	return status;
}

STATIC_INLINE int write_mreg(inv_serif_auth_t *s, uint32_t reg, uint32_t len, const uint8_t *buf)
{
	int status = 0;
	uint8_t data[3];

	/* First two bytes are the address where we want to write */
	data[0] = (reg & 0xFF00) >> 8;
	data[1] = reg & 0xFF;
	/* 3rd byte is the first data to write*/
	data[2] = buf[0];

	/* Burst write address and first byte */
	status |= wait_for_ireg_access_completion(s);
	status |= s->write_reg_auth(IREG_ADDR_15_8_DREG_BANK1, data, 3);
	status |= wait_for_ireg_access_completion(s);

	/* Loop on the remaining bytes */
	for (uint32_t i = 1; i < len; i++) {
		status |= s->write_reg_auth(IREG_DATA_DREG_BANK1, &buf[i], 1);
		status |= wait_for_ireg_access_completion(s);
	}

	return status;
}

STATIC_INLINE int wait_for_ireg_access_completion(inv_serif_auth_t *s)
{
	int status = 0;
	uint8_t ireg_done;

	do {
		status |= s->read_reg_auth(REG_MISC2_DREG_BANK1, &ireg_done, 1);
	} while (!status && !(ireg_done & 0x01 /* REG_MISC2_IREG_DONE_MASK */));

	return status;
}

STATIC_INLINE int run_edmp_ondemand(inv_serif_auth_t *s)
{
	int status = 0;
	uint8_t data;
	uint32_t reg_addr;

	/*
	 * Interrupt mask register for edmp interrupts are located in 3 consecutive     * registers
	 * starting with register STATUS_MASK_PIN_0_7_IPREG_TOP1 for interrupt0.
	 */
	reg_addr = STATUS_MASK_PIN_0_7_IPREG_TOP1 + 2 /* INV_IMU_EDMP_INT2 */;

	/* Clear bits passed in param to unmask corresponding interrupts */
	status |= read_mreg(s, reg_addr, 1, &data);
	data &= ~(0x20 /* EDMP_INT_SRC_ON_DEMAND_MASK */);
	status |= write_mreg(s, reg_addr, 1, &data);

	status |= s->read_reg_auth(EDMP_APEX_EN1_DREG_BANK1, &data, 1);
	data |= 0x40 /* edmp_enable = 1 */;
	status |= s->write_reg_auth(EDMP_APEX_EN1_DREG_BANK1, &data, 1);

	status |= s->read_reg_auth(REG_HOST_MSG_DREG_BANK1, &data, 1);
	data |= 0x20 /* edmp_on_demand_en = 1 */;
	status |= s->write_reg_auth(REG_HOST_MSG_DREG_BANK1, &data, 1);

	return status;
}

STATIC_INLINE int save_settings(inv_serif_auth_t *s, device_settings_t *settings)
{
	int status = 0;

	/* Direct access registers */
	status |= s->read_reg_auth(FIFO_CONFIG0_DREG_BANK1, &settings->fifo_config0, 1);
	status |= s->read_reg_auth(ACCEL_CONFIG0_DREG_BANK1, &settings->accel_config0, 1);
	status |= s->read_reg_auth(GYRO_CONFIG0_DREG_BANK1, &settings->gyro_config0, 1);
	status |= s->read_reg_auth(PWR_MGMT0_DREG_BANK1, &settings->pwr_mgmt0, 1);
	status |= s->read_reg_auth(INT_APEX_CONFIG1_DREG_BANK1, &settings->int_apex_config1, 1);
	status |= s->read_reg_auth(REG_HOST_MSG_DREG_BANK1, &settings->reg_host_msg, 1);
	status |= s->read_reg_auth(EDMP_APEX_EN1_DREG_BANK1, &settings->edmp_apex_en1, 1);

	/* Indirect access registers */
	status |= read_mreg(s, FIFO_SRAM_SLEEP_IPREG_TOP1, 1, &settings->fifo_sram_sleep);
	status |= read_mreg(s, STATUS_MASK_PIN_0_7_IPREG_TOP1, 1, &settings->status_mask_pin_0_7);
	status |= read_mreg(s, STATUS_MASK_PIN_8_15_IPREG_TOP1, 1, &settings->status_mask_pin_8_15);
	status |=
		read_mreg(s, STATUS_MASK_PIN_16_23_IPREG_TOP1, 1, &settings->status_mask_pin_16_23);

	return status;
}

STATIC_INLINE int recover_settings(inv_serif_auth_t *s, device_settings_t *settings)
{
	int status = 0;

	/* Direct access registers */
	status |= s->write_reg_auth(FIFO_CONFIG0_DREG_BANK1, &settings->fifo_config0, 1);
	status |= s->write_reg_auth(ACCEL_CONFIG0_DREG_BANK1, &settings->accel_config0, 1);
	status |= s->write_reg_auth(GYRO_CONFIG0_DREG_BANK1, &settings->gyro_config0, 1);
	status |= s->write_reg_auth(PWR_MGMT0_DREG_BANK1, &settings->pwr_mgmt0, 1);
	status |= s->write_reg_auth(INT_APEX_CONFIG1_DREG_BANK1, &settings->int_apex_config1, 1);
	status |= s->write_reg_auth(REG_HOST_MSG_DREG_BANK1, &settings->reg_host_msg, 1);
	status |= s->write_reg_auth(EDMP_APEX_EN1_DREG_BANK1, &settings->edmp_apex_en1, 1);

	/* Indirect access registers */
	status |= write_mreg(s, FIFO_SRAM_SLEEP_IPREG_TOP1, 1, &settings->fifo_sram_sleep);
	status |= write_mreg(s, STATUS_MASK_PIN_0_7_IPREG_TOP1, 1, &settings->status_mask_pin_0_7);
	status |=
		write_mreg(s, STATUS_MASK_PIN_8_15_IPREG_TOP1, 1, &settings->status_mask_pin_8_15);
	status |= write_mreg(s, STATUS_MASK_PIN_16_23_IPREG_TOP1, 1,
			     &settings->status_mask_pin_16_23);

	return status;
}
#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_AUTH_H_ */

/** @} */
