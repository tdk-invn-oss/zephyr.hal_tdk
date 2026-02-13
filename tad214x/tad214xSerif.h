/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup Drivertad214xSerif tad214x driver serif
 *  @brief Interface for low-level serial (I2C/SPI) access
 *  @ingroup  Drivertad214x
 *  @{
 */

#ifndef _TAD214X_SERIF_H_
#define _TAD214X_SERIF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Invn/InvError.h"

#include <stdint.h>
/** @brief Serif type definition.
 */
typedef enum {
	IF_I2C = 0, /**< identifies I2C interface. */
	IF_SPI = 1, /**< identifies 3/4-wire SPI interface. */
	IF_ENC = 2 /**< identifies Encoder interface. */
} tad214x_serif_type_t;

/** @brief tad214x serial interface
 */
struct tad214x_serif {
	void *context;
	tad214x_serif_type_t if_tad;
	int (*read_reg)(void *serif, uint8_t reg, uint16_t *buf, uint32_t len);
	int (*write_reg)(void *serif, uint8_t reg, const uint16_t *buf, uint32_t len);
	void (*sleep_us)(uint32_t us);
	int (*csn_pulse)(void *serif);
	uint32_t max_read;
	uint32_t max_write;
	uint8_t  is_first_transaction; /* indicates if the upcoming transaction will be the first one */
};

static inline uint32_t tad214x_serif_max_read(struct tad214x_serif *s)
{
	if (!s)
		return INV_ERROR;

	return s->max_read;
}

static inline uint32_t tad214x_serif_max_write(struct tad214x_serif *s)
{
	if (!s)
		return INV_ERROR;

	return s->max_write;
}

static inline int tad214x_serif_read_reg(struct tad214x_serif *s, uint8_t reg,
                                              uint16_t *buf, uint32_t len)
{
	int rc;

	if (!s)
		return INV_ERROR;

	if (len > s->max_read)
		return INV_ERROR_SIZE;

	rc = s->read_reg(s->context, reg, buf, len);

	/* 
	 * After device power-up, the user may receive NACK for the very first I2C transaction.
	 * The user should perform one retry on the very first I2C transaction if it receives a NACK.
	 */
	if (s->is_first_transaction) {
		s->is_first_transaction = 0;
		if (rc != 0)
			rc = s->read_reg(s->context, reg, buf, len);
	}

	if (rc != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

static inline int tad214x_serif_write_reg(struct tad214x_serif *s, uint8_t reg,
                                               const uint16_t *buf, uint32_t len)
{
	int rc;

	if (!s)
		return INV_ERROR;

	if (len > s->max_write)
		return INV_ERROR_SIZE;

	rc = s->write_reg(s->context, reg, buf, len);

	/* 
	 * After device power-up, the user may receive NACK for the very first I2C transaction.
	 * The user should perform one retry on the very first I2C transaction if it receives a NACK.
	 */
	if (s->is_first_transaction) {
		s->is_first_transaction = 0;
		if (rc != 0)
			rc = s->write_reg(s->context, reg, buf, len);
	}

	if (rc != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

static inline int tad214x_serif_cs_pulse(struct tad214x_serif *s)
{
	int rc = s->csn_pulse(s->context);
	if (rc != 0)
		return INV_ERROR_TRANSPORT;
	return 0;
}
#ifdef __cplusplus
}
#endif

#endif /* _TAD214X_SERIF_H_ */

/** @} */
