/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup DriverIct1531xSerif Ict1531x driver serif
 *  @brief Interface for low-level serial (I2C/SPI) access
 *  @ingroup  DriverIct1531x
 *  @{
 */

#ifndef _INV_ICT1531X_SERIF_H_
#define _INV_ICT1531X_SERIF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Invn/InvError.h"

#include <stdint.h>

/** @brief Ict1531x serial interface
 */
struct inv_ict1531x_serif {
	void *context;
	int (*read_reg)(void *serif, uint8_t reg, uint8_t *buf, uint32_t len);
	int (*write_reg)(void *serif, uint8_t reg, const uint8_t *buf, uint32_t len);
	uint32_t max_read;
	uint32_t max_write;
	uint8_t  is_spi;
};

static inline uint8_t inv_ict1531x_serif_is_spi(struct inv_ict1531x_serif *s)
{
	if (!s)
		return INV_ERROR;

	return s->is_spi;
}

static inline uint32_t inv_ict1531x_serif_max_read(struct inv_ict1531x_serif *s)
{
	if (!s)
		return INV_ERROR;

	return s->max_read;
}

static inline uint32_t inv_ict1531x_serif_max_write(struct inv_ict1531x_serif *s)
{
	if (!s)
		return INV_ERROR;

	return s->max_write;
}

static inline int inv_ict1531x_serif_read_reg(struct inv_ict1531x_serif *s, uint8_t reg,
                                              uint8_t *buf, uint32_t len)
{
	if (!s)
		return INV_ERROR;

	if (len > s->max_read)
		return INV_ERROR_SIZE;

	if (s->read_reg(s->context, reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

static inline int inv_ict1531x_serif_write_reg(struct inv_ict1531x_serif *s, uint8_t reg,
                                               const uint8_t *buf, uint32_t len)
{
	if (!s)
		return INV_ERROR;

	if (len > s->max_write)
		return INV_ERROR_SIZE;

	if (s->write_reg(s->context, reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICT1531X_SERIF_H_ */

/** @} */
