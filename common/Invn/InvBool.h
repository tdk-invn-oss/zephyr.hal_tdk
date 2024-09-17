/*
 * Copyright (c) 2017 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @brief    Custom definition for boolean type to avoid compiler disrepenrencies
 *	@{
 */

#ifndef _INV_BOOL_H_
#define _INV_BOOL_H_

typedef int inv_bool_t;

#ifndef __cplusplus

#ifndef true
	#define true    1
#endif

#ifndef false
	#define false   0
#endif

#endif /* __cplusplus */

#endif /* _INV_BOOL_H_ */

/** @} */
