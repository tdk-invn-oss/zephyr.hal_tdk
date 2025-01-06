/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

/** @defgroup DriverIcp201xxExt Icp201xx driver extended functions
 *  @brief    Extended functions for Icp201xx devices
 *  @ingroup  DriverIcp201xx
 *  @{
 */

#ifndef _INV_ICP201XX_EXT_FUNC_H_
#define _INV_ICP201XX_EXT_FUNC_H_

#include <stdint.h>
#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Hook for low-level high res system sleep() function to be implemented by upper layer
 *  ~100us resolution is sufficient
 *  @param[in] us number of us the calling thread should sleep
 */
 
extern void inv_icp201xx_sleep_us(int us);


#ifdef __cplusplus
}
#endif

#endif /* _INV_ICP201XX_EXT_FUNC_H_ */

/** @} */
