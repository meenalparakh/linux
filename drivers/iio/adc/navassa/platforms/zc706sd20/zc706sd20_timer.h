/**
 * \file
 * Analog Devices ZC706 Platform + microzed hardware abstraction layer
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Support for linux layer timer functions
 */

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/
#ifndef __ZC706SD20_TIMER_H__
#define __ZC706SD20_TIMER_H__

#include <stdint.h>
#include "adi_platform_types.h"


#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
int32_t zc706sd20_TimerOpen(void *devHalCfg);
int32_t zc706sd20_TimerClose(void *devHalCfg);
int32_t zc706sd20_TimerInit(void *devHalCfg);
int32_t zc706sd20_TimerWait_ms(void *devHalCfg, uint32_t time_ms);
int32_t zc706sd20_TimerWait_us(void *devHalCfg, uint32_t time_us);
#endif /* CLIENT_IGNORE */

#ifdef __cplusplus
}
#endif
#endif /* __ZC706SD20_TIMER_H__ */
