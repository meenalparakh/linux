/**
* \file
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ZC706SD20_LOGGING_H_
#define _ZC706SD20_LOGGING_H_

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdio.h>
#include <stdint.h>
#endif

#include "adi_platform_types.h"

#ifdef __cplusplus
extern "C" {
#endif


    /*
    * Only allow log file to have MAX_NUM_LINES to prevent it from growing too large
    * If max lines reached, start back at line 0.
    * */
#define ZC706SD20_LOG_MAX_NUM_LINES 250000

#ifndef CLIENT_IGNORE
int32_t zc706sd20_LogFileOpen(void *devHalCfg, const char *filename);
int32_t zc706sd20_LogFileFlush(void *devHalCfg);
int32_t zc706sd20_LogFileClose(void *devHalCfg);
int32_t zc706sd20_LogLevelSet(void *devHalCfg, int32_t logLevel);
int32_t zc706sd20_LogLevelGet(void *devHalCfg, int32_t *logLevel);
int32_t zc706sd20_LogWrite(void *devHalCfg, int32_t logLevel, const char *comment, va_list argp);
#endif /* CLIENT_IGNORE */

#ifdef __cplusplus
}
#endif

#endif /* _ZC706SD20_LOGGING_H_ */