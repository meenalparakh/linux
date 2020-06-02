/**
 * \file
 * Analog Devices ZC706 Platform + microzed hardware abstraction layer
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Support for linux layer SPI functions
 */

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/
#ifndef __ZC706SD20_SPI_H__
#define __ZC706SD20_SPI_H__

#include <stdint.h>
#include "adi_platform_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
int32_t zc706sd20_SpiOpen(void *devHalCfg);
int32_t zc706sd20_SpiClose(void *devHalCfg);
int32_t zc706sd20_SpiInit(void *devHalCfg);
int32_t zc706sd20_SpiWrite(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes);
int32_t zc706sd20_SpiRead(void *devHalCfg, const uint8_t txData[], uint8_t rxData[], uint32_t numTxRxBytes);
#endif /* CLIENT_IGNORE */

#ifdef __cplusplus
}
#endif
#endif /* __ZC706SD20_SPI_H__ */
