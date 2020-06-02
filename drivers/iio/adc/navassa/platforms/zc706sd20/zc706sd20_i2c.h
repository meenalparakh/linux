/**
 * \file
 * \brief Support for linux layer I2C functions
 */

/**
* Copyright 2018 - 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef __ZC706SD20_I2C_H__
#define __ZC706SD20_I2C_H__

#include <stdint.h>
#include "adi_platform_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
int32_t zc706sd20_I2cOpen(void *devHalCfg);
int32_t zc706sd20_I2cClose(void *devHalCfg);
int32_t zc706sd20_I2cInit(void *devHalCfg);
int32_t zc706sd20_I2cWrite(void *devHalCfg, uint8_t slaveAddress, const uint8_t txData[], uint32_t numTxBytes);
int32_t zc706sd20_I2cRead(void *devHalCfg, uint8_t slaveAddress, const uint8_t txData[], uint32_t numTxBytes, uint8_t rxData[], uint32_t numRxBytes);
#endif /* CLIENT_IGNORE */

#ifdef __cplusplus
}
#endif
#endif /* __ZC706SD20_I2C_H__ */
