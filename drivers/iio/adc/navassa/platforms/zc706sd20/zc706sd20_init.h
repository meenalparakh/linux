/**
* \file
* \brief Contains ADI Transceiver Hardware Abstraction functions for the ZC706 Platform.
*        Analog Devices maintains and provides updates to this code layer for the ZC706 platform.
*        The end user should not modify this file or any code in this directory. The end user
*        may provide a similar platform layer that can be used in place of this platform layer,
*        that uses the same function prototypes.
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef __ZC706SD20_INIT_H__
#define __ZC706SD20_INIT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
int32_t zc706sd20_HwOpen(void *devHalCfg);
int32_t zc706sd20_HwReset(void *devHalCfg, uint8_t pinLevel);
int32_t zc706sd20_HwClose(void *devHalCfg);

void* zc706sd20_DevHalCfgCreate(uint32_t interfaceMask, uint8_t spiChipSelect, const char *logFilename);
int32_t zc706sd20_DevHalCfgFree(void *devHalCfg);
#endif /* CLIENT_IGNORE */

#ifdef __cplusplus
}
#endif
#endif /*__ZC706SD20_INIT_H__*/
