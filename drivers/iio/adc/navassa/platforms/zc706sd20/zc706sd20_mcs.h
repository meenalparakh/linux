/**
 * \file
 * Analog Devices ZC706 Platform + microzed hardware abstraction layer
 */

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef __ZC706SD20_MCS_H__
#define __ZC706SD20_MCS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
int32_t zc706sd20_Mcs_Pulse(void* devHalCfg, uint8_t numberOfPulses);
#endif /* CLIENT_IGNORE */

#ifdef __cplusplus
}
#endif
#endif /* __ZC706SD20_MCS_H__ */