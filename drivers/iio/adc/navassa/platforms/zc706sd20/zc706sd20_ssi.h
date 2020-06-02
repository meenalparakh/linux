/**
 * \file
 * Analog Devices ZC706 Platform + microzed hardware abstraction layer
 */

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef __ZC706SD20_SSI_H__
#define __ZC706SD20_SSI_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
int32_t zc706sd20_ssi_Reset(void* devHalCfg);
#endif /* CLIENT_IGNORE */

#ifdef __cplusplus
}
#endif
#endif /* __ZC706SD20_SSI_H__ */