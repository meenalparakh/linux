/*
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_DECODE_BF_ENUM_H_
#define _ADRV9001_DECODE_BF_ENUM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_common_types.h"
    
#include "adrv9001_bf_nvs_regmap_rx.h"
#include "adrv9001_bf_nvs_regmap_rxb.h"
#include "adrv9001_bf_analog_rx_mem_map.h"
#include "adrv9001_bf_nvs_pll_mem_map.h"
    
adrv9001_BfNvsRegmapRxChanAddr_e adrv9001_RxChanAddr_Get(adi_common_ChannelNumber_e channel);
adrv9001_BfNvsRegmapRxbChanAddr_e adrv9001_RxbChanAddr_Get(adi_common_ChannelNumber_e channel);
adrv9001_BfAnalogRxMemMapChanAddr_e adrv9001_AnalogRxChanAddr_Get(adi_common_ChannelNumber_e channel);

adrv9001_BfNvsPllMemMapChanAddr_e adrv9001_PllMemMapAddr_Get(adi_adrv9001_ClkPllMode_e mode, adi_adrv9001_PllType_e pllType);
    
#ifdef __cplusplus
}
#endif

#endif