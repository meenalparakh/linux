/**
 * \file adrv9001_bf_analog_rx_mem_map.h Automatically generated file with generator ver 0.0.1.0.
 *
 * \brief Contains BitField functions to support ADRV9001 transciever device.
 *
 * ADRV9001 BITFIELD VERSION: 0.0.0.1
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef __ADRV9001_BF_ANALOG_RX_MEM_MAP_H__
#define __ADRV9001_BF_ANALOG_RX_MEM_MAP_H__

#include "adrv9001_bf_analog_rx_mem_map_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_AnalogRxMemMapAuxLoHighRBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapAuxLoHighRBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapAuxLoLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapAuxLoLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapAuxLoLdoFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapAuxLoLdoFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapAuxLoLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapAuxLoLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapAuxLoLdoStatusBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapAuxLoLdoVoutSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapAuxLoLdoVoutSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapAuxLoLdoVoutTrimBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapAuxLoLdoVoutTrimBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapCalToneDriverAttnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                       uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapCalToneDriverAttnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapCalToneDriverInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapCalToneDriverInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapCalToneDriverPdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapCalToneDriverPdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapEnRxattenInputCmBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapEnRxattenInputCmBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapEnableAuxLoSourceBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapEnableAuxLoSourceBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapEnableLoSourceLb1BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapEnableLoSourceLb1BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapEnableLoSourceLb2BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapEnableLoSourceLb2BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapEnableTxLoSourceBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapEnableTxLoSourceBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapIdistBypassBiasResBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapIdistBypassBiasResBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1AttnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1AttnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1AttnBBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1AttnBBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayCoarseCorrIBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayCoarseCorrIBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayCoarseCorrIbBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayCoarseCorrIbBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayCoarseCorrQBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayCoarseCorrQBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayCoarseCorrQbBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayCoarseCorrQbBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayCorrPdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayCorrPdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayDacBiasResBypassBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayDacBiasResBypassBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayDacResBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayDacResBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayFineCorrDacBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayFineCorrDacBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayFineCorrDacBBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayFineCorrDacBBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1LoDelayLatchEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1LoDelayLatchEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1MixerConfigBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1MixerConfigBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1feLocmBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1feLocmBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1fePdBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1fePdBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1fePdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1fePdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1fePdOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1fePdOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1fecapBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1fecapBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLb1fecapBBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLb1fecapBBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLoopbackFilter1StageForceEnBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLoopbackFilter1StageForceEnBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLoopbackFilterDiffCh1BfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLoopbackFilterDiffCh1BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapLoopbackSwCh1BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapLoopbackSwCh1BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1Force1p0vAttnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1Force1p0vAttnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1ForceEnableOverrideLoBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1ForceEnableOverrideLoBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1ForceMaxAttnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1ForceMaxAttnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1HiLoResEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1HiLoResEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1HiLoResEnBBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1HiLoResEnBBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCoarseCorrIBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCoarseCorrIBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCoarseCorrIbBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCoarseCorrIbBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCoarseCorrQBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCoarseCorrQBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCoarseCorrQbBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCoarseCorrQbBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCorrPdBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayCorrPdBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayDacBiasResBypassBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayDacBiasResBypassBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayDacResBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayDacResBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayFineCorrDacBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayFineCorrDacBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayFineCorrDacBBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayFineCorrDacBBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1LoDelayLatchEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1LoDelayLatchEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1MixerConfigBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1MixerConfigBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1Sel1p8vSupplyBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1Sel1p8vSupplyBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1SelAttenRshuntWordBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1SelAttenRshuntWordBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1feLocmBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1feLocmBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1feLodcPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1feLodcPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1fePdBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1fePdBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1fePdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1fePdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1fePdOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1fePdOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1feRbbDisableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1feRbbDisableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1fecapBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1fecapBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx1fecapBBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx1fecapBBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2Force1p0vAttnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2Force1p0vAttnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2ForceEnableOverrideLoBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2ForceEnableOverrideLoBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2ForceMaxAttnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2ForceMaxAttnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2HiLoResEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2HiLoResEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2HiLoResEnBBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2HiLoResEnBBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCoarseCorrIBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCoarseCorrIBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCoarseCorrIbBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCoarseCorrIbBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCoarseCorrQBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCoarseCorrQBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCoarseCorrQbBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCoarseCorrQbBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCorrPdBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayCorrPdBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayDacBiasResBypassBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayDacBiasResBypassBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayDacResBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayDacResBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayFineCorrDacBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayFineCorrDacBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayFineCorrDacBBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayFineCorrDacBBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2LoDelayLatchEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2LoDelayLatchEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2MixerConfigBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2MixerConfigBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2Sel1p8vSupplyBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2Sel1p8vSupplyBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2SelAttenRshuntWordBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2SelAttenRshuntWordBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2feLocmBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2feLocmBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2feLodcPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2feLodcPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2fePdBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2fePdBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2fePdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2fePdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2fePdOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2fePdOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2feRbbDisableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2feRbbDisableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2fecapBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2fecapBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrx2fecapBBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrx2fecapBBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxAttnCalSigBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxAttnCalSigBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxBlockDetDecayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxBlockDetDecayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxBlockDetLlbthBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxBlockDetLlbthBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxBlockDetUlbthBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxBlockDetUlbthBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxBlockdetPdBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxBlockdetPdBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxBlockdetPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxBlockdetPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxBlockdetPdOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxBlockdetPdOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxCalSigPdBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxCalSigPdBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxOsdacPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxOsdacPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxOsdacPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxOsdacPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxOsdacPdOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxOsdacPdOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxTiaForceUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxTiaForceUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxTiaModeDirectBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxTiaModeDirectBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxTiaModeDirectDiv16BfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxTiaModeDirectDiv16BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxTiaModeLfsrBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxTiaModeLfsrBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxTiaModeLfsrTapSeedbBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxTiaModeLfsrTapSeedbBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxTiaModeSeqBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxTiaModeSeqBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxmixerILogenOvlpCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxmixerILogenOvlpCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxmixerQLogenOvlpCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxmixerQLogenOvlpCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaBypBiasRBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaBypBiasRBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaIbiasCntrlStg1BfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaIbiasCntrlStg1BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaIbiasCntrlStg2BfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaIbiasCntrlStg2BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaPdTiaDelaySpiOvrdBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaPdTiaDelaySpiOvrdBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaPowCntrlStg1BfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                        uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaPowCntrlStg1BfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                        uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaPowCntrlStg2BfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                        uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaPowCntrlStg2BfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                        uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaSpiPdVcmgenOvrdBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaSpiPdVcmgenOvrdBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaTunerResetbBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaTunerResetbBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapOrxtiaVcmgenOutlevelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapOrxtiaVcmgenOutlevelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdBbAmuxOrx1Tx1lbBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdBbAmuxOrx1Tx1lbBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdBbAmuxOrx2Tx2lbBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdBbAmuxOrx2Tx2lbBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdIDistBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdIDistBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdLb1LodelayBufferBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdLb1LodelayBufferBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdLb1LodelayBufferOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdLb1LodelayBufferOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdLb1LodelayBufferOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdLb1LodelayBufferOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdLb1MixerCapdacBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdLb1MixerCapdacBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdLb1MixerCapdacOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdLb1MixerCapdacOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdLb1MixerCapdacOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdLb1MixerCapdacOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx1LodelayBufferBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx1LodelayBufferBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx1LodelayBufferOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx1LodelayBufferOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx1LodelayBufferOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx1LodelayBufferOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx1MixerCapdacBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx1MixerCapdacBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx1MixerCapdacOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx1MixerCapdacOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx1MixerCapdacOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx1MixerCapdacOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx2LodelayBufferBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx2LodelayBufferBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx2LodelayBufferOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx2LodelayBufferOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx2LodelayBufferOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx2LodelayBufferOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx2MixerCapdacBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx2MixerCapdacBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx2MixerCapdacOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx2MixerCapdacOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOrx2MixerCapdacOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOrx2MixerCapdacOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdOverrideOutRx1LoopbackBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdOverrideOutRx1LoopbackBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapPdTuneDacBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapPdTuneDacBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRfLoHighRBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRfLoHighRBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRx1LoopbackFilterBiasConfigBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRx1LoopbackFilterBiasConfigBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRx1LoopbackFilterRgateBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRx1LoopbackFilterRgateBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRx1PdLoopbackFilterOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRx1PdLoopbackFilterOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxAuxLoBufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxAuxLoBufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxAuxloRTermBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxAuxloRTermBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxCalToneAuxloInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxCalToneAuxloInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxCalToneOutbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxCalToneOutbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxCalToneRxloInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxCalToneRxloInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxCalToneSelAuxLoBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxCalToneSelAuxLoBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxEnAuxloTermBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxEnAuxloTermBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxLoHighRBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxLoHighRBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxLoLineRTermBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxLoLineRTermBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxLoLineRcvBufPdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxLoLineRcvBufPdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxLoLineTermBiasPdBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxLoLineTermBiasPdBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxLoLineTermEnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxLoLineTermEnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxMixAuxloInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxMixAuxloInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxMixLoOutbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxMixLoOutbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxMixLoSelAuxLoBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxMixLoSelAuxLoBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxMixRxloInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxMixRxloInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxPdAuxloTermBiasBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxPdAuxloTermBiasBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxSpare1BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxSpare1BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxSpare2BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxSpare2BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaBbinjAmpCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaBbinjAmpCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaBbinjPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaBbinjPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaBbinjSelchopBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaBbinjSelchopBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaBbinjStdbyBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaBbinjStdbyBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaC1BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaC1BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaC1FineBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaC1FineBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaC2BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaC2BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaC2FineBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaC2FineBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaModeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                               uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaModeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                               uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaR2BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaR2BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaR3BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaR3BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaR4BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaR4BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaSelccStg1BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaSelccStg1BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaSelccStg2BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaSelccStg2BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaSelrzStg1BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaSelrzStg1BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapRxtiaSelrzStg2BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapRxtiaSelrzStg2BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapSelRxattennVgateHiBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                        uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapSelRxattennVgateHiBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                        uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapSelbInjCalBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapSelbInjCalBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapSelectOrx1OrTx1lbBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapSelectOrx1OrTx1lbBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapSelectOrx2OrTx2lbBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapSelectOrx2OrTx2lbBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTiaPdBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTiaPdBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTiaPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTiaPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTiaPdOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTiaPdOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTiaTunerClkDriverInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTiaTunerClkDriverInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTiaTunerClkDriverPdBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTiaTunerClkDriverPdBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTuneDacGm1BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapTuneDacGm1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTuneDacGm2BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                uint16_t bfValue);
int32_t adrv9001_AnalogRxMemMapTuneDacGm2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                uint16_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTuneDacResBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTuneDacResBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxCalToneAuxloInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxCalToneAuxloInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxCalToneOutbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxCalToneOutbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxCalToneSelAuxLoBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxCalToneSelAuxLoBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxCalToneTxloInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxCalToneTxloInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxLbAuxloInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxLbAuxloInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxLbLoOutbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxLbLoOutbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxLbLoSelAuxLoBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxLbLoSelAuxLoBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxLbTxloInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxLbTxloInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapTxLoHighRBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapTxLoHighRBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_AnalogRxMemMapVhiGateCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_AnalogRxMemMapVhiGateCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogRxMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_ANALOG_RX_MEM_MAP_H__

/*  EOF: adrv9001_bf_analog_rx_mem_map.h */

