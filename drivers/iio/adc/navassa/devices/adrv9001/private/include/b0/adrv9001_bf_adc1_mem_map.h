/**
 * \file adrv9001_bf_adc1_mem_map.h Automatically generated file with generator ver 0.0.1.0.
 *
 * \brief Contains BitField functions to support ADRV9001 transciever device.
 *
 * ADRV9001 BITFIELD VERSION: 0.0.0.1
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef __ADRV9001_BF_ADC1_MEM_MAP_H__
#define __ADRV9001_BF_ADC1_MEM_MAP_H__

#include "adrv9001_bf_adc1_mem_map_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_Adc1MemMapAdcAmp1Cap1BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp1Cap1BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp1Cap2BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp1Cap2BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp1EnBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp1EnBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp1SpareBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                              uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp1SpareBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                              uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp2Cap1BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp2Cap1BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp2Cap2BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp2Cap2BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp2EnBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp2EnBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp2SpareBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                              uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp2SpareBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                              uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp3EnBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp3EnBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcAmp3SpareBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                              uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcAmp3SpareBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                              uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                    uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasDacAmpBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasDacAmpNcurrBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpNcurrBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasDacAmpPcurrBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpPcurrBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasDacBiasPplyBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasDacBiasPplyBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasDacIcalPplyBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasDacIcalPplyBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasDacIlsbBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasDacIlsbBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasFlashBiasBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                  uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasFlashBiasBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                  uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasFlashRefgenBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasFlashRefgenBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasRcalBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasRcalBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                             uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasVcmAmpBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasVcmAmpBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasVonBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasVonBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcBiasVonAmpBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcBiasVonAmpBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcC1BfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcC1BfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcC2BfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcC2BfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcC3BfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcC3BfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcCdfbBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                         uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcCdfbBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                         uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcDacsBypassRfBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcDacsBypassRfBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcFlashCalBiasRefCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcFlashCalBiasRefCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcFlashEslCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcFlashEslCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcFlashFckDelayModeBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcFlashFckDelayModeBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcFlashRBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcFlashRBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcFlashRefCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcFlashRefCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                                 uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcFlashResetModeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcFlashResetModeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcFlashSBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcFlashSBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcI3BfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcI3BfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcInputzeroBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcInputzeroBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcInputAltBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcInputAltBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcPdBiasBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcPdBiasBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcPdDacsBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcPdDacsBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcPdDacsDelayBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcPdDacsDelayBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcPdFlashBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcPdFlashBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcPdLfBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcPdLfBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcR1BfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcR1BfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcR1AltBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                          uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcR1AltBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                          uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcR2BfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcR2BfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcR3BfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t bfValue);
int32_t adrv9001_Adc1MemMapAdcR3BfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint16_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcR4BfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcR4BfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                       uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcRcmBfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                        uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcRcmBfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
uint8_t channelId,                                        uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcResetInt1BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcResetInt1BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcRstDigBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcRstDigBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_Adc1MemMapAdcSpareBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_Adc1MemMapAdcSpareBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfAdc1MemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_ADC1_MEM_MAP_H__

/*  EOF: adrv9001_bf_adc1_mem_map.h */

