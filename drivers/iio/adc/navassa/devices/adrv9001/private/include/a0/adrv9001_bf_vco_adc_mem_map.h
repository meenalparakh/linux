/**
 * \file adrv9001_bf_vco_adc_mem_map.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __ADRV9001_BF_VCO_ADC_MEM_MAP_H__
#define __ADRV9001_BF_VCO_ADC_MEM_MAP_H__

#include "adrv9001_bf_vco_adc_mem_map_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_VcoAdcMemMapAcc0DataBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint16_t *bfValue);


int32_t adrv9001_VcoAdcMemMapAcc1DataBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint16_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcalcenterBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint16_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcalcenterBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint16_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcapBfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcapBfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterAccCntValBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcenterAccCntValBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterAdaptDecreaseBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcenterAdaptDecreaseBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterAdaptIncreaseBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcenterAdaptIncreaseBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterAdaptStepsBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcenterAdaptStepsBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterCalEnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcenterCalEnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterInitValBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcenterInitValBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterScaleShiftBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcenterScaleShiftBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterStartInitValBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBcenterStartInitValBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBcenterValueBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBdithdac1BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBdithdac1BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBdithdac2BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBdithdac2BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBgainBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBgainBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBiasDisLpfBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBiasDisLpfBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBvtiamp1BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBvtiamp1BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBvtiamp2BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBvtiamp2BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBvtiampcmBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBvtiampcmBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBvtiampsBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapBvtiampsBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBvticenterBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint16_t bfValue);
int32_t adrv9001_VcoAdcMemMapBvticenterBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint16_t *bfValue);


int32_t adrv9001_VcoAdcMemMapBvtishiftBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint16_t bfValue);
int32_t adrv9001_VcoAdcMemMapBvtishiftBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint16_t *bfValue);


int32_t adrv9001_VcoAdcMemMapCalDutyCycleBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapCalDutyCycleBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapCalForceBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapCalForceBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapCalFreqControlBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapCalFreqControlBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapCalRstBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapCalRstBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapCalRzBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapCalRzBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapClkshiftBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapClkshiftBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDacvalKgainBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDacvalKgainBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDigCalRstBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDigCalRstBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDisBiasAmpBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDisBiasAmpBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDisOrcBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDisOrcBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDithDisAutoBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDithDisAutoBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDithDivBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDithDivBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDithModeBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDithModeBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDithRstClkgenBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDithRstClkgenBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDithRstDacBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDithRstDacBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDithRstLfsrBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapDithRstLfsrBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDoneBcenterCalBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDoneKgainCalBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapDoneRampCalBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEn34BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEn34BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEnAdcclkIBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEnAdcclkIBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEnAdcclkQBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEnAdcclkQBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEnBiasBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEnBiasBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEnCalAnaBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEnCalAnaBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEnCalClkBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEnCalClkBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEnVtiIBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEnVtiIBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEnVtiQBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEnVtiQBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapEnableClockBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapEnableClockBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapForceDacLevelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapForceDacLevelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapForceDacLevelEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapForceDacLevelEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapForceOrc1BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapForceOrc1BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapForceOrc2BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapForceOrc2BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapGainBcenterBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapGainBcenterBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapGainValueBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint32_t bfValue);
int32_t adrv9001_VcoAdcMemMapGainValueBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t adrv9001_VcoAdcMemMapGainValueRegmapEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapGainValueRegmapEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapKgainAccCntValBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapKgainAccCntValBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapKgainCalEnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapKgainCalEnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapKgainValBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint32_t bfValue);
int32_t adrv9001_VcoAdcMemMapKgainValBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint32_t *bfValue);


int32_t adrv9001_VcoAdcMemMapLutDataBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapLutDataBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
uint8_t channelId,                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapLutEnClkgateBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapLutEnClkgateBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapLutLoadedgeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapLutLoadedgeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapLutRegmapAccessInvalidBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapLutRstBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapLutRstBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapMatrixDataEnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapMatrixDataEnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapOutCtrlBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapOutCtrlBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapOutDisDemuxBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapOutDisDemuxBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapOvrDisBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapOvrDisBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapOvrRstBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapOvrRstBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapPwrupWaitClkBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapPwrupWaitClkBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapRampAccCntValBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapRampAccCntValBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapRampCalEnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapRampCalEnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapRegshiftBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapRegshiftBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapRstCalBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapRstCalBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSelRstBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSelRstBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSelShortBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSelShortBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg017BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg017BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg025BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg025BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg026BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg026BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg027BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg027BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg036BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg036BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg037BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg037BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg045BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg045BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg046BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg046BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg047BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg047BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg050BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg050BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg051BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg051BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg052BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg052BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg053BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg053BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg054BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg054BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg055BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg055BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg056BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg056BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg057BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg057BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg063BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg063BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg067BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg067BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg076BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg076BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg077BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg077BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0b3BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0b3BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0b4BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0b4BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0b5BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0b5BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0b6BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0b6BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0b7BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0b7BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0c0BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0c0BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0c1BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0c1BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0c2BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0c2BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0c3BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0c3BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0c4BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0c4BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0c5BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0c5BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0c6BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0c6BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg0c7BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg0c7BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg123BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg123BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg124BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg124BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg125BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg125BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg126BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg126BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg127BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg127BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg143BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg143BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg144BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg144BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg145BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg145BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg146BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg146BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg147BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg147BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpareReg157BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpareReg157BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapSpiLoadBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapSpiLoadBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapVStateBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapVtiDisShiftBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapVtiDisShiftBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapVtiForceBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapVtiForceBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_VcoAdcMemMapVtiShortBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_VcoAdcMemMapVtiShortBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_VCO_ADC_MEM_MAP_H__

/*  EOF: adrv9001_bf_vco_adc_mem_map.h */

