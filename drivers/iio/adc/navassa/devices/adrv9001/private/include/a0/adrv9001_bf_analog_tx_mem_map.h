/**
 * \file adrv9001_bf_analog_tx_mem_map.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __ADRV9001_BF_ANALOG_TX_MEM_MAP_H__
#define __ADRV9001_BF_ANALOG_TX_MEM_MAP_H__

#include "adrv9001_bf_analog_tx_mem_map_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_AnalogTxMemMapAnalogTxMemSpareBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapAnalogTxMemSpareBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapAuxDacTempSensorObsBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapAuxDacTempSensorObsBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapAuxDacTestModeEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapAuxDacTestModeEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapCalToneDriverAttnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapCalToneDriverAttnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapCalToneDriverInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapCalToneDriverInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapCalToneDriverPdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapCalToneDriverPdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapConverterLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapConverterLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapConverterLdoDisThermalShutdownBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                   uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapConverterLdoDisThermalShutdownBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapConverterLdoFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapConverterLdoFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapConverterLdoIlimitIncBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapConverterLdoIlimitIncBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapConverterLdoStatusBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapConverterLdoThermalShutdown0cBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapConverterLdoThermalShutdown0cBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapConverterLdoVoutSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapConverterLdoVoutSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapConverterLdoVoutTrimBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapConverterLdoVoutTrimBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapEnableLoSourceTxBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapEnableLoSourceTxBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapIdistBypassBiasResBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapIdistBypassBiasResBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterAcCoupleEnBfSet(adi_adrv9001_Device_t *device, 
                                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                       uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterAcCoupleEnBfGet(adi_adrv9001_Device_t *device, 
                                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetPdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetPdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapPdConverterLdoBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapPdConverterLdoBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapPdIDistBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapPdIDistBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxAbbfCgBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                             uint16_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxAbbfCgBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                             uint16_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxAbbfPeakdetDecayBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxAbbfPeakdetDecayBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxAuxLoBufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxAuxLoBufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxBbfDebugBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               uint16_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxBbfDebugBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxBbfPowCtrlBBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxBbfPowCtrlBBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxBbfSpareBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxBbfSpareBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmCorrPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmCorrPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmCorrPdOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmloBiasBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmloBiasBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmloDacIBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int16_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmloDacIBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int16_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmloDacLatchbBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmloDacLatchbBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmloDacQBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int16_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmloDacQBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int16_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmloSel2xDiodeBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmloSel2xDiodeBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxCmloSel2xMixerBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxCmloSel2xMixerBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoCmBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                           int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoCmBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                           int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                  int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                  int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoDelayDacBiasResBypassBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoDelayDacBiasResBypassBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoDelayDacResBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoDelayDacResBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLdoFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLdoFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLdoStatusBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLdoVoutSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLdoVoutSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLdoVoutTrimBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLdoVoutTrimBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLineRTermBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLineRTermBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLineRcvBufPdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLineRcvBufPdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLineTermBiasPdBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLineTermBiasPdBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxLoLineTermEnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxLoLineTermEnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPdAmpBiasResCalBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPdAmpBiasResCalBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPdDmEnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPdDmEnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPdLowNoiseSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPdLowNoiseSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPdProgOtaBiasBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPdProgOtaBiasBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPdRinBoostBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPdRinBoostBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPdbiasBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPdbiasBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasManualStartupBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasManualStartupBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasPtatStartBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasPtatStartBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistAmpCcBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistAmpCcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistAmpFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistAmpFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistAmpRzBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistAmpRzBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistPdBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistPdBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistPdoverrideBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistPdoverrideBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxPredistPdoverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                              int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxPredistPdoverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                              int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxUpconvPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxUpconvPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxUpconvPdoverrideBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxUpconvPdoverrideBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxUpconvPdoverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxUpconvPdoverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxVcmCntrlBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxVcmCntrlBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxabbfCcCntABfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxabbfCcCntABfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxabbfCcCntBBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxabbfCcCntBBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxbbfPdBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxbbfPdBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxbbfPdoverrideBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxbbfPdoverrideBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxbbfPdoverrideSelectBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxbbfPdoverrideSelectBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxucLvBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                           int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxucLvBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                           int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxucRfcascLvlBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxucRfcascLvlBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxucVcascctrlBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxucVcascctrlBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_AnalogTxMemMapTxucVcascoffBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_AnalogTxMemMapTxucVcascoffBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr, 
                                                 int8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_ANALOG_TX_MEM_MAP_H__

/*  EOF: adrv9001_bf_analog_tx_mem_map.h */

