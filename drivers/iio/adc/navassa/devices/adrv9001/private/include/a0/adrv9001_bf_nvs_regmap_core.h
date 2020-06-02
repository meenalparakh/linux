/**
 * \file adrv9001_bf_nvs_regmap_core.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __ADRV9001_BF_NVS_REGMAP_CORE_H__
#define __ADRV9001_BF_NVS_REGMAP_CORE_H__

#include "adrv9001_bf_nvs_regmap_core_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_NvsRegmapCoreAdc1Clk1105SelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc1Clk1105SelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc1ClkDiv2SelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc1ClkDiv2SelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc1DataClk1105SelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc1DataClk1105SelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc1DataClkDiv2SelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc1DataClkDiv2SelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc1LpHpbSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc1LpHpbSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc2Clk1105SelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc2Clk1105SelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc2ClkDiv2SelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc2ClkDiv2SelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc2DataClk1105SelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc2DataClk1105SelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc2DataClkDiv2SelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc2DataClkDiv2SelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdc2LpHpbSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdc2LpHpbSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdcTunerRCCalClkDiv2SelBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdcTunerRCCalClkDiv2SelBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdcTunerRCCalClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAdcTunerRCCalClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAddrAscension2BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAddrAscension2BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAddrAscension5BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAddrAscension5BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdiCommonSpiReserved0BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAdiCommonSpiReserved1BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAhbSpiBridgeEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAhbSpiBridgeEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaAuxPllSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaAuxPllSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaClkPllLpSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaClkPllLpSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaClkPllSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaClkPllSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiRx1ClkPdBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaLssiRx1ClkPdBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiRx1McsCountBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaLssiRx1McsCountBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiRx1McsCounterActiveBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiRx1McsOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaLssiRx1McsOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiRx2ClkPdBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaLssiRx2ClkPdBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiRx2McsCountBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaLssiRx2McsCountBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiRx2McsCounterActiveBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiRx2McsOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaLssiRx2McsOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiTx1ClkPdBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaLssiTx1ClkPdBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaLssiTx2ClkPdBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaLssiTx2ClkPdBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRf1PllSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRf1PllSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRf2PllSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRf2PllSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx1Adc1SwResetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx1Adc1SwResetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx1Adc2ISwResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx1Adc2ISwResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx1Adc2QSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx1Adc2QSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx1AdcLpSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx1AdcLpSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx1AnalogSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx1AnalogSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx2Adc1SwResetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx2Adc1SwResetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx2Adc2ISwResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx2Adc2ISwResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx2Adc2QSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx2Adc2QSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx2AdcLpSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx2AdcLpSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaRx2AnalogSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaRx2AnalogSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaTx1AnalogSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaTx1AnalogSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaTx1DacSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaTx1DacSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaTx2AnalogSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaTx2AnalogSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnaTx2DacSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnaTx2DacSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnalogHsDigClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnalogHsDigClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnalogLvdsRx1ClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnalogLvdsRx1ClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnalogLvdsRx2ClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnalogLvdsRx2ClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnalogLvdsTx1ClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnalogLvdsTx1ClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnalogLvdsTx2ClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnalogLvdsTx2ClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAnalogSpareBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAnalogSpareBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArbDisableRespBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArbDisableRespBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArbRdCountBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArbRdCountBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArbRdCountAnalogBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArbRdCountAnalogBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmBootAddrBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int32_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmBootAddrBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmCalibrationErrorBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmCalibrationErrorBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmClkSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmClkSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmClockEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmClockEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmClockRateBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmClockRateBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmCommandBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmCommandBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmCommandBusyBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDebugEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDebugEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpCaptureEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpCaptureEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpCaptureForceBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpCaptureForceBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpCaptureSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpCaptureSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpDriveCaptureSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpDriveCaptureSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpDrvCounterLimitBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int16_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpDrvCounterLimitBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpDrvCounterResetBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpDrvCounterResetBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpDrvCounterStartBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int16_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpDrvCounterStartBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpDrvDacAdcbSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpDrvDacAdcbSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmDpDrvEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmDpDrvEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmErrorBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmErrorBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmExtCmdByte1BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmExtCmdByte1BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmExtCmdByte2BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmExtCmdByte2BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmExtCmdByte3BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmExtCmdByte3BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmExtCmdByte4BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmExtCmdByte4BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmExtCmdByte5BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmExtCmdByte5BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmForceResetBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmForceResetBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmHrespCodeSlaveBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmHrespSysSlaveBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmJtagSelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmJtagSelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmM3RunBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmM3RunBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmMonitorInterruptBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmMonitorInterruptBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmOverrideControlBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmOverrideControlBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmStackPtrBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int32_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmStackPtrBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreArmSystemErrorBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreArmSystemErrorBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAutoIncrBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAutoIncrBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc0DecimationCtlBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc0DecimationCtlBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc0PdBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc0PdBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc0ReadBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc0ResetBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc0ResetBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc0SelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc0SelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc1DecimationCtlBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc1DecimationCtlBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc1PdBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc1PdBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc1ReadBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc1ResetBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc1ResetBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc1SelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc1SelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc2DecimationCtlBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc2DecimationCtlBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc2PdBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc2PdBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc2ReadBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc2ResetBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc2ResetBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc2SelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc2SelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc3DecimationCtlBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc3DecimationCtlBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc3PdBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc3PdBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc3ReadBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc3ResetBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc3ResetBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdc3SelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdc3SelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdcTempSenseDecimationCtlBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdcTempSenseDecimationCtlBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdcTempSenseDigResetbBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdcTempSenseDigResetbBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdcTempSensePdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdcTempSensePdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxAdcTempSenseResetBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxAdcTempSenseResetBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxSynthLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxSynthLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxSynthLdoFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxSynthLdoFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxSynthLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxSynthLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxSynthLdoStatusBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxSynthLdoVoutSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxSynthLdoVoutSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreAuxSynthLdoVoutTrimBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreAuxSynthLdoVoutTrimBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreBlockoutWindowSizeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreBlockoutWindowSizeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreBusAddrBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                           uint32_t bfValue);
int32_t adrv9001_NvsRegmapCoreBusAddrBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                           uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreBusResponseBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreBusResponseBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreBusSizeBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreBusSizeBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreBusWaitingBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreChipGradeLowerNibbleBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreChipGradeUpperNibbleBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreChipTypeRadBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClk1105ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClk1105ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkPllLpVcoLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkPllLpVcoLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkPllVcoLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkPllVcoLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLdoFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLdoFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLdoStatusBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLdoVoutSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLdoVoutSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLdoVoutTrimBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLdoVoutTrimBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoStatusBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoVoutSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoVoutSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoVoutTrimBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreClkSynthLpLdoVoutTrimBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreCompletedStreamNumBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreControllerDocRevisionBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreCsbStallBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreCsbStallBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreCssiTxdpIntfSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreCssiTxdpIntfSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDac1ClkHsclkDivRatioSelBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDac1ClkHsclkDivRatioSelBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDac2ClkHsclkDivRatioSelBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDac2ClkHsclkDivRatioSelBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevClkLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkLdoFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevClkLdoFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevClkLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkLdoStatusBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkLdoVoutSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevClkLdoVoutSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkLdoVoutTrimBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevClkLdoVoutTrimBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkSysrefHoldWordBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkSysrefSetupWordBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkbufOffsetEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevClkbufOffsetEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkbufTrmIbiasBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevClkbufTrmIbiasBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevClkbufTrmVcmBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevClkbufTrmVcmBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDevclkDividerMcsResetbBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDevclkDividerMcsResetbBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDeviceClkBufferEnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDeviceClkBufferEnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDeviceClkDivideRatioBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDeviceClkDivideRatioBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDeviceConfigCustomOpModesBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDeviceConfigOpModesBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDeviceConfigStatusBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDigDeviceClkBufferEnableBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDigDeviceClkBufferEnableBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDigSysrefSampleEnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDigSysrefSampleEnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDivAnaLssiRx1ClkBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDivAnaLssiRx1ClkBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDivAnaLssiRx2ClkBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDivAnaLssiRx2ClkBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDivAnaLssiTx1ClkBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDivAnaLssiTx1ClkBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDivAnaLssiTx2ClkBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDivAnaLssiTx2ClkBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDivHsDigClkBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDivHsDigClkBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaAbortBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaAbortBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaAbortFlagBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaAhbErrorBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaBlockBaseOffsetAddrBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaBlockBaseOffsetAddrBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaConfigIndexModeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaConfigIndexModeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaContinuousIndexModeBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaContinuousIndexModeBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaCurrIndexBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaCurrentBlockAddrBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaCurrentIndexAddrBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaCurrentProfileAddrBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaEnableBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaEnableBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaEndOfTransferFlagBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaErrorFlagBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaFieldTypeErrorBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaGenbusErrorBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaIgnoreTxrxMatchBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaIgnoreTxrxMatchBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaMaxNumberOfWordBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaMaxNumberOfWordBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaMaxWordCountErrorBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaNextRxBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaNextRxBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaNextRxMismatchErrorBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaNextSkipBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaNextSkipBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaNextTxBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaNextTxBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaNextTxMismatchErrorBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaRegmapOffsetAddrBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaRegmapOffsetAddrBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaStartTableAddrBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaStartTableAddrBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaStateBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaStatusBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaStopBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaStopBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaStopFlagBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaTableBaseOffsetAddrBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreDmaTableBaseOffsetAddrBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaUnexpectedNextRxErrorBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaUnexpectedNextSkipErrorBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDmaUnexpectedNextTxErrorBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreDoOutDrvBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreDoOutDrvBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccDataParityCountBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccDataParityErrorBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreEccDataParityErrorBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccDataParityInBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccDataParityIndexBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccDataParityOutBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccDataParityTagBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccDisableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreEccDisableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccIrqSensitivityBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreEccIrqSensitivityBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccProgParityCountBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccProgParityErrorBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreEccProgParityErrorBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccProgParityInBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccProgParityIndexBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccProgParityOutBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreEccProgParityTagBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreForceGpInterruptBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreForceGpInterruptBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoBypass1BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoBypass1BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoBypass2BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoBypass2BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoFilterBypass1BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoFilterBypass1BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoFilterBypass2BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoFilterBypass2BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoPd1BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoPd1BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoPd2BfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoPd2BfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoStatus1BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoStatus2BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoVoutSel1BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoVoutSel1BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoVoutSel2BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoVoutSel2BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoVoutTrim1BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoVoutTrim1BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpLdoVoutTrim2BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpLdoVoutTrim2BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio1110InvertBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio1110InvertBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio1110SourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio1110SourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio1312InvertBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio1312InvertBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio1312SourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio1312SourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio1514InvertBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio1514InvertBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio1514SourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio1514SourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio10InvertBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio10InvertBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio10SourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio10SourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio32InvertBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio32InvertBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio32SourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio32SourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio54InvertBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio54InvertBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio54SourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio54SourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio76InvertBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio76InvertBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio76SourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio76SourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio98InvertBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio98InvertBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpio98SourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpio98SourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreGpioTelemetryEnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreGpioTelemetryEnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreHs2refClockSwitchEnableBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreHs2refClockSwitchEnableBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreHsDigClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreHsDigClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreHsDigClkPdBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreHsDigClkPdBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLastStreamNumBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLastStreamNumBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLegacyModeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLegacyModeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLoopback1FilterFlipBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLoopback1FilterFlipBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLoopback2FilterFlipBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLoopback2FilterFlipBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLpbkEnableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLpbkEnableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLsbFirst1BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLsbFirst1BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLsbFirst6BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLsbFirst6BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLssiTxdpIntfSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLssiTxdpIntfSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLvdsIbiasDistPdBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLvdsIbiasDistPdBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreLvdsIbiasDistResBypassBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreLvdsIbiasDistResBypassBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMainSpBbicStreamErrorToSpiBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint32_t bfValue);
int32_t adrv9001_NvsRegmapCoreMainSpBbicStreamErrorToSpiBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMainSpBbicStreamStatusBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMaskRevisionMajorBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMaskRevisionMinorBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMasterSlaveReadbackCtlBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMasterSlaveTransferBitBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsAndSsiTx1FifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsAndSsiTx2FifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsCaptureEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMcsCaptureEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsCaptureReceivedBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsCaptureReceivedClearBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMcsCaptureReceivedClearBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsClkPllSdmSyncStatusBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsDeviceClkDividerSyncStatusBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsDigitalClocksSyncStatusBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsInternalGenBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMcsInternalGenBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsInternalGenSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMcsInternalGenSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsJesdSysrefStatusBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsRetimeBypassBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMcsRetimeBypassBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsSecondCaptureReceivedBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsSecondCaptureReceivedClearBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMcsSecondCaptureReceivedClearBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMcsStretchedTestSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMcsStretchedTestSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMemHrespMaskBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMemHrespMaskBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMemWriteDataBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int32_t bfValue);
int32_t adrv9001_NvsRegmapCoreMemWriteDataBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonTimerAliveTimeBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonTimerAliveTimeBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonTimerCurrTimeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonTimerEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonTimerEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonTimerEnableWithMonitorBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonTimerEnableWithMonitorBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonTimerInitTimeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonTimerInitTimeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonTimerModeBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonTimerModeBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonTimerSleepTimeBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonTimerSleepTimeBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonTimerStateBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorClkDivBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorClkDivBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableFromSpiSelBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableFromSpiSelBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableGpioMaskBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableGpioMaskBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableGpioSelectBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableGpioSelectBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableHsclkGaterBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableHsclkGaterBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableSwitchRef2hsclkBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableSwitchRef2hsclkBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableWakeupFromMonTimerBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableWakeupFromMonTimerBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableWakeupFromPinBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableWakeupFromPinBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorEnableWakeupFromSpiBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorEnableWakeupFromSpiBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorForcePllLockBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorForcePllLockBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorInterruptClkExtendBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorInterruptClkExtendBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorKeepArmRunningBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorKeepArmRunningBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorSleepEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorSleepEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorStateBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorTimerAliveFirstBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorTimerAliveFirstBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorWakeupFromSpiBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorWakeupFromSpiBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorWakeupGpioMaskBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorWakeupGpioMaskBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreMonitorWakeupGpioSelectBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreMonitorWakeupGpioSelectBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreNvsGpioDirectionControlOeBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreNvsGpioDirectionControlOeBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreNvsGpioSpiReadBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreNvsGpioSpiSourceBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreNvsGpioSpiSourceBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrx2rxWithoutPowerDownBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrx2rxWithoutPowerDownBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrxEnableBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrxEnableBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrxEnableByArmBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrxEnableByArmBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrxEnableGpioSelectBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrxEnableGpioSelectBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrxPinModeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrxPinModeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrxSelectBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrxSelectBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrxSingleChModeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrxSingleChModeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrxSingleChSelectPinModeBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrxSingleChSelectPinModeBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreOrxUsesRxPinsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreOrxUsesRxPinsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpIntDrvBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpIntDrvBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpIntPeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpIntPeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpIntPsBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpIntPsBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpIntSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpIntSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p80DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p80DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p80PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p80PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p80PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p80PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p80SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p80SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p810DrvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p810DrvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p810PeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p810PeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p810PsBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p810PsBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p810SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p810SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p811DrvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p811DrvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p811PeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p811PeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p811PsBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p811PsBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p811SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p811SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p81DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p81DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p81PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p81PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p81PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p81PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p81SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p81SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p82DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p82DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p82PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p82PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p82PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p82PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p82SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p82SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p83DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p83DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p83PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p83PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p83PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p83PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p83SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p83SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p84DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p84DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p84PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p84PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p84PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p84PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p84SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p84SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p85DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p85DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p85PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p85PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p85PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p85PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p85SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p85SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p86DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p86DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p86PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p86PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p86PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p86PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p86SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p86SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p87DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p87DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p87PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p87PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p87PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p87PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p87SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p87SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p88DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p88DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p88PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p88PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p88PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p88PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p88SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p88SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p89DrvBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p89DrvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p89PeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p89PeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p89PsBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p89PsBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadGpio1p89SmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadGpio1p89SmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadModeDrvBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadModeDrvBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadModeSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadModeSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRefClkDrvBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRefClkDrvBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRefClkPeBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRefClkPeBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRefClkPsBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRefClkPsBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRefClkSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRefClkSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadResetNDrvBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadResetNDrvBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadResetNSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadResetNSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1IdataCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1QdataCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1ClkCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1EnableDrvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1EnableDrvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1EnablePeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1EnablePeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1EnablePsBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1EnablePsBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1EnableSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1EnableSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx1StrobeCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2IdataCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2QdataCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2ClkCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2EnableDrvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2EnableDrvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2EnablePeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2EnablePeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2EnablePsBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2EnablePsBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2EnableSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2EnableSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadRx2StrobeCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiClkDrvBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiClkDrvBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiClkPeBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiClkPeBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiClkPsBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiClkPsBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiClkSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiClkSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiCsbDrvBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiCsbDrvBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiCsbPeBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiCsbPeBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiCsbPsBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiCsbPsBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiCsbSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiCsbSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiDiDrvBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiDiDrvBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiDiPeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiDiPeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiDiPsBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiDiPsBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiDiSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiDiSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiDoDrvBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiDoDrvBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiDoPeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiDoPeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiDoPsBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiDoPsBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadSpiDoSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadSpiDoSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1IdataCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1QdataCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1ClkCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1EnableDrvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1EnableDrvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1EnablePeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1EnablePeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1EnablePsBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1EnablePsBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1EnableSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1EnableSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1RefclkCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx1StrobeCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2IdataCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2QdataCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2ClkCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2EnableDrvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2EnableDrvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2EnablePeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2EnablePeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2EnablePsBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2EnablePsBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2EnableSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2EnableSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2RefclkCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosDrvSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosDrvSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosPeNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosPeNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosPePBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosPePBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosPsNBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosPsNBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosPsPBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosPsPBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosSmittTrigBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePadTx2StrobeCmosSmittTrigBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePagePointerDeviceIndexValue0BfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePagePointerDeviceIndexValue1BfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePdBbRefclkActiveShieldBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePdBbRefclkActiveShieldBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePdMasterbiasBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePdMasterbiasBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirCoeffAddr70BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirCoeffAddr70BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirCoeffAddr8BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirCoeffAddr8BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirCoeffDataBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirCoeffDataBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirCoeffRxASelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirCoeffRxASelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirCoeffRxBSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirCoeffRxBSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirCoeffRxCSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirCoeffRxCSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirCoeffRxDSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirCoeffRxDSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirCoeffWrEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirCoeffWrEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirLoopbackBankSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirLoopbackBankSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirRx1BankSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirRx1BankSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirRx2BankSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirRx2BankSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirTxACoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirTxACoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirTxBCoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirTxBCoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirTxCCoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirTxCCoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCorePfirTxDCoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCorePfirTxDCoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRcalStartBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRcalStartBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRdWrbBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRdWrbBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore0BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore1BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore10BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore11BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore12BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore13BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore14BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore15BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore2BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore3BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore4BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore5BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore6BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore7BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore8BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReadbackValSemaphore9BfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRefClkGenPdAuxPllBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRefClkGenPdAuxPllBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRefClkGenPdClkPllBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRefClkGenPdClkPllBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRefClkGenPdClkPllLpBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRefClkGenPdClkPllLpBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRefClkGenPdRfpll1BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRefClkGenPdRfpll1BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRefClkGenPdRfpll2BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRefClkGenPdRfpll2BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRefPadBypBiasRBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRefPadBypBiasRBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReferenceClockCyclesBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreReferenceClockCyclesBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreResampleClkDivRatioBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreResampleClkDivRatioBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreResetRwPolarityBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreResetRwPolarityBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreResetbTflashBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreResetbTflashBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreReverseRwPolarityBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreReverseRwPolarityBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1IdataVcmBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1IdataVcmBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1QdataVcmBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1QdataVcmBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1AdcDelaySelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1AdcDelaySelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1ClkVcmBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1ClkVcmBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1DemuxEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1DemuxEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1LvdsMcsCaptureReceivedBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1LvdsMcsSecondCaptureReceivedBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1MagBankSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1MagBankSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1McsDelayCntBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1McsDelayCntBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1McsReadFifoDelayCntBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1McsReadFifoDelayCntBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1McsSecondCaptureReceivedClearBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1McsSecondCaptureReceivedClearBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1NbCoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1NbCoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1RxqecCorrRdyGenOffTimeBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int16_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1RxqecCorrRdyGenOffTimeBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1RxqecCorrRdyGenOnTimeBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1RxqecCorrRdyGenOnTimeBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1RxqecCorrRdyGenSoftResetBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1RxqecCorrRdyGenSoftResetBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1StrobeVcmBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1StrobeVcmBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfEnableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfEnableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoDataLostClearBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoDataLostClearBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoDataLostIrqEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoDataLostIrqEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoDataLostStatusBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoRdStartThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoRdStartThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoReadErrorClearBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoReadErrorClearBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoReadErrorIrqEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoReadErrorIrqEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfFifoReadErrorStatusBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfIdataBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfIrqEdgeTriggeredBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfIrqEdgeTriggeredBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfQdataBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfReadRequestClearBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfReadRequestClearBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfReadRequestIrqAutoclearBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfReadRequestIrqAutoclearBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfReadRequestIrqEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx1dpIntfReadRequestIrqEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx1dpIntfReadRequestStatusBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2IdataVcmBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2IdataVcmBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2QdataVcmBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2QdataVcmBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2AdcDelaySelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2AdcDelaySelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2ClkVcmBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2ClkVcmBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2DemuxEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2DemuxEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2LvdsMcsCaptureReceivedBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2LvdsMcsSecondCaptureReceivedBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2MagBankSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2MagBankSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2McsDelayCntBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2McsDelayCntBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2McsReadFifoDelayCntBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2McsReadFifoDelayCntBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2McsSecondCaptureReceivedClearBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2McsSecondCaptureReceivedClearBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2NbCoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2NbCoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2RxqecCorrRdyGenOffTimeBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int16_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2RxqecCorrRdyGenOffTimeBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2RxqecCorrRdyGenOnTimeBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2RxqecCorrRdyGenOnTimeBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2RxqecCorrRdyGenSoftResetBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2RxqecCorrRdyGenSoftResetBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2StrobeVcmBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2StrobeVcmBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRx2orxWithoutPowerDownBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRx2orxWithoutPowerDownBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxAlgMux301FiraBypBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxAlgMux301FiraBypBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxAlgMux302Ch1SelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxAlgMux302Ch1SelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxAlgMux303Ch1SelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxAlgMux303Ch1SelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankAPfirDecimationBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankAPfirDecimationBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankAPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankAPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankAPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankAPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankASymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankASymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankBPfirDecimationBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankBPfirDecimationBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankBPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankBPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankBPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankBPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankBSymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankBSymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankCPfirDecimationBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankCPfirDecimationBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankCPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankCPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankCPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankCPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankCSymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankCSymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankDPfirDecimationBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankDPfirDecimationBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankDPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankDPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankDPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankDPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxBankDSymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxBankDSymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxChannel1IDataMaskBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxChannel1IDataMaskBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxChannel1QDataMaskBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxChannel1QDataMaskBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxChannel2IDataMaskBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxChannel2IDataMaskBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxChannel2QDataMaskBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxChannel2QDataMaskBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxDpGainComp9UpdateBothBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxDpGainComp9UpdateBothBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxEnableBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxEnableBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxEnableByArmBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxEnableByArmBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxHpAdcSamplesLsbAlignedBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxHpAdcSamplesLsbAlignedBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxMagACoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxMagACoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxMagBCoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxMagBCoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxMagCCoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxMagCCoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxMagDCoeffSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxMagDCoeffSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxadc1InputMaskBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxadc1InputMaskBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxadc2InputMaskBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxadc2InputMaskBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecAlgrBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecAlgrBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecCalenBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecCalenBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecCidBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecCidBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecClearBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecClearBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecCommonClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecCommonClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecHaltBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecHaltBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecIaddrBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecIaddrBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecIaddr0BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecIaddr0BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecObsIreadySelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecObsIreadySelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecObsRdyGenOffTimeBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int16_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecObsRdyGenOffTimeBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecObsRdyGenOnTimeBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecObsRdyGenOnTimeBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecObsRdyGenSoftResetBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecObsRdyGenSoftResetBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecOclkRateBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecOclkRateBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecRdata0BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecRdata1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecRdata2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecRdata3BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecReadBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecReadBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecResetBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecResetBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecRptBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecRptBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecSourceBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecSourceBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecSpareBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecSpareBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecWdata0BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecWdata0BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecWdata1BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecWdata1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecWdata2BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecWdata2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecWdata3BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecWdata3BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecWindenBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecWindenBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreRxqecWriteBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreRxqecWriteBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreScratchPadReadOnlyWordUpperAddressBfGet(adi_adrv9001_Device_t *device, 
                                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreScratchPadWordBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreScratchPadWordBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreScratchPadWordUpperAddressBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreScratchPadWordUpperAddressBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSdoActive3BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSdoActive3BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSdoActive4BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSdoActive4BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSelClkPllLpBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSelClkPllLpBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore0LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore0LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore0ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore0ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore10LockBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore10LockBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore10ProcBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore10ProcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore11LockBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore11LockBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore11ProcBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore11ProcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore12LockBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore12LockBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore12ProcBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore12ProcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore13LockBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore13LockBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore13ProcBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore13ProcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore14LockBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore14LockBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore14ProcBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore14ProcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore15LockBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore15LockBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore15ProcBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore15ProcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore1LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore1LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore1ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore1ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore2LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore2LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore2ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore2ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore3LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore3LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore3ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore3ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore4LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore4LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore4ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore4ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore5LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore5LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore5ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore5ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore6LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore6LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore6ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore6ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore7LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore7LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore7ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore7ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore8LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore8LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore8ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore8ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore9LockBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore9LockBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSemaphore9ProcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSemaphore9ProcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSingleInstructionBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSingleInstructionBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSlowInterfaceCtlBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSnooperEnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSnooperEnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSoftReset0BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSoftReset0BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSoftReset0011BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSoftReset0011BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSoftReset0012BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSoftReset0012BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSoftReset7BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSoftReset7BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat0BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat1BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat10BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat11BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat12BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat13BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat14BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat15BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat2BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat3BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat4BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat5BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat6BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat7BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat8BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStat9BfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamBaseBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamBaseBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamErrorBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamErrorBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamProcAddrBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamProcAddrBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamProcDataBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamProcGpio0PinSelectBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamProcGpio0PinSelectBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamProcGpio1PinSelectBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamProcGpio1PinSelectBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamProcGpio2PinSelectBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamProcGpio2PinSelectBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamProcGpio3PinSelectBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamProcGpio3PinSelectBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamProcGpioPinMaskBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamProcGpioPinMaskBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamProcRdenBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamProcRdenBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamResetBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamResetBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocCh2uBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocCh2uBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocDebugDataBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint64_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocDebugDataBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint64_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocLpbk1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocLpbk1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocLpbk1EnableCgBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocLpbk1EnableCgBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocLpbk2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocLpbk2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocLpbk2EnableCgBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocLpbk2EnableCgBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocOrx1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocOrx1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocOrx2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocOrx2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocRx1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocRx1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocRx2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocRx2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocRxPfirModeEnableBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocRxPfirModeEnableBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocRxorx1EnableCgBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocRxorx1EnableCgBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocRxorx2EnableCgBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocRxorx2EnableCgBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocTx1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocTx1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocTx1EnableCgBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocTx1EnableCgBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocTx2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocTx2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreStreamprocTx2EnableCgBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreStreamprocTx2EnableCgBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSwInterruptBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSwInterruptBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoBypassRf1BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoBypassRf1BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoBypassRf2BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoBypassRf2BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoFilterBypassRf1BfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoFilterBypassRf1BfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoFilterBypassRf2BfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoFilterBypassRf2BfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoPdRf1BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoPdRf1BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoPdRf2BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoPdRf2BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoStatusRf1BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoStatusRf2BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoVoutSelRf1BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoVoutSelRf1BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoVoutSelRf2BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoVoutSelRf2BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoVoutTrimRf1BfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoVoutTrimRf1BfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSynthLdoVoutTrimRf2BfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSynthLdoVoutTrimRf2BfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSysCodebBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSysCodebBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSysrefBufferEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSysrefBufferEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSysrefDrvrForcePdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSysrefDrvrForcePdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSysrefEdgeSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSysrefEdgeSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSysrefRegisterEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSysrefRegisterEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSysrefSampleEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSysrefSampleEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreSysrefTransitionSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreSysrefTransitionSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcDacinRefBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreTdegcDacinRefBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcDacinRefOvdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTdegcDacinRefOvdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcDoneBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcInitBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTdegcInitBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcMeasCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTdegcMeasCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcMeasMuxSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTdegcMeasMuxSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcPtatBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTdegcPtatBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcReadbackBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTdegcWaitCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTdegcWaitCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTestMcsEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTestMcsEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTopClockDividerResetBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTopClockDividerResetBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTrxMemClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTrxMemClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IdataCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IdataCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IdataCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IdataCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IdataCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IdataCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IdataCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IdataCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IdataLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IdataLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IdataLvdsIeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IdataLvdsIeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IdataLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IdataLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IdataLvdsRxOnchipTermBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IdataLvdsRxOnchipTermBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QdataCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QdataCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QdataCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QdataCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QdataCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QdataCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QdataCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QdataCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QdataLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QdataLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QdataLvdsIeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QdataLvdsIeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QdataLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QdataLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QdataLvdsRxOnchipTermBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QdataLvdsRxOnchipTermBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1ClkCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1ClkCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1ClkCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1ClkCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1ClkCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1ClkCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1ClkCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1ClkCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1ClkLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1ClkLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1ClkLvdsIeBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1ClkLvdsIeBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1ClkLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1ClkLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1ClkLvdsRxOnchipTermBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1ClkLvdsRxOnchipTermBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1IBankSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1IBankSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1McsDelayCntBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1McsDelayCntBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1McsReadFifoDelayCntBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1McsReadFifoDelayCntBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1McsVsDataValidCntBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1QBankSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1QBankSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1RefclkVcmBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1RefclkVcmBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1StrobeCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1StrobeCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1StrobeCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1StrobeCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1StrobeCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1StrobeCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1StrobeCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1StrobeCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1StrobeLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1StrobeLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1StrobeLvdsIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1StrobeLvdsIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1StrobeLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1StrobeLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx1StrobeLvdsRxOnchipTermBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx1StrobeLvdsRxOnchipTermBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IdataCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IdataCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IdataCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IdataCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IdataCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IdataCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IdataCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IdataCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IdataLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IdataLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IdataLvdsIeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IdataLvdsIeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IdataLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IdataLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IdataLvdsRxOnchipTermBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IdataLvdsRxOnchipTermBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QdataCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QdataCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QdataCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QdataCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QdataCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QdataCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QdataCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QdataCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QdataLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QdataLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QdataLvdsIeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QdataLvdsIeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QdataLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QdataLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QdataLvdsRxOnchipTermBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QdataLvdsRxOnchipTermBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2ClkCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2ClkCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2ClkCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2ClkCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2ClkCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2ClkCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2ClkCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2ClkCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2ClkLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2ClkLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2ClkLvdsIeBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2ClkLvdsIeBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2ClkLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2ClkLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2ClkLvdsRxOnchipTermBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2ClkLvdsRxOnchipTermBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2IBankSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2IBankSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2McsDelayCntBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2McsDelayCntBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2McsReadFifoDelayCntBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2McsReadFifoDelayCntBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2McsVsDataValidCntBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2QBankSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2QBankSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkLvdsCalibBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkLvdsCalibBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkLvdsOeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkLvdsOeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2RefclkVcmBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2RefclkVcmBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2StrobeCmosNIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2StrobeCmosNIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2StrobeCmosNOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2StrobeCmosNOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2StrobeCmosPIeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2StrobeCmosPIeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2StrobeCmosPOeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2StrobeCmosPOeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2StrobeLvdsDelayBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2StrobeLvdsDelayBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2StrobeLvdsIeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2StrobeLvdsIeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2StrobeLvdsPnInvBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2StrobeLvdsPnInvBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTx2StrobeLvdsRxOnchipTermBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTx2StrobeLvdsRxOnchipTermBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankAPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankAPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankAPfirSymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankAPfirSymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankAPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankAPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankBPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankBPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankBPfirSymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankBPfirSymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankBPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankBPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankCPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankCPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankCPfirSymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankCPfirSymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankCPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankCPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankDPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankDPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankDPfirSymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankDPfirSymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxBankDPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxBankDPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxComplexGainUpdateBothBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxComplexGainUpdateBothBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxDataMaskCh1BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxDataMaskCh1BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxDataMaskCh2BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxDataMaskCh2BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxDataMaskOnRfpllUnlockCh1BfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxDataMaskOnRfpllUnlockCh1BfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxDataMaskOnRfpllUnlockCh2BfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxDataMaskOnRfpllUnlockCh2BfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxDataRampDownOnRfpllUnlockCh1BfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxDataRampDownOnRfpllUnlockCh1BfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxDataRampDownOnRfpllUnlockCh2BfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxDataRampDownOnRfpllUnlockCh2BfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxEnableBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxEnableBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxEnableByArmBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxEnableByArmBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultDataEnableBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultDataEnableBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultDataUsedClearBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultDataUsedClearBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultDataUsedIrqEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultDataUsedIrqEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultDataUsedStatusBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultDataUsedTransitionsCountBfGet(adi_adrv9001_Device_t *device, 
                                                                           adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultIdataBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultIdataBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultQdataBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfDefaultQdataBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoRdStartThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoRdStartThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoWrReqThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoWrReqThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoWriteErrorClearBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoWriteErrorClearBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoWriteErrorIrqEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoWriteErrorIrqEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfFifoWriteErrorStatusBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfIdataBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfIdataBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfIrqEdgeTriggeredBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfIrqEdgeTriggeredBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfQdataBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfQdataBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfWriteRequestClearBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfWriteRequestClearBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfWriteRequestIrqAutoclearBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfWriteRequestIrqAutoclearBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfWriteRequestIrqEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxdpIntfWriteRequestIrqEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxdpIntfWriteRequestStatusBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreTxrxPinModeBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreTxrxPinModeBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreVendorIdBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                            uint16_t *bfValue);


int32_t adrv9001_NvsRegmapCoreWrEnDelayBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapCoreWrEnDelayBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapCoreWriteRegClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapCoreWriteRegClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapCoreChanAddr_e baseAddr, 
                                                    int8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_NVS_REGMAP_CORE_H__

/*  EOF: adrv9001_bf_nvs_regmap_core.h */

