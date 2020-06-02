/**
 * \file adrv9001_bf_nvs_regmap_tx.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __ADRV9001_BF_NVS_REGMAP_TX_H__
#define __ADRV9001_BF_NVS_REGMAP_TX_H__

#include "adrv9001_bf_nvs_regmap_tx_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_NvsRegmapTxAllTxEnableBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxAllTxEnableBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxAnalogSpareBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxAnalogSpareBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxAnaDigDelayPreBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxAnaLssiClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxAnaLssiClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxAttenRampDownBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapTxAttenRampDownBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxAttenRampUpBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapTxAttenRampUpBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc0NumBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc0NumBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc0NumBBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc0NumBBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc0NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc0NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc0NumRecipBBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc0NumRecipBBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc1NumBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc1NumBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc1NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc1NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc2NumBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc2NumBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc2NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc2NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc3NumBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc3NumBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAcc3NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAcc3NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAccClearBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAccClearBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAccOvrBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAccOvrBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAccOvrValBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAccOvrValBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAccSelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAccSelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmAccSelBBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmAccSelBBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmClearModeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmClearModeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmModeBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmModeBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmReadStrobeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmReadStrobeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmTlAvgExpBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmTlAvgExpBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmTlAvgManBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmTlAvgManBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmTlInstBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmTlInstBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmTuAvgExpBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmTuAvgExpBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmTuAvgManBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmTuAvgManBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmTuInstBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmTuInstBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcPmXSelBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcPmXSelBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClgcRssiCcDataCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClgcRssiCcDataCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxClkgenOutputBufferEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxClkgenOutputBufferEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxComplexGainBypassBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxComplexGainBypassBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxComplexGainIBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxComplexGainIBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxComplexGainQBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxComplexGainQBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxComplexGainUpdateSingleBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxComplexGainUpdateSingleBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxConstStepModeStepSizeBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapTxConstStepModeStepSizeBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxConstStepModeWaitDurationBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapTxConstStepModeWaitDurationBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxControlOutMuxSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxControlOutMuxSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxdpIntfSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxdpIntfSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxClearDdrStrobeAlignErrorBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxClearDdrStrobeAlignErrorBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxClearRampShiftErrorBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxClearRampShiftErrorBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDataFormatBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDataFormatBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDataFormatFourLanesBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDataFormatFourLanesBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDdrClkInvBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDdrClkInvBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDdrNegStrobeSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDdrNegStrobeSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDdrPosClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDdrPosClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDdrSelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDdrSelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDdrStrobeAlignErrorBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugCapturedDataBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugCapturedDataSecondBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugLoad4bitsModeBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDebugLoad4bitsModeBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugModeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDebugModeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugNibbleSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDebugNibbleSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugQSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDebugQSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugRampShiftErrorBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugStartCaptureBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDebugStartCaptureBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugStartRampBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDebugStartRampBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxDebugStartShiftBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxDebugStartShiftBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxEnableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxEnableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxFifoClearBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxFifoClearBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxFifoClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxFifoClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxFifoEmptyBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxFifoFullBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxForceMcsBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxForceMcsBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxInvertClkBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxInvertClkBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxLongStrobeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxLongStrobeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxLsbFirstBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxLsbFirstBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxQFirstBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxQFirstBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxRdFifoEnableDelayBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxCssiTxRdFifoEnableDelayBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxCssiTxStrobeAlignErrorBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDacClockEnableBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDacClockEnableBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDataCaptureDriveEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDataCaptureDriveEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDirectModAnalogClkdivValueBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDirectModAnalogClkdivValueBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDirectModClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDirectModClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDirectModClkPolarityBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDirectModClkPolarityBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDirectModEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDirectModEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDirectModFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDirectModFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDirectModNumClkCyclesBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDirectModNumClkCyclesBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxDirectModRdFifoEnableDelayBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxDirectModRdFifoEnableDelayBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxEnableSpiCsbUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapTxEnableSpiCsbUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxEnConstStepModeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxEnConstStepModeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxFlushPeriodBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxFlushPeriodBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanAmpRssiCcDataCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanAmpRssiCcDataCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc0NumBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc0NumBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc0NumBBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc0NumBBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc0NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc0NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc0NumRecipBBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc0NumRecipBBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc1NumBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc1NumBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc1NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc1NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc2NumBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc2NumBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc2NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc2NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc3NumBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc3NumBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAcc3NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAcc3NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAccClearBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAccClearBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAccOvrBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAccOvrBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAccOvrValBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAccOvrValBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAccSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAccSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmAccSelBBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmAccSelBBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmClearModeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmClearModeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmModeBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmModeBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmReadStrobeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmReadStrobeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmTlAvgExpBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmTlAvgExpBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmTlAvgManBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmTlAvgManBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmTlInstBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmTlInstBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmTuAvgExpBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmTuAvgExpBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmTuAvgManBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmTuAvgManBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmTuInstBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmTuInstBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxGanPaPmXSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGanPaPmXSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGdcalAnaGain1BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGdcalAnaGain1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGdcalAnaGain2BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGdcalAnaGain2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGdcalDigGain1BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGdcalDigGain1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGdcalDigGain2BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGdcalDigGain2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGdcalStatePreBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGdcalTxStartCalBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGdcalTxStartCalBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxGdcalWindowSizeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxGdcalWindowSizeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxInterruptArmOnlyForAttenHpChangesBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapTxInterruptArmOnlyForAttenHpChangesBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxIntDelay0EnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxIntDelay0EnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxIntDelay1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxIntDelay1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxIntTxEnableBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxIntTxEnableBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLdoResLoadBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLdoResLoadBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLoLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLoLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLoLdoGmBoostBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLoLdoGmBoostBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLoLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLoLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxdpIntfSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxdpIntfSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxClearEarlyStrobeDetectedFlagBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxClearEarlyStrobeDetectedFlagBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxClearStrobeAlignErrorBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxClearStrobeAlignErrorBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDataFormat12bitBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDataFormat12bitBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDataOneLaneBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDataOneLaneBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugAfterFifoErrorClearBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugAfterFifoErrorClearBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugCapturedDataBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugCapturedDataSecondBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                               int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugErrorBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugErrorCounterReadBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugErrorCounterReadBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugModeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugModeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs15EnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs15EnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs15RampShiftErrorCountBfGet(adi_adrv9001_Device_t *device, 
                                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs15RestartBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs15RestartBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7EnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7EnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7ErrorClearBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7ErrorClearBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7ErrorCountBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7RestartBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7RestartBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7SwapOddEvenBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugPrbs7SwapOddEvenBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugQSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugQSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugStartCaptureBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugStartCaptureBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugStartErrorCheckBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugStartErrorCheckBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugStartRampBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugStartRampBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDebugStartShiftBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDebugStartShiftBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int16_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxEarlyStrobeDetectedFlagBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxEnableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxEnableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFifoClearBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxFifoClearBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFifoClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxFifoClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFifoEmptyBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFifoFullBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFifoReadResetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxFifoReadResetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFifoWriteResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxFifoWriteResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxForceMcsBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxForceMcsBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFullRefclkClkGatingEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxFullRefclkClkGatingEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFullRefclkDutyCycleHighLongerBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxFullRefclkDutyCycleHighLongerBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxFullRefclkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxFullRefclkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxIBitInversionBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxIBitInversionBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxLoadDividerBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxLoadDividerBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxLongStrobeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxLongStrobeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxLsbFirstBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxLsbFirstBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxQBitInversionBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxQBitInversionBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxQFirstBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxQFirstBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxRdFifoEnableDelayBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxRdFifoEnableDelayBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxRefclkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxRefclkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxStrobeAlignErrorBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxLssiTxStrobeBitInversionBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapTxLssiTxStrobeBitInversionBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAprEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionAprEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAveragePeakRatioBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAveragePowerBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAvgpowerEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionAvgpowerEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAvgpowerErrorBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAvgpowerErrorClearBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionAvgpowerErrorClearBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAvgpowerErrorClearRequiredBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionAvgpowerErrorClearRequiredBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAvgThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionAvgThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionAvrgDurBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionAvrgDurBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionErrorPowerBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionGainRampDownEnBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionGainRampDownEnBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionGainRampUpEnBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionGainRampUpEnBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionGainStepDownEnBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionGainStepDownEnBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionInputSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionInputSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionPeakpowerEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionPeakpowerEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionPeakpowerErrorBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionPeakpowerErrorClearBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionPeakpowerErrorClearBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionPeakpowerErrorClearRequiredBfSet(adi_adrv9001_Device_t *device, 
                                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionPeakpowerErrorClearRequiredBfGet(adi_adrv9001_Device_t *device, 
                                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionPeakCountBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionPeakCountBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionPeakDurBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionPeakDurBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionPeakPowerBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionPeakThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionPeakThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionRampDownInProgressBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionRampMaxAttenuationBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionRampMaxAttenuationBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionRampStepDurationBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionRampStepDurationBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionRampStepSizeBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionRampStepSizeBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionRampUpInProgressBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPaProtectionTxAttenStepBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPaProtectionTxAttenStepBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPdbias2bit04bit1BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPdbias2bit04bit1BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPdbias4bitCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPdbias4bitCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPdIDistBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPdIDistBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPllProtClrBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPllProtClrBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPllProtClrReqdBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPllProtClrReqdBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPllProtEventBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxPowerThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxPowerThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxReserved0BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxReserved0BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxReserved1BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxReserved1BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxReserved2BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxReserved2BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxReserved3BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxReserved3BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxRxToTxDatapathLoopbackBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxRxToTxDatapathLoopbackBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare0BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare0BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare1BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare1BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare10BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare10BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare11BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare11BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare12BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare12BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare13BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare13BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare14BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare14BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare15BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare15BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare16BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare16BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare17BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare17BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare18BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare18BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare19BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare19BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare2BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare2BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare3BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare3BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare4BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare4BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare5BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare5BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare6BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare6BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare7BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare7BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare8BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare8BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSoftwareSpare9BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSoftwareSpare9BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSrlEnBfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSrlEnBfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSrlIrqBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSrlIrqEnBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSrlIrqEnBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSrlSlewOffsetBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSrlSlewOffsetBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSrlStatBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxSrlStatEnBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSrlStatEnBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSrlStatModeBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSrlStatModeBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSrlTableSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSrlTableSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSwStatus0BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSwStatus0BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSwStatus1BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSwStatus1BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSysArm0EnableBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSysArm0EnableBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxSysArm1EnableBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxSysArm1EnableBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTddRampMaxAttenIndexBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTddRampMaxAttenIndexBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTddRampStepSizeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTddRampStepSizeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTddRampWaitDurationBwStepsBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTddRampWaitDurationBwStepsBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTgCaptureIq16bitSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTgCaptureIq16bitSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTgCaptureISelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTgCaptureISelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTgCaptureSelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTgCaptureSelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTgDriveIZeroSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTgDriveIZeroSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTgDriveQZeroSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTgDriveQZeroSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTgDriveSelBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTgDriveSelBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxThresholdBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxThresholdBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                           uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxThresholdShiftBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxThresholdShiftBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTpnxxSelBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTpnxxSelBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxattenDpathUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxattenDpathUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxchestPnadderAErrBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxchestPnadderASelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxchestPnadderASelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxchestPnadderCErrBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxchestPnadderCSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxchestPnadderCSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxqecLolDataCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxqecLolDataCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfC1QecTrimBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfC1QecTrimBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfC2QecTrimBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfC2QecTrimBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfForceClockEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfForceClockEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalAiFcBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalAiFcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalAiQBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalAiQBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalAqBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalAqBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalC1BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalC1BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalC2CutoffBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalC2CutoffBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalCalEnQCalBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalCalEnQCalBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalCalMultpassBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalCalMultpassBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalFiltertimeBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalFiltertimeBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalFFcBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalFFcBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalFLowBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalFLowBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalFQBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalFQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalGentleBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalGentleBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalNcoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalNcoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalNcoIBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalNcoIBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalNcoQBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalNcoQBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   int32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetForcePowerupBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetForcePowerupBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetLatchBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetLatchBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetOutBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetResetBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalPkdetResetBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalResettimeBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalResettimeBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalSettletimeBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalSettletimeBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalTrimqBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfFreqcalTrimqBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAbbfNco2UseIOutBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAbbfNco2UseIOutBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup10ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup10ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup11ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup11ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup1ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup1ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup2ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup2ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup2ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup2ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup3ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup3ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup3ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup3ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup4ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup4ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup4ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup4ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup5ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup5ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup5ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup5ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup6ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup6ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup6ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup6ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup7ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup7ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup7ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup7ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup8ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup8ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup8ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup8ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup9ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup9ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgGroup9ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgGroup9ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux401bSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux401bSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux401cSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux401cSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux401SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux401SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux402aNbdpdi0SelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux402aNbdpdi0SelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux402aTb2SelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux402aTb2SelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux402bTaSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux402bTaSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux402bTcSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux402bTcSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux402cPnAdderCSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux402cPnAdderCSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux402DpdActSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux402DpdActSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux402From401SelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux402From401SelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux403bMagCompfir13SelBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux403bMagCompfir13SelBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux403From402cSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux403From402cSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux403TxlolQecSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux403TxlolQecSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux404Tb4SelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux404Tb4SelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux405aSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux405aSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux406Nco2SelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux406Nco2SelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux407aFifoASelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux407aFifoASelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux407aFifoBSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux407aFifoBSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux407bFifoASelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux407bFifoASelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux407bFifoBSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux407bFifoBSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux407cFifoBSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux407cFifoBSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux407SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux407SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux408aNcoASelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux408aNcoASelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux408aNcoBSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux408aNcoBSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux408bNcoASelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux408bNcoASelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux408bNcoBSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux408bNcoBSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux408cNcoASelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux408cNcoASelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux409aSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux409aSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux409bSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux409bSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux410bTgSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux410bTgSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux410SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux410SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux411SrlSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux411SrlSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux412SrlSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux412SrlSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux413SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux413SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux414SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux414SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux415From410bSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux415From410bSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux415SrlSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux415SrlSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux416aSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux416aSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux416bSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux416bSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux417SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux417SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgMux418SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgMux418SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgObsTwinFirAClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgObsTwinFirAClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgObsTwinFirBClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgObsTwinFirBClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgRxmux301ObsSourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgRxmux301ObsSourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgRxmux302ObsSourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgRxmux302ObsSourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgTwinFirAClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgTwinFirAClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAlgTwinFirBClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAlgTwinFirBClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenuationBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenuationBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenuationBBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenuationBBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenuationEarlyDelayCounterForAnalogBfSet(adi_adrv9001_Device_t *device, 
                                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenuationEarlyDelayCounterForAnalogBfGet(adi_adrv9001_Device_t *device, 
                                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenuationLateDelayCounterForAnalogBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenuationLateDelayCounterForAnalogBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenuationReadBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenuationReadBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenuationTableGainFifoResetBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenuationTableGainFifoResetBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenuationTableResetBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenuationTableResetBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenConfigBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenConfigBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenDigitalReadBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenEccErrAddrBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenEccErrClrBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenEccErrClrBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenEccErrFlagBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenEccErrIrqSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenEccErrIrqSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenForceAnalogDelaysBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenForceAnalogDelaysBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenForceAnalogDelaysOffNotOnBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenForceAnalogDelaysOffNotOnBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenForceDigitalBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenForceDigitalBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenForceHpBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenForceHpBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenHpLatenOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenHpLatenOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenHpReadBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenHpRefBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenHpRefBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenModeBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenModeBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenOffsetBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenOffsetBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenQeclolUpdEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenQeclolUpdEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenTableEccDisableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenTableEccDisableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenTableRf1PllLockMaskBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenTableRf1PllLockMaskBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxAttenTableRf2PllLockMaskBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxAttenTableRf2PllLockMaskBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxBbfTunerCalStartBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxBbfTunerCalStartBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxCalMuxSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxCalMuxSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxClkgenCssiDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxClkgenCssiDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxClkgenCssiLoadDividerBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxClkgenCssiLoadDividerBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxCssiEnBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxCssiEnBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDacOutIZeroSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDacOutIZeroSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDacOutQZeroSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDacOutQZeroSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDatapathIInputZeroSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDatapathIInputZeroSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDatapathIOnlyEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDatapathIOnlyEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDatapathQInputZeroSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDatapathQInputZeroSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDataRampDownOnPllUnlockBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDataRampDownOnPllUnlockBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdDataCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdDataCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam0ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam0ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam1ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam1ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam2ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam2ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam3ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam3ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam4ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam4ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam5ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam5ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam6ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam6ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam7ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam7ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdIslandRam8ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdIslandRam8ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpdRamClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpdRamClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpAttenSelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpAttenSelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpFreqDevMapper47EnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpFreqDevMapper47EnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt212EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt212EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt214EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt214EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt216EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt216EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt218EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt218EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt220EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt220EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt2221EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt2221EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt222EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt222EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt224EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt224EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt226EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt226EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt228EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt228EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt230EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt230EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt236EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt236EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt238EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt238EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt28EnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt28EnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt310EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt310EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt340EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt340EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpInt3441EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpInt3441EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpModemModeBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpModemModeBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpModemModeDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpModemModeDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpNbdpd0EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpNbdpd0EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpNbdpd1EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpNbdpd1EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpNbdpd2EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpNbdpd2EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpNcoAttenPhaseOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpNcoAttenPhaseOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpSinc242EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpSinc242EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpSinc344EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpSinc344EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpSpectralInvertIBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpSpectralInvertIBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpSpectralInvertQBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpSpectralInvertQBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpSpiIntfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpSpiIntfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTpfirI3ResetBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTpfirI3ResetBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTpfirQ5ResetBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTpfirQ5ResetBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTscic32DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTscic32DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTscic32EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTscic32EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTscic32ResetBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTscic32ResetBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTscic32RippleSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTscic32RippleSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTxqec34ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTxqec34ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTxqec34EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTxqec34EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTxPreproc2DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTxPreproc2DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTxPreproc2EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTxPreproc2EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpTxRound46EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpTxRound46EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpWbdpd0EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpWbdpd0EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpWbdpd1EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpWbdpd1EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDpWbdpd2EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDpWbdpd2EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDucEnBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDucEnBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDucFtwBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDucFtwBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDucFtwOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDucFtwOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxDucFtwUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxDucFtwUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxFreqDevMapperMpllBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxFreqDevMapperMpllBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxFreqDevMapperNchBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint64_t bfValue);
int32_t adrv9001_NvsRegmapTxTxFreqDevMapperNchBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint64_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxFreqDevMapperRrc2fracdevfacBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxFreqDevMapperRrc2fracdevfacBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxFreqDevMapperThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxFreqDevMapperThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxFuseTxSlrEnableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxFuseTxSlrEnableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxGainChangeFifoOverflowBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxGainChangeFifoOverflowBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxGdcalPreSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxGdcalPreSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxIncrDecrWordBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxIncrDecrWordBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxIqdmEnBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxIqdmEnBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxIqdmFreqDevBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxIqdmFreqDevBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxIqdmFreqOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxIqdmFreqOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxIqdmNcoMultiChipSyncedBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxIqdmNcoMultiChipSyncedBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxIqdmScalerBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxIqdmScalerBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxIqdmThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxIqdmThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLolResetBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLolResetBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLolSelBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLolSelBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLoDelayCoarseCorrIBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLoDelayCoarseCorrIBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLoDelayCoarseCorrIBBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLoDelayCoarseCorrIBBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLoDelayCoarseCorrQBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLoDelayCoarseCorrQBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLoDelayCoarseCorrQBBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLoDelayCoarseCorrQBBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLoDelayCorrDacBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLoDelayCorrDacBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLoDelayFineCorrDacBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLoDelayFineCorrDacBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLoDelayFineCorrDacBBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLoDelayFineCorrDacBBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLoDelayLatchEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLoDelayLatchEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxLssiEnBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxLssiEnBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxNcoFtwBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxNcoFtwBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                          uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxNcoFtwUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxNcoFtwUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxNcoMcsAdjAmpBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxNcoMcsAdjAmpBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxNcoMcsAdjAmp1BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxNcoMcsAdjAmp1BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxNcoMcsAdjAmp2BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxNcoMcsAdjAmp2BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxOnBfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxOnBfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPaProtectionErrorClearBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPaProtectionErrorClearBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPfir4xDiv4p5BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPfir4xDiv4p5BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPfir4xDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPfir4xDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPfirI3DontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPfirI3DontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPfirIEnableBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPfirIEnableBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPfirQ5DontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPfirQ5DontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPfirQEnableBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPfirQEnableBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPreprocMuxSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPreprocMuxSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPreprocOutSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPreprocOutSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPreprocSym0BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPreprocSym0BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPreprocSym1BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPreprocSym1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPreprocSym2BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPreprocSym2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPreprocSym3BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPreprocSym3BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxProfileDpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxProfileDpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxProfileSsiGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxProfileSsiGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxPulseShapingModeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxPulseShapingModeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxSsiFreeRunningClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxSsiFreeRunningClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxSsiGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxSsiGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxSsiIInvertBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxSsiIInvertBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxSsiISwapBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxSsiISwapBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxSsiQInvertBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxSsiQInvertBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxSsiQSwapBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxSsiQSwapBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxattenMultiplierIBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxattenMultiplierIBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxattenMultiplierQBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxattenMultiplierQBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxattenMultiplierSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxattenMultiplierSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestBypassPnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestBypassPnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestEnableABfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestEnableABfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestEnableBBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestEnableBBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestPnLengthBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestPnLengthBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestPnRepeatBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestPnRepeatBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestPnTapsIBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestPnTapsIBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestPosValIBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestPosValIBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestPosValQBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestPosValQBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestProgDlyBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestProgDlyBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxchestSelPnNot0BfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxchestSelPnNot0BfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxdigQecErrorBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorBypassScalerBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorBypassScalerBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorErrorBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetIBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetIBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetIBBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetIBBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetQBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetQBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetQBBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetQBBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorFilterDcOffsetUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorOldSettingSelBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorOldSettingSelBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorScalerBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorScalerBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorScalerBBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorScalerBBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxlolActuatorScalerUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxlolActuatorScalerUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecControlBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecControlBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                int32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterChPerterbUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterChPerterbUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterChScalerUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterChScalerUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI0BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI0BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI0BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI0BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI1BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI1BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI1BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI1BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI2BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI2BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI2BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI2BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI3BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI3BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI3BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI3BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI4BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI4BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI4BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffI4BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ0BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ0BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ0BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ0BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ1BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ1BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ1BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ1BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ2BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ2BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ2BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ2BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ3BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ3BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ3BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ3BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ4BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ4BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ4BBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffQ4BBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterCoeffUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterDcOffsetUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterDcOffsetUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterOffsetIBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterOffsetIBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterOffsetIBBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterOffsetIBBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterOffsetQBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterOffsetQBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterOffsetQBBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterOffsetQBBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterPerterbIBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterPerterbIBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterPerterbIBBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterPerterbIBBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterPerterbQBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterPerterbQBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterPerterbQBBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterPerterbQBBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterScalerBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterScalerBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                     int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecFilterScalerBBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecFilterScalerBBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                                      int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecIaddrBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int16_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecIaddrBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int16_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecRdataBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecReadBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecReadBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecResetBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecResetBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecWdataBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int32_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecWdataBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int32_t *bfValue);


int32_t adrv9001_NvsRegmapTxTxTxqecWriteBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapTxTxTxqecWriteBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxChanAddr_e baseAddr, 
                                              int8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_NVS_REGMAP_TX_H__

/*  EOF: adrv9001_bf_nvs_regmap_tx.h */

