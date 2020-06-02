/**
 * \file adrv9001_bf_nvs_regmap_rxb.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __ADRV9001_BF_NVS_REGMAP_RXB_H__
#define __ADRV9001_BF_NVS_REGMAP_RXB_H__

#include "adrv9001_bf_nvs_regmap_rxb_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_NvsRegmapRxbAdcControlBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAdcValidOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAdcValidOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAdcovrg2ndHighCounterBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAdcovrg2ndHighCounterBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcAdcHighOvrgExceededCounterBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcAdcHighOvrgExceededCounterBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcAdcLowOvrgExceededCounterBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcAdcLowOvrgExceededCounterBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcAdcovrgIp3HighCounterBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcAdcovrgIp3HighCounterBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcAdcovrgLowInt0CounterBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcAdcovrgLowInt0CounterBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcAdcovrgLowInt1CounterBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcAdcovrgLowInt1CounterBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcApdLowFreqErrorMitigationModeBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcApdLowFreqErrorMitigationModeBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcAttackDelayBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcAttackDelayBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcAutoGainLockDelayBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcAutoGainLockDelayBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcCurrentModeBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcCurrentModeBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcDecimateBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcDecimateBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcForceRoundPathBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcForceRoundPathBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcForceRoundPathSelectBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcForceRoundPathSelectBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcLoadAccBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcLoadAccBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcLoadAccIBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcLoadAccIBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcLoadAccQBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcLoadAccQBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcMShiftBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcMShiftBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcManualPauseBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcManualPauseBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcManualRestoreAccBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcManualRestoreAccBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcMaskOutputBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcMaskOutputBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcMultiplierBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcMultiplierBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcMultiplierEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcMultiplierEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcPauseBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcPauseBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcResetAtGainchangeBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcResetAtGainchangeBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcTrackInputIBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcTrackInputIBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcTrackInputQBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcTrackInputQBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcTrackingEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcTrackingEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcTrackingISpiBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcTrackingQSpiBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcUpdateSpiReadbackBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcUpdateSpiReadbackBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcBbdcUpdateTrackingWordBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcBbdcUpdateTrackingWordBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcChangeGainIfAdcovrgHighBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcChangeGainIfAdcovrgHighBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcChangeGainIfUlbthHighBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcChangeGainIfUlbthHighBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDecrGainStepSizeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDecrGainStepSizeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDelayCounterBaseRateBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDelayCounterBaseRateBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDigSatBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDigSaturationExceededCounterBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDigSaturationExceededCounterBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDigsatGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDigsatGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandExtTableLowerIndexBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandExtTableLowerIndexBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandExtTableUpperIndexBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandExtTableUpperIndexBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandHighLnaThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandHighLnaThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandIndexBandABfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandIndexBandABfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandIndexOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandIndexOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandLnaStepBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandLnaStepBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandLowLnaThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandLowLnaThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandMaxIndexBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandMaxIndexBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcDualbandPwrMarginBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcDualbandPwrMarginBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableAutoGainLockBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableAutoGainLockBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableDigsatOvrgBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableDigsatOvrgBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableFastRecoveryLoopBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableFastRecoveryLoopBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableGainChangeWithPinControlBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableGainChangeWithPinControlBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableGainFreezeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableGainFreezeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableGainIndexUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableGainIndexUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableIp3OptimizationThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableIp3OptimizationThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableResetToMaxgainBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableResetToMaxgainBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableSetGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableSetGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableSetGainWGainFreezeExitBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableSetGainWGainFreezeExitBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnableSyncPulseForGainCounterBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnableSyncPulseForGainCounterBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnergyDetectCountBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnergyDetectCountBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnergyLostFreezeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnergyLostFreezeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcEnergyLostThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcEnergyLostThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcFastAttackSettlingDelayBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcFastAttackSettlingDelayBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcFastAttackStateBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcFinalOverrangeCountBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcFinalOverrangeCountBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcForceFreezeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcForceFreezeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcForcePdResetBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcForcePdResetBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcFullGainFinalSettlingStepBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcFullGainFinalSettlingStepBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcGainChangeSyncUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcGainChangeSyncUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcGainIndexBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcGainLockBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcGainTableStepSizeBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcGainTableStepSizeBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcGainUpdateCounterBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcGainUpdateCounterBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcGainlockExitCountBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcGainlockExitCountBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcHaltGainUpdateCounterOnFreezeBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcHaltGainUpdateCounterOnFreezeBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcIncrGainStepSizeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcIncrGainStepSizeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLastGainIndexBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLlBlockerFreezesGainIncrBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLlBlockerFreezesGainIncrBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLlbGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLlbGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLlbThresholdExceededCounterBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLlbThresholdExceededCounterBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLockLevelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLockLevelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLowThsPreventGainIncBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLowThsPreventGainIncBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLower0ThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLower0ThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLower0ThresholdExceededGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLower0ThresholdExceededGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLower1ThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLower1ThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcLower1ThresholdExceededGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcLower1ThresholdExceededGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexBBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexBBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexElbBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexElbBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexElbBBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexElbBBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexOrxBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexOrxBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexOrxBBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcManualGainIndexOrxBBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcManualGainPinControlBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcManualGainPinControlBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcMultClkRateCompBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcMultClkRateCompBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcMultitoneIp3IndexBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcMultitoneIp3IndexBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcNoUnrgGainIncAtMaxIp3BfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcNoUnrgGainIncAtMaxIp3BfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOptMaxGainWhenEnlostBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOptMaxGainWhenEnlostBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOptimMaxGainBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOptimMaxGainBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOptimalIp3IndexBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOptimalIp3IndexBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOptimizeMaxGainThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOptimizeMaxGainThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOvrgFreezeBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOvrgFreezeBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOvrgHighGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOvrgHighGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOvrgLowGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOvrgLowGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOvrgLowInt0GainStepBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOvrgLowInt0GainStepBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOvrgLowInt1GainStepBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOvrgLowInt1GainStepBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcOvrgResetpdHighCountBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcOvrgResetpdHighCountBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcPeakThresholdGainControlModeBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcPeakThresholdGainControlModeBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcPeakWaitTimeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcPeakWaitTimeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcResetCountersBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcResetCountersBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcResetOnRxonBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcResetOnRxonBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcResetOnRxonGainIndexBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcResetOnRxonGainIndexBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcSetupBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcSetupBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcSlowLoopSettlingDelayBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcSlowLoopSettlingDelayBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcSlowLoopStateBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcSlowloopFreezeEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcSlowloopFreezeEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcSoftResetBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcSoftResetBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcStrongSignalFreezeBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcStrongSignalFreezeBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcStrongerSignalThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcStrongerSignalThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDelayBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDelayBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDurationBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                            uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDurationBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                            uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUlbGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUlbGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUlbThresholdExceededCounterBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUlbThresholdExceededCounterBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUpdateSetgainLockLevelDuringBurstBfSet(adi_adrv9001_Device_t *device, 
                                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUpdateSetgainLockLevelDuringBurstBfGet(adi_adrv9001_Device_t *device, 
                                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUpper0ThresholdExceededGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUpper0ThresholdExceededGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUpper1ThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUpper1ThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUpper1ThresholdExceededGainStepBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUpper1ThresholdExceededGainStepBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUpperLimitOnGainIncreaseForLockLevelBfSet(adi_adrv9001_Device_t *device, 
                                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUpperLimitOnGainIncreaseForLockLevelBfGet(adi_adrv9001_Device_t *device, 
                                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUrangeInterval0BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUrangeInterval0BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUrangeInterval1MultBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUrangeInterval1MultBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUrangeInterval2MultBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUrangeInterval2MultBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbAgcUseCountersForMgcBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbAgcUseCountersForMgcBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbApdHighSrcSelectBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbApdHighSrcSelectBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbApdLowSrcSelectBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbApdLowSrcSelectBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBandASlicerPositionBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcDecimateBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcDecimateBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcForceRoundPathBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcForceRoundPathBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcForceRoundPathSelectBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcForceRoundPathSelectBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoadAccBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoadAccBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoadAccIBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoadAccIBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoadAccQBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoadAccQBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoopbackDecimateBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoopbackDecimateBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoopbackMShiftBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoopbackMShiftBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoopbackMultiplierBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoopbackMultiplierBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoopbackMultiplierEnableBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoopbackMultiplierEnableBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoopbackPauseBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoopbackPauseBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcLoopbackTrackingEnableBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcLoopbackTrackingEnableBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcMShiftBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcMShiftBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcManualPauseBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcManualPauseBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcManualRestoreAccBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcManualRestoreAccBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcMaskOutputBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcMaskOutputBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcMultiplierBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcMultiplierBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcMultiplierEnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcMultiplierEnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcOrxDecimateBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcOrxDecimateBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcOrxMShiftBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcOrxMShiftBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcOrxMultiplierBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcOrxMultiplierBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcOrxMultiplierEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcOrxMultiplierEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcOrxPauseBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcOrxPauseBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcOrxTrackingEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcOrxTrackingEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcPauseBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcPauseBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcTrackingEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcTrackingEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcTrackingResetAccOnGainChangeBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcTrackingResetAccOnGainChangeBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbBbdcUpdateTrackingWordBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbBbdcUpdateTrackingWordBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbCurrentGainIndexBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgDpOutShiftBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgDpOutShiftBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgModeBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgModeBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgNsbOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgNsbOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgNsbsBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgNsbsBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgNsbsDlyBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgNsbsDlyBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmTlAvgExpBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmTlAvgExpBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmTlAvgManBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmTlAvgManBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmTuAvgExpBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmTuAvgExpBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmTuAvgManBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmTuAvgManBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmAcc0NumBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmAcc0NumBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmAcc0NumBBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmAcc0NumBBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmAcc0NumRecipBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmAcc0NumRecipBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmAcc0NumRecipBBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmAcc0NumRecipBBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmAccClearBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmAccClearBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmClearModeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmClearModeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmCnt0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmDselBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmDselBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmModeBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmModeBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmPowExp0BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmPowMan0BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmReadStrobeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgPmReadStrobeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgPmStatusOutBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgShiftBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDcsgShiftBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDcsgSlicerDpOutShiftBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerAddDigGainBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerAddDigGainBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerEnSpiOrAgcSelectBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerEnSpiOrAgcSelectBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerEnableMeasBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerEnableMeasBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerInputSelectBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerInputSelectBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerLatchLinearPowerBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerLatchLinearPowerBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerLinearPowerBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerLogShiftBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerLogShiftBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerMeasurementDurationBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerMeasurementDurationBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerState5MeasurementDurationBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerState5MeasurementDurationBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecPowerWaitDelayBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecPowerWaitDelayBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDecimatedDataOverloadSecondaryUpperThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDecimatedDataOverloadSecondaryUpperThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformFpEnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformFpEnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformFpExponentBitsBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformFpExponentBitsBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformFpFloatDataFormatBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformFpFloatDataFormatBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformFpHideLeadingOnesBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformFpHideLeadingOnesBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformFpIntDataAttenBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformFpIntDataAttenBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformFpNanEncEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformFpNanEncEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformFpRoundModeBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformFpRoundModeBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformIntDataFormatBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformIntDataFormatBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformIntDataResolutionBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformIntDataResolutionBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformIntEmbedSlicerBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformIntEmbedSlicerBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformIntEmbedSlicerNumberBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformIntEmbedSlicerNumberBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformIntEmbedSlicerPosBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformIntEmbedSlicerPosBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformIntEvenParityBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformIntEvenParityBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformIntParitySupportBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformIntParitySupportBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformInvertRxIBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformInvertRxIBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformInvertRxQBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformInvertRxQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformMaskRxOutputBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformMaskRxOutputBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformRxMonFormatI0BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformRxMonFormatI0BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformRxMonFormatI1BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformRxMonFormatI1BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformRxMonFormatI2BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformRxMonFormatI2BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformRxMonFormatQ0BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformRxMonFormatQ0BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformRxMonFormatQ1BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformRxMonFormatQ1BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDformRxMonFormatQ2BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDformRxMonFormatQ2BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDigitalDcOffsetTrackingCorrectionWordIBfGet(adi_adrv9001_Device_t *device, 
                                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                         int32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDigitalDcOffsetTrackingCorrectionWordQBfGet(adi_adrv9001_Device_t *device, 
                                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                         int32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDigitalGainBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDigitalPhaseDelayBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbDigitalPhaseDelayBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDualbandControlBandABfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbDualbandGainBandABfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbEnableExtLnaSyncModeBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbEnableExtLnaSyncModeBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbExtControlBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbExtLnaSettlingDelayBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbExtLnaSettlingDelayBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbFeGainDelayDecrBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int16_t bfValue);
int32_t adrv9001_NvsRegmapRxbFeGainDelayDecrBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbFeGainDelayIncrBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int16_t bfValue);
int32_t adrv9001_NvsRegmapRxbFeGainDelayIncrBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbForceAdcControlBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbForceAdcControlBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbForceDigitalGainBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbForceDigitalGainBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbForceDualbandControlAndGainBandABfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbForceDualbandControlAndGainBandABfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbForceExtControlBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbForceExtControlBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbForcePhaseOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbForcePhaseOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbForceRxFeGainBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbForceRxFeGainBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbForceTiaControlBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbForceTiaControlBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbFreqHopForceSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbFreqHopForceSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGainChangePulseDurationBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGainChangePulseDurationBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGainFifoMinWrIntvlBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbGainFifoMinWrIntvlBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalAnaGain1BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalAnaGain1BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalAnaGain2BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalAnaGain2BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalDigGain1BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalDigGain1BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalDigGain2BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalDigGain2BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalFlushPeriodBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalFlushPeriodBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalInSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalInSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalPowerThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalPowerThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalStartRxCalBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalStartRxCalBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalStartTxCalBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalStartTxCalBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalStateReadbackBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalThresholdShiftBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalThresholdShiftBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalTxAnaDigDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalTxStateBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGdcalWindowSizeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGdcalWindowSizeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtAdcControlBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtAdcControlBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtDigitalGainBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int16_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtDigitalGainBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtDualbandABfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtDualbandABfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtDualbandControlBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtDualbandControlBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtDualbandControlBandABfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtDualbandControlBandABfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtDualbandGainBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtDualbandGainBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtDualbandGainBandABfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtDualbandGainBandABfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtExtControlBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtExtControlBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtPhaseOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtPhaseOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtRxFeGainBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtRxFeGainBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtTiaControlBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtTiaControlBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbGtUncorrectableParityErrorBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbGtUncorrectableParityErrorBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbHb2HighSrcSelectBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbHb2HighSrcSelectBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbHb2LowSrcSelectBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbHb2LowSrcSelectBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbInvertApdLowBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbInvertApdLowBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbPcaGainControlModeBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbPcaGainControlModeBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbPcaGainIndexOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbPcaGainIndexOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbPcaGainStepBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbPcaGainStepBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbPcaUpdateManualGainModeBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbPcaUpdateManualGainModeBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbPhaseOffsetBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbReadGainTableBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbReadGainTableBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRegLoadShadowBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRegLoadShadowBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbResetOnRxonGainIndexBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbResetOnRxonGainIndexBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcAdcRstMaskBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcAdcRstMaskBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcAdcovrldmaskBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcAdcovrldmaskBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcCalStartBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcCalStartBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcCorrDoutIBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcCorrDoutQBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcCtrlStateBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcDcestDoutIBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcDcestDoutQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcDecSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcDecSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcEstBitselectBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcEstBitselectBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcEstBitselectLoidBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcEstBitselectLoidBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcEstDoneBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcFegainSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcFegainSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcFfhModeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcFfhModeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcForceAhbABfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcForceAhbABfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcForceAhbBBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcForceAhbBBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcForceFsBitselBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcForceFsBitselBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcForceIdacBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcForceIdacBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcForgetFacBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcForgetFacBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcFreezeBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcFreezeBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcFsBitselBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcFsBitselBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcFsBitselBBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcFsBitselBBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcGainctrlBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcGainindexBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIcalBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcIcalBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIcalBypassBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcIcalBypassBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIdacIBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIdacIForcedBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcIdacIForcedBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIdacIForcedBBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcIdacIForcedBBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIdacQBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIdacQForcedBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcIdacQForcedBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIdacQForcedBBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcIdacQForcedBBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIndexOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcIndexOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcIndexSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcIndexSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcInitDoneBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidEnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLoidEnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidFifoStatusBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidIntvlBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLoidIntvlBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidLog0BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidLog1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidLog2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidLog3BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidLthldBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLoidLthldBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLoidUthldBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLoidUthldBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLpbckCorrIaBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLpbckCorrIaBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLpbckCorrIbBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLpbckCorrIbBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLpbckCorrQaBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLpbckCorrQaBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLpbckCorrQbBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLpbckCorrQbBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcLthldBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcLthldBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcMaxGainindxBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcMaxGainindxBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcMemrstBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcMemrstBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcMinGainindxBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcMinGainindxBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcMthldBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcMthldBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcNthldBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcNthldBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcOpmodeBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcOpmodeBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcOrxCorrIaBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcOrxCorrIaBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcOrxCorrIbBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcOrxCorrIbBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcOrxCorrQaBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcOrxCorrQaBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcOrxCorrQbBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcOrxCorrQbBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcPauseResumeModeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcPauseResumeModeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcSoftRstBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcSoftRstBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcSpiAccessSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcSpiAccessSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcSpiAccessSelBBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcSpiAccessSelBBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcSpiAddrBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcSpiAddrBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcSpiDataInBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint64_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcSpiDataInBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint64_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcSpiDataOutBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint64_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcSpiRdBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcSpiRdBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcSpiWrBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcSpiWrBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcStartBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcStartBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcUpdateModeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcUpdateModeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcUthldBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcUthldBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcViScalingBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcViScalingBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcWthldBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcWthldBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRfdcoLoidBypassBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRfdcoLoidBypassBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiDefaultModeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiDefaultModeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiDelayBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            int16_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiDelayBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiEnableMeasBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiEnableMeasBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiInputSelectBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiInputSelectBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiLogshiftBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiLogshiftBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiMeasurementDuration0BfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiMeasurementDuration0BfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiMeasurementDuration1BfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiMeasurementDuration1BfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiMeasurementDuration2BfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiMeasurementDuration2BfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiMeasurementDuration3BfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiMeasurementDuration3BfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiModeSelectBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiModeSelectBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiOffsetBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiOffsetBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiPreambleBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiSymbolBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiWaitTimeBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int16_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiWaitTimeBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiWeightedMultiplier0BfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiWeightedMultiplier0BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiWeightedMultiplier1BfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiWeightedMultiplier1BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiWeightedMultiplier2BfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiWeightedMultiplier2BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRssiWeightedMultiplier3BfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRssiWeightedMultiplier3BfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxBandAEnableBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRxBandAEnableBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxFeGainBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxGdcalAnaDigDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxNbwbSelBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRxNbwbSelBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxTiaProfileBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRxTiaProfileBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxTiaProfileValBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRxTiaProfileValBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxfeValidDelayBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRxfeValidDelayBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxfeValidOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRxfeValidOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxfuncGainCompInSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRxfuncGainCompInSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbRxfuncSlicerInSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbRxfuncSlicerInSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSlicerPositionOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSlicerPositionOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare0BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare0BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare1BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare1BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare10BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare10BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare11BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare11BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare12BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare12BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare13BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare13BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare14BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare14BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare15BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare15BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare16BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare16BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare17BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare17BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare18BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare18BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare19BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare19BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare2BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare2BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare3BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare3BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare4BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare4BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare5BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare5BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare6BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare6BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare7BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare7BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare8BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare8BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSoftwareSpare9BfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSoftwareSpare9BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbSpiRfdcLnapFreezeMaskBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbSpiRfdcLnapFreezeMaskBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbTiaControlBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbTiaValidOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxbTiaValidOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxbUseGainIndexFromAnotherChannelBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxbUseGainIndexFromAnotherChannelBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_NVS_REGMAP_RXB_H__

/*  EOF: adrv9001_bf_nvs_regmap_rxb.h */

