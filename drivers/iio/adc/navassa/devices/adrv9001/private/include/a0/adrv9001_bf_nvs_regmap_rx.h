/**
 * \file adrv9001_bf_nvs_regmap_rx.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __ADRV9001_BF_NVS_REGMAP_RX_H__
#define __ADRV9001_BF_NVS_REGMAP_RX_H__

#include "adrv9001_bf_nvs_regmap_rx_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_NvsRegmapRxRcDriveIZeroSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRcDriveIZeroSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRcDriveQZeroSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRcDriveQZeroSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRcDriveSelBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRcDriveSelBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRkCaptureISelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRkCaptureISelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRkCaptureIq16bitSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRkCaptureIq16bitSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAdcClockEnableBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxAdcClockEnableBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAdcRxSampleDelayBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAdcRxSampleDelayBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAgcEnableBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAgcEnableBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgNbdemDpinfifoDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgNbdemDpinfifoDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgNbdemDpinfifoSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgNbdemDpinfifoSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgNbdemDpDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgNbdemDpDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgNbdemDpSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgNbdemDpSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgNbdemRxpfir52DontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgNbdemRxpfir52DontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgNbdemRxpfir52SwResetBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgNbdemRxpfir52SwResetBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgNbficCorrClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgNbficCorrClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgNbficDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgNbficDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgRxChFuncsDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgRxChFuncsDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAlgSlicerRssiDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAlgSlicerRssiDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAllRxEnableBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAllRxEnableBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxAnalogSpareBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxAnalogSpareBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxBbdcEnableBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxBbdcEnableBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxClkgenOutputBufferEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxClkgenOutputBufferEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxControlOutMuxSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxControlOutMuxSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCorrCtrlBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxCorrCtrlBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDataFormatBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDataFormatBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDataFormatFourLanesBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDataFormatFourLanesBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDdrClkInvBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDdrClkInvBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDdrPosClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDdrPosClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDdrSelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDdrSelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDebugInjectErrorBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDebugInjectErrorBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDebugLoadBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDebugLoadBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDebugLoadValueBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDebugLoadValueBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDebugModeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDebugModeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDebugNibbleSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDebugNibbleSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDebugQSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDebugQSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDebugStartRampBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDebugStartRampBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxDebugStartShiftBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxDebugStartShiftBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxEnableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxEnableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxInvertClkBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxInvertClkBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxLongStrobeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxLongStrobeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxLoopbackFromTxBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxLoopbackFromTxBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxLsbFirstBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxLsbFirstBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxCssiRxQFirstBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxCssiRxQFirstBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDataCaptureDriveEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDataCaptureDriveEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDdcLoSyncEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDdcLoSyncEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDigGainCompOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxDigGainCompOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDigitalGainDelayBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxDigitalGainDelayBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDigitalGainEnableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxDigitalGainEnableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDparmSelBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDparmSelBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoClearBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoClearBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoDetectedBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoDetectedBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoDiffabsConfigBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoDiffabsConfigBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoDiffabsUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoDiffabsUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoDiffbpConfigBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoDiffbpConfigBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoDiffbpUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoDiffbpUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoEnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoEnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoRdStartstopBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoRdStartstopBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoTestDataUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoTestDataUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoTestdataIBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoTestdataIBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoTestdataQBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoTestdataQBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoTestdataSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoTestdataSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxDpinfifoWrStartstopBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxDpinfifoWrStartstopBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxExtLnaDigitalGainDelayBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int16_t bfValue);
int32_t adrv9001_NvsRegmapRxExtLnaDigitalGainDelayBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxExtlpbkRxqecObsCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxExtlpbkRxqecObsCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxFdInSelBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxFdInSelBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxFdModeSelBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxFdModeSelBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxFdmax1BfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                        uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxFdmax1BfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                        uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxFdmax2BfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                        uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxFdmax2BfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                        uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxFicObsCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxFicObsCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxFlashClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxFlashClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxForceDigGainStrobeBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxForceDigGainStrobeBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxGainChangeRampStepDurationBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxGainChangeRampStepDurationBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxGainChangeRampStepsBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxGainChangeRampStepsBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxGainCompDualbandCommonBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxGainCompDualbandCommonBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxGainCompForExtGainBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxGainCompForExtGainBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxGainCompForTempGainBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxGainCompForTempGainBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxHd2ObsCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxHd2ObsCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxIntDelay0EnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxIntDelay0EnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxIntDelay1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxIntDelay1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxIntOrxEnableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxIntOrxEnableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxIntRxEnableBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxIntRxEnableBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxIntlpbkRxqecObsCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxIntlpbkRxqecObsCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxClkInversionBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxClkInversionBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDataFormat12bitBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDataFormat12bitBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDataOneLaneBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDataOneLaneBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugInjectErrorBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugInjectErrorBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugLoadBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugLoadBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugLoadValueBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int16_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugLoadValueBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugModeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugModeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs15EnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs15EnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs15RestartBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs15RestartBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs7EnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs7EnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs7RestartBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs7RestartBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs7SwapOddEvenBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugPrbs7SwapOddEvenBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugQSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugQSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugStartRampBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugStartRampBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDebugStartShiftBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDebugStartShiftBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                int16_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxEnableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxEnableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxFifoClearBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxFifoClearBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxFifoClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxFifoClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxFifoReadResetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxFifoReadResetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxFifoWriteResetBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxFifoWriteResetBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxForceMcsBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxForceMcsBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxIBitInversionBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxIBitInversionBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxLoadDividerBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxLoadDividerBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxLongStrobeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxLongStrobeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxLoopbackFromTxBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxLoopbackFromTxBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxLsbFirstBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxLsbFirstBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxQBitInversionBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxQBitInversionBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxQFirstBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxQFirstBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxRdFifoEnableDelayBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxRdFifoEnableDelayBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxStrobeBitInversionBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxStrobeBitInversionBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxLssiRxUseLsbIn12bitModeBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapRxLssiRxUseLsbIn12bitModeBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxMagp2LshiftBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxMagp2LshiftBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbdemCnt1ClearBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbdemCnt1ClearBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbdemCnt1ThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxNbdemCnt1ThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbdemCnt1ThresholdUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbdemCnt1ThresholdUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbdemCnt3EnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbdemCnt3EnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbdemFskdemModeBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbdemFskdemModeBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbfDecintpLRatioBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbfDecintpLRatioBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbfDecintpMRatioBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbfDecintpMRatioBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbfDecintpNRatioBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbfDecintpNRatioBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbfDecintpORatioBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbfDecintpORatioBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNbfFhModeBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNbfFhModeBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNcoDontUseGlobalClockGatingBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNcoDontUseGlobalClockGatingBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNcoFreeRunClkDivBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNcoFreeRunClkDivBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNcoFreeRunClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNcoFreeRunClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNcoLoSync0EnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNcoLoSync0EnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNcoLoSync1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNcoLoSync1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNcoLoSync2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNcoLoSync2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNcoTxloFreeRunClkDivBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNcoTxloFreeRunClkDivBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNcoTxloFreeRunClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNcoTxloFreeRunClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxNormEnBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxNormEnBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgGainCompClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgGainCompClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgGainCompEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgGainCompEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgGainPhaseDelayClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgGainPhaseDelayClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgGainPhaseDelayEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgGainPhaseDelayEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgGroup1184EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgGroup1184EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgMagcompBbdcClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgMagcompBbdcClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgMagcompBbdcEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgMagcompBbdcEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemDpinfifoDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemDpinfifoDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemDpinfifoSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemDpinfifoSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemDpDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemDpDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemDpSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemDpSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemRxpfir52DontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemRxpfir52DontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemRxpfir52SwResetBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbdemRxpfir52SwResetBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficCorrClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficCorrClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicMClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicMClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicMRatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicMRatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicNClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicNClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicNRatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicNRatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicOClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicOClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicORatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgNbficObsScicORatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgRfirAfterRxqecRoutEnBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgRfirAfterRxqecRoutEnBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgScic1Dec8EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgScic1Dec8EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgScic2Dec8EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgScic2Dec8EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgScicClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgScicClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgSlicerRssiEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgSlicerRssiEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgTxGainDelayClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgTxGainDelayClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1AlgTxGainDelayEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1AlgTxGainDelayEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1ClkgenCssiDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1ClkgenCssiDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1ClockUseRealignBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1ClockUseRealignBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DataCaptureDriveEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DataCaptureDriveEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DdcDataSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DdcDataSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpAlgScicDivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpAlgScicDivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec225EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec225EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec227EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec227EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec229EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec229EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec231EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec231EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec2331EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec2331EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec233EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec233EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec235EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec235EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec237EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec237EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec239EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec239EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec241EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec241EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec243EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec243EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec247EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec247EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec315EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec315EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpDec345EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpDec345EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpHb2171EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpHb2171EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpHb2172EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpHb2172EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpHb2191EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpHb2191EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpHb2192EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpHb2192EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpHb26EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpHb26EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpHb27EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpHb27EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpHb28EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpHb28EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpScic23DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpScic23DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpScic23EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpScic23EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpSinc3EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpSinc3EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpSinc4EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpSinc4EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpSinc6EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpSinc6EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpSpiIntfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpSpiIntfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1DpinfifoEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1DpinfifoEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemArmoutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemArmoutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemCorroutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemCorroutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemDpoutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemDpoutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemTedFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemTedFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemCounter1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemCounter1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemFreqDiscriminatorEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemFreqDiscriminatorEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemNco50EnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemNco50EnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbdemPulsePfirEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbdemPulsePfirEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1NbwbSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1NbwbSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1Nco1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1Nco1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1OverloadHb2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1OverloadHb2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1Pfir4xDiv4p5BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1Pfir4xDiv4p5BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1Pfir4xDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1Pfir4xDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1RxqecClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1RxqecClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1nbResampEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1nbResampEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx1nbRxpfir52InEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx1nbRxpfir52InEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgGainCompClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgGainCompClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgGainCompEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgGainCompEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgGainPhaseDelayClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgGainPhaseDelayClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgGainPhaseDelayEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgGainPhaseDelayEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgGroup1184EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgGroup1184EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgMagcompBbdcClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgMagcompBbdcClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgMagcompBbdcEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgMagcompBbdcEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemDpinfifoDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemDpinfifoDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemDpinfifoSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemDpinfifoSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemDpDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemDpDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemDpSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemDpSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemRxpfir52DontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemRxpfir52DontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemRxpfir52SwResetBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbdemRxpfir52SwResetBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficCorrClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficCorrClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicMClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicMClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicMRatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicMRatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicNClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicNClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicNRatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicNRatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicOClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicOClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicORatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgNbficObsScicORatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgRfirAfterRxqecRoutEnBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgRfirAfterRxqecRoutEnBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgScic1Dec8EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgScic1Dec8EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgScic2Dec8EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgScic2Dec8EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgScicClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgScicClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgSlicerRssiEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgSlicerRssiEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgTxGainDelayClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgTxGainDelayClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2AlgTxGainDelayEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2AlgTxGainDelayEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2ClkgenCssiDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2ClkgenCssiDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2ClockUseRealignBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2ClockUseRealignBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DataCaptureDriveEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DataCaptureDriveEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DdcDataSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DdcDataSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpAlgScicDivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpAlgScicDivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec225EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec225EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec227EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec227EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec229EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec229EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec231EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec231EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec2331EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec2331EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec233EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec233EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec235EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec235EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec237EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec237EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec239EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec239EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec241EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec241EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec243EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec243EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec247EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec247EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec315EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec315EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpDec345EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpDec345EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpHb2171EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpHb2171EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpHb2172EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpHb2172EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpHb2191EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpHb2191EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpHb2192EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpHb2192EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpHb26EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpHb26EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpHb27EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpHb27EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpHb28EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpHb28EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpScic23DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpScic23DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpScic23EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpScic23EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpSinc3EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpSinc3EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpSinc4EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpSinc4EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpSinc6EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpSinc6EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpSpiIntfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpSpiIntfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2DpinfifoEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2DpinfifoEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemArmoutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemArmoutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemCorroutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemCorroutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemDpoutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemDpoutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemTedFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemTedFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemCounter1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemCounter1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemFreqDiscriminatorEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemFreqDiscriminatorEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemNco50EnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemNco50EnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbdemPulsePfirEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbdemPulsePfirEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2NbwbSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2NbwbSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2Nco1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2Nco1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2OverloadHb2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2OverloadHb2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2Pfir4xDiv4p5BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2Pfir4xDiv4p5BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2Pfir4xDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2Pfir4xDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2RxqecClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2RxqecClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2nbResampEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2nbResampEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx2nbRxpfir52InEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx2nbRxpfir52InEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgGainCompClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgGainCompClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgGainCompEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgGainCompEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgGainPhaseDelayClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgGainPhaseDelayClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgGainPhaseDelayEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgGainPhaseDelayEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgGroup1184EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgGroup1184EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgMagcompBbdcClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgMagcompBbdcClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgMagcompBbdcEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgMagcompBbdcEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemDpinfifoDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemDpinfifoDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemDpinfifoSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemDpinfifoSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemDpDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemDpDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemDpSwResetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemDpSwResetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemRxpfir52DontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemRxpfir52DontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemRxpfir52SwResetBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbdemRxpfir52SwResetBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficCorrClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficCorrClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicMClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicMClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicMRatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicMRatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicNClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicNClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicNRatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicNRatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicOClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicOClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicORatioBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgNbficObsScicORatioBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgRfirAfterRxqecRoutEnBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgRfirAfterRxqecRoutEnBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgScic1Dec8EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgScic1Dec8EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgScic2Dec8EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgScic2Dec8EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgScicClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgScicClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgSlicerRssiEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgSlicerRssiEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgTxGainDelayClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgTxGainDelayClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3AlgTxGainDelayEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3AlgTxGainDelayEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3ClkgenCssiDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3ClkgenCssiDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DataCaptureDriveEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DataCaptureDriveEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DdcDataSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DdcDataSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpAlgScicDivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpAlgScicDivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec225EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec225EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec227EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec227EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec229EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec229EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec231EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec231EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec2331EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec2331EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec233EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec233EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec235EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec235EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec237EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec237EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec239EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec239EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec241EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec241EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec243EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec243EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec247EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec247EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec315EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec315EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpDec345EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpDec345EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpHb2171EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpHb2171EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpHb2172EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpHb2172EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpHb2191EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpHb2191EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpHb2192EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpHb2192EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpHb26EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpHb26EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpHb27EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpHb27EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpHb28EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpHb28EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpScic23DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpScic23DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpScic23EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpScic23EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpSinc3EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpSinc3EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpSinc4EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpSinc4EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpSinc6EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpSinc6EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpSpiIntfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpSpiIntfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3DpinfifoEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3DpinfifoEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemArmoutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemArmoutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemCorroutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemCorroutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemDpoutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemDpoutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemTedFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemTedFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemCounter1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemCounter1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemFreqDiscriminatorEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemFreqDiscriminatorEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemNco50EnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemNco50EnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbdemPulsePfirEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbdemPulsePfirEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3NbwbSelBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3NbwbSelBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3Nco1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3Nco1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3OverloadHb2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3OverloadHb2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3Pfir4xDiv4p5BfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3Pfir4xDiv4p5BfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3Pfir4xDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3Pfir4xDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3RxqecClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3RxqecClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3nbResampEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3nbResampEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrx3nbRxpfir52InEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrx3nbRxpfir52InEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrxProfileSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrxProfileSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrxSelBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrxSelBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrxSsiProfileSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrxSsiProfileSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrxSsiSelBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrxSsiSelBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOrxqecObsCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOrxqecObsCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOverflowRfirBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOverloadClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOverloadClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxOverloadHb2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxOverloadHb2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxPhaseSyncFtwUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxPhaseSyncFtwUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxPhaseSyncNcoPhaseAvThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxPhaseSyncNcoPhaseAvThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxPhaseSyncNcoSwControlBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxPhaseSyncNcoSwControlBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxPwrMonRssiEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxPwrMonRssiEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxReserved0BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxReserved0BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxReserved1BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxReserved1BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRfdcEnableBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRfdcEnableBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRpfirInSelBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRpfirInSelBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2DataBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2LengthBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2LengthBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2ReadSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2ReadSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2RxEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2RxEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2StartSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcFlashCal2StartSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcFlashCompAddrBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcFlashCompAddrBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcFlashReadSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcFlashReadSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcFlashWaitBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcFlashWaitBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcOverloadLowerThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcOverloadLowerThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetDurationBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetDurationBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetRfirInDurationBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetRfirInDurationBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                  int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetRfirOutDurationBfSet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetRfirOutDurationBfGet(adi_adrv9001_Device_t *device, 
                                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetSizeBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetSizeBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcOverloadResetThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcOverloadUpperThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcOverloadUpperThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcOvldDurationBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcOvldDurationBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcProfileBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcProfileBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcSelBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcSelBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAdcSpeedSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAdcSpeedSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAgcRssiEnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAgcRssiEnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgRhriClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgRhriClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgRmrnClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgRmrnClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgGainCompClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgGainCompClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgGainCompEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgGainCompEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgGainPhaseDelayClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgGainPhaseDelayClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgGainPhaseDelayEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgGainPhaseDelayEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgGroup1184EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgGroup1184EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMagcompBbdcClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMagcompBbdcClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMagcompBbdcDontUseGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMagcompBbdcDontUseGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMagcompBbdcEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMagcompBbdcEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux200aSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux200aSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux200SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux200SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux201SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux201SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux202bRdSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux202bRdSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux202bRjSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux202bRjSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux202SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux202SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux203RhSelBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux203RhSelBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux204SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux204SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux205NbficSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux205NbficSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux206RxqecRx1CorrSelBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux206RxqecRx1CorrSelBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgMux207SelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgMux207SelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficCorrClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficCorrClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficObsClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficObsClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficObsClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficObsClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicMClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicMClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicMRatioBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicMRatioBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicNClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicNClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicNRatioBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicNRatioBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicOClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicOClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicORatioBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgNbficObsScicORatioBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgRfirAfterRxqecRoutEnBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgRfirAfterRxqecRoutEnBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgRxqecEnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgRxqecEnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgScic1Dec8EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgScic1Dec8EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgScic2Dec8EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgScic2Dec8EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgScicClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgScicClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgSlicerRssiEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgSlicerRssiEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgTxGainDelayClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgTxGainDelayClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAlgTxGainDelayEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxAlgTxGainDelayEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxAveragingEndBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxChanFuncsRadioStatesBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxChanFuncsRadioStatesBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxClkgenCssiDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxClkgenCssiDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxClkgenCssiLoadDividerBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxClkgenCssiLoadDividerBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxClockUseRealignBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxClockUseRealignBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDatapathForceZeroBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDatapathForceZeroBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDdcDataSelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDdcDataSelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDdcDin1SelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDdcDin1SelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDdcDin2SelBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDdcDin2SelBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDdcZeroIBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDdcZeroIBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDdcZeroQBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDdcZeroQBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecOverloadDurationCountBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecOverloadDurationCountBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecOverloadPowerModeBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecOverloadPowerModeBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecOverloadThresholdCountBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecOverloadThresholdCountBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecThresholdConfigBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecThresholdConfigBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt0LowerThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt0LowerThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt1LowerThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt1LowerThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadIp3UpperThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadIp3UpperThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadLowerThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadLowerThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadUpperThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDecimatedDataOverloadUpperThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDigitalGainSaturationBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDisableAdcStartupResetBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDisableAdcStartupResetBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpAgcWb10EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpAgcWb10EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpAlgScicDivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpAlgScicDivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpAlgScicRippleSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpAlgScicRippleSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec225EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec225EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec227EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec227EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec229EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec229EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec231EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec231EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec2331EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec2331EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec233EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec233EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec235EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec235EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec237EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec237EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec239EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec239EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec241EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec241EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec243EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec243EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec247EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec247EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec315EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec315EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDec345EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDec345EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDmrR0EnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDmrR0EnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpDmrR1EnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpDmrR1EnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpFic54EnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpFic54EnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpFmR1EnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpFmR1EnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI0BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI0BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI1BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI1BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI2BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI2BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI3BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI3BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI4BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI4BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI5BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainI5BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ0BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ0BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ1BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ1BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ2BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ2BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ3BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ3BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ4BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ4BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ5BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9GainQ5BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGainComp9UpdateSingleBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGainComp9UpdateSingleBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpGlobalResetBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpGlobalResetBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpHb2171EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpHb2171EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpHb2172EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpHb2172EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpHb2191EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpHb2191EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpHb2192EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpHb2192EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpHb26EnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpHb26EnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpHb27EnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpHb27EnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpHb28EnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpHb28EnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpP25R1EnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpP25R1EnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpPfir52ResetBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpPfir52ResetBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpRssiInSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpRssiInSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpRxqec21EnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpRxqec21EnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpRxqecMuxSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpRxqecMuxSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpScic23DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpScic23DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpScic23EnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpScic23EnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpScic23RippleSelBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpScic23RippleSelBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpSinc2Sinc3GainLevelFromSpiBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpSinc2Sinc3GainLevelFromSpiBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpSinc3EnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpSinc3EnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpSinc4EnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpSinc4EnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpSinc6EnBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpSinc6EnBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpSpectralInvertIBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpSpectralInvertIBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpSpectralInvertQBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpSpectralInvertQBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpSpiIntfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpSpiIntfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxDpWbagcMuxSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxDpWbagcMuxSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxEnableAdcOverloadBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxEnableAdcOverloadBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxEnableDecOverloadBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxEnableDecOverloadBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxFirSelBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxFirSelBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxForceAdcOverloadResetBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxForceAdcOverloadResetBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxFuncGainCompEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxFuncGainCompEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxMagCompPfir21SwResetBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxMagCompPfir21SwResetBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxMisrBypassBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxMisrBypassBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxMisrDatapathSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxMisrDatapathSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxMisrEnableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxMisrEnableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxMisrLengthBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxMisrLengthBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxMisrSignatureBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 int16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemArmoutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemArmoutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemCorroutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemCorroutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemDpoutfifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemDpoutfifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemTedFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemTedFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemCorrelatorParallelEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemCorrelatorParallelEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemCounter1EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemCounter1EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemCounter2EnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemCounter2EnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemFreqDiscriminatorEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemFreqDiscriminatorEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemNco50EnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemNco50EnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbdemPulsePfirEnableBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbdemPulsePfirEnableBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfEnBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfEnBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfEnableCntLimitBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfEnableCntLimitBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfEnableThldBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfEnableThldBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFic1BypassBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFic1BypassBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFic1IsourceBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFic1IsourceBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicCoefDciBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicCoefDcqBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicCoefGainBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicCoefPhaseBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicCoefWrBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicCoefWrBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicConfigBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicConfigBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicCtlRdataBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicCtrlWrBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicCtrlWrBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicEstCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicEstForceBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicEstForceBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicGctlBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicGctlBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicGnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicGnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicGpRdSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicGpRdSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicPctlBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicPctlBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicPhBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFicPhBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicRegConfigBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicRegCountBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicRegDcBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicRegGainBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicRegPhaseBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicStateBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFicStatusBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfFirBypBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfFirBypBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfHaltCntLimitBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfHaltCntLimitBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfHaltThldBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfHaltThldBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfPgobsDselBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfPgobsDselBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRestartBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfRestartBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssiABfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssiBBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssiCompEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfRssiCompEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssiDivBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfRssiDivBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssiFicCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssiIntvlBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfRssiIntvlBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssiMetricBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssicompEnableCntrBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssicompHaltCntrBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfRssicompTransCntrBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfScobsEnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfScobsEnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbfTransCntLimitBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbfTransCntLimitBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNbgainMuxSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNbgainMuxSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1EnableBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1EnableBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1Ftw158BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1Ftw158BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1Ftw2316BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1Ftw2316BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1Ftw3124BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1Ftw3124BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1Ftw70BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1Ftw70BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1FtwUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1FtwUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1MultiChipSyncedBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1MultiChipSyncedBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1PhaseBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1PhaseOffset158BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1PhaseOffset158BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1PhaseOffset70BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1PhaseOffset70BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1PhaseSyncFtw158BfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1PhaseSyncFtw158BfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1PhaseSyncFtw2316BfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1PhaseSyncFtw2316BfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1PhaseSyncFtw70BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1PhaseSyncFtw70BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco1RealOutEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco1RealOutEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2Ftw158BfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2Ftw158BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2Ftw2316BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2Ftw2316BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2Ftw3124BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2Ftw3124BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2Ftw70BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2Ftw70BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2FtwUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2FtwUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2MultiChipSyncedBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2MultiChipSyncedBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2PhaseOffset158BfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2PhaseOffset158BfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2PhaseOffset70BfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2PhaseOffset70BfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNco2RealOutEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNco2RealOutEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNcoPhaseAvBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNcoPhaseOutBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxNcoUseNonTxloMcsPhaseAccumBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxNcoUseNonTxloMcsPhaseAccumBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxPfir4xDiv4p5BfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxPfir4xDiv4p5BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxPfir4xDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxPfir4xDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxPhaseSyncFtwRemainderBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxPhaseSyncFtwRemainderBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxPhaseSyncNcoSyncCyclesBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxPhaseSyncNcoSyncCyclesBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxPhaseSyncStartSyncBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxPhaseSyncStartSyncBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxRealignClk1105StatusBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxRealignClkRestartBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxRealignClkRestartBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxRealignRxCoreClkStatusBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxRegLoadShadowBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxRegLoadShadowBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxRxqecClockSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxRxqecClockSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxTempGainCompBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxTempGainCompBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxddcPosEjwtSelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxddcPosEjwtSelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxgtClkEnBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           int8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxgtClkEnBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxnbPfirGainBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxnbPfirGainBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxnbPfirSymmetricBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxnbPfirSymmetricBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxnbPfirTapsBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxnbPfirTapsBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxnbResampEnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxnbResampEnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxnbResampPhaseIBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxnbResampPhaseIBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxnbResampPhaseQBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t bfValue);
int32_t adrv9001_NvsRegmapRxRxnbResampPhaseQBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxnbRxpfir52InEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxnbRxpfir52InEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxoutSelBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxoutSelBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxRxqecObsCaptEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxRxqecObsCaptEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxSecondNcoPhaseAvBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapRxSlicerGainDisableOffsetWordBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t bfValue);
int32_t adrv9001_NvsRegmapRxSlicerGainDisableOffsetWordBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                             int8_t *bfValue);


int32_t adrv9001_NvsRegmapRxSlicerPinControlModeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxSlicerPinControlModeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxSlicerPinControlStepBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxSlicerPinControlStepBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxSwStatus0BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxSwStatus0BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxSwStatus1BfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxSwStatus1BfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxSysArm0EnableBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxSysArm0EnableBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxSysArm1EnableBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxSysArm1EnableBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxTedCntClearBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxTedCntClearBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxTedDataSelBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxTedDataSelBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxTxAlgRxmux301ObsSourceSelBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxTxAlgRxmux301ObsSourceSelBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapRxVcorrsigBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
uint8_t channelId,                                           uint32_t bfValue);
int32_t adrv9001_NvsRegmapRxVcorrsigBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
uint8_t channelId,                                           uint32_t *bfValue);


int32_t adrv9001_NvsRegmapRxX4x5SelBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapRxX4x5SelBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapRxChanAddr_e baseAddr, 
                                         uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_NVS_REGMAP_RX_H__

/*  EOF: adrv9001_bf_nvs_regmap_rx.h */

