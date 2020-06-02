/**
 * \file adrv9001_bf_nvs_regmap_txb.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __ADRV9001_BF_NVS_REGMAP_TXB_H__
#define __ADRV9001_BF_NVS_REGMAP_TXB_H__

#include "adrv9001_bf_nvs_regmap_txb_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_NvsRegmapTxbClgcCcCnt0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcImagAcc0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint64_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcInvertXiBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbClgcCcInvertXiBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcInvertXqBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbClgcCcInvertXqBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcInvertYiBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbClgcCcInvertYiBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcInvertYqBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbClgcCcInvertYqBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcOutI0BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcOutI1BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcOutI2BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcOutI3BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcOutQ0BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcOutQ1BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcOutQ2BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcOutQ3BfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcCcRealAcc0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint64_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmAcc0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint64_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmCnt0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmPowExp0BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmPowExp1BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmPowExp2BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmPowExp3BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmPowMan0BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmPowMan1BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmPowMan2BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmPowMan3BfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbClgcPmStatusOutBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcCnt0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcImagAcc0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint64_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcInvertXiBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbGanPaCcInvertXiBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcInvertXqBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbGanPaCcInvertXqBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcInvertYiBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbGanPaCcInvertYiBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcInvertYqBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbGanPaCcInvertYqBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcOutI0BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcOutI1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcOutI2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcOutI3BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcOutQ0BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcOutQ1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcOutQ2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcOutQ3BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaCcRealAcc0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint64_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmAcc0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint64_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmCnt0IntermValBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmPowExp0BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmPowExp1BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmPowExp2BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmPowExp3BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmPowMan0BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmPowMan1BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmPowMan2BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmPowMan3BfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbGanPaPmStatusOutBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup10ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup10ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup11ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup11ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup1ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup1ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup2ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup2ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup2ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup2ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup3ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup3ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup3ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup3ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup4ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup4ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup4ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup4ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup5ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup5ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup5ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup5ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup6ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup6ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup6ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup6ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup7ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup7ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup7ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup7ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup8ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup8ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup8ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup8ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup9ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup9ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup9ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1AlgGroup9ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1CssiClkgenDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1CssiClkgenDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1CssiClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1CssiClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpDucEnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpDucEnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt212EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt212EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt214EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt214EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt216EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt216EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt218EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt218EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt220EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt220EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt2221EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt2221EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt222EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt222EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt224EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt224EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt226EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt226EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt228EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt228EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt230EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt230EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt236EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt236EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt238EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt238EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt28EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt28EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt310EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt310EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt340EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt340EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpInt3441EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpInt3441EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpIqdmEnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpIqdmEnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpModemModeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpModemModeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpModemModeDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpModemModeDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpSinc242EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpSinc242EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpSinc344EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpSinc344EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpSpiIntfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpSpiIntfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpTscic32DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpTscic32DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpTscic32EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpTscic32EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpTxFreqDevMapper47EnBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpTxFreqDevMapper47EnBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpTxPreproc2DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpTxPreproc2DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpTxPreproc2EnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpTxPreproc2EnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1DpTxRound46EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1DpTxRound46EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1LssiClkgenDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1LssiClkgenDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1LssiClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1LssiClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1LssiDataDestClkDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1LssiDataDestClkDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1LssiDataDestClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1LssiDataDestClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1Pfir4xDiv4p5BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1Pfir4xDiv4p5BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1Pfir4xDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1Pfir4xDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1PfirIEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1PfirIEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1PfirQEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1PfirQEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1PfirXISelForceBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1PfirXISelForceBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1PfirXISelForceValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1PfirXISelForceValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1PfirXQSelForceBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1PfirXQSelForceBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1PfirXQSelForceValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1PfirXQSelForceValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1ProfileDpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1ProfileDpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx1ProfileSsiGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx1ProfileSsiGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup10ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup10ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup11ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup11ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup1ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup1ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup2ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup2ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup2ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup2ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup3ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup3ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup3ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup3ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup4ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup4ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup4ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup4ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup5ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup5ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup5ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup5ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup6ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup6ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup6ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup6ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup7ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup7ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup7ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup7ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup8ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup8ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup8ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup8ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup9ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup9ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup9ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2AlgGroup9ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2CssiClkgenDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2CssiClkgenDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2CssiClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2CssiClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpDucEnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpDucEnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt212EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt212EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt214EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt214EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt216EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt216EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt218EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt218EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt220EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt220EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt2221EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt2221EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt222EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt222EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt224EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt224EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt226EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt226EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt228EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt228EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt230EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt230EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt236EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt236EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt238EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt238EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt28EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt28EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt310EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt310EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt340EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt340EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpInt3441EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpInt3441EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpIqdmEnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpIqdmEnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpModemModeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpModemModeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpModemModeDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpModemModeDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpSinc242EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpSinc242EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpSinc344EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpSinc344EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpSpiIntfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpSpiIntfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpTscic32DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpTscic32DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpTscic32EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpTscic32EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpTxFreqDevMapper47EnBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpTxFreqDevMapper47EnBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpTxPreproc2DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpTxPreproc2DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpTxPreproc2EnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpTxPreproc2EnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2DpTxRound46EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2DpTxRound46EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2LssiClkgenDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2LssiClkgenDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2LssiClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2LssiClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2LssiDataDestClkDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2LssiDataDestClkDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2LssiDataDestClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2LssiDataDestClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2Pfir4xDiv4p5BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2Pfir4xDiv4p5BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2Pfir4xDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2Pfir4xDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2PfirIEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2PfirIEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2PfirQEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2PfirQEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2PfirXISelForceBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2PfirXISelForceBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2PfirXISelForceValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2PfirXISelForceValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2PfirXQSelForceBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2PfirXQSelForceBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2PfirXQSelForceValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2PfirXQSelForceValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2ProfileDpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2ProfileDpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx2ProfileSsiGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx2ProfileSsiGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup10ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup10ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup11ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup11ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup1ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup1ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup2ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup2ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup2ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup2ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup3ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup3ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup3ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup3ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup4ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup4ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup4ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup4ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup5ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup5ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup5ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup5ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup6ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup6ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup6ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup6ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup7ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup7ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup7ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup7ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup8ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup8ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup8ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup8ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup9ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup9ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup9ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3AlgGroup9ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3CssiClkgenDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3CssiClkgenDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3CssiClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3CssiClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpDucEnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpDucEnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt212EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt212EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt214EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt214EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt216EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt216EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt218EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt218EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt220EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt220EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt2221EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt2221EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt222EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt222EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt224EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt224EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt226EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt226EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt228EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt228EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt230EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt230EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt236EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt236EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt238EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt238EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt28EnBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt28EnBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt310EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt310EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt340EnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt340EnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpInt3441EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpInt3441EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpIqdmEnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpIqdmEnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpModemModeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpModemModeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpModemModeDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpModemModeDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpSinc242EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpSinc242EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpSinc344EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpSinc344EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpSpiIntfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpSpiIntfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpTscic32DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpTscic32DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpTscic32EnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpTscic32EnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpTxFreqDevMapper47EnBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpTxFreqDevMapper47EnBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpTxPreproc2DivFactorBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpTxPreproc2DivFactorBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpTxPreproc2EnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpTxPreproc2EnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3DpTxRound46EnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3DpTxRound46EnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3LssiClkgenDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3LssiClkgenDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3LssiClkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3LssiClkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3LssiDataDestClkDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3LssiDataDestClkDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3LssiDataDestClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3LssiDataDestClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3Pfir4xDiv4p5BfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3Pfir4xDiv4p5BfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3Pfir4xDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3Pfir4xDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3PfirIEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3PfirIEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3PfirQEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3PfirQEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3PfirXISelForceBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3PfirXISelForceBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3PfirXISelForceValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3PfirXISelForceValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3PfirXQSelForceBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3PfirXQSelForceBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3PfirXQSelForceValueBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3PfirXQSelForceValueBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3ProfileDpGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                    uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3ProfileDpGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                                    adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                    uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtx3ProfileSsiGlobalClkEnableResetBfSet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtx3ProfileSsiGlobalClkEnableResetBfGet(adi_adrv9001_Device_t *device, 
                                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtxProfileSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtxProfileSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtxSelBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtxSelBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtxSsiProfileSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtxSsiProfileSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbOtxSsiSelBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbOtxSsiSelBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgArmOrGroup11ClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgArmOrGroup11ClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11CgClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11CgClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11CgClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11CgClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11ClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11ClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11GdcPreClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11GdcPreClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11GdcPreClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11GdcPreClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11LolbClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11LolbClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11LolbClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11LolbClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11Mc21ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11Mc21ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11PaProtClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11PaProtClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11PaProtClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11PaProtClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11SrlClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11SrlClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11SrlClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11SrlClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxbbfClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxbbfClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxbbfClkResetBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxbbfClkResetBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxbbfClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxbbfClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxAttenDpathClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxAttenDpathClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxAttenDpathClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup11TxAttenDpathClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup1Fifo2ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup1Fifo2ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup1GanPaClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup1GanPaClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup1GanPaResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup1GanPaResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup1Intp2ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup1Intp2ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup1Nco2ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup1Nco2ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup1Pfir2ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup1Pfir2ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup2ClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup2ClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup3DpdTopClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup3DpdTopClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup3DpdTopClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup3DpdTopClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Fifo1ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Fifo1ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Intp1ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Intp1ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup3LolClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup3LolClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Mc13ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Mc13ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Nco1ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Nco1ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Pfir1ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup3Pfir1ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup4CorrClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup4CorrClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup4CorrClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                               uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup4CorrClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup4Fifo0ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup4Fifo0ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup4Intp0ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup4Intp0ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup4Nco0ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup4Nco0ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup4Pfir0ClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup4Pfir0ClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup4TwinfirClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup4TwinfirClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup5ClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup5ClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup7ClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup7ClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup8ClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup8ClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgGroup9ClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgGroup9ClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgNco0FreeClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgNco0FreeClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgNco0FreeResetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgNco0FreeResetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgNco1FreeClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgNco1FreeClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgNco1FreeResetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgNco1FreeResetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgNco2FreeClkEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgNco2FreeClkEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAlgNco2FreeResetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAlgNco2FreeResetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxAttenTableSramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxAttenTableSramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxCssiDataDestClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxCssiDataDestClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxCssiFifoVariableStrobeClearBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxCssiFifoVariableStrobeClearBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxCssiFifoVariableStrobeEmptyBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxCssiFifoVariableStrobeFullBfGet(adi_adrv9001_Device_t *device, 
                                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                               uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxCssiMaskStrobeEnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxCssiMaskStrobeEnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxCssiMaskStrobeValueBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxCssiMaskStrobeValueBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxCssiVariableStrobeModeEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxCssiVariableStrobeModeEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdAcc0SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdAcc0SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdAcc1SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdAcc1SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdAcc2SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdAcc2SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdAcc3SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdAcc3SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdCrossaccSram0NapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdCrossaccSram0NapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut0SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut0SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut1SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut1SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut2SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut2SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut3SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut3SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut4SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut4SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut5SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut5SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut6SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut6SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut7SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut7SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut8SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt1Lut8SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut0SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut0SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut1SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut1SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut2SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut2SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut3SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut3SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut4SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut4SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut5SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut5SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut6SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut6SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut7SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut7SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut8SramNapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdExt2Lut8SramNapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdTxsbufSram0NapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdTxsbufSram0NapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpdTxsbufSram1NapRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpdTxsbufSram1NapRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpIqdmInClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpIqdmInClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxDpIqdmNcoClkResetbMaskBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxDpIqdmNcoClkResetbMaskBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxLssiDataDestClkDivValueBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxLssiDataDestClkDivValueBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                            uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxLssiDataDestClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxLssiDataDestClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxLssiDataDestDbgModeClkSelBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxLssiDataDestDbgModeClkSelBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxLssiFifoEmptyBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxLssiFifoFullBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxLssiMaskStrobeEnableBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxLssiMaskStrobeEnableBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxLssiMaskStrobeValueBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxLssiMaskStrobeValueBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxLssiVariableStrobeModeEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxLssiVariableStrobeModeEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxNco12DcIBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                             uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxNco12DcIBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxNco12DcQBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                             uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxNco12DcQBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxNco12DcUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxNco12DcUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxNco12PhaseOffsetWordBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxNco12PhaseOffsetWordBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxNco1DcIBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                            uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxNco1DcIBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxNco1DcQBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                            uint32_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxNco1DcQBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxNco1DcUpdateBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxNco1DcUpdateBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxNco1PhaseOffsetWordBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxNco1PhaseOffsetWordBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxPfirXISelForceBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxPfirXISelForceBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxPfirXISelForceValueBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxPfirXISelForceValueBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxPfirXQSelForceBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxPfirXQSelForceBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxPfirXQSelForceValueBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxPfirXQSelForceValueBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxQecFifoSram0NapARetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxQecFifoSram0NapARetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxQecFifoSram0NapBRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxQecFifoSram0NapBRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxQecFifoSram1NapARetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxQecFifoSram1NapARetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxQecFifoSram1NapBRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxQecFifoSram1NapBRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxQecFifoSram2NapARetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxQecFifoSram2NapARetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsRegmapTxbTxQecFifoSram2NapBRetPsdBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsRegmapTxbTxQecFifoSram2NapBRetPsdBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_NVS_REGMAP_TXB_H__

/*  EOF: adrv9001_bf_nvs_regmap_txb.h */

