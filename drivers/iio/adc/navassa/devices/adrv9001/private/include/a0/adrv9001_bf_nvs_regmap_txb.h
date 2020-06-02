/**
 * \file adrv9001_bf_nvs_regmap_txb.h Automatically generated file with generator ver 0.0.1.0.
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

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_NVS_REGMAP_TXB_H__

/*  EOF: adrv9001_bf_nvs_regmap_txb.h */

