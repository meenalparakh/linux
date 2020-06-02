/**
 * \file adrv9001_bf_txdac_mem_map.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __ADRV9001_BF_TXDAC_MEM_MAP_H__
#define __ADRV9001_BF_TXDAC_MEM_MAP_H__

#include "adrv9001_bf_txdac_mem_map_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_TxdacMemMapTxdacCalclkDivBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalclkDiv_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalclkDivBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalclkDiv_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalclkEnIBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalclkEnIBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalclkEnQBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalclkEnQBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCaldacFscBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCaldacFsc_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCaldacFscBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCaldacFsc_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCaldacOffsetBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   adrv9001_BfTxdacMemMapTxdacCaldacOffset_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCaldacOffsetBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   adrv9001_BfTxdacMemMapTxdacCaldacOffset_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalActiveIBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalActiveQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalAddrIBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               adrv9001_BfTxdacMemMapTxdacCalAddrI_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalAddrIBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               adrv9001_BfTxdacMemMapTxdacCalAddrI_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalAddrQBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               adrv9001_BfTxdacMemMapTxdacCalAddrQ_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalAddrQBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               adrv9001_BfTxdacMemMapTxdacCalAddrQ_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalDebugIBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalDebugI_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalDebugIBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalDebugI_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalDebugQBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalDebugQ_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalDebugQBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalDebugQ_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalDoneIBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalDoneQBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalDoIsbrefBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalDoIsbrefBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalDoOffsetCalBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalDoOffsetCalBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalIsbrefInitBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalIsbrefInitBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalIsbLoopBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalIsbLoopBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalIsbSatIBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalIsbSatQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalMsbLoopBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalMsbLoopBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalMsbSatIBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalMsbSatQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalOffsetSatIBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalOffsetSatQBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalRdDataIBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalRdDataQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalSettleDelayBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalSettleDelayBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalStartIBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalStartI_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalStartIBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalStartI_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalStartQBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalStartQ_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalStartQBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacCalStartQ_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalUseSelectBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalUseSelectBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalWeIBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalWeIBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalWeQBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalWeQBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalWrDataIBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalWrDataIBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCalWrDataQBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCalWrDataQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCmCtrlBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                             adrv9001_BfTxdacMemMapTxdacCmCtrl_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCmCtrlBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                             adrv9001_BfTxdacMemMapTxdacCmCtrl_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompAvgmsbIBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompAvgmsbQBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompBiaspdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompBiaspdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompFastcmEnbBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompFastcmEnbBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompGainIsbBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  adrv9001_BfTxdacMemMapTxdacCompGainIsb_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompGainIsbBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  adrv9001_BfTxdacMemMapTxdacCompGainIsb_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompGainMsbBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  adrv9001_BfTxdacMemMapTxdacCompGainMsb_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompGainMsbBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  adrv9001_BfTxdacMemMapTxdacCompGainMsb_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompIsbavgBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 adrv9001_BfTxdacMemMapTxdacCompIsbavg_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompIsbavgBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 adrv9001_BfTxdacMemMapTxdacCompIsbavg_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompIsbpdBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompIsbpdBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompMsbavgBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 adrv9001_BfTxdacMemMapTxdacCompMsbavg_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompMsbavgBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 adrv9001_BfTxdacMemMapTxdacCompMsbavg_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompMsbpdBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompMsbpdBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompOneshotEnbBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacCompOneshotEnbBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompOutIBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacCompOutQBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacDitherAmplitudeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacDitherAmplitudeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacDitherEnableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacDitherEnableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacDitherPeriodBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint16_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacDitherPeriodBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacExtcalEnableIBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacExtcalEnableIBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacExtcalEnableQBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacExtcalEnableQBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacFscBoostIBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacFscBoostI_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacFscBoostIBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacFscBoostI_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacFscBoostQBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacFscBoostQ_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacFscBoostQBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfTxdacMemMapTxdacFscBoostQ_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacFscTuneIBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacFscTuneIBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacFscTuneQBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacFscTuneQBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacIsbShuffleEnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacIsbShuffleEnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacMsbShuffleEnableBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacMsbShuffleEnableBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacOutputProtect0p2vBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacOutputProtect0p2vBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacOutputProtectGndBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacOutputProtectGndBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPddriverEnableBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacPddriverEnableBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPdMbiasOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacPdMbiasOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerdownClkdivBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                      adrv9001_BfTxdacMemMapTxdacPowerdownClkdiv_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacPowerdownClkdivBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                      adrv9001_BfTxdacMemMapTxdacPowerdownClkdiv_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerdownCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacPowerdownCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerdownIBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacPowerdownIBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerdownOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacPowerdownOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerdownQBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacPowerdownQBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerdownStatIBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerdownStatQBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerupStatIBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacPowerupStatQBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacSelCalToPinBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  adrv9001_BfTxdacMemMapTxdacSelCalToPin_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacSelCalToPinBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                  adrv9001_BfTxdacMemMapTxdacSelCalToPin_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacShuffleAmplitudeBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacShuffleAmplitudeBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacShuffleResetbBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacShuffleResetbBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacSpareCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacSpareCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacStartupManclkBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacStartupManclkBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacStartupManselBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    adrv9001_BfTxdacMemMapTxdacStartupMansel_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacStartupManselBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                    adrv9001_BfTxdacMemMapTxdacStartupMansel_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacStartupReadyBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacThrowIsbCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   adrv9001_BfTxdacMemMapTxdacThrowIsbCtrl_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacThrowIsbCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   adrv9001_BfTxdacMemMapTxdacThrowIsbCtrl_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacThrowLsbCtrlBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   adrv9001_BfTxdacMemMapTxdacThrowLsbCtrl_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacThrowLsbCtrlBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                   adrv9001_BfTxdacMemMapTxdacThrowLsbCtrl_e *bfValue);


int32_t adrv9001_TxdacMemMapTxdacUseGainBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacUseGainBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacVdsMarginBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacVdsMarginBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacWalkEnableBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_TxdacMemMapTxdacWalkEnableBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_TxdacMemMapTxdacWalkPeriodBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 adrv9001_BfTxdacMemMapTxdacWalkPeriod_e bfValue);
int32_t adrv9001_TxdacMemMapTxdacWalkPeriodBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr, 
                                                 adrv9001_BfTxdacMemMapTxdacWalkPeriod_e *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_TXDAC_MEM_MAP_H__

/*  EOF: adrv9001_bf_txdac_mem_map.h */

