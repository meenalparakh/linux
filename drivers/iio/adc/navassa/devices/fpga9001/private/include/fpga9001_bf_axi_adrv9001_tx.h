/**
 * \file fpga9001_bf_axi_adrv9001_tx.h Automatically generated file with generator ver 0.0.1.0.
 *
 * \brief Contains BitField functions to support FPGA9001 transciever device.
 *
 * FPGA9001 BITFIELD VERSION: 0.0.0.1
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef __FPGA9001_BF_AXI_ADRV9001_TX_H__
#define __FPGA9001_BF_AXI_ADRV9001_TX_H__

#include "fpga9001_bf_axi_adrv9001_tx_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_AxiAdrv9001TxTxBufEnableBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxBufEnableBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxClkMsbBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxClkMsbBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxClkSelBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxClkSelBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxCmos1Lvds0BfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxCmosLvdsIdBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxDataPatBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                             uint32_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxDataPatBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxDataSelBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxDataSelBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxInitBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxInitBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxLsb1Msb0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxLsb1Msb0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxLstrb1Pstrb0BfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxLstrb1Pstrb0BfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxMlane1Slane0BfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxMlane1Slane0BfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxNumOfBitsBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxNumOfBitsBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxQ1I0BfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxQ1I0BfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxRddelay0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxRddelay1BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxRddelay2BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxRddelay3BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxRddelayCBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxRddelaySBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxSdr1Ddr0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxSdr1Ddr0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxWrdelay0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxWrdelay0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxWrdelay1BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxWrdelay1BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxWrdelay2BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxWrdelay2BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxWrdelay3BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxWrdelay3BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxWrdelayCBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxWrdelayCBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001TxTxWrdelaySBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001TxTxWrdelaySBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_AXI_ADRV9001_TX_H__

/*  EOF: fpga9001_bf_axi_adrv9001_tx.h */

