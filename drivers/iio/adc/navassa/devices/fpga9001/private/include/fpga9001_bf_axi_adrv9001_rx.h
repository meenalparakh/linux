/**
 * \file fpga9001_bf_axi_adrv9001_rx.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_AXI_ADRV9001_RX_H__
#define __FPGA9001_BF_AXI_ADRV9001_RX_H__

#include "fpga9001_bf_axi_adrv9001_rx_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_AxiAdrv9001RxOrxDecRatioBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxOrxDecRatioBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxClkRateBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxClkRateBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxCmos1Lvds0BfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxCmosLvdsIdBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxDataPatBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                             uint32_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxDataPatBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxDataSelBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxDataSelBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxDecRatioBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxDecRatioBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxFall1Rise0BfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxFall1Rise0BfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxFrmErrBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxFrmErrBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxFrmOosBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxFrmOosBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxInitBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxInitBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxLsb1Msb0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxLsb1Msb0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxLstrb1Pstrb0BfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxLstrb1Pstrb0BfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxMlane1Slane0BfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxMlane1Slane0BfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxMonErrBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxMonErrBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxMonOosBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxMonOosBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxNumOfBitsBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxNumOfBitsBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxQ1I0BfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxQ1I0BfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxRddelay0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxRddelay1BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxRddelay2BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxRddelay3BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxRddelaySBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxSdr1Ddr0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxSdr1Ddr0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxWrdelay0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxWrdelay0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxWrdelay1BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxWrdelay1BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxWrdelay2BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxWrdelay2BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxWrdelay3BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxWrdelay3BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001RxRxWrdelaySBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001RxRxWrdelaySBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_AXI_ADRV9001_RX_H__

/*  EOF: fpga9001_bf_axi_adrv9001_rx.h */

