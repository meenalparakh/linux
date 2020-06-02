/**
 * \file fpga9001_bf_axi_tdd_debug.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_AXI_TDD_DEBUG_H__
#define __FPGA9001_BF_AXI_TDD_DEBUG_H__

#include "./../../private/include/fpga9001_bf_axi_tdd_debug_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_AxiTddDebugDebugPatternBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiTddDebugChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiTddDebugDebugPatternBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiTddDebugChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiTddDebugDebugSelBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiTddDebugChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiTddDebugDebugSelBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiTddDebugChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiTddDebugInstanceIdBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiTddDebugChanAddr_e baseAddr, 
                                            uint32_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_AXI_TDD_DEBUG_H__

/*  EOF: fpga9001_bf_axi_tdd_debug.h */

