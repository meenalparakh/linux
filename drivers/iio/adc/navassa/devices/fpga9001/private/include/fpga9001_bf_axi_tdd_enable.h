/**
 * \file fpga9001_bf_axi_tdd_enable.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_AXI_TDD_ENABLE_H__
#define __FPGA9001_BF_AXI_TDD_ENABLE_H__

#include "fpga9001_bf_axi_tdd_enable_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_AxiTddEnableDmamanualassertBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_AxiTddEnableDmamanualassertBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_AxiTddEnableInstanceIdBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinframeprimaryassertBfSet(adi_fpga9001_Device_t *device, 
                                                        fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                        uint32_t bfValue);
int32_t fpga9001_AxiTddEnablePinframeprimaryassertBfGet(adi_fpga9001_Device_t *device, 
                                                        fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                        uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinframeprimarydeassertBfSet(adi_fpga9001_Device_t *device, 
                                                          fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                          uint32_t bfValue);
int32_t fpga9001_AxiTddEnablePinframeprimarydeassertBfGet(adi_fpga9001_Device_t *device, 
                                                          fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                          uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinframesecondaryassertBfSet(adi_fpga9001_Device_t *device, 
                                                          fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                          uint32_t bfValue);
int32_t fpga9001_AxiTddEnablePinframesecondaryassertBfGet(adi_fpga9001_Device_t *device, 
                                                          fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                          uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinframesecondarydeassertBfSet(adi_fpga9001_Device_t *device, 
                                                            fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                            uint32_t bfValue);
int32_t fpga9001_AxiTddEnablePinframesecondarydeassertBfGet(adi_fpga9001_Device_t *device, 
                                                            fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                            uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinmanualassertBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_AxiTddEnablePinmanualassertBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_AxiTddEnablePinprimaryassertBfSet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                   uint32_t bfValue);
int32_t fpga9001_AxiTddEnablePinprimaryassertBfGet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                   uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinprimarydeassertBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t fpga9001_AxiTddEnablePinprimarydeassertBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinprimaryenableBfSet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t fpga9001_AxiTddEnablePinprimaryenableBfGet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t fpga9001_AxiTddEnablePinsecondaryassertBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t fpga9001_AxiTddEnablePinsecondaryassertBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinsecondarydeassertBfSet(adi_fpga9001_Device_t *device, 
                                                       fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                       uint32_t bfValue);
int32_t fpga9001_AxiTddEnablePinsecondarydeassertBfGet(adi_fpga9001_Device_t *device, 
                                                       fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                       uint32_t *bfValue);


int32_t fpga9001_AxiTddEnablePinsecondaryenableBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t fpga9001_AxiTddEnablePinsecondaryenableBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_AXI_TDD_ENABLE_H__

/*  EOF: fpga9001_bf_axi_tdd_enable.h */

