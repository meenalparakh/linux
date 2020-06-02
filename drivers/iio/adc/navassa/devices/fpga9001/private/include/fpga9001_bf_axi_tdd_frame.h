/**
 * \file fpga9001_bf_axi_tdd_frame.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_AXI_TDD_FRAME_H__
#define __FPGA9001_BF_AXI_TDD_FRAME_H__

#include "fpga9001_bf_axi_tdd_frame_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_AxiTddFrameInstanceIdBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t fpga9001_AxiTddFrameTddframecountersresetBfSet(adi_fpga9001_Device_t *device, 
                                                       fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t fpga9001_AxiTddFrameTddframecountersresetBfGet(adi_fpga9001_Device_t *device, 
                                                       fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t fpga9001_AxiTddFrameTddframeperiodBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t fpga9001_AxiTddFrameTddframeperiodBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t fpga9001_AxiTddFrameTddframeperiodenableBfSet(adi_fpga9001_Device_t *device, 
                                                      fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t fpga9001_AxiTddFrameTddframeperiodenableBfGet(adi_fpga9001_Device_t *device, 
                                                      fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t fpga9001_AxiTddFrameTddframesequencerepeatBfSet(adi_fpga9001_Device_t *device, 
                                                        fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t fpga9001_AxiTddFrameTddframesequencerepeatBfGet(adi_fpga9001_Device_t *device, 
                                                        fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t fpga9001_AxiTddFrameTddframeswitchtimeBfSet(adi_fpga9001_Device_t *device, 
                                                    fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t fpga9001_AxiTddFrameTddframeswitchtimeBfGet(adi_fpga9001_Device_t *device, 
                                                    fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t fpga9001_AxiTddFrameTddframeswitchtimechannelBfSet(adi_fpga9001_Device_t *device, 
                                                           fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t fpga9001_AxiTddFrameTddframeswitchtimechannelBfGet(adi_fpga9001_Device_t *device, 
                                                           fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t fpga9001_AxiTddFrameTddnumberframesBfSet(adi_fpga9001_Device_t *device, 
                                                 fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                 uint32_t bfValue);
int32_t fpga9001_AxiTddFrameTddnumberframesBfGet(adi_fpga9001_Device_t *device, 
                                                 fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t fpga9001_AxiTddFrameTddnumberframesenableBfSet(adi_fpga9001_Device_t *device, 
                                                       fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t fpga9001_AxiTddFrameTddnumberframesenableBfGet(adi_fpga9001_Device_t *device, 
                                                       fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t fpga9001_AxiTddFrameTddtriggerenableBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_AxiTddFrameTddtriggerenableBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddFrameChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_AXI_TDD_FRAME_H__

/*  EOF: fpga9001_bf_axi_tdd_frame.h */

