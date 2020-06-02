/**
 * \file fpga9001_bf_dp_capture_control.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_DP_CAPTURE_CONTROL_H__
#define __FPGA9001_BF_DP_CAPTURE_CONTROL_H__

#include "fpga9001_bf_dp_capture_control_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_DpCaptureControlDatapathActiveBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t fpga9001_DpCaptureControlDatapathActiveBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t fpga9001_DpCaptureControlModuleVersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                                         fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t fpga9001_DpCaptureControlModuleVersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                                         fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t fpga9001_DpCaptureControlPassTokenBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t fpga9001_DpCaptureControlPassTokenBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_DpCaptureControlResetBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_DpCaptureControlResetBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
                                            uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_DP_CAPTURE_CONTROL_H__

/*  EOF: fpga9001_bf_dp_capture_control.h */

