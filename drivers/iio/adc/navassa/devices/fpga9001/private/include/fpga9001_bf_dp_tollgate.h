/**
 * \file fpga9001_bf_dp_tollgate.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_DP_TOLLGATE_H__
#define __FPGA9001_BF_DP_TOLLGATE_H__

#include "fpga9001_bf_dp_tollgate_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_DpTollgateEdgeLevelBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                          uint32_t bfValue);
int32_t fpga9001_DpTollgateEdgeLevelBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                          uint32_t *bfValue);


int32_t fpga9001_DpTollgateHighRisingLowFallingBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                                     uint32_t bfValue);
int32_t fpga9001_DpTollgateHighRisingLowFallingBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                                     uint32_t *bfValue);


int32_t fpga9001_DpTollgateModuleVersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t fpga9001_DpTollgateModuleVersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                                   uint16_t *bfValue);


int32_t fpga9001_DpTollgateResetTollgateBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_DpTollgateResetTollgateBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_DpTollgateTriggerSelectBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                              uint32_t bfValue);
int32_t fpga9001_DpTollgateTriggerSelectBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfDpTollgateChanAddr_e baseAddr, 
                                              uint32_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_DP_TOLLGATE_H__

/*  EOF: fpga9001_bf_dp_tollgate.h */

