/**
 * \file fpga9001_bf_zc706.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_ZC706_H__
#define __FPGA9001_BF_ZC706_H__

#include "fpga9001_bf_zc706_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_Zc706ExtTrig0DirBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrig0DirBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_Zc706ExtTrig0OutputSrcBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrig0OutputSrcBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_Zc706ExtTrig1DirBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrig1DirBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_Zc706ExtTrig1OutputSrcBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrig1OutputSrcBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_Zc706ExtTrigRxdp00CtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrigRxdp00CtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_Zc706ExtTrigRxdp01CtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrigRxdp01CtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_Zc706ExtTrigTddCtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrigTddCtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_Zc706ExtTrigTxdp00CtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrigTxdp00CtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_Zc706ExtTrigTxdp01CtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t fpga9001_Zc706ExtTrigTxdp01CtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_Zc706GlobalResetBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t fpga9001_Zc706GlobalResetBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_Zc706IicMuxResetBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t fpga9001_Zc706IicMuxResetBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_Zc706LedBlinkDurationBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfZc706ChanAddr_e baseAddr, 
                                            uint16_t bfValue);
int32_t fpga9001_Zc706LedBlinkDurationBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfZc706ChanAddr_e baseAddr, 
                                            uint16_t *bfValue);


int32_t fpga9001_Zc706LedDelayToModeChangeBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint16_t bfValue);
int32_t fpga9001_Zc706LedDelayToModeChangeBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                uint16_t *bfValue);


int32_t fpga9001_Zc706LedMaskBfSet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfZc706ChanAddr_e baseAddr, 
                                   uint8_t bfValue);
int32_t fpga9001_Zc706LedMaskBfGet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfZc706ChanAddr_e baseAddr, 
                                   uint8_t *bfValue);


int32_t fpga9001_Zc706LedModeBfSet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfZc706ChanAddr_e baseAddr, 
                                   uint8_t bfValue);
int32_t fpga9001_Zc706LedModeBfGet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfZc706ChanAddr_e baseAddr, 
                                   uint8_t *bfValue);


int32_t fpga9001_Zc706LedNumBlinksBfSet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfZc706ChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t fpga9001_Zc706LedNumBlinksBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfZc706ChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_Zc706LedSwDefinedValueBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_Zc706LedSwDefinedValueBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_Zc706VersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfZc706ChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_Zc706VersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfZc706ChanAddr_e baseAddr, 
                                        uint16_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_ZC706_H__

/*  EOF: fpga9001_bf_zc706.h */

