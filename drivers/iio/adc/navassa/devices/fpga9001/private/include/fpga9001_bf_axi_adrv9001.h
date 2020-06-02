/**
 * \file fpga9001_bf_axi_adrv9001.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_AXI_ADRV9001_H__
#define __FPGA9001_BF_AXI_ADRV9001_H__

#include "fpga9001_bf_axi_adrv9001_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_AxiAdrv9001DelayLockedBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioGainIndexBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl00BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl00BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl01BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl01BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl02BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl02BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl03BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl03BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl04BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl04BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl05BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl05BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl06BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl06BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl07BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl07BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl08BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl08BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl09BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl09BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl10BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl10BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioctl11BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioPioctl11BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioPioDataBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                              uint16_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl00BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl00BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl01BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl01BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl02BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl02BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl03BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl03BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl04BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl04BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl05BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl05BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl06BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl06BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl07BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl07BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl08BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl08BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl09BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl09BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl10BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl10BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl11BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl11BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl12BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl12BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl13BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl13BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl14BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl14BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl15BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl15BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl16BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl16BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl17BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl17BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl18BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl18BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl19BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl19BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl20BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl20BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl21BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl21BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl22BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl22BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl23BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl23BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl24BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl24BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl25BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl25BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl26BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl26BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl27BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl27BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl28BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl28BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl29BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl29BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl30BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl30BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsictl31BfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001DgpioSsictl31BfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001DgpioSsiDataBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001GpintBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001GpintSbBfSet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001GpintSbBfGet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001GpintSelBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001GpintSelBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001GpintTestSelBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001GpintTestSelBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001GpintTestValBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001GpintTestValBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001InstanceIdBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001J58SelBfSet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001J58SelBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001McsEdgeBfSet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001McsEdgeBfGet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001McsNumBfSet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001McsNumBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001McsPeriodBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001McsPeriodBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001McsRequestBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001McsRequestBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001McsStatusBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001McsWidthBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001McsWidthBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001MmcmAddrBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001MmcmAddrBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001MmcmBusyBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001MmcmLockedBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001MmcmRd1Wr0BfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001MmcmRd1Wr0BfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001MmcmRddataBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint16_t *bfValue);


int32_t fpga9001_AxiAdrv9001MmcmRequestBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001MmcmRequestBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001MmcmResetBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001MmcmResetBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001MmcmWrdataBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint16_t bfValue);
int32_t fpga9001_AxiAdrv9001MmcmWrdataBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint16_t *bfValue);


int32_t fpga9001_AxiAdrv9001ModeBfSet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                      uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001ModeBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001ResetbBfSet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t fpga9001_AxiAdrv9001ResetbBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_AxiAdrv9001ScratchBfSet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                         uint32_t bfValue);
int32_t fpga9001_AxiAdrv9001ScratchBfGet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                         uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001SsiGainIndexBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001TimerBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                       uint32_t bfValue);
int32_t fpga9001_AxiAdrv9001TimerBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                       uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001UsrGpioInBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                           uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001UsrGpioOutBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint32_t bfValue);
int32_t fpga9001_AxiAdrv9001UsrGpioOutBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                            uint32_t *bfValue);


int32_t fpga9001_AxiAdrv9001VersionBfGet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr, 
                                         uint32_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_AXI_ADRV9001_H__

/*  EOF: fpga9001_bf_axi_adrv9001.h */

