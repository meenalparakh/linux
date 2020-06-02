/**
 * \file fpga9001_bf_tdd_dp_ctrl.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_TDD_DP_CTRL_H__
#define __FPGA9001_BF_TDD_DP_CTRL_H__

#include "fpga9001_bf_tdd_dp_ctrl_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_TddDpCtrlDmaEnableBfSet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t fpga9001_TddDpCtrlDmaEnableBfGet(adi_fpga9001_Device_t *device, 
                                         fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t fpga9001_TddDpCtrlFramePrimaryAssertBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                  uint32_t bfValue);
int32_t fpga9001_TddDpCtrlFramePrimaryAssertBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t fpga9001_TddDpCtrlFramePrimaryDeassertBfSet(adi_fpga9001_Device_t *device, 
                                                    fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t fpga9001_TddDpCtrlFramePrimaryDeassertBfGet(adi_fpga9001_Device_t *device, 
                                                    fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t fpga9001_TddDpCtrlFrameSecondaryAssertBfSet(adi_fpga9001_Device_t *device, 
                                                    fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                    uint32_t bfValue);
int32_t fpga9001_TddDpCtrlFrameSecondaryAssertBfGet(adi_fpga9001_Device_t *device, 
                                                    fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                    uint32_t *bfValue);


int32_t fpga9001_TddDpCtrlFrameSecondaryDeassertBfSet(adi_fpga9001_Device_t *device, 
                                                      fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                      uint32_t bfValue);
int32_t fpga9001_TddDpCtrlFrameSecondaryDeassertBfGet(adi_fpga9001_Device_t *device, 
                                                      fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                      uint32_t *bfValue);


int32_t fpga9001_TddDpCtrlManualAssertBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t fpga9001_TddDpCtrlManualAssertBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_TddDpCtrlPrimaryAssertBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                             uint32_t bfValue);
int32_t fpga9001_TddDpCtrlPrimaryAssertBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t fpga9001_TddDpCtrlPrimaryDeassertBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t fpga9001_TddDpCtrlPrimaryDeassertBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t fpga9001_TddDpCtrlPrimaryEnableBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_TddDpCtrlPrimaryEnableBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_TddDpCtrlSecondaryAssertBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t fpga9001_TddDpCtrlSecondaryAssertBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t fpga9001_TddDpCtrlSecondaryDeassertBfSet(adi_fpga9001_Device_t *device, 
                                                 fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                 uint32_t bfValue);
int32_t fpga9001_TddDpCtrlSecondaryDeassertBfGet(adi_fpga9001_Device_t *device, 
                                                 fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                                 uint32_t *bfValue);


int32_t fpga9001_TddDpCtrlSecondaryEnableBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_TddDpCtrlSecondaryEnableBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_TddDpCtrlVersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t fpga9001_TddDpCtrlVersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfTddDpCtrlChanAddr_e baseAddr, 
                                            uint16_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_TDD_DP_CTRL_H__

/*  EOF: fpga9001_bf_tdd_dp_ctrl.h */

