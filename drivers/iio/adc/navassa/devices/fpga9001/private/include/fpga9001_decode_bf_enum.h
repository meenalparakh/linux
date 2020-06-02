/*
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _FPGA9001_DECODE_BF_ENUM_H_
#define _FPGA9001_DECODE_BF_ENUM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_common_types.h"
#include "fpga9001_bf_axi_tdd_enable_types.h"
#include "fpga9001_bf_tdd_dp_ctrl_types.h"
    
fpga9001_BfAxiTddEnableChanAddr_e fpga9001_TddEnableChanAddr_Get(adi_common_Port_e port,
                                                                 adi_common_ChannelNumber_e channel);

fpga9001_BfTddDpCtrlChanAddr_e fpga9001_TddDpCtrlChanAddr_Get(fpga9001_BfAxiTddEnableChanAddr_e baseAddr);

#ifdef __cplusplus
}
#endif

#endif