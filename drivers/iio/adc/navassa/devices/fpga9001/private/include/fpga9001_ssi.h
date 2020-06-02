/**
 * \file
 *
 * \brief Helper functions for FPGA9001 Synchronous Serial Interface (SSI) configuration
 *
 * These functions call the appropriate SSI Rx/Tx bitfield set/get functions
 * based on the ssiSel/baseAddr
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _FPGA9001_SSI_H_
#define _FPGA9001_SSI_H_

#include <stdint.h>
#include "adi_fpga9001_types.h"
#include "adi_fpga9001_ssi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t fpga9001_BfSsiChanAddrGet(adi_fpga9001_Device_t *device,
                                   adi_common_Port_e port,
                                   adi_common_ChannelNumber_e channel);

#ifdef __cplusplus
}
#endif

#endif /* _FPGA9001_SSI_H_ */