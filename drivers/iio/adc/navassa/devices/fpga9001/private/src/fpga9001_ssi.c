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

#include  "fpga9001_ssi.h"
#include "fpga9001_bf_axi_adrv9001_rx.h"
#include "fpga9001_bf_axi_adrv9001_tx.h"

uint32_t fpga9001_BfSsiChanAddrGet(adi_fpga9001_Device_t *device,
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel)
{
    if ((ADI_RX == port && ADI_CHANNEL_1 == channel)  ||
        (ADI_ORX == port && ADI_CHANNEL_1 == channel) ||
        (ADI_ILB == port && ADI_CHANNEL_1 == channel) ||
        (ADI_ELB == port && ADI_CHANNEL_1 == channel))
    {
        return FPGA9001_BF_AXI_ADRV9001_RX_0;
    }
    else if ((ADI_RX == port && ADI_CHANNEL_2 == channel)  ||
             (ADI_ORX == port && ADI_CHANNEL_2 == channel) ||
             (ADI_ILB == port && ADI_CHANNEL_2 == channel) ||
             (ADI_ELB == port && ADI_CHANNEL_2 == channel))
    {
        return FPGA9001_BF_AXI_ADRV9001_RX_1;
    }
    else if (ADI_TX == port && ADI_CHANNEL_1 == channel)
    {
        return FPGA9001_BF_AXI_ADRV9001_TX_0;
    }
    else if (ADI_TX == port && ADI_CHANNEL_2 == channel)
    {
        return FPGA9001_BF_AXI_ADRV9001_TX_1;
    }
    else
    {
        return 0;
    }
}
