/*
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "fpga9001_decode_bf_enum.h"

fpga9001_BfAxiTddEnableChanAddr_e fpga9001_TddEnableChanAddr_Get(adi_common_Port_e port,
                                                                 adi_common_ChannelNumber_e channel)
{
    fpga9001_BfAxiTddEnableChanAddr_e addr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0;
    
    if (channel == ADI_CHANNEL_1)
    {
        if (port == ADI_RX)
        {
            addr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0;
        }
        else if (port == ADI_TX)
        {
            addr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0;
        }
        else if (port == ADI_ORX)
        {
            addr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0;
        }
    }
    else if (channel == ADI_CHANNEL_2)
    {
        if (port == ADI_RX)
        {
            addr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1;
        }
        else if (port == ADI_TX)
        {
            addr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1;
        }
        else if (port == ADI_ORX)
        {
            addr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1;
        }
    }
    
    return addr;
}

fpga9001_BfTddDpCtrlChanAddr_e fpga9001_TddDpCtrlChanAddr_Get(fpga9001_BfAxiTddEnableChanAddr_e baseAddr)
{
    switch (baseAddr)
    {
    case FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0:
        return FPGA9001_BF_RX_DP_CTRL_00;
    case FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1:
        return FPGA9001_BF_RX_DP_CTRL_01;
    case FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0:
        return FPGA9001_BF_RX_DP_CTRL_02;
    case FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1:
        return FPGA9001_BF_RX_DP_CTRL_03;
    case FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0:
        return FPGA9001_BF_TX_DP_CTRL_00;
    case FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1:
        return FPGA9001_BF_TX_DP_CTRL_01;
    default:
        return 0;
    }
}