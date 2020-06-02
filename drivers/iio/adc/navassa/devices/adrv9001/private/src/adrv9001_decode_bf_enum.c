/*
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adrv9001_decode_bf_enum.h"

adrv9001_BfNvsRegmapRxChanAddr_e adrv9001_RxChanAddr_Get(adi_common_ChannelNumber_e channel)
{
    switch (channel)
    {
    case ADI_CHANNEL_1:
        return ADRV9001_BF_RX1_CORE;
    case ADI_CHANNEL_2:
        return ADRV9001_BF_RX2_CORE;
    default:
        return 0;
    }
}

adrv9001_BfNvsRegmapRxbChanAddr_e adrv9001_RxbChanAddr_Get(adi_common_ChannelNumber_e channel)
{
    switch (channel)
    {
    case ADI_CHANNEL_1:
        return ADRV9001_BF_RXB1_CORE;
    case ADI_CHANNEL_2:
        return ADRV9001_BF_RXB2_CORE;
    default:
        return 0;
    }
}

adrv9001_BfAnalogRxMemMapChanAddr_e adrv9001_AnalogRxChanAddr_Get(adi_common_ChannelNumber_e channel)
{
    switch (channel)
    {
    case ADI_CHANNEL_1:
        return ADRV9001_BF_RX1_ANA;
    case ADI_CHANNEL_2:
        return ADRV9001_BF_RX2_ANA;
    default:
        return 0;
    }
}

adrv9001_BfNvsPllMemMapChanAddr_e adrv9001_PllMemMapAddr_Get(adi_adrv9001_ClkPllMode_e mode, adi_adrv9001_PllType_e pllType)
{
    switch (pllType)
    {
    case ADI_ADRV9001_PLL_TYPE_CLK:
        if (ADI_ADRV9001_CLK_PLL_LP_MODE == mode)
        {
            return ADRV9001_BF_CLK_PLL_LP;
        }
        else if (ADI_ADRV9001_CLK_PLL_HP_MODE == mode)
        {
            return ADRV9001_BF_CLK_PLL;
        }
        else
        {
            return 0;
        }
    case ADI_ADRV9001_PLL_TYPE_RF1:
        return ADRV9001_BF_RF1_PLL;
    case ADI_ADRV9001_PLL_TYPE_RF2:
        return ADRV9001_BF_RF2_PLL;
    case ADI_ADRV9001_PLL_TYPE_AUX:
        return ADRV9001_BF_AUX_PLL;
    default: 
        return 0;
    }
}