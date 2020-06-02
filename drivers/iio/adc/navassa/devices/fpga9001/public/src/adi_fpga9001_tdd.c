/**
 * \file
 * \brief Contains top level TDD fpga9001 related functions
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include <stdbool.h>
#include "adi_adrv9001_user.h"
#include "adi_common_error.h"
#include "adi_common_macros.h"
#include "adi_fpga9001_user.h"
#include "adi_fpga9001_tdd.h"
#include "adi_fpga9001.h"

#include "fpga9001_bf_tdd_dp_ctrl.h"
#include "fpga9001_bf_axi_tdd_enable.h"
#include "fpga9001_bf_axi_tdd_frame.h"
#include "fpga9001_decode_bf_enum.h"

static int32_t adi_fpga9001_Channel_Validate(adi_fpga9001_Device_t *device,
                                             adi_common_ChannelNumber_e channel)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Port_Validate(adi_fpga9001_Device_t *device,
                                          adi_common_Port_e port)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_ELB);
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ManualEnable_Set_Validate(adi_fpga9001_Device_t *device,
                                                      adi_common_Port_e port,
                                                      adi_common_ChannelNumber_e channel)
{
    ADI_EXPECT(adi_fpga9001_Channel_Validate, device, channel);
    ADI_EXPECT(adi_fpga9001_Port_Validate, device, port);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_ManualEnable_Set(adi_fpga9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         bool enable)
{
    fpga9001_BfAxiTddEnableChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0;
    fpga9001_BfTddDpCtrlChanAddr_e dpBaseAddr = FPGA9001_BF_RX_DP_CTRL_00;

    ADI_PERFORM_VALIDATION(adi_fpga9001_ManualEnable_Set_Validate, device, port, channel);

    if ((port == ADI_ORX) ||
        (port == ADI_ILB) ||
        (port == ADI_ELB))
    {
        port = ADI_ORX;
    }

    baseAddr = fpga9001_TddEnableChanAddr_Get(port, channel);
    dpBaseAddr = fpga9001_TddDpCtrlChanAddr_Get(baseAddr);
    
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimaryenableBfSet, device, baseAddr, false);
    ADI_EXPECT(fpga9001_AxiTddEnablePinsecondaryenableBfSet, device, baseAddr, false);
    ADI_EXPECT(fpga9001_AxiTddEnablePinmanualassertBfSet, device, baseAddr, enable);
    ADI_EXPECT(fpga9001_AxiTddEnableDmamanualassertBfSet, device, baseAddr, enable);
    ADI_EXPECT(fpga9001_TddDpCtrlManualAssertBfSet, device, dpBaseAddr, enable);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ManualEnable_Get_Validate(adi_fpga9001_Device_t *device,
                                                      adi_common_Port_e port,
                                                      adi_common_ChannelNumber_e channel,
                                                      bool *enable)
{
    ADI_EXPECT(adi_fpga9001_Channel_Validate, device, channel);
    ADI_EXPECT(adi_fpga9001_Port_Validate, device, port);
    
    ADI_NULL_PTR_RETURN(&device->common, enable);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_ManualEnable_Get(adi_fpga9001_Device_t *device,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         bool *enable)
{
    fpga9001_BfAxiTddEnableChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_ManualEnable_Get_Validate, device, port, channel, enable);

    if ((port == ADI_ORX) ||
        (port == ADI_ILB) ||
        (port == ADI_ELB))
    {
        port = ADI_ORX;
    }

    baseAddr = fpga9001_TddEnableChanAddr_Get(port, channel);
    
    ADI_EXPECT(fpga9001_AxiTddEnablePinmanualassertBfGet, device, baseAddr, (uint8_t*)enable);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddFraming_Set(adi_fpga9001_Device_t *device, adi_fpga9001_TddFraming_t *framing)
{
    fpga9001_BfAxiTddFrameChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0;

    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodenableBfSet, device, baseAddr, false);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesenableBfSet, device, baseAddr, false);
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddtriggerenableBfSet, device, baseAddr, framing->enableSyncExtTrig);
    ADI_EXPECT(fpga9001_AxiTddFrameTddframesequencerepeatBfSet, device, baseAddr, framing->repeatFrameSequence);
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeswitchtimechannelBfSet, device, baseAddr, framing->frameSwitchTimeChannel);
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeswitchtimeBfSet, device, baseAddr, framing->frameSwitchTime);
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodBfSet, device, baseAddr, framing->framePeriod);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesBfSet, device, baseAddr, framing->numberFrames);
        
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddFraming_Get(adi_fpga9001_Device_t *device, adi_fpga9001_TddFraming_t *framing)
{
    uint8_t bfVal = 0;
    fpga9001_BfAxiTddFrameChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0;

    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodenableBfGet, device, baseAddr, false);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesenableBfGet, device, baseAddr, false);
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddtriggerenableBfGet, device, baseAddr, &bfVal);
    framing->enableSyncExtTrig = (bool)bfVal;
    ADI_EXPECT(fpga9001_AxiTddFrameTddframesequencerepeatBfGet, device, baseAddr, &bfVal);
    framing->repeatFrameSequence = (bool)bfVal;
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeswitchtimechannelBfGet, device, baseAddr, &framing->frameSwitchTimeChannel);
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeswitchtimeBfGet, device, baseAddr, &framing->frameSwitchTime);
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodBfGet, device, baseAddr, &framing->framePeriod);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesBfGet, device, baseAddr, &framing->numberFrames);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddTiming_Set_Validate(adi_fpga9001_Device_t *device,
                                               fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                               adi_fpga9001_TddTiming_t *tddTiming,
                                               uint32_t framePeriod)
{
    ADI_RANGE_CHECK(device, tddTiming->primaryAssert, 1, (framePeriod - 1));
    ADI_RANGE_CHECK(device, tddTiming->primaryDeassert, 1, (framePeriod - 1));
    if (tddTiming->primaryAssert == tddTiming->primaryDeassert)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         tddTiming->primaryDeassert,
                         "Invalid parameter value. tddTiming->primaryDeassert must not equal tddTiming->primaryAssert");
    }
        
    /* Secondary event is optional */
    if (tddTiming->secondaryAssert > 0)
    {
        /* Primary event cannot cross frame boundary if there is a secondary event */
        ADI_RANGE_CHECK(device, tddTiming->primaryDeassert, (tddTiming->primaryAssert + 1), (framePeriod - 1));
        
        /* Secondary event cannot overlap primary event */
        ADI_RANGE_CHECK(device, tddTiming->secondaryAssert, (tddTiming->primaryDeassert + 1), (framePeriod - 1));
        ADI_RANGE_CHECK(device, tddTiming->secondaryDeassert, 1, (framePeriod - 1));
        
        if (tddTiming->secondaryAssert == tddTiming->secondaryDeassert)
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_INV_PARAM,
                             ADI_COMMON_ACT_ERR_CHECK_PARAM,
                             tddTiming->secondaryDeassert,
                             "Invalid parameter value. tddTiming->secondaryDeassert must not equal tddTiming->secondaryAssert");
        }
        
        /* If secondary event crosses frame boundary */
        if (tddTiming->secondaryDeassert < tddTiming->secondaryAssert)
        {
            /* Secondary event cannot overlap primary event */
            if (tddTiming->secondaryDeassert >= tddTiming->primaryAssert)
            {
                ADI_ERROR_REPORT(&device->common,
                                 ADI_COMMON_ERRSRC_API,
                                 ADI_COMMON_ERR_INV_PARAM,
                                 ADI_COMMON_ACT_ERR_CHECK_PARAM,
                                 tddTiming->secondaryDeassert,
                                 "Invalid parameter value. Secondary event cannot overlap primary event, tddTiming->secondaryDeassert must be less than tddTiming->primaryAssert");
            }
        }
    }
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddPinTiming_Set(adi_fpga9001_Device_t *device,
                                         fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                         adi_fpga9001_TddTiming_t *timing,
                                         bool channelEnabled)
{
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimaryenableBfSet, device, baseAddr, channelEnabled);
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimaryassertBfSet, device, baseAddr, timing->primaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimarydeassertBfSet, device, baseAddr, timing->primaryDeassert);
    
    ADI_EXPECT(fpga9001_AxiTddEnablePinsecondaryenableBfSet, device, baseAddr, channelEnabled);
    ADI_EXPECT(fpga9001_AxiTddEnablePinsecondaryassertBfSet, device, baseAddr, timing->secondaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinsecondarydeassertBfSet, device, baseAddr, timing->secondaryDeassert);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddPinTiming_Get(adi_fpga9001_Device_t *device,
                                         fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                         adi_fpga9001_TddTiming_t *timing)
{
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimaryassertBfGet, device, baseAddr, &timing->primaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinprimarydeassertBfGet, device, baseAddr, &timing->primaryDeassert);
    
    ADI_EXPECT(fpga9001_AxiTddEnablePinsecondaryassertBfGet, device, baseAddr, &timing->secondaryAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinsecondarydeassertBfGet, device, baseAddr, &timing->secondaryDeassert);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddTollgateTiming_Set(adi_fpga9001_Device_t *device,
                                              fpga9001_BfTddDpCtrlChanAddr_e baseAddr,
                                              adi_fpga9001_TddTiming_t *timing,
                                              bool channelEnabled)
{
    ADI_EXPECT(fpga9001_TddDpCtrlPrimaryEnableBfSet, device, baseAddr, channelEnabled);
    ADI_EXPECT(fpga9001_TddDpCtrlPrimaryAssertBfSet, device, baseAddr, timing->primaryAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlPrimaryDeassertBfSet, device, baseAddr, timing->primaryDeassert);
    
    ADI_EXPECT(fpga9001_TddDpCtrlSecondaryEnableBfSet, device, baseAddr, channelEnabled);
    ADI_EXPECT(fpga9001_TddDpCtrlSecondaryAssertBfSet, device, baseAddr, timing->secondaryAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlSecondaryDeassertBfSet, device, baseAddr, timing->secondaryDeassert);
      
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddTollgateTiming_Get(adi_fpga9001_Device_t *device,
                                         fpga9001_BfTddDpCtrlChanAddr_e baseAddr,
                                         adi_fpga9001_TddTiming_t *timing)
{
    ADI_EXPECT(fpga9001_TddDpCtrlPrimaryAssertBfGet, device, baseAddr, &timing->primaryAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlPrimaryDeassertBfGet, device, baseAddr, &timing->primaryDeassert);
    
    ADI_EXPECT(fpga9001_TddDpCtrlSecondaryAssertBfGet, device, baseAddr, &timing->secondaryAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlSecondaryDeassertBfGet, device, baseAddr, &timing->secondaryDeassert);
      
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddSequence_Set_Validate(adi_fpga9001_Device_t *device,
                                                 fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                                 adi_fpga9001_TddSequence_t *sequence,
                                                 uint32_t numberFrames)
{
    /* Only applies for a non-zero numberFrames */
    if (numberFrames > 0)
    {
        /* All assert/deasserts must be less than or equal to numberFrames */
        /* At least 1 frame must be active */
        ADI_RANGE_CHECK(device, sequence->primaryFrameAssert, 1, numberFrames);
        
        /* Deassert must not come before assert */
        ADI_RANGE_CHECK(device, sequence->primaryFrameDeassert, sequence->primaryFrameAssert, numberFrames);
        
        /* Secondary is not required */
        if (sequence->secondaryFrameAssert > 0)
        {
            /* Secondary assert must come after primary deassert */
            ADI_RANGE_CHECK(device, sequence->secondaryFrameAssert, sequence->primaryFrameDeassert + 1, numberFrames);
            
            /* Deassert must not come before assert */
            ADI_RANGE_CHECK(device, sequence->secondaryFrameDeassert, sequence->secondaryFrameAssert, numberFrames);
        }
    }
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddPinSequence_Set(adi_fpga9001_Device_t *device,
                                           fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                           adi_fpga9001_TddSequence_t *sequence)
{
    ADI_EXPECT(fpga9001_AxiTddEnablePinframeprimaryassertBfSet, device, baseAddr, sequence->primaryFrameAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinframeprimarydeassertBfSet, device, baseAddr, sequence->primaryFrameDeassert);
    
    ADI_EXPECT(fpga9001_AxiTddEnablePinframesecondaryassertBfSet, device, baseAddr, sequence->secondaryFrameAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinframesecondarydeassertBfSet, device, baseAddr, sequence->secondaryFrameDeassert);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddPinSequence_Get(adi_fpga9001_Device_t *device,
                                           fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                           adi_fpga9001_TddSequence_t *sequence)
{
    ADI_EXPECT(fpga9001_AxiTddEnablePinframeprimaryassertBfGet, device, baseAddr, &sequence->primaryFrameAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinframeprimarydeassertBfGet, device, baseAddr, &sequence->primaryFrameDeassert);
    
    ADI_EXPECT(fpga9001_AxiTddEnablePinframesecondaryassertBfGet, device, baseAddr, &sequence->secondaryFrameAssert);
    ADI_EXPECT(fpga9001_AxiTddEnablePinframesecondarydeassertBfGet, device, baseAddr, &sequence->secondaryFrameDeassert);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddTollgateSequence_Set(adi_fpga9001_Device_t *device,
                                                fpga9001_BfTddDpCtrlChanAddr_e baseAddr,
                                                adi_fpga9001_TddSequence_t *sequence)
{
    ADI_EXPECT(fpga9001_TddDpCtrlFramePrimaryAssertBfSet, device, baseAddr, sequence->primaryFrameAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlFramePrimaryDeassertBfSet, device, baseAddr, sequence->primaryFrameDeassert);
    
    ADI_EXPECT(fpga9001_TddDpCtrlFrameSecondaryAssertBfSet, device, baseAddr, sequence->secondaryFrameAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlFrameSecondaryDeassertBfSet, device, baseAddr, sequence->secondaryFrameDeassert);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddTollgateSequence_Get(adi_fpga9001_Device_t *device,
                                                fpga9001_BfTddDpCtrlChanAddr_e baseAddr,
                                                adi_fpga9001_TddSequence_t *sequence)
{
    ADI_EXPECT(fpga9001_TddDpCtrlFramePrimaryAssertBfGet, device, baseAddr, &sequence->primaryFrameAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlFramePrimaryDeassertBfGet, device, baseAddr, &sequence->primaryFrameDeassert);
    
    ADI_EXPECT(fpga9001_TddDpCtrlFrameSecondaryAssertBfGet, device, baseAddr, &sequence->secondaryFrameAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlFrameSecondaryDeassertBfGet, device, baseAddr, &sequence->secondaryFrameDeassert);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddChannel_Configure_Validate(adi_fpga9001_Device_t *device,
                                                      fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                                      adi_fpga9001_TddChannel_t *tddChannel)
{
    uint32_t framePeriod = 0;
    uint32_t numberFrames = 0;
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodBfGet, device, FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0, &framePeriod);
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesBfGet, device, FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0, &numberFrames);
    
    ADI_EXPECT(fpga9001_TddTiming_Set_Validate, device, baseAddr, &tddChannel->pinTiming, framePeriod);
    ADI_EXPECT(fpga9001_TddSequence_Set_Validate, device, baseAddr, &tddChannel->pinSequence, numberFrames);

    /* Ignore DMA for GPIO baseAddr */
    if (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0 &&
        baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1)
    {
        ADI_EXPECT(fpga9001_TddTiming_Set_Validate, device, baseAddr, &tddChannel->tollgateTiming, framePeriod);
        ADI_EXPECT(fpga9001_TddSequence_Set_Validate, device, baseAddr, &tddChannel->tollgateSequence, numberFrames);    
    }
        
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddChannel_Configure(adi_fpga9001_Device_t *device,
                                             fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                             adi_fpga9001_TddChannel_t *tddChannel)
{
    fpga9001_BfTddDpCtrlChanAddr_e dpCtrlAddr = FPGA9001_BF_RX_DP_CTRL_00;
    
    ADI_PERFORM_VALIDATION(fpga9001_TddChannel_Configure_Validate, device, baseAddr, tddChannel);
    
    dpCtrlAddr = fpga9001_TddDpCtrlChanAddr_Get(baseAddr);
    
    ADI_EXPECT(fpga9001_TddPinTiming_Set, device, baseAddr, &tddChannel->pinTiming, tddChannel->enable);
    ADI_EXPECT(fpga9001_TddPinSequence_Set, device, baseAddr, &tddChannel->pinSequence);

    /* AUX (txGpioControls) has no associated DMA/datapath */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
        (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
        (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
        (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
        (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
        (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_EXPECT(fpga9001_TddTollgateTiming_Set, device, dpCtrlAddr, &tddChannel->tollgateTiming, tddChannel->enable);
        ADI_EXPECT(fpga9001_TddTollgateSequence_Set, device, dpCtrlAddr, &tddChannel->tollgateSequence);
    }
    
    ADI_EXPECT(fpga9001_AxiTddEnableDmamanualassertBfSet, device, baseAddr, tddChannel->enable);
    
    ADI_API_RETURN(device);
}

static int32_t fpga9001_TddChannel_Inspect(adi_fpga9001_Device_t *device,
                                           fpga9001_BfAxiTddEnableChanAddr_e baseAddr,
                                           adi_fpga9001_TddChannel_t *tddChannel)
{
    fpga9001_BfTddDpCtrlChanAddr_e dpCtrlAddr = fpga9001_TddDpCtrlChanAddr_Get(baseAddr);

    ADI_EXPECT(fpga9001_TddPinTiming_Get, device, baseAddr, &tddChannel->pinTiming);
    ADI_EXPECT(fpga9001_TddPinSequence_Get, device, baseAddr, &tddChannel->pinSequence);

    /* AUX (txGpioControls) has no associated DMA/datapath */
    if (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0 &&
        baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1)
    {
        ADI_EXPECT(fpga9001_TddTollgateTiming_Get, device, dpCtrlAddr, &tddChannel->tollgateTiming);
        ADI_EXPECT(fpga9001_TddTollgateSequence_Get, device, dpCtrlAddr, &tddChannel->tollgateSequence);
    }
    
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Tdd_Configure_Validate(adi_fpga9001_Device_t *device,
                                                   adi_fpga9001_TddConfig_t *tddConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, tddConfig);
    
    if (tddConfig->framing.framePeriod == 0)
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_API, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         tddConfig->framing.framePeriod,
                         "Invalid parameter value. tddConfig->framing.framePeriod must be greater than 0");
    }
    
    ADI_RANGE_CHECK(device, tddConfig->framing.frameSwitchTime, 0, tddConfig->framing.framePeriod);
    
    if ((0 == tddConfig->framing.numberFrames) &&
        (true == tddConfig->framing.repeatFrameSequence))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         tddConfig->framing.repeatFrameSequence,
                         "Invalid parameter value. tddConfig->framing.repeatFrameSequence cannot be true when numberFrames is 0");
    }
    
    /* TODO: MMCM must be locked */
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_Configure(adi_fpga9001_Device_t *device, adi_fpga9001_TddConfig_t *tddConfig)
{
    ADI_PERFORM_VALIDATION(adi_fpga9001_Tdd_Configure_Validate, device, tddConfig);
    
    ADI_EXPECT(fpga9001_TddFraming_Set, device, &tddConfig->framing);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0, &tddConfig->rxControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1, &tddConfig->rxControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0, &tddConfig->orxControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1, &tddConfig->orxControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0, &tddConfig->txControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1, &tddConfig->txControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0, &tddConfig->txGpioControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1, &tddConfig->txGpioControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1, &tddConfig->interFrameControlSignal1[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2, &tddConfig->interFrameControlSignal1[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1, &tddConfig->interFrameControlSignal2[0]);
    ADI_EXPECT(fpga9001_TddChannel_Configure, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2, &tddConfig->interFrameControlSignal2[1]);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_Inspect(adi_fpga9001_Device_t *device, adi_fpga9001_TddConfig_t *tddConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, tddConfig);
    
    ADI_EXPECT(fpga9001_TddFraming_Get, device, &tddConfig->framing);
    
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0, &tddConfig->rxControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1, &tddConfig->rxControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0, &tddConfig->orxControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1, &tddConfig->orxControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0, &tddConfig->txControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1, &tddConfig->txControls[1]);
    
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0, &tddConfig->txGpioControls[0]);
    ADI_EXPECT(fpga9001_TddChannel_Inspect, device, FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1, &tddConfig->txGpioControls[1]);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_ProgrammedEnable_Set(adi_fpga9001_Device_t *device, bool enable)
{
    fpga9001_BfAxiTddFrameChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0;
    uint32_t numberFrames = 0;
    
    ADI_API_ENTRY_EXPECT(device);
    /* TODO: MMCM must be locked */
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddframecountersresetBfSet, device, baseAddr, true);
    
    /* Only set TddNumberFramesEnable if TddNumberFrames > 0*/
    ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesBfGet, device, baseAddr, &numberFrames);
    if (numberFrames > 0)
    {
        ADI_EXPECT(fpga9001_AxiTddFrameTddnumberframesenableBfSet, device, baseAddr, enable);
    }
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodenableBfSet, device, baseAddr, enable);
    
    /* FIXME: To disable, is it necessary to disable individual enable control modules and datapath control modules, as in Tokelau? */
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Tdd_ProgrammedEnable_Get(adi_fpga9001_Device_t *device, bool *enable)
{
    fpga9001_BfAxiTddFrameChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0;
    uint8_t bfValue = 0;
    
    ADI_API_ENTRY_PTR_EXPECT(device, enable);
    
    ADI_EXPECT(fpga9001_AxiTddFrameTddframeperiodenableBfGet, device, baseAddr, &bfValue);
    *enable = (bool)bfValue;
    
    ADI_API_RETURN(device);
}
