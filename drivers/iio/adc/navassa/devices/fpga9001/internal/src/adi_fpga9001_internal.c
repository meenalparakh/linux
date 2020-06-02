
/**
 * \file
 * \brief ADI internal-only FPGA9001 functions
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include <stdlib.h>
#include "adi_adrv9001_user.h"
#include "adi_common_error.h"
#include "adi_fpga9001_gpio.h"
#include "adi_fpga9001_hal.h"
#include "adi_fpga9001_internal.h"
#include "adi_fpga9001_tdd.h"
#include "fpga9001_bf_axi_adrv9001.h"
#include "fpga9001_bf_axi_tdd_enable.h"
#include "fpga9001_bf_dp_capture_control.h"
#include "fpga9001_bf_dp_rx_dma.h"
#include "fpga9001_bf_dp_tollgate.h"
#include "fpga9001_bf_tdd_dp_ctrl.h"
#include "fpga9001_decode_bf_enum.h"

static int32_t adi_fpga9001_internal_SsiGpio_Configure(adi_fpga9001_Device_t *device,
                                                       uint8_t bfValue,
                                                       adi_fpga9001_internal_ObservationGpioCfg_t *observationGpioConfig)
{
    uint8_t idx = 0;

    for (idx = 0; idx < 8; idx++)
    {
        switch (observationGpioConfig->ssiGpioPins[idx])
        {
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_00:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl00BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_01:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl01BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_02:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl02BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_03:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl03BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_04:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl04BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_05:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl05BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_06:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl06BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_07:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl07BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_08:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl08BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_09:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl09BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_10:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl10BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_11:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl11BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_12:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl12BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_13:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl13BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_14:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl14BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_15:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl15BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_16:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl16BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_17:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl17BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_18:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl18BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_19:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl19BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_20:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl20BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_21:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl21BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_22:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl22BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_23:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl23BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_24:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl24BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_25:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl25BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_26:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl26BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_27:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl27BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_28:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl28BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_29:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl29BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_30:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl30BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        case ADI_FPGA9001_INTERNAL_SSI_GPIO_31:
            ADI_EXPECT(fpga9001_AxiAdrv9001DgpioSsictl31BfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, bfValue);
            break;
        default:
            /* Should never actually reach here */
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, "ssiGpioPins", "Invalid ssiGpioPins selection");
        }
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_internal_ObservationGpioGet_Validate(adi_fpga9001_Device_t *device,
                                                                 uint32_t *obsData)
{
    ADI_NULL_DEVICE_PTR_RETURN(device);
    ADI_NULL_PTR_RETURN(&device->common, obsData);
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_internal_ObservationGpio_Configure_Validate(adi_fpga9001_Device_t *device,
                                                                        adi_fpga9001_internal_ObservationGpioCfg_t *observationGpioConfig)
{
    uint8_t i = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);
    ADI_NULL_PTR_RETURN(&device->common, observationGpioConfig);

    for (i = 0; i < 8; i++)
    {
        ADI_RANGE_CHECK(device, observationGpioConfig->ssiGpioPins[i], ADI_FPGA9001_INTERNAL_SSI_GPIO_00, ADI_FPGA9001_INTERNAL_SSI_GPIO_ALL);
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_internal_DataChain_DmaRun_Set(adi_fpga9001_Device_t *device,
                                                          adi_common_ChannelNumber_e channel)
{
    uint8_t dmaActive = 0;
    uint8_t haltComplete = 0;

    ADI_EXPECT(fpga9001_DpRxDmaActiveBfGet, device, FPGA9001_BF_RX_DP_DMA_04, &dmaActive);

    ADI_EXPECT(fpga9001_DpRxDmaRunStopBfSet, device, FPGA9001_BF_RX_DP_DMA_04, false);

    if (1 == dmaActive)
    {
        while (0 == haltComplete)
        {
            ADI_EXPECT(fpga9001_DpRxDmaHaltCompleteBfGet, device, FPGA9001_BF_RX_DP_DMA_04, &haltComplete);
        }
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_internal_DataChain_Dma_Configure(adi_fpga9001_Device_t *device,
                                                             adi_common_ChannelNumber_e channel,
                                                             adi_fpga9001_DmaCfg_t *dmaCfg)
{
    uint32_t regVal = 0;
    static const uint8_t CONTROL_REG_OFFSET = 0x8;
    static const uint8_t VALID_DATA_WIDTH_OFFSET = 8;
    static const uint8_t VALID_DATA_WIDTH_MASK = 0xF;
    static const uint8_t ENABLE_ENHANCED_OFFSET = 3;
    static const uint8_t ENABLE_ENHANCED_MASK = 0x1;
    static const uint8_t ENABLE_SG_OFFSET = 2;
    static const uint8_t ENABLE_SG_MASK = 0x1;
    static const uint8_t CONTINUOUS_OFFSET = 1;
    static const uint8_t CONTINUOUS_MASK = 0x1;
    static const uint8_t RUN_STOP_OFFSET = 0;
    static const uint8_t RUN_STOP_MASK = 0x1;

    /* DMA start address and length must be multiples of 64 */
    if (dmaCfg->simpleStartAddr % 64 != 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            dmaCfg->simpleStartAddr,
            "Invalid parameter value. dmaCfg->simpleStartAddr must be a multiple of 64");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    if (dmaCfg->length % 64 != 0)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            dmaCfg->length,
            "Invalid parameter value. dmaCfg->length must be a multiple of 64");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Validate dmaCfg->validDataWidthStream */
    switch (dmaCfg->validDataWidthStream)
    {
    case ADI_FPGA9001_STREAM_32_BITS:   /* Falls through */
    case ADI_FPGA9001_STREAM_64_BITS:   /* Falls through */
    case ADI_FPGA9001_STREAM_128_BITS:  /* Falls through */
    case ADI_FPGA9001_STREAM_256_BITS:  /* Falls through */
    case ADI_FPGA9001_STREAM_512_BITS:  /* Falls through */
        break;
    default:
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            dmaCfg->validDataWidthStream,
            "Invalid parameter value. dmaCfg->validDataWidthStream must be of adi_fpga9001_ValidDataWidth_e type.");
    }

    /* TODO: start address could be changed at runtime by this function, and this length check is not enough to ensure RAM buffers do not overlap */
    if (dmaCfg->length > RX_DMA_SIZE)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            dmaCfg->length,
            "FPGA DMA capture length is too large for RAM buffer area");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_EXPECT(fpga9001_DpRxDmaSimpleStartAddrBfSet, device, FPGA9001_BF_RX_DP_DMA_04, dmaCfg->simpleStartAddr);

    ADI_EXPECT(fpga9001_DpRxDmaLengthBfSet, device, FPGA9001_BF_RX_DP_DMA_04, dmaCfg->length);

    /* Write the DMA control register */
    regVal = ((dmaCfg->validDataWidthStream & VALID_DATA_WIDTH_MASK)    << VALID_DATA_WIDTH_OFFSET) |
             ((dmaCfg->enableEnhancedMode   & ENABLE_ENHANCED_MASK)     << ENABLE_ENHANCED_OFFSET)  |
             ((dmaCfg->enableSg             & ENABLE_SG_MASK)           << ENABLE_SG_OFFSET)        |
             ((dmaCfg->continuous           & CONTINUOUS_MASK)          << CONTINUOUS_OFFSET)       |
             ((dmaCfg->runStop              & RUN_STOP_MASK)            << RUN_STOP_OFFSET);

    ADI_EXPECT(adi_fpga9001_hal_Register_Write, device, FPGA9001_BF_RX_DP_DMA_04 + CONTROL_REG_OFFSET, regVal);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_internal_DataChain_Configure(adi_fpga9001_Device_t *device,
                                                         adi_common_ChannelNumber_e channel,
                                                         adi_fpga9001_TollgateCfg_t *tollgateCfg,
                                                         adi_fpga9001_DmaCfg_t *dmaCfg)
{
    ADI_EXPECT(adi_fpga9001_internal_DataChain_DmaRun_Set, device, channel);
    
    /* Capture control datapath active */
    fpga9001_DpCaptureControlDatapathActiveBfSet(device, FPGA9001_BF_RX_DP_CAPTURE_CONTROL, 1);

    /* Set up tollgate */
    ADI_EXPECT(fpga9001_DpTollgateTriggerSelectBfSet, device, FPGA9001_BF_RX_DP_TOLLGATE_04, tollgateCfg->tollGateTrigSource);

    /* Configure DMAs */
    ADI_EXPECT(adi_fpga9001_internal_DataChain_Dma_Configure, device, channel, dmaCfg);

    /* Reset capture control */
    ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_RX_DP_CAPTURE_CONTROL, 1);

    /* Reset tollgate */
    ADI_EXPECT(fpga9001_DpTollgateResetTollgateBfSet, device, FPGA9001_BF_RX_DP_TOLLGATE_04, 1);

    /* Reset DMAs */
    ADI_EXPECT(fpga9001_DpRxDmaResetBfSet, device, FPGA9001_BF_RX_DP_DMA_04, 1);

    /* Un-reset capture control */
    ADI_EXPECT(fpga9001_DpCaptureControlResetBfSet, device, FPGA9001_BF_RX_DP_CAPTURE_CONTROL, 0);

    /* Un-reset tollgate */
    ADI_EXPECT(fpga9001_DpTollgateResetTollgateBfSet, device, FPGA9001_BF_RX_DP_TOLLGATE_04, 0);

    /* Un-reset DMAs */
    ADI_EXPECT(fpga9001_DpRxDmaResetBfSet, device, FPGA9001_BF_RX_DP_DMA_04, 0);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_internal_DataChain_RxCapture_Wait(adi_fpga9001_Device_t *device,
                                                              adi_common_ChannelNumber_e channel,
                                                              uint32_t timeout_ms)
{
    uint8_t bfVal = 0;
    uint32_t i = 0;

    for (i = 0; i < timeout_ms; i++)
    {
        ADI_EXPECT(fpga9001_DpRxDmaCompleteBfGet, device, FPGA9001_BF_RX_DP_DMA_04, &bfVal);

        if (bfVal == 1)
        {
            return ADI_COMMON_ACT_NO_ACTION;
        }

        adi_common_hal_Wait_us(&device->common, 1000);
    }

    ADI_ERROR_REPORT(&device->common,
        ADI_COMMON_ERRSRC_API,
        ADI_FPGA9001_ERR_RX_DATA_MOVER_WAIT_TIMEOUT,
        ADI_COMMON_ACT_ERR_RESET_FEATURE,
        NULL,
        "internal_DataChain_RxCaptureWait timeout");

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_internal_ObservationGpio_Get(adi_fpga9001_Device_t *device,
                                                  uint32_t *obsData)
{
    uint32_t regVal = 0;

    static uint32_t ADI_FPGA9001_DGPIO_SSI_DATA_READ_ADDR = 0x430500A0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_internal_ObservationGpioGet_Validate, device, obsData);

    ADI_EXPECT(adi_fpga9001_hal_Register_Read, device, ADI_FPGA9001_DGPIO_SSI_DATA_READ_ADDR, &regVal)

    *obsData = regVal;

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_internal_ObservationGpio_Configure(adi_fpga9001_Device_t *device,
                                                        adi_fpga9001_internal_ObservationGpioCfg_t *observationGpioConfig)
{
    uint8_t bfValue = 0;
    static uint8_t DGPIO_SSI_GAIN_INDEX_VALUE = 0x2;

    /* if 'true', then configure SSI GPIO to read gain_index */
    if (observationGpioConfig->readGainIndex)
    {
        bfValue |= DGPIO_SSI_GAIN_INDEX_VALUE;
    }

    ADI_PERFORM_VALIDATION(adi_fpga9001_internal_ObservationGpio_Configure_Validate, device, observationGpioConfig);

    ADI_EXPECT(adi_fpga9001_internal_SsiGpio_Configure, device, bfValue, observationGpioConfig);

    /* Tollgate timing set */
    ADI_EXPECT(fpga9001_TddDpCtrlPrimaryEnableBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.enable);
    ADI_EXPECT(fpga9001_TddDpCtrlPrimaryAssertBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.tollgateTiming.primaryAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlPrimaryDeassertBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.tollgateTiming.primaryDeassert);
    
    ADI_EXPECT(fpga9001_TddDpCtrlSecondaryEnableBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.enable);
    ADI_EXPECT(fpga9001_TddDpCtrlSecondaryAssertBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.tollgateTiming.secondaryAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlSecondaryDeassertBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.tollgateTiming.secondaryDeassert);

    /* Tollgate sequence set */
    ADI_EXPECT(fpga9001_TddDpCtrlFramePrimaryAssertBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.tollgateSequence.primaryFrameAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlFramePrimaryDeassertBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.tollgateSequence.primaryFrameDeassert);
    
    ADI_EXPECT(fpga9001_TddDpCtrlFrameSecondaryAssertBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.tollgateSequence.secondaryFrameAssert);
    ADI_EXPECT(fpga9001_TddDpCtrlFrameSecondaryDeassertBfSet, device, FPGA9001_BF_RX_DP_CTRL_04, observationGpioConfig->observationGpioControls.tollgateSequence.secondaryFrameDeassert);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_internal_GainIndex_Capture(adi_fpga9001_Device_t *device,
                                                adi_common_ChannelNumber_e channel,
                                                uint32_t timeout_ms,
                                                uint32_t numBytes,
                                                adi_fpga9001_TollgateTriggerSources_e trigger)
{
    uint32_t ramAddress = 0;

    adi_fpga9001_TollgateCfg_t tollgateCfg = {
        .tollGateTrigSource = trigger
    };

    adi_fpga9001_DmaCfg_t dmaCfg = {
        .enableEnhancedMode = 0,
        .length = numBytes,
        .validDataWidthStream = ADI_FPGA9001_STREAM_32_BITS,
        .runStop = 1
    };

    ADI_API_ENTRY_EXPECT(device);

    if (ADI_CHANNEL_1 == channel)
    {
        /* To capture Rx1 gain_index(using Rx2 SSI pins), Rx2 RAM location is used*/
        ramAddress = ADI_FPGA9001_RX2_ADDR_OFFSET;
    }
    else if (ADI_CHANNEL_2 == channel)
    {
        /* To capture Rx2 gain_index(using Rx1 SSI pins), Rx1 RAM location is used*/
        ramAddress = ADI_FPGA9001_RX1_ADDR_OFFSET;
    }
    else
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    dmaCfg.simpleStartAddr = ADI_FPGA9001_RAM_START_ADDR + ramAddress;

    ADI_EXPECT(adi_fpga9001_internal_DataChain_Configure, device, channel, &tollgateCfg, &dmaCfg);
    ADI_EXPECT(adi_fpga9001_internal_DataChain_RxCapture_Wait, device, channel, timeout_ms);

    ADI_API_RETURN(device);
}
