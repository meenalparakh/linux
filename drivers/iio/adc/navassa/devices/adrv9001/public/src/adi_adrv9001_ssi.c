/**
* \file
* \brief Contains SSI calibration related function implementation defined in
* adi_adrv9001_ssi.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_user.h"
#include "adi_adrv9001_ssi.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_spi.h"
#include "adrv9001_init.h"
#include "adrv9001_reg_addr_macros.h"
#include "adrv9001_bf_nvs_regmap_rx.h"
#include "adrv9001_bf_nvs_regmap_tx.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_bf_nvs_regmap_core_3.h"

static int32_t __maybe_unused adi_adrv9001_Ssi_Rx_TestMode_Configure_Validate(adi_adrv9001_Device_t *device,
									      adi_common_ChannelNumber_e channel,
									      adi_adrv9001_SsiType_e ssiType,
									      adi_adrv9001_SsiDataFormat_e dataFormat,
									      adi_adrv9001_RxSsiTestModeCfg_t *ssiTestModeConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, ssiTestModeConfig);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_RANGE_CHECK(device, ssiType, ADI_ADRV9001_SSI_TYPE_CMOS, ADI_ADRV9001_SSI_TYPE_LVDS);
    ADI_RANGE_CHECK(device, dataFormat, ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA, ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA);

    if (ADI_ADRV9001_SSI_TYPE_CMOS == ssiType)
    {
        ADI_RANGE_CHECK(device, ssiTestModeConfig->testData, ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL, ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE);
    }
    else
    {
        ADI_RANGE_CHECK(device, ssiTestModeConfig->testData, ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL, ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7);
    }

    if ((ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN != ssiTestModeConfig->testData) &&
        (dataFormat != ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            dataFormat,
            "Invalid SSI data format for the selected SSI calibration test type");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Ssi_Rx_TestMode_Configure(adi_adrv9001_Device_t *device,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_SsiType_e ssiType,
                                               adi_adrv9001_SsiDataFormat_e dataFormat,
                                               adi_adrv9001_RxSsiTestModeCfg_t *ssiTestModeConfig)
{
    int8_t i = 0;
    adrv9001_BfNvsRegmapRxChanAddr_e baseAddress = ADRV9001_BF_RX1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Ssi_Rx_TestMode_Configure_Validate, device, channel, ssiType, dataFormat, ssiTestModeConfig);

    if (channel == ADI_CHANNEL_2)
    {
        baseAddress = ADRV9001_BF_RX2_CORE;
    }

    if ((ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN != ssiTestModeConfig->testData) &&
        (dataFormat != ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            dataFormat,
            "Invalid SSI data format for the selected SSI calibration test type");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (ADI_ADRV9001_SSI_TYPE_CMOS == ssiType)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxCssiRxDebugModeBfSet, device, baseAddress, 0x1);
        if (ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE == ssiTestModeConfig->testData)
        {
            ADI_EXPECT(adrv9001_NvsRegmapRxCssiRxDebugStartRampBfSet, device, baseAddress, 0x1);
        }
        else
        {
            ADI_EXPECT(adrv9001_NvsRegmapRxCssiRxDebugStartRampBfSet, device, baseAddress, 0x0);

            for (i = 0; i < 2; i++)
            {
                ADI_EXPECT(adrv9001_NvsRegmapRxCssiRxDebugQSelBfSet, device, baseAddress, i);   // 0: I_data; 1: Q_data
                ADI_EXPECT(adrv9001_NvsRegmapRxCssiRxDebugNibbleSelBfSet, device, baseAddress, 0x3);
                ADI_EXPECT(adrv9001_NvsRegmapRxCssiRxDebugLoadValueBfSet, device, baseAddress, (uint8_t)(ssiTestModeConfig->fixedDataPatternToTransmit & 0xF));
                ADI_EXPECT(adrv9001_NvsRegmapRxCssiRxDebugLoadBfSet, device, baseAddress, 0x1);
                ADI_EXPECT(adrv9001_NvsRegmapRxCssiRxDebugLoadBfSet, device, baseAddress, 0x0);
            }
        }
    }
    else /* LVDS */
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugModeBfSet, device, baseAddress, 0x1);
        if (ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_16_BIT == ssiTestModeConfig->testData)
        {
            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugStartRampBfSet, device, baseAddress, 0x1);
        }
        else if (ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7 == ssiTestModeConfig->testData)
        {

            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugPrbs7RestartBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugPrbs7EnableBfSet, device, baseAddress, 0x1);
        }
        else if (ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15 == ssiTestModeConfig->testData)
        {
            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugPrbs15RestartBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugPrbs15EnableBfSet, device, baseAddress, 0x1);

        }
        else /* Fixed Pattern */
        {
            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugStartRampBfSet, device, baseAddress, 0x0);
            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugLoadValueBfSet, device, baseAddress, (uint16_t)(ssiTestModeConfig->fixedDataPatternToTransmit & 0xFFFF));
            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugLoadBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapRxLssiRxDebugLoadBfSet, device, baseAddress, 0x0);
        }
    }

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Ssi_Tx_TestMode_Configure_Validate(adi_adrv9001_Device_t *device,
									      adi_common_ChannelNumber_e channel,
									      adi_adrv9001_SsiType_e ssiType,
									      adi_adrv9001_SsiDataFormat_e dataFormat,
									      adi_adrv9001_TxSsiTestModeCfg_t *ssiTestModeConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, ssiTestModeConfig);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_RANGE_CHECK(device, ssiType, ADI_ADRV9001_SSI_TYPE_CMOS, ADI_ADRV9001_SSI_TYPE_LVDS);
    ADI_RANGE_CHECK(device, dataFormat, ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA, ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA);

    if (ADI_ADRV9001_SSI_TYPE_CMOS == ssiType)
    {
        ADI_RANGE_CHECK(device, ssiTestModeConfig->testData, ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL, ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE);
    }
    else
    {
        ADI_RANGE_CHECK(device, ssiTestModeConfig->testData, ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL, ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7);
    }

    if ((ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN != ssiTestModeConfig->testData) &&
        (dataFormat != ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            dataFormat,
            "Invalid SSI data format for the selected SSI calibration test type");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Ssi_Tx_TestMode_Configure(adi_adrv9001_Device_t *device,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_SsiType_e ssiType,
                                               adi_adrv9001_SsiDataFormat_e dataFormat,
                                               adi_adrv9001_TxSsiTestModeCfg_t *ssiTestModeConfig)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    adrv9001_BfNvsRegmapTxChanAddr_e baseAddress = ADRV9001_BF_TX1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Ssi_Tx_TestMode_Configure_Validate, device, channel, ssiType, dataFormat, ssiTestModeConfig);

    if (channel == ADI_CHANNEL_2)
    {
        baseAddress = ADRV9001_BF_TX2_CORE;
    }

    if (ADI_ADRV9001_SSI_TYPE_CMOS == ssiType)
    {
        ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugModeBfSet, device, baseAddress, 0x1);
        /* Nothing to be configured for fixed pattern */
        if (ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE == ssiTestModeConfig->testData)
        {
            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugStartRampBfSet, device, baseAddress, 0x0);

            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearRampShiftErrorBfSet, device, baseAddress, 0x0);
            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearRampShiftErrorBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearRampShiftErrorBfSet, device, baseAddress, 0x0);

            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxFifoClearBfSet, device, baseAddress, 0x0);
            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxFifoClearBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxFifoClearBfSet, device, baseAddress, 0x0);

            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x0);
            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x0);

            ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugStartRampBfSet, device, baseAddress, 0x1);
        }
    }
    else /* LVDS */
    {
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugModeBfSet, device, baseAddress, 0x1);
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugStartRampBfSet, device, baseAddress, 0x0);
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugStartErrorCheckBfSet, device, baseAddress, 0x0);
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugAfterFifoErrorClearBfSet, device, baseAddress, 0x1);
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugAfterFifoErrorClearBfSet, device, baseAddress, 0x0);
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x1);
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x0);
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxFifoClearBfSet, device, baseAddress, 0x1);
        ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxFifoClearBfSet, device, baseAddress, 0x0);

        /* Nothing to be configured for fixed pattern */
        if (ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_16_BIT == ssiTestModeConfig->testData)
        {
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugStartRampBfSet, device, baseAddress, 0x1);

            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugStartErrorCheckBfSet, device, baseAddress, 0x1);

            recoveryAction = adi_common_hal_Wait_us(&device->common, 1000);

            ADI_ERROR_REPORT(&device->common,
                             ADI_ADRV9001_SRC_ARMCMD,
                             recoveryAction,
                             ADI_COMMON_ACT_ERR_CHECK_TIMER,
                             device,
                             "Timer not working in adi_adrv9001_Ssi_Tx_TestMode_Configure()");
            ADI_ERROR_RETURN(device->common.error.newAction);

            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugErrorCounterReadBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugErrorCounterReadBfSet, device, baseAddress, 0x0);
        }
        else if (ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7 == ssiTestModeConfig->testData)
        {
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugPrbs7EnableBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugPrbs7ErrorClearBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugPrbs7RestartBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugStartErrorCheckBfSet, device, baseAddress, 0x1);
            recoveryAction = adi_common_hal_Wait_us(&device->common, 1000);

            ADI_ERROR_REPORT(&device->common,
                ADI_ADRV9001_SRC_ARMCMD,
                recoveryAction,
                ADI_COMMON_ACT_ERR_CHECK_TIMER,
                device,
                "Timer not working in adi_adrv9001_Ssi_Tx_TestMode_Configure()");
            ADI_ERROR_RETURN(device->common.error.newAction);

            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugErrorCounterReadBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugErrorCounterReadBfSet, device, baseAddress, 0x0);
        }
        else /* ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15 */
        {
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugPrbs15EnableBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugAfterFifoErrorClearBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugPrbs15RestartBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugStartErrorCheckBfSet, device, baseAddress, 0x1);
            recoveryAction = adi_common_hal_Wait_us(&device->common, 1000);

            ADI_ERROR_REPORT(&device->common,
                ADI_ADRV9001_SRC_ARMCMD,
                recoveryAction,
                ADI_COMMON_ACT_ERR_CHECK_TIMER,
                device,
                "Timer not working in adi_adrv9001_Ssi_Tx_TestMode_Configure()");
            ADI_ERROR_RETURN(device->common.error.newAction);

            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugErrorCounterReadBfSet, device, baseAddress, 0x1);
            ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugErrorCounterReadBfSet, device, baseAddress, 0x0);
        }
    }

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect_Validate(adi_adrv9001_Device_t *device,
										   adi_common_ChannelNumber_e channel,
										   adi_adrv9001_SsiType_e ssiType,
										   adi_adrv9001_SsiDataFormat_e dataFormat,
										   adi_adrv9001_TxSsiTestModeCfg_t *ssiTestModeConfig,
										   adi_adrv9001_TxSsiTestModeStatus_t *ssiTestModeStatus)
{
    ADI_NULL_PTR_RETURN(&device->common, ssiTestModeConfig);
    ADI_NULL_PTR_RETURN(&device->common, ssiTestModeStatus);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_RANGE_CHECK(device, ssiType, ADI_ADRV9001_SSI_TYPE_CMOS, ADI_ADRV9001_SSI_TYPE_LVDS);

    if (ADI_ADRV9001_SSI_TYPE_CMOS == ssiType)
    {
        ADI_RANGE_CHECK(device, ssiTestModeConfig->testData, ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL, ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE);
        ADI_RANGE_CHECK(device, dataFormat, ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA, ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA);
        if (ADI_ADRV9001_SSI_FORMAT_12_BIT_I_Q_DATA == dataFormat)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                dataFormat,
                "Invalid SSI data format for the selected SSI calibration test type");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }
    else
    {
        ADI_RANGE_CHECK(device, ssiTestModeConfig->testData, ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL, ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7);
        ADI_RANGE_CHECK(device, dataFormat, ADI_ADRV9001_SSI_FORMAT_12_BIT_I_Q_DATA, ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA);
    }

    if ((ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN != ssiTestModeConfig->testData) &&
        (dataFormat != ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            dataFormat,
            "Invalid SSI data format for the selected SSI calibration test type");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(adi_adrv9001_Device_t *device,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_adrv9001_SsiType_e ssiType,
                                                    adi_adrv9001_SsiDataFormat_e dataFormat,
                                                    adi_adrv9001_TxSsiTestModeCfg_t *ssiTestModeConfig,
                                                    adi_adrv9001_TxSsiTestModeStatus_t *ssiTestModeStatus)
{
    int8_t i = 0;
    int8_t nibSel = 0;
    uint8_t bfValue = 0;
    int16_t dataRead = 0;
    uint16_t iqValue[2] = { 0 };

    adrv9001_BfNvsRegmapTxChanAddr_e baseAddress = ADRV9001_BF_TX1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect_Validate, device, channel, ssiType, dataFormat, ssiTestModeConfig, ssiTestModeStatus);

    if (channel == ADI_CHANNEL_2)
    {
        baseAddress = ADRV9001_BF_TX2_CORE;
    }

    if (ADI_ADRV9001_SSI_TYPE_CMOS == ssiType)
    {
        if (ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN == ssiTestModeConfig->testData)
        {
            for(nibSel = 0 ; nibSel < 4 ; nibSel++)
            {
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugModeBfSet, device, baseAddress, 0x1);
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugStartRampBfSet, device, baseAddress, 0x0);
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugQSelBfSet, device, baseAddress, 0);   // 0: I_data; 1: Q_data
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugNibbleSelBfSet, device, baseAddress, nibSel);
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x0);
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x1);
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x0);
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugStartCaptureBfSet, device, baseAddress, 0x1);
                ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugCapturedDataBfGet, device, baseAddress, (int8_t *)(&dataRead));
                iqValue[0] |= ((dataRead & 0xF) << 4 * nibSel);
                dataRead = 0;
            }
            if ((dataFormat == ADI_ADRV9001_SSI_FORMAT_12_BIT_I_Q_DATA) || (dataFormat == ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA))
            {
                for (nibSel = 0; nibSel < 4; nibSel++)
                {
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugModeBfSet, device, baseAddress, 0x1);
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugStartRampBfSet, device, baseAddress, 0x0);
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugQSelBfSet, device, baseAddress, 1);     // 0: I_data; 1: Q_data
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugNibbleSelBfSet, device, baseAddress, nibSel);
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x0);
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x1);
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x0);
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugStartCaptureBfSet, device, baseAddress, 0x1);
                    ADI_EXPECT(adrv9001_NvsRegmapTxCssiTxDebugCapturedDataBfGet, device, baseAddress, (int8_t *)(&dataRead));
                    iqValue[1] |= ((dataRead & 0xF) << 4 * nibSel);
                    dataRead = 0;
                }
            }

            ssiTestModeStatus->dataError = 0;
            switch (dataFormat)
            {
            case ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA:
                /* Only 2 LSB */
                if (iqValue[0] != ((ssiTestModeConfig->fixedDataPatternToCheck >> 16) & 0x0003))
                {
                    ssiTestModeStatus->dataError = 1;
                }
                break;
            case ADI_ADRV9001_SSI_FORMAT_8_BIT_SYMBOL_DATA:
                /* Only 8 LSB */
                if (iqValue[0] != ((ssiTestModeConfig->fixedDataPatternToCheck >> 16) & 0x00FF))
                {
                    ssiTestModeStatus->dataError = 1;
                }
                break;
            case ADI_ADRV9001_SSI_FORMAT_16_BIT_SYMBOL_DATA:
                /* Only 16 bit I data */
                if (iqValue[0] != ((ssiTestModeConfig->fixedDataPatternToCheck >> 16) & 0xFFFF))
                {
                    ssiTestModeStatus->dataError = 1;
                }
                break;
            case ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA:
                /* Only 16 bit I and Q data */
                if ((iqValue[0] != ((ssiTestModeConfig->fixedDataPatternToCheck >> 16) & 0xFFFF)) ||
                    (iqValue[1] != (ssiTestModeConfig->fixedDataPatternToCheck & 0xFFFF)))
                {
                    ssiTestModeStatus->dataError = 1;
                }
                break;
            default:
                /* This should not happen */
                break;
            }
        }
        else
        {
            ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x17), &bfValue, 0x1, 0);
            ssiTestModeStatus->dataError = bfValue;
        }

        ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x17), &bfValue, 0x2, 0x1);
        ssiTestModeStatus->strobeAlignError = bfValue;

        ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x17), &bfValue, 0x4, 0x2);
        ssiTestModeStatus->fifoFull = bfValue;

        ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x17), &bfValue, 0x8, 0x3);
        ssiTestModeStatus->fifoEmpty = bfValue;
    }
    else /* LVDS */
    {
        if ((ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_16_BIT == ssiTestModeConfig->testData) ||
            (ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15 == ssiTestModeConfig->testData))
        {
            ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x22), &bfValue, 0xFF, 0x0);
            ssiTestModeStatus->dataError = bfValue;
        }
        else if (ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7 == ssiTestModeConfig->testData)
        {
            ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x24), &bfValue, 0xFF, 0x0);
            ssiTestModeStatus->dataError = bfValue;
        }
        else /* Fixed Pattern */
        {
            for (i = 0; i < 2; i++)
            {
                ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugModeBfSet, device, baseAddress, 0x1);
                ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugStartRampBfSet, device, baseAddress, 0x0);
                ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugQSelBfSet, device, baseAddress, i);   // 0: I_data; 1: Q_data
                ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxClearStrobeAlignErrorBfSet, device, baseAddress, 0x1);
                ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugStartCaptureBfSet, device, baseAddress, 0x1);
                ADI_EXPECT(adrv9001_NvsRegmapTxLssiTxDebugCapturedDataBfGet, device, baseAddress, &dataRead);
                iqValue[i] = dataRead;
            }

            ssiTestModeStatus->dataError = 0;
            switch (dataFormat)
            {
            case ADI_ADRV9001_SSI_FORMAT_12_BIT_I_Q_DATA:
                /* Only 12 bit I and Q data */
                if ((iqValue[0] != ((ssiTestModeConfig->fixedDataPatternToCheck >> 16) & 0x0FFF)) ||
                    (iqValue[1] != (ssiTestModeConfig->fixedDataPatternToCheck & 0x0FFF)))
                {
                    ssiTestModeStatus->dataError = 1;
                }
                break;
            case ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA:
                /* Only 16 bit I and Q data */
                if ((iqValue[0] != ((ssiTestModeConfig->fixedDataPatternToCheck >> 16) & 0xFFFF)) ||
                    (iqValue[1] != (ssiTestModeConfig->fixedDataPatternToCheck & 0xFFFF)))
                {
                    ssiTestModeStatus->dataError = 1;
                }
                break;
            default:
                /* This should not happen */
                break;
            }
        }

        ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x25), &bfValue, 0x10, 0x4);
        ssiTestModeStatus->strobeAlignError = bfValue;

        ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x25), &bfValue, 0x1, 0x0);
        ssiTestModeStatus->fifoFull = bfValue;

        ADI_EXPECT(adi_adrv9001_spi_Field_Read, device, ((uint16_t)baseAddress + 0x25), &bfValue, 0x2, 0x1);
        ssiTestModeStatus->fifoEmpty = bfValue;
    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Ssi_LvdsDelayConfigSet(adi_adrv9001_Device_t *device,
                                                   adi_adrv9001_SsiCalibrationCfg_t *ssiCalibrationCfg)
{
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1ClkLvdsDelayBfSet,      device, ADRV9001_BF_CORE, ssiCalibrationCfg->rxClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1StrobeLvdsDelayBfSet,   device, ADRV9001_BF_CORE, ssiCalibrationCfg->rxStrobeDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1IdataLvdsDelayBfSet,    device, ADRV9001_BF_CORE, ssiCalibrationCfg->rxIDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1QdataLvdsDelayBfSet,    device, ADRV9001_BF_CORE, ssiCalibrationCfg->rxQDataDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2ClkLvdsDelayBfSet,      device, ADRV9001_BF_CORE, ssiCalibrationCfg->rxClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2StrobeLvdsDelayBfSet,   device, ADRV9001_BF_CORE, ssiCalibrationCfg->rxStrobeDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2IdataLvdsDelayBfSet,    device, ADRV9001_BF_CORE, ssiCalibrationCfg->rxIDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2QdataLvdsDelayBfSet,    device, ADRV9001_BF_CORE, ssiCalibrationCfg->rxQDataDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1ClkLvdsDelayBfSet,      device, ADRV9001_BF_CORE, ssiCalibrationCfg->txClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1RefclkLvdsDelayBfSet,   device, ADRV9001_BF_CORE, ssiCalibrationCfg->txRefClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1StrobeLvdsDelayBfSet,   device, ADRV9001_BF_CORE, ssiCalibrationCfg->txStrobeDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1IdataLvdsDelayBfSet,    device, ADRV9001_BF_CORE, ssiCalibrationCfg->txIDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1QdataLvdsDelayBfSet,    device, ADRV9001_BF_CORE, ssiCalibrationCfg->txQDataDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2ClkLvdsDelayBfSet,      device, ADRV9001_BF_CORE, ssiCalibrationCfg->txClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2RefclkLvdsDelayBfSet,   device, ADRV9001_BF_CORE, ssiCalibrationCfg->txRefClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2StrobeLvdsDelayBfSet,   device, ADRV9001_BF_CORE, ssiCalibrationCfg->txStrobeDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2IdataLvdsDelayBfSet,    device, ADRV9001_BF_CORE, ssiCalibrationCfg->txIDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2QdataLvdsDelayBfSet,    device, ADRV9001_BF_CORE, ssiCalibrationCfg->txQDataDelay[1]);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Ssi_CmosDelayConfigSet(adi_adrv9001_Device_t *device,
                                                   adi_adrv9001_SsiCalibrationCfg_t *ssiCalibrationCfg)
{
    /* Rx1 */
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1ClkCmosNDelayBfSet,    device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1ClkCmosPDelayBfSet,    device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxClkDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1StrobeCmosNDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxStrobeDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1StrobeCmosPDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxStrobeDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1IdataCmosNDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxIDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1IdataCmosPDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxIDataDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1QdataCmosNDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxQDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1QdataCmosPDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxQDataDelay[0]);

    /* Rx2 */
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2ClkCmosNDelayBfSet,    device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2ClkCmosPDelayBfSet,    device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxClkDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2StrobeCmosNDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxStrobeDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2StrobeCmosPDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxStrobeDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2IdataCmosNDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxIDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2IdataCmosPDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxIDataDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2QdataCmosNDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxQDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2QdataCmosPDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->rxQDataDelay[1]);

    /* Tx1 */
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1ClkCmosNDelayBfSet,    device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1ClkCmosPDelayBfSet,    device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txClkDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1RefclkCmosNDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txRefClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1RefclkCmosPDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txRefClkDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1StrobeCmosNDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txStrobeDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1StrobeCmosPDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txStrobeDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1IdataCmosNDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txIDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1IdataCmosPDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txIDataDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1QdataCmosNDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txQDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1QdataCmosPDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txQDataDelay[0]);

    /* Tx2 */
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2ClkCmosNDelayBfSet,    device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2ClkCmosPDelayBfSet,    device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txClkDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2RefclkCmosNDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txRefClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2RefclkCmosPDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txRefClkDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2StrobeCmosNDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txStrobeDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2StrobeCmosPDelayBfSet, device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txStrobeDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2IdataCmosNDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txIDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2IdataCmosPDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txIDataDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2QdataCmosNDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txQDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2QdataCmosPDelayBfSet,  device, ADRV9001_BF_CORE_3, ssiCalibrationCfg->txQDataDelay[1]);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Ssi_Delay_Configure_Validate(adi_adrv9001_Device_t *device,
									adi_adrv9001_SsiType_e ssiType,
									adi_adrv9001_SsiCalibrationCfg_t *ssiCalibration)
{
    static const uint8_t MAX_DELAY = 7;
    ADI_RANGE_CHECK(device, ssiType, ADI_ADRV9001_SSI_TYPE_CMOS, ADI_ADRV9001_SSI_TYPE_LVDS);

    ADI_NULL_PTR_RETURN(&device->common, ssiCalibration);

    ADI_RANGE_CHECK(device, ssiCalibration->rxClkDelay[0],      0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxClkDelay[1],      0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxStrobeDelay[0],   0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxStrobeDelay[1],   0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxIDataDelay[0],    0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxIDataDelay[1],    0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxQDataDelay[0],    0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxQDataDelay[1],    0, MAX_DELAY);

    ADI_RANGE_CHECK(device, ssiCalibration->txClkDelay[0],      0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txClkDelay[1],      0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txRefClkDelay[0],   0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txRefClkDelay[1],   0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txStrobeDelay[0],   0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txStrobeDelay[1],   0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txIDataDelay[0],    0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txIDataDelay[1],    0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txQDataDelay[0],    0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txQDataDelay[1],    0, MAX_DELAY);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Ssi_Delay_Configure(adi_adrv9001_Device_t *device,
                                         adi_adrv9001_SsiType_e ssiType,
                                         adi_adrv9001_SsiCalibrationCfg_t *ssiCalibration)
{
    ADI_PERFORM_VALIDATION(adi_adrv9001_Ssi_Delay_Configure_Validate, device, ssiType, ssiCalibration);

    if (ADI_ADRV9001_SSI_TYPE_LVDS == ssiType)
    {
        ADI_EXPECT(adi_adrv9001_Ssi_LvdsDelayConfigSet, device, ssiCalibration);
    }
    else
    {
        ADI_EXPECT(adi_adrv9001_Ssi_CmosDelayConfigSet, device, ssiCalibration);
    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Ssi_LvdsDelayConfigGet(adi_adrv9001_Device_t *device,
                                                   adi_adrv9001_SsiCalibrationCfg_t *ssiCalibrationCfg)
{
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1ClkLvdsDelayBfGet,      device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->rxClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1StrobeLvdsDelayBfGet,   device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->rxStrobeDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1IdataLvdsDelayBfGet,    device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->rxIDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1QdataLvdsDelayBfGet,    device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->rxQDataDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2ClkLvdsDelayBfGet,      device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->rxClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2StrobeLvdsDelayBfGet,   device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->rxStrobeDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2IdataLvdsDelayBfGet,    device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->rxIDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2QdataLvdsDelayBfGet,    device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->rxQDataDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1ClkLvdsDelayBfGet,      device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1RefclkLvdsDelayBfGet,   device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txRefClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1StrobeLvdsDelayBfGet,   device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txStrobeDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1IdataLvdsDelayBfGet,    device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txIDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1QdataLvdsDelayBfGet,    device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txQDataDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2ClkLvdsDelayBfGet,      device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2RefclkLvdsDelayBfGet,   device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txRefClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2StrobeLvdsDelayBfGet,   device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txStrobeDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2IdataLvdsDelayBfGet,    device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txIDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2QdataLvdsDelayBfGet,    device, ADRV9001_BF_CORE, (int8_t *)&ssiCalibrationCfg->txQDataDelay[1]);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Ssi_CmosDelayConfigGet(adi_adrv9001_Device_t *device,
                                                   adi_adrv9001_SsiCalibrationCfg_t *ssiCalibrationCfg)
{
    /* Rx1 */
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1ClkCmosNDelayBfGet,    device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1ClkCmosPDelayBfGet,    device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxClkDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1StrobeCmosNDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxStrobeDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1StrobeCmosPDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxStrobeDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1IdataCmosNDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxIDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1IdataCmosPDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxIDataDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1QdataCmosNDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxQDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx1QdataCmosPDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxQDataDelay[0]);

    /* Rx2 */
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2ClkCmosNDelayBfGet,    device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2ClkCmosPDelayBfGet,    device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxClkDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2StrobeCmosNDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxStrobeDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2StrobeCmosPDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxStrobeDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2IdataCmosNDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxIDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2IdataCmosPDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxIDataDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2QdataCmosNDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxQDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Rx2QdataCmosPDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->rxQDataDelay[1]);

    /* Tx1 */
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1ClkCmosNDelayBfGet,    device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1ClkCmosPDelayBfGet,    device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txClkDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1RefclkCmosNDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txRefClkDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1RefclkCmosPDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txRefClkDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1StrobeCmosNDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txStrobeDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1StrobeCmosPDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txStrobeDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1IdataCmosNDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txIDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1IdataCmosPDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txIDataDelay[0]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1QdataCmosNDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txQDataDelay[0]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx1QdataCmosPDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txQDataDelay[0]);

    /* Tx2 */
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2ClkCmosNDelayBfGet,    device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2ClkCmosPDelayBfGet,    device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txClkDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2RefclkCmosNDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txRefClkDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2RefclkCmosPDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txRefClkDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2StrobeCmosNDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txStrobeDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2StrobeCmosPDelayBfGet, device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txStrobeDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2IdataCmosNDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txIDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2IdataCmosPDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txIDataDelay[1]);

    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2QdataCmosNDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txQDataDelay[1]);
    ADI_EXPECT(adrv9001_NvsRegmapCore3Tx2QdataCmosPDelayBfGet,  device, ADRV9001_BF_CORE_3, &ssiCalibrationCfg->txQDataDelay[1]);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Ssi_Delay_Inspect_Validate(adi_adrv9001_Device_t *device,
								      adi_adrv9001_SsiType_e ssiType,
								      adi_adrv9001_SsiCalibrationCfg_t *ssiCalibration)
{
    ADI_RANGE_CHECK(device, ssiType, ADI_ADRV9001_SSI_TYPE_CMOS, ADI_ADRV9001_SSI_TYPE_LVDS);
    ADI_NULL_PTR_RETURN(&device->common, ssiCalibration);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Ssi_Delay_Inspect(adi_adrv9001_Device_t *device,
                                       adi_adrv9001_SsiType_e ssiType,
                                       adi_adrv9001_SsiCalibrationCfg_t *ssiCalibration)
{
    ADI_PERFORM_VALIDATION(adi_adrv9001_Ssi_Delay_Inspect_Validate, device, ssiType, ssiCalibration);

    if (ADI_ADRV9001_SSI_TYPE_LVDS == ssiType)
    {
        ADI_EXPECT(adi_adrv9001_Ssi_LvdsDelayConfigGet, device, ssiCalibration);
    }
    else
    {
        ADI_EXPECT(adi_adrv9001_Ssi_CmosDelayConfigGet, device, ssiCalibration);
    }

    ADI_API_RETURN(device);
}
