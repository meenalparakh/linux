/**
* \file
* \brief Private core board functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001ee01.h"

#include "adrv9001ee01.h"

#include "adi_adrv9001_rx.h"

#include "adi_fpga9001.h"
#include "adi_fpga9001_hal.h"
#include "adi_fpga9001_datachain.h"

#include "adrv9001_init.h"
#include "adrv9001_reg_addr_macros.h"

#include "fpga9001_utilities.h"
#include "fpga9001_bf_axi_adrv9001.h"

int32_t adi_adrv9001Ee01_SelectFpgaBin(adi_adrv9001Ee01_Board_t *adrv9001Ee01, adi_adrv9001_Init_t *adrv9001Init)
{
    adi_adrv9001_SsiType_e ssiType = adrv9001Init->rx.rxChannelCfg[0].profile.rxSsiConfig.ssiType |
        adrv9001Init->rx.rxChannelCfg[1].profile.rxSsiConfig.ssiType | 
        adrv9001Init->tx.txProfile[0].txSsiConfig.ssiType |
        adrv9001Init->tx.txProfile[1].txSsiConfig.ssiType;
    
    if (ssiType == ADI_ADRV9001_SSI_TYPE_CMOS)
    {
        ADI_EXPECT(adi_fpga9001_SwitchBin, adrv9001Ee01->fpga9001Device, ADI_FPGA9001_CMOS);
    }
    else if (ssiType == ADI_ADRV9001_SSI_TYPE_LVDS)
    {
        ADI_EXPECT(adi_fpga9001_SwitchBin, adrv9001Ee01->fpga9001Device, ADI_FPGA9001_LVDS);
    }
    else
    {
        ADI_ERROR_REPORT(&adrv9001Ee01->fpga9001Device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         adrv9001Init,
                         "Invalid SSI type. Must be LVDS or CMOS. All initialized channels must use the same SSI type");
    }

    ADI_EXPECT(adi_adrv9001_HwClose, adrv9001Ee01->adrv9001Device);

    adi_adrv9001_SpiSettings_t spiSettings = {
        .msbFirst = 1,
        .enSpiStreaming = 0,
        .autoIncAddrUp = 1,
        .fourWireMode = 1,
        .cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
    };

    ADI_EXPECT(adi_adrv9001_HwOpen, adrv9001Ee01->adrv9001Device, &spiSettings);

    ADI_API_RETURN(adrv9001Ee01->fpga9001Device);
}

static int32_t adi_fpga9001_CalculateLcmInterfaceSamplingRate_Hz(adi_fpga9001_Device_t *fpga9001Device,
                                                                 adi_adrv9001_Init_t *init,
                                                                 uint32_t *lcmInterfaceSampleRate_Hz)
{
    int32_t i = 0;
    uint32_t rxChannels[] = { ADI_ADRV9001_RX1, ADI_ADRV9001_RX2 };
    uint32_t txChannels[] = { ADI_ADRV9001_TX1, ADI_ADRV9001_TX2 };

    *lcmInterfaceSampleRate_Hz = 1;

    for (i = 0; i < ADI_ADRV9001_MAX_RX_ONLY; i++)
    {
        if (ADRV9001_BF_EQUAL(init->rx.rxInitChannelMask, rxChannels[i]))
        {
            if (init->rx.rxChannelCfg[i].profile.rxInterfaceSampleRate_Hz > *lcmInterfaceSampleRate_Hz)
            {
                ADI_EXPECT(fpga9001_Utilities_CalculateLcm,
                           fpga9001Device,
                           init->rx.rxChannelCfg[i].profile.rxInterfaceSampleRate_Hz,
                           *lcmInterfaceSampleRate_Hz,
                           lcmInterfaceSampleRate_Hz);
            }
        }
    }

    for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
    {
        if (ADRV9001_BF_EQUAL(init->tx.txInitChannelMask, txChannels[i]))
        {
            if (init->tx.txProfile[i].txInterfaceSampleRate_Hz > *lcmInterfaceSampleRate_Hz)
            {
                ADI_EXPECT(fpga9001_Utilities_CalculateLcm,
                           fpga9001Device,
                           init->tx.txProfile[i].txInterfaceSampleRate_Hz,
                           *lcmInterfaceSampleRate_Hz,
                           lcmInterfaceSampleRate_Hz);
            }
        }
    }

    if (*lcmInterfaceSampleRate_Hz == 1)
    {
        ADI_ERROR_REPORT(&fpga9001Device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            lcmInterfaceSampleRate_Hz,
            "No channels are enabled.");
    }

    ADI_API_RETURN(fpga9001Device);
}

/* The hard-coded values are specific to the FPGA9001/Zc706 platform. */
static void ssiDelayConfigInitDefault(adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{

    ssiCalibration->rxIDataDelay[0] = 0x13;
    ssiCalibration->rxIDataDelay[1] = 0x13;
    ssiCalibration->rxQDataDelay[0] = 0x13;
    ssiCalibration->rxQDataDelay[1] = 0x13;
    ssiCalibration->rxStrobeDelay[0] = 0x13;
    ssiCalibration->rxStrobeDelay[1] = 0x13;

    ssiCalibration->txIDataDelay[0] = 0;
    ssiCalibration->txIDataDelay[1] = 0;
    ssiCalibration->txQDataDelay[0] = 0;
    ssiCalibration->txQDataDelay[1] = 0;
    ssiCalibration->txStrobeDelay[0] = 0;
    ssiCalibration->txStrobeDelay[1] = 0;
    ssiCalibration->txClkDelay[0] = 0xF;
    ssiCalibration->txClkDelay[1] = 0xF;
}

int32_t adi_fpga9001_Initialize(adi_fpga9001_Device_t *fpga9001Device,
                                adi_adrv9001_Init_t *init,
                                adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockDivisor)
{
    uint8_t i = 0;
    uint8_t rxClkRate = 0;
    adi_fpga9001_Version_t version = { 0 };
    uint32_t targetInterfaceSampleRate_Hz = 0;

    static const adi_common_ChannelNumber_e channels[] = { ADI_CHANNEL_1, ADI_CHANNEL_2 };
    uint32_t rxChannels[] = { ADI_ADRV9001_RX1, ADI_ADRV9001_RX2 };
    uint32_t txChannels[] = { ADI_ADRV9001_TX1, ADI_ADRV9001_TX2 };

    ADI_API_ENTRY_EXPECT(fpga9001Device);

    ADI_EXPECT(adi_fpga9001_VersionGet, fpga9001Device, &version);

    adi_adrv9001_SsiType_e ssiType = init->rx.rxChannelCfg[0].profile.rxSsiConfig.ssiType |
        init->rx.rxChannelCfg[1].profile.rxSsiConfig.ssiType | 
        init->tx.txProfile[0].txSsiConfig.ssiType |
        init->tx.txProfile[1].txSsiConfig.ssiType;
    adi_fpga9001_SsiCalibrationCfg_t ssiCalibration = { { 0 } };
    ssiDelayConfigInitDefault(&ssiCalibration);

    /* Checking only major and minor; ignoring patch version check */
    if (version.major == 0 && version.minor >= 3 )
    {
        ADI_EXPECT(fpga9001_AxiAdrv9001ResetbBfSet, fpga9001Device, FPGA9001_BF_AXI_ADRV9001_TOP, 0x1);

        ADI_EXPECT(adi_fpga9001_CalculateLcmInterfaceSamplingRate_Hz, fpga9001Device, init, &targetInterfaceSampleRate_Hz);

        /* Configure MMCM registers; THis function will reset the FPGA at the end */
        /* TODO: For now, sample rate of RX1 is used. Mick to confirm the right value to use */
        ADI_EXPECT(adi_fpga9001_Mmcm_Configure,
                   fpga9001Device,
                   init->clocks.deviceClock_kHz,
                   targetInterfaceSampleRate_Hz,
                   adrv9001DeviceClockDivisor);

        ADI_EXPECT(adi_fpga9001_Ssi_Delay_Configure,
                   fpga9001Device,
                   (adi_fpga9001_SsiType_e)ssiType,
                   &ssiCalibration);

        for (i = 0; i < ADI_ADRV9001_MAX_RX_ONLY; i++)
        {
            if (ADRV9001_BF_EQUAL(init->rx.rxInitChannelMask, rxChannels[i]))
            {
                ADI_EXPECT(adi_fpga9001_ssi_Mode_Set, 
                           fpga9001Device,
                           ADI_RX,
                           channels[i],
                           (ADI_ADRV9001_SSI_TYPE_CMOS == ssiType) && !init->rx.rxChannelCfg[i].profile.rxSsiConfig.cmosDdrEn,
                           init->rx.rxChannelCfg[i].profile.rxSsiConfig.numLaneSel != ADI_ADRV9001_SSI_1_LANE,
                           (adi_fpga9001_SsiFormat_e)init->rx.rxChannelCfg[i].profile.rxSsiConfig.ssiDataFormatSel);

                /* TODO: Add new parameter to explicitly determine clock divisor.  It's really an ORx over-sampling ratio.*/
                /* 0: clkRate; 1: clkRate/2, 2: clkRate/4, ... */
                rxClkRate = 0;

                ADI_EXPECT(adi_fpga9001_DataChain_RxClkRate_Set, fpga9001Device, channels[i], rxClkRate);
            }
        }

        for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
        {
            if (ADRV9001_BF_EQUAL(init->tx.txInitChannelMask, txChannels[i]))
            {
                ADI_EXPECT(adi_fpga9001_ssi_Mode_Set, 
                           fpga9001Device,
                           ADI_TX,
                           channels[i],
                           (ADI_ADRV9001_SSI_TYPE_CMOS == ssiType) && !init->tx.txProfile[i].txSsiConfig.cmosDdrEn,
                           init->tx.txProfile[i].txSsiConfig.numLaneSel != ADI_ADRV9001_SSI_1_LANE,
                           (adi_fpga9001_SsiFormat_e)init->tx.txProfile[i].txSsiConfig.ssiDataFormatSel);
            }
        }
    }
    else
    {
        ADI_ERROR_REPORT(&fpga9001Device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, NULL, "Unrecognized FPGA version");
        ADI_API_RETURN(fpga9001Device);
    }

    ADI_API_RETURN(fpga9001Device);
}
