/**
* \file
* \brief Core board functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2018 - 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include <stdlib.h>
#include <string.h>

#include "adi_platform.h"
#include "adi_adrv9001ee01.h"

#include "adrv9001ee01.h"

#include "adi_fpga9001_ssi.h"
#include "adi_fpga9001_mmcm.h"

#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_mcs.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_utilities.h"

#include "adrv9001_init.h"
#include "adrv9001_reg_addr_macros.h"

int32_t adi_adrv9001Ee01_Create(adi_adrv9001Ee01_Board_t *adrv9001Ee01)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    /* Initialize spi setting for the EE01 board type */
    adi_adrv9001_SpiSettings_t spiSettings = {
        .msbFirst = 1,
        .enSpiStreaming = 0,
        .autoIncAddrUp = 1,
        .fourWireMode = 1,
        .cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
    };
    adi_hal_Cfg_t *halCfg = NULL;

    if (adrv9001Ee01 == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    adrv9001Ee01->adrv9001Device = (adi_adrv9001_Device_t*)calloc(1, sizeof(adi_adrv9001_Device_t));
    if (adrv9001Ee01->adrv9001Device == NULL)
    {
        adi_adrv9001Ee01_Destroy(adrv9001Ee01);
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    adrv9001Ee01->fpga9001Device = (adi_fpga9001_Device_t*)calloc(1, sizeof(adi_fpga9001_Device_t));
    if (adrv9001Ee01->fpga9001Device == NULL)
    {
        adi_adrv9001Ee01_Destroy(adrv9001Ee01);
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    /* Set function in the platform layer to be ADI_HW_PLATFORM*/
    adi_hal_PlatformSetup(NULL, ADI_HW_PLATFORM);

    /* Creat devHalInfo for the devices*/
    /* Check if function pointer is populated */
    if (adi_hal_DevHalCfgCreate == NULL)
    {
        /* TODO: add an error structure to the board level*/
        printf("NULL Hal layer function pointer");
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    adrv9001Ee01->adrv9001Device->common.devHalInfo = adi_hal_DevHalCfgCreate((ADI_HAL_INTERFACE_SPI | ADI_HAL_INTERFACE_LOG | ADI_HAL_INTERFACE_HWRESET | ADI_HAL_INTERFACE_TIMER), 0, "adrv9001Log.txt");
    if (adrv9001Ee01->adrv9001Device->common.devHalInfo == NULL)
    {
        adi_adrv9001Ee01_Destroy(adrv9001Ee01);
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    adrv9001Ee01->fpga9001Device->common.devHalInfo = adi_hal_DevHalCfgCreate((ADI_HAL_INTERFACE_BBICCTRL | ADI_HAL_INTERFACE_LOG | ADI_HAL_INTERFACE_I2C | ADI_HAL_INTERFACE_TIMER), 0, "fpgaLog.txt");
    if (adrv9001Ee01->fpga9001Device->common.devHalInfo == NULL)
    {
        adi_adrv9001Ee01_Destroy(adrv9001Ee01);
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    halCfg = (adi_hal_Cfg_t *)adrv9001Ee01->adrv9001Device->common.devHalInfo;
    halCfg->fpga9001 = adrv9001Ee01->fpga9001Device;

    /* Open hardware for different devices*/
    recoveryAction = adi_hal_HwOpen(adrv9001Ee01->fpga9001Device->common.devHalInfo);
    if (recoveryAction != (int32_t)ADI_HAL_OK)
    {
        printf("Board HwOpen failed. %s \n", adrv9001Ee01->fpga9001Device->common.error.errormessage);
        return recoveryAction;
    }

    recoveryAction = adi_adrv9001_HwOpen(adrv9001Ee01->adrv9001Device, &spiSettings);
    if (recoveryAction != (int32_t)ADI_HAL_OK)
    {
        printf("Navassa HwOpen failed. %s \n", adrv9001Ee01->adrv9001Device->common.error.errormessage);
        return recoveryAction;
    }

    /* Clear any previous adrv9001 Errors */
    recoveryAction = adi_common_ErrorClear(&adrv9001Ee01->adrv9001Device->common);

    return recoveryAction;
}

int32_t adi_adrv9001Ee01_Destroy(adi_adrv9001Ee01_Board_t *adrv9001Ee01)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    if (adrv9001Ee01 == NULL)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    /* Check if function pointer is populated */
    if (adi_hal_DevHalCfgFree == NULL)
    {
        /* TODO: add an error structure to the board level*/
        printf("NULL Hal layer function pointer");
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }

    if (adrv9001Ee01->adrv9001Device != NULL)
    {
        recoveryAction = adi_adrv9001_HwClose(adrv9001Ee01->adrv9001Device);
        if (recoveryAction != (int32_t)ADI_HAL_OK)
        {
            return recoveryAction;
        }
        adi_hal_DevHalCfgFree(adrv9001Ee01->adrv9001Device->common.devHalInfo);
        free(adrv9001Ee01->adrv9001Device);
        adrv9001Ee01->adrv9001Device = NULL;
    }

    if (adrv9001Ee01->fpga9001Device != NULL)
    {
        recoveryAction = adi_hal_HwClose(adrv9001Ee01->fpga9001Device->common.devHalInfo);
        if (recoveryAction != (int32_t)ADI_HAL_OK)
        {
            return recoveryAction;
        }
        adi_hal_DevHalCfgFree(adrv9001Ee01->fpga9001Device->common.devHalInfo);
        free(adrv9001Ee01->fpga9001Device);
        adrv9001Ee01->fpga9001Device = NULL;
    }

    free(adrv9001Ee01);
    adrv9001Ee01 = NULL;

    return recoveryAction;
}

#ifdef ADI_INITIALIZE_FROM_PROFILE
int32_t adi_adrv9001Ee01_InitializeFromProfile(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                               const char* profileFilename,
                                               adi_adrv9001_RadioCtrlInit_t *radioCtrlInit,
                                               adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles)
{
    adi_adrv9001_Device_t *adrv9001Device = NULL;

    adi_adrv9001_Init_t adrv9001InitInst = { {0} };
    adi_adrv9001_Init_t *adrv9001Init = &adrv9001InitInst;

    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->adrv9001Device);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->fpga9001Device);
    adrv9001Device = adrv9001Ee01->adrv9001Device;
    adrv9001Device = adrv9001Ee01->adrv9001Device;

    ADI_NULL_PTR_RETURN(&adrv9001Device->common, radioCtrlInit);
    ADI_NULL_PTR_RETURN(&adrv9001Device->common, adrv9001PlatformFiles);

    ADI_EXPECT(adi_adrv9001_Utilities_DeviceProfile_Load, adrv9001Device, profileFilename, adrv9001Init);

    adi_adrv9001Ee01_Initialize(adrv9001Ee01, adrv9001Init, radioCtrlInit, adrv9001PlatformFiles);

    ADI_API_RETURN(adrv9001Device);
}
#endif

int32_t adi_adrv9001Ee01_Initialize(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                    adi_adrv9001_Init_t *adrv9001Init,
                                    adi_adrv9001_RadioCtrlInit_t *radioCtrlInit,
                                    adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles)
{
    int32_t recoveryAction = 0;

    adi_adrv9001_Device_t *adrv9001Device = NULL;
    adi_fpga9001_Device_t *fpga9001Device = NULL;

    adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockOutDivisor = ADI_FPGA9001_MMCM_CLKDIV_BYPASS;

    uint8_t channelMask = 0;
    uint32_t i = 0;
    uint32_t txChannels[] = { ADI_ADRV9001_TX1, ADI_ADRV9001_TX2 };
    static const adi_common_ChannelNumber_e channels[] = { ADI_CHANNEL_1, ADI_CHANNEL_2 };
    static const uint16_t DEFAULT_ATTENUATION_MDB = 10000;

    static const uint8_t ALL_RX_CHANNEL_MASK = ADI_ADRV9001_RX1 | ADI_ADRV9001_RX2 | ADI_ADRV9001_ORX1 | ADI_ADRV9001_ORX2;

    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->adrv9001Device);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->fpga9001Device);
    adrv9001Device = adrv9001Ee01->adrv9001Device;
    fpga9001Device = adrv9001Ee01->fpga9001Device;

    adi_common_ErrorClear(&adrv9001Device->common);

    ADI_NULL_PTR_RETURN(&adrv9001Device->common, adrv9001Init);
    ADI_NULL_PTR_RETURN(&adrv9001Device->common, radioCtrlInit);
    ADI_NULL_PTR_RETURN(&adrv9001Device->common, adrv9001PlatformFiles);

    adi_adrv9001_ResourceCfg_t adrv9001ResourceCfg = { adrv9001Init, radioCtrlInit, adrv9001PlatformFiles };

    channelMask = adrv9001Init->tx.txInitChannelMask | (adrv9001Init->rx.rxInitChannelMask & ALL_RX_CHANNEL_MASK);

    recoveryAction = adi_adrv9001Ee01_SelectFpgaBin(adrv9001Ee01, adrv9001Init);
    ADI_ERROR_RETURN(recoveryAction);

    ADI_EXPECT(adi_adrv9001_HwReset, adrv9001Device);

    adrv9001DeviceClockOutDivisor = (adi_fpga9001_Mmcm_ClockOutput_Divisor_e)(radioCtrlInit->adrv9001DeviceClockOutputDivisor);

    ADI_EXPECT(adi_adrv9001_InitAnalog,
               adrv9001Device,
               adrv9001Init,
               radioCtrlInit->adrv9001DeviceClockOutputDivisor);

#ifdef SI_REV_A0
    ADI_EXPECT(adi_adrv9001_InitDigital, adrv9001Device, adrv9001Init);

    ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, adrv9001Device, 2);
#endif
    ADI_EXPECT(adi_adrv9001_Utilities_Resources_Load, adrv9001Device, &adrv9001ResourceCfg);

    ADI_EXPECT(adi_fpga9001_Initialize, fpga9001Device, adrv9001Init, adrv9001DeviceClockOutDivisor);

    if (adrv9001Device->devStateInfo.profilesValid & ADI_ADRV9001_TX_PROFILE_VALID)
    {
        adrv9001Device->devStateInfo.outputSignaling[0] = adrv9001Init->tx.txProfile[0].outputSignaling;
        adrv9001Device->devStateInfo.outputSignaling[1] = adrv9001Init->tx.txProfile[1].outputSignaling;
        adrv9001Device->devStateInfo.txInputRate_kHz[0] = adrv9001Init->tx.txProfile[0].txInputRate_Hz / 1000;
        adrv9001Device->devStateInfo.txInputRate_kHz[1] = adrv9001Init->tx.txProfile[1].txInputRate_Hz / 1000;
        for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
        {
            /* For each tx channel enabled */
            if (ADRV9001_BF_EQUAL(adrv9001Init->tx.txInitChannelMask, txChannels[i]))
            {
                ADI_EXPECT(adi_adrv9001_Tx_Attenuation_Configure,
                           adrv9001Device,
                           channels[i],
                           &radioCtrlInit->txAttenConfig);

                ADI_EXPECT(adi_adrv9001_Tx_Attenuation_Set, adrv9001Device, channels[i], DEFAULT_ATTENUATION_MDB);
            }
        }
    }

    ADI_EXPECT(adi_adrv9001_Utilities_InitRadio_Load,
               adrv9001Device,
               &adrv9001ResourceCfg,
               channelMask);
    /* Reset FPGA SSI channel reset before Init Cal to sync FPGA with the LVDS/CMOS delay configurations set in Navassa device */
    ADI_EXPECT(adi_fpga9001_ssi_Reset, fpga9001Device);

    ADI_API_RETURN(adrv9001Device);
}