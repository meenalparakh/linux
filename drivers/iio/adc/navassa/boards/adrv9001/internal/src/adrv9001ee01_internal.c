/**
* \file
* \brief ADI internal-only core board functions
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
#include "adrv9001ee01_internal.h"

#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_utilities.h"
#include "adi_adrv9001_mcs.h"
#include "adi_fpga9001_gpio.h"
#include "adrv9001_init.h"
#include "adrv9001_reg_addr_macros.h"
#include "adi_fpga9001_mmcm.h"

int32_t adrv9001Ee01_internal_InitializeFromProfile(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                                    uint32_t day1ExecPhaseMask,
                                                    const char* profileFilename,
                                                    adi_adrv9001_RadioCtrlInit_t *radioCtrlInit,
                                                    adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles,
                                                    uint8_t channelMask)
{
    adi_adrv9001_Device_t *adrv9001Device = NULL;

    adi_adrv9001_Init_t adrv9001InitInst = { {0} };
    adi_adrv9001_Init_t *adrv9001Init = &adrv9001InitInst;

    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->adrv9001Device);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->fpga9001Device);
    adrv9001Device = adrv9001Ee01->adrv9001Device;

    ADI_EXPECT(adi_adrv9001_Utilities_DeviceProfile_Load, adrv9001Device, profileFilename, adrv9001Init);

    adrv9001Ee01_internal_Initialize(adrv9001Ee01, day1ExecPhaseMask, adrv9001Init, radioCtrlInit, adrv9001PlatformFiles, channelMask);

    ADI_API_RETURN(adrv9001Device);
}

int32_t adrv9001Ee01_internal_Initialize(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                         uint32_t day1ExecPhaseMask,
                                         adi_adrv9001_Init_t *adrv9001Init,
                                         adi_adrv9001_RadioCtrlInit_t *radioCtrlInit,
                                         adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles,
                                         uint8_t channelMask)
{
    int32_t recoveryAction = 0;
    adi_fpga9001_Gpio_PinCfg_t jtagPinCfg = {
        .gpioPins = { ADI_FPGA9001_GPIO_11, ADI_FPGA9001_GPIO_10, ADI_FPGA9001_GPIO_09, ADI_FPGA9001_GPIO_08 },
        .gpioSourceMode = {
            ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_CONSTANT_ZERO,
            ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_CONSTANT_ZERO,
            ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_CONSTANT_ONE,
            ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_CONSTANT_ONE
        }
    };

    static const adi_common_ChannelNumber_e channels[] = { ADI_CHANNEL_1, ADI_CHANNEL_2 };
    uint32_t txChannels[] = { ADI_ADRV9001_TX1, ADI_ADRV9001_TX2 };
    static const uint16_t DEFAULT_ATTENUATION_MDB = 10000;

    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->adrv9001Device);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ee01->fpga9001Device);

    adi_adrv9001_Device_t *adrv9001Device = adrv9001Ee01->adrv9001Device;
    adi_fpga9001_Device_t *fpga9001Device = adrv9001Ee01->fpga9001Device;

    ADI_API_ENTRY_EXPECT(adrv9001Device);
    adi_common_ErrorClear(&adrv9001Device->common);

    ADI_NULL_PTR_RETURN(&adrv9001Device->common, adrv9001Init);
    ADI_NULL_PTR_RETURN(&adrv9001Device->common, radioCtrlInit);
    ADI_NULL_PTR_RETURN(&adrv9001Device->common, adrv9001PlatformFiles);

    adi_adrv9001_ResourceCfg_t adrv9001ResourceCfg = { adrv9001Init, radioCtrlInit, adrv9001PlatformFiles };

    adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockOutDivisor = ADI_FPGA9001_MMCM_CLKDIV_BYPASS;

    uint32_t i = 0;

    if (ADRV9001_BF_EQUAL(day1ExecPhaseMask, ADI_ADRV9001_DAY1_EXEC_PHASE_JTAG))
    {
        recoveryAction = adi_adrv9001Ee01_SelectFpgaBin(adrv9001Ee01, adrv9001Init);
        ADI_ERROR_RETURN(recoveryAction);

        ADI_EXPECT(adi_adrv9001_HwReset, adrv9001Device);

        ADI_EXPECT(adi_fpga9001_gpio_pin_SourceMode_Configure, fpga9001Device, &jtagPinCfg, 4);

        ADI_EXPECT(adi_fpga9001_gpio_Mode_Set, fpga9001Device, ADI_FPGA9001_GPIO_MODE_JTAG);
    }
    if (ADRV9001_BF_EQUAL(day1ExecPhaseMask, ADI_ADRV9001_DAY1_EXEC_PHASE_INIT_ANALOG))
    {
        ADI_EXPECT(adi_adrv9001_InitAnalog,
                   adrv9001Device,
                   adrv9001Init,
                   radioCtrlInit->adrv9001DeviceClockOutputDivisor);
    }
    if (ADRV9001_BF_EQUAL(day1ExecPhaseMask, ADI_ADRV9001_DAY1_EXEC_PHASE_INIT_DIGITAL_CLOCKS))
    {
#ifdef SI_REV_A0
        ADI_EXPECT(adi_adrv9001_InitDigital, adrv9001Device, adrv9001Init);

        ADI_EXPECT(adi_adrv9001_Mcs_DigitalInt_Set, adrv9001Device, 2);
#endif // SI_REV_A0
    }

    if (ADRV9001_BF_EQUAL(day1ExecPhaseMask, ADI_ADRV9001_DAY1_EXEC_PHASE_RESOURCES_LOAD))
    {
        ADI_EXPECT(adi_adrv9001_Utilities_Resources_Load, adrv9001Device, &adrv9001ResourceCfg);

        adrv9001DeviceClockOutDivisor = (adi_fpga9001_Mmcm_ClockOutput_Divisor_e)(radioCtrlInit->adrv9001DeviceClockOutputDivisor);

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
    }

    if (ADRV9001_BF_EQUAL(day1ExecPhaseMask, ADI_ADRV9001_DAY1_EXEC_PHASE_INIT_RADIO))
    {
        ADI_EXPECT(adi_adrv9001_Utilities_InitRadio_Load, adrv9001Device, &adrv9001ResourceCfg, channelMask);

        /* Reset FPGA SSI channel reset before Init Cal to sync FPGA with the LVDS/CMOS delay configurations set in Navassa device */
        ADI_EXPECT(adi_fpga9001_ssi_Reset, fpga9001Device);
    }

    ADI_API_RETURN(adrv9001Device);
}
