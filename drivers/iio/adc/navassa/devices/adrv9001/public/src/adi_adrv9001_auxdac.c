/**
* \file
* \brief Contains Aux DAC features related function implementation defined in
* adi_adrv9001_auxdac.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
/* "adi_adrv9001_user.h" contains the #define that other header file use */
#include "adi_adrv9001_user.h"

/* Header file corresponding to the C file */
#include "adi_adrv9001_auxdac.h"

/* ADI specific header files */
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_gpio.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_bf_analog_tx_mem_map.h"
#include "adrv9001_bf_nvs_regmap_tx.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_reg_addr_macros.h"
#include "adrv9001_bf_nvs_regmap_core_2.h"

/* Header files related to libraries */

/* System header files */

/*
*********************************************************************************************************
*                                             Static functions
*********************************************************************************************************
*/
static int32_t adrv9001_AuxDac_Configure(adi_adrv9001_Device_t *device,
                                         adi_adrv9001_AuxDacs_e auxDacSel)
{
    static const uint8_t AUXDAC_DEFAULT_MODE = 0x01;
    adrv9001_BfNvsRegmapCore2ChanAddr_e baseAddr = ADRV9001_BF_CORE_2;

    /* Check device pointer is not null */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    if (auxDacSel == ADI_ADRV9001_AUXDAC0)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac0ConfigBfSet, device, baseAddr, AUXDAC_DEFAULT_MODE);
    }
    else if (auxDacSel == ADI_ADRV9001_AUXDAC1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac1ConfigBfSet, device, baseAddr, AUXDAC_DEFAULT_MODE);
    }
    else if (auxDacSel == ADI_ADRV9001_AUXDAC2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac2ConfigBfSet, device, baseAddr, AUXDAC_DEFAULT_MODE);
    }
    else
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac3ConfigBfSet, device, baseAddr, AUXDAC_DEFAULT_MODE);
    }

    ADI_API_RETURN(device);
}

static int32_t adrv9001_AuxDacBf_Get(adi_adrv9001_Device_t *device,
                                     adrv9001_BfNvsRegmapCore2ChanAddr_e baseAddr,
                                     uint8_t *bfEnable)
{
    uint8_t bfValue = 0;

    /* Check for null */
    ADI_API_ENTRY_PTR_EXPECT(device, bfEnable);

    ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac0PdBfGet, device, baseAddr, &bfValue);
    *bfEnable |= bfValue;
    bfValue = 0;

    ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac1PdBfGet, device, baseAddr, &bfValue);
    *bfEnable |= bfValue << 1;
    bfValue = 0;

    ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac2PdBfGet, device, baseAddr, &bfValue);
    *bfEnable |= bfValue << 2;
    bfValue = 0;

    ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac3PdBfGet, device, baseAddr, &bfValue);
    *bfEnable |= bfValue << 3;

    ADI_API_RETURN(device);
}

static int32_t adrv9001_AuxDacBf_Set(adi_adrv9001_Device_t *device,
                                     adrv9001_BfNvsRegmapCore2ChanAddr_e baseAddr,
                                     uint8_t bfEnable)
{
    uint8_t bfValue = 0;

    /* Check for null */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    bfValue = bfEnable & 0x1;
    ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac0PdBfSet, device, baseAddr, bfValue);

    bfValue = (bfEnable >> 1) & 0x1;
    ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac1PdBfSet, device, baseAddr, bfValue);

    bfValue = (bfEnable >> 2) & 0x1;
    ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac2PdBfSet, device, baseAddr, bfValue);

    bfValue = (bfEnable >> 3) & 0x1;
    ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac3PdBfSet, device, baseAddr, bfValue);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_AuxDac_Configure_Validate(adi_adrv9001_Device_t *device,
								     adi_adrv9001_AuxDacs_e auxDac)
{
    /* Check device pointer and gain pointer are not null */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_RANGE_CHECK(device, auxDac, ADI_ADRV9001_AUXDAC0, ADI_ADRV9001_AUXDAC3);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_AuxDac_Inspect_Validate(adi_adrv9001_Device_t *device,
								   adi_adrv9001_AuxDacs_e auxDac,
								   bool *enabled)
{
    /* Check device pointer and gain pointer are not null */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_NULL_PTR_RETURN(&device->common, enabled);

    ADI_RANGE_CHECK(device, auxDac, ADI_ADRV9001_AUXDAC0, ADI_ADRV9001_AUXDAC3);

    ADI_API_RETURN(device);
}

static int32_t adrv9001_AuxDacValue_Set(adi_adrv9001_Device_t *device,
                                        adi_adrv9001_AuxDacs_e auxDacSel,
                                        uint16_t dacWord)
{
    adrv9001_BfNvsRegmapCore2ChanAddr_e baseAddr = ADRV9001_BF_CORE_2;

    /* Check device pointer is not null */
    ADI_NULL_DEVICE_PTR_RETURN(device);

    if (auxDacSel == ADI_ADRV9001_AUXDAC0)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac0BfSet, device, baseAddr, dacWord);
    }
    else if (auxDacSel == ADI_ADRV9001_AUXDAC1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac1BfSet, device, baseAddr, dacWord);
    }
    else if (auxDacSel == ADI_ADRV9001_AUXDAC2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac2BfSet, device, baseAddr, dacWord);
    }
    else
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac3BfSet, device, baseAddr, dacWord);
    }

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_AuxDac_Code_Set_Validate(adi_adrv9001_Device_t *device,
								    adi_adrv9001_AuxDacs_e auxDac,
								    uint16_t auxDacCode)
{
    static const uint16_t AUX_DAC_VALUE_MIN = 0;
    static const uint16_t AUX_DAC_VALUE_MAX = 4095;

    /* Check device pointer and gain pointer are not null */
    ADI_NULL_DEVICE_PTR_RETURN(device);
    ADI_RANGE_CHECK(device, auxDac, ADI_ADRV9001_AUXDAC0, ADI_ADRV9001_AUXDAC3);
    ADI_RANGE_CHECK(device, auxDacCode, AUX_DAC_VALUE_MIN, AUX_DAC_VALUE_MAX);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_AuxDac_Code_Get_Validate(adi_adrv9001_Device_t *device,
								    adi_adrv9001_AuxDacs_e auxDac,
								    uint16_t *auxDacCode)
{
    /* Check device pointer and gain pointer are not null */
    ADI_NULL_DEVICE_PTR_RETURN(device);
    ADI_NULL_PTR_RETURN(&device->common, auxDacCode);
    ADI_RANGE_CHECK(device, auxDac, ADI_ADRV9001_AUXDAC0, ADI_ADRV9001_AUXDAC3);

    ADI_API_RETURN(device);
}

static int32_t adrv9001_AuxDacValue_Get(adi_adrv9001_Device_t *device,
                                        adrv9001_BfNvsRegmapCore2ChanAddr_e baseAddr,
                                        adi_adrv9001_AuxDacs_e auxDacSel,
                                        uint16_t *auxDacValue)
{
    uint16_t dacWord = 0;

    /* Get 12 bit DAC word from selected AuxDAC */
    if (auxDacSel == ADI_ADRV9001_AUXDAC0)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac0BfGet, device, baseAddr, &dacWord);
    }
    else if (auxDacSel == ADI_ADRV9001_AUXDAC1)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac1BfGet, device, baseAddr, &dacWord);
    }
    else if (auxDacSel == ADI_ADRV9001_AUXDAC2)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac2BfGet, device, baseAddr, &dacWord);
    }
    else
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore2Auxdac3BfGet, device, baseAddr, &dacWord);
    }

    *auxDacValue = dacWord;

    ADI_API_RETURN(device);
}
/*
*********************************************************************************************************
*                                             DDAPIs
*********************************************************************************************************
*/
int32_t adi_adrv9001_AuxDac_Configure(adi_adrv9001_Device_t *device,
                                      adi_adrv9001_AuxDacs_e auxDac,
                                      bool enable)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    uint8_t bfEnable = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_AuxDac_Configure_Validate, device, auxDac);

    /* Set AuxDAC configuration */
    ADI_MSG_EXPECT("Unable to set AuxDAC configuration", adrv9001_AuxDac_Configure, device, auxDac);

    /* Read AuxDAC enable status */
    ADI_MSG_EXPECT("Error while reading AuxDACs enabled bits",
                   adrv9001_AuxDacBf_Get,
                   device,
                   ADRV9001_BF_CORE_2,
                   &bfEnable);

    /* Power up/down selected AuxDACs at this channel */
    bfEnable = bfEnable | auxDac;

    if (enable > 0)
    {
        bfEnable = bfEnable & (~auxDac);

        /* Set Analog GPIO analog pins as output */
        if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
        {
            /*Clear the error so that clean up functions execute */
            recoveryAction = adi_common_ErrorClear(&device->common);
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             device->common.error.errCode,
                             recoveryAction,
                             NULL,
                             "Error while clearing error ");
            ADI_ERROR_RETURN(device->common.error.newAction);

            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_API_FAIL,
                             ADI_COMMON_ACT_ERR_RESET_INTERFACE,
                             gpioAnalogInputOutputMask,
                             "Unable to set GPIO output direction for AuxDACs");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    ADI_MSG_EXPECT("Error while setting enable/disable for AuxDACs",
                   adrv9001_AuxDacBf_Set,
                   device,
                   ADRV9001_BF_CORE_2,
                   bfEnable);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_AuxDac_Inspect(adi_adrv9001_Device_t *device,
                                    adi_adrv9001_AuxDacs_e auxDac,
                                    bool *enabled)
{
    uint8_t bfEnable = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_AuxDac_Inspect_Validate, device, auxDac, enabled);

    /* Read AuxDAC enable status */
    ADI_MSG_EXPECT("Error while reading AuxDACs enabled bits",
                   adrv9001_AuxDacBf_Get,
                   device,
                   ADRV9001_BF_CORE_2,
                   &bfEnable);

    bfEnable = (~bfEnable) & (uint8_t)auxDac;
    *enabled = (bool)bfEnable;

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_AuxDac_Code_Set(adi_adrv9001_Device_t *device,
                                     adi_adrv9001_AuxDacs_e auxDac,
                                     uint16_t auxDacCode)
{
    ADI_PERFORM_VALIDATION(adi_adrv9001_AuxDac_Code_Set_Validate, device, auxDac, auxDacCode);

    /* Set 12 bit AuxDAC word */
    ADI_MSG_EXPECT("Unable to set 12 bit AuxDAC word",
                   adrv9001_AuxDacValue_Set,
                   device,
                   auxDac,
                   auxDacCode);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_AuxDac_Code_Get(adi_adrv9001_Device_t *device,
                                     adi_adrv9001_AuxDacs_e auxDac,
                                     uint16_t *auxDacCode)
{
    uint16_t auxDacCodeTmp = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_AuxDac_Code_Get_Validate, device, auxDac, auxDacCode);

    /* Get 12 bit DAC word from selected AuxDAC */
    ADI_MSG_EXPECT("Unable to get 12 bit AuxDAC word for the selected AuxDAC",
                   adrv9001_AuxDacValue_Get,
                   device,
                   ADRV9001_BF_CORE_2,
                   auxDac,
                   &auxDacCodeTmp);

    *auxDacCode = auxDacCodeTmp;

    ADI_API_RETURN(device);
}
