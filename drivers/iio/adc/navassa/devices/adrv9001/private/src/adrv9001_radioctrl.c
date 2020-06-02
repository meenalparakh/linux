/**
 * \file
 * \brief Contains ADRV9001 radio control related private function implementations
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adrv9001_radioctrl.h"
#include "adrv9001_bf_nvs_regmap_core_1.h"

int32_t adrv9001_PllGpInterruptMaskGet(adi_adrv9001_Device_t *device, adi_adrv9001_PllName_e pllNameSel, uint8_t *pllGpInterruptMask1)
{
    uint8_t bfValue = 0;

    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, pllGpInterruptMask1);

    switch (pllNameSel)
    {
        case(ADI_ADRV9001_LO1_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskRfSynthLockBfGet,
                device,
                ADRV9001_BF_CORE_1,
                &bfValue);
            *pllGpInterruptMask1 = bfValue;
            break;
        }
        case(ADI_ADRV9001_LO2_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskRf2SynthLockBfGet,
                device,
                ADRV9001_BF_CORE_1,
                &bfValue);
            *pllGpInterruptMask1 = bfValue;
            break;
        }
        case(ADI_ADRV9001_AUX_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskAuxSynthLockBfGet,
                device,
                ADRV9001_BF_CORE_1,
                &bfValue);
            *pllGpInterruptMask1 = bfValue;
            break;
        }
        default:
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                pllNameSel,
                "Invalid PLL encountered while attempting to retrieve PLL GP Interrupt mask");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_PllGpInterruptMaskSet(adi_adrv9001_Device_t *device, adi_adrv9001_PllName_e pllNameSel, uint8_t pllGpInterruptMask1)
{
    /* Check device pointer is not null */
    ADI_API_PRIV_ENTRY_EXPECT(device);

    switch (pllNameSel)
    {
        case(ADI_ADRV9001_LO1_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskRfSynthLockBfSet,
                device,
                ADRV9001_BF_CORE_1,
                pllGpInterruptMask1);
            break;
        }
        case(ADI_ADRV9001_LO2_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskRf2SynthLockBfSet,
                device,
                ADRV9001_BF_CORE_1,
                pllGpInterruptMask1);
            break;
        }
        case(ADI_ADRV9001_AUX_PLL):
        {
            ADI_EXPECT(adrv9001_NvsRegmapCore1GpMaskAuxSynthLockBfSet,
                device,
                ADRV9001_BF_CORE_1,
                pllGpInterruptMask1);
            break;
        }
        default:
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                pllNameSel,
                "Invalid PLL encountered while attempting to set PLL GP Interrupt mask");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    ADI_API_RETURN(device);
}