/**
* \file
* \brief Core functions specific to the customer evaluation (CE) board
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_platform.h"
#include "adi_adrv9001ee01.h"
#include "adi_adrv9001ce01.h"

int32_t adi_adrv9001Ce01_Create(adi_adrv9001Ce01_Board_t *adrv9001Ce01)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    
    recoveryAction = adi_adrv9001Ee01_Create((adi_adrv9001Ee01_Board_t *)adrv9001Ce01);
    ADI_ERROR_RETURN(recoveryAction);
    
    adrv9001Ce01->common.devHalInfo = adi_hal_DevHalCfgCreate(ADI_HAL_INTERFACE_I2C, 0, "");
    if (NULL == adrv9001Ce01->common.devHalInfo)
    {
        adi_adrv9001Ce01_Destroy(adrv9001Ce01);
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
    
    recoveryAction = adi_common_hal_HwOpen(&adrv9001Ce01->common);
    
    return recoveryAction;
}

int32_t adi_adrv9001Ce01_Destroy(adi_adrv9001Ce01_Board_t *adrv9001Ce01)
{
    if (NULL == adrv9001Ce01)
    {
        return ADI_COMMON_ACT_ERR_CHECK_PARAM;
    }
    
    adi_common_hal_HwClose(&adrv9001Ce01->common);
    
    if (NULL != adrv9001Ce01->common.devHalInfo)
    {
        adi_hal_DevHalCfgFree(adrv9001Ce01->common.devHalInfo);
    }
    
    return adi_adrv9001Ee01_Destroy((adi_adrv9001Ee01_Board_t *)adrv9001Ce01);
}