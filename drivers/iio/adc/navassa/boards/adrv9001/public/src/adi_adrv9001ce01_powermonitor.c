/**
* \file
* \brief Functions for interacting with the power monitoring chips on the CE board
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_platform.h"

#include "adi_adrv9001ce01_powermonitor.h"

int32_t adi_adrv9001Ce01_PowerMonitor_Configure(adi_adrv9001Ce01_Board_t *adrv9001Ce01)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t i = 0;
    adi_adrv9001Ce01_PowerMonitor_e powerMonitors[] = { 
        ADI_ADRV9001CE01_POWERMONITOR_VDD_1P0,
        ADI_ADRV9001CE01_POWERMONITOR_VDDA_1P0,
        ADI_ADRV9001CE01_POWERMONITOR_VDD_1P8,
        ADI_ADRV9001CE01_POWERMONITOR_VDDA_1P8,
        ADI_ADRV9001CE01_POWERMONITOR_VDDA_1P3
    };
    uint8_t wrData[][3] = {
        { 0xD4, 0x14, 0x07 },
        { 0xD4, 0x14, 0x07 },
        { 0xD4, 0x18, 0x07 },
        { 0xD4, 0x18, 0x07 },
        { 0xD4, 0x18, 0x07 },
    };
    
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ce01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ce01->parent.adrv9001Device);
    
    for (i = 0; i < ADI_ARRAY_LEN(powerMonitors); i++)
    {
        recoveryAction = adi_hal_I2C_Write(adrv9001Ce01->common.devHalInfo, powerMonitors[i], wrData[i], ADI_ARRAY_LEN(wrData[i]));
        ADI_ERROR_RETURN(recoveryAction);
    }
    
    return recoveryAction;
}

int32_t adi_adrv9001Ce01_PowerMonitor_Read(adi_adrv9001Ce01_Board_t *adrv9001Ce01,
                                           adi_adrv9001Ce01_PowerMonitor_e powerMonitor,
                                           float *measuredVoltage_mV,
                                           float *measuredCurrent_mA)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t buff[2] = { 0 };
    uint16_t voltage = 0;
    uint16_t current = 0;
    
    static const uint8_t VOLTAGE_ADDR = 0x88;
    static const uint8_t CURRENT_ADDR = 0x8c;
    static const float VOLTAGE_OFFSET = 0.5;
    static const float CURRENT_OFFSET = 1.0;
    
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ce01);
    ADI_NULL_DEVICE_PTR_RETURN(adrv9001Ce01->parent.adrv9001Device);
    
    recoveryAction = adi_hal_I2C_Read(adrv9001Ce01->common.devHalInfo, powerMonitor, &VOLTAGE_ADDR, 1, buff, 2);
    ADI_ERROR_REPORT(&adrv9001Ce01->common, 
                     ADI_COMMON_ERRSRC_ADI_HAL,
                     ADI_COMMON_ERR_API_FAIL,
                     ADI_COMMON_ACT_ERR_RESET_FEATURE,
                     NULL,
                     "i2c read failed");
    ADI_ERROR_RETURN(recoveryAction);
    voltage = ((uint16_t)buff[1] << 8) | buff[0];
    
    recoveryAction = adi_hal_I2C_Read(adrv9001Ce01->common.devHalInfo, powerMonitor, &CURRENT_ADDR, 1, buff, 2);
    ADI_ERROR_REPORT(&adrv9001Ce01->common, 
                     ADI_COMMON_ERRSRC_ADI_HAL,
                     ADI_COMMON_ERR_API_FAIL,
                     ADI_COMMON_ACT_ERR_RESET_FEATURE,
                     NULL,
                     "i2c read failed");
    ADI_ERROR_RETURN(recoveryAction);
    current = ((uint16_t)buff[1] << 8) | buff[0];
    
    if ((ADI_ADRV9001CE01_POWERMONITOR_VDD_1P0 == powerMonitor) ||
        (ADI_ADRV9001CE01_POWERMONITOR_VDDA_1P0 == powerMonitor))
    {
        *measuredVoltage_mV = ((float)voltage + VOLTAGE_OFFSET) * 0.3;
    }
    else
    {
        *measuredVoltage_mV = ((float)voltage + VOLTAGE_OFFSET) * 1.801;
    }
    
    *measuredCurrent_mA = current == 0xFFFF ? 0.0 : ((float)current + CURRENT_OFFSET) * 1.25;
    
    return recoveryAction;
}