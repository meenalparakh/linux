/**
* \file
* \brief Contains ADI Transceiver Hardware Abstraction functions
*        Analog Devices maintains and provides updates to this code layer.
*        The end user should not modify this file or any code in this directory.
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

/* Intermediate platform HAL layer maintained by Analog Devices */

#include <stdio.h>

#include "adi_fpga9001_hal.h"
#include "adi_platform.h"


int32_t adi_fpga9001_hal_Register_Read(adi_fpga9001_Device_t *device, uint32_t addr, uint32_t *data)
{
    int32_t halError = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

#ifdef ADI_FPGA9001_VERBOSE
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);
#endif

    halError = adi_hal_BbicRegisterRead(device->common.devHalInfo, addr, data);
    ADI_ERROR_REPORT(&device->common, 2, (adi_common_ErrSources_e)halError, ADI_COMMON_ACT_ERR_RESET_FULL, NULL, "FPGA AXI Reg Read error");
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_hal_Register_Write(adi_fpga9001_Device_t *device, uint32_t addr, uint32_t data)
{
    int32_t halError = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

#ifdef ADI_FPGA9001_VERBOSE
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);
#endif

    halError = adi_hal_BbicRegisterWrite(device->common.devHalInfo, addr, data);
    ADI_ERROR_REPORT(&device->common, 2, (adi_common_ErrSources_e)halError, ADI_COMMON_ACT_ERR_RESET_FULL, NULL, "FPGA Reg write error");
    ADI_ERROR_RETURN(device->common.error.newAction);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_hal_Field_Read(adi_fpga9001_Device_t *device,
    const uint32_t addr,
    uint32_t *fieldVal,
    uint32_t mask,
    uint8_t startBit)
{
    uint32_t rdData = 0;
    uint32_t regVal = 0;
    //uint8_t i = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

#ifdef ADI_FPGA9001_VERBOSE
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);
#endif

    ADI_NULL_PTR_RETURN(&device->common, fieldVal);

    ADI_EXPECT(adi_fpga9001_hal_Register_Read, device, addr, &rdData);

    regVal = rdData;
    *fieldVal = (regVal & mask) >> startBit;

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_hal_Field_Write(adi_fpga9001_Device_t *device, const uint32_t addr, uint32_t fieldVal, uint32_t mask, uint8_t startBit)
{
    uint32_t registerVal = 0;
    uint32_t newRegVal = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

#ifdef ADI_FPGA9001_VERBOSE
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);
#endif

    ADI_EXPECT(adi_fpga9001_hal_Register_Read, device, addr, &registerVal);

    /* modify the register value*/
    newRegVal = (registerVal & ~mask) | fieldVal;

    /* write to the register with modified value*/
    ADI_EXPECT(adi_fpga9001_hal_Register_Write, device, addr, newRegVal);

    ADI_API_RETURN(device);
}

/**
* \brief Read from the ZC706 memory
*/
int32_t adi_fpga9001_hal_Ram_Read(adi_fpga9001_Device_t *device, const uint32_t ramAddress, uint32_t data[], uint32_t length)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

#ifdef ADI_FPGA9001_VERBOSE
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);
#endif

    if (ramAddress % sizeof(uint32_t) != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, ramAddress, "RAM address must be word aligned");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    recoveryAction = adi_hal_BbicRegistersRead(device->common.devHalInfo, ramAddress, data, length);

    return recoveryAction;
}


/**
* \brief Write to the ZC706 memory
*/
int32_t adi_fpga9001_hal_Ram_Write(adi_fpga9001_Device_t *device, const uint32_t ramAddress, uint32_t data[], uint32_t length)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

#ifdef ADI_FPGA9001_VERBOSE
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);
#endif

    if (ramAddress % sizeof(uint32_t) != 0)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_INV_PARAM, ADI_COMMON_ACT_ERR_CHECK_PARAM, ramAddress, "RAM address must be word aligned");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    recoveryAction = adi_hal_BbicRegistersWrite(device->common.devHalInfo, ramAddress, data, length);

    return recoveryAction;
}
