/*!
* \file
* \brief Functions to configure the FPGA9001 AXI Advanced SPI
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include <stdbool.h>
#include "adi_fpga9001_axiadvspi.h"

int32_t adi_fpga9001_AxiAdvSpi_Configure(adi_fpga9001_Device_t *device, adi_fpga9001_AxiAdvSpiConfig_t *advSpiCfg, uint8_t slaveSelect)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t registerValue = 0;
    /* starting address is slaveSelect0's address. Multiply parameter by 4 since it's byte-addressing and each SS gets a full register */
    uint32_t registerAddress = 0x44A00080 + (slaveSelect * 4);
    
    ADI_API_ENTRY_PTR_EXPECT(device, advSpiCfg);

#ifdef ADI_FPGA9001_VERBOSE
    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_HAL);
#endif

    /* Build up the registerValue */
    registerValue |= ((uint32_t)advSpiCfg->enable & 0x1) << 0;
    registerValue |= ((uint32_t)advSpiCfg->cpol & 0x1) << 1;
    registerValue |= ((uint32_t)advSpiCfg->cpha & 0x1) << 2;
    registerValue |= ((uint32_t)advSpiCfg->lsbFirst & 0x1) << 3;
    registerValue |= ((uint32_t)advSpiCfg->fourWire & 0x1) << 4;
    registerValue |= ((uint32_t)advSpiCfg->misoSampleSlip & 0x1) << 5;
    registerValue |= ((uint32_t)advSpiCfg->threeWireTriPoint & 0xFF) << 8;
    registerValue |= ((uint32_t)advSpiCfg->clockDiv & 0xFFFF) << 16;
    
    recoveryAction = adi_hal_BbicRegisterWrite(device->common.devHalInfo, registerAddress, registerValue);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, recoveryAction, ADI_COMMON_ACT_ERR_CHECK_PARAM, advSpiCfg, "Error configuring Advanced SPI");
    ADI_ERROR_RETURN(device->common.error.newAction);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_AxiAdvSpi_SelectAllChips(adi_fpga9001_Device_t *device, uint8_t enable)
{
    /* Manual Slave Select Assertion Enable Bit */
    static const uint32_t MAN_SS_EN = 0x00000080;
    
    /* ADRV9001 Device/s slave select indices*/
    /* TODO: This variable should be calculated at Board Discover in future */
    static const uint32_t ADRV9001_CS = 0x00000001;
    
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    
    /* Address of SPI control register */
    uint32_t spicrAddress = 0x44A00060;

    /* Address of SPI slave select register */
    uint32_t spissrAddress = 0x44A00070;

    uint32_t spicrValue = 0;
    
    recoveryAction = adi_hal_BbicRegisterRead(device->common.devHalInfo, spicrAddress, &spicrValue);
    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, recoveryAction, ADI_COMMON_ACT_ERR_CHECK_PARAM, spicrAddress, "Error reading spicrAddress");
    ADI_ERROR_RETURN(device->common.error.newAction);
    
    if (enable == true)
    {
        /* Select all ADRV9001 devices */
        recoveryAction = adi_hal_BbicRegisterWrite(device->common.devHalInfo, spissrAddress, ~ADRV9001_CS);
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, recoveryAction, ADI_COMMON_ACT_ERR_CHECK_PARAM, spissrAddress, "Error selecting all ADRV9001 devices");
        ADI_ERROR_RETURN(device->common.error.newAction);
        
        /* Enable manual select mode */
        recoveryAction = adi_hal_BbicRegisterWrite(device->common.devHalInfo, spicrAddress, (spicrValue | MAN_SS_EN));
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, recoveryAction, ADI_COMMON_ACT_ERR_CHECK_PARAM, spicrAddress, "Error enabling manual select mode");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    else
    {
        /* Disable manual select mode */
        recoveryAction = adi_hal_BbicRegisterWrite(device->common.devHalInfo, spicrAddress, (spicrValue & (~MAN_SS_EN)));
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, recoveryAction, ADI_COMMON_ACT_ERR_CHECK_PARAM, spicrAddress, "Error disabling manual select mode");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}
