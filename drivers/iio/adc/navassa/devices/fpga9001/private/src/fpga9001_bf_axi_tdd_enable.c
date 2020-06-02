/**
 * \file fpga9001_bf_axi_tdd_enable.c Automatically generated file with generator ver 0.0.1.0.
 *
 * \brief Contains BitField functions to support FPGA9001 transciever device.
 *
 * FPGA9001 BITFIELD VERSION: 0.0.0.1
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "fpga9001_bf_axi_tdd_enable.h"
#include "fpga9001_bf_hal.h"
#include "adi_common_error.h"

/**
 * \brief Set to 0x1 to enable dma.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnableDmamanualassertBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                   uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnableDmamanualassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnableDmamanualassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x40), (uint32_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to 0x1 to enable dma.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnableDmamanualassertBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                   uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnableDmamanualassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x40), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief This field uniquely identifies this module refer to the top level for the value of this field.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnableInstanceIdBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                              uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnableInstanceIdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x0), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to the sequence count value to set frames as active. (Sequenced mode only).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinframeprimaryassertBfSet(adi_fpga9001_Device_t *device, 
                                                        fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                         uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4294967295U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinframeprimaryassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinframeprimaryassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0x24), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to the sequence count value to set frames as active. (Sequenced mode only).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinframeprimaryassertBfGet(adi_fpga9001_Device_t *device, 
                                                        fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                         uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinframeprimaryassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x24), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to the sequence count value to set frames as inactive. (Sequenced mode only).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinframeprimarydeassertBfSet(adi_fpga9001_Device_t *device, 
                                                          fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                           uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4294967295U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinframeprimarydeassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinframeprimarydeassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0x28), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to the sequence count value to set frames as inactive. (Sequenced mode only).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinframeprimarydeassertBfGet(adi_fpga9001_Device_t *device, 
                                                          fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                           uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinframeprimarydeassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x28), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to the sequence count value to set frames as active. (Sequenced mode only).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinframesecondaryassertBfSet(adi_fpga9001_Device_t *device, 
                                                          fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                           uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4294967295U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinframesecondaryassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinframesecondaryassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0x2C), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to the sequence count value to set frames as active. (Sequenced mode only).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinframesecondaryassertBfGet(adi_fpga9001_Device_t *device, 
                                                          fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                           uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinframesecondaryassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x2C), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to the sequence count value to set frames as active. (Sequenced mode only).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinframesecondarydeassertBfSet(adi_fpga9001_Device_t *device, 
                                                            fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                             uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4294967295U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinframesecondarydeassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinframesecondarydeassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0x30), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to the sequence count value to set frames as active. (Sequenced mode only).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinframesecondarydeassertBfGet(adi_fpga9001_Device_t *device, 
                                                            fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                             uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinframesecondarydeassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x30), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to 0x1 to assert tdd signal when in manual tdd mode. (Has no effect in programmed tdd mode).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinmanualassertBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                   uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinmanualassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinmanualassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x20), (uint32_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to 0x1 to assert tdd signal when in manual tdd mode. (Has no effect in programmed tdd mode).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinmanualassertBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                   uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinmanualassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Set to the count value within a tdd frame to assert the enable signal.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinprimaryassertBfSet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                    uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4294967295U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinprimaryassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinprimaryassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0x4), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to the count value within a tdd frame to assert the enable signal.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinprimaryassertBfGet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                    uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinprimaryassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x4), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to the count value within a tdd frame to de-assert the enable signal.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinprimarydeassertBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                      uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4294967295U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinprimarydeassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinprimarydeassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0x8), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to the count value within a tdd frame to de-assert the enable signal.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinprimarydeassertBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                      uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinprimarydeassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x8), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to 0x1 to enable programmed tdd frame activity. (Set to 0 for manual tdd mode).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinprimaryenableBfSet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                    uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinprimaryenableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinprimaryenableBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x20), (uint32_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to 0x1 to enable programmed tdd frame activity. (Set to 0 for manual tdd mode).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinprimaryenableBfGet(adi_fpga9001_Device_t *device, 
                                                   fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                    uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinprimaryenableBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Set to the count value within a tdd frame to assert the enable signal.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinsecondaryassertBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                      uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4294967295U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinsecondaryassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinsecondaryassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0xC), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to the count value within a tdd frame to assert the enable signal.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinsecondaryassertBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                      uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinsecondaryassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0xC), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to the count value within a tdd frame to de-assert the enable signal.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinsecondarydeassertBfSet(adi_fpga9001_Device_t *device, 
                                                       fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                        uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4294967295U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinsecondarydeassertBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinsecondarydeassertBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0x10), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to the count value within a tdd frame to de-assert the enable signal.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinsecondarydeassertBfGet(adi_fpga9001_Device_t *device, 
                                                       fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                        uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinsecondarydeassertBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x10), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief Set to 0x1 to enable secondary assert/deassert activity in programmed tdd mode.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinsecondaryenableBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                      uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiTddEnablePinsecondaryenableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinsecondaryenableBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x20), (uint32_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set to 0x1 to enable secondary assert/deassert activity in programmed tdd mode.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiTddEnableChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiTddEnablePinsecondaryenableBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiTddEnableChanAddr_e baseAddr, 
                                                      uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_NULL_CHECK > 0
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* FPGA9001_BITFIELD_NULL_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_TX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_RX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_ORX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_AUX_1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL0_CONTROL2) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL1) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TDD_ENABLE_CTL1_CONTROL2))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiTddEnablePinsecondaryenableBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/*  EOF: fpga9001_bf_axi_tdd_enable.c */

