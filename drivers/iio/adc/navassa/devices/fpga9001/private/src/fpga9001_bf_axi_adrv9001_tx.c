/**
 * \file fpga9001_bf_axi_adrv9001_tx.c Automatically generated file with generator ver 0.0.1.0.
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

#include "fpga9001_bf_axi_adrv9001_tx.h"
#include "fpga9001_bf_hal.h"
#include "adi_common_error.h"

/**
 * \brief The core, by default, tri-states all the IO unless explicitly enabled by software. After a profile is loaded, this bit must be set to 0x1 to enable all the output buffers in both LVDS and CMOS mode
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxBufEnableBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxBufEnableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxBufEnableBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x8), (uint32_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief The core, by default, tri-states all the IO unless explicitly enabled by software. After a profile is loaded, this bit must be set to 0x1 to enable all the output buffers in both LVDS and CMOS mode
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxBufEnableBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxBufEnableBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x8), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief The transmit interface clock output may be inverted by setting this field to 0x1, as it controls the MSB bit in the clock output.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxClkMsbBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxClkMsbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxClkMsbBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x8), (uint32_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief The transmit interface clock output may be inverted by setting this field to 0x1, as it controls the MSB bit in the clock output.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxClkMsbBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxClkMsbBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x8), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief If this field is set, the transmit interface internal clock is sourced from the receive interface clock, otherwise, the transmit reference clock input sources the transmit interface internal clock.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxClkSelBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxClkSelBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxClkSelBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x8), (uint32_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief If this field is set, the transmit interface internal clock is sourced from the receive interface clock, otherwise, the transmit reference clock input sources the transmit interface internal clock.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxClkSelBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxClkSelBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x8), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief This is a read-only bit indicating the current configuration of this interface. If read 0x1, interface is in CMOS mode. If read 0x0, it is in LVDS mode. The software is expected to qualify the remaining fields in this register based on the value read in this bit.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxCmos1Lvds0BfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxCmos1Lvds0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x400, 0xCA);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief An unambigous identifier indicates the current configuration of SSI interface. If configured as CMOS, this register reads 0x43-0x4d-0x4f-0x53, and if LVDS, reads 0x4c-0x56-0x44-0x53.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxCmosLvdsIdBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxCmosLvdsIdBfGet Address");
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
 * \brief The data pattern transmitted at the interface. In non 32 bit modes, only the lower bits are used as per the NUM_OF_BITS field above.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxDataPatBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxDataPatBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxDataPatBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Register */
    recoveryAction = fpga9001_BfRegisterWrite(device, (baseAddr + 0x14), ((uint32_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief The data pattern transmitted at the interface. In non 32 bit modes, only the lower bits are used as per the NUM_OF_BITS field above.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxDataPatBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxDataPatBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Register */
    recoveryAction = fpga9001_BfRegisterRead(device, (baseAddr + 0x14), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief This field selects the transmit data source. The following are supported: 4b0000: DMA. 4b0001: Receive loop back. 4b0010: Zeros 4b0011: Pattern (see TX_DATA_PAT below). 4b0100: Nibble ramp 4b0101: Word ramp 4b0110: PRBS15 4b0111: PRBS7 4b1000: Fs/8 tone @ -12 dBfs.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxDataSelBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                              uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 15U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001TxTxDataSelBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxDataSelBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x10), (uint32_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field selects the transmit data source. The following are supported: 4b0000: DMA. 4b0001: Receive loop back. 4b0010: Zeros 4b0011: Pattern (see TX_DATA_PAT below). 4b0100: Nibble ramp 4b0101: Word ramp 4b0110: PRBS15 4b0111: PRBS7 4b1000: Fs/8 tone @ -12 dBfs.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxDataSelBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxDataSelBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x10), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief A 0 to 1 transition on this bit initiates a reset request at the interface. This is self-cleared, it is recommended that software set this bit after a profile is loaded to ensure a clean start.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxInitBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxInitBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxInitBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0xC), (uint32_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief A 0 to 1 transition on this bit initiates a reset request at the interface. This is self-cleared, it is recommended that software set this bit after a profile is loaded to ensure a clean start.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxInitBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxInitBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field must be set to the desired bit order within the data frame. If set to 0x1, the samples are transmitted lsb first, otherwise msb first.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxLsb1Msb0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxLsb1Msb0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxLsb1Msb0BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field must be set to the desired bit order within the data frame. If set to 0x1, the samples are transmitted lsb first, otherwise msb first.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxLsb1Msb0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxLsb1Msb0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field must be set to the desired strobe type, It supports the following two modes. If set to 1 and is a two channel interface, a level strobe is asserted for the entire first channel. If this bit is set to 0, strobe is generated as a pulse, asserted only during the first clock cycle of the data frame.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxLstrb1Pstrb0BfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxLstrb1Pstrb0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxLstrb1Pstrb0BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field must be set to the desired strobe type, It supports the following two modes. If set to 1 and is a two channel interface, a level strobe is asserted for the entire first channel. If this bit is set to 0, strobe is generated as a pulse, asserted only during the first clock cycle of the data frame.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxLstrb1Pstrb0BfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxLstrb1Pstrb0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field must be set to the desired number of lanes (or lines) at the interface. If this bit is set to 0x1, the interface is a multiple lane (or line) interface. In CMOS mode this is 4 lines, in LVDS it is 2 lanes. If this bit is set to 0x0, it is a single lane (or line) interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxMlane1Slane0BfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxMlane1Slane0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxMlane1Slane0BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field must be set to the desired number of lanes (or lines) at the interface. If this bit is set to 0x1, the interface is a multiple lane (or line) interface. In CMOS mode this is 4 lines, in LVDS it is 2 lanes. If this bit is set to 0x0, it is a single lane (or line) interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxMlane1Slane0BfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxMlane1Slane0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field must be set to the desired number of bits in a sample. It supports the following sample widths. 2b00: 32bits, LVDS, or CMOS 2 channel (IQ) interface. 2b01: 24bits, LVDS only 2 channel (IQ) interface. 2b01: 16bits, CMOS only 1 channel (I) interface. 2b10: 8bits, CMOS only 1 channel (I) interface. 2b11: 2bits, CMOS only 1 channel (I) interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxNumOfBitsBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                                uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001TxTxNumOfBitsBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxNumOfBitsBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0xC, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field must be set to the desired number of bits in a sample. It supports the following sample widths. 2b00: 32bits, LVDS, or CMOS 2 channel (IQ) interface. 2b01: 24bits, LVDS only 2 channel (IQ) interface. 2b01: 16bits, CMOS only 1 channel (I) interface. 2b10: 8bits, CMOS only 1 channel (I) interface. 2b11: 2bits, CMOS only 1 channel (I) interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxNumOfBitsBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxNumOfBitsBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0xC, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field must be set to the desired channel order within the data frame. If set to 0x1, the Q data is sent as the first sample, otherwise I data.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxQ1I0BfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxQ1I0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxQ1I0BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x100, 0x8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field must be set to the desired channel order within the data frame. If set to 0x1, the Q data is sent as the first sample, otherwise I data.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxQ1I0BfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxQ1I0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x100, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief This field indicates the current delay setting at the interface for data lane 1. If done correctly, the value read is the same as the RX_WRDELAY.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxRddelay0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxRddelay0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x1F00, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_RDDELAY_0 above, but for the data lane 2.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxRddelay1BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxRddelay1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x24), &rxBfData[0], 0x1F00, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_RDDELAY_0 above, but for the data lane 3. It is unused in LVDS mode.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxRddelay2BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxRddelay2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x28), &rxBfData[0], 0x1F00, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_RDDELAY_0 above, but for the data lane 4. It is unused in LVDS mode.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxRddelay3BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxRddelay3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x2C), &rxBfData[0], 0x1F00, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_RDDELAY_0 above, but for the clock lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxRddelayCBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxRddelayCBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x34), &rxBfData[0], 0x1F00, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_RDDELAY_0 above, but for the strobe lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxRddelaySBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxRddelaySBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x30), &rxBfData[0], 0x1F00, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field must be set to the desired clock type. If set to 0x1, interface is SDR. If set to 0x0, interface is in DDR mode. This bit must be set carefully to match the device. The FPGA core supports SDR or DDR in all modes.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxSdr1Ddr0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001TxTxSdr1Ddr0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxSdr1Ddr0BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field must be set to the desired clock type. If set to 0x1, interface is SDR. If set to 0x0, interface is in DDR mode. This bit must be set carefully to match the device. The FPGA core supports SDR or DDR in all modes.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxSdr1Ddr0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxSdr1Ddr0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field sets the IO delay value at the interface for data lane 1. The resolution is ~78ps. The range is 0 to 31.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelay0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 31U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay0BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x20), (uint32_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field sets the IO delay value at the interface for data lane 1. The resolution is ~78ps. The range is 0 to 31.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelay0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the data lane 2.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelay1BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 31U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x24), (uint32_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the data lane 2.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelay1BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x24), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the data lane 3. It is unused in LVDS mode.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelay2BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 31U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x28), (uint32_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the data lane 3. It is unused in LVDS mode.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelay2BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x28), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the data lane 4. It is unused in LVDS mode.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelay3BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 31U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay3BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay3BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x2C), (uint32_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the data lane 4. It is unused in LVDS mode.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelay3BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelay3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x2C), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the clock lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelayCBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 31U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelayCBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelayCBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x34), (uint32_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the clock lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelayCBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelayCBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x34), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the strobe lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelaySBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 31U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelaySBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelaySBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x30), (uint32_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field is the same as RX_WRDELAY_0 above, but for the strobe lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001TxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001TxTxWrdelaySBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_TX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001TxTxWrdelaySBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x30), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/*  EOF: fpga9001_bf_axi_adrv9001_tx.c */

