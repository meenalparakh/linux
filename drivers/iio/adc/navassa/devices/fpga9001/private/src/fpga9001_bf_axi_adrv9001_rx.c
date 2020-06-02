/**
 * \file fpga9001_bf_axi_adrv9001_rx.c Automatically generated file with generator ver 0.0.1.0.
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

#include "fpga9001_bf_axi_adrv9001_rx.h"
#include "fpga9001_bf_hal.h"
#include "adi_common_error.h"

/**
 * \brief The decimation ratio at the interface for the observation channel. The behavior is the same as above, except that it is applied to frames that are marked observation channel by the associated GPIO.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxOrxDecRatioBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxOrxDecRatioBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxOrxDecRatioBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x18), (uint32_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief The decimation ratio at the interface for the observation channel. The behavior is the same as above, except that it is applied to frames that are marked observation channel by the associated GPIO.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxOrxDecRatioBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxOrxDecRatioBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x18), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This field must be set to the desired interface clock rate. It supports the following data rates in relation to the interface clock. 2b00: data-rate = clock-rate, 2b01: data-rate = (clock-rate/2), 2b10: data-rate = (clock-rate/4), 2b11: data-rate = (clock-rate/8).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxClkRateBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxClkRateBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxClkRateBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field must be set to the desired interface clock rate. It supports the following data rates in relation to the interface clock. 2b00: data-rate = clock-rate, 2b01: data-rate = (clock-rate/2), 2b10: data-rate = (clock-rate/4), 2b11: data-rate = (clock-rate/8).
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxClkRateBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxClkRateBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x3, 0xC0);
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxCmos1Lvds0BfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxCmos1Lvds0BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxCmosLvdsIdBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxCmosLvdsIdBfGet Address");
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
 * \brief The data pattern expected at the interface. In non 32 bit modes, only the lower bits are used as per the NUM_OF_BITS field above.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxDataPatBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxDataPatBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxDataPatBfSet Address");
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
 * \brief The data pattern expected at the interface. In non 32 bit modes, only the lower bits are used as per the NUM_OF_BITS field above.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxDataPatBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxDataPatBfGet Address");
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
 * \brief This field selects the data types for the monitor at the interface (RX_MON_OOS and RX_MON_ERR above). The following are supported. 3b011: Fixed pattern (see RX_DATA_PAT below). 3b100: Nibble ramp (0x0000-0x1111- all the way to 0xffff). 3b101: word ramp (0x0000 to 0xffff). 3b110: PRBS15 3b111: PRBS7
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxDataSelBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
                                              uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 7U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_AxiAdrv9001RxRxDataSelBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxDataSelBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x10), (uint32_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This field selects the data types for the monitor at the interface (RX_MON_OOS and RX_MON_ERR above). The following are supported. 3b011: Fixed pattern (see RX_DATA_PAT below). 3b100: Nibble ramp (0x0000-0x1111- all the way to 0xffff). 3b101: word ramp (0x0000 to 0xffff). 3b110: PRBS15 3b111: PRBS7
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxDataSelBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxDataSelBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x10), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief The decimation ratio at the interface. If set to 0x0 all the samples are valid and passed down stream. If set to a value N, only one among N samples are considered valid and passed down stream. This is only applicable if the GPIO is set to indicate a receive channel at the interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxDecRatioBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxDecRatioBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxDecRatioBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x18), (uint32_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief The decimation ratio at the interface. If set to 0x0 all the samples are valid and passed down stream. If set to a value N, only one among N samples are considered valid and passed down stream. This is only applicable if the GPIO is set to indicate a receive channel at the interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxDecRatioBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxDecRatioBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x18), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This bit controls the data sampling edge in SDR mode, if set to 0x1, falling clock edges are used to capture data. If set to 0x0, data is sampled at the rising edges of the interface clock.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxFall1Rise0BfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxFall1Rise0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxFall1Rise0BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x200, 0x9);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This bit controls the data sampling edge in SDR mode, if set to 0x1, falling clock edges are used to capture data. If set to 0x0, data is sampled at the rising edges of the interface clock.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxFall1Rise0BfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxFall1Rise0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x200, 0xC9);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This bit indicates spurious errors in the framer, but is not sufficient to cause an OOS assertion. Software must write 0x1 to clear this bit. Once cleared, this bit is not set until another frame error is detected.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxFrmErrBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxFrmErrBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxFrmErrBfSet Address");
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
 * \brief This bit indicates spurious errors in the framer, but is not sufficient to cause an OOS assertion. Software must write 0x1 to clear this bit. Once cleared, this bit is not set until another frame error is detected.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxFrmErrBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxFrmErrBfGet Address");
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
 * \brief This bit indicates the status of the delineation of data frames based on the strobe signal at the interface. If this bit is read 0x1, the framer is in out of sync and must be considered as an error state. If this bit is read 0x0, the framer is in sync with the received data frames. As part of the initialization sequence, software must write 0x1 to clear this bit and make sure that it stays cleared.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxFrmOosBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxFrmOosBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxFrmOosBfSet Address");
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
 * \brief This bit indicates the status of the delineation of data frames based on the strobe signal at the interface. If this bit is read 0x1, the framer is in out of sync and must be considered as an error state. If this bit is read 0x0, the framer is in sync with the received data frames. As part of the initialization sequence, software must write 0x1 to clear this bit and make sure that it stays cleared.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxFrmOosBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxFrmOosBfGet Address");
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
 * \brief A 0 to 1 transition on this bit initiates a reset request at the interface. This is self-cleared, it is recommended that software set this bit after a profile is loaded to ensure a clean start.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxInitBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxInitBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxInitBfSet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxInitBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxInitBfGet Address");
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
 * \brief This field must be set to the desired bit order within the data frame. If set to 0x1, the first received bit is assumed to be the LSB, if set to 0x0, the first received bit is the MSB.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxLsb1Msb0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxLsb1Msb0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxLsb1Msb0BfSet Address");
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
 * \brief This field must be set to the desired bit order within the data frame. If set to 0x1, the first received bit is assumed to be the LSB, if set to 0x0, the first received bit is the MSB.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxLsb1Msb0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxLsb1Msb0BfGet Address");
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
 * \brief This field must be set to the desired strobe type, It supports the following two modes. If set to 1, a level strobe is expected. The strobe must stay asserted for the entire channel (I or Q) in a 2 channel interface. This bit is ignored in single channel mode. If this bit is set to 0, strobe is expected to be a pulse, asserted only during the first clock cycle of the data frame.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxLstrb1Pstrb0BfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxLstrb1Pstrb0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxLstrb1Pstrb0BfSet Address");
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
 * \brief This field must be set to the desired strobe type, It supports the following two modes. If set to 1, a level strobe is expected. The strobe must stay asserted for the entire channel (I or Q) in a 2 channel interface. This bit is ignored in single channel mode. If this bit is set to 0, strobe is expected to be a pulse, asserted only during the first clock cycle of the data frame.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxLstrb1Pstrb0BfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxLstrb1Pstrb0BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxMlane1Slane0BfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxMlane1Slane0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxMlane1Slane0BfSet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxMlane1Slane0BfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxMlane1Slane0BfGet Address");
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
 * \brief This bit indicates spurious errors in the data monitor but is not sufficient to cause an OOS assertion. The bit is cleared by writing a 0x1.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxMonErrBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxMonErrBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxMonErrBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x8), (uint32_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This bit indicates spurious errors in the data monitor but is not sufficient to cause an OOS assertion. The bit is cleared by writing a 0x1.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxMonErrBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxMonErrBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x8), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This bit indicates the data monitor OOS similar to the framer OOS, software may use this bit for the delay calibration at the interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxMonOosBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxMonOosBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxMonOosBfSet Address");
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
 * \brief This bit indicates the data monitor OOS similar to the framer OOS, software may use this bit for the delay calibration at the interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxMonOosBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxMonOosBfGet Address");
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
 * \brief This field must be set to the desired number of bits in a sample. It supports the following sample widths. 2b00: 32bits, LVDS, or CMOS 2 channel (IQ) interface. 2b01: 24bits, LVDS only 2 channel (IQ) interface. 2b01: 16bits, CMOS only 1 channel (I) interface. 2b10: 8bits, CMOS only 1 channel (I) interface. 2b11: 2bits, CMOS only 1 channel (I) interface.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxNumOfBitsBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxNumOfBitsBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxNumOfBitsBfSet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxNumOfBitsBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxNumOfBitsBfGet Address");
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
 * \brief This field must be set to the desired channel order within the data frame. If set to 0x1, the first sample is assumed to be the Q channel. If set to 0x0, the first received channel is the I.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxQ1I0BfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxQ1I0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxQ1I0BfSet Address");
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
 * \brief This field must be set to the desired channel order within the data frame. If set to 0x1, the first sample is assumed to be the Q channel. If set to 0x0, the first received channel is the I.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxQ1I0BfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxQ1I0BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxRddelay0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxRddelay0BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxRddelay1BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxRddelay1BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxRddelay2BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxRddelay2BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxRddelay3BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxRddelay3BfGet Address");
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
 * \brief This field is the same as RX_RDDELAY_0 above, but for the strobe lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxRddelaySBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxRddelaySBfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxSdr1Ddr0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxSdr1Ddr0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxSdr1Ddr0BfSet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxSdr1Ddr0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxSdr1Ddr0BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelay0BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay0BfSet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelay0BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay0BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelay1BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay1BfSet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelay1BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay1BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelay2BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay2BfSet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelay2BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay2BfGet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelay3BfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay3BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay3BfSet Address");
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
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelay3BfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelay3BfGet Address");
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
 * \brief This field is the same as RX_WRDELAY_0 above, but for the srobe lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelaySBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelaySBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelaySBfSet Address");
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
 * \brief This field is the same as RX_WRDELAY_0 above, but for the srobe lane.
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfAxiAdrv9001RxChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_AxiAdrv9001RxRxWrdelaySBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_0) &&
            (baseAddr != FPGA9001_BF_AXI_ADRV9001_RX_1))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_AxiAdrv9001RxRxWrdelaySBfGet Address");
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

/*  EOF: fpga9001_bf_axi_adrv9001_rx.c */

