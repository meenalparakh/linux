/**
 * \file fpga9001_bf_zc706.c Automatically generated file with generator ver 0.0.1.0.
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

#include "fpga9001_bf_zc706.h"
#include "fpga9001_bf_hal.h"
#include "adi_common_error.h"

/**
 * \brief External Trigger 0 Direction
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrig0DirBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrig0DirBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrig0DirBfSet Address");
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
 * \brief External Trigger 0 Direction
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrig0DirBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrig0DirBfGet Address");
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
 * \brief External Trigger 0 Output Source
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrig0OutputSrcBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrig0OutputSrcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrig0OutputSrcBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x20), (uint32_t)bfValue, 0x70, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief External Trigger 0 Output Source
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrig0OutputSrcBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrig0OutputSrcBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x70, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief External Trigger 1 Direction
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrig1DirBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrig1DirBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrig1DirBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x20), (uint32_t)bfValue, 0x100, 0x8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief External Trigger 1 Direction
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrig1DirBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrig1DirBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x100, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief External Trigger 1 Output Source
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrig1OutputSrcBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrig1OutputSrcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrig1OutputSrcBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x20), (uint32_t)bfValue, 0x7000, 0xC);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief External Trigger 1 Output Source
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrig1OutputSrcBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrig1OutputSrcBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x7000, 0xCC);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief External Trigger Source for Receive Data Path 00 Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigRxdp00CtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrigRxdp00CtrlSrcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigRxdp00CtrlSrcBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x24), (uint32_t)bfValue, 0x30, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief External Trigger Source for Receive Data Path 00 Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigRxdp00CtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigRxdp00CtrlSrcBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x24), &rxBfData[0], 0x30, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief External Trigger Source for Receive Data Path 01 Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigRxdp01CtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrigRxdp01CtrlSrcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigRxdp01CtrlSrcBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x24), (uint32_t)bfValue, 0x300, 0x8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief External Trigger Source for Receive Data Path 01 Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigRxdp01CtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigRxdp01CtrlSrcBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x24), &rxBfData[0], 0x300, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief External Trigger Source for TDD State Machine Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigTddCtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrigTddCtrlSrcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigTddCtrlSrcBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x24), (uint32_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief External Trigger Source for TDD State Machine Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigTddCtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigTddCtrlSrcBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x24), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief External Trigger Source for Transmit Data Path 00 Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigTxdp00CtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrigTxdp00CtrlSrcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigTxdp00CtrlSrcBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x24), (uint32_t)bfValue, 0x30000, 0x10);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief External Trigger Source for Transmit Data Path 00 Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigTxdp00CtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigTxdp00CtrlSrcBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x24), &rxBfData[0], 0x30000, 0xD0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief External Trigger Source for Transmit Data Path 01 Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigTxdp01CtrlSrcBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706ExtTrigTxdp01CtrlSrcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigTxdp01CtrlSrcBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x24), (uint32_t)bfValue, 0x300000, 0x14);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief External Trigger Source for Transmit Data Path 01 Control
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706ExtTrigTxdp01CtrlSrcBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706ExtTrigTxdp01CtrlSrcBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x24), &rxBfData[0], 0x300000, 0xD4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Global Reset
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706GlobalResetBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706GlobalResetBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706GlobalResetBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Global Reset
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706GlobalResetBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706GlobalResetBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief IIC Mux Reset
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706IicMuxResetBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706IicMuxResetBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706IicMuxResetBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief IIC Mux Reset
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706IicMuxResetBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706IicMuxResetBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief LED Blinking Mode Blink Duration
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedBlinkDurationBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 65535U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_Zc706LedBlinkDurationBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedBlinkDurationBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x14), (uint32_t)bfValue, 0xFFFF0000, 0x10);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief LED Blinking Mode Blink Duration
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedBlinkDurationBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfZc706ChanAddr_e baseAddr, 
                                             uint16_t *bfValue)
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedBlinkDurationBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0xFFFF0000, 0xD0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief LED Mode Change Delay in Milliseconds
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedDelayToModeChangeBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                 uint16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 65535U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_Zc706LedDelayToModeChangeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedDelayToModeChangeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x10), (uint32_t)bfValue, 0xFFFF0000, 0x10);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief LED Mode Change Delay in Milliseconds
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedDelayToModeChangeBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfZc706ChanAddr_e baseAddr, 
                                                 uint16_t *bfValue)
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedDelayToModeChangeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x10), &rxBfData[0], 0xFFFF0000, 0xD0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief LED Mask
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedMaskBfSet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706LedMaskBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedMaskBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x18), (uint32_t)bfValue, 0xF00, 0x8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief LED Mask
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedMaskBfGet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedMaskBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x18), &rxBfData[0], 0xF00, 0xC8);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief LED Mode
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedModeBfSet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfZc706ChanAddr_e baseAddr, 
                                    uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#if FPGA9001_BITFIELD_VALUE_CHECK > 0
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 255U))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM, 
                         bfValue,
                         "Invalid fpga9001_Zc706LedModeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedModeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x10), (uint32_t)bfValue, 0xFF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief LED Mode
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedModeBfGet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedModeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x10), &rxBfData[0], 0xFF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief LED Blinking Mode Number of Blinks
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedNumBlinksBfSet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706LedNumBlinksBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedNumBlinksBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x14), (uint32_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief LED Blinking Mode Number of Blinks
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedNumBlinksBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedNumBlinksBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief LED Software Define Value
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedSwDefinedValueBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_Zc706LedSwDefinedValueBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedSwDefinedValueBfSet Address");
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
 * \brief LED Software Define Value
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706LedSwDefinedValueBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706LedSwDefinedValueBfGet Address");
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
 * \brief Major Version
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706VersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfZc706ChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706VersionMajorBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0xFF0000, 0xD0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief Minor Version
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfZc706ChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_Zc706VersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfZc706ChanAddr_e baseAddr, 
                                         uint16_t *bfValue)
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
    if ((baseAddr != FPGA9001_BF_ZC706_0))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_Zc706VersionMinorBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0xFFFF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/*  EOF: fpga9001_bf_zc706.c */

