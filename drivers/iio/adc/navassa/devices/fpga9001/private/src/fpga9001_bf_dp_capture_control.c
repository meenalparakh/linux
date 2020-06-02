/**
 * \file fpga9001_bf_dp_capture_control.c Automatically generated file with generator ver 0.0.1.0.
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

#include "fpga9001_bf_dp_capture_control.h"
#include "fpga9001_bf_hal.h"
#include "adi_common_error.h"

/**
 * \brief Data Path Active
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfDpCaptureControlChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_DpCaptureControlDatapathActiveBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_DpCaptureControlDatapathActiveBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_RX_DP_CAPTURE_CONTROL) &&
            (baseAddr != FPGA9001_BF_TX_DP_CAPTURE_CONTROL))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_DpCaptureControlDatapathActiveBfSet Address");
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
 * \brief Data Path Active
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfDpCaptureControlChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_DpCaptureControlDatapathActiveBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_RX_DP_CAPTURE_CONTROL) &&
            (baseAddr != FPGA9001_BF_TX_DP_CAPTURE_CONTROL))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_DpCaptureControlDatapathActiveBfGet Address");
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
 * \brief Module Major Version
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfDpCaptureControlChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_DpCaptureControlModuleVersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                                         fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_RX_DP_CAPTURE_CONTROL) &&
            (baseAddr != FPGA9001_BF_TX_DP_CAPTURE_CONTROL))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_DpCaptureControlModuleVersionMajorBfGet Address");
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
 * \brief Module Minor Version
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfDpCaptureControlChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_DpCaptureControlModuleVersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                                         fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_RX_DP_CAPTURE_CONTROL) &&
            (baseAddr != FPGA9001_BF_TX_DP_CAPTURE_CONTROL))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_DpCaptureControlModuleVersionMinorBfGet Address");
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
/**
 * \brief Pass Token
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfDpCaptureControlChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_DpCaptureControlPassTokenBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_DpCaptureControlPassTokenBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_RX_DP_CAPTURE_CONTROL) &&
            (baseAddr != FPGA9001_BF_TX_DP_CAPTURE_CONTROL))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_DpCaptureControlPassTokenBfSet Address");
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
 * \brief Pass Token
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfDpCaptureControlChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_DpCaptureControlPassTokenBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_RX_DP_CAPTURE_CONTROL) &&
            (baseAddr != FPGA9001_BF_TX_DP_CAPTURE_CONTROL))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_DpCaptureControlPassTokenBfGet Address");
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
 * \brief Reset
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfDpCaptureControlChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_DpCaptureControlResetBfSet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
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
                         "Invalid fpga9001_DpCaptureControlResetBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_VALUE_CHECK */

#if FPGA9001_BITFIELD_ADDR_CHECK > 0
    /* Base Address check */
    if ((baseAddr != FPGA9001_BF_RX_DP_CAPTURE_CONTROL) &&
            (baseAddr != FPGA9001_BF_TX_DP_CAPTURE_CONTROL))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_DpCaptureControlResetBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = fpga9001_BfFieldWrite(device, (baseAddr + 0x4), (uint32_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Reset
 * 
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 * 
 * \param device Pointer to the FPGA9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::fpga9001_BfDpCaptureControlChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 * 
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 * 
 */
int32_t fpga9001_DpCaptureControlResetBfGet(adi_fpga9001_Device_t *device, 
                                            fpga9001_BfDpCaptureControlChanAddr_e baseAddr, 
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
    if ((baseAddr != FPGA9001_BF_RX_DP_CAPTURE_CONTROL) &&
            (baseAddr != FPGA9001_BF_TX_DP_CAPTURE_CONTROL))
    {
        ADI_ERROR_REPORT(&device->common, 
                         ADI_COMMON_ERRSRC_DEVICEBF, 
                         ADI_COMMON_ERR_INV_PARAM, 
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr, 
                         "Invalid fpga9001_DpCaptureControlResetBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* FPGA9001_BITFIELD_ADDR_CHECK */ 

    recoveryAction = fpga9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = fpga9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = fpga9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/*  EOF: fpga9001_bf_dp_capture_control.c */

