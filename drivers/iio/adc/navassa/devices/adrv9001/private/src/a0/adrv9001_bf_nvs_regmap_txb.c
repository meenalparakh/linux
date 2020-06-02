#ifdef SI_REV_A0
/**
 * \file adrv9001_bf_nvs_regmap_txb.c Automatically generated file with generator ver 0.0.1.0.
 *
 * \brief Contains BitField functions to support ADRV9001 transciever device.
 *
 * ADRV9001 BITFIELD VERSION: 0.0.0.1
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adrv9001_bf_nvs_regmap_txb.h"
#include "adrv9001_bf_hal.h"
#include "adi_common_error.h"


/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcCnt0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                       uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcCnt0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1F), &rxBfData[0x0], 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x1E), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x1D), &rxBfData[0x2], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x3, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint64_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcImagAcc0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                           uint64_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcImagAcc0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x1B), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x1A), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x19), &rxBfData[0x3], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x18), &rxBfData[0x4], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x17), &rxBfData[0x5], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x16), &rxBfData[0x6], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x15), &rxBfData[0x7], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x8, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint64_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcOutI0BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                               uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcOutI0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2F), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2E), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2D), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2C), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcOutI1BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                               uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcOutI1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x33), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x32), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x31), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x30), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcOutI2BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                               uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcOutI2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x37), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x36), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x35), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x34), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcOutI3BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                               uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcOutI3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3B), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3A), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x39), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x38), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcOutQ0BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                               uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcOutQ0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3F), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3E), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3D), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3C), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcOutQ1BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                               uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcOutQ1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x43), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x42), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x41), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x40), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcOutQ2BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                               uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcOutQ2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x47), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x46), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x45), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x44), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcOutQ3BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                               uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcOutQ3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x4B), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x4A), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x49), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x48), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint64_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcCcRealAcc0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                          adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                           uint64_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcCcRealAcc0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x13), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x12), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x11), &rxBfData[0x3], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x10), &rxBfData[0x4], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xF), &rxBfData[0x5], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xE), &rxBfData[0x6], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xD), &rxBfData[0x7], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x8, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint64_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint64_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmAcc0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                       uint64_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmAcc0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x9), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x8), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x7), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x6), &rxBfData[0x3], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x5), &rxBfData[0x4], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x4), &rxBfData[0x5], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3), &rxBfData[0x6], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2), &rxBfData[0x7], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x8, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint64_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmCnt0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                       uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmCnt0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0x0], 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xB), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xA), &rxBfData[0x2], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x3, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmPowExp0BfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                 uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmPowExp0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x22), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmPowExp1BfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                 uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmPowExp1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x25), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmPowExp2BfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                 uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmPowExp2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x28), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmPowExp3BfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                 uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmPowExp3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2B), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmPowMan0BfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                 uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmPowMan0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x21), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x20), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmPowMan1BfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                 uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmPowMan1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x24), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x23), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmPowMan2BfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                 uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmPowMan2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x27), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x26), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmPowMan3BfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                 uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmPowMan3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2A), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x29), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbClgcPmStatusOutBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                   uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbClgcPmStatusOutBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0x0], 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x0), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcCnt0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                        uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcCnt0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6B), &rxBfData[0x0], 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x6A), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x69), &rxBfData[0x2], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x3, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint64_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcImagAcc0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                            uint64_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcImagAcc0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x68), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x67), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x66), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x65), &rxBfData[0x3], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x64), &rxBfData[0x4], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x63), &rxBfData[0x5], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x62), &rxBfData[0x6], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x61), &rxBfData[0x7], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x8, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint64_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcOutI0BfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcOutI0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x7B), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x7A), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x79), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x78), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcOutI1BfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcOutI1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x7F), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x7E), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x7D), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x7C), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcOutI2BfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcOutI2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x83), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x82), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x81), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x80), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcOutI3BfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcOutI3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x87), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x86), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x85), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x84), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcOutQ0BfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcOutQ0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x8B), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x8A), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x89), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x88), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcOutQ1BfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcOutQ1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x8F), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x8E), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x8D), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x8C), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcOutQ2BfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcOutQ2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x93), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x92), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x91), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x90), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcOutQ3BfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcOutQ3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x97), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x96), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x95), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x94), &rxBfData[0x3], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x4, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint64_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaCcRealAcc0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                           adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                            uint64_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaCcRealAcc0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x60), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x5F), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x5E), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x5D), &rxBfData[0x3], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x5C), &rxBfData[0x4], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x5B), &rxBfData[0x5], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x5A), &rxBfData[0x6], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x59), &rxBfData[0x7], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x8, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint64_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint64_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmAcc0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                        uint64_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmAcc0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x55), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x54), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x53), &rxBfData[0x2], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x52), &rxBfData[0x3], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x51), &rxBfData[0x4], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x50), &rxBfData[0x5], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x4F), &rxBfData[0x6], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x4E), &rxBfData[0x7], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x8, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint64_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmCnt0IntermValBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                        uint32_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmCnt0IntermValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x58), &rxBfData[0x0], 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x57), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x56), &rxBfData[0x2], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x3, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmPowExp0BfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                  uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmPowExp0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6E), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmPowExp1BfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                  uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmPowExp1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x71), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmPowExp2BfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                  uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmPowExp2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x74), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmPowExp3BfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                  uint8_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmPowExp3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x77), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmPowMan0BfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                  uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmPowMan0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x6D), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x6C), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmPowMan1BfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                  uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmPowMan1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x70), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x6F), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmPowMan2BfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                  uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmPowMan2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x73), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x72), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmPowMan3BfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                  uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmPowMan3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x76), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x75), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfNvsRegmapTxbChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_NvsRegmapTxbGanPaPmStatusOutBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfNvsRegmapTxbChanAddr_e baseAddr,
                                                    uint16_t *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TXB1_CORE) &&
            (baseAddr != ADRV9001_BF_TXB2_CORE))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_NvsRegmapTxbGanPaPmStatusOutBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4D), &rxBfData[0x0], 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x4C), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/*  EOF: adrv9001_bf_nvs_regmap_txb.c */

#endif