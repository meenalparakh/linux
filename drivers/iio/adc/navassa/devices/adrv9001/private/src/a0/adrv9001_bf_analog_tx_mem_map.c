#ifdef SI_REV_A0
/**
 * \file adrv9001_bf_analog_tx_mem_map.c Automatically generated file with generator ver 0.0.1.0.
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

#include "adrv9001_bf_analog_tx_mem_map.h"
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapAnalogTxMemSpareBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 255U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapAnalogTxMemSpareBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapAnalogTxMemSpareBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x71), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapAnalogTxMemSpareBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapAnalogTxMemSpareBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x71), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief write this bit high to observe the temp sensor data from the serdes IP on aux dac 0.  Active high in burn-in mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapAuxDacTempSensorObsBfSet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                         uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 15U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapAuxDacTempSensorObsBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapAuxDacTempSensorObsBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x70), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief write this bit high to observe the temp sensor data from the serdes IP on aux dac 0.  Active high in burn-in mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapAuxDacTempSensorObsBfGet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapAuxDacTempSensorObsBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x70), &rxBfData[0], 0xF, 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapAuxDacTestModeEnBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapAuxDacTestModeEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapAuxDacTestModeEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6F), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapAuxDacTestModeEnBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapAuxDacTestModeEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6F), &rxBfData[0], 0x2, 0xC1);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapCalToneDriverAttnBfSet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                       uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverAttnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverAttnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x55), (uint8_t)bfValue, 0xC, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapCalToneDriverAttnBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverAttnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x55), &rxBfData[0], 0xC, 0xC2);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapCalToneDriverInbufPdBfSet(adi_adrv9001_Device_t *device,
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                          uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverInbufPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverInbufPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x55), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapCalToneDriverInbufPdBfGet(adi_adrv9001_Device_t *device,
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverInbufPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x55), &rxBfData[0], 0x2, 0xC1);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapCalToneDriverPdBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x55), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapCalToneDriverPdBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapCalToneDriverPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x55), &rxBfData[0], 0x1, 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoBypassBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoBypassBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoBypassBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x60), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoBypassBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoBypassBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x60), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoDisThermalShutdownBfSet(adi_adrv9001_Device_t *device,
                                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                                    uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoDisThermalShutdownBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoDisThermalShutdownBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x61), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoDisThermalShutdownBfGet(adi_adrv9001_Device_t *device,
                                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoDisThermalShutdownBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x61), &rxBfData[0], 0x20, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoFilterBypassBfSet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoFilterBypassBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoFilterBypassBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x61), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoFilterBypassBfGet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoFilterBypassBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x61), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoIlimitIncBfSet(adi_adrv9001_Device_t *device,
                                                          adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                           uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoIlimitIncBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoIlimitIncBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x60), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoIlimitIncBfGet(adi_adrv9001_Device_t *device,
                                                          adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoIlimitIncBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x60), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoStatusBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoStatusBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x60), &rxBfData[0], 0x70, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoThermalShutdown0cBfSet(adi_adrv9001_Device_t *device,
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                                   uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoThermalShutdown0cBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoThermalShutdown0cBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x61), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoThermalShutdown0cBfGet(adi_adrv9001_Device_t *device,
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoThermalShutdown0cBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x61), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoVoutSelBfSet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                         int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -4) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoVoutSelBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoVoutSelBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x60), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoVoutSelBfGet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                         int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoVoutSelBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x60), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoVoutTrimBfSet(adi_adrv9001_Device_t *device,
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                          int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -16) ||
        (bfValue > 15U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoVoutTrimBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoVoutTrimBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x61), (uint8_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapConverterLdoVoutTrimBfGet(adi_adrv9001_Device_t *device,
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                          int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapConverterLdoVoutTrimBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x61), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapEnableLoSourceTxBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapEnableLoSourceTxBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapEnableLoSourceTxBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4A), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapEnableLoSourceTxBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapEnableLoSourceTxBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4A), &rxBfData[0], 0x1, 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapIdistBypassBiasResBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapIdistBypassBiasResBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapIdistBypassBiasResBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapIdistBypassBiasResBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapIdistBypassBiasResBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5), &rxBfData[0], 0x2, 0xC1);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterBfSet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 7U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4A), (uint8_t)bfValue, 0xE, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterBfGet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4A), &rxBfData[0], 0xE, 0xC1);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterAcCoupleEnBfSet(adi_adrv9001_Device_t *device,
                                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                                        uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterAcCoupleEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterAcCoupleEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4A), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterAcCoupleEnBfGet(adi_adrv9001_Device_t *device,
                                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetFilterAcCoupleEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4A), &rxBfData[0], 0x20, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetPdBfSet(adi_adrv9001_Device_t *device,
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                          uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4A), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetPdBfGet(adi_adrv9001_Device_t *device,
                                                         adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapLoOffsetLoPhaseDetPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4A), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdConverterLdoBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                    int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapPdConverterLdoBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdConverterLdoBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdConverterLdoBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                    int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdConverterLdoBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdIDistBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                             uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapPdIDistBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdIDistBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdIDistBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdIDistBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting this bit powers down the TX mixer LO DELAY BUFFER.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferBfSet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                       uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting this bit powers down the TX mixer LO DELAY BUFFER.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting this bit powers down the TX mixer LO DELAY BUFFER.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideBfSet(adi_adrv9001_Device_t *device,
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting this bit powers down the TX mixer LO DELAY BUFFER.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideBfGet(adi_adrv9001_Device_t *device,
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting this bit powers down the TX mixer LO DELAY BUFFER.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideSelectBfSet(adi_adrv9001_Device_t *device,
                                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                                     uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideSelectBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideSelectBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting this bit powers down the TX mixer LO DELAY BUFFER.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideSelectBfGet(adi_adrv9001_Device_t *device,
                                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapPdTxLoDelayBufferOverrideSelectBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits program the cap for the RC filter at the input of the transmit upconverter.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxAbbfCgBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                              uint16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1023U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxAbbfCgBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxAbbfCgBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x1F), (uint8_t)(bfValue >> 2));
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x20), (uint8_t)(bfValue >> 0), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits program the cap for the RC filter at the input of the transmit upconverter.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxAbbfCgBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxAbbfCgBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x1F), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0x1], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief These bits control the bias current for the ABBF peak detectors according to the following settings. Increasing this value increases the bias current, decreases the peak detector attack time, and increases peak detector ripple.   000 --   800nA 001 --   400nA 010 --   200nA 011 --   100nA 100 --   50nA 101 --   50nA 110 --   50nA 111 --   50nA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxAbbfPeakdetDecayBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -4) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxAbbfPeakdetDecayBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxAbbfPeakdetDecayBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1E), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits control the bias current for the ABBF peak detectors according to the following settings. Increasing this value increases the bias current, decreases the peak detector attack time, and increases peak detector ripple.   000 --   800nA 001 --   400nA 010 --   200nA 011 --   100nA 100 --   50nA 101 --   50nA 110 --   50nA 111 --   50nA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxAbbfPeakdetDecayBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxAbbfPeakdetDecayBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1E), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxAuxLoBufPdBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                  uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxAuxLoBufPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxAuxLoBufPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxAuxLoBufPdBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxAuxLoBufPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief TX BBF block poweredows - debug only
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxBbfDebugBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                uint16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 8191U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfDebugBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfDebugBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x22), (uint8_t)(bfValue >> 8), 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x21), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief TX BBF block poweredows - debug only
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxBbfDebugBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfDebugBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x22), &rxBfData[0x0], 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x21), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief TX BBF power control.  When programmed to > 0, reduces the power consumed by the filter amplifiers.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxBbfPowCtrlBBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                   uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 7U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfPowCtrlBBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfPowCtrlBBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x22), (uint8_t)bfValue, 0xE0, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief TX BBF power control.  When programmed to > 0, reduces the power consumed by the filter amplifiers.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxBbfPowCtrlBBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfPowCtrlBBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x22), &rxBfData[0], 0xE0, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxBbfSpareBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 255U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfSpareBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfSpareBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x23), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxBbfSpareBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxBbfSpareBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x23), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief When this bit is set, the Tx synthesizer is forced to power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief When this bit is set, the Tx synthesizer is forced to power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief When this bit is set, the Tx synthesizer is forced to power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdOverrideBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdOverrideBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdOverrideBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief When this bit is set, the Tx synthesizer is forced to power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdOverrideBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdOverrideBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief When this bit is set, the Tx synthesizer is forced to power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdOverrideSelectBfSet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdOverrideSelectBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdOverrideSelectBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief When this bit is set, the Tx synthesizer is forced to power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmCorrPdOverrideSelectBfGet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmCorrPdOverrideSelectBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits set the value of Tx1 LO select when in SPI Direct mode.
 *         tx1 cmlo bias
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloBiasBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloBiasBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloBiasBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x40), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits set the value of Tx1 LO select when in SPI Direct mode.
 *         tx1 cmlo bias
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloBiasBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloBiasBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x40), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits set the value of Tx1 LO select when in SPI Direct mode.
 *         tx1 cmlo correction dac i
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloDacIBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2048) ||
        (bfValue > 2047U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacIBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x3C), (uint8_t)(bfValue >> 4));
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3D), (uint8_t)(bfValue >> 0), 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits set the value of Tx1 LO select when in SPI Direct mode.
 *         tx1 cmlo correction dac i
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloDacIBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int16_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3C), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3D), &rxBfData[0x1], 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int16_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloDacLatchbBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacLatchbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacLatchbBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x40), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloDacLatchbBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacLatchbBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x40), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits set the value of Tx1 LO select when in SPI Direct mode.
 *         tx1 cmlo correction dac q
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloDacQBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2048) ||
        (bfValue > 2047U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x3E), (uint8_t)(bfValue >> 4));
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3F), (uint8_t)(bfValue >> 0), 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits set the value of Tx1 LO select when in SPI Direct mode.
 *         tx1 cmlo correction dac q
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloDacQBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int16_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloDacQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3E), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3F), &rxBfData[0x1], 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int16_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloSel2xDiodeBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloSel2xDiodeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloSel2xDiodeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x40), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloSel2xDiodeBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloSel2xDiodeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x40), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloSel2xMixerBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloSel2xMixerBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloSel2xMixerBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x40), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxCmloSel2xMixerBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxCmloSel2xMixerBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x40), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits set the Tx LO common mode voltage according to the following settings: 000 --  Vdd18 '- 0.25V 001 --  Vdd18 '- 0.25V 010 --  Vdd18 '- 0.45V 011 --  Vdd18 '- 0.55V
 *         Tx Lo Cm
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoCmBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                            int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -4) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoCmBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoCmBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3B), (uint8_t)bfValue, 0xE0, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits set the Tx LO common mode voltage according to the following settings: 000 --  Vdd18 '- 0.25V 001 --  Vdd18 '- 0.25V 010 --  Vdd18 '- 0.45V 011 --  Vdd18 '- 0.55V
 *         Tx Lo Cm
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoCmBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                            int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoCmBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3B), &rxBfData[0], 0xE0, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting this bit powers down the Tx1 LO delay correction DAC. Note that this does not disable the LO path, it only disables the I/Q LO delay tuning.
 *         Tx1 Lo Delay Corr Pd
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting this bit powers down the Tx1 LO delay correction DAC. Note that this does not disable the LO path, it only disables the I/Q LO delay tuning.
 *         Tx1 Lo Delay Corr Pd
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting this bit powers down the Tx1 LO delay correction DAC. Note that this does not disable the LO path, it only disables the I/Q LO delay tuning.
 *         Tx1 Lo Delay Corr Pd
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideBfSet(adi_adrv9001_Device_t *device,
                                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                             int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting this bit powers down the Tx1 LO delay correction DAC. Note that this does not disable the LO path, it only disables the I/Q LO delay tuning.
 *         Tx1 Lo Delay Corr Pd
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideBfGet(adi_adrv9001_Device_t *device,
                                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                             int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting this bit powers down the Tx1 LO delay correction DAC. Note that this does not disable the LO path, it only disables the I/Q LO delay tuning.
 *         Tx1 Lo Delay Corr Pd
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideSelectBfSet(adi_adrv9001_Device_t *device,
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                                   int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideSelectBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideSelectBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting this bit powers down the Tx1 LO delay correction DAC. Note that this does not disable the LO path, it only disables the I/Q LO delay tuning.
 *         Tx1 Lo Delay Corr Pd
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideSelectBfGet(adi_adrv9001_Device_t *device,
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                                   int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayCorrPdOverrideSelectBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayDacBiasResBypassBfSet(adi_adrv9001_Device_t *device,
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayDacBiasResBypassBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayDacBiasResBypassBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x11), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayDacBiasResBypassBfGet(adi_adrv9001_Device_t *device,
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayDacBiasResBypassBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x11), &rxBfData[0], 0x4, 0xC2);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayDacResBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayDacResBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayDacResBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x11), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoDelayDacResBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoDelayDacResBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x11), &rxBfData[0], 0x3, 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoBypassBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                   int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoBypassBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoBypassBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6C), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoBypassBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                   int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoBypassBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6C), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoFilterBypassBfSet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                         int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoFilterBypassBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoFilterBypassBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6D), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoFilterBypassBfGet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                         int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoFilterBypassBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6D), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoPdBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6C), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoPdBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6C), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoStatusBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoStatusBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6C), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoVoutSelBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                    int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -4) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoVoutSelBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoVoutSelBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6C), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoVoutSelBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                    int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoVoutSelBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6C), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoVoutTrimBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -16) ||
        (bfValue > 15U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoVoutTrimBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoVoutTrimBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6D), (uint8_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLdoVoutTrimBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLdoVoutTrimBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6D), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLineRTermBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                   uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineRTermBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineRTermBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLineRTermBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineRTermBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x3, 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLineRcvBufPdBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineRcvBufPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineRcvBufPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLineRcvBufPdBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineRcvBufPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x4, 0xC2);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLineTermBiasPdBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineTermBiasPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineTermBiasPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLineTermBiasPdBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineTermBiasPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLineTermEnableBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineTermEnableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineTermEnableBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxLoLineTermEnableBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxLoLineTermEnableBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdAmpBiasResCalBfSet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                       uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 255U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPdAmpBiasResCalBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdAmpBiasResCalBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x3A), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdAmpBiasResCalBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdAmpBiasResCalBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3A), &rxBfData[0], 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdDmEnBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                              uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPdDmEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdDmEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdDmEnBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdDmEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x1, 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdLowNoiseSelBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPdLowNoiseSelBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdLowNoiseSelBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x6, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdLowNoiseSelBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdLowNoiseSelBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x6, 0xC1);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdProgOtaBiasBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPdProgOtaBiasBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdProgOtaBiasBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x18, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdProgOtaBiasBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdProgOtaBiasBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x18, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdRinBoostBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                  uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPdRinBoostBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdRinBoostBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdRinBoostBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdRinBoostBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits set how many data lanes are enabled for the transmit path according to the following settings: 00 --  1 data lane from transceiver to BBP in CMOS mode, invalid in LVDS mode 01 --  2 data lanes from transceiver to BBP 10 --  4 data lanes from transceiver to BBP 11 --  Invalid configuration in both CMOS and LVDS modes
 *         Tx Pdbias
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdbiasBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                              int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPdbiasBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdbiasBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x36), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits set how many data lanes are enabled for the transmit path according to the following settings: 00 --  1 data lane from transceiver to BBP in CMOS mode, invalid in LVDS mode 01 --  2 data lanes from transceiver to BBP 10 --  4 data lanes from transceiver to BBP 11 --  Invalid configuration in both CMOS and LVDS modes
 *         Tx Pdbias
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPdbiasBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                              int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPdbiasBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x36), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief This bit selects different bias sources for Tx pre-distortion amplifier. 0 selects a local PTAT source which gives a bias current proportional to absolute temperature and is roughly 80A at room temperature. The current also varies with poly resistance. 1 selects external resistor bias which is 100A constant across temperature. This bit is for internal test only.
 *         Tx Predist Amp Bias
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x38), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This bit selects different bias sources for Tx pre-distortion amplifier. 0 selects a local PTAT source which gives a bias current proportional to absolute temperature and is roughly 80A at room temperature. The current also varies with poly resistance. 1 selects external resistor bias which is 100A constant across temperature. This bit is for internal test only.
 *         Tx Predist Amp Bias
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                      int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x38), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasManualStartupBfSet(adi_adrv9001_Device_t *device,
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                                   uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasManualStartupBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasManualStartupBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x39), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasManualStartupBfGet(adi_adrv9001_Device_t *device,
                                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasManualStartupBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x39), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasPtatStartBfSet(adi_adrv9001_Device_t *device,
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                               uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 7U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasPtatStartBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasPtatStartBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x39), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpBiasPtatStartBfGet(adi_adrv9001_Device_t *device,
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpBiasPtatStartBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x39), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief This bit selects different bias sources for Tx pre-distortion amplifier. 0 selects a local PTAT source which gives a bias current proportional to absolute temperature and is roughly 80A at room temperature. The current also varies with poly resistance. 1 selects external resistor bias which is 100A constant across temperature. This bit is for internal test only.
 *         Tx Predist Amp Cc
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpCcBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                    int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -4) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpCcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpCcBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x38), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This bit selects different bias sources for Tx pre-distortion amplifier. 0 selects a local PTAT source which gives a bias current proportional to absolute temperature and is roughly 80A at room temperature. The current also varies with poly resistance. 1 selects external resistor bias which is 100A constant across temperature. This bit is for internal test only.
 *         Tx Predist Amp Cc
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpCcBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                    int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpCcBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x38), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting these bits bypasses two huge filtering resistors in the bias circuits of the Tx pre-distortion amplifier. The bits are for internal test only.
 *         Tx Predist Amp Filter Bypass
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpFilterBypassBfSet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpFilterBypassBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpFilterBypassBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x38), (uint8_t)bfValue, 0x60, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting these bits bypasses two huge filtering resistors in the bias circuits of the Tx pre-distortion amplifier. The bits are for internal test only.
 *         Tx Predist Amp Filter Bypass
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpFilterBypassBfGet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpFilterBypassBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x38), &rxBfData[0], 0x60, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief This bit selects different bias sources for Tx pre-distortion amplifier. 0 selects a local PTAT source which gives a bias current proportional to absolute temperature and is roughly 80A at room temperature. The current also varies with poly resistance. 1 selects external resistor bias which is 100A constant across temperature. This bit is for internal test only.
 *         Tx Predist Amp Rz
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpRzBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                    int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpRzBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpRzBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x38), (uint8_t)bfValue, 0x18, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This bit selects different bias sources for Tx pre-distortion amplifier. 0 selects a local PTAT source which gives a bias current proportional to absolute temperature and is roughly 80A at room temperature. The current also varies with poly resistance. 1 selects external resistor bias which is 100A constant across temperature. This bit is for internal test only.
 *         Tx Predist Amp Rz
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistAmpRzBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                    int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistAmpRzBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x38), &rxBfData[0], 0x18, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief When this bit is set, the Tx synthesizer is forced to power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistPdBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                 int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief When this bit is set, the Tx synthesizer is forced to power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistPdBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                 int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits set how many data lanes are enabled for the transmit path according to the following settings: 00 --  1 data lane from transceiver to BBP in CMOS mode, invalid in LVDS mode 01 --  2 data lanes from transceiver to BBP 10 --  4 data lanes from transceiver to BBP 11 --  Invalid configuration in both CMOS and LVDS modes
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistPdoverrideBfSet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                         int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdoverrideBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdoverrideBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits set how many data lanes are enabled for the transmit path according to the following settings: 00 --  1 data lane from transceiver to BBP in CMOS mode, invalid in LVDS mode 01 --  2 data lanes from transceiver to BBP 10 --  4 data lanes from transceiver to BBP 11 --  Invalid configuration in both CMOS and LVDS modes
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistPdoverrideBfGet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                         int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdoverrideBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistPdoverrideSelectBfSet(adi_adrv9001_Device_t *device,
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                               int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdoverrideSelectBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdoverrideSelectBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxPredistPdoverrideSelectBfGet(adi_adrv9001_Device_t *device,
                                                              adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                               int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxPredistPdoverrideSelectBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting these bits enable the power down override for the Tx upconverters. Setting [D1] enables the power down override for the Tx2 upconverter in 0x20A[D1]. Setting [D0] enables the power down override for the Tx1 upconverter in 0x20A[D0].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxUpconvPdBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting these bits enable the power down override for the Tx upconverters. Setting [D1] enables the power down override for the Tx2 upconverter in 0x20A[D1]. Setting [D0] enables the power down override for the Tx1 upconverter in 0x20A[D0].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxUpconvPdBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting these bits enable the power down override for the Tx upconverters. Setting [D1] enables the power down override for the Tx2 upconverter in 0x20A[D1]. Setting [D0] enables the power down override for the Tx1 upconverter in 0x20A[D0].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxUpconvPdoverrideBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdoverrideBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdoverrideBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting these bits enable the power down override for the Tx upconverters. Setting [D1] enables the power down override for the Tx2 upconverter in 0x20A[D1]. Setting [D0] enables the power down override for the Tx1 upconverter in 0x20A[D0].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxUpconvPdoverrideBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                        int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdoverrideBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting these bits enable the power down override for the Tx upconverters. Setting [D1] enables the power down override for the Tx2 upconverter in 0x20A[D1]. Setting [D0] enables the power down override for the Tx1 upconverter in 0x20A[D0].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxUpconvPdoverrideSelectBfSet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdoverrideSelectBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdoverrideSelectBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting these bits enable the power down override for the Tx upconverters. Setting [D1] enables the power down override for the Tx2 upconverter in 0x20A[D1]. Setting [D0] enables the power down override for the Tx1 upconverter in 0x20A[D0].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxUpconvPdoverrideSelectBfGet(adi_adrv9001_Device_t *device,
                                                             adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                              int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxUpconvPdoverrideSelectBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxVcmCntrlBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 3U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxVcmCntrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxVcmCntrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0xC0, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxVcmCntrlBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxVcmCntrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0xC0, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits are the compensation capacitor control word for the input opamp in the BBF biquad (opamp A). Increasing this word increases the compensation capacitor setting, which increases stability at the expense of wideband linearity performance. See the user guide for more details.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxabbfCcCntABfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                  int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxabbfCcCntABfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxabbfCcCntABfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1C), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits are the compensation capacitor control word for the input opamp in the BBF biquad (opamp A). Increasing this word increases the compensation capacitor setting, which increases stability at the expense of wideband linearity performance. See the user guide for more details.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxabbfCcCntABfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                  int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxabbfCcCntABfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits are the compensation capacitor control word for the input opamp in the BBF biquad (opamp A). Increasing this word increases the compensation capacitor setting, which increases stability at the expense of wideband linearity performance. See the user guide for more details.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxabbfCcCntBBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                  int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxabbfCcCntBBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxabbfCcCntBBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1D), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits are the compensation capacitor control word for the input opamp in the BBF biquad (opamp A). Increasing this word increases the compensation capacitor setting, which increases stability at the expense of wideband linearity performance. See the user guide for more details.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxabbfCcCntBBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                  int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxabbfCcCntBBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1D), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting these bits enable the power down override for the Tx baseband filters. Setting [D5] enables the power down override for the Tx2 BBF in 0x209[D5]. Setting [D4] enables the power down override for the Tx1 BBF in 0x209[D4].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxbbfPdBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                             int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting these bits enable the power down override for the Tx baseband filters. Setting [D5] enables the power down override for the Tx2 BBF in 0x209[D5]. Setting [D4] enables the power down override for the Tx1 BBF in 0x209[D4].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxbbfPdBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                             int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting these bits enable the power down override for the Tx baseband filters. Setting [D5] enables the power down override for the Tx2 BBF in 0x209[D5]. Setting [D4] enables the power down override for the Tx1 BBF in 0x209[D4].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxbbfPdoverrideBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdoverrideBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdoverrideBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting these bits enable the power down override for the Tx baseband filters. Setting [D5] enables the power down override for the Tx2 BBF in 0x209[D5]. Setting [D4] enables the power down override for the Tx1 BBF in 0x209[D4].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxbbfPdoverrideBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                     int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdoverrideBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief Setting these bits enable the power down override for the Tx baseband filters. Setting [D5] enables the power down override for the Tx2 BBF in 0x209[D5]. Setting [D4] enables the power down override for the Tx1 BBF in 0x209[D4].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxbbfPdoverrideSelectBfSet(adi_adrv9001_Device_t *device,
                                                          adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                           int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdoverrideSelectBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdoverrideSelectBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Setting these bits enable the power down override for the Tx baseband filters. Setting [D5] enables the power down override for the Tx2 BBF in 0x209[D5]. Setting [D4] enables the power down override for the Tx1 BBF in 0x209[D4].
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxbbfPdoverrideSelectBfGet(adi_adrv9001_Device_t *device,
                                                          adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                           int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxbbfPdoverrideSelectBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief This bit sets the Tx output supply voltage being used. Clear this bit for 1.8V, set it for 1.3V
 *         Txuc Lv
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxucLvBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                            int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -1) ||
        (bfValue > 0U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxucLvBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxucLvBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x36), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief This bit sets the Tx output supply voltage being used. Clear this bit for 1.8V, set it for 1.3V
 *         Txuc Lv
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxucLvBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                            int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxucLvBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x36), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxucRfcascLvlBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                   int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxucRfcascLvlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxucRfcascLvlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x36), (uint8_t)bfValue, 0x18, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

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
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxucRfcascLvlBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                   int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxucRfcascLvlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x36), &rxBfData[0], 0x18, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits set the cascode off voltage according to the following equation:
 *         Txuc Vcascctrl
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxucVcascctrlBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                   int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -8) ||
        (bfValue > 7U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxucVcascctrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxucVcascctrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x37), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits set the cascode off voltage according to the following equation:
 *         Txuc Vcascctrl
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxucVcascctrlBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                   int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxucVcascctrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x37), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}
/**
 * \brief These bits set the cascode off voltage according to the following equation:
 *         Txuc Vcascoff
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxucVcascoffBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                  int8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < -2) ||
        (bfValue > 1U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_AnalogTxMemMapTxucVcascoffBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxucVcascoffBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x37), (uint8_t)bfValue, 0x30, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief These bits set the cascode off voltage according to the following equation:
 *         Txuc Vcascoff
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAnalogTxMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type int8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_AnalogTxMemMapTxucVcascoffBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAnalogTxMemMapChanAddr_e baseAddr,
                                                  int8_t *bfValue)
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
    if ((baseAddr != ADRV9001_BF_TX1_ANA) &&
            (baseAddr != ADRV9001_BF_TX2_ANA))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_AnalogTxMemMapTxucVcascoffBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x37), &rxBfData[0], 0x30, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (int8_t) data;

    return recoveryAction;
}

/*  EOF: adrv9001_bf_analog_tx_mem_map.c */

#endif