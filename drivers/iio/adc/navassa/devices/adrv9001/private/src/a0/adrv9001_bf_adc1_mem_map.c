#ifdef SI_REV_A0
/**
 * \file adrv9001_bf_adc1_mem_map.c Automatically generated file with generator ver 0.0.1.0.
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

#include "adrv9001_bf_adc1_mem_map.h"
#include "adrv9001_bf_hal.h"
#include "adi_common_error.h"

/**
 * \brief Compensation capacitor on ADC Amplifier 1 4th/5th-order path, from 160f to 1280f by 160f
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp1Cap1BfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
                                            uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 63U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap1BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x16 + channelId*64), (uint8_t)bfValue, 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Compensation capacitor on ADC Amplifier 1 4th/5th-order path, from 160f to 1280f by 160f
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp1Cap1BfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap1BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x16 + channelId*64), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Compensation capacitor on ADC Amplifier 1 5th-order path, from 160f to 1280f by 160f
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp1Cap2BfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
                                            uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 63U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap2BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x14 + channelId*64), (uint8_t)bfValue, 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Compensation capacitor on ADC Amplifier 1 5th-order path, from 160f to 1280f by 160f
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp1Cap2BfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1Cap2BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14 + channelId*64), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Enable ADC Amplifier 1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp1EnBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcAmp1EnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1EnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x2D), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Enable ADC Amplifier 1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp1EnBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1EnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2D), &rxBfData[0], 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp1SpareBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                             uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcAmp1SpareBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1SpareBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1SpareBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x15 + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
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
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp1SpareBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                             uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1SpareBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp1SpareBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x15 + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Compensation capacitor on ADC Amplifier 2 4th/5th-order path, from 160f to 1280f by 160f
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp2Cap1BfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
                                            uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 63U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap1BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x17 + channelId*64), (uint8_t)bfValue, 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Compensation capacitor on ADC Amplifier 2 4th/5th-order path, from 160f to 1280f by 160f
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp2Cap1BfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap1BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x17 + channelId*64), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Compensation capacitor on ADC Amplifier 2 5th-order path, from 160f to 1280f by 160f
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp2Cap2BfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
                                            uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 63U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap2BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x18 + channelId*64), (uint8_t)bfValue, 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Compensation capacitor on ADC Amplifier 2 5th-order path, from 160f to 1280f by 160f
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp2Cap2BfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2Cap2BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x18 + channelId*64), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Enable ADC Amplifier 2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp2EnBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcAmp2EnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2EnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x2E), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Enable ADC Amplifier 2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp2EnBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2EnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2E), &rxBfData[0], 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp2SpareBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                             uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcAmp2SpareBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2SpareBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2SpareBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x19 + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
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
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp2SpareBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                             uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2SpareBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp2SpareBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x19 + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Enable ADC Amplifier 3
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp3EnBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcAmp3EnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp3EnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x2F), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Enable ADC Amplifier 3
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp3EnBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp3EnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2F), &rxBfData[0], 0xC0);
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
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp3SpareBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                             uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcAmp3SpareBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp3SpareBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp3SpareBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1B + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
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
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcAmp3SpareBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                             uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcAmp3SpareBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcAmp3SpareBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1B + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 1, input GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1F + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 1, input GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1bBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1F + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 1, input GM stage on 1.8V, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x20 + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 1, input GM stage on 1.8V, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1dBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x20 + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 1, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1E + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 1, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1aBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1E + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 1, high speed GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                   uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1E + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 1, high speed GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                   uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm1a1p8BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1E + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 1, high gain GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1F + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 1, high gain GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm2aBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1F + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 1, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x20 + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 1, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp1Gm4aBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x20 + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 2, input GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x22 + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 2, input GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1bBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x22 + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 2, input GM stage on 1.8V, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x23 + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 2, input GM stage on 1.8V, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1dBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x23 + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 2, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x21 + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 2, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1aBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x21 + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 2, high speed GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                   uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x21 + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 2, high speed GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                   uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm1a1p8BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x21 + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 2, high gain GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x22 + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 2, high gain GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm2aBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x22 + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 2, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x23 + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 2, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp2Gm4aBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x23 + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 3, input GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x25 + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 3, input GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1bBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x25 + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 3, input GM stage on 1.8V, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x26 + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 3, input GM stage on 1.8V, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1dBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x26 + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 3, high speed GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                   uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x24 + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 3, high speed GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                   uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm1a1p8BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x24 + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for ADC Amplifier 3, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x26 + channelId*64), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for ADC Amplifier 3, midband GM stage, from 5uA to 42.5uA
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasAmp3Gm4aBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x26 + channelId*64), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief No longer used
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x39), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief No longer used
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x39), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for op-amp controlling DAC N-current sink in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpNcurrBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpNcurrBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpNcurrBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x34), (uint8_t)bfValue, 0x30, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for op-amp controlling DAC N-current sink in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpNcurrBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpNcurrBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x34), &rxBfData[0], 0x30, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for op-amp controlling DAC P-current source in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpPcurrBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpPcurrBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpPcurrBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x34), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for op-amp controlling DAC P-current source in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacAmpPcurrBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacAmpPcurrBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x34), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current controlling DAC cascode transistor voltage in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacBiasPplyBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacBiasPplyBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacBiasPplyBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x35), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current controlling DAC cascode transistor voltage in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacBiasPplyBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacBiasPplyBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x35), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias control to set DAC calibration current
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacIcalPplyBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacIcalPplyBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacIcalPplyBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x35), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias control to set DAC calibration current
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacIcalPplyBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacIcalPplyBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x35), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias control to set DAC LSB current in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacIlsbBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacIlsbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacIlsbBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x36), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias control to set DAC LSB current in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasDacIlsbBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasDacIlsbBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x36), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for FLASH in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasFlashBiasBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                 uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasFlashBiasBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasFlashBiasBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasFlashBiasBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1C + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for FLASH in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasFlashBiasBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                 uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasFlashBiasBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasFlashBiasBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for FLASH reference voltage generator in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasFlashRefgenBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasFlashRefgenBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasFlashRefgenBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x33), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for FLASH reference voltage generator in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasFlashRefgenBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasFlashRefgenBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x33), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable 1k reference resistor in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasRcalBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasRcalBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasRcalBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasRcalBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x1D + channelId*64), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable 1k reference resistor in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasRcalBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                            uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasRcalBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcBiasRcalBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x1D + channelId*64), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for op-amp controlling the common-mode voltage in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasVcmAmpBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasVcmAmpBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasVcmAmpBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x38), (uint8_t)bfValue, 0x30, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for op-amp controlling the common-mode voltage in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasVcmAmpBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasVcmAmpBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x38), &rxBfData[0], 0x30, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current to set the on-voltage of switch transistors of programmable R&C in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasVonBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasVonBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasVonBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x37), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current to set the on-voltage of switch transistors of programmable R&C in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasVonBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasVonBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x37), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Bias current for op-amp controlling VON voltage generation in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasVonAmpBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcBiasVonAmpBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasVonAmpBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x37), (uint8_t)bfValue, 0x18, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bias current for op-amp controlling VON voltage generation in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcBiasVonAmpBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcBiasVonAmpBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x37), &rxBfData[0], 0x18, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable capacitor in ADC 1st stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcC1BfSet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
                                      uint16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 2047U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcC1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcC1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcC1BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xA + channelId*64), (uint8_t)(bfValue >> 8), 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x9 + channelId*64), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable capacitor in ADC 1st stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcC1BfGet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcC1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcC1BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xA + channelId*64), &rxBfData[0x0], 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x9 + channelId*64), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable capacitor in ADC 2nd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcC2BfSet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcC2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcC2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcC2BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC + channelId*64), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0xB + channelId*64), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable capacitor in ADC 2nd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcC2BfGet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcC2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcC2BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC + channelId*64), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xB + channelId*64), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable capacitor in ADC 3rd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcC3BfSet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
                                      uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 127U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcC3BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcC3BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcC3BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xD + channelId*64), (uint8_t)bfValue, 0x7F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable capacitor in ADC 3rd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcC3BfGet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcC3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcC3BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xD + channelId*64), &rxBfData[0], 0x7F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable capacitor for capacitively-coupled feedback in ADC 3rd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcCdfbBfSet(adi_adrv9001_Device_t *device,
                                        adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                        uint8_t channelId,
                                        uint16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4095U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcCdfbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcCdfbBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcCdfbBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xF + channelId*64), (uint8_t)(bfValue >> 8), 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x10 + channelId*64), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable capacitor for capacitively-coupled feedback in ADC 3rd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcCdfbBfGet(adi_adrv9001_Device_t *device,
                                        adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                        uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcCdfbBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcCdfbBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xF + channelId*64), &rxBfData[0x0], 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x10 + channelId*64), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief Bypass DAC filter resistors in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcDacsBypassRfBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcDacsBypassRfBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcDacsBypassRfBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3A), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Bypass DAC filter resistors in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcDacsBypassRfBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcDacsBypassRfBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3A), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Flash Cal Reference Voltage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashCalBiasRefCtrlBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcFlashCalBiasRefCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashCalBiasRefCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2C), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Flash Cal Reference Voltage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashCalBiasRefCtrlBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashCalBiasRefCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2C), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Flash Reference Shuffler Control. [2:0] control mode, [7:3] is the override state
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashEslCtrlBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcFlashEslCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashEslCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcFlashEslCtrlBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x12 + channelId*64), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Flash Reference Shuffler Control. [2:0] control mode, [7:3] is the override state
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashEslCtrlBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashEslCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcFlashEslCtrlBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x12 + channelId*64), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Flash-DAC Clock Delay Mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashFckDelayModeBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcFlashFckDelayModeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashFckDelayModeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2C), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Flash-DAC Clock Delay Mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashFckDelayModeBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashFckDelayModeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2C), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Sets a comparator's output to 0.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashRBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                           uint16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 65535U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcFlashRBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashRBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x29), (uint8_t)(bfValue >> 8));
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x28), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Sets a comparator's output to 0.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashRBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashRBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x29), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x28), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief Controls the flash LSB.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashRefCtrlBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcFlashRefCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashRefCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcFlashRefCtrlBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x13 + channelId*64), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Controls the flash LSB.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashRefCtrlBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                                uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashRefCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcFlashRefCtrlBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x13 + channelId*64), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Determines which comparators are set to 1 and which are set to 0. This is to balacing out aging effects from reset.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashResetModeBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcFlashResetModeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashResetModeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2C), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Determines which comparators are set to 1 and which are set to 0. This is to balacing out aging effects from reset.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashResetModeBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashResetModeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2C), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Set a comparator's output to 1. Superceded by adc_flash_r/
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashSBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                           uint16_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 65535U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcFlashSBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashSBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x2B), (uint8_t)(bfValue >> 8));
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x2A), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set a comparator's output to 1. Superceded by adc_flash_r/
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcFlashSBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcFlashSBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2B), &rxBfData[0x0], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2A), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief ILSB current setting in ADC 3rd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcI3BfSet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
                                      uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 63U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcI3BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcI3BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcI3BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xE + channelId*64), (uint8_t)bfValue, 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief ILSB current setting in ADC 3rd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcI3BfGet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcI3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcI3BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xE + channelId*64), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief ADC input resistor selection for gain step
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcInputAltBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcInputAltBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcInputAltBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3B), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief ADC input resistor selection for gain step
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcInputAltBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcInputAltBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3B), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Open-circuits ADC input
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcInputzeroBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcInputzeroBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcInputzeroBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3B), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Open-circuits ADC input
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcInputzeroBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcInputzeroBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3B), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Powerdown of master bias circuit in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdBiasBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcPdBiasBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdBiasBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3A), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Powerdown of master bias circuit in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdBiasBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdBiasBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3A), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DAC powerdown in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdDacsBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcPdDacsBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdDacsBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3A), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DAC powerdown in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdDacsBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdDacsBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3A), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DAC delayed powerdown in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdDacsDelayBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcPdDacsDelayBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdDacsDelayBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3A), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DAC delayed powerdown in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdDacsDelayBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdDacsDelayBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3A), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief FLASH powerdown in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdFlashBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcPdFlashBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdFlashBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3A), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief FLASH powerdown in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdFlashBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdFlashBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3A), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Loopfilter powerdown in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdLfBfSet(adi_adrv9001_Device_t *device,
                                        adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcPdLfBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdLfBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3A), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Loopfilter powerdown in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcPdLfBfGet(adi_adrv9001_Device_t *device,
                                        adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcPdLfBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3A), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable input resistor in ADC 1st stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR1BfSet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcR1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR1BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1 + channelId*64), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x0 + channelId*64), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable input resistor in ADC 1st stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR1BfGet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR1BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1 + channelId*64), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x0 + channelId*64), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable alternate setting of input resistor in ADC 1st stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR1AltBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                         uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcR1AltBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR1AltBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR1AltBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3 + channelId*64), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x2 + channelId*64), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable alternate setting of input resistor in ADC 1st stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR1AltBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                         uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR1AltBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR1AltBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3 + channelId*64), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x2 + channelId*64), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable resistor in ADC 2nd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR2BfSet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcR2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR2BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5 + channelId*64), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x4 + channelId*64), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable resistor in ADC 2nd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR2BfGet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR2BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5 + channelId*64), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x4 + channelId*64), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable resistor in ADC 3rd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR3BfSet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcR3BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR3BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR3BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x7 + channelId*64), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x6 + channelId*64), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable resistor in ADC 3rd stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR3BfGet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR3BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7 + channelId*64), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x6 + channelId*64), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable feedback resistor in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR4BfSet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
                                      uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 127U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_Adc1MemMapAdcR4BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR4BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR4BfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x8 + channelId*64), (uint8_t)bfValue, 0x7F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable feedback resistor in ADC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcR4BfGet(adi_adrv9001_Device_t *device,
                                      adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                      uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcR4BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcR4BfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x8 + channelId*64), &rxBfData[0], 0x7F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Programmable resistor that sinks ADC input common-mode current
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcRcmBfSet(adi_adrv9001_Device_t *device,
                                       adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                       uint8_t channelId,
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
                         "Invalid adrv9001_Adc1MemMapAdcRcmBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcRcmBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcRcmBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x11 + channelId*64), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Programmable resistor that sinks ADC input common-mode current
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param channelId Channel ID to be configured.
 *  Parameter is of type uint8_t
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcRcmBfGet(adi_adrv9001_Device_t *device,
                                       adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
                                       uint8_t channelId,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcRcmBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);


#if ADRV9001_CHANNELID_CHECK > 0
    /* Channel ID check */
    if (
            (channelId != 0x0) &&
            (channelId != 0x1) &&
            (channelId != 0x2) &&
            (channelId != 0x3)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_Adc1MemMapAdcRcmBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x11 + channelId*64), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Reset for integrator in ADC 1st stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcResetInt1BfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcResetInt1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcResetInt1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2C), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Reset for integrator in ADC 1st stage
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcResetInt1BfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcResetInt1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2C), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief ADC digital reset
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcRstDigBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcRstDigBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcRstDigBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3A), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief ADC digital reset
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcRstDigBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcRstDigBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3A), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcSpareBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_Adc1MemMapAdcSpareBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcSpareBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x3C), ((uint8_t)bfValue));
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
 *  Parameter is of type ::adrv9001_BfAdc1MemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_Adc1MemMapAdcSpareBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfAdc1MemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC) &&
            (baseAddr != ADRV9001_BF_RX2_ADC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_Adc1MemMapAdcSpareBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x3C), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/*  EOF: adrv9001_bf_adc1_mem_map.c */

#endif