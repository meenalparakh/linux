#ifdef SI_REV_B0
/**
 * \file adrv9001_bf_vco_adc_mem_map.c Automatically generated file with generator ver 0.0.1.0.
 *
 * \brief Contains BitField functions to support ADRV9001 transciever device.
 *
 * ADRV9001 BITFIELD VERSION: 0.0.0.1
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adrv9001_bf_vco_adc_mem_map.h"
#include "adrv9001_bf_hal.h"
#include "adi_common_error.h"


/**
 * \brief acc0 data from the analog. changes every clock cycle.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapAcc0DataBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapAcc0DataBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x131), &rxBfData[0x0], 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x12F), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}

/**
 * \brief acc1 data from analog. changes every clock cycle.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapAcc1DataBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapAcc1DataBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x131), &rxBfData[0x0], 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x130), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief cal center
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcalcenterBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcalcenterBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcalcenterBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x14), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x13), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief cal center
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcalcenterBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcalcenterBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x13), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief bcap adjustment
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcapBfSet(adi_adrv9001_Device_t *device,
                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcapBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcapBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3), (uint8_t)bfValue, 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief bcap adjustment
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcapBfGet(adi_adrv9001_Device_t *device,
                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcapBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief samples to accumulate for bcenter cal --0x0 corresponds to 16*8 samples, 0x1 is 64*8, 0x2 is 128*8, 0x3 is 256*8, 0x4 is 512*8
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterAccCntValBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcenterAccCntValBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterAccCntValBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x122), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief samples to accumulate for bcenter cal --0x0 corresponds to 16*8 samples, 0x1 is 64*8, 0x2 is 128*8, 0x3 is 256*8, 0x4 is 512*8
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterAccCntValBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterAccCntValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x122), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief setting this to 1 decreases the mu for bcenter adaptation. this should never need to be used. if used, value of 1 corresponds to 2^-13
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterAdaptDecreaseBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptDecreaseBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptDecreaseBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x122), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief setting this to 1 decreases the mu for bcenter adaptation. this should never need to be used. if used, value of 1 corresponds to 2^-13
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterAdaptDecreaseBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptDecreaseBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x122), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief setting this to 1 increases mu by a factor of 8. we should never need to use this.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterAdaptIncreaseBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptIncreaseBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptIncreaseBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x122), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief setting this to 1 increases mu by a factor of 8. we should never need to use this.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterAdaptIncreaseBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptIncreaseBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x122), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief setting this to 1 changes number of bcenter steps from 32 to 16
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterAdaptStepsBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptStepsBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptStepsBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x122), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief setting this to 1 changes number of bcenter steps from 32 to 16
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterAdaptStepsBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterAdaptStepsBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x122), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable bcenter cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterCalEnBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcenterCalEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterCalEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x123), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief enable bcenter cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterCalEnBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterCalEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x123), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief initial value for bcenter (out of reset initial bcenter value is 10'h0 -- this control allows us to load in different bcenter initial value once we are out of reset)
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterInitValBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcenterInitValBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterInitValBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x121), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x120), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief initial value for bcenter (out of reset initial bcenter value is 10'h0 -- this control allows us to load in different bcenter initial value once we are out of reset)
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterInitValBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterInitValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x121), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x120), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief setting this to anything other than 0 scales adaptation coefficient. This only takes effect when bcenter_adapt_decrease is set to 1.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterScaleShiftBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcenterScaleShiftBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterScaleShiftBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x121), (uint8_t)bfValue, 0xE0, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief setting this to anything other than 0 scales adaptation coefficient. This only takes effect when bcenter_adapt_decrease is set to 1.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterScaleShiftBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterScaleShiftBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x121), &rxBfData[0], 0xE0, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief setting this to "1" loads bcenter_start_init_val as initial bcenter value prior to bcenter cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterStartInitValBfSet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBcenterStartInitValBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterStartInitValBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x122), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief setting this to "1" loads bcenter_start_init_val as initial bcenter value prior to bcenter cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterStartInitValBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterStartInitValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x122), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief bcenter value
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBcenterValueBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBcenterValueBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x133), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x132), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief bdither DAC1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBdithdac1BfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBdithdac1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBdithdac1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x8), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief bdither DAC1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBdithdac1BfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBdithdac1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x8), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief bdither DAC2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBdithdac2BfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBdithdac2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBdithdac2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x9), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief bdither DAC2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBdithdac2BfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBdithdac2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x9), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief bgain adjustment
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBgainBfSet(adi_adrv9001_Device_t *device,
                                        adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
                                         uint8_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 31U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_VcoAdcMemMapBgainBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBgainBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief bgain adjustment
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBgainBfGet(adi_adrv9001_Device_t *device,
                                        adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBgainBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief disable vcoadc bias low-pass filter.  Used for faster startup from power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBiasDisLpfBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBiasDisLpfBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBiasDisLpfBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x30, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief disable vcoadc bias low-pass filter.  Used for faster startup from power down.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBiasDisLpfBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBiasDisLpfBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x30, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief vti amp current 1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtiamp1BfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBvtiamp1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtiamp1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0xE), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief vti amp current 1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtiamp1BfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtiamp1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xE), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief vti amp current 2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtiamp2BfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBvtiamp2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtiamp2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0xF), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief vti amp current 2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtiamp2BfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtiamp2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xF), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief vti input common mode level
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtiampcmBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBvtiampcmBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtiampcmBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0xD), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief vti input common mode level
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtiampcmBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtiampcmBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xD), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief vti amp mirror current
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtiampsBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBvtiampsBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtiampsBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x10), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief vti amp mirror current
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtiampsBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtiampsBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x10), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief bvti center
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvticenterBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBvticenterBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvticenterBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0xA), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief bvti center
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvticenterBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvticenterBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0xA), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief vti shift current
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtishiftBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapBvtishiftBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtishiftBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12), (uint8_t)(bfValue >> 8), 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x11), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief vti shift current
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapBvtishiftBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapBvtishiftBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12), &rxBfData[0x0], 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x11), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief vco adc is calibrated every 2^(cal_duty_cycle+14+8) clock cycles
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalDutyCycleBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapCalDutyCycleBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalDutyCycleBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x121), (uint8_t)bfValue, 0x1C, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief vco adc is calibrated every 2^(cal_duty_cycle+14+8) clock cycles
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalDutyCycleBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalDutyCycleBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x121), &rxBfData[0], 0x1C, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief force cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalForceBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapCalForceBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalForceBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x14), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief force cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalForceBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalForceBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief control for calibration modes: 2'b0 -- foreground cal 2'b1 -- duty cycle background cal (re-runs every cal_duty_cycle clock (fs) samples), otherwise continuous background cal.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalFreqControlBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapCalFreqControlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalFreqControlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x123), (uint8_t)bfValue, 0x18, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief control for calibration modes: 2'b0 -- foreground cal 2'b1 -- duty cycle background cal (re-runs every cal_duty_cycle clock (fs) samples), otherwise continuous background cal.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalFreqControlBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalFreqControlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x123), &rxBfData[0], 0x18, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief reset cal, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalRstBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapCalRstBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalRstBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief reset cal, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalRstBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalRstBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief calibration sequence load mode, set to 0
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalRzBfSet(adi_adrv9001_Device_t *device,
                                        adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapCalRzBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalRzBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x15), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief calibration sequence load mode, set to 0
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapCalRzBfGet(adi_adrv9001_Device_t *device,
                                        adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapCalRzBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x15), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief calibration DAC shift clock divide rate (00=div-by-2, 01=div-by-4, 1x=div-by-8)
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapClkshiftBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapClkshiftBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapClkshiftBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x15), (uint8_t)bfValue, 0xC, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief calibration DAC shift clock divide rate (00=div-by-2, 01=div-by-4, 1x=div-by-8)
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapClkshiftBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapClkshiftBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x15), &rxBfData[0], 0xC, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief In kgain calculation, one of the codes used is 128 and the other code used is 128+dacval_kgain
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDacvalKgainBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDacvalKgainBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDacvalKgainBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x128), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief In kgain calculation, one of the codes used is 128 and the other code used is 128+dacval_kgain
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDacvalKgainBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDacvalKgainBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x128), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief resets all digital blocks except memory map - self clearing
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDigCalRstBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDigCalRstBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDigCalRstBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x123), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief resets all digital blocks except memory map - self clearing
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDigCalRstBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDigCalRstBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x123), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief disable bias amplifier
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDisBiasAmpBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDisBiasAmpBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDisBiasAmpBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief disable bias amplifier
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDisBiasAmpBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDisBiasAmpBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief disable slope-based over-range correction
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDisOrcBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDisOrcBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDisOrcBfSet Address");
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
 * \brief disable slope-based over-range correction
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDisOrcBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDisOrcBfGet Address");
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
 * \brief dither auto disable, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithDisAutoBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDithDisAutoBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithDisAutoBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x7), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief dither auto disable, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithDisAutoBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithDisAutoBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief dither clock divide
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithDivBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDithDivBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithDivBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x7), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief dither clock divide
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithDivBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithDivBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief dither mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithModeBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDithModeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithModeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x7), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief dither mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithModeBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithModeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief reset dither clock gen, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithRstClkgenBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDithRstClkgenBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithRstClkgenBfSet Address");
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
 * \brief reset dither clock gen, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithRstClkgenBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithRstClkgenBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief reset dither dac, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithRstDacBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDithRstDacBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithRstDacBfSet Address");
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
 * \brief reset dither dac, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithRstDacBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithRstDacBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief reset dither lfsr, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithRstLfsrBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapDithRstLfsrBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithRstLfsrBfSet Address");
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
 * \brief reset dither lfsr, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDithRstLfsrBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDithRstLfsrBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief read only bit that indicates bcenter cal is completed
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDoneBcenterCalBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDoneBcenterCalBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x122), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief read bit only indicating kgain cal is done.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDoneKgainCalBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDoneKgainCalBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x128), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief read only bit indicating ramp cal is complete
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapDoneRampCalBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapDoneRampCalBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x129), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief clock enable for the VCO ADC digital
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnableClockBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEnableClockBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnableClockBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x123), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief clock enable for the VCO ADC digital
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnableClockBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnableClockBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x123), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable additional two signal channels for 3dB noise improvement
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEn34BfSet(adi_adrv9001_Device_t *device,
                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEn34BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEn34BfSet Address");
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
 * \brief enable additional two signal channels for 3dB noise improvement
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEn34BfGet(adi_adrv9001_Device_t *device,
                                       adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEn34BfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable channel i adc clock, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnAdcclkIBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEnAdcclkIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnAdcclkIBfSet Address");
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
 * \brief enable channel i adc clock, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnAdcclkIBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnAdcclkIBfGet Address");
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
 * \brief enable channel q adc clock, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnAdcclkQBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEnAdcclkQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnAdcclkQBfSet Address");
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
 * \brief enable channel q adc clock, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnAdcclkQBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnAdcclkQBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable for adc bias block, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnBiasBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEnBiasBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnBiasBfSet Address");
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
 * \brief enable for adc bias block, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnBiasBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnBiasBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable calibration analog blocks, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnCalAnaBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEnCalAnaBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnCalAnaBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief enable calibration analog blocks, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnCalAnaBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnCalAnaBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable calibration clock and digital blocks, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnCalClkBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEnCalClkBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnCalClkBfSet Address");
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
 * \brief enable calibration clock and digital blocks, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnCalClkBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnCalClkBfGet Address");
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
 * \brief enable channel i analog front end, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnVtiIBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEnVtiIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnVtiIBfSet Address");
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
 * \brief enable channel i analog front end, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnVtiIBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnVtiIBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable channel q analog front end, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnVtiQBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapEnVtiQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnVtiQBfSet Address");
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
 * \brief enable channel q analog front end, set to 0 for power-down
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapEnVtiQBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapEnVtiQBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief set this to 1 if we want to force a dac level.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapForceDacLevelBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapForceDacLevelBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapForceDacLevelBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x12D), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief set this to 1 if we want to force a dac level.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapForceDacLevelBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapForceDacLevelBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x12D), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief set this to 1 in order to force dac level.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapForceDacLevelEnBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapForceDacLevelEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapForceDacLevelEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12E), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief set this to 1 in order to force dac level.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapForceDacLevelEnBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapForceDacLevelEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12E), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief use over-range correction counter method 2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapForceOrc1BfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapForceOrc1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapForceOrc1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief use over-range correction counter method 2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapForceOrc1BfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapForceOrc1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief use over-range correction counter method 2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapForceOrc2BfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapForceOrc2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapForceOrc2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief use over-range correction counter method 2
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapForceOrc2BfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapForceOrc2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief value of gain_bcenter that multiplies accumulated value for bcenter calculation. Can range from 1/64 to (2^7-1)/2^4 (how these bits are interpreted by hardware depends on gain_range bits in bcenter_cal register.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapGainBcenterBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapGainBcenterBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapGainBcenterBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x126), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief value of gain_bcenter that multiplies accumulated value for bcenter calculation. Can range from 1/64 to (2^7-1)/2^4 (how these bits are interpreted by hardware depends on gain_range bits in bcenter_cal register.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapGainBcenterBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapGainBcenterBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x126), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief instead of computing kgain value, enter it via regmap. the format is 4.17
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapGainValueBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
                                             uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 2097151U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_VcoAdcMemMapGainValueBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapGainValueBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x136), (uint8_t)(bfValue >> 16), 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x135), (uint8_t)(bfValue >> 8));
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x134), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief instead of computing kgain value, enter it via regmap. the format is 4.17
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapGainValueBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapGainValueBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x136), &rxBfData[0x0], 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x135), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x134), &rxBfData[0x2], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x3, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief set this to 1 in order to use kgain value from the regmap.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapGainValueRegmapEnBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapGainValueRegmapEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapGainValueRegmapEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12E), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief set this to 1 in order to use kgain value from the regmap.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapGainValueRegmapEnBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapGainValueRegmapEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12E), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief samples to accumulate for kgain cal -- entry 0x0 is 16*8 samples, 0x1 is 32*8, 0x2 is 64*8, 0x3 is 128*8, 0x4 is 256*8, 0x5 is 512*8, 0x6 is 1024*8, 0x7 is 2048*8.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapKgainAccCntValBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapKgainAccCntValBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapKgainAccCntValBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x128), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief samples to accumulate for kgain cal -- entry 0x0 is 16*8 samples, 0x1 is 32*8, 0x2 is 64*8, 0x3 is 128*8, 0x4 is 256*8, 0x5 is 512*8, 0x6 is 1024*8, 0x7 is 2048*8.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapKgainAccCntValBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapKgainAccCntValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x128), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable kgain cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapKgainCalEnBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapKgainCalEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapKgainCalEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x123), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief enable kgain cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapKgainCalEnBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapKgainCalEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x123), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief this is kgain value from Gerry's matlab code that is used as a constant for ramp calculation. it is (1/16)*(1/vmax)*(1/Kvti)*2^13. Enter this floating point value as 21 bit value with 9 integer bits and 12 subLSB bits.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapKgainValBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
                                            uint32_t bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Range check */
    if ((bfValue < 0) ||
        (bfValue > 4194303U))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_VcoAdcMemMapKgainValBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapKgainValBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12C), (uint8_t)(bfValue >> 16), 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x12B), (uint8_t)(bfValue >> 8));
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x12A), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief this is kgain value from Gerry's matlab code that is used as a constant for ramp calculation. it is (1/16)*(1/vmax)*(1/Kvti)*2^13. Enter this floating point value as 21 bit value with 9 integer bits and 12 subLSB bits.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint32_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapKgainValBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapKgainValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12C), &rxBfData[0x0], 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x12B), &rxBfData[0x1], 0x00);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x12A), &rxBfData[0x2], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x3, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint32_t) data;

    return recoveryAction;
}
/**
 * \brief lut coefficients 128x16 bits
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
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
int32_t adrv9001_VcoAdcMemMapLutDataBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapLutDataBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutDataBfSet Address");
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
            (channelId != 0x3) &&
            (channelId != 0x4) &&
            (channelId != 0x5) &&
            (channelId != 0x6) &&
            (channelId != 0x7) &&
            (channelId != 0x8) &&
            (channelId != 0x9) &&
            (channelId != 0xA) &&
            (channelId != 0xB) &&
            (channelId != 0xC) &&
            (channelId != 0xD) &&
            (channelId != 0xE) &&
            (channelId != 0xF) &&
            (channelId != 0x10) &&
            (channelId != 0x11) &&
            (channelId != 0x12) &&
            (channelId != 0x13) &&
            (channelId != 0x14) &&
            (channelId != 0x15) &&
            (channelId != 0x16) &&
            (channelId != 0x17) &&
            (channelId != 0x18) &&
            (channelId != 0x19) &&
            (channelId != 0x1A) &&
            (channelId != 0x1B) &&
            (channelId != 0x1C) &&
            (channelId != 0x1D) &&
            (channelId != 0x1E) &&
            (channelId != 0x1F) &&
            (channelId != 0x20) &&
            (channelId != 0x21) &&
            (channelId != 0x22) &&
            (channelId != 0x23) &&
            (channelId != 0x24) &&
            (channelId != 0x25) &&
            (channelId != 0x26) &&
            (channelId != 0x27) &&
            (channelId != 0x28) &&
            (channelId != 0x29) &&
            (channelId != 0x2A) &&
            (channelId != 0x2B) &&
            (channelId != 0x2C) &&
            (channelId != 0x2D) &&
            (channelId != 0x2E) &&
            (channelId != 0x2F) &&
            (channelId != 0x30) &&
            (channelId != 0x31) &&
            (channelId != 0x32) &&
            (channelId != 0x33) &&
            (channelId != 0x34) &&
            (channelId != 0x35) &&
            (channelId != 0x36) &&
            (channelId != 0x37) &&
            (channelId != 0x38) &&
            (channelId != 0x39) &&
            (channelId != 0x3A) &&
            (channelId != 0x3B) &&
            (channelId != 0x3C) &&
            (channelId != 0x3D) &&
            (channelId != 0x3E) &&
            (channelId != 0x3F) &&
            (channelId != 0x40) &&
            (channelId != 0x41) &&
            (channelId != 0x42) &&
            (channelId != 0x43) &&
            (channelId != 0x44) &&
            (channelId != 0x45) &&
            (channelId != 0x46) &&
            (channelId != 0x47) &&
            (channelId != 0x48) &&
            (channelId != 0x49) &&
            (channelId != 0x4A) &&
            (channelId != 0x4B) &&
            (channelId != 0x4C) &&
            (channelId != 0x4D) &&
            (channelId != 0x4E) &&
            (channelId != 0x4F) &&
            (channelId != 0x50) &&
            (channelId != 0x51) &&
            (channelId != 0x52) &&
            (channelId != 0x53) &&
            (channelId != 0x54) &&
            (channelId != 0x55) &&
            (channelId != 0x56) &&
            (channelId != 0x57) &&
            (channelId != 0x58) &&
            (channelId != 0x59) &&
            (channelId != 0x5A) &&
            (channelId != 0x5B) &&
            (channelId != 0x5C) &&
            (channelId != 0x5D) &&
            (channelId != 0x5E) &&
            (channelId != 0x5F) &&
            (channelId != 0x60) &&
            (channelId != 0x61) &&
            (channelId != 0x62) &&
            (channelId != 0x63) &&
            (channelId != 0x64) &&
            (channelId != 0x65) &&
            (channelId != 0x66) &&
            (channelId != 0x67) &&
            (channelId != 0x68) &&
            (channelId != 0x69) &&
            (channelId != 0x6A) &&
            (channelId != 0x6B) &&
            (channelId != 0x6C) &&
            (channelId != 0x6D) &&
            (channelId != 0x6E) &&
            (channelId != 0x6F) &&
            (channelId != 0x70) &&
            (channelId != 0x71) &&
            (channelId != 0x72) &&
            (channelId != 0x73) &&
            (channelId != 0x74) &&
            (channelId != 0x75) &&
            (channelId != 0x76) &&
            (channelId != 0x77) &&
            (channelId != 0x78) &&
            (channelId != 0x79) &&
            (channelId != 0x7A) &&
            (channelId != 0x7B) &&
            (channelId != 0x7C) &&
            (channelId != 0x7D) &&
            (channelId != 0x7E) &&
            (channelId != 0x7F) &&
            (channelId != 0x80) &&
            (channelId != 0x81) &&
            (channelId != 0x82) &&
            (channelId != 0x83) &&
            (channelId != 0x84) &&
            (channelId != 0x85) &&
            (channelId != 0x86) &&
            (channelId != 0x87) &&
            (channelId != 0x88) &&
            (channelId != 0x89) &&
            (channelId != 0x8A) &&
            (channelId != 0x8B) &&
            (channelId != 0x8C) &&
            (channelId != 0x8D) &&
            (channelId != 0x8E) &&
            (channelId != 0x8F) &&
            (channelId != 0x90) &&
            (channelId != 0x91) &&
            (channelId != 0x92) &&
            (channelId != 0x93) &&
            (channelId != 0x94) &&
            (channelId != 0x95) &&
            (channelId != 0x96) &&
            (channelId != 0x97) &&
            (channelId != 0x98) &&
            (channelId != 0x99) &&
            (channelId != 0x9A) &&
            (channelId != 0x9B) &&
            (channelId != 0x9C) &&
            (channelId != 0x9D) &&
            (channelId != 0x9E) &&
            (channelId != 0x9F) &&
            (channelId != 0xA0) &&
            (channelId != 0xA1) &&
            (channelId != 0xA2) &&
            (channelId != 0xA3) &&
            (channelId != 0xA4) &&
            (channelId != 0xA5) &&
            (channelId != 0xA6) &&
            (channelId != 0xA7) &&
            (channelId != 0xA8) &&
            (channelId != 0xA9) &&
            (channelId != 0xAA) &&
            (channelId != 0xAB) &&
            (channelId != 0xAC) &&
            (channelId != 0xAD) &&
            (channelId != 0xAE) &&
            (channelId != 0xAF) &&
            (channelId != 0xB0) &&
            (channelId != 0xB1) &&
            (channelId != 0xB2) &&
            (channelId != 0xB3) &&
            (channelId != 0xB4) &&
            (channelId != 0xB5) &&
            (channelId != 0xB6) &&
            (channelId != 0xB7) &&
            (channelId != 0xB8) &&
            (channelId != 0xB9) &&
            (channelId != 0xBA) &&
            (channelId != 0xBB) &&
            (channelId != 0xBC) &&
            (channelId != 0xBD) &&
            (channelId != 0xBE) &&
            (channelId != 0xBF) &&
            (channelId != 0xC0) &&
            (channelId != 0xC1) &&
            (channelId != 0xC2) &&
            (channelId != 0xC3) &&
            (channelId != 0xC4) &&
            (channelId != 0xC5) &&
            (channelId != 0xC6) &&
            (channelId != 0xC7) &&
            (channelId != 0xC8) &&
            (channelId != 0xC9) &&
            (channelId != 0xCA) &&
            (channelId != 0xCB) &&
            (channelId != 0xCC) &&
            (channelId != 0xCD) &&
            (channelId != 0xCE) &&
            (channelId != 0xCF) &&
            (channelId != 0xD0) &&
            (channelId != 0xD1) &&
            (channelId != 0xD2) &&
            (channelId != 0xD3) &&
            (channelId != 0xD4) &&
            (channelId != 0xD5) &&
            (channelId != 0xD6) &&
            (channelId != 0xD7) &&
            (channelId != 0xD8) &&
            (channelId != 0xD9) &&
            (channelId != 0xDA) &&
            (channelId != 0xDB) &&
            (channelId != 0xDC) &&
            (channelId != 0xDD) &&
            (channelId != 0xDE) &&
            (channelId != 0xDF) &&
            (channelId != 0xE0) &&
            (channelId != 0xE1) &&
            (channelId != 0xE2) &&
            (channelId != 0xE3) &&
            (channelId != 0xE4) &&
            (channelId != 0xE5) &&
            (channelId != 0xE6) &&
            (channelId != 0xE7) &&
            (channelId != 0xE8) &&
            (channelId != 0xE9) &&
            (channelId != 0xEA) &&
            (channelId != 0xEB) &&
            (channelId != 0xEC) &&
            (channelId != 0xED) &&
            (channelId != 0xEE) &&
            (channelId != 0xEF) &&
            (channelId != 0xF0) &&
            (channelId != 0xF1) &&
            (channelId != 0xF2) &&
            (channelId != 0xF3) &&
            (channelId != 0xF4) &&
            (channelId != 0xF5) &&
            (channelId != 0xF6) &&
            (channelId != 0xF7) &&
            (channelId != 0xF8) &&
            (channelId != 0xF9) &&
            (channelId != 0xFA) &&
            (channelId != 0xFB) &&
            (channelId != 0xFC) &&
            (channelId != 0xFD) &&
            (channelId != 0xFE) &&
            (channelId != 0xFF)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_VcoAdcMemMapLutDataBfSet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x20 + channelId*1), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief lut coefficients 128x16 bits
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
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
int32_t adrv9001_VcoAdcMemMapLutDataBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutDataBfGet Address");
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
            (channelId != 0x3) &&
            (channelId != 0x4) &&
            (channelId != 0x5) &&
            (channelId != 0x6) &&
            (channelId != 0x7) &&
            (channelId != 0x8) &&
            (channelId != 0x9) &&
            (channelId != 0xA) &&
            (channelId != 0xB) &&
            (channelId != 0xC) &&
            (channelId != 0xD) &&
            (channelId != 0xE) &&
            (channelId != 0xF) &&
            (channelId != 0x10) &&
            (channelId != 0x11) &&
            (channelId != 0x12) &&
            (channelId != 0x13) &&
            (channelId != 0x14) &&
            (channelId != 0x15) &&
            (channelId != 0x16) &&
            (channelId != 0x17) &&
            (channelId != 0x18) &&
            (channelId != 0x19) &&
            (channelId != 0x1A) &&
            (channelId != 0x1B) &&
            (channelId != 0x1C) &&
            (channelId != 0x1D) &&
            (channelId != 0x1E) &&
            (channelId != 0x1F) &&
            (channelId != 0x20) &&
            (channelId != 0x21) &&
            (channelId != 0x22) &&
            (channelId != 0x23) &&
            (channelId != 0x24) &&
            (channelId != 0x25) &&
            (channelId != 0x26) &&
            (channelId != 0x27) &&
            (channelId != 0x28) &&
            (channelId != 0x29) &&
            (channelId != 0x2A) &&
            (channelId != 0x2B) &&
            (channelId != 0x2C) &&
            (channelId != 0x2D) &&
            (channelId != 0x2E) &&
            (channelId != 0x2F) &&
            (channelId != 0x30) &&
            (channelId != 0x31) &&
            (channelId != 0x32) &&
            (channelId != 0x33) &&
            (channelId != 0x34) &&
            (channelId != 0x35) &&
            (channelId != 0x36) &&
            (channelId != 0x37) &&
            (channelId != 0x38) &&
            (channelId != 0x39) &&
            (channelId != 0x3A) &&
            (channelId != 0x3B) &&
            (channelId != 0x3C) &&
            (channelId != 0x3D) &&
            (channelId != 0x3E) &&
            (channelId != 0x3F) &&
            (channelId != 0x40) &&
            (channelId != 0x41) &&
            (channelId != 0x42) &&
            (channelId != 0x43) &&
            (channelId != 0x44) &&
            (channelId != 0x45) &&
            (channelId != 0x46) &&
            (channelId != 0x47) &&
            (channelId != 0x48) &&
            (channelId != 0x49) &&
            (channelId != 0x4A) &&
            (channelId != 0x4B) &&
            (channelId != 0x4C) &&
            (channelId != 0x4D) &&
            (channelId != 0x4E) &&
            (channelId != 0x4F) &&
            (channelId != 0x50) &&
            (channelId != 0x51) &&
            (channelId != 0x52) &&
            (channelId != 0x53) &&
            (channelId != 0x54) &&
            (channelId != 0x55) &&
            (channelId != 0x56) &&
            (channelId != 0x57) &&
            (channelId != 0x58) &&
            (channelId != 0x59) &&
            (channelId != 0x5A) &&
            (channelId != 0x5B) &&
            (channelId != 0x5C) &&
            (channelId != 0x5D) &&
            (channelId != 0x5E) &&
            (channelId != 0x5F) &&
            (channelId != 0x60) &&
            (channelId != 0x61) &&
            (channelId != 0x62) &&
            (channelId != 0x63) &&
            (channelId != 0x64) &&
            (channelId != 0x65) &&
            (channelId != 0x66) &&
            (channelId != 0x67) &&
            (channelId != 0x68) &&
            (channelId != 0x69) &&
            (channelId != 0x6A) &&
            (channelId != 0x6B) &&
            (channelId != 0x6C) &&
            (channelId != 0x6D) &&
            (channelId != 0x6E) &&
            (channelId != 0x6F) &&
            (channelId != 0x70) &&
            (channelId != 0x71) &&
            (channelId != 0x72) &&
            (channelId != 0x73) &&
            (channelId != 0x74) &&
            (channelId != 0x75) &&
            (channelId != 0x76) &&
            (channelId != 0x77) &&
            (channelId != 0x78) &&
            (channelId != 0x79) &&
            (channelId != 0x7A) &&
            (channelId != 0x7B) &&
            (channelId != 0x7C) &&
            (channelId != 0x7D) &&
            (channelId != 0x7E) &&
            (channelId != 0x7F) &&
            (channelId != 0x80) &&
            (channelId != 0x81) &&
            (channelId != 0x82) &&
            (channelId != 0x83) &&
            (channelId != 0x84) &&
            (channelId != 0x85) &&
            (channelId != 0x86) &&
            (channelId != 0x87) &&
            (channelId != 0x88) &&
            (channelId != 0x89) &&
            (channelId != 0x8A) &&
            (channelId != 0x8B) &&
            (channelId != 0x8C) &&
            (channelId != 0x8D) &&
            (channelId != 0x8E) &&
            (channelId != 0x8F) &&
            (channelId != 0x90) &&
            (channelId != 0x91) &&
            (channelId != 0x92) &&
            (channelId != 0x93) &&
            (channelId != 0x94) &&
            (channelId != 0x95) &&
            (channelId != 0x96) &&
            (channelId != 0x97) &&
            (channelId != 0x98) &&
            (channelId != 0x99) &&
            (channelId != 0x9A) &&
            (channelId != 0x9B) &&
            (channelId != 0x9C) &&
            (channelId != 0x9D) &&
            (channelId != 0x9E) &&
            (channelId != 0x9F) &&
            (channelId != 0xA0) &&
            (channelId != 0xA1) &&
            (channelId != 0xA2) &&
            (channelId != 0xA3) &&
            (channelId != 0xA4) &&
            (channelId != 0xA5) &&
            (channelId != 0xA6) &&
            (channelId != 0xA7) &&
            (channelId != 0xA8) &&
            (channelId != 0xA9) &&
            (channelId != 0xAA) &&
            (channelId != 0xAB) &&
            (channelId != 0xAC) &&
            (channelId != 0xAD) &&
            (channelId != 0xAE) &&
            (channelId != 0xAF) &&
            (channelId != 0xB0) &&
            (channelId != 0xB1) &&
            (channelId != 0xB2) &&
            (channelId != 0xB3) &&
            (channelId != 0xB4) &&
            (channelId != 0xB5) &&
            (channelId != 0xB6) &&
            (channelId != 0xB7) &&
            (channelId != 0xB8) &&
            (channelId != 0xB9) &&
            (channelId != 0xBA) &&
            (channelId != 0xBB) &&
            (channelId != 0xBC) &&
            (channelId != 0xBD) &&
            (channelId != 0xBE) &&
            (channelId != 0xBF) &&
            (channelId != 0xC0) &&
            (channelId != 0xC1) &&
            (channelId != 0xC2) &&
            (channelId != 0xC3) &&
            (channelId != 0xC4) &&
            (channelId != 0xC5) &&
            (channelId != 0xC6) &&
            (channelId != 0xC7) &&
            (channelId != 0xC8) &&
            (channelId != 0xC9) &&
            (channelId != 0xCA) &&
            (channelId != 0xCB) &&
            (channelId != 0xCC) &&
            (channelId != 0xCD) &&
            (channelId != 0xCE) &&
            (channelId != 0xCF) &&
            (channelId != 0xD0) &&
            (channelId != 0xD1) &&
            (channelId != 0xD2) &&
            (channelId != 0xD3) &&
            (channelId != 0xD4) &&
            (channelId != 0xD5) &&
            (channelId != 0xD6) &&
            (channelId != 0xD7) &&
            (channelId != 0xD8) &&
            (channelId != 0xD9) &&
            (channelId != 0xDA) &&
            (channelId != 0xDB) &&
            (channelId != 0xDC) &&
            (channelId != 0xDD) &&
            (channelId != 0xDE) &&
            (channelId != 0xDF) &&
            (channelId != 0xE0) &&
            (channelId != 0xE1) &&
            (channelId != 0xE2) &&
            (channelId != 0xE3) &&
            (channelId != 0xE4) &&
            (channelId != 0xE5) &&
            (channelId != 0xE6) &&
            (channelId != 0xE7) &&
            (channelId != 0xE8) &&
            (channelId != 0xE9) &&
            (channelId != 0xEA) &&
            (channelId != 0xEB) &&
            (channelId != 0xEC) &&
            (channelId != 0xED) &&
            (channelId != 0xEE) &&
            (channelId != 0xEF) &&
            (channelId != 0xF0) &&
            (channelId != 0xF1) &&
            (channelId != 0xF2) &&
            (channelId != 0xF3) &&
            (channelId != 0xF4) &&
            (channelId != 0xF5) &&
            (channelId != 0xF6) &&
            (channelId != 0xF7) &&
            (channelId != 0xF8) &&
            (channelId != 0xF9) &&
            (channelId != 0xFA) &&
            (channelId != 0xFB) &&
            (channelId != 0xFC) &&
            (channelId != 0xFD) &&
            (channelId != 0xFE) &&
            (channelId != 0xFF)
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channelId,
                         "Invalid adrv9001_VcoAdcMemMapLutDataBfGet channelId");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x20 + channelId*1), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable LUT clock gate, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapLutEnClkgateBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapLutEnClkgateBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutEnClkgateBfSet Address");
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
 * \brief enable LUT clock gate, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapLutEnClkgateBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutEnClkgateBfGet Address");
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
 * \brief LUT load edge
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapLutLoadedgeBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapLutLoadedgeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutLoadedgeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief LUT load edge
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapLutLoadedgeBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutLoadedgeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief this bit will flag high if we are trying to write to the lut regmap when matrix_data_en is high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapLutRegmapAccessInvalidBfGet(adi_adrv9001_Device_t *device,
                                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutRegmapAccessInvalidBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x137), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief reset lut, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapLutRstBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapLutRstBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutRstBfSet Address");
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
 * \brief reset lut, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapLutRstBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapLutRstBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable writes to lut regmap from the dig accum block
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapMatrixDataEnBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapMatrixDataEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapMatrixDataEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x137), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief enable writes to lut regmap from the dig accum block
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapMatrixDataEnBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapMatrixDataEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x137), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief output enables for clocks
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapOutCtrlBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapOutCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapOutCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x137), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief output enables for clocks
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapOutCtrlBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapOutCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x137), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief disable demux output, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapOutDisDemuxBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapOutDisDemuxBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapOutDisDemuxBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief disable demux output, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapOutDisDemuxBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapOutDisDemuxBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief disable over-range, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapOvrDisBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapOvrDisBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapOvrDisBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x15), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief disable over-range, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapOvrDisBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapOvrDisBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x15), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief reset over-range, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapOvrRstBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapOvrRstBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapOvrRstBfSet Address");
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
 * \brief reset over-range, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapOvrRstBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapOvrRstBfGet Address");
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

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief number of wait cycles for powerup (these are clk cycles with respect to fs/8 clock). The number of clock cycles is 1 <<< this reg entry -- so it can be 1,2,4,8,16 up to 32768
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapPwrupWaitClkBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapPwrupWaitClkBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapPwrupWaitClkBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x129), (uint8_t)bfValue, 0xF0, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief number of wait cycles for powerup (these are clk cycles with respect to fs/8 clock). The number of clock cycles is 1 <<< this reg entry -- so it can be 1,2,4,8,16 up to 32768
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapPwrupWaitClkBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapPwrupWaitClkBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x129), &rxBfData[0], 0xF0, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief samples to accumulate for ramp cal. 0x0 is 16*8, 0x1 is 64*8, 0x2 is 128*8, 0x3 is 256*8, 0x4 is 512*8, 0x5 is 1024*8, 0x6 is 2048*8, 0x7 is 4096*8
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapRampAccCntValBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapRampAccCntValBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapRampAccCntValBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x129), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief samples to accumulate for ramp cal. 0x0 is 16*8, 0x1 is 64*8, 0x2 is 128*8, 0x3 is 256*8, 0x4 is 512*8, 0x5 is 1024*8, 0x6 is 2048*8, 0x7 is 4096*8
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapRampAccCntValBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapRampAccCntValBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x129), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief enable ramp cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapRampCalEnBfSet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapRampCalEnBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapRampCalEnBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x123), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief enable ramp cal
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapRampCalEnBfGet(adi_adrv9001_Device_t *device,
                                            adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapRampCalEnBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x123), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief calibration DAC bit shift value (00=shift-by-4, 01=shift-b- 8, 1x=shift-by-16)
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapRegshiftBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapRegshiftBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapRegshiftBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x15), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief calibration DAC bit shift value (00=shift-by-4, 01=shift-b- 8, 1x=shift-by-16)
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapRegshiftBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapRegshiftBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x15), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief reset cal, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapRstCalBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapRstCalBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapRstCalBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x15), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief reset cal, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapRstCalBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapRstCalBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x15), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief adc reset mode, set to 0
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSelRstBfSet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSelRstBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSelRstBfSet Address");
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
 * \brief adc reset mode, set to 0
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSelRstBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSelRstBfGet Address");
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
 * \brief reset adc when LP ADC enters deep saturation
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSelShortBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSelShortBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSelShortBfSet Address");
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
 * \brief reset adc when LP ADC enters deep saturation
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSelShortBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSelShortBfGet Address");
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg017BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg017BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg017BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg017BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg017BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg025BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg025BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg025BfSet Address");
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
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg025BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg025BfGet Address");
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg026BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg026BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg026BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg026BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg026BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg027BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg027BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg027BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg027BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg027BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg036BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg036BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg036BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg036BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg036BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg037BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg037BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg037BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg037BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg037BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg045BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg045BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg045BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x20, 0x5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg045BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg045BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x20, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg046BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg046BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg046BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg046BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg046BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg047BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg047BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg047BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x4), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg047BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg047BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x4), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg050BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg050BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg050BfSet Address");
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg050BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg050BfGet Address");
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
 * \brief No description provided
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg051BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg051BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg051BfSet Address");
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg051BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg051BfGet Address");
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg052BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg052BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg052BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5), (uint8_t)bfValue, 0x4, 0x2);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg052BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg052BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5), &rxBfData[0], 0x4, 0xC2);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg053BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg053BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg053BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5), (uint8_t)bfValue, 0x8, 0x3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg053BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg053BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg054BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg054BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg054BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5), (uint8_t)bfValue, 0x10, 0x4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg054BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg054BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg055BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg055BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg055BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5), (uint8_t)bfValue, 0x20, 0x5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg055BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg055BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5), &rxBfData[0], 0x20, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg056BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg056BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg056BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg056BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg056BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg057BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg057BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg057BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x5), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg057BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg057BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x5), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg063BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg063BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg063BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x8, 0x3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg063BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg063BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg067BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg067BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg067BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg067BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg067BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg076BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg076BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg076BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x7), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg076BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg076BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg077BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg077BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg077BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x7), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg077BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg077BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b3BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b3BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b3BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)bfValue, 0x8, 0x3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b3BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b4BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b4BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b4BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)bfValue, 0x10, 0x4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b4BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b4BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b5BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b5BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b5BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)bfValue, 0x20, 0x5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b5BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b5BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0], 0x20, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b6BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b6BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b6BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b6BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b6BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b7BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b7BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b7BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0b7BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0b7BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c0BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c0BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c0BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x1, 0x0);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c0BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c0BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x1, 0xC0);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c1BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c1BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c1BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x2, 0x1);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c1BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c1BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x2, 0xC1);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c2BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c2BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c2BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x4, 0x2);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c2BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c2BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x4, 0xC2);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c3BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c3BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c3BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x8, 0x3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c3BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c3BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c4BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c4BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c4BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x10, 0x4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c4BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c4BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c5BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c5BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c5BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x20, 0x5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c5BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c5BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x20, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c6BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c6BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c6BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c6BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c6BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c7BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c7BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c7BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg0c7BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg0c7BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg123BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg123BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg123BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12), (uint8_t)bfValue, 0x8, 0x3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg123BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg123BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg124BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg124BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg124BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12), (uint8_t)bfValue, 0x10, 0x4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg124BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg124BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg125BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg125BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg125BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12), (uint8_t)bfValue, 0x20, 0x5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg125BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg125BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12), &rxBfData[0], 0x20, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg126BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg126BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg126BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg126BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg126BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg127BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg127BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg127BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg127BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg127BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg143BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg143BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg143BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x14), (uint8_t)bfValue, 0x8, 0x3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg143BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg143BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0x8, 0xC3);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg144BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg144BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg144BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x14), (uint8_t)bfValue, 0x10, 0x4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg144BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg144BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0x10, 0xC4);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg145BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg145BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg145BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x14), (uint8_t)bfValue, 0x20, 0x5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg145BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg145BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0x20, 0xC5);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg146BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg146BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg146BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x14), (uint8_t)bfValue, 0x40, 0x6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg146BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg146BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0x40, 0xC6);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg147BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg147BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg147BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x14), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg147BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg147BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0x80, 0xC7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg157BfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpareReg157BfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg157BfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x15), (uint8_t)bfValue, 0x80, 0x7);
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
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpareReg157BfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpareReg157BfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x15), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief load lut data from lut regmap into the LUT
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpiLoadBfSet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapSpiLoadBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpiLoadBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x137), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief load lut data from lut regmap into the LUT
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapSpiLoadBfGet(adi_adrv9001_Device_t *device,
                                          adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapSpiLoadBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x137), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief disable vti shift, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapVtiDisShiftBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapVtiDisShiftBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapVtiDisShiftBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief disable vti shift, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapVtiDisShiftBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapVtiDisShiftBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief force vti, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapVtiForceBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapVtiForceBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapVtiForceBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief force vti, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapVtiForceBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapVtiForceBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief short vti, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapVtiShortBfSet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_VcoAdcMemMapVtiShortBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapVtiShortBfSet Address");
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
 * \brief short vti, active high
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapVtiShortBfGet(adi_adrv9001_Device_t *device,
                                           adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapVtiShortBfGet Address");
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
 * \brief state of vco_fsm
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfVcoAdcMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_VcoAdcMemMapVStateBfGet(adi_adrv9001_Device_t *device,
                                         adrv9001_BfVcoAdcMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_RX1_ADC_LP) &&
            (baseAddr != ADRV9001_BF_RX2_ADC_LP))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_VcoAdcMemMapVStateBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12E), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/*  EOF: adrv9001_bf_vco_adc_mem_map.c */

#endif