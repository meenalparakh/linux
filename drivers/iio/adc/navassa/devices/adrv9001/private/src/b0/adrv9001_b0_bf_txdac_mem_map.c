#ifdef SI_REV_B0
/**
 * \file adrv9001_bf_txdac_mem_map.c Automatically generated file with generator ver 0.0.1.0.
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

#include "adrv9001_bf_txdac_mem_map.h"
#include "adrv9001_bf_hal.h"
#include "adi_common_error.h"

/**
 * \brief DAC clock divide ratio for calibration clock.  Set to 3-6 MHz
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalclkDiv_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalclkDivBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalclkDiv_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_OFF            ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV17          ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV16          ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV15          ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV14          ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV13          ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV12          ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV11          ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV10          ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV9           ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV8           ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV7           ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV6           ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV6_13        ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV6_14        ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALCLK_DIV6_15        )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkDivBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkDivBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DAC clock divide ratio for calibration clock.  Set to 3-6 MHz
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalclkDiv_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalclkDivBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalclkDiv_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkDivBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCalclkDiv_e) data;

    return recoveryAction;
}
/**
 * \brief I DAC calibration clock enable.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalclkEnIBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkEnIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkEnIBfSet Address");
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
 * \brief I DAC calibration clock enable.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalclkEnIBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkEnIBfGet Address");
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
 * \brief Q DAC calibration clock enable.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalclkEnQBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkEnQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkEnQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x6), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Q DAC calibration clock enable.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalclkEnQBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalclkEnQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x6), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Calibration DAC's Fullscale Current Setting
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCaldacFsc_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCaldacFscBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCaldacFsc_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_STEP_0P04      ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_STEP_0P06      ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_STEP_0P08      ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_STEP_0P12      )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCaldacFscBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCaldacFscBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Calibration DAC's Fullscale Current Setting
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCaldacFsc_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCaldacFscBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCaldacFsc_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCaldacFscBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCaldacFsc_e) data;

    return recoveryAction;
}
/**
 * \brief Calibration MSB DAC's offset
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCaldacOffset_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCaldacOffsetBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                    adrv9001_BfTxdacMemMapTxdacCaldacOffset_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_OFFSET_0P0     ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_OFFSET_3P5     ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_OFFSET_4P0     ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_OFFSET_4P5     ) &&
            (bfValue != ADRV9001_BF_TXDAC_CALDAC_OFFSET_5P0     )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCaldacOffsetBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCaldacOffsetBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xC), (uint8_t)bfValue, 0x70, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Calibration MSB DAC's offset
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCaldacOffset_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCaldacOffsetBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                    adrv9001_BfTxdacMemMapTxdacCaldacOffset_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCaldacOffsetBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xC), &rxBfData[0], 0x70, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCaldacOffset_e) data;

    return recoveryAction;
}

/**
 * \brief IDAC Calibration status
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalActiveIBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalActiveIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief QDAC Calibration status
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalActiveQBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalActiveQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief IDAC Address for CALDAC read and write operations.  Also address for debug mode.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalAddrI_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalAddrIBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                adrv9001_BfTxdacMemMapTxdacCalAddrI_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISBREF    ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB0      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB1      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB2      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB3      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB4      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB5      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB6      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB7      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB8      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB9      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB10     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB11     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB12     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB13     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISB14     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_LSBSUM    ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED0   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED1   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED2   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED3   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED4   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED5   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED6   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSBCOMPPOS) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSBCOMPNEG) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISBCOMPPOS) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_ISBCOMPNEG) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED7   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED8   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_UNUSED9   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSBREF    ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSBGAIN   ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB0      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB1      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB2      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB3      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB4      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB5      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB6      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB7      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB8      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB9      ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB10     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB11     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB12     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB13     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB14     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB15     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB16     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB17     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB18     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB19     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB20     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB21     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB22     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB23     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB24     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB25     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB26     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB27     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB28     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB29     ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_CALADDR_MSB30     )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCalAddrIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalAddrIBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x10), (uint8_t)bfValue, 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief IDAC Address for CALDAC read and write operations.  Also address for debug mode.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalAddrI_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalAddrIBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                adrv9001_BfTxdacMemMapTxdacCalAddrI_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalAddrIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x10), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCalAddrI_e) data;

    return recoveryAction;
}
/**
 * \brief QDAC Address for CALDAC read and write operations.  Also address for debug mode.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalAddrQ_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalAddrQBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                adrv9001_BfTxdacMemMapTxdacCalAddrQ_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISBREF    ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB0      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB1      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB2      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB3      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB4      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB5      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB6      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB7      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB8      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB9      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB10     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB11     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB12     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB13     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISB14     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_LSBSUM    ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED0   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED1   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED2   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED3   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED4   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED5   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED6   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSBCOMPPOS) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSBCOMPNEG) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISBCOMPPOS) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_ISBCOMPNEG) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED7   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED8   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_UNUSED9   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSBREF    ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSBGAIN   ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB0      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB1      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB2      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB3      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB4      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB5      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB6      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB7      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB8      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB9      ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB10     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB11     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB12     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB13     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB14     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB15     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB16     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB17     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB18     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB19     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB20     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB21     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB22     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB23     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB24     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB25     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB26     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB27     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB28     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB29     ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_CALADDR_MSB30     )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCalAddrQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalAddrQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x13), (uint8_t)bfValue, 0x3F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief QDAC Address for CALDAC read and write operations.  Also address for debug mode.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalAddrQ_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalAddrQBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                adrv9001_BfTxdacMemMapTxdacCalAddrQ_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalAddrQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x13), &rxBfData[0], 0x3F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCalAddrQ_e) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: IDAC Calibration debug mode.  Changes behavior of calibration state machine.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalDebugI_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDebugIBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalDebugI_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_CAL_IDAC_NORMAL        ) &&
            (bfValue != ADRV9001_BF_TXDAC_CAL_IDAC_DEBUG         )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDebugIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDebugIBfSet Address");
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
 * \brief DEBUG: IDAC Calibration debug mode.  Changes behavior of calibration state machine.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalDebugI_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDebugIBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalDebugI_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDebugIBfGet Address");
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

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCalDebugI_e) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: QDAC Calibration debug mode.  Changes behavior of calibration state machine.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalDebugQ_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDebugQBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalDebugQ_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_CAL_QDAC_NORMAL        ) &&
            (bfValue != ADRV9001_BF_TXDAC_CAL_QDAC_DEBUG         )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDebugQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDebugQBfSet Address");
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
 * \brief DEBUG: QDAC Calibration debug mode.  Changes behavior of calibration state machine.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalDebugQ_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDebugQBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalDebugQ_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDebugQBfGet Address");
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

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCalDebugQ_e) data;

    return recoveryAction;
}

/**
 * \brief IDAC Calibration status
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDoneIBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDoneIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief QDAC Calibration status
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDoneQBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDoneQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x7), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief SPI initialize ISB calibration reference
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDoIsbrefBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalDoIsbrefBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDoIsbrefBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xA), (uint8_t)bfValue, 0x4, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief SPI initialize ISB calibration reference
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDoIsbrefBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDoIsbrefBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xA), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Enables Calibration Machine to correct for Comparator Offsets
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDoOffsetCalBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalDoOffsetCalBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDoOffsetCalBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xA), (uint8_t)bfValue, 0x8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Enables Calibration Machine to correct for Comparator Offsets
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalDoOffsetCalBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalDoOffsetCalBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xA), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Initial value for ISB REF CALDAC.  See also txdac_cal_skip_isbref.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalIsbrefInitBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalIsbrefInitBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalIsbrefInitBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x15), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: Initial value for ISB REF CALDAC.  See also txdac_cal_skip_isbref.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalIsbrefInitBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalIsbrefInitBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x15), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Loop on ISB calibration to center ISB cal codes
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalIsbLoopBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalIsbLoopBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalIsbLoopBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xA), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Loop on ISB calibration to center ISB cal codes
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalIsbLoopBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalIsbLoopBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xA), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief IDAC Count of ISB CALDACS that saturated to 0 or 15.  Count saturates at 7.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalIsbSatIBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalIsbSatIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x8), &rxBfData[0], 0x70, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief QDAC Count of ISB CALDACS that saturated to 0 or 15.  Count saturates at 7.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalIsbSatQBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalIsbSatQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x9), &rxBfData[0], 0x70, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Loop on MSB calibration to center MSB cal codes (priority over ISB loop)
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalMsbLoopBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalMsbLoopBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalMsbLoopBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xA), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Loop on MSB calibration to center MSB cal codes (priority over ISB loop)
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalMsbLoopBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalMsbLoopBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xA), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief IDAC Count of MSB CALDACS that saturated to 0 or 127.  Count saturates at 15.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalMsbSatIBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalMsbSatIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x8), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief QDAC Count of MSB CALDACS that saturated to 0 or 127.  Count saturates at 15.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalMsbSatQBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalMsbSatQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x9), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief IDAC Flag indicating one of the Offset CALDACS saturated to 127.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalOffsetSatIBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalOffsetSatIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x8), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief QDAC Flag indicating one of the Offset CALDACS saturated to 127.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalOffsetSatQBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalOffsetSatQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x9), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief IDAC CALDAC read data.  Valid range for MSB CALDACs is 0 to 127.  Valid range for ISB CALDACs is 0 to 15.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalRdDataIBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalRdDataIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x11), &rxBfData[0], 0x7F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief QDAC CALDAC read data.  Valid range for MSB CALDACs is 0 to 127.  Valid range for ISB CALDACs is 0 to 15.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalRdDataQBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalRdDataQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x14), &rxBfData[0], 0x7F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Amount of time to wait for signals to settle before performing SAR calibration on a CALDAC.  Units are calibration clocks.  0=no delay 31=maximum delay
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalSettleDelayBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalSettleDelayBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalSettleDelayBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)bfValue, 0xF8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Amount of time to wait for signals to settle before performing SAR calibration on a CALDAC.  Units are calibration clocks.  0=no delay 31=maximum delay
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalSettleDelayBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalSettleDelayBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0], 0xF8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief I DAC Calibration Start
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalStartI_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalStartIBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalStartI_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_CAL_START_I_DISABLE    ) &&
            (bfValue != ADRV9001_BF_TXDAC_CAL_START_I_TRIGGER    )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCalStartIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalStartIBfSet Address");
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
 * \brief I DAC Calibration Start
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalStartI_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalStartIBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalStartI_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalStartIBfGet Address");
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

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCalStartI_e) data;

    return recoveryAction;
}
/**
 * \brief Q DAC Calibration Start
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalStartQ_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalStartQBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalStartQ_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_CAL_START_Q_DISABLE    ) &&
            (bfValue != ADRV9001_BF_TXDAC_CAL_START_Q_TRIGGER    )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCalStartQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalStartQBfSet Address");
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
 * \brief Q DAC Calibration Start
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCalStartQ_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalStartQBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacCalStartQ_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalStartQBfGet Address");
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

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCalStartQ_e) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Select cell for calibration debug: 0 = off, 1=driven according to TXDAC_CAL_ADDR_I/ TXDAC_CAL_ADDR_Q
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalUseSelectBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalUseSelectBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalUseSelectBfSet Address");
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
 * \brief DEBUG: Select cell for calibration debug: 0 = off, 1=driven according to TXDAC_CAL_ADDR_I/ TXDAC_CAL_ADDR_Q
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalUseSelectBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalUseSelectBfGet Address");
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
 * \brief I DAC CALDAC write operation.  0 to 1 transition will save txdac_cal_wr_data to txdac_cal_addr.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalWeIBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalWeIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalWeIBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xE), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief I DAC CALDAC write operation.  0 to 1 transition will save txdac_cal_wr_data to txdac_cal_addr.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalWeIBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalWeIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xE), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Q DAC CALDAC write operation.  0 to 1 transition will save txdac_cal_wr_data to txdac_cal_addr.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalWeQBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalWeQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalWeQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xE), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Q DAC CALDAC write operation.  0 to 1 transition will save txdac_cal_wr_data to txdac_cal_addr.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalWeQBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalWeQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xE), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief IDAC CALDAC write data.  Valid range for MSB CALDACs is 0 to 127.  Valid range for ISB CALDACs is 0 to 15.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalWrDataIBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalWrDataIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalWrDataIBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xF), (uint8_t)bfValue, 0x7F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief IDAC CALDAC write data.  Valid range for MSB CALDACs is 0 to 127.  Valid range for ISB CALDACs is 0 to 15.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalWrDataIBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalWrDataIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xF), &rxBfData[0], 0x7F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief QDAC CALDAC write data.  Valid range for MSB CALDACs is 0 to 127.  Valid range for ISB CALDACs is 0 to 15.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalWrDataQBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCalWrDataQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalWrDataQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x12), (uint8_t)bfValue, 0x7F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief QDAC CALDAC write data.  Valid range for MSB CALDACs is 0 to 127.  Valid range for ISB CALDACs is 0 to 15.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCalWrDataQBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCalWrDataQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x12), &rxBfData[0], 0x7F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Assumed DAC Output Common Mode Level.  Needs to match BBF Common Mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCmCtrl_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCmCtrlBfSet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                              adrv9001_BfTxdacMemMapTxdacCmCtrl_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_CM_240MV               ) &&
            (bfValue != ADRV9001_BF_TXDAC_CM_270MV               ) &&
            (bfValue != ADRV9001_BF_TXDAC_CM_300MV               ) &&
            (bfValue != ADRV9001_BF_TXDAC_CM_330MV               ) &&
            (bfValue != ADRV9001_BF_TXDAC_CM_360MV               ) &&
            (bfValue != ADRV9001_BF_TXDAC_CM_390MV               ) &&
            (bfValue != ADRV9001_BF_TXDAC_CM_420MV               ) &&
            (bfValue != ADRV9001_BF_TXDAC_CM_450MV               )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCmCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCmCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x23), (uint8_t)bfValue, 0x38, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Assumed DAC Output Common Mode Level.  Needs to match BBF Common Mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCmCtrl_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCmCtrlBfGet(adi_adrv9001_Device_t *device,
                                             adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                              adrv9001_BfTxdacMemMapTxdacCmCtrl_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCmCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x23), &rxBfData[0], 0x38, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCmCtrl_e) data;

    return recoveryAction;
}

/**
 * \brief IDAC Average Value of all 32 MSB CALDAC codes
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompAvgmsbIBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompAvgmsbIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x16), &rxBfData[0], 0x7F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief QDAC Average Value of all 32 MSB CALDAC codes
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompAvgmsbQBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompAvgmsbQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x17), &rxBfData[0], 0x7F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: comparator powerdown value to use when txdac_powerdown_override==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompBiaspdBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCompBiaspdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompBiaspdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x18), (uint8_t)bfValue, 0x30, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: comparator powerdown value to use when txdac_powerdown_override==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompBiaspdBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompBiaspdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x18), &rxBfData[0], 0x30, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Disable Comparator Settling Speed up
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompFastcmEnbBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCompFastcmEnbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompFastcmEnbBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xA), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Disable Comparator Settling Speed up
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompFastcmEnbBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompFastcmEnbBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xA), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief ISB Comparator: Trade off Comparator Settling Time for Accuracy.  Larger value is higher gain/slower settling.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCompGainIsb_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompGainIsbBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                   adrv9001_BfTxdacMemMapTxdacCompGainIsb_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_COMP_GAIN_ISB_LOW      ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_GAIN_ISB_HIGH     )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCompGainIsbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompGainIsbBfSet Address");
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
 * \brief ISB Comparator: Trade off Comparator Settling Time for Accuracy.  Larger value is higher gain/slower settling.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCompGainIsb_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompGainIsbBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                   adrv9001_BfTxdacMemMapTxdacCompGainIsb_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompGainIsbBfGet Address");
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

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCompGainIsb_e) data;

    return recoveryAction;
}
/**
 * \brief MSB Comparator: Trade off Comparator Settling Time for Accuracy.  Larger value is higher gain/slower settling.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCompGainMsb_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompGainMsbBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                   adrv9001_BfTxdacMemMapTxdacCompGainMsb_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_COMP_GAIN_MSB_LOWER    ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_GAIN_MSB_LOW      ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_GAIN_MSB_HIGH     ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_GAIN_MSB_HIGHER   )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCompGainMsbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompGainMsbBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xB), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief MSB Comparator: Trade off Comparator Settling Time for Accuracy.  Larger value is higher gain/slower settling.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCompGainMsb_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompGainMsbBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                   adrv9001_BfTxdacMemMapTxdacCompGainMsb_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompGainMsbBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xB), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCompGainMsb_e) data;

    return recoveryAction;
}
/**
 * \brief Number of SAR runs to average for each ISB CALDAC measurement.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCompIsbavg_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompIsbavgBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                  adrv9001_BfTxdacMemMapTxdacCompIsbavg_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_COMP_ISBAVG_1          ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_ISBAVG_2          ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_ISBAVG_4          ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_ISBAVG_8          ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_ISBAVG_16         ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_ISBAVG_32         ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_ISBAVG_64         ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_ISBAVG_128        )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCompIsbavgBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompIsbavgBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xD), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Number of SAR runs to average for each ISB CALDAC measurement.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCompIsbavg_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompIsbavgBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                  adrv9001_BfTxdacMemMapTxdacCompIsbavg_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompIsbavgBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xD), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCompIsbavg_e) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: comparator powerdown value to use when txdac_powerdown_override==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompIsbpdBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCompIsbpdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompIsbpdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x18), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: comparator powerdown value to use when txdac_powerdown_override==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompIsbpdBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompIsbpdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x18), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Number of SAR runs to average for each MSB CALDAC measurement.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCompMsbavg_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompMsbavgBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                  adrv9001_BfTxdacMemMapTxdacCompMsbavg_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_COMP_MSBAVG_1          ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_MSBAVG_2          ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_MSBAVG_4          ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_MSBAVG_8          ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_MSBAVG_16         ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_MSBAVG_32         ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_MSBAVG_64         ) &&
            (bfValue != ADRV9001_BF_TXDAC_COMP_MSBAVG_128        )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacCompMsbavgBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompMsbavgBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xD), (uint8_t)bfValue, 0x70, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Number of SAR runs to average for each MSB CALDAC measurement.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacCompMsbavg_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompMsbavgBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                  adrv9001_BfTxdacMemMapTxdacCompMsbavg_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompMsbavgBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xD), &rxBfData[0], 0x70, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacCompMsbavg_e) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: comparator powerdown value to use when txdac_powerdown_override==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompMsbpdBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCompMsbpdBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompMsbpdBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x18), (uint8_t)bfValue, 0xC, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: comparator powerdown value to use when txdac_powerdown_override==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompMsbpdBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompMsbpdBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x18), &rxBfData[0], 0xC, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Disable Comparator Settling Speed Up
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompOneshotEnbBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacCompOneshotEnbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompOneshotEnbBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xA), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Disable Comparator Settling Speed Up
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompOneshotEnbBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompOneshotEnbBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xA), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief IDAC Comparator raw output value.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompOutIBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompOutIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x16), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief QDAC Comparator raw output value.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacCompOutQBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacCompOutQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x17), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Set amplitude of dither
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacDitherAmplitudeBfSet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacDitherAmplitudeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacDitherAmplitudeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3), (uint8_t)bfValue, 0xF8, 0x3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set amplitude of dither
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacDitherAmplitudeBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacDitherAmplitudeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3), &rxBfData[0], 0xF8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Enable DAC MSB dither
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacDitherEnableBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacDitherEnableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacDitherEnableBfSet Address");
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
 * \brief Enable DAC MSB dither
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacDitherEnableBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacDitherEnableBfGet Address");
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
 * \brief Update rate of triangle waveform used to dither the thermometer code start pointer.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacDitherPeriodBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacDitherPeriodBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacDitherPeriodBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x2), (uint8_t)(bfValue >> 8), 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field 8 bits */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x1), (uint8_t)(bfValue >> 0));
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Update rate of triangle waveform used to dither the thermometer code start pointer.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint16_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacDitherPeriodBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacDitherPeriodBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x2), &rxBfData[0x0], 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field 8 bits */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x1), &rxBfData[0x1], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);

    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x2, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint16_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Bring IDAC Currents to a Test Port.  Current selection driven by txdac_cal_debug, txdac_cal_address, and txdac_sel_cal_to_pin
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacExtcalEnableIBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacExtcalEnableIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacExtcalEnableIBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x19), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: Bring IDAC Currents to a Test Port.  Current selection driven by txdac_cal_debug, txdac_cal_address, and txdac_sel_cal_to_pin
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacExtcalEnableIBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacExtcalEnableIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x19), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Bring QDAC Currents to a Test Port.  Current selection driven by txdac_cal_debug, txdac_cal_address, and txdac_sel_cal_to_pin
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacExtcalEnableQBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacExtcalEnableQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacExtcalEnableQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x19), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: Bring QDAC Currents to a Test Port.  Current selection driven by txdac_cal_debug, txdac_cal_address, and txdac_sel_cal_to_pin
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacExtcalEnableQBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacExtcalEnableQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x19), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Boost IDAC Output Power by 3dB
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacFscBoostI_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacFscBoostIBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacFscBoostI_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_IDAC_FSC4MA            ) &&
            (bfValue != ADRV9001_BF_TXDAC_IDAC_FSC5P6MA          )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacFscBoostIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacFscBoostIBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x20), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Boost IDAC Output Power by 3dB
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacFscBoostI_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacFscBoostIBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacFscBoostI_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacFscBoostIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacFscBoostI_e) data;

    return recoveryAction;
}
/**
 * \brief Boost QDAC Output Power by 3dB
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacFscBoostQ_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacFscBoostQBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacFscBoostQ_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_QDAC_FSC4MA            ) &&
            (bfValue != ADRV9001_BF_TXDAC_QDAC_FSC5P6MA          )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacFscBoostQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacFscBoostQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x21), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Boost QDAC Output Power by 3dB
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacFscBoostQ_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacFscBoostQBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                 adrv9001_BfTxdacMemMapTxdacFscBoostQ_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacFscBoostQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x21), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacFscBoostQ_e) data;

    return recoveryAction;
}
/**
 * \brief Fine tune IDAC full scale current between [-256uA,240uA) in 16uA steps.  Offset binary format.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacFscTuneIBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacFscTuneIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacFscTuneIBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x20), (uint8_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Fine tune IDAC full scale current between [-256uA,240uA) in 16uA steps.  Offset binary format.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacFscTuneIBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacFscTuneIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x20), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Fine tune QDAC full scale current between [-256uA,240uA) in 16uA steps.  Offset binary format.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacFscTuneQBfSet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacFscTuneQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacFscTuneQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x21), (uint8_t)bfValue, 0x1F, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Fine tune QDAC full scale current between [-256uA,240uA) in 16uA steps.  Offset binary format.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacFscTuneQBfGet(adi_adrv9001_Device_t *device,
                                               adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacFscTuneQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x21), &rxBfData[0], 0x1F, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Enable random shuffle for DAC ISBs
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacIsbShuffleEnableBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacIsbShuffleEnableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacIsbShuffleEnableBfSet Address");
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
 * \brief Enable random shuffle for DAC ISBs
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacIsbShuffleEnableBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacIsbShuffleEnableBfGet Address");
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
 * \brief Enable random shuffle for DAC MSBs
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacMsbShuffleEnableBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacMsbShuffleEnableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacMsbShuffleEnableBfSet Address");
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
 * \brief Enable random shuffle for DAC MSBs
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacMsbShuffleEnableBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacMsbShuffleEnableBfGet Address");
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
 * \brief Forces DAC Output to 0.2-0.3V in case BBF is not setting common mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacOutputProtect0p2vBfSet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacOutputProtect0p2vBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacOutputProtect0p2vBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1D), (uint8_t)bfValue, 0x10, 0x4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Forces DAC Output to 0.2-0.3V in case BBF is not setting common mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacOutputProtect0p2vBfGet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacOutputProtect0p2vBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1D), &rxBfData[0], 0x10, 0xC4);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Forces DAC Output to ground in case BBF is not setting common mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacOutputProtectGndBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacOutputProtectGndBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacOutputProtectGndBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1D), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Forces DAC Output to ground in case BBF is not setting common mode
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacOutputProtectGndBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacOutputProtectGndBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1D), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Power down Driver Control when txdac_powerdown_override==1.  Needed for Comparator or DAC Manual Powerup/Powerdown
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPddriverEnableBfSet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacPddriverEnableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPddriverEnableBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1D), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: Power down Driver Control when txdac_powerdown_override==1.  Needed for Comparator or DAC Manual Powerup/Powerdown
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPddriverEnableBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPddriverEnableBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1D), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Force Masterbias to stay on
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPdMbiasOverrideBfSet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacPdMbiasOverrideBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPdMbiasOverrideBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1C), (uint8_t)bfValue, 0x20, 0x5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Force Masterbias to stay on
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPdMbiasOverrideBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPdMbiasOverrideBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0], 0x20, 0xC5);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DAC clock divide ratio for powerdown state machine clock. Set to 200kHz - 400kHz
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacPowerdownClkdiv_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownClkdivBfSet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                       adrv9001_BfTxdacMemMapTxdacPowerdownClkdiv_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_OFF              ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV17            ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV16            ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV15            ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV14            ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV13            ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV12            ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV11            ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV10            ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV9             ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV8             ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV7             ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV6             ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV6_13          ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV6_14          ) &&
            (bfValue != ADRV9001_BF_TXDAC_PDCLK_DIV6_15          )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownClkdivBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownClkdivBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1B), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DAC clock divide ratio for powerdown state machine clock. Set to 200kHz - 400kHz
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacPowerdownClkdiv_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownClkdivBfGet(adi_adrv9001_Device_t *device,
                                                      adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                       adrv9001_BfTxdacMemMapTxdacPowerdownClkdiv_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownClkdivBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1B), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacPowerdownClkdiv_e) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: dac powerdown control to use when txdac_powerdown_override==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownCtrlBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1D), (uint8_t)bfValue, 0xF, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: dac powerdown control to use when txdac_powerdown_override==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownCtrlBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1D), &rxBfData[0], 0xF, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief I DAC Powerdown Control
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownIBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownIBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownIBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1C), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief I DAC Powerdown Control
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownIBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: manual override of txdac_powerdown_i and txdac_powerdown_q signals
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownOverrideBfSet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownOverrideBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownOverrideBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1D), (uint8_t)bfValue, 0x80, 0x7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: manual override of txdac_powerdown_i and txdac_powerdown_q signals
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownOverrideBfGet(adi_adrv9001_Device_t *device,
                                                        adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownOverrideBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1D), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Q DAC powerdown Control
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownQBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownQBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownQBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1C), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Q DAC powerdown Control
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownQBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief I DAC Powerdown Status
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownStatIBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownStatIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0], 0x4, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief Q DAC Powerdown Status
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerdownStatQBfGet(adi_adrv9001_Device_t *device,
                                                     adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerdownStatQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief I DAC Powerup Status
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerupStatIBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerupStatIBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0], 0x8, 0xC3);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}

/**
 * \brief Q DAC Powerup Status
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacPowerupStatQBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacPowerupStatQBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1C), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Selects which current branch to bring out
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacSelCalToPin_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacSelCalToPinBfSet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                   adrv9001_BfTxdacMemMapTxdacSelCalToPin_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_ISBREF_TO_PIN          ) &&
            (bfValue != ADRV9001_BF_TXDAC_ISB_TO_PIN             ) &&
            (bfValue != ADRV9001_BF_TXDAC_MSBREF_TO_PIN          ) &&
            (bfValue != ADRV9001_BF_TXDAC_MSB_TO_PIN             )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacSelCalToPinBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacSelCalToPinBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x19), (uint8_t)bfValue, 0xC0, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: Selects which current branch to bring out
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacSelCalToPin_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacSelCalToPinBfGet(adi_adrv9001_Device_t *device,
                                                  adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                   adrv9001_BfTxdacMemMapTxdacSelCalToPin_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacSelCalToPinBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x19), &rxBfData[0], 0xC0, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacSelCalToPin_e) data;

    return recoveryAction;
}
/**
 * \brief Set amplitude of random shuffle
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacShuffleAmplitudeBfSet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacShuffleAmplitudeBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacShuffleAmplitudeBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x3), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Set amplitude of random shuffle
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacShuffleAmplitudeBfGet(adi_adrv9001_Device_t *device,
                                                       adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacShuffleAmplitudeBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x3), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief LOW resets the decoder shuffle state
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacShuffleResetbBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacShuffleResetbBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacShuffleResetbBfSet Address");
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
 * \brief LOW resets the decoder shuffle state
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacShuffleResetbBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacShuffleResetbBfGet Address");
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
 * \brief Currently Unassigned Bits
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacSpareCtrlBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacSpareCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacSpareCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Single Byte */
    recoveryAction = adrv9001_BfByteWrite(device, (baseAddr + 0x24), ((uint8_t)bfValue));
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Currently Unassigned Bits
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacSpareCtrlBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacSpareCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read to Single Byte */
    recoveryAction = adrv9001_BfByteRead(device, (baseAddr + 0x24), &rxBfData[0], 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: SPI driven Clock for poweron state machine when txdac_startup_mansel==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacStartupManclkBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacStartupManclkBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacStartupManclkBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1E), (uint8_t)bfValue, 0x2, 0x1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: SPI driven Clock for poweron state machine when txdac_startup_mansel==1
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacStartupManclkBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacStartupManclkBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1E), &rxBfData[0], 0x2, 0xC1);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: poweron state machine clock bypass
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacStartupMansel_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacStartupManselBfSet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                     adrv9001_BfTxdacMemMapTxdacStartupMansel_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_STARTUP_SEL_DACCLK     ) &&
            (bfValue != ADRV9001_BF_TXDAC_STARTUP_SEL_SPI        )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacStartupManselBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacStartupManselBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x1E), (uint8_t)bfValue, 0x1, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: poweron state machine clock bypass
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacStartupMansel_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacStartupManselBfGet(adi_adrv9001_Device_t *device,
                                                    adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                     adrv9001_BfTxdacMemMapTxdacStartupMansel_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacStartupManselBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1E), &rxBfData[0], 0x1, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacStartupMansel_e) data;

    return recoveryAction;
}

/**
 * \brief DAC is ready to be taken out of powerdown
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacStartupReadyBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacStartupReadyBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x1E), &rxBfData[0], 0x80, 0xC7);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Performance vs Power tradeoff
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacThrowIsbCtrl_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacThrowIsbCtrlBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                    adrv9001_BfTxdacMemMapTxdacThrowIsbCtrl_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_ISBTHROW_0UA           ) &&
            (bfValue != ADRV9001_BF_TXDAC_ISBTHROW_600UA         ) &&
            (bfValue != ADRV9001_BF_TXDAC_ISBTHROW_1200UA        ) &&
            (bfValue != ADRV9001_BF_TXDAC_ISBTHROW_1800UA        )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacThrowIsbCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacThrowIsbCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x27), (uint8_t)bfValue, 0xC, 0x2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: Performance vs Power tradeoff
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacThrowIsbCtrl_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacThrowIsbCtrlBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                    adrv9001_BfTxdacMemMapTxdacThrowIsbCtrl_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacThrowIsbCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x27), &rxBfData[0], 0xC, 0xC2);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacThrowIsbCtrl_e) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Performance vs Power tradeoff
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacThrowLsbCtrl_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacThrowLsbCtrlBfSet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                    adrv9001_BfTxdacMemMapTxdacThrowLsbCtrl_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_LSBTHROW_0UA           ) &&
            (bfValue != ADRV9001_BF_TXDAC_LSBTHROW_400UA         ) &&
            (bfValue != ADRV9001_BF_TXDAC_LSBTHROW_800UA         ) &&
            (bfValue != ADRV9001_BF_TXDAC_LSBTHROW_1200UA        )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacThrowLsbCtrlBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacThrowLsbCtrlBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x27), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: Performance vs Power tradeoff
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacThrowLsbCtrl_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacThrowLsbCtrlBfGet(adi_adrv9001_Device_t *device,
                                                   adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                    adrv9001_BfTxdacMemMapTxdacThrowLsbCtrl_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacThrowLsbCtrlBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x27), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacThrowLsbCtrl_e) data;

    return recoveryAction;
}
/**
 * \brief Use Calibrated FSC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacUseGainBfSet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacUseGainBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacUseGainBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0xA), (uint8_t)bfValue, 0x40, 0x6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Use Calibrated FSC
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacUseGainBfGet(adi_adrv9001_Device_t *device,
                                              adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacUseGainBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0xA), &rxBfData[0], 0x40, 0xC6);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief DEBUG: Set Current Source Headroom.  margin = 60mV + code*30mV
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacVdsMarginBfSet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacVdsMarginBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacVdsMarginBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x23), (uint8_t)bfValue, 0x7, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief DEBUG: Set Current Source Headroom.  margin = 60mV + code*30mV
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacVdsMarginBfGet(adi_adrv9001_Device_t *device,
                                                adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacVdsMarginBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x23), &rxBfData[0], 0x7, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (uint8_t) data;

    return recoveryAction;
}
/**
 * \brief Enable MSB start pointer to slowly increment over time.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacWalkEnableBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
                         "Invalid adrv9001_TxdacMemMapTxdacWalkEnableBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacWalkEnableBfSet Address");
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
 * \brief Enable MSB start pointer to slowly increment over time.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type uint8_t.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacWalkEnableBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
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
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacWalkEnableBfGet Address");
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
 * \brief Start pointer for the MSB thermometer code will slowly increase over time.  This sets the pointer update rate.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacWalkPeriod_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacWalkPeriodBfSet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                  adrv9001_BfTxdacMemMapTxdacWalkPeriod_e bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_VALUE_CHECK
    /* Enumeration check */
    if (
            (bfValue != ADRV9001_BF_TXDAC_WALK_PERIOD_20         ) &&
            (bfValue != ADRV9001_BF_TXDAC_WALK_PERIOD_23         ) &&
            (bfValue != ADRV9001_BF_TXDAC_WALK_PERIOD_26         ) &&
            (bfValue != ADRV9001_BF_TXDAC_WALK_PERIOD_29         )
           )
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         bfValue,
                         "Invalid adrv9001_TxdacMemMapTxdacWalkPeriodBfSet parameter");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_VALUE_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacWalkPeriodBfSet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfWriteCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Write to Field */
    recoveryAction = adrv9001_BfFieldWrite(device, (baseAddr + 0x0), (uint8_t)bfValue, 0x3, 0x0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfWriteCacheFlush(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return recoveryAction;
}

/**
 * \brief Start pointer for the MSB thermometer code will slowly increase over time.  This sets the pointer update rate.
 *
 * \dep_begin
 * \dep{device->common.devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the ADRV9001 device data structure.
 * \param baseAddr Base Address of instance to be configured.
 *  Parameter is of type ::adrv9001_BfTxdacMemMapChanAddr_e.
 * \param bfValue Data to be configured. Parameter is of type adrv9001_BfTxdacMemMapTxdacWalkPeriod_e.
 *
 * \retval ADI_COMMON_ACT_WARN_RESET_LOG Recovery action for log reset.
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check.
 * \retval ADI_COMMON_ACT_ERR_RESET_INTERFACE Recovery action for SPI reset required.
 * \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required.
 *
 */
int32_t adrv9001_TxdacMemMapTxdacWalkPeriodBfGet(adi_adrv9001_Device_t *device,
                                                 adrv9001_BfTxdacMemMapChanAddr_e baseAddr,
                                                  adrv9001_BfTxdacMemMapTxdacWalkPeriod_e *bfValue)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t rxBfData[8] = { 0 };
    uint64_t data = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

#ifdef ADRV9001_BITFIELD_ADDR_CHECK
    /* Base Address check */
    if ((baseAddr != ADRV9001_BF_TX1_DAC) &&
            (baseAddr != ADRV9001_BF_TX2_DAC))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         baseAddr,
                         "Invalid adrv9001_TxdacMemMapTxdacWalkPeriodBfGet Address");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif /* ADRV9001_BITFIELD_ADDR_CHECK */

    recoveryAction = adrv9001_BfReadCacheInit(device);
    ADI_ERROR_RETURN(device->common.error.newAction);

    /* Read Field */
    recoveryAction = adrv9001_BfFieldRead(device, (baseAddr + 0x0), &rxBfData[0], 0x3, 0xC0);
    ADI_ERROR_RETURN(device->common.error.newAction);
    recoveryAction = adrv9001_BfReadAssembleData(device, &rxBfData[0x0], 0x1, &data);
    ADI_ERROR_RETURN(device->common.error.newAction);

    *bfValue = (adrv9001_BfTxdacMemMapTxdacWalkPeriod_e) data;

    return recoveryAction;
}

/*  EOF: adrv9001_bf_txdac_mem_map.c */

#endif