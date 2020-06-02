/**
* \file adrv9001_bf_hal.c
* \brief Contains ADI Transceiver Hardware Abstraction functions
*        Analog Devices maintains and provides updates to this code layer.
*        The end user should not modify this file or any code in this directory.
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_common_hal.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_error.h"
#include "adrv9001_bf_hal.h"



int32_t adrv9001_BfWriteCacheEn(adi_adrv9001_Device_t *device,
                                adrv9001_Bf_WrCache_State_e wrCacheState)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);
    if (wrCacheState > ADRV9001_WR_HW_RMW_ON)
    {

        if (device->common.cacheInfo.HW_RMW_Enabled == false && device->common.cacheInfo.wrByteOnly == false)
        {
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_DEVICEBF, ADRV9001_BF_HW_RMW_NOT_INITIALISED, ADI_COMMON_ACT_ERR_RESET_INTERFACE, NULL, "Caching not available");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    device->common.cacheInfo.wrCacheState = wrCacheState;
    return recoveryAction;
}

int32_t adrv9001_BfReadCacheEn(adi_adrv9001_Device_t *device,
                               adrv9001_Bf_RdCache_State_e rdCacheState)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

    device->common.cacheInfo.rdCacheState = rdCacheState;
    return recoveryAction;
}


int32_t adrv9001_BfReadCacheInit(adi_adrv9001_Device_t *device)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint16_t numRdBytes = 0;
    uint64_t rdData = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

    if (device->common.cacheInfo.rdCacheState == ADRV9001_RD_BF_CACHE_ON)
    {
        recoveryAction = adrv9001_BfReadCacheFlush(device, &rdData, &numRdBytes);
        ADI_ERROR_RETURN(device->common.error.newAction);
        /* if Global Caching is being used, no further flushing should be performed
        *  utill the user calls bf flush*/
    }
    return recoveryAction;
}

int32_t adrv9001_BfWriteCacheInit(adi_adrv9001_Device_t *device)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

    if (device->common.cacheInfo.wrCacheState > ADRV9001_WR_HW_RMW_ON)
    {

        if (device->common.cacheInfo.HW_RMW_Enabled == false && device->common.cacheInfo.wrByteOnly == false)
        {
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_DEVICEBF, ADRV9001_BF_HW_RMW_NOT_INITIALISED, ADI_COMMON_ACT_ERR_RESET_INTERFACE, NULL, "Caching not available");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }


    /* skip cacheInit if Global level Cache is on.*/
    if (device->common.cacheInfo.wrCacheState == ADRV9001_WR_BF_CACHE_ON)
    {
        /*Ensures the cache is empty if bitField Level Caching is on*/

        recoveryAction = adrv9001_BfWriteCacheFlush(device);
        ADI_ERROR_RETURN(device->common.error.newAction);
        /* if using bitField level caching reenable caching after each flush*/
        device->common.cacheInfo.wrFlushEn = true;
    }
    else
    {
        /* disable flushing within the bitfield function when global
           caching is being used*/
        device->common.cacheInfo.wrFlushEn = false;
    }

    return recoveryAction;
}


int32_t adrv9001_BfByteWrite(adi_adrv9001_Device_t *device,
                             uint16_t addr,
                             uint8_t regVal)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    int32_t tmp_wrCacheIdx = 0;
    uint32_t i = 0;
    uint8_t addrFoundInCache = false;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

   //TODO add array size check

    if (device->common.cacheInfo.wrCacheState == ADRV9001_WR_GBL_MERGE_ON)
    {
        /* Write merging is on, so the cache is searched to see if address is in the cache*/
        for (i = 0; i < device->common.cacheInfo.wrCacheIdx; i++)
        {
            /* The cache is packed in the form AABBMMDD
            where AA is MSB Addr, BB is the LSB addr,
            MM is the mask and DD is the Data */
            //TODO fix this
            //if the address part of the cache word = wanted address
            if ((uint16_t)(device->common.cacheInfo.wrCache[i] >> SPI_ADDR_SIZE) == addr)
            {
                //TODO: Warn of overwriteing data
                //TODO: add warning as error in Error Handler.
                addrFoundInCache = true;
                ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);
                /* preserve the address but clear the data. */

                //TODO: make a test to check this
                device->common.cacheInfo.wrCache[i] &= 0xFFFFFF00;
                device->common.cacheInfo.wrCache[i] |= (((uint32_t)0xFF << SPI_MASK_SIZE) | regVal);
            }
        }
    }

    if ((device->common.cacheInfo.wrCacheState != ADRV9001_WR_CACHE_OFF) &&
        (addrFoundInCache == false))
    {
        /* Write caching is turned on and the bitField has not been merged*/
        tmp_wrCacheIdx = device->common.cacheInfo.wrCacheIdx;
        if ((tmp_wrCacheIdx++) >= ADRV9001_BF_WRCACHE_ARRAY_TRIP_SIZE)
        {
            /* if cache is full then flush cache and add next instruction.
            should not be a problem flushing a write cache.
            */
            device->common.cacheInfo.wrFlushEn = true;
            recoveryAction = adrv9001_BfWriteCacheFlush(device);
            ADI_ERROR_RETURN(device->common.error.newAction);
            device->common.cacheInfo.wrFlushEn = false;
        }

        device->common.cacheInfo.wrCache[device->common.cacheInfo.wrCacheIdx++] = ((uint32_t)addr << SPI_ADDR_SIZE) | 0xFF << SPI_MASK_SIZE | regVal;
    }


    if (device->common.cacheInfo.wrCacheState == ADRV9001_WR_CACHE_OFF)
    {
        /* Write caching is not being used, so do an immediate write */
#if ADI_ADRV9001_DEVICE_NOT_CONNECTED
        recoveryAction = adi_adrv9001_spi_Byte_Write(device, addr, regVal);
        ADI_ERROR_RETURN(device->common.error.newAction);
#else
        if (device->common.devHalInfo != NULL)
        {
            recoveryAction = adi_adrv9001_spi_Byte_Write(device, addr, regVal);
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
#endif
    }
    return recoveryAction;
}


int32_t adrv9001_BfByteRead(adi_adrv9001_Device_t *device,
                            uint16_t addr,
                            uint8_t *regVal,
                            uint8_t endOfBf)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

    ADI_NULL_PTR_RETURN(&device->common, regVal);

    if (device->common.cacheInfo.rdCacheState > ADRV9001_RD_CACHE_OFF)
    {
        /* Read Caching is turned on */
        int tmp_rdCacheIdx = device->common.cacheInfo.rdCacheIdx;
        if ((tmp_rdCacheIdx++) >= ADRV9001_BF_RDCACHE_ARRAY_TRIP_SIZE)
        {
            /* if the cache is full then raise an error*/
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_DEVICEBF,
                         ADRV9001_BF_CACHE_OVERFLOW,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         NULL,
                         "Rx Cache size exceeded error");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        /* set bits [31:16] of the cache to the 16 bit address
           set bits [15:8] of cache to the 8 bit mask address
           set bits [7:0] of cache to the 8 bit endOfBf.
           endOfBf contains two 4 bit fields.
           [7:4] is 0xF or 0x0 and masks a end of bitField data value.
           [3:0] is the register startBit ie where the value resides in the register.
           */
        device->common.cacheInfo.rdCache[device->common.cacheInfo.rdCacheIdx++] = ((uint32_t)addr << SPI_ADDR_SIZE) |
                                                                    (0xFF << SPI_DATA_SIZE) |
                                                                    endOfBf;
    }
    else
    {
        recoveryAction = adi_adrv9001_spi_Byte_Read(device, addr, regVal);
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    return recoveryAction;
}

int32_t adrv9001_BfFieldWrite(adi_adrv9001_Device_t *device,
                              uint16_t addr,
                              uint8_t fieldVal,
                              uint8_t mask,
                              uint8_t startBit)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t tmp_wrCacheIdx = 0;
    uint32_t i = 0;
    uint8_t regVal = ((fieldVal << startBit) & mask);
    uint8_t addrFoundInCache = 0;

    uint8_t cachemask = 0;
    uint8_t newMask = 0;
    uint8_t cacheVal = 0;
    uint8_t newVal = 0;

    uint8_t registerVal = 0;
    uint8_t newRegVal = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);


    if (device->common.cacheInfo.wrCacheState > ADRV9001_WR_HW_RMW_ON)
    {
        if (device->common.cacheInfo.wrByteOnly == true)
        {
            mask = 0xFF; //ensures cached field is writen as a byte.
        }
        /* if caching is turned on*/
        if (device->common.cacheInfo.wrCacheState == ADRV9001_WR_GBL_MERGE_ON)
        {
            /* if merging of multiple register writes with the same address is turned on*/
            for (i = 0; i < device->common.cacheInfo.wrCacheIdx; i++)
            {
                /* search the cache for addr in the cache
                *  if found merge field val and the mask into the original values*/
                if ((uint16_t)(device->common.cacheInfo.wrCache[i] >> SPI_ADDR_SIZE) == addr)
                {
                    addrFoundInCache = true;
                    cachemask = (uint8_t)(device->common.cacheInfo.wrCache[i] >> SPI_DATA_SIZE);
                    newMask = cachemask | mask;
                    cacheVal = (uint8_t)(device->common.cacheInfo.wrCache[i]);
                    newVal = cacheVal | regVal;

                    device->common.cacheInfo.wrCache[i] &= 0xFFFF0000;     /* clear org mask and data. */
                    device->common.cacheInfo.wrCache[i] |= (uint32_t)(newMask << 8) | newVal;
                }
            }
        }
        if ((device->common.cacheInfo.wrCacheState > ADRV9001_WR_CACHE_OFF) & !addrFoundInCache)
        {
            /* if caching is on or
            * merging is on but addr was not found in the cache
            * add the wr instruction to the cache. */
            tmp_wrCacheIdx = device->common.cacheInfo.wrCacheIdx;
            if ((tmp_wrCacheIdx++) >= ADRV9001_BF_WRCACHE_ARRAY_TRIP_SIZE)
            {
                recoveryAction = adrv9001_BfWriteCacheFlush(device);
                ADI_ERROR_RETURN(device->common.error.newAction);
            }


            device->common.cacheInfo.wrCache[device->common.cacheInfo.wrCacheIdx++] = ((uint32_t)addr << SPI_ADDR_SIZE) |
                                                                        ((uint32_t)mask << 8) |
                                                                        regVal;
        }
    }
    else
    {
        /* if caching is off */
        if (device->common.cacheInfo.wrByteOnly)
        {
            /* immediate field write as if it were a byte*/
            
            recoveryAction = adi_adrv9001_spi_Byte_Write(device, addr, regVal);
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        else
        {

        /* Immediate Read modify write using HW_RMW function */
            if (device->common.cacheInfo.wrCacheState == ADRV9001_WR_HW_RMW_ON &&
                 (device->common.cacheInfo.HW_RMW_Enabled > 0))
            {
                device->common.cacheInfo.wrCache[device->common.cacheInfo.wrCacheIdx++] = ((uint32_t)addr << SPI_ADDR_SIZE) |
                                                                            ((uint32_t)mask << 8) |
                                                                            regVal;

                recoveryAction = adi_adrv9001_spi_Cache_Write(device, device->common.cacheInfo.wrCache, device->common.cacheInfo.wrCacheIdx);
                ADI_ERROR_RETURN(device->common.error.newAction);
            }
            else
            {
                /* Immediate Standard read Modify Write using seperate read
                   and writespi transactions. */
                recoveryAction = adi_adrv9001_spi_Byte_Read(device, addr, &registerVal);
                ADI_ERROR_RETURN(device->common.error.newAction);

                            /* modify the register value*/
                newRegVal = (registerVal & ~mask) | regVal;

                            /* write to the register with modified value*/
                recoveryAction = adi_adrv9001_spi_Byte_Write(device, addr, newRegVal);
                ADI_ERROR_RETURN(device->common.error.newAction);
            }
        }
    }
    return recoveryAction;
}


int32_t adrv9001_BfFieldRead(adi_adrv9001_Device_t *device,
                             uint16_t addr,
                             uint8_t *fieldVal,
                             uint8_t mask,
                             uint8_t startBit)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

    ADI_NULL_PTR_RETURN(&device->common, fieldVal);

    if (device->common.cacheInfo.rdCacheState > ADRV9001_RD_CACHE_OFF)
    {
        //adds spi call to cache
        int rdCacheIdx = device->common.cacheInfo.rdCacheIdx;
        if ((rdCacheIdx++) >= ADRV9001_BF_RDCACHE_ARRAY_TRIP_SIZE)
        {
            ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_DEVICEBF,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         NULL,
                         "Rx Cache size exceeded error");

            ADI_ERROR_RETURN(device->common.error.newAction);
        }
        device->common.cacheInfo.rdCache[device->common.cacheInfo.rdCacheIdx++] = ((uint32_t)addr << SPI_ADDR_SIZE) |
                                                                    ((uint32_t)mask << SPI_MASK_SIZE) |
                                                                    startBit;
    }
    else
    {
        /* Implements immediate spi transaction call only used when caching is turned off. */
        recoveryAction = adi_adrv9001_spi_Field_Read(device, addr, fieldVal, mask, startBit);
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    return recoveryAction;
}

int32_t adrv9001_BfReadAssembleData(adi_adrv9001_Device_t *device,
                                    uint8_t *rxBfData,
                                    uint8_t numRxBfData,
                                    uint64_t *rxData)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t i = 0;
    uint16_t numRxBitFields = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

    ADI_NULL_PTR_RETURN(&device->common, rxBfData);
    ADI_NULL_PTR_RETURN(&device->common, rxData);

    if (device->common.cacheInfo.rdCacheState == ADRV9001_RD_BF_CACHE_ON)
    {
        recoveryAction = adrv9001_BfReadCacheFlush(device, rxData, &numRxBitFields);
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    else if (device->common.cacheInfo.rdCacheState == ADRV9001_RD_CACHE_OFF)
    {
        /* Caching is not used so then assembly the bitFiled register values
           into a bitField Value. The max bitField Value is 64bits long.
           so rdData could be shifted 8 times to concatenate a 64 bit value.*/
        for (i = 0; i < numRxBfData; i++)
        {
            /* shift rxData to the left to make room for the next lower byte
               in the bitField*/
            *rxData = (*rxData << 8) | (uint64_t)rxBfData[i];
        }
    }
    return recoveryAction;
}


int32_t adrv9001_BfReadCacheFlush(adi_adrv9001_Device_t *device,
                                  uint64_t *rdData,
                                  uint16_t *numRdBitFields)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint32_t bfValue = 0;
    uint32_t i = 0;
    uint8_t regVal = 0;
    uint8_t mask = 0;
    uint8_t startBit = 0;
    uint8_t endOfBitfield = 0;

    uint8_t regData[ADRV9001_BF_RDCACHE_ARRAY_SIZE * 3] = { 0 };

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

    ADI_NULL_PTR_RETURN(&device->common, rdData);
    ADI_NULL_PTR_RETURN(&device->common, numRdBitFields);

    *numRdBitFields = 0;

    if (device->common.cacheInfo.rdCacheIdx > ADRV9001_BF_RDCACHE_ARRAY_TRIP_SIZE)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_DEVICEBF, ADRV9001_BF_CACHE_OVERFLOW,
                     ADI_ADRV9001_ACT_ERR_FLUSH_CACHE,
                     NULL,
                     "Rd Cache index exceeded the spi cache Size ");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (device->common.cacheInfo.rdCacheIdx > 0)
    {
        recoveryAction = adi_adrv9001_spi_Cache_Read(device, device->common.cacheInfo.rdCache, &regData[0], device->common.cacheInfo.rdCacheIdx);
        ADI_ERROR_RETURN(device->common.error.newAction);

            /* depack the array of register values into an array of bitField values*/
        for (i = 0; i < device->common.cacheInfo.rdCacheIdx; i++)
        {
            /* for each of the register values in the cache (if any)
               combine the data into a bitfield value*/

        mask = (uint8_t)(device->common.cacheInfo.rdCache[i] >> 8);
        startBit = (uint8_t)(device->common.cacheInfo.rdCache[i]);

        /*The last byte in the bit field is marked by setting the upper
        nible of the startBit to F, this allows the lower nible to
        handle the byte with values from 0 to 7.
        This marker is added within the get bitfield function.
        */
        endOfBitfield = startBit > 0xC;

        if (endOfBitfield)
        {
            startBit = startBit & 0x3F;
        }
        regVal = regData[((3 * i) + 2)];

            bfValue = ((uint32_t)bfValue << 8) | ((regVal & mask) >> startBit);

            device->common.cacheInfo.rdCache[i] = '\0';

            if (endOfBitfield)
            {
                rdData[(*numRdBitFields)++] = bfValue;
                bfValue = 0;
            }
        }
    }
    device->common.cacheInfo.rdCacheIdx = 0;
    device->common.cacheInfo.rdCache[device->common.cacheInfo.rdCacheIdx] = '\0';
    return recoveryAction;
}

int32_t adrv9001_BfWriteCacheFlush(adi_adrv9001_Device_t *device)
{
    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_BF);

    if (device->common.cacheInfo.wrCacheIdx > ADRV9001_BF_WRCACHE_ARRAY_SIZE)
    {
        ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_DEVICEBF,
                     ADRV9001_BF_CACHE_OVERFLOW,
                     ADI_ADRV9001_ACT_ERR_FLUSH_CACHE,
                     NULL,
                     "Wr Cache index exceeded the spi cache Size ");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    /* only call flush if there is somthing in the cache */
    if ((device->common.cacheInfo.wrFlushEn == true) & (device->common.cacheInfo.wrCacheIdx > 0))
    {
        recoveryAction = adi_adrv9001_spi_Cache_Write(device, device->common.cacheInfo.wrCache,
                                                        device->common.cacheInfo.wrCacheIdx);
        ADI_ERROR_RETURN(device->common.error.newAction);

        device->common.cacheInfo.wrCacheIdx = 0;
        device->common.cacheInfo.wrCache[device->common.cacheInfo.wrCacheIdx] = '\0';
    }
    device->common.cacheInfo.wrFlushEn = true;

    return recoveryAction;
}
