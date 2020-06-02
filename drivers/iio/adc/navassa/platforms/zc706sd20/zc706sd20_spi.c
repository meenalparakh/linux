/**
 * \file
 * Analog Devices ZC706 Platform + microzed hardware abstraction layer
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Support for linux layer SPI functions
 */

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifdef __GNUC__
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#endif /* __GNUC__ */

#include <fcntl.h>
#include <stdio.h>
#include "zc706sd20_spi.h"

/**
* \brief Opens/allocates any necessary resources to communicate via SPI to a 
         particular device specified in the devHalCfg structure.
*
* This function should perform any necessary steps to open the SPI master resource
* on the BBIC to enable SPI communications to a particular SPI device.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
* \retval ADI_HAL_SPI_FAIL the device driver was not opened successfully
*/
int32_t zc706sd20_SpiOpen(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_SpiCfg_t *spiCfg = NULL;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    spiCfg = &halCfg->spiCfg;
    
    if (spiCfg == NULL)
    {
        return ADI_HAL_NULL_PTR;
    }
    
#ifdef __GNUC__
    if (spiCfg->fd <= 0)
    {
        spiCfg->fd = open(spiCfg->spidevName, O_RDWR);

        if (spiCfg->fd < 0)
        {
            return ADI_HAL_SPI_FAIL;
        }
    }
#endif /* __GNUC__ */
    return halError;
}

/**
* \brief Closes any resources open/allocated for a specific SPI device
*
* Any information needed to close the particular SPI device should be passed in
* the devHalCfg structure.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
* \retval ADI_HAL_SPI_FAIL the device driver was not closed successfully
*/
int32_t zc706sd20_SpiClose(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    
    if (halCfg->spiCfg.spiActionDisable == 0)
    {

        if (halCfg->spiCfg.fd != 0)
        {
#ifdef __GNUC__
            halError = close(halCfg->spiCfg.fd);
            if (halError < 0)
            {
                return ADI_HAL_SPI_FAIL;
            }
#endif /* __GNUC__ */

            halCfg->spiCfg.fd = 0;
        }

    }
    return halError;
}

/**
* \brief Initializes the SPI device driver mode, bits per word, and speed
*
* Any settings needed should be passed in the devHalCfg structure
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
* \retval ADI_HAL_SPI_FAIL the SPI initialization failed
*/
int32_t zc706sd20_SpiInit(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;
    uint8_t spiMode = 0;
    uint32_t bitsPerWord = 0;
    uint32_t speed = 0;
    int32_t result = 0;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    
    /* Initialize SPI driver with clockrate, SPI mode, etc */
    if (halCfg->spiCfg.spiActionDisable == 0)
    {

        bitsPerWord = 8;
        speed = halCfg->spiCfg.spiClkFreq_Hz;
        result = 0;

        /*Get Chips SPI Driver File Descriptor*/
        if (halCfg->spiCfg.fd <= 0)
        {
            return ADI_HAL_SPI_FAIL;
        }

        /*
         * spi mode
         */
        if ((halCfg->spiCfg.CPOL > 1) || (halCfg->spiCfg.CPHA > 1))
        {
            /*
            * Invalid SPI mode: CPOL and CPHA should be 1 or 0
            */
            return ADI_HAL_SPI_FAIL;
        }

        spiMode = (halCfg->spiCfg.CPOL << 1) | halCfg->spiCfg.CPHA;
        
#ifdef __GNUC__
        result = ioctl(halCfg->spiCfg.fd, SPI_IOC_WR_MODE, &spiMode);
        if (result < 0)
        {
            /*
             * Error settings requested SPI mode
             */
            return ADI_HAL_SPI_FAIL;
        }

        /*
         * bits per word
         */
        result = ioctl(halCfg->spiCfg.fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord);
        if (result < 0)
        {
            /*
             * Error setting the number of Bits per SPI word
             */
            return ADI_HAL_SPI_FAIL;
        }

        /*
         * max speed hz
         */
        result = ioctl(halCfg->spiCfg.fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (result < 0)
        {
            /*
             * Error Setting max SPI clock frequency for current chip select index.
             */
            return ADI_HAL_SPI_FAIL;
        }
#endif /* __GNUC__ */
    } /* If SPI hardware interface enabled */


    return halError;
}

/**
* \brief Write an array of 8-bit data to a SPI device
*
* The function will write numTxBytes number of bytes to the SPI device 
* selected in the devHalCfg structure.
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param txData Pointer to byte array txData buffer that has numTxBytes number of bytes
* \param numTxBytes The length of txData array
*
* \retval ADI_HAL_OK function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
* \retval ADI_HAL_SPI_FAIL the data was not written successfully
*/
int32_t zc706sd20_SpiWrite(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes)
{
    static const uint32_t MAX_SIZE = 4096;
    uint32_t toWrite = 0;
    int32_t remaining = numTxBytes;
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;

    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    halCfg = (adi_hal_Cfg_t *)devHalCfg;

    if (halCfg->spiCfg.spiActionDisable == 0)
    {
#ifdef __GNUC__
        int32_t result = 0;

        if (halCfg->spiCfg.fd > 0)
        {
            do
            {
                toWrite = (remaining > MAX_SIZE) ? MAX_SIZE : remaining;
                result = write(halCfg->spiCfg.fd, &txData[numTxBytes - remaining], toWrite);
                if (result <= 0)
                {
                    return ADI_HAL_SPI_FAIL;
                }
                remaining -= toWrite;
            } while (remaining > 0);
        }
        else
        {
            /* Invalid file descriptor */
            halError = ADI_HAL_SPI_FAIL;
        }
#endif /* __GNUC__ */
    }

    return halError;
}

/**
* \brief Read one or more bytes from the device specified by the devHalCfg structure
*
* The function will read numTxRxBytes number of bytes from the SPI device selected in
* the devHalCfg parameter and store the resulting data sent by the device in the rxData
* data buffer.
*
* For each byte in txData written to the device, a byte is read and returned by this 
* function at the pointer provided by the rxData parameter.
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param txData Pointer to byte array that has numTxRxBytes number of bytes
* \param rxData Pointer to byte array where read back data will be returned, that is at least numTxRxBytes in size.
* \param numTxBytes The length of txData and rxData arrays
*
* \retval ADI_HAL_OK function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
* \retval ADI_HAL_SPI_FAIL the data was not read successfully
*/
int32_t zc706sd20_SpiRead(void *devHalCfg, const uint8_t txData[], uint8_t rxData[],
                     uint32_t numTxRxBytes)
{
    static const uint32_t MAX_SIZE = 4096;
    int32_t remaining = numTxRxBytes;
    uint32_t offset = 0;
    int32_t halError = (int32_t)ADI_HAL_OK;
    int32_t result = 0;
    adi_hal_Cfg_t *halCfg = NULL;
    struct spi_ioc_transfer tr = { 0 };
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }
    
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    
    if (halCfg->spiCfg.fd <= 0)
    {
        /* Invalid file descriptor */
        return ADI_HAL_SPI_FAIL;
    }

    if(halCfg->spiCfg.spiActionDisable == 0)
    {
#ifdef __GNUC__

        do
        {
            offset = numTxRxBytes - remaining;
            
            tr.tx_buf = (uintptr_t)&txData[offset];
            tr.rx_buf = (uintptr_t)&rxData[offset];
            tr.len = (remaining > MAX_SIZE) ? MAX_SIZE : remaining;
            tr.delay_usecs = 0;

            result = ioctl(halCfg->spiCfg.fd, SPI_IOC_MESSAGE(1), &tr);
            if (result <= 0)
            {
                halError = ADI_HAL_SPI_FAIL;
            }
            
            remaining -= tr.len;
        } while (remaining > 0);

#endif /* __GNUC__ */
    }

    return halError;
}
