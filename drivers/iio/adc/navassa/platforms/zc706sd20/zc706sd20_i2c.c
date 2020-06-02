/**
 * \file
 * \brief Support for linux layer I2C functions
 */

/**
* Copyright 2018 - 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifdef __GNUC__
#include <unistd.h>
#endif /* __GNUC__ */

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "zc706sd20_i2c.h"

/**
* \brief Function to open/allocate any necessary resources for the I2C 
*        functions below.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cOpen(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_I2cCfg_t *i2cCfg = NULL;

    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    i2cCfg = &halCfg->i2cCfg;
    
    if (i2cCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
    
#ifdef __GNUC__
    if (i2cCfg->fd <= 0)
    {
        i2cCfg->fd = open(i2cCfg->drvName, O_RDWR);

        if (i2cCfg->fd < 0)
        {
            return (int32_t)ADI_HAL_I2C_FAIL;
        }
    }
#endif /* __GNUC__ */
    return halError;
}

/**
* \brief Function to close/deallocate any necessary resources for the I2C 
*        functions below.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cClose(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_I2cCfg_t *i2cCfg = NULL;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    i2cCfg = &halCfg->i2cCfg;
    
    if (i2cCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
    
#ifdef __GNUC__
    if (i2cCfg->fd != 0)
    {
        halError = close(i2cCfg->fd);
        if (halError < 0)
        {
            return (int32_t)ADI_HAL_I2C_FAIL;
        }
        
        i2cCfg->fd = 0;
    }
#endif /* __GNUC__ */

    return halError;
}

/**
* \brief Function to init any necessary resources for the I2C 
*        functions below.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cInit(void *devHalCfg)
{
    return (int32_t)ADI_HAL_OK;
}

/**
* \brief Function to write to an I2C device from the BBIC
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param txData Byte array of data to write to the I2C device.  First byte 
*               should be the I2C register address followed by one or more data bytes.
* \param numTxBytes Number of bytes in the txData array
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cWrite(void *devHalCfg, uint8_t slaveAddress, const uint8_t txData[], uint32_t numTxBytes)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    int retVal = 0;
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_I2cCfg_t *i2cCfg = NULL;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    i2cCfg = &halCfg->i2cCfg;
    if (i2cCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }

    if ((numTxBytes > 0) &&
        (txData != NULL))
    {
        if (i2cCfg->fd > 0)
        {
#ifdef __GNUC__
            if (ioctl(i2cCfg->fd, I2C_SLAVE, slaveAddress) < 0)
            {
                return (int32_t)ADI_HAL_I2C_FAIL;
            }
            
            retVal = write(i2cCfg->fd, &txData[0], numTxBytes);
            if (retVal != (int)numTxBytes)
            {
                perror("I2C : Failed to Write to device");
                return (int32_t)ADI_HAL_I2C_FAIL;
            }
#endif
        }
        else
        {
            perror("I2C : Failed to Write to device, file descriptor invalid");
            return (int32_t)ADI_HAL_GEN_SW;
        }
    }
    
    return halError;
}

/**
* \brief Function to read from an I2C device to the BBIC
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param txData Byte array of data to write to the I2C device. Depending on the
*               I2C device, this might just be 1 byte containing the register 
*               address to read
* \param numTxBytes Number of bytes in the txData array
* \param rxData Byte array to return the read back data
* \param numRxBytes Number of bytes to read back, and size of rxData array
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_I2cRead(void *devHalCfg, 
                          uint8_t slaveAddress,
                          const uint8_t wrData[],
                          uint32_t numWrBytes,
                          uint8_t rdData[],
                          uint32_t numRdBytes)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_I2cCfg_t *i2cCfg = NULL;
    
    struct i2c_msg msgs[2] = {
    {
        .addr = slaveAddress,
        .flags = 0,
        .len = numWrBytes,
        .buf = (uint8_t *)wrData
    },
    {
        .addr = slaveAddress,
        .flags = I2C_M_RD,
        .len = numRdBytes,
        .buf = rdData
    }
    };
    struct i2c_rdwr_ioctl_data msgset = {
        .msgs = msgs,
        .nmsgs = 2
    };
    
    if (devHalCfg == NULL)
    {
        return ADI_HAL_NULL_PTR;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    i2cCfg = &halCfg->i2cCfg;
    
    if (i2cCfg == NULL)
    {
        return ADI_HAL_NULL_PTR;
    }

    if (i2cCfg->fd > 0)
    {
        /* Write Register address */
        if ((numWrBytes > 0) && 
            (wrData != NULL))
        {
            if ((numRdBytes > 0) &&
                (rdData != NULL))
            {
                
                ioctl(i2cCfg->fd, I2C_RDWR, &msgset);
                rdData = msgset.msgs[1].buf;
            }
        }
        else
        {
            printf("Invalid parameter rdData. Can't be NULL");
            return ADI_HAL_I2C_FAIL;
        }
    }
    else
    {
        printf("I2C : Failed to Write to device, file descriptor invalid\n");
        return ADI_HAL_GEN_SW;
    }
        
    
    return halError;
}
