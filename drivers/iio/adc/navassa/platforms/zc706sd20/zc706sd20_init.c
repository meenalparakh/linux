/**
* \file
* \brief Contains ADI Transceiver Hardware Abstraction functions for hardware mode.
*        Analog Devices maintains and provides updates to this code layer for the ZC706 platform.
*        The end user should not modify this file or any code in this directory. The end user
*        may provide a similar platform layer that can be used in place of this platform layer, 
*        that uses the same function prototypes.
*/

/**
* \Page Disclaimer Legal Disclaimer
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "zc706sd20_init.h"
#include "zc706sd20_spi.h"
#include "zc706sd20_logging.h"
#include "zc706sd20_timer.h"
#include "zc706sd20_bbic_control.h"
#include "zc706sd20_i2c.h"
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#ifdef __GNUC__
#include <unistd.h>
#endif /* __GNUC__ */

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>

typedef unsigned char   u8;

/**
* \brief Opens all neccessary files and device drivers for a specific device
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
* \retval errors returned by other function calls.
*/
int32_t zc706sd20_HwOpen(void *devHalCfg)
{
    adi_hal_Cfg_t *halCfg = NULL;
    int32_t halError = (int32_t)ADI_HAL_OK;
    char logMessage[ADI_HAL_MAX_LOG_LINE] = { 0 };
    int32_t result = 0;
#ifdef __GNUC__
    va_list empty_va_list; 
#else
    va_list empty_va_list = { 0 };
#endif

    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    halCfg = (adi_hal_Cfg_t *)devHalCfg;

    if (halCfg->logCfg.interfaceEnabled > 0)
    {
        if (&halCfg->logCfg.logFileName[0] == '\0')
        {
            halError = zc706sd20_LogFileOpen(devHalCfg, "zc706sd20_adihal.log");
        }
        else
        {
            halError = zc706sd20_LogFileOpen(devHalCfg, NULL);
        }

        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
   
        result = snprintf(logMessage + strlen(logMessage), ADI_HAL_MAX_LOG_LINE, "zc706sd20_LogFileOpen: device index %d\n", halCfg->spiCfg.chipSelectIndex);
        if (result < 0)
        {
            return (int32_t)ADI_HAL_LOGGING_FAIL;
        }
        
        halError = zc706sd20_LogWrite(devHalCfg, ADI_HAL_LOG_MSG, logMessage, empty_va_list);
        
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
    }

    if (halCfg->spiCfg.interfaceEnabled > 0)
    {
        halError = zc706sd20_SpiOpen(devHalCfg);
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
        
        halError = zc706sd20_SpiInit(devHalCfg);
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
    }
    
    if (halCfg->bbicCfg.interfaceEnabled > 0)
    {
        halError = zc706sd20_BbicOpen(devHalCfg);
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
        
        halError = zc706sd20_BbicInit(devHalCfg);
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
    }
    
    if (halCfg->hwResetCfg.interfaceEnabled > 0)
    {
        /* ZC706 has nothing to open, as this interface is supported by the 
         * BBIC control interface */
        
        /* Verify BBIC control interface was enabled */
        if (halCfg->bbicCfg.interfaceEnabled == 0)
        {
            halError = ADI_HAL_GEN_SW;
            return halError;
        }
    }


    if (halCfg->i2cCfg.interfaceEnabled > 0)
    {

        // Open i2C Bus and initialize
        halError = zc706sd20_I2cOpen(devHalCfg);
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
        halError = zc706sd20_I2cInit(devHalCfg);
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
    }
    
    if (halCfg->timerCfg.interfaceEnabled > 0)
    {
        halError = zc706sd20_TimerOpen(devHalCfg);
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
        
        halError = zc706sd20_TimerInit(devHalCfg);
        if (halError != (int32_t)ADI_HAL_OK)
        {
            return halError;
        }
    }
    
    /* ZC706 does not require any code to open /init timer support since using linux nanosleep function */
    
    return halError;
}

/**
* \brief Gracefully shuts down the the hardware closing any open resources
*        such as log files, I2C, SPI, GPIO drivers, timer resources, etc.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
*/
int32_t zc706sd20_HwClose(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }
    
    halError = zc706sd20_LogLevelSet(devHalCfg, ADI_HAL_LOG_NONE);
    if (halError != (int32_t)ADI_HAL_OK)
    {
        return halError;
    }

    halError = zc706sd20_LogFileClose(devHalCfg);
    if (halError != (int32_t)ADI_HAL_OK)
    {
        return halError;
    }

    halError = zc706sd20_SpiClose(devHalCfg);
    if (halError != (int32_t)ADI_HAL_OK)
    {
        return halError;
    }

    halError = zc706sd20_BbicClose(devHalCfg);
    if (halError != (int32_t)ADI_HAL_OK)
    {
        return halError;
    }
    
    halError = zc706sd20_I2cClose(devHalCfg);
    if (halError != (int32_t)ADI_HAL_OK)
    {
        return halError;
    }
    
    halError = zc706sd20_TimerClose(devHalCfg);
    if (halError != (int32_t)ADI_HAL_OK)
    {
        return halError;
    }

    return halError;
}

/**
* \brief This function control a BBIC GPIO pin that connects to the reset pin
*        of each device.
*
*  This function is called by each device API giving access to the Reset pin
*  connected to each device.
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param pinLevel The desired pin logic level 0=low, 1=high to set the GPIO pin to.
*
* \retval ADI_HAL_OK Function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
*/
int32_t zc706sd20_HwReset(void *devHalCfg, uint8_t pinLevel)
{
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_HwResetCfg_t *hwResetCfg = NULL;
    int32_t halError = (int32_t)ADI_HAL_OK;
    int32_t retVal = 0;
    uint32_t readData = 0;
    uint32_t writeData = 0;
    uint32_t pinLevelBit = 0;
    uint32_t resetPinBitPostion = 0;
    static const uint32_t FPGA_RESETPIN_REGISTER = 0x43050020;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }
    
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    hwResetCfg = &halCfg->hwResetCfg;
    
    if (hwResetCfg == NULL)
    {
        return ADI_HAL_NULL_PTR;
    }
    
#ifdef __GNUC__

    if (hwResetCfg->interfaceEnabled > 0)
    {
        pinLevelBit = pinLevel;
        resetPinBitPostion = hwResetCfg->resetPinIndex;
        
        /* Perform read/modify/write to change the bit of interest in the FPGA
         * register that controls the reset pins to each device */
        retVal = zc706sd20_BbicRegisterRead(devHalCfg, FPGA_RESETPIN_REGISTER, &readData);
        if (retVal != ADI_HAL_OK)
        {
            return ADI_HAL_GPIO_FAIL;
        }
        
        writeData = readData & ~(1UL << resetPinBitPostion);
        writeData |= (pinLevelBit << resetPinBitPostion);
        
        retVal = zc706sd20_BbicRegisterWrite(devHalCfg, FPGA_RESETPIN_REGISTER, writeData);
        if (retVal != ADI_HAL_OK)
        {
            return ADI_HAL_GPIO_FAIL;
        }
    }

#endif /* __GNUC__ */

    return halError;
}

/**
* \brief This function is a helper function to fill out the members of the 
*         devHalCfg structure with driver names for this specific (ZC706) platform.
*
* This function allows keeping the platform specific knowledge of driver names
* in this layer of code.  Higher layers of code can call this to fill out the
* devHalCfg structure correctly for the ZC706 platform.
*
* Each device gets a unique instance of the devHalCfg structure describing the
* device resources for that device, such as the SPI chip select, etc.
*
* \param interfaceMask A mask that represents which platform interfaces are required by a particular device (SPI, Logging, etc)
* \param spiChipSelect The chip select index describing the particular device this structure is for (0, 1, 2 valid on ZC706)
* \param logFilename The filename of the logfile for this device.
*/
void* zc706sd20_DevHalCfgCreate(uint32_t interfaceMask, uint8_t spiChipSelect, const char *logFilename)
{
    adi_hal_Cfg_t *halCfg = (adi_hal_Cfg_t*)calloc(1, sizeof(adi_hal_Cfg_t));
    
    if (halCfg == NULL)
    {
        return NULL;
    }

    /* Disable all interfaces, and enable only if they are needed by the specific device */
    halCfg->spiCfg.interfaceEnabled = 0;
    halCfg->logCfg.interfaceEnabled = 0;
    halCfg->bbicCfg.interfaceEnabled = 0;
    
    /* If device requires SPI interface */
    if ((interfaceMask & (uint32_t)ADI_HAL_INTERFACE_SPI) > 0)
    {
        halCfg->spiCfg.interfaceEnabled = 1;
        halCfg->spiCfg.chipSelectIndex = spiChipSelect;
        
        switch (spiChipSelect)
        {
        case 0: 
            {
                strncpy(halCfg->spiCfg.spidevName, "/dev/spidev12345.0\0", ADI_HAL_STRING_LENGTH);
                break;
            }
        case 1:
            {
                strncpy(halCfg->spiCfg.spidevName, "/dev/spidev12345.1\0", ADI_HAL_STRING_LENGTH);
                break;
            }
        case 2:
            {
                strncpy(halCfg->spiCfg.spidevName, "/dev/spidev12345.2\0", ADI_HAL_STRING_LENGTH);
                break;
            }
        default:
            {
                free(halCfg);
                return NULL;
            }
        }
    
        /* Default SPI settings */
        halCfg->spiCfg.autoIncAddrUp = 1;
        halCfg->spiCfg.CPHA = 0;
        halCfg->spiCfg.CPOL = 0;
        halCfg->spiCfg.enSpiStreaming = 0;
        halCfg->spiCfg.fourWireMode = 1;
        halCfg->spiCfg.spiActionDisable = 0;
        halCfg->spiCfg.spiClkFreq_Hz = 25000000;
    }
    
    /* If device requires Logging interface */
    if ((interfaceMask & (uint32_t)ADI_HAL_INTERFACE_LOG) > 0)
    {
        halCfg->logCfg.interfaceEnabled = 1;
        halCfg->logCfg.logLevel = ADI_HAL_LOG_ALL;
        strncpy(halCfg->logCfg.logFileName, logFilename, ADI_HAL_STRING_LENGTH);
        /* ensure null termination */
        halCfg->logCfg.logFileName[sizeof(halCfg->logCfg.logFileName)-1] = '\0';
    }
    
    /* If device requires FPGA register interface */
    if (((interfaceMask & (uint32_t)ADI_HAL_INTERFACE_BBICCTRL) > 0) ||
        ((interfaceMask & (uint32_t)ADI_HAL_INTERFACE_HWRESET) > 0))  /* ZC706 HWRESET interface requires BBICCTRL interface */
    {
        halCfg->bbicCfg.interfaceEnabled = 1;
        strncpy(halCfg->bbicCfg.coreRegsCfg.drvName, "/dev/uio0\0", ADI_HAL_STRING_LENGTH);
        halCfg->bbicCfg.coreRegsCfg.mapSize = 0x1000000; /* 16MB */
        strncpy(halCfg->bbicCfg.ramCfg.drvName, "/dev/uio1\0", ADI_HAL_STRING_LENGTH);
        halCfg->bbicCfg.ramCfg.mapSize = 0x40000000; /* 1GB */
        
        strncpy(halCfg->bbicCfg.spiAdvRegsCfg.drvName, "/dev/uio2\0", ADI_HAL_STRING_LENGTH);
        halCfg->bbicCfg.spiAdvRegsCfg.mapSize = 0x1000; /* 16MB */
    }
    
    if ((interfaceMask & (uint32_t)ADI_HAL_INTERFACE_HWRESET) > 0)
    {
        halCfg->hwResetCfg.interfaceEnabled = 1;
        
        switch(spiChipSelect)
        {
        case 0:
            {
                halCfg->hwResetCfg.resetPinIndex = 0; /* ADRV9001 reset pin bit position in FPGA register */
                break;
            }
        case 1:
            { 
                halCfg->hwResetCfg.resetPinIndex = 2; /* AD9528 reset pin bit position in FPGA register */ 
                break;
            }
            
        default:
            free(halCfg);
            return NULL;
        }
        
    }
    
    if ((interfaceMask & (uint32_t)ADI_HAL_INTERFACE_I2C) > 0)
    {
        /* TODO: Place holder for future support */
        // Set drvName equal to i2C bus # 7
        halCfg->i2cCfg.interfaceEnabled = 1;
        strncpy(halCfg->i2cCfg.drvName, "/dev/i2c-7\0", ADI_HAL_STRING_LENGTH);
    }
    
    if ((interfaceMask & (uint32_t)ADI_HAL_INTERFACE_TIMER) > 0)
    {
        halCfg->timerCfg.interfaceEnabled = 1;
        /* Currently no timer config structure for ZC706 platform */
    }
    
    return halCfg;
}

int32_t zc706sd20_DevHalCfgFree(void *devHalCfg)
{
    int32_t halError = (int32_t)ADI_HAL_OK;
    
    free(devHalCfg);
    
    return halError;
}
