/**
* \file
* \brief Contains Adrv9001 features related function implementation defined in
* adrv9001_hal_sw_only.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* "adi_adrv9001_user.h" contains the #define that other header file use */
#include "adi_adrv9001_user.h"

/* Header file corresponding to the C file */
#include "adrv9001_hal_sw_only.h"

/* ADI specific header files */
#include "adi_common_macros.h"


/* Header files related to libraries */

/* System header files */
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>
#else
#include <stdint.h>
#include <string.h>
#endif

/*********************************************************************************************************/
static int16_t swSpiAddrBuff[SW_SPI_MAP_SIZE];
static uint8_t swSpiInitialized = false;
static uint8_t swSpiValBuff[SW_SPI_MAP_SIZE] = { 0 };

void initAddrCache(void)
{
    memset(swSpiAddrBuff, -1, sizeof(swSpiAddrBuff));
    swSpiInitialized = true;
}

int32_t hal_sw_SpiWrite(adi_adrv9001_Device_t *device, uint16_t addr, uint8_t data)
{
    int i = 0;

    if (swSpiInitialized != 1)
    {
        initAddrCache();
    }
    ADI_API_PRIV_ENTRY_EXPECT(device);

    while (i < SW_SPI_MAP_SIZE && swSpiAddrBuff[i] != -1)
    {
        if (addr == swSpiAddrBuff[i])
        {
            swSpiValBuff[i] = data;
            return ADI_COMMON_HAL_OK;
        }
        i++;
    }
    if (i < SW_SPI_MAP_SIZE)
    {
        swSpiAddrBuff[i] = addr;
        swSpiValBuff[i] = data;
        return ADI_COMMON_HAL_OK;
    }
    else
    {
        return ADI_COMMON_HAL_SPI_FAIL;
    }
}

int32_t hal_sw_SpiRead(adi_adrv9001_Device_t *device, uint16_t addr, uint8_t *data)
{
    int i = 0;

    if (swSpiInitialized != 1)
    {
        initAddrCache();
    }
    ADI_API_PRIV_ENTRY_EXPECT(device);

    while (i < SW_SPI_MAP_SIZE && swSpiAddrBuff[i] != -1)
    {
        if (addr == swSpiAddrBuff[i])
        {
            *data = swSpiValBuff[i];
            return ADI_COMMON_HAL_OK;
        }
        i++;
    }
    *data = 0;
    return ADI_COMMON_HAL_OK;
}