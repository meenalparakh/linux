/**
* \file
* \brief Contains prototype of HAL SW ONLY private features related function implemented in
*        adrv9001_hal_sw_only.c
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADRV9001_HAL_SW_ONLY_H_
#define _ADRV9001_HAL_SW_ONLY_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* ADI specific header files */
#include "adi_adrv9001.h"

#define SW_SPI_MAP_SIZE 1024

int32_t hal_sw_SpiWrite(adi_adrv9001_Device_t *device, uint16_t addr, uint8_t data);

int32_t hal_sw_SpiRead(adi_adrv9001_Device_t *device, uint16_t addr, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif