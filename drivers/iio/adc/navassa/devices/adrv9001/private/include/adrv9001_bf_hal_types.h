/**
* \brief Contains BitField definitions to support ADRV9001 transceiver device.
*
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef __ADRV9001_BF_HAL_TYPES_H__
#define __ADRV9001_BF_HAL_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif



typedef enum adrv9001_Bf_Error
{
    ADRV9001_BF_NO_ACTION = 10, /*!< BitField function successful. No error Detected */
    ADRV9001_BF_SPI_FAIL,      /*!< HAL SPI operation failure. SPI controller Down */
    ADRV9001_BF_CACHE_OVERFLOW, /*!< The bitField Cache has exceeded the cache size */
    ADRV9001_BF_HW_RMW_NOT_INITIALISED /*!< The HW_RMW is not initialised */
} adrv9001_Bf_Error_e;


typedef enum adrv9001_Bf_WrCache_State
{
    ADRV9001_WR_CACHE_OFF = 0x0,
    ADRV9001_WR_HW_RMW_ON = 0x1,
    ADRV9001_WR_BF_CACHE_ON = 0x2,
    ADRV9001_WR_GLB_CACHE_ON = 0x4,
    ADRV9001_WR_GBL_MERGE_ON = 0x8
} adrv9001_Bf_WrCache_State_e;
 

typedef enum adrv9001_Bf_RdCache_State
{
    ADRV9001_RD_CACHE_OFF = 0x0,
    ADRV9001_RD_BF_CACHE_ON = 0x1,
    ADRV9001_RD_GLB_CACHE_ON = 0x2
} adrv9001_Bf_RdCache_State_e;

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_HAL_TYPES_H__
