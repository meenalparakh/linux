/**
* \file
* \brief Core functions specific to the customer evaluation (CE) board
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001CE01_H_
#define _ADI_ADRV9001CE01_H_

#include "adi_adrv9001ee01_types.h"

#include "adi_adrv9001_utilities_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
/**
* \brief Configure a board instance for CE01 daughter card, creating and initializing member device instances
* 
* \note Does NOT allocate memory for the adi_adrv9001Ce01_Board_t, but does allocate memory for the member devices
*
* \param[in] adrv9001Ce01		Context variable - Pointer to the ADRV9001 CE01 daughter card instance
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001Ce01_Create(adi_adrv9001Ce01_Board_t *adrv9001Ce01);

/**
* \brief Close all drivers and deallocate all memory used by the board
*
* \param[in] adrv9001Ce01		Context variable - Pointer to the ADRV9001 CE01 daughter card instance
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001Ce01_Destroy(adi_adrv9001Ce01_Board_t *adrv9001Ce01);
#endif // !CLIENT_IGNORE

#ifdef __cplusplus
}
#endif

#endif
