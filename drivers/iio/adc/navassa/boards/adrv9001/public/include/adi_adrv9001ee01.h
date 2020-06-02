/**
 * \file
 * \brief Core board functions
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

/**
 * Copyright 2018 - 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001EE01_H_
#define _ADI_ADRV9001EE01_H_

#include "adi_adrv9001ee01_types.h"

#include "adi_adrv9001_utilities_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
/**
 * \brief Configure a board instance for EE01 daughter card, creating and initializing member devices
 * 
 * \note Does NOT allocate memory for the adi_adrv9001Ee01_Board_t, but does allocate memory for the member devices
 *
 * \param[in] adrv9001Ee01		Context variable - Pointer to the ADRV9001 EE01 daughter card instance
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ee01_Create(adi_adrv9001Ee01_Board_t *adrv9001Ee01);

/**
 * \brief Close all drivers and deallocate all memory used by the board
 *
 * \param[in] adrv9001Ee01		Context variable - Pointer to the ADRV9001 EE01 daughter card instance
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ee01_Destroy(adi_adrv9001Ee01_Board_t *adrv9001Ee01);
#endif // !CLIENT_IGNORE

/**
 * \brief This function programs the device using a profile file to configure the ARM.
 *
 * \param[in] adrv9001Ee01		Context variable - Pointer to ADRV9001EE01 daughter card instance
 * \param[in] profileFilename	String containing the absolute path of the profile file
 * \param[in] radioCtrlInit	    Initial radio settings
 * \param[in] platformFiles		Pointer to platform files on SD card (Arm and Stream bins, Rx Gain table, Tx Atten Table)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ee01_InitializeFromProfile(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                               const char* profileFilename,
                                               adi_adrv9001_RadioCtrlInit_t *radioCtrlInit,
                                               adi_adrv9001_PlatformFiles_t *platformFiles);

/**
 * \brief This function programs the device using an init struct to configure the ARM.
 *
 * \param[in] adrv9001Ee01      Context variable - Pointer to ADRV9001EE01 daughter card instance
 * \param[in] init			    Pointer to ADRV9001 init structure
 * \param[in] radioCtrlInit	    Initial radio settings
 * \param[in] platformFiles	    Files on SD card (ARM and Stream bins, Rx Gain table, Tx Atten Table)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ee01_Initialize(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                    adi_adrv9001_Init_t *init,
                                    adi_adrv9001_RadioCtrlInit_t *radioCtrlInit,
                                    adi_adrv9001_PlatformFiles_t *platformFiles);

#ifdef __cplusplus
}
#endif

#endif
