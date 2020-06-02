/**
* \file
* \brief ADI internal-only core board functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADRV9001EE01_INTERNAL_H_
#define _ADRV9001EE01_INTERNAL_H_

#include "adi_adrv9001ee01_types.h"

#include "adi_adrv9001_utilities_types.h"

#include "adrv9001ee01_internal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief This function programs the device using a profile file to configure the ARM.
 *
 * \param[in] adrv9001Ee01            Context variable - Pointer to ADRV9001EE01 daughter card instance
 * \param[in] day1ExecPhaseMask       An OR'd combination of adi_adrv9001_Day1ExecPhase_e specifying which steps to execute
 * \param[in] profileFilename         String containing the absolute path of the profile file
 * \param[in] radioCtrlInit           Initial radio settings
 * \param[in] adrv9001PlatformFiles   Pointer to platform files on SD card (Arm and Stream bins, Rx Gain table, Tx Atten Table)
 * \param[in] channelMask             The mask of Tx/Rx channels
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adrv9001Ee01_internal_InitializeFromProfile(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                                    uint32_t day1ExecPhaseMask,
                                                    const char* profileFilename,
                                                    adi_adrv9001_RadioCtrlInit_t *radioCtrlInit,
                                                    adi_adrv9001_PlatformFiles_t *adrv9001PlatformFiles,
                                                    uint8_t channelMask);

/**
 * \brief This function programs the device using an init struct to configure the ARM.
 *
 * \param[in] adrv9001Ee01      Context variable - Pointer to ADRV9001EE01 daughter card instance to be programmed
 * \param[in] day1ExecPhaseMask An OR'd combination of adi_adrv9001_Day1ExecPhase_e specifying which steps to execute
 * \param[in] adrv9001Init      Pointer to ADRV9001 init structure
 * \param[in] radioCtrlInit     Initial radio settings
 * \param[in] platformFiles     Pointer to platform files on SD card (ARM and Stream bins, Rx Gain table, Tx Atten Table)
 * \param[in] channelMask       The mask of Tx/Rx channels
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adrv9001Ee01_internal_Initialize(adi_adrv9001Ee01_Board_t *adrv9001Ee01,
                                         uint32_t day1ExecPhaseMask,
                                         adi_adrv9001_Init_t *adrv9001Init,
                                         adi_adrv9001_RadioCtrlInit_t *radioCtrlInit,
                                         adi_adrv9001_PlatformFiles_t *platformFiles,
                                         uint8_t channelMask);

#ifdef __cplusplus
}
#endif

#endif
