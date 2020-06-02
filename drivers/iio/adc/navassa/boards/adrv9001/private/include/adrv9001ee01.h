/**
* \file
* \brief Private core board functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADRV9001EE01_H_
#define _ADRV9001EE01_H_

#include "adi_adrv9001ee01_types.h"

#include "adi_fpga9001_ssi.h"
#include "adi_fpga9001_mmcm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Program the FPGA with the correct binary based on initialization settings
 * 
 * \param[in] adrv9001Ee01      Pointer to ADRV9001EE01 daughter card instance to be programmed
 * \param[in] adrv9001Init      Pointer to ADRV9001 init structure
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ee01_SelectFpgaBin(adi_adrv9001Ee01_Board_t *adrv9001Ee01, adi_adrv9001_Init_t *adrv9001Init);

int32_t adi_fpga9001_Initialize(adi_fpga9001_Device_t *fpga9001Device,
                                adi_adrv9001_Init_t *init,
                                adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockDivisor);

#ifdef __cplusplus
}
#endif

#endif
