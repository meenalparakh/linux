/**
* \file
* \brief Functions for interacting with the power monitoring chips on the CE board
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001CE01_POWERMONITOR_H_
#define _ADI_ADRV9001CE01_POWERMONITOR_H_

#include "adi_adrv9001ee01_types.h"
#include "adi_adrv9001ce01_powermonitor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure the power monitoring chips for operation
 * 
 * \note This function must be called once before calling adi_adrv9001Ce01_PowerMonitor_Read()
 *
 * \param[in] adrv9001Ce01      Context variable - Pointer to the ADRV9001 CE01 daughter card instance
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ce01_PowerMonitor_Configure(adi_adrv9001Ce01_Board_t *adrv9001Ce01);

/**
 * \brief Read the voltage and current from the specified power monitor
 * 
 * \param[in]  adrv9001Ce01             Context variable - Pointer to the ADRV9001 CE01 daughter card instance
 * \param[in]  powerMonitor             The power monitoring chip from which to read the voltage and current
 * \param[out] measuredVoltage_mV       The measured voltage, denoted in millivolts (mV)
 * \param[out] measuredCurrent_mA       The measured current, denoted in milliamperes (mA)
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001Ce01_PowerMonitor_Read(adi_adrv9001Ce01_Board_t *adrv9001Ce01,
                                           adi_adrv9001Ce01_PowerMonitor_e powerMonitor,
                                           float *measuredVoltage_mV,
                                           float *measuredCurrent_mA);

#ifdef __cplusplus
}
#endif

#endif
