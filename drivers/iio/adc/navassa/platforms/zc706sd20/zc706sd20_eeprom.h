/**
* \file
* \brief Functions to read I2C EEPROM info based on FMC EEPROM format
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ZC706SD20_EEPROM_H__
#define _ZC706SD20_EEPROM_H__

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "fru.h"

#include "zc706sd20_eeprom_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Read the EEPROM of the attached daughter card based on the FMC FRU standard
 *
 * \param[out] fru      The parsed representation of the EEPROM data
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t zc706sd20_eeprom_FmcFru_Read(fru_data_t *fru);

#ifdef __cplusplus
}
#endif
#endif
