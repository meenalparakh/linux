/**
* \file
* \brief Functions to read I2C EEPROM info based on FMC EEPROM format
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ZC706SD20_EEPROM_TYPES_H__
#define _ZC706SD20_EEPROM_TYPES_H__

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "fru.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief FMC FRU board information
 *
 * \note Based on fru_tools/fru.h::BOARD_INFO - modified for use with eRPC
 */
typedef struct board_info {
    uint8_t language_code;
    uint32_t mfg_date;
    uint8_t manufacturer[128];
    uint8_t product_name[128];
    uint8_t serial_number[128];
    uint8_t part_number[128];
    uint8_t FRU_file_ID[128];
    uint8_t pcb_rev[128];                  /*!< ADI custom field 0 */
    uint8_t pcb_id[128];                   /*!< ADI custom field 1 */
    uint8_t bom_rev[128];                  /*!< ADI custom field 2 */
    uint8_t uses_lvds[128];                /*!< ADI custom field 3 */
//    char *tuning;
} board_info_t;

/**
 * \brief FMC FRU EEPROM information
 *
 * \note Based on fru_tools/fru.h::FRU_DATA - modified for use with eRPC
 */
typedef struct fru_data{
//    char *Internal_Area;
//    char *Chassis_Info;
    board_info_t Board_Area;
//    char *Product_Info;
//    multirecord_info_t MultiRecord_Area;
} fru_data_t;

#ifdef __cplusplus
}
#endif
#endif
