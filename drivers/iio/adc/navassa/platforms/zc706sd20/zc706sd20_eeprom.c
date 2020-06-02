/**
* \file
* \brief Functions to read I2C EEPROM info based on FMC EEPROM format
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#include <stdlib.h>
#include <string.h>
#include "fru.h"

#include "zc706sd20_eeprom.h"
#include "adi_platform_types.h"


/*
 * Read in the file of the disk, or from the EEPROM
 */
unsigned char * read_file(const char *file_name)
{
    FILE *fp;
    unsigned char *p;
    size_t byteRead = 1;

    if ((fp = fopen(file_name, "rb")) == NULL)
    {
        printf_err("Cannot open EEPROM file '%s'\n", file_name);
        return NULL;
    }

    if ((p = (unsigned char *)x_calloc(1024, 1)) == NULL)
    {
        if (fclose(fp) != 0)
        {
            printf_err("Cannot close file.\n");
        }
        return NULL;
    }

    byteRead = fread(p, 1, 1024, fp);

    if ((byteRead < 256) || !feof(fp))
    {
        printf("Didn't read the entire EEPROM input file %s, it's too long\n", file_name);
    }


    if (fclose(fp) != 0)
    {
        printf_err("Cannot close file.\n");
    }
    return p;
}

/* This function is loosely based on fru_tools/operating-system.c::dump_BOARD() */
int32_t zc706sd20_eeprom_FmcFru_Read(fru_data_t *fru)
{
    const char *input_file = "/sys/bus/i2c/devices/7-0050/eeprom";
    uint8_t *raw_input_data = NULL;
    struct FRU_DATA *_fru = NULL;
    uint32_t i = 0;

    if ((raw_input_data = read_file(input_file)) != NULL)
    {
        if ((_fru = parse_FRU(raw_input_data)) != NULL)
        {
            fru->Board_Area.language_code = _fru->Board_Area->language_code;
            fru->Board_Area.mfg_date = _fru->Board_Area->mfg_date;
            strncpy((char *)fru->Board_Area.manufacturer, (char *)_fru->Board_Area->manufacturer + 1, 128);
            strncpy((char *)fru->Board_Area.product_name, (char *)_fru->Board_Area->product_name + 1, 128);
            strncpy((char *)fru->Board_Area.serial_number, (char *)_fru->Board_Area->serial_number + 1, 128);
            strncpy((char *)fru->Board_Area.part_number, (char *)_fru->Board_Area->part_number + 1, 128);
            strncpy((char *)fru->Board_Area.FRU_file_ID, (char *)_fru->Board_Area->FRU_file_ID + 1, 128);
            for (i = 0; i < 4; i++)
            {
                if (_fru->Board_Area->custom[i] && _fru->Board_Area->custom[i][0] & 0x3F)
                {
                    switch (_fru->Board_Area->custom[i][1])
                    {
                    case 0:
                        strncpy((char *)fru->Board_Area.pcb_rev, (char *)_fru->Board_Area->custom[i] + 2, 128);
                        break;
                    case 1:
                        strncpy((char *)fru->Board_Area.pcb_id, (char *)_fru->Board_Area->custom[i] + 2, 128);
                        break;
                    case 2:
                        strncpy((char *)fru->Board_Area.bom_rev, (char *)_fru->Board_Area->custom[i] + 2, 128);
                        break;
                    case 3:
                        strncpy((char *)fru->Board_Area.uses_lvds, (char *)_fru->Board_Area->custom[i] + 2, 128);
                        break;
                    default:
                        /* Unknown field */
                        break;
                    }
                }
            }
            return 0;
        }
        free(raw_input_data);
    }

    return ADI_HAL_GEN_SW;
}