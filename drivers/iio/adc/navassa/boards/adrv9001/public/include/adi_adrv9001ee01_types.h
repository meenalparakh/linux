/**
* \file
* \brief Contains ADRV9001EE01 board specific types
*	
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001EE01_CORE_TYPES_H_
#define _ADI_ADRV9001EE01_CORE_TYPES_H_
    
#include "adi_adrv9001_types.h"
#include "adi_fpga9001_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
/**
* \brief Data structure to hold devices in an EE01 daughter card
*/
typedef struct adi_adrv9001Ee01_Board {
    adi_adrv9001_Device_t *adrv9001Device; /*!< Pointer to ADRV9001 device on EE01 board */
    adi_fpga9001_Device_t *fpga9001Device; /*!< Pointer to FPGA device on EE01 board */
} adi_adrv9001Ee01_Board_t;
    
/**
* \brief Data structure to hold devices in an CE01 daughter card
*/
typedef struct adi_adrv9001Ce01_Board {
    adi_adrv9001Ee01_Board_t parent;        /*!< The CE board features a super-set of the EE board features */
    adi_common_Device_t common;             /*!< Common layer structure */
} adi_adrv9001Ce01_Board_t;
#endif // !CLIENT_IGNORE
    
#ifdef __cplusplus
}
#endif

#endif
