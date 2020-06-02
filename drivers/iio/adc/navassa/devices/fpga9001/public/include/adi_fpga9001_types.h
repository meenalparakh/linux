/*!
* \file
* \brief Contains ADRV9001 API configuration and run-time type definitions
*
* ADRV9001 API Version: $ADI_FPGA9001_API_VERSION$
*
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_TYPES_H_
#define _ADI_FPGA9001_TYPES_H_

#include "adi_common.h"

#ifdef __cplusplus
extern "C" {
#endif
    
/**
 * \brief List of possible FPGA binary images to select 
 */
typedef enum adi_fpga9001_Binary
{
    ADI_FPGA9001_CMOS,
    ADI_FPGA9001_LVDS
} adi_fpga9001_Binary_e;
    
/**
 * \brief FPGA version number
 */
typedef struct adi_fpga9001_Version
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} adi_fpga9001_Version_t;

#ifndef CLIENT_IGNORE
/**
* \brief Data structure to hold FPGA9001 device instance settings
*/
typedef struct adi_fpga9001_Device
{
    adi_common_Device_t      common;       /*!< Common layer structure */
} adi_fpga9001_Device_t;
#endif // !CLIENT_IGNORE

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_TYPES_H_ */
