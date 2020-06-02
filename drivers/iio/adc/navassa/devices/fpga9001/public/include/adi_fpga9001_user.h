/**
* \file
* \brief FPGA9001 user-configurable options
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_USER_H_
#define _ADI_FPGA9001_USER_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************
* FPGA Interface Macros
*******************************************/
    
#ifndef ADI_FPGA9001_LOGGING
#define ADI_FPGA9001_LOGGING 0x7F               /*LogLevel Set to API, Messages, Warnings, Error*/
#endif /* !ADI_FPGA9001_LOGGING */

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_USER_H_ */