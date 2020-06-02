/*!
* \file
* \brief FPGA9001 MCS types
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_MCS_TYPES_H_
#define _ADI_FPGA9001_MCS_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif
   
/**
 * \brief MCS edge alignment with device clock
 */
typedef enum adi_fpga9001_McsEdge
{
    ADI_FPGA9001_MCS_EDGE_RISING = 0,   /*!< Align MCS assert/deassert to rising edge of device clock */
    ADI_FPGA9001_MCS_EDGE_FALLING       /*!< Align MCS assert/deassert to falling edge of device clock */
} adi_fpga9001_McsEdge_e;

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_MCS_TYPES_H_ */
