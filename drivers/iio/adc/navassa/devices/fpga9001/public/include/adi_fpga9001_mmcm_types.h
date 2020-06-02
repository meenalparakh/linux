/*!
* \file
* \brief FPGA9001 MMCM types
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_CLK_TYPES_H_
#define _ADI_FPGA9001_CLK_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
* \brief Device clock output divisor
*/
typedef enum adi_fpga9001_Mmcm_ClockOutput_Divisor
{
    ADI_FPGA9001_MMCM_CLKDIV_BYPASS = 0,
    ADI_FPGA9001_MMCM_CLKDIV_2      = 1,
    ADI_FPGA9001_MMCM_CLKDIV_4      = 2,
    ADI_FPGA9001_MMCM_CLKDIV_8      = 3,
    ADI_FPGA9001_MMCM_CLKDIV_16     = 4,
    ADI_FPGA9001_MMCM_CLKDIV_32     = 5,
    ADI_FPGA9001_MMCM_CLKDIV_64     = 6
} adi_fpga9001_Mmcm_ClockOutput_Divisor_e;

/**
* \brief Data structure to hold FPGA9001 device MMCM configuration settings
*/
typedef struct adi_fpga9001_MmcmCfg
{
    uint32_t fpgaVcoClockFreq_kHz;      /*!< FPGA VCO clock frequency */
    uint32_t mcsClock_kHz;              /*!< The programmable period of MCS pulses will be dictated by this clock */
    uint32_t observationGpiosClock_kHz; /*!< Target clock is the frequency at which gainTable updates the gainIndex */
    uint32_t tddClock_kHz;              /*!< TDD clock frequency to support TDD mode */
} adi_fpga9001_MmcmCfg_t;

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_CLK_TYPES_H_ */
