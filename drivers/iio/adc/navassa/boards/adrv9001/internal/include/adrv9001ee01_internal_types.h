/**
* \file
* \brief ADI internal-only types for core board functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADRV9001EE01_INTERNAL_TYPES_H_
#define _ADRV9001EE01_INTERNAL_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
*  \brief Enum of possible execution phases for day 1
*/
typedef enum adi_adrv9001_Day1ExecPhase
{
    ADI_ADRV9001_DAY1_EXEC_PHASE_SELECT_PROFILE = 0x0001,
    /*!< Optional */
    ADI_ADRV9001_DAY1_EXEC_PHASE_JTAG = 0x0002,
    ADI_ADRV9001_DAY1_EXEC_PHASE_INIT_ANALOG = 0x0004,
    ADI_ADRV9001_DAY1_EXEC_PHASE_INIT_DIGITAL_CLOCKS = 0x0008,
    ADI_ADRV9001_DAY1_EXEC_PHASE_RESOURCES_LOAD = 0x0010, 
    ADI_ADRV9001_DAY1_EXEC_PHASE_INIT_RADIO = 0x0020
} adi_adrv9001_Day1ExecPhase_e;

#ifdef __cplusplus
}
#endif

#endif