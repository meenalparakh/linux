/**
* \file
* \brief FPGA9001 Multi-Chip Synchronization (MCS) functions
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/
/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.pdf" file in this zip file.
 */

#ifndef _ADI_FPGA9001_MCS_H_
#define _ADI_FPGA9001_MCS_H_

#include <stdbool.h>

#include "adi_fpga9001_types.h"
#include "adi_fpga9001_mcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initiate a sequence of MCS pulses
 *
 * \param[in] fpga9001              Context variable - Pointer to FPGA9001 device data structure
 * \param[in] numberOfPulses        Number of pulses to generate (valid values: 1 - 15)
 * \param[in] mcsPeriod             Number of device clock cycles between consecutive MCS pulses
 * \param[in] mcsPulseWidth         Width of the pulse, denoted in number of device clock cycles
 * \param[in] edge                  The device clock edge to which to align the pulse
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_Mcs_Start(adi_fpga9001_Device_t *fpga9001, 
                               uint8_t numberOfPulses, 
                               uint8_t mcsPeriod,
                               uint8_t mcsPulseWidth,
                               adi_fpga9001_McsEdge_e edge);

/**
 * \brief Check if an MCS has been requested
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[out] requested        Whether or not an MCS has been requested
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_Mcs_Requested(adi_fpga9001_Device_t *fpga9001, bool *requested);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_MCS_H_ */