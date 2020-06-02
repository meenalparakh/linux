/**
 * \file
 * \brief ADI internal-only FPGA9001 functione
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */
/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_INTERNAL_H_
#define _ADI_FPGA9001_INTERNAL_H_

#include "adi_fpga9001_internal_types.h"
#include "adi_fpga9001_datachain.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure FPGA to assign GPIO pins as observation GPIO
 *
 * \param[in] fpga9001                  Context variable - Pointer to FPGA9001 device data structure
 * \param[in] observationGpioConfig     The desired configuration to assign FPGA GPIO pins as observation GPIO
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_internal_ObservationGpio_Configure(adi_fpga9001_Device_t *fpga9001,
                                                        adi_fpga9001_internal_ObservationGpioCfg_t *observationGpioConfig);

/**
 * \brief Read observation GPIO data from FPGA capture memory
 *
 * \param[in]  fpga9001                 Context variable - Pointer to FPGA9001 device data structure
 * \param[out] obsData                  Buffer to store the captured data
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_internal_ObservationGpio_Get(adi_fpga9001_Device_t *fpga9001,
                                                  uint32_t *obsData);

/**
 * \brief Configure the data chains, initiate capture, and wait for capture to complete
 *
 * This function configures the specified data chains, starts an gain index capture, and blocks until the captures
 * complete or the timeout is reached. Essentially, this function fills the RAM. Use the DataGet_X utility
 * functions to read data from RAM in the appropriate format.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] timeout_ms        The maximum time (in milliseconds) to wait for the capture to complete
 * \param[in] numBytes          Length of the capture in bytes
 * \param[in] trigger           The tollgate trigger source to use
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_internal_GainIndex_Capture(adi_fpga9001_Device_t *fpga9001,
                                                adi_common_ChannelNumber_e channel,
                                                uint32_t timeout_ms,
                                                uint32_t numBytes,
                                                adi_fpga9001_TollgateTriggerSources_e trigger);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_INTERNAL_H_ */