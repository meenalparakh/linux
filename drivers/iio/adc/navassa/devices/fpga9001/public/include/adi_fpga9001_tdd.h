/**
 * \file
 * \brief Contains top level fpga9001 related function prototypes for TDD specific features
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */
/**
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.pdf" file in this zip file.
 */

#ifndef _ADI_FPGA9001_TDD_H_
#define _ADI_FPGA9001_TDD_H_

#include "adi_fpga9001_types.h"
#include "adi_fpga9001_tdd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief    Manually asserts the TDD enable pin for the selected channel and port
 *
 * This is a manual control used for debug and is overridden anytime TDD programed mode is enabled. The function will
 * NOT disable all TDD modules prior to asserting the enables, it is incumbent on the user to disable TDD prior to using
 * manual enable assertion.
 *
 * \pre The ADRV9001 must be configured for pin enable mode by calling adi_adrv9001_Radio_ChannelEnableMode_Set()
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] channel           The channel of the specified port
 * \param[in] port              The port that the channel refers to
 * \param[in] enable            The desired enable state of the pin
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_Tdd_ManualEnable_Set(adi_fpga9001_Device_t *fpga9001,
                                          adi_common_Port_e port,
                                          adi_common_ChannelNumber_e channel,
                                          bool enable);
/**
 * \brief    Reads back the TDD enable pin for the selected channel and port
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  channel          The channel of the specified port
 * \param[in]  port             The port that the channel refers to
 * \param[out] enable           The current enable state of the pin
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_Tdd_ManualEnable_Get(adi_fpga9001_Device_t *fpga9001,
                                          adi_common_Port_e port,
                                          adi_common_ChannelNumber_e channel,
                                          bool *enable);

/**
 * \brief Configure the FPGA TDD for programmed mode
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] tddConfig         The desired TDD programmed mode configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_Tdd_Configure(adi_fpga9001_Device_t *fpga9001, adi_fpga9001_TddConfig_t *tddConfig);

/**
 * \brief Inspect the FPGA TDD programmed mode configuration
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[out] tddConfig        The current TDD programmed mode configuration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_Tdd_Inspect(adi_fpga9001_Device_t *fpga9001, adi_fpga9001_TddConfig_t *tddConfig);

/**
 * \brief Set the enabledness of FPGA TDD programmed mode
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] enable            Enable (true) or disable (false) TDD programmed mode
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_Tdd_ProgrammedEnable_Set(adi_fpga9001_Device_t *fpga9001, bool enable);

/**
 * \brief Get the enabledness of FPGA TDD programmed mode
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[out] enabled          The current enabledness of TDD programmed mode
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_Tdd_ProgrammedEnable_Get(adi_fpga9001_Device_t *fpga9001, bool *enabled);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_TDD_H_ */