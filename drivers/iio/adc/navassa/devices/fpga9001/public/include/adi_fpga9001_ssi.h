/**
 * \file
 * \brief Functions for FPGA9001 Synchronous Serial Interface (SSI) configuration
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_SSI_H_
#define _ADI_FPGA9001_SSI_H_

#include <stdint.h>
#include "adi_adrv9001_types.h"
#include "adi_fpga9001_types.h"
#include "adi_fpga9001_ssi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Set the CMOS/LVDS SSI mode for the specified SSI Lane
 *
 * \param[in] fpga9001      Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port          The port that the channel refers to
 * \param[in] channel       The channel of the specified port for which to set the SSI mode
 * \param[in] sdrEnable     Whether the interface is SDR (true) or DDR (false)
 * \param[in] multiLane     Whether the interface is 1 lane (false) or multiple lanes (true)
 * \param[in] format        The format in which data will be encoded
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_ssi_Mode_Set(adi_fpga9001_Device_t *fpga9001,
                                  adi_common_Port_e port,
                                  adi_common_ChannelNumber_e channel,
                                  bool sdrEnable,
                                  bool multiLane,
                                  adi_fpga9001_SsiFormat_e format);

/**
 * \brief Get the CMOS/LVDS SSI mode for the specified SSI Lane
 *
 * \param[in]  fpga9001     Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port         The port that the channel refers to
 * \param[in]  channel      The channel of the specified port for which to get the SSI mode
 * \param[out] sdrEnabled   Whether the interface is SDR (true) or DDR (false)
 * \param[out] multiLane    Whether the interface is 1 lane (false) or multiple lanes (true)
 * \param[out] format       The format in which data is encoded
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_ssi_Mode_Get(adi_fpga9001_Device_t *fpga9001,
                                  adi_common_Port_e port,
                                  adi_common_ChannelNumber_e channel,
                                  bool *sdrEnabled,
                                  bool *multiLane,
                                  adi_fpga9001_SsiFormat_e *format);

/**
 * \brief Get the number of bytes per sample for the specified SSI Lane
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port for which to determine the number of bytes per sample
 * \param[out] bytesPerSample   The number of bytes per sample
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_fpga9001_ssi_BytesPerSample_Get(adi_fpga9001_Device_t *fpga9001,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint8_t *bytesPerSample);

/**
 * \brief Reset the SSI interface
 *
 * \param[in] fpga9001      Context variable - Pointer to FPGA9001 device data structure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_ssi_Reset(adi_fpga9001_Device_t *fpga9001);

/**
 * \brief Configure the SSI test mode for the specified Rx channel
 *
 * \note Message type: \ref timing_direct "Direct register acccess"
 *
 * \param[in] fpga9001             Context variable - Pointer to the fpga9001 device data structure
 * \param[in] channel              The Rx channel for which to configure SSI test mode
 * \param[in] ssiTestModeConfig    The desired SSI test mode to configure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_ssi_Rx_TestMode_Configure(adi_fpga9001_Device_t *fpga9001,
                                               adi_common_ChannelNumber_e channel,
                                               adi_fpga9001_RxSsiTestModeCfg_t *ssiTestModeConfig);

/**
 * \brief Inspect the SSI test mode status for the specified Tx channel
 *
 * \note Message type: \ref timing_direct "Direct register acccess"
 *
 * \param[in] fpga9001      Context variable - Pointer to the fpga9001 device data structure
 * \param[in] channel       The Tx channel for which to inspect SSI test mode status
 * \param[out] dataError    The current test mode status of the received data over interface against a fixed pattern
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_ssi_Rx_TestMode_Inspect(adi_fpga9001_Device_t *fpga9001,
                                             adi_common_ChannelNumber_e channel,
                                             bool *dataError);

/**
 * \brief Configure the SSI test mode for the specified Tx channel
 *
 * \note Message type: \ref timing_direct "Direct register acccess"
 *
 * \param[in] fpga9001             Context variable - Pointer to the fpga9001 device data structure
 * \param[in] channel              The Tx channel for which to configure SSI test mode
 * \param[in] ssiTestModeConfig    The desired SSI test mode to configure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_ssi_Tx_TestMode_Configure(adi_fpga9001_Device_t *fpga9001,
                                               adi_common_ChannelNumber_e channel,
                                               adi_fpga9001_TxSsiTestModeCfg_t *ssiTestModeConfig);

/**
 * \brief This function programs the SSI delay configuration in FPGA9001 device.
 *
 * \note Message type: \ref timing_direct "Direct register acccess"
 *
 * \param[in] fpga9001          Context variable - Pointer to the FPGA9001 device data structure
 * \param[in] ssiType           LVDS or CMOS mode
 * \param[in] ssiCalibration    The desired SSI calibration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */

int32_t adi_fpga9001_Ssi_Delay_Configure(adi_fpga9001_Device_t *fpga9001,
                                         adi_fpga9001_SsiType_e ssiType,
                                         adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration);

/**
 * \brief This function gets the SSI delay configuration from FPGA9001 device.
 *
 * \note Message type: \ref timing_direct "Direct register acccess"
 *
 * \param[in]  fpga9001         Context variable - Pointer to the FPGA9001 device data structure
 * \param[in]  ssiType          LVDS or CMOS mode
 * \param[out] ssiCalibration   The current SSI calibration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */

int32_t adi_fpga9001_Ssi_Delay_Inspect(adi_fpga9001_Device_t *fpga9001,
                                       adi_fpga9001_SsiType_e ssiType,
                                       adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration);

#ifdef __cplusplus
}
#endif

#endif // !_ADI_FPGA9001_SSI_H_