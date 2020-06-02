/**
* \file
* \brief Contains ADRV9001 ADI specific utility functions to read/write ADRV9001 device memory, internal RAM capture/drive.
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_INTERNAL_UTILITIES_H_
#define _ADI_ADRV9001_INTERNAL_UTILITIES_H_

#include "adi_adrv9001_internal_utilities_types.h"
#include "jsmn.h"

#ifdef __cplusplus
extern "C" {
#endif
    
/**
* \brief Sets the RAM drive and capture configurations.
*
* \pre This function can be called only after ARM is initialized
*
* \param[in] adrv9001               Context variable - Pointer to the ADRV9001 device data structure
* \param[in] ramDriveCaptureConfig  Desired RAM drive and capture configuration
* \param[in] channel                The channel of the specified port for which to configure the RAM capture
* \param[in] port                   The port that the channel refers to
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_RamDriveCapture_Configure(adi_adrv9001_Device_t *adrv9001, 
                                                                  adi_adrv9001_RamDriveCaptureConfig_t *ramDriveCaptureConfig, 
                                                                  adi_common_ChannelNumber_e channel,
                                                                  adi_common_Port_e port);

/**
* \brief Sets the RAM drive and capture start mailbox command.
*
* \pre This function can be called only after ARM is initialized
*
* \param[in] adrv9001               Context variable - Pointer to the ADRV9001 device data structure
* \param[in] ramDriveCaptureConfig  Desired RAM drive and capture configuration
* \param[in] channel                The channel of the specified port for which to configure and start the RAM capture
* \param[in] port                   The port that the channel refers to
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_RamDriveCapture_Start(adi_adrv9001_Device_t *adrv9001, 
                                                              adi_adrv9001_RamDriveCaptureConfig_t *ramDriveCaptureConfig,
                                                              adi_common_ChannelNumber_e channel,
                                                              adi_common_Port_e port);

/**
* \brief Read valid memory locations from the ADRV9001 device
*
* Calling function must ensure that the memory region requested is within valid memory range
*
* \pre This function is used for only debug/Engineering purpose
*
* \param[in]  adrv9001              Context variable - Pointer to the ADRV9001 device data structure
* \param[in]  address				The 32bit ARM address to read from.
* \param[out] returnData			Byte(uint8_t) array containing the data read from the ARM memory.
* \param[in]  byteCount				Number of bytes in the returnData array.
* \param[in]  autoIncrement			A boolean flag to enable or disable auto-increment of ARM register address
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_DeviceMemory_Read(adi_adrv9001_Device_t *adrv9001, 
                                                          uint32_t address, 
                                                          uint8_t returnData[], 
                                                          uint32_t byteCount, 
                                                          uint8_t autoIncrement);

/**
* \brief Write data in valid memory locations of the ADRV9001 device
*
* Calling function must ensure that the memory region requested is within valid memory range
*
* \pre This function is used for only debug/Engineering purpose
*
* \param[in] adrv9001               Context variable - Pointer to the ADRV9001 device data structure
* \param[in] address                    The 32bit ARM address to write.
* \param[in] returnData                 Byte array (uint8_t) containing data to be written to ARM memory
* \param[in] byteCount                  Number of bytes in the returnData array.
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_DeviceMemory_Write(adi_adrv9001_Device_t *adrv9001, 
                                                           uint32_t address, 
                                                           const uint8_t data[], 
                                                           uint32_t byteCount);

/**
* \brief Configure GPIO pins as observation ports for the internal ADRV9001 control out signals
*
* \param[in] adrv9001           Context variable - Pointer to the ADRV9001 device data structure
* \param[in] port               The port that the channel refers to
* \param[in] channel            The channel of the specified port for which to configure the control out signals
* \param[in] controlOutMuxCfg   The desired control out mux configuration to be routed to GPIO for internal access/debug purpose
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_ControlOutMux_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                adi_common_Port_e port,
                                                                adi_common_ChannelNumber_e channel,
                                                                adi_adrv9001_ControlOutMuxCfg_t *controlOutMuxCfg);

/**
* \brief Enable the receiver SSI (CSSI or LSSI) interface for the receiver path
*
* \param[in] adrv9001      Context variable - Pointer to the ADRV9001 device data structure
* \param[in] channel       The channel of the specified port for which to enable the Rx SSI interface
* \param[in] enableIlb     For the selected channel, True:Enable SSI interface for ILB; False:Disable RX SSI interface
* \param[in] enableElb     For the selected channel, True:Enable SSI interface for ELB; False:Disable RX SSI interface
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Rx_Ssi_Enable(adi_adrv9001_Device_t *adrv9001,
                                                                     adi_common_ChannelNumber_e channel,
                                                                     bool enableIlb,
                                                                     bool enableElb);

/**
* \brief Configure GPIO pins as observation ports for the internal ADRV9001 control out signals
*
* \param[in] adrv9001      Context variable - Pointer to the ADRV9001 device data structure
* \param[in] rxDataPathSel The Rx datapath configuration port(Rx/ORx/ILB/ELB) that the channel refers to 
* \param[in] channel       The channel of the specified port for which to configure the Rx data path
* \param[in] loType        The LO type for the specified channel for which to configure to LO type:ON/OFFSET LO
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Rx_Datapath_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                             adi_common_Port_e rxDataPathSel,
                                                                             adi_common_ChannelNumber_e channel,
                                                                             adi_adrv9001_LoType_e loType);

/**
* \brief Configure to send a calibration tone into the LB mixer for ILB.
*
* \param[in] adrv9001                       Context variable - Pointer to the ADRV9001 device data structure
* \param[in] ilbCalToneStatus               Enumeration to select desired calibration tone attenuation value 
* \param[in] pllAtten                       Enumeration to select desired PLL attenuation value 
* \param[in] calToneSwitchAtten             Enumeration to disable/enable/update calibration tone frequency 
* \param[in] calibrationToneFrequency_Hz    Desired calibration tone frequency to send for the given channel in Hz 
* \param[in] channel                        The channel of the specified port for which to configure the calibration tone
* \param[in] enableLo                       For the selected channel, True:Enable RF LO; False:Disable RF LO
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Ilb_Calibration_Tone_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                                      adi_adrv9001_IlbCalToneStatusSel_e ilbCalToneStatus,
                                                                                      adi_adrv9001_PllAtten_e pllAtten,
                                                                                      adi_adrv9001_CalToneSwitchAtten_e calToneSwitchAtten,
                                                                                      int64_t calibrationToneFrequency_Hz,
                                                                                      adi_common_ChannelNumber_e channel,
                                                                                      bool enableLo);

/**
* \brief Configure to access a generic internal debug firmware function.
*
* \param[in] adrv9001               Context variable - Pointer to the ADRV9001 device data structure
* \param[in] channel                The channel of the specified port
* \param[in] port                   The port that the channel refers to
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Generic_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                         adi_common_ChannelNumber_e channel,
                                                                         adi_common_Port_e port);

/**
* \brief Read initial/tracking calibration status
*
* \pre Channel state any of STANDBY, CALIBRATED, PRIMED, RF_ENABLED
*
* \param[in]  adrv9001               Context variable - Pointer to the ADRV9001 device data structure
* \param[in]  channel                The channel of the specified port for which the status is requested
* \param[in]  port                   The port that the channel refers to
* \param[in]  calAlgoObjId           The object id of the calibration algorithm for which the status is requested
* \param[out] returnData             Byte array containing the status information read from the configuration data structure
* \param[in]  byteCount              The length of the data to read from ARM memory
* \param[in]  offset                 The starting location in the ARM firmware configuration data structure to read the status
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_utilities_Cal_Status_Get(adi_adrv9001_Device_t *adrv9001, 
                                                       adi_common_ChannelNumber_e channel,
                                                       adi_common_Port_e port,
                                                       adi_adrv9001_CalAlgoObjId_e calAlgoObjId,
                                                       uint8_t returnData[],
                                                       uint32_t byteCount,
                                                       uint16_t offset);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_INTERNAL_UTILITIES_H_ */