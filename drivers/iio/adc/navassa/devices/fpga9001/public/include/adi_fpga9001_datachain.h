/**
 * \file
 *
 * \brief Functions to configure the FPGA9001 Data Chains
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_DATACHAIN_H_
#define _ADI_FPGA9001_DATACHAIN_H_

#include "adi_common_types.h"

#include "adi_fpga9001_datachain_types.h"
#include "adi_fpga9001_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADI_FPGA9001_RAM_START_ADDR  0x80000000
#define ADI_FPGA9001_RX1_ADDR_OFFSET 0x00000000
#define ADI_FPGA9001_RX2_ADDR_OFFSET 0x08000000
#define ADI_FPGA9001_TX1_ADDR_OFFSET 0x10000000
#define ADI_FPGA9001_TX2_ADDR_OFFSET 0x18000000
#define ADI_FPGA9001_ORX1_ADDR_OFFSET 0x20000000
#define ADI_FPGA9001_ORX2_ADDR_OFFSET 0x28000000

/**
 * \brief Configure the specified Tollgate modules
 *
 * This function writes the adi_fpga9001_TollgateCfg_t structure to the Tollgate
 * modules selected in the dataChainSelMask. This function could be used as part
 * of a transmit setup procedure if more than just the trigger select should be
 * configured.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] tollGateCfg       The tollgate configuration to apply
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Tollgate_Configure(adi_fpga9001_Device_t *fpga9001,
                                                  adi_common_Port_e port,
                                                  adi_common_ChannelNumber_e channel,
                                                  adi_fpga9001_TollgateCfg_t *tollGateCfg);

/**
 * \brief Reads the configuration for a single tollgate module
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] tollGateCfg      The tollgate configuration that was read
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
^ \*/
int32_t adi_fpga9001_DataChain_Tollgate_Inspect(adi_fpga9001_Device_t *fpga9001,
                                                adi_common_Port_e port,
                                                adi_common_ChannelNumber_e channel,
                                                adi_fpga9001_TollgateCfg_t *tollGateCfg);

/**
 * \brief Write the reset bit for the specified tollgate modules
 *
 * This function writes the reset bit to the tollgate modules selected in
 * the dataChainSelMask.
 *
 * \note The reset bit is not self-clearing so a full reset will require
 * a call to this function to write a 1 then another call to write a 0
 * for normal operation.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] resetVal          The reset value (1 for reset; 0 for normal operation)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_TollgateReset_Set(adi_fpga9001_Device_t *fpga9001,
                                                 adi_common_Port_e port,
                                                 adi_common_ChannelNumber_e channel,
                                                 uint8_t resetVal);

/**
 * \brief Reads the reset bit for the specified tollgate modules
 *
 * This function reads the reset bit for the tollgate modules selected in the
 * dataChainSelMask. It zeros the resetVal parameter and copies each
 * module's reset value to the bit location given by the adi_fpga9001_DataChainSel_e enum.
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] resetVal         Each bit corresponds to that data chain's tollgate reset bit
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_TollgateReset_Get(adi_fpga9001_Device_t *fpga9001,
                                                 adi_common_Port_e port,
                                                 adi_common_ChannelNumber_e channel,
                                                 uint16_t *resetVal);

/**
 * \brief Sets the tollgate trigger source for one or more Data Chains
 *
 * \param[in] fpga9001           Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port               The port that the channel refers to
 * \param[in] channel            The channel of the specified port to configure
 * \param[in] triggerSource      An OR'd combination of adi_fpga9001_TollgateTriggerSources_e enum values
 *                               specifying desired trigger source(s)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_TollgateTrigger_Set(adi_fpga9001_Device_t *fpga9001,
                                                   adi_common_Port_e port,
                                                   adi_common_ChannelNumber_e channel,
                                                   uint32_t triggerSource);

/**
 * \brief Reads the tollgate trigger source for the specified Data Chain
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] triggerSelect    An OR'd combination of adi_fpga9001_TollgateTriggerSources_e enum values
 *                              indicating the current trigger source(s)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_TollgateTrigger_Get(adi_fpga9001_Device_t *fpga9001,
                                                   adi_common_Port_e port,
                                                   adi_common_ChannelNumber_e channel,
                                                   uint32_t *triggerSelect);

/**
 * \brief Configures the specified DMA modules
 *
 * This function writes the adi_fpga9001_DmaCfg_t structure to the DMA
 * modules selected in the dataChainSelMask. This function could be used as part
 * of a transmit setup procedure.
 *
 * \note The LastDescriptor bitfield and RunStop bits are not written during this function call.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] dmaCfg            The DMA configuration to set
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Dma_Configure(adi_fpga9001_Device_t *fpga9001,
                                             adi_common_Port_e port,
                                             adi_common_ChannelNumber_e channel,
                                             adi_fpga9001_DmaCfg_t *dmaCfg);

/**
 * \brief Reads the DMA configuration for a single DMA module
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] dmaCfg           The DMA configuration that was read
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Dma_Inspect(adi_fpga9001_Device_t *fpga9001,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           adi_fpga9001_DmaCfg_t *dmaCfg);

/**
 * \brief Writes the reset bit for the specified DMA modules
 *
 * \note The reset bit is self-clearing so there is no need to clear after setting.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] resetVal          The reset value (1 for reset; 0 for normal operation)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_DmaReset_Set(adi_fpga9001_Device_t *fpga9001,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint8_t reset);

/**
 * \brief Run or stop the specified DMA modules
 *
 * This function writes the run-stop bit to the DMA modules selected in the dataChainSelMask.
 * This can be used during the middle of a transfer to gracefully halt the DMA.
 * More commonly used prior to a transmit to start the DMAs before triggering.
 *
 * \note After halting, the DMA module must be reset
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] run               Run (true) or stop (false) the transfer
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_DmaRun_Set(adi_fpga9001_Device_t *fpga9001,
                                          adi_common_Port_e port,
                                          adi_common_ChannelNumber_e channel,
                                          bool run);

/**
 * \brief Sets the length for the specified DMA modules
 *
 * \note The length should be set prior to performing a transmit.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] numBytes          The number of bytes the DMA will run for after triggering.
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_DmaLength_Set(adi_fpga9001_Device_t *fpga9001,
                                             adi_common_Port_e port,
                                             adi_common_ChannelNumber_e channel,
                                             uint32_t numBytes);

/**
 * \brief Configure the Capture Control, Tollgate, & DMA modules for the specified Data Chains
 *
 * This function will configure all the selected Data Chains in the dataChainSelMask.
 * A Data Chain consists of the Capture Control, Tollgate, & DMA modules. The tollgate trigger
 * is configured using adi_fpga9001_DataChain_TollgateTrigger_Set,
 * the DMA is reset then configured using adi_fpga9001_DataChain_Dma_Configure,
 * the DMA is started by setting the run-stop bit using adi_fpga9001_DataChain_DmaRunStop_Set,
 * and finally the token passing is initiated in the Capture Control module. This function
 * is used in transmit sequences and is the final step before triggering.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] tollgateCfg       The tollgate configuration to apply
 * \param[in] dmaCfg            The DMA configuration to apply
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Configure(adi_fpga9001_Device_t *fpga9001,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         adi_fpga9001_TollgateCfg_t *tollgateCfg,
                                         adi_fpga9001_DmaCfg_t *dmaCfg);

/**
 * \brief Clear and reset all Data Chains
 *
 * This function clears and resets all Data Chains to prepare for a new transmit or as part
 * of a tear-down routine. The Capture Controls are reset and deactivated, the DMAs are halted
 * and will return errors if halted ungracefully, and the Tollgates are held in reset.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Reset(adi_fpga9001_Device_t *fpga9001);

/**
 * \brief Stops all enabled FPGA Tx Data Chains
 *
 * This function stops the FPGA from sending data to the FPGA9001 transceiver to be transmitted.
 *
 * \note This does NOT stop the FPGA9001 transceiver from transmitting. Call adi_fpga9001_ChannelEnableSet for that purpose.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_TxData_Stop(adi_fpga9001_Device_t *fpga9001);

/**
 * \brief Block until the captures complete or the timeout is reached
 *
 * \note This function should be called after adi_fpga9001_RxCaptureStart
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] timeout_ms        The maximum time (in milliseconds) to wait for the capture to complete
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Capture_Wait(adi_fpga9001_Device_t *fpga9001,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint32_t timeout_ms);

/**
 * \brief Configure the data chains, initiate capture, and wait for capture to complete
 *
 * This function configures the specified data chains, starts an Rx capture, and blocks until the captures
 * complete or the timeout is reached. Essentially, this function fills the RAM. Use the DataGet_X utility
 * functions to read data from RAM in the appropriate format.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] timeout_ms        The maximum time (in milliseconds) to wait for the capture to complete
 * \param[in] numBytes          Length of the capture in bytes
 * \param[in] trigger           The tollgate trigger source to use
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_PerformCapture(adi_fpga9001_Device_t *fpga9001,
                                              adi_common_Port_e port,
                                              adi_common_ChannelNumber_e channel,
                                              uint32_t timeout_ms,
                                              uint32_t numBytes,
                                              adi_fpga9001_TollgateTriggerSources_e trigger);

/**
 * \brief Configure the data chains and start transmitting in continuous mode
 *
 * \note This function does NOT set the data to be transmitted. For that, use one of the DataSet_X utility functions
 * to write data to RAM in the appropriate format.
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] trigger           The tollgate trigger source to use
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_PerformTx(adi_fpga9001_Device_t *fpga9001,
                                         adi_common_ChannelNumber_e channel,
                                         adi_fpga9001_TollgateTriggerSources_e trigger);

/**
 * \brief Variant of adi_fpga9001_DataChain_Data_Get_16I16Q with data deduplication.
 * 
 * Certain configurations on A0 silicon will transmit data samples multiple times.
 * This function removes them.  
 */ 
#ifdef CLIENT_IGNORE
 /** \copydoc adi_fpga9001_DataChain_Data_Get_16I16Q */
 #endif
/**
 * \sa adi_fpga9001_DataChain_Data_Get_16I16Q
 * \param[in] deduplicateBy       Decimation factor.  1 means no deduplication.
 */
int32_t adi_fpga9001_DataChain_Data_Get_16I16Q_Deduplicate(adi_fpga9001_Device_t *fpga9001,
                                                           adi_common_Port_e port,
                                                           adi_common_ChannelNumber_e channel,
                                                           uint16_t iData[],
                                                           uint16_t qData[],
                                                           uint32_t length,
                                                           uint32_t deduplicateBy);

/**
 * \brief Read data from FPGA capture memory for 16I16Q mode
 *
 * In 16I16Q mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 16I0,16Q0
 *                  0x4 | 16I1,16Q1
 *                  ... | ...
 *
 * This function unpacks the I and Q data into separate 16-bit arrays for convenient indexing
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] iData            The returned 16-bit I values
 * \param[out] qData            The returned 16-bit Q values
 * \param[in]  length           The length of the iData and qData arrays; i.e., the number of IQ samples
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Get_16I16Q(adi_fpga9001_Device_t *fpga9001,
                                               adi_common_Port_e port,
                                               adi_common_ChannelNumber_e channel,
                                               uint16_t iData[],
                                               uint16_t qData[],
                                               uint32_t length);

/**
 * \brief Variant of adi_fpga9001_DataChain_Data_Get_16IQInterleaved with data deduplication.
 * 
 * Certain configurations on A0 silicon will transmit data samples multiple times.
 * This function removes them.  
 */ 
#ifdef CLIENT_IGNORE
/** \copydoc adi_fpga9001_DataChain_Data_Get_16IQInterleaved */
#endif
 /**
  * \sa adi_fpga9001_DataChain_Data_Get_16IQInterleaved
  * \param[in]  deduplicateBy    Decimation factor.  1 means no deduplication.
 */
int32_t adi_fpga9001_DataChain_Data_Get_16IQInterleaved_Deduplicate(adi_fpga9001_Device_t *fpga9001,
                                                                    adi_common_Port_e port,
                                                                    adi_common_ChannelNumber_e channel,
                                                                    uint16_t iqData[],
                                                                    uint32_t length,
                                                                    uint32_t deduplicateBy);
    
/**
 * \brief Read data from FPGA capture memory in interleaved format for 16I16Q mode
 *
 * In 16I16Q mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 16I0,16Q0
 *                  0x4 | 16I1,16Q1
 *                  ... | ...
 *
 * This function unpacks the I and Q data into a single interleaved 16-bit array
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] iqData
 * \parblock
 * Interleaved I and Q data in the format:
 *     iqData[index] | Value
 *     ---------------------
 *                 0 | I0
 *                 1 | Q0
 *                 2 | I1
 *                 3 | Q1
 *               ... | ...
 * \endparblock
 * \param[in]  length           The length of the iqData array; i.e., twice the number of IQ samples
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Get_16IQInterleaved(adi_fpga9001_Device_t *fpga9001,
                                                        adi_common_Port_e port,
                                                        adi_common_ChannelNumber_e channel,
                                                        uint16_t iqData[],
                                                        uint32_t length);

/**
 * \brief Read data from FPGA capture memory in for 16I Only mode
 *
 * In 16I Only mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 16I1,16I0
 *                  0x4 | 16I3,16I2
 *                  ... | ...
 *
 * This function unpacks the I data into a 16-bit array for convenient indexing
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] iData
 * \parblock
 * I data in the format:
 *     iData[index] | Value
 *     ---------------------
 *                0 | I0
 *                1 | I1
 *                2 | I2
 *                3 | I3
 *              ... | ...
 * \endparblock
 * \param[in]  length           The length of the iData array; i.e., the number of I samples
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Get_16I(adi_fpga9001_Device_t *fpga9001,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint16_t iData[],
                                            uint32_t length);

/**
 * \brief Read data from FPGA capture memory in for 8-Bit Symbol mode
 *
 * In 8-Bit Symbol mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 8S3,8S2,8S1,8S0
 *                  0x4 | 8S7,8S6,8S5,8S4
 *                  ... | ...
 *
 * This function unpacks the data into an 8-bit array for convenient indexing
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] symbols
 * \parblock
 * Symbol data in the format:
 *     symbols[index] | Value
 *     ---------------------
 *                  0 | S0
 *                  1 | S1
 *                  2 | S2
 *                  3 | S3
 *                ... | ...
 * \endparblock
 * \param[in]  length           The length of the symbols array
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Get_8S(adi_fpga9001_Device_t *fpga9001,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           uint8_t symbols[],
                                           uint32_t length);

/**
 * \brief Read data from FPGA capture memory in for 2-Bit Symbol mode
 *
 * In 2-Bit Symbol mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 6X,2S3,6X,2S2,6X,2S1,6X,2S0
 *                  0x4 | 6X,2S7,6X,2S6,6X,2S5,6X,2S4
 *                  ... | ...
 *
 * This function unpacks the data into an 8-bit array containing 2 bit symbols for convenient indexing
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  port             The port that the channel refers to
 * \param[in]  channel          The channel of the specified port to configure
 * \param[out] symbols
 * \parblock
 * Symbol data in the format:
 *     symbols[index] | Value
 *     ---------------------
 *                  0 | S0
 *                  1 | S1
 *                  2 | S2
 *                  3 | S3
 *                ... | ...
 * \endparblock
 * \param[in]  length           The length of the symbols array
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Get_2S(adi_fpga9001_Device_t *fpga9001,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           uint8_t symbols[],
                                           uint32_t length);

/**
 * \brief Write data to FPGA transmit memory for 16I16Q mode
 *
 * In 16I16Q mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 16I0,16Q0
 *                  0x4 | 16I1,16Q1
 *                  ... | ...
 *
 * This function takes separate 16-bit arrays of I and Q data and packs them into RAM appropriately
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] iData             Array of 16-bit I values to be packed into RAM
 * \param[in] qData             Array of 16-bit Q values to be packed into RAM
 * \param[in] length            The length of the iData and qData arrays; i.e., the number of IQ samples
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Set_16I16Q(adi_fpga9001_Device_t *fpga9001,
                                               adi_common_Port_e port,
                                               adi_common_ChannelNumber_e channel,
                                               uint16_t iData[],
                                               uint16_t qData[],
                                               uint32_t length);

/**
 * \brief Write data to FPGA transmit memory from interleaved format for 16I16Q mode
 *
 * In 16I16Q mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 16I0,16Q0
 *                  0x4 | 16I1,16Q1
 *                  ... | ...
 *
 * This function takes a single 16-bit array of interleaved I and Q data and packs it into RAM appropriately
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] iqData
 * \parblock
 * Interleaved I and Q data in the format:
 *     iqData[index] | Value
 *     ---------------------
 *                 0 | I0
 *                 1 | Q0
 *                 2 | I1
 *                 3 | Q1
 *               ... | ...
 * \endparblock
 * \param[in] length            The length of the iqData array; i.e., twice the number of IQ samples
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Set_16IQInterleaved(adi_fpga9001_Device_t *fpga9001,
                                                        adi_common_Port_e port,
                                                        adi_common_ChannelNumber_e channel,
                                                        uint16_t iqData[],
                                                        uint32_t length);

/**
 * \brief Write data to FPGA transmit memory for 16I Only mode
 *
 * In 16I Only mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 16I1,16I0
 *                  0x4 | 16I3,16I2
 *                  ... | ...
 *
 * This function takes a 16-bit array of I data and packs it into RAM appropriately
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] iData
 * \parblock
 * I data in the format:
 *     iData[index] | Value
 *     ---------------------
 *                0 | I0
 *                1 | I1
 *                2 | I2
 *                3 | I3
 *              ... | ...
 * \endparblock
 * \param[in] length            The length of the iData array; i.e., the number of I samples
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Set_16I(adi_fpga9001_Device_t *fpga9001,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint16_t iData[],
                                            uint32_t length);

/**
 * \brief Write data to FPGA transmit memory for 8-Bit Symbol mode
 *
 * In 8-Bit Symbol mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 8S3,8S2,8S1,8S0
 *                  0x4 | 8S7,8S6,8S5,8S4
 *                  ... | ...
 *
 * This function takes an 8-bit array of symbols and packs it into RAM appropriately
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] symbols
 * \parblock
 * Symbol data in the format:
 *     symbols[index] | Value
 *     ---------------------
 *                  0 | S0
 *                  1 | S1
 *                  2 | S2
 *                  3 | S3
 *                ... | ...
 * \endparblock
 * \param[in] length            The length of the symbols array
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Set_8S(adi_fpga9001_Device_t *fpga9001,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           uint8_t symbols[],
                                           uint32_t length);

/**
 * \brief Write data to FPGA transmit memory for 2-Bit Symbol mode
 *
 * In 2-Bit Symbol mode, data is packed into RAM in the format:
 *     Address (offset) | Data
 *     ----------------------------
 *                  0x0 | 6X,2S3,6X,2S2,6X,2S1,6X,2S0
 *                  0x4 | 6X,2S7,6X,2S6,6X,2S5,6X,2S4
 *                  ... | ...
 *
 * This function takes an 8-bit array of 2-bit symbols and packs it into RAM appropriately
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] port              The port that the channel refers to
 * \param[in] channel           The channel of the specified port to configure
 * \param[in] symbols
 * \parblock
 * Symbol data in the format:
 *     symbols[index] | Value
 *     ---------------------
 *                  0 | S0
 *                  1 | S1
 *                  2 | S2
 *                  3 | S3
 *                ... | ...
 * \endparblock
 * \param[in] length            The length of the symbols array
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_Data_Set_2S(adi_fpga9001_Device_t *fpga9001,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           uint8_t symbols[],
                                           uint32_t length);

/**
 * \brief Set the Rx clock rate for the specified channel
 * 
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] channel           The channel for which to set the clock rate
 * \param[in] rxClkRate_Hz      The desired clock rate in Hertz
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_RxClkRate_Set(adi_fpga9001_Device_t *fpga9001, 
                                             adi_common_ChannelNumber_e channel, 
                                             uint8_t rxClkRate_Hz);
/**
 * \brief Get the Rx clock rate for the specified channel
 * 
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  channel          The channel for which to set the clock rate
 * \param[out] rxClkRate_Hz     The current clock rate in Hertz
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_DataChain_RxClkRate_Get(adi_fpga9001_Device_t *fpga9001, 
                                             adi_common_ChannelNumber_e channel, 
                                             uint8_t *rxClkRate_Hz);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_DATACHAIN_H_ */