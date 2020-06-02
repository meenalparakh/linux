/**
* \file
* \brief Contains FPGA9001 Data Chain types
*
* ADRV9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_DATACHAIN_TYPES_H_
#define _ADI_FPGA9001_DATACHAIN_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADI_FPGA9001_NUM_RX_DATA_CHAINS 2
#define ADI_FPGA9001_NUM_TX_DATA_CHAINS 2
#define ADI_FPGA9001_NUM_DATA_CHAINS (ADI_FPGA9001_NUM_RX_DATA_CHAINS + ADI_FPGA9001_NUM_TX_DATA_CHAINS)

/* 0x2AAAAA80, 2G /3, then reduced to a 64byte boundary */
#define RX_DMA_SIZE 715827840
#define TX_DMA_SIZE 0x40000000

/**
* \brief Enumerated list of FPGA Tollgate trigger sources
*/
typedef enum adi_fpga9001_TollgateTriggerSources
{
    ADI_FPGA9001_TRIGGER_IMMEDIATE  = 0x1,  /*!< Immediate trigger. */
    ADI_FPGA9001_TRIGGER_EXTERNAL   = 0x2,  /*!< External trigger from SMA */
    ADI_FPGA9001_TRIGGER_TDD_DP     = 0x4,  /*!< TDD datapath control */
    ADI_FPGA9001_TRIGGER_TDD_GPIO   = 0x8,  /*!< Trigger from GPIO pin */
} adi_fpga9001_TollgateTriggerSources_e;

/**
 * \brief Enumerated list of valid FPGA DMA widths
 */
typedef enum adi_fpga9001_ValidDataWidth
{
    ADI_FPGA9001_STREAM_32_BITS     = 0x0,
    ADI_FPGA9001_STREAM_64_BITS     = 0x1,
    ADI_FPGA9001_STREAM_128_BITS    = 0x3,
    ADI_FPGA9001_STREAM_256_BITS    = 0x7,
    ADI_FPGA9001_STREAM_512_BITS    = 0xF
} adi_fpga9001_ValidDataWidth_e;

/**
*  \brief Data structure to hold the FPGA Data Chain Tollgate configuration
*/
typedef struct adi_fpga9001_TollgateCfg
{
    uint32_t tollGateEdgeOrLvl;         /*!< 1 indicates that the trigger input is edge-detected.
                                         *   0 indicates that the trigger input is level-sensitive. */
    uint32_t tollGateHiRiseOrLoFall;    /*!< 1 indicates either high level or rising edge.
                                         *   0 indicates either low level or falling edge. */
    uint8_t tollgateReset;              /*!< Set to 1 resets the Tollgate.
                                         *   Clear to 0 allows for normal operation.. */
    uint32_t tollGateTrigSource;        /*!< An OR'd combination of one or more adi_fpga9001_TollgateTriggerSources_e enum values
                                         *   specifying the trigger inputs that are used to open the Tollgate. */
} adi_fpga9001_TollgateCfg_t;

/**
*  \brief Data structure to hold the FPGA Data Chain DMA configuration
*/
typedef struct adi_fpga9001_DmaCfg
{
    uint8_t enableEnhancedMode;                         /*!< When set to 1, the enhanced mode of the selected transfer is performed.
                                                         *  When cleared to 0, the traditional mode of the selected transfer is performed. */
    uint8_t enableSg;                                   /*!< When set to 1, the DMA performs a scatter-gather transfer.
                                                         *  When cleared to 0, the DMA performs a simple transfer. */
    uint32_t simpleStartAddr;                           /*!< Indicates the starting address in memory where the data should be written.
                                                         *  Must be a multiple of 64 */
    uint32_t length;                                    /*!< The number of bytes to transfer in the simple mode and enhanced scatter-gather mode.
                                                         *  Must be a multiple of 64*/
    adi_fpga9001_ValidDataWidth_e validDataWidthStream; /*!< The width of valid data on the stream interface.
                                                         *  This field is used to pack/unpack the data from the memory.*/
    uint32_t sgFirstDescriptor;                         /*!< Contains the location of the first descriptor used in both scatter-gather modes. */
    uint32_t sgLastDescriptor;                          /*!< Contains the location of the last descriptor used in both scatter-gather modes. */
    uint8_t runStop;                                    /*!< When set to 1, the DMA begins the transfer.
                                                         *  When cleared to 0, any active transfer is gracefully halted.
                                                         *  When halted, the module needs to be reset after the halt completes.*/
    uint8_t continuous;                                 /*!< NOTE: This field is meaningless for Rx DMA modules
                                                         *  When set to 1, the DMA will restart the transfer.
                                                         *  When cleared to 0, the DMA performs one transfer and stops.*/
    uint8_t active;                                     /*!< When this bit is 1, the DMA is actively transferring data. Read-only */
    uint8_t complete;                                   /*!< When this bit is 1, the transfer has completed successfully.
                                                         *  This bit will clear when run_stop is cleared to '0'. Read-only */
    uint8_t haltComplete;                               /*!< When this bit is 1, the halt that was previously issued has completed.
                                                         *  This module must be reset after a halt completes. Read-only */
    uint8_t overUnderflow;                              /*!< Overflow for Rx. Underflow for Tx. 1 means an over- or underflow has occurred on the FIFO.
                                                         *  This is a sticky bit.  Write a 1 to this bit to clear it. Read-only */
} adi_fpga9001_DmaCfg_t;

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_DATACHAIN_TYPES_H_ */
