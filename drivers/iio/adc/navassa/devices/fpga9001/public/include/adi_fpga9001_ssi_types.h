/**
 * \file
 * \brief Types for FPGA9001 Synchronous Serial Interface (SSI) configuration
 *
 * ADRV9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_SSI_TYPES_H_
#define _ADI_FPGA9001_SSI_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_adrv9001_types.h"

/**
 * \brief Enumeration of SSI type
 */
typedef enum adi_fpga9001_SsiType
{
    ADI_FPGA9001_SSI_TYPE_DISABLE = 0,  /*!< Disable SSI Type */
    ADI_FPGA9001_SSI_TYPE_CMOS = 1,     /*!< CMOS SSI Type */
    ADI_FPGA9001_SSI_TYPE_LVDS = 2      /*!< LVDS SSI Type */
} adi_fpga9001_SsiType_e;

/**
 * \brief SSI formats
 * \note Copy of adi_adrv9001_SsiDataFormat_e
 */
typedef enum adi_fpga9001_SsiFormat
{
    ADI_FPGA9001_SSI_FORMAT_2_BIT_SYMBOL_DATA,  /*!< 2 bit symbol data (CMOS) */
    ADI_FPGA9001_SSI_FORMAT_8_BIT_SYMBOL_DATA,  /*!< 8 bit symbol data (CMOS) */
    ADI_FPGA9001_SSI_FORMAT_16_BIT_SYMBOL_DATA, /*!< 16 bit symbol data (CMOS) */
    ADI_FPGA9001_SSI_FORMAT_12_BIT_I_Q_DATA,    /*!< 12 bit I/Q data (LVDS) */
    ADI_FPGA9001_SSI_FORMAT_16_BIT_I_Q_DATA     /*!< 16 bit I/Q data (CMOS/LVDS) */
} adi_fpga9001_SsiFormat_e;

/**
 * \brief Type of data to transmit over SSI
 */
typedef enum adi_fpga9001_SsiTestModeData
{
    ADI_FPGA9001_SSI_TESTMODE_DATA_NORMAL = 0,      /*!< No check available for this data type */
    ADI_FPGA9001_SSI_TESTMODE_DATA_FIXED_PATTERN,
    ADI_FPGA9001_SSI_TESTMODE_DATA_RAMP_NIBBLE,
    ADI_FPGA9001_SSI_TESTMODE_DATA_RAMP_16_BIT,     /*!< LSSI-Only */
    ADI_FPGA9001_SSI_TESTMODE_DATA_PRBS15,          /*!< LSSI-Only */
    ADI_FPGA9001_SSI_TESTMODE_DATA_PRBS7,           /*!< LSSI-Only */
} adi_fpga9001_SsiTestModeData_e;

/**
 * \brief Data structure to transmit FPGA9001 SSI test mode configuration
 */
typedef struct adi_fpga9001_TxSsiTestModeCfg
{
    adi_fpga9001_SsiTestModeData_e testData;   /*!< Type of data to transmit over SSI */
    uint32_t fixedDataPatternToTransmit;       /*!< Value of Fixed pattern to transmit over interface */
} adi_fpga9001_TxSsiTestModeCfg_t;

/**
 * \brief Data structure to receive FPGA9001 SSI test mode configuration
 */
typedef struct adi_fpga9001_RxSsiTestModeCfg
{
    adi_fpga9001_SsiTestModeData_e testData;   /*!< Type of data to receive over SSI and check */
    uint32_t fixedDataPatternToCheck;          /*!< Value of Fixed pattern to check against pattern recevied over interface */
} adi_fpga9001_RxSsiTestModeCfg_t;

/**
 * \brief Data structure to hold FPGA9001 CMOS delay calibration configuration for each channel.
 */
typedef struct adi_fpga9001_SsiCalibrationCfg
{
    uint8_t rxStrobeDelay[ADI_ADRV9001_NUM_CHANNELS];   /*!< Strobe delay for Rx1/Rx2 channels */
    uint8_t rxIDataDelay[ADI_ADRV9001_NUM_CHANNELS];    /*!< I data delay for Rx1/Rx2 channels */
    uint8_t rxQDataDelay[ADI_ADRV9001_NUM_CHANNELS];    /*!< Q data delay for Rx1/Rx2 channels */

    uint8_t txClkDelay[ADI_ADRV9001_NUM_CHANNELS];      /*!< CLK delay for Tx1/Tx2 channels */
    uint8_t txStrobeDelay[ADI_ADRV9001_NUM_CHANNELS];   /*!< Strobe delay for Tx1/Tx2 channels */
    uint8_t txIDataDelay[ADI_ADRV9001_NUM_CHANNELS];    /*!< I data delay for Tx1/Tx2 channels */
    uint8_t txQDataDelay[ADI_ADRV9001_NUM_CHANNELS];    /*!< Q data delay for Tx1/Tx2 channels */
} adi_fpga9001_SsiCalibrationCfg_t;

#ifdef __cplusplus
}
#endif

#endif // !_ADI_FPGA9001_SSI_TYPES_H_