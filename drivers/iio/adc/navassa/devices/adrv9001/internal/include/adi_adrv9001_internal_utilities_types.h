/*!
* \file
* \brief Contains ADRV9001 API utility initialization type definitions for ADI specific APIs
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_INTERNAL_UTILITIES_TYPES_H_
#define _ADI_ADRV9001_INTERNAL_UTILITIES_TYPES_H_

#include <stdint.h>
#include "adi_adrv9001_types.h"
#include "adi_common_error_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/
#define ADRV9001_ARM_OBJECTID_INTERNAL_CAL_STATUS                    0x63
#define ADRV9001_ARM_OBJECTID_INTERNAL_RAM_DRIVE_AND_CAPTURE_CONFIG  0x95
#define ADRV9001_ARM_OBJECTID_INTERNAL_RAM_DRIVE_AND_CAPTURE_START   0x96
#define ADRV9001_ARM_OBJECTID_INTERNAL_BBDC_TEST_DISABLE             0x97
#define ADRV9001_ARM_OBJECTID_INTERNAL_DEBUG                         0x99

#define ADI_ADRV9001_INTERNAL_DEBUG_TIMEOUT_US     5000000
#define ADI_ADRV9001_INTERNAL_DEBUG_INTERVAL_US       1000

#define ADI_ADRV9001_SETBBDC_DISABLE_TIMEOUT_US    1000000
#define ADI_ADRV9001_SETBBDC_DISABLE_INTERVAL_US      1000

/*
*********************************************************************************************************
*                                             ENUMs
*********************************************************************************************************
*/

/**
 * \brief RAM drive type enumeration 
 */
typedef enum adi_adrv9001_RamDriveType
{
    ADI_ADRV9001_RAM_DRIVE_TYPE_DIFFERENT_IQ = 0, /*!< Different IQ data from 2 parallel RAMs (4K words) */
    ADI_ADRV9001_RAM_DRIVE_TYPE_SAME_IQ      = 1, /*!< Same IQ data from 2 sequenced RAMs (8K words) */
} adi_adrv9001_RamDriveType_e;

/**
 * \brief RAM capture type enumeration 
 */
typedef enum adi_adrv9001_RamCaptureType
{
    ADI_ADRV9001_RAM_CAPTURE_TYPE_I_ONLY = 0, /*!< I data only */
    ADI_ADRV9001_RAM_CAPTURE_TYPE_Q_ONLY = 1, /*!< Q data only */
    ADI_ADRV9001_RAM_CAPTURE_TYPE_IQ     = 2, /*!< IQ data */
} adi_adrv9001_RamCaptureType_e;

/**
 * \brief RAM capture point enumeration 
 */
typedef enum adi_adrv9001_RamCapturePoint
{
    ADI_ADRV9001_RAM_CAPTURE_POINT_RK = 0, /*!< RX capture point RK */
    ADI_ADRV9001_RAM_CAPTURE_POINT_TH = 1, /*!< TX capture point TH */
    ADI_ADRV9001_RAM_CAPTURE_POINT_TG = 2, /*!< TX capture point TG */
} adi_adrv9001_RamCapturePoint_e;

/**
 * \brief RAM drive/capture mode enumeration 
 */
typedef enum adi_adrv9001_RamDriveCaptureMode
{
    ADI_ADRV9001_RAM_DRIVE_CAPTURE_DISABLED = 0, /*!< RAM drive/capture disabled */
    ADI_ADRV9001_RAM_DRIVE_ENABLED          = 1, /*!< RAM drive enabled */
    ADI_ADRV9001_RAM_CAPTURE_ENABLED        = 2, /*!< RAM capture enabled */
} adi_adrv9001_RamDriveCaptureMode_e;

/**
 * \brief RAM drive/capture mode enumeration 
 */
typedef enum adi_adrv9001_RamDriveCaptureStart
{
    ADI_ADRV9001_RAM_DRIVE_CAPTURE_STOP  = 0, /*!< RAM drive/capture stop */
    ADI_ADRV9001_RAM_DRIVE_CAPTURE_START = 1, /*!< RAM drive/capture start */
} adi_adrv9001_RamDriveCaptureStart_e;

/**
 * \brief Control out mux type enumeration 
 */
typedef enum adi_adrv9001_ControlOutMuxSel
{
    ADI_ADRV9001_CONTROL_OUT_MUX_RX  = 0, /*!< Control mux to select Rx observation signals */
    ADI_ADRV9001_CONTROL_OUT_MUX_TX = 1,  /*!< Control mux to select Tx observation signals */
    ADI_ADRV9001_CONTROL_OUT_MUX_MAIN = 2,/*!< Control mux to select main observation signals*/
} adi_adrv9001_ControlOutMuxSel_e;

/**
 * \brief ILB calibration tone selection status enumeration 
 */
typedef enum adi_adrv9001_IlbCalToneStatusSel
{
    ADI_ADRV9001_ILB_CAL_TONE_DISABLE = 0,  /*!< Disables calibration tone */
    ADI_ADRV9001_ILB_CAL_TONE_ENABLE  = 1,  /*!< Enables calibration tone */
    ADI_ADRV9001_ILB_CAL_TONE_UPDATE = 2,   /*!< Updates calibration tone frequency and attenuation */
} adi_adrv9001_IlbCalToneStatusSel_e;

/**
 * \brief PLL aatenuation enumeration 
 */
typedef enum adi_adrv9001_PllAtten
{
    ADI_ADRV9001_PLL_ATTEN_ZERO_DB  = 0,  /*!< PLL attenuation  0 dB */
    ADI_ADRV9001_PLL_ATTEN_5P25_DB  = 1,  /*!< PLL attenuation  5.25 dB */
    ADI_ADRV9001_PLL_ATTEN_8P25_DB  = 2,  /*!< PLL attenuation  8.25 dB */
    ADI_ADRV9001_PLL_ATTEN_10P25_DB = 3,  /*!< PLL attenuation 10.25 dB */
} adi_adrv9001_PllAtten_e;

/**
 * \brief PLL aatenuation enumeration 
 */
typedef enum adi_adrv9001_CalToneSwitchAtten
{
    ADI_ADRV9001_CAL_TONE_SWITCH_ATTEN_ZERO_DB  = 0,  /*!< Calibration tone attenuation  0 dB */
    ADI_ADRV9001_CAL_TONE_SWITCH_ATTEN_6_DB     = 1,  /*!< Calibration tone attenuation  6 dB */
    ADI_ADRV9001_CAL_TONE_SWITCH_ATTEN_12_DB    = 2,  /*!< Calibration tone attenuation 12 dB */
    ADI_ADRV9001_CAL_TONE_SWITCH_ATTEN_18_DB    = 3,  /*!< Calibration tone attenuation 18 dB */
} adi_adrv9001_CalToneSwitchAtten_e;

/**
 * \brief OBJ ID of init Cal and tracking cal algorithms 
 */
typedef enum adi_adrv9001_CalAlgoObjId
{
    ADI_ADRV9001_OBJID_IC_TX_QEC         = 0x20,
    ADI_ADRV9001_OBJID_IC_TX_LOL         = 0x21,
    ADI_ADRV9001_OBJID_IC_TX_LBPD        = 0x22,
    ADI_ADRV9001_OBJID_IC_TX_DCC         = 0x23,
    ADI_ADRV9001_OBJID_IC_TX_BBAF        = 0x24,
    ADI_ADRV9001_OBJID_IC_TX_BBAF_GD     = 0x25,
    ADI_ADRV9001_OBJID_IC_TX_ATTD        = 0x26,
    ADI_ADRV9001_OBJID_IC_TX_DAC         = 0x27,
    ADI_ADRV9001_OBJID_IC_TX_PD          = 0x28,
    ADI_ADRV9001_OBJID_IC_RX_HPADC_RC    = 0x29,
    ADI_ADRV9001_OBJID_IC_RX_HPADC_FLASH = 0x2A,
    ADI_ADRV9001_OBJID_IC_RX_HPADC_DAC   = 0x2B,
    ADI_ADRV9001_OBJID_IC_RX_DCC         = 0x2C,
    ADI_ADRV9001_OBJID_IC_RX_LPADC       = 0x2D,
    ADI_ADRV9001_OBJID_IC_RX_TIA_CUTOFF  = 0x2E,
    ADI_ADRV9001_OBJID_IC_RX_TIA_FINE    = 0x2F,
    ADI_ADRV9001_OBJID_IC_RX_TCAL        = 0x30,
    ADI_ADRV9001_OBJID_IC_RX_FIIC        = 0x31,
    ADI_ADRV9001_OBJID_IC_RX_ILB_LOD     = 0x32,
    ADI_ADRV9001_OBJID_IC_RX_RFDC_OFFSET = 0x33,
    ADI_ADRV9001_OBJID_IC_RX_GPD         = 0x34,
    ADI_ADRV9001_OBJID_IC_PLL            = 0x35,
    ADI_ADRV9001_OBJID_IC_AUXPLL         = 0x36,
    /* TODO: Add obj id of tracking cal algo once available */
} adi_adrv9001_CalAlgoObjId_e;

/**
 * \brief LO type enumeration 
 */
typedef enum adi_adrv9001_LoType
{
    ADI_ADRV9001_LO_OFFSET_FREQUENCY_ZERO = 0,       /*!< Carrier frequency equal to LO Frequency */
    ADI_ADRV9001_LO_OFFSET_FREQUENCY_NON_ZERO = 1,   /*!< Carrier frequency equal to LO Frequency + Offset Frequency */
} adi_adrv9001_LoType_e;

/*
*********************************************************************************************************
*                                             Structure definition
*********************************************************************************************************
*/

/**
* \brief Data structure to hold Internal RAM drrive and capture configurations
*/
typedef struct adi_adrv9001_RamDriveCaptureConfig
{
    adi_adrv9001_RamDriveCaptureMode_e ramDriveCaptureMode;
    adi_adrv9001_RamDriveType_e ramDriveType;
    adi_adrv9001_RamCaptureType_e ramCaptureType;
    adi_adrv9001_RamCapturePoint_e ramCapturePoint;
    adi_adrv9001_RamDriveCaptureStart_e ramDriveCaptureStart;
    uint16_t sampleNumber;
} adi_adrv9001_RamDriveCaptureConfig_t;

/**
* \brief Data structure to hold Control Out Mux configuration to be routed to GPIO for internal access/debug purpose
*/
typedef struct adi_adrv9001_ControlOutMuxCfg
{
    uint8_t controlOutMuxAddress;                     /*!< Address of the desired control out signal */
    adi_adrv9001_ControlOutMuxSel_e controlOutMuxSel; /*!< Type of control out mux */
} adi_adrv9001_ControlOutMuxCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_INTERNAL_UTILITIES_TYPES_H_ */