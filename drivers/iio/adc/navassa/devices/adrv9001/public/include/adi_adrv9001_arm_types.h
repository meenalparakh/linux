/**
 * \file
 * \brief Contains ADRV9001 ARM data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_ARM_TYPES_H_
#define _ADI_ADRV9001_ARM_TYPES_H_

#include "adi_adrv9001_stream_types.h"
#include "adi_adrv9001_gpio_types.h"

/* System includes */
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief Enumerated list of ARM System States.
*/
typedef enum adi_adrv9001_ArmSystemStates
{
    ADI_ADRV9001_ARM_SYSTEM_POWERUP,        /*!< Powerup State */
    ADI_ADRV9001_ARM_SYSTEM_NORMALMODE,     /*!< Normal TDD/FDD State */
    ADI_ADRV9001_ARM_SYSTEM_POWERSAVINGMODE /*!< System Power Saving Mode */
} adi_adrv9001_ArmSystemStates_e;

/**
* \brief Enumerated list of ARM Monitor Mode States.
*
* Only valid when System State is in monitor mode
*/
typedef enum adi_adrv9001_ArmMonitorModeStates
{
    ADI_ADRV9001_ARM_MONITORMODE_SLEEP,     /*!< Sleep state in monitor mode */
    ADI_ADRV9001_ARM_MONITORMODE_DETECTING, /*!< Detecting state in monitor mode */
    ADI_ADRV9001_ARM_MONITORMODE_DETECTED,  /*!< Detected state in monitor mode */
} adi_adrv9001_ArmMonitorModeStates_e;

/**
* \brief Enumerated list of ARM bootup states.
*/
typedef enum adi_adrv9001_ArmBootStates
{
    ADI_ADRV9001_ARM_BOOT_POWERUP                           = 0,    /*!< Used to put API in wait for ARM state */
    ADI_ADRV9001_ARM_BOOT_READY                             = 1,    /*!< ARM booted with no failure */
    ADI_ADRV9001_ARM_BOOT_FW_CHECKSUM_ERR                   = 2,    /*!< ARM firmware checksum error */
    ADI_ADRV9001_ARM_BOOT_DATAMEM_ERR                       = 3,    /*!< ARM data memory error */
    ADI_ADRV9001_ARM_BOOT_STREAM_IMG_CHECKSUM_ERR           = 4,    /*!< Stream image checksum error */
    ADI_ADRV9001_ARM_BOOT_DEVICE_PROFILE_CHECKSUM_ERR       = 5,    /*!< Device profile checksum error */
    ADI_ADRV9001_ARM_BOOT_CLKGEN_ERR                        = 6,    /*!< Bootup clkgen setup error */
    ADI_ADRV9001_ARM_BOOT_CLKSSI_ERR                        = 7,    /*!< Bootup SSI setup error */
    ADI_ADRV9001_ARM_BOOT_POWER_INIT_ERR                    = 8,    /*!< Power init setup error */
    ADI_ADRV9001_ARM_BOOT_JTAG_BUILD_STATUS_READY           = 9,    /*!< JTAG build status ready indication */
    ADI_ADRV9001_ARM_BOOT_CLKLOGEN_ERR                      = 10,   /*!< Bootup clock LOGEN error */
    ADI_ADRV9001_ARM_BOOT_RXQECHW_ERR                       = 11,   /*!< Error initializing RxQEC hardware */
    ADI_ADRV9001_ARM_BOOT_HM_TIMER_ERR                      = 12,   /*!< Failed to create health monitor timers */
    ADI_ADRV9001_ARM_BOOT_ADC_RCAL_ERR                      = 13,   /*!< ADC RCAL error */
    ADI_ADRV9001_ARM_BOOT_STREAM_RUNTIME_ERR                = 14,   /*!< Stream runtime error */
    ADI_ADRV9001_ARM_BOOT_CLKGEN_RCAL_ERR                   = 15,   /*!< Bootup clkgen setup after RCAL error */
} adi_adrv9001_ArmBootStates_e;

/**
* \brief Enumerated list of ARM image build options.
*/
typedef enum adi_adrv9001_ArmBuildType
{
    ADI_ADRV9001_ARMBUILD_RELEASE = 0,
    ADI_ADRV9001_ARMBUILD_DEBUG = 1,
    ADI_ADRV9001_ARMBUILD_TESTOBJ = 2
} adi_adrv9001_ArmBuildType_e;

/**
 * \brief Enumerated list of ARM Clock Type options.
 */
typedef enum adi_adrv9001_ArmClockType
{
    ADI_ADRV9001_ARMCLOCK_DATAPATH = 0,
    ADI_ADRV9001_ARMCLOCK_MAX
} adi_adrv9001_ArmClockType_e;

/**
 * \brief Per-channel power down modes
 */
typedef enum adi_adrv9001_ChannelPowerDownMode
{
    ADI_ADRV9001_CHANNEL_POWER_DOWN_MODE_DISABLED = 0,  /*!< Default radio operation, no extra power down */
    ADI_ADRV9001_CHANNEL_POWER_DOWN_MODE_RFPLL = 1,     /*!< RF PLL power down */
    ADI_ADRV9001_CHANNEL_POWER_DOWN_MODE_LDO = 2,       /*!< Channel LDO power down */
} adi_adrv9001_ChannelPowerDownMode_e;

/**
 * \brief System level power down modes
 */
typedef enum adi_adrv9001_SystemPowerDownMode
{
    ADI_ADRV9001_SYSTEM_POWER_DOWN_MODE_CLKPLL = 3,     /*!< CLK PLL and LDO power down */
    ADI_ADRV9001_SYSTEM_POWER_DOWN_MODE_ARM = 4         /*!< ARM power down */
} adi_adrv9001_SystemPowerDownMode_e;

/**
 * \brief Monitor modes.
 */
typedef enum adi_adrv9001_MonitorDetectionMode
{
    ADI_ADRV9001_MONITOR_DETECTION_MODE_RSSI,        /*!< RSSI detection only */
    ADI_ADRV9001_MONITOR_DETECTION_MODE_SYNC,        /*!< Sync detection only */
    ADI_ADRV9001_MONITOR_DETECTION_MODE_FFT,         /*!< FFT detection only */
    ADI_ADRV9001_MONITOR_DETECTION_MODE_RSSI_SYNC,   /*!< RSSI and Sync detection */
    ADI_ADRV9001_MONITOR_DETECTION_MODE_RSSI_FFT,    /*!< RSSI and FFT detection */
} adi_adrv9001_MonitorDetectionMode_e;

/**
* \brief Data structure to hold ARM version information
*/
typedef struct adi_adrv9001_ArmVersion
{
    uint8_t majorVer;
    uint8_t minorVer;
    uint8_t maintVer;
    uint8_t rcVer;
    adi_adrv9001_ArmBuildType_e armBuildType;
} adi_adrv9001_ArmVersion_t;

/**
* \brief Data structure to hold ARM image build and runtime calculated checksums.
*/
typedef struct adi_adrv9001_Checksum
{
    uint32_t buildChecksum;
    uint32_t runChecksum;
} adi_adrv9001_Checksum_t;

/**
* \brief Data structure to hold a table of additional ARM checksums
*/
typedef struct adi_adrv9001_ChecksumTable
{
    adi_adrv9001_Checksum_t fwCheckSums;
    adi_adrv9001_Checksum_t streamsCheckSum[ADRV9001_MAX_NUM_STREAM];
    adi_adrv9001_Checksum_t deviceProfileCheckSum;
    adi_adrv9001_Checksum_t adcProfilefwCheckSum;
    uint32_t fwError;
    uint32_t fwErrorCode;
} adi_adrv9001_ChecksumTable_t;

/**
 * \brief Data structure to hold ARM clock information
 */
typedef struct adi_adrv9001_ArmClock
{
    uint8_t channelMask;                     /* !< The mask of Tx / Rx channels */
    uint8_t clockEnable;                     /* !< Clock Diasble(0) or Enable(1) */
    adi_adrv9001_ArmClockType_e clockType;   /* !< Clock type */
} adi_adrv9001_ArmClock_t;

/**
 * \brief Data structure to hold power saving configuration parameters
 */
typedef struct adi_adrv9001_ChannelPowerSavingCfg
{
    adi_adrv9001_ChannelPowerDownMode_e channelDisabledPowerDownMode;   /*!< Pin power down mode */
    /** Additional GPIO pin power down mode; only valid if greater than channelDisabledPowerDownMode */
    adi_adrv9001_ChannelPowerDownMode_e gpioPinPowerDownMode;
} adi_adrv9001_ChannelPowerSavingCfg_t;

/**
 * \brief Data structure to hold monitor mode configuration parameters
 */
typedef struct adi_adrv9001_SystemPowerSavingAndMonitorModeCfg
{
    adi_adrv9001_SystemPowerDownMode_e powerDownMode;   /*!< Power down mode for Monitor */
    uint32_t initialBatterySaverDelay_us;               /*!< Delay Timer (us) for detection before entering Monitor Mode  */
    uint32_t detectionTime_us;                          /*!< Timer for detection state (us) */
    uint32_t sleepTime_us;                              /*!< Timer for sleep state (us) */
    uint8_t detectionFirst;                             /*!< Select first state when entering Monitor Mode, 0-sleep first, 1-detection first */
    adi_adrv9001_MonitorDetectionMode_e detectionMode;  /*!< Mode of detection in the detect state */
    bool detectionDataBufferEnable;                     /*!< Enable data buffer in detect mode, 0-disable, 1-enable */
    bool externalPllEnable;                             /*!< External PLL Enable, 0-disable, 1-enable */
} adi_adrv9001_SystemPowerSavingAndMonitorModeCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_ARM_TYPES_H_ */
