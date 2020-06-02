/**
* \file
* \brief Types related to the power monitoring chips on the CE board
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001CE01_POWERMONITOR_TYPES_H_
#define _ADI_ADRV9001CE01_POWERMONITOR_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Available power monitoring chips by associated I2C slave address
 */
typedef enum adi_adrv9001Ce01_PowerMonitor
{
    ADI_ADRV9001CE01_POWERMONITOR_VDD_1P0   = 0x30,     /*!< VDD 1.0V domain */
    ADI_ADRV9001CE01_POWERMONITOR_VDDA_1P0  = 0x31,     /*!< VDDA 1.0V domain */
    ADI_ADRV9001CE01_POWERMONITOR_VDD_1P8   = 0x32,     /*!< VDD 1.8V domain */
    ADI_ADRV9001CE01_POWERMONITOR_VDDA_1P8  = 0x33,     /*!< VDDA 1.8V domain */
    ADI_ADRV9001CE01_POWERMONITOR_VDDA_1P3  = 0x3B      /*!< VDDA 1.3V domain */
} adi_adrv9001Ce01_PowerMonitor_e;

#ifdef __cplusplus
}
#endif

#endif
