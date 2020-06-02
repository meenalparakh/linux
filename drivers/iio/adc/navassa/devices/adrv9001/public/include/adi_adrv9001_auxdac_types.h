/**
 * \file
 * \brief Contains functions to configure AUX DAC channels on the ADRV9001 device
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_AUXDAC_TYPES_H_
#define _ADI_ADRV9001_AUXDAC_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             ENUMs
*********************************************************************************************************
*/

/**
*  \brief Enum to select AuxDAC
*/
typedef enum adi_adrv9001_AuxDacs
{
    ADI_ADRV9001_AUXDAC0 = 0x01, /*!< AuxDAC0 */
    ADI_ADRV9001_AUXDAC1 = 0x02, /*!< AuxDAC1 */
    ADI_ADRV9001_AUXDAC2 = 0x04, /*!< AuxDAC2 */
    ADI_ADRV9001_AUXDAC3 = 0x08, /*!< AuxDAC3 */
} adi_adrv9001_AuxDacs_e;

/*
*********************************************************************************************************
*                                             Structure definition
*********************************************************************************************************
*/


#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_AUXDAC_TYPES_H_ */
