/**
 * \file
 * \brief Contains internal use only BBDC engineering declarations
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_INTERNAL_BBDC_H_
#define _ADI_ADRV9001_INTERNAL_BBDC_H_

#include <stdbool.h>
#include "adi_common.h"
#include "adi_common_macros.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief Set the disable for the Rx Base-band DC rejection for the given channel.
*
* \pre Channel state any of CALIBRATED, PRIMED, RF_ENABLED
*
* \param[in] adrv9001		Context variable - Pointer to the ADRV9001 device data structure containing settings
* \param[in] channel		The channel of the Rx port to enable
* \param[in] disable		The disable for the Rx Base-band DC rejection algorithm
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_bbdc_TestDisable_Set(adi_adrv9001_Device_t *adrv9001, 
                                                   adi_common_ChannelNumber_e channel,
                                                   bool disable);

/**
* \brief Get the disable for the Rx Base-band DC rejection for the given channel.
*
* \pre Channel state any of STANDBY, CALIBRATED, PRIMED, RF_ENABLED
*
* \param[in] adrv9001		Context variable - Pointer to the ADRV9001 device data structure containing settings
* \param[in] channel		The channel of the Rx port to enable
* \param[out] disable		The disable for the Rx Base-band DC rejection algorithm
*
* \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv9001_internal_bbdc_TestDisable_Get(adi_adrv9001_Device_t *adrv9001, 
                                                   adi_common_ChannelNumber_e channel,
                                                   bool *disable);
#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_INTERNAL_BBDC_H_ */

