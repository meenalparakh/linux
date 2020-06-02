/**
* \file
* \brief FPGA9001 specific errors
*
* Extension of adi_common_error
*
* API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADI API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_FPGA9001_ERROR_H_
#define _ADI_FPGA9001_ERROR_H_

#include "adi_common_error.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* Extending Common error for FPGA9001 */
#define ADI_FPGA9001_ERR_DRP_ERROR                      (ADI_COMMON_ERR_EXTENSION_2 + 0x01)
#define ADI_FPGA9001_ERR_CHANNEL_POWER_GOOD_FAILURE     (ADI_COMMON_ERR_EXTENSION_2 + 0x02)
#define ADI_FPGA9001_ERR_RX_DATA_MOVER_WAIT_TIMEOUT     (ADI_COMMON_ERR_EXTENSION_2 + 0x03)     /*!< API NG - Timeout occurred during Rx transaction */
#define ADI_FPGA9001_ERR_INVALID_FPGA_IMAGE             (ADI_COMMON_ERR_EXTENSION_2 + 0x04)

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_ERROR_H_ */