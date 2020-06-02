/**
* \file
* \brief Functions to configure the FPGA9001 AXI Advanced SPI
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_AXI_ADV_SPI_H_
#define _ADI_FPGA9001_AXI_ADV_SPI_H_

#include "adi_fpga9001_axiadvspi_types.h"
#include <stdint.h>
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure the AXI Advanced SPI
 *
 * \param[in] fpga9001      Context variable - Pointer to FPGA9001 device data structure
 * \param[in] advSpiCfg     The desired configuration
 * \param[in] slaveSelect   The SPI interface to configure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_AxiAdvSpi_Configure(adi_fpga9001_Device_t *fpga9001,
                                         adi_fpga9001_AxiAdvSpiConfig_t *advSpiCfg,
                                         uint8_t slaveSelect);

/**
 * \brief Select all slave devices
 *
 * \param[in] fpga9001  Context variable - Pointer to FPGA9001 device data structure
 * \param[in] enable    Whether to select (1) all slave devices or not (0)
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_AxiAdvSpi_SelectAllChips(adi_fpga9001_Device_t *fpga9001, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_AXI_ADV_SPI_H_ */