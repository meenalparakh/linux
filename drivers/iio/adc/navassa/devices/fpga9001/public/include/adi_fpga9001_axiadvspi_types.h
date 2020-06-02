/**
* \file
* \brief Types for AXI Advanced SPI
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_AXI_ADV_SPI_TYPES_H_
#define _ADI_FPGA9001_AXI_ADV_SPI_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct adi_fpga9001_AxiAdvSpiConfig
{
    uint16_t clockDiv;                  /*!< Advanced Feature Set Clock Divider.*/
    uint8_t threeWireTriPoint;          /*!< Advanced Feature Set Three Wire Tristate Point.*/
    uint8_t misoSampleSlip;             /*!< Advanced Feature Set MISO Sample Slip.*/
    uint8_t fourWire;                   /*!< Advanced Feature Set Four Wire Mode.*/
    uint8_t lsbFirst;                   /*!< Advanced Feature Set LSB First.*/
    uint8_t cpha;                       /*!< Advanced Feature Set Clock Phase.*/ 
    uint8_t cpol;                       /*!< Advanced Feature Set Clock Polarity.*/ 
    uint8_t enable;                     /*!< Advanced Feature Set Enable.*/ 
} adi_fpga9001_AxiAdvSpiConfig_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_AXI_ADV_SPI_TYPES_H_ */
