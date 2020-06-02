/**
* \file
* \brief Contains FPGA9001 GPIO enum and struct definitions
*
* ADRV9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */


#ifndef _ADI_FPGA9001_GPIO_TYPES_H_
#define _ADI_FPGA9001_GPIO_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADI_FPGA9001_NUM_GPIO_PINS 12

/**
 * \brief Possible modes of operation for FPGA9001 GPIO
 */
typedef enum adi_fpga9001_GpioModes
{
    ADI_FPGA9001_GPIO_MODE_NORMAL = 0,
    ADI_FPGA9001_GPIO_MODE_JTAG,
} adi_fpga9001_GpioModes_e;

/**
 * \brief Possible source select modes for FPGA9001 GPIO
 */
typedef enum adi_fpga9001_GpioSourceModes
{
    ADI_FPGA9001_GPIO_SOURCE_MODE_INPUT                             = 0x00,
    ADI_FPGA9001_GPIO_SOURCE_MODE_INPUT_GAIN_CHANGE_INDICATOR       = 0x01,
    ADI_FPGA9001_GPIO_SOURCE_MODE_INPUT_GAIN_INDEX_DATA             = 0x02,
    ADI_FPGA9001_GPIO_SOURCE_MODE_RESERVED                          = 0x03,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_TX1_ENABLE             = 0x04,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_RX1_ENABLE             = 0x05,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_ORX1_ENABLE            = 0x06,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_AUX1_ENABLE            = 0x07,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_TX2_ENABLE             = 0x08,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_RX2_ENABLE             = 0x09,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_ORX2_ENABLE            = 0x0A,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_AUX2_ENABLE            = 0x0B,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_CTL0_CONTROL1_ENABLE   = 0x0C,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_CTL0_CONTROL2_ENABLE   = 0x0D,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_CTL1_CONTROL1_ENABLE   = 0x0E,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_CTL1_CONTROL2_ENABLE   = 0x0F,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_CONSTANT_ZERO              = 0x10,
    ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_CONSTANT_ONE               = 0x11
} adi_fpga9001_GpioSourceModes_e;

/**
 * \brief FPGA9001 GPIO pin selections
 */
typedef enum adi_fpga9001_GpioPin
{
    ADI_FPGA9001_GPIO_00,
    ADI_FPGA9001_GPIO_01,
    ADI_FPGA9001_GPIO_02,
    ADI_FPGA9001_GPIO_03,
    ADI_FPGA9001_GPIO_04,
    ADI_FPGA9001_GPIO_05,
    ADI_FPGA9001_GPIO_06,
    ADI_FPGA9001_GPIO_07,
    ADI_FPGA9001_GPIO_08,
    ADI_FPGA9001_GPIO_09,
    ADI_FPGA9001_GPIO_10,
    ADI_FPGA9001_GPIO_11
} adi_fpga9001_GpioPin_e;

/**
 * \brief The desired configuration to assign FPGA GPIO pins as digital GPIO
 */
typedef struct adi_fpga9001_Gpio_PinCfg
{
    adi_fpga9001_GpioPin_e gpioPins[ADI_FPGA9001_NUM_GPIO_PINS]; /*!< FPGA GPIO Pins to be used to route the signals from/to ADRV9001 */
    adi_fpga9001_GpioSourceModes_e gpioSourceMode[ADI_FPGA9001_NUM_GPIO_PINS]; /*!< source select mode for the specified FPGA9001 GPIO */
} adi_fpga9001_Gpio_PinCfg_t;

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_GPIO_TYPES_H_ */