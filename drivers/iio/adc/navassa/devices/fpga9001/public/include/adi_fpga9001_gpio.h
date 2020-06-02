/**
 * \file
 * \brief Functions to configure and control the FPGA9001 GPIO pins
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */
/**
 * Copyright 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_GPIO_H_
#define _ADI_FPGA9001_GPIO_H_

#include "adi_fpga9001_gpio_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Set the gpioMode
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] gpioMode          The mode to set
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_gpio_Mode_Set(adi_fpga9001_Device_t *fpga9001, adi_fpga9001_GpioModes_e gpioMode);

/**
 * \brief Get the current gpioMode
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[out] gpioMode         The current mode
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_gpio_Mode_Get(adi_fpga9001_Device_t *fpga9001, adi_fpga9001_GpioModes_e *gpioMode);

/**
 * \brief Read the values on the GPIO input register
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[out] rdData           The current input values
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_gpio_Read(adi_fpga9001_Device_t *fpga9001, uint16_t *rdData);

/**
 * \brief Configure a single GPIO pin
 * 
 * \param[in] fpga9001      Context variable - Pointer to FPGA9001 device data structure
 * \param[in] pin           The pin to configure
 * \param[in] sourceMode    The desired source mode
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_gpio_pin_Configure(adi_fpga9001_Device_t *fpga9001,
                                        adi_fpga9001_GpioPin_e pin,
                                        adi_fpga9001_GpioSourceModes_e sourceMode);

/**
 * \brief Inspect a single GPIO pin
 * 
 * \param[in]  fpga9001     Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  pin          The pin to inspect
 * \param[out] sourceMode   The current source mode
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_gpio_pin_Inspect(adi_fpga9001_Device_t *fpga9001,
                                      adi_fpga9001_GpioPin_e pin,
                                      adi_fpga9001_GpioSourceModes_e *sourceMode);
    
/**
 * \brief Configures the specified FPGA9001 GPIO pin with the desired source mode
 *
 * \param[in] fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in] gpioPinConfig    GPIO pin configurations to write
 * \param[in] length           Number of GPIO pin configurations to write
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_gpio_pin_SourceMode_Configure(adi_fpga9001_Device_t *fpga9001,
                                                   adi_fpga9001_Gpio_PinCfg_t *gpioPinConfig,
                                                   uint8_t length);

/**
 * \brief Inspect the FPGA9001 GPIO pins
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[out] gpioPinConfig    GPIO pin configurations to write
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_gpio_pin_SourceMode_Inspect(adi_fpga9001_Device_t *fpga9001,
                                                 adi_fpga9001_Gpio_PinCfg_t *gpioPinConfig);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_GPIO_H_ */