/**
 * \file:
 * \brief Functions to configure and control the FPGA9001 GPIO pins
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2018-2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_fpga9001_user.h"
#include "adi_fpga9001_gpio.h"
#include "adi_fpga9001.h"
#include "fpga9001_bf_axi_adrv9001.h"

static int32_t adi_fpga9001_gpio_Mode_SetValidate(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e gpioMode)
{
    ADI_RANGE_CHECK(device, gpioMode, ADI_FPGA9001_GPIO_MODE_NORMAL, ADI_FPGA9001_GPIO_MODE_JTAG);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_gpio_Mode_Set(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e gpioMode)
{
    ADI_PERFORM_VALIDATION(adi_fpga9001_gpio_Mode_SetValidate, device, gpioMode);
    ADI_EXPECT(fpga9001_AxiAdrv9001ModeBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, gpioMode);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_gpio_Mode_GetValidate(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e *gpioMode)
{
    ADI_NULL_PTR_RETURN(&device->common, gpioMode);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_gpio_Mode_Get(adi_fpga9001_Device_t *device, adi_fpga9001_GpioModes_e *gpioMode)
{
    uint8_t jtagStatus = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_gpio_Mode_GetValidate, device, gpioMode);

    ADI_EXPECT(fpga9001_AxiAdrv9001ModeBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &jtagStatus);

    *gpioMode = ADI_FPGA9001_GPIO_MODE_NORMAL;

    if (jtagStatus > 0)
    {
        *gpioMode = ADI_FPGA9001_GPIO_MODE_JTAG;
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_gpio_ReadValidate(adi_fpga9001_Device_t *device, uint16_t *rdData)
{
    ADI_NULL_PTR_RETURN(&device->common, rdData);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_gpio_Read(adi_fpga9001_Device_t *device, uint16_t *rdData)
{
    ADI_PERFORM_VALIDATION(adi_fpga9001_gpio_ReadValidate, device, rdData);

    ADI_EXPECT(fpga9001_AxiAdrv9001DgpioPioDataBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, rdData);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_gpio_pin_Configure_Validate(adi_fpga9001_Device_t *device,
                                                        adi_fpga9001_GpioPin_e pin,
                                                        adi_fpga9001_GpioSourceModes_e sourceMode)
{
    ADI_RANGE_CHECK(device, pin, ADI_FPGA9001_GPIO_00, ADI_FPGA9001_GPIO_11);
    ADI_RANGE_CHECK(device, 
                    sourceMode,
                    ADI_FPGA9001_GPIO_SOURCE_MODE_INPUT,
                    (ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_TDD_CTL1_CONTROL2_ENABLE | ADI_FPGA9001_GPIO_SOURCE_MODE_OUTPUT_CONSTANT_ZERO));
    if (sourceMode == ADI_FPGA9001_GPIO_SOURCE_MODE_RESERVED)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FULL,
                         sourceMode,
                         "Selected GPIO source mode is not valid");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_gpio_pin_Configure(adi_fpga9001_Device_t *device,
                                        adi_fpga9001_GpioPin_e pin,
                                        adi_fpga9001_GpioSourceModes_e sourceMode)
{
    static int32_t(*const DgpioPioctlBfSet[])(adi_fpga9001_Device_t*, fpga9001_BfAxiAdrv9001ChanAddr_e, uint8_t) = { 
        fpga9001_AxiAdrv9001DgpioPioctl00BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl01BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl02BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl03BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl04BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl05BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl06BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl07BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl08BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl09BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl10BfSet,
        fpga9001_AxiAdrv9001DgpioPioctl11BfSet,
    };
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_gpio_pin_Configure_Validate, device, pin, sourceMode);
    
    ADI_EXPECT(DgpioPioctlBfSet[pin], device, FPGA9001_BF_AXI_ADRV9001_TOP, sourceMode);
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_gpio_pin_Inspect_Validate(adi_fpga9001_Device_t *device, 
                                                      adi_fpga9001_GpioPin_e pin,
                                                      adi_fpga9001_GpioSourceModes_e *sourceMode)
{
    ADI_RANGE_CHECK(device, pin, ADI_FPGA9001_GPIO_00, ADI_FPGA9001_GPIO_11);
    ADI_NULL_PTR_RETURN(&device->common, sourceMode);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_gpio_pin_Inspect(adi_fpga9001_Device_t *device, 
                                      adi_fpga9001_GpioPin_e pin,
                                      adi_fpga9001_GpioSourceModes_e *sourceMode)
{
    static int32_t(*const DgpioPioctlBfGet[])(adi_fpga9001_Device_t*, fpga9001_BfAxiAdrv9001ChanAddr_e, uint8_t*) = { 
        fpga9001_AxiAdrv9001DgpioPioctl00BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl01BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl02BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl03BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl04BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl05BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl06BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl07BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl08BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl09BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl10BfGet,
        fpga9001_AxiAdrv9001DgpioPioctl11BfGet,
    };
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_gpio_pin_Inspect_Validate, device, pin, sourceMode);
    
    ADI_EXPECT(DgpioPioctlBfGet[pin], device, FPGA9001_BF_AXI_ADRV9001_TOP, (uint8_t *)sourceMode);
    ADI_API_RETURN(device);
}

static uint8_t adi_fpga9001_gpio_pin_SourceMode_Configure_Validate(adi_fpga9001_Device_t *device,
                                                                   adi_fpga9001_Gpio_PinCfg_t *gpioPinConfig,
                                                                   uint8_t length)
{
    ADI_NULL_PTR_RETURN(&device->common, gpioPinConfig);

    ADI_RANGE_CHECK(device, length, 1, ADI_FPGA9001_NUM_GPIO_PINS);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_gpio_pin_SourceMode_Configure(adi_fpga9001_Device_t *device, adi_fpga9001_Gpio_PinCfg_t *gpioPinConfig, uint8_t length)
{
    uint8_t i = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_gpio_pin_SourceMode_Configure_Validate, device, gpioPinConfig, length);

    for (i = 0; i < length; i++)
    {
        ADI_EXPECT(adi_fpga9001_gpio_pin_Configure, device, gpioPinConfig->gpioPins[i], gpioPinConfig->gpioSourceMode[i]);
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_gpio_pin_SourceMode_Inspect_Validate(adi_fpga9001_Device_t *device,
                                                                 adi_fpga9001_Gpio_PinCfg_t *gpioPinConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, gpioPinConfig);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_gpio_pin_SourceMode_Inspect(adi_fpga9001_Device_t *device,
                                                 adi_fpga9001_Gpio_PinCfg_t *gpioPinConfig)
{
    uint8_t i = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_gpio_pin_SourceMode_Inspect_Validate, device, gpioPinConfig);

    for (i = 0; i < ADI_FPGA9001_NUM_GPIO_PINS; i++)
    {
        ADI_EXPECT(adi_fpga9001_gpio_pin_Inspect, device, gpioPinConfig->gpioPins[i], &gpioPinConfig->gpioSourceMode[i]);
    }

    ADI_API_RETURN(device);
}
