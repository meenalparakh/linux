/**
 * \file
 * \brief Contains private fpga9001 utility related functions
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
* Copyright 2015 - 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "fpga9001_utilities.h"

int32_t fpga9001_Utilities_CalculateLcm(adi_fpga9001_Device_t *fpga9001,
                                        uint32_t number1,
                                        uint32_t number2,
                                        uint32_t *lcm)
{
    uint32_t num1 = 0;
    uint32_t num2 = 0;
    uint8_t i = 0;
    
    *lcm = 0;
    
    if (number1 == 0)
    {
        ADI_ERROR_REPORT(&fpga9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         number1,
                         "Cannot take LCM with first number equal to 0");
    }
    
    if (number2 == 0)
    {
        ADI_ERROR_REPORT(&fpga9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         number2,
                         "Cannot take LCM with second number equal to 0");
    }

    if (number1 > number2)
    {
        num1 = number1;
        num2 = number2;
    }
    else
    {
        num1 = number2;
        num2 = number1;
    }

    for (i = 1; i < num2; i++)
    {
        if (((num1 * i) % num2) == 0)
        {
            *lcm = num1 * i;
            ADI_API_RETURN(fpga9001);

        }
    }

    *lcm = num1 * num2;
    ADI_API_RETURN(fpga9001);
}
