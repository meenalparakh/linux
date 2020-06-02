/**
 * \file
 * Analog Devices ZC706 Platform + microzed hardware abstraction layer
 */

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_platform.h"
#include "zc706sd20_ssi.h"
#include "adi_fpga9001_ssi.h"

int32_t zc706sd20_ssi_Reset(void* devHalCfg)
{
    adi_hal_Cfg_t *halCfg = NULL;
    
    if (NULL == devHalCfg)
    {
        return ADI_HAL_NULL_PTR;
    }
    
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    if (NULL == halCfg->fpga9001)
    {
        return ADI_HAL_NULL_PTR;
    }
    
    return adi_fpga9001_ssi_Reset(halCfg->fpga9001);
}