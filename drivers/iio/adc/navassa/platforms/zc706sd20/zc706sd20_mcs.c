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
#include "zc706sd20_mcs.h"
#include "adi_fpga9001_mcs.h"

int32_t zc706sd20_Mcs_Pulse(void* devHalCfg, uint8_t numberOfPulses)
{
    static const uint8_t MCS_PERIOD = 10; /* TODO: Value TBD by characterization */
    static const uint8_t MCS_WIDTH = 4; /* TODO: Value TBD by characterization */
    static const adi_fpga9001_McsEdge_e MCS_EDGE = ADI_FPGA9001_MCS_EDGE_RISING; /* TODO: Value TBD by characterization */
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
    
    return adi_fpga9001_Mcs_Start(halCfg->fpga9001, numberOfPulses, MCS_PERIOD, MCS_WIDTH, MCS_EDGE);
}