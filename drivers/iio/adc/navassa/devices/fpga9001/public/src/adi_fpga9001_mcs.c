/**
* \file
* \brief FPGA9001 Multi-Chip Synchronization (MCS) functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/
#include "adi_adrv9001_user.h"
#include "adi_fpga9001_mcs.h"
#include "fpga9001_bf_axi_adrv9001.h"


static int32_t adi_fpga9001_Mcs_Start_Validate(adi_fpga9001_Device_t *device, 
                                               uint8_t numberOfPulses, 
                                               uint8_t mcsPeriod,
                                               uint8_t mcsPulseWidth,
                                               adi_fpga9001_McsEdge_e edge)
{
    /* TODO: Should max be 6 (most required by ADRV9001) or 15 (most FPGA can do) */
    ADI_RANGE_CHECK(device, numberOfPulses, 1, 15);
    
    ADI_RANGE_CHECK(device, mcsPulseWidth, 1, 7);
    
    ADI_RANGE_CHECK(device, edge, ADI_FPGA9001_MCS_EDGE_RISING, ADI_FPGA9001_MCS_EDGE_FALLING);
    
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Mcs_Start(adi_fpga9001_Device_t *device, 
                               uint8_t numberOfPulses, 
                               uint8_t mcsPeriod,
                               uint8_t mcsPulseWidth,
                               adi_fpga9001_McsEdge_e edge)
{
    uint8_t mcsInProgress = 1;
    uint8_t ii = 0;
    int32_t halError = 0;
    static const uint8_t MAX_ITER = 100;
    static const uint16_t WAIT_INTERVAL_US = 1000;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_Mcs_Start_Validate, device, numberOfPulses, mcsPeriod, mcsPulseWidth, edge);
    
    /* Wait for any previous MCS sequences to complete */
    while (0 != mcsInProgress && ii < MAX_ITER)
    {
        ADI_EXPECT(fpga9001_AxiAdrv9001McsStatusBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &mcsInProgress);
        halError = adi_common_hal_Wait_us(&device->common, WAIT_INTERVAL_US);
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_ADI_HAL,
                         halError,
                         ADI_COMMON_ACT_ERR_CHECK_TIMER,
                         device,
                         "Timer not working");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    
    if (1 == mcsInProgress)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_RESET_FEATURE,
                         device,
                         "Timed out waiting for previous MCS sequence to complete");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    
    /* Configure the sequence */
    ADI_EXPECT(fpga9001_AxiAdrv9001McsNumBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, numberOfPulses);
    ADI_EXPECT(fpga9001_AxiAdrv9001McsPeriodBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, mcsPeriod);
    ADI_EXPECT(fpga9001_AxiAdrv9001McsWidthBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, mcsPulseWidth);
    ADI_EXPECT(fpga9001_AxiAdrv9001McsEdgeBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, edge);
    
    /* Start the sequence */
    ADI_EXPECT(fpga9001_AxiAdrv9001McsRequestBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, true);
    ADI_EXPECT(fpga9001_AxiAdrv9001McsRequestBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, false);
    
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Mcs_Requested_Validate(adi_fpga9001_Device_t *device, bool *requested)
{
    ADI_NULL_PTR_RETURN(&device->common, requested);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Mcs_Requested(adi_fpga9001_Device_t *device, bool *requested)
{
    uint8_t mcsRequested = 0;
    ADI_PERFORM_VALIDATION(adi_fpga9001_Mcs_Requested_Validate, device, requested);
    ADI_EXPECT(fpga9001_AxiAdrv9001GpintSbBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &mcsRequested);
    
    *requested = (bool)mcsRequested;
    if (*requested)
    {
        /* Must write 1 to manually clear sticky bit */
        ADI_EXPECT(fpga9001_AxiAdrv9001GpintSbBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, true);
    }
    
    ADI_API_RETURN(device);
}
