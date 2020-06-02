/**
 * \file
 * \brief Contains top level fpga9001 MMCM related functions
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include <stdbool.h>
#include "adi_adrv9001_user.h"
#include "adi_common_error.h"
#include "adi_common_macros.h"
#include "adi_fpga9001_mmcm.h"

#include "fpga9001_utilities.h"
#include "fpga9001_bf_axi_adrv9001.h"

static int32_t adi_fpga9001_ClkFbOutReg_Calculate(adi_fpga9001_Device_t *device,
                                                  uint32_t adrv9001DeviceClockOut_kHz,
                                                  uint32_t lcm,
                                                  uint8_t  *mVal,
                                                  uint16_t *bfValue1,
                                                  uint16_t *bfValue2)
{
    uint8_t edgeSelect = 0;
    uint8_t lowCount = 0;
    uint8_t highCount = 0;
    uint8_t bypassDivider = 0;

    uint16_t clkFbOutVal = 0;
    uint16_t clkReg1 = 0;
    uint16_t clkReg2 = 0;

    static uint8_t MMCM_CLKFBOUT_M_VAL_MIN = 1;
    static uint8_t MMCM_CLKFBOUT_M_VAL_MAX = 126;

    static uint32_t MMCM_FVCO_MAX = 1400000000;

    clkFbOutVal = (uint16_t)((lcm / adrv9001DeviceClockOut_kHz) * (MMCM_FVCO_MAX / lcm));
    if (clkFbOutVal > 126)
    {
        clkFbOutVal = 126;
    }

    /* clkFbOutVal must be between 1 and 126 */
    ADI_RANGE_CHECK(device, clkFbOutVal, MMCM_CLKFBOUT_M_VAL_MIN, MMCM_CLKFBOUT_M_VAL_MAX);

    edgeSelect = (clkFbOutVal & 1) ? 1 : 0;  /* if odd, edgeSelect = 1 */
    highCount = ((clkFbOutVal >> 1) & 0x3F); /* high count is divider/2 */
    lowCount = (edgeSelect == 1) ? ((highCount + 1) & 0x3F) : (highCount & 0x3F);
    bypassDivider = (clkFbOutVal == 1) ? 1 : 0;

    clkReg1 |= ((highCount << 6) | (lowCount));
    clkReg2 |= ((edgeSelect << 7) | (bypassDivider << 6));

    *mVal = (uint8_t)clkFbOutVal;
    *bfValue1 = clkReg1;
    *bfValue2 = clkReg2;

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ClkOutReg_Calculate(adi_fpga9001_Device_t *device,
                                                uint8_t targetClockDivisor,
                                                uint16_t *bfValue1,
                                                uint16_t *bfValue2)
{
    uint8_t edgeSelect = 0;
    uint8_t lowCount = 0;
    uint8_t highCount = 0;
    uint8_t bypassDivider = 0;

    uint16_t clkReg1 = 0;
    uint16_t clkReg2 = 0;

    static uint8_t MMCM_CLKOUT_M_VAL_MIN = 1;
    static uint8_t MMCM_CLKOUT_M_VAL_MAX = 126;

    /* clkOutVal must be between 1 and 126 */
    ADI_RANGE_CHECK(device, targetClockDivisor, MMCM_CLKOUT_M_VAL_MIN, MMCM_CLKOUT_M_VAL_MAX);

    edgeSelect = (targetClockDivisor & 1) ? 1 : 0;  /* if odd, edgeSelect = 1 */
    highCount = ((targetClockDivisor >> 1) & 0x3F); /* high count is divider/2 */
    lowCount = (edgeSelect == 1) ? ((highCount + 1) & 0x3F) : (highCount & 0x3F); /* low count is high count +1 if ClkOutVal is odd */
    bypassDivider = (targetClockDivisor == 1) ? 1 : 0;

    clkReg1 |= ((highCount << 6) | (lowCount));
    clkReg2 |= ((edgeSelect << 7) | (bypassDivider << 6));

    *bfValue1 = clkReg1;
    *bfValue2 = clkReg2;

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_MmcmRegBf_Set(adi_fpga9001_Device_t *device,
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr,
                                          uint8_t drpRegAddr,
                                          uint16_t bfValue)
{
    static uint8_t WRITE_ACCESS = 0x0;
    static uint8_t MMCM_REQUEST = 0x1;

    /* Reset MMCM clock wizard */
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmResetBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, 0x1);

    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmAddrBfSet, device, baseAddr, drpRegAddr);
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmWrdataBfSet, device, baseAddr, bfValue);
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmRd1Wr0BfSet, device, baseAddr, WRITE_ACCESS);
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmRequestBfSet, device, baseAddr, MMCM_REQUEST);

    /* un-reset MMCM clock wizard */
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmResetBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, 0x0);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_MmcmRegBf_Get(adi_fpga9001_Device_t *device,
                                          fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr,
                                          uint8_t drpRegAddr,
                                          uint16_t *bfValue)
{
    static uint8_t READ_ACCESS = 0x1;
    static uint8_t MMCM_REQUEST = 0x1;

    uint16_t tempBfValue = 0;

    /* Reset MMCM clock wizard */
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmResetBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, 0x1);

    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmAddrBfSet, device, baseAddr, drpRegAddr);
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmRd1Wr0BfSet, device, baseAddr, READ_ACCESS);
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmRequestBfSet, device, baseAddr, MMCM_REQUEST);
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmRddataBfGet, device, baseAddr, &tempBfValue);

    /* un-reset MMCM clock wizard */
    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmResetBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, 0x0);

    *bfValue = tempBfValue;

    ADI_API_RETURN(device);
}

static uint32_t adi_fpga9001_Mmcm_Configure_Validate(adi_fpga9001_Device_t *device, 
                                                     adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockOutDivisor)
{
    ADI_API_ENTRY_EXPECT(device);

    /* Device clock divisor value should be between 0 and 6 */
    ADI_RANGE_CHECK(device, adrv9001DeviceClockOutDivisor, ADI_FPGA9001_MMCM_CLKDIV_BYPASS, ADI_FPGA9001_MMCM_CLKDIV_64);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Mmcm_Configure(adi_fpga9001_Device_t *device, 
                                    uint32_t adrv9001DeviceClockIn_kHz,
                                    uint32_t targetInterfaceSampleRate_Hz,
                                    adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockOutDivisor)
{
    uint8_t mVal = 0;
    uint8_t i = 0;
    uint8_t drpRegAddr[][2] = { { 0x0A, 0x0B }, /* CLK_OUT1_REG1/2 */
                                { 0x0C, 0x0D }, /* CLK_OUT2_REG1/2 */
                              };

    uint16_t clkReg1 = 0;
    uint16_t clkReg2 = 0;
    uint16_t bfValue = 0;

    uint32_t adrv9001DeviceClockOut_kHz = 0;
    uint32_t actualFpgaVcoClockFreq_kHz = 0;
    uint32_t minTddClockDivisor = 0;
    uint32_t maxTddClockDivisor = 0;
    uint32_t tddClock_kHz = 0;
    uint32_t tddClockDivisor = 1;
    uint32_t observationGpioClockDivisor = 1;
    uint32_t lcm = 0;

    static const uint8_t CLK_FB_OUT_REG1_ADDR = 0x14;
    static const uint8_t CLK_FB_OUT_REG2_ADDR = 0x15;
    static const uint8_t CLK_OUT0_REG1_ADDR = 0x08;
    static const uint8_t CLK_OUT0_REG2_ADDR = 0x09;
    static const uint8_t MAX_USED_MMCM_CLKOUT = 2;
    static const uint32_t TDD_CLK_MIN = 61440;
    static const uint32_t TDD_CLK_MAX = 200000;
    static const uint32_t OBSGPIO_CLK_MIN = 184320;

    fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TOP;

    double reminder = 0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_Mmcm_Configure_Validate, device, adrv9001DeviceClockOutDivisor);

    uint8_t targetClockDivisor[] = { observationGpioClockDivisor,
                                     tddClockDivisor
                                   };

    adrv9001DeviceClockOut_kHz = adrv9001DeviceClockIn_kHz >> adrv9001DeviceClockOutDivisor;

    /* Calculate Least Common Multiple (LCM) of deviceClockOut and SampleRate */
    ADI_EXPECT(fpga9001_Utilities_CalculateLcm,
               device,
               KILO_TO_BASE_UNIT(adrv9001DeviceClockOut_kHz),
               targetInterfaceSampleRate_Hz,
               &lcm);

    /* Calculate 'M' value */
    ADI_EXPECT(adi_fpga9001_ClkFbOutReg_Calculate, 
               device, 
               KILO_TO_BASE_UNIT(adrv9001DeviceClockOut_kHz),
               lcm,
               &mVal,
               &clkReg1,
               &clkReg2);

    ADI_EXPECT(adi_fpga9001_MmcmRegBf_Get, device, baseAddr, CLK_FB_OUT_REG1_ADDR, &bfValue);

    /* Bit 12 is reserved; Retain the previous value stored here, otherwise MMCM loses lock */
    clkReg1 = (bfValue & 0xf000) | (clkReg1 & 0x0fff);
    ADI_EXPECT(adi_fpga9001_MmcmRegBf_Set, device, baseAddr, CLK_FB_OUT_REG1_ADDR, clkReg1);

    ADI_EXPECT(adi_fpga9001_MmcmRegBf_Set, device, baseAddr, CLK_FB_OUT_REG2_ADDR, clkReg2);

    /* MCS CLK is the same as ADV9001 device clock output */
    ADI_EXPECT(adi_fpga9001_MmcmRegBf_Get, device, baseAddr, CLK_OUT0_REG1_ADDR, &bfValue);

    /* Bit 12 is reserved; Retain the previous value stored here */
    clkReg1 = (bfValue & 0xf000) | (clkReg1 & 0x0fff);
    ADI_EXPECT(adi_fpga9001_MmcmRegBf_Set, device, baseAddr, CLK_OUT0_REG1_ADDR, clkReg1);

    ADI_EXPECT(adi_fpga9001_MmcmRegBf_Set, device, baseAddr, CLK_OUT0_REG2_ADDR, clkReg2);

    actualFpgaVcoClockFreq_kHz = mVal * adrv9001DeviceClockOut_kHz;

    /* Find the min and max possible TDD clock divisor based on minimum TDD clock = 61.44MHz and maximum TDD clock = 245.76MHz */
    maxTddClockDivisor = actualFpgaVcoClockFreq_kHz / TDD_CLK_MIN;
    minTddClockDivisor = actualFpgaVcoClockFreq_kHz / TDD_CLK_MAX;

    /* Find highest allowed value of TDD clock that is a multiple of targetInterfaceSampleRate_Hz */
    for (i = maxTddClockDivisor; i >= minTddClockDivisor; i--)
    {
        tddClock_kHz = actualFpgaVcoClockFreq_kHz / i;

        reminder = KILO_TO_BASE_UNIT(tddClock_kHz) % targetInterfaceSampleRate_Hz;
        if (reminder == 0)
        {
            /* tddClockDivisor */
            targetClockDivisor[1] = i;
        }
    }

    if (targetClockDivisor[1] < minTddClockDivisor)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         targetClockDivisor[1],
                         "Unable to calculate a valid TDD Clock.");
    }

    /* Find the max possible observation GPIO clock divisor based on minimum observationGpiosClock_kHz = 184.32MHz (which is same as Navassa system clock) */
    targetClockDivisor[0] = (uint8_t)(actualFpgaVcoClockFreq_kHz / OBSGPIO_CLK_MIN); /* observationGpioClockDivisor */

    for (i = 0; i < MAX_USED_MMCM_CLKOUT; i++)
    {
        /* Calculate CLKOUT_1 -> observation GPIO,
                     CLKOUT_2 -> TDD */
        ADI_EXPECT(adi_fpga9001_ClkOutReg_Calculate, 
                   device,
                   targetClockDivisor[i],
                   &clkReg1,
                   &clkReg2);

        ADI_EXPECT(adi_fpga9001_MmcmRegBf_Get, device, baseAddr, drpRegAddr[i][0], &bfValue);

        /* Bit 12 is reserved; Retain the previous value stored here */
        clkReg1 = (bfValue & 0xf000) | (clkReg1 & 0x0fff);
        ADI_EXPECT(adi_fpga9001_MmcmRegBf_Set, device, baseAddr, drpRegAddr[i][0], clkReg1);

        ADI_EXPECT(adi_fpga9001_MmcmRegBf_Set, device, baseAddr, drpRegAddr[i][1], clkReg2);
    }

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Mmcm_Inspect(adi_fpga9001_Device_t *device, 
                                  uint32_t adrv9001DeviceClockIn_kHz,
                                  adi_fpga9001_Mmcm_ClockOutput_Divisor_e adrv9001DeviceClockOutDivisor,
                                  adi_fpga9001_MmcmCfg_t *actualMmcmCfg)
{
    uint8_t mVal = 0;
    uint8_t lowCount = 0;
    uint8_t highCount = 0;
    uint8_t clkOut = 0;
    uint8_t i = 0;
    uint8_t drpRegAddr[] = { 0x08, 0x0A, 0x0C };

    static const uint8_t CLK_FB_OUT_REG1_ADDR = 0x14;
    static const uint8_t MAX_USED_MMCM_CLKOUT = 3;

    uint16_t clkReg1 = 0;

    /* Temporarily store MCS, GPIO and TDD CLK OUT in this order in the array below */
    uint32_t clock_kHz[] = { 0, 0, 0 };
    uint32_t adrv9001DeviceClockOut_kHz = 0;

    fpga9001_BfAxiAdrv9001ChanAddr_e baseAddr = FPGA9001_BF_AXI_ADRV9001_TOP;

    ADI_API_ENTRY_PTR_EXPECT(device, actualMmcmCfg);

    /* Device clock divisor value should be between 0 and 6 */
    ADI_RANGE_CHECK(device, adrv9001DeviceClockOutDivisor, ADI_FPGA9001_MMCM_CLKDIV_BYPASS, ADI_FPGA9001_MMCM_CLKDIV_64);

    adrv9001DeviceClockOut_kHz = adrv9001DeviceClockIn_kHz >> adrv9001DeviceClockOutDivisor;

    ADI_EXPECT(adi_fpga9001_MmcmRegBf_Get, device, baseAddr, CLK_FB_OUT_REG1_ADDR, &clkReg1);
    lowCount = (uint8_t)(clkReg1 & 0x3F);
    highCount = (uint8_t)((clkReg1 >> 6) & 0x3F);

    mVal = highCount + lowCount;
    actualMmcmCfg->fpgaVcoClockFreq_kHz = mVal * adrv9001DeviceClockOut_kHz;

    for (i = 0; i < MAX_USED_MMCM_CLKOUT; i++)
    {
        ADI_EXPECT(adi_fpga9001_MmcmRegBf_Get, device, baseAddr, drpRegAddr[i], &clkReg1);

        lowCount = (uint8_t)(clkReg1 & 0x3F);
        highCount = (uint8_t)((clkReg1 >> 6) & 0x3F);

        clkOut = highCount + lowCount;
        clock_kHz[i] = (uint32_t)(actualMmcmCfg->fpgaVcoClockFreq_kHz / clkOut);
    }

    actualMmcmCfg->mcsClock_kHz = clock_kHz[0];
    actualMmcmCfg->observationGpiosClock_kHz = clock_kHz[1];
    actualMmcmCfg->tddClock_kHz = clock_kHz[2];

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Mmcm_Lock_Get(adi_fpga9001_Device_t *device, bool *mmcmLocked)
{
    uint8_t mmcmStatus = 0;
    static const uint8_t MMCM_LOCKED = 0x1;

    ADI_API_ENTRY_PTR_EXPECT(device, mmcmLocked);

    ADI_EXPECT(fpga9001_AxiAdrv9001MmcmLockedBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &mmcmStatus);

    *mmcmLocked = (mmcmStatus & MMCM_LOCKED) ? true : false;

    ADI_API_RETURN(device);
}
