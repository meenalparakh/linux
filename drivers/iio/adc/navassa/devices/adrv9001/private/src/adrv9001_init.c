/**
 * \file
 * \brief Contains ADRV9001 init related private function implementations
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_adrv9001_user.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_error.h"
#include "adrv9001_init.h"
#include "adrv9001_init_types.h"
#include "adrv9001_reg_addr_macros.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_bf_nvs_pll_mem_map.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_bf_nvs_regmap_core_1.h"

#ifdef SI_REV_B0
#include "adrv9001_bf_nvs_regmap_core_3.h"
#endif

/*
Set Master Bias
*/
static int32_t adrv9001_MasterBiasSet(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    uint8_t masterBiasConfig0 = 0;

    static const uint8_t PD_MASTER_BIAS = 0x80;

    /* -SPI : core_bf.pd_masterbias  =  1'b0(Default) */
    masterBiasConfig0 &= ~PD_MASTER_BIAS;

    ADRV9001_SPIWRITEBYTE(device, "MASTERBIAS_CONFIG_0", ADRV9001_ADDR_MASTERBIAS_CONFIG_0, masterBiasConfig0);

    ADI_API_RETURN(device);
}

static int32_t adrv9001_RefClockEnable(adi_adrv9001_Device_t *device)
{
    static const uint8_t DEVICE_CLK_BUFFER_ENABLE      = 0x02;
    static const uint8_t DEVCLK_DIVIDER_MCS_RESETB     = 0x10;

    /* REF Clock Block Power Up */
    /* -SPI : core_1_bf.refclk_osc_en   = 1'b1(Default) */

    /* Digital REF Clock Block Power Up */
    /* -SPI : core_bf.dig_device_clk_buffer_enable = 1'b1(Default) */
    ADRV9001_SPIWRITEBYTE(device, "ANALOG_CONTROL_18",
        ADRV9001_ADDR_ANALOG_CONTROL_18,
        (ANALOG_CONTROL_18_RESET | DIG_DEVICE_CLK_BUFFER_ENABLE));

    /* Analog REF Clock Block Power Up */
    /* -SPI : core_bf.device_clk_buffer_enable = 1'b1(Default) */
    /* Disable MCS Reset */
    /* - SPI: core_bf.devclk_divider_mcs_resetb =  1'b1 (Default) */
    ADRV9001_SPIWRITEBYTE(device, "DEVICE_CLK_CONTROL_1",
        ADRV9001_ADDR_DEVICE_CLK_CONTROL_1,
        (DEVICE_CLK_BUFFER_ENABLE | DEVCLK_DIVIDER_MCS_RESETB));

    ADI_API_RETURN(device);
}

/*
Set pad configurations for SYSREF, REFCLK and SYNCB pins
*/
static int32_t adrv9001_PadConfigsSet(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    static const uint8_t REF_CLK_GEN_PD_CLK_PLL        = 0x04;
    static const uint8_t REF_CLK_GEN_PD_LP_CLK_PLL     = 0x08;
    static const uint8_t REF_CLK_GEN_OUT_CLKS_PD_RESET = 0x1F;
    static const uint8_t PAD_REF_CLK_DRV_MASK          = 0x03;

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        /* Device Clock + Reference Clock Buffer Out Configuration */
        /* - SPI: core_bf.ref_clk_gen_pd_clk_pll */
        ADRV9001_SPIWRITEBYTE(device,
                              "REF_CLK_GEN_OUT_CLKS_PD",
                              ADRV9001_ADDR_REF_CLK_GEN_OUT_CLKS_PD,
                              (REF_CLK_GEN_OUT_CLKS_PD_RESET & ~REF_CLK_GEN_PD_LP_CLK_PLL));
    }
    else
    {
        /* Device Clock + Reference Clock Buffer Out Configuration */
        /* - SPI: core_bf.ref_clk_gen_pd_clk_pll */
        ADRV9001_SPIWRITEBYTE(device,
                              "REF_CLK_GEN_OUT_CLKS_PD",
                              ADRV9001_ADDR_REF_CLK_GEN_OUT_CLKS_PD,
                              (REF_CLK_GEN_OUT_CLKS_PD_RESET & ~REF_CLK_GEN_PD_CLK_PLL));

    }

    /* - SPI: core_bf.pad_ref_clk_drv[1:0] */
    ADRV9001_SPIWRITEBYTE(device,
                          "REF_CLK_PAD_CONFIG",
                          ADRV9001_ADDR_REF_CLK_PAD_CONFIG,
                          (init->clocks.padRefClkDrv & PAD_REF_CLK_DRV_MASK));

    ADI_API_RETURN(device);
}

static int32_t adrv9001_ClockVerify(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init, uint8_t *modeAdc)
{
    static const uint32_t LVDS_CLOCK_MAX_KHZ                        = 1000000;
    static const uint32_t CMOS_CLOCK_MAX_KHZ                        = 80000;

    /* Get the power-up reference clock */
    ADI_EXPECT(adrv9001_NvsRegmapCore1ModeAdcThermBfGet, device, ADRV9001_BF_CORE_1, modeAdc);

    if ((*modeAdc != 0) && (*modeAdc != 1) && (*modeAdc != 3) && (*modeAdc != 7) && (*modeAdc != 15))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            modeAdc,
            "Invalid modeAdc from SPI read");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    if (*modeAdc == 0)
    {
        /* LVDS mode */
        if (init->clocks.deviceClock_kHz > LVDS_CLOCK_MAX_KHZ)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                init->clocks.deviceClock_kHz,
                "Invalid LVDS mode frequency.  Valid Range 0-1GHz");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }
    else
    {
        /* CMOS or XTAL mode */
        if (init->clocks.deviceClock_kHz > CMOS_CLOCK_MAX_KHZ)
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                init->clocks.deviceClock_kHz,
                "Invalid CMOS or XTAL mode frequency.  Valid Range 0-80MHz");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }
    }

    ADI_API_RETURN(device);
}

#ifdef SI_REV_A0
static uint32_t adrv9001_ClockDiv(uint32_t clock, uint32_t clock_max)
{
    uint32_t div = 0;
    static const uint32_t ADRV9001_POWER2_MAX = 8;

    for (div = 0; div < ADRV9001_POWER2_MAX; div++)
    {
        if (clock <= (clock_max << div))
        {
            return div;
        }
    }

    return div;
}
#endif

static int32_t adrv9001_ClocksSet(adi_adrv9001_Device_t *device,
                                  adi_adrv9001_Init_t *init,
                                  adi_adrv9001_DeviceClockDivisor_e adrv9001DeviceClockOutDivisor)
{
    adi_adrv9001_ClockSettings_t *clocks = NULL;

    int32_t recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    uint8_t modeAdc = 0;
#ifdef SI_REV_A0
    uint32_t digDeviceClockDiv = 0;
    uint32_t regDeviceClockDiv = 0;
    uint32_t armDeviceClockDiv = 0;
    uint32_t agcDeviceClockDiv = 0;
    uint32_t txAttenDeviceClockDiv = 0;
    uint32_t refClockDiv = 0;
    uint8_t deviceClkControl3 = 0;
    uint8_t refClkByte0 = 0;
    uint8_t clkPllLp = 0;
    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;

    static const uint32_t DIG_CLOCK_MAX_KHZ = 250000;
    static const uint32_t DIG_DEVICE_CLOCK_DIV_MAX = 3;
    static const uint32_t REF_CLOCK_DIV_MAX = 3;
    static const uint32_t REG_DEVICE_CLOCK_DIV_MAX = 5;

    static const uint8_t DEVICE_CLK_DIVIDE_RATIO_MASK  = 0x07;
    static const uint8_t DEVICE_CLK_DIVIDE_RATIO_SHIFT = 0;
    static const uint8_t REF_CLK_DIVIDE_RATIO_MASK     = 0x03;
    static const uint8_t REF_CLK_DIVIDE_RATIO_SHIFT    = 0;
    static const uint32_t REF_CLOCK_MAX_KHZ = 80000;
#endif
    static const uint32_t DEV_CLOCK_MIN_KHZ = 10000;
    static const uint32_t DEV_CLOCK_MAX_KHZ = 1000000;

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, init);
    clocks = &init->clocks;

    ADI_RANGE_CHECK(device, clocks->deviceClock_kHz, DEV_CLOCK_MIN_KHZ, DEV_CLOCK_MAX_KHZ);

#ifdef SI_REV_A0
    if ((clocks->deviceClock_kHz >> adrv9001DeviceClockOutDivisor) > REF_CLOCK_MAX_KHZ)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         adrv9001DeviceClockOutDivisor,
                         "Invalid ADRV9001 Device clock output divisor; (deviceClock_kHz / 2^DeviceClockOutDivisor) must be less than 80MHz");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

#if ADI_ADRV9001_PRE_MCS_BROADCAST_DISABLE > 0
    recoveryAction = adrv9001_ClockVerify(device, init, &modeAdc);
    if (recoveryAction != ADI_COMMON_ACT_NO_ACTION)
    {
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
#endif

    if (modeAdc == 0)
    {
        /* Core1.analog_ref_clk_reg_1[D2:D0]
           0 => bypass 1 => div 2 2 => div 4 3 => div 8 4 => div 16 5 => div 32 6 => div 64 */
        ADI_EXPECT(adrv9001_NvsRegmapCore1DigDevclkDivideRatioDiffRcvBfSet,
                   device,
                   ADRV9001_BF_CORE_1,
                   (uint8_t)adrv9001DeviceClockOutDivisor);
    }
    else
    {
        /* Core1.analog_ref_clk_reg_1[D6:D4]
           0 => bypass 1 => div 2 2 => div 4 3 => div 8 4 => div 16 5 => div 32 6 => div 64 */
        ADI_EXPECT(adrv9001_NvsRegmapCore1DigDevclkDivideRatioOscRcvBfSet,
                   device,
                   ADRV9001_BF_CORE_1,
                   (uint8_t)adrv9001DeviceClockOutDivisor);
    }
#ifdef SI_REV_A0
    /* Set Digital device clock divider */
    digDeviceClockDiv = adrv9001_ClockDiv(clocks->deviceClock_kHz, DEV_CLOCK_MAX_KHZ);
    if (digDeviceClockDiv > DIG_DEVICE_CLOCK_DIV_MAX)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         device,
                         "Invalid clocks deviceClock_kHz");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Set Digital Reg device clock divider  */
    regDeviceClockDiv = adrv9001_ClockDiv(device->devStateInfo.hsDigClk_kHz, DIG_CLOCK_MAX_KHZ);
    if (regDeviceClockDiv > REG_DEVICE_CLOCK_DIV_MAX)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         device,
                         "Invalid clocks hsDigClk_kHz");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    armDeviceClockDiv = regDeviceClockDiv;
    agcDeviceClockDiv = regDeviceClockDiv;
    txAttenDeviceClockDiv = regDeviceClockDiv;

    /* Set Ref clock divider  */
    refClockDiv = adrv9001_ClockDiv((clocks->deviceClock_kHz >> digDeviceClockDiv), DEV_CLOCK_MAX_KHZ);
    if (refClockDiv > REF_CLOCK_DIV_MAX)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         device,
                         "Invalid clocks deviceClock_kHz");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Set Device clock divider ratio */
    deviceClkControl3 = 0;
    deviceClkControl3 |= ADRV9001_BF_ENCODE(digDeviceClockDiv, DEVICE_CLK_DIVIDE_RATIO_MASK, DEVICE_CLK_DIVIDE_RATIO_SHIFT);
    ADRV9001_SPIWRITEBYTE(device, "DEVICE_CLK_CONTROL_3", ADRV9001_ADDR_DEVICE_CLK_CONTROL_3, deviceClkControl3);

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;
        clkPllLp = 1;
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL;
        clkPllLp = 0;
    }

    /* clkpll.ref_clk_divide_ratio  */
    refClkByte0 = ADRV9001_BF_ENCODE(refClockDiv, REF_CLK_DIVIDE_RATIO_MASK, REF_CLK_DIVIDE_RATIO_SHIFT);
    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_REF_CLK_BYTE0",
        baseAddr + ADRV9001_ADDR_CLK_PLL_REF_CLK_BYTE0_OFFSET, refClkByte0);

    ADRV9001_SPIWRITEBYTE(device, "CLOCK_CONTROL_2", ADRV9001_ADDR_CLOCK_CONTROL_2, clkPllLp);

    device->devStateInfo.clkDivideRatios.devClkDivideRatio = digDeviceClockDiv;
    device->devStateInfo.clkDivideRatios.refClkDivideRatio = refClockDiv;
    device->devStateInfo.clkDivideRatios.agcClkDivideRatio = agcDeviceClockDiv;
    device->devStateInfo.clkDivideRatios.armClkDivideRatio = armDeviceClockDiv;
    device->devStateInfo.clkDivideRatios.regClkDivideRatio = regDeviceClockDiv;
    device->devStateInfo.clkDivideRatios.txAttenDeviceClockDivideRatio = txAttenDeviceClockDiv;
#endif

    device->devStateInfo.clkPllMode = init->clocks.clkPllMode;

    device->devStateInfo.clkDivideRatios.anaRefClockRatio = (uint8_t)adrv9001DeviceClockOutDivisor;
    device->devStateInfo.deviceClock_kHz = init->clocks.deviceClock_kHz;

    ADI_API_RETURN(device);
}

#ifdef SI_REV_A0
static int32_t adrv9001_LdoEnable(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    int32_t halError = (int32_t)ADI_COMMON_HAL_OK;

    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr = ADRV9001_BF_CLK_PLL_LP;

    static const uint8_t DEV_CLK_LDO_BYTE1_RESET     = 0x05;
    static const uint8_t DEV_CLK_LDO_PD              = 0x08;
    static const uint8_t CLK_SYNTH_LDO_BYTE2_RESET   = 0x23;
    static const uint8_t CLK_PLL_VCO_LDO_PD          = 0x20;
    static const uint8_t CLK_PLL_VCO_LDO_BYTE1_RESET = 0x05;
    static const uint8_t CLK_SYNTH_LDO_BYTE1_RESET   = 0x05;
    static const uint8_t CLK_PLL_VCO_LDO_BYTE2_RESET = 0x03;
    static const uint8_t CLK_SYNTH_LDO_PD            = 0x08;
    static const uint8_t VCO_CORE_BUF_PD_SET         = 0x00;
    static const uint8_t VCO_CORE_SW_DISABLE_SET     = 0x00;
    static const uint8_t SYNTH_VCO_PD_SET            = 0x00;
    static const uint8_t LO_SYNC_DET_VCO_PD_SET      = 0xF1;

    static const uint32_t WAIT_200_US                = 200;

    /* - SPI : core_bf.dev_clk_ldo_pd    = 1'b0(Default) */
    ADRV9001_SPIWRITEBYTE(device, "DEV_CLK_LDO_BYTE1",
        ADRV9001_ADDR_DEV_CLK_LDO_BYTE1,
        (DEV_CLK_LDO_BYTE1_RESET & ~DEV_CLK_LDO_PD));

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        baseAddr = ADRV9001_BF_CLK_PLL_LP;

        /* Power down CLK PLL in case of LP CLK PLL*/
        /* -SPI : core_bf.clk_pll_vco_ldo_pd */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE2",
            ADRV9001_ADDR_CLK_SYNTH_LDO_BYTE2,
            CLK_SYNTH_LDO_BYTE2_RESET);

        /* CLK_SYNTH_LP_LDO_BYTE1(0x01B2) = 0x0D */
        /* - VCO LDO output voltage = 1.075V */
        /* - SPI: core_bf.lp_clk_pll_vco_ldo_pd = 1   (PLL)*/
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE1",
            ADRV9001_ADDR_CLK_SYNTH_LP_LDO_BYTE1,
            (CLK_SYNTH_LDO_BYTE1_RESET | CLK_SYNTH_LDO_PD));

        /* -SPI : core_bf.lp_clk_pll_vco_ldo_pd */
        /* clk_synth_lp_ldo_vout_trim = 0x3 */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE2",
            ADRV9001_ADDR_CLK_SYNTH_LP_LDO_BYTE2,
            CLK_SYNTH_LDO_BYTE2_RESET);

        /* - PLL LDO Power Up. */
        /* - SPI: core_bf.lp_clk_synth_ldo_pd = 0   (PLL)*/
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE1",
            ADRV9001_ADDR_CLK_SYNTH_LP_LDO_BYTE1,
            (CLK_SYNTH_LDO_BYTE1_RESET & ~CLK_SYNTH_LDO_PD));

        /* Wait for 200us */
        halError = adi_common_hal_Wait_us(&device->common, WAIT_200_US);
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllWaitForTemp()");
        ADI_ERROR_RETURN(device->common.error.newAction);

        /* Reset VCO LDO byte1 LSB */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_LDO_BYTE1_RESET",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_LDO_BYTE1_OFFSET,
            CLK_PLL_VCO_LDO_BYTE1_RESET);

        /* Reset VCO LDO byte2 LSB */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_LDO_BYTE2_RESET",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_LDO_BYTE2_OFFSET,
            CLK_PLL_VCO_LDO_BYTE2_RESET);

        /* -SPI : core_bf.clk_pll_vco_ldo_pd */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE2",
            ADRV9001_ADDR_CLK_SYNTH_LP_LDO_BYTE2,
            (CLK_SYNTH_LDO_BYTE2_RESET & ~CLK_PLL_VCO_LDO_PD));

        /* Wait for 200us */
        halError = adi_common_hal_Wait_us(&device->common, WAIT_200_US);
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllWaitForTemp()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }
    else
    {
        baseAddr = ADRV9001_BF_CLK_PLL;

        /* Power down LP CLK PLL in case of CLK PLL*/
        /* -SPI : core_bf.lp_clk_pll_vco_ldo_pd */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE2",
            ADRV9001_ADDR_CLK_SYNTH_LP_LDO_BYTE2,
            CLK_SYNTH_LDO_BYTE2_RESET);

        /* CLK_SYNTH_LDO_BYTE1(0x01AA) = 0x0D */
        /* - VCO LDO output voltage = 1.075V */
        /* - SPI: core_bf.clk_pll_vco_ldo_pd = 1   (PLL)*/
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE1",
            ADRV9001_ADDR_CLK_SYNTH_LDO_BYTE1,
            (CLK_SYNTH_LDO_BYTE1_RESET | CLK_SYNTH_LDO_PD));

        /* -SPI : core_bf.clk_pll_vco_ldo_pd */
        /* clk_synth_ldo_vout_trim = 0x3 */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE2",
            ADRV9001_ADDR_CLK_SYNTH_LDO_BYTE2,
            CLK_SYNTH_LDO_BYTE2_RESET);

        /* - PLL LDO Power Up. */
        /* - SPI: core_bf.clk_synth_ldo_pd = 0   (PLL)*/
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE1",
            ADRV9001_ADDR_CLK_SYNTH_LDO_BYTE1,
            (CLK_SYNTH_LDO_BYTE1_RESET & ~CLK_SYNTH_LDO_PD));

        /* Wait for 200us */
        halError = adi_common_hal_Wait_us(&device->common, WAIT_200_US);
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllWaitForTemp()");
        ADI_ERROR_RETURN(device->common.error.newAction);

        /* Reset VCO LDO byte1 LSB */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_LDO_BYTE1_RESET",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_LDO_BYTE1_OFFSET,
            CLK_PLL_VCO_LDO_BYTE1_RESET);

        /* Reset VCO LDO byte2 LSB */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_PLL_VCO_LDO_BYTE2_RESET",
            baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_LDO_BYTE2_OFFSET,
            CLK_PLL_VCO_LDO_BYTE2_RESET);

        /* -SPI : core_bf.clk_pll_vco_ldo_pd */
        ADRV9001_SPIWRITEBYTE(device,
            "CLK_SYNTH_LDO_BYTE2",
            ADRV9001_ADDR_CLK_SYNTH_LDO_BYTE2,
            (CLK_SYNTH_LDO_BYTE2_RESET & ~CLK_PLL_VCO_LDO_PD));

        /* Wait for 200us */
        halError = adi_common_hal_Wait_us(&device->common, WAIT_200_US);
        ADI_ERROR_REPORT(&device->common,
            ADI_ADRV9001_SRC_INIT,
            halError,
            ADI_COMMON_ACT_ERR_CHECK_TIMER,
            device,
            "Timer not working in adrv9001_PllWaitForTemp()");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* Vco pd & vco_buf_pd */
    /* - SPI : clk_pll_lp_bf.vco_pd =  4'd0 */
    /* - SPI : clk_pll_lp_bf.vco_buf_pd = 0 */
    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_CORE_BUF_PD",
        baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_CORE_BUF_PD_OFFSET,
        VCO_CORE_BUF_PD_SET);

    /* Core Sw Disable */
    /* Prescaler_pd */
    /* - SPI: clk_pll_bf.vco_core_sw_disable = 4'd0 */
    /* - SPI: clk_pll_bf.prescaler_pd = 1'b0 */
    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_VCO_CORE_SW_DISABLE",
        baseAddr + ADRV9001_ADDR_CLK_PLL_VCO_CORE_SW_DISABLE_OFFSET,
        VCO_CORE_SW_DISABLE_SET);

    /* - SPI: clk_pll_bf.vco_ldo_pd = 1'b0 */
    /* - SPI: clk_pll_bf.vco_peak_det_pd = 1'b0 */
    /* - SPI: clk_pll_bf.vco_ptat_pd = 1'b0 */
    /* - SPI: clk_pll_bf.vco_alc_pd = 1'b0 */
    /* - SPI: clk_pll_bf.ref_clk_divider_pd = 1'b0 */
    /* - SPI: clk_pll_bf.cp_leveldet_pd = 1'b0 */
    /* - SPI: clk_pll_bf.synth_pd = 1'b0 */
    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_SYNTH_VCO_PD",
        baseAddr + ADRV9001_ADDR_CLK_PLL_SYNTH_VCO_PD_OFFSET,
        SYNTH_VCO_PD_SET);

    /* - SPI: clk_pll_bf.vco_tc_pd = 1'b0
    - SPI: clk_pll_bf.vco_cal_tcf_pd = 1'b0
    - SPI: clk_pll_bf.vco_bias_pd = 1'b0
    - SPI: clk_pll_bf.vco_comp_pd = 1'b0
    */
    ADRV9001_SPIWRITEBYTE(device, "CLK_PLL_LO_SYNC_DET_VCO_PD",
        baseAddr + ADRV9001_ADDR_CLK_PLL_LO_SYNC_DET_VCO_PD_OFFSET,
        LO_SYNC_DET_VCO_PD_SET);

    ADI_API_RETURN(device);
}
#endif // SI_REV_A0

int32_t adrv9001_AnalogClockSet(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    uint8_t divRatio = 0;
    uint8_t divHsDigClk = 0;
    uint8_t rfLvdsDiv = 0;
    int i = 0;
    uint32_t maskBit = 0;

    static const uint8_t RESAMPLE_CLK_DIV_RATIO_MASK = 0x03;
    static const uint8_t DIV_HS_DIG_CLK_MASK         = 0x1F;
    static const uint8_t HS_DIG_CLK_PD               = 0x80;
    static const uint8_t ANALOG_SPARE_6              = 0x0C;
    static const uint8_t DIV_ANA_LSSI_RX_CLK_MASK    = 0x1F;
    static const uint8_t DIV_ANA_LSSI_TX_CLK_MASK    = 0x1F;

    static const uint16_t ADRV9001_ADDR_CH_RX_BASE[ADI_ADRV9001_MAX_RX_ONLY] =
    {
        ADRV9001_ADDR_ANALOG_CONTROL_2,
        ADRV9001_ADDR_ANALOG_CONTROL_3
    };

    static const uint16_t ADRV9001_ADDR_CH_TX_BASE[ADI_ADRV9001_MAX_TXCHANNELS] =
    {
        ADRV9001_ADDR_ANALOG_CONTROL_4,
        ADRV9001_ADDR_ANALOG_CONTROL_5
    };


    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_HP_MODE)
    {
        /*
       - SPI: core_bf.resample_clk_div_ratio
       - SPI: core_bf.div_hs_dig_clk = 2
       - High Speed clock resampling:
         Total_hs_dig_ratio = (CLK_PLL_VCO_FREQ) / (HS_DIG_CLK);
       - if    4    then    core_bf.resample_clk_div_ratio  =  1
       - if    6    then    core_bf.resample_clk_div_ratio  =  2
       - if    8    then    core_bf.resample_clk_div_ratio  =  3
      */

        divHsDigClk = 2;

        if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_4)
        {
            divRatio = 1;
        }
        else if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_6)
        {
            divRatio = 2;
        }
        else if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_8)
        {
            divRatio = 3;
        }
        else
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                device,
                "Invalid clkPllHsDiv");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }

    }
    else if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        /*
           - SPI: core_bf.resample_clk_div_ratio  =  0
           - SPI: core_bf.div_hs_dig_clk
           - High Speed Clock Selection:
             Total_hs_dig_ratio = (CLK_PLL_VCO_FREQ) / (HS_DIG_CLK);
             - if    2    then    core_bf.div_hs_dig_clk =  2
             - if    3    then    core_bf.div_hs_dig_clk =  3
             - if    4    then    core_bf.div_hs_dig_clk =  4
         */

        divRatio = 0;

        if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_4)
        {
            divHsDigClk = 2;
        }
        else if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_6)
        {
            divHsDigClk = 3;
        }
        else if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_8)
        {
            divHsDigClk = 4;
        }
        else
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                device,
                "Invalid clkPllHsDiv");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }

    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         device,
                         "Invalid clkPllMode");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }


    ADRV9001_SPIWRITEBYTE(device,
                          "SYSREF_CONTROL_3",
                          ADRV9001_ADDR_SYSREF_CONTROL_3,
                          (divRatio & RESAMPLE_CLK_DIV_RATIO_MASK));

    /* Resample and High Speed clocks Power Up.
     * - SPI: core_bf.hs_dig_clk_pd = 0 */
    ADRV9001_SPIWRITEBYTE(device,
                          "ANALOG_CONTROL_1",
                          ADRV9001_ADDR_ANALOG_CONTROL_1,
                          ((divHsDigClk & DIV_HS_DIG_CLK_MASK) & ~HS_DIG_CLK_PD));

    /* Power up the analog re - sample clock in differential mode. */
    /* Analog_spare_6 = 8'b00001100 = 0x0C */
    ADRV9001_SPIWRITEBYTE(device, "ANALOG_SPARE_6", ADRV9001_ADDR_ANALOG_SPARE_6, ANALOG_SPARE_6);

    /* If LVDS mode, Those dividers need to be setup
     * according to the PLL VCO clock frequency.
     * - SPI:
     *     spi_reg_O_spi_core_div_ana_lssi_rx1_clk[4:0]
           spi_reg_O_spi_core_div_ana_lssi_rx2_clk[4:0]
           spi_reg_O_spi_core_div_ana_lssi_tx1_clk[4:0]
           spi_reg_O_spi_core_div_ana_lssi_tx2_clk[4:0] */
    for (i = 0; i < ADI_ADRV9001_MAX_RX_ONLY; i++)
    {
        maskBit = (1 << i);
        if (ADRV9001_BF_EQUAL(init->rx.rxInitChannelMask, maskBit))
        {
            rfLvdsDiv = init->rx.rxChannelCfg[i].profile.rxSsiConfig.rfLvdsDiv;
            ADRV9001_SPIWRITEBYTE(device,
                                  "ANALOG_CONTROL_2_OR_3",
                                  ADRV9001_ADDR_CH_RX_BASE[i],
                                  (rfLvdsDiv & DIV_ANA_LSSI_RX_CLK_MASK));
        }
    }

    for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
    {
        maskBit = (1 << i);
        if (ADRV9001_BF_EQUAL(init->tx.txInitChannelMask, maskBit))
        {
            rfLvdsDiv = init->tx.txProfile[i].txSsiConfig.rfLvdsDiv;
            ADRV9001_SPIWRITEBYTE(device,
                                  "ANALOG_CONTROL_4_OR_5",
                                  ADRV9001_ADDR_CH_TX_BASE[i],
                                  (rfLvdsDiv & DIV_ANA_LSSI_TX_CLK_MASK));
        }
    }

    ADI_API_RETURN(device);
}

int32_t adrv9001_InitAnalog(adi_adrv9001_Device_t *device,
                            adi_adrv9001_Init_t *init,
                            adi_adrv9001_DeviceClockDivisor_e adrv9001DeviceClockOutDivisor)
{
#ifdef __KERNEL__
    uint8_t i = 0;
    static const uint8_t refClockDivisor[7] = { 0, 1, 2, 3, 4, 5, 6};
    static const uint32_t REF_CLOCK_MAX_KHZ = 200000;
    adi_adrv9001_DeviceClockDivisor_e adrv9001ReferenceClockOutDivisor = ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED;
#endif

    ADI_API_ENTRY_PTR_EXPECT(device, init);

    /* Device clock divisor value should be between 0 and 6 */
#ifdef SI_REV_A0
    ADI_RANGE_CHECK(device, adrv9001DeviceClockOutDivisor, ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS, ADI_ADRV9001_DEVICECLOCKDIVISOR_64);
#else
    ADI_RANGE_CHECK(device, adrv9001DeviceClockOutDivisor, ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS, ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED);
#endif // SI_REV_A0

    ADI_EXPECT(adrv9001_MasterBiasSet, device, init);

    ADI_EXPECT(adrv9001_RefClockEnable, device);

    ADI_EXPECT(adrv9001_ProfilesVerify, device, init);

#ifdef SI_REV_A0
    ADI_EXPECT(adrv9001_ClocksSet, device, init, adrv9001DeviceClockOutDivisor);
#else
#ifndef __KERNEL__
    uint8_t i = 0;
    static const uint8_t refClockDivisor[7] = { 0, 1, 2, 3, 4, 5, 6};
    static const uint32_t REF_CLOCK_MAX_KHZ = 200000;
    adi_adrv9001_DeviceClockDivisor_e adrv9001ReferenceClockOutDivisor = ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED;
#endif
    /* Maximize internal reference clock such that it does not exceed 200MHz */
    for (i = 0; i < ADI_ARRAY_LEN(refClockDivisor); i++)
    {
        if ((init->clocks.deviceClock_kHz >> i) <= REF_CLOCK_MAX_KHZ)
        {
            adrv9001ReferenceClockOutDivisor = (adi_adrv9001_DeviceClockDivisor_e)refClockDivisor[i];
            break;
        }
    }

    /* Error out if none of the divisor satisfy 'refClk <=200MHz' condition */
    if (ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED == adrv9001ReferenceClockOutDivisor)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            adrv9001ReferenceClockOutDivisor,
            "Invalid ADRV9001 Device clock output divisor; (deviceClock_kHz / 2^DeviceClockOutDivisor) must be less than 200MHz");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    /* To disable device clock out, select 'adrv9001DeviceClockOutDivisor  = ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS' and
       then use a different bitfield (core1.refclk_config(0x03DC)[D0] - refclk_pad_oe = 0x0) to disable the DEV_CLK_OUT pin */
    if (ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED == adrv9001DeviceClockOutDivisor)
    {
        adrv9001DeviceClockOutDivisor = ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS;
        ADI_EXPECT(adrv9001_ClocksSet, device, init, adrv9001DeviceClockOutDivisor);
        ADI_EXPECT(adrv9001_NvsRegmapCore1RefclkPadOeBfSet, device, ADRV9001_BF_CORE_1, 0);
    }
    else
    {
        ADI_EXPECT(adrv9001_ClocksSet, device, init, adrv9001DeviceClockOutDivisor);
    }

    ADI_EXPECT(adrv9001_NvsRegmapCore3RefClkIntDevclkDivideRatioBfSet, device, ADRV9001_BF_CORE_3, (uint8_t)adrv9001ReferenceClockOutDivisor);

#endif // SI_REV_A0

    ADI_EXPECT(adrv9001_PadConfigsSet, device, init);

#ifdef SI_REV_A0
    ADI_EXPECT(adrv9001_LdoEnable, device, init);

    ADI_EXPECT(adrv9001_ProgramClkPll, device, init);
#endif

    ADI_API_RETURN(device);
}

/**
* \brief Verifies the Tx profile members are valid (in range).
* \brief This function is private and is not called directly by the user.
*
* \dep_begin
* \dep{device->common.devHalInfo}
* \dep_end
*
* \param device Structure pointer to ADRV9001 device data structure
* \param txProfile txProfile settings to be verified
* \param txHsDigClk_kHz Return value of the calculated HS Dig Clock required by the Tx profile
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_VerifyTxProfile(adi_adrv9001_Device_t *device,
                                        adi_adrv9001_TxProfile_t *txProfile)
{
    static const uint32_t TX_INPUT_RATE_MIN = KILO_TO_BASE_UNIT(1);
    static const uint32_t TX_INPUT_RATE_MAX = MEGA_TO_BASE_UNIT(100);

    static const uint32_t PRIMARY_SIGNAL_BW_MIN = 6250;
    static const uint32_t PRIMARY_SIGNAL_BW_MAX = MEGA_TO_BASE_UNIT(40);

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, txProfile);

    /********************************/
    /* Check for a valid Tx profile */
    /********************************/
    ADI_RANGE_CHECK(device, txProfile->txInputRate_Hz, TX_INPUT_RATE_MIN, TX_INPUT_RATE_MAX);

    ADI_RANGE_CHECK(device, txProfile->primarySigBandwidth_Hz, PRIMARY_SIGNAL_BW_MIN, PRIMARY_SIGNAL_BW_MAX);

    ADI_API_RETURN(device);
}

/**
* \brief Verifies the Rx profile members are valid (in range).
* \brief This function is private and is not called directly by the user.
*
* \dep_begin
* \dep{device->common.devHalInfo}
* \dep_end
*
* \param device Structure pointer to Adrv9001 device data structure
* \param rxProfile rxProfile settings to be verified
* \param rxHsDigClk_kHz Return value of the calculated HS Dig Clock required by the Rx profile
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_VerifyRxProfile(adi_adrv9001_Device_t *device,
                                        adi_adrv9001_RxProfile_t *rxProfile)
{
    static const uint32_t RX_OUTPUT_RATE_MIN = KILO_TO_BASE_UNIT(1);
    static const uint32_t RX_OUTPUT_RATE_MAX = MEGA_TO_BASE_UNIT(100);

    static const uint32_t RX_BANDWIDTH_MIN = 6250;
    static const uint32_t RX_BANDWIDTH_MAX = MEGA_TO_BASE_UNIT(40);

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, rxProfile);

    /********************************/
    /* Check for a valid Rx profile */
    /********************************/
    ADI_RANGE_CHECK(device, rxProfile->rxOutputRate_Hz, RX_OUTPUT_RATE_MIN, RX_OUTPUT_RATE_MAX);

    ADI_RANGE_CHECK(device, rxProfile->primarySigBandwidth_Hz, RX_BANDWIDTH_MIN, RX_BANDWIDTH_MAX);

    ADI_API_RETURN(device);
}

/**
* \brief Verifies the ORx profile members are valid (in range).
* \brief This function is private and is not called directly by the user.
*
* \dep_begin
* \dep{device->common.devHalInfo}
* \dep_end
*
* \param device Structure pointer to Adrv9001 device data structure
* \param orxProfile rxProfile settings to be verified
* \param orxHsDigClk_kHz Return value of the calculated HS Dig Clock required by the ORx profile
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_VerifyOrxProfile(adi_adrv9001_Device_t *device,
                                         adi_adrv9001_RxProfile_t *orxProfile)
{
    static const uint32_t ORX_OUTPUT_RATE_MIN = KILO_TO_BASE_UNIT(1);
    static const uint32_t ORX_OUTPUT_RATE_MAX = MEGA_TO_BASE_UNIT(100);

    static const uint32_t ORX_BANDWIDTH_MIN = 6250;
    static const uint32_t ORX_BANDWIDTH_MAX = MEGA_TO_BASE_UNIT(40);

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, orxProfile);

    /********************************/
    /* Check for a valid ORx profile */
    /********************************/
    ADI_RANGE_CHECK(device, orxProfile->rxOutputRate_Hz, ORX_OUTPUT_RATE_MIN, ORX_OUTPUT_RATE_MAX);

    ADI_RANGE_CHECK(device, orxProfile->primarySigBandwidth_Hz, ORX_BANDWIDTH_MIN, ORX_BANDWIDTH_MAX);

    ADI_API_RETURN(device);
}

/**
* \brief Verifies the Loopback profile members are valid (in range).
* \brief This function is private and is not called directly by the user.
*
* \dep_begin
* \dep{device->common.devHalInfo}
* \dep_end
*
* \param device Structure pointer to Adrv9001 device data structure
* \param lbProfile lbProfile settings to be verified
* \param lbHsDigClk_kHz Return value of the calculated HS Dig Clock required by the LB profile
*
* \retval ADI_COMMON_ACT_ERR_CHECK_PARAM Recovery action for bad parameter check
* \retval ADI_COMMON_ACT_NO_ACTION Function completed successfully, no action required
*/
static int32_t adrv9001_VerifyLbProfile(adi_adrv9001_Device_t *device,
                                        adi_adrv9001_RxProfile_t *lbProfile)
{
    static const uint32_t LB_OUTPUT_RATE_MIN = KILO_TO_BASE_UNIT(1);
    static const uint32_t LB_OUTPUT_RATE_MAX = MEGA_TO_BASE_UNIT(100);

    static const uint32_t LB_BANDWIDTH_MIN = 6250;
    static const uint32_t LB_BANDWIDTH_MAX = MEGA_TO_BASE_UNIT(40);

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, lbProfile);

    /********************************/
    /* Check for a valid LB profile */
    /********************************/
    ADI_RANGE_CHECK(device, lbProfile->rxOutputRate_Hz, LB_OUTPUT_RATE_MIN, LB_OUTPUT_RATE_MAX);

    ADI_RANGE_CHECK(device, lbProfile->primarySigBandwidth_Hz, LB_BANDWIDTH_MIN, LB_BANDWIDTH_MAX);

    ADI_API_RETURN(device);
}

int32_t adrv9001_ProfilesVerify(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init)
{
    adi_adrv9001_RxProfile_t *rxProfile = NULL;
    adi_adrv9001_TxProfile_t *txProfile = NULL;
    int i = 0;
    int ilbValidForTxMask = 0;
    uint32_t TX_CHANNELS[] = { ADI_ADRV9001_TX1, ADI_ADRV9001_TX2 };
    uint32_t RX_CHANNELS[] = {
        ADI_ADRV9001_RX1,
        ADI_ADRV9001_RX2,
        ADI_ADRV9001_ORX1,
        ADI_ADRV9001_ORX2,
        ADI_ADRV9001_ILB1,
        ADI_ADRV9001_ILB2,
        ADI_ADRV9001_ELB1,
        ADI_ADRV9001_ELB2
     };
    uint8_t divHsDigClk = 0;

    ADI_API_PRIV_ENTRY_PTR_EXPECT(device, init);

    device->devStateInfo.profilesValid = 0;

    if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_HP_MODE)
    {
        if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_4)
        {
            divHsDigClk = 4;
        }
        else if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_6)
        {
            divHsDigClk = 6;
        }
        else if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_8)
        {
            divHsDigClk = 8;
        }
        else
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                device,
                "Invalid clkPllHsDiv");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }

    }
    else if (init->clocks.clkPllMode == ADI_ADRV9001_CLK_PLL_LP_MODE)
    {
        if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_4)
        {
            divHsDigClk = 2;
        }
        else if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_6)
        {
            divHsDigClk = 3;
        }
        else if (init->clocks.clkPllHsDiv == ADI_ADRV9001_HSDIV_8)
        {
            divHsDigClk = 4;
        }
        else
        {
            ADI_ERROR_REPORT(&device->common,
                ADI_COMMON_ERRSRC_API,
                ADI_COMMON_ERR_INV_PARAM,
                ADI_COMMON_ACT_ERR_CHECK_PARAM,
                device,
                "Invalid clkPllHsDiv");
            ADI_ERROR_RETURN(device->common.error.newAction);
        }

    }
    else
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            device,
            "Invalid clkPllMode");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    device->devStateInfo.hsDigClk_kHz = init->clocks.clkPllVcoFreq_kHz / divHsDigClk;

    // Validate Rx channels
    for (i = 0; i < ADI_ADRV9001_MAX_RXCHANNELS; i++)
    {
        if (ADRV9001_BF_EQUAL(init->rx.rxInitChannelMask, RX_CHANNELS[i]))
        {
            rxProfile = &init->rx.rxChannelCfg[i].profile;
            switch (RX_CHANNELS[i])
            {
            case ADI_ADRV9001_RX1:  /* Falls through */
            case ADI_ADRV9001_RX2:
                ADI_EXPECT(adrv9001_VerifyRxProfile, device, rxProfile);
                device->devStateInfo.profilesValid |= ADI_ADRV9001_RX_PROFILE_VALID;
                break;
            case ADI_ADRV9001_ORX1: /* Falls through */
            case ADI_ADRV9001_ORX2:
                ADI_EXPECT(adrv9001_VerifyOrxProfile, device, rxProfile);
                device->devStateInfo.profilesValid |= ADI_ADRV9001_ORX_PROFILE_VALID;
                break;
            case ADI_ADRV9001_ILB1:
                ADI_EXPECT(adrv9001_VerifyLbProfile, device, rxProfile);
                ilbValidForTxMask |= TX_CHANNELS[0];
                break;
            case ADI_ADRV9001_ILB2:
                ADI_EXPECT(adrv9001_VerifyLbProfile, device, rxProfile);
                ilbValidForTxMask |= TX_CHANNELS[1];
                break;
            case ADI_ADRV9001_ELB1: /* Falls through */
            case ADI_ADRV9001_ELB2:
                ADI_EXPECT(adrv9001_VerifyLbProfile, device, rxProfile);
                device->devStateInfo.profilesValid |= ADI_ADRV9001_ELB_PROFILE_VALID;
                break;
            default:
                ADI_SHOULD_NOT_EXECUTE(device);
            }

            device->devStateInfo.initializedChannels |= RX_CHANNELS[i];
        }
    }

    // Validate Tx channels
    for (i = 0; i < ADI_ADRV9001_MAX_TXCHANNELS; i++)
    {
        if (ADRV9001_BF_EQUAL(init->tx.txInitChannelMask, TX_CHANNELS[i]))
        {
            txProfile = &init->tx.txProfile[i];
            ADI_EXPECT(adrv9001_VerifyTxProfile, device, txProfile);
	    device->devStateInfo.profilesValid |= ADI_ADRV9001_TX_PROFILE_VALID;
            // ILB for this Tx channel must be configured for Tx to be valid
            if (ADRV9001_BF_EQUAL(ilbValidForTxMask, TX_CHANNELS[i]))
            {
                device->devStateInfo.initializedChannels |= TX_CHANNELS[i];
            }
            else
            {
                device->devStateInfo.profilesValid = 0;
            }
        }
    }

    if ((init->clocks.rfPllPhaseSyncMode != ADI_ADRV9001_RFPLLMCS_NOSYNC) &&
        (init->clocks.rfPllPhaseSyncMode != ADI_ADRV9001_RFPLLMCS_INIT_AND_SYNC) &&
        (init->clocks.rfPllPhaseSyncMode != ADI_ADRV9001_RFPLLMCS_INIT_AND_CONTTRACK))
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            device,
            "init->clocks.rfPllPhaseSyncMode structure member has invalid option for RFPLL phase sync mode");
        ADI_ERROR_RETURN(device->common.error.newAction);
    }

    ADI_API_RETURN(device);
}
