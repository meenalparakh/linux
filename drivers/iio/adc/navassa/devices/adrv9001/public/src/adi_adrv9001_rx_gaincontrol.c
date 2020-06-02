/**
* \file
* \brief Contains AGC features related function implementation defined in
* adi_adrv9001_agc.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_user.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_rx_gaincontrol.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_error.h"

#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adrv9001_rx.h"

#include "adrv9001_bf_analog_rx_mem_map.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_bf_nvs_regmap_core_1.h"
#include "adrv9001_bf_nvs_regmap_rx.h"
#include "adrv9001_bf_nvs_regmap_rxb.h"
#include "adrv9001_decode_bf_enum.h"
#include "adi_adrv9001_gpio.h"
#include "adrv9001_init.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_radio.h"

/* Forward declaration */
static int32_t adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Set_Validate(adi_adrv9001_Device_t *device,
                                                                        adi_common_ChannelNumber_e channel,
                                                                        uint8_t minGainIndex,
                                                                        uint8_t maxGainIndex);

static int32_t __maybe_unused adi_adrv9001_Rx_GainControl_Mode_Set_Validate(adi_adrv9001_Device_t *device,
									    adi_common_ChannelNumber_e channel,
									    adi_adrv9001_RxGainControlMode_e gainCtrlMode)
{
    ADI_API_ENTRY_EXPECT(device);
    ADI_RANGE_CHECK(device, gainCtrlMode, ADI_ADRV9001_RX_GAIN_CONTROL_MODE_SPI, ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_Mode_Set(adi_adrv9001_Device_t *device,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_RxGainControlMode_e gainCtrlMode)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e rxChannelBitfieldAddr = ADRV9001_BF_RXB1_CORE;
    uint8_t bfVal = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_Mode_Set_Validate, device, channel, gainCtrlMode);

    ADI_EXPECT(adrv9001_RxBitfieldAddressGet, device, channel, &rxChannelBitfieldAddr);

    if (gainCtrlMode == ADI_ADRV9001_RX_GAIN_CONTROL_MODE_SPI ||
        gainCtrlMode == ADI_ADRV9001_RX_GAIN_CONTROL_MODE_PIN)
    {
        bfVal = 0;
    }
    else
    {
        bfVal = 2;
    }
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSetupBfSet, device, rxChannelBitfieldAddr, bfVal);

    if (gainCtrlMode == ADI_ADRV9001_RX_GAIN_CONTROL_MODE_SPI)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUseCountersForMgcBfSet, device, rxChannelBitfieldAddr, true);
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcManualGainPinControlBfSet, device, rxChannelBitfieldAddr, false);
    }
    else if (gainCtrlMode == ADI_ADRV9001_RX_GAIN_CONTROL_MODE_PIN)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUseCountersForMgcBfSet, device, rxChannelBitfieldAddr, true);
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcManualGainPinControlBfSet, device, rxChannelBitfieldAddr, true);
    }
    else if (gainCtrlMode == ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUseCountersForMgcBfSet, device, rxChannelBitfieldAddr, false);
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcManualGainPinControlBfSet, device, rxChannelBitfieldAddr, false);
    }
    else
    {
        ADI_SHOULD_NOT_EXECUTE(device);
    }

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Rx_GainControl_Mode_Get_Validate(adi_adrv9001_Device_t *device,
									    adi_common_ChannelNumber_e channel,
									    adi_adrv9001_RxGainControlMode_e *gainCtrlMode)
{
    ADI_API_ENTRY_PTR_EXPECT(device, gainCtrlMode);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_Mode_Get(adi_adrv9001_Device_t *device,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_RxGainControlMode_e *gainCtrlMode)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;
    uint8_t agcSetup = 0;
    uint8_t pinControl = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_Mode_Get_Validate, device, channel, gainCtrlMode);

    switch (channel)
    {
    case(ADI_CHANNEL_1):
        baseAddr = ADRV9001_BF_RXB1_CORE;
        break;
    case(ADI_CHANNEL_2):
        baseAddr = ADRV9001_BF_RXB2_CORE;
        break;
    default:
        ADI_SHOULD_NOT_EXECUTE(device);
    }

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSetupBfGet, device, baseAddr, &agcSetup);
    if (ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO == agcSetup)
    {
        *gainCtrlMode = ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO;
    }
    else
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcManualGainPinControlBfGet, device, baseAddr, &pinControl);
        if (true == pinControl)
        {
            *gainCtrlMode = ADI_ADRV9001_RX_GAIN_CONTROL_MODE_PIN;
        }
        else
        {
            *gainCtrlMode = ADI_ADRV9001_RX_GAIN_CONTROL_MODE_SPI;
        }
    }

    ADI_API_RETURN(device);
}


static int32_t __maybe_unused adi_adrv9001_Rx_GainControl_Configure_Validate(adi_adrv9001_Device_t *device,
									     adi_common_ChannelNumber_e channel,
									     adi_adrv9001_GainControlCfg_t *agcCfg)
{
    static const uint8_t PEAK_WAIT_TIME_MAX = 0x1F;
    static const uint32_t GAIN_UPDATE_COUNTER_MAX = 0x003FFFFF;
    static const uint8_t UNDER_RANGE_HIGH_POWER_THRESH_MAX = 0x7F;
    static const uint8_t UNDER_RANGE_LOW_POWER_THRESH_MAX = 0x0F;
    static const uint8_t UNDER_RANGE_HIGH_POWER_GAIN_STEP_RECOVERY_MAX = 0x1F;
    static const uint8_t UNDER_RANGE_LOW_POWER_GAIN_STEP_RECOVERY_MAX = 0x1F;
    static const uint8_t POWER_MEASUREMENT_DURATION_MAX = 0x1F;
    static const uint8_t SLOW_LOOP_SETTLING_DELAY_MAX = 0x7F;
    static const uint8_t APD_HIGH_THRESH_MAX = 0x3F;
    static const uint8_t APD_LOW_THRESH_MAX = 0x3F;
    static const uint8_t APD_LOW_GAIN_STEP_ATTACK_MAX = 0x1F;
    static const uint8_t APD_GAIN_STEP_RECOVERY_MAX = 0x1F;
    static const uint8_t HB_OVER_LOAD_DURATION_CNT_MAX = 0x07;
    static const uint8_t HB_OVER_LOAD_THRESH_CNT_MAX = 0x0F;
    static const uint8_t HB_GAIN_STEP_HIGH_RECOVERY_MAX = 0x1F;
    static const uint8_t HB_GAIN_STEP_LOW_RECOVERY_MAX = 0x1F;
    static const uint8_t HB_GAIN_STEP_ATTACK_MAX = 0x1F;
    static const uint8_t HB_GAIN_STEP_MID_RECOVERY_MAX = 0x1F;
    static const uint8_t OVER_RANGE_HIGH_POWER_THRESH_MID_RECOVERY_MAX = 0x0F;
    static const uint8_t OVER_RANGE_LOW_POWER_THRESH_MID_RECOVERY_MAX = 0x7F;
    static const uint8_t OVER_RANGE_HIGH_POWER_GAIN_STEP_ATTACK_MAX = 0x1F;
    static const uint8_t OVER_RANGE_LOW_POWER_GAIN_STEP_ATTACK_MAX = 0x1F;
    static const uint8_t CHANGE_GAIN_IF_THRESH_HIGH_MAX = 0x03;
    static const uint8_t ATTACK_DELAY_MAX = 0x3F;
    static const uint8_t UNDER_RANGE_MID_INTERVAL_MAX = 0x3F;
    static const uint8_t UNDER_RANGE_HIGH_INTERVAL_MAX = 0x3F;
    static const uint8_t ENABLE_MAX = 0x01;
#ifdef SI_REV_A0
    static const uint16_t HB_THRESH_MAX = 0x00FF;
#else
    static const uint16_t HB_THRESH_MAX = 0x3FFF;
#endif // SI_REV_A0


    ADI_NULL_PTR_RETURN(&device->common, agcCfg);

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_EXPECT(adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Set_Validate, device, channel, agcCfg->minGainIndex, agcCfg->maxGainIndex);
    ADI_RANGE_CHECK(device, agcCfg->peakWaitTime,           0, PEAK_WAIT_TIME_MAX);
    ADI_RANGE_CHECK(device, agcCfg->gainUpdateCounter,      0, GAIN_UPDATE_COUNTER_MAX);
    ADI_RANGE_CHECK(device, agcCfg->attackDelay_us,         0, ATTACK_DELAY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->slowLoopSettlingDelay,  0, SLOW_LOOP_SETTLING_DELAY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->changeGainIfThreshHigh, 0, CHANGE_GAIN_IF_THRESH_HIGH_MAX);

    /* Power Configuration register */
    ADI_RANGE_CHECK(device, agcCfg->power.underRangeHighPowerThresh,           0, UNDER_RANGE_HIGH_POWER_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.underRangeLowPowerThresh,            0, UNDER_RANGE_LOW_POWER_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.underRangeHighPowerGainStepRecovery, 0, UNDER_RANGE_HIGH_POWER_GAIN_STEP_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.underRangeLowPowerGainStepRecovery,  0, UNDER_RANGE_LOW_POWER_GAIN_STEP_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.powerMeasurementDuration,            0, POWER_MEASUREMENT_DURATION_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.overRangeHighPowerThresh,            0, OVER_RANGE_HIGH_POWER_THRESH_MID_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.overRangeLowPowerThresh,             0, OVER_RANGE_LOW_POWER_THRESH_MID_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.overRangeHighPowerGainStepAttack,    0, OVER_RANGE_HIGH_POWER_GAIN_STEP_ATTACK_MAX);
    ADI_RANGE_CHECK(device, agcCfg->power.overRangeLowPowerGainStepAttack,     0, OVER_RANGE_LOW_POWER_GAIN_STEP_ATTACK_MAX);

    /* Agc Peak */
    ADI_RANGE_CHECK(device, agcCfg->peak.agcUnderRangeMidInterval,  0, UNDER_RANGE_MID_INTERVAL_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.agcUnderRangeHighInterval, 0, UNDER_RANGE_HIGH_INTERVAL_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.apdHighThresh,             0, APD_HIGH_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.apdLowThresh,              0, APD_LOW_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.apdGainStepAttack,         0, APD_LOW_GAIN_STEP_ATTACK_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.apdGainStepRecovery,       0, APD_GAIN_STEP_RECOVERY_MAX);

    /* HB Configuration */
    ADI_RANGE_CHECK(device, agcCfg->peak.hbOverloadDurationCount,  0, HB_OVER_LOAD_DURATION_CNT_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbOverloadThreshCount,    0, HB_OVER_LOAD_THRESH_CNT_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbGainStepHighRecovery, 0, HB_GAIN_STEP_HIGH_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbGainStepLowRecovery,  0, HB_GAIN_STEP_LOW_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbGainStepMidRecovery,  0, HB_GAIN_STEP_MID_RECOVERY_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbGainStepAttack,       0, HB_GAIN_STEP_ATTACK_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbOverloadPowerMode,    0, ENABLE_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbHighThresh,           0, HB_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbUnderRangeLowThresh,  0, HB_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbUnderRangeMidThresh,  0, HB_THRESH_MAX);
    ADI_RANGE_CHECK(device, agcCfg->peak.hbUnderRangeHighThresh, 0, HB_THRESH_MAX);

    ADI_RANGE_CHECK(device,
                    agcCfg->peak.feedback_low_threshold_counter_exceeded,
                    ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED,
                    ADI_ADRV9001_GPIO_PIN_CRUMB_15_14);
    ADI_RANGE_CHECK(device,
                    agcCfg->peak.feedback_high_threshold_counter_exceeded,
                    ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED,
                    ADI_ADRV9001_GPIO_PIN_CRUMB_15_14);
    ADI_RANGE_CHECK(device,
                    agcCfg->power.feedback_lowThreshold_gainChange,
                    ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED,
                    ADI_ADRV9001_GPIO_PIN_CRUMB_15_14);
    ADI_RANGE_CHECK(device,
                    agcCfg->power.feedback_high_threshold_exceeded,
                    ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED,
                    ADI_ADRV9001_GPIO_PIN_CRUMB_15_14);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_Configure(adi_adrv9001_Device_t *device,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_GainControlCfg_t *agcCfg)
{
    static const uint8_t HB_THRESH_CONFIG = 0x3;
    static const uint8_t POWER_INPUT_SELECT = 0x0;
    static const uint8_t POWER_LOG_SHIFT = 0x1;

    adrv9001_BfNvsRegmapRxChanAddr_e rxAddr = ADRV9001_BF_RX1_CORE;
    adrv9001_BfNvsRegmapRxbChanAddr_e rxbAddr = ADRV9001_BF_RXB1_CORE;
    adrv9001_BfAnalogRxMemMapChanAddr_e anaAddr = ADRV9001_BF_RX1_ANA;
    uint8_t bfValue = 0;
    uint8_t regId = 0;
    uint32_t threshCalc = 0;

    /* apdHighThresh and apdLowThresh have to be written to 7 registers */
    static uint8_t REGISTER_ID = 7;

    uint8_t controlMuxAddress = 0;
    adi_adrv9001_RxGainControlMode_e gainCtrlMode = ADI_ADRV9001_RX_GAIN_CONTROL_MODE_SPI;
    adrv9001_BfNvsRegmapRxChanAddr_e rxChanAddress = ADRV9001_BF_RX1_CORE;
    adrv9001_BfNvsRegmapRxbChanAddr_e rxChannelBitfieldAddr = ADRV9001_BF_RXB1_CORE;

    static const uint8_t ADI_ADRV9001_GAIN_PEAK_ADDRESS = 0x28;
    static const uint8_t ADI_ADRV9001_GAIN_PEAK_POWER_ADDRESS = 0x29;

    /* GPIO variables */
    uint8_t gpioCrumb1_0 = 0;
    uint8_t gpioCrumb3_2 = 0;
    uint8_t gpioSource1_0 = 0;
    uint8_t gpioSource3_2 = 0;
    uint16_t gpioOutEn = 0;
    static const uint16_t GPIO_SOURCE_SEL_ADDR = 0x56;
    static const uint8_t ADI_ADRV9001_GPIO_SOURCE_RX1_1_0 = 0x10; /* peak power */
    static const uint8_t ADI_ADRV9001_GPIO_SOURCE_RX1_3_2 = 0x11;
    static const uint8_t ADI_ADRV9001_GPIO_SOURCE_RX1_5_4 = 0x12; /* peak */
    static const uint8_t ADI_ADRV9001_GPIO_SOURCE_RX1_7_6 = 0x13;
    static const uint8_t ADI_ADRV9001_GPIO_SOURCE_RX2_1_0 = 0x14; /* peak power */
    static const uint8_t ADI_ADRV9001_GPIO_SOURCE_RX2_3_2 = 0x15;
    static const uint8_t ADI_ADRV9001_GPIO_SOURCE_RX2_5_4 = 0x16; /* peak */
    static const uint8_t ADI_ADRV9001_GPIO_SOURCE_RX2_7_6 = 0x17;
#ifdef __KERNEL__
    /* APD Low Frequency MITIGATION Mode Setup */
    static const uint8_t APD_LOW_FREQ_ADCOVRG_2ND_HIGH_COUNTER = 3;
    static const uint8_t APD_LOW_FREQ_ERROR_MITIGATION_MODE = 1;

    static const uint32_t APD_LOW_FREQ_THRESH_SUBTRACTION_FACTOR = 33352;
    static const uint32_t APD_LOW_FREQ_THRESH_DIVISION_FACTOR = 133352;
    static const uint32_t APD_LOW_FREQ_THRESH_MULTIPLICATION_FACTOR = 100000;
#endif

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_Configure_Validate, device, channel, agcCfg);

    rxAddr = adrv9001_RxChanAddr_Get(channel);
    rxbAddr = adrv9001_RxbChanAddr_Get(channel);
    anaAddr = adrv9001_AnalogRxChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcPeakWaitTimeBfSet,           device, rxbAddr, agcCfg->peakWaitTime);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfSet,       device, rxbAddr, agcCfg->maxGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfSet,       device, rxbAddr, agcCfg->minGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcGainUpdateCounterBfSet,      device, rxbAddr, agcCfg->gainUpdateCounter);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAttackDelayBfSet,            device, rxbAddr, agcCfg->attackDelay_us);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcEnableFastRecoveryLoopBfSet, device, rxbAddr, agcCfg->enableFastRecoveryLoop);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLowThsPreventGainIncBfSet,   device, rxbAddr, agcCfg->lowThreshPreventGainInc);

    bfValue = agcCfg->changeGainIfThreshHigh & 0x01;
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcChangeGainIfUlbthHighBfSet, device, rxbAddr, bfValue);
    bfValue = (agcCfg->changeGainIfThreshHigh >> 1) & 0x01;
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcChangeGainIfAdcovrgHighBfSet, device, rxbAddr, bfValue);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcPeakThresholdGainControlModeBfSet, device, rxbAddr, agcCfg->agcMode);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcResetOnRxonBfSet,                   device, rxbAddr, agcCfg->resetOnRxon);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcResetOnRxonGainIndexBfSet,          device, rxbAddr, agcCfg->resetOnRxonGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcEnableSyncPulseForGainCounterBfSet, device, rxbAddr, agcCfg->enableSyncPulseForGainCounter);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSlowLoopSettlingDelayBfSet,         device, rxbAddr, agcCfg->slowLoopSettlingDelay);

    /* Agc Peak */
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval0BfSet,     device, rxbAddr, agcCfg->peak.agcUnderRangeLowInterval);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval1MultBfSet, device, rxbAddr, agcCfg->peak.agcUnderRangeMidInterval);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval2MultBfSet, device, rxbAddr, agcCfg->peak.agcUnderRangeHighInterval);

    for (regId = 0; regId < REGISTER_ID; regId++)
    {
        ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetUlbthBfSet, device, anaAddr, regId, agcCfg->peak.apdHighThresh);
        ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetLlbthBfSet, device, anaAddr, regId, agcCfg->peak.apdLowThresh);
    }

    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetDecayBfSet,  device, anaAddr, 0);
    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxTiaForceUpdateBfSet, device, anaAddr, 1);
    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxTiaForceUpdateBfSet, device, anaAddr, 0);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlbThresholdExceededCounterBfSet, device, rxbAddr, agcCfg->peak.apdUpperThreshPeakExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLlbThresholdExceededCounterBfSet, device, rxbAddr, agcCfg->peak.apdLowerThreshPeakExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlbGainStepBfSet,                 device, rxbAddr, agcCfg->peak.apdGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLlbGainStepBfSet,                 device, rxbAddr, agcCfg->peak.apdGainStepRecovery);

    /*HB Configuration register*/
    ADI_EXPECT(adrv9001_NvsRegmapRxRxEnableDecOverloadBfSet,         device, rxAddr, agcCfg->peak.enableHbOverload);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadDurationCountBfSet,  device, rxAddr, agcCfg->peak.hbOverloadDurationCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadThresholdCountBfSet, device, rxAddr, agcCfg->peak.hbOverloadThreshCount);

    /* HB */
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadUpperThresholdBfSet,     device, rxAddr, agcCfg->peak.hbHighThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt0LowerThresholdBfSet, device, rxAddr, agcCfg->peak.hbUnderRangeLowThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt1LowerThresholdBfSet, device, rxAddr, agcCfg->peak.hbUnderRangeMidThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadLowerThresholdBfSet,     device, rxAddr, agcCfg->peak.hbUnderRangeHighThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcHighOvrgExceededCounterBfSet,            device, rxbAddr, agcCfg->peak.hbUpperThreshPeakExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcLowOvrgExceededCounterBfSet,             device, rxbAddr, agcCfg->peak.hbUnderRangeHighThreshExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowGainStepBfSet,                       device, rxbAddr, agcCfg->peak.hbGainStepHighRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowInt0GainStepBfSet,                   device, rxbAddr, agcCfg->peak.hbGainStepLowRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowInt1GainStepBfSet,                   device, rxbAddr, agcCfg->peak.hbGainStepMidRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgHighGainStepBfSet,                      device, rxbAddr, agcCfg->peak.hbGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadPowerModeBfSet,                    device, rxAddr, agcCfg->peak.hbOverloadPowerMode);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecThresholdConfigBfSet,                      device, rxAddr, HB_THRESH_CONFIG);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcovrgLowInt1CounterBfSet,                 device, rxbAddr, agcCfg->peak.hbUnderRangeMidThreshExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcovrgLowInt0CounterBfSet,                 device, rxbAddr, agcCfg->peak.hbUnderRangeLowThreshExceededCount);

#ifndef __KERNEL__
    /* APD Low Frequency MITIGATION Mode Setup */
    static const uint8_t APD_LOW_FREQ_ADCOVRG_2ND_HIGH_COUNTER = 3;
    static const uint8_t APD_LOW_FREQ_ERROR_MITIGATION_MODE = 1;

    static const uint32_t APD_LOW_FREQ_THRESH_SUBTRACTION_FACTOR = 33352;
    static const uint32_t APD_LOW_FREQ_THRESH_DIVISION_FACTOR = 133352;
    static const uint32_t APD_LOW_FREQ_THRESH_MULTIPLICATION_FACTOR = 100000;
#endif
    /* The new formula should be:
       "decimated_data_overload_secondary_upper_threshold = Round((hbHighThresh - 0.33352)/1.33352)"
       The equation above is derived based on the fact that the 2nd high counter should be set 1.5dB below the hbHighThresh value.
       20*log10((hbHighThresh + 1)/2^15) - 20*log10((secondary_upper_threshold + 1)/2^15) = 1.5
    */
    threshCalc = (agcCfg->peak.hbHighThresh * APD_LOW_FREQ_THRESH_MULTIPLICATION_FACTOR) - APD_LOW_FREQ_THRESH_SUBTRACTION_FACTOR;
    threshCalc = DIV_ROUND_CLOSEST(threshCalc, APD_LOW_FREQ_THRESH_DIVISION_FACTOR);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecimatedDataOverloadSecondaryUpperThresholdBfSet, device, rxbAddr, threshCalc);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAdcovrg2ndHighCounterBfSet,            device, rxbAddr, APD_LOW_FREQ_ADCOVRG_2ND_HIGH_COUNTER);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcApdLowFreqErrorMitigationModeBfSet, device, rxbAddr, APD_LOW_FREQ_ERROR_MITIGATION_MODE);

    /* Power Configuration register */
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerEnableMeasBfSet,                 device, rxbAddr, agcCfg->power.powerEnableMeasurement);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerInputSelectBfSet,                device, rxbAddr, POWER_INPUT_SELECT);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower1ThresholdBfSet,                 device, rxbAddr, agcCfg->power.underRangeLowPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower0ThresholdBfSet,                 device, rxbAddr, agcCfg->power.underRangeHighPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower0ThresholdExceededGainStepBfSet, device, rxbAddr, agcCfg->power.underRangeHighPowerGainStepRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower1ThresholdExceededGainStepBfSet, device, rxbAddr, agcCfg->power.underRangeLowPowerGainStepRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerMeasurementDurationBfSet,        device, rxbAddr, agcCfg->power.powerMeasurementDuration);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerWaitDelayBfSet,                  device, rxbAddr, agcCfg->power.powerMeasurementDelay);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDurationBfSet,          device, rxbAddr, agcCfg->power.rxTddPowerMeasDuration);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDelayBfSet,             device, rxbAddr, agcCfg->power.rxTddPowerMeasDelay);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLockLevelBfSet,                       device, rxbAddr, agcCfg->power.overRangeLowPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper1ThresholdBfSet,                 device, rxbAddr, agcCfg->power.overRangeHighPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerLogShiftBfSet,                   device, rxbAddr, POWER_LOG_SHIFT);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper1ThresholdExceededGainStepBfSet, device, rxbAddr, agcCfg->power.overRangeHighPowerGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper0ThresholdExceededGainStepBfSet, device, rxbAddr, agcCfg->power.overRangeLowPowerGainStepAttack);

    /* External LNA */
    ADI_EXPECT(adrv9001_NvsRegmapRxbExtLnaSettlingDelayBfSet, device, rxbAddr, agcCfg->extLna.settlingDelay);

    if (agcCfg->agcMode == ADI_ADRV9001_RX_GAIN_CONTROL_DETECTION_MODE_PEAK)
    {
        controlMuxAddress = ADI_ADRV9001_GAIN_PEAK_ADDRESS;
    }
    else
    {
        controlMuxAddress = ADI_ADRV9001_GAIN_PEAK_POWER_ADDRESS;
    }

    ADI_EXPECT(adi_adrv9001_Rx_GainControl_Mode_Get, device, channel, &gainCtrlMode);

    rxChanAddress = adrv9001_RxChanAddr_Get(channel);
    rxChannelBitfieldAddr = adrv9001_RxbChanAddr_Get(channel);

    if (gainCtrlMode == ADI_ADRV9001_RX_GAIN_CONTROL_MODE_SPI)
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUseCountersForMgcBfSet, device, rxChannelBitfieldAddr, true);
    }
    else /* AGC mode */
    {
        ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUseCountersForMgcBfSet, device, rxChannelBitfieldAddr, false);
    }

    /* Determine crumbs and sources to set */
    if (agcCfg->agcMode == ADI_ADRV9001_RX_GAIN_CONTROL_DETECTION_MODE_PEAK)
    {
        gpioCrumb1_0 = (uint8_t)agcCfg->peak.feedback_low_threshold_counter_exceeded;
        gpioCrumb3_2 = (uint8_t)agcCfg->peak.feedback_high_threshold_counter_exceeded;
        if (ADI_CHANNEL_1 == channel)
        {
            gpioSource1_0 = ADI_ADRV9001_GPIO_SOURCE_RX1_5_4;
            gpioSource3_2 = ADI_ADRV9001_GPIO_SOURCE_RX1_7_6;
        }
        else
        {
            gpioSource1_0 = ADI_ADRV9001_GPIO_SOURCE_RX2_5_4;
            gpioSource3_2 = ADI_ADRV9001_GPIO_SOURCE_RX2_7_6;
        }
    }
    else
    {
        gpioCrumb1_0 = (uint8_t)agcCfg->power.feedback_lowThreshold_gainChange;
        gpioCrumb3_2 = (uint8_t)agcCfg->power.feedback_high_threshold_exceeded;
        if (ADI_CHANNEL_1 == channel)
        {
            gpioSource1_0 = ADI_ADRV9001_GPIO_SOURCE_RX1_1_0;
            gpioSource3_2 = ADI_ADRV9001_GPIO_SOURCE_RX1_3_2;
        }
        else
        {
            gpioSource1_0 = ADI_ADRV9001_GPIO_SOURCE_RX2_1_0;
            gpioSource3_2 = ADI_ADRV9001_GPIO_SOURCE_RX2_3_2;
        }
    }

    /* Configure pins as outputs */
    ADI_EXPECT(adrv9001_NvsRegmapCoreNvsGpioDirectionControlOeBfGet, device, ADRV9001_BF_CORE, &gpioOutEn);
    gpioOutEn |= (1 << (gpioCrumb1_0 * 2 - 1)) | (1 << (gpioCrumb1_0 * 2 - 2));
    gpioOutEn |= (1 << (gpioCrumb3_2 * 2 - 1)) | (1 << (gpioCrumb3_2 * 2 - 2));
    ADI_EXPECT(adrv9001_NvsRegmapCoreNvsGpioDirectionControlOeBfSet, device, ADRV9001_BF_CORE, gpioOutEn);

    /* Configure source */
    ADRV9001_SPIWRITEBYTE(device, "GPIO_SOURCE_SEL", (GPIO_SOURCE_SEL_ADDR + gpioCrumb1_0 - 1), gpioSource1_0);
    ADRV9001_SPIWRITEBYTE(device, "GPIO_SOURCE_SEL", (GPIO_SOURCE_SEL_ADDR + gpioCrumb3_2 - 1), gpioSource3_2);

    ADI_EXPECT(adrv9001_NvsRegmapRxControlOutMuxSelBfSet, device, rxChanAddress, controlMuxAddress);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Rx_GainControl_Inspect_Validate(adi_adrv9001_Device_t *device,
									   adi_common_ChannelNumber_e channel,
									   adi_adrv9001_GainControlCfg_t *agcCfg)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&device->common, agcCfg);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_Inspect(adi_adrv9001_Device_t *device,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_GainControlCfg_t *agcCfg)
{
    adrv9001_BfNvsRegmapRxChanAddr_e rxAddr = ADRV9001_BF_RX1_CORE;
    adrv9001_BfNvsRegmapRxbChanAddr_e rxbAddr = ADRV9001_BF_RXB1_CORE;
    adrv9001_BfAnalogRxMemMapChanAddr_e anaAddr = ADRV9001_BF_RX1_ANA;
    uint8_t bfValue = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_Inspect_Validate, device, channel, agcCfg);

    rxAddr = adrv9001_RxChanAddr_Get(channel);
    rxbAddr = adrv9001_RxbChanAddr_Get(channel);
    anaAddr = adrv9001_AnalogRxChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcPeakWaitTimeBfGet,      device, rxbAddr, &agcCfg->peakWaitTime);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfGet,  device, rxbAddr, &agcCfg->maxGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfGet,  device, rxbAddr, &agcCfg->minGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcGainUpdateCounterBfGet, device, rxbAddr, &agcCfg->gainUpdateCounter);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAttackDelayBfGet,       device, rxbAddr, &agcCfg->attackDelay_us);

    /* AGC Control register - Slowloop_config*/
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcEnableFastRecoveryLoopBfGet, device, rxbAddr, &bfValue);
    agcCfg->enableFastRecoveryLoop = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLowThsPreventGainIncBfGet, device, rxbAddr, &bfValue);
    agcCfg->lowThreshPreventGainInc = bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcChangeGainIfUlbthHighBfGet, device, rxbAddr, &bfValue);
    agcCfg->changeGainIfThreshHigh = (bfValue & 0x01);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcChangeGainIfAdcovrgHighBfGet, device, rxbAddr, &bfValue);
    agcCfg->changeGainIfThreshHigh |= ((bfValue << 1) & 0x02);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcPeakThresholdGainControlModeBfGet, device, rxbAddr, &bfValue);
    agcCfg->agcMode = (adi_adrv9001_RxGainControlDetectionMode_e)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcResetOnRxonBfGet, device, rxbAddr, &bfValue);
    agcCfg->resetOnRxon = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcResetOnRxonGainIndexBfGet, device, rxbAddr, &agcCfg->resetOnRxonGainIndex);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcEnableSyncPulseForGainCounterBfGet, device, rxbAddr, &bfValue);
    agcCfg->enableSyncPulseForGainCounter = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSlowLoopSettlingDelayBfGet, device, rxbAddr, &agcCfg->slowLoopSettlingDelay);

    /* Agc Peak */
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval0BfGet,             device, rxbAddr, &agcCfg->peak.agcUnderRangeLowInterval);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval1MultBfGet,         device, rxbAddr, &agcCfg->peak.agcUnderRangeMidInterval);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUrangeInterval2MultBfGet,         device, rxbAddr, &agcCfg->peak.agcUnderRangeHighInterval);
    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetUlbthBfGet,             device, anaAddr, 0, &agcCfg->peak.apdHighThresh);
    ADI_EXPECT(adrv9001_AnalogRxMemMapOrxBlockDetLlbthBfGet,             device, anaAddr, 0, &agcCfg->peak.apdLowThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlbThresholdExceededCounterBfGet, device, rxbAddr, &agcCfg->peak.apdUpperThreshPeakExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLlbThresholdExceededCounterBfGet, device, rxbAddr, &agcCfg->peak.apdLowerThreshPeakExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlbGainStepBfGet,                 device, rxbAddr, &agcCfg->peak.apdGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLlbGainStepBfGet,                 device, rxbAddr, &agcCfg->peak.apdGainStepRecovery);

    /*HB Configuration register*/
    ADI_EXPECT(adrv9001_NvsRegmapRxRxEnableDecOverloadBfGet, device, rxAddr, &bfValue);
    agcCfg->peak.enableHbOverload = (bool)bfValue;

    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadDurationCountBfGet,                device, rxAddr,  &agcCfg->peak.hbOverloadDurationCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadThresholdCountBfGet,               device, rxAddr,  &agcCfg->peak.hbOverloadThreshCount);

#ifdef SI_REV_A0
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadUpperThresholdBfGet,     device, rxAddr,  &bfValue);
    agcCfg->peak.hbHighThresh = bfValue;
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt0LowerThresholdBfGet, device, rxAddr,  &bfValue);
    agcCfg->peak.hbUnderRangeLowThresh = bfValue;
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt1LowerThresholdBfGet, device, rxAddr,  &bfValue);
    agcCfg->peak.hbUnderRangeMidThresh = bfValue;
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadLowerThresholdBfGet,     device, rxAddr,  &bfValue);
    agcCfg->peak.hbUnderRangeHighThresh = bfValue;
#else
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadUpperThresholdBfGet,     device, rxAddr,  &agcCfg->peak.hbHighThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt0LowerThresholdBfGet, device, rxAddr,  &agcCfg->peak.hbUnderRangeLowThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadInt1LowerThresholdBfGet, device, rxAddr,  &agcCfg->peak.hbUnderRangeMidThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecimatedDataOverloadLowerThresholdBfGet,     device, rxAddr,  &agcCfg->peak.hbUnderRangeHighThresh);
#endif // SI_REV_A0

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcHighOvrgExceededCounterBfGet,            device, rxbAddr, &agcCfg->peak.hbUpperThreshPeakExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcLowOvrgExceededCounterBfGet,             device, rxbAddr, &agcCfg->peak.hbUnderRangeHighThreshExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowGainStepBfGet,                       device, rxbAddr, &agcCfg->peak.hbGainStepHighRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowInt0GainStepBfGet,                   device, rxbAddr, &agcCfg->peak.hbGainStepLowRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgLowInt1GainStepBfGet,                   device, rxbAddr, &agcCfg->peak.hbGainStepMidRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcOvrgHighGainStepBfGet,                      device, rxbAddr, &agcCfg->peak.hbGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxRxDecOverloadPowerModeBfGet,                    device, rxAddr,  &agcCfg->peak.hbOverloadPowerMode);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcovrgLowInt1CounterBfGet,                 device, rxbAddr, &agcCfg->peak.hbUnderRangeMidThreshExceededCount);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcAdcovrgLowInt0CounterBfGet,                 device, rxbAddr, &agcCfg->peak.hbUnderRangeLowThreshExceededCount);

    /* Power Configuration register */
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerEnableMeasBfGet, device, rxbAddr, &bfValue);
    agcCfg->power.powerEnableMeasurement = (bool)bfValue;
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower1ThresholdBfGet,                 device, rxbAddr, &agcCfg->power.underRangeLowPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower0ThresholdBfGet,                 device, rxbAddr, &agcCfg->power.underRangeHighPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower0ThresholdExceededGainStepBfGet, device, rxbAddr, &agcCfg->power.underRangeHighPowerGainStepRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLower1ThresholdExceededGainStepBfGet, device, rxbAddr, &agcCfg->power.underRangeLowPowerGainStepRecovery);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerMeasurementDurationBfGet,        device, rxbAddr, &agcCfg->power.powerMeasurementDuration);
    ADI_EXPECT(adrv9001_NvsRegmapRxbDecPowerWaitDelayBfGet,                  device, rxbAddr, &agcCfg->power.powerMeasurementDelay);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDurationBfGet,          device, rxbAddr, &agcCfg->power.rxTddPowerMeasDuration);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUlSigPowerMeasDelayBfGet,             device, rxbAddr, &agcCfg->power.rxTddPowerMeasDelay);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcLockLevelBfGet,                       device, rxbAddr, &agcCfg->power.overRangeLowPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper1ThresholdBfGet,                 device, rxbAddr, &agcCfg->power.overRangeHighPowerThresh);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper1ThresholdExceededGainStepBfGet, device, rxbAddr, &agcCfg->power.overRangeHighPowerGainStepAttack);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcUpper0ThresholdExceededGainStepBfGet, device, rxbAddr, &agcCfg->power.overRangeLowPowerGainStepAttack);

    /* External LNA */
    ADI_EXPECT(adrv9001_NvsRegmapRxbExtLnaSettlingDelayBfGet, device, rxbAddr, &agcCfg->extLna.settlingDelay);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Set_Validate(adi_adrv9001_Device_t *device,
                                                                        adi_common_ChannelNumber_e channel,
                                                                        uint8_t minGainIndex,
                                                                        uint8_t maxGainIndex)
{
    uint8_t absMinGainIdx = 0;
    uint8_t absMaxGainIdx = 0;

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    /* TODO: Remove the fields from devStateInfo - requires reading and parsing gain table or storing values in firmware */
    if (ADI_CHANNEL_1 == channel)
    {
        absMinGainIdx = device->devStateInfo.gainIndexes.rx1MinGainIndex;
        absMaxGainIdx = device->devStateInfo.gainIndexes.rx1MaxGainIndex;
    }
    else
    {
        absMinGainIdx = device->devStateInfo.gainIndexes.rx2MinGainIndex;
        absMaxGainIdx = device->devStateInfo.gainIndexes.rx2MaxGainIndex;
    }
    ADI_RANGE_CHECK(device, minGainIndex, absMinGainIdx, (maxGainIndex - 1));
    ADI_RANGE_CHECK(device, maxGainIndex, (minGainIndex + 1), absMaxGainIdx);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Set(adi_adrv9001_Device_t *device,
                                                        adi_common_ChannelNumber_e channel,
                                                        uint8_t minGainIndex,
                                                        uint8_t maxGainIndex)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Set_Validate, device, channel, minGainIndex, maxGainIndex);

    baseAddr = adrv9001_RxbChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfSet, device, baseAddr, minGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfSet, device, baseAddr, maxGainIndex);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Get_Validate(adi_adrv9001_Device_t *device,
										       adi_common_ChannelNumber_e channel,
										       uint8_t *minGainIndex,
										       uint8_t *maxGainIndex)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&device->common, minGainIndex);
    ADI_NULL_PTR_RETURN(&device->common, maxGainIndex);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Get(adi_adrv9001_Device_t *device,
                                                        adi_common_ChannelNumber_e channel,
                                                        uint8_t *minGainIndex,
                                                        uint8_t *maxGainIndex)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Get_Validate, device, channel, minGainIndex, maxGainIndex);

    baseAddr = adrv9001_RxbChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMaximumGainIndexBfGet, device, baseAddr, maxGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcMinimumGainIndexBfGet, device, baseAddr, minGainIndex);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Rx_GainControl_Reset_Validate(adi_adrv9001_Device_t *device,
									 adi_common_ChannelNumber_e channel)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_Reset(adi_adrv9001_Device_t *device, adi_common_ChannelNumber_e channel)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_Reset_Validate, device, channel);

    baseAddr = adrv9001_RxbChanAddr_Get(channel);

    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSoftResetBfSet, device, baseAddr, true);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcSoftResetBfSet, device, baseAddr, false);

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Rx_GainControl_PinMode_Configure_Validate(adi_adrv9001_Device_t *device,
										     adi_common_ChannelNumber_e channel,
										     adi_adrv9001_RxGainControlPinCfg_t *config)
{
    static const uint8_t MAX_STEP_SIZE = 8;

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    /* Note: config->minGainIndex and ->maxGainIndex are validated by the called function, no need to duplicate */

    ADI_RANGE_CHECK(device, config->incrementStepSize, 0, MAX_STEP_SIZE);
    ADI_RANGE_CHECK(device, config->decrementStepSize, 0, MAX_STEP_SIZE);

    ADI_RANGE_CHECK(device, config->incrementPin, ADI_ADRV9001_GPIO_DIGITAL_00, ADI_ADRV9001_GPIO_DIGITAL_15);
    ADI_RANGE_CHECK(device, config->decrementPin, ADI_ADRV9001_GPIO_DIGITAL_00, ADI_ADRV9001_GPIO_DIGITAL_15);
    if (config->incrementPin == config->decrementPin)
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         config->decrementPin,
                         "Invalid pin assignment. config->incrementPin cannot be the same as config->decrementPin");
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_PinMode_Configure(adi_adrv9001_Device_t *device,
                                                      adi_common_ChannelNumber_e channel,
                                                      adi_adrv9001_RxGainControlPinCfg_t *config)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_PinMode_Configure_Validate, device, channel, config);

    baseAddr = adrv9001_RxbChanAddr_Get(channel);

    /* Set gain parameters */
    ADI_EXPECT(adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Set, device, channel, config->minGainIndex, config->maxGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcIncrGainStepSizeBfSet, device, baseAddr, (config->incrementStepSize - 1));
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcDecrGainStepSizeBfSet, device, baseAddr, (config->decrementStepSize - 1));

    /* Set up GPIO pins */
    ADI_EXPECT(adi_adrv9001_gpio_ManualInput_Configure, device, config->incrementPin);
    ADI_EXPECT(adi_adrv9001_gpio_ManualInput_Configure, device, config->decrementPin);
    if (ADI_CHANNEL_1 == channel)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx1IncgainGpioSelectBfSet, device, ADRV9001_BF_CORE_1, (config->incrementPin - 1));
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx1DecgainGpioSelectBfSet, device, ADRV9001_BF_CORE_1, (config->decrementPin - 1));

        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx1IncgainGpioMaskBfSet, device, ADRV9001_BF_CORE_1, 0);
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx1DecgainGpioMaskBfSet, device, ADRV9001_BF_CORE_1, 0);
    }
    else if (ADI_CHANNEL_2 == channel)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx2IncgainGpioSelectBfSet, device, ADRV9001_BF_CORE_1, (config->incrementPin - 1));
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx2DecgainGpioSelectBfSet, device, ADRV9001_BF_CORE_1, (config->decrementPin - 1));

        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx2IncgainGpioMaskBfSet, device, ADRV9001_BF_CORE_1, 0);
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx2DecgainGpioMaskBfSet, device, ADRV9001_BF_CORE_1, 0);
    }
    else
    {
        ADI_SHOULD_NOT_EXECUTE(device);
    }

    ADI_API_RETURN(device);
}

static int32_t __maybe_unused adi_adrv9001_Rx_GainControl_PinMode_Inspect_Validate(adi_adrv9001_Device_t *device,
										   adi_common_ChannelNumber_e channel,
										   adi_adrv9001_RxGainControlPinCfg_t *config)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&device->common, config);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_GainControl_PinMode_Inspect(adi_adrv9001_Device_t *device,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_adrv9001_RxGainControlPinCfg_t *config)
{
    adrv9001_BfNvsRegmapRxbChanAddr_e baseAddr = ADRV9001_BF_RXB1_CORE;
    uint8_t bfVal = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_GainControl_PinMode_Inspect_Validate, device, channel, config);

    baseAddr = adrv9001_RxbChanAddr_Get(channel);

    /* Get gain parameters */
    ADI_EXPECT(adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Get, device, channel, &config->minGainIndex, &config->maxGainIndex);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcIncrGainStepSizeBfGet, device, baseAddr, &config->incrementStepSize);
    ADI_EXPECT(adrv9001_NvsRegmapRxbAgcDecrGainStepSizeBfGet, device, baseAddr, &config->decrementStepSize);
    config->incrementStepSize += 1;
    config->decrementStepSize += 1;

    /* Set up GPIO pins */
    if (ADI_CHANNEL_1 == channel)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx1IncgainGpioSelectBfGet, device, ADRV9001_BF_CORE_1, &bfVal);
        config->incrementPin = (adi_adrv9001_GpioPin_e)(bfVal + 1);
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx1DecgainGpioSelectBfGet, device, ADRV9001_BF_CORE_1, &bfVal);
        config->decrementPin = (adi_adrv9001_GpioPin_e)(bfVal + 1);
    }
    else if (ADI_CHANNEL_2 == channel)
    {
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx2IncgainGpioSelectBfGet, device, ADRV9001_BF_CORE_1, &bfVal);
        config->incrementPin = (adi_adrv9001_GpioPin_e)(bfVal + 1);
        ADI_EXPECT(adrv9001_NvsRegmapCore1Rx2DecgainGpioSelectBfGet, device, ADRV9001_BF_CORE_1, &bfVal);
        config->decrementPin = (adi_adrv9001_GpioPin_e)(bfVal + 1);
    }
    else
    {
        ADI_SHOULD_NOT_EXECUTE(device);
    }

    ADI_API_RETURN(device);
}
