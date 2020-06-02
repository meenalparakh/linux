/**
 * \file
 * \brief Functions for FPGA9001 Synchronous Serial Interface (SSI) configuration
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_adrv9001_user.h"
#include "adi_common_error.h"
#include "adi_common_macros.h"
#include "adi_common_types.h"
#include "adrv9001_reg_addr_macros.h"
#include "adi_fpga9001_ssi.h"
#include "adi_fpga9001_hal.h"
#include "fpga9001_ssi.h"
#include "fpga9001_bf_axi_tdd_frame.h"
#include "fpga9001_bf_axi_tdd_enable.h"
#include "fpga9001_bf_axi_adrv9001_rx.h"
#include "fpga9001_bf_axi_adrv9001_tx.h"
#include "fpga9001_bf_axi_adrv9001.h"

static int32_t SsiFormatToBitfieldValue_Convert(adi_fpga9001_Device_t *device, adi_fpga9001_SsiFormat_e format, uint8_t *bfVal)
{
    switch (format)
    {
    case ADI_FPGA9001_SSI_FORMAT_2_BIT_SYMBOL_DATA:
        *bfVal = 0b11;
        break;
    case ADI_FPGA9001_SSI_FORMAT_8_BIT_SYMBOL_DATA:
        *bfVal = 0b10;
        break;
    case ADI_FPGA9001_SSI_FORMAT_16_BIT_SYMBOL_DATA:    /* Falls through */
    case ADI_FPGA9001_SSI_FORMAT_12_BIT_I_Q_DATA:
        *bfVal = 0b01;
        break;
    case ADI_FPGA9001_SSI_FORMAT_16_BIT_I_Q_DATA:
        *bfVal = 0b00;
        break;
    default:
        return -2;
    }
    return 0;
}

static int32_t adi_fpga9001_ssi_Mode_Set_Validate(adi_fpga9001_Device_t *device,
                                                  adi_common_Port_e port,
                                                  adi_common_ChannelNumber_e channel,
                                                  bool sdrEnable,
                                                  bool multiLane,
                                                  adi_fpga9001_SsiFormat_e format)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_ELB);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_RANGE_CHECK(device, format, ADI_FPGA9001_SSI_FORMAT_2_BIT_SYMBOL_DATA, ADI_FPGA9001_SSI_FORMAT_16_BIT_I_Q_DATA);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_Mode_Set(adi_fpga9001_Device_t *device,
                                  adi_common_Port_e port,
                                  adi_common_ChannelNumber_e channel,
                                  bool sdrEnable,
                                  bool multiLane,
                                  adi_fpga9001_SsiFormat_e format)
{
    uint32_t baseAddr = 0;
    uint32_t regVal = 0;
    uint8_t numBitsField = 0;
    static const uint32_t SDR1_DDR0 = 0x40;
    static const uint32_t MLANE1_SLANE0 = 0x20;
    static const uint8_t SSI_CONFIG_OFFSET = 0x04;
    static const uint8_t SSI_RESET_OFFSET = 0x0C;
    static const uint8_t TX_IO_CONTROL_OFFSET = 0x08;

    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_Mode_Set_Validate, device, port, channel, sdrEnable, multiLane, format);

    baseAddr = fpga9001_BfSsiChanAddrGet(device, port, channel);
    
    ADI_EXPECT(SsiFormatToBitfieldValue_Convert, device, format, &numBitsField);
    regVal = (sdrEnable ? SDR1_DDR0 : 0) | (multiLane ? MLANE1_SLANE0 : 0) | (numBitsField << 2);

    ADI_EXPECT(fpga9001_AxiTddFrameTddframecountersresetBfSet, device, FPGA9001_BF_AXI_ADRV9001_TDD_FRAME_0, 0x1);

    ADI_EXPECT(adi_fpga9001_hal_Register_Write, device, (baseAddr + SSI_CONFIG_OFFSET), regVal);
    ADI_EXPECT(adi_fpga9001_hal_Register_Write, device, (baseAddr + SSI_RESET_OFFSET), 0x01);

    if (ADI_TX == port)
    {
        ADI_EXPECT(adi_fpga9001_hal_Register_Write, device, (baseAddr + TX_IO_CONTROL_OFFSET), 0x01);
    }

    ADI_API_RETURN(device);
}

static int32_t RegisterValueToSsiFormat_Convert(adi_fpga9001_Device_t *device, uint32_t regVal, adi_fpga9001_SsiFormat_e *format)
{
    static const uint32_t CMOS1_LVDS0 = 0x400;
    static const uint32_t NUM_BITS = 0x0C;
    bool cmos = ADRV9001_BF_EQUAL(regVal, CMOS1_LVDS0);
    uint8_t numBits = (regVal & NUM_BITS) >> 2;
    
    switch (numBits)
    {
    case 0b00:
        *format = ADI_FPGA9001_SSI_FORMAT_16_BIT_I_Q_DATA;
        break;
    case 0b01:
        *format = cmos ? ADI_FPGA9001_SSI_FORMAT_16_BIT_SYMBOL_DATA : ADI_FPGA9001_SSI_FORMAT_12_BIT_I_Q_DATA;
        break;
    case 0b10:
        *format = ADI_FPGA9001_SSI_FORMAT_8_BIT_SYMBOL_DATA;
        break;
    case 0b11:
        *format = ADI_FPGA9001_SSI_FORMAT_2_BIT_SYMBOL_DATA;
        break;
    default:
        return -2;
    }
    return 0;
}

static int32_t adi_fpga9001_ssi_Mode_Get_Validate(adi_fpga9001_Device_t *device,
                                                  adi_common_Port_e port,
                                                  adi_common_ChannelNumber_e channel,
                                                  bool *sdrEnabled,
                                                  bool *multiLane,
                                                  adi_fpga9001_SsiFormat_e *format)
{
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_ELB);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&device->common, sdrEnabled);
    ADI_NULL_PTR_RETURN(&device->common, multiLane);
    ADI_NULL_PTR_RETURN(&device->common, format);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_Mode_Get(adi_fpga9001_Device_t *device,
                                  adi_common_Port_e port,
                                  adi_common_ChannelNumber_e channel,
                                  bool *sdrEnabled,
                                  bool *multiLane,
                                  adi_fpga9001_SsiFormat_e *format)
{
    uint32_t baseAddr = 0;
    uint32_t regVal = 0;
    static const uint8_t CMOS_SSI_CONFIG_OFFSET = 0x4;
    static const uint32_t SDR1_DDR0 = 0x40;
    static const uint32_t MLANE1_SLANE0 = 0x20;
    
    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_Mode_Get_Validate, device, port, channel, sdrEnabled, multiLane, format);
    
    baseAddr = fpga9001_BfSsiChanAddrGet(device, port, channel);

    ADI_EXPECT(adi_fpga9001_hal_Register_Read, device, baseAddr + CMOS_SSI_CONFIG_OFFSET, &regVal);
    *sdrEnabled = ADRV9001_BF_EQUAL(regVal, SDR1_DDR0);
    *multiLane = ADRV9001_BF_EQUAL(regVal, MLANE1_SLANE0);
    ADI_EXPECT(RegisterValueToSsiFormat_Convert, device, regVal, format);
    
    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ssi_BytesPerSample_GetValidate(adi_fpga9001_Device_t *device, uint8_t *bytesPerSample)
{
    ADI_NULL_PTR_RETURN(&device->common, bytesPerSample);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_BytesPerSample_Get(adi_fpga9001_Device_t *device,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            uint8_t *bytesPerSample)
{
    bool sdrEnabled = false;
    bool multiLane = false;
    adi_fpga9001_SsiFormat_e format = ADI_FPGA9001_SSI_FORMAT_2_BIT_SYMBOL_DATA;

    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_BytesPerSample_GetValidate, device, bytesPerSample);

    ADI_EXPECT(adi_fpga9001_ssi_Mode_Get, device, port, channel, &sdrEnabled, &multiLane, &format);
    
    switch (format)
    {
    case ADI_FPGA9001_SSI_FORMAT_2_BIT_SYMBOL_DATA:     /* Falls through */
    case ADI_FPGA9001_SSI_FORMAT_8_BIT_SYMBOL_DATA:
        *bytesPerSample = 1;
        break;
    case ADI_FPGA9001_SSI_FORMAT_16_BIT_SYMBOL_DATA:
        *bytesPerSample = 2;
        break;
    case ADI_FPGA9001_SSI_FORMAT_12_BIT_I_Q_DATA:
        *bytesPerSample = 3;
        break;
    case ADI_FPGA9001_SSI_FORMAT_16_BIT_I_Q_DATA:
        *bytesPerSample = 4;
        break;
    default:
        ADI_SHOULD_NOT_EXECUTE(device);
    }

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_Reset(adi_fpga9001_Device_t *fpga9001Device)
{
    static const uint32_t SSI_CONFIG_TX1_FPGA_BYTE1_ADDR = 0x4305600c;
    static const uint32_t SSI_CONFIG_TX2_FPGA_BYTE1_ADDR = 0x4305700c;
    static const uint32_t SSI_CONFIG_RX1_FPGA_BYTE1_ADDR = 0x4305400c;
    static const uint32_t SSI_CONFIG_RX2_FPGA_BYTE1_ADDR = 0x4305500c;
    static const uint16_t SSI_CONFIG_SERDES_RESET = 0x1;

    ADI_EXPECT(adi_fpga9001_hal_Register_Write, fpga9001Device, SSI_CONFIG_TX1_FPGA_BYTE1_ADDR, SSI_CONFIG_SERDES_RESET);
    ADI_EXPECT(adi_fpga9001_hal_Register_Write, fpga9001Device, SSI_CONFIG_TX2_FPGA_BYTE1_ADDR, SSI_CONFIG_SERDES_RESET);
    ADI_EXPECT(adi_fpga9001_hal_Register_Write, fpga9001Device, SSI_CONFIG_RX1_FPGA_BYTE1_ADDR, SSI_CONFIG_SERDES_RESET);
    ADI_EXPECT(adi_fpga9001_hal_Register_Write, fpga9001Device, SSI_CONFIG_RX2_FPGA_BYTE1_ADDR, SSI_CONFIG_SERDES_RESET);

    ADI_API_RETURN(fpga9001Device);
}

static int32_t adi_fpga9001_ssi_Rx_TestMode_Configure_Validate(adi_fpga9001_Device_t *device,
                                                               adi_common_ChannelNumber_e channel,
                                                               adi_fpga9001_RxSsiTestModeCfg_t *ssiTestModeConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, ssiTestModeConfig);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_RANGE_CHECK(device, ssiTestModeConfig->testData, ADI_FPGA9001_SSI_TESTMODE_DATA_NORMAL, ADI_FPGA9001_SSI_TESTMODE_DATA_PRBS7);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_Rx_TestMode_Configure(adi_fpga9001_Device_t *device,
                                               adi_common_ChannelNumber_e channel,
                                               adi_fpga9001_RxSsiTestModeCfg_t *ssiTestModeConfig)
{
    uint8_t testMode = 0;
    uint8_t dataPattern[5] = { 3, 4, 5, 6, 7};
    fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddress = FPGA9001_BF_AXI_ADRV9001_RX_0;
    uint32_t timerValue = 1000;

    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_Rx_TestMode_Configure_Validate, device, channel, ssiTestModeConfig);

    if (channel == ADI_CHANNEL_2)
    {
        baseAddress = FPGA9001_BF_AXI_ADRV9001_RX_1;
    }

    if (ssiTestModeConfig->testData != ADI_FPGA9001_SSI_TESTMODE_DATA_NORMAL)
    {
        testMode = (uint8_t)ssiTestModeConfig->testData - 1;
    }

    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxDataSelBfSet, device, baseAddress, dataPattern[testMode]);

    if (ADI_FPGA9001_SSI_TESTMODE_DATA_FIXED_PATTERN == ssiTestModeConfig->testData)
    {
        ADI_EXPECT(fpga9001_AxiAdrv9001RxRxDataPatBfSet, device, baseAddress, ssiTestModeConfig->fixedDataPatternToCheck);
    }
    
    /* Delay 10us. 10ns is the approximate LSB delay */
    ADI_EXPECT(fpga9001_AxiAdrv9001TimerBfSet, device, FPGA9001_BF_AXI_ADRV9001_TOP, timerValue);
    while (timerValue != 0)
    {
        ADI_EXPECT(fpga9001_AxiAdrv9001TimerBfGet, device, FPGA9001_BF_AXI_ADRV9001_TOP, &timerValue);
    }
    ADI_EXPECT(adi_fpga9001_hal_Register_Write, device, (baseAddress + 0x08), 0x0F);    /* Write to clear error bits */

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ssi_Rx_TestMode_Inspect_Validate(adi_fpga9001_Device_t *device,
                                                             adi_common_ChannelNumber_e channel,
                                                             bool *dataError)
{
    ADI_NULL_PTR_RETURN(&device->common, dataError);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_Rx_TestMode_Inspect(adi_fpga9001_Device_t *device,
                                             adi_common_ChannelNumber_e channel,
                                             bool *dataError)
{
    uint32_t rxStatus = 0;
    fpga9001_BfAxiAdrv9001RxChanAddr_e baseAddress = FPGA9001_BF_AXI_ADRV9001_RX_0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_Rx_TestMode_Inspect_Validate, device, channel, dataError);

    if (channel == ADI_CHANNEL_2)
    {
        baseAddress = FPGA9001_BF_AXI_ADRV9001_RX_1;
    }

    ADI_EXPECT(adi_fpga9001_hal_Register_Read, device, (baseAddress + 0x08), &rxStatus);

    *dataError = ((rxStatus & 0xF) != 0) ? true : false;

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_ssi_Tx_TestMode_Configure_Validate(adi_fpga9001_Device_t *device,
                                                               adi_common_ChannelNumber_e channel,
                                                               adi_fpga9001_TxSsiTestModeCfg_t *ssiTestModeConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, ssiTestModeConfig);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_ssi_Tx_TestMode_Configure(adi_fpga9001_Device_t *device,
                                               adi_common_ChannelNumber_e channel,
                                               adi_fpga9001_TxSsiTestModeCfg_t *ssiTestModeConfig)
{
    uint8_t testMode = 0;
    uint8_t dataPattern[5] = { 3, 4, 5, 6, 7 };

    fpga9001_BfAxiAdrv9001TxChanAddr_e baseAddress = FPGA9001_BF_AXI_ADRV9001_TX_0;

    ADI_PERFORM_VALIDATION(adi_fpga9001_ssi_Tx_TestMode_Configure_Validate, device, channel, ssiTestModeConfig);

    if (channel == ADI_CHANNEL_2)
    {
        baseAddress = FPGA9001_BF_AXI_ADRV9001_TX_1;
    }

    if (ssiTestModeConfig->testData != ADI_FPGA9001_SSI_TESTMODE_DATA_NORMAL)
    {
        testMode = (uint8_t)ssiTestModeConfig->testData - 1;
    }

    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxDataSelBfSet, device, baseAddress, dataPattern[testMode]);

    if (ADI_FPGA9001_SSI_TESTMODE_DATA_FIXED_PATTERN == ssiTestModeConfig->testData)
    {
        ADI_EXPECT(fpga9001_AxiAdrv9001TxTxDataPatBfSet, device, baseAddress, ssiTestModeConfig->fixedDataPatternToTransmit);
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Ssi_Delay_Configure_Validate(adi_fpga9001_Device_t *device,
                                                         adi_fpga9001_SsiType_e ssiType,
                                                         adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{
    static const uint8_t MAX_DELAY = 31;

    ADI_RANGE_CHECK(device, ssiType, ADI_FPGA9001_SSI_TYPE_CMOS, ADI_FPGA9001_SSI_TYPE_LVDS);
    ADI_NULL_PTR_RETURN(&device->common, ssiCalibration);

    ADI_RANGE_CHECK(device, ssiCalibration->rxStrobeDelay[0], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxStrobeDelay[1], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxIDataDelay[0], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxIDataDelay[1], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxQDataDelay[0], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->rxQDataDelay[1], 0, MAX_DELAY);
    
    ADI_RANGE_CHECK(device, ssiCalibration->txClkDelay[0], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txClkDelay[1], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txStrobeDelay[0], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txStrobeDelay[1], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txIDataDelay[0], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txIDataDelay[1], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txQDataDelay[0], 0, MAX_DELAY);
    ADI_RANGE_CHECK(device, ssiCalibration->txQDataDelay[1], 0, MAX_DELAY);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Ssi_LvdsDelayConfigSet(adi_fpga9001_Device_t *device,
                                                   adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{
    /* Rx1 */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay0BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, ssiCalibration->rxIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay1BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, ssiCalibration->rxQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelaySBfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, ssiCalibration->rxStrobeDelay[0]);
    /* Unused for LVDS */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay2BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, 0);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay3BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, 0);

    /* Rx2 */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay0BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, ssiCalibration->rxIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay1BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, ssiCalibration->rxQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelaySBfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, ssiCalibration->rxStrobeDelay[1]);
    /* Unused for LVDS */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay2BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, 0);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay3BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, 0);

    /* Tx1 */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay0BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay1BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelaySBfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txStrobeDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelayCBfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txClkDelay[0]);
    /* Unused for LVDS */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay2BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, 0);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay3BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, 0);

    /* Tx2 */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay0BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay1BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelaySBfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txStrobeDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelayCBfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txClkDelay[1]);
    /* Unused for LVDS */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay2BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, 0);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay3BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, 0);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Ssi_CmosDelayConfigSet(adi_fpga9001_Device_t *device,
                                                   adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{
    /* Rx1 */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay0BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, ssiCalibration->rxIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay1BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, ssiCalibration->rxIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay2BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, ssiCalibration->rxQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay3BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, ssiCalibration->rxQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelaySBfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, ssiCalibration->rxStrobeDelay[0]);

    /* Rx2 */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay0BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, ssiCalibration->rxIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay1BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, ssiCalibration->rxIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay2BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, ssiCalibration->rxQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay3BfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, ssiCalibration->rxQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelaySBfSet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, ssiCalibration->rxStrobeDelay[1]);

    /* Tx1 */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay0BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay1BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay2BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay3BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelaySBfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txStrobeDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelayCBfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, ssiCalibration->txClkDelay[0]);

    /* Tx2 */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay0BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay1BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay2BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay3BfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelaySBfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txStrobeDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelayCBfSet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, ssiCalibration->txClkDelay[1]);

    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Ssi_Delay_Configure(adi_fpga9001_Device_t *device,
                                         adi_fpga9001_SsiType_e ssiType,
                                         adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{
    ADI_PERFORM_VALIDATION(adi_fpga9001_Ssi_Delay_Configure_Validate, device, ssiType, ssiCalibration);

    if (ADI_FPGA9001_SSI_TYPE_LVDS == ssiType)
    {
        ADI_EXPECT(adi_fpga9001_Ssi_LvdsDelayConfigSet, device, ssiCalibration);
    }
    else
    {
        ADI_EXPECT(adi_fpga9001_Ssi_CmosDelayConfigSet, device, ssiCalibration);
    }

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Ssi_CmosDelayConfigGet(adi_fpga9001_Device_t *device,
                                                   adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{
    /* Rx1 */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay0BfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, &ssiCalibration->rxIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay2BfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, &ssiCalibration->rxQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelaySBfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, &ssiCalibration->rxStrobeDelay[0]);

    /* Rx2 */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay0BfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, &ssiCalibration->rxIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay2BfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, &ssiCalibration->rxQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelaySBfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, &ssiCalibration->rxStrobeDelay[1]);

    /* Tx1 */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay0BfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, &ssiCalibration->txIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay2BfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, &ssiCalibration->txQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelaySBfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, &ssiCalibration->txStrobeDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelayCBfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, &ssiCalibration->txClkDelay[0]);

    /* Tx2 */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay0BfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, &ssiCalibration->txIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay2BfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, &ssiCalibration->txQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelaySBfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, &ssiCalibration->txStrobeDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelayCBfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, &ssiCalibration->txClkDelay[1]);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Ssi_LvdsDelayConfigGet(adi_fpga9001_Device_t *device,
                                                   adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{
    /* Rx1 */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay0BfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, &ssiCalibration->rxIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay1BfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, &ssiCalibration->rxQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelaySBfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_0, &ssiCalibration->rxStrobeDelay[0]);

    /* Rx2 */
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay0BfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, &ssiCalibration->rxIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelay1BfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, &ssiCalibration->rxQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001RxRxWrdelaySBfGet, device, FPGA9001_BF_AXI_ADRV9001_RX_1, &ssiCalibration->rxStrobeDelay[1]);

    /* Tx1 */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay0BfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, &ssiCalibration->txIDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay1BfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, &ssiCalibration->txQDataDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelaySBfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, &ssiCalibration->txStrobeDelay[0]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelayCBfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_0, &ssiCalibration->txClkDelay[0]);

    /* Tx2 */
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay0BfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, &ssiCalibration->txIDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelay1BfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, &ssiCalibration->txQDataDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelaySBfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, &ssiCalibration->txStrobeDelay[1]);
    ADI_EXPECT(fpga9001_AxiAdrv9001TxTxWrdelayCBfGet, device, FPGA9001_BF_AXI_ADRV9001_TX_1, &ssiCalibration->txClkDelay[1]);

    ADI_API_RETURN(device);
}

static int32_t adi_fpga9001_Ssi_Delay_Inspect_Validate(adi_fpga9001_Device_t *device,
                                                       adi_fpga9001_SsiType_e ssiType,
                                                       adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{
    ADI_RANGE_CHECK(device, ssiType, ADI_FPGA9001_SSI_TYPE_CMOS, ADI_FPGA9001_SSI_TYPE_LVDS);
    ADI_NULL_PTR_RETURN(&device->common, ssiCalibration);
    ADI_API_RETURN(device);
}

int32_t adi_fpga9001_Ssi_Delay_Inspect(adi_fpga9001_Device_t *device,
                                       adi_fpga9001_SsiType_e ssiType,
                                       adi_fpga9001_SsiCalibrationCfg_t *ssiCalibration)
{
    ADI_PERFORM_VALIDATION(adi_fpga9001_Ssi_Delay_Inspect_Validate, device, ssiType, ssiCalibration);

    if (ADI_FPGA9001_SSI_TYPE_LVDS == ssiType)
    {
        ADI_EXPECT(adi_fpga9001_Ssi_LvdsDelayConfigGet, device, ssiCalibration);
    }
    else
    {
        ADI_EXPECT(adi_fpga9001_Ssi_CmosDelayConfigGet, device, ssiCalibration);
    }

    ADI_API_RETURN(device);
}
