/**
* \file
* \brief Contains Utility features related function implementation defined in 
* adi_adrv9001_internal_utilities.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_user.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_internal_utilities.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_radio.h"
#include "adrv9001_bf_nvs_regmap_core.h"
#include "adrv9001_bf_nvs_regmap_core_1.h"
#include "adrv9001_bf_nvs_regmap_rx.h"
#include "adrv9001_bf_nvs_regmap_tx.h"
#include "adrv9001_init.h"

static int32_t adi_adrv9001_RamDriveCaptureConfigSetValidate(adi_adrv9001_Device_t *device, 
                                                             adi_adrv9001_RamDriveCaptureConfig_t *ramDriveCaptureConfig)
{
    static const uint8_t  IQ_SAMPLE_NUMBER_MIN = 1;
    static const uint16_t DIFFERENT_IQ_SAMPLE_NUMBER_MAX = 4096;
    static const uint16_t SAME_IQ_SAMPLE_NUMBER_MAX = 8192;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    if (ramDriveCaptureConfig->ramDriveType == ADI_ADRV9001_RAM_DRIVE_TYPE_DIFFERENT_IQ)
    {
        /*Check that sample number selected is valid while using different IQ*/
        ADI_RANGE_CHECK(device, 
                            ramDriveCaptureConfig->sampleNumber, 
                            IQ_SAMPLE_NUMBER_MIN, 
                            DIFFERENT_IQ_SAMPLE_NUMBER_MAX);
    }
    else
    {
        /*Check that sample number selected is valid while using same IQ*/
        ADI_RANGE_CHECK(device, 
                            ramDriveCaptureConfig->sampleNumber, 
                            IQ_SAMPLE_NUMBER_MIN, 
                            SAME_IQ_SAMPLE_NUMBER_MAX);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_RamDriveCapture_Configure(adi_adrv9001_Device_t *device, 
                                                                  adi_adrv9001_RamDriveCaptureConfig_t *ramDriveCaptureConfig, 
                                                                  adi_common_ChannelNumber_e channel,
                                                                  adi_common_Port_e port)
{
    uint8_t payload[5] = { 0 };
    uint8_t captureConfig = 0;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    payload[0] = adi_adrv9001_Radio_MailboxChannel_Get(port, channel);

    ADI_PERFORM_VALIDATION(adi_adrv9001_RamDriveCaptureConfigSetValidate, device, ramDriveCaptureConfig);

    captureConfig  =  (uint8_t)ramDriveCaptureConfig->ramDriveCaptureMode & 0x03;     /* [0:1] = ramDriveCaptureMode */
    captureConfig |= (((uint8_t)ramDriveCaptureConfig->ramDriveType & 0x01) << 2);    /* [2]   = ramDriveType */
    captureConfig |= (((uint8_t)ramDriveCaptureConfig->ramCaptureType & 0x03) << 3);  /* [3:4] = ramCaptureType */
    captureConfig |= (((uint8_t)ramDriveCaptureConfig->ramCapturePoint & 0x03) << 5); /* [5:6] = ramCapturePoint */

    /* Executing the SET RAM drive and capture config command */
    payload[1] = ADRV9001_ARM_OBJECTID_INTERNAL_RAM_DRIVE_AND_CAPTURE_CONFIG;
    payload[2] = captureConfig;
    payload[3] = (uint8_t)((ramDriveCaptureConfig->sampleNumber >> 0) & 0xFF);
    payload[4] = (uint8_t)((ramDriveCaptureConfig->sampleNumber >> 8) & 0xFF);

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
                   device, 
                   (uint8_t)ADRV9001_ARM_SET_OPCODE,
                   &payload[0], 
                   sizeof(payload));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        payload[1], 
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_INTERVAL_US);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_RamDriveCapture_Start(adi_adrv9001_Device_t *device,
                                           adi_adrv9001_RamDriveCaptureConfig_t *ramDriveCaptureConfig, 
                                           adi_common_ChannelNumber_e channel,
                                           adi_common_Port_e port)
{
    uint8_t payload[3] = { 0 };

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    /* Internal RAM capture uses the same ARM DM where the main stream processor binary image is loaded.
       Calling this API wipes out the main stream binary in ARM DM. Reading version information of Stream is invalid in this case.
       The only way to get back main stream binary is to go through ADRV9001 initial sequence or call adi_adrv9001_Stream_Image_Write() */
    if (ramDriveCaptureConfig->ramDriveCaptureStart == ADI_ADRV9001_RAM_DRIVE_CAPTURE_START)
    {
        device->devStateInfo.devState = (adi_adrv9001_ApiStates_e)(device->devStateInfo.devState & ~ADI_ADRV9001_STATE_STREAM_LOADED);
    }

    payload[0] = adi_adrv9001_Radio_MailboxChannel_Get(port, channel);

    /* Executing the SET RAM drive and capture start command */
    payload[1] = ADRV9001_ARM_OBJECTID_INTERNAL_RAM_DRIVE_AND_CAPTURE_START;
    payload[2] = (uint8_t)ramDriveCaptureConfig->ramDriveCaptureStart;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
                   device,
                   (uint8_t)ADRV9001_ARM_SET_OPCODE,
                   &payload[0],
                   sizeof(payload));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        payload[1],
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_INTERVAL_US);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_DeviceMemory_Read(adi_adrv9001_Device_t *device, 
                                   uint32_t address, 
                                   uint8_t returnData[], 
                                   uint32_t byteCount, 
                                   uint8_t autoIncrement)
{
    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, returnData, byteCount);

    ADI_MSG_EXPECT("DeviceMemRead failed during DMA transaction", 
                       adrv9001_DmaMemRead, 
                       device, 
                       address, 
                       returnData, 
                       byteCount, 
                       autoIncrement);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_DeviceMemory_Write(adi_adrv9001_Device_t *device, 
                                    uint32_t address, 
                                    const uint8_t data[], 
                                    uint32_t byteCount)
{
    ADI_API_ENTRY_PTR_ARRAY_EXPECT(device, data, byteCount);

    ADI_MSG_EXPECT("DeviceMemWrite failed during DMA transaction", 
                       adrv9001_DmaMemWrite, 
                       device, 
                       address, 
                       data, 
                       byteCount);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_Rx1_ControlOutMux_SsiGpio_Configure(adi_adrv9001_Device_t *device,
                                                                adi_adrv9001_ControlOutMuxCfg_t *controlOutMuxCfg)
{
    static uint32_t RX2_CLK_CMOS_PD_ADDR = 0x0083;
    static uint32_t RX2_STROBE_CMOS_PD_ADDR = 0x0086;
    static uint32_t RX2_IDATA_CMOS_PD_ADDR = 0x0089;
    static uint32_t RX2_QDATA_CMOS_PD_ADDR = 0x008C;

    static uint32_t RX2_CMOS_CONFIG_VALUE = 0x44;

    /* Disable RX2 SSI port */
    ADI_EXPECT(adrv9001_NvsRegmapCore1SsiRx2PortDisableBfSet, device, ADRV9001_BF_CORE_1, 0x01);

    /* Disable LVDS mode for RX2 */
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2ClkLvdsOeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2IdataLvdsOeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2QdataLvdsOeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx2StrobeLvdsOeBfSet, device, ADRV9001_BF_CORE, 0);

    /* Configure all RX2 SSI PAD in CMOS output mode */
    ADRV9001_SPIWRITEBYTE(device, "RX2_SSI_CMOS_GPIO_6_7", RX2_CLK_CMOS_PD_ADDR, RX2_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "RX2_SSI_CMOS_GPIO_4_5", RX2_STROBE_CMOS_PD_ADDR, RX2_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "RX2_SSI_CMOS_GPIO_0_1", RX2_IDATA_CMOS_PD_ADDR, RX2_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "RX2_SSI_CMOS_GPIO_2_3", RX2_QDATA_CMOS_PD_ADDR, RX2_CMOS_CONFIG_VALUE);

    ADI_API_RETURN(device);
}

static uint32_t adi_adrv9001_Rx2_ControlOutMux_SsiGpio_Configure(adi_adrv9001_Device_t *device,
                                                                 adi_adrv9001_ControlOutMuxCfg_t *controlOutMuxCfg)
{
    static uint32_t RX1_CLK_CMOS_PD_ADDR = 0x0077;
    static uint32_t RX1_STROBE_CMOS_PD_ADDR = 0x007A;
    static uint32_t RX1_IDATA_CMOS_PD_ADDR = 0x007D;
    static uint32_t RX1_QDATA_CMOS_PD_ADDR = 0x0080;

    static uint32_t RX1_CMOS_CONFIG_VALUE = 0x44;

    /* Disable RX1 SSI port */
    ADI_EXPECT(adrv9001_NvsRegmapCore1SsiRx1PortDisableBfSet, device, ADRV9001_BF_CORE_1, 0x01);

    /* Disable LVDS mode for RX1 */
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1ClkLvdsOeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1IdataLvdsOeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1QdataLvdsOeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreRx1StrobeLvdsOeBfSet, device, ADRV9001_BF_CORE, 0);

    /* Configure all RX1 SSI PAD in CMOS output mode */
    ADRV9001_SPIWRITEBYTE(device, "RX1_SSI_CMOS_GPIO_6_7", RX1_CLK_CMOS_PD_ADDR, RX1_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "RX1_SSI_CMOS_GPIO_4_5", RX1_STROBE_CMOS_PD_ADDR, RX1_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "RX1_SSI_CMOS_GPIO_0_1", RX1_IDATA_CMOS_PD_ADDR, RX1_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "RX1_SSI_CMOS_GPIO_2_3", RX1_QDATA_CMOS_PD_ADDR, RX1_CMOS_CONFIG_VALUE);

    ADI_API_RETURN(device);
}

static uint32_t adi_adrv9001_Tx2_ControlOutMux_SsiGpio_Configure(adi_adrv9001_Device_t *device,
                                                                 adi_adrv9001_ControlOutMuxCfg_t *controlOutMuxCfg)
{
    static uint32_t TX1_CLK_CMOS_PD_ADDR = 0x0090;
    static uint32_t TX1_STROBE_CMOS_PD_ADDR = 0x0094;
    static uint32_t TX1_IDATA_CMOS_PD_ADDR = 0x0098;
    static uint32_t TX1_QDATA_CMOS_PD_ADDR = 0x009C;

    static uint32_t TX1_CMOS_CONFIG_VALUE = 0x44;

    /* Disable TX1 SSI port */
    ADI_EXPECT(adrv9001_NvsRegmapCore1SsiTx1PortDisableBfSet, device, ADRV9001_BF_CORE_1, 0x01);

    /* Disable LVDS mode for TX1 */
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1ClkLvdsIeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1IdataLvdsIeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1QdataLvdsIeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx1StrobeLvdsIeBfSet, device, ADRV9001_BF_CORE, 0);

    /* Configure all TX1 SSI PAD in CMOS output mode */
    ADRV9001_SPIWRITEBYTE(device, "TX1_SSI_CMOS_GPIO_6_7", TX1_CLK_CMOS_PD_ADDR, TX1_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "TX1_SSI_CMOS_GPIO_4_5", TX1_STROBE_CMOS_PD_ADDR, TX1_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "TX1_SSI_CMOS_GPIO_0_1", TX1_IDATA_CMOS_PD_ADDR, TX1_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "TX1_SSI_CMOS_GPIO_2_3", TX1_QDATA_CMOS_PD_ADDR, TX1_CMOS_CONFIG_VALUE);

    ADI_API_RETURN(device);
}

static uint32_t adi_adrv9001_Tx1_ControlOutMux_SsiGpio_Configure(adi_adrv9001_Device_t *device,
                                                                 adi_adrv9001_ControlOutMuxCfg_t *controlOutMuxCfg)
{
    static uint32_t TX2_CLK_CMOS_PD_ADDR = 0x0083;
    static uint32_t TX2_STROBE_CMOS_PD_ADDR = 0x0086;
    static uint32_t TX2_IDATA_CMOS_PD_ADDR = 0x0089;
    static uint32_t TX2_QDATA_CMOS_PD_ADDR = 0x008C;

    static uint32_t TX2_CMOS_CONFIG_VALUE = 0x44;

    /* Disable TX2 SSI port */
    ADI_EXPECT(adrv9001_NvsRegmapCore1SsiTx2PortDisableBfSet, device, ADRV9001_BF_CORE_1, 0x01);

    /* Disable LVDS mode for TX2 */
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2ClkLvdsIeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2IdataLvdsIeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2QdataLvdsIeBfSet, device, ADRV9001_BF_CORE, 0);
    ADI_EXPECT(adrv9001_NvsRegmapCoreTx2StrobeLvdsIeBfSet, device, ADRV9001_BF_CORE, 0);

    /* Configure all TX2 SSI PAD in CMOS output mode */
    ADRV9001_SPIWRITEBYTE(device, "TX2_SSI_CMOS_GPIO_6_7", TX2_CLK_CMOS_PD_ADDR, TX2_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "TX2_SSI_CMOS_GPIO_4_5", TX2_STROBE_CMOS_PD_ADDR, TX2_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "TX2_SSI_CMOS_GPIO_0_1", TX2_IDATA_CMOS_PD_ADDR, TX2_CMOS_CONFIG_VALUE);
    ADRV9001_SPIWRITEBYTE(device, "TX2_SSI_CMOS_GPIO_2_3", TX2_QDATA_CMOS_PD_ADDR, TX2_CMOS_CONFIG_VALUE);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_internal_utilities_ControlOutMux_Configure_Validate(adi_adrv9001_Device_t *device,
                                                                                adi_common_Port_e port,
                                                                                adi_common_ChannelNumber_e channel,
                                                                                adi_adrv9001_ControlOutMuxCfg_t *controlOutMuxCfg)
{
    //uint8_t i = 0;
    static const uint8_t RX_CONTROL_OUT_MUX_ADDRESS_MIN = 0x18;
    static const uint8_t RX_CONTROL_OUT_MUX_ADDRESS_MAX = 0x2E;
    static const uint8_t TX_CONTROL_OUT_MUX_ADDRESS_MIN = 0x01;
    static const uint8_t TX_CONTROL_OUT_MUX_ADDRESS_MAX = 0x05;
    static const uint8_t MAIN_CONTROL_OUT_MUX_ADDRESS_MIN = 0;
    static const uint8_t MAIN_CONTROL_OUT_MUX_ADDRESS_MAX = 0x39;

    /* Check device pointer and gain pointer are not null */
    ADI_NULL_DEVICE_PTR_RETURN(device);
    ADI_NULL_PTR_RETURN(&device->common, controlOutMuxCfg);

    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_RANGE_CHECK(device, controlOutMuxCfg->controlOutMuxSel, ADI_ADRV9001_CONTROL_OUT_MUX_RX, ADI_ADRV9001_CONTROL_OUT_MUX_MAIN);

    if (controlOutMuxCfg->controlOutMuxSel == ADI_ADRV9001_CONTROL_OUT_MUX_RX)
    {
        if (port != ADI_RX)
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_INV_PARAM,
                             ADI_COMMON_ACT_ERR_CHECK_PARAM,
                             port,
                             "Selected port type is incompatible with 'Control Out' Mux selected");
        }
        else
        {
            ADI_RANGE_CHECK(device, controlOutMuxCfg->controlOutMuxAddress, RX_CONTROL_OUT_MUX_ADDRESS_MIN, RX_CONTROL_OUT_MUX_ADDRESS_MAX);
        }
    }
    else if (controlOutMuxCfg->controlOutMuxSel == ADI_ADRV9001_CONTROL_OUT_MUX_TX)
    {
        if (port != ADI_TX)
        {
            ADI_ERROR_REPORT(&device->common,
                             ADI_COMMON_ERRSRC_API,
                             ADI_COMMON_ERR_INV_PARAM,
                             ADI_COMMON_ACT_ERR_CHECK_PARAM,
                             port,
                             "Selected port type is incompatible with 'Control Out' Mux selected");
        }
        else
        {
            ADI_RANGE_CHECK(device, controlOutMuxCfg->controlOutMuxAddress, TX_CONTROL_OUT_MUX_ADDRESS_MIN, TX_CONTROL_OUT_MUX_ADDRESS_MAX);
        }
    }
    else
    {
        ADI_RANGE_CHECK(device, controlOutMuxCfg->controlOutMuxAddress, MAIN_CONTROL_OUT_MUX_ADDRESS_MIN, MAIN_CONTROL_OUT_MUX_ADDRESS_MAX);
    }

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_ControlOutMux_Configure(adi_adrv9001_Device_t *device,
                                                                adi_common_Port_e port,
                                                                adi_common_ChannelNumber_e channel,
                                                                adi_adrv9001_ControlOutMuxCfg_t *controlOutMuxCfg)
{
    static const uint8_t RX1_CHAN_CONTROL_OUT_SEL = 0x01;
    static const uint8_t RX1_MAIN_CONTROL_OUT_SEL = 0x02;
    static const uint8_t RX2_CHAN_CONTROL_OUT_SEL = 0x04;
    static const uint8_t RX2_MAIN_CONTROL_OUT_SEL = 0x08;
    static const uint8_t TX1_CHAN_CONTROL_OUT_SEL = 0x10;
    static const uint8_t TX1_MAIN_CONTROL_OUT_SEL = 0x20;
    static const uint8_t TX2_CHAN_CONTROL_OUT_SEL = 0x40;
    static const uint8_t TX2_MAIN_CONTROL_OUT_SEL = 0x80;

    static const uint32_t SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR = 0x0344;

    ADI_PERFORM_VALIDATION(adi_adrv9001_internal_utilities_ControlOutMux_Configure_Validate, device, port, channel, controlOutMuxCfg);
    
    if ((port == ADI_RX) && (channel == ADI_CHANNEL_1))
    {
        if (controlOutMuxCfg->controlOutMuxSel == ADI_ADRV9001_CONTROL_OUT_MUX_RX)
        {
            /* Select Rx1 Control Out Mux configuration */
            ADI_EXPECT(adrv9001_NvsRegmapRxControlOutMuxSelBfSet, device, ADRV9001_BF_RX1_CORE, controlOutMuxCfg->controlOutMuxAddress);

            /* Route RX1 control out mux to RX2 SSI GPIO */
            ADRV9001_SPIWRITEBYTE(device, "RX2_CHAN_CONTROL_OUT_SEL", SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR, RX2_CHAN_CONTROL_OUT_SEL);
        }
        else /* ADI_ADRV9001_CONTROL_OUT_MUX_MAIN */
        {
            /* Select Main Control Out Mux configuration */
            ADI_EXPECT(adrv9001_NvsRegmapCore1ControlOutMuxSelBfSet, device, ADRV9001_BF_CORE_1, controlOutMuxCfg->controlOutMuxAddress);

            /* Route Main control out mux to RX2 SSI GPIO */
            ADRV9001_SPIWRITEBYTE(device, "RX2_MAIN_CONTROL_OUT_SEL", SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR, RX2_MAIN_CONTROL_OUT_SEL);
        }

        ADI_EXPECT(adi_adrv9001_Rx1_ControlOutMux_SsiGpio_Configure, device, controlOutMuxCfg);
    }
    else if ((port == ADI_RX) && (channel == ADI_CHANNEL_2))
    {
        if (controlOutMuxCfg->controlOutMuxSel == ADI_ADRV9001_CONTROL_OUT_MUX_RX)
        {
            /* Select Rx2 Control Out Mux configuration */
            ADI_EXPECT(adrv9001_NvsRegmapRxControlOutMuxSelBfSet, device, ADRV9001_BF_RX2_CORE, controlOutMuxCfg->controlOutMuxAddress);

            /* Route RX2 control out mux to RX1 SSI GPIO */
            ADRV9001_SPIWRITEBYTE(device, "RX1_CHAN_CONTROL_OUT_SEL", SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR, RX1_CHAN_CONTROL_OUT_SEL);
        }
        else /* ADI_ADRV9001_CONTROL_OUT_MUX_MAIN */
        {
            /* Select Main Control Out Mux configuration */
            ADI_EXPECT(adrv9001_NvsRegmapCore1ControlOutMuxSelBfSet, device, ADRV9001_BF_CORE_1, controlOutMuxCfg->controlOutMuxAddress);

            /* Route Main control out mux to RX1 SSI GPIO */
            ADRV9001_SPIWRITEBYTE(device, "RX1_MAIN_CONTROL_OUT_SEL", SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR, RX1_MAIN_CONTROL_OUT_SEL);
        }

        ADI_EXPECT(adi_adrv9001_Rx2_ControlOutMux_SsiGpio_Configure,device, controlOutMuxCfg);
    }
    else if ((port == ADI_TX) && (channel == ADI_CHANNEL_1))
    {
        if (controlOutMuxCfg->controlOutMuxSel == ADI_ADRV9001_CONTROL_OUT_MUX_TX)
        {
            /* Select Tx1 Control Out Mux configuration */
            ADI_EXPECT(adrv9001_NvsRegmapTxControlOutMuxSelBfSet, device, ADRV9001_BF_TX1_CORE, controlOutMuxCfg->controlOutMuxAddress);

            /* Route TX1 control out mux to TX2 SSI GPIO */
            ADRV9001_SPIWRITEBYTE(device, "TX2_CHAN_CONTROL_OUT_SEL", SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR, TX2_CHAN_CONTROL_OUT_SEL);
        }
        else /* ADI_ADRV9001_CONTROL_OUT_MUX_MAIN */
        {
            /* Select Main Control Out Mux configuration */
            ADI_EXPECT(adrv9001_NvsRegmapCore1ControlOutMuxSelBfSet, device, ADRV9001_BF_CORE_1, controlOutMuxCfg->controlOutMuxAddress);

            /* Route Main control out mux to TX2 SSI GPIO */
            ADRV9001_SPIWRITEBYTE(device, "TX2_MAIN_CONTROL_OUT_SEL", SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR, TX2_MAIN_CONTROL_OUT_SEL);
        }

        ADI_EXPECT(adi_adrv9001_Tx1_ControlOutMux_SsiGpio_Configure, device, controlOutMuxCfg);
    }
    else /* This should be "port == ADI_TX" and "channel == ADI_CHANNEL_2" */
    {
        if (controlOutMuxCfg->controlOutMuxSel == ADI_ADRV9001_CONTROL_OUT_MUX_TX)
        {
            /* Select Tx2 Control Out Mux configuration */
            ADI_EXPECT(adrv9001_NvsRegmapTxControlOutMuxSelBfSet, device, ADRV9001_BF_TX2_CORE, controlOutMuxCfg->controlOutMuxAddress);

            /* Route TX2 control out mux to TX1 SSI GPIO */
            ADRV9001_SPIWRITEBYTE(device, "TX1_CHAN_CONTROL_OUT_SEL", SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR, TX1_CHAN_CONTROL_OUT_SEL);
        }
        else /* ADI_ADRV9001_CONTROL_OUT_MUX_MAIN */
        {
            /* Select Main Control Out Mux configuration */
            ADI_EXPECT(adrv9001_NvsRegmapCore1ControlOutMuxSelBfSet, device, ADRV9001_BF_CORE_1, controlOutMuxCfg->controlOutMuxAddress);

            /* Route Main control out mux to TX1 SSI GPIO */
            ADRV9001_SPIWRITEBYTE(device, "TX1_MAIN_CONTROL_OUT_SEL", SSI_CMOS_GPIO_CONTROL_OUT_SEL_ADDR, TX1_MAIN_CONTROL_OUT_SEL);
        }

        ADI_EXPECT(adi_adrv9001_Tx2_ControlOutMux_SsiGpio_Configure, device, controlOutMuxCfg);
    }

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Rx_Ssi_Enable_Validate(adi_adrv9001_Device_t *device,
                                                                                     adi_common_ChannelNumber_e channel)
{
    ADI_NULL_DEVICE_PTR_RETURN(device);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Rx_Ssi_Enable(adi_adrv9001_Device_t *device,
                                                                     adi_common_ChannelNumber_e channel,
                                                                     bool enableIlb,
                                                                     bool enableElb)
{
    uint8_t payload[4] = { 0 };
    uint8_t ssiEnableMask = 0;
    static const uint8_t INTERNAL_DEBUG_RX_SSI_ENABLE = 0;
    static uint8_t ILB_SSI_ENABLE = 0x1;
    static uint8_t ELB_SSI_ENABLE = 0x2;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    ADI_PERFORM_VALIDATION(adi_adrv9001_internal_utilities_Firmware_Debug_Rx_Ssi_Enable_Validate, device, channel);

    payload[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel);
    payload[1] = ADRV9001_ARM_OBJECTID_INTERNAL_DEBUG;
    payload[2] = INTERNAL_DEBUG_RX_SSI_ENABLE;

    if (true == enableIlb)
    {
        ssiEnableMask |= ILB_SSI_ENABLE;
    }
    if (true == enableElb)
    {
        ssiEnableMask |= ELB_SSI_ENABLE;
    }

    payload[3] = ssiEnableMask;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
        device, 
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        &payload[0], 
        sizeof(payload));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        payload[1], 
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_INTERVAL_US);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Rx_Datapath_Configure_Validate(adi_adrv9001_Device_t *device,
                                                                                             adi_common_Port_e rxDataPathSel,
                                                                                             adi_common_ChannelNumber_e channel,
                                                                                             adi_adrv9001_LoType_e loType)
{
    ADI_NULL_DEVICE_PTR_RETURN(device);

    if (rxDataPathSel == ADI_TX)
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            port,
            "Selected port type is not supported for Rx datapath firmware debug");
    }

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_RANGE_CHECK(device, loType, ADI_ADRV9001_LO_OFFSET_FREQUENCY_ZERO, ADI_ADRV9001_LO_OFFSET_FREQUENCY_NON_ZERO);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Rx_Datapath_Configure(adi_adrv9001_Device_t *device,
                                                                             adi_common_Port_e rxDataPathSel,
                                                                             adi_common_ChannelNumber_e channel,
                                                                             adi_adrv9001_LoType_e loType)
{
    uint8_t payload[5] = { 0 };
    static const uint8_t INTERNAL_DEBUG_RX_DATAPATH_CONFIG = 0x01;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    ADI_PERFORM_VALIDATION(adi_adrv9001_internal_utilities_Firmware_Debug_Rx_Datapath_Configure_Validate, device, rxDataPathSel, channel, loType);

    payload[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel); 
    payload[1] = ADRV9001_ARM_OBJECTID_INTERNAL_DEBUG;
    payload[2] = INTERNAL_DEBUG_RX_DATAPATH_CONFIG;

    if (rxDataPathSel == ADI_RX)
    {
        payload[3] = (uint8_t)rxDataPathSel;
    }
    else
    {
        /* This payload byte accepts only RX ports with 0:RX, 1:ORX, 2:ILB, 3:ELB */
        payload[3] = ((uint8_t)rxDataPathSel - 1);
        /* Pass LO type as payload byte 5 if  rxDataPathSel = ORx/ELB/ILB */
        payload[4] = (uint8_t)loType;
    }

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
        device, 
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        &payload[0], 
        sizeof(payload));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        payload[1], 
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_INTERVAL_US);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Ilb_Calibration_Tone_Configure_Validate(adi_adrv9001_Device_t *device,
                                                                                                      adi_adrv9001_IlbCalToneStatusSel_e ilbCalToneStatus,
                                                                                                      adi_adrv9001_PllAtten_e pllAtten,
                                                                                                      adi_adrv9001_CalToneSwitchAtten_e calToneSwitchAtten,
                                                                                                      int64_t calibrationToneFrequency_Hz,
                                                                                                      adi_common_ChannelNumber_e channel)
{
    static const int64_t CALIBRATION_TONE_FREQUENCY_MIN_HZ = -92160000; /* -92.16 MHz */
    static const int64_t CALIBRATION_TONE_FREQUENCY_MAX_HZ =  92160000; /* 92.16 MHzz */

    ADI_NULL_DEVICE_PTR_RETURN(device);
    ADI_RANGE_CHECK(device, ilbCalToneStatus, ADI_ADRV9001_ILB_CAL_TONE_DISABLE, ADI_ADRV9001_ILB_CAL_TONE_UPDATE);
    ADI_RANGE_CHECK(device, pllAtten, ADI_ADRV9001_PLL_ATTEN_ZERO_DB, ADI_ADRV9001_PLL_ATTEN_10P25_DB);
    ADI_RANGE_CHECK(device, calToneSwitchAtten, ADI_ADRV9001_CAL_TONE_SWITCH_ATTEN_ZERO_DB, ADI_ADRV9001_CAL_TONE_SWITCH_ATTEN_18_DB);

    if (calibrationToneFrequency_Hz < CALIBRATION_TONE_FREQUENCY_MIN_HZ || calibrationToneFrequency_Hz > CALIBRATION_TONE_FREQUENCY_MAX_HZ)
    {
        snprintf(device->common.error.errormessage,
            sizeof(device->common.error.errormessage),
            "Invalid parameter value. %s was %llu, but must be between %llu and %llu, inclusive.",
            "calibrationToneFrequency_Hz",
            calibrationToneFrequency_Hz,
            CALIBRATION_TONE_FREQUENCY_MIN_HZ,
            CALIBRATION_TONE_FREQUENCY_MAX_HZ);

        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_INV_PARAM,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            calibrationToneFrequency_Hz,
            device->common.error.errormessage);
    }

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Ilb_Calibration_Tone_Configure(adi_adrv9001_Device_t *device,
                                                                                      adi_adrv9001_IlbCalToneStatusSel_e ilbCalToneStatus,
                                                                                      adi_adrv9001_PllAtten_e pllAtten,
                                                                                      adi_adrv9001_CalToneSwitchAtten_e calToneSwitchAtten,
                                                                                      int64_t calibrationToneFrequency_Hz,
                                                                                      adi_common_ChannelNumber_e channel,
                                                                                      bool enableLo)
{
    uint8_t armData[10] = { 0 };
    uint8_t payload[4] = { 0 };
    static const uint8_t INTERNAL_DEBUG_ILB_CAL_TONE_SET = 0x02;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    ADI_PERFORM_VALIDATION(adi_adrv9001_internal_utilities_Firmware_Debug_Ilb_Calibration_Tone_Configure_Validate, 
                           device,
                           ilbCalToneStatus,
                           pllAtten,
                           calToneSwitchAtten,
                           calibrationToneFrequency_Hz,
                           channel);

    armData[0] = (uint8_t)((calibrationToneFrequency_Hz >> 0) & 0xFF);
    armData[1] = (uint8_t)((calibrationToneFrequency_Hz >> 8) & 0xFF);
    armData[2] = (uint8_t)((calibrationToneFrequency_Hz >> 16) & 0xFF);
    armData[3] = (uint8_t)((calibrationToneFrequency_Hz >> 24) & 0xFF);
    armData[4] = (uint8_t)((calibrationToneFrequency_Hz >> 32) & 0xFF);
    armData[5] = (uint8_t)((calibrationToneFrequency_Hz >> 40) & 0xFF);
    armData[6] = (uint8_t)((calibrationToneFrequency_Hz >> 48) & 0xFF);
    armData[7] = (uint8_t)((calibrationToneFrequency_Hz >> 56) & 0xFF);
    armData[8] = (uint8_t)pllAtten;
    armData[9] = (uint8_t)calToneSwitchAtten;

    /* Write carrier Frequency to ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, device, (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_SET, &armData[0], sizeof(armData));

    payload[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel);
    payload[1] = ADRV9001_ARM_OBJECTID_INTERNAL_DEBUG;
    payload[2] = INTERNAL_DEBUG_ILB_CAL_TONE_SET;
    payload[3] = (uint8_t)ilbCalToneStatus;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
        device, 
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        &payload[0], 
        sizeof(payload));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        payload[1], 
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_INTERVAL_US);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Generic_Configure_Validate(adi_adrv9001_Device_t *device,
                                                                                         adi_common_ChannelNumber_e channel,
                                                                                         adi_common_Port_e port)
{
    ADI_NULL_DEVICE_PTR_RETURN(device);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_Firmware_Debug_Generic_Configure(adi_adrv9001_Device_t *device,
                                                                         adi_common_ChannelNumber_e channel,
                                                                         adi_common_Port_e port)
{
    uint8_t payload[5] = { 0 };
    static const uint8_t INTERNAL_DEBUG_GENERIC = 0xFF;

    /* Check device pointer is not null */
    ADI_API_ENTRY_EXPECT(device);

    ADI_PERFORM_VALIDATION(adi_adrv9001_internal_utilities_Firmware_Debug_Generic_Configure_Validate, device, channel, port);

    /* TODO: This API is just a place holder and probably for a basic FW testing.
             Other payload bytes and SET buffer values are not defined by FW team yet.
             This API may be updated in future based on FW requirement */

    payload[0] = adi_adrv9001_Radio_MailboxChannel_Get(port, channel);
    payload[1] = ADRV9001_ARM_OBJECTID_INTERNAL_DEBUG;
    payload[2] = INTERNAL_DEBUG_GENERIC;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write,
        device, 
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        &payload[0], 
        sizeof(payload));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        (uint8_t)ADRV9001_ARM_SET_OPCODE,
        payload[1], 
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_INTERVAL_US);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_internal_utilities_Cal_Status_Get_Validate(adi_adrv9001_Device_t *device, 
                                                                       adi_common_ChannelNumber_e channel,
                                                                       adi_common_Port_e port,
                                                                       adi_adrv9001_CalAlgoObjId_e calAlgoObjId,
                                                                       uint8_t returnData[],
                                                                       uint32_t byteCount)
{
    ADI_NULL_PTR_RETURN(&device->common, returnData);
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_RANGE_CHECK(device, port, ADI_RX, ADI_TX);
    /* TODO: Range check of calAlgoObjId to be corrected once obj id for tracking cal algorithms are added */
    ADI_RANGE_CHECK(device, calAlgoObjId, ADI_ADRV9001_OBJID_IC_TX_QEC, ADI_ADRV9001_OBJID_IC_AUXPLL);

    /* The maximum bytes of the data to read from ARM memory is 256, which is the size of ARM mailbox GET buffer size */
    ADI_RANGE_CHECK(device, byteCount, 1, 256);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_internal_utilities_Cal_Status_Get(adi_adrv9001_Device_t *device, 
                                                       adi_common_ChannelNumber_e channel,
                                                       adi_common_Port_e port,
                                                       adi_adrv9001_CalAlgoObjId_e calAlgoObjId,
                                                       uint8_t returnData[],
                                                       uint32_t byteCount,
                                                       uint16_t offset)
{
    uint8_t armData[1] = { 0 };
    uint8_t armExtData[5] = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_internal_utilities_Cal_Status_Get_Validate, device, channel, port, calAlgoObjId, returnData, byteCount);

    armExtData[0] = adi_adrv9001_Radio_MailboxChannel_Get(port, channel);
    armExtData[1] = ADRV9001_ARM_OBJECTID_INTERNAL_CAL_STATUS;
    armExtData[2] = (uint8_t)calAlgoObjId;
    armExtData[3] = (uint8_t)((offset >> 0) & 0xFF); 
    armExtData[4] = (uint8_t)((offset >> 8) & 0xFF);

    /* Write byteCount to ARM mailbox GET buffer */
    armData[0] = byteCount;
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, device, (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_GET, &armData[0], 1);

    /* send ARM GET opcode */
    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_GET_OPCODE, armExtData, sizeof(armExtData));

    /* wait for command completion */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        ADRV9001_ARM_GET_OPCODE,
        armExtData[1],
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_TIMEOUT_US,
        (uint32_t)ADI_ADRV9001_INTERNAL_DEBUG_INTERVAL_US);

    /* Read PLL Frequency from ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read,
        device,
        ADRV9001_ADDR_ARM_MAILBOX_GET,
        returnData,
        byteCount,
        ADRV9001_ARM_MEM_READ_AUTOINCR);

    ADI_API_RETURN(device);
}
