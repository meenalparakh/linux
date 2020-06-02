/**
* \file
* \brief Contains internal use only BBDC engineering functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_internal_bbdc.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_radio.h"
#include "adrv9001_arm.h"
#include "adrv9001_arm_macros.h"
#include "adi_adrv9001_internal_utilities_types.h"

static int32_t adi_adrv9001_internal_bbdc_TestDisable_Set_Validate(adi_adrv9001_Device_t *device, 
                                                                   adi_common_ChannelNumber_e channel)
{
    uint8_t port_index = 0;
    uint8_t chan_index = 0;
    adi_adrv9001_RadioState_t currentState = { 0 };

    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_EXPECT(adi_adrv9001_Radio_State_Get, device, &currentState);

    adi_common_port_to_index(ADI_RX, &port_index);
    adi_common_channel_to_index(channel, &chan_index);
    if (ADI_ADRV9001_CHANNEL_STANDBY == currentState.channelStates[port_index][chan_index])
    {
        ADI_ERROR_REPORT(&device->common,
            ADI_COMMON_ERRSRC_API,
            ADI_COMMON_ERR_API_FAIL,
            ADI_COMMON_ACT_ERR_CHECK_PARAM,
            currentState.channelStates[port_index][chan_index],
            "Error while attempting to set BBDC test disable. Specified channel is in wrong state.");
        ADI_API_RETURN(device)
    }

    ADI_API_RETURN(device);
}


int32_t adi_adrv9001_internal_bbdc_TestDisable_Set(adi_adrv9001_Device_t *device, 
                                                   adi_common_ChannelNumber_e channel,
                                                   bool disable)
{
    uint8_t armData = 0;
    uint8_t armExtData[2] = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_internal_bbdc_TestDisable_Set_Validate, device, channel);
    
    /* Mailbox expects a disable parameter */
    armData = (disable) ? 1 : 0;
    
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, 
               device, 
               ADRV9001_ADDR_ARM_MAILBOX_SET, 
               &armData,
               sizeof(armData));
    
    armExtData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel);
    armExtData[1] = ADRV9001_ARM_OBJECTID_INTERNAL_BBDC_TEST_DISABLE;

    /* send ARM SET opcode */
    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_SET_OPCODE, armExtData, sizeof(armExtData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        ADRV9001_ARM_SET_OPCODE,
        armExtData[1],
        ADI_ADRV9001_SETBBDC_DISABLE_TIMEOUT_US,
        ADI_ADRV9001_SETBBDC_DISABLE_INTERVAL_US);

    ADI_API_RETURN(device);
}

static int32_t adi_adrv9001_internal_bbdc_TestDisable_Get_Validate(adi_adrv9001_Device_t *device, 
                                                                   adi_common_ChannelNumber_e channel,
                                                                   bool *disable)
{
    ADI_RANGE_CHECK(device, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&device->common, disable);
    ADI_API_RETURN(device);
}


int32_t adi_adrv9001_internal_bbdc_TestDisable_Get(adi_adrv9001_Device_t *device, 
                                                   adi_common_ChannelNumber_e channel,
                                                   bool *disable)
{
    uint8_t armData[2] = { 0 };
    uint8_t armExtData[2] = { 0 };

    ADI_PERFORM_VALIDATION(adi_adrv9001_internal_bbdc_TestDisable_Get_Validate, device, channel, disable);

    armExtData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_RX, channel);
    armExtData[1] = ADRV9001_ARM_OBJECTID_INTERNAL_BBDC_TEST_DISABLE;

    /* send ARM GET opcode */
    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, device, (uint8_t)ADRV9001_ARM_GET_OPCODE, armExtData, sizeof(armExtData));

    /* wait for command completion */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(device,
        ADRV9001_ARM_GET_OPCODE,
        armExtData[1],
        ADI_ADRV9001_SETBBDC_DISABLE_TIMEOUT_US,
        ADI_ADRV9001_SETBBDC_DISABLE_INTERVAL_US);

    /* Read PLL Frequency from ARM mailbox */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read,
                   device,
                   ADRV9001_ADDR_ARM_MAILBOX_GET,
                   armData,
                   sizeof(armData),
                   ADRV9001_ARM_MEM_READ_AUTOINCR);

    /* Mailbox expects a disable parameter */
    *disable = (armData[0] != 0);

    ADI_API_RETURN(device);
}
