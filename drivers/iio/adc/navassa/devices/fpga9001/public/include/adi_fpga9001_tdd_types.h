/*!
* \file
* \brief TDD types
*
* FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
*/

/**
 * Copyright 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_TDD_TYPES_H_
#define _ADI_FPGA9001_TDD_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
   
#include "adi_adrv9001_types.h"

/**
* The TDD timing, sequence, and framing parameters are illustrated in the following timing diagram.
* 
* \image html "FPGA TDD timing diagram.png" "FPGA TDD timing diagram"
*/
    
/**
* \brief Timing of enable assert/deassert within a TDD frame
* 
* The TDD counter starts at 1, so primaryAssert must be greater than 0. 
* The secondary enable event is optional - set secondaryAssert = secondaryDeassert = 0 to ignore.
*/
typedef struct adi_fpga9001_TddTiming
{
    uint32_t primaryAssert;         /*!< Number of tdd_clk cycles after start of TDD frame to perform primary assert */
    uint32_t primaryDeassert;       /*!< Number of tdd_clk cycles after start of TDD frame to perform primary deassert */
    uint32_t secondaryAssert;       /*!< Number of tdd_clk cycles after start of TDD frame to perform secondary assert */
    uint32_t secondaryDeassert;     /*!< Number of tdd_clk cycles after start of TDD frame to perform secondary deassert */
} adi_fpga9001_TddTiming_t;
    
/**
* \brief Sequencing of TDD frame enabledness
* 
* Constraints:
* \li All assert/deasserts must be less than or equal to numberFrames.
* \li At least 1 frame must be active
* \li Deassert must not come before assert
* \li Secondary event is optional - set secondaryFrameAssert = secondaryFrameDeassert = 0 to ignore
* \li Secondary assert must come after primary deassert
* 
* \note If numberFrames is 0 these parameters have no effect.
*/
typedef struct adi_fpga9001_TddSequence
{
    uint32_t primaryFrameAssert;       /*!< First frame in primary sequence during which pin or tollgate assert/deassert activity is enabled */
    uint32_t primaryFrameDeassert;     /*!< Final frame in primary sequence during which pin or tollgate assert/deassert activity is enabled */
    uint32_t secondaryFrameAssert;     /*!< First frame in secondary sequence during which pin or tollgate assert/deassert activity is enabled */
    uint32_t secondaryFrameDeassert;   /*!< Final frame in secondary sequence during which pin or tollgate assert/deassert activity is enabled */
} adi_fpga9001_TddSequence_t;

typedef struct adi_fpga9001_TddChannel
{
    bool enable;                                    /*!< Whether or not TDD is enabled for the channel */
    adi_fpga9001_TddTiming_t pinTiming;             /*!< Controls the timing of the GPIO enable pin transitions */
    adi_fpga9001_TddSequence_t pinSequence;         /*!< Controls the sequencing of the GPIO enable pin */
    adi_fpga9001_TddTiming_t tollgateTiming;        /*!< Controls the timing of the DMA enabledness; Does not apply to txGpioControls */
    adi_fpga9001_TddSequence_t tollgateSequence;    /*!< Controls the sequencing of the DMA enabledness; Does not apply to txGpioControls */
} adi_fpga9001_TddChannel_t;

typedef struct adi_fpga9001_TddFraming
{
    bool enableSyncExtTrig;         /*!< Set this value to 1 to enable external triggering. Otherwise, set to 0 */
    bool repeatFrameSequence;       /*!< Set this value to 1 to repeat the TDD frame sequence infinitely */
    uint8_t frameSwitchTimeChannel; /*!< Channel which controls the frameSwitchTime: 0 = rxControls[0]/txControls[0], 1 = rxControls[1]/txControls[1] */
    uint32_t frameSwitchTime;       /*!< When frameSwitchTime = 0 or frameSwitchTime = framePeriod, this variable has no effect.
                                         When 0 < frameSwitchTime < framePeriod frames will terminate at time = frameSwitchTime TDD_CLK cycles during the frame where:
                                         Frame# = rxControls[frameSwitchTimeChannel]. pinSequence.primaryFrameDeassert
                                         Frame# = rxControls[frameSwitchTimeChannel]. pinSequence.secondaryFrameDeassert
                                         Frame# = txControls[frameSwitchTimeChannel]. pinSequence.primaryFrameDeassert
                                         Frame# = txControls[frameSwitchTimeChannel]. pinSequence.secondaryFrameDeassert */
    uint32_t framePeriod;           /*!< Duration of TDD frame in tdd_clk cycles */
    uint32_t numberFrames;          /*!< Desired number of TDD frames in sequence; set to 0 for infinite */
} adi_fpga9001_TddFraming_t;

typedef struct adi_fpga9001_TddConfig
{
    adi_fpga9001_TddFraming_t framing;

    adi_fpga9001_TddChannel_t rxControls[ADI_ADRV9001_NUM_CHANNELS];
    adi_fpga9001_TddChannel_t txControls[ADI_ADRV9001_NUM_CHANNELS];
    adi_fpga9001_TddChannel_t orxControls[ADI_ADRV9001_NUM_CHANNELS];
    
    adi_fpga9001_TddChannel_t txGpioControls[ADI_ADRV9001_NUM_CHANNELS];
    adi_fpga9001_TddChannel_t interFrameControlSignal1[ADI_ADRV9001_NUM_CHANNELS];
    adi_fpga9001_TddChannel_t interFrameControlSignal2[ADI_ADRV9001_NUM_CHANNELS];
} adi_fpga9001_TddConfig_t;

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_FPGA9001_TDD_TYPES_H_ */
