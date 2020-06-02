/**
* \file
* \brief Contains ADI Transceiver Hardware Abstraction functions for ZC706 interface to FPGA (BBIC) control registers.
*        Analog Devices maintains and provides updates to this code layer.
*        The end user should not modify this file or any code in this directory.
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef __ZC706SD20_BBIC_CONTROL_H__
#define __ZC706SD20_BBIC_CONTROL_H__

#include <stdint.h>
#include "adi_common_error.h"
#include "adi_platform_types.h"

#ifdef __cplusplus
extern "C" {
#endif
#define ZC706SD20_FPGA_UIO0_START_ADDRESS 0x43000000
#define ZC706SD20_FPGA_UIO0_STOP_ADDRESS  0x4340E9FF
// 1GB memory
#define ZC706SD20_FPGA_UIO1_START_ADDRESS 0x80000000
#define ZC706SD20_FPGA_UIO1_STOP_ADDRESS  0xBFFFFFFF
#define ZC706SD20_FPGA_UIO2_START_ADDRESS 0x44A00000
#define ZC706SD20_FPGA_UIO2_STOP_ADDRESS  0x44A000FF

#ifndef CLIENT_IGNORE
/**
* \brief Function to open any necessary resources for the BBIC interface functions in this file.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
* \retval ADI_HAL_NULL_PTR If devHalCfg pointer is NULL
* \retval ADI_HAL_BBICCTRL_FAIL If critical error trying to open resources
*/
int32_t zc706sd20_BbicOpen(void *devHalCfg);

/**
* \brief Function to close any necessary resources for the BBIC interface functions in this file.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
* \retval ADI_HAL_NULL_PTR If devHalCfg pointer is NULL
* \retval ADI_HAL_BBICCTRL_FAIL If critical error trying to close resources
*/
int32_t zc706sd20_BbicClose(void *devHalCfg);

/**
* \brief Function to initialize any necessary resources for the BBIC interface functions in this file.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_OK Function completed successfully
*/
int32_t zc706sd20_BbicInit(void *devHalCfg);

/**
 * \brief Function to read a single BBIC control register
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param addr 32bit address of BBIC register to read
 * \param data 32bit Pointer to store return value representing the data of the register at the specified address
 *
 * \retval ADI_HAL_OK Function completed successfully
 * \retval ADI_HAL_NULL_PTR devHalCfg pointer is NULL or data pointer is NULL
 */
int32_t zc706sd20_BbicRegisterRead(void *devHalCfg, uint32_t addr, uint32_t *data);

/**
 * \brief Function to write a single BBIC control register
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param addr 32bit address of BBIC register to write
 * \param data 32bit data to write to the register at the specified address
 *
 * \retval ADI_HAL_OK Function completed successfully
 * \retval ADI_HAL_NULL_PTR devHalCfg pointer is NULL
 */
int32_t zc706sd20_BbicRegisterWrite(void *devHalCfg, uint32_t addr, uint32_t data);

/**
 * \brief Function to read multiple consecutive BBIC control registers starting at a specified register address.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param addr 32bit address of BBIC register to start reading from
 * \param data 32bit Pointer to store return array representing the data starting at the specified register address
               and ending at (addr + numDataWords -1)
 * \param numDataWords Number of elements in the data array
 *
 * \retval ADI_HAL_OK Function completed successfully
 * \retval ADI_HAL_NULL_PTR devHalCfg pointer is NULL
 */
int32_t zc706sd20_BbicRegistersRead(void *devHalCfg, uint32_t addr, uint32_t data[], uint32_t numDataWords);

/**
 * \brief Function to write multiple consecutive BBIC control registers starting at a specified register address.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param addr 32bit address of BBIC register to start writing to
 * \param data 32bit Pointer to array representing the data to write starting at the specified register address
               and ending at (addr + numDataWords -1)
 * \param numDataWords Number of elements in the data array
 *
 * \retval ADI_HAL_OK Function completed successfully
 * \retval ADI_HAL_NULL_PTR devHalCfg pointer is NULL
 */
int32_t zc706sd20_BbicRegistersWrite(void *devHalCfg, uint32_t addr, uint32_t data[], uint32_t numDataWords);
#endif /* CLIENT_IGNORE */

#ifdef __cplusplus
}
#endif
#endif /*__ZC706SD20_BBIC_CONTROL_H__*/
