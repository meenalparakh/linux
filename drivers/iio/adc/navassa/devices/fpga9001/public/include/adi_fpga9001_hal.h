/**
 * \file
 * \brief Contains ADI Transceiver Hardware Abstraction functions
 *        Analog Devices maintains and provides updates to this code layer.
 *        The end user should not modify this file or any code in this directory.
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */
#ifndef _ADI_FPGA9001_HAL_H_
#define _ADI_FPGA9001_HAL_H_

#include "adi_fpga9001_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Performs a read of a single 32-bit register
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  addr             Address of the register to be read. Must be on a word boundary
 * \param[out] data             The value read from the register
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_fpga9001_hal_Register_Read(adi_fpga9001_Device_t *fpga9001, uint32_t addr, uint32_t *data);

/**
 * \brief Performs a write of a single 32-bit register
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] addr              Address of the register to be written. Must be on a word boundary
 * \param[in] data              The value to write to the register
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_hal_Register_Write(adi_fpga9001_Device_t *fpga9001, uint32_t addr, uint32_t data);

/**
 * \brief Performs bit field read
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  addr             Address of the register where the field resides
 * \param[out] fieldVal         A pointer to store the value in the field
 * \param[in]  mask             A bit-wise mask that determines which bits are part of the field
 * \param[in]  startBit         The bit location in the addr where the field starts
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_hal_Field_Read(adi_fpga9001_Device_t *fpga9001,
                                    const uint32_t addr,
                                    uint32_t *fieldVal,
                                    uint32_t mask,
                                    uint8_t startBit);

/**
 * \brief Performs bit field write
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] addr              Address of the register where the field resides
 * \param[in] fieldVal          The value to write to the field
 * \param[in] mask              A bit-wise mask that determines which bits are part of the field
 * \param[in] startBit          The bit location in the addr where the field starts
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_hal_Field_Write(adi_fpga9001_Device_t *fpga9001,
                                     const uint32_t addr,
                                     uint32_t fieldVal,
                                     uint32_t mask,
                                     uint8_t startBit);

/**
 * \brief Performs a RAM memory read
 *
 * \param[in]  fpga9001         Context variable - Pointer to FPGA9001 device data structure
 * \param[in]  ramAddress       Address to begin the read from
 * \param[out] data             The array of values read from RAM. Must have enough space to store 'length' elements
 * \param[in]  length           The number of words to read starting from ramAddress
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_hal_Ram_Read(adi_fpga9001_Device_t *fpga9001, 
                                  const uint32_t ramAddress,
                                  uint32_t data[],
                                  uint32_t length);

/**
 * \brief Performs a RAM memory write
 *
 * \param[in] fpga9001          Context variable - Pointer to FPGA9001 device data structure
 * \param[in] ramAddress        Address to begin writing to
 * \param[in] data              Array of values to write to RAM
 * \param[in] length            The number of words to write starting at ramAddress
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_hal_Ram_Write(adi_fpga9001_Device_t *fpga9001, 
                                   const uint32_t ramAddress,
                                   uint32_t data[],
                                   uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_HAL_H_ */
