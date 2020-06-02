/**
 * \file fpga9001_bf_hal.h 
 *
 * \brief Contains BitField functions to support FPGA9001 transciever device.
 *
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */
#ifndef FPGA9001_BF_HAL_H_
#define FPGA9001_BF_HAL_H_

#include "adi_common_hal.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BF_LOG_LEVEL 0xF

#define ADRV9001_BF_WRCACHE_ARRAY_SIZE 15
#define ADRV9001_BF_WRCACHE_ARRAY_TRIP_SIZE ADRV9001_BF_WRCACHE_ARRAY_SIZE -2

#define ADRV9001_BF_RDCACHE_ARRAY_SIZE 15
#define ADRV9001_BF_RDCACHE_ARRAY_TRIP_SIZE ADRV9001_BF_RDCACHE_ARRAY_SIZE -2

int32_t fpga9001_BfReadCacheInit(adi_fpga9001_Device_t *device);
int32_t fpga9001_BfWriteCacheInit(adi_fpga9001_Device_t *device);

int32_t fpga9001_BfRegisterWrite(adi_fpga9001_Device_t *device, uint32_t addr, uint32_t regVal);
int32_t fpga9001_BfRegisterRead(adi_fpga9001_Device_t *device, uint32_t addr, uint32_t *regVal, uint8_t endOfBf);

int32_t fpga9001_BfFieldWrite(adi_fpga9001_Device_t *device, uint32_t addr, uint32_t  fieldVal, uint32_t mask, uint8_t startBit);
int32_t fpga9001_BfFieldRead(adi_fpga9001_Device_t *device, uint32_t addr, uint32_t *fieldVal, uint32_t mask, uint8_t startBit);

int32_t fpga9001_BfReadAssembleData(adi_fpga9001_Device_t *device, uint32_t *rxBfData, uint8_t numRdBfData, uint64_t *rdData);
int32_t fpga9001_BfWriteCacheFlush(adi_fpga9001_Device_t *device);
int32_t fpga9001_BfReadCacheFlush(adi_fpga9001_Device_t *device, uint64_t *rdData, uint16_t *numRxBitFields);

#ifdef __cplusplus
}
#endif

#endif /* ADRV9001_BF_HAL_H_ */