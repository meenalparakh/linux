/**
 * \file
 * \brief Contains top level fpga9001 related function prototypes for
 *        adi_fpga9001.c
 *
 * FPGA9001 API Version: $ADI_FPGA9001_API_VERSION$
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_FPGA9001_H_
#define _ADI_FPGA9001_H_
#include "adi_platform.h"
#include "adi_fpga9001_user.h"
#include "adi_fpga9001_types.h"

#include "adi_fpga9001_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Performs a Hardware Initialization for FPGA Device.
 *
 * \pre This function may be called after device->common.devHalInfo has been initialized with
 * user values
 *
 * \param[in] fpga9001      Context variable - Pointer to FPGA9001 device data structure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_HwOpen(adi_fpga9001_Device_t *fpga9001);

/**
 * \brief Performs a hardware shutdown  for FPGA Device.
 *
 * \pre This function may be called after device->common.devHalInfo has been initialized with
 * user values
 *
 * \param[in] fpga9001      Context variable - Pointer to FPGA9001 device data structure
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_HwClose(adi_fpga9001_Device_t *fpga9001);

// FIXME: This is an awful way to do this, but it should go away in the future
extern char fpgaBinDirectory[256];
/**
 * \brief Change the value of the fpgaBinDirectory global variable used by adi_fpga9001_SwitchBin()
 * 
 * \deprecated Store binaries in the default location
 *
 * \param[in] directory The new path to the directory containing FPGA binaries
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_FpgaBinDirectorySet(const char* directory);

/**
 * \brief Reprogram the FPGA to the specified binary
 *
 * \param[in] fpga9001      Context variable - Pointer to FPGA9001 device data structure
 * \param[in] bin               The binary image to switch to
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_SwitchBin(adi_fpga9001_Device_t *fpga9001, adi_fpga9001_Binary_e bin);

/**
 * \brief Get the version of the FPGA binary
 *
 * \param[in]  fpga9001      Context variable - Pointer to FPGA9001 device data structure
 * \param[out] fpgaVersion         The current version
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_fpga9001_VersionGet(adi_fpga9001_Device_t *fpga9001, adi_fpga9001_Version_t *fpgaVersion);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_FPGA9001_H_ */