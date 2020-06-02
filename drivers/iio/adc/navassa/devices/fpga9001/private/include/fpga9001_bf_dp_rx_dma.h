/**
 * \file fpga9001_bf_dp_rx_dma.h Automatically generated file with generator ver 0.0.1.0.
 *
 * \brief Contains BitField functions to support FPGA9001 transciever device.
 *
 * FPGA9001 BITFIELD VERSION: 0.0.0.1
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the FPGA9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef __FPGA9001_BF_DP_RX_DMA_H__
#define __FPGA9001_BF_DP_RX_DMA_H__

#include "fpga9001_bf_dp_rx_dma_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_DpRxDmaActiveBfGet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                    uint8_t *bfValue);


int32_t fpga9001_DpRxDmaCompleteBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_DpRxDmaEnableEnhancedModeBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t fpga9001_DpRxDmaEnableEnhancedModeBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_DpRxDmaEnableSgBfSet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                      uint8_t bfValue);
int32_t fpga9001_DpRxDmaEnableSgBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_DpRxDmaHaltCompleteBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_DpRxDmaLengthBfSet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                    uint32_t bfValue);
int32_t fpga9001_DpRxDmaLengthBfGet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                    uint32_t *bfValue);


int32_t fpga9001_DpRxDmaOverflowBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_DpRxDmaResetBfSet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                   uint8_t bfValue);
int32_t fpga9001_DpRxDmaResetBfGet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                   uint8_t *bfValue);


int32_t fpga9001_DpRxDmaRunStopBfSet(adi_fpga9001_Device_t *device, 
                                     fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                     uint8_t bfValue);
int32_t fpga9001_DpRxDmaRunStopBfGet(adi_fpga9001_Device_t *device, 
                                     fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                     uint8_t *bfValue);


int32_t fpga9001_DpRxDmaSgFirstDescriptorBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t fpga9001_DpRxDmaSgFirstDescriptorBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t fpga9001_DpRxDmaSgLastDescriptorBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                              uint32_t bfValue);
int32_t fpga9001_DpRxDmaSgLastDescriptorBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t fpga9001_DpRxDmaSimpleStartAddrBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                             uint32_t bfValue);
int32_t fpga9001_DpRxDmaSimpleStartAddrBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t fpga9001_DpRxDmaSimpleStartAddrMswBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t fpga9001_DpRxDmaSimpleStartAddrMswBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t fpga9001_DpRxDmaValidDataWidthStreamBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_DpRxDmaValidDataWidthStreamBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_DpRxDmaVersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_DpRxDmaVersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfDpRxDmaChanAddr_e baseAddr, 
                                          uint16_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_DP_RX_DMA_H__

/*  EOF: fpga9001_bf_dp_rx_dma.h */

