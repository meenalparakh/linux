/**
 * \file fpga9001_bf_dp_tx_dma.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_DP_TX_DMA_H__
#define __FPGA9001_BF_DP_TX_DMA_H__

#include "fpga9001_bf_dp_tx_dma_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_DpTxDmaActiveBfGet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                    uint8_t *bfValue);


int32_t fpga9001_DpTxDmaCompleteBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_DpTxDmaContinuousBfSet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t fpga9001_DpTxDmaContinuousBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_DpTxDmaEnableEnhancedModeBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t fpga9001_DpTxDmaEnableEnhancedModeBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t fpga9001_DpTxDmaEnableSgBfSet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                      uint8_t bfValue);
int32_t fpga9001_DpTxDmaEnableSgBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_DpTxDmaHaltCompleteBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_DpTxDmaLengthBfSet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                    uint32_t bfValue);
int32_t fpga9001_DpTxDmaLengthBfGet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                    uint32_t *bfValue);


int32_t fpga9001_DpTxDmaResetBfSet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                   uint8_t bfValue);
int32_t fpga9001_DpTxDmaResetBfGet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                   uint8_t *bfValue);


int32_t fpga9001_DpTxDmaRunStopBfSet(adi_fpga9001_Device_t *device, 
                                     fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                     uint8_t bfValue);
int32_t fpga9001_DpTxDmaRunStopBfGet(adi_fpga9001_Device_t *device, 
                                     fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                     uint8_t *bfValue);


int32_t fpga9001_DpTxDmaSgLastDescriptorBfSet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                              uint32_t bfValue);
int32_t fpga9001_DpTxDmaSgLastDescriptorBfGet(adi_fpga9001_Device_t *device, 
                                              fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                              uint32_t *bfValue);


int32_t fpga9001_DpTxDmaSimpleStartAddrBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                             uint32_t bfValue);
int32_t fpga9001_DpTxDmaSimpleStartAddrBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                             uint32_t *bfValue);


int32_t fpga9001_DpTxDmaSimpleStartAddrMswBfSet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t fpga9001_DpTxDmaSimpleStartAddrMswBfGet(adi_fpga9001_Device_t *device, 
                                                fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t fpga9001_DpTxDmaSqFirstDescriptorBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                               uint32_t bfValue);
int32_t fpga9001_DpTxDmaSqFirstDescriptorBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                               uint32_t *bfValue);


int32_t fpga9001_DpTxDmaUnderflowBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_DpTxDmaValidDataWidthStreamBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_DpTxDmaValidDataWidthStreamBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_DpTxDmaVersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_DpTxDmaVersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfDpTxDmaChanAddr_e baseAddr, 
                                          uint16_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_DP_TX_DMA_H__

/*  EOF: fpga9001_bf_dp_tx_dma.h */

