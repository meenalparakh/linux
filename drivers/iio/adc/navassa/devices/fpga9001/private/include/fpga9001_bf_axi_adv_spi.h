/**
 * \file fpga9001_bf_axi_adv_spi.h Automatically generated file with generator ver 0.0.1.0.
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

#ifndef __FPGA9001_BF_AXI_ADV_SPI_H__
#define __FPGA9001_BF_AXI_ADV_SPI_H__

#include "fpga9001_bf_axi_adv_spi_types.h"
#include "adi_fpga9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t fpga9001_AxiAdvSpiAdvFeat3wireTriPointBfSet(adi_fpga9001_Device_t *device, 
                                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                     uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiAdvFeat3wireTriPointBfGet(adi_fpga9001_Device_t *device, 
                                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                     uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiAdvFeatClockDivBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                uint16_t bfValue);
int32_t fpga9001_AxiAdvSpiAdvFeatClockDivBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                uint16_t *bfValue);


int32_t fpga9001_AxiAdvSpiAdvFeatCphaBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                            uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiAdvFeatCphaBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiAdvFeatCpolBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                            uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiAdvFeatCpolBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                            uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiAdvFeatEnableBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                              uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiAdvFeatEnableBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                              uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiAdvFeatFourWireBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiAdvFeatFourWireBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiAdvFeatLsbFirstBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiAdvFeatLsbFirstBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiAdvFeatMisoSampleSlipBfSet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                      uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiAdvFeatMisoSampleSlipBfGet(adi_fpga9001_Device_t *device, 
                                                     fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
uint8_t channelId,                                                      uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiCphaBfSet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                    uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiCphaBfGet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                    uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiCpolBfSet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                    uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiCpolBfGet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                    uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiDrrFullBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiDrrFullBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiDrrFullIenBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiDrrFullIenBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiDrrOverrunBfSet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiDrrOverrunBfGet(adi_fpga9001_Device_t *device, 
                                          fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiDrrOverrunIenBfSet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiDrrOverrunIenBfGet(adi_fpga9001_Device_t *device, 
                                             fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiDtrEmptyBfSet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiDtrEmptyBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiDtrEmptyIenBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiDtrEmptyIenBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiGieBfSet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                   uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiGieBfGet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                   uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiLoopBfSet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                    uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiLoopBfGet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                    uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiLsbFirstBfSet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiLsbFirstBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiManSsEnBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiManSsEnBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiMasterBfSet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                      uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiMasterBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiModfBfGet(adi_fpga9001_Device_t *device, 
                                    fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                    uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiModfFlagBfGet(adi_fpga9001_Device_t *device, 
                                        fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiModfIenBfSet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiModfIenBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiModuleVersionMajorBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiModuleVersionMinorBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                                  uint16_t *bfValue);


int32_t fpga9001_AxiAdvSpiMstrInhibitBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiMstrInhibitBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiResetBfSet(adi_fpga9001_Device_t *device, 
                                     fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                     uint32_t bfValue);


int32_t fpga9001_AxiAdvSpiRxDataBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiRxEmptyBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiRxFifoOccupancyBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t fpga9001_AxiAdvSpiRxFifoResetBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                           uint8_t bfValue);


int32_t fpga9001_AxiAdvSpiRxFullBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiSlaveSelectBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                           uint32_t bfValue);
int32_t fpga9001_AxiAdvSpiSlaveSelectBfGet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                           uint32_t *bfValue);


int32_t fpga9001_AxiAdvSpiSpeBfSet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                   uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiSpeBfGet(adi_fpga9001_Device_t *device, 
                                   fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                   uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiTxDataBfSet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                      uint8_t bfValue);


int32_t fpga9001_AxiAdvSpiTxEmptyBfGet(adi_fpga9001_Device_t *device, 
                                       fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiTxFifoHalfEmptyBfSet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiTxFifoHalfEmptyBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiTxFifoHalfEmptyIenBfSet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t fpga9001_AxiAdvSpiTxFifoHalfEmptyIenBfGet(adi_fpga9001_Device_t *device, 
                                                  fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t fpga9001_AxiAdvSpiTxFifoOccupancyBfGet(adi_fpga9001_Device_t *device, 
                                               fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                               uint16_t *bfValue);


int32_t fpga9001_AxiAdvSpiTxFifoResetBfSet(adi_fpga9001_Device_t *device, 
                                           fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                           uint8_t bfValue);


int32_t fpga9001_AxiAdvSpiTxFullBfGet(adi_fpga9001_Device_t *device, 
                                      fpga9001_BfAxiAdvSpiChanAddr_e baseAddr, 
                                      uint8_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __FPGA9001_BF_AXI_ADV_SPI_H__

/*  EOF: fpga9001_bf_axi_adv_spi.h */

