/**
 * \file adrv9001_bf_nvs_pll_mem_map.h Automatically generated file with generator ver 0.0.1.0.
 *
 * \brief Contains BitField functions to support ADRV9001 transciever device.
 *
 * ADRV9001 BITFIELD VERSION: 0.0.0.1
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef __ADRV9001_BF_NVS_PLL_MEM_MAP_H__
#define __ADRV9001_BF_NVS_PLL_MEM_MAP_H__

#include "adrv9001_bf_nvs_pll_mem_map_types.h"
#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C"{
#endif


int32_t adrv9001_NvsPllMemMapAbCounterResetbBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapAbCounterResetbBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapAbortBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapAbortBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapAuxAdcReadyBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapAuxLoBufNorthPdBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapAuxLoBufNorthPdBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapAuxLoBufSouthPdBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapAuxLoBufSouthPdBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapAuxLoDriverOffchipLobufPdBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapAuxLoDriverOffchipLobufPdBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapAuxLoDriverOffchipPdBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapAuxLoDriverOffchipPdBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapAuxLoTiaTunerBufPdBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapAuxLoTiaTunerBufPdBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapBypLoadDelayBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapBypLoadDelayBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCalperBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         adrv9001_BfNvsPllMemMapCalper_e bfValue);
int32_t adrv9001_NvsPllMemMapCalperBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         adrv9001_BfNvsPllMemMapCalper_e *bfValue);


int32_t adrv9001_NvsPllMemMapCaltypBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         adrv9001_BfNvsPllMemMapCaltyp_e bfValue);
int32_t adrv9001_NvsPllMemMapCaltypBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         adrv9001_BfNvsPllMemMapCaltyp_e *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenBackupResetBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenBackupResetBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenDivOrxPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenDivOrxPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenDivRxPdBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenDivRxPdBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenDivTxPdBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenDivTxPdBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenHsdigclkOutbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenHsdigclkOutbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenHsdigclkSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenHsdigclkSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenMcs2digCountBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenMcs2digCountBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenOrx12OutbufPdOvrdBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenOrx12OutbufPdOvrdBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenOrx12OutbufPdOvrdSelBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenOrx12OutbufPdOvrdSelBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenOrx34OutbufPdOvrdBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenOrx34OutbufPdOvrdBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenOrx34OutbufPdOvrdSelBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenOrx34OutbufPdOvrdSelBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenResetSrcSelBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenResetSrcSelBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenRx12OutbufPdOvrdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenRx12OutbufPdOvrdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenRx12OutbufPdOvrdSelBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenRx12OutbufPdOvrdSelBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenRx34OutbufPdOvrdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenRx34OutbufPdOvrdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenRx34OutbufPdOvrdSelBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenRx34OutbufPdOvrdSelBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSampleDivInitOrxBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSampleDivInitOrxBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSampleDivInitRxBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSampleDivInitRxBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSampleDivInitTxBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSampleDivInitTxBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSampleDivSelOrxBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSampleDivSelOrxBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSampleDivSelRxBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSampleDivSelRxBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSampleDivSelTxBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSampleDivSelTxBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSerdesOutbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSerdesOutbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSubRxForOrxBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSubRxForOrxBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSyncDivSelBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSyncDivSelBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenSyncclkOutputEnBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenSyncclkOutputEnBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenTx12OutbufPdOvrdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenTx12OutbufPdOvrdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenTx12OutbufPdOvrdSelBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenTx12OutbufPdOvrdSelBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenTx34OutbufPdOvrdBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenTx34OutbufPdOvrdBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenTx34OutbufPdOvrdSelBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenTx34OutbufPdOvrdSelBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenUseBackupResetBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenUseBackupResetBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenUseSerdesMcsBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenUseSerdesMcsBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenVcoSelBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenVcoSelBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenXfaceDivInitOrxBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenXfaceDivInitOrxBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenXfaceDivInitRxBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenXfaceDivInitRxBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenXfaceDivInitTxBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenXfaceDivInitTxBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenXfaceDivSelOrxBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenXfaceDivSelOrxBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenXfaceDivSelRxBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenXfaceDivSelRxBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapClkgenXfaceDivSelTxBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapClkgenXfaceDivSelTxBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpBiasRfiltBypBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpBiasRfiltBypBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpCalClkDivideBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpCalClkDivideBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpCalEnBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpCalEnBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpCalInitBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpCalInitBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpCalValidBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpFCalBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpFCalBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpFCalBitsBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpFCalBitsBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpIBfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpIBfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpLeveldetPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpLeveldetPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpOffsetLvlBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpOffsetLvlBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpOffsetOffBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpOffsetOffBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpOverrangeHighFlagBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpOverrangeHighTcBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpOverrangeLowFlagBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpOverrangeLowTcBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpPumpUpBiasCntrlBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpPumpUpBiasCntrlBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpTestBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpTestBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpVlevelHighFlagBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpVlevelHighFlagBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpVlevelHighTcBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpVlevelHighTcBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpVlevelLowFlagBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpVlevelLowFlagBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpVlevelLowTcBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCpVlevelLowTcBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCpendBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapCstepBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapCstepBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapCtstepBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapCtstepBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDirectModAnalogClkdivValueBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDirectModAnalogClkdivValueBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDirectModEmptyBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDirectModEnableBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDirectModEnableBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDirectModFifoBypassBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDirectModFifoBypassBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDirectModFifoClrBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDirectModFifoClrBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDirectModFifoEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDirectModFifoEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDirectModFullBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDirectModRdFifoEnableDelayBfSet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDirectModRdFifoEnableDelayBfGet(adi_adrv9001_Device_t *device, 
                                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl0BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl0BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl1BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl1BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl2BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl2BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl3BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl3BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl4BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl4BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl5BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl5BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl6BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl6BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl7BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl7BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnl8BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnl8BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDnlOffsetBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDnlOffsetBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDtapsB0BfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDtapsB0BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDtapsB1BfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDtapsB1BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDtapsB2BfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDtapsB2BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapDtapsB3BfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapDtapsB3BfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapEndvcocalMaxcntEnBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapEndvcocalMaxcntEnBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapFbClockAdvBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapFbClockAdvBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapFcalSingleUpdBandsBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapFcalSingleUpdBandsBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapFdovdBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint32_t bfValue);
int32_t adrv9001_NvsPllMemMapFdovdBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapFdovdmBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapFdovdmBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapForceLockBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapForceLockBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapForceVcoInitAlcValueBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapForceVcoInitAlcValueBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapFreqCalCntRdselBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapFreqCalCntRdselBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapFreqCalMaxCntBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint32_t bfValue);
int32_t adrv9001_NvsPllMemMapFreqCalMaxCntBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapFreqCalSingleBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapFreqCalSingleBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapFreqCalcntRdbckBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapIcalwaitBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           adrv9001_BfNvsPllMemMapIcalwait_e bfValue);
int32_t adrv9001_NvsPllMemMapIcalwaitBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           adrv9001_BfNvsPllMemMapIcalwait_e *bfValue);


int32_t adrv9001_NvsPllMemMapIdistBypassResExtBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapIdistBypassResExtBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapIdistBypassResIntBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapIdistBypassResIntBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapIdistExtPdBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapIdistExtPdBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapIdistIntPdBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapIdistIntPdBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapInvLoIBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapInvLoIBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapInvLoQBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapInvLoQBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfBypassC1BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfBypassC1BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfBypassC2BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfBypassC2BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfBypassR1BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfBypassR1BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfBypassR3BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfBypassR3BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfC1BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfC1BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfC2BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfC2BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfC3BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfC3BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfR1BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfR1BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLfR3BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLfR3BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLgDivBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapLgDivBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapLo1OffsetLoPhaseDetFilterBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLo1OffsetLoPhaseDetFilterBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLo1OffsetLoPhaseDetPdBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLo1OffsetLoPhaseDetPdBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLo2OffsetLoPhaseDetFilterBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLo2OffsetLoPhaseDetFilterBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLo2OffsetLoPhaseDetPdBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLo2OffsetLoPhaseDetPdBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLoSyncResetbBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLoSyncResetbBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLoSyncSamplerFlopsPdBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLoSyncSamplerFlopsPdBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLoSyncSamplerLoInputBufPdBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLoSyncSamplerLoInputBufPdBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLockDetectResetbBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLockDetectResetbBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLockdetCntBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLockdetCntBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLockdetModeBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLockdetModeBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLogenDivideModeBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLogenDivideModeBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLogenInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLogenInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLogenPdBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLogenPdBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLogenQuadDivPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLogenQuadDivPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLogenResetBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLogenResetBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapLogenVcoBufHighRBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapLogenVcoBufHighRBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsClkgenSyncEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsClkgenSyncEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsDeviceClkDividerSyncEnableBfSet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsDeviceClkDividerSyncEnableBfGet(adi_adrv9001_Device_t *device, 
                                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsDigitalClockSyncEnableBfSet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsDigitalClockSyncEnableBfGet(adi_adrv9001_Device_t *device, 
                                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsEnableBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsEnableBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsJesdSysrefEnableBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsJesdSysrefEnableBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsPulseDelayBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsPulseDelayBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsPulseDelayDigSerdesBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         adrv9001_BfNvsPllMemMapMcsPulseDelayDigSerdes_e bfValue);
int32_t adrv9001_NvsPllMemMapMcsPulseDelayDigSerdesBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         adrv9001_BfNvsPllMemMapMcsPulseDelayDigSerdes_e *bfValue);


int32_t adrv9001_NvsPllMemMapMcsPulseWidthDigSerdesBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         adrv9001_BfNvsPllMemMapMcsPulseWidthDigSerdes_e bfValue);
int32_t adrv9001_NvsPllMemMapMcsPulseWidthDigSerdesBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         adrv9001_BfNvsPllMemMapMcsPulseWidthDigSerdes_e *bfValue);


int32_t adrv9001_NvsPllMemMapMcsResetbBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsResetbBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsSdmSyncEnableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsSdmSyncEnableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsSerdesAlignBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 adrv9001_BfNvsPllMemMapMcsSerdesAlign_e bfValue);
int32_t adrv9001_NvsPllMemMapMcsSerdesAlignBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 adrv9001_BfNvsPllMemMapMcsSerdesAlign_e *bfValue);


int32_t adrv9001_NvsPllMemMapMcsSpiStatusBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMcsWaitCountBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapMcsWaitCountBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapMod0BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint32_t bfValue);
int32_t adrv9001_NvsPllMemMapMod0BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapMod1BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint32_t bfValue);
int32_t adrv9001_NvsPllMemMapMod1BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapMod2BfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint32_t bfValue);
int32_t adrv9001_NvsPllMemMapMod2BfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapMpendBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapNumDitherBitsBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapNumDitherBitsBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapPfdClockEdgeBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapPfdClockEdgeBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapPfdResetbBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapPfdResetbBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapPfdWidthBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapPfdWidthBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapPhadjBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint32_t bfValue);
int32_t adrv9001_NvsPllMemMapPhadjBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapPhdiffBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapPoaiBfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapPoaqBfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapPrescalerPdBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapPrescalerPdBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapPrscBiasBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapPrscBiasBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapPsbusyBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapPsenBfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapPsenBfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapQthrBfSet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       adrv9001_BfNvsPllMemMapQthr_e bfValue);
int32_t adrv9001_NvsPllMemMapQthrBfGet(adi_adrv9001_Device_t *device, 
                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                       adrv9001_BfNvsPllMemMapQthr_e *bfValue);


int32_t adrv9001_NvsPllMemMapQuickFreqCalEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapQuickFreqCalEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapQuickFreqCalThresholdBfSet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapQuickFreqCalThresholdBfGet(adi_adrv9001_Device_t *device, 
                                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapReadEffectFtwBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapReadEffectFtwBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRefClkDivideRatioBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRefClkDivideRatioBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRefClkDividerPdBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRefClkDividerPdBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRefClkDividerResetbBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRefClkDividerResetbBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloInAmpBiasBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloInAmpBiasBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloInDivideModeBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloInDivideModeBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloInFiltBypBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloInFiltBypBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloInPdBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloInPdBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloInSwapInputPnBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloInSwapInputPnBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloOutDivResetBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloOutDivResetBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloOutDivideModeBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloOutDivideModeBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloOutInbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloOutInbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfExtloOutPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfExtloOutPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRfLoBufPdBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRfLoBufPdBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx12Lo1InbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx12Lo1InbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx12Lo2InbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx12Lo2InbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx12LomuxPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx12LomuxPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx12LomuxPdOverrideSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx12LomuxPdOverrideSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx12SelLo2BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx12SelLo2BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx34Lo1InbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx34Lo1InbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx34Lo2InbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx34Lo2InbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx34LomuxPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx34LomuxPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx34LomuxPdOverrideSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx34LomuxPdOverrideSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapRx34SelLo2BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapRx34SelLo2BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapSdaBfSet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapSdaBfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapSdmBypBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapSdmBypBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapSdmFracBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint32_t bfValue);
int32_t adrv9001_NvsPllMemMapSdmFracBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint32_t *bfValue);


int32_t adrv9001_NvsPllMemMapSdmIntBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapSdmIntBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapSdmPdBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapSdmPdBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapSdmResetbBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapSdmResetbBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapSparesBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapSparesBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapSynLockBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapSynthPdBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapSynthPdBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTcforceBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTcforceBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTcforcenBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTcforcenBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTcidacBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapTcidacBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapTcpolBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTcpolBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTcupdinitBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTcupdinitBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTsprscBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         adrv9001_BfNvsPllMemMapTsprsc_e bfValue);
int32_t adrv9001_NvsPllMemMapTsprscBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         adrv9001_BfNvsPllMemMapTsprsc_e *bfValue);


int32_t adrv9001_NvsPllMemMapTszeroBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTszeroBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx12Lo1InbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx12Lo1InbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx12Lo2InbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx12Lo2InbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx12LomuxPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx12LomuxPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx12LomuxPdOverrideSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx12LomuxPdOverrideSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx12SelLo2BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx12SelLo2BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx34Lo1InbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx34Lo1InbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx34Lo2InbufPdBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx34Lo2InbufPdBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx34LomuxPdOverrideBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx34LomuxPdOverrideBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx34LomuxPdOverrideSelBfSet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx34LomuxPdOverrideSelBfGet(adi_adrv9001_Device_t *device, 
                                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapTx34SelLo2BfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapTx34SelLo2BfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoAlcCalEnBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoAlcCalEnBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoAlcPdBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoAlcPdBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBiasDacLvlShift2maPmosEnBfSet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                              uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBiasDacLvlShift2maPmosEnBfGet(adi_adrv9001_Device_t *device, 
                                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBiasPdBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBiasPdBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBiasRefBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBiasRefBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBiasStartupBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBiasStartupBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBiasTcfBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBiasTcfBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBufPdBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBufPdBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBypRfilt1BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBypRfilt1BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBypRfilt2BfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBypRfilt2BfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoBypassBiasrBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoBypassBiasrBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalAlcClkDivBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalAlcClkDivBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalAlcInitWaitBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalAlcInitWaitBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalAlcStepBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalAlcStepBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalAlcWaitBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalAlcWaitBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalBusyBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalInitBfSet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalInitBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalInitDelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfNvsPllMemMapVcoCalInitDel_e bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalInitDelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                adrv9001_BfNvsPllMemMapVcoCalInitDel_e *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalLogicResetbBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalLogicResetbBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalOffsetBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalOffsetBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalRefMonitorBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalRefMonitorBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalRefTcfBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalRefTcfBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCalTcfPdBfSet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCalTcfPdBfGet(adi_adrv9001_Device_t *device, 
                                              adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                              uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCoarseCalEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCoarseCalEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCompBypBiasrBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCompBypBiasrBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCompOutBfGet(adi_adrv9001_Device_t *device, 
                                             adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                             uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCompPdBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCompPdBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoCoreSwDisableBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoCoreSwDisableBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoFAlcBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoFAlcBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoFAlcEnBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoFAlcEnBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoFCoarseBandBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint16_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoFCoarseBandBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint16_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoFCoarseBandEnBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoFCoarseBandEnBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoFFineBandBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoFFineBandBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoFFineBandEnBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoFFineBandEnBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoFFineBandIndexEnBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoFFineBandIndexEnBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoFineCalEnBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoFineCalEnBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoInitAlcValueBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoInitAlcValueBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoBypassBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoLdoBypassBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoDisThermalShutdownBfSet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                           uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoLdoDisThermalShutdownBfGet(adi_adrv9001_Device_t *device, 
                                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                           uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoFilterBypassBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoLdoFilterBypassBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoIlimitIncBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoLdoIlimitIncBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoPdBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoLdoPdBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoStatusBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoThermalShutdown0cBfSet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoLdoThermalShutdown0cBfGet(adi_adrv9001_Device_t *device, 
                                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoVoutSelBfSet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoLdoVoutSelBfGet(adi_adrv9001_Device_t *device, 
                                                adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoLdoVoutTrimBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoLdoVoutTrimBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoOutLvlBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoOutLvlBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoPdBfSet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoPdBfGet(adi_adrv9001_Device_t *device, 
                                        adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                        uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoPeakDetLdoLvlBfSet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoPeakDetLdoLvlBfGet(adi_adrv9001_Device_t *device, 
                                                   adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                   uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoPeakDetPdBfSet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoPeakDetPdBfGet(adi_adrv9001_Device_t *device, 
                                               adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                               uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoPtatBiasRcalBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoPtatBiasRcalBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoPtatPdBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoPtatPdBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoPtatStartupLeakbBfSet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoPtatStartupLeakbBfGet(adi_adrv9001_Device_t *device, 
                                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                      uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcCalResetbBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcCalResetbBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcIdacBypBiasFiltBfSet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcIdacBypBiasFiltBfGet(adi_adrv9001_Device_t *device, 
                                                       adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                       uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcIdacScaleLsbBfSet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcIdacScaleLsbBfGet(adi_adrv9001_Device_t *device, 
                                                    adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                    uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcMirrorIoutBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcMirrorIoutBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcPdBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcPdBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcRfilt1SelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcRfilt1SelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcRfilt2SelBfSet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcRfilt2SelBfGet(adi_adrv9001_Device_t *device, 
                                                 adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                 uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcTrackingEnBfSet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcTrackingEnBfGet(adi_adrv9001_Device_t *device, 
                                                  adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                  uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoTcWaitBfSet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoTcWaitBfGet(adi_adrv9001_Device_t *device, 
                                            adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                            uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoVarBfSet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoVarBfGet(adi_adrv9001_Device_t *device, 
                                         adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                         uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcoVarTcBfSet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcoVarTcBfGet(adi_adrv9001_Device_t *device, 
                                           adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                           uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVcocalMaxcntbandEnBfSet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVcocalMaxcntbandEnBfGet(adi_adrv9001_Device_t *device, 
                                                     adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                                     uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapVtForceBfSet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t bfValue);
int32_t adrv9001_NvsPllMemMapVtForceBfGet(adi_adrv9001_Device_t *device, 
                                          adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                          uint8_t *bfValue);


int32_t adrv9001_NvsPllMemMapWpaBfGet(adi_adrv9001_Device_t *device, 
                                      adrv9001_BfNvsPllMemMapChanAddr_e baseAddr, 
                                      uint32_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif // __ADRV9001_BF_NVS_PLL_MEM_MAP_H__

/*  EOF: adrv9001_bf_nvs_pll_mem_map.h */

