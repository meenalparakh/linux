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

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>

#ifdef __GNUC__
#include <unistd.h>
#include <sys/mman.h>
#endif

#include "zc706sd20_bbic_control.h"

/************************** Static helper functions **************************/
static int32_t zc706sd20_fpga_AxiReadWrite(void *devHalCfg, uint8_t writeFlag, uint32_t addr, uint32_t *data);
static int32_t zc706sd20_fpga_AxiBulkReadWrite(void *devHalCfg, uint8_t writeFlag, uint32_t addr, uint32_t data[], const uint32_t numData);

/********************/
/* Public functions */
/********************/

int32_t zc706sd20_BbicOpen(void *devHalCfg)
{
    adi_hal_Cfg_t *halCfg = NULL;
    int32_t halError = (int32_t)ADI_HAL_OK;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;

#ifdef __GNUC__
    /* Open UIO driver to FPGA register interface */
    if (halCfg->bbicCfg.coreRegsCfg.fd <= 0)
    {
        halCfg->bbicCfg.coreRegsCfg.fd = open(halCfg->bbicCfg.coreRegsCfg.drvName, O_RDWR);

        if (halCfg->bbicCfg.coreRegsCfg.fd < 0)
        {
            return (int32_t)ADI_HAL_BBICCTRL_FAIL;
        }
    }
    
    /* Open UIO driver to FPGA Data capture memory RAM interface */
    if (halCfg->bbicCfg.ramCfg.fd <= 0)
    {
        halCfg->bbicCfg.ramCfg.fd = open(halCfg->bbicCfg.ramCfg.drvName, O_RDWR);

        if (halCfg->bbicCfg.ramCfg.fd < 0)
        {
            return (int32_t)ADI_HAL_BBICCTRL_FAIL;
        }
    }
    
        /* Open UIO driver to FPGA Advanced SPI settings interface */
    if (halCfg->bbicCfg.spiAdvRegsCfg.fd <= 0)
    {
        halCfg->bbicCfg.spiAdvRegsCfg.fd = open(halCfg->bbicCfg.spiAdvRegsCfg.drvName, O_RDWR);

        if (halCfg->bbicCfg.spiAdvRegsCfg.fd < 0)
        {
            return (int32_t)ADI_HAL_BBICCTRL_FAIL;
        }
    }
    
#endif /* __GNUC__ */
    return halError;
}

int32_t zc706sd20_BbicClose(void *devHalCfg)
{
    adi_hal_Cfg_t *halCfg = NULL;
    int32_t halError = (int32_t)ADI_HAL_OK;
    
    if (devHalCfg == NULL)
    {
        halError = (int32_t)ADI_HAL_NULL_PTR;
        return halError;
    }

    /* Cast void *devHalCfg structure to the specific structure for ZC706 platform */
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    
#ifdef __GNUC__
    if (halCfg->bbicCfg.coreRegsCfg.fd != 0)
    {
        halError = close(halCfg->bbicCfg.coreRegsCfg.fd);
        if (halError < 0)
        {
            return (int32_t)ADI_HAL_BBICCTRL_FAIL;
        }
        halCfg->bbicCfg.coreRegsCfg.fd = 0;
    }
    
    if (halCfg->bbicCfg.ramCfg.fd != 0)
    {
        halError = close(halCfg->bbicCfg.ramCfg.fd);
        if (halError < 0)
        {
            return (int32_t)ADI_HAL_BBICCTRL_FAIL;
        }
        halCfg->bbicCfg.ramCfg.fd = 0;
    }
    
    if (halCfg->bbicCfg.spiAdvRegsCfg.fd != 0)
    {
        halError = close(halCfg->bbicCfg.spiAdvRegsCfg.fd);
        if (halError < 0)
        {
            return (int32_t)ADI_HAL_BBICCTRL_FAIL;
        }
        halCfg->bbicCfg.spiAdvRegsCfg.fd = 0;
    }
#endif /* __GNUC__ */

    return halError;
}

int32_t zc706sd20_BbicInit(void *devHalCfg)
{
    /* ADI ZC706 platform does not require any BBIC register/control init */
    return (int32_t)ADI_HAL_OK;
}

int32_t zc706sd20_BbicRegisterRead(void *devHalCfg, uint32_t addr, uint32_t *data)
{
    static const uint8_t READ_REG = 0;
   
    if ((devHalCfg == NULL) || (data == NULL))
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }

    return zc706sd20_fpga_AxiReadWrite(devHalCfg, READ_REG, addr, data);
}

int32_t zc706sd20_BbicRegisterWrite(void *devHalCfg, uint32_t addr, uint32_t data)
{
    static const uint8_t WRITE_REG = 1;
   
    if (devHalCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }

    return zc706sd20_fpga_AxiReadWrite(devHalCfg, WRITE_REG, addr, &data);
}

int32_t zc706sd20_BbicRegistersRead(void *devHalCfg, uint32_t addr, uint32_t data[], uint32_t numDataWords)
{
    static const uint8_t READ_REG = 0;
    
    if (devHalCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }

    return zc706sd20_fpga_AxiBulkReadWrite(devHalCfg, READ_REG, addr, data, numDataWords);
}

int32_t zc706sd20_BbicRegistersWrite(void *devHalCfg, uint32_t addr, uint32_t data[], uint32_t numDataWords)
{
    static const uint8_t WRITE_REG = 1;
    
    if (devHalCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }

    return zc706sd20_fpga_AxiBulkReadWrite(devHalCfg, WRITE_REG, addr, data, numDataWords);
}

/**
* \brief Writes and reads to the BBIC (FPGA) AXI memory mapped registers.
*
* \param devHalCfg  Pointer to device instance specific platform settings
* \param writeFlag  Specifies whether to write to (1) or read from (0) addr
* \param addr       The memory address to write to or read from
* \param data       The data to write or the data that was read
*
* \retval ADI_HAL_OK function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
* \retval ADI_HAL_GEN_SW If addr is out of valid range
* \retval ADI_HAL_BBICCTRL_FAIL If critical error during read or write operation
*/
int32_t zc706sd20_fpga_AxiReadWrite(void *devHalCfg, uint8_t writeFlag, uint32_t addr, uint32_t *data)
{
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_fpga_AxiCfg_t *axiCfg = NULL;
    uint32_t offsetAddr = 0;
    int fd = 0;
    void *ptr = NULL;
    
    if (devHalCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
    
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    
    if (data == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
     

    if ((addr >= ZC706SD20_FPGA_UIO0_START_ADDRESS) &&
        (addr <= ZC706SD20_FPGA_UIO0_STOP_ADDRESS))
    {
        offsetAddr = addr - ZC706SD20_FPGA_UIO0_START_ADDRESS;
        axiCfg =  &halCfg->bbicCfg.coreRegsCfg;
    }
    else if ((addr >= ZC706SD20_FPGA_UIO1_START_ADDRESS) &&
        (addr <= ZC706SD20_FPGA_UIO1_STOP_ADDRESS))
    {
        offsetAddr = addr - ZC706SD20_FPGA_UIO1_START_ADDRESS;
        axiCfg =  &halCfg->bbicCfg.ramCfg;
    }    
    else if ((addr >= ZC706SD20_FPGA_UIO2_START_ADDRESS) &&
             (addr <= ZC706SD20_FPGA_UIO2_STOP_ADDRESS))
    {
        offsetAddr = addr - ZC706SD20_FPGA_UIO2_START_ADDRESS;
        axiCfg =  &halCfg->bbicCfg.spiAdvRegsCfg;
    }
    else 
    {
        /* The offset parameter is the actual fpga memory address. Range check to make sure it's valid */
        return (int32_t)ADI_HAL_GEN_SW;
    }

#ifdef __GNUC__
    fd = axiCfg->fd;
    
    /* If UIO driver file not open */
    if (fd <= 0)
    {
        perror("Invalid UIO device file");
        return (int32_t)ADI_HAL_BBICCTRL_FAIL;
    }

    /* offset must be page aligned */
    ptr = mmap(NULL, axiCfg->mapSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    if (ptr == MAP_FAILED)
    {
        munmap(ptr, axiCfg->mapSize);
        perror("Error mapping the file ");
        return (int32_t)ADI_HAL_BBICCTRL_FAIL;
    }
    else
    {
        if (writeFlag)
        {
            *((uint32_t *)((uintptr_t)ptr + offsetAddr)) = *data;
        }
        else
        {
            *data = *((uint32_t*)((uintptr_t)ptr + offsetAddr));
        }

        /* clean up */
        munmap(ptr, axiCfg->mapSize);
    }
    
#endif /* __GNUC__ */

    return (int32_t)ADI_HAL_OK;
}

/**
* \brief Writes and reads to the BBIC (FPGA) AXI memory mapped registers.
*
* \param devHalCfg  Pointer to device instance specific platform settings
* \param writeFlag  Specifies whether to write to (1) or read from (0) addr
* \param addr       The memory address to write to or read from
* \param data       The data to write or the data that was read
* \param numData    Number of data words to write or read
*
* \retval ADI_HAL_OK function completed successfully, no action required
* \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
* \retval ADI_HAL_BBICCTRL_FAIL If critcal error during read or write transaction
*/
int32_t zc706sd20_fpga_AxiBulkReadWrite(void *devHalCfg, uint8_t writeFlag, uint32_t addr, uint32_t data[], const uint32_t numData)
{
    adi_hal_Cfg_t *halCfg = NULL;
    adi_hal_fpga_AxiCfg_t *ramCfg = NULL;
    /* uint32_t offsetAddr = 0; */
    int fd = 0;
    void *ptr = NULL;
    
    if (devHalCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
    
    if (data == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
    
    halCfg = (adi_hal_Cfg_t *)devHalCfg;
    ramCfg = &halCfg->bbicCfg.ramCfg;

    if (ramCfg == NULL)
    {
        return (int32_t)ADI_HAL_NULL_PTR;
    }
    
    if ((addr + numData) > ramCfg->mapSize)
    {
        return (int32_t)ADI_HAL_BBICCTRL_FAIL;
    }

#ifdef __GNUC__
    
    fd = ramCfg->fd;
    
    /* If UIO driver file not open */
    if (fd <= 0)
    {
        perror("Invalid UIO device file");
        return (int32_t)ADI_HAL_BBICCTRL_FAIL;
    }

    /* offset must be page aligned */
    ptr = mmap(NULL, ramCfg->mapSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    if (ptr == MAP_FAILED)
    {
        munmap(ptr, ramCfg->mapSize);
        perror("Error mapping the file ");
        return (int32_t)ADI_HAL_BBICCTRL_FAIL;
    }
    else
    {
        if (writeFlag)
        {
            memcpy((void *)((uintptr_t)ptr + addr), data, numData * sizeof(uint32_t));
        }
        else
        {
            memcpy(data, (void *)((uintptr_t)ptr + addr), numData * sizeof(uint32_t));
        }

        /* clean up */
        munmap(ptr, ramCfg->mapSize);
    }
#endif /* __GNUC__ */
    
    return (int32_t)ADI_HAL_OK;
}
