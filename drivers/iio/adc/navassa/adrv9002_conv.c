// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9002 RF Transceiver
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include "adrv9002.h"

#if IS_ENABLED(CONFIG_CF_AXI_ADC)
#include "../cf_axi_adc.h"

#define ADI_TX1_REG_OFF			0x2000
#define ADI_TX_REG_RATE			0x4c
#define AIM_AXI_REG(off, addr)		((off) + (addr))

#define AIM_CHAN(_chan, _mod, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .modified = 1,						\
	  .channel = _chan,						\
	  .channel2 = _mod,						\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	  .scan_index = _si,						\
	  .scan_type = {						\
		.sign = _sign,						\
		.realbits = _bits,					\
		.storagebits = 16,					\
		.shift = 0,						\
	  },								\
	}

static const unsigned long adrv9002_available_scan_masks[] = {
	0x01, 0x02, 0x03, 0x04, 0x08, 0x0C, 0x0F,
	0x00
};

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_ADRV9002] = {
		.name = "ADRV9002",
		.max_rate = 245760000,
		.max_testmode = 0,
		.num_channels = 4,
		.scan_masks = adrv9002_available_scan_masks,
		.channel[0] = AIM_CHAN(0, IIO_MOD_I, 0, 16, 'S'),
		.channel[1] = AIM_CHAN(0, IIO_MOD_Q, 1, 16, 'S'),
		.channel[2] = AIM_CHAN(1, IIO_MOD_I, 2, 16, 'S'),
		.channel[3] = AIM_CHAN(1, IIO_MOD_Q, 3, 16, 'S'),
	},
};

static int adrv9002_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val,
			     int *val2,
			     long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate(conv->clk);

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int adrv9002_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val,
			      int val2,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return -ENODEV;
	default:
		return -EINVAL;
	}
}

static int adrv9002_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval,
			       u32 *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	if (!readval)
		axiadc_write(st, reg & 0xFFFF, writeval);
	else
		*readval = axiadc_read(st, reg & 0xFFFF);

	return 0;
}

int adrv9002_hdl_loopback(struct adrv9002_rf_phy *phy, bool enable)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st;
	unsigned int reg, addr, chan, version;

	if (!conv)
		return -ENODEV;

	st = iio_priv(conv->indio_dev);
	version = axiadc_read(st, 0x4000);

	/* Still there but implemented a bit different */
	if (ADI_AXI_PCORE_VER_MAJOR(version) > 7)
		addr = 0x4418;
	else
		addr = 0x4414;

	for (chan = 0; chan < conv->chip_info->num_channels; chan++) {
		reg = axiadc_read(st, addr + (chan) * 0x40);

		if (ADI_AXI_PCORE_VER_MAJOR(version) > 7) {
			if (enable && reg != 0x8) {
				conv->scratch_reg[chan] = reg;
				reg = 0x8;
			} else if (reg == 0x8) {
				reg = conv->scratch_reg[chan];
			}
		} else {
			/* DAC_LB_ENB If set enables loopback of receive data */
			if (enable)
				reg |= BIT(1);
			else
				reg &= ~BIT(1);
		}
		axiadc_write(st, addr + (chan) * 0x40, reg);
	}

	return 0;
}
EXPORT_SYMBOL(adrv9002_hdl_loopback);

static int adrv9002_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	u32 num_chan;
	int i;

	num_chan = conv->chip_info->num_channels;

	conv->indio_dev = indio_dev;

	/* default to rx2tx2 mode */
	axiadc_write(st, ADI_REG_CNTRL, 0);

	for (i = 0; i < num_chan; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_1(i),
			     ADI_DCFILT_OFFSET(0));
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
			     ADI_ENABLE | ADI_IQCOR_ENB);
	}

	/* FIXME: Defaut DAC interface to LVDS 2 Lane for now */
	axiadc_write(st, AIM_AXI_REG(ADI_TX1_REG_OFF, ADI_TX_REG_RATE), 1);

	return 0;
}

static int adrv9002_post_iio(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	/*
	 * FIXME: This is a workaround. Apparently, there is an issue with
	 * the hdl core in the rx1 port on the Q data. Doing a reset on the core
	 * (in the final step of probing - this __only__ worked on the
	 * post_iio_register cb) fixes the problem. However, the real issue
	 * needs to be found and this should be removed...
	 */
	axiadc_write(st, ADI_REG_RSTN, 0);
	msleep(20);
	axiadc_write(st, ADI_REG_RSTN, 0x3);

	return 0;
}

int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy)
{
	struct axiadc_converter *conv;
	struct spi_device *spi = phy->spi;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	conv->chip_info = &axiadc_chip_info_tbl[phy->spi_device_id];
	conv->write_raw = adrv9002_write_raw;
	conv->read_raw = adrv9002_read_raw;
	conv->post_setup = adrv9002_post_setup;
	conv->reg_access = adrv9002_reg_access;
	conv->post_iio_register = adrv9002_post_iio;
	conv->spi = spi;
	conv->phy = phy;

	conv->clk = phy->clks[RX1_SAMPL_CLK];
	conv->adc_clk = clk_get_rate(conv->clk);

	spi_set_drvdata(spi, conv); /* Take care here */

	return 0;
}
EXPORT_SYMBOL(adrv9002_register_axi_converter);

struct adrv9002_rf_phy *adrv9002_spi_to_phy(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);

	return conv->phy;
}
EXPORT_SYMBOL(adrv9002_spi_to_phy);

#else  /* CONFIG_CF_AXI_ADC */

int adrv9002_hdl_loopback(struct adrv9002_rf_phy *phy, bool enable)
{
	return -ENODEV;
}
EXPORT_SYMBOL(adrv9002_hdl_loopback);

int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;

	spi_set_drvdata(spi, phy); /* Take care here */

	return 0;
}
EXPORT_SYMBOL(adrv9002_register_axi_converter);

struct adrv9002_rf_phy *adrv9002_spi_to_phy(struct spi_device *spi)
{
	return spi_get_drvdata(spi);
}
EXPORT_SYMBOL(adrv9002_spi_to_phy);

#endif /* CONFIG_CF_AXI_ADC */
