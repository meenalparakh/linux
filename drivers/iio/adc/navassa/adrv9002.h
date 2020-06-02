/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV9002
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_TRX_ADRV9002_H_
#define IIO_TRX_ADRV9002_H_

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include "adi_adrv9001.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_arm_types.h"
#include "adi_adrv9001_auxdac.h"
#include "adi_adrv9001_auxdac_types.h"
#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_cals_types.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_gpio.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_mcs.h"
#include "adi_adrv9001_mcs_types.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_rx_gaincontrol.h"
#include "adi_adrv9001_rx_gaincontrol_types.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_rx_types.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_spi_types.h"
#include "adi_adrv9001_stream.h"
#include "adi_adrv9001_stream_types.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_user.h"
#include "adi_adrv9001_utilities.h"
#include "adi_adrv9001_utilities_types.h"
#include "adi_adrv9001_version.h"

#define ADRV_ADDRESS(port, chan)	((port) << 8 | (chan))
#define ADRV_ADDRESS_PORT(addr)		((addr) >> 8)
#define ADRV_ADDRESS_CHAN(addr)		((addr) & 0xFF)

enum {
	ADRV9002_CHANN_1,
	ADRV9002_CHANN_2,
	ADRV9002_CHANN_MAX,
};

#ifdef ADI_COMMON_VERBOSE
/*
 * Setting logEnable enables error reports which are only available if
 * ADI_COMMON_VERBOSE is defined
 */
#define	adrv9002_set_loglevel(common, level)	\
	adi_common_LogLevelSet(common, level); \
	(common)->error.logEnable = true
#else
#define	adrv9002_set_loglevel(common, level)	\
	adi_common_LogLevelSet(common, level)
#endif

enum ad900x_device_id {
	ID_ADRV9002,
};

enum adrv9002_clocks {
	RX1_SAMPL_CLK,
	RX2_SAMPL_CLK,
	TX1_SAMPL_CLK,
	TX2_SAMPL_CLK,
	NUM_ADRV9002_CLKS,
};

enum adrv9002_rx_ext_info {
	RX_QEC_FIC,
	RX_QEC_W_POLY,
	RX_AGC,
	RX_BBDC,
	RX_HD2,
	RX_RSSI_CAL,
	RX_RFDC,
	RX_RSSI,
	RX_DECIMATION_POWER,
	RX_RF_BANDWIDTH,
	RX_POWERDOWN,
	RX_GAIN_CTRL_PIN_MODE,
	RX_ENSM_MODE,
	RX_NCO_FREQUENCY,
	RX_ADC_SWITCH,
};

enum adrv9002_tx_ext_info {
	TX_QEC,
	TX_LOL,
	TX_LB_PD,
	TX_PAC,
	TX_CLGC,
	TX_RF_BANDWIDTH,
	TX_POWERDOWN,
	TX_ATTN_CTRL_PIN_MODE,
	TX_ENSM_MODE,
};

struct adrv9002_clock {
	struct clk_hw		hw;
	struct spi_device	*spi;
	struct adrv9002_rf_phy	*phy;
	unsigned long		rate;
	enum adrv9002_clocks	source;
};

struct adrv9002_chan {
	adi_adrv9001_ChannelState_e cached_state;
	adi_common_ChannelNumber_e number;
	u32 power;
	u8 enabled;
};

struct adrv9002_rx_chan {
	struct adrv9002_chan channel;
	struct adi_adrv9001_GainControlCfg *agc;
	struct adi_adrv9001_RxGainControlPinCfg *pin_cfg;
	int nco_freq;
};

struct adrv9002_tx_chan {
	struct adrv9002_chan channel;
	struct adi_adrv9001_TxAttenuationPinControlCfg *pin_cfg;
	u8 dac_boost_en;
};

#define to_clk_priv(_hw) container_of(_hw, struct adrv9002_clock, hw)

struct adrv9002_rf_phy {
	struct spi_device	*spi;
	struct iio_dev		*indio_dev;
	struct gpio_desc	*reset_gpio;

	/* Protect against concurrent accesses to the device */
	struct mutex		lock;

	struct clk		*clks[NUM_ADRV9002_CLKS];
	struct adrv9002_clock	clk_priv[NUM_ADRV9002_CLKS];
	struct clk_onecell_data	clk_data;
	struct bin_attribute	bin;
	char			*bin_attr_buf;
	struct adrv9002_rx_chan	rx_channels[ADRV9002_CHANN_MAX];
	struct adrv9002_tx_chan	tx_channels[ADRV9002_CHANN_MAX];
	struct adi_adrv9001_GpioCfg *adrv9002_gpios;
	struct adi_adrv9001_Device	adrv9001_device;
	struct adi_adrv9001_Device	*adrv9001;
	struct adi_hal_Cfg		hal;
	struct adi_adrv9001_Init	*curr_profile;
	struct adi_adrv9001_Init	profile;
	struct adi_adrv9001_InitCals	init_cals;

	int			spi_device_id;
	int			ngpios;

};

int adrv9002_hdl_loopback(struct adrv9002_rf_phy *phy, bool enable);
int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy);
struct adrv9002_rf_phy *adrv9002_spi_to_phy(struct spi_device *spi);
int adrv9002_spi_read(struct spi_device *spi, u32 reg);
int adrv9002_spi_write(struct spi_device *spi, u32 reg, u32 val);
/* get init structs */
struct adi_adrv9001_SpiSettings *adrv9002_spi_settings_get(void);
struct adi_adrv9001_Init *adrv9002_init_get(void);
struct adi_adrv9001_RadioCtrlInit *adrv9002_radio_ctrl_init_get(void);
struct adi_adrv9001_PlatformFiles *adrv9002_platform_files_get(void);

#endif
