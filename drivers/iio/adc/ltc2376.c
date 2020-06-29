// SPDX-License-Identifier: GPL-2.0
/*
 *LTC2376 Device driver_register
 *
 *Copyright (C) 2020 Analog Devices, Inc.
 */

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define LTC2376_READ_COMMAND    0x54
#define LTC2376_WRITE_COMMAND   0x14
#define LTC2376_RESERVED_MSK    0xE0

struct ltc2376_state {
	struct spi_device *spi;
	struct regulator *vref;
    
    bool bus_locked;
    
	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;
	
    unsigned int num_bits;
    
    uint8_t data[4] ____cacheline_aligned;
};

static int ltc2376_write_reg(struct ltc2376_state *st, uint8_t val)
{
    struct spi_transfer t = {
        .tx_buf        = st->data,
        .len           = 4,
        .bits_per_word = st->num_bits,
    };
    struct spi_message m;

    st->data[0] = LTC2376_WRITE_COMMAND;
    st->data[1] = val;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    
    if (st->bus_locked)
        return spi_sync_locked(st->spi, &m);

    return spi_sync(st->spi, &m);
}

static int ltc2376_read_reg(struct ltc2376_state *st, unsigned int *val)
{
    struct spi_message m;
    struct spi_transfer t = {0};
    int ret;

    st->data[0] = LTC2376_READ_COMMAND;

    t.rx_buf = st->data;
    t.tx_buf = st->data;
    t.len = 2;
    t.bits_per_word = 16; /* reg reads are only 16 clocks pulses */

    spi_message_init_with_transfers(&m, &t, 1);

    if (st->bus_locked)
        ret = spi_sync_locked(st->spi, &m);
    else
        ret = spi_sync(st->spi, &m);

    if (ret < 0)
        return ret;

    *val = st->data[0];

    return ret;
}


static int ltc2376_reg_access(struct iio_dev *indio_dev,
                 unsigned int reg,
                 unsigned int writeval,
                 unsigned int *readval)
{
    struct ltc2376_state *st = iio_priv(indio_dev);
    int ret;

    mutex_lock(&indio_dev->mlock);
    spi_bus_lock(st->spi->master);
    st->bus_locked = true;

    if (readval)
        ret = ltc2376_read_reg(st, readval);
    else
        ret = ltc2376_write_reg(st, writeval);

    st->bus_locked = false;
    spi_bus_unlock(st->spi->master);
    mutex_unlock(&indio_dev->mlock);

    return ret;
}

static int ltc2376_read_sample(struct ltc2376_state *st, uint32_t *val)
{
    struct spi_message m;
    struct spi_transfer t = {0};
    uint8_t input[4] = {0};
    int ret;

    t.rx_buf = input;
    t.len = 4;
    t.bits_per_word = st->num_bits;

    spi_message_init_with_transfers(&m, &t, 1);

    if (st->bus_locked)
        ret = spi_sync_locked(st->spi, &m);
    else
        ret = spi_sync(st->spi, &m);

    if (ret < 0)
        return ret;

    *val = (input[2] << 16) | (input[1] << 8) | input[0];

    return ret;
}

static int ltc2376_single_conversion(struct iio_dev *indio_dev,
    const struct iio_chan_spec *chan, int *val)
{
    struct ltc2376_state *st = iio_priv(indio_dev);
    unsigned int sample, raw_sample;
    int ret;

    ret = iio_device_claim_direct_mode(indio_dev);
    if (ret)
        return ret;

    spi_bus_lock(st->spi->master);
    st->bus_locked = true;

    ret = ltc2376_read_sample(st, &raw_sample);
    iio_device_release_direct_mode(indio_dev);

    st->bus_locked = false;
    spi_bus_unlock(st->spi->master);

    if (ret)
        return ret;

    sample = raw_sample >> chan->scan_type.shift;
    sample &= (1 << chan->scan_type.realbits) - 1;

    *val = sample;

    return IIO_VAL_INT;
}

static int ltc2376_read_raw(struct iio_dev *indio_dev,
    struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
//    struct ltc2376_state *st = iio_priv(indio_dev);

    switch (info) {
    case IIO_CHAN_INFO_RAW:
        return ltc2376_single_conversion(indio_dev, chan, val);
//    case IIO_CHAN_INFO_SCALE:
//        ret = regulator_get_voltage(st->vref);
//        if (ret < 0)
//            return ret;
//
//        *val = ret / 1000;
//        *val2 = chan->scan_type.realbits;
//
//        return IIO_VAL_FRACTIONAL_LOG2;
//    case IIO_CHAN_INFO_SAMP_FREQ:
//        *val = 1800000;
//
//        return IIO_VAL_INT;
//    case IIO_CHAN_INFO_OFFSET:
//        *val = -(1 << chan->scan_type.realbits);
//
//        return IIO_VAL_INT;
    default:
        break;
    }

    return -EINVAL;
}

static int ltc2376_write_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan,
                int val, int val2, long info)
{
    return 0;
}

static const struct iio_info ltc2376_info = {
	.read_raw = &ltc2376_read_raw,
    .debugfs_reg_access = &ltc2376_reg_access,
};

#define LTC2376_CHANNEL(real_bits)  \
{                                   \
    .type = IIO_VOLTAGE,            \
    .indexed = 1,                   \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
    .scan_type = {                  \
        .sign = 's',                \
        .realbits = real_bits,      \
        .storagebits = 32,          \
    },                              \
    .differential = 1,              \
}                                   \

static const struct iio_chan_spec ltc2376_channels[] = {
    LTC2376_CHANNEL(16)
};

static int ltc2376_probe(struct spi_device *spi)
{
	struct ltc2376_state *st;
	struct iio_dev *indio_dev;

	dev_dbg(&spi->dev, "ltc2376: Hello! Entered ltc2376 probe.\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ltc2376";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ltc2376_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltc2376_channels);
	indio_dev->info = &ltc2376_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ltc2376_of_match[] = {
	{ .compatible = "adi,ltc2376" },
	{ }
};

MODULE_DEVICE_TABLE(of, ltc2376_of_match);

static struct spi_driver ltc2376_driver = {
	.driver = {
		.name = "ltc2376",
		.of_match_table = ltc2376_of_match,
	},
	.probe = ltc2376_probe,
};
module_spi_driver(ltc2376_driver);

MODULE_AUTHOR("Meenal Parakh");
MODULE_DESCRIPTION("Linear Technology LTC2376 SAR ADC driver");
MODULE_LICENSE("GPL");
