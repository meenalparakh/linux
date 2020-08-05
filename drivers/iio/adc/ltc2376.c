// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linear Technology LTC2376 Low Power SAR ADC
 *
 * Copyright (C) 2020 Analog Devices, Inc.
 */

#include <asm/unaligned.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>


#define DEFAULT_SAMPLING_FREQUENCY 2000
#define DEFAULT_DUTY_CYCLE 200
#define MINIMUM_PERIOD 201
#define MAX_WAIT_TIME 5

struct ltc2376_state {
    struct spi_device *spi;
    struct regulator *reg;
    struct pwm_device *pwm;
    struct spi_message spi_msg;
    struct spi_transfer spi_transfer;

    struct mutex lock;
    unsigned int num_bits;

    bool busy;
    unsigned int frequency;
    u8 data[2];
};

ssize_t ltc2376_show_freq (struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
     struct iio_dev *indio_dev = dev_to_iio_dev(dev);
     struct ltc2376_state *st = iio_priv(indio_dev);

    return sprintf(buf, "%d\n", st->frequency);
}

ssize_t ltc2376_store_freq (struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct ltc2376_state *st = iio_priv(indio_dev);

    int period;
    int ret;
    unsigned int freq;

    mutex_lock(&st->lock);

    ret = kstrtouint(buf, 10, &freq);

    if (ret < 0) {
        mutex_unlock(&st->lock);
        return ret;
    }

    period = DIV_ROUND_CLOSEST(NSEC_PER_SEC, freq);
    if (period < MINIMUM_PERIOD){
        mutex_unlock(&st->lock);
        return 0;
    }
    ret = pwm_config(st->pwm, DEFAULT_DUTY_CYCLE, period);

    if (ret < 0){
        printk(KERN_ALERT "ltc2376: pwm_config failed! Aborting write\n");
    }
    st->frequency = freq;
    mutex_unlock(&st->lock);

    return count;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO, ltc2376_show_freq, ltc2376_store_freq);

static struct attribute *ltc2376_attributes[] = {
    &iio_dev_attr_sampling_frequency.dev_attr.attr,
    NULL
};

static const struct attribute_group ltc2376_attribute_group = {
    .attrs = ltc2376_attributes,
};

static int ltc2376_conversion(struct iio_dev *indio_dev, short *val)
{
    struct ltc2376_state *st = iio_priv(indio_dev);
    int ret;
    struct spi_transfer t[] = {
        {
            .rx_buf = st->data,
            .bits_per_word = 8,
            .len = 2,
        },
    };

    ret = iio_device_claim_direct_mode(indio_dev);
    if (ret)
        return ret;

    mutex_lock(&st->lock);

    ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));

    if (ret < 0) {
        dev_err(&st->spi->dev, "error reading 16b \n");
        goto err_unlock;
    }

    *val = ((st->data[0] << 8) | st->data[1]);
//    printk(KERN_ALERT "high byte %u\n", st->data.rx[0]);
//    printk(KERN_ALERT "low byte %u\n", st->data.rx[1]);

err_unlock:
    iio_device_release_direct_mode(indio_dev);
    mutex_unlock(&st->lock);
    return ret;
}

static int ltc2376_read_raw(struct iio_dev *indio_dev,
               const struct iio_chan_spec *chan,
               int *val, int *val2, long m)
{
    int ret;
    short temp;
    struct ltc2376_state *st = iio_priv(indio_dev);

    switch (m) {
        case IIO_CHAN_INFO_RAW:
            if (iio_buffer_enabled(indio_dev))
                return -EBUSY;
            ret = ltc2376_conversion(indio_dev, &temp);
            if (ret < 0)
                    return ret;
            *val = temp;
            return IIO_VAL_INT;
        case IIO_CHAN_INFO_SCALE:
            ret = 2*regulator_get_voltage(st->reg);
            if (ret < 0)
                    return ret;
            *val = ret/1000;
            *val2 = 16;
            return IIO_VAL_FRACTIONAL_LOG2;
        case IIO_CHAN_INFO_OFFSET:
//            *val = -(1 << (st->num_bits-1));
            *val = 0;
            return IIO_VAL_INT;
        default:
            return -EINVAL;
            }
}

static const struct iio_info ltc2376_info = {
    .attrs = &ltc2376_attribute_group,
    .read_raw = &ltc2376_read_raw,
};

static const struct iio_chan_spec ltc2376_channels[] = {
    {
        .type = IIO_VOLTAGE,
        .indexed = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
        .differential = 1,
        .scan_type = {
            .sign = 's',
            .endianness = IIO_BE,
            .realbits = 16,
            .storagebits = 16,

        },
    },

};

static int ltc2376_probe(struct spi_device *spi)
{
    struct ltc2376_state *st;
    struct iio_dev *indio_dev;
    int ret;
    unsigned int period;
    struct pwm_state state = {};

    printk(KERN_ALERT "ltc2376 check \n");

    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
    if (!indio_dev)
        return -ENOMEM;

    st = iio_priv(indio_dev);
//    spi->max_speed_hz = 500000;
    st->spi = spi;
    spi->mode = SPI_CPHA;
    spi_set_drvdata(spi, indio_dev);

    st->frequency = DEFAULT_SAMPLING_FREQUENCY;
    period = DIV_ROUND_CLOSEST(NSEC_PER_SEC, st->frequency);

    st->pwm = devm_pwm_get(&spi->dev, NULL);
    if (IS_ERR(st->pwm)) {
        ret = PTR_ERR(st->pwm);
        if (ret != -EPROBE_DEFER)
            pr_err("pwm_get failed: %d\n", ret);
        return ret;
    }

    pwm_init_state(st->pwm, &state);
    state.period = period;
    state.duty_cycle = DEFAULT_DUTY_CYCLE;
    state.enabled = true;
    ret = pwm_apply_state(st->pwm, &state);
    if (ret) {
        pr_err("Failed to configure PWM: %d\n", ret);
        return ret;
    }

    st->reg = devm_regulator_get(&spi->dev, "vref");
    if (IS_ERR(st->reg))
        return PTR_ERR(st->reg);

    ret = regulator_enable(st->reg);
    if (ret < 0)
        return ret;
    indio_dev->dev.parent = &spi->dev;
    indio_dev->name = "ltc2376";
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = ltc2376_channels;
    indio_dev->num_channels = ARRAY_SIZE(ltc2376_channels);
    indio_dev->info = &ltc2376_info;

//    st->num_bits = indio_dev->channels->scan_type.realbits;

    mutex_init(&st->lock);

    printk(KERN_ALERT "exiting\n");
    return devm_iio_device_register(&spi->dev, indio_dev);
}

static int ltc2376_remove(struct spi_device *spi)
{
   struct iio_dev *indio_dev = spi_get_drvdata(spi);
   struct ltc2376_state *st = iio_priv(indio_dev);

    pwm_disable(st->pwm);
    return 0;
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
    .remove = ltc2376_remove
};
module_spi_driver(ltc2376_driver);

MODULE_AUTHOR("Meenal Parakh <meenalparakh18@gmail.com>");
MODULE_DESCRIPTION("Linear Technology LTC2376 SAR ADC driver");
MODULE_LICENSE("GPL");
