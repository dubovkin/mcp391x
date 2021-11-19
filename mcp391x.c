/*
 * mcp391x.c - driver for the Microchip mcp391x/3/4 chip 
 *
 * Copyright (C) 2017, Aleksandr Dubovkin
 * Author: Aleksandr Dubovkin <dubovkin.as@gmail.com>
 *
 * Datasheet mcp3914 : http://ww1.microchip.com/downloads/en/DeviceDoc/20005216A.pdf
 * Datasheet mcp3913 : http://ww1.microchip.com/downloads/en/DeviceDoc/20005227A.pdf
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/crc16.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <asm/unaligned.h>

#include "mcp391x.h"

enum  mcp391x_device_ids {
	id_mcp3913,
	id_mcp3914,
};

struct mcp391x_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
};

static int mcp391x_write_8reg(struct mcp391x_state *st, u8 reg_address)
{
	int ret;
	
	mutex_lock(&st->lock);

	st->tx[0] = MCP391x_WRITE_REG(reg_address);
	ret = spi_write(st->spi, &st->tx, 1);

	mutex_unlock(&st->lock);
	
	return ret;
}

static int mcp391x_write_24reg(struct mcp391x_state *st, u8 reg_address, u32 value)
{
	int ret;
	
	mutex_lock(&st->lock);

	st->tx[0] = MCP391x_WRITE_REG(reg_address);
	st->tx[1] = (value >> 16) & 0xFF;
	st->tx[2] = (value >> 8) & 0xFF;
	st->tx[3] = value & 0xFF;
	ret = spi_write(st->spi, &st->tx, 4);

	mutex_unlock(&st->lock);
	
	return ret;
}

static int mcp391x_read_24reg(struct mcp391x_state *st, u8 reg_address)
{
	int ret;
	
	struct spi_transfer	xfers [] = {
	{
			.tx_buf        = st->tx,
			.len		   = 1,
			.bits_per_word = 8,
	}, {
			.rx_buf		   = st->rx,
			.len		   = 3,
			.bits_per_word = 8,
		},
	};
		
	mutex_lock(&st->lock);
	st->tx[0] = MCP391x_READ_REG(reg_address);
	memset(&st->tx[1], 0, 3);
		
	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret < 0)
		return ret;
	mutex_unlock(&st->lock);
	
	return (st->rx[0] << 16 | st->rx[1] << 8 | st->rx[2]);
}

static int mcp391x_read_32reg(struct mcp391x_state *st, u8 reg_address)
{
	int ret;
	u8 tmp[1];
	
	struct spi_transfer	xfers [] = {
	{
			.tx_buf        = st->tx,
			.rx_buf		   = &tmp[0],
			.len		   = 1,
			.bits_per_word = 8,
	}, {
			.rx_buf		   = st->rx,
			.len		   = 4,
			.bits_per_word = 32,
		},
	};
			
	mutex_lock(&st->lock);
	st->tx[0] = MCP391x_READ_REG(reg_address);
	memset(&st->tx[1], 0, 3);
	memset(&tmp[0], 0, 1);
		
	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret < 0)
		return ret;
	mutex_unlock(&st->lock);

	return (st->rx[3] << 16 | st->rx[2] << 8 | st->rx[1]);
}

static int mcp391x_setup_read_continuos_24reg(struct mcp391x_state *st)
{
		int ret;
		
		ret = mcp391x_write_24reg(st, MCP39xx_STATUSCOM,
	          MCP391x_STATUSCOM_READ_TYPES |
	          MCP391x_STATUSCOM_WIDTH_DATA_24 |
	          MCP391x_STATUSCOM_DR_LINK);
	
		//mcp391x_write_8reg(st, MCP39xx_CH_0);

		return ret;
}

static int mcp391x_setup_read_continuos_32reg(struct mcp391x_state *st)
{
		int ret;
		
		ret = mcp391x_write_24reg(st, MCP39xx_STATUSCOM,
	          MCP391x_STATUSCOM_READ_TYPES |
	          MCP391x_STATUSCOM_WIDTH_DATA_32_SIGN |
	          MCP391x_STATUSCOM_DR_LINK);

		//mcp391x_write_8reg(st, MCP39xx_CH_0);

		return ret;
}

static int mcp391x_read_continuos(struct mcp391x_state *st, u8 *buffer, int len)
{
	struct spi_transfer	xfers [] = {
		{
				.tx_buf        = st->tx,
				.len		   = 1,
				.bits_per_word = 8,
		}, {
				.rx_buf		   = buffer,
				.len		   = len,
				.bits_per_word = 32,
				//.bits_per_word = 8,
		},
	};
	struct spi_message	m;
	int ret;
	unsigned int val, i;

	st->tx[0] = MCP391x_READ_REG(MCP39xx_CH_0);
	memset(&st->tx[1], 0, 3);
	
	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret < 0)
		return ret;

	return ret;
}

static void mcp391x_read_regs(struct mcp391x_state *st)
{
	int ret = 0;
	int i;
	
	for (i = 0; i < 32; i++) {
		ret = mcp391x_read_24reg(st, i);
		printk(KERN_INFO "ret:  %ul 0x%x\n\n", ret, ret);
	}

	return;
}

static int mcp391x_setup(struct mcp391x_state *st)
{
	int ret = 0;

	printk(KERN_INFO "Write MCP39xx_STATUSCOM:  0x%x\n", MCP391x_STATUSCOM_WIDTH_DATA_32_SIGN |
	          MCP391x_STATUSCOM_DR_LINK);

	ret = mcp391x_write_24reg(st, MCP39xx_STATUSCOM,
	          MCP391x_STATUSCOM_WIDTH_DATA_32_SIGN |
	          MCP391x_STATUSCOM_DR_LINK);
	ret = mcp391x_read_24reg(st, MCP39xx_STATUSCOM);
	printk(KERN_INFO "Read MCP39xx_STATUSCOM:  0x%x\n\n", ret);
	
	ret = mcp391x_write_24reg(st, MCP39xx_CONFIG1,
	          MCP391x_CONFIG1_CLKEXT_EN | MCP391x_CONFIG1_VREFEXT_EN);       

#if 1
	ret = mcp391x_read_24reg(st, MCP39xx_CONFIG1);
	printk(KERN_INFO "Read MCP39xx_CONFIG1:  0x%x\n", ret);

	ret = mcp391x_write_24reg(st, MCP39xx_CONFIG0, 
								MCP391x_CONFIG0_DIFER(3)| 
								MCP391x_CONFIG0_BOOST(2)|
								MCP391x_CONFIG0_OSR_256 |
								MCP391x_CONFIG0_VREFCAL);
	ret = mcp391x_read_24reg(st, MCP39xx_CONFIG0);
	printk(KERN_INFO "Read MCP39xx_CONFIG0:  0x%x\n", ret);

	ret = mcp391x_read_24reg(st, MCP39xx_CONFIG0);

	printk(KERN_INFO "Write MCP39xx_GAIN:  0x%x\n", 
												MCP39xx_GAIN_PGA_CH0(0)|
												MCP39xx_GAIN_PGA_CH1(0)|
												MCP39xx_GAIN_PGA_CH2(0)|
												MCP39xx_GAIN_PGA_CH3(0)|
												MCP39xx_GAIN_PGA_CH4(0)|
												MCP39xx_GAIN_PGA_CH5(0));

	ret = mcp391x_write_24reg(st, MCP39xx_GAIN, MCP39xx_GAIN_PGA_CH0(0)|
												MCP39xx_GAIN_PGA_CH1(0)|
												MCP39xx_GAIN_PGA_CH2(0)|
												MCP39xx_GAIN_PGA_CH3(0)|
												MCP39xx_GAIN_PGA_CH4(0)|
												MCP39xx_GAIN_PGA_CH5(0));
	ret = mcp391x_read_32reg(st, MCP39xx_GAIN);
	printk(KERN_INFO "Read MCP39xx_GAIN:  0x%x\n", ret);
#endif
	return ret;
}

static int mcp391x_adc_conversion(struct mcp391x_state *st, u8 channel)
{
	u32 ret = 0;

	ret = mcp391x_read_32reg(st, channel);

	if (ret < 0)
		return ret;
	
	return ret;
}

static int mcp391x_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct mcp391x_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if(iio_buffer_enabled(indio_dev))
		    return -EBUSY;
	    ret = mcp391x_adc_conversion(st, chan->address);
	    if (ret < 0)
			return ret;

		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = 1200000; //regulator_get_voltage(st->reg);
		if (ret < 0)
			return ret;

		/* convert regulator output voltage to mV */
		*val = ret / 1000;
		*val2 = 24;
		ret = IIO_VAL_FRACTIONAL_LOG2;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int mcp391x_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct mcp391x_state *st = iio_priv(indio_dev);
	int ret;

#if 0
	if (state) {
		ret = mcp391x_write_reg(st, MCP39xx_STATUSCOM,
	          MCP391x_STATUSCOM_WIDTH_DATA_24 |
	          MCP391x_STATUSCOM_DR_LINK);
		if (ret < 0)
			return ret;
	} else {
	    ret = mcp391x_write_reg(st, MCP39xx_STATUSCOM,
	          MCP391x_STATUSCOM_WIDTH_DATA_24);
	    if (ret < 0)
			return ret;
	}
#endif
	return 0;
}

static int mcp391x_validate_device(struct iio_trigger *trig,
				   struct iio_dev *indio_dev)
{
	struct iio_dev *indio = iio_trigger_get_drvdata(trig);

	if (indio != indio_dev)
		return -EINVAL;

	return 0;
}

static int mcp391x_preenable(struct iio_dev *indio_dev)
{
	struct mcp391x_state *st = iio_priv(indio_dev);
	
	return mcp391x_setup_read_continuos_32reg(st);
}

static int mcp391x_postdisable(struct iio_dev *indio_dev)
{
	struct mcp391x_state *st = iio_priv(indio_dev);
	int ret;
	
	gpio_set_value(114, 0);
	
	ret = mcp391x_write_24reg(st, MCP39xx_STATUSCOM,
	          MCP391x_STATUSCOM_WIDTH_DATA_24 |
	           /*MCP391x_STATUSCOM_WIDTH_DATA_32 |*/
	           /*MCP391x_STATUSCOM_WIDTH_DATA_32_SIGN |*/
	          MCP391x_STATUSCOM_DR_LINK);
	
	return ret;
}

static const struct iio_buffer_setup_ops iio_triggered_buffer_setup_ops = {
	.preenable = &mcp391x_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &mcp391x_postdisable,
};

static irqreturn_t mcp391x_trigger_handler(int irq, void *private)
{
	struct iio_poll_func *pf = (struct iio_poll_func *)private;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct mcp391x_state *st = iio_priv(indio_dev);
	int ret;

	memset(st->data, 0x00, ADC_CHn_DATA_LENGTH(32));

	ret = mcp391x_read_continuos(st, st->data, ADC_CHn_DATA_LENGTH(32));
	if (ret == 0) {
		iio_push_to_buffers(indio_dev, st->data);
	}
	
	iio_trigger_notify_done(indio_dev->trig);
	
	return IRQ_HANDLED;
}

static const struct iio_trigger_ops mcp391x_trigger_ops = {
	.owner = THIS_MODULE,
	.validate_device = &mcp391x_validate_device,
	.set_trigger_state = &mcp391x_set_trigger_state,
};

static const struct iio_info mcp391x_info = {
	.read_raw = mcp391x_read_raw,
	.driver_module = THIS_MODULE,
};

#define MCP391x_DIFF_CHAN(_channel, _channel2)          \
	{                                                   \
		.type = IIO_VOLTAGE,                            \
		.indexed = 1,                                   \
		.channel = _channel,                            \
		.channel2 = _channel2,                          \
		.address = _channel,				            \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
		.scan_index = _channel,                     	\
		.scan_type = {                                  \
			.sign = 's',                                \
			.realbits = 32,                             \
			.storagebits = 32,                          \
			.shift = 0,                                 \
			.endianness = IIO_LE,                       \
		},                                              \
		.differential = 1,                              \
	}

	
static const struct iio_chan_spec mcp3913_channels[] = {
	MCP391x_DIFF_CHAN(0, 0),
	MCP391x_DIFF_CHAN(1, 1),
	MCP391x_DIFF_CHAN(2, 2),
	MCP391x_DIFF_CHAN(3, 3),
	MCP391x_DIFF_CHAN(4, 4),
	MCP391x_DIFF_CHAN(5, 5),
};

static const struct iio_chan_spec mcp3914_channels[] = {
	MCP391x_DIFF_CHAN(0, 0),
	MCP391x_DIFF_CHAN(1, 1),
	MCP391x_DIFF_CHAN(2, 2),
	MCP391x_DIFF_CHAN(3, 3),
	MCP391x_DIFF_CHAN(4, 4),
	MCP391x_DIFF_CHAN(5, 5),
	MCP391x_DIFF_CHAN(6, 6),
	MCP391x_DIFF_CHAN(7, 7),
};

static const struct mcp391x_chip_info mcp391x_chip_infos[] = {
	[id_mcp3913] = {
		.channels = mcp3913_channels,
		.num_channels = ARRAY_SIZE(mcp3913_channels),
	},
	[id_mcp3914] = {
		.channels = mcp3914_channels,
		.num_channels = ARRAY_SIZE(mcp3914_channels),
	},
};

static irqreturn_t mcp391x_event_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = (struct iio_dev *)private;
	struct mcp391x_state *st = iio_priv(indio_dev);
		
	iio_trigger_poll(st->trig);
		
	return IRQ_HANDLED;
}

static int mcp391x_probe(struct spi_device *spi)
{
	struct mcp391x_state *st;
	struct iio_dev *indio_dev;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *dataready_gpio;
	int ret;
	int irq;
	
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	dataready_gpio = devm_gpiod_get(&spi->dev, "dataready", GPIOD_IN);
	if (IS_ERR(dataready_gpio)) {
		ret = PTR_ERR(dataready_gpio);
		dev_err(&spi->dev, "cannot get dataready_gpio %d\n", ret);
		goto reg_disable;
	} 
	
	irq = gpiod_to_irq(dataready_gpio);
		
	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mcp391x_info;

	st->chip_info =
		&mcp391x_chip_infos[spi_get_device_id(spi)->driver_data];

	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	st->reg = devm_regulator_get(&spi->dev, "vcc3v3");
	if (IS_ERR(st->reg))
		return PTR_ERR(st->reg);

	ret = regulator_enable(st->reg);
	if (ret < 0)
		return ret;
	
	st->buffer = kmalloc(ADC_CHn_DATA_LENGTH(4), GFP_KERNEL);

	mutex_init(&st->lock);

	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
					 &mcp391x_trigger_handler, &iio_triggered_buffer_setup_ops);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "Failed to setup buffer\n");
		return ret;
	}

	st->trig = devm_iio_trigger_alloc(&spi->dev, "%s-trigger",
							indio_dev->name);
	if (st->trig == NULL) {
		ret = -ENOMEM;
		dev_err(&indio_dev->dev, "Failed to allocate iio trigger\n");
		goto fail_trigger_alloc;
	}
	
	st->trig->ops = &mcp391x_trigger_ops;
	st->trig->dev.parent = &spi->dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	indio_dev->trig = st->trig;
	iio_trigger_get(indio_dev->trig);
	ret = iio_trigger_register(st->trig);
	if (ret)
		goto trigger_free;

	ret = request_irq(irq, mcp391x_event_handler, IRQF_TRIGGER_FALLING,
	                spi_get_device_id(spi)->name,
	                indio_dev);
	if (ret) {
		dev_err(&spi->dev, "unable to request IRQ\n");
		goto trigger_free;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto reg_disable;

	mcp391x_setup(st);

	return 0;
		
trigger_free:
	iio_trigger_free(st->trig);
reg_disable:
	regulator_disable(st->reg);
fail_dev_register:
fail_trigger_alloc:
	iio_triggered_buffer_cleanup(indio_dev);
	
	return ret;
}

static int mcp391x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct mcp391x_state *st = iio_priv(indio_dev);

	iio_triggered_buffer_cleanup(indio_dev);
	iio_device_unregister(indio_dev);
	regulator_disable(st->reg);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id mcp391x_dt_ids[] = {
	{
		.compatible = "mcp3913",
		.data = &mcp391x_chip_infos[id_mcp3913],
	}, {
		.compatible = "mcp3914",
		.data = &mcp391x_chip_infos[id_mcp3914],
	}, {
	}
};
MODULE_DEVICE_TABLE(of, mcp391x_dt_ids);
#endif

static const struct spi_device_id mcp391x_id[] = {
	{"mcp3913", id_mcp3913},
	{"mcp3913", id_mcp3914},
	{}
};

static struct spi_driver mcp391x_driver = {
	.driver = {
		.name = "mcp391x",
	},
	.probe = mcp391x_probe,
	.remove = mcp391x_remove,
	.id_table	= mcp391x_id,
};
module_spi_driver(mcp391x_driver);

MODULE_AUTHOR("Aleksandr Dubovkin <dubovkin.as@gmail.com>");
MODULE_DESCRIPTION("Microchip mcp3914 driver");
MODULE_LICENSE("GPL v2");
