
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>

#include <linux/mfd/tlv320aic3262-core.h>
#include <linux/mfd/tlv320aic3262-registers.h>
#define DEBUG
struct aic3262_gpio {
	unsigned int reg;
	u8 mask;
	u8 shift;
};
struct aic3262_gpio  aic3262_gpio_control[] = {
	{
		.reg = AIC3262_GPIO1_IO_CNTL,
		.mask = AIC3262_GPIO_D6_D2,
		.shift = AIC3262_GPIO_D2_SHIFT,
	},
	{
		.reg = AIC3262_GPIO2_IO_CNTL,
		.mask = AIC3262_GPIO_D6_D2,
		.shift = AIC3262_GPIO_D2_SHIFT,
	},
	{
		.reg = AIC3262_GPI1_EN,
		.mask = AIC3262_GPI1_D2_D1,
		.shift = AIC3262_GPIO_D1_SHIFT,
	},
	{
		.reg = AIC3262_GPI2_EN,
		.mask = AIC3262_GPI2_D5_D4,
		.shift = AIC3262_GPIO_D4_SHIFT,
	},
	{
		.reg = AIC3262_GPO1_OUT_CNTL,
		.mask = AIC3262_GPO1_D4_D1,
		.shift = AIC3262_GPIO_D1_SHIFT,
	},
};
static int aic3262_read(struct aic3262 *aic3262, unsigned int reg,
		       int bytes, void *dest)
{
	int ret;
	int i;
	u8 *buf = dest;

	BUG_ON(bytes <= 0);

	ret = aic3262->read_dev(aic3262, reg, bytes, dest);
	if (ret < 0)
		return ret;

	for (i = 0; i < bytes ; i++) {
		dev_vdbg(aic3262->dev, "Read %04x from R%d(0x%x)\n",
			 buf[i], reg + i, reg + i);
	}

	return ret;
}

/**
 * aic3262_reg_read: Read a single TLV320AIC3262 register.
 *
 * @aic3262: Device to read from.
 * @reg: Register to read.
 */
int aic3262_reg_read(struct aic3262 *aic3262, unsigned int reg)
{
	unsigned char val;
	int ret;

	mutex_lock(&aic3262->io_lock);

	ret = aic3262_read(aic3262, reg, 1, &val);

	mutex_unlock(&aic3262->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(aic3262_reg_read);

/**
 * aic3262_bulk_read: Read multiple TLV320AIC3262 registers
 *
 * @aic3262: Device to read from
 * @reg: First register
 * @count: Number of registers
 * @buf: Buffer to fill.  The data will be returned big endian.
 */
int aic3262_bulk_read(struct aic3262 *aic3262, unsigned int reg,
		     int count, u8 *buf)
{
	int ret;

	mutex_lock(&aic3262->io_lock);

	ret = aic3262_read(aic3262, reg, count, buf);

	mutex_unlock(&aic3262->io_lock);


	return ret;
}
EXPORT_SYMBOL_GPL(aic3262_bulk_read);

static int aic3262_write(struct aic3262 *aic3262, unsigned int reg,
			int bytes, const void *src)
{
	const u8 *buf = src;
	int i;

	BUG_ON(bytes <= 0);

	for (i = 0; i < bytes ; i++) {
		dev_vdbg(aic3262->dev, "Write %04x to R%d(0x%x)\n",
			buf[i], reg + i, reg + i);
	}

	return aic3262->write_dev(aic3262, reg, bytes, src);
}

/**
 * aic3262_reg_write: Write a single TLV320AIC3262 register.
 *
 * @aic3262: Device to write to.
 * @reg: Register to write to.
 * @val: Value to write.
 */
int aic3262_reg_write(struct aic3262 *aic3262, unsigned int reg,
		     unsigned char val)
{
	int ret;


	mutex_lock(&aic3262->io_lock);

	dev_dbg(aic3262->dev, "w 30 %x %x", reg, val);
	ret = aic3262_write(aic3262, reg, 1, &val);

	mutex_unlock(&aic3262->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(aic3262_reg_write);

/**
 * aic3262_bulk_write: Write multiple TLV320AIC3262 registers
 *
 * @aic3262: Device to write to
 * @reg: First register
 * @count: Number of registers
 * @buf: Buffer to write from.  Data must be big-endian formatted.
 */
int aic3262_bulk_write(struct aic3262 *aic3262, unsigned int reg,
		      int count, const u8 *buf)
{
	int ret;

	mutex_lock(&aic3262->io_lock);

	ret = aic3262_write(aic3262, reg, count, buf);

	mutex_unlock(&aic3262->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(aic3262_bulk_write);

/**
 * aic3262_set_bits: Set the value of a bitfield in a TLV320AIC3262 register
 *
 * @aic3262: Device to write to.
 * @reg: Register to write to.
 * @mask: Mask of bits to set.
 * @val: Value to set (unshifted)
 */
int aic3262_set_bits(struct aic3262 *aic3262, unsigned int reg,
		    unsigned char mask, unsigned char val)
{
	int ret;
	u8 r;

	mutex_lock(&aic3262->io_lock);

	ret = aic3262_read(aic3262, reg, 1, &r);
	if (ret < 0)
		goto out;


	r &= ~mask;
	r |= (val & mask);

	dev_dbg(aic3262->dev, "w 30 %x %x", reg, r);
	ret = aic3262_write(aic3262, reg, 1, &r);

out:
	mutex_unlock(&aic3262->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(aic3262_set_bits);

/**
 * aic3262_wait_bits: wait for a value of a bitfield in a TLV320AIC3262 register
 *
 * @aic3262: Device to write to.
 * @reg: Register to write to.
 * @mask: Mask of bits to set.
 * @val: Value to set (unshifted)
 * @sleep: mdelay value in each iteration in milliseconds
 * @count: iteration count for timeout
 */
int aic3262_wait_bits(struct aic3262 *aic3262, unsigned int reg,
		unsigned char mask, unsigned char val, int sleep, int counter)
{
	int status;
	int timeout =  sleep*counter;

	status = aic3262_reg_read(aic3262, reg);
	while (((status & mask) != val) && counter) {
		mdelay(sleep);
		status = aic3262_reg_read(aic3262, reg);
		counter--;
	};
	if (!counter)
		dev_err(aic3262->dev,
		 "wait_bits timedout (%d millisecs). lastval 0x%x\n",
		 timeout, status);
	return counter;
}
EXPORT_SYMBOL_GPL(aic3262_wait_bits);

/* to be changed -- Mukund*/
static struct resource aic3262_codec_resources[] = {
	{
		.start = AIC3262_IRQ_HEADSET_DETECT,
		.end   = AIC3262_IRQ_SPEAKER_OVER_TEMP,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource aic3262_gpio_resources[] = {
	{
		.start = AIC3262_GPIO1,
		.end   = AIC3262_GPO1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell aic3262_devs[] = {
	{
		.name = "tlv320aic3262-codec",
		.num_resources = ARRAY_SIZE(aic3262_codec_resources),
		.resources = aic3262_codec_resources,
	},

	{
		.name = "tlv320aic3262-gpio",
		.num_resources = ARRAY_SIZE(aic3262_gpio_resources),
		.resources = aic3262_gpio_resources,
		.pm_runtime_no_callbacks = true,
	},
};


#ifdef CONFIG_PM
static int aic3262_suspend(struct device *dev)
{
	struct aic3262 *aic3262 = dev_get_drvdata(dev);

	aic3262->suspended = true;

	return 0;
}

static int aic3262_resume(struct device *dev)
{
	struct aic3262 *aic3262 = dev_get_drvdata(dev);


	aic3262->suspended = false;

	return 0;
}

static UNIVERSAL_DEV_PM_OPS(aic3262_pm_ops, aic3262_suspend, aic3262_resume,
				NULL);
#endif


/*
 * Instantiate the generic non-control parts of the device.
 */
static int aic3262_device_init(struct aic3262 *aic3262, int irq)
{
	struct aic3262_pdata *pdata = aic3262->dev->platform_data;
	const char *devname;
	int ret, i;
	u8 revID, pgID;
	unsigned int naudint = 0;
	u8 resetVal = 1;

	mutex_init(&aic3262->io_lock);
	dev_set_drvdata(aic3262->dev, aic3262);
	if (pdata) {
		if (pdata->gpio_reset) {
			ret = gpio_request(pdata->gpio_reset,
				"aic3262-reset-pin");
			if (ret != 0) {
				dev_err(aic3262->dev,
				"Failed to reset aic3262 using gpio %d\n",
				pdata->gpio_reset);
				goto err_return;
			}
			gpio_direction_output(pdata->gpio_reset, 1);
			mdelay(5);
			gpio_direction_output(pdata->gpio_reset, 0);
			mdelay(5);
			gpio_direction_output(pdata->gpio_reset, 1);
			mdelay(5);
		}
	}


	/* run the codec through software reset */
	ret = aic3262_reg_write(aic3262, AIC3262_RESET_REG, resetVal);
	if (ret < 0) {
		dev_err(aic3262->dev, "Could not write to AIC3262 register\n");
		goto err_return;
	}

	mdelay(10);

	ret = aic3262_reg_read(aic3262, AIC3262_REV_PG_ID);
	if (ret < 0) {
		dev_err(aic3262->dev, "Failed to read ID register\n");
		goto err_return;
	}
	revID = (ret & AIC3262_REV_MASK) >> AIC3262_REV_SHIFT;
	pgID = (ret & AIC3262_PG_MASK) >> AIC3262_PG_SHIFT;
	switch (revID) {
	case 3:
		devname = "TLV320AIC3262";
		if (aic3262->type != TLV320AIC3262)
			dev_warn(aic3262->dev, "Device registered as type %d\n",
				 aic3262->type);
		aic3262->type = TLV320AIC3262;
		break;
	case 1:
		devname = "TLV320AIC3262";
		if (aic3262->type != TLV320AIC3262)
			dev_warn(aic3262->dev, "Device registered as type %d\n",
				 aic3262->type);
		aic3262->type = TLV320AIC3262;
		break;

	default:
		dev_err(aic3262->dev, "Device is not a TLV320AIC3262, ID is %x\n",
			ret);
		ret = -EINVAL;
		goto err_return;

	}

	dev_info(aic3262->dev, "%s revision %c\n", devname, 'D' + ret);


	if (pdata) {
		if (pdata->gpio_irq == 1) {
			naudint = gpio_to_irq(pdata->naudint_irq);
			gpio_request(pdata->naudint_irq, "aic3262-gpio-irq");
			gpio_direction_input(pdata->naudint_irq);
		} else
			naudint = pdata->naudint_irq;

		aic3262->irq = naudint;
		aic3262->irq_base = pdata->irq_base;
		for (i = 0; i < AIC3262_NUM_GPIO; i++) {
			if (pdata->gpio[i].used) {
				/* Direction is input */
				if (pdata->gpio[i].in) {
					/* set direction to input for GPIO,
					and enable for GPI */
					aic3262_set_bits(aic3262,
						aic3262_gpio_control[i].reg,
						aic3262_gpio_control[i].mask,
						0x1 <<
						aic3262_gpio_control[i].shift);

					if (pdata->gpio[i].in_reg)
						/* Some input modes, does not
						need extra registers to be
						written */
						aic3262_set_bits(aic3262,
							pdata->gpio[i].in_reg,
							pdata->gpio[i].
							in_reg_bitmask,
							pdata->gpio[i].value <<
							pdata->gpio[i].
							in_reg_shift);
				} else {
					/* Direction si output */
					aic3262_set_bits(aic3262,
						aic3262_gpio_control[i].reg,
						aic3262_gpio_control[i].mask,
						pdata->gpio[i].value <<
						aic3262_gpio_control[i].shift);
				}
			} else
				aic3262_set_bits(aic3262,
					aic3262_gpio_control[i].reg,
					aic3262_gpio_control[i].mask, 0x0);
		}
	}

	if (naudint) {
		/* codec interrupt */
		ret = aic3262_irq_init(aic3262);
		if (ret)
			goto err_irq;
	}

	ret = mfd_add_devices(aic3262->dev, -1,
			      aic3262_devs, ARRAY_SIZE(aic3262_devs),
			      NULL, 0);
	if (ret != 0) {
		dev_err(aic3262->dev, "Failed to add children: %d\n", ret);
		goto err_irq;
	}

	pm_runtime_enable(aic3262->dev);
	pm_runtime_resume(aic3262->dev);

	return 0;

err_irq:
	aic3262_irq_exit(aic3262);
err_return:
	kfree(aic3262);
	return ret;
}

static void aic3262_device_exit(struct aic3262 *aic3262)
{
	pm_runtime_disable(aic3262->dev);
	mfd_remove_devices(aic3262->dev);
	aic3262_irq_exit(aic3262);
	kfree(aic3262);
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)


static int aic3262_i2c_read_device(struct aic3262 *aic3262, unsigned int reg,
				  int bytes, void *dest)
{
	struct i2c_client *i2c = aic3262->control_data;
	union aic326x_reg_union *aic_reg = (union aic326x_reg_union *) &reg;
	char *value;
	int ret;
	u8 buf[2];
	u8 page, book, offset;
	page = aic_reg->aic326x_register.page;
	book = aic_reg->aic326x_register.book;
	offset = aic_reg->aic326x_register.offset;
	if (aic3262->book_no != book) {
		/* We should change to page 0.
		Change the book by writing to offset 127 of page 0
		Change the page back to whatever was set before change page */
		buf[0] = 0x0;
		buf[1] = 0x0;
		ret = i2c_master_send(i2c, (unsigned char *)buf, 2);
		if (ret < 0)
			return ret;
		if (ret != 2)
			return -EIO;
		buf[0] = 127;
		buf[1] = book;
		ret = i2c_master_send(i2c, (unsigned char *)buf, 2);
		if (ret < 0)
			return ret;
		if (ret != 2)
			return -EIO;
		aic3262->book_no = book;
		aic3262->page_no = 0x0;
	}

	if (aic3262->page_no != page) {
		buf[0] = 0x0;
		buf[1] = page;
		ret = i2c_master_send(i2c, (unsigned char *) buf, 2);

		if (ret < 0)
			return ret;
		if (ret != 2)
			return -EIO;
		aic3262->page_no = page;
	}

	/* Send the required offset */
	buf[0] = offset ;
	ret = i2c_master_send(i2c, (unsigned char *)buf, 1);
	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;

	ret = i2c_master_recv(i2c, dest, bytes);
	value = dest;
	if (ret < 0)
		return ret;
	if (ret != bytes)
		return -EIO;
	return ret;
}

static int aic3262_i2c_write_device(struct aic3262 *aic3262, unsigned int reg,
				   int bytes, const void *src)
{
	struct i2c_client *i2c = aic3262->control_data;
	int ret;

	union aic326x_reg_union *aic_reg = (union aic326x_reg_union *) &reg;

	u8 buf[2];
	u8 write_buf[bytes + 1];
	u8 page, book, offset;
	page = aic_reg->aic326x_register.page;
	book = aic_reg->aic326x_register.book;
	offset = aic_reg->aic326x_register.offset;
	if (aic3262->book_no != book) {
		/* We should change to page 0.
		Change the book by writing to offset 127 of page 0
		Change the page back to whatever was set before change page*/
		buf[0] = 0x0;
		buf[1] = 0x0;
		ret = i2c_master_send(i2c, (unsigned char *)buf, 2);

		if (ret < 0)
			return ret;
		if (ret != 2)
			return -EIO;
		buf[0] = 127;
		buf[1] = book;
		ret = i2c_master_send(i2c, (unsigned char *)buf, 2);
		if (ret < 0)
			return ret;
		if (ret != 2)
			return -EIO;
		aic3262->book_no = book;
		aic3262->page_no = 0x0;
	}

	if (aic3262->page_no != page) {
		buf[0] = 0x0;
		buf[1] = page;
		ret = i2c_master_send(i2c, (unsigned char *) buf, 2);
		if (ret < 0)
			return ret;
		if (ret != 2)
			return -EIO;
		aic3262->page_no = page;
	}
	write_buf[0] = offset;
	memcpy(&write_buf[1], src, bytes);
	ret = i2c_master_send(i2c, write_buf, bytes + 1);
	if (ret < 0)
		return ret;
	if (ret != (bytes + 1))
		return -EIO;

	return 0;
}

static int aic3262_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct aic3262 *aic3262;

	aic3262 = kzalloc(sizeof(struct aic3262), GFP_KERNEL);
	if (aic3262 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, aic3262);
	aic3262->dev = &i2c->dev;
	aic3262->control_data = i2c;
	aic3262->read_dev = aic3262_i2c_read_device;
	aic3262->write_dev = aic3262_i2c_write_device;
	aic3262->type = id->driver_data;
	aic3262->book_no = 255;
	aic3262->page_no = 255;

	return aic3262_device_init(aic3262, i2c->irq);
}

static int aic3262_i2c_remove(struct i2c_client *i2c)
{
	struct aic3262 *aic3262 = i2c_get_clientdata(i2c);

	aic3262_device_exit(aic3262);

	return 0;
}

static const struct i2c_device_id aic3262_i2c_id[] = {
	{ "tlv320aic3262", TLV320AIC3262 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aic3262_i2c_id);


static struct i2c_driver aic3262_i2c_driver = {
	.driver = {
		.name = "tlv320aic3262",
		.owner = THIS_MODULE,
		.pm = &aic3262_pm_ops,
	},
	.probe = aic3262_i2c_probe,
	.remove = aic3262_i2c_remove,
	.id_table = aic3262_i2c_id,
};

static int __init aic3262_i2c_init(void)
{
	int ret;
	ret = i2c_add_driver(&aic3262_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register aic3262 I2C driver: %d\n", ret);

	return ret;
}
module_init(aic3262_i2c_init);

static void __exit aic3262_i2c_exit(void)
{
	i2c_del_driver(&aic3262_i2c_driver);
}
module_exit(aic3262_i2c_exit);
#endif
#if defined(CONFIG_SPI_MASTER)
/* TODO: UGLY
 * NVidia's CS differs from what TI requires on the SPI bus. So before
 * we do any write/read we pull down the CS gpio :(
 * The problem is in spi_read.
 * Can we set the flag spi_transfer.cs_change during read so that CS is
 * pulled low until the next transaction occurs
 * (spi_read requires a spi_write followed by spi_read)
 */
#include <linux/gpio.h>
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#include <linux/delay.h>
#define SPI_CS  TEGRA_GPIO_PX3
#define CS(a)  gpio_set_value(SPI_CS, a)
void nvidia_spi_cs_en(bool stop)
{
	if (stop) {
		CS(1);
		udelay(1);
	} else {
		CS(0);
		udelay(1);
	}
	return;
}
static int aic3262_spi_read_device(struct aic3262 *aic3262, unsigned int reg,
				  int bytes, void *dest)
{
	struct spi_device *spi = aic3262->control_data;
	union aic326x_reg_union *aic_reg = (union aic326x_reg_union *) &reg;
	u8 *write_read_buf;
	unsigned int i;
	unsigned int time;
	unsigned int last_count;
	unsigned int spi_read_bufsize = max(32, SMP_CACHE_BYTES)-1;
	struct spi_message	message;
	struct spi_transfer	x[2];
	int ret;
	u8 buf[2];
	u8 page, book, offset;
	page = aic_reg->aic326x_register.page;
	book = aic_reg->aic326x_register.book;
	offset = aic_reg->aic326x_register.offset;
	if (aic3262->book_no != book) {
		/* We should change to page 0.
		Change the book by writing to offset 127 of page 0
		Change the page back to whatever was set before change page */

		buf[0] = 0x0;
		buf[1] = 0x0;

		nvidia_spi_cs_en(0);
		ret = spi_write(spi, (unsigned char *)buf, 2);
		nvidia_spi_cs_en(1);

		if (ret < 0)
			return ret;
		buf[0] = (127 << 1) ;
		buf[1] = book;

		nvidia_spi_cs_en(0);
		ret = spi_write(spi, (unsigned char *)buf, 2);
		nvidia_spi_cs_en(1);

		if (ret < 0)
			return ret;
		aic3262->book_no = book;
		aic3262->page_no = 0x0;
	}

	if (aic3262->page_no != page) {
		buf[0] = 0x0;
		buf[1] = page;

		nvidia_spi_cs_en(0);
		ret = spi_write(spi, (unsigned char *)buf, 2);
		nvidia_spi_cs_en(1);

		if (ret < 0)
			return ret;
		aic3262->page_no = page;
	}

	buf[0] = (offset << 1) | (0x01) ;
	memset(x, 0, sizeof x);
	spi_message_init(&message);
	x[0].len = 1;
	x[0].tx_buf = buf;
	x[1].len = bytes;
	x[1].rx_buf = dest ;

	spi_message_add_tail(&x[0], &message);
	spi_message_add_tail(&x[1], &message);

	nvidia_spi_cs_en(0);
	ret = spi_sync(spi, &message);
	nvidia_spi_cs_en(1);
	if (ret < 0)
		return ret;

	return bytes;

}
/* NVidia's CS differs from what TI requires on the SPI bus. So before
 * we do any write/read we pull down the CS gpio :(
 */
static int aic3262_spi_write_device(struct aic3262 *aic3262, unsigned int reg,
				   int bytes, const void *src)
{
	struct spi_device *spi = aic3262->control_data;
	int ret;

	union aic326x_reg_union *aic_reg = (union aic326x_reg_union *) &reg;

	u8 buf[2];
	u8 write_buf[bytes + 1];
	u8 page, book, offset;
	page = aic_reg->aic326x_register.page;
	book = aic_reg->aic326x_register.book;
	offset = aic_reg->aic326x_register.offset;
	if (aic3262->book_no != book) {
		/* We should change to page 0.
		Change the book by writing to offset 127 of page 0
		Change the page back to whatever was set before change page */

		buf[0] = 0x0;
		buf[1] = 0x0;

		nvidia_spi_cs_en(0);
		ret = spi_write(spi, (unsigned char *)buf, 2);
		nvidia_spi_cs_en(1);

		if (ret < 0)
			return ret;
		buf[0] = (127 << 1) ;
		buf[1] = book;

		nvidia_spi_cs_en(0);
		ret = spi_write(spi, (unsigned char *)buf, 2);
		nvidia_spi_cs_en(1);

		if (ret < 0)
			return ret;
		aic3262->book_no = book;
		aic3262->page_no = 0x0;
	}

	if (aic3262->page_no != page) {
		buf[0] = 0x0;
		buf[1] = page;
		nvidia_spi_cs_en(0);
		ret = spi_write(spi, (unsigned char *) buf, 2);
		nvidia_spi_cs_en(1);
		if (ret < 0)
			return ret;
		aic3262->page_no = page;
	}
	write_buf[0] = offset << 1 ;
	memcpy(&write_buf[1], src, bytes);
	nvidia_spi_cs_en(0);
	ret = spi_write(spi, write_buf, bytes + 1);
	nvidia_spi_cs_en(1);
	if (ret < 0)
		return ret;

	return bytes;
}

static int aic3262_spi_probe(struct spi_device *spi)
{
	struct aic3262 *aic3262;

	aic3262 = kzalloc(sizeof(struct aic3262), GFP_KERNEL);
	if (aic3262 == NULL)
		return -ENOMEM;

	spi_set_drvdata(spi, aic3262);
	aic3262->dev = &spi->dev;
	aic3262->control_data = spi;
	aic3262->read_dev = aic3262_spi_read_device;
	aic3262->write_dev = aic3262_spi_write_device;
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_1;
	spi->max_speed_hz = 4000*1000;
	spi_setup(spi);

	if (strcmp(spi->modalias, "tlv320aic3262") == 0)
		aic3262->type = TLV320AIC3262;
	aic3262->book_no = 255;
	aic3262->page_no = 255;

	return aic3262_device_init(aic3262, spi->irq);
}

static int aic3262_spi_remove(struct spi_device *spi)
{
	struct aic3262 *aic3262 = spi_get_drvdata(spi);

	aic3262_device_exit(aic3262);

	return 0;
}

static struct spi_driver aic3262_spi_driver = {
	.driver = {
		.name = "tlv320aic3262",
		.owner = THIS_MODULE,
		.pm = &aic3262_pm_ops,
	},
	.probe = aic3262_spi_probe,
	.remove = aic3262_spi_remove,
};

static int __init aic3262_spi_init(void)
{
	int ret;
	ret = spi_register_driver(&aic3262_spi_driver);
	if (ret != 0)
		pr_err("Failed to register aic3262 SPI driver: %d\n", ret);

	return ret;
}
module_init(aic3262_spi_init);

static void __exit aic3262_spi_exit(void)
{
	spi_unregister_driver(&aic3262_spi_driver);
}
module_exit(aic3262_spi_exit);
#endif

MODULE_DESCRIPTION("Core support for the TLV320AIC3262 audio CODEC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mukund Navada <navada@ti.com>");
