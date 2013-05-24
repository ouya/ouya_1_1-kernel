/*
 * Core driver for MAXIM MAX77665
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77665.h>
#include <linux/slab.h>

#define MAX77665_INT_STS	0x22
#define MAX77665_INT_MSK	0x23
#define MAX77665_PMIC_FLASH	0x00 ... 0x10
#define MAX77665_PMIC_PMIC	0x20 ... 0x2D
#define MAX77665_PMIC_CHARGER	0xB0 ... 0xC6
#define MAX77665_MUIC		0x00 ... 0x0E
#define MAX77665_HAPTIC		0x00 ... 0x10

static u8 max77665_i2c_slave_address[] = {
	[MAX77665_I2C_SLAVE_PMIC] = 0x66,
	[MAX77665_I2C_SLAVE_MUIC] = 0x25,
	[MAX77665_I2C_SLAVE_HAPTIC] = 0x48,
};

struct max77665_irq_data {
	int bit;
};

#define MAX77665_IRQ(_id, _bit_pos)				\
	[MAX77665_IRQ_##_id] =	{				\
					.bit	= (_bit_pos),	\
				}

static const struct max77665_irq_data max77665_irqs[] = {
	MAX77665_IRQ(CHARGER, 0),
	MAX77665_IRQ(TOP_SYS, 1),
	MAX77665_IRQ(FLASH, 2),
	MAX77665_IRQ(MUIC, 3),
};

static struct mfd_cell max77665s[] = {
	{.name = "max77665-charger",},
	{.name = "max77665-flash",},
	{.name = "max77665-muic",},
	{.name = "max77665-haptic",},
};

static void max77665_irq_lock(struct irq_data *data)
{
	struct max77665 *max77665 = irq_data_get_irq_chip_data(data);

	mutex_lock(&max77665->irq_lock);
}

static void max77665_irq_mask(struct irq_data *irq_data)
{
	struct max77665 *max77665 = irq_data_get_irq_chip_data(irq_data);
	unsigned int __irq = irq_data->irq - max77665->irq_base;
	const struct max77665_irq_data *data = &max77665_irqs[__irq];
	int ret;

	ret = max77665_set_bits(max77665->dev, MAX77665_I2C_SLAVE_PMIC,
				MAX77665_INT_MSK, data->bit);
	if (ret < 0)
		dev_err(max77665->dev,
			"Clearing mask reg failed e = %d\n", ret);
}

static void max77665_irq_unmask(struct irq_data *irq_data)
{
	struct max77665 *max77665 = irq_data_get_irq_chip_data(irq_data);
	unsigned int __irq = irq_data->irq - max77665->irq_base;
	const struct max77665_irq_data *data = &max77665_irqs[__irq];
	int ret;

	ret = max77665_clr_bits(max77665->dev, MAX77665_I2C_SLAVE_PMIC,
				MAX77665_INT_MSK, data->bit);
	if (ret < 0)
		dev_err(max77665->dev,
			"Setting mask reg failed e = %d\n", ret);
}

static void max77665_irq_sync_unlock(struct irq_data *data)
{
	struct max77665 *max77665 = irq_data_get_irq_chip_data(data);

	mutex_unlock(&max77665->irq_lock);
}

static irqreturn_t max77665_irq(int irq, void *data)
{
	struct max77665 *max77665 = data;
	int ret = 0;
	u8 status = 0;
	unsigned long int acks = 0;
	int i;

	ret = max77665_read(max77665->dev, MAX77665_I2C_SLAVE_PMIC,
					MAX77665_INT_STS, &status);
	if (ret < 0) {
		dev_err(max77665->dev,
				"failed to read status regi, e %d\n", ret);
		return IRQ_NONE;
	}
	acks = status;
	for_each_set_bit(i, &acks, ARRAY_SIZE(max77665_irqs))
		handle_nested_irq(max77665->irq_base + i);
	return acks ? IRQ_HANDLED : IRQ_NONE;
}

#ifdef CONFIG_PM_SLEEP
static int max77665_irq_set_wake(struct irq_data *data, unsigned int enable)
{
	struct max77665 *max77665 = irq_data_get_irq_chip_data(data);

	return irq_set_irq_wake(max77665->irq_base, enable);
}

#else
#define max77665_irq_set_wake NULL
#endif

static int __devinit max77665_irq_init(struct max77665 *max77665, int irq,
	int irq_base)
{
	int i, ret;

	if (irq_base <= 0) {
		dev_err(max77665->dev, "IRQ base not set, int not supported\n");
		return -EINVAL;
	}

	mutex_init(&max77665->irq_lock);

	ret = max77665_write(max77665->dev, MAX77665_I2C_SLAVE_PMIC,
					MAX77665_INT_MSK, 0xFF);
	if (ret < 0) {
		dev_err(max77665->dev,
			"Int mask reg write failed, e %d\n", ret);
		return ret;
	}

	max77665->irq_base = irq_base;
	max77665->irq_chip.name = "max77665";
	max77665->irq_chip.irq_mask = max77665_irq_mask;
	max77665->irq_chip.irq_unmask = max77665_irq_unmask;
	max77665->irq_chip.irq_bus_lock = max77665_irq_lock;
	max77665->irq_chip.irq_bus_sync_unlock = max77665_irq_sync_unlock;
	max77665->irq_chip.irq_set_wake = max77665_irq_set_wake;

	for (i = 0; i < ARRAY_SIZE(max77665_irqs); i++) {
		int __irq = i + max77665->irq_base;
		irq_set_chip_data(__irq, max77665);
		irq_set_chip_and_handler(__irq, &max77665->irq_chip,
					 handle_simple_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#endif
	}

	ret = request_threaded_irq(irq, NULL, max77665_irq, IRQF_ONESHOT,
				"max77665", max77665);
	if (ret < 0) {
		dev_err(max77665->dev, "Int registration failed, e %d\n", ret);
		return ret;
	}

	device_init_wakeup(max77665->dev, 1);
	return ret;
}

static bool rd_wr_reg_pmic(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77665_PMIC_FLASH:
	case MAX77665_PMIC_PMIC:
	case MAX77665_PMIC_CHARGER:
		return true;
	default:
		dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
		return false;
	}
}

static bool rd_wr_reg_muic(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77665_MUIC:
		return true;
	default:
		dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
		return false;
	}
}

static bool rd_wr_reg_haptic(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77665_HAPTIC:
		return true;
	default:
		dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
		return false;
	}
}

static const struct regmap_config max77665_regmap_config[] = {
	{
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0xFF,
		.writeable_reg = rd_wr_reg_pmic,
		.readable_reg = rd_wr_reg_pmic,
		.cache_type = REGCACHE_RBTREE,
	}, {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0x0E,
		.writeable_reg = rd_wr_reg_muic,
		.readable_reg = rd_wr_reg_muic,
		.cache_type = REGCACHE_RBTREE,
	}, {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0x10,
		.writeable_reg = rd_wr_reg_haptic,
		.readable_reg = rd_wr_reg_haptic,
		.cache_type = REGCACHE_RBTREE,
	},
};

static int __devinit max77665_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct max77665_platform_data *pdata = client->dev.platform_data;
	struct max77665 *max77665;
	struct i2c_client *slv_client;
	int ret;
	int i;

	if (!pdata) {
		dev_err(&client->dev, "max77665 requires platform data\n");
		return -EINVAL;
	}

	max77665 = devm_kzalloc(&client->dev, sizeof(*max77665), GFP_KERNEL);
	if (!max77665) {
		dev_err(&client->dev, "mem alloc for max77665 failed\n");
		return -ENOMEM;
	}

	max77665->dev = &client->dev;

	for (i = 0; i < MAX77665_I2C_SLAVE_MAX; ++i) {
		slv_client = max77665->client[i];
		if (i == 0)
			slv_client = client;
		else
			slv_client = i2c_new_dummy(client->adapter,
						max77665_i2c_slave_address[i]);
		if (!slv_client) {
			dev_err(&client->dev, "can't attach client %d\n", i);
			ret = -ENOMEM;
			goto err_exit;
		}
		i2c_set_clientdata(slv_client, max77665);

		max77665->regmap[i] = devm_regmap_init_i2c(slv_client,
					&max77665_regmap_config[i]);
		if (IS_ERR(max77665->regmap[i])) {
			ret = PTR_ERR(max77665->regmap[i]);
			dev_err(&client->dev,
				"regmap %d init failed with err: %d\n",	i, ret);
			goto err_exit;
		}
	}

	if (client->irq > 0)
		max77665_irq_init(max77665, client->irq, pdata->irq_base);

	ret = mfd_add_devices(max77665->dev, -1, max77665s,
		ARRAY_SIZE(max77665s), NULL, 0);
	if (ret) {
		dev_err(&client->dev, "add mfd devices failed with err: %d\n",
			ret);
		goto err_irq_exit;
	}

	return 0;

err_irq_exit:
	if (client->irq > 0)
		free_irq(client->irq, max77665);
err_exit:
	for (i = 0; i < MAX77665_I2C_SLAVE_MAX; ++i) {
		slv_client = max77665->client[i];
		if (slv_client && slv_client != client)
			i2c_unregister_device(slv_client);
	}
	return ret;
}

static int __devexit max77665_i2c_remove(struct i2c_client *client)
{
	struct max77665 *max77665 = i2c_get_clientdata(client);
	int i;
	struct i2c_client *slv_client;

	mfd_remove_devices(max77665->dev);
	if (client->irq > 0)
		free_irq(client->irq, max77665);

	for (i = 0; i < MAX77665_I2C_SLAVE_MAX; ++i) {
		slv_client = max77665->client[i];
		if (slv_client && slv_client != client)
			i2c_unregister_device(slv_client);
	}

	return 0;
}

static const struct i2c_device_id max77665_id_table[] = {
	{ "max77665", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max77665_id_table);

static struct i2c_driver max77665_driver = {
	.driver	= {
		.name	= "max77665",
		.owner	= THIS_MODULE,
	},
	.probe		= max77665_i2c_probe,
	.remove		= __devexit_p(max77665_i2c_remove),
	.id_table	= max77665_id_table,
};

static int __init max77665_init(void)
{
	return i2c_add_driver(&max77665_driver);
}
subsys_initcall(max77665_init);

static void __exit max77665_exit(void)
{
	i2c_del_driver(&max77665_driver);
}
module_exit(max77665_exit);

MODULE_DESCRIPTION("MAXIM MAX77665 core driver");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
