/*
 * Core driver interface for MAXIM77665
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

#ifndef __LINUX_MFD_MAX77665_H
#define __LINUX_MFD_MAX77665_H

#include <linux/irq.h>
#include <linux/regmap.h>

/* MAX77665 Interrups */
enum {
	MAX77665_IRQ_CHARGER,
	MAX77665_IRQ_TOP_SYS,
	MAX77665_IRQ_FLASH,
	MAX77665_IRQ_MUIC,
};

enum {
	MAX77665_I2C_SLAVE_PMIC,
	MAX77665_I2C_SLAVE_MUIC,
	MAX77665_I2C_SLAVE_HAPTIC,
	MAX77665_I2C_SLAVE_MAX,
};

struct max77665 {
	struct device		*dev;
	struct i2c_client	*client[MAX77665_I2C_SLAVE_MAX];
	struct regmap		*regmap[MAX77665_I2C_SLAVE_MAX];
	struct irq_chip		irq_chip;
	struct mutex		irq_lock;
	int			irq_base;
};

struct max77665_platform_data {
	int irq_base;
};

static inline int max77665_write(struct device *dev, int slv_id,
		int reg, uint8_t val)
{
	struct max77665 *maxim = dev_get_drvdata(dev);

	return regmap_write(maxim->regmap[slv_id], reg, val);
}

static inline int max77665_read(struct device *dev, int slv_id,
		int reg, uint8_t *val)
{
	struct max77665 *maxim = dev_get_drvdata(dev);
	unsigned int temp_val;
	int ret;

	ret = regmap_read(maxim->regmap[slv_id], reg, &temp_val);
	if (!ret)
		*val = temp_val;
	return ret;
}

static inline int max77665_set_bits(struct device *dev, int slv_id,
		int reg, uint8_t bit_num)
{
	struct max77665 *maxim = dev_get_drvdata(dev);

	return regmap_update_bits(maxim->regmap[slv_id],
				reg, BIT(bit_num), ~0u);
}

static inline int max77665_clr_bits(struct device *dev, int slv_id,
		int reg, uint8_t bit_num)
{
	struct max77665 *maxim = dev_get_drvdata(dev);

	return regmap_update_bits(maxim->regmap[slv_id],
				reg, BIT(bit_num), 0u);
}

#endif /*__LINUX_MFD_MAX77665_H */
