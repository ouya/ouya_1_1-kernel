/*
 * arch/arm/mach-tegra/tegra3_throttle.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <mach/thermal.h>

#include "clock.h"
#include "cpu-tegra.h"
#include "dvfs.h"

static struct mutex *cpu_throttle_lock;
static DEFINE_MUTEX(bthrot_list_lock);
static LIST_HEAD(bthrot_list);

static struct {
	const char *cap_name;
	struct clk *cap_clk;
	unsigned long cap_freq;
} cap_freqs_table[] = {
	{ .cap_name = "cap.throttle.cbus" },
	{ .cap_name = "cap.throttle.sclk" },
	{ .cap_name = "cap.throttle.emc" },
};

#define INDEX_CBUS		0
#define INDEX_SCLK		1
#define INDEX_EMC		2

#define CAP_TBL_IDX(name)	INDEX_##name
#define CAP_TBL_CAP_CLK(name)	cap_freqs_table[CAP_TBL_IDX(name)].cap_clk
#define CAP_TBL_CAP_FREQ(name)	cap_freqs_table[CAP_TBL_IDX(name)].cap_freq

static unsigned int clip_to_table(unsigned int cpu_freq)
{
	int i;
	struct cpufreq_frequency_table *cpu_freq_table;
	struct tegra_cpufreq_table_data *table_data =
		tegra_cpufreq_table_get();

	if (IS_ERR_OR_NULL(table_data))
		return -EINVAL;

	cpu_freq_table = table_data->freq_table;

	for (i = 0; cpu_freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (cpu_freq_table[i].frequency > cpu_freq)
			break;
	}
	i = (i == 0) ? 0 : i-1;
	return cpu_freq_table[i].frequency;
}

unsigned int tegra_throttle_governor_speed(unsigned int requested_speed)
{
	struct balanced_throttle *bthrot;
	unsigned int throttle_speed = requested_speed;
	unsigned int bthrot_speed;
	unsigned int lowest_speed;
	struct cpufreq_frequency_table *cpu_freq_table;
	struct tegra_cpufreq_table_data *table_data =
		tegra_cpufreq_table_get();

	if (!table_data)
		return requested_speed;


	cpu_freq_table = table_data->freq_table;
	lowest_speed = cpu_freq_table[table_data->throttle_lowest_index].frequency;

	mutex_lock(&bthrot_list_lock);

	list_for_each_entry(bthrot, &bthrot_list, node) {
		if (bthrot->cur_state) {
			bthrot_speed = bthrot->cpu_cap_freq;

			if (bthrot_speed == CPU_THROT_LOW)
				bthrot_speed = lowest_speed;
			else
				bthrot_speed = clip_to_table(bthrot_speed);

			throttle_speed = min(throttle_speed, bthrot_speed);
		}
	}
	mutex_unlock(&bthrot_list_lock);

	return throttle_speed;
}

bool tegra_is_throttling(void)
{
	struct balanced_throttle *bthrot;
	bool is_throttling = false;

	mutex_lock(&bthrot_list_lock);
	list_for_each_entry(bthrot, &bthrot_list, node) {
		if (bthrot->cur_state) {
			is_throttling = true;
			break;
		}
	}
	mutex_unlock(&bthrot_list_lock);

	return is_throttling;
}

static int
tegra_throttle_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *max_state)
{
	struct balanced_throttle *bthrot = cdev->devdata;

	*max_state = bthrot->throt_tab_size;

	return 0;
}

static int
tegra_throttle_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *cur_state)
{
	struct balanced_throttle *bthrot = cdev->devdata;

	*cur_state = bthrot->cur_state;

	return 0;
}

static void
tegra_throttle_cap_freqs_update(unsigned long cbus_rate,
				unsigned long sclk_rate,
				unsigned long emc_rate,
				int direction)
{
	if (cbus_rate == 0)
		cbus_rate = clk_get_max_rate(CAP_TBL_CAP_CLK(CBUS));
	if (sclk_rate == 0)
		sclk_rate = clk_get_max_rate(CAP_TBL_CAP_CLK(SCLK));
	if (emc_rate == 0)
		emc_rate = clk_get_max_rate(CAP_TBL_CAP_CLK(EMC));

	if (direction == 1) { /* up */
		if (CAP_TBL_CAP_FREQ(EMC) != emc_rate) {
			clk_set_rate(CAP_TBL_CAP_CLK(EMC), emc_rate);
			CAP_TBL_CAP_FREQ(EMC) = emc_rate;
		}

		if (CAP_TBL_CAP_FREQ(SCLK) != sclk_rate) {
			clk_set_rate(CAP_TBL_CAP_CLK(SCLK), sclk_rate);
			CAP_TBL_CAP_FREQ(SCLK) = sclk_rate;
		}

		if (CAP_TBL_CAP_FREQ(CBUS) != cbus_rate) {
			clk_set_rate(CAP_TBL_CAP_CLK(CBUS), cbus_rate);
			CAP_TBL_CAP_FREQ(CBUS) = cbus_rate;
		}
	} else { /* down */
		if (CAP_TBL_CAP_FREQ(CBUS) != cbus_rate) {
			clk_set_rate(CAP_TBL_CAP_CLK(CBUS), cbus_rate);
			CAP_TBL_CAP_FREQ(CBUS) = cbus_rate;
		}

		if (CAP_TBL_CAP_FREQ(SCLK) != sclk_rate) {
			clk_set_rate(CAP_TBL_CAP_CLK(SCLK), sclk_rate);
			CAP_TBL_CAP_FREQ(SCLK) = sclk_rate;
		}

		if (CAP_TBL_CAP_FREQ(EMC) != emc_rate) {
			clk_set_rate(CAP_TBL_CAP_CLK(EMC), emc_rate);
			CAP_TBL_CAP_FREQ(EMC) = emc_rate;
		}
	}
}

static int
tegra_throttle_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long cur_state)
{
	struct balanced_throttle *bthrot = cdev->devdata;
	int index;
	int direction;

	direction = bthrot->cur_state > cur_state;
	bthrot->cur_state = cur_state;

	if (cur_state == 0) {
		tegra_throttle_cap_freqs_update(0, 0, 0, 1);/* uncap freqs */

		mutex_lock(cpu_throttle_lock);
		tegra_cpu_set_speed_cap(NULL);
		mutex_unlock(cpu_throttle_lock);
	} else {
		index = cur_state - 1;
		tegra_throttle_cap_freqs_update(
				bthrot->throt_tab[index].cbus_freq * 1000UL,
				bthrot->throt_tab[index].sclk_freq * 1000UL,
				bthrot->throt_tab[index].emc_freq * 1000UL,
				direction);

		if (bthrot->cpu_cap_freq != bthrot->throt_tab[index].cpu_freq) {
			bthrot->cpu_cap_freq =
					bthrot->throt_tab[index].cpu_freq;

			mutex_lock(cpu_throttle_lock);
			tegra_cpu_set_speed_cap(NULL);
			mutex_unlock(cpu_throttle_lock);
		}
	}

	return 0;
}

static struct thermal_cooling_device_ops tegra_throttle_cooling_ops = {
	.get_max_state = tegra_throttle_get_max_state,
	.get_cur_state = tegra_throttle_get_cur_state,
	.set_cur_state = tegra_throttle_set_cur_state,
};

#ifdef CONFIG_DEBUG_FS
static int table_show(struct seq_file *s, void *data)
{
	struct balanced_throttle *bthrot = s->private;
	int i;

	for (i = 0; i < bthrot->throt_tab_size; i++)
		seq_printf(s, "[%d] = %7lu %7lu %7lu %7lu\n",
			i, bthrot->throt_tab[i].cpu_freq,
			bthrot->throt_tab[i].cbus_freq,
			bthrot->throt_tab[i].sclk_freq,
			bthrot->throt_tab[i].emc_freq);

	return 0;
}

static int table_open(struct inode *inode, struct file *file)
{
	return single_open(file, table_show, inode->i_private);
}

static ssize_t table_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct balanced_throttle *bthrot = file->private_data;
	char buf[80];
	int table_idx;
	unsigned long cpu_freq, cbus_freq, sclk_freq, emc_freq;

	if (sizeof(buf) <= count)
		return -EINVAL;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count] = '\0';
	strim(buf);

	if (sscanf(buf, "[%d] = %lu %lu %lu %lu", &table_idx,
		&cpu_freq, &cbus_freq, &sclk_freq, &emc_freq) != 5)
		return -1;

	if ((table_idx < 0) || (table_idx >= bthrot->throt_tab_size))
		return -EINVAL;

	/* round new settings before updating table */
	bthrot->throt_tab[table_idx].cpu_freq = cpu_freq;
	bthrot->throt_tab[table_idx].cbus_freq = cbus_freq;
	bthrot->throt_tab[table_idx].sclk_freq = sclk_freq;
	bthrot->throt_tab[table_idx].emc_freq = emc_freq;

	return count;
}

static const struct file_operations table_fops = {
	.open		= table_open,
	.read		= seq_read,
	.write		= table_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *throttle_debugfs_root;
#endif /* CONFIG_DEBUG_FS */


int balanced_throttle_register(struct balanced_throttle *bthrot)
{
#ifdef CONFIG_DEBUG_FS
	char name[32];
#endif
	struct balanced_throttle *dev;

	mutex_lock(&bthrot_list_lock);
	list_for_each_entry(dev, &bthrot_list, node) {
		if (dev->id == bthrot->id) {
			mutex_unlock(&bthrot_list_lock);
			return -EINVAL;
		}
	}


	list_add(&bthrot->node, &bthrot_list);
	mutex_unlock(&bthrot_list_lock);

	bthrot->cdev = thermal_cooling_device_register(
						"balanced",
						bthrot,
						&tegra_throttle_cooling_ops);

	if (IS_ERR(bthrot->cdev)) {
		bthrot->cdev = NULL;
		return -ENODEV;
	}

#ifdef CONFIG_DEBUG_FS
	sprintf(name, "throttle_table%d", bthrot->id);
	debugfs_create_file(name,0644, throttle_debugfs_root,
				bthrot, &table_fops);
#endif

	return 0;
}

int __init tegra_throttle_init(struct mutex *cpu_lock)
{
	int i;
	struct clk *c;

	cpu_throttle_lock = cpu_lock;
#ifdef CONFIG_DEBUG_FS
	throttle_debugfs_root = debugfs_create_dir("tegra_throttle", 0);
#endif

	for (i = 0; i < ARRAY_SIZE(cap_freqs_table); i++) {
		c = tegra_get_clock_by_name(cap_freqs_table[i].cap_name);
		if (!c) {
			pr_err("tegra3_throttle: cannot get clock %s\n",
				cap_freqs_table[i].cap_name);
			continue;
		}

		cap_freqs_table[i].cap_clk = c;
		cap_freqs_table[i].cap_freq = clk_get_max_rate(c);
	}
	pr_info("tegra3_throttle : init done\n");

	return 0;
}

void tegra_throttle_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(throttle_debugfs_root);
#endif
}

