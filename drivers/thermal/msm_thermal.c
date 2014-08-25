/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * 2014, Modified for jolla-kernel by jollaman999
 * Based on @myfluxy n4 kernel
 * Referenced : @motley-git 'msm thermal: enhancements' from CallMeAldy/AK-Mako
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <mach/cpufreq.h>

/*
 * ==Tuneables==
 * - DEFAULT_THROTTLE_TEMP : default throttle temp at boot time
 * (Throttling is begin with this temp. Temperature showing is slow in TricksterMod.
 *  So check real current temp by typing 'watch -n 1 dmesg -c | grep msm_thermal'
 *  in terminal or adb shell with root permission.)
 *
 * - MAX_THROTTLE_TEMP : max able to be set by user
 *   (TricksterMod interface only support from 60 to 80)
 *
 * - DEFAULT_MIN_FREQ_INDEX : frequency table index for the lowest frequency
 *                          to drop to during throttling.
 * (You can set by '/sys/module/msm_thermal/parameters/min_freq_index' manually,
 *   but TricksterMod interface only show from 5 to 9)
 *
 * (The throtting is not applied to all cpus at one time. It is choiced by
 *  'for_each_possible_cpu' fuction. So it dosen't matter while freq index is
 *  too low when overclocked. If you want more performance, incrase
 *  DEFAULT_THROTTLE_TEMP or just turn off throttling.)
 */

#define DEFAULT_THROTTLE_TEMP		60
#define MAX_THROTTLE_TEMP		80
#define DEFAULT_MIN_FREQ_INDEX		9  // 918000 (acpuclock-8064.c)

static int enabled;
static uint32_t limited_max_freq = MSM_CPUFREQ_NO_LIMIT;

static unsigned int polling = HZ*2;
static unsigned int cpu = 0;
static unsigned int limit_idx;
static unsigned int min_freq_index;
static unsigned int throttle_temp = DEFAULT_THROTTLE_TEMP;

static uint32_t freq_max;
static uint32_t freq_buffer;

static struct msm_thermal_data msm_thermal_info;
static struct delayed_work check_temp_work;
static struct cpufreq_frequency_table *table;

static int msm_thermal_get_freq_table(void)
{
	int ret = 0;
	int i = 0;

	table = cpufreq_frequency_get_table(0);
	if (table == NULL) {
		pr_debug("%s: error reading cpufreq table\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	while (table[i].frequency != CPUFREQ_TABLE_END)
		i++;

	min_freq_index = DEFAULT_MIN_FREQ_INDEX - 1;
	limit_idx = i - 1;
	BUG_ON(limit_idx <= 0 || limit_idx <= min_freq_index);
fail:
	return ret;
}

static int update_cpu_max_freq(int cpu, uint32_t max_freq)
{
	int ret = 0;

	ret = msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, max_freq);
	if (ret)
		return ret;

	limited_max_freq = max_freq;

	ret = cpufreq_update_policy(cpu);

	return ret;
}

static void check_temp(struct work_struct *work)
{
	unsigned long temp = 0;
	struct tsens_device tsens_dev;

	if (!limit_idx)
		msm_thermal_get_freq_table();

	freq_max = table[limit_idx].frequency;

	if (freq_buffer == 0)
		freq_buffer = freq_max;

	tsens_dev.sensor_num = msm_thermal_info.sensor_id;
	tsens_get_temp(&tsens_dev, &temp);

	if (temp > throttle_temp + 8) {
		freq_max = table[min_freq_index].frequency;
		polling = HZ/8;

	} else if (temp > throttle_temp + 6) {
		freq_max = table[min_freq_index].frequency;
		polling = HZ/6;

	} else if (temp > throttle_temp + 4) {
		freq_max = table[min_freq_index + 1].frequency;
		polling = HZ/4;

	} else if (temp > throttle_temp + 2) {
		freq_max = table[min_freq_index + 2].frequency;
		polling = HZ/2;

	} else if (temp > throttle_temp) {
		polling = HZ;

	} else {
		polling = HZ*2;
	}

	if (freq_buffer != freq_max) {
		freq_buffer = freq_max;
		for_each_possible_cpu(cpu)
			msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, freq_max);
		pr_info("msm_thermal: CPU temp: %luC, max: %dMHz, polling: %dms",
			temp, freq_max/1000, jiffies_to_msecs(polling));
	}

	if (enabled)
		schedule_delayed_work(&check_temp_work, polling);
}

static void disable_msm_thermal(void)
{
	int cpu = 0;

	/* make sure check_temp is no longer running */
	cancel_delayed_work(&check_temp_work);
	flush_scheduled_work();

	if (limited_max_freq == MSM_CPUFREQ_NO_LIMIT)
		return;

	for_each_possible_cpu(cpu) {
		update_cpu_max_freq(cpu, MSM_CPUFREQ_NO_LIMIT);
	}
}
 
static int set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	ret = param_set_bool(val, kp);
	if (!enabled)
		disable_msm_thermal();
	else
		pr_info("msm_thermal: no action for enabled = %d\n", enabled);

	pr_info("msm_thermal: enabled = %d\n", enabled);

	return ret;
}

static int set_throttle_temp(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	long num;

	if (!val)
		return -EINVAL;

	ret = strict_strtol(val, 0, &num);
	if (ret == -EINVAL || num > MAX_THROTTLE_TEMP || num < 60)
		return -EINVAL;

	ret = param_set_int(val, kp);

	pr_info("msm_thermal: throttle_temp = %d\n", throttle_temp);

	return ret;
}

static int set_min_freq_index(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	long num;

	if (!val)
		return -EINVAL;

	ret = strict_strtol(val, 0, &num);
	if (ret == -EINVAL || num > limit_idx || num < 4)
		return -EINVAL;

	ret = param_set_int(val, kp);

	pr_info("msm_thermal: min_freq_index = %d\n", min_freq_index);

	return ret;
}

static struct kernel_param_ops module_ops_enabled = {
	.set = set_enabled,
	.get = param_get_bool,
};

static struct kernel_param_ops module_ops_thermal_temp = {
	.set = set_throttle_temp,
	.get = param_get_uint,
};

static struct kernel_param_ops module_ops_min_freq_index = {
	.set = set_min_freq_index,
	.get = param_get_uint,
};

module_param_cb(enabled, &module_ops_enabled, &enabled, 0775);
MODULE_PARM_DESC(enabled, "msm_thermal enforce limit on cpu (Y/N)");

module_param_cb(throttle_temp, &module_ops_thermal_temp, &throttle_temp, 0775);
MODULE_PARM_DESC(throttle_temp, "msm_thermal throttle temperature (C)");

module_param_cb(min_freq_index, &module_ops_min_freq_index, &min_freq_index, 0775);
MODULE_PARM_DESC(min_freq_index, "msm_thermal minimum throttle frequency index");

int __devinit msm_thermal_init(struct msm_thermal_data *pdata)
{
	int ret = 0;

	BUG_ON(!pdata);
	BUG_ON(pdata->sensor_id >= TSENS_MAX_SENSORS);
	memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

	pr_info("msm_thermal: throttle_temp: %dC", throttle_temp);

 	enabled = 1;
	INIT_DELAYED_WORK(&check_temp_work, check_temp);
	schedule_delayed_work(&check_temp_work, HZ*20);

	return ret;
}

static int __devinit msm_thermal_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	char *key = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct msm_thermal_data data;

	memset(&data, 0, sizeof(struct msm_thermal_data));
	key = "qcom,sensor-id";
	ret = of_property_read_u32(node, key, &data.sensor_id);
	if (ret)
		goto fail;
	WARN_ON(data.sensor_id >= TSENS_MAX_SENSORS);

fail:
	if (ret)
		pr_err("%s: Failed reading node=%s, key=%s\n",
		       __func__, node->full_name, key);
	else
		ret = msm_thermal_init(&data);

	return ret;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,msm-thermal"},
	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
};

int __init msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}
