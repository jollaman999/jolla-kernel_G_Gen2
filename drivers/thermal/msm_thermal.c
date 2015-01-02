/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014, Dynamic thermal control - By jollaman999
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/reboot.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/hrtimer.h>
#include <mach/cpufreq.h>
#ifdef CONFIG_MSM_MPDEC_INPUTBOOST_CPUMIN
#include "../../arch/arm/mach-msm/msm_mpdecision.h"
#endif

static DEFINE_MUTEX(emergency_shutdown_mutex);

static int enabled;

//Throttling indicator, 0=not throttled, 1=low, 2=mid, 3=max
int bricked_thermal_throttled = 0;
EXPORT_SYMBOL_GPL(bricked_thermal_throttled);

//Save the cpu max freq before throttling
static int pre_throttled_max = 0;

static struct msm_thermal_data msm_thermal_info;

static struct msm_thermal_stat msm_thermal_stats = {
    .time_low_start = 0,
    .time_mid_start = 0,
    .time_max_start = 0,
    .time_low = 0,
    .time_mid = 0,
    .time_max = 0,
};

static struct delayed_work check_temp_work;
static struct workqueue_struct *check_temp_workq;

// Dynamic thermal control - By jollaman999
/* Working with drivers/cpufreq/cpufreq.c */
bool cpufreq_max_changed = false;
bool msm_thermal_throttle_called = false;
EXPORT_SYMBOL(cpufreq_max_changed);
EXPORT_SYMBOL(msm_thermal_throttle_called);

// Dynamic thermal control - By jollaman999
/* MSM cpu freq tables */
/* See arch/arm/mach-msm/acpuclock-8064.c */
uint32_t msm_thermal_cpufreq[] = {
	81000,		// 0
	162000,		// 1
	270000,		// 2
	384000,		// 3
	486000,		// 4
	594000,		// 5
	702000,		// 6
	810000,		// 7
	918000,		// 8
	1026000,	// 9
	1134000,	// 10
	1242000,	// 11
	1350000,	// 12
	1458000,	// 13
	1512000,	// 14
#ifdef CONFIG_CPU_OVERCLOCK
	1566000,	// 15
	1620000,	// 16
	1674000,	// 17
	1728000,	// 18
	1782000,	// 19
	1836000,	// 20
	1890000,	// 21
#endif /* CONFIG_CPU_OVERCLOCK */
	0		// 22
};

// Dynamic thermal control - By jollaman999
unsigned int mid_freq_index;

// Dynamic thermal control - By jollaman999
#ifdef CONFIG_CPU_OVERCLOCK
#define MSM_THERMAL_FREQ_TABLES 22
#else
#define MSM_THERMAL_FREQ_TABLES 15
#endif /* CONFIG_CPU_OVERCLOCK */

static void update_stats(void)
{
    if (msm_thermal_stats.time_low_start > 0) {
        msm_thermal_stats.time_low += (ktime_to_ms(ktime_get()) - msm_thermal_stats.time_low_start);
        msm_thermal_stats.time_low_start = 0;
    }
    if (msm_thermal_stats.time_mid_start > 0) {
        msm_thermal_stats.time_mid += (ktime_to_ms(ktime_get()) - msm_thermal_stats.time_mid_start);
        msm_thermal_stats.time_mid_start = 0;
    }
    if (msm_thermal_stats.time_max_start > 0) {
        msm_thermal_stats.time_max += (ktime_to_ms(ktime_get()) - msm_thermal_stats.time_max_start);
        msm_thermal_stats.time_max_start = 0;
    }
}

static void start_stats(int status)
{
    switch (bricked_thermal_throttled) {
        case 1:
            msm_thermal_stats.time_low_start = ktime_to_ms(ktime_get());
            break;
        case 2:
            msm_thermal_stats.time_mid_start = ktime_to_ms(ktime_get());
            break;
        case 3:
            msm_thermal_stats.time_max_start = ktime_to_ms(ktime_get());
            break;
    }
}

static int update_cpu_max_freq(struct cpufreq_policy *cpu_policy,
                   int cpu, int max_freq)
{
    int ret = 0;

    if (!cpu_policy)
        return -EINVAL;

    cpufreq_verify_within_limits(cpu_policy, cpu_policy->min, max_freq);
    cpu_policy->user_policy.max = max_freq;

    ret = cpufreq_update_policy(cpu);
    if (!ret)
        pr_debug("msm_thermal: Setting CPU%d max frequency to %d\n",
                 cpu, max_freq);
    return ret;
}

#ifdef CONFIG_MSM_MPDEC_INPUTBOOST_CPUMIN
static int update_cpu_min_freq(struct cpufreq_policy *cpu_policy,
                               int cpu, int new_freq)
{
    int ret = 0;

    if (!cpu_policy)
        return -EINVAL;

    cpufreq_verify_within_limits(cpu_policy, new_freq, cpu_policy->max);
    cpu_policy->user_policy.min = new_freq;

    ret = cpufreq_update_policy(cpu);
    if (!ret) {
        pr_debug("msm_thermal: Setting CPU%d min frequency to %d\n",
            cpu, new_freq);
    }
    return ret;
}

DECLARE_PER_CPU(struct msm_mpdec_cpudata_t, msm_mpdec_cpudata);
#endif

// Dynamic thermal control - By jollaman999
unsigned int max_freq_finder(uint32_t msm_thermal_cpufreq[],
				uint32_t cpufreq_max,
				unsigned int low_freq_index,
				unsigned int high_freq_index)
{
	// Return index of msm_thermal_cpufreq[mid_freq_index] = cpufreq_max
	if(low_freq_index <= high_freq_index) {
		mid_freq_index = (low_freq_index + high_freq_index) / 2;

		if(cpufreq_max == msm_thermal_cpufreq[mid_freq_index]) {
			return mid_freq_index;
		} else if(cpufreq_max < msm_thermal_cpufreq[mid_freq_index]) {
			return max_freq_finder(msm_thermal_cpufreq, cpufreq_max,
						low_freq_index, mid_freq_index - 1);
		} else if(cpufreq_max > msm_thermal_cpufreq[mid_freq_index]) {
			return max_freq_finder(msm_thermal_cpufreq, cpufreq_max,
						mid_freq_index + 1, high_freq_index);
		}
	}

	return 10; // If fail to find max freq, return 10 index value.
}

// Dynamic thermal control - By jollaman999
void dynamic_thermal(void)
{
	struct cpufreq_policy *cpu_policy = NULL;
	int cpu = 0;
	unsigned int low_freq_index = 0, high_freq_index = MSM_THERMAL_FREQ_TABLES;
	unsigned int freq_find_index = 0;

	// Get current cpu policy from cpu0.
	cpu_policy = cpufreq_cpu_get(cpu);

	// Find current max cpu frequency.
	if(cpu_policy) {
		freq_find_index = max_freq_finder(msm_thermal_cpufreq, cpu_policy->max,
							low_freq_index, high_freq_index);
	} else // If fail to get cpu policy, set find index to midium freq.
		freq_find_index = (low_freq_index + high_freq_index) / 2;

	// Below 486000 is too low frequency. So ignore it.
	if(freq_find_index <= 4)
		return;

	msm_thermal_info.allowed_low_freq = msm_thermal_cpufreq[freq_find_index];

	// Adjust thermal temperatures.
	msm_thermal_info.allowed_low_low = 30 + 2 * freq_find_index;
	msm_thermal_info.allowed_low_high = msm_thermal_info.allowed_low_low + 6;

	msm_thermal_info.allowed_mid_low = msm_thermal_info.allowed_low_low + 4;
	msm_thermal_info.allowed_mid_high = msm_thermal_info.allowed_mid_low + 4;

	msm_thermal_info.allowed_max_low = msm_thermal_info.allowed_mid_low + 2;
	msm_thermal_info.allowed_max_high = msm_thermal_info.allowed_max_low + 12;

	msm_thermal_info.allowed_mid_freq = msm_thermal_cpufreq[freq_find_index-1];
	msm_thermal_info.allowed_max_freq = msm_thermal_cpufreq[freq_find_index-2];

	return;
}

static void __cpuinit check_temp(struct work_struct *work)
{
    struct cpufreq_policy *cpu_policy = NULL;
    struct tsens_device tsens_dev;
    unsigned long temp = 0;
    uint32_t max_freq = 0;
    bool update_policy = false;
    int i = 0, cpu = 0, ret = 0;

    tsens_dev.sensor_num = msm_thermal_info.sensor_id;
    ret = tsens_get_temp(&tsens_dev, &temp);
    if (ret) {
        pr_err("msm_thermal: FATAL: Unable to read TSENS sensor %d\n",
               tsens_dev.sensor_num);
        goto reschedule;
    }

    if (temp >= msm_thermal_info.shutdown_temp) {
        mutex_lock(&emergency_shutdown_mutex);
        pr_warn("################################\n");
        pr_warn("################################\n");
        pr_warn("- %u OVERTEMP! SHUTTING DOWN! -\n", msm_thermal_info.shutdown_temp);
        pr_warn("- cur temp:%lu measured by:%u -\n", temp, msm_thermal_info.sensor_id);
        pr_warn("################################\n");
        pr_warn("################################\n");
        /* orderly poweroff tries to power down gracefully
           if it fails it will force it. */
        orderly_poweroff(true);
        for_each_possible_cpu(cpu) {
            update_policy = true;
            max_freq = msm_thermal_info.allowed_max_freq;
            bricked_thermal_throttled = 3;
            pr_warn("msm_thermal: Emergency throttled CPU%i to %u! temp:%lu\n",
                    cpu, msm_thermal_info.allowed_max_freq, temp);
        }
        mutex_unlock(&emergency_shutdown_mutex);
    }

    for_each_possible_cpu(cpu) {
        update_policy = false;
        cpu_policy = cpufreq_cpu_get(cpu);
        if (!cpu_policy) {
            pr_debug("msm_thermal: NULL policy on cpu %d\n", cpu);
            continue;
        }

        /* save pre-throttled max freq value */
        if ((bricked_thermal_throttled == 0) && (cpu == 0))
            pre_throttled_max = cpu_policy->max;

	// Dynamic thermal control - By jollaman999
	if(msm_thermal_info.dynamic_thermal_control && cpufreq_max_changed)
		dynamic_thermal();

        //low trip point
        if ((temp >= msm_thermal_info.allowed_low_high) &&
            (temp < msm_thermal_info.allowed_mid_high) &&
            (bricked_thermal_throttled < 1)) {
            msm_thermal_throttle_called = true; // Dynamic thermal control - By jollaman999
            update_policy = true;
            max_freq = msm_thermal_info.allowed_low_freq;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                bricked_thermal_throttled = 1;
                pr_warn("msm_thermal: Thermal Throttled (low)! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        //low clr point
        } else if ((temp < msm_thermal_info.allowed_low_low) &&
               (bricked_thermal_throttled > 0)) {
            msm_thermal_throttle_called = true; // Dynamic thermal control - By jollaman999
            if (pre_throttled_max != 0)
                max_freq = pre_throttled_max;
            else {
                max_freq = CONFIG_MSM_CPU_FREQ_MAX;
                pr_warn("msm_thermal: ERROR! pre_throttled_max=0, falling back to %u\n", max_freq);
            }
            update_policy = true;
            for (i = 1; i < CONFIG_NR_CPUS; i++) {
                if (cpu_online(i))
                        continue;
                cpu_up(i);
            }
            if (cpu == (CONFIG_NR_CPUS-1)) {
                bricked_thermal_throttled = 0;
                pr_warn("msm_thermal: Low thermal throttle ended! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }

#ifdef CONFIG_MSM_MPDEC_INPUTBOOST_CPUMIN
            if (cpu_online(cpu)) {
                if (mutex_trylock(&per_cpu(msm_mpdec_cpudata, cpu).unboost_mutex)) {
                    per_cpu(msm_mpdec_cpudata, cpu).is_boosted = false;
                    update_cpu_min_freq(cpu_policy, cpu, per_cpu(msm_mpdec_cpudata, cpu).norm_min_freq);
                    mutex_unlock(&per_cpu(msm_mpdec_cpudata, cpu).unboost_mutex);
                }
            }
#endif
        //mid trip point
        } else if ((temp >= msm_thermal_info.allowed_mid_high) &&
               (temp < msm_thermal_info.allowed_max_high) &&
               (bricked_thermal_throttled < 2)) {
            msm_thermal_throttle_called = true; // Dynamic thermal control - By jollaman999
            update_policy = true;
            max_freq = msm_thermal_info.allowed_mid_freq;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                bricked_thermal_throttled = 2;
                pr_warn("msm_thermal: Thermal Throttled (mid)! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        //mid clr point
        } else if ((temp < msm_thermal_info.allowed_mid_low) &&
               (bricked_thermal_throttled > 1)) {
            msm_thermal_throttle_called = true; // Dynamic thermal control - By jollaman999
            max_freq = msm_thermal_info.allowed_low_freq;
            update_policy = true;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                bricked_thermal_throttled = 1;
                pr_warn("msm_thermal: Mid thermal throttle ended! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        //max trip point
        } else if (temp >= msm_thermal_info.allowed_max_high) {
            msm_thermal_throttle_called = true; // Dynamic thermal control - By jollaman999
            update_policy = true;
            max_freq = msm_thermal_info.allowed_max_freq;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                bricked_thermal_throttled = 3;
                pr_warn("msm_thermal: Thermal Throttled (max)! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        //max clr point
        } else if ((temp < msm_thermal_info.allowed_max_low) &&
               (bricked_thermal_throttled > 2)) {
            msm_thermal_throttle_called = true; // Dynamic thermal control - By jollaman999
            max_freq = msm_thermal_info.allowed_mid_freq;
            update_policy = true;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                bricked_thermal_throttled = 2;
                pr_warn("msm_thermal: Max thermal throttle ended! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        } else {
            msm_thermal_throttle_called = false; // Dynamic thermal control - By jollaman999
        }
        update_stats();
        start_stats(bricked_thermal_throttled);
        if (update_policy)
            update_cpu_max_freq(cpu_policy, cpu, max_freq);

        cpufreq_cpu_put(cpu_policy);
    }

reschedule:
    if (enabled)
        queue_delayed_work(check_temp_workq, &check_temp_work,
                           msecs_to_jiffies(msm_thermal_info.poll_ms));

    return;
}

static void disable_msm_thermal(void)
{
    int cpu = 0;
    struct cpufreq_policy *cpu_policy = NULL;

     enabled = 0;
    /* make sure check_temp is no longer running */
    cancel_delayed_work(&check_temp_work);
    flush_scheduled_work();

    if (pre_throttled_max != 0) {
        for_each_possible_cpu(cpu) {
            cpu_policy = cpufreq_cpu_get(cpu);
            if (cpu_policy) {
                if (cpu_policy->max < cpu_policy->cpuinfo.max_freq)
                    update_cpu_max_freq(cpu_policy, cpu, pre_throttled_max);
                cpufreq_cpu_put(cpu_policy);
            }
        }
    }

   pr_warn("msm_thermal: Warning! Thermal guard disabled!");
}

static void enable_msm_thermal(void)
{
    enabled = 1;
    /* make sure check_temp is running */
    queue_delayed_work(check_temp_workq, &check_temp_work,
                       msecs_to_jiffies(msm_thermal_info.poll_ms));

    pr_info("msm_thermal: Thermal guard enabled.");
}

static int set_enabled(const char *val, const struct kernel_param *kp)
{
    int ret = 0;

    ret = param_set_bool(val, kp);
    if (!enabled)
        disable_msm_thermal();
    else if (enabled == 1)
        enable_msm_thermal();
    else
        pr_info("msm_thermal: no action for enabled = %d\n", enabled);

    pr_info("msm_thermal: enabled = %d\n", enabled);

    return ret;
}

static struct kernel_param_ops module_ops = {
    .set = set_enabled,
    .get = param_get_bool,
};

module_param_cb(enabled, &module_ops, &enabled, 0644);
MODULE_PARM_DESC(enabled, "enforce thermal limit on cpu");

/**************************** SYSFS START ****************************/
struct kobject *msm_thermal_kobject;

#define show_one(file_name, object)                             \
static ssize_t show_##file_name                                 \
(struct kobject *kobj, struct attribute *attr, char *buf)       \
{                                                               \
    return sprintf(buf, "%u\n", msm_thermal_info.object);       \
}

show_one(shutdown_temp, shutdown_temp);
show_one(allowed_max_high, allowed_max_high);
show_one(allowed_max_low, allowed_max_low);
show_one(allowed_max_freq, allowed_max_freq);
show_one(allowed_mid_high, allowed_mid_high);
show_one(allowed_mid_low, allowed_mid_low);
show_one(allowed_mid_freq, allowed_mid_freq);
show_one(allowed_low_high, allowed_low_high);
show_one(allowed_low_low, allowed_low_low);
show_one(allowed_low_freq, allowed_low_freq);
show_one(poll_ms, poll_ms);
// Dynamic thermal control - By jollaman999
show_one(dynamic_thermal_control, dynamic_thermal_control);

static ssize_t store_shutdown_temp(struct kobject *a, struct attribute *b,
                                   const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.shutdown_temp = input;

    return count;
}

static ssize_t store_allowed_max_high(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_max_high = input;

    return count;
}

static ssize_t store_allowed_max_low(struct kobject *a, struct attribute *b,
                                     const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_max_low = input;

    return count;
}

static ssize_t store_allowed_max_freq(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_max_freq = input;

    return count;
}

static ssize_t store_allowed_mid_high(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_mid_high = input;

    return count;
}

static ssize_t store_allowed_mid_low(struct kobject *a, struct attribute *b,
                                     const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_mid_low = input;

    return count;
}

static ssize_t store_allowed_mid_freq(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_mid_freq = input;

    return count;
}

static ssize_t store_allowed_low_high(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_low_high = input;

    return count;
}

static ssize_t store_allowed_low_low(struct kobject *a, struct attribute *b,
                                     const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_low_low = input;

    return count;
}

static ssize_t store_allowed_low_freq(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_low_freq = input;

    return count;
}

static ssize_t store_poll_ms(struct kobject *a, struct attribute *b,
                             const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.poll_ms = input;

    return count;
}

// Dynamic thermal control - By jollaman999
static ssize_t store_dynamic_thermal_control(struct kobject *a, struct attribute *b,
                             const char *buf, size_t count)
{
    if (buf[0] == '0' && buf[1] == '\n')
        msm_thermal_info.dynamic_thermal_control = 0;
    else if (buf[0] == '1' && buf[1] == '\n')
        msm_thermal_info.dynamic_thermal_control = 1;

    return count;
}

define_one_global_rw(shutdown_temp);
define_one_global_rw(allowed_max_high);
define_one_global_rw(allowed_max_low);
define_one_global_rw(allowed_max_freq);
define_one_global_rw(allowed_mid_high);
define_one_global_rw(allowed_mid_low);
define_one_global_rw(allowed_mid_freq);
define_one_global_rw(allowed_low_high);
define_one_global_rw(allowed_low_low);
define_one_global_rw(allowed_low_freq);
define_one_global_rw(poll_ms);
// Dynamic thermal control - By jollaman999
define_one_global_rw(dynamic_thermal_control);

static struct attribute *msm_thermal_attributes[] = {
    &shutdown_temp.attr,
    &allowed_max_high.attr,
    &allowed_max_low.attr,
    &allowed_max_freq.attr,
    &allowed_mid_high.attr,
    &allowed_mid_low.attr,
    &allowed_mid_freq.attr,
    &allowed_low_high.attr,
    &allowed_low_low.attr,
    &allowed_low_freq.attr,
    &poll_ms.attr,
    // Dynamic thermal control - By jollaman999
    &dynamic_thermal_control.attr,
    NULL
};


static struct attribute_group msm_thermal_attr_group = {
    .attrs = msm_thermal_attributes,
    .name = "conf",
};

/********* STATS START *********/

static ssize_t show_throttle_times(struct kobject *a, struct attribute *b,
                                 char *buf)
{
    ssize_t len = 0;

    if (bricked_thermal_throttled == 1) {
        len += sprintf(buf + len, "%s %llu\n", "low",
                       (msm_thermal_stats.time_low +
                        (ktime_to_ms(ktime_get()) -
                         msm_thermal_stats.time_low_start)));
    } else
        len += sprintf(buf + len, "%s %llu\n", "low", msm_thermal_stats.time_low);

    if (bricked_thermal_throttled == 2) {
        len += sprintf(buf + len, "%s %llu\n", "mid",
                       (msm_thermal_stats.time_mid +
                        (ktime_to_ms(ktime_get()) -
                         msm_thermal_stats.time_mid_start)));
    } else
        len += sprintf(buf + len, "%s %llu\n", "mid", msm_thermal_stats.time_mid);

    if (bricked_thermal_throttled == 3) {
        len += sprintf(buf + len, "%s %llu\n", "max",
                       (msm_thermal_stats.time_max +
                        (ktime_to_ms(ktime_get()) -
                         msm_thermal_stats.time_max_start)));
    } else
        len += sprintf(buf + len, "%s %llu\n", "max", msm_thermal_stats.time_max);

    return len;
}
define_one_global_ro(throttle_times);

static ssize_t show_is_throttled(struct kobject *a, struct attribute *b,
                                 char *buf)
{
    return sprintf(buf, "%u\n", bricked_thermal_throttled);
}
define_one_global_ro(is_throttled);

static struct attribute *msm_thermal_stats_attributes[] = {
    &is_throttled.attr,
    &throttle_times.attr,
    NULL
};


static struct attribute_group msm_thermal_stats_attr_group = {
    .attrs = msm_thermal_stats_attributes,
    .name = "stats",
};
/**************************** SYSFS END ****************************/

int __devinit msm_thermal_init(struct msm_thermal_data *pdata)
{
    int ret = 0, rc = 0;

    BUG_ON(!pdata);
    BUG_ON(pdata->sensor_id >= TSENS_MAX_SENSORS);
    memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

    enabled = 1;
    check_temp_workq=alloc_workqueue("msm_thermal", WQ_UNBOUND | WQ_RESCUER, 1);
    if (!check_temp_workq)
        BUG_ON(ENOMEM);
    INIT_DELAYED_WORK(&check_temp_work, check_temp);
    queue_delayed_work(check_temp_workq, &check_temp_work, 0);

    msm_thermal_kobject = kobject_create_and_add("msm_thermal", kernel_kobj);
    if (msm_thermal_kobject) {
        rc = sysfs_create_group(msm_thermal_kobject, &msm_thermal_attr_group);
        if (rc) {
            pr_warn("msm_thermal: sysfs: ERROR, could not create sysfs group");
        }
        rc = sysfs_create_group(msm_thermal_kobject,
                                &msm_thermal_stats_attr_group);
        if (rc) {
            pr_warn("msm_thermal: sysfs: ERROR, could not create sysfs stats group");
        }
    } else
        pr_warn("msm_thermal: sysfs: ERROR, could not create sysfs kobj");

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

	key = "qcom,poll-ms";
	ret = of_property_read_u32(node, key, &data.poll_ms);
	if (ret)
		goto fail;

	// Dynamic thermal control - By jollaman999
	key = "qcom,dynamic_thermal_control";
	ret = of_property_read_u32(node, key, &data.dynamic_thermal_control);
	if (ret)
		goto fail;

	key = "qcom,shutdown_temp";
	ret = of_property_read_u32(node, key, &data.shutdown_temp);
	if (ret)
		goto fail;

	key = "qcom,allowed_max_high";
	ret = of_property_read_u32(node, key, &data.allowed_max_high);
	if (ret)
		goto fail;
	key = "qcom,allowed_max_low";
	ret = of_property_read_u32(node, key, &data.allowed_max_low);
	if (ret)
		goto fail;
	key = "qcom,allowed_max_freq";
	ret = of_property_read_u32(node, key, &data.allowed_max_freq);
	if (ret)
		goto fail;

	key = "qcom,allowed_mid_high";
	ret = of_property_read_u32(node, key, &data.allowed_mid_high);
	if (ret)
		goto fail;
	key = "qcom,allowed_mid_low";
	ret = of_property_read_u32(node, key, &data.allowed_mid_low);
	if (ret)
		goto fail;
	key = "qcom,allowed_mid_freq";
	ret = of_property_read_u32(node, key, &data.allowed_mid_freq);
	if (ret)
		goto fail;

	key = "qcom,allowed_low_high";
	ret = of_property_read_u32(node, key, &data.allowed_low_high);
	if (ret)
		goto fail;
	key = "qcom,allowed_low_low";
	ret = of_property_read_u32(node, key, &data.allowed_low_low);
	if (ret)
		goto fail;
	key = "qcom,allowed_low_freq";
	ret = of_property_read_u32(node, key, &data.allowed_low_freq);
	if (ret)
		goto fail;

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

fs_initcall(msm_thermal_device_init);

