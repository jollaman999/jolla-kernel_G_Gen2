/*
 * drivers/input/touchscreen/doubletap2wake.c
 *
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/input/doubletap2wake.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
/*
#ifndef CONFIG_HAS_EARLYSUSPEND
#include <linux/lcd_notify.h>
#else
#include <linux/earlysuspend.h>
#endif
*/
#include <linux/hrtimer.h>
#include <asm-generic/cputime.h>

// dt2w: Tuneable touch screen off voltage - by jollaman999
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
#include <linux/earlysuspend.h>
#include <linux/regulator/consumer.h>
#endif

/* uncomment since no touchscreen defines android touch, do that here */
//#define ANDROID_TOUCH_DECLARED

/* if Sweep2Wake is compiled it will already have taken care of this */
#ifdef CONFIG_TOUCHSCREEN_SWEEP2WAKE
#define ANDROID_TOUCH_DECLARED
#endif

/* Version, author, desc, etc */
#define DRIVER_AUTHOR "Dennis Rassmann <showp1984@gmail.com>"
#define DRIVER_DESCRIPTION "Doubletap2wake for almost any device"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[doubletap2wake]: "

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

/* Tuneables */
#define DT2W_DEBUG		0
#define DT2W_DEFAULT		1

#define DT2W_PWRKEY_DUR		0
#define DT2W_FEATHER		200
#define DT2W_TIME		200

// dt2w: Tuneable touch screen off voltage - by jollaman999
/* Original values are here
   arch/arm/mach-msm/lge/mako/board-mako-input.c
   static struct touch_power_module touch_pwr */
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
#define DT2W_DEFAULT_SCREEN_OFF_VDD	2725000
#endif

/* Resources */
int dt2w_switch = DT2W_DEFAULT;
static cputime64_t tap_time_pre = 0;
static int touch_x = 0, touch_y = 0, touch_nr = 0, x_pre = 0, y_pre = 0;
static bool touch_x_called = false, touch_y_called = false, touch_cnt = true;
static bool scr_suspended = false, exec_count = true;

// dt2w: Tuneable touch screen off voltage - by jollaman999
/* Touch Screen Voltages */
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
int screen_off_vdd;
bool synaptics_t1320_volatage_change_called;
EXPORT_SYMBOL(synaptics_t1320_volatage_change_called);
#endif

// To prevent doubletap2wake 3 taps issue when suspended. - by jollaman999
bool dt2w_suspend_enter;
cputime64_t dt2w_suspend_exit_time;
EXPORT_SYMBOL(dt2w_suspend_enter);
EXPORT_SYMBOL(dt2w_suspend_exit_time);

#ifndef CONFIG_HAS_EARLYSUSPEND
static struct notifier_block dt2w_lcd_notif;
#endif

static struct input_dev * doubletap2wake_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static struct workqueue_struct *dt2w_input_wq;
static struct work_struct dt2w_input_work;

/* Read cmdline for dt2w */
static int __init read_dt2w_cmdline(char *dt2w)
{
	if (strcmp(dt2w, "1") == 0) {
		pr_info("[cmdline_dt2w]: DoubleTap2Wake enabled. | dt2w='%s'\n", dt2w);
		dt2w_switch = 1;
	} else if (strcmp(dt2w, "0") == 0) {
		pr_info("[cmdline_dt2w]: DoubleTap2Wake disabled. | dt2w='%s'\n", dt2w);
		dt2w_switch = 0;
	} else {
		pr_info("[cmdline_dt2w]: No valid input found. Going with default: | dt2w='%u'\n", dt2w_switch);
	}
	return 1;
}
__setup("dt2w=", read_dt2w_cmdline);

/* reset on finger release */
static void doubletap2wake_reset(void) {
	exec_count = true;
	touch_nr = 0;
	tap_time_pre = 0;
	x_pre = 0;
	y_pre = 0;
}

/* PowerKey work func */
static void doubletap2wake_presspwr(struct work_struct * doubletap2wake_presspwr_work) {
	if (!mutex_trylock(&pwrkeyworklock))
                return;
	input_event(doubletap2wake_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	msleep(DT2W_PWRKEY_DUR);
	input_event(doubletap2wake_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
	msleep(DT2W_PWRKEY_DUR);
        mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(doubletap2wake_presspwr_work, doubletap2wake_presspwr);

/* PowerKey trigger */
static void doubletap2wake_pwrtrigger(void) {
	schedule_work(&doubletap2wake_presspwr_work);
        return;
}

/* unsigned */
static unsigned int calc_feather(int coord, int prev_coord) {
	int calc_coord = 0;
	calc_coord = coord-prev_coord;
	if (calc_coord < 0)
		calc_coord = calc_coord * (-1);
	return calc_coord;
}

/* init a new touch */
static void new_touch(int x, int y) {
	tap_time_pre = ktime_to_ms(ktime_get());
	x_pre = x;
	y_pre = y;
	touch_nr++;
}

/* Doubletap2wake main function */
static void detect_doubletap2wake(int x, int y, bool st)
{
        bool single_touch = st;
#if DT2W_DEBUG
        pr_info(LOGTAG"x,y(%4d,%4d) single:%s\n",
                x, y, (single_touch) ? "true" : "false");
#endif
	if ((single_touch) && (dt2w_switch > 0) && (exec_count) && (touch_cnt)) {
		touch_cnt = false;
		// Make enable to set touch counts (Max : 10) - by jollaman999
		if (touch_nr == dt2w_switch - 1) {
			new_touch(x, y);
			// To prevent doubletap2wake 3 taps issue when suspended. - by jollaman999
			if(dt2w_suspend_enter) {
#if DT2W_DEBUG
				pr_info("[jolla-dt2w_debug] doubletap2wake 3 taps solution time check = %lld\n",
					(ktime_to_ms(ktime_get())-dt2w_suspend_exit_time));
#endif
				// Make enable to set touch counts (Max : 10) - by jollaman999
				if((ktime_to_ms(ktime_get())-dt2w_suspend_exit_time) < (DT2W_TIME/2*(dt2w_switch+1))) {
					touch_nr++;
#if DT2W_DEBUG
					pr_info("[jolla-dt2w_debug] touch_nr++ by doubletap2wake 3 taps solution\n");
#endif
				}
			}
		// Make enable to set touch counts (Max : 10) - by jollaman999
		} else if (touch_nr >= 1 && touch_nr <= dt2w_switch) {
			if ((calc_feather(x, x_pre) < DT2W_FEATHER) &&
			    (calc_feather(y, y_pre) < DT2W_FEATHER) &&
			    // Make enable to set touch counts (Max : 10) - by jollaman999
			    ((ktime_to_ms(ktime_get())-tap_time_pre) < (DT2W_TIME/2*(dt2w_switch+1)))) {
				touch_nr++;
#if DT2W_DEBUG
				pr_info("[jolla-dt2w_debug] touch_nr++\n");
				pr_info("[jolla-dt2w_debug] touch_nr = %d\n", touch_nr);
#endif
			} else {
				doubletap2wake_reset();
				new_touch(x, y);
#if DT2W_DEBUG
				pr_info("[jolla-dt2w_debug] dt2w reseted!!\n");
				pr_info("[jolla-dt2w_debug] touch_nr = %d\n", touch_nr);
#endif
			}
		} else {
			doubletap2wake_reset();
			new_touch(x, y);
		}
		// Make enable to set touch counts (Max : 10) - by jollaman999
		if ((touch_nr > dt2w_switch)) {
			pr_info(LOGTAG"ON\n");
			exec_count = false;
			doubletap2wake_pwrtrigger();
			doubletap2wake_reset();
		}
	}
	// To prevent doubletap2wake 3 taps issue when suspended. - by jollaman999
	dt2w_suspend_enter = false;
}

static void dt2w_input_callback(struct work_struct *unused) {

	detect_doubletap2wake(touch_x, touch_y, true);

	return;
}

static void dt2w_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {
#if DT2W_DEBUG
	pr_info("doubletap2wake: code: %s|%u, val: %i\n",
		((code==ABS_MT_POSITION_X) ? "X" :
		(code==ABS_MT_POSITION_Y) ? "Y" :
		(code==ABS_MT_TRACKING_ID) ? "ID" :
		"undef"), code, value);
#endif
	if (!scr_suspended)
		return;

	if (code == ABS_MT_SLOT) {
		doubletap2wake_reset();
		return;
	}

	if (code == ABS_MT_TRACKING_ID && value == -1) {
		touch_cnt = true;
		return;
	}

	if (code == ABS_MT_POSITION_X) {
		touch_x = value;
		touch_x_called = true;
	}

	if (code == ABS_MT_POSITION_Y) {
		touch_y = value;
		touch_y_called = true;
	}

	if (touch_x_called || touch_y_called) {
		touch_x_called = false;
		touch_y_called = false;
		queue_work_on(0, dt2w_input_wq, &dt2w_input_work);
	}
}

static int input_dev_filter(struct input_dev *dev) {
	if (strstr(dev->name, "touch")) {
		return 0;
	} else {
		return 1;
	}
}

static int dt2w_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "dt2w";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dt2w_input_disconnect(struct input_handle *handle) {
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dt2w_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler dt2w_input_handler = {
	.event		= dt2w_input_event,
	.connect	= dt2w_input_connect,
	.disconnect	= dt2w_input_disconnect,
	.name		= "dt2w_inputreq",
	.id_table	= dt2w_ids,
};

#ifndef CONFIG_HAS_EARLYSUSPEND
static int lcd_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	switch (event) {
	case LCD_EVENT_ON_END:
		scr_suspended = false;
		break;
	case LCD_EVENT_OFF_END:
		scr_suspended = true;
		break;
	default:
		break;
	}

	return 0;
}
#else
static void dt2w_early_suspend(struct early_suspend *h) {
	scr_suspended = true;
}

static void dt2w_late_resume(struct early_suspend *h) {
	scr_suspended = false;
}

static struct early_suspend dt2w_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = dt2w_early_suspend,
	.resume = dt2w_late_resume,
};
#endif

/*
 * SYSFS stuff below here
 */
static ssize_t dt2w_doubletap2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", dt2w_switch);

	return count;
}

static ssize_t dt2w_doubletap2wake_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	// Make enable to set touch counts (Max : 10) - by jollaman999
	// You should tap 1 more from set number to wake your device.
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
                if (dt2w_switch != buf[0] - '0')
		        dt2w_switch = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(doubletap2wake, (S_IWUSR|S_IRUGO),
	dt2w_doubletap2wake_show, dt2w_doubletap2wake_dump);

static ssize_t dt2w_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%s\n", DRIVER_VERSION);

	return count;
}

static ssize_t dt2w_version_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(doubletap2wake_version, (S_IWUSR|S_IRUGO),
	dt2w_version_show, dt2w_version_dump);

// dt2w: Tuneable touch screen off voltage - by jollaman999
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
static ssize_t dt2w_screen_off_vdd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", screen_off_vdd);

	return count;
}

static ssize_t dt2w_screen_off_vdd_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long input_vdd;
	int err;

	err = kstrtol(buf, 10, &input_vdd);
	if(err)
		return err;

	/* See the range in arch/arm/mach-msm/lge/make/board-mako-regulator.c
	   When failed to apply, kernel message look like this
	   "request v=[%d, %d] cannot be met by any set point next set point: %d\n" */
	if (input_vdd >= 2125000 && input_vdd <= 3300000)
                screen_off_vdd = (int)input_vdd;

	return count;
}

static DEVICE_ATTR(dt2w_screen_off_vdd, (S_IWUSR|S_IRUGO),
	dt2w_screen_off_vdd_show, dt2w_screen_off_vdd_dump);

static void dt2w_synaptics_t1320_early_suspend(struct early_suspend *h)
{
	int rc;
	static struct regulator *vreg_l15 = NULL;

	/* 3.3V_TOUCH_VDD, VREG_L15: 2.75 ~ 3.3 */
	vreg_l15 = regulator_get(NULL, "touch_vdd");
	if (IS_ERR(vreg_l15)) {
		pr_err("%s: regulator get of 8921_l15 failed (%ld)\n",
				__func__,
		       PTR_ERR(vreg_l15));
		return;
	}

	synaptics_t1320_volatage_change_called = true;
	rc = regulator_set_voltage(vreg_l15, screen_off_vdd, screen_off_vdd);
	synaptics_t1320_volatage_change_called = false;
	if (rc < 0) {
		printk(KERN_INFO "%s: cannot control regulator\n",
		       __func__);
		return;
	} else {
		printk(KERN_INFO "%s: regulator changed\n", __func__);
	}

	return;
}

static void dt2w_synaptics_t1320_late_resume(struct early_suspend *h)
{
	int rc, i;
	static struct regulator *vreg_l15 = NULL;

	/* 3.3V_TOUCH_VDD, VREG_L15: 2.75 ~ 3.3 */
	vreg_l15 = regulator_get(NULL, "touch_vdd");
	if (IS_ERR(vreg_l15)) {
		pr_err("%s: regulator get of 8921_l15 failed (%ld)\n",
				__func__,
		       PTR_ERR(vreg_l15));
		return;
	}

	synaptics_t1320_volatage_change_called = true;
	for(i=screen_off_vdd; i<=3300000; i+=25000) {
		regulator_set_voltage(vreg_l15, i, i);
		msleep_interruptible(50);
	}
	rc = regulator_set_voltage(vreg_l15, 3300000, 3300000);
	synaptics_t1320_volatage_change_called = false;
	if (rc < 0) {
		printk(KERN_INFO "%s: cannot control regulator\n",
		       __func__);
		return;
	} else {
		printk(KERN_INFO "%s: regulator reset\n", __func__);
	}

	return;
}

static struct early_suspend dt2w_synaptics_t1320_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = dt2w_synaptics_t1320_early_suspend,
	.resume = dt2w_synaptics_t1320_late_resume,
};
#endif /* CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4 */

/*
 * INIT / EXIT stuff below here
 */
#ifdef ANDROID_TOUCH_DECLARED
extern struct kobject *android_touch_kobj;
#else
struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);
#endif
static int __init doubletap2wake_init(void)
{
	int rc = 0;

	// dt2w: Tuneable touch screen off voltage - by jollaman999
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
	screen_off_vdd = DT2W_DEFAULT_SCREEN_OFF_VDD;
#endif

	doubletap2wake_pwrdev = input_allocate_device();
	if (!doubletap2wake_pwrdev) {
		pr_err("Can't allocate suspend autotest power button\n");
		goto err_alloc_dev;
	}

	input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_POWER);
	doubletap2wake_pwrdev->name = "dt2w_pwrkey";
	doubletap2wake_pwrdev->phys = "dt2w_pwrkey/input0";

	rc = input_register_device(doubletap2wake_pwrdev);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}

	dt2w_input_wq = create_workqueue("dt2wiwq");
	if (!dt2w_input_wq) {
		pr_err("%s: Failed to create dt2wiwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&dt2w_input_work, dt2w_input_callback);
	rc = input_register_handler(&dt2w_input_handler);
	if (rc)
		pr_err("%s: Failed to register dt2w_input_handler\n", __func__);

#ifndef CONFIG_HAS_EARLYSUSPEND
	dt2w_lcd_notif.notifier_call = lcd_notifier_callback;
	if (lcd_register_client(&dt2w_lcd_notif) != 0) {
		pr_err("%s: Failed to register lcd callback\n", __func__);
	}
#else
	register_early_suspend(&dt2w_early_suspend_handler);
#endif
	// dt2w: Tuneable touch screen off voltage - by jollaman999
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
	register_early_suspend(&dt2w_synaptics_t1320_early_suspend_handler);
#endif

#ifndef ANDROID_TOUCH_DECLARED
	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		pr_warn("%s: android_touch_kobj create_and_add failed\n", __func__);
	}
#endif
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for doubletap2wake\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake_version.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for doubletap2wake_version\n", __func__);
	}
	// dt2w: Tuneable touch screen off voltage - by jollaman999
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_dt2w_screen_off_vdd.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for dt2w_screen_off_vdd\n", __func__);
	}
#endif

err_input_dev:
	input_free_device(doubletap2wake_pwrdev);
err_alloc_dev:
	pr_info(LOGTAG"%s done\n", __func__);

	return 0;
}

static void __exit doubletap2wake_exit(void)
{
#ifndef ANDROID_TOUCH_DECLARED
	kobject_del(android_touch_kobj);
#endif
#ifndef CONFIG_HAS_EARLYSUSPEND
	lcd_unregister_client(&dt2w_lcd_notif);
#endif
	input_unregister_handler(&dt2w_input_handler);
	destroy_workqueue(dt2w_input_wq);
	input_unregister_device(doubletap2wake_pwrdev);
	input_free_device(doubletap2wake_pwrdev);
	return;
}

module_init(doubletap2wake_init);
module_exit(doubletap2wake_exit);
