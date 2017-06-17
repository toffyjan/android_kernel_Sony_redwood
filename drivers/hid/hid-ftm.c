//--------------- Header files ------------------//
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/interrupt.h>
#include <linux/input-polldev.h>
#include <linux/kobject.h>

//-------------- Define ---------------//
#define DEBUG 0


int hid_state = 0;

static ssize_t hid_show_state(struct device_driver *ddri, char *buf) {
	printk("%s:buf=%x %x\n",__func__,buf[0],buf[1]);
	return sprintf(buf, "%d\n", hid_state);
}

static DRIVER_ATTR(state, 0644, hid_show_state, NULL);

static struct driver_attribute *hid_attr_list[] = {
	&driver_attr_state,
};

static struct platform_driver hid_attr_driver;

static int hid_attr_probe(struct platform_device *pdev) {
	int rt;
	printk("****hid_attr_probe\n");

	rt = driver_create_file(&hid_attr_driver.driver, hid_attr_list[0]);
	if (rt) {
		pr_err("hid_attr_probe: driver_create_file[0] fail!!! \n");
	}

	return 0;
}

static struct platform_driver hid_attr_driver = {
	.probe      = hid_attr_probe,
	//.remove     = sensor_remove,
	//.suspend    = sensor_suspend,
	//.resume     = sensor_resume,
	.driver = {
		.name   = "hid_attr",
		.owner  = THIS_MODULE,
	},
};

// Mike, fix it.
static struct platform_device hid_attr_device = {
	.name = "hid_attr",
	.id = -1,
};

//[Bug 861] Luke 2011.0908 T1 boot fail due to sensors
//extern int cci_ftm_boot;
static int __init hid_init(void) {
	int rc;

	printk("****hid_init\n");
	rc = platform_device_register(&hid_attr_device); // Mike, fix.
	if (rc) {
		pr_err("mike, register platform_device fail !!!\n");
	}


	rc = platform_driver_register(&hid_attr_driver);
	if (rc < 0) {
		printk("****register hid_attr_driver fail!\n");
	}
	return rc;
}

static void __exit hid_exit(void) {
	platform_driver_unregister(&hid_attr_driver);
}

module_init(hid_init);
module_exit(hid_exit);

MODULE_DESCRIPTION("hid ftm driver");
MODULE_LICENSE("GPL");
//MODULE_VERSION(DRIVER_VERSION);
