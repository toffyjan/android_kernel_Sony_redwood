/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include "kd_flashlight_type.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__, ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif


static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_timeOutTimeMs;

extern int init_tlv61310(void);
extern int uninit_tlv61310(void);
extern int setDuty_tlv61310_2(int duty);
extern int flashEnable_tlv61310_2(void);
extern int flashDisable_tlv61310_2(void);
extern int g_EnableReg_check(int camId);

static void work_timeOutFunc(struct work_struct *data);

static struct hrtimer g_timeOutTimer;
static struct work_struct workTimeOut;

static int g_FL_enable = 0;

static int FL_Init(void)
{
	PK_DBG(" FL_Init line=%d\n", __LINE__);
	return init_tlv61310();
}

static int FL_dim_duty(kal_uint32 duty)
{
	setDuty_tlv61310_2(duty);
	PK_DBG("FL_dim_duty-done");
	if (g_FL_enable)
	{
		g_EnableReg_check(1);
	}
	return 0;
}

static int FL_Enable(void)
{
	g_FL_enable = 1;
	flashEnable_tlv61310_2();
	PK_DBG("FL_Enable-");

	return 0;
}

static int FL_Disable(void)
{
	g_FL_enable = 0;
	flashDisable_tlv61310_2();
	PK_DBG("FL_Disable-");

	return 0;
}

static int FL_Uninit(void)
{
	FL_Disable();
	uninit_tlv61310();
	return 0;
}

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("TLV61310 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		if (g_FL_enable) //timeout time change, reset timeout timer
		{
			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			hrtimer_cancel(&g_timeOutTimer);
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
		}
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("sub_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("sub_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);

	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}

	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("sub_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
	PK_DBG("sub_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}
