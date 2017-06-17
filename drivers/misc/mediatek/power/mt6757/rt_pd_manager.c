/*
 * Copyright (C) 2016 Richtek Technology Corp.
 *
 * drivers/misc/mediatek/power/mt6757/rt_pd_manager.c
 *
 * Author: Sakya <jeff_chang@richtek.com>
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
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include "tcpm.h"
#include "mtk_direct_charge_vdm.h"
#include <linux/of_device.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/charging.h>

#include "musb_core.h"
#define RT_PD_MANAGER_VERSION	"1.0.5_MTK"

static DEFINE_MUTEX(param_lock);
static DEFINE_MUTEX(tcpc_pe30_rdy_lock);
static DEFINE_MUTEX(tcpc_pd_rdy_lock);
static DEFINE_MUTEX(tcpc_usb_connect_lock);

enum {
	MTK_USB_UNATTACHE,
	MTK_USB_ATTACHED,
};

static struct tcpc_device *tcpc_dev;
static struct notifier_block pd_nb;
static unsigned char pd_state;
static int pd_sink_voltage_new;
static int pd_sink_voltage_old;
static int pd_sink_current_new;
static int pd_sink_current_old;
static unsigned char pd_sink_type;
static bool is_pep30_en_unlock;
static bool tcpc_usb_connected;
static bool tcpc_kpoc;
static unsigned char bc12_chr_type;

#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
#include "mtk_pep30_intf.h"
static DEFINE_MUTEX(pd_chr_mutex);
struct task_struct *pd_thread_handle;

static bool isCableIn;
static bool updatechrdet;

#if !defined CONFIG_HAS_WAKELOCKS
struct wakeup_source chrdet_Thread_lock;
#else
struct wake_lock chrdet_Thread_lock;
#endif

void pd_wake_lock(void)
{
#if !defined CONFIG_HAS_WAKELOCKS
	__pm_stay_awake(&chrdet_Thread_lock);
#else
	wake_lock(&chrdet_Thread_lock);
#endif
}

void pd_wake_unlock(void)
{
#if !defined CONFIG_HAS_WAKELOCKS
	__pm_relax(&chrdet_Thread_lock);
#else
	wake_unlock(&chrdet_Thread_lock);
#endif
}

void tcpc_mt_power_off(void)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	mt_power_off();
	pmic_set_register_value(PMIC_RG_USBDL_RST, 1);
	do_chrdet_int_task();
#endif /* CONFIG_MTK_KERNEL_POWER_OFF_CHARGING */
}

void pd_chrdet_int_handler(void)
{
	pr_err("[pd_chrdet_int_handler]CHRDET status = %d....\n",
		pmic_get_register_value(PMIC_RGS_CHRDET));

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (!upmu_get_rgs_chrdet()) {
		int boot_mode = 0;

		boot_mode = get_boot_mode();

		if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			|| boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
			pr_err("[pd_chrdet_int_handler] Unplug Charger/USB\n");
			mt_power_off();
		}
	}
#endif

	pmic_set_register_value(PMIC_RG_USBDL_RST, 1);
	do_chrdet_int_task();
}

int chrdet_thread_kthread(void *x)
{
	struct sched_param param = {.sched_priority = 98 };

	sched_setscheduler(current, SCHED_FIFO, &param);
	set_current_state(TASK_INTERRUPTIBLE);

	pr_err("[chrdet_thread_kthread] enter\n");
	pmic_enable_interrupt(CHRDET_INT_NO, 0, "pd_manager");

	/* Run on a process content */
	while (1) {
		mutex_lock(&pd_chr_mutex);
		if (updatechrdet == true) {
			pr_err("chrdet_work_handler\n");
			pd_chrdet_int_handler();
		} else
			pr_err("chrdet_work_handler no update\n");
		mutex_unlock(&pd_chr_mutex);
		set_current_state(TASK_INTERRUPTIBLE);
		pd_wake_unlock();
		schedule();
	}

	return 0;
}

void wake_up_pd_chrdet(void)
{
	pr_err("[wake_up_pd_chrdet]\r\n");
	pd_wake_lock();
	if (pd_thread_handle != NULL)
		wake_up_process(pd_thread_handle);
}
#endif /* CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT */

enum {
	SINK_TYPE_REMOVE,
	SINK_TYPE_TYPEC,
	SINK_TYPE_PD_TRY,
	SINK_TYPE_PD_CONNECTED,
	SINK_TYPE_REQUEST,
};

int tcpc_is_usb_connect(void)
{
	signed int vbus;

	if (tcpc_dev == NULL) {
		vbus = battery_meter_get_charger_voltage();
		pr_info("tcpc_dev is null , vbus:%d\n", vbus);
		if (vbus < 4300)
			return 0;
		return 1;
	}

	if (!tcpm_get_boot_check_flag(tcpc_dev)) {
		pr_info("%s ta hw %d\n",
			__func__, tcpm_get_ta_hw_exist(tcpc_dev));
		return tcpm_get_ta_hw_exist(tcpc_dev);
	}

	return tcpc_usb_connected ? 1 : 0;
}

bool mtk_is_ta_typec_only(void)
{
	bool stat;

	if (!tcpc_is_usb_connect())
		return false;
	stat = ((pd_state == PD_CONNECT_TYPEC_ONLY_SNK_DFT) ||
		(pd_state == PD_CONNECT_TYPEC_ONLY_SNK)) ? true : false;
	return stat;
}

bool mtk_is_pd_chg_ready(void)
{
	bool ready;

	if (!tcpc_is_usb_connect())
		return false;
	ready = (pd_state == PD_CONNECT_PE_READY_SNK) ? true : false;
	return ready;
}

bool mtk_is_pep30_en_unlock(void)
{
#ifdef CONFIG_RT7207_ADAPTER
	if (!mtk_is_pd_chg_ready())
		return false;
	return is_pep30_en_unlock;
#else
	return false;
#endif /* CONFIG_RT7207_ADAPTER */
}

static int pd_tcp_notifier_call(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	u32 vbus = 0;

	switch (event) {
	case TCP_NOTIFY_PR_SWAP:
		break;
	case TCP_NOTIFY_DR_SWAP:
		break;
	case TCP_NOTIFY_SOURCE_VCONN:
		break;
	case TCP_NOTIFY_VCONN_SWAP:
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		break;
	case TCP_NOTIFY_SINK_VBUS:
		mutex_lock(&param_lock);
		pd_sink_voltage_new = noti->vbus_state.mv;
		pd_sink_current_new = noti->vbus_state.ma;

		if (noti->vbus_state.type&TCP_VBUS_CTRL_PD_DETECT)
			pd_sink_type = SINK_TYPE_PD_CONNECTED;
		else if (noti->vbus_state.type == TCP_VBUS_CTRL_REMOVE)
			pd_sink_type = SINK_TYPE_REMOVE;
		else if (noti->vbus_state.type == TCP_VBUS_CTRL_TYPEC)
			pd_sink_type = SINK_TYPE_TYPEC;
		else if (noti->vbus_state.type == TCP_VBUS_CTRL_PD)
			pd_sink_type = SINK_TYPE_PD_TRY;
		else if (noti->vbus_state.type == TCP_VBUS_CTRL_REQUEST)
			pd_sink_type = SINK_TYPE_REQUEST;
		pr_info("%s sink vbus %dmv %dma type(%d)\n", __func__,
				pd_sink_voltage_new, pd_sink_current_new, pd_sink_type);
		mutex_unlock(&param_lock);

		if ((pd_sink_voltage_new != pd_sink_voltage_old) ||
				(pd_sink_current_new != pd_sink_current_old)) {
			if (pd_sink_voltage_new) {
				/* enable charger */
				pd_sink_voltage_old = pd_sink_voltage_new;
				pd_sink_current_old = pd_sink_current_new;
			} else {
				if (pd_sink_type == SINK_TYPE_REMOVE) {
					if (tcpc_kpoc)
						break;
					pd_sink_voltage_old = pd_sink_voltage_new;
					pd_sink_current_old = pd_sink_current_new;
				} else {
					bc12_chr_type = mt_get_charger_type();
					if (bc12_chr_type >= STANDARD_HOST &&
						bc12_chr_type <= STANDARD_CHARGER)
						break;
					if (tcpc_kpoc)
						break;
					/* disable charge */
					pd_sink_voltage_old = pd_sink_voltage_new;
					pd_sink_current_old = pd_sink_current_new;
				}
			}
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			mutex_lock(&tcpc_usb_connect_lock);
			tcpc_usb_connected = true;
			mutex_unlock(&tcpc_usb_connect_lock);
			pr_info("%s USB Plug in, pol = %d\n", __func__,
				noti->typec_state.polarity);
			if (!tcpm_get_boot_check_flag(tcpc_dev))
				tcpm_set_boot_check_flag(tcpc_dev, 1);
#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
			mutex_lock(&pd_chr_mutex);
			isCableIn = true;
			updatechrdet = true;
			wake_up_pd_chrdet();
			mutex_unlock(&pd_chr_mutex);
			pr_err("TCP_NOTIFY_SINK_VBUS=> plug in");
#endif
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SNK &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
			mutex_lock(&tcpc_usb_connect_lock);
			tcpc_usb_connected = false;
			mutex_unlock(&tcpc_usb_connect_lock);
			if (tcpc_kpoc) {
				vbus = battery_meter_get_charger_voltage();
				pr_info("%s KPOC Plug out, vbus = %d\n", __func__, vbus);
				#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
				tcpc_mt_power_off();
				break;
				#endif /* CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT */
			}
			#ifdef CONFIG_USB_PD_ALT_MODE_RTDC
			mutex_lock(&tcpc_pe30_rdy_lock);
			is_pep30_en_unlock = false;
			mutex_unlock(&tcpc_pe30_rdy_lock);
			#endif /* CONFIG_USB_PD_ALT_MODE_RTDC */
			#ifdef CONFIG_RT7207_ADAPTER
			tcpm_reset_pe30_ta(tcpc_dev);
			#endif /* CONFIG_RT7207_ADAPTER */
			pr_info("%s USB Plug out\n", __func__);
#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
			mutex_lock(&pd_chr_mutex);
			isCableIn = false;
			updatechrdet = true;
			wake_up_pd_chrdet();
			mutex_unlock(&pd_chr_mutex);
			pr_err("TCP_NOTIFY_SINK_VBUS=> plug out");
#endif
		}
		break;
	case TCP_NOTIFY_PD_STATE:
		pr_info("%s pd state = %d\n",
			__func__, noti->pd_state.connected);
		mutex_lock(&tcpc_pd_rdy_lock);
		pd_state = noti->pd_state.connected;
		mutex_unlock(&tcpc_pd_rdy_lock);

#ifdef CONFIG_MTK_SMART_BATTERY
		pr_err("%s pd state = %d %d\n",
			__func__, noti->pd_state.connected, mtk_check_pe_ready_snk());
		if (mtk_is_pd_chg_ready() == true || mtk_is_pep30_en_unlock() == true)
			wake_up_bat3();
#endif /* CONFIG_MTK_SMART_BATTERY */
		break;
#ifdef CONFIG_USB_PD_ALT_MODE_RTDC
	case TCP_NOTIFY_DC_EN_UNLOCK:
		mutex_lock(&tcpc_pe30_rdy_lock);
		is_pep30_en_unlock = true;
		mutex_unlock(&tcpc_pe30_rdy_lock);
#ifdef CONFIG_MTK_SMART_BATTERY
		wake_up_bat3();
#endif /* CONFIG_MTK_SMART_BATTERY */

		pr_info("%s Direct Charge En Unlock\n", __func__);
		break;
#endif /* CONFIG_USB_PD_ALT_MODE_RTDC */
	case TCP_NOTIFY_EXIT_MODE:
		pr_info("%s Must Stop PE3.0 Righte Now\n", __func__);
		mutex_lock(&tcpc_pe30_rdy_lock);
		is_pep30_en_unlock = false;
#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
		mtk_pep30_plugout_reset();
#endif /* CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT */
		mutex_unlock(&tcpc_pe30_rdy_lock);
		break;
	default:
		break;
	};
	return NOTIFY_OK;
}

static int rt_pd_manager_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("%s (%s)\n", __func__, RT_PD_MANAGER_VERSION);
	ret = get_boot_mode();
	if (ret == KERNEL_POWER_OFF_CHARGING_BOOT
			|| ret == LOW_POWER_OFF_CHARGING_BOOT)
		tcpc_kpoc = true;
	pr_info("%s KPOC(%d)\n", __func__, tcpc_kpoc);

	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	pd_nb.notifier_call = pd_tcp_notifier_call;
	ret = register_tcp_dev_notifier(tcpc_dev, &pd_nb);
	if (ret < 0) {
		pr_err("%s: register tcpc notifer fail\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_RT7207_ADAPTER
	ret = mtk_direct_charge_vdm_init();
	if (ret < 0) {
		pr_err("%s mtk direct charge vdm init fail\n", __func__);
		return -EINVAL;
	}
#endif /* CONFIG_RT7207_ADAPTER */

#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT
#if !defined CONFIG_HAS_WAKELOCKS
		wakeup_source_init(&chrdet_Thread_lock, "pd chrdet wakelock");
#else
		wake_lock_init(&chrdet_Thread_lock, WAKE_LOCK_SUSPEND, "pd chrdet wakelock");
#endif
	pd_thread_handle = kthread_create(chrdet_thread_kthread, (void *)NULL, "pd_chrdet_thread");
	if (IS_ERR(pd_thread_handle)) {
		pd_thread_handle = NULL;
		pr_err("[pd_thread_handle] creation fails\n");
	} else {
		pr_err("[pd_thread_handle] kthread_create Done\n");
	}
#endif /* CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT */

	pr_info("%s OK!!\n", __func__);
	return ret;
}

static int rt_pd_manager_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id rt_pd_manager_of_match[] = {
	{ .compatible = "mediatek,rt-pd-manager" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt_pd_manager_of_match);

static struct platform_driver rt_pd_manager_driver = {
	.driver = {
		.name = "rt-pd-manager",
		.of_match_table = of_match_ptr(rt_pd_manager_of_match),
	},
	.probe = rt_pd_manager_probe,
	.remove = rt_pd_manager_remove,
};

static int __init rt_pd_manager_init(void)
{
	return platform_driver_register(&rt_pd_manager_driver);
}

static void __exit rt_pd_manager_exit(void)
{
	platform_driver_unregister(&rt_pd_manager_driver);
}

late_initcall_sync(rt_pd_manager_init);
module_exit(rt_pd_manager_exit);

MODULE_AUTHOR("Jeff Chang");
MODULE_DESCRIPTION("Richtek pd manager driver");
MODULE_LICENSE("GPL");
