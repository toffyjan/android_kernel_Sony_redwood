/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kconfig.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <mrdump.h>
#include <asm/memory.h>
#include <mach/wd_api.h>
#include "mrdump_private.h"

#ifdef CONFIG_SONY_S1_SUPPORT
#define S1_WARMBOOT_MAGIC_VAL (0xBEEF)
#define S1_WARMBOOT_NORMAL    (0x7651)
#define S1_WARMBOOT_S1        (0x6F53)
#define S1_WARMBOOT_FB        (0x7700)
#define S1_WARMBOOT_NONE      (0x0000)
#define S1_WARMBOOT_CLEAR     (0xABAD)
#define S1_WARMBOOT_TOOL      (0x7001)
#define S1_WARMBOOT_RECOVERY  (0x7711)
#define S1_WARMBOOT_FOTA      (0x6F46)
#define S1_WARMBOOT_VERIFIED_BOOT_UPDATE      (0x7708)
#define S1_WARMBOOT_FOTA_CACHE (0x6F50)

extern void write_magic(volatile unsigned long magic_write, int log_option);
#endif

static void mrdump_hw_enable(bool enabled)
{
#ifndef CONFIG_MTK_LASTPC_V2
	int res;
	struct wd_api *wd_api = NULL;

	res = get_wd_api(&wd_api);
	if (res < 0)
		pr_alert("wd_ddr_reserved_mode, get wd api error %d\n", res);
	else
		wd_api->wd_dram_reserved_mode(enabled);
#endif
}

static void mrdump_reboot(void)
{
	int res;
	struct wd_api *wd_api = NULL;

	res = get_wd_api(&wd_api);
	if (res < 0) {
		pr_alert("arch_reset, get wd api error %d\n", res);
		while (1)
			cpu_relax();
	} else {

#ifdef CONFIG_SONY_S1_SUPPORT
    write_magic(S1_WARMBOOT_MAGIC_VAL | (S1_WARMBOOT_NORMAL << 16), 0);
#endif
		wd_api->wd_sw_reset(0);
	}
}

const struct mrdump_platform mrdump_v1_platform = {
	.hw_enable = mrdump_hw_enable,
	.reboot = mrdump_reboot
};

int __init mrdump_init(void)
{
	return mrdump_platform_init(&mrdump_v1_platform);
}
