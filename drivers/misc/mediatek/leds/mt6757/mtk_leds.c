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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/of.h>
/* #include <linux/leds-mt65xx.h> */
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/delay.h>

#ifdef CONFIG_MTK_AAL_SUPPORT
#include <ddp_aal.h>
/* #include <linux/aee.h> */
#endif

#include <mt-plat/mtk_pwm.h>
#include <mt-plat/upmu_common.h>

#include "mtk_leds_sw.h"
#include "mtk_leds_hal.h"
#include "ddp_pwm.h"
#include "mtkfb.h"

//S : Creeds - Modify manual backlight brightness curve to match M4 manual curve  : {
#include <../../../misc/mediatek/cei_hw_id/cei_hw_id.h>
//E : Creeds - Modify manual backlight brightness curve to match M4 manual curve  : }

#define MET_USER_EVENT_SUPPORT
#ifdef MET_USER_EVENT_SUPPORT
#include <mt-plat/met_drv.h>
#endif

/* for LED&Backlight bringup, define the dummy API */
#ifndef CONFIG_MTK_PMIC
u16 pmic_set_register_value(u32 flagname, u32 val)
{
	return 0;
}
#endif

/* #ifndef CONFIG_BACKLIGHT_SUPPORT_LM3697 */
#if 0
static int mtkfb_set_backlight_level(unsigned int level)
{
	return 0;
}
#endif /* CONFIG_BACKLIGHT_SUPPORT_LM3697 */

#ifndef CONFIG_MTK_PWM
s32 pwm_set_spec_config(struct pwm_spec_config *conf)
{
	return 0;
}

void mt_pwm_disable(u32 pwm_no, u8 pmic_pad)
{
}
#endif

static DEFINE_MUTEX(leds_mutex);
static DEFINE_MUTEX(leds_pmic_mutex);

/****************************************************************************
 * variables
 ***************************************************************************/
/* struct cust_mt65xx_led* bl_setting_hal = NULL; */
static unsigned int bl_brightness_hal = 102;
static unsigned int bl_duty_hal = 21;
static unsigned int bl_div_hal = CLK_DIV1;
static unsigned int bl_frequency_hal = 32000;
/* for button led don't do ISINK disable first time */
static int button_flag_isink0;
static int button_flag_isink1;
static int button_flag_isink2;
static int button_flag_isink3;
//S : Creeds - Modify manual backlight brightness curve to match M4 manual curve : {
//extern int get_lcm_id(void);
//E : Creeds - Modify manual backlight brightness curve to match M4 manual curve : }
struct wake_lock leds_suspend_lock;

char *leds_name[MT65XX_LED_TYPE_TOTAL] = {
	"red",
	"green",
	"blue",
	"jogball-backlight",
	"keyboard-backlight",
	"button-backlight",
	"lcd-backlight",
};

//S : Creeds - Modify manual backlight brightness curve to match M4 manual curve : {
//Mark add for INX sec source
/*
static int backlight_change[255] = {
	3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 28,
	28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 32, 32, 32, 32, 32, 32, 32, 33, 33, 33, 34, 34, 34, 35, 35, 35, 35, 35, 36, 36, 36, 
	37, 37, 38, 38, 38, 38, 38, 39, 39, 39, 40, 40, 41, 41, 41, 41, 41, 42, 42, 42, 42, 43, 44, 44, 44, 44, 44, 45, 45, 45, 46, 46, 48, 48, 49, 49, 50, 51, 51, 52, 
	52, 53, 55, 55, 56, 56, 57, 57, 58, 59, 59, 60, 60, 62, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 87, 88, 
	88, 89, 90, 91, 92, 94, 95, 96, 97, 98, 99, 100, 102, 103, 104, 106, 107, 109, 110, 111, 112, 113, 115, 116, 117, 118, 120, 122, 123, 124, 125, 127, 128, 129, 131, 132, 134, 136, 137, 139, 
	140, 141, 143, 144, 145, 147, 148, 150, 152, 153, 155, 157, 158, 160, 162, 164, 165, 168, 170, 172, 173, 175, 177, 179, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 
	199, 201, 203, 205, 207, 210, 212, 214, 215, 217, 219, 221, 223, 225, 227, 230, 231, 233, 235, 237, 239	
	};
*/
//Mark add for mapping M4 aqua backlight curve
	/*
static int backlight_change_truly[255] = {
	2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24,
	24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 29, 29, 29,
	29, 29, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 35, 35, 35, 36, 36, 37, 37, 38, 38, 39, 40, 40, 41,
	41, 42, 43, 43, 44, 44, 45, 45, 46, 47, 47, 48, 48, 49, 49, 50, 51, 52, 53, 54, 55, 56, 57, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
	73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 86, 87, 88, 89, 90, 92, 93, 94, 95, 96, 98, 99, 100, 101, 102, 104, 105, 106, 107, 109, 110, 111, 113, 114, 115, 117, 118, 120,
	121, 122, 124, 125, 126, 128, 129, 130, 132, 133, 135, 137, 138, 140, 142, 144, 145, 147, 149, 151, 152, 154, 156, 158, 159, 161, 163, 165, 166, 168, 170, 172, 174, 176,
	177, 179, 181, 183, 185, 187, 189, 191, 192, 194, 196, 198, 200, 202, 204, 206, 207, 209, 211, 213, 215
	};
	*/
	//v7 - for SM11 final
	static int backlight_change_hinoki[255] = {
	13, 13, 14, 14, 15, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, //25
	34, 34, 34, 35, 35, 35, 35, 36, 36, 36, 36, 37, 37, 37, 37, 38, 38, 38, 38, 39, 39, 39, 39, 39, 40, //50 
	40, 40, 40, 41, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 43, 44, 44, 44, 44, 45, 45, 45, 45, 46, 46, //75
	46, 46, 47, 47, 47, 48, 48, 48, 49, 49, 49, 50, 50, 50, 51, 51, 51, 52, 52, 52, 53, 53, 53, 54, 54, //100
	54, 55, 55, 56, 56, 57, 57, 58, 58, 59, 59, 60, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, //125
	72, 74, 76, 78, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, //150
	102, 104, 106, 108, 110, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 128, 129, 130, 131, 132, //175
	132, 133, 134, 135, 136, 137, 138, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 176, 177, 178, //200
	180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220, 222, 224, 226, 228,  //225
	228, 228, 229, 229, 230, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 245, 246, 247, 248, 249, 250,  //250
	251, 252, 253, 254, 255 //255
	};

	//v5 - for SM 21 final
	static int backlight_change_redwood[255] = {
	6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, //25
	30, 30, 30, 30, 31, 31, 31, 32, 32, 32, 32, 33, 33, 33, 33, 35, 35, 35, 35, 36, 36, 36, 36, 36, 36, //50 
	37, 37, 37, 38, 38, 38, 39, 39, 39, 40, 40, 40, 40, 41, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 43, //75
	43, 43, 44, 44, 44, 45, 45, 45, 45, 46, 46, 46, 47, 47, 47, 48, 48, 49, 49, 50, 50, 50, 51, 51, 52, //100
	54, 55, 55, 56, 56, 57, 57, 58, 58, 59, 59, 60, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 69, 70, 70, //125
	72, 74, 76, 78, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, //150
	102, 104, 106, 108, 110, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 130, //175
	132, 133, 134, 135, 136, 137, 138, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 171, 172, 173, 174, //200
	180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220, 222, 224, 225, 226,  //225
	228, 228, 229, 229, 230, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 245, 246, 247, 248, 249, 250,  //250
	251, 252, 253, 254, 255 //255
	};
//E : Creeds - Modify manual backlight brightness curve to match M4 manual curve : }

struct cust_mt65xx_led *pled_dtsi;
/****************************************************************************
 * DEBUG MACROS
 ***************************************************************************/
static int debug_enable_led_hal = 1;
#define LEDS_DEBUG(format, args...) do { \
	if (debug_enable_led_hal) {	\
		pr_debug("[LED]"format, ##args);\
	} \
} while (0)

/*****************PWM *************************************************/
#define PWM_DIV_NUM 8
static int time_array_hal[PWM_DIV_NUM] = {
	256, 512, 1024, 2048, 4096, 8192, 16384, 32768 };
static unsigned int div_array_hal[PWM_DIV_NUM] = {
	1, 2, 4, 8, 16, 32, 64, 128 };

static unsigned int backlight_PWM_div_hal = CLK_DIV1;	/* this para come from cust_leds. */

/****************************************************************************
 * func:return global variables
 ***************************************************************************/
static unsigned long long current_time, last_time;
static int count;
static char buffer[4096] = "[BL] Set Backlight directly ";

static void backlight_debug_log(int level, int mappingLevel)
{
	unsigned long cur_time_mod = 0;
	unsigned long long cur_time_display = 0;

	current_time = sched_clock();
	cur_time_display = current_time;
	cur_time_mod = do_div(cur_time_display, 1000000000);

	sprintf(buffer + strlen(buffer), "T:%lld.%ld,L:%d map:%d    ",
		cur_time_display, cur_time_mod/1000000, level, mappingLevel);

	count++;

	if (level == 0 || count >= 5 || (current_time - last_time) > 1000000000) {
		LEDS_DEBUG("%s", buffer);
		count = 0;
		buffer[strlen("[BL] Set Backlight directly ")] = '\0';
	}

	last_time = sched_clock();
}

void mt_leds_wake_lock_init(void)
{
	wake_lock_init(&leds_suspend_lock, WAKE_LOCK_SUSPEND, "leds wakelock");
}

unsigned int mt_get_bl_brightness(void)
{
	return bl_brightness_hal;
}

unsigned int mt_get_bl_duty(void)
{
	return bl_duty_hal;
}

unsigned int mt_get_bl_div(void)
{
	return bl_div_hal;
}

unsigned int mt_get_bl_frequency(void)
{
	return bl_frequency_hal;
}

unsigned int *mt_get_div_array(void)
{
	return &div_array_hal[0];
}

void mt_set_bl_duty(unsigned int level)
{
	bl_duty_hal = level;
}

void mt_set_bl_div(unsigned int div)
{
	bl_div_hal = div;
}

void mt_set_bl_frequency(unsigned int freq)
{
	bl_frequency_hal = freq;
}

struct cust_mt65xx_led *get_cust_led_dtsi(void)
{
	struct device_node *led_node = NULL;
	bool isSupportDTS = false;
	int i, ret;
	int mode, data;
	int pwm_config[5] = { 0 };

	/*LEDS_DEBUG("get_cust_led_dtsi: get the leds info from device tree\n");*/

	if (pled_dtsi == NULL) {
		/* this can allocat an new struct array */
		pled_dtsi = kmalloc(MT65XX_LED_TYPE_TOTAL *
						      sizeof(struct
							     cust_mt65xx_led),
						      GFP_KERNEL);
		if (pled_dtsi == NULL) {
			LEDS_DEBUG("get_cust_led_dtsi kmalloc fail\n");
			goto out;
		}

		for (i = 0; i < MT65XX_LED_TYPE_TOTAL; i++) {

			char node_name[32] = "mediatek,";
			if (strlen(node_name) + strlen(leds_name[i]) + 1 > sizeof(node_name)) {
				LEDS_DEBUG("buffer for %s%s not enough\n", node_name, leds_name[i]);
				pled_dtsi[i].mode = 0;
				pled_dtsi[i].data = -1;
				continue;
			}

			pled_dtsi[i].name = leds_name[i];

			led_node =
			    of_find_compatible_node(NULL, NULL,
						    strncat(node_name,
							   leds_name[i], sizeof(node_name) - strlen(node_name) - 1));
			if (!led_node) {
				LEDS_DEBUG("Cannot find LED node from dts\n");
				pled_dtsi[i].mode = 0;
				pled_dtsi[i].data = -1;
			} else {
				isSupportDTS = true;
				ret =
				    of_property_read_u32(led_node, "led_mode",
							 &mode);
				if (!ret) {
					pled_dtsi[i].mode = mode;
					LEDS_DEBUG
					    ("The %s's led mode is : %d\n",
					     pled_dtsi[i].name,
					     pled_dtsi[i].mode);
				} else {
					LEDS_DEBUG
					    ("led dts can not get led mode");
					pled_dtsi[i].mode = 0;
				}

				ret =
				    of_property_read_u32(led_node, "data",
							 &data);
				if (!ret) {
					pled_dtsi[i].data = data;
					LEDS_DEBUG
					    ("The %s's led data is : %ld\n",
					     pled_dtsi[i].name,
					     pled_dtsi[i].data);
				} else {
					LEDS_DEBUG
					    ("led dts can not get led data");
					pled_dtsi[i].data = -1;
				}

				ret =
				    of_property_read_u32_array(led_node,
							       "pwm_config",
							       pwm_config,
							       ARRAY_SIZE
							       (pwm_config));
				if (!ret) {
					LEDS_DEBUG
					    ("The %s's pwm config data is %d %d %d %d %d\n",
					     pled_dtsi[i].name, pwm_config[0],
					     pwm_config[1], pwm_config[2],
					     pwm_config[3], pwm_config[4]);
					pled_dtsi[i].config_data.clock_source =
					    pwm_config[0];
					pled_dtsi[i].config_data.div =
					    pwm_config[1];
					pled_dtsi[i].config_data.low_duration =
					    pwm_config[2];
					pled_dtsi[i].config_data.High_duration =
					    pwm_config[3];
					pled_dtsi[i].config_data.pmic_pad =
					    pwm_config[4];

				} else
					LEDS_DEBUG
					    ("led dts can not get pwm config data.\n");

				switch (pled_dtsi[i].mode) {
				case MT65XX_LED_MODE_CUST_LCM:
#if defined(CONFIG_BACKLIGHT_SUPPORT_LM3697)
					pled_dtsi[i].data =
					    (long)chargepump_set_backlight_level;
					LEDS_DEBUG
					    ("backlight set by chargepump_set_backlight_level\n");
#else
					pled_dtsi[i].data =
					    (long)mtkfb_set_backlight_level;
#endif /* CONFIG_BACKLIGHT_SUPPORT_LM3697 */
					LEDS_DEBUG
					    ("kernel:the backlight hw mode is LCM.\n");
					break;
				case MT65XX_LED_MODE_CUST_BLS_PWM:
					pled_dtsi[i].data =
					    (long)disp_bls_set_backlight;
					LEDS_DEBUG
					    ("kernel:the backlight hw mode is BLS.\n");
					break;
				default:
					break;
				}
			}
		}

		if (!isSupportDTS) {
			kfree(pled_dtsi);
			pled_dtsi = NULL;
		}
	}
 out:
	return pled_dtsi;
}

struct cust_mt65xx_led *mt_get_cust_led_list(void)
{
	struct cust_mt65xx_led *cust_led_list = get_cust_led_dtsi();
	return cust_led_list;
}

/****************************************************************************
 * internal functions
 ***************************************************************************/
static int brightness_mapto64(int level)
{
	if (level < 30)
		return (level >> 1) + 7;
	else if (level <= 120)
		return (level >> 2) + 14;
	else if (level <= 160)
		return level / 5 + 20;
	else
		return (level >> 3) + 33;
}

static int find_time_index(int time)
{
	int index = 0;

	while (index < 8) {
		if (time < time_array_hal[index])
			return index;
		index++;
	}
	return PWM_DIV_NUM - 1;
}

int mt_led_set_pwm(int pwm_num, struct nled_setting *led)
{
	struct pwm_spec_config pwm_setting;
	int time_index = 0;

	memset(&pwm_setting, 0, sizeof(struct pwm_spec_config));
	pwm_setting.pwm_no = pwm_num;
	pwm_setting.mode = PWM_MODE_OLD;

	LEDS_DEBUG("led_set_pwm: mode=%d,pwm_no=%d\n", led->nled_mode,
		   pwm_num);
	/* We won't choose 32K to be the clock src of old mode because of system performance. */
	/* The setting here will be clock src = 26MHz, CLKSEL = 26M/1625 (i.e. 16K) */
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;
	pwm_setting.pmic_pad = 0;

	switch (led->nled_mode) {
	/* Actually, the setting still can not to turn off NLED. We should disable PWM to turn off NLED. */
	case NLED_OFF:
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = 0;
		pwm_setting.clk_div = CLK_DIV1;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100 / 2;
		break;

	case NLED_ON:
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = 30 / 2;
		pwm_setting.clk_div = CLK_DIV1;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100 / 2;
		break;

	case NLED_BLINK:
		LEDS_DEBUG("LED blink on time = %d offtime = %d\n",
			   led->blink_on_time, led->blink_off_time);
		time_index =
		    find_time_index(led->blink_on_time + led->blink_off_time);
		LEDS_DEBUG("LED div is %d\n", time_index);
		pwm_setting.clk_div = time_index;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH =
		    (led->blink_on_time +
		     led->blink_off_time) * MIN_FRE_OLD_PWM /
		    div_array_hal[time_index];
		pwm_setting.PWM_MODE_OLD_REGS.THRESH =
		    (led->blink_on_time * 100) / (led->blink_on_time +
						  led->blink_off_time);
		break;
	default:
		LEDS_DEBUG("Invalid nled mode\n");
		return -1;
	}

	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
	pwm_set_spec_config(&pwm_setting);

	return 0;
}

/************************ led breath function*****************************/
/*************************************************************************
 * func is to swtich to breath mode from PWM mode of ISINK
 * para: enable: 1 : breath mode; 0: PWM mode;
 *************************************************************************/
#if 0
static int led_switch_breath_pmic(enum mt65xx_led_pmic pmic_type,
				  struct nled_setting *led, int enable)
{
	/* int time_index = 0; */
	/* int duty = 0; */
	LEDS_DEBUG("led_blink_pmic: pmic_type=%d\n", pmic_type);

	if ((pmic_type != MT65XX_LED_PMIC_NLED_ISINK0
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK1
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK2
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK3)
	    || led->nled_mode != NLED_BLINK) {
		return -1;
	}
	if (enable == 1) {
		switch (pmic_type) {
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH0_MODE,
						ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL,
						0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL,
						0x03);
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK1:
			pmic_set_register_value(PMIC_ISINK_CH1_MODE,
						ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH1_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TR1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TR2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TF1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TF2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TON_SEL,
						0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TOFF_SEL,
						0x03);
			pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH1_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK2:
			pmic_set_register_value(PMIC_ISINK_CH2_MODE,
						ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH2_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TR1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TR2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TF1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TF2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TON_SEL,
						0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TOFF_SEL,
						0x03);
			pmic_set_register_value(PMIC_ISINK_DIM2_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM2_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH2_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK3:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH3_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TR1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TR2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TF1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TF2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TON_SEL,
						0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TOFF_SEL,
						0x03);
			pmic_set_register_value(PMIC_ISINK_DIM3_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM3_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH3_EN,NLED_ON); */
			break;
		default:
			break;
		}
	} else {
		switch (pmic_type) {
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_PWM_MODE);
			break;
		default:
			break;
		}
	}
	return 0;

}
#endif

#if 0 // Mike, remove MTK implement.
#define PMIC_PERIOD_NUM 8
/* 100 * period, ex: 0.01 Hz -> 0.01 * 100 = 1 */
int pmic_period_array[] = { 250, 500, 1000, 1250, 1666, 2000, 2500, 10000 };

/* int pmic_freqsel_array[] = {99999, 9999, 4999, 1999, 999, 499, 199, 4, 0}; */
int pmic_freqsel_array[] = { 0, 4, 199, 499, 999, 1999, 1999, 1999 };

static int find_time_index_pmic(int time_ms)
{
	int i;

	for (i = 0; i < PMIC_PERIOD_NUM; i++) {
		if (time_ms <= pmic_period_array[i])
			return i;
	}
	return PMIC_PERIOD_NUM - 1;
}
#endif
// Mike, load blinking isink bias.
static /*enum MT65XX_PMIC_ISINK_STEP*/ u8 _blink_steps[3] = {0,};
void load_blink_steps() {
	struct device_node *node = of_find_compatible_node(NULL, NULL, "led-blinking");
	if (!node) {
		pr_err("can not load 'led-blinking'");
		return;
	}
	if(of_property_read_u8_array(node, "isink-step", _blink_steps, ARRAY_SIZE(_blink_steps))) {
		pr_err("load array fail!");
	}
}
int mt_led_blink_pmic(enum mt65xx_led_pmic pmic_type, struct nled_setting *led)
{
	int duty = 0;
	u32 period = led->blink_on_time + led->blink_off_time;

	LEDS_DEBUG("led_blink_pmic: pmic_type=%d\n", pmic_type);
	pr_debug("using r/g/b = %d/%d/%d\n", _blink_steps[0],_blink_steps[1],_blink_steps[2]);

	if (led->nled_mode != NLED_BLINK)
		return -1;
	duty =
	    32 * led->blink_on_time / (led->blink_on_time +
				       led->blink_off_time);
	LEDS_DEBUG("LED blink on time = %d offtime = %d, duty = %d\n",
		led->blink_on_time, led->blink_off_time, duty);
	if (pmic_type > MT65XX_LED_PMIC_NLED_ISINK_MIN && pmic_type < MT65XX_LED_PMIC_NLED_ISINK_MAX)
		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */

	switch (pmic_type) {
	case MT65XX_LED_PMIC_NLED_ISINK0:
		pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
		pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_CH0_STEP, _blink_steps[0]);	/* 24mA */
		pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, duty);
		pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, period);
		pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_ON);
		break;
	case MT65XX_LED_PMIC_NLED_ISINK1:
		pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_CKSEL, 0);
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_CH1_STEP, _blink_steps[1]);	/* 24mA */
		pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, duty);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, period);
		pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_ON);
		break;
	case MT65XX_LED_PMIC_NLED_ISINK2:
		pmic_set_register_value(PMIC_RG_DRV_ISINK4_CK_PDN, 0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK4_CK_CKSEL, 0);
		pmic_set_register_value(PMIC_ISINK_CH4_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_CH4_STEP, _blink_steps[2]);	/* 24mA */
		pmic_set_register_value(PMIC_ISINK_DIM4_DUTY, duty);
		pmic_set_register_value(PMIC_ISINK_DIM4_FSEL, period);
		pmic_set_register_value(PMIC_ISINK_CH4_EN, NLED_ON);
		break;
	case MT65XX_LED_PMIC_NLED_ISINK3:
	default:
		LEDS_DEBUG("[LEDS] pmic_type %d is not handled\n", pmic_type);
		break;
	}
	return 0;
}

int mt_backlight_set_pwm(int pwm_num, u32 level, u32 div,
			 struct PWM_config *config_data)
{
	struct pwm_spec_config pwm_setting;
	unsigned int BacklightLevelSupport =
	    Cust_GetBacklightLevelSupport_byPWM();
	pwm_setting.pwm_no = pwm_num;

	if (BacklightLevelSupport == BACKLIGHT_LEVEL_PWM_256_SUPPORT)
		pwm_setting.mode = PWM_MODE_OLD;
	else
		pwm_setting.mode = PWM_MODE_FIFO;	/* New mode fifo and periodical mode */

	pwm_setting.pmic_pad = config_data->pmic_pad;

	if (config_data->div) {
		pwm_setting.clk_div = config_data->div;
		backlight_PWM_div_hal = config_data->div;
	} else
		pwm_setting.clk_div = div;

	if (BacklightLevelSupport == BACKLIGHT_LEVEL_PWM_256_SUPPORT) {
		/* PWM_CLK_OLD_MODE_32K is block/1625 = 26M/1625 = 16KHz @ MT6571 */
		if (config_data->clock_source)
			pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;
		else
			pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;
		pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
		pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
		pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
		pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 255;	/* 256 level */
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = level;

		LEDS_DEBUG("[LEDS][%d]backlight_set_pwm:duty is %d/%d\n",
			   BacklightLevelSupport, level,
			   pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
		LEDS_DEBUG("[LEDS][%d]backlight_set_pwm:clk_src/div is %d%d\n",
			   BacklightLevelSupport, pwm_setting.clk_src,
			   pwm_setting.clk_div);
		if (level > 0 && level < 256) {
			pwm_set_spec_config(&pwm_setting);
			LEDS_DEBUG
			    ("[LEDS][%d]backlight_set_pwm: old mode: thres/data_width is %d/%d\n",
			     BacklightLevelSupport,
			     pwm_setting.PWM_MODE_OLD_REGS.THRESH,
			     pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
		} else {
			LEDS_DEBUG("[LEDS][%d]Error level in backlight\n",
				   BacklightLevelSupport);
			mt_pwm_disable(pwm_setting.pwm_no,
				       config_data->pmic_pad);
		}
		return 0;

	} else {
		if (config_data->clock_source) {
			pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		} else {
			pwm_setting.clk_src =
			    PWM_CLK_NEW_MODE_BLOCK_DIV_BY_1625;
		}

		if (config_data->High_duration && config_data->low_duration) {
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION =
			    config_data->High_duration;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION =
			    pwm_setting.PWM_MODE_FIFO_REGS.HDURATION;
		} else {
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 4;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 4;
		}

		pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
		pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
		pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 31;
		pwm_setting.PWM_MODE_FIFO_REGS.GDURATION =
		    (pwm_setting.PWM_MODE_FIFO_REGS.HDURATION + 1) * 32 - 1;
		pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;

		LEDS_DEBUG("[LEDS]backlight_set_pwm:duty is %d\n", level);
		LEDS_DEBUG
		    ("[LEDS]backlight_set_pwm:clk_src/div/high/low is %d%d%d%d\n",
		     pwm_setting.clk_src, pwm_setting.clk_div,
		     pwm_setting.PWM_MODE_FIFO_REGS.HDURATION,
		     pwm_setting.PWM_MODE_FIFO_REGS.LDURATION);

		if (level > 0 && level <= 32) {
			pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
			pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 =
			    (1 << level) - 1;
			pwm_set_spec_config(&pwm_setting);
		} else if (level > 32 && level <= 64) {
			pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 1;
			level -= 32;
			pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 =
			    (1 << level) - 1;
			pwm_set_spec_config(&pwm_setting);
		} else {
			LEDS_DEBUG("[LEDS]Error level in backlight\n");
			mt_pwm_disable(pwm_setting.pwm_no,
				       config_data->pmic_pad);
		}

		return 0;

	}
}

void mt_led_pwm_disable(int pwm_num)
{
	struct cust_mt65xx_led *cust_led_list = get_cust_led_dtsi();

	mt_pwm_disable(pwm_num, cust_led_list->config_data.pmic_pad);
}

void mt_backlight_set_pwm_duty(int pwm_num, u32 level, u32 div,
			       struct PWM_config *config_data)
{
	mt_backlight_set_pwm(pwm_num, level, div, config_data);
}

void mt_backlight_set_pwm_div(int pwm_num, u32 level, u32 div,
			      struct PWM_config *config_data)
{
	mt_backlight_set_pwm(pwm_num, level, div, config_data);
}

void mt_backlight_get_pwm_fsel(unsigned int bl_div, unsigned int *bl_frequency)
{

}

void mt_store_pwm_register(unsigned int addr, unsigned int value)
{

}

unsigned int mt_show_pwm_register(unsigned int addr)
{
	return 0;
}

// Mike, implement fetch duty/step from device tree.
static inline u16 fetch_dutyStep(const struct device_node * const, u8);
int mt_brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div)
{
	static bool first_time = true;

	static struct device_node *rgb_nodes[3] = {0};
	do {
		if ( rgb_nodes[0] == 0) {
			rgb_nodes[0] = of_find_compatible_node(NULL, NULL, "mediatek,red");
			rgb_nodes[1] = of_find_compatible_node(NULL, NULL, "mediatek,green");
			rgb_nodes[2] = of_find_compatible_node(NULL, NULL, "mediatek,blue");
		}
	} while (0);

#define _duty(x) ((u8)(x>>8))
#define _step(x) ((u8)x&0x0ff)

	LEDS_DEBUG("PMIC#%d:%d\n", pmic_type, level);
	mutex_lock(&leds_pmic_mutex);
	if (pmic_type == MT65XX_LED_PMIC_NLED_ISINK0) {
		/* button flag ==0, means this ISINK is not for button backlight */
		if ((button_flag_isink0 == 0) && (first_time == true)) {
			/* sw workround for sync leds status */
			if (button_flag_isink1 == 0)
				pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_OFF);
			if (button_flag_isink2 == 0)
				pmic_set_register_value(PMIC_ISINK_CH2_EN, NLED_OFF);
			if (button_flag_isink3 == 0)
				pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_OFF);
			first_time = false;
		}
		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
		pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, ISINK_1KHZ);  /* 1KHz */
		if (level) {
			u16 ds = fetch_dutyStep(rgb_nodes[0], level);
			LEDS_DEBUG("R: %d, %d\n", _duty(ds), _step(ds) );
			pmic_set_register_value(PMIC_ISINK_CH0_STEP, _step(ds));	/* 16mA */
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, _duty(ds));
			pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_ON);
		}
		else
			pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);
		mutex_unlock(&leds_pmic_mutex);
		return 0;
	} else if (pmic_type == MT65XX_LED_PMIC_NLED_ISINK1) {
		/* button flag ==0, means this ISINK is not for button backlight */
		if ((button_flag_isink1 == 0) && (first_time == true)) {
			/* sw workround for sync leds status */
			if (button_flag_isink0 == 0)
				pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);
			if (button_flag_isink2 == 0)
				pmic_set_register_value(PMIC_ISINK_CH2_EN, NLED_OFF);
			if (button_flag_isink3 == 0)
				pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_OFF);
			first_time = false;
		}
		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_CKSEL, 0);
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, ISINK_1KHZ);  /* 1KHz */
		if (level) {
			u16 ds = fetch_dutyStep(rgb_nodes[1], level);
			LEDS_DEBUG("G: %d, %d\n", _duty(ds), _step(ds) );
			pmic_set_register_value(PMIC_ISINK_CH1_STEP, _step(ds));	/* 16mA */
			pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, _duty(ds));
			pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_ON);
		}
		else
			pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_OFF);
		mutex_unlock(&leds_pmic_mutex);
		return 0;
	} else if (pmic_type == MT65XX_LED_PMIC_NLED_ISINK2) {
		/* button flag ==0, means this ISINK is not for button backlight */
		if ((button_flag_isink1 == 0) && (first_time == true)) {
			/* sw workround for sync leds status */
			if (button_flag_isink0 == 0)
				pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);
			if (button_flag_isink2 == 0)
				pmic_set_register_value(PMIC_ISINK_CH2_EN, NLED_OFF);
			if (button_flag_isink3 == 0)
				pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_OFF);
			first_time = false;
		}
		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_RG_DRV_ISINK4_CK_PDN, 0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK4_CK_CKSEL, 0);
		pmic_set_register_value(PMIC_ISINK_CH4_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_DIM4_FSEL, ISINK_1KHZ);  /* 1KHz */
		if (level) {
			u16 ds = fetch_dutyStep(rgb_nodes[2], level);
			LEDS_DEBUG("B: %d, %d\n", _duty(ds), _step(ds) );
			pmic_set_register_value(PMIC_ISINK_CH4_STEP, _step(ds));	/* 16mA */
			pmic_set_register_value(PMIC_ISINK_DIM4_DUTY, _duty(ds));
			pmic_set_register_value(PMIC_ISINK_CH4_EN, NLED_ON);
		}
		else
			pmic_set_register_value(PMIC_ISINK_CH4_EN, NLED_OFF);
		mutex_unlock(&leds_pmic_mutex);
		return 0;
	} else if (pmic_type == MT65XX_LED_PMIC_NLED_ISINK3) {
		/* button flag ==0, means this ISINK is not for button backlight */
		if ((button_flag_isink1 == 0) && (first_time == true)) {
			/* sw workround for sync leds status */
			if (button_flag_isink0 == 0)
				pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);
			if (button_flag_isink2 == 0)
				pmic_set_register_value(PMIC_ISINK_CH2_EN, NLED_OFF);
			if (button_flag_isink3 == 0)
				pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_OFF);
			first_time = false;
		}
		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_RG_DRV_ISINK5_CK_PDN, 0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK5_CK_CKSEL, 0);
		pmic_set_register_value(PMIC_ISINK_CH5_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_CH5_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(PMIC_ISINK_DIM5_DUTY, 15);
		pmic_set_register_value(PMIC_ISINK_DIM5_FSEL, ISINK_1KHZ);	/* 1KHz */
		if (level)
			pmic_set_register_value(PMIC_ISINK_CH5_EN, NLED_ON);
		else
			pmic_set_register_value(PMIC_ISINK_CH5_EN, NLED_OFF);
		mutex_unlock(&leds_pmic_mutex);
		return 0;
	}
	mutex_unlock(&leds_pmic_mutex);
	return -1;
}

int mt_brightness_set_pmic_duty_store(u32 level, u32 div)
{
	return -1;
}

int mt_mt65xx_led_set_cust(struct cust_mt65xx_led *cust, int level)
{
	struct nled_setting led_tmp_setting = { 0, 0, 0 };
	int tmp_level = level;
	static bool button_flag;
	unsigned int BacklightLevelSupport =
	    Cust_GetBacklightLevelSupport_byPWM();

	switch (cust->mode) {

	case MT65XX_LED_MODE_PWM:
		if (strcmp(cust->name, "lcd-backlight") == 0) {
			bl_brightness_hal = level;
			if (level == 0) {
				mt_pwm_disable(cust->data,
					       cust->config_data.pmic_pad);

			} else {

				if (BacklightLevelSupport ==
				    BACKLIGHT_LEVEL_PWM_256_SUPPORT)
					level = brightness_mapping(tmp_level);
				else
					level = brightness_mapto64(tmp_level);
				mt_backlight_set_pwm(cust->data, level,
						     bl_div_hal,
						     &cust->config_data);
			}
			bl_duty_hal = level;

		} else {
			if (level == 0) {
				led_tmp_setting.nled_mode = NLED_OFF;
				mt_led_set_pwm(cust->data, &led_tmp_setting);
				mt_pwm_disable(cust->data,
					       cust->config_data.pmic_pad);
			} else {
				led_tmp_setting.nled_mode = NLED_ON;
				mt_led_set_pwm(cust->data, &led_tmp_setting);
			}
		}
		return 1;

	case MT65XX_LED_MODE_GPIO:
		LEDS_DEBUG("brightness_set_cust:go GPIO mode!!!!!\n");
		return ((cust_set_brightness) (cust->data)) (level);

	case MT65XX_LED_MODE_PMIC:
		/* for button baclight used SINK channel, when set button ISINK,
		 * don't do disable other ISINK channel
		 */
		if ((strcmp(cust->name, "button-backlight") == 0)) {
			if (button_flag == false) {
				switch (cust->data) {
				case MT65XX_LED_PMIC_NLED_ISINK0:
					button_flag_isink0 = 1;
					break;
				case MT65XX_LED_PMIC_NLED_ISINK1:
					button_flag_isink1 = 1;
					break;
				case MT65XX_LED_PMIC_NLED_ISINK2:
					button_flag_isink2 = 1;
					break;
				case MT65XX_LED_PMIC_NLED_ISINK3:
					button_flag_isink3 = 1;
					break;
				default:
					break;
				}
				button_flag = true;
			}
		}
		return mt_brightness_set_pmic(cust->data, level, bl_div_hal);

	case MT65XX_LED_MODE_CUST_LCM:
		if (strcmp(cust->name, "lcd-backlight") == 0)
			bl_brightness_hal = level;
		LEDS_DEBUG("brightness_set_cust:backlight control by LCM\n");
		/* warning for this API revork */
		return ((cust_brightness_set) (cust->data)) (level, bl_div_hal);

	case MT65XX_LED_MODE_CUST_BLS_PWM:
		if (strcmp(cust->name, "lcd-backlight") == 0)
			bl_brightness_hal = level;
#ifdef MET_USER_EVENT_SUPPORT
		if (enable_met_backlight_tag())
			output_met_backlight_tag(level);
#endif
		return ((cust_set_brightness) (cust->data)) (level);

	case MT65XX_LED_MODE_NONE:
	default:
		break;
	}
	return -1;
}

void mt_mt65xx_led_work(struct work_struct *work)
{
	struct mt65xx_led_data *led_data =
	    container_of(work, struct mt65xx_led_data, work);

	LEDS_DEBUG("%s:%d\n", led_data->cust.name, led_data->level);
	mutex_lock(&leds_mutex);
	mt_mt65xx_led_set_cust(&led_data->cust, led_data->level);
	mutex_unlock(&leds_mutex);
}

void mt_mt65xx_led_set(struct led_classdev *led_cdev, enum led_brightness level)
{
	struct mt65xx_led_data *led_data =
	    container_of(led_cdev, struct mt65xx_led_data, cdev);
	/* unsigned long flags; */
	/* spin_lock_irqsave(&leds_lock, flags); */
	//S : Creeds - Modify manual backlight brightness curve to match M4 manual curve  : {
	if (strcmp(led_data->cust.name, "lcd-backlight") == 0) {
		if((level > 0) && (level < 256)) {
			if(get_cei_customer_project_id() == CUSTOMER_PROJECT_SM11){
				LEDS_DEBUG("Backlight-level tunning.\n");
				level = backlight_change_hinoki[level-1];
			}else if(get_cei_customer_project_id() == CUSTOMER_PROJECT_SM21){
				LEDS_DEBUG("Backlight-level tunning..\n");
				level = backlight_change_redwood[level-1];
			}
		}
	}
	//E : Creeds - Modify manual backlight brightness curve to match M4 manual curve : }
#ifdef CONFIG_MTK_AAL_SUPPORT
	if (led_data->level != level) {
		led_data->level = level;
		if (strcmp(led_data->cust.name, "lcd-backlight") != 0) {
			LEDS_DEBUG("Set NLED directly %d at time %lu\n",
				   led_data->level, jiffies);
			schedule_work(&led_data->work);
		} else {
			if (level != 0
			    && level * CONFIG_LIGHTNESS_MAPPING_VALUE < 255) {
				level = 1;
			} else {
				level =
				    (level * CONFIG_LIGHTNESS_MAPPING_VALUE) /
				    255;
			}
			backlight_debug_log(led_data->level, level);
			disp_aal_notify_backlight_changed((((1 <<
							     MT_LED_INTERNAL_LEVEL_BIT_CNT)
							    - 1) * level +
							   127) / 255);
		}
	}
#else
	/* do something only when level is changed */
	if (led_data->level != level) {
		led_data->level = level;
		if (strcmp(led_data->cust.name, "lcd-backlight") != 0) {
			LEDS_DEBUG("Set NLED directly %d at time %lu\n",
				   led_data->level, jiffies);
			schedule_work(&led_data->work);
		} else {
			if (level != 0
			    && level * CONFIG_LIGHTNESS_MAPPING_VALUE < 255) {
				level = 1;
			} else {
				level =
				    (level * CONFIG_LIGHTNESS_MAPPING_VALUE) /
				    255;
			}
			backlight_debug_log(led_data->level, level);
			if (led_data->cust.mode == MT65XX_LED_MODE_CUST_BLS_PWM) {
				mt_mt65xx_led_set_cust(&led_data->cust,
						       ((((1 <<
							   MT_LED_INTERNAL_LEVEL_BIT_CNT)
							  - 1) * level +
							 127) / 255));
			} else {
				mt_mt65xx_led_set_cust(&led_data->cust, level);
			}
		}
	}
	/* spin_unlock_irqrestore(&leds_lock, flags); */
#endif
/* if(0!=aee_kernel_Powerkey_is_press()) */
/* aee_kernel_wdt_kick_Powkey_api("mt_mt65xx_led_set",WDT_SETBY_Backlight); */
}

int mt_mt65xx_blink_set(struct led_classdev *led_cdev,
			unsigned long *delay_on, unsigned long *delay_off)
{
	struct mt65xx_led_data *led_data =
	    container_of(led_cdev, struct mt65xx_led_data, cdev);
	static int got_wake_lock;
	struct nled_setting nled_tmp_setting = { 0, 0, 0 };

	/* only allow software blink when delay_on or delay_off changed */
	if (*delay_on != led_data->delay_on
	    || *delay_off != led_data->delay_off) {
		led_data->delay_on = *delay_on;
		led_data->delay_off = *delay_off;
		if (led_data->delay_on && led_data->delay_off) {	/* enable blink */
			led_data->level = 255;	/* when enable blink  then to set the level  (255) */
			/* AP PWM all support OLD mode */
			if (led_data->cust.mode == MT65XX_LED_MODE_PWM) {
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time =
				    led_data->delay_off;
				nled_tmp_setting.blink_on_time =
				    led_data->delay_on;
				mt_led_set_pwm(led_data->cust.data,
					       &nled_tmp_setting);
				return 0;
			} else if ((led_data->cust.mode == MT65XX_LED_MODE_PMIC)
				   && (led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK0
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK1
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK2
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK3)) {
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time =
				    led_data->delay_off;
				nled_tmp_setting.blink_on_time =
				    led_data->delay_on;
				mt_led_blink_pmic(led_data->cust.data,
						  &nled_tmp_setting);
				return 0;
			} else if (!got_wake_lock) {
				wake_lock(&leds_suspend_lock);
				got_wake_lock = 1;
			}
		} else if (!led_data->delay_on && !led_data->delay_off) {	/* disable blink */
			/* AP PWM all support OLD mode */
			if (led_data->cust.mode == MT65XX_LED_MODE_PWM) {
				nled_tmp_setting.nled_mode = NLED_OFF;
				mt_led_set_pwm(led_data->cust.data,
					       &nled_tmp_setting);
				return 0;
			} else if ((led_data->cust.mode == MT65XX_LED_MODE_PMIC)
				   && (led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK0
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK1
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK2
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK3)) {
				mt_brightness_set_pmic(led_data->cust.data, 0,
						       0);
				return 0;
			} else if (got_wake_lock) {
				wake_unlock(&leds_suspend_lock);
				got_wake_lock = 0;
			}
		}
		return -1;
	}
	/* delay_on and delay_off are not changed */
	return 0;
}

static inline u16 fetch_dutyStep(const struct device_node * const node, u8 level) {
	int res;
	u32 data;
	u8 duty, step;
#define merge(x,y) ({u16 t=x;t<<=8;t+=y;})
	res = of_property_read_u32_index(node, "cei,pwm_currents", level, &data);
	if (res) goto error;
	step = (u8) data;
	res = of_property_read_u32_index(node, "cei,pwm_dutys", level, &data);
	if (res) goto error;
	duty = (u8) data;
	return merge(duty, step);
error:
	return 0;
}


/** Mike, discuss call flow.
 * mt65xx_led_set -> mt_mt65xx_led_set -> mt_mt65xx_led_set_cust -> mt_brightness_set_pmic
 * func: mt65xx_led_set, registered to led.set_brightness function pointer.
 * 	Will be callback if userspace write to /sy/class/leds device node.
 *
 *
 */
