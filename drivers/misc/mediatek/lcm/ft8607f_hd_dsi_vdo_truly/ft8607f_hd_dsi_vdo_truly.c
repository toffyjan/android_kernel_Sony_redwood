/*****************************************************************************/
/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include <mt-plat/upmu_common.h>	/* hwPowerOn */
#include <linux/regulator/consumer.h>

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#include <mt-plat/mtk_gpio.h>
#include <mt-plat/mtk_gpio_core.h>
#include <../../cei_hw_id/cei_hw_id.h>

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util;
#define MDELAY(n) 											(lcm_util.mdelay(n))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
#define set_gpio_lcd_enp(cmd)								lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd)								lcm_util.set_gpio_lcd_enn_bias(cmd)
#define set_gpio_lcd_bkl(cmd)								lcm_util.set_gpio_lcm_backlight(cmd)
#define set_reset_pin(v)									lcm_util.set_reset_pin((v))
#define set_tp_reset_pin(cmd)									lcm_util.set_gpio_tp_reset(cmd)

//#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE                    0
#define LCM_USE_PIN_CTL                     1

#define FRAME_WIDTH 						(720)
#define FRAME_HEIGHT 						(1280)
#define GPIO_TP_ID 							23
#define GPIO_LCM_ID 						54
//#define GPIO_RESET_PIN 158

#define GPIO_FT8607_ENN_EN 					12
#define GPIO_FT8607_ENP_EN 					24
#define GPIO_LCD_LED_EN      				102
#define GPIO_RESET_PIN     					158

#ifndef GPIO_TOUCH_RESET
#define GPIO10                              10
#define GPIO_TOUCH_RESET					(GPIO10 | 0x80000000)
#define GPIO_TOUCH_RESET_M_GPIO				GPIO_MODE_00
#define GPIO_TOUCH_RESET_M_LCM_RST			GPIO_MODE_01
#endif

#define REGFLAG_DELAY             			0xFC
#define REGFLAG_UDELAY             			0xFB

#define REGFLAG_END_OF_TABLE      			0xFD   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW       			0xFE
#define REGFLAG_RESET_HIGH      			0xFF

#define HX8394F_HD_ID						(0x94)
#define PHYSICAL_WIDTH						62
#define PHYSICAL_HEIGHT						110

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;
#define DEBUG_MODE 							99

#define LCM_IS_INNOLUX 						0
#define LCM_IS_TRUELY						1

#ifndef ENABLE
	#define ENABLE 1
#endif

#ifndef DISABLE
	#define DISABLE 0
#endif

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef BUILD_LK
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

#define _LCM_DEBUG_

#ifdef BUILD_LK
#define printk printf
#endif

#ifdef _LCM_DEBUG_
#define lcm_debug(fmt, args...) printk(fmt, ##args)
#else
#define lcm_debug(fmt, args...) do { } while (0)
#endif

#ifdef _LCM_INFO_
#define lcm_info(fmt, args...) printk(fmt, ##args)
#else
#define lcm_info(fmt, args...) do { } while (0)
#endif
#define lcm_err(fmt, args...) printk(fmt, ##args)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

//Read display power mode
//{0x15, 0x00,1, {0x00}},
//{0x06, 0x0A,1, {}},
static void get_display_power_mode(void){
	char buffer[3];
	//read_reg_v2(0x0A, buffer, 1);
	//{0x39, 0xF0,5, {0x46,0x23,0x11,0x01,0x00}}, 
	//{0x39, 0xF0,5, {0x46,0x23,0x11,0x01,0x00}}, 
	//read_reg_v2();
	read_reg_v2(0xBF,  buffer, 1);
	LCM_LOGI("get_display_power_mode : [0xBF]=0x%02x\n", buffer[0]);

	read_reg_v2(0x0A,  buffer, 1);
	LCM_LOGI("get_display_power_mode : [0x0A]=0x%02x\n", buffer[0]);
}

static inline void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, (output>0)? GPIO_OUT_ONE: GPIO_OUT_ZERO);
}

static inline int lcm_get_gpio_intput(unsigned int GPIO){
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_IN);
	return mt_get_gpio_in(GPIO);
}

static inline void touch_reset_pin (int mode)
{
#ifdef LCM_USE_PIN_CTL
	if (mode == 1) {
		set_tp_reset_pin(ENABLE);
	} else if (mode == 0) {
		set_tp_reset_pin(DISABLE);
	}
#else
	mt_set_gpio_mode(GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_RESET, GPIO_DIR_OUT);

	if (mode == 1) {
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ONE);
	} else if (mode == 0) {
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
	}
#endif
}

static inline void pull_reset_pin_low(void){
#ifdef LCM_USE_PIN_CTL
	set_reset_pin(0);
#else
	lcm_set_gpio_output(GPIO_RESET_PIN, GPIO_OUT_ZERO);
#endif
}

static inline void pull_reset_pin_high(void){
#ifdef LCM_USE_PIN_CTL
	set_reset_pin(1);
#else
	lcm_set_gpio_output(GPIO_RESET_PIN, GPIO_OUT_ONE);
#endif
}

static inline void pull_enp_pin_low(void){
#ifdef LCM_USE_PIN_CTL
	set_gpio_lcd_enp(DISABLE);
#else
	lcm_set_gpio_output(GPIO_FT8607_ENP_EN, GPIO_OUT_ZERO);
#endif
}

static inline void pull_enp_pin_high(void){
#ifdef LCM_USE_PIN_CTL
	set_gpio_lcd_enp(ENABLE);
#else
	lcm_set_gpio_output(GPIO_FT8607_ENP_EN, GPIO_OUT_ONE);
#endif

}

static inline void pull_enn_pin_low(void){
#ifdef LCM_USE_PIN_CTL
	set_gpio_lcd_enn(DISABLE);
#else
	lcm_set_gpio_output(GPIO_FT8607_ENN_EN, GPIO_OUT_ZERO);
#endif

}

static inline void pull_enn_pin_high(void){
#ifdef LCM_USE_PIN_CTL
	set_gpio_lcd_enn(ENABLE);
#else
	lcm_set_gpio_output(GPIO_FT8607_ENN_EN, GPIO_OUT_ONE);
#endif

}

static inline void pull_lcd_backlight_low(void){
#ifdef LCM_USE_PIN_CTL
	set_gpio_lcd_bkl(DISABLE);
#else
	lcm_set_gpio_output(GPIO_LCD_LED_EN, GPIO_OUT_ZERO);
#endif

}

static inline void pull_lcd_backlight_high(void){
#ifdef LCM_USE_PIN_CTL
	set_gpio_lcd_bkl(ENABLE);
#else
	lcm_set_gpio_output(GPIO_LCD_LED_EN, GPIO_OUT_ONE);
#endif

}

//Read display id
static void get_lcd_id(void){
	unsigned int id = 0;
	unsigned char buffer[3];

	read_reg_v2(0x04, buffer, 3);

	id = (buffer[0] << 8) | buffer[1];	/* we only need ID */

	LCM_LOGI("[KERNEL]get_lcd_id - read id, buf:0x%02x ,0x%02x,0x%02x, id=0X%X", buffer[0], buffer[1], buffer[2], id);
}

static void inline init_lcm_registers_sleep_out(void)
{
	unsigned int data_array[16];
	//lcm_debug("%s,[KERNEL] init_lcm_registers_sleep \n",__func__);
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void inline init_lcm_registers_sleep_in(void){
	
	unsigned int data_array[16];
	//lcm_debug("%s,[KERNEL] Display Off, Sleep In \n", __func__);
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static int lcm_get_lcm_id(void){
	int lcd_id_gpio_node = -1;
	lcd_id_gpio_node = gpio_get_value(GPIO_LCM_ID);
    lcm_debug("lcd_id_gpio_node = %d \n", lcd_id_gpio_node);
    return lcd_id_gpio_node;
}

static void lcm_get_params(LCM_PARAMS *params)
{
		int lcm_id = -1;

		memset(params, 0, sizeof(LCM_PARAMS));

		params->type									= LCM_TYPE_DSI;
		params->width  									= FRAME_WIDTH;
		params->height 									= FRAME_HEIGHT;
		params->physical_width 							= PHYSICAL_WIDTH;
   		params->physical_height							= PHYSICAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   								= CMD_MODE;
		params->dsi.switch_mode 						= SYNC_PULSE_VDO_MODE;
#else
		//BURST_VDO_MODE; 
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;
		//params->dsi.switch_mode = CMD_MODE;
		//SYNC_EVENT_VDO_MODE;
		params->dsi.mode 								= SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
		params->dsi.switch_mode 						= CMD_MODE;//BURST_VDO_MODE;
		
		lcm_id = lcm_get_lcm_id();
		lcm_debug("[Mark][Kernel] LCM ID = %d \n", lcm_id);
		//Set porch value
		if(lcm_id == LCM_IS_INNOLUX){
            lcm_debug("LCM module - LCM_IS_INNOLUX\n");
            params->dsi.vertical_sync_active				= 4;
            params->dsi.vertical_backporch					= 60;
            params->dsi.vertical_frontporch					= 40;
            params->dsi.horizontal_sync_active				= 4;
            params->dsi.horizontal_backporch				= 50;
            params->dsi.horizontal_frontporch				= 100;
	   // Bit rate calculation
	   params->dsi.PLL_CLOCK 							= 230;
        }else if(lcm_id == LCM_IS_TRUELY){
            lcm_debug("LCM module - LCM_IS_TRUELY\n");
            params->dsi.vertical_sync_active				= 4;
            params->dsi.vertical_backporch					= 20;
            params->dsi.vertical_frontporch					= 36;
            params->dsi.horizontal_sync_active				= 8;
            params->dsi.horizontal_backporch				= 50;
            params->dsi.horizontal_frontporch				= 92;
	   // Bit rate calculation
	   params->dsi.PLL_CLOCK 							= 223;
        }
#endif
		params->dsi.switch_mode_enable 					= 0;

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM							= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order 			= LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   			= LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     			= LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      			= LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size							= 256;

		// Video mode setting		
		params->dsi.PS									= LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		//esd setting
		params->dsi.clk_lp_per_line_enable 				= 0;
		params->dsi.esd_check_enable 				 	= 0;
		params->dsi.customization_esd_check_enable 	 	= 0;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
}

static void lcm_init_power(void)
{
}

static inline void lcm_init(void)
{
	int hw_version = 0;
	int lcm_source_pin = 0;
	//lcm_debug("[KERNEL]%s %d\n", __func__,__LINE__);

	init_lcm_registers_sleep_out();
	lcm_source_pin = lcm_get_lcm_id();
	lcm_debug("%s, [KERNEL]: lcm_id_pin = %d\n", __func__,  lcm_source_pin);
	pull_lcd_backlight_high();

	//Check LCM ic power - 0x0A
	if(hw_version == DEBUG_MODE){
		get_display_power_mode();
		get_lcd_id();
		lcm_source_pin = lcm_get_gpio_intput(GPIO_LCM_ID);
		hw_version = get_cei_customer_project_id();
		lcm_debug("%s, [KERNEL]: lcm_source_pin = %d\n", __func__,  lcm_source_pin);
		lcm_debug("%s, [KERNEL]: hw_version = %d\n", __func__,  hw_version);
	}
}

static inline void lcm_resume_power(void)
{
	//lcm_debug("[KERNEL]%s \n", __func__);
	pmic_set_register_value(MT6351_PMIC_RG_VLDO28_EN_1, 1);
	MDELAY(2);

	pull_enp_pin_high();
	MDELAY(2);

	pull_enn_pin_high();
	MDELAY(2);

	pull_reset_pin_high();
	touch_reset_pin(GPIO_OUT_ONE);
	MDELAY(2);

	touch_reset_pin(GPIO_OUT_ZERO);
	MDELAY(2);

	pull_reset_pin_low();
	UDELAY(10);

	pull_reset_pin_high();
	UDELAY(5);
	touch_reset_pin(GPIO_OUT_ONE);

	lcm_init();
}

static inline void lcm_suspend_power(void)
{
	//lcm_debug("[KERNEL]%s \n", __func__);
}

static void lcm_suspend(void)
{
	//lcm_debug("[KERNEL]%s \n", __func__);
	pull_lcd_backlight_low();

	init_lcm_registers_sleep_in();

	touch_reset_pin(GPIO_OUT_ZERO);
	MDELAY(1);

	pull_reset_pin_low();
	MDELAY(2);

	pull_enn_pin_low();
	MDELAY(2);

	pull_enp_pin_low();
	MDELAY(2);

	pmic_set_register_value(MT6351_PMIC_RG_VLDO28_EN_1, 0);
}

static void lcm_resume(void)
{
	//lcm_debug("[KERNEL]%s \n", __func__);
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
	int lcm_source_pin = 2;
	int hw_version = 0;

	//We use LCM ID to identify main or second source LCM module
	lcm_source_pin = lcm_get_gpio_intput(GPIO_LCM_ID);

	printf("%s, [KERNEL]: LCM identify id-LCM_ID = %d\n", __func__,  lcm_source_pin);

	hw_version = get_cei_customer_project_id();

	printf("%s, [KERNEL]: hw_version = %d\n", __func__,  hw_version);
	return hw_version;
}
#endif


LCM_DRIVER ft8607f_hd_dsi_vdo_truly_lcm_drv = 
{
    .name			= "ft8607f_hd_dsi_vdo_truly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	#ifdef BUILD_LK
	.compare_id     = lcm_compare_id,
	#endif
     .init_power		= lcm_init_power,
     .resume_power = lcm_resume_power,
     .suspend_power = lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
