/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/stk3x1x.c - stk3x1x ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/security.h>
#include "./../../../../../fs/sysfs/sysfs.h"
#include "./../../../../base/base.h"
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include "../../hwmon/include/hwmsen_helper.h"
#include "../../hwmon/include/hwmsen_dev.h"
#include "../../../../i2c/i2c-core.h"
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/version.h>
#include <linux/fs.h>  
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/wakelock.h> 
#include <asm/io.h>
#include <linux/module.h>
#include <linux/time.h>
#include <cust_alsps.h>
#include "stk3x1x.h"
#include <alsps.h>
#define LightSensorK
#define DRIVER_VERSION          "3.1.2.1"
//#define STK_PS_POLLING_LOG
#define STK_TUNE0
//#define STK_FIR
//#define STK_IRS
//extern int board_type_with_hw_id(void);
//extern int SENSOR_OPTION;
extern int get_cei_customer_project_id(void);
int hwid=-1;
#define BY57_hwid 0
#define BY86_hwid 1


/*MT6516&MT6573 define-------------------------------*/
#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#if ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589) || (defined MT6572) || (defined MT6755))	 //LT8X porting for STK3210
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif //LT8X porting for STK3210
#define USE_LINUX_PIN_CONTROL
/******************************************************************************
 * configuration
*******************************************************************************/
#define CONFIG_OF_DT

/*----------------------------------------------------------------------------*/
#define stk3x1x_DEV_NAME     "stk3x1x"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[stk_ALS/PS] "
// Celia start for LY28
#define APS_FUN(f)               printk(APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(fmt, ##args)
// Celia end
/******************************************************************************
 * extern functions
*******************************************************************************/
#define STK2213_PID			0x23
#define STK2213I_PID			0x22
#define STK3010_PID			0x33
#define STK3210_STK3310_PID	0x13
#define STK3211_STK3311_PID	0x12
#define STK3311_WV_PID	0x1d
#define STK3311_R35A	0x14
//CCI Colby add start 20130312
int CCI_debug_for_stk3x1x = 0;
//CCI Colby add end 20130312
int CCI_FTM_INTERRUPT = 0;//add for FTM interrupt check 20130416
//int CCI_FTM = 0;//add for FTM interrupt check 20130416
// CCI add First boot H/L thd for debug start
int FIRST_BOOT_H_THD = 0;
int FIRST_BOOT_L_THD = 0;
// CCI add First boot H/L thd for debug end
#ifdef STK_TUNE0
	#define STK_MAX_MIN_DIFF	200
	#define STK_LT_N_CT	150
	#define STK_HT_N_CT	250
	//STK add for CCI start 20130112
	//#define STK_COVER_LIMIT	500
	#define MAX_COMPARE(a,b) (a>b)? a:b
	#define MIN_COMPARE(a,b) (a<b)? a:b


	unsigned int cci_ps_high_thd=0;
	unsigned int cci_ps_low_thd=0;
	//STK add for CCI end 20130112

	/*VY36 use ony start*/
	int offset_H=0;
	int offset_L=0;
	/*VY36 use ony end*/
#endif /* #ifdef STK_TUNE0 */

#define STK_IRC_MAX_ALS_CODE		20000
#define STK_IRC_MIN_ALS_CODE		25
#define STK_IRC_MIN_IR_CODE		50
#define STK_IRC_ALS_DENOMI		2		
#define STK_IRC_ALS_NUMERA		5
#define STK_IRC_ALS_CORREC		748

/*----------------------------------------------------------------------------*/
static struct i2c_client *stk3x1x_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
/*static const struct i2c_device_id stk3x1x_i2c_id[] = {{stk3x1x_DEV_NAME,0},{}};
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(3,0,0))	
static struct i2c_board_info __initdata i2c_stk3x1x={ I2C_BOARD_INFO("stk3x1x", (0x90>>1))};
#else
//the adapter id & i2c address will be available in customization
static unsigned short stk3x1x_force[] = {0x00, 0x00, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const stk3x1x_forces[] = { stk3x1x_force, NULL };
static struct i2c_client_address_data stk3x1x_addr_data = { .forces = stk3x1x_forces,};
#endif*/
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int stk3x1x_i2c_remove(struct i2c_client *client);
#if (LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	
static int stk3x1x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#endif
/*----------------------------------------------------------------------------*/
//static int stk3x1x_i2c_suspend(struct i2c_client *client, pm_message_t msg);
//static int stk3x1x_i2c_resume(struct i2c_client *client);

//static struct stk3x1x_priv *g_stk3x1x_ptr = NULL;
struct platform_device *alspsPltFmDev= NULL;
/*----------------------------------------------------------------------------*/
typedef enum {
    STK_TRC_ALS_DATA= 0x0001,
    STK_TRC_PS_DATA = 0x0002,
    STK_TRC_EINT    = 0x0004,
    STK_TRC_IOCTL   = 0x0008,
    STK_TRC_I2C     = 0x0010,
    STK_TRC_CVT_ALS = 0x0020,
    STK_TRC_CVT_PS  = 0x0040,
    STK_TRC_DEBUG   = 0x8000,
} STK_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
    STK_BIT_ALS    = 1,
    STK_BIT_PS     = 2,
} STK_BIT;
/*----------------------------------------------------------------------------*/
struct stk3x1x_i2c_addr {    
/*define a series of i2c slave address*/
    u8  state;      	/* enable/disable state */
    u8  psctrl;     	/* PS control */
    u8  alsctrl;    	/* ALS control */
    u8  ledctrl;   		/* LED control */
    u8  intmode;    	/* INT mode */
    u8  wait;     		/* wait time */
    u8  thdh1_ps;   	/* PS INT threshold high 1 */
	u8	thdh2_ps;		/* PS INT threshold high 2 */
    u8  thdl1_ps;   	/* PS INT threshold low 1 */
	u8  thdl2_ps;   	/* PS INT threshold low 2 */
    u8  thdh1_als;   	/* ALS INT threshold high 1 */
	u8	thdh2_als;		/* ALS INT threshold high 2 */
    u8  thdl1_als;   	/* ALS INT threshold low 1 */
	u8  thdl2_als;   	/* ALS INT threshold low 2 */	
	u8  flag;			/* int flag */
	u8  data1_ps;		/* ps data1 */
	u8  data2_ps;		/* ps data2 */
	u8  data1_als;		/* als data1 */
	u8  data2_als;		/* als data2 */
	u8  data1_offset;	/* offset data1 */
	u8  data2_offset;	/* offset data2 */
	u8  data1_ir;		/* ir data1 */
	u8  data2_ir;		/* ir data2 */
	u8  soft_reset;		/* software reset */
};
/*----------------------------------------------------------------------------*/
#ifdef STK_FIR
struct data_filter {
    s16 raw[8];
    int sum;
    int num;
    int idx;
};
#endif
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

struct stk3x1x_priv {
    struct mutex			stk3x1x_op_mutex;
    struct input_dev *als_input_dev;	
    struct input_dev *ps_input_dev;
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;//delayed_work
    struct workqueue_struct *stk_wq;	
     int32_t irq;
    /*i2c address group*/
    struct stk3x1x_i2c_addr  addr;
    
    /*misc*/
    atomic_t    trace;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16         ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

	atomic_t	state_val;
	atomic_t 	psctrl_val;
	atomic_t 	alsctrl_val;
	u8 			wait_val;
	u8		 	ledctrl_val;
	u8		 	int_val;
	
    atomic_t    ps_high_thd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_low_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
	atomic_t	recv_reg;
    /*early suspend*/
//#if defined(CONFIG_HAS_EARLYSUSPEND)
    //struct early_suspend    early_drv;
//#endif     
	bool first_boot;
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;
	uint16_t psi_set;
	struct hrtimer ps_tune0_timer;
	struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;
	ktime_t ps_tune0_delay;
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;
#endif
#ifdef STK_FIR
	struct data_filter      fir;
#endif
	uint16_t ir_code;
	uint16_t als_correct_factor;	
#ifdef USE_LINUX_PIN_CONTROL
	/* pinctrl data */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pins_cfg;
	struct pinctrl_state *pin_sleep;
#endif
	struct device_node *irq_node;
};
static struct stk3x1x_priv *stk3x1x_obj = NULL;

u32 interrupt_gpio_num = -1;
/*----------------------------------------------------------------------------*/
static int stk3x1x_local_init(void);
static int stk3x1x_local_remove(void);
static struct alsps_init_info stk3x1x_init_info = {
	.name = "stk3x1x",
	.init = stk3x1x_local_init,
	.uninit = stk3x1x_local_remove,
};

static struct of_device_id stk3x1x_dts_table[] = {
		{ .compatible  = "mediatek,alsps",}, //stk,stk3x1x
		{ },
};
static int stk3x1x_enable_als(struct i2c_client *client, int enable);

/*----------------------------------------------------------------------------*/
int check_irqdepth(int irqnum)
{
	struct irq_desc *desc=irq_to_desc(irqnum);
	int depth;
	depth=desc->depth;
	return depth;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int stk3x1x_suspend(struct device *dev)
{
	 int err;
        int old = atomic_read(&stk3x1x_obj->state_val);
	printk("stk3x1x_suspend \n");	
	APS_FUN();
        if(!stk3x1x_obj)
        {
                APS_ERR("null pointer!!\n");
                return -1;
        }

        if(old & STK_STATE_EN_ALS_MASK)
        {
                atomic_set(&stk3x1x_obj->als_suspend, 1);
		  err = stk3x1x_enable_als(stk3x1x_obj->client, 0);
                if(err<0)
                {
                        APS_ERR("disable als fail: %d\n", err);
                }
        }
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_resume(struct device *dev)
{
	 int err;
	APS_FUN();
	printk("stk3x1x_resume \n");	
        if(!stk3x1x_obj)
        {
                APS_ERR("null pointer!!\n");
                return 0;
        }
        if(atomic_read(&stk3x1x_obj->als_suspend))
        {
                atomic_set(&stk3x1x_obj->als_suspend, 0);
                if(test_bit(STK_BIT_ALS, &stk3x1x_obj->enable))
                {
                	err = stk3x1x_enable_als(stk3x1x_obj->client, 1);
                        if(err<0)
                        {
                                APS_ERR("enable als fail: %d\n", err);

                        }
                }
        }
	return 0;
}
static const struct dev_pm_ops stk3x1x_mt_pm_ops = {
	.suspend	= stk3x1x_suspend,
	.resume		= stk3x1x_resume,
};
static const struct i2c_device_id stk_ps_id[] =
{
    { "stk_ps", 0},
    {}
};
static struct i2c_driver stk3x1x_i2c_driver = {	
	.probe      = stk3x1x_i2c_probe,
	.remove     = stk3x1x_i2c_remove,
	.id_table = stk_ps_id,
#if (LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	
	.detect     = stk3x1x_i2c_detect,
#endif
#if (LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	
	.address_data = &stk3x1x_addr_data,
#endif
	.driver = {
//#if (LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	
		.owner          = THIS_MODULE,
//#endif
		.of_match_table = stk3x1x_dts_table,
		.pm = &stk3x1x_mt_pm_ops,
		.name           = stk3x1x_DEV_NAME,
	},
};
// add for FTM interrupt check 20130416 start
struct CCI_threshold {
	unsigned  int ps_high_thd;
	unsigned  int ps_low_thd;
};
// add for FTM interrupt check 20130416 end
//CCI Colby add start 20130111
struct stk3x1x_threshold {
	unsigned  int result_ct;
	unsigned  int result_ps_cover;
	unsigned  int result_als_value;
	unsigned int cci_als_adc_test;
       unsigned  int cci_transmittance_cali; // add for FTM ALS CALI
       unsigned  int cci_als_adc_cali;// add for FTM ALS CALI
       unsigned int cci_als_lux_cali;
	unsigned  int ps_high_thd;
	unsigned  int ps_low_thd;
};
unsigned int cci_result_ct=0;
unsigned int cci_ps_cover=0;
unsigned int cci_als_value=0;
//CCI Colby add end 20130111
//CCI Colby add start
unsigned int cci_transmittance_cali = 900;
unsigned int cci_als_adc_cali=0; //cali adc
unsigned int cci_als_lux_cali=0;//cali lux
unsigned int cci_als_adc_test=0;//test adc
int dummyLuxValue = -1;/* 20161101 add lux control */

//CCI Colby add end

//static struct platform_driver stk3x1x_alsps_driver;
static int stk3x1x_get_ps_value(struct stk3x1x_priv *obj, u16 ps);
static int stk3x1x_get_ps_value_only(struct stk3x1x_priv *obj, u16 ps);
static int stk3x1x_get_als_lux(struct stk3x1x_priv *obj, u16 als);
static int stk3x1x_read_als(struct i2c_client *client, u16 *data);
static int stk3x1x_read_ps(struct i2c_client *client, u16 *data);
static int stk3x1x_set_als_int_thd(struct i2c_client *client, u16 als_data_reg);
static int32_t stk3x1x_get_ir_value(struct stk3x1x_priv *obj);
#ifdef STK_TUNE0
//STK add for checking if the register value is ok 20130318 start
static int stk3x1x_init_client(struct i2c_client *client);
//STK add for checking if the register value is ok 20130318 end
static int stk_ps_tune_zero_func_fae(struct stk3x1x_priv *obj);
#endif
struct wake_lock ps_lock;

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <asm/atomic.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <linux/file.h>

/*----------------------------------------------------------------------------*/
inline uint32_t stk_alscode2lux(uint32_t alscode)
{ 
//CCI Colby add start 20131024
//add for ALS cali
	/* 20161101 add lux control */
	if (dummyLuxValue >= 0) {
		return dummyLuxValue;
	}
	/* 20161101 add lux control */
    if(((alscode)*cci_transmittance_cali) > 72088500){  //(((*data)*cci_transmittance_cali)/1100) > 65535
        alscode = 65535;
        //APS_DBG("*data > 65535, set *data = 65535");
        }
    else
    	{
    	if(cci_transmittance_cali>0)
        alscode = (alscode*1100)/cci_transmittance_cali;//(alscode*cci_transmittance_cali)/1100;
	else
		return 0;
    	}
    	if(CCI_debug_for_stk3x1x)
		APS_DBG("stk cci_transmittance_cali = %d, alscode = %d.", cci_transmittance_cali,alscode);
//CCI Colby add end 20131024
	  return alscode;

}
int stk3x1x_get_addr(struct alsps_hw *hw, struct stk3x1x_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->state   = STK_STATE_REG; 
	addr->psctrl   = STK_PSCTRL_REG;         
	addr->alsctrl  = STK_ALSCTRL_REG;
	addr->ledctrl  = STK_LEDCTRL_REG;
	addr->intmode    = STK_INT_REG;
	addr->wait    = STK_WAIT_REG;
	addr->thdh1_ps    = STK_THDH1_PS_REG;
	addr->thdh2_ps    = STK_THDH2_PS_REG;
	addr->thdl1_ps = STK_THDL1_PS_REG;
	addr->thdl2_ps = STK_THDL2_PS_REG;
	addr->thdh1_als    = STK_THDH1_ALS_REG;
	addr->thdh2_als    = STK_THDH2_ALS_REG;
	addr->thdl1_als = STK_THDL1_ALS_REG ;
	addr->thdl2_als = STK_THDL2_ALS_REG;
	addr->flag = STK_FLAG_REG;	
	addr->data1_ps = STK_DATA1_PS_REG;
	addr->data2_ps = STK_DATA2_PS_REG;
	addr->data1_als = STK_DATA1_ALS_REG;	
	addr->data2_als = STK_DATA2_ALS_REG;	
	addr->data1_offset = STK_DATA1_OFFSET_REG;
	addr->data2_offset = STK_DATA2_OFFSET_REG;
	addr->data1_ir = STK_DATA1_IR_REG;	
	addr->data2_ir = STK_DATA2_IR_REG;		
	addr->soft_reset = STK_SW_RESET_REG;	
	
	return 0;
}
/*----------------------------------------------------------------------------*/
#define C_I2C_FIFO_SIZE 8
int stk3x1x_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr; 
	struct i2c_msg msgs[2] = 
	{
		{
			.addr = client->addr,	 
			.flags = 0,
			.len = 1,				 
			.buf= &beg,
		},
		{
			.addr = client->addr,	 
			.flags = I2C_M_RD,
			.len = len, 			 
			.buf = data,
		}
	};
	int err;

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) 
	{		 
		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) 
	{
		APS_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else 
	{
		err = 0;/*no error*/
	}
	return err;
}
/*----------------------------------------------------------------------------*/
/*
int stk3x1x_get_timing(void)
{
	return 200;

//	u32 base = I2C2_BASE; 
//	return (__raw_readw(mt6516_I2C_HS) << 16) | (__raw_readw(mt6516_I2C_TIMING));

}*/

/*----------------------------------------------------------------------------*/
int stk3x1x_master_recv(struct i2c_client *client, u16 addr, u8 *buf ,int count)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0, retry = 0;
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);

	while(retry++ < max_try)
	{
		ret = stk3x1x_hwmsen_read_block(client, addr, buf, count);
		if(ret == 0)
            break;
		udelay(100);
	}

	if(unlikely(trc))
	{
		if((retry != 1) && (trc & STK_TRC_DEBUG))
		{
			APS_LOG("(recv) %d/%d\n", retry-1, max_try); 

		}
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_master_send(struct i2c_client *client, u16 addr, u8 *buf ,int count)
{
	int ret = 0, retry = 0;
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);


	while(retry++ < max_try)
	{
		ret = hwmsen_write_block(client, addr, buf, count);
		if (ret == 0)
		    break;
		udelay(100);
	}

	if(unlikely(trc))
	{
		if((retry != 1) && (trc & STK_TRC_DEBUG))
		{
			APS_LOG("(send) %d/%d\n", retry-1, max_try);
		}
	}
	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_led(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
    
    ret = stk3x1x_master_send(client, obj->addr.ledctrl, &data, 1);
	if(ret < 0)
	{
		APS_ERR("write led = %d\n", ret);
		return -EFAULT;
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_als(struct i2c_client *client, u16 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2]={0,0};
	int32_t als_comperator;	
	u16 als_data;
    //CCI Colby add for ALS cali
    //u16 als_ori;
#ifdef STK_FIR
	int idx;   
#endif
	if(NULL == client)
	{
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, obj->addr.data1_als, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		als_data = (buf[0] << 8) | (buf[1]);
#ifdef STK_FIR
		if(obj->fir.num < 8)
		{                
			obj->fir.raw[obj->fir.num] = als_data;
			obj->fir.sum += als_data;
			obj->fir.num++;
			obj->fir.idx++;
		}
		else
		{
			idx = obj->fir.idx % 8;
			obj->fir.sum -= obj->fir.raw[idx];
			obj->fir.raw[idx] = als_data;
			obj->fir.sum += als_data;
			obj->fir.idx++;
			als_data = obj->fir.sum/8;
		}	
#endif
	}
	
	if(obj->ir_code)
	{
		obj->als_correct_factor = 1000;
		if(als_data < STK_IRC_MAX_ALS_CODE && als_data > STK_IRC_MIN_ALS_CODE && 
			obj->ir_code > STK_IRC_MIN_IR_CODE)
		{
			als_comperator = als_data * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(obj->ir_code > als_comperator)
				obj->als_correct_factor = STK_IRC_ALS_CORREC;
		}
		APS_LOG("%s: als=%d, ir=%d, als_correct_factor=%d", __func__, als_data, obj->ir_code, obj->als_correct_factor);
		obj->ir_code = 0;
	}	
	*data = als_data * obj->als_correct_factor / 1000;
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("ALS: 0x%04X\n", (u32)(*data));
	}
	
	return 0;    
}

static uint32_t stk3x1x_get_als_reading_AVG(int sSampleNo){
	uint32_t ALSData = 0;
	uint32_t DataCount = 0;
	uint32_t sAveAlsData = 0;
	int err=0;
	
	while(DataCount < sSampleNo)
	{
		msleep(100);
		if((err = stk3x1x_read_als(stk3x1x_obj->client, &stk3x1x_obj->als))){
			APS_ERR("[STK]%s: stk3x1x_read_als adc fail\n", __func__);
			}
		else{
			ALSData = stk3x1x_obj->als;
			if(CCI_debug_for_stk3x1x)
				APS_ERR("[STK]%s: als adc code = %d\n", __func__, ALSData);
			sAveAlsData +=  ALSData;
			DataCount++;
			}
	}
	sAveAlsData /= sSampleNo;
	return sAveAlsData;
}

/*----------------------------------------------------------------------------*/


int stk3x1x_cci_als_cali(int sSampleNo)
{
  //  int res;
    int32_t als_reading=0;
    APS_ERR("[#23]stk3x1x_cci_als_cali\n");
    if(!stk3x1x_obj)
    {
        APS_ERR("stk3x1x_obj is null!!\n");
        return 0;
    }
    cci_transmittance_cali = 1100; //set cci_transmittance_cali as default 1100
    als_reading = stk3x1x_get_als_reading_AVG(sSampleNo); 

    //do ALS cali
    if(als_reading == 0) {
		APS_LOG("[#23]%s als_reading = 0", __func__);
        return -1;
	}

    //if ALS out of range, dont do calibration
    if(als_reading <= 0 || als_reading >= 65535){
        APS_ERR("[#23]stk3x1x_cci_als_cali() ALS out of range, als_reading = %d\n", als_reading);
        return -1;
        }

    // Count transmittance value temperary (Avoid decimal)
    cci_transmittance_cali = (als_reading*cci_transmittance_cali)/500;//   (500*cci_transmittance_cali)/als_reading;
    cci_als_adc_cali = als_reading;
    if(cci_transmittance_cali == 0)
        cci_transmittance_cali =900;
    APS_ERR("[#23]stk3x1x_cci_als_cali() cci_transmittance_cali = %d, als_reading = %d\n", cci_transmittance_cali, cci_als_adc_cali);

    return 1;
}
//CCI Colby add end 20131022
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
    
    ret = stk3x1x_master_send(client, obj->addr.alsctrl, &data, 1);
	if(ret < 0)
	{
		APS_ERR("write als = %d\n", ret);
		return -EFAULT;
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
//STK add for checking if the register value is ok 20130318 start
/*----------------------------------------------------------------------------*/
int stk3x1x_read_wait(struct i2c_client *client, u8 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
	u8 buf=0;

	if(NULL == client)
	{
		return -EINVAL;
	}
	ret = hwmsen_read_block(client, obj->addr.wait, &buf, 0x01);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		*data = buf;
	}

	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("PS NF flag: 0x%04X\n", (u32)(*data));
	}

	return 0;
}
//STK add for checking if the register value is ok 20130318 end


/*----------------------------------------------------------------------------*/
int stk3x1x_read_flag(struct i2c_client *client, u8 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf=0;
	
	if(NULL == client)
	{
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, obj->addr.flag, &buf, 0x01);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		*data = buf;
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("PS NF flag: 0x%04X\n", (u32)(*data));
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_id(struct i2c_client *client)
{
	int ret = 0;
	u8 buf[2]={0,0};
	
	if(NULL == client)
	{
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, STK_PDT_ID_REG, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	APS_LOG("%s: PID=0x%x, VID=0x%x\n", __func__, buf[0], buf[1]);
	
	if(buf[1] == 0xC0)
		APS_LOG( "%s: RID=0xC0!\n", __func__);		
	
	switch(buf[0])
	{
		case STK2213_PID:
		case STK2213I_PID:
		case STK3010_PID:
		case STK3210_STK3310_PID:
		case STK3211_STK3311_PID:
		case STK3311_WV_PID:	
		case STK3311_R35A:
			return buf[0];
		case 0x0:
			APS_ERR("PID=0x0, please make sure the chip is stk3x1x!\n");
			return 0;     
		default:
			APS_ERR( "%s: invalid PID(%#x)\n", __func__, buf[0]);	
			return 0;     
	}	
	return 0;     
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_ps(struct i2c_client *client, u16 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2]={0,0};
	
	if(NULL == client)
	{
		APS_ERR("i2c client is NULL\n");
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, obj->addr.data1_ps, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		*data = (buf[0] << 8) | (buf[1]);
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("PS: 0x%04X\n", (u32)(*data));
	}
	
	return 0;     
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.psctrl, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_wait(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.wait, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write wait = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_int(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;
    ret = stk3x1x_master_send(client, obj->addr.intmode, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write intmode = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_state(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.state, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write state = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_flag(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3x1x_master_send(client, obj->addr.flag, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_sw_reset(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf = 0, r_buf = 0;	
	int ret = 0;

	buf = 0x7F;
    ret = stk3x1x_master_send(client, obj->addr.wait, (char*)&buf, sizeof(buf));
	if (ret < 0)
	{
		APS_ERR("i2c write test error = %d\n", ret);
		return -EFAULT;
	} 	
	
    ret = stk3x1x_master_recv(client, obj->addr.wait, &r_buf, 1);
	if (ret < 0)
	{
		APS_ERR("i2c read test error = %d\n", ret);
		return -EFAULT;
	}	
	
	if(buf != r_buf)
	{
        APS_ERR("i2c r/w test error, read-back value is not the same, write=0x%x, read=0x%x\n", buf, r_buf);		
		return -EIO;
	}
	
	buf = 0;
    ret = stk3x1x_master_send(client, obj->addr.soft_reset, (char*)&buf, sizeof(buf));
	if (ret < 0)
	{
		APS_ERR("write software reset error = %d\n", ret);
		return -EFAULT;
	} 
	msleep(1);
	return 0;   
}

/* 20161025 start ESD check start */

int stk3x1x_init_all_reg(struct i2c_client *client)
{
        struct stk3x1x_priv *obj = i2c_get_clientdata(client);
        int err_init = 0;
        u8 ps_ctrl=0;

        if((err_init = stk3x1x_write_state(client, atomic_read(&obj->state_val)))) {
                APS_ERR("write stete error: %d\n", err_init);
                return err_init;
        }
        ps_ctrl = (u8)atomic_read(&obj->psctrl_val);
        if(obj->hw->polling_mode_ps == 1) {
                ps_ctrl = (u8)atomic_read(&obj->psctrl_val);
                ps_ctrl &= 0x3F;
        }
        if((err_init = stk3x1x_write_ps(client, ps_ctrl))) {
                APS_ERR("write ps error: %d\n", err_init);
                return err_init;
        }
        if((err_init = stk3x1x_write_als(client, atomic_read(&obj->alsctrl_val)))) {
                APS_ERR("write als error: %d\n", err_init);
                return err_init;
        }
        if((err_init = stk3x1x_write_led(client, obj->ledctrl_val))) {
                APS_ERR("write led error: %d\n", err_init);
                return err_init;
        }
        if((err_init = stk3x1x_write_wait(client, obj->wait_val))) {
                APS_ERR("write wait error: %d\n", err_init);
                return err_init;
        }
#ifndef STK_TUNE0
        if((err_init = stk3x1x_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val)))) {
                APS_ERR("write high thd error: %d\n", err_init);
                return err_init;
        }
        if((err_init = stk3x1x_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val)))) {
                APS_ERR("write low thd error: %d\n", err_init);
		return err_init;
	}
#endif
        if((err_init = stk3x1x_write_int(client, obj->int_val))) {
                APS_ERR("write int mode error: %d\n", err_init);
                return err_init;
        }

        return 0;
}

static int stk3x1x_validate_n_handle(struct i2c_client *client)
{
        /* struct stk3x1x_priv *obj = i2c_get_clientdata(client); */
        int err=0;

	APS_LOG("%s: stk software reset start \n", __FUNCTION__);
        if((err = stk3x1x_write_sw_reset(client))) { /* do sw_reset */
                APS_ERR("%s: stk software reset error, err=%d", __FUNCTION__, err);
                return err;
        }
        err = stk3x1x_read_id(client);

        if(err < 0)
                return err;
        else if(err==0)
                printk("[stk] check sensor type!\n");

	APS_LOG("%s: stk Init_all_reg start \n", __FUNCTION__);
        if((err = stk3x1x_init_all_reg(client))) { /* do sw_init */
                APS_ERR("%s: stk software init error, err=%d", __FUNCTION__, err);
                return err;
        }
        return 0;
}

/* 20161025 start ESD check end */
/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps_high_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2]={0,0};
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & thd) >> 8);
    buf[1] = (u8) (0x00FF & thd);	
    ret = stk3x1x_master_send(client, obj->addr.thdh1_ps, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %d\n",  ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.thdh2_ps, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %d\n", ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps_low_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2]={0,0};
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & thd) >> 8);
    buf[1] = (u8) (0x00FF & thd);	
    ret = stk3x1x_master_send(client, obj->addr.thdl1_ps, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.thdl2_ps, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als_high_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2]={0,0};
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & thd) >> 8);
    buf[1] = (u8) (0x00FF & thd);	
    ret = stk3x1x_master_send(client, obj->addr.thdh1_als, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.thdh2_als, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als_low_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2]={0,0};
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & thd) >> 8);
    buf[1] = (u8) (0x00FF & thd);	
    ret = stk3x1x_master_send(client, obj->addr.thdl1_als, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.thdl2_als, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
#if 0
int stk3x1x_write_foffset(struct i2c_client *client, u16 ofset)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
    buf[0] = (u8) ((0xFF00 & ofset) >> 8);
    buf[1] = (u8) (0x00FF & ofset);	
    ret = stk3x1x_master_send(client, obj->addr.data1_offset, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, obj->addr.data2_offset, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;   	
}

/*----------------------------------------------------------------------------*/

int stk3x1x_write_aoffset(struct i2c_client *client,  u16 ofset)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	u8 s_buf = 0, re_en;
    ret = stk3x1x_master_recv(client, obj->addr.state, &s_buf, 1);
	if (ret < 0)
	{
		APS_ERR("i2c read state error = %d\n", ret);
		return -EFAULT;
	}		
	re_en = (s_buf & STK_STATE_EN_AK_MASK) ? 1: 0;
	if(re_en)
	{
		s_buf &= (~STK_STATE_EN_AK_MASK); 		
		ret = stk3x1x_master_send(client, obj->addr.state, &s_buf, 1);
		if (ret < 0)
		{
			APS_ERR("write state = %d\n", ret);
			return -EFAULT;
		} 			
		msleep(3);		
	}	

    buf[0] = (u8) ((0xFF00 & ofset) >> 8);
    buf[1] = (u8) (0x00FF & ofset);	
    ret = stk3x1x_master_send(client, 0x0E, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
    ret = stk3x1x_master_send(client, 0x0F, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	if(!re_en)
		return 0;
	s_buf |= STK_STATE_EN_AK_MASK; 		
	ret = stk3x1x_master_send(client, obj->addr.state, &s_buf, 1);
	if (ret < 0)
	{
		APS_ERR("write state = %d\n", ret);
		return -EFAULT;
	} 			
	return 0;  	
}
#endif
/*----------------------------------------------------------------------------*/
/*static void stk3x1x_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "stk3x1x")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "stk3x1x")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}*/

/*----------------------------------------------------------------------------*/
static int stk3x1x_enable_als(struct i2c_client *client, int enable)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err=0, cur = 0, old=0;
	int trc = atomic_read(&obj->trace);
	mutex_lock(&stk3x1x_obj->stk3x1x_op_mutex);
	old = atomic_read(&obj->state_val);
	APS_LOG("%s: enable=%d, state_val = %d \n", __func__, enable,old);
	cur = old & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK)); 
	if(enable)
	{
		cur |= STK_STATE_EN_ALS_MASK;
	      /*Nina modify 2015/12/8 for state correct start*/
		if(!(cur & STK_STATE_EN_PS_MASK))
			cur |= STK_STATE_EN_WAIT_MASK;
 		/*Nina modify 2015/12/8 for state correct end*/	
	}
	else
	{	if(cur & STK_STATE_EN_PS_MASK)
			cur |= STK_STATE_EN_WAIT_MASK;   
	}
	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}
	
	if(0 == (cur ^ old))
	{
		APS_ERR("light sensor state val : cur = %d, old = %d\n", cur, old);
		mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
		return 0;
	}
		
#ifdef STK_IRS		
	if(enable && !(old & STK_STATE_EN_PS_MASK))
	{		
		err =  stk3x1x_get_ir_value(obj);
		if(err > 0)
			obj->ir_code = err;
	}			
#endif
		
	if(enable && obj->hw->polling_mode_als == 0)
	{
		stk3x1x_write_als_high_thd(client, 0x0);
		stk3x1x_write_als_low_thd(client, 0xFFFF);
	}
	err = stk3x1x_write_state(client, cur);
	if(err < 0){
		APS_ERR("light sensor write_state err = %d\n", err);
		mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
		return err;
	}
	else
		atomic_set(&obj->state_val, cur);
	
	if(enable)
	{
		if(obj->hw->polling_mode_als)
		{
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)*HZ/1000);
		}
		else
		{
			//set_bit(STK_BIT_ALS,  &obj->pending_intr);
			schedule_work(&obj->eint_work); //,220*HZ/1000
		}
	}

	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("enable als (%d)\n", enable);
	}
	mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_enable_ps(struct i2c_client *client, int enable)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err=0, cur = 0, old=0 ,i=0 ,depth=0;
	int trc = atomic_read(&obj->trace);
	int32_t near_far_state;	
	//STK add for checking if the register value is ok 20130318 start
	u8  reg_wait=0;
	int err_wait=0, err_init=0;
	//STK add for checking if the register value is ok 20130318 end
	//STK add for CCI start 20130430 dust issue for variation in 2 consequential calls 
	unsigned int hang_up_hthd=0, hang_up_lthd=0, fac_ct=0;
	int ret=0;
	u8 rbuf[4]={0,0,0,0};
	mutex_lock(&stk3x1x_obj->stk3x1x_op_mutex);
	old = atomic_read(&obj->state_val);
	fac_ct = cci_result_ct;// 2*cci_ps_low_thd-cci_ps_high_thd;
	//STK add for CCI end 20130430 dust issue for variation in 2 consequential calls 
	cur = old;	

#ifdef STK_TUNE0
	APS_LOG("%s: stk3x1x_enable_ps() enter enable=%d,hwid = %d \n", __FUNCTION__, enable, hwid);
// CCI add First boot H/L thd for debug start
	if(enable){
		APS_LOG("%s: stk3x1x_enable_ps() show first boot H/L THD => FIRST_BOOT_H_THD = %d, FIRST_BOOT_L_THD = %d\n", __FUNCTION__, FIRST_BOOT_H_THD, FIRST_BOOT_L_THD);
		APS_LOG("%s: stk3x1x_enable_ps() show factory H/L THD => cci_ps_high_thd = %d, cci_ps_low_thd = %d, cci_result_ct = %d\n", __FUNCTION__, cci_ps_high_thd, cci_ps_low_thd, cci_result_ct);
		}
// CCI add First boot H/L thd for debug end
// CCI read H_thd/L_thd when PS enable/disable start
	
	//int ret;
	ret = stk3x1x_master_recv(client, 6, &rbuf[0], 4);
	if(ret < 0){
		disable_irq(obj->irq);
		//mt_eint_mask(CUST_EINT_ALS_NUM);
		APS_DBG("error: %d\n", ret);
		mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
		return ret;
		}
	APS_LOG("%s: stk3x1x_enable_ps() read H/L THD from register before auto cal => H_THD = %2X, %2X, L_THD = %2X, %2X\n", __FUNCTION__, rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
// CCI read H_thd/L_thd when PS enable/disable end
	if (!(obj->psi_set) && !enable)
	{
		hrtimer_cancel(&obj->ps_tune0_timer);
		cancel_work_sync(&obj->stk_ps_tune0_work);
	}

	//STK add for checking if the register value is ok 20130318 start
	if(enable)
	{
		err_wait = 0;
		err_init = 0;
		reg_wait = 0x00;
		err_wait = stk3x1x_read_wait(client, &reg_wait);
		APS_ERR("[Colby]stk3x1x_enable_ps() reg_wait = %d\n", reg_wait);
		if(err_wait < 0)
		{
			APS_ERR("stk3x1x get wait value: %d\n", err);
			mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
			return err_wait;
		}
		else if (reg_wait != obj->wait_val)
		{
			//1)reset
			//2)initialize
			/* if((err_init = stk3x1x_init_client(client))) */
			if((err_init = stk3x1x_validate_n_handle(client))) /* 20161025 add ESD check */
			{
				APS_ERR("stk3x1x init (PS_ENABLE for ESD reset) failed: %d\n", err);
				mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
				return err_init;
			}
		}
	}
	//STK add for checking if the register value is ok 20130318 end

	if(obj->first_boot == true)
	{		
		obj->first_boot = false;
/*
		atomic_set(&obj->ps_high_thd_val, 0xFFFF); 
		atomic_set(&obj->ps_low_thd_val, 0); 					
		obj->psa = 0;
		obj->psi = 0xFFFF;
		
		if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", err);
			return err;        
		}
		
		if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", err);
			return err;        
		}
*/
	}
#else
	if(obj->first_boot == true)
	{			
		obj->first_boot = false;
/*		
		atomic_set(&obj->ps_high_thd_val, obj->hw->ps_high_thd_val ); 
		atomic_set(&obj->ps_low_thd_val, obj->hw->ps_low_thd_val ); 
		
		if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", err);
			return err;        
		}
		
		if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", err);
			return err;        
		}		
*/		
	}			
#endif

	APS_LOG("%s: enable=%d\n", __FUNCTION__, enable);	
	cur &= (~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | STK_STATE_EN_AK_MASK)); 
	if(enable)
	{
		cur |= (STK_STATE_EN_PS_MASK);
		if(!(old & STK_STATE_EN_ALS_MASK))
			cur |= STK_STATE_EN_WAIT_MASK;
		if(1 == obj->hw->polling_mode_ps)
			wake_lock(&ps_lock);
	}
	else
	{
		/*Nina modify 2015/12/8 for state correct start*/
		if(old & STK_STATE_EN_ALS_MASK)
			cur |= STK_STATE_EN_WAIT_MASK;
		/*Nina modify 2015/12/8 for state correct end*/
		if(1 == obj->hw->polling_mode_ps)		
			wake_unlock(&ps_lock);
	}
	
	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}
	
	if(0 == (cur ^ old))
	{
		APS_ERR("p sensor state val : cur = %d, old = %d\n", cur, old);
		mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
		return 0;
	}
	
	err = stk3x1x_write_state(client, cur);// enable ps by change state
	if(err < 0){
		APS_ERR("p sensor write_state err = %d\n", err);
		mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
		return err;
	}
	else
		atomic_set(&obj->state_val, cur);

	if(enable)
	{

		/*2017/1/17 depth check start*/
		depth= check_irqdepth(obj->irq);

		if(depth == 1){
			enable_irq(obj->irq);
		}
		else if(depth > 1){
			APS_ERR("p sensor unblance disable too much depth=%d,modify it\n",depth);
			for(i=0;i<depth;i++){
				enable_irq(obj->irq);
				msleep(10);
			}
		}
		else if(depth < 0){
			APS_ERR("p sensor unblance enable too much depth=%d,modify it\n",depth);
			for(i=0;i<depth;i++){
				disable_irq_nosync(obj->irq);
				msleep(10);
			}
		}

		/*2017/1/17 depth check end*/
#ifdef STK_TUNE0		
		obj->psi_set = 0;
		obj->psa = 0;
		obj->psi = 0xFFFF;
		if (!(obj->psi_set))
			hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay, HRTIMER_MODE_REL);			
#endif		
		if(obj->hw->polling_mode_ps)
		{
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)*HZ/1000);
		}
		else
		{
			msleep(4);
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
			{
				APS_ERR("stk3x1x read ps data: %d\n", err);
				mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
				return err;
			}
			
			err = stk3x1x_get_ps_value_only(obj, obj->ps);
			if(err < 0)
			{
				APS_ERR("stk3x1x get ps value: %d\n", err);
				mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
				return err;
			}
			else if(stk3x1x_obj->hw->polling_mode_ps == 0)
			{	
				near_far_state= err;
				/*sensor_data.values[0] = err;
				sensor_data.value_divide = 1;
				sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;*/
				APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps,near_far_state);
				err = ps_report_interrupt_data(near_far_state);
				if(err!=0)//hwmsen_get_interrupt_data
				{	
					APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
				}			
			}			
		}
	}
	else //STK add for CCI start 20130430 dust issue for variation in 2 consequential calls 
	{
		hang_up_hthd = atomic_read(&stk3x1x_obj->ps_high_thd_val);
		hang_up_lthd = atomic_read(&stk3x1x_obj->ps_low_thd_val);
		if(cci_ps_low_thd==0 || cci_ps_high_thd==0 || cci_ps_high_thd>65535 || cci_ps_low_thd >65535)
		{
			//if((2*fac_ct-STK_LT_N_CT) > hang_up_hthd)
			//{
			//	APS_ERR("%s: [Colby] use STK default value\n", __FUNCTION__);		
			//	atomic_set(&obj->ps_high_thd_val, 2*fac_ct-STK_LT_N_CT); 
			//	atomic_set(&obj->ps_low_thd_val, 2*fac_ct-STK_HT_N_CT); 		
			//}
			//else
			//{
				APS_ERR("%s: [Colby] use STK default value\n", __FUNCTION__);		
				atomic_set(&obj->ps_high_thd_val, hang_up_hthd); 
				atomic_set(&obj->ps_low_thd_val, hang_up_lthd);
			//}
		}
		else
		{

			if((2*fac_ct+(cci_ps_high_thd-cci_result_ct)-offset_H) > hang_up_hthd)
			{
				printk(KERN_ERR "%s: use CCI calibration value: HT = %d, LT = %d CT = %d\n", __func__, 2*fac_ct + (cci_ps_high_thd-cci_result_ct)-offset_H, 2*fac_ct + (cci_ps_low_thd-cci_result_ct)-offset_L,fac_ct);
				atomic_set(&stk3x1x_obj->ps_high_thd_val, 2*fac_ct + (cci_ps_high_thd-cci_result_ct)-offset_H); 
				atomic_set(&stk3x1x_obj->ps_low_thd_val, 2*fac_ct + (cci_ps_low_thd-cci_result_ct)-offset_L);
			}
			else
			{
				APS_ERR("%s: [Colby] use hang_up_hthd = %d, hang_up_lthd = %d\n", __FUNCTION__, hang_up_hthd, hang_up_lthd);
				atomic_set(&stk3x1x_obj->ps_high_thd_val, hang_up_hthd); 
				atomic_set(&stk3x1x_obj->ps_low_thd_val, hang_up_lthd);
			}
			
		}
				
		if((ret = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", ret);
			mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
			return ret;        
		}		
		if((ret = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", ret);
			mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
			return ret;        
		}	
		disable_irq(obj->irq);
	}
	//STK add for CCI end 20130430 dust issue for variation in 2 consequential calls 

	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("enable ps  (%d)\n", enable);
	}
	mutex_unlock(&stk3x1x_obj->stk3x1x_op_mutex);
	return err;
}
/*----------------------------------------------------------------------------*/

static int stk3x1x_check_intr(struct i2c_client *client, u8 *status) 
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err=0;

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	err = stk3x1x_read_flag(client, status);	
	if (err < 0)
	{
		APS_ERR("WARNING: read flag reg error: %d\n", err);
		return -EFAULT;
	}
	APS_LOG("%s: read status reg: 0x%x\n", __func__, *status);
    
	if(*status & STK_FLG_ALSINT_MASK)
	{
		set_bit(STK_BIT_ALS, &obj->pending_intr);
	}
	else
	{
	   clear_bit(STK_BIT_ALS, &obj->pending_intr);
	}
	
	if(*status & STK_FLG_PSINT_MASK)
	{
		set_bit(STK_BIT_PS,  &obj->pending_intr);
	}
	else
	{
	    clear_bit(STK_BIT_PS, &obj->pending_intr);
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_DEBUG)
	{
		APS_LOG("check intr: 0x%02X => 0x%08lX\n", *status, obj->pending_intr);
	}

	return 0;
}


static int stk3x1x_clear_intr(struct i2c_client *client, u8 status, u8 disable_flag) 
{
    int err = 0;

    status = status | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
    status &= (~disable_flag);
	APS_LOG(" set flag reg: 0x%x\n", status);
	if((err = stk3x1x_write_flag(client, status)))
		APS_ERR("stk3x1x_write_flag failed, err=%d\n", err);
    return err;
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_set_als_int_thd(struct i2c_client *client, u16 als_data_reg) 
{
	s32 als_thd_h=0, als_thd_l=0;	
		
    als_thd_h = als_data_reg + STK_ALS_CODE_CHANGE_THD;
    als_thd_l = als_data_reg - STK_ALS_CODE_CHANGE_THD;
    if (als_thd_h >= (1<<16))
        als_thd_h = (1<<16) -1;
    if (als_thd_l <0)
        als_thd_l = 0;
	APS_LOG("stk3x1x_set_als_int_thd:als_thd_h:%d,als_thd_l:%d\n", als_thd_h, als_thd_l);	
		
	stk3x1x_write_als_high_thd(client, als_thd_h);
	stk3x1x_write_als_low_thd(client, als_thd_l);

	return 0;
}

#ifdef STK_TUNE0	
static int stk_tune_zero_set_fac_ct(struct stk3x1x_priv *obj)
{
	unsigned int fac_ct=0;
	int err=0;

	fac_ct = cci_result_ct;// 2*cci_ps_low_thd-cci_ps_high_thd;		
	//if((obj->psi_set - fac_ct) > STK_COVER_LIMIT)
	//STK add for CCI start 20130430 dust issue for variation in 2 consequential calls 
	//if(obj->psi_set > (2 * fac_ct))
	//{
		//obj->psi_set = fac_ct;
		obj->psi_set = 2 * fac_ct;							
//CCI add to avoid H/L thd < 0 start
	if(cci_ps_high_thd - fac_ct >= (fac_ct - 50)){
		atomic_set(&obj->ps_high_thd_val, obj->psi_set + (cci_ps_high_thd -fac_ct)); 
		atomic_set(&obj->ps_low_thd_val, obj->psi_set + (cci_ps_low_thd - fac_ct));
		// CCI add First boot H/L thd for debug start
		FIRST_BOOT_H_THD = obj->psi_set + (cci_ps_high_thd -fac_ct);
		FIRST_BOOT_L_THD = obj->psi_set + (cci_ps_low_thd - fac_ct);
		// CCI add First boot H/L thd for debug end
		APS_LOG("%s: [CCI] (cci_ps_high_thd - fac_ct) >= (fac_ct - 50)", __func__);
	}
//CCI add to avoid H/L thd < 0 start
	else{
		atomic_set(&obj->ps_high_thd_val, obj->psi_set - (cci_ps_low_thd -fac_ct)); 
		atomic_set(&obj->ps_low_thd_val, obj->psi_set - (cci_ps_high_thd - fac_ct)); 		
		// CCI add First boot H/L thd for debug start
		FIRST_BOOT_H_THD = obj->psi_set - (cci_ps_low_thd -fac_ct);
		FIRST_BOOT_L_THD = obj->psi_set - (cci_ps_high_thd - fac_ct);
		// CCI add First boot H/L thd for debug end
	}
		if((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", err);
			return err;        
		}	
		if((err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", err);
			return err;        
		}	
		APS_LOG("%s: set HT=%d,LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val),  atomic_read(&obj->ps_low_thd_val));					
	//}
	//STK add for CCI start 20130430 dust issue for variation in 2 consequential calls 
	return 0;	
}

static int stk_ps_tune_zero_final(struct stk3x1x_priv *obj)
{
	int err=0;
	
	obj->tune_zero_init_proc = false;
	if((err = stk3x1x_write_int(obj->client, obj->int_val)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}	
	
	if((err = stk3x1x_write_state(obj->client, atomic_read(&obj->state_val))))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}		
	
	obj->psa = obj->ps_stat_data[0];
	obj->psi = obj->ps_stat_data[2];							
	obj->psi_set = obj->ps_stat_data[1];							
	atomic_set(&obj->ps_high_thd_val, obj->ps_stat_data[1] + STK_HT_N_CT); 
	atomic_set(&obj->ps_low_thd_val, obj->ps_stat_data[1] + STK_LT_N_CT); 		
	// CCI add First boot H/L thd for debug start
	FIRST_BOOT_H_THD = obj->ps_stat_data[1] + STK_HT_N_CT;
	FIRST_BOOT_L_THD = obj->ps_stat_data[1] + STK_LT_N_CT;

	// CCI add First boot H/L thd for debug end
	if((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;        
	}	
	if((err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;        
	}	
	APS_LOG("%s: set HT=%d,LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val),  atomic_read(&obj->ps_low_thd_val));		
	hrtimer_cancel(&obj->ps_tune0_timer);					
	return 0;
}

static int32_t stk_tune_zero_get_ps_data(struct stk3x1x_priv *obj)
{
	int err=0;
	
	if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
	{
		APS_ERR("stk3x1x read ps data: %d\n", err);
		return err;
	}	
	APS_LOG("%s: ps #%d=%d\n", __func__, obj->data_count, obj->ps);
	
	obj->ps_stat_data[1]  +=  obj->ps;			
	if(obj->ps > obj->ps_stat_data[0])
		obj->ps_stat_data[0] = obj->ps;
	if(obj->ps < obj->ps_stat_data[2])
		obj->ps_stat_data[2] = obj->ps;						
	obj->data_count++;	
	
	if(obj->data_count == 5)
	{
		obj->ps_stat_data[1]  /= obj->data_count;			
		stk_ps_tune_zero_final(obj);
	}		
	
	return 0;
}

static int stk_ps_tune_zero_init(struct stk3x1x_priv *obj)
{
	u8 w_state_reg=0;	
	int err=0;
	
	obj->psi_set = 0;	
	obj->tune_zero_init_proc = true;		
	obj->ps_stat_data[0] = 0;
	obj->ps_stat_data[2] = 9999;
	obj->ps_stat_data[1] = 0;
	obj->data_count = 0;
	
	if((err = stk3x1x_write_int(obj->client, 0)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}	
	
	w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);			
	if((err = stk3x1x_write_state(obj->client, w_state_reg)))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}			
	hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay, HRTIMER_MODE_REL);		
	return 0;	
}

static int stk3x1x_ps_tune_zero_val(void)
{
	int mode=0;
	int32_t word_data=0, lii=0;
	u8 buf[2]={0,0};
	int ret=0;
	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x20, buf, 2);
	if(ret < 0)	
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}
	word_data = (buf[0] << 8) | buf[1];
	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x22, buf, 2);
	if(ret < 0)		
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}	
	word_data += (buf[0] << 8) | buf[1];	
	
	//mode = atomic_read(&stk3x1x_obj->psctrl_val) & 0x3F;
	mode = 0x33 & 0x3F;
	if(mode == 0x30)	
		lii = 100;	
	else if (mode == 0x31)
		lii = 200;		
	else if (mode == 0x32)
		lii = 400;				
	else if (mode == 0x33)
		lii = 800;			
	else
	{
		APS_ERR("%s: unsupported PS_IT(0x%x)\n", __FUNCTION__, mode);
		return -1;
	}
	
	if(word_data > lii)	
	{
		APS_LOG( "%s: word_data=%d, lii=%d\n", __FUNCTION__, word_data, lii);		
		return 0xFFFF;	
	}
	return 0;
}

static int stk_ps_tune_zero_func_fae(struct stk3x1x_priv *obj)
{
	int32_t word_data=0;
	u8 flag=0;
	bool ps_enabled = false;
	u8 buf[2]={0,0};
	int ret, diff;
	//uint16_t MAX_CT;	
	//STK add for CCI start 20130112
	unsigned int fac_ct;
	//STK add for CCI end 20130112	
	
	//STK add for CCI start 20130430 dust issue for variation in 2 consequential calls 
	static int32_t temp_max_min_diff = 0;
	//STK add for CCI start 20130430 dust issue for variation in 2 consequential calls 
	
	ps_enabled = (atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK) ? true : false;	
	if(obj->psi_set || !(ps_enabled))
	{
		//APS_LOG("%s: FAE tune0 close\n", __func__);
		return 0;
	}	
	//APS_LOG("%s: FAE tune0 entry\n", __func__);
	
	ret = stk3x1x_read_flag(obj->client, &flag);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: get flag failed, err=0x%x\n", __func__, ret);
		return ret;
	}
	if(!(flag&STK_FLG_PSDR_MASK))
	{
		return 0;
	}
	
	ret = stk3x1x_ps_tune_zero_val();	
	if(ret == 0)
	{
		ret = stk3x1x_master_recv(obj->client, 0x11, buf, 2);
		if(ret < 0)
		{
			printk(KERN_ERR "%s fail, err=0x%x", __func__, ret);
			return ret;	   
		}
		word_data = (buf[0] << 8) | buf[1];
		APS_LOG("%s: word_data=%d\n", __func__, word_data);
		
		if(word_data == 0)
		{
			printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		printk("[stk] show psa psi: psa=%d,psi=%d\n", obj->psa, obj->psi);		
		if(word_data > obj->psa)
		{
			if(obj->psa == 0)
			{
				temp_max_min_diff = word_data>>3;
			}
			obj->psa = word_data;
			APS_LOG("[stk] update psa: psa=%d,psi=%d\n",obj->psa, obj->psi);
		}
		if(word_data < obj->psi)
		{
			obj->psi = word_data;	
			APS_LOG("[stk] update psi: psa=%d,psi=%d\n", obj->psa, obj->psi);	
		}	
	}	
	
	diff = obj->psa - obj->psi;
	if(diff > STK_MAX_MIN_DIFF && diff > temp_max_min_diff)
	{
		obj->psi_set = obj->psi;
		
		fac_ct = cci_result_ct;// 2*cci_ps_low_thd-cci_ps_high_thd;		
		//STK add for CCI start 20130112
		stk3x1x_obj->psi_set = MAX_COMPARE(stk3x1x_obj->psi_set, fac_ct);
		//STK add for CCI end 20130112
		
		//STK add for CCI start 20130112
		if(cci_ps_low_thd==0 || cci_ps_high_thd==0 ||cci_ps_low_thd>65535 ||cci_ps_high_thd>65535 || cci_result_ct>65535)
		{
			APS_ERR("%s: [Colby] use STK default delta value HT = %d LT = %d \n", __FUNCTION__,obj->psi_set + STK_HT_N_CT,obj->psi_set + STK_LT_N_CT);		
			atomic_set(&obj->ps_high_thd_val, obj->psi_set + STK_HT_N_CT); 
			atomic_set(&obj->ps_low_thd_val, obj->psi_set + STK_LT_N_CT); 		
		}
		else
		{
			//printk(KERN_INFO "%s:============START CHECK IF CT IS TOO BIG============\n", __func__);
			printk(KERN_INFO "%s:cci_ps_cover(%d) and 2*CT(%d)\n", __func__, cci_ps_cover, 2*cci_result_ct);
			//MAX_CT = MAX_COMPARE(cci_ps_cover, 2*cci_result_ct); Nina removed for VY36, cause grease and protector on CT is less than LT
			//printk(KERN_INFO "%s:MAX_CT = %d\n",  __func__, MAX_CT);

			printk(KERN_INFO "%s:MIN Compare psi_set(%d) and cci_ps_high_thd(%d)\n", __func__, obj->psi_set, cci_ps_high_thd);
			obj->psi_set = MIN_COMPARE(obj->psi_set, cci_ps_high_thd );
			printk(KERN_INFO "%s:The Result CT: psi_set = %d\n", __func__, obj->psi_set);
			//printk(KERN_INFO "%s:=========================END========================\n", __func__);

			APS_ERR(" [stk] HT = %d, LT = %d\n", obj->psi_set + cci_ps_high_thd-fac_ct-offset_H, obj->psi_set + cci_ps_low_thd-fac_ct-offset_L);
			atomic_set(&stk3x1x_obj->ps_high_thd_val, obj->psi_set + cci_ps_high_thd-fac_ct-offset_H); 
			atomic_set(&stk3x1x_obj->ps_low_thd_val, obj->psi_set + cci_ps_low_thd-fac_ct-offset_L); 
				
		}
		//STK add for CCI end 20130112					
		if((ret = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", ret);
			return ret;        
		}		
		if((ret = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", ret);
			return ret;        
		}	
#ifdef STK_DEBUG_PRINTF				
		APS_LOG("%s: FAE tune0 psa-psi(%d) > DIFF found\n", __func__, diff);
#endif					
		hrtimer_cancel(&obj->ps_tune0_timer);
	}
	
	//APS_LOG("%s: FAE tune0 exit\n", __func__);
	return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
	struct stk3x1x_priv *obj = container_of(work, struct stk3x1x_priv, stk_ps_tune0_work);		
	if(obj->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(obj);
	else
		stk_ps_tune_zero_func_fae(obj);
	return;
}	


static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct stk3x1x_priv *obj = container_of(timer, struct stk3x1x_priv, ps_tune0_timer);
	queue_work(obj->stk_ps_tune0_wq, &obj->stk_ps_tune0_work);	
	hrtimer_forward_now(&obj->ps_tune0_timer, obj->ps_tune0_delay);
	return HRTIMER_RESTART;	
}
	
#endif	/*#ifdef STK_TUNE0	*/

/*----------------------------------------------------------------------------*/
void stk3x1x_eint_func(void)
{
	struct stk3x1x_priv *obj = stk3x1x_obj;
	
	if(!obj)
	{
		return;
	}
	//schedule_work(&obj->eint_work);
	if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
		schedule_work(&obj->eint_work);
	if(atomic_read(&obj->trace) & STK_TRC_EINT)
	{
		APS_LOG("eint: als/ps intrs\n");
	}
}
/*----------------------------------------------------------------------------*/
static void stk3x1x_eint_work(struct work_struct *work)
{
	struct stk3x1x_priv *obj = stk3x1x_obj;
	int err=0;
	//hwm_sensor_data sensor_data;
	int32_t near_far_state;
	u8 flag_reg=0, disable_flag = 0;
	// CCI read H_thd/L_thd when PS interrupt trigger start
	u8 rbuf[4]={0,0,0,0};
	int ret=0;
	// CCI read H_thd/L_thd when PS interrupt trigger start
	//memset(&sensor_data, 0, sizeof(sensor_data));
	
	if((err = stk3x1x_check_intr(obj->client, &flag_reg)))
	{
		APS_ERR("stk3x1x_check_intr fail: %d\n", err);
		goto err_i2c_rw;
	}
	
	//add for FTM interrupt check 20130416 start
	//APS_LOG("CCI_FTM = %d\n", CCI_FTM);
	//if(CCI_FTM){
		if(obj->pending_intr == 4){
			CCI_FTM_INTERRUPT++;
			APS_LOG("CCI_FTM_INTERRUPT = %d\n", CCI_FTM_INTERRUPT);
			}
	//	}
	//add for FTM interrupt check 20130416 end

    APS_LOG(" &obj->pending_intr =%lx\n",obj->pending_intr);
	
	if(((1<<STK_BIT_ALS) & obj->pending_intr) && (obj->hw->polling_mode_als == 0))
	{
		//get raw data
		APS_LOG("stk als change\n");
		disable_flag |= STK_FLG_ALSINT_MASK;
		if((err = stk3x1x_read_als(obj->client, &obj->als)))
		{
			APS_ERR("stk3x1x_read_als failed %d\n", err);			
			goto err_i2c_rw;
		}
		
		err=stk3x1x_set_als_int_thd(obj->client, obj->als);
		near_far_state=stk3x1x_get_als_lux(obj, obj->als);
		/*sensor_data.values[0] = stk3x1x_get_als_value(obj, obj->als);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;*/
		if(CCI_debug_for_stk3x1x)
			APS_LOG("%s:als raw 0x%x -> value 0x%x \n", __FUNCTION__, obj->als,near_far_state);
		//let up layer to know
		if((err = ps_report_interrupt_data(near_far_state)))
		{
			APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}	  
	}
	if(((1<<STK_BIT_PS) &  obj->pending_intr) && (obj->hw->polling_mode_ps == 0))
	{
		disable_flag |= STK_FLG_PSINT_MASK;
		
		if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
		{
			APS_ERR("stk3x1x read ps data: %d\n", err);
			goto err_i2c_rw;
		}

		// CCI read H_thd/L_thd when PS interrupt trigger start
		ret = stk3x1x_master_recv(obj->client, 6, &rbuf[0], 4);
		if(ret < 0){
			APS_DBG("error: %d\n", ret);
			}
		APS_LOG("%s: stk3x1x_eint_work() read H/L THD from register when interrupt trigger => H_THD = %2X, %2X, L_THD = %2X, %2X\n", __FUNCTION__, rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
		// CCI read H_thd/L_thd when PS interrupt trigger end
		near_far_state=(flag_reg & STK_FLG_NF_MASK)? 1 : 0;
		/*sensor_data.values[0] = (flag_reg & STK_FLG_NF_MASK)? 1 : 0;
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;*/
		APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps,near_far_state);
		//let up layer to know
		if((err =ps_report_interrupt_data(near_far_state)))//hwmsen_get_interrupt_data
		{	
			APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
		
	}
	
	if((err = stk3x1x_clear_intr(obj->client, flag_reg, disable_flag)))
	{
		APS_ERR("fail: %d\n", err);
		goto err_i2c_rw;
	}		

	msleep(1);
   	enable_irq(obj->irq);
	return;
	
err_i2c_rw:	
	msleep(30);
	enable_irq(obj->irq);
	return;
}
static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	//printk("stk_oss_irq_handler disable irq\n");
	disable_irq_nosync(stk3x1x_obj->irq);
	queue_work(stk3x1x_obj->stk_wq,&stk3x1x_obj->eint_work);//g_stk3x1x_ptr
	return IRQ_HANDLED;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_setup_eint(struct i2c_client *client)
{	
	int err = -EIO;
	u32 ints[2] = { 0, 0 };
	struct stk3x1x_priv *ps_data = i2c_get_clientdata(client);   

	if(ps_data==NULL)
			printk("stk debug ps_data==NULL\n ");
	alspsPltFmDev = get_alsps_platformdev();
	if(alspsPltFmDev==NULL)
		{
		printk("stk alspsPltFmDev ==NULL \n");
		return 0;
		}

	ps_data->pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	
	
	if (IS_ERR_OR_NULL(ps_data->pinctrl)) {
		printk("stk Failed to get pinctrl\n");
		return PTR_ERR(ps_data->pinctrl);
		}

	ps_data->pin_default =	pinctrl_lookup_state(ps_data->pinctrl, "pin_default");
	if(ps_data->pin_default==NULL)
		printk("stk pin_default=NULL \n");
	else
		printk("stk pin_default not NULL\n");

	ps_data->pins_cfg = pinctrl_lookup_state(ps_data->pinctrl, "pin_cfg");

	if(ps_data->pins_cfg==NULL)
		printk("stk pins_cfg=NULL \n");
	else
		printk("stk pins_cfg not NULL\n");

	
	if (IS_ERR_OR_NULL(ps_data->pins_cfg)) {
		printk("stk Failed to look up pins_cfg state\n");
		return PTR_ERR(ps_data->pins_cfg);
	}
	else
		{printk("stk success to look up pins_cfg state\n");}

	err = pinctrl_select_state(ps_data->pinctrl, ps_data->pins_cfg);
	if (err) {
		printk("stk Can't select pinctrl default state\n");
		return err;
	}
	printk("stk ps_data->irq_node\n");
	if (!ps_data->irq_node) {
		printk("stk irq_node NULL \n");
		return -2;//EINVAL
		}
	else{	
		of_property_read_u32_array(ps_data->irq_node, "debounce", ints,ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		printk("stk ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		interrupt_gpio_num = ints[0];

		ps_data->irq = irq_of_parse_and_map(ps_data->irq_node, 0);
		printk("stk ps_data->irq = %d\n", ps_data->irq);
		if (!ps_data->irq) {
			printk("irq_of_parse_and_map fail!!\n");
			return -3; 
		}
		
		err = request_any_context_irq(ps_data->irq, stk_oss_irq_handler, IRQF_TRIGGER_LOW, "ALS-eint", ps_data); //stk_oss_irq_handler
		if (err < 0){
			printk(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n", __func__,ps_data-> irq, err);		
			return -4; //err_request_any_context_irq
		}
		enable_irq_wake(ps_data->irq);
		disable_irq(ps_data->irq);
		}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_init_client(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err=0;
	u8 ps_ctrl=0;
	//u8 int_status;
	
	if((err = stk3x1x_write_sw_reset(client)))
	{
		APS_ERR("software reset error, err=%d", err);
		return err;
	}

	err = stk3x1x_read_id(client);
	if(err < 0)
		return err;
	else if(err==0)
		printk("[stk] check sensor type!\n");	
	
	if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
	{		
		if((err = stk3x1x_setup_eint(client)))
		{
			APS_ERR("setup eint error: %d\n", err);
			printk("stk err  = %d\n", err);
			return err;
		}
	}
	
	if((err = stk3x1x_write_state(client, atomic_read(&obj->state_val))))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}	
	
	/*
	if((err = stk3x1x_check_intr(client, &int_status)))
	{
		APS_ERR("check intr error: %d\n", err);
		//    return err;
	}
	
	if((err = stk3x1x_clear_intr(client, int_status, STK_FLG_PSINT_MASK | STK_FLG_ALSINT_MASK)))
	{
		APS_ERR("clear intr error: %d\n", err);	
		return err;
	}
	*/
	ps_ctrl = (u8)atomic_read(&obj->psctrl_val);

	if(obj->hw->polling_mode_ps == 1)
	{
		ps_ctrl = (u8)atomic_read(&obj->psctrl_val);
		ps_ctrl &= 0x3F;
	}
	
	if((err = stk3x1x_write_ps(client, ps_ctrl)))
	{
		APS_ERR("write ps error: %d\n", err);
		return err;        
	}
	
	if((err = stk3x1x_write_als(client, atomic_read(&obj->alsctrl_val))))
	{
		APS_ERR("write als error: %d\n", err);
		return err;
	}	
	
	if((err = stk3x1x_write_led(client, obj->ledctrl_val)))
	{
		APS_ERR("write led error: %d\n", err);
		return err;
	}	
	
	if((err = stk3x1x_write_wait(client, obj->wait_val)))
	{
		APS_ERR("write wait error: %d\n", err);
		return err;
	}	
#ifndef STK_TUNE0	
	if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;        
	}
	
	if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;        
	}
#endif	
	if((err = stk3x1x_write_int(client, obj->int_val)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}	
#ifdef STK_FIR
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
#ifdef STK_TUNE0
	err=stk_ps_tune_zero_init(obj);
#endif	
	return 0;
}

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t stk3x1x_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res=0;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	res = scnprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d)\n", 
	 atomic_read(&stk3x1x_obj->i2c_retry), atomic_read(&stk3x1x_obj->als_debounce), 
	 atomic_read(&stk3x1x_obj->ps_mask), atomic_read(&stk3x1x_obj->ps_high_thd_val),atomic_read(&stk3x1x_obj->ps_low_thd_val), atomic_read(&stk3x1x_obj->ps_debounce));     
	return res;    
}

static ssize_t stk3x1x_store_config(struct device_driver *ddri, const char *buf,  size_t count)
{
	int retry=0, als_deb=0, ps_deb=0, mask=0, hthres=0, lthres=0, err=0;
	struct i2c_client *client;
	client = stk3x1x_i2c_client;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if(6 == sscanf(buf, "%d %d %d %d %d %d", &retry, &als_deb, &mask, &hthres, &lthres, &ps_deb))
	{ 
		atomic_set(&stk3x1x_obj->i2c_retry, retry);
		atomic_set(&stk3x1x_obj->als_debounce, als_deb);
		atomic_set(&stk3x1x_obj->ps_mask, mask);
		atomic_set(&stk3x1x_obj->ps_high_thd_val, hthres);    
		atomic_set(&stk3x1x_obj->ps_low_thd_val, lthres);        
		atomic_set(&stk3x1x_obj->ps_debounce, ps_deb);

		if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&stk3x1x_obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", err);
			return err;        
		}
		
		if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&stk3x1x_obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", err);
			return err;        
		}
	}
	else
	{
		APS_ERR("invalid content: %s \n", buf);
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res=0;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}

	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3x1x_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace=0;
    if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&stk3x1x_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;    
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_ir(struct device_driver *ddri, char *buf)
{
    int32_t reading=0;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
    reading = stk3x1x_get_ir_value(stk3x1x_obj);
	if(reading < 0)
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", reading);

	stk3x1x_obj->ir_code = reading;
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", stk3x1x_obj->ir_code);     
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_als(struct device_driver *ddri, char *buf)
{
	int res=0;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	if((res = stk3x1x_read_als(stk3x1x_obj->client, &stk3x1x_obj->als)))
	{
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "%d\n", stk3x1x_obj->als);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_ps(struct device_driver *ddri, char *buf)
{
	int res=0;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if((res = stk3x1x_read_ps(stk3x1x_obj->client, &stk3x1x_obj->ps)))
	{
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "%d\n", stk3x1x_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_reg(struct device_driver *ddri, char *buf)
{
	u8 int_status=0;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	/*read*/
	stk3x1x_check_intr(stk3x1x_obj->client, &int_status);
	//stk3x1x_clear_intr(stk3x1x_obj->client, int_status, 0x0);
	stk3x1x_read_ps(stk3x1x_obj->client, &stk3x1x_obj->ps);
	stk3x1x_read_als(stk3x1x_obj->client, &stk3x1x_obj->als);
	/*write*/
	stk3x1x_write_als(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->alsctrl_val));
	stk3x1x_write_ps(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->psctrl_val)); 
	stk3x1x_write_ps_high_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_high_thd_val));
	stk3x1x_write_ps_low_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_low_thd_val));
	return 0;
}


static ssize_t stk3x1x_show_hwid(struct device_driver *ddri, char *buf)
{
	APS_LOG("%s: hwid = %d\n", __FUNCTION__, hwid);
	
    return scnprintf(buf, PAGE_SIZE, "hwid = %d\n", hwid);     
}

/*
static ssize_t stk3x1x_show_tptype(struct device_driver *ddri, char *buf)
{
	switch(SENSOR_OPTION)
	{
		case 0x9999:
			printk("%s: TP type = HS_TR (DP1), TP option = %x\n", __FUNCTION__,SENSOR_OPTION);
			return scnprintf(buf, PAGE_SIZE, "TP type = HS_TR (DP1)\n");      
			break;
		case 0x00BA:
			printk("%s: TP type = CPT/PMMA-TR (DP2), TP option = %x\n", __FUNCTION__,SENSOR_OPTION);
			return scnprintf(buf, PAGE_SIZE, "TP type = CPT/PMMA-TR (DP2)\n");      
			break;
		case 0x00BE:
			printk("%s: TP type = CPT/Glass-TR (DP2), TP option = %x\n", __FUNCTION__,SENSOR_OPTION);
			return scnprintf(buf, PAGE_SIZE, "TP type = CPT/Glass-TR (DP2)\n");      
			break;
		default:
			printk("%s: TP type = Unknown\n", __FUNCTION__);
			return scnprintf(buf, PAGE_SIZE, "TP type = Unknown \n");      
			break;
	}
	
}



-*/
static ssize_t stk3x1x_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr=0, cmd=0;
	u8 dat=0;

	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	APS_LOG("send(%02X, %02X) = %d\n", addr, cmd, 
	stk3x1x_master_send(stk3x1x_obj->client, (u16)addr, &dat, sizeof(dat)));
	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_recv(struct device_driver *ddri, char *buf)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3x1x_obj->recv_reg));     	
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr=0;
	u8 dat=0;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	APS_LOG("recv(%02X) = %d, 0x%02X\n", addr, 
	stk3x1x_master_recv(stk3x1x_obj->client, (u16)addr, (char*)&dat, sizeof(dat)), dat);
	atomic_set(&stk3x1x_obj->recv_reg, dat);	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_allreg(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	u8 rbuf[27]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int cnt=0;	
	//ssize_t len = 0;
	
	memset(rbuf, 0, sizeof(rbuf));
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0, &rbuf[0], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 7, &rbuf[7], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 14, &rbuf[14], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 21, &rbuf[21], 4);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	
	for(cnt=0;cnt<25;cnt++)
	{
		//len += scnprintf(buf+len, PAGE_SIZE - len, "%x ", rbuf[cnt]);
		APS_LOG("reg[0x%x]=0x%x\n", cnt, rbuf[cnt]);
	}	
	
    return scnprintf(buf, PAGE_SIZE, "[0]%2X [1]%2X [2]%2X [3]%2X [4]%2X [5]%2X [6/7 HTHD]%2X,%2X [8/9 LTHD]%2X, %2X [A]%2X [B]%2X [C]%2X [D]%2X [E/F Aoff]%2X,%2X,[10]%2X [11/12 PS]%2X,%2X [13]%2X [14]%2X [15/16 Foff]%2X,%2X [17]%2X [18]%2X [3E]%2X [3F]%2X\n", 	
		rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4], rbuf[5], rbuf[6], rbuf[7], rbuf[8], 
		rbuf[9], rbuf[10], rbuf[11], rbuf[12], rbuf[13], rbuf[14], rbuf[15], rbuf[16], rbuf[17], 
		rbuf[18], rbuf[19], rbuf[20], rbuf[21], rbuf[22], rbuf[23], rbuf[24], rbuf[25], rbuf[26]);	
			
	//return len;
}

/*----------------------------------------------------------------------------
static ssize_t stk3x1x_show_status(struct device_driver *ddri, char *buf)
{
    int32_t ps_reg[27];
	uint8_t cnt;
	int ret = 0;
	
	for(cnt=0;cnt<25;cnt++)
	{
		ret = stk3x1x_master_recv(stk3x1x_obj->client, cnt, &ps_reg[cnt], 7);
		if(ret < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
			return -EINVAL;
		}
		else
		{
			printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ret = stk3x1x_master_recv(stk3x1x_obj->client, STK_PDT_ID_REG, &ps_reg[cnt], 7);
	//ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(stk3x1x_obj->client, STK_PDT_ID_REG);
	if(ret < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt++]);	
	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, STK_RSRVD_REG, &ps_reg[cnt], 7);
	//ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(stk3x1x_obj->client, STK_RSRVD_REG);
	if(ret< 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt++]);		

    return scnprintf(buf, PAGE_SIZE, "[PS=%2X] [ALS=%2X] [WAIT=0x%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n", 
		ps_reg[0]&0x01,(ps_reg[0]&0x02)>>1,((ps_reg[0]&0x04)>>2)*ps_reg[5]*6,(ps_reg[0]&0x20)>>5,
		(ps_reg[0]&0x40)>>6,ps_reg[16]&0x01,(ps_reg[16]&0x04)>>2,(ps_reg[16]&0x10)>>4,(ps_reg[16]&0x20)>>5);		
}

----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct stk3x1x_priv *obj, const char* buf, size_t count,
                             u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}

static ssize_t stk3x1x_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx=0;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < stk3x1x_obj->als_level_num; idx++)
	{
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", stk3x1x_obj->hw->als_level[idx]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}

static ssize_t stk3x1x_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(stk3x1x_obj->als_level, stk3x1x_obj->hw->als_level, sizeof(stk3x1x_obj->als_level));
	}
	else if(stk3x1x_obj->als_level_num != read_int_from_buf(stk3x1x_obj, buf, count, 
			stk3x1x_obj->hw->als_level, stk3x1x_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}

static ssize_t stk3x1x_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx=0;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < stk3x1x_obj->als_value_num; idx++)
	{
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", stk3x1x_obj->hw->als_value[idx]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}

static ssize_t stk3x1x_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(stk3x1x_obj->als_value, stk3x1x_obj->hw->als_value, sizeof(stk3x1x_obj->als_value));
	}
	else if(stk3x1x_obj->als_value_num != read_int_from_buf(stk3x1x_obj, buf, count, 
			stk3x1x_obj->hw->als_value, stk3x1x_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}

#ifdef STK_TUNE0

static ssize_t stk3x1x_store_cali(struct device_driver *ddri, const char *buf,size_t size)
{
	 // add first H/L thd when cci_result_ct get start
	//struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);
	u16 cci_result_ct_x2 = 0;
	u16 low_thd=0,high_thd=0;
	 // add first H/L thd when cci_result_ct get end
	unsigned long value = 0;
	int ret, i;	
	char *token[10];
	//struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	
#ifdef LightSensorK	
	for (i = 0; i < 9; i++)
#else
	for (i = 0; i < 5; i++)
#endif
	token[i] = strsep((char **)&buf, " ");
	if((ret = kstrtoul(token[0], 16, &value)) < 0)//strict_strtoul
	{
		printk(KERN_ERR "%s:[STK]kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	if(hwid ==  BY57_hwid)
		cci_ps_high_thd = value + 80;
	else
		cci_ps_high_thd = value;
	
	if((ret = kstrtoul(token[1], 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:[STK]kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	if(hwid ==  BY57_hwid)
		cci_ps_low_thd = value + 80;
	else
		cci_ps_low_thd = value;
	
	printk(KERN_INFO "%s [STK]cci_ps_high_thd = %d, cci_ps_low_thd = %d", __FUNCTION__, cci_ps_high_thd, cci_ps_low_thd);
	
	if((ret = kstrtoul(token[2], 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:[STK]kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}	
	cci_result_ct = value;

	// add first H/L thd when cci_result_ct get start
	cci_result_ct_x2 = cci_result_ct * 2;
	printk(KERN_INFO "%s: cci_result_ct_x2 = %d, cci_result_ct = %d\n", __func__, cci_result_ct_x2, cci_result_ct);	
	if(cci_result_ct_x2 >= 0xFFFF)
		cci_result_ct_x2 = 15000;

	printk(KERN_INFO "%s: original  HT=%d, oriinal  LT=%d, CT=%d, offset_H=%d, offset_L=%d\n", __func__, (cci_result_ct_x2 + (cci_ps_high_thd - cci_result_ct)),   (cci_result_ct_x2 + (cci_ps_low_thd - cci_result_ct)),cci_result_ct ,offset_H,offset_L);				
	
	atomic_set(&stk3x1x_obj->ps_high_thd_val, cci_result_ct_x2 +(cci_ps_high_thd - cci_result_ct)-offset_H);
	atomic_set(&stk3x1x_obj->ps_low_thd_val, cci_result_ct_x2 + (cci_ps_low_thd - cci_result_ct)-offset_L);
	high_thd=cci_result_ct_x2 + (cci_ps_high_thd - cci_result_ct)-offset_H;
	low_thd=cci_result_ct_x2 + (cci_ps_low_thd - cci_result_ct)-offset_L;
	printk(KERN_INFO "%s: set HT=%d,LT=%d according to cci_result_ct_x2\n", __func__, (cci_result_ct_x2 + (cci_ps_high_thd - cci_result_ct)-offset_H),  (cci_result_ct_x2 + (cci_ps_low_thd - cci_result_ct))-offset_L);				

	ret = stk3x1x_write_ps_high_thd(stk3x1x_obj->client, high_thd);
	if(ret!=0)
	{
		APS_ERR("write high thd error: %d\n", ret);
		return ret;        
	}	
	ret = stk3x1x_write_ps_low_thd(stk3x1x_obj->client, low_thd);
	if(ret!=0)
	{
		APS_ERR("write low thd error: %d\n", ret);
		return ret;        
	}			
	// add first H/L thd when cci_result_ct get end

	if((ret = kstrtoul(token[3], 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:[STK]kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	cci_ps_cover = value;
	
	if((ret = kstrtoul(token[4], 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:[STK]kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}	
	cci_als_value = value;	

	printk(KERN_INFO "%s: cci_als_value = %d\n", __func__, cci_als_value);


	if((ret = kstrtoul(token[5], 16, &value)) < 0)
        {
                printk(KERN_ERR "%s:[STK]strict_strtoul failed, ret=0x%x\n", __func__, ret);
                return ret;
        }
        cci_als_adc_test = value;

        printk(KERN_INFO "%s: cci_als_adc_test = %d\n", __func__, cci_als_adc_test);	

	
#ifdef LightSensorK
	if((ret = kstrtoul(token[6], 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:[STK]strict_strtoul failed, ret=0x%x\n", __func__,ret);
                return ret;
        }
	cci_als_lux_cali=value;

	if((ret = kstrtoul(token[7], 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:[STK]strict_strtoul failed, ret=0x%x\n", __func__, ret);
		return ret;
	}
	
	cci_transmittance_cali = value;
	if(cci_transmittance_cali ==0)
		 cci_transmittance_cali=900; // writ back to als_transmittance
	if((ret = kstrtoul(token[8], 16, &value)) < 0)
        {
                printk(KERN_ERR "%s:[STK]strict_strtoul failed, ret=0x%x\n", __func__, ret);
                return ret;
        }
        cci_als_adc_cali = value;
#endif	
	return size;
}

static ssize_t stk3x1x_show_cali(struct device_driver *ddri, char *buf)
{
	int32_t word_data=0;
	u8 r_buf[2]={0,0};
	int ret=0;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}

	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x20, r_buf, 2);
	if(ret < 0)	
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}
	word_data = (r_buf[0] << 8) | r_buf[1];

	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x22, r_buf, 2);
	if(ret < 0)		
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}	
	word_data += (r_buf[0] << 8) | r_buf[1];	

	APS_LOG("%s: psi_set=%d, psa=%d,psi=%d, word_data=%d\n", __FUNCTION__, 
		stk3x1x_obj->psi_set, stk3x1x_obj->psa, stk3x1x_obj->psi, word_data);	
	
	return 0;
}

#endif

static ssize_t stk3x1x_show_ccidebug(struct device_driver *ddri, char *buf)
{

	APS_ERR("[Colby]stk3x1x_show_ccidebug\n");
	APS_ERR("[Colby]CCI_debug_for_stk3x1x = %d\n", CCI_debug_for_stk3x1x);

	return scnprintf(buf, PAGE_SIZE, "CCI_debug_for_stk3x1x = %d\n", CCI_debug_for_stk3x1x);

}

static ssize_t stk3x1x_store_ccidebug(struct device_driver *ddri, const char *buf, size_t count)
{
	int enable = 0;
	APS_ERR("[Colby]stk3x1x_store_ccidebug\n");
	if(1 != sscanf(buf, "%d", &enable))
	{
		APS_ERR("[Colby]invalid format: '%s'\n", buf);
		return 0;
	}
	APS_ERR("[Colby]stk3x1x_store_ccidebug enable = %d", enable);

	if(enable){
		CCI_debug_for_stk3x1x = 1;
		APS_ERR("[Colby]stk3x1x_store_ccidebug set enable");
		}
	else{
		CCI_debug_for_stk3x1x = 0;
		APS_ERR("[Colby]stk3x1x_store_ccidebug set disable");
		}
	return count;
}



static ssize_t stk3x1x_show_nvdata(struct device_driver *ddri, char *buf)
{
	APS_ERR("[STK]stk3x1x_show_nvdata\n");
	APS_ERR("[STK]cci_ps_high_thd = %d\n", cci_ps_high_thd);
	APS_ERR("[STK]cci_ps_low_thd =  %d\n", cci_ps_low_thd);
	APS_ERR("[STK]cci_result_ct = %d\n", cci_result_ct);
	APS_ERR("[STK]cci_ps_cover = %d\n", cci_ps_cover);
	
	APS_ERR("[STK]cci_als_lux_cali = %d\n", cci_als_lux_cali);
	APS_ERR("[STK]cci_transmittance_cali = %d\n", cci_transmittance_cali);
	APS_ERR("[STK]cci_als_adc_cali = %d\n", cci_als_adc_cali);

	APS_ERR("[STK]cci_als_value = %d\n", cci_als_value);
	APS_ERR("[STK]cci_als_adc_test = %d\n", cci_als_adc_test);

	return scnprintf(buf, PAGE_SIZE, "cci_ps_high_thd = %d, cci_ps_low_thd = %d, cci_result_ct = %d, cci_ps_cover = %d, cci_als_value = %d, cci_als_adc_test = %d, cci_als_lux_cali=%d , cci_transmittance_cali = %d, cci_als_adc_cali =%d\n", cci_ps_high_thd, cci_ps_low_thd, cci_result_ct, cci_ps_cover, cci_als_value,cci_als_adc_test,cci_als_lux_cali, cci_transmittance_cali, cci_als_adc_cali);  		
}

//===========PS K & Test START========================================================================================

static uint32_t stk3x1x_get_ps_reading_AVG(int sSampleNo){
	uint32_t PSData = 0;
	uint32_t DataCount = 0;
	uint32_t sAvePsData = 0;
	int err=0;

	while(DataCount < sSampleNo)
	{
		msleep(100);
		err =stk3x1x_read_ps(stk3x1x_obj->client, &stk3x1x_obj->ps);
		if(err!=0)
			{
			APS_ERR("[STK]%s: stk3x1x_read_ps fail\n", __func__);
			}
		else{
			PSData = stk3x1x_obj->ps;
			APS_ERR("[STK]%s: ps code = %d\n", __func__, PSData);
			sAvePsData +=  PSData;
			DataCount++;
			}
	}
	sAvePsData /= sSampleNo;
	return sAvePsData;
}

static ssize_t stk3x1x_show_GC_HT(struct device_driver *ddri, char *buf)
{

	uint32_t reading=0;
	int err=0, res=-1;
	if(!stk3x1x_obj)
	{
		APS_ERR("[STK]STK stk3x1x_obj is null!!\n");
		res = -2;
	}
	//---set default THD as ps_thd_l = 100 first then, ps_thd_h = 300,  to ensure int trigger start---
	atomic_set(&stk3x1x_obj->ps_low_thd_val, 100);
	if((err = stk3x1x_write_ps_low_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_low_thd_val))))
	{
		APS_ERR("[STK]write low thd error: %d\n", err);
		res = -3;
	}	
	atomic_set(&stk3x1x_obj->ps_high_thd_val, 300);
	if((err = stk3x1x_write_ps_high_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_high_thd_val))))
	{
		APS_ERR("[STK]write high thd error: %d\n", err);
		res = -4;		
	}	
	//------set default THD as ps_thd_h = 300, ps_thd_l = 100 to ensure int trigger end---
	msleep(150);
	
	APS_ERR("[STK]P sensor start calibrating HT--\n");
	
	reading = stk3x1x_get_ps_reading_AVG(5);	//get ps code for 5 times;
	cci_ps_high_thd = reading; 
	res = 0;
	APS_ERR("[STK] P sensor calibrate HT done. Res = %d, cci_ps_high_thd = %d\n", res, cci_ps_high_thd);
	return scnprintf(buf, PAGE_SIZE, "cci_ps_high_thd = %d\n",  cci_ps_high_thd);
}

static ssize_t stk3x1x_show_GC_LT(struct device_driver *ddri, char *buf)
{
	uint32_t reading=0;
	int res = -1;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("[STK]stk3x1x_obj is null!!\n");
		res = -2;
	}
	
	APS_ERR("[STK]%s:-P sensor start calibrating Low threshold--\n", __func__);
	msleep(150);
	reading = stk3x1x_get_ps_reading_AVG(5);//get ps code for 5 times;
	cci_ps_low_thd = reading; 
	res = 0;
	APS_ERR("[STK] P sensor calibrate LT done. Res = %d, cci_ps_low_thd = %d\n", res, cci_ps_low_thd);
	return scnprintf(buf, PAGE_SIZE, " cci_ps_low_thd = %d\n", cci_ps_low_thd);
}

	
static ssize_t stk3x1x_show_ServiceCenter(struct device_driver *ddri, char *buf)
{
	uint32_t reading=0;
	int  err = 0, res = -1;
	bool result = false;

	if(!stk3x1x_obj)
	{
		APS_ERR("[STK]stk3x1x_obj is null!!\n");
		res = -2;
	}
	reading = stk3x1x_get_ps_reading_AVG(5);
	
	cci_result_ct = reading;

	if(hwid ==  BY57_hwid){
		cci_ps_high_thd = cci_result_ct + STK_HT_N_CT ;
		cci_ps_low_thd = cci_result_ct + STK_LT_N_CT ;
	} else { /* BY86 */
		cci_ps_high_thd = cci_result_ct + 100;
		cci_ps_low_thd = cci_result_ct + 50;
	}

	if( hwid ==  BY57_hwid ){
		if( (cci_result_ct > 0) && (cci_result_ct <= 400) ) {
			atomic_set(&stk3x1x_obj->ps_high_thd_val, cci_ps_high_thd);
			atomic_set(&stk3x1x_obj->ps_low_thd_val,cci_ps_low_thd );
			
			if((err = stk3x1x_write_ps_high_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_high_thd_val))))
			{
				APS_ERR("[STK]write high thd error: %d\n", err);
				res = -3;
			}
			if((err = stk3x1x_write_ps_low_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_low_thd_val))))
			{
				APS_ERR("[STK]write low thd error: %d\n", err);
				res = -4;
			}
			res = 0;
			result = true;
		} 
		else{
		res = -5;
		}
	}
	else{  //BY86
		if((cci_result_ct > 0) && (cci_result_ct < 700)) {
			atomic_set(&stk3x1x_obj->ps_high_thd_val, cci_ps_high_thd);
			atomic_set(&stk3x1x_obj->ps_low_thd_val,cci_ps_low_thd );
		if((err = stk3x1x_write_ps_high_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_high_thd_val))))
		{
			APS_ERR("[STK]write high thd error: %d\n", err);
			res = -3;
		}
		if((err = stk3x1x_write_ps_low_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_low_thd_val))))
		{
			APS_ERR("[STK]write low thd error: %d\n", err);
			res = -4;
		}
			res = 0;
			result = true;
		} 
		else{
		res = -5;
		}
	}
	printk(KERN_ERR "[STK]%s : Psensor ServiceCenter calibrate CT done.  Res = %d, cci_result_ct = %d, cci_ps_high_thd = %d, cci_ps_low_thd = %d, \n", result ? "PASS" : "FAIL", res, cci_result_ct, cci_ps_high_thd, cci_ps_low_thd);
	return scnprintf(buf, PAGE_SIZE, "%s : cci_result_ct = %d, cci_ps_high_thd = %d, cci_ps_low_thd = %d \n",  result ? "PASS" : "FAIL", cci_result_ct, cci_ps_high_thd, cci_ps_low_thd);
}



static ssize_t stk3x1x_show_GC_CT(struct device_driver *ddri, char *buf)
{
	uint32_t reading=0;
	int Diff_ThdL_CT = 0, Diff_ThdH_L = 0, err = 0, res = -1;
	bool result = false;

	if(!stk3x1x_obj)
	{
		APS_ERR("[STK]stk3x1x_obj is null!!\n");
		res = -2;
	}
	reading = stk3x1x_get_ps_reading_AVG(5);
	
	cci_result_ct = reading;
	Diff_ThdL_CT = cci_ps_low_thd - cci_result_ct;
	Diff_ThdH_L=cci_ps_high_thd-cci_ps_low_thd;
	
	printk("[STK] P sensor stk calibrate CT---\n cci_ps_high_thd = %d, cci_ps_low_thd = %d\n Diff_ThdH_L = %d, Diff_ThdL_CT = %d, \n", cci_ps_high_thd, cci_ps_low_thd, Diff_ThdH_L, Diff_ThdL_CT);

	if( hwid ==  BY57_hwid ){
		if( (cci_result_ct > 0) && (cci_result_ct <= 1000) && (Diff_ThdL_CT > 10) && (Diff_ThdL_CT < 150) && (Diff_ThdH_L >= 20) && (Diff_ThdH_L < 200)){
			atomic_set(&stk3x1x_obj->ps_high_thd_val, cci_ps_high_thd);
			atomic_set(&stk3x1x_obj->ps_low_thd_val, cci_ps_low_thd);
			if((err = stk3x1x_write_ps_high_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_high_thd_val))))
			{
				APS_ERR("[STK]write high thd error: %d\n", err);
				res = -3;
			}
			if((err = stk3x1x_write_ps_low_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_low_thd_val))))
			{
				APS_ERR("[STK]write low thd error: %d\n", err);
				res = -4;
			}
			res = 0;
			result = true;
		} 
		else{
		res = -5;
		}
	}
	else{
		if( (cci_result_ct > 0) && (cci_result_ct < 1000) && (Diff_ThdL_CT > 20) && (Diff_ThdH_L > 20) ){ /* 20170214 ct < 1000 */
			atomic_set(&stk3x1x_obj->ps_high_thd_val, cci_ps_high_thd);
			atomic_set(&stk3x1x_obj->ps_low_thd_val, cci_ps_low_thd);
		if((err = stk3x1x_write_ps_high_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_high_thd_val))))
		{
			APS_ERR("[STK]write high thd error: %d\n", err);
			res = -3;
		}
		if((err = stk3x1x_write_ps_low_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_low_thd_val))))
		{
			APS_ERR("[STK]write low thd error: %d\n", err);
			res = -4;
		}
			res = 0;
			result = true;
		} 
		else{
		res = -5;
		}
	}
	printk(KERN_ERR "[STK]%s : Psensor stk calibrate CT done.  Res = %d, cci_result_ct = %d, cci_ps_high_thd = %d, cci_ps_low_thd = %d, Diff_ThdH_L = %d, Diff_ThdL_CT = %d, \n", result ? "PASS" : "FAIL", res, cci_result_ct, cci_ps_high_thd, cci_ps_low_thd, Diff_ThdH_L, Diff_ThdL_CT);
	return scnprintf(buf, PAGE_SIZE, "%s : cci_result_ct = %d, cci_ps_high_thd = %d, cci_ps_low_thd = %d \n",  result ? "PASS" : "FAIL", cci_result_ct, cci_ps_high_thd, cci_ps_low_thd);
}

static ssize_t stk3x1x_show_TEST(struct device_driver *ddri, char *buf)
{
	uint32_t reading = 0;
	bool ps_code_result = false;
	char *interrupt_reult, *test_result;
	
	APS_ERR("[STK]%s:P sensor stk start test.\n", __func__);
	reading = stk3x1x_get_ps_reading_AVG(5);
	
	cci_ps_cover = reading;
	
	APS_ERR("[STK] P sensor stk test done, INTERRUPT counter = %d, Test adc = %d \n", CCI_FTM_INTERRUPT,cci_ps_cover);

	ps_code_result = (cci_ps_cover > cci_ps_high_thd)? true : false;
	interrupt_reult = (CCI_FTM_INTERRUPT > 0)? "PASS" : "FAIL";

	test_result = (ps_code_result && (CCI_FTM_INTERRUPT > 0))? "SUCCESS": "FAIL";


	APS_ERR("[STK]%s: Psensor stk test done. cover_result = %d, test_ps_adc = %d, INT %s\n", test_result, ps_code_result, cci_ps_cover, interrupt_reult);
	return scnprintf(buf, PAGE_SIZE, "%s: cover_result = %d, test_ps_adc = %d, INT %s\n", test_result, ps_code_result, cci_ps_cover, interrupt_reult);
		
}

//===========PS K & Test END========================================================================================

//===========ALS K & Test START======================================================================================
static ssize_t stk3x1_cali_Light(struct device_driver *ddri, char *buf)
{
	int ret=0;
	bool result = 0;
	APS_ERR("[STK]%s:L sensor stk start calibrate.\n", __func__);
  	ret = stk3x1x_cci_als_cali(5);
	cci_als_lux_cali = stk_alscode2lux(cci_als_adc_cali);
	if( hwid ==  BY57_hwid )
		result = (ret && (cci_als_adc_cali >= 178) && ( cci_als_adc_cali <= 568))? 1:0;
	else //hwid ==  BY86_hwid
		result = (ret && (cci_als_adc_cali >= 150) && ( cci_als_adc_cali <= 560))? 1:0;
	printk("%s:[STK] L sensor stk calibrate done. cci_als_value_cali = %d lux, cci_transmittance_cali = %d, cci_als_value_cali_adc = %d code\n", result ? "PASS" : "FAIL", cci_als_lux_cali, cci_transmittance_cali, cci_als_adc_cali);
	
	return scnprintf(buf, PAGE_SIZE, "%s : cci_als_value_cali = %d, cci_transmittance_cali = %d, cci_als_value_cali_adc = %d \n", result ? "PASS" : "FAIL", cci_als_lux_cali, cci_transmittance_cali, cci_als_adc_cali);
}

/* 20161101 add lux control */
static ssize_t cci_dummy_lux_show(struct device_driver *ddri, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "CCI dummy Lux value = %d\n", dummyLuxValue);
}

static ssize_t cci_dummy_lux_store(struct device_driver *ddri, const char *buf, size_t size)
{
	int value = -1;
	int ret;

	if ((ret = kstrtoint(buf, 10, &value)) < 0) {
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;
	}

	if (value < 0 || value > 100000) {
		dummyLuxValue = -1;
		return -EINVAL;
	}

	dummyLuxValue = value;
	return size;
}
/* 20161101 add lux control */
	
static ssize_t stk3x1x_test_Light(struct device_driver *ddri, char *buf)
{	
	bool result=false;

	APS_ERR("[STK] L sensor stk start test.\n");
	msleep(150);
	cci_als_adc_test = stk3x1x_get_als_reading_AVG(5);
	cci_als_value = stk_alscode2lux(cci_als_adc_test);

	result = (cci_als_value >= 270 && cci_als_value <= 330)?  true : false;

	APS_ERR("[STK] L sensor stk test done. result = %d, cci_als_value = %d lux, cci_als_adc_test = %d\n", result, cci_als_value, cci_als_adc_test);

	return scnprintf(buf, PAGE_SIZE, "%s : cci_als_value = %d, cci_als_adc_test = %d \n", result ? "PASS" : "FAIL", cci_als_value,cci_als_adc_test);
}
//===========ALS K & Test END======================================================================================

static ssize_t ps_Pin_sensor_show(struct device_driver *ddri, char *buf)
{
	//struct stk3x1x_data *ps_data =  dev_get_drvdata(dev);	
	int32_t ret = stk3x1x_read_id(stk3x1x_i2c_client);
	if(ret >= 0){
		 printk("STK check PID = %d success\n",ret);
    		 return scnprintf(buf, PAGE_SIZE,"[stk] I2C Work, PID = %x\n",ret);
		}
	else{
		 printk("STK check PID = %d fail\n",ret);
    		 return scnprintf(buf, PAGE_SIZE,"[stk] I2C mayFail!\n Check PID = %x number\n",ret);
		}
}
static ssize_t stk3x1x_store_alsenable(struct device_driver *ddri, const char *buf, size_t count)
{
	uint8_t en=0;
	int err=0;
	//int x=0;
	if (*buf==49)
	{
		en = 1;
	//	als_open_file(en);
	}
	else if (*buf==48)
	{
		en = 0;
	//	als_open_file(en);
	}
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
	//	return -EINVAL;
	}
    printk(KERN_INFO "%s: Enable ALS : %d\n", __func__, en);
	err=stk3x1x_enable_als(stk3x1x_obj->client,en);
    if(err)
	{
		APS_ERR("enable ps fail: %d\n", err); 
		return -1;
	}

	set_bit(STK_BIT_ALS, &stk3x1x_obj->enable);
   // mutex_unlock(&ps_data->io_lock);
   //err = scnprintf(buf, PAGE_SIZE, "done\n");
    return count;
}

static ssize_t stk3x1x_show_alsenable(struct device_driver *ddri, char *buf)
{
		return 0;
}
static ssize_t stk3x1x_store_psenable(struct device_driver *ddri, const char *buf, size_t count)
{
	uint8_t en=0;
	int err=0;
	
	if (*buf==49)//sysfs_streq(buf, 49))
	{
		en = 1;
	//	ps_open_file(en);
	}	
	else if (*buf==48)//sysfs_streq(buf, 48))
	{
		en = 0;
	//	ps_open_file(en);
	}
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
	//	return -EINVAL;
	}
    printk(KERN_INFO "%s: Enable PS : %d\n", __func__, en);
	err = stk3x1x_enable_ps(stk3x1x_obj->client, en);
    if(err)
	{
		APS_ERR("enable ps fail: %d\n", err); 
		return -1;
	}

	set_bit(STK_BIT_PS, &stk3x1x_obj->enable);
   // mutex_unlock(&ps_data->io_lock);
//    err = snprintf(buf, PAGE_SIZE, "done\n");

    return count;
}

static ssize_t stk3x1x_show_psenable(struct device_driver *ddri, char *buf)
{
		return 0;
}

static DRIVER_ATTR(alsenable,     S_IWUSR | S_IRUGO, stk3x1x_show_alsenable,   stk3x1x_store_alsenable);
static DRIVER_ATTR(psenable,     S_IWUSR | S_IRUGO, stk3x1x_show_psenable,   stk3x1x_store_psenable);
static DRIVER_ATTR(nvdata,     S_IWUSR | S_IRUGO, stk3x1x_show_nvdata,   NULL);
static DRIVER_ATTR(GC_HT,    S_IWUSR | S_IRUGO, stk3x1x_show_GC_HT,  NULL);
static DRIVER_ATTR(GC_LT,    S_IWUSR | S_IRUGO, stk3x1x_show_GC_LT,  NULL);
static DRIVER_ATTR(GC_CT,    S_IWUSR | S_IRUGO, stk3x1x_show_GC_CT,  NULL);
static DRIVER_ATTR(GC_TEST,    S_IWUSR | S_IRUGO, stk3x1x_show_TEST,  NULL);
static DRIVER_ATTR(cali_Light,  S_IWUSR | S_IRUGO, stk3x1_cali_Light,   NULL);
static DRIVER_ATTR(test_Light,    S_IWUSR | S_IRUGO, stk3x1x_test_Light,  NULL);
static DRIVER_ATTR(ccidebug,     S_IWUSR | S_IRUGO, stk3x1x_show_ccidebug,   stk3x1x_store_ccidebug);
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, stk3x1x_show_als,   NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, stk3x1x_show_ps,    NULL);
static DRIVER_ATTR(ir,      S_IWUSR | S_IRUGO, stk3x1x_show_ir,    NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, stk3x1x_show_config,stk3x1x_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, stk3x1x_show_alslv, stk3x1x_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, stk3x1x_show_alsval,stk3x1x_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, stk3x1x_show_trace, stk3x1x_store_trace);
//static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, stk3x1x_show_status,  NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, stk3x1x_show_send,  stk3x1x_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, stk3x1x_show_recv,  stk3x1x_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, stk3x1x_show_reg,   NULL);
static DRIVER_ATTR(allreg,  S_IWUSR | S_IRUGO, stk3x1x_show_allreg,   NULL);
/* 20161101 add lux control */
static DRIVER_ATTR(dummy_lux, S_IWUSR | S_IRUGO, cci_dummy_lux_show, cci_dummy_lux_store);


#ifdef STK_TUNE0
static DRIVER_ATTR(cali,    S_IWUSR | S_IRUGO, stk3x1x_show_cali,  stk3x1x_store_cali);
#endif
static DRIVER_ATTR(hwid,      S_IWUSR | S_IRUGO, stk3x1x_show_hwid,    NULL);
//static DRIVER_ATTR(tptype,      S_IWUSR | S_IRUGO, stk3x1x_show_tptype,    NULL);
static DRIVER_ATTR(Pin_Sensor,S_IWUSR | S_IRUGO,ps_Pin_sensor_show,NULL);

static DRIVER_ATTR(ServiceCenter,S_IWUSR | S_IRUGO,stk3x1x_show_ServiceCenter,NULL);



static struct driver_attribute *stk3x1x_attr_list[] = {
    &driver_attr_ps,    
    &driver_attr_ir,    
    &driver_attr_trace,        
    &driver_attr_config,
  //  &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_allreg,
//    &driver_attr_i2c,
    &driver_attr_reg,
#ifdef STK_TUNE0
    &driver_attr_cali,
#endif	
    &driver_attr_nvdata,
    &driver_attr_psenable,
    &driver_attr_GC_HT,
    &driver_attr_GC_LT,    
    &driver_attr_GC_CT,
    &driver_attr_GC_TEST,
	&driver_attr_dummy_lux,/* 20161101 add lux control */
    &driver_attr_ccidebug,
	//CCI Colby add end 20130312	
    &driver_attr_hwid,
     //&driver_attr_tptype,
     &driver_attr_Pin_Sensor,
     &driver_attr_als,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_alsenable,
    &driver_attr_test_Light,
    &driver_attr_cali_Light,
    &driver_attr_ServiceCenter,
};

static int stk3x1x_create_attr(struct device_driver *driver) 
{
	int idx=0;
	int err=0;
	int num = (int)(sizeof(stk3x1x_attr_list)/sizeof(stk3x1x_attr_list[0]));
        APS_DBG("%s enter\n",__func__);
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		APS_DBG("%s, idx:%d, attr.name:%s\n",__func__,idx,stk3x1x_attr_list[idx]->attr.name);
		if((err = driver_create_file(driver, stk3x1x_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) error = %d\n", stk3x1x_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*---------------------------------------------------------------------------*/
static int stk3x1x_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(stk3x1x_attr_list)/sizeof(stk3x1x_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, stk3x1x_attr_list[idx]);
	}
	return err;
}
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int stk3x1x_get_als_lux(struct stk3x1x_priv *obj, u16 als)
{
	//int idx=0;
	int invalid = 0,lux = 0;
	lux=stk_alscode2lux(als);

/*-----------for removing MTK lux interval calculation START-----------*/
/*
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
*/
/*-----------for removing MTK lux interval calculation END-----------*/	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if (atomic_read(&obj->trace) & STK_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d\n", als, lux);
		}
		return lux;
	}
	else
	{
		if(atomic_read(&obj->trace) & STK_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d (-1)\n", als, lux);    
		}
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_get_ps_value_only(struct stk3x1x_priv *obj, u16 ps)
{
	int mask = atomic_read(&obj->ps_mask);
	int invalid = 0, val=0;
	u8 flag=0;
	int32_t word_data=0;
	int ret=0;
	u8 buf[2]={0,0};
	
	ret = stk3x1x_read_flag(obj->client, &flag);
	if(ret)
		return ret;
	val = (flag & STK_FLG_NF_MASK)? 1 : 0;	
	
	if(val == 0 && obj->psi_set > 0)
	{
		printk(KERN_INFO "%s: FAE dust entry\n", __func__);
		msleep(50);
		
		ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x11, buf, 2);
		if(ret < 0)	
		{
			APS_ERR("%s fail, ret=0x%x", __FUNCTION__, ret);
			return ret;	   
		}
		word_data = (buf[0] << 8) | buf[1];			
		//printk(KERN_INFO "%s: word_data=%d\n", __func__, word_data);
		
		if(word_data == 0)
		{
			//printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		printk(KERN_INFO "%s: show word_data psi_set: word_data=%d,psi_set=%d\n", __func__, word_data, obj->psi_set);
		if((word_data - obj->psi_set) > (cci_ps_high_thd-cci_ps_low_thd+700))
		{				
			if(cci_ps_low_thd==0 || cci_ps_high_thd==0)
			{
				APS_ERR("%s: [Colby] use STK default value\n", __FUNCTION__);		
				atomic_set(&obj->ps_high_thd_val, obj->psi_set + STK_HT_N_CT+500); 
				atomic_set(&obj->ps_low_thd_val, obj->psi_set + STK_LT_N_CT+500); 		
			}
			else
			{
				APS_ERR("%s: [Colby] use CCI calibration value: cci_ps_high_thd = %d, cci_ps_low_thd = %d\n", __FUNCTION__, cci_ps_high_thd, cci_ps_low_thd);
				atomic_set(&stk3x1x_obj->ps_high_thd_val, obj->psi_set + 2*(cci_ps_high_thd-cci_ps_low_thd)+500); 
				atomic_set(&stk3x1x_obj->ps_low_thd_val, obj->psi_set + cci_ps_high_thd-cci_ps_low_thd+500); 				
			}
			//STK add for CCI end 20130112					
			if((ret = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
			{
				APS_ERR("dust : write high thd error: %d\n", ret);
				return ret;        
			}		
			if((ret = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
			{
				APS_ERR("write low thd error: %d\n", ret);
				return ret;        
			}				
			APS_ERR("%s: FAE dust changed\n", __func__);
		}
		printk(KERN_INFO "%s: FAE dust exit\n", __func__);
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}		
	
	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		return val;
		
	}	
	else
	{
		APS_ERR(" ps value is invalid, PS:  %05d => %05d\n", ps, val);
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_get_ps_value(struct stk3x1x_priv *obj, u16 ps)
{
	int mask = atomic_read(&obj->ps_mask);
	int invalid = 0, val=0;
	int err=0;
	u8 flag=0;

	err = stk3x1x_read_flag(obj->client, &flag);
	if(err)
		return err;
	
	val = (flag & STK_FLG_NF_MASK)? 1 : 0;	
	if((err = stk3x1x_clear_intr(obj->client, flag, STK_FLG_OUI_MASK)))
	{
		APS_ERR("fail: %d\n", err);
		return err;
	}	

	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
		
	
	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		return val;
		
	}	
	else
	{
		APS_ERR(" ps value is invalid, PS:  %05d => %05d\n", ps, val);
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/

static int32_t stk3x1x_set_irs_it_slp(struct stk3x1x_priv *obj, uint16_t *slp_time)
{
	uint8_t irs_alsctrl=0;
	int32_t ret=0;
		
	irs_alsctrl = (atomic_read(&obj->alsctrl_val) & 0x0F) - 2;		
	switch(irs_alsctrl)
	{
		case 6:
			*slp_time = 12;
			break;
		case 7:
			*slp_time = 24;			
			break;
		case 8:
			*slp_time = 48;			
			break;
		case 9:
			*slp_time = 96;			
			break;				
		default:
			printk(KERN_ERR "%s: unknown ALS IT=0x%x\n", __func__, irs_alsctrl);
			ret = -EINVAL;	
			return ret;
	}
	irs_alsctrl |= (atomic_read(&obj->alsctrl_val) & 0xF0);
	ret = i2c_smbus_write_byte_data(obj->client, STK_ALSCTRL_REG, irs_alsctrl);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;		
	}		
	return 0;
}

static int32_t stk3x1x_get_ir_value(struct stk3x1x_priv *obj)
{
    int32_t word_data, ret;
	uint8_t w_reg, retry = 0;	
	uint16_t irs_slp_time = 100;
	bool re_enable_ps = false;
	u8 flag;
	u8 buf[2];
	
	re_enable_ps = (atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK) ? true : false;	
	if(re_enable_ps)
	{
#ifdef STK_TUNE0		
		if (!(obj->psi_set))
		{
			hrtimer_cancel(&obj->ps_tune0_timer);					
			cancel_work_sync(&obj->stk_ps_tune0_work);
		}		
#endif		
		stk3x1x_enable_ps(obj->client, 0);
	}
	
	ret = stk3x1x_set_irs_it_slp(obj, &irs_slp_time);
	if(ret < 0)
		goto irs_err_i2c_rw;
		
	w_reg = atomic_read(&obj->state_val) | STK_STATE_EN_IRS_MASK;		
    ret = i2c_smbus_write_byte_data(obj->client, STK_STATE_REG, w_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}	
	msleep(irs_slp_time);	
	
	do
	{
		msleep(3);		
		ret = stk3x1x_read_flag(obj->client, &flag);	
		if (ret < 0)
		{
			APS_ERR("WARNING: read flag reg error: %d\n", ret);
			goto irs_err_i2c_rw;
		}	
		retry++;
	}while(retry < 10 && ((flag&STK_FLG_IR_RDY_MASK) == 0));
	
	if(retry == 10)
	{
		printk(KERN_ERR "%s: ir data is not ready for 300ms\n", __func__);
		ret = -EINVAL;
		goto irs_err_i2c_rw;
	}

	ret = stk3x1x_clear_intr(obj->client, flag, STK_FLG_IR_RDY_MASK);	
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}		
	
	ret = stk3x1x_master_recv(obj->client, STK_DATA1_IR_REG, buf, 2);
	if(ret < 0)	
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret); 
		goto irs_err_i2c_rw;		
	}
	word_data =  (buf[0] << 8) | buf[1];

	ret = i2c_smbus_write_byte_data(obj->client, STK_ALSCTRL_REG, atomic_read(&obj->alsctrl_val));
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}
	if(re_enable_ps)
		stk3x1x_enable_ps(obj->client, 1);		
	return word_data;

irs_err_i2c_rw:	
	if(re_enable_ps)
		stk3x1x_enable_ps(obj->client, 1);	
	return ret;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int stk3x1x_open(struct inode *inode, struct file *file)
{
	file->private_data = stk3x1x_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/


#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
static long stk3x1x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int stk3x1x_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)      
#endif
{

	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);  
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
	long err = 0;
#else
	int err = 0;
#endif
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	//CCI Colby add start 20130111
	//uint32_t ftm_auto_cali;
	struct stk3x1x_threshold calibration_threshold;
	struct CCI_threshold ftm_thd; // add for FTM interrupt check 20130416
	//CCI Colby add end 20130111
	
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable))) //copy from kernel space to user space
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			//CCI_FTM = 1;//add for FTM interrupt check 20130416
				if((err = stk3x1x_enable_ps(obj->client, 1)))
				{
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
					APS_ERR("enable ps fail: %ld\n", err); 
#else
					APS_ERR("enable ps fail: %d\n", err); 
#endif
					goto err_out;
				}
				
				set_bit(STK_BIT_PS, &obj->enable);
			}
			else
			{
			//CCI_FTM = 0;//add for FTM interrupt check 20130416
				if((err = stk3x1x_enable_ps(obj->client, 0)))
				{
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
					APS_ERR("disable ps fail: %ld\n", err); 
#else
					APS_ERR("disable ps fail: %d\n", err); 
#endif
	
					goto err_out;
				}
				
				clear_bit(STK_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(STK_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable))) //copy from user space to kernel space
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = stk3x1x_get_ps_value(obj, obj->ps);
			if(dat < 0)
			{
				err = dat;
				goto err_out;
			}
#ifdef STK_PS_POLLING_LOG	
			APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps, dat);			
#endif			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;            

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = stk3x1x_enable_als(obj->client, 1)))
				{
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
					APS_ERR("enable als fail: %ld\n", err); 
#else
					APS_ERR("enable als fail: %d\n", err); 
#endif

					goto err_out;
				}
				set_bit(STK_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = stk3x1x_enable_als(obj->client, 0)))
				{
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
					APS_ERR("disable als fail: %ld\n", err); 
#else
					APS_ERR("disable als fail: %d\n", err); 
#endif

					goto err_out;
				}
				clear_bit(STK_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(STK_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = stk3x1x_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = stk3x1x_get_als_lux(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = stk3x1x_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}
			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		//CCI Colby add start 20130110	
// add for FTM interrupt check 20130416 start

		case ALSPS_SET_THD:
			APS_ERR("%s ALSPS_SET_THD enter", __FUNCTION__);
			if(copy_from_user(&ftm_thd, ptr, sizeof(ftm_thd))){
				err = -EFAULT;
				goto err_out;
			}
			atomic_set(&obj->ps_high_thd_val, ftm_thd.ps_high_thd);
			atomic_set(&obj->ps_low_thd_val, ftm_thd.ps_low_thd);
			APS_ERR("%s ps_high_thd = %d, ps_low_thd = %d", __FUNCTION__, ftm_thd.ps_high_thd, ftm_thd.ps_low_thd);	
			if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
			{
			APS_ERR("write high thd error: %ld\n", err);
			return err;
			}

			if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val))))
			{
			APS_ERR("write low thd error: %ld\n", err);
			return err;
			}

			break;

		case ALSPS_GET_INTERRUPT_COUNT:    
			APS_ERR("%s ALSPS_GET_INTERRUPT_COUNT enter CCI_FTM_INTERRUPT = %d", __FUNCTION__, CCI_FTM_INTERRUPT);
			dat = CCI_FTM_INTERRUPT;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;         		

// add for FTM interrupt check 20130416 end		
		//----------------------------------for factory mode test---------------------------------------
		case ALSPS_IOCTL_SET_CALI:
			break;
			
		case ALSPS_IOCTL_SET_CALI_CCI:
			if(copy_from_user(&calibration_threshold, ptr, sizeof(calibration_threshold))) {
				err = -EFAULT;
				goto err_out;
			}

			cci_ps_high_thd=calibration_threshold.ps_high_thd;
			cci_ps_low_thd=calibration_threshold.ps_low_thd;
			APS_ERR("%s ALSPS_IOCTL_SET_CALI_CCI cci_ps_high_thd = %d, cci_ps_low_thd = %d\n", __FUNCTION__, cci_ps_high_thd, cci_ps_low_thd);

			cci_result_ct = calibration_threshold.result_ct;
			cci_ps_cover = calibration_threshold.result_ps_cover;
			cci_als_value = calibration_threshold.result_als_value;
			cci_als_adc_test = calibration_threshold.cci_als_adc_test;
			APS_ERR("%s ALSPS_IOCTL_SET_CALI_CCI cci_result_ct = %d, cci_ps_cover = %d, cci_als_value = %d  cci_als_adc_test =%d \n", __FUNCTION__, cci_result_ct, cci_ps_cover, cci_als_value,cci_als_adc_test);

			// add for FTM ALS CALI 20131024 start
			if(calibration_threshold.cci_transmittance_cali > 0)
				cci_transmittance_cali = calibration_threshold.cci_transmittance_cali;

			cci_als_adc_cali = calibration_threshold.cci_als_adc_cali;
			cci_als_lux_cali = calibration_threshold.cci_als_lux_cali;
			APS_ERR("%s ALSPS_IOCTL_SET_CALI_CCI als_transmittance_cali = %d, als_adc_cali = %d cci_als_lux_cali = %d \n", __FUNCTION__, cci_transmittance_cali, cci_als_adc_cali,cci_als_lux_cali);
			// add for FTM ALS CALI 20131024 end
			if(cci_ps_high_thd!=0 &&cci_ps_low_thd!=0&&cci_result_ct!=0)
			stk_tune_zero_set_fac_ct(obj);
			break;
#if 0

		case ALSPS_SET_AUTO_CALI:			
			APS_ERR("%s ALSPS_SET_AUTO_CALI enter", __FUNCTION__);			
			if(copy_from_user(&ftm_auto_cali, ptr, sizeof(ftm_auto_cali))){				
				err = -EFAULT;				
				goto err_out;			
			}			
			if(ftm_auto_cali){				
				APS_ERR("%s ALSPS_SET_AUTO_CALI disable auto calibration", __FUNCTION__);				
				stk3x1x_write_state(obj->client, 0x03);	
			}			
			else{				
				APS_ERR("%s ALSPS_SET_AUTO_CALI enable auto calibration", __FUNCTION__);				
				stk3x1x_write_state(obj->client, 0x43);				
			}			
			break;					
#endif
		/*----------------------------------for factory mode test---------------------------------------*/
		//CCI Colby add end 20130110
		
			
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations stk3x1x_fops = {
#if (LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))
	.owner = THIS_MODULE,
#endif
	.open = stk3x1x_open,
	.release = stk3x1x_release,
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,36))	
	.unlocked_ioctl = stk3x1x_unlocked_ioctl,
#else
	.ioctl = stk3x1x_ioctl,
#endif

};
/*----------------------------------------------------------------------------*/
static struct miscdevice stk3x1x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &stk3x1x_fops,
};

/*----------------------------------------------------------------------------*/
#if 0
static void stk3x1x_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	int err;
	struct stk3x1x_priv *obj = container_of(h, struct stk3x1x_priv, early_drv);   	
	int old = atomic_read(&obj->state_val);
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	if(old & STK_STATE_EN_ALS_MASK)
	{
		atomic_set(&obj->als_suspend, 1);    
		if((err = stk3x1x_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
}
/*----------------------------------------------------------------------------*/
static void stk3x1x_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	int err;
	//hwm_sensor_data sensor_data;
	struct stk3x1x_priv *obj = container_of(h, struct stk3x1x_priv, early_drv);         
	
	//memset(&sensor_data, 0, sizeof(sensor_data));
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	if(atomic_read(&obj->als_suspend))
	{
		atomic_set(&obj->als_suspend, 0);
		if(test_bit(STK_BIT_ALS, &obj->enable))
		{
			if((err = stk3x1x_enable_als(obj->client, 1)))
			{
				APS_ERR("enable als fail: %d\n", err);        

			}
		}
	}
}
#endif

/*----------------------------------------------------------------------------*/
#if (LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	
static int stk3x1x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, stk3x1x_DEV_NAME);
	return 0;
}
#endif

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("stk3x1x_obj als enable value = %d\n", en);

	if (!stk3x1x_obj) {
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}
	if(en)
		set_bit(STK_BIT_ALS, &stk3x1x_obj->enable);
	else
		clear_bit(STK_BIT_ALS, &stk3x1x_obj->enable);

	res=stk3x1x_enable_als(stk3x1x_obj->client,en);
	if (res!=0) {
		APS_ERR("stk3x1x_obj als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_get_data(int *value, int *status)
{
	int err = 0;
	int32_t als_reading=0,reading_lux=0;
	if (!stk3x1x_obj) {
		printk("stk3x1x objs is null!!\n");
		return -1;
	}
	als_reading=stk3x1x_get_als_reading_AVG(1);
	if (als_reading<0) {
		err = -1;
	} else {
		
		reading_lux = stk3x1x_get_als_lux(stk3x1x_obj, als_reading);//adc als to lux & mapping to table 
		//if(abs(stk3x1x_ptr->als_lux_last - reading_lux) >= STK_ALS_CHANGE_THD)
		//{
			*value=reading_lux;
			if(CCI_debug_for_stk3x1x)
				printk("stk3x1x als_get_data Light  als adc = %d ,mapping lux= %d \n",als_reading,*value);
			if (*value < 0)
				err = -1;
			*status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//}
	}
	return err;
}


static int als_set_delay(u64 ns)
{
	return 0;
}
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}
static int ps_get_data(int *value, int *status)
{
	int err = 0;
	if (!stk3x1x_obj) {
		printk("stk3x1x obj is null!!\n");
		return -1;
	}
	return err;
}


/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int ps_enable_nodata(int en)
{
	int res = 0;

        APS_LOG("stk3x1x_obj ps enable value = %d\n", en);

        if (!stk3x1x_obj) {
                APS_ERR("stk3x1x_obj is null!!\n");
                return -1;
        }
        if(en)
                set_bit(STK_BIT_PS, &stk3x1x_obj->enable);
        else
                clear_bit(STK_BIT_PS, &stk3x1x_obj->enable);

        res=stk3x1x_enable_ps(stk3x1x_obj->client,en);
        if (res!=0) {
                APS_ERR("stk3x1x_obj als_enable_nodata is failed!!\n");
                return-1;
        }
        return 0;
}

static int ps_set_delay(u64 ns)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stk3x1x_priv *obj;
	int err = 0;
	struct als_control_path als_ctl = { 0 };
	struct als_data_path als_data_path = { 0 };
	struct ps_control_path ps_ctl = { 0 };
       struct ps_data_path ps_data_path = { 0 };
	static struct kobject *alsps_sysfs_link; 

	hwid = get_cei_customer_project_id();
	APS_LOG("%s: HWID =%d\n", __FUNCTION__, hwid);


	printk("stk %s: driver version: %s\n", __FUNCTION__, DRIVER_VERSION);

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));

	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");  
	if(obj->irq_node!=NULL)
		printk("stk irq !=null\n");
	else
		printk("stk irq ==null\n");
	//obj->hw = get_cust_alsps_hw();
	obj->hw = hw;
	stk3x1x_get_addr(obj->hw, &obj->addr);
	obj->stk_wq = create_singlethread_workqueue("stk_wq");
	INIT_WORK(&obj->eint_work, stk3x1x_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 100);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);

	atomic_set(&obj->state_val, obj->hw->state_val);
	atomic_set(&obj->psctrl_val, obj->hw->psctrl_val);
	atomic_set(&obj->alsctrl_val, obj->hw->alsctrl_val);

	obj->ledctrl_val = obj->hw->ledctrl_val;
	obj->wait_val = obj->hw->wait_val;
	obj->int_val = 1;
	obj->first_boot = true;			 
	obj->als_correct_factor = 1000;
	
	atomic_set(&obj->recv_reg, 0);  
	
	if(obj->hw->polling_mode_ps == 0)
	{
		APS_LOG("%s: enable PS interrupt\n", __FUNCTION__);
	}
	obj->int_val |= STK_INT_PS_MODE1;
	
	if(obj->hw->polling_mode_als == 0)
	{
	  obj->int_val |= STK_INT_ALS;		
	  APS_LOG("%s: enable ALS interrupt\n", __FUNCTION__);
	}	

	APS_LOG("%s: state_val=0x%x, psctrl_val=0x%x, alsctrl_val=0x%x, ledctrl_val=0x%x, wait_val=0x%x, int_val=0x%x\n", 
		__FUNCTION__, atomic_read(&obj->state_val), atomic_read(&obj->psctrl_val), atomic_read(&obj->alsctrl_val), 
		obj->ledctrl_val, obj->wait_val, obj->int_val);
	
	
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	printk("stk obj->als_level_num  = %d , obj->als_value_num = %d \n",obj->als_level_num,obj->als_value_num);
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	if(atomic_read(&obj->state_val) & STK_STATE_EN_ALS_MASK)
	{
		set_bit(STK_BIT_ALS, &obj->enable);
	}
	
	if(atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK)
	{
		set_bit(STK_BIT_PS, &obj->enable);
	}
//	client->timing=400;
	stk3x1x_i2c_client = client;

#ifdef STK_TUNE0
	obj->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
	INIT_WORK(&obj->stk_ps_tune0_work, stk_ps_tune0_work_func);
	hrtimer_init(&obj->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	obj->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	obj->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif		
	mdelay(3); // delay 3 ms for power ready when issue first I2C command 
		
	if((err = misc_register(&stk3x1x_device)))
	{
		APS_ERR("stk3x1x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = stk3x1x_init_client(client);

	if(err == 0 ){
		APS_ERR("stk3x1x_init_client success\n");
	}
	else if( err == -2 ){
		APS_ERR("stk3x1x_init_client err_irq_node_error\n");
			goto err_irq_node_error;
	}
	else if( err == -3 ){
		APS_ERR("stk3x1x_init_client err_irq_parse_map_err\n");
			goto err_irq_parse_map_err;
	}
	else if( err == -4 ){
		APS_ERR("stk3x1x_init_client err_irq_request_err\n");
			goto err_irq_request_err;
	}
	else	{
		APS_ERR( "%s:stk3x1x_init_client invalid err(%dx)\n", __func__, err);
	}


	if(1 == obj->hw->polling_mode_ps)
	{
		wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock");
	}
	als_ctl.is_use_common_factory = false;	
	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
       als_ctl.set_delay = als_set_delay;

	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);
	if (err) {
		printk("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	als_data_path.get_data = als_get_data;
	als_data_path.vender_div = 100;
	err = als_register_data_path(&als_data_path);
	if (err) {
		printk("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	ps_ctl.is_use_common_factory = false;

	err = ps_register_control_path(&ps_ctl);
	if (err) {
		printk("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data_path.get_data = ps_get_data;
	ps_data_path.vender_div = 100;
	err = ps_register_data_path(&ps_data_path);
	if (err) {
		printk("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
/*--------------------------device node create------------------------------------------------*/	

err = stk3x1x_create_attr(&stk3x1x_init_info.platform_diver_addr->driver);
	if (err < 0) 
	{
		printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
		goto exit_create_attr_failed;
	}
/*--------------------------device node dynamic link create--------------------------------------*/	

	alsps_sysfs_link = kobject_create_and_add("alsps", NULL);

	
	if (alsps_sysfs_link != NULL) {
		err = sysfs_create_link(alsps_sysfs_link, &(stk3x1x_init_info.platform_diver_addr->driver.p->kobj), "link") ;
	} 																																	
	else {
		err = -ENODEV;
	}
	if (err < 0) {
		printk("could not create sysfs link for als\n");
		goto err_alsps_sysfs_link;
	}
#if 0
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = stk3x1x_early_suspend,
	obj->early_drv.resume   = stk3x1x_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif
	stk3x1x_obj = obj;
	mutex_init(&stk3x1x_obj->stk3x1x_op_mutex);
	
	printk("stk3x1x_i2c_probe() OK!\n");
	return 0;
	
err_alsps_sysfs_link:
	stk3x1x_obj = NULL; 					
			
exit_create_attr_failed:
exit_sensor_obj_attach_fail:
	free_irq(obj->irq,NULL);
err_irq_request_err:
	disable_irq(obj->irq);
err_irq_parse_map_err:
	if(interrupt_gpio_num>=0)
	gpio_free(interrupt_gpio_num);
err_irq_node_error:
	misc_deregister(&stk3x1x_device);
exit_misc_device_register_failed:
	stk3x1x_i2c_client = NULL;           
	kfree(obj);
exit:
	APS_ERR("%s: stk3x1x err = %d\n", __FUNCTION__, err);
	return err;
}

static int stk3x1x_i2c_remove(struct i2c_client *client)
{
	int err;	
#ifdef STK_TUNE0
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);		
	destroy_workqueue(obj->stk_ps_tune0_wq);	
#endif		
	
	if((err = stk3x1x_delete_attr(&stk3x1x_i2c_driver.driver)))
	{
		APS_ERR("stk3x1x_delete_attr fail: %d\n", err);
	} 

	misc_deregister(&stk3x1x_device);
	
	free_irq(obj->irq, obj);
	stk3x1x_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init stk3x1x_init(void)
{
	int ret=1;
	const char *name = "mediatek,stk3x1x";
	printk("stk __init stk3x1x_init\n");
	hw =   get_alsps_dts_func(name, hw);
	if (!hw)
		printk("get dts info fail\n");
	else
	ret=alsps_driver_add(&stk3x1x_init_info);//.platform_diver_addr->driver
	if (ret)
	{	
	printk("stk alsps_driver_add fail \n");
        return ret;
	}
		
    return 0;
}

static int stk3x1x_local_init(void)
{
	int ret =-1;
	ret = i2c_add_driver(&stk3x1x_i2c_driver);
	printk("stk add i2c_add_driver(&stk3x1x_i2c_driver) ret = %d\n",ret);
	if (ret)
	{	printk("stk delet i2c driver\n");
		i2c_del_driver(&stk3x1x_i2c_driver);
        return ret;
	}
	else
		{printk("stk i2c_add_driver\n");}
	return 0;
}
static int stk3x1x_local_remove(void)
{
return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit stk3x1x_exit(void)
{
	APS_FUN();
	i2c_del_driver(&stk3x1x_i2c_driver);
	//platform_driver_unregister(&stk3x1x_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(stk3x1x_init);
module_exit(stk3x1x_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("MingHsien Hsieh");
MODULE_DESCRIPTION("SensorTek stk3x1x proximity and light sensor driver");
MODULE_LICENSE("GPL");
