#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include <mt-plat/upmu_common.h>/* hwPowerOn */
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


#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>
#endif

#include <../../cei_hw_id/cei_hw_id.h>
#include <mt-plat/mtk_gpio.h>
#include <mt-plat/mtk_gpio_core.h>

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))
#define UDELAY(n) 											(lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
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

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/
#ifndef CONFIG_FPGA_EARLY_PORTING
#define KTD_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 0
#define I2C_ID_NAME "nt36670"
#define KTD_ADDR 0x3E
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
 #if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info nt36670_board_info __initdata = {I2C_BOARD_INFO(I2C_ID_NAME, KTD_ADDR)};
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif
/*static struct i2c_client *ktd2151_i2c_client = NULL;*/
struct i2c_client *nt36670_i2c_client;

/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int nt36670_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int nt36670_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct nt36670_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id nt36670_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver nt36670_iic_driver = {
	.id_table	= nt36670_id,
	.probe		= nt36670_probe,
	.remove		= nt36670_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "nt36670",
#if !defined(CONFIG_MTK_LEGACY)
		.of_match_table = lcm_of_match,
#endif
	},
 
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 
 
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int nt36670_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "*********nt36670_iic_probe\n");
	printk("********* KTD: info==>name=%s addr=0x%x\n",client->name,client->addr);
	nt36670_i2c_client  = client;		
	return 0;      
}

static int nt36670_remove(struct i2c_client *client)
{  	
	printk( "********* nt36670_remove\n");
	nt36670_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

/*
 * module load/unload record keeping
 */
static int __init nt36670_iic_init(void)
{
   printk( "********* nt36670_iic_init\n");
#if defined(CONFIG_MTK_LEGACY)
   i2c_register_board_info(KTD_I2C_BUSNUM, &nt36670_board_info, 1);
#endif
   printk( "********* nt36670_iic_init2\n");
   i2c_add_driver(&nt36670_iic_driver);
   printk( "********* nt36670_iic_init success\n");	
   return 0;
}

static void __exit nt36670_iic_exit(void)
{
  printk( "********* nt36670_iic_exit\n");
  i2c_del_driver(&nt36670_iic_driver);  
}

module_init(nt36670_iic_init);
module_exit(nt36670_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK NT36670 I2C Driver");
MODULE_LICENSE("GPL"); 
#endif
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE  1
#define LCM_DSI_VDO_MODE  !LCM_DSI_CMD_MODE
#define LCM_USE_PIN_CTL 1

#define FRAME_WIDTH                 (1080)
#define FRAME_HEIGHT                (1920)

#define GPIO_TP_ID                  23
#define GPIO_LCM_ID                 54
#define GPIO_NT36670_ENN_EN          12
#define GPIO_NT36670_ENP_EN          24
#define GPIO_LCD_LED_EN             102
#define GPIO_RESET_PIN              158

#define REGFLAG_DELAY               0xFC
#define REGFLAG_UDELAY              0xFB

#define REGFLAG_END_OF_TABLE        0xFD   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW           0xFE
#define REGFLAG_RESET_HIGH          0xFF


#define HX8394F_HD_ID  (0x94)

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;
#define DEBUG_MODE 99

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
	read_reg_v2(0xBF, buffer, 1);
	LCM_LOGI("get_display_power_mode : [0xBF]=0x%02x\n", buffer[0]);

	read_reg_v2(0x0A, buffer, 1);
	LCM_LOGI("get_display_power_mode : [0x0A]=0x%02x\n", buffer[0]);
}


#if !(LCM_USE_PIN_CTL)
static int lcm_get_gpio_intput(unsigned int GPIO){
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_IN);
	return mt_get_gpio_in(GPIO);
}
#endif		
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, (output>0)? GPIO_OUT_ONE: GPIO_OUT_ZERO);
}

static void pull_reset_pin_low(void){
#if 0 //fixme, pinctrl function error
	printk("%s, GPIO_RESET_PIN - 0\n", __func__);
#if (LCM_USE_PIN_CTL)
	set_reset_pin(0);
#else
	lcm_set_gpio_output(GPIO_RESET_PIN,GPIO_OUT_ZERO);
#endif
#else
	lcm_set_gpio_output(GPIO_RESET_PIN,GPIO_OUT_ZERO);
#endif
}

static void pull_reset_pin_high(void){
	printk("%s, GPIO_RESET_PIN - 1\n", __func__);
#if 0 //fixme, pinctrl function error
#if (LCM_USE_PIN_CTL)
	set_reset_pin(1);
#else
	lcm_set_gpio_output(GPIO_RESET_PIN,GPIO_OUT_ONE);
#endif		
#else
	lcm_set_gpio_output(GPIO_RESET_PIN,GPIO_OUT_ONE);
#endif
}

static void pull_enp_pin_low(void){
	printk("%s, GPIO_NT36670_ENP_EN - 0\n", __func__);
#if (LCM_USE_PIN_CTL)
	set_gpio_lcd_enp(0);
#else
	lcm_set_gpio_output(GPIO_NT36670_ENP_EN,GPIO_OUT_ZERO);
#endif	

}

static void pull_enp_pin_high(void){
	printk("%s, GPIO_NT36670_ENP_EN - 1\n", __func__);
#if (LCM_USE_PIN_CTL)
	set_gpio_lcd_enp(1);
#else
	lcm_set_gpio_output(GPIO_NT36670_ENP_EN,GPIO_OUT_ONE);
#endif		
}

static void pull_enn_pin_low(void){
	printk("%s, GPIO_nt36670_ENN_EN - 0\n", __func__);
#if (LCM_USE_PIN_CTL)
	set_gpio_lcd_enn(0);
#else
	lcm_set_gpio_output(GPIO_NT36670_ENN_EN,GPIO_OUT_ZERO);
#endif	
}

static void pull_enn_pin_high(void){
	//printk("%s, GPIO_NT36670_ENN_EN - 1\n", __func__);
#if (LCM_USE_PIN_CTL)
	set_gpio_lcd_enn(1);
#else
	lcm_set_gpio_output(GPIO_NT36670_ENN_EN,GPIO_OUT_ONE);
#endif

}

static void pull_lcd_backlight_low(void){
	printk("%s, GPIO_LCD_LED_EN - 0\n", __func__);
#if (LCM_USE_PIN_CTL)
	set_gpio_lcd_bkl(0);
#else
	lcm_set_gpio_output(GPIO_LCD_LED_EN,GPIO_OUT_ZERO);
#endif
}

static void pull_lcd_backlight_high(void){
	printk("%s, GPIO_LCD_LED_EN - 1\n", __func__);
#if (LCM_USE_PIN_CTL)
	set_gpio_lcd_bkl(1);
#else
	lcm_set_gpio_output(GPIO_LCD_LED_EN,GPIO_OUT_ONE);
#endif
}

//Read display id
static void get_lcd_id(void){
	unsigned int id = 0;
	unsigned char buffer[3];

	read_reg_v2(0x04, buffer, 3);

	id = (buffer[0] << 8) | buffer[1];	/* we only need ID */

	LCM_LOGI("get_lcd_id - read id, buf:0x%02x ,0x%02x,0x%02x, id=0X%X", buffer[0], buffer[1], buffer[2], id);
}

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_cmd_code_setting[] = {
	{ 0xFF,1,{0x10}},
	{ 0x35,1,{0x00}},
	{ 0x51,1,{0xFF}},
	{ 0x53,1,{0x2C}},
	{ 0x55,1,{0x00}},

	{ 0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},  

	{ 0x29,1, {0x00}},  
	{REGFLAG_DELAY, 10, {}}, 
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update){
	unsigned int i;

    for(i = 0; i < count; i++) {		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
}

static void init_lcm_cmd_registers(void){
	printk("%s - start - push_table - lcm_initialization_code_cmd_mode \n",__func__);

	push_table(lcm_initialization_cmd_code_setting, 
		sizeof(lcm_initialization_cmd_code_setting) / sizeof(struct LCM_setting_table), 1);

	printk("%s - end - push_table - lcm_initialization_code_cmd_mode \n",__func__);
}

static void init_lcm_registers_sleep_in(void){
	unsigned int data_array[16];
	lcm_debug("%s,Display Off, Sleep In \n", __func__);
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(70);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		params->physical_width=62;
   		params->physical_height=110;

#if (LCM_DSI_CMD_MODE)
		printk("[KERNEL]Set params->dsi.mode = CMD_MODE \n");
		params->dsi.mode   = CMD_MODE;
#else
		printk("[KERNEL]Set params->dsi.mode = BURST_VDO_MODE \n");
		params->dsi.mode = BURST_VDO_MODE;
        //params->dsi.mode = SYNC_EVENT_VDO_MODE;
		params->dsi.switch_mode = BURST_VDO_MODE;

        params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 4;
		params->dsi.vertical_frontporch					= 4;

        params->dsi.horizontal_sync_active				= 20;
		params->dsi.horizontal_backporch				= 67;
		params->dsi.horizontal_frontporch				= 12;	
		
#endif
		params->dsi.switch_mode_enable = 0;
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE; //Fill value to reference Tim's mail.
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size = 256;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		params->dsi.PLL_CLOCK = 450;

		params->dsi.clk_lp_per_line_enable = 0;
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable = 0;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
}

static void lcm_init_power(void)
{
}

static void lcm_init(void)
{
    int hw_version=0;
    //int lcm_source_pin = 0;

    lcm_debug("[%s]++\n", __func__);

    pull_reset_pin_low();
    MDELAY(10);
    lcm_debug("[%s]++1\n", __func__);
    pull_enp_pin_high();
    MDELAY(2);
    lcm_debug("[%s]++2\n", __func__);
    pull_enn_pin_high();
    MDELAY(10);
    lcm_debug("[%s]++3\n", __func__);
    pull_reset_pin_high();
    MDELAY(100);

#if (LCM_DSI_CMD_MODE)
	//lcm_debug("[DebugCreeds]%s %d:[LK] In LCM_DSI_CMD_MODE > \n", __func__,__LINE__);
	lcm_debug("[%s] %d: LCM_DSI_CMD_MODE init> \n", __func__,__LINE__);
	init_lcm_cmd_registers();
#endif
	

	//Check LCM ic power - 0x0A
	if(hw_version == DEBUG_MODE){
		init_lcm_cmd_registers();
		get_display_power_mode();
		get_lcd_id();
		pull_lcd_backlight_high();
		pull_lcd_backlight_low();
	}

    //lcm_source_pin = lcm_get_gpio_intput(GPIO_LCM_ID);
	hw_version = get_cei_customer_project_id();
	pull_lcd_backlight_high();
	lcm_debug("[%s] -- hw_version = %d\n", __func__, hw_version);
	
}

static void lcm_resume_power(void)
{
	pr_debug("%s ++\n", __func__);
}

static void lcm_suspend(void)
{
	pr_debug("%s, begin\n", __func__);

	pull_lcd_backlight_low();

	init_lcm_registers_sleep_in();

	pull_reset_pin_low();
	MDELAY(5);

	pull_enn_pin_low();
	MDELAY(5);

	pull_enp_pin_low();
	MDELAY(5);

	pr_debug("%s, end\n", __func__);
}

static void lcm_suspend_power(void)
{
	pr_debug("%s ++\n", __func__);
}

static void lcm_resume(void)
{
	pr_debug("%s ++\n", __func__);
	lcm_init();
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


LCM_DRIVER nt36670_fhd_dsi_cmd_tianma_drv = 
{
    .name			= "nt36670_fhd_dsi_cmd_tianma",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .init_power		= lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
