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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#include "tpd_fts_common.h"
#include <linux/input/mt.h>
#include "focaltech_core.h"
/* #include "ft5x06_ex_fun.h" */

#include "tpd.h"

/* #define TIMER_DEBUG */


#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif

#ifdef USE_WORK_AUTO_UPDATE
#include <linux/workqueue.h>
#endif

#include "cei_touch_log.h"

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
enum DOZE_T {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
};
static DOZE_T doze_status = DOZE_DISABLED;
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client);

enum TOUCH_IPI_CMD_T {
	/* SCP->AP */
	IPI_COMMAND_SA_GESTURE_TYPE,
	/* AP->SCP */
	IPI_COMMAND_AS_CUST_PARAMETER,
	IPI_COMMAND_AS_ENTER_DOZEMODE,
	IPI_COMMAND_AS_ENABLE_GESTURE,
	IPI_COMMAND_AS_GESTURE_SWITCH,
};

struct Touch_Cust_Setting {
	u32 i2c_num;
	u32 int_num;
	u32 io_int;
	u32 io_rst;
};

struct Touch_IPI_Packet {
	u32 cmd;
	union {
		u32 data;
		Touch_Cust_Setting tcs;
	} param;
};

#define CFG_MAX_TOUCH_POINTS	5
#define MT_MAX_TOUCH_POINTS	10
#define FTS_MAX_ID		0x0F
#define FTS_TOUCH_STEP		6
#define FTS_FACE_DETECT_POS	1
#define FTS_TOUCH_X_H_POS	3
#define FTS_TOUCH_X_L_POS	4
#define FTS_TOUCH_Y_H_POS	5
#define FTS_TOUCH_Y_L_POS	6
#define FTS_TOUCH_EVENT_POS	3
#define FTS_TOUCH_ID_POS	5
#define FT_TOUCH_POINT_NUM	2
#define FTS_TOUCH_XY_POS	7
#define FTS_TOUCH_MISC		8
#define POINT_READ_BUF		(3 + FTS_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*touch event info*/
struct ts_event
{
	u16 au16_x[CFG_MAX_TOUCH_POINTS];		/* x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];		/* y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/* touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];		/* touch ID */
	u16 pressure[CFG_MAX_TOUCH_POINTS];
	u16 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
	int touchs;
	u8 touch_point_num;
};

/* static bool tpd_scp_doze_en = FALSE; */
static bool tpd_scp_doze_en = TRUE;
DEFINE_MUTEX(i2c_access);
#endif

#define TPD_SUPPORT_POINTS	 4
#ifdef CONFIG_FOCAL_CHARGER_MODE
extern unsigned int tp_charger_state;
extern int cei_charger_mode(int);
#endif

struct i2c_client *i2c_client = NULL;
struct task_struct *thread_tpd = NULL;
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *fts_i2c_client = NULL;
struct input_dev *fts_input_dev = NULL;
bool fw_is_update;
#ifdef FTS_GLOVE_MODE
static int fts_glove_status = 0;
#endif
u8 tptype = 0;
int lcd_id_gpio_value = -1;
unsigned int LCD_ID_gpio_num = 54;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

#ifdef USE_WORK_AUTO_UPDATE
static struct workqueue_struct *auto_update_workqueue;
static void do_auto_firmware_update_work(struct work_struct *work);
static DECLARE_WORK(auto_update_work, do_auto_firmware_update_work);
#endif

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);


static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static int tpd_flag;
/*
static int point_num = 0;
static int p_point_num = 0;
*/

unsigned int tpd_rst_gpio_number = 0;
unsigned int tpd_int_gpio_number = 1;
unsigned int touch_irq = 0;

#define TPD_OK 0

/* Register define */
#define DEVICE_MODE	0x00
#define GEST_ID		0x01
#define TD_STATUS	0x02

#define TOUCH1_XH	0x03
#define TOUCH1_XL	0x04
#define TOUCH1_YH	0x05
#define TOUCH1_YL	0x06

#define TOUCH2_XH	0x09
#define TOUCH2_XL	0x0A
#define TOUCH2_YH	0x0B
#define TOUCH2_YL	0x0C

#define TOUCH3_XH	0x0F
#define TOUCH3_XL	0x10
#define TOUCH3_YH	0x11
#define TOUCH3_YL	0x12

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT	3

#ifdef TIMER_DEBUG

static struct timer_list test_timer;

static void timer_func(unsigned long data)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	mod_timer(&test_timer, jiffies + 100 * (1000 / HZ));
}

static int init_test_timer(void)
{
	memset((void *)&test_timer, 0, sizeof(test_timer));
	test_timer.expires  = jiffies + 100 * (1000 / HZ);
	test_timer.function = timer_func;
	test_timer.data     = 0;
	init_timer(&test_timer);
	add_timer(&test_timer);

	return 0;
}
#endif


#if defined(CONFIG_TPD_ROTATE_90) || defined(CONFIG_TPD_ROTATE_270) || defined(CONFIG_TPD_ROTATE_180)
/*
static void tpd_swap_xy(int *x, int *y)
{
	int temp = 0;

	temp = *x;
	*x = *y;
	*y = temp;
}
*/
/*
static void tpd_rotate_90(int *x, int *y)
{
//	int temp;

	*x = TPD_RES_X + 1 - *x;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
static void tpd_rotate_180(int *x, int *y)
{
	*y = TPD_RES_Y + 1 - *y;
	*x = TPD_RES_X + 1 - *x;
}
/*
static void tpd_rotate_270(int *x, int *y)
{
//	int temp;

	*y = TPD_RES_Y + 1 - *y;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
#endif
struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

/*dma declare, allocate and release*/
//#define __MSG_DMA_MODE__
#ifdef CONFIG_MTK_I2C_EXTENSION
	u8 *g_dma_buff_va = NULL;
	dma_addr_t g_dma_buff_pa = 0;
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION

static void msg_dma_alloct(void)
{
	if (NULL == g_dma_buff_va) {
		tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
		g_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 128, &g_dma_buff_pa, GFP_KERNEL);
	}

	if (!g_dma_buff_va) {
		TP_LOGE("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	}
}

static void msg_dma_release(void)
{
	if (g_dma_buff_va) {
		dma_free_coherent(NULL, 128, g_dma_buff_va, g_dma_buff_pa);
		g_dma_buff_va = NULL;
		g_dma_buff_pa = 0;
		TP_LOGI("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
}
#endif

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
/* static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX; */
/* static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX; */
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

static const struct i2c_device_id ft8607_tpd_id[] = {{"ft8607_touch", 0}, {} };
static const struct of_device_id ft8607_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, ft8607_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ft8607_dt_match),
		.name = "ft8607_touch",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft8607_tpd_id,
	.detect = tpd_i2c_detect,
};

static int of_get_ft8607_platform_data(struct device *dev)
{
	/*int ret, num;*/

	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(ft8607_dt_match), dev);
		if (!match) {
			TP_LOGE("Error: No device match found\n");
			return -ENODEV;
		}
	}

	TP_LOGI("tpd reset pin number (this is not GPIO NUM): %d\n", tpd_rst_gpio_number);
	TP_LOGI("tpd int pin number (this is not GPIO NUM): %d\n", tpd_int_gpio_number);

	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static ssize_t show_scp_ctrl(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t store_scp_ctrl(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 cmd;
	Touch_IPI_Packet ipi_pkt;

	if (kstrtoul(buf, 10, &cmd)) {
		TPD_DEBUG("[SCP_CTRL]: Invalid values\n");
		return -EINVAL;
	}

	TPD_DEBUG("SCP_CTRL: Command=%d", cmd);
	switch (cmd) {
	case 1:
		/* make touch in doze mode */
		tpd_scp_wakeup_enable(TRUE);
		tpd_suspend(NULL);
		break;
	case 2:
		tpd_resume(NULL);
		break;
/*
	case 3:
		// emulate in-pocket on
		ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
		ipi_pkt.param.data = 1;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		break;
	case 4:
		// emulate in-pocket off
		ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
		ipi_pkt.param.data = 0;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		break;
*/
	case 5:
		Touch_IPI_Packet ipi_pkt;

		ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
		ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
		ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
		ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
		ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;
		if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0) < 0)
			TPD_DEBUG("[TOUCH] IPI cmd failed (%d)\n", ipi_pkt.cmd);

		break;
	default:
		TPD_DEBUG("[SCP_CTRL] Unknown command");
		break;
	}

	return size;
}
static DEVICE_ATTR(tpd_scp_ctrl, 0664, show_scp_ctrl, store_scp_ctrl);
#endif

static struct device_attribute *ft8607_attrs[] = {
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	&dev_attr_tpd_scp_ctrl,
#endif
};

#ifndef MT_PROTOCOL_B
static void tpd_down(int x, int y, int p, int id)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
		TP_LOGD("x:%d y:%d p:%d\n", x, y, p);
		input_report_key(tpd->dev, BTN_TOUCH, 1);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(tpd->dev);
		if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {
			tpd_button(x, y, 1);	// MTK_PLATFORM
		}
	}
}

static void tpd_up(int x, int y)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		TP_LOGI("x:%d y:%d\n", x, y);
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_mt_sync(tpd->dev);

		if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {
			tpd_button(x, y, 0);	// MTK_PLATFORM
		}
	}
}
#endif /* MT_PROTOCOL_B */

/* Coordination mapping */
/*
static void tpd_calibrate_driver(int *x, int *y)
{
	int tx;

	tx = ((tpd_def_calmat[0] * (*x)) + (tpd_def_calmat[1] * (*y)) + (tpd_def_calmat[2])) >> 12;
	*y = ((tpd_def_calmat[3] * (*x)) + (tpd_def_calmat[4] * (*y)) + (tpd_def_calmat[5])) >> 12;
	*x = tx;
}
*/

#ifndef MT_PROTOCOL_B
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[3+6*TPD_SUPPORT_POINTS] = {0};
	u8 report_rate = 0;
	u16 high_byte, low_byte;
	char writebuf[10] = {0};
//	u8 fwversion = 0;

	writebuf[0] = 0x00;
	fts_i2c_read(i2c_client, writebuf, 1, data, (3+6*TPD_SUPPORT_POINTS));

//	fts_read_reg(i2c_client, FTS_REG_FW_VER, &fwversion);
	fts_read_reg(i2c_client, FTS_REG_POINT_RATE, &report_rate);

//	TP_LOGD("FW version=0x%02X\n", fwversion);

#if 0
	TPD_DEBUG("received raw data from touch panel as following:\n");
	for (i = 0; i < 8; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for (i = 8; i < 16; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for (i = 16; i < 24; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
	for (i = 24; i < 32; i++)
		TPD_DEBUG("data[%d] = 0x%02X ", i, data[i]);
	TPD_DEBUG("\n");
#endif
	if (report_rate < 8) {
		report_rate = 0x8;
		if ((fts_write_reg(i2c_client, FTS_REG_POINT_RATE, report_rate)) < 0)
			TP_LOGE("I2C write report rate error, line: %d\n", __LINE__);
	}

	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if ((data[0] & 0x70) != 0)
		return false;

	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	for (i = 0; i < TPD_SUPPORT_POINTS; i++)
		cinfo->p[i] = 1;	/* Put up */

	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;

	
	TP_LOGD("Number of touch points = %d, CEI define TPD_SUPPORT_POINTS = %d \n", cinfo->count,TPD_SUPPORT_POINTS);
	
	if(cinfo->count > TPD_SUPPORT_POINTS)
		cinfo->count =TPD_SUPPORT_POINTS;
	

	for (i = 0; i < cinfo->count; i++) {
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; /* event flag */
		cinfo->id[i] = data[3 + 6 * i + 2] >> 4; // touch id

		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;

		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;

//		TP_LOGD(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
//				cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}

#ifdef CONFIG_TPD_HAVE_CALIBRATION
	for (i = 0; i < cinfo->count; i++) {
		tpd_calibrate_driver(&(cinfo->x[i]), &(cinfo->y[i]));
		TP_LOGD(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
				cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}
#endif

	return true;

};
#endif /* MT_PROTOCOL_B */

 /************************************************************************
* Name: fts_read_Touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
#define COVER_MODE_EN		0x94
#define COVER_WINDOW_LEFT_HIGH	0x8c
#define COVER_WINDOW_LEFT_LOW	0x8d
#define COVER_WINDOW_UP_HIGH	0x8e
#define COVER_WINDOW_UP_LOW	0x9f
#define COVER_WINDOW_RIGHT_HIGH	0x90
#define COVER_WINDOW_RIGHT_LOW	0x91
#define COVER_WINDOW_DOWN_HIGH	0x92
#define COVER_WINDOW_DOWN_LOW	0x93

void fts_holster_enable(int enable)
{
	if (enable)
		fts_write_reg(fts_i2c_client, COVER_MODE_EN, 0x01);
	else
		fts_write_reg(fts_i2c_client, COVER_MODE_EN, 0x00);
}

#ifdef MT_PROTOCOL_B
static int fts_read_Touchdata(struct ts_event *data)
{
	u8 buf[POINT_READ_BUF] = { 0 }; // 0xFF
	int ret = -1;
	int i = 0;
	u8 pointid = FTS_MAX_ID;
//	u8 pt00f = 0;
/*
	if (tpd_halt) {
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
*/
//	mutex_lock(&i2c_access);
	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		TP_LOGE("read touchdata failed.\n");
//		mutex_unlock(&i2c_access);
		return ret;
	}

//	mutex_unlock(&i2c_access);
	memset(data, 0, sizeof(struct ts_event));
	data->touch_point = 0;
	data->touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;

//	TP_LOGD("fts_updateinfo_curr.TPD_MAX_POINTS=%d fts_updateinfo_curr.chihID=%d \n",
//			fts_updateinfo_curr.TPD_MAX_POINTS, fts_updateinfo_curr.CHIP_ID);

	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++) {
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			data->touch_point++;

		data->au16_x[i] =
				(s16) (buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F) << 8 |
				(s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];

		data->au16_y[i] =
				(s16) (buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F) << 8 |
				(s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];

		data->au8_touch_event[i] =
				buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;

		data->au8_finger_id[i] =
				(buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;

		data->pressure[i] =
				(buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]); // cannot constant value

		data->area[i] =
				(buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;

		TP_LOGD("touch[%d]: X=(%d) Y=(%d) touch_event=(%d) finger_id=(%d) pressure=(%d) area=(%d)\n",
				i, data->au16_x[i], data->au16_y[i], data->au8_touch_event[i],
				data->au8_finger_id[i], data->pressure[i], data->area[i]);

		if ((data->au8_touch_event[i] == 0 || data->au8_touch_event[i] == 2) &&
				((data->touch_point_num == 0) /*||(data->pressure[i] == 0 && data->area[i] == 0)*/))
			return 1;

		if (data->pressure[i] <= 0) {
			data->pressure[i] = 0x3f;
		}

		if (data->area[i] <= 0) {
			data->area[i] = 0x05;
		}

//		if ( pinfo->au16_x[i] == 0 && pinfo->au16_y[i] == 0)
//			pt00f++;
	}

	return 0;
}

 /************************************************************************
* Name: fts_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_report_value(struct ts_event *data)
 {
	int i = 0, j = 0;
	int up_point = 0;
 	int touchs = 0;

	for (i = 0; i < data->touch_point; i++) {
		input_mt_slot(tpd->dev, data->au8_finger_id[i]);

		if (data->au8_touch_event[i] == 0 || data->au8_touch_event[i] == 2) {
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
			input_report_abs(tpd->dev, ABS_MT_PRESSURE,data->pressure[i]);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,data->area[i]);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X,data->au16_x[i]);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y,data->au16_y[i]);
			touchs |= BIT(data->au8_finger_id[i]);
			data->touchs |= BIT(data->au8_finger_id[i]);
		} else {
			up_point++;
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(data->au8_finger_id[i]);
		}
	}

//	if(unlikely(data->touchs ^ touchs)) {
		for (i = 0; i < 10 /* fts_updateinfo_curr.TPD_MAX_POINTS */; i++) {
			if (BIT(i) & (data->touchs ^ touchs)) {
				TP_LOGD("Up by manual  id=%d \n", i);

				data->touchs &= ~BIT(i);
				input_mt_slot(tpd->dev, i);
				input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
			}
		}
//	}

	data->touchs = touchs;

	if (/* (last_touchpoint>0) && */(data->touch_point_num == 0)) {	// release all touches in final
		for (j = 0; j <10 /* fts_updateinfo_curr.TPD_MAX_POINTS */; j++) {
			input_mt_slot(tpd->dev, j);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
		}
//		ast_touchpoint = 0;
		data->touchs = 0;
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_sync(tpd->dev);

		return 0;
    	}

	if (data->touch_point == up_point)
		input_report_key(tpd->dev, BTN_TOUCH, 0);
	else
		input_report_key(tpd->dev, BTN_TOUCH, 1);

	input_sync(tpd->dev);

	return 0;
}
#endif /* MT_PROTOCOL_B */

#ifdef CONFIG_MTK_I2C_EXTENSION

int fts_i2c_read(struct i2c_client *client, char *writebuf,
		int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	/* for DMA I2c transfer */

	mutex_lock(&i2c_rw_access);

	if ((NULL != client) && (writelen > 0) && (writelen <= 128)) {
		/* DMA Write */
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

		if ((ret = i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen)) != writelen) {
			TP_LOGE("i2c write len=0x%x, buffaddr=0x%x\n", ret, *g_dma_buff_pa);
			TP_LOGE("i2c write failed\n");
		}
		client->addr = (client->addr & I2C_MASK_FLAG) & (~I2C_DMA_FLAG);
	}

	/* DMA Read */

	if ((NULL != client) && (readlen > 0) && (readlen <= 128)) {
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);

		memcpy(readbuf, g_dma_buff_va, readlen);

		client->addr = (client->addr & I2C_MASK_FLAG) & (~I2C_DMA_FLAG);
	}

	mutex_unlock(&i2c_rw_access);

	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int i = 0;
	int ret = 0;

	if (writelen <= 8) {
		client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG);
		return i2c_master_send(client, writebuf, writelen);

	} else if ((writelen > 8) && (NULL != g_dma_buff_pa)) {
		for (i = 0; i < writelen; i++)
			g_dma_buff_pa[i] = writebuf[i];

		client->addr = (client->addr & I2C_MASK_FLAG ) | I2C_DMA_FLAG;
//		client->ext_flag = client->ext_flag | I2C_DMA_FLAG;

		ret = i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen);

		client->addr = client->addr & I2C_MASK_FLAG & (~I2C_DMA_FLAG);

//		ret = i2c_master_send(client, (u8 *)(uintptr_t)tpd_i2c_dma_pa, writelen);

//		client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG);

		return ret;
	}

	return 1;
}

#else

int fts_i2c_read(struct i2c_client *client, char *writebuf,
		int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);

	if (readlen > 0) {
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
					.addr = client->addr,
					.flags = 0,
					.len = writelen,
					.buf = writebuf,
				},
				{
					.addr = client->addr,
					.flags = I2C_M_RD,
					.len = readlen,
					.buf = readbuf,
				},
			};
			ret = i2c_transfer(client->adapter, msgs, 2);
			if (ret < 0)
				TP_LOGE("i2c read error.\n");
		} else {
			struct i2c_msg msgs[] = {
				{
					.addr = client->addr,
					.flags = I2C_M_RD,
					.len = readlen,
					.buf = readbuf,
				},
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0)
				TP_LOGE("i2c read error.\n");
		}
	}

	mutex_unlock(&i2c_rw_access);

	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		 },
	};

	mutex_lock(&i2c_rw_access);

	if (writelen > 0) {
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			TP_LOGE("i2c write error.\n");
	}

	mutex_unlock(&i2c_rw_access);

	return ret;
}

#endif /* CONFIG_MTK_I2C_EXTENSION */

/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write(client, buf, sizeof(buf));
}
/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);

}

#ifdef FTS_GLOVE_MODE

#define FT8607_GLOVE_MODE_REG		0xC0
#define FT8607_GLOVE_MODE_ENABLE	1
#define FT8607_GLOVE_MODE_DISABLE	0

/*
 * Enable or Disable glove mode
 */
int fts_set_glove_mode(int enable)
{
	int ret = 0;

	switch (enable) {
	case 0:
		/* disable */
		ret = fts_write_reg(fts_i2c_client,
				FT8607_GLOVE_MODE_REG,
				FT8607_GLOVE_MODE_DISABLE);
		if (ret < 0)
			TP_LOGE("I2C error, enable glove mode failed\n");
		else {
			fts_glove_status = 0;
			TP_LOGI("Glove mode is disabled\n");
		}
		break;
	case 1:
		/* enable */
		ret = fts_write_reg(fts_i2c_client,
				FT8607_GLOVE_MODE_REG,
				FT8607_GLOVE_MODE_ENABLE);
		if (ret < 0)
			TP_LOGE("I2C error, disable glove mode failed\n");
		else {
			fts_glove_status = 1;
			TP_LOGI("Glove mode is enable\n");
		}
		break;
	default:
		TP_LOGE("invailed value (%d)\n", enable);
	}

	return ret;
}

int fts_get_glove_mode(void)
{
	int ret;
	int enable = 0;
	u8 read;

	ret = fts_read_reg(fts_i2c_client,
			FT8607_GLOVE_MODE_REG,
			&read);
	if (ret < 0) {
		TP_LOGW("I2C error, use global variable value as glove mode status\n");
		enable = fts_glove_status;
	} else {
		TP_LOGI("glove mode status (0x%02X)\n", read);

		if (read == 0x01) {
			TP_LOGI("glove mode status (Enabled)\n");
			enable = 1;
		} else if (read == 0x00) {
			TP_LOGI("glove mode status (Disabled)\n");
			enable = 0;
		} else {
			TP_LOGE("unknow glove mode status\n");
		}
	}

	return enable;
}
#endif /* FTS_GLOVE_MODE */

static int touch_event_handler(void *unused)
{
	int i = 0;
#if FTS_GESTRUE_EN || defined(MT_PROTOCOL_B)
	int ret = 0;
#if FTS_GESTRUE_EN
	u8 state = 0;
#endif
#endif

#ifndef MT_PROTOCOL_B
	struct touch_info cinfo;
	struct touch_info pinfo;
#endif
	struct touch_info finfo;
	struct sched_param param = { .sched_priority = 4 };
#ifdef MT_PROTOCOL_B
	struct ts_event pevent;
#endif

	TP_LOGI("touch_event_handler init\n");

	if (tpd_dts_data.use_tpd_button) {
		memset(&finfo, 0, sizeof(struct touch_info));
		for (i = 0; i < TPD_SUPPORT_POINTS; i++)
			finfo.p[i] = 1;
	}

	sched_setscheduler(current, SCHED_RR, &param);

	do {
		/* enable_irq(touch_irq); */
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

#if FTS_GESTRUE_EN
		ret = fts_read_reg(fts_i2c_client, 0xd0, &state);
		if (ret < 0) {
			TP_LOGW("i2c read 0xd0 value failed\n");
//			return ret;
		}

//		TP_LOGD("fts_read_Gestruedata state=%d\n",state);

	     	if (state == 1) {
			fts_read_Gestruedata();
			continue;
	    	}
#endif

		TP_LOGD("touch_event_handler start\n");

#ifdef MT_PROTOCOL_B
		ret = fts_read_Touchdata(&pevent);
		if (ret == 0)
			fts_report_value(&pevent);
#else
		if (tpd_touchinfo(&cinfo, &pinfo)) {
			if (tpd_dts_data.use_tpd_button) {
				if (cinfo.p[0] == 0)
					memcpy(&finfo, &cinfo, sizeof(struct touch_info));
			}

			if ((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y)
					&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
				TP_LOGD("Dummy release --->\n");
				tpd_up(pinfo.x[0], pinfo.y[0]);
				input_sync(tpd->dev);
				continue;
			}

			if (tpd_dts_data.use_tpd_button) {
				if ((cinfo.y[0] <= TPD_RES_Y && cinfo.y[0] != 0) && (pinfo.y[0] > TPD_RES_Y)
						&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
					TP_LOGD("Dummy key release --->\n");
					tpd_button(pinfo.x[0], pinfo.y[0], 0);
					input_sync(tpd->dev);
					continue;
				}

				if ((cinfo.y[0] > TPD_RES_Y) || (pinfo.y[0] > TPD_RES_Y)) {
					if (finfo.y[0] > TPD_RES_Y) {
						if ((cinfo.p[0] == 0) || (cinfo.p[0] == 2)) {
							TP_LOGD("Key press --->\n");
							tpd_button(pinfo.x[0], pinfo.y[0], 1);

						} else if ((cinfo.p[0] == 1) &&
								((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
							TP_LOGD("Key release --->\n");
							tpd_button(pinfo.x[0], pinfo.y[0], 0);
						}
						input_sync(tpd->dev);
					}
					continue;
				}
			}

			if (cinfo.count > 0) {
				for (i = 0; i < cinfo.count; i++)
					tpd_down(cinfo.x[i], cinfo.y[i], i + 1, cinfo.id[i]);
			} else {
#ifdef TPD_SOLVE_CHARGING_ISSUE
				tpd_up(1, 48);
#else
				tpd_up(pinfo.x[0], pinfo.y[0]);
#endif

			}
			input_sync(tpd->dev);
		}
#endif
	} while (!kthread_should_stop());

	TP_LOGI("touch_event_handler exit\n");

	return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	TP_LOGD("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	return IRQ_HANDLED;
}
static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0, 0};

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		TP_LOGI("touch_irq = %d\n", touch_irq);

		/* falling edge interrupt trigger */
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
				IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
		if (ret) {
			TP_LOGE("request_irq IRQ LINE NOT AVAILABLE!.\n");
			return -EINVAL;
		} else {
			TP_LOGI("request_irq successed.\n");
		}
	} else {
		TP_LOGE("request_irq can not find touch eint device node!.\n");
		return -ENODEV;
	}

	return 0;
}

#if 0
int hidi2c_to_stdi2c(struct i2c_client * client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;

	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;

	fts_i2c_write(client, auc_i2c_write_buf, 3);

	msleep(10);

	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;

	fts_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

	if (0xeb == auc_i2c_write_buf[0] && 0xaa == auc_i2c_write_buf[1] && 0x08 == auc_i2c_write_buf[2]) {
		bRet = 1;
	} else
		bRet = 0;

	return bRet;

}
#endif

#ifdef USE_WORK_AUTO_UPDATE
static void do_auto_firmware_update_work(struct work_struct *work)
{
	TP_LOGI("********************Enter CTP Auto Upgrade********************\n");

	mutex_lock(&fts_input_dev->mutex);

	fw_is_update = true;
	fts_ctpm_auto_upgrade(fts_i2c_client);
	fw_is_update = false;

	/* enable IRQ */
	enable_irq(touch_irq);
	mutex_unlock(&fts_input_dev->mutex);

	TP_LOGI("********************Exit CTP Auto Upgrade*********************\n");
}
#endif /* USE_WORK_AUTO_UPDATE */

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 report_rate = 0;
	int reset_count = 0;
	char data;
	int err = 0;
	u8 ver,vendorid;
	i2c_client = client;
	fts_i2c_client = client;
	fts_input_dev = tpd->dev;

	/* force set i2c address */
	// FIXME: need this workaroung?
	if (i2c_client->addr != 0x38) {
		i2c_client->addr = 0x38;
		TP_LOGW("Force set i2c_client->addr = 0x%02X\n", i2c_client->addr);
	}

	of_get_ft8607_platform_data(&client->dev);

	TP_LOGI("start ft8607\n");

	/* FIXME: vldo28 should be enabled in LK by LCM dirver */
//	retval = regulator_enable(tpd->reg);
//	if (retval != 0)
//		TP_LOGW("Failed to enable reg-vldo28: %d\n", retval);

#ifdef CONFIG_MTK_I2C_EXTENSION
	msg_dma_alloct();
#endif

reset_proc:

	/* Test i2c transfer */
	err = fts_read_reg(i2c_client, 0x00, &data);

	TP_LOGI("fts_i2c: err=%d, data=%d\n", err, data);

	if (err < 0 || data != 0) {	// reg0 data running state is 0; other state is not 0
		TP_LOGE("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
		if (++reset_count < TPD_MAX_RESET_COUNT){
			/* Reset CTP */
			tpd_gpio_output(tpd_rst_gpio_number, 0);
			msleep(5);
			tpd_gpio_output(tpd_rst_gpio_number, 1);
			msleep(295);	
			goto reset_proc;
		}
#endif
		goto err_i2c_transfer_fail;
	}
//--------------------TP vendor ID recognize START--------------------------------------------	
	/* select lcd id*/
	tpd_gpio_lcdid(LCD_ID_gpio_num);

	lcd_id_gpio_value = gpio_get_value(LCD_ID_gpio_num);

	if(lcd_id_gpio_value==0){  //INX
		tptype=0x03;
	}
	else if(lcd_id_gpio_value==1){
			if (fts_read_reg(fts_i2c_client, FTS_REG_VENDOR_ID, &vendorid) < 0) { //Truely + AUO or Truely + Sharp 
				TP_LOGE("Cannot read TP FTS_REG_VENDOR_ID (I2C failed?)\n");
				goto err_register_irq_fail;
			}
			if ( vendorid == 1 || vendorid == 2 ){
				TP_LOGI("Read TP vendor ID ,vendorid =0x%02X\n", vendorid);  //wait for remove
				tptype = vendorid;
			}
			else{
				TP_LOGE("Read TP vendor ID error,vendorid =0x%02X\n", vendorid);
			}			
	}
	else{
		TP_LOGE("Cannot recognize TP lcd_id_gpio_value = %d\n", lcd_id_gpio_value);
		}	

	TP_LOGI("TP tytype = 0x%02X\n", tptype);
//--------------------TP vendor ID recognize END--------------------------------------------	
	
	/* set INT mode */
	tpd_gpio_as_int(tpd_int_gpio_number);

	/* register IRQ */
	err = tpd_irq_registration();
	if (err) {
		TP_LOGE("register irq failed\n");
		goto err_register_irq_fail;
	}

	/* disable IRQ */
	disable_irq(touch_irq);

	msleep(100);

#if FTS_GESTRUE_EN
	fts_Gesture_init(tpd->dev);
#endif

	/* setup input parameters */
#ifdef MT_PROTOCOL_B
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
	input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS);
#else
	input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS, 2);
#endif
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#else
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, MT_MAX_TOUCH_POINTS, 0, 0);
#endif

//	hidi2c_to_stdi2c(fts_i2c_client);
	fts_get_upgrade_array();

#if 0
	/* Reset CTP */
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
#endif

#if 0
	/* Reset CTP */
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
#endif

	/* Set report rate 80Hz */
	report_rate = 0x8;
	if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0) {
		if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0)
			TP_LOGW("I2C write report rate error, line: %d\n", __LINE__);
	}

	/* tpd_load_status = 1; */

	thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread_tpd)) {
		err = PTR_ERR(thread_tpd);
		TP_LOGE("failed to create kernel thread_tpd: %d\n", err);

		goto err_kthread_fail;
	}

#ifdef TPD_AUTO_UPGRADE
#ifdef USE_WORK_AUTO_UPDATE
	auto_update_workqueue = create_singlethread_workqueue("ft8607_fw_auto_update_wq");
	queue_work(auto_update_workqueue, &auto_update_work);
#else
	TP_LOGI("********************Enter CTP Auto Upgrade********************\n");
	fw_is_update = true;
	fts_ctpm_auto_upgrade(fts_i2c_client);
	fw_is_update = false;

	/* enable IRQ */
	enable_irq(touch_irq);
#endif
#endif

#ifdef SYSFS_DEBUG
	fts_create_sysfs(fts_i2c_client);
#endif
#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(fts_i2c_client);
#endif
#ifdef FTS_CTL_IIC
	if (fts_rw_iic_drv_init(fts_i2c_client) < 0)
		TP_LOGE("create fts control iic driver failed\n");
#endif
	fts_test_init(fts_i2c_client);

	/* let mtk_tpd know this driver is success probed */
	tpd_load_status = 1;

	TP_LOGI("Touch Panel FT8607 Probe PASS\n");

#ifdef TIMER_DEBUG
	init_test_timer();
#endif

	/* Read firmware version & chip id */
	fts_read_reg(client, FTS_REG_FW_VER, &ver);
	TP_LOGI("fts_read_reg FW version : 0x%02X\n", ver);

	fts_read_reg(client, FTS_REG_CHIP_ID, &ver);
	TP_LOGI("fts_read_reg CHIP ID : 0x%02X\n", ver);

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int ret;

	ret = get_md32_semaphore(SEMAPHORE_TOUCH);
	if (ret < 0)
		pr_err("[TOUCH] HW semaphore reqiure timeout\n");
#endif

	return 0;

err_kthread_fail:
	TP_LOGE("Touch Panel FT8607 Probe FAILED\n");

	free_irq(touch_irq, NULL);
err_register_irq_fail:
err_i2c_transfer_fail:
#ifdef CONFIG_MTK_I2C_EXTENSION
	msg_dma_release();
#endif

//	retval = regulator_disable(tpd->reg); // disable regulator
//	if (retval) {
//		TP_LOGW("focaltech tpd_probe regulator_disable() failed!\n");
//	}
//	regulator_put(tpd->reg);

	fw_is_update = false;

	return err;
}

static int tpd_remove(struct i2c_client *client)
{
	TP_LOGI("TPD removed\n");
#ifdef FTS_CTL_IIC
	fts_rw_iic_drv_exit();
#endif
#ifdef SYSFS_DEBUG
	fts_remove_sysfs(client);
#endif

#ifdef FTS_APK_DEBUG
	fts_release_apk_debug_channel();
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION
	msg_dma_release();
#endif

//	gpio_free(tpd_rst_gpio_number);
//	gpio_free(tpd_int_gpio_number);

	return 0;
}

static int tpd_local_init(void)
{
//	int retval;

	TP_LOGI("Focaltech FT8607 I2C Touchscreen Driver...\n");

	/* enable regulators */
//	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");

//	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
//	if (retval != 0) {
//		TP_LOGE("Failed to set reg-vgp6 voltage: %d\n", retval);
//		return -1;
//	}

	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		TP_LOGE("unable to add i2c driver.\n");
		return -1;
	}

	/* tpd_load_status = 1; */
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num,
				   tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

	memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);
#endif

	tpd_type_cap = 1;

	TP_LOGI("end\n");

	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	char gestrue_on = 0x01;
	char gestrue_data;
	int i;

	/* TPD_DEBUG("Entering doze mode..."); */
	pr_alert("Entering doze mode...");

	/* Enter gestrue recognition mode */
	ret = fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);
	if (ret < 0) {
		/* TPD_DEBUG("Failed to enter Doze %d", retry); */
		pr_alert("Failed to enter Doze %d", retry);
		return ret;
	}
	msleep(30);

	for (i = 0; i < 10; i++) {
		fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
		if (gestrue_data == 0x01) {
			doze_status = DOZE_ENABLED;
			/* TPD_DEBUG("FTP has been working in doze mode!"); */
			pr_alert("FTP has been working in doze mode!");
			break;
		}
		msleep(20);
		fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);
	}

	return ret;
}
#endif

static void tpd_resume(struct device *h)
{
//	int retval = TPD_OK;
#if defined(CONFIG_FOCAL_CHARGER_MODE) || defined(FTS_GLOVE_MODE)
	int ret =0;
#endif
	TP_LOGI("TPD resume +++\n");

//	retval = regulator_enable(tpd->reg);
//	if (retval != 0)
//		TP_LOGW("Failed to enable reg-vldo28: %d\n", retval);
#if 0
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(5);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(295);
#endif

#ifdef CONFIG_FOCAL_CHARGER_MODE
       if(tp_charger_state){
                ret = cei_charger_mode(1);
                TP_LOGI(" resume fun cei_charger_mode(1) = %d \n",ret);
        }
        else{
                ret = cei_charger_mode(0);
                TP_LOGI(" resume fun cei_charger_mode(0) = %d \n",ret);
        }
#endif

#if FTS_GESTRUE_EN
//	if (fts_updateinfo_curr.CHIP_ID != 0x86) {
		fts_write_reg(fts_i2c_client, 0xD0, 0x00);
//	}
#endif

#ifdef FTS_GLOVE_MODE
	ret = fts_set_glove_mode(fts_glove_status);
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int ret;

	if (tpd_scp_doze_en) {
		ret = get_md32_semaphore(SEMAPHORE_TOUCH);
		if (ret < 0) {
			TPD_DEBUG("[TOUCH] HW semaphore reqiure timeout\n");
		} else {
			Touch_IPI_Packet ipi_pkt = {
				.cmd = IPI_COMMAND_AS_ENABLE_GESTURE,
				.param.data = 0
			};

			md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		}
	}
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	doze_status = DOZE_DISABLED;
	/* tpd_halt = 0; */
	int data;

	data = 0x00;

	fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, data);
#else
	enable_irq(touch_irq);
#endif

	TP_LOGI("TPD resume ---\n");
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
void tpd_scp_wakeup_enable(bool en)
{
	tpd_scp_doze_en = en;
}

void tpd_enter_doze(void)
{

}
#endif

static void tpd_suspend(struct device *h)
{
//	int retval = TPD_OK;
	static char data = 0x3;
#if FTS_GESTRUE_EN
	int i = 0;
	u8 state = 0;
#endif
	TP_LOGI("TPD suspend +++\n");

#if FTS_GESTRUE_EN
	if (1) {
//		memset(coordinate_x, 0, 255);
//		memset(coordinate_y, 0, 255);

		fts_write_reg(i2c_client, 0xd0, 0x01);
		fts_write_reg(i2c_client, 0xd1, 0xff);
		fts_write_reg(i2c_client, 0xd2, 0xff);
		fts_write_reg(i2c_client, 0xd5, 0xff);
		fts_write_reg(i2c_client, 0xd6, 0xff);
		fts_write_reg(i2c_client, 0xd7, 0xff);
		fts_write_reg(i2c_client, 0xd8, 0xff);

		msleep(10);

		for (i = 0; i < 10; i++) {
			TP_LOGD("tpd_suspend4 %d", i);
			fts_read_reg(i2c_client, 0xd0, &state);

			if (state == 1) {
				TP_LOGD("TPD gesture write 0x01\n");
				return;
			} else {
				fts_write_reg(i2c_client, 0xd0, 0x01);
				fts_write_reg(i2c_client, 0xd1, 0xff);
				fts_write_reg(i2c_client, 0xd2, 0xff);
				fts_write_reg(i2c_client, 0xd5, 0xff);
				fts_write_reg(i2c_client, 0xd6, 0xff);
				fts_write_reg(i2c_client, 0xd7, 0xff);
				fts_write_reg(i2c_client, 0xd8, 0xff);
				msleep(10);
			}
		}

		if (i >= 9) {
			TP_LOGE("TPD gesture write 0x01 to d0 fail \n");
			return;
		}
	}
#endif /* FTS_GESTRUE_EN */

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int sem_ret;

	tpd_enter_doze();

	int ret;
	char gestrue_data;
	char gestrue_cmd = 0x03;
	static int scp_init_flag;

	/* TP_LOGD("[tpd_scp_doze]:init=%d en=%d", scp_init_flag, tpd_scp_doze_en); */

	mutex_lock(&i2c_access);

	sem_ret = release_md32_semaphore(SEMAPHORE_TOUCH);

	if (scp_init_flag == 0) {
		Touch_IPI_Packet ipi_pkt;

		ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
		ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
		ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
		ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
		ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;

		TP_LOGD("[TOUCH]SEND CUST command :%d ", IPI_COMMAND_AS_CUST_PARAMETER);

		ret = md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		if (ret < 0)
			TP_LOGW(" IPI cmd failed (%d)\n", ipi_pkt.cmd);

		msleep(20); /* delay added between continuous command */
		/* Workaround if suffer MD32 reset */
		/* scp_init_flag = 1; */
	}

	if (tpd_scp_doze_en) {
		TP_LOGD("[TOUCH]SEND ENABLE GES command :%d ", IPI_COMMAND_AS_ENABLE_GESTURE);
		ret = ftp_enter_doze(i2c_client);
		if (ret < 0) {
			TP_LOGW("FTP Enter Doze mode failed\n");
		} else {
			int retry = 5;

			/* check doze mode */
			fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
			TP_LOGD("========================>0x%x", gestrue_data);

			msleep(20);
			Touch_IPI_Packet ipi_pkt = {
				.cmd = IPI_COMMAND_AS_ENABLE_GESTURE,
				.param.data = 1
			};

			do {
				if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 1) == DONE)
					break;
				msleep(20);
				TP_LOGD("==>retry=%d", retry);
			} while (retry--);

			if (retry <= 0)
				TP_LOGD("############# md32_ipi_send failed retry=%d", retry);
/*
			while(release_md32_semaphore(SEMAPHORE_TOUCH) <= 0) {
				//TPD_DEBUG("GTP release md32 sem failed\n");
				pr_alert("GTP release md32 sem failed\n");
			}
*/
		}
		/* disable_irq(touch_irq); */
	}

	mutex_unlock(&i2c_access);
#else
	disable_irq(touch_irq);
	fts_write_reg(i2c_client, 0xA5, data);  /* TP enter sleep mode */

//	retval = regulator_disable(tpd->reg);
//	if (retval != 0)
//		TP_LOGW("Failed to disable reg-vldo28: %d\n", retval);

#endif

	TP_LOGI("TPD suspend ---\n");
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "FT8607_touch",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = ft8607_attrs,
		.num  = ARRAY_SIZE(ft8607_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	/* for factory check, keep true until auto firmware upgrade done */
	fw_is_update = true;

	TP_LOGI("MediaTek FT8607 touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TP_LOGE("add FT8607 driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TP_LOGI("MediaTek FT8607 touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

