/*
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * $Revision: 8363 $
 * $Date: 2016-12-29 20:42:38 +0800 (週四, 29 十二月 2016) $
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
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/sched.h>
#include <linux/kthread.h>
/* #include <linux/rtpm_prio.h> 20170118 mask */ 

#include "tpd.h"
#include "nt36xxx.h"

/* #define P_GPIO_LCD_RST_PIN	180	20170118 mask (GPIO180|0x80000000) */
#define GPIO_CTP_RST_PIN 62

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
#endif

struct nvt_ts_data *ts;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

static const struct nvt_ts_mem_map NT36772_memory_map = {
	.EVENT_BUF_ADDR      = 0x11E00,
	.RAW_PIPE0_ADDR      = 0x10000,
	.RAW_PIPE1_ADDR      = 0x12000,
	.BASELINE_ADDR       = 0x10E70,
	.BASELINE_BTN_ADDR   = 0x12E70,
	.DIFF_PIPE0_ADDR     = 0x10830,
	.DIFF_PIPE1_ADDR     = 0x12830,
	.RAW_BTN_PIPE0_ADDR  = 0x10E60,
	.RAW_BTN_PIPE1_ADDR  = 0x12E60,
	.DIFF_BTN_PIPE0_ADDR = 0x10E68,
	.DIFF_BTN_PIPE1_ADDR = 0x12E68,
};

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_POWER,  //GESTURE_DOUBLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif


static int tpd_flag = 0;
static int tpd_halt = 0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = &buf[1];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c dummy read function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address)
{
	uint8_t buf[8] = {0};
	int32_t ret = -1;

	ret = CTP_I2C_READ(client, address, buf, 2);
	if (ret < 0)
		NVT_ERR("CTP_I2C_READ_DUMMY failed.(%d)\n", ret);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
Novatek touchscreen IC hardware reset function.

return:
n.a.
 *******************************************************/
void nvt_hw_reset(void)
{
	//---trigger rst-pin to reset---
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(20);
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(50);
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(20);
}

/*******************************************************
Description:
	Novatek touchscreen set i2c debounce function.

return:
	n.a.
*******************************************************/
void nvt_set_i2c_debounce(void)
{
	uint8_t buf[8] = {0};
	uint8_t reg1_val = 0;
	uint8_t reg2_val = 0;
	uint32_t retry = 0;

	do {
		msleep(10);

		//---dummy read to resume TP before writing command---
		CTP_I2C_READ_DUMMY(ts->client, I2C_BLDR_Address);

		// set xdata index to 0x1F000
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF0;
		CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);

		// REGW 0x36 @0x1F020
		buf[0] = 0x20;
		buf[1] = 0x36;
		CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 2);

		buf[0] = 0x20;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 2);
		reg1_val = buf[1];

		// REGW 0x00 @0x1F06F
		buf[0] = 0x6F;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 2);

		buf[0] = 0x6F;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 2);
		reg2_val = buf[1];
	} while (((reg1_val != 0x36) || (reg2_val != 0x00)) && (retry++ < 20));

	if(retry == 20) {
		NVT_ERR("set i2c debounce failed, reg1_val=0x%02X, reg2_val=0x%02X\n", reg1_val, reg2_val);
	}
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
	function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	uint8_t buf[4]={0};

	//---write i2c cmds to reset idle---
	buf[0]=0x00;
	buf[1]=0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);

	nvt_set_i2c_debounce();
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	// need 35ms delay after bootloader reset
	msleep(35);
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		msleep(10);

		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] >= check_reset_state) && (buf[1] < 0xFF)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > 200)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X\n", retry, buf[1]);
			ret = -1;
			break;
		}
	}

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t str[68] = {0};
	int32_t ret = -1;
	int32_t retries = 0;
	int8_t i2c_wr = 0;

	if (count > sizeof(str))
		return -EFAULT;

	if (copy_from_user(str, buff, count))
		return -EFAULT;

	i2c_wr = str[0] >> 7;

	if (i2c_wr == 0) {	//I2C write
		while (retries < 20) {
			ret = CTP_I2C_WRITE(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 1)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else if (i2c_wr == 1) {	//I2C read
		while (retries < 20) {
			ret = CTP_I2C_READ(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 2)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		// copy buff to user if i2c transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		return -EFAULT;
	}
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/NVTflash\n");
	NVT_LOG("============================================================\n");

	return 0;
}
#endif

#if WAKEUP_GESTURE
#define GESTURE_WORD_C			12
#define GESTURE_WORD_W			13
#define GESTURE_WORD_V			14
#define GESTURE_DOUBLE_CLICK	15
#define GESTURE_WORD_Z			16
#define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18
#define GESTURE_WORD_e			19
#define GESTURE_WORD_S			20
#define GESTURE_SLIDE_UP		21
#define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24

#if WAKEUP_GESTURE
static struct wake_lock gestrue_wakelock;
static uint8_t bTouchIsAwake = 1;
#endif

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id)
{
	uint32_t keycode = 0;

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			NVT_LOG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			NVT_LOG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			NVT_LOG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			NVT_LOG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			NVT_LOG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			NVT_LOG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			NVT_LOG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			NVT_LOG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}

	msleep(250);
}
#endif

#if POINT_DATA_CHECKSUM
/*******************************************************
Description:
	Novatek touchscreen check i2c packet checksum function.

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int32_t nvt_ts_point_data_checksum(struct i2c_client *client, uint8_t *buf, uint8_t length)
{
	uint8_t checksum = 0;
	int32_t i = 0;

	// Generate checksum
	for (i = 0; i < length; i++) {
		checksum += buf[i+1];
	}
	checksum = (~checksum + 1);

	// Compare ckecksum and dump fail data
	if (checksum != buf[length + 1]) {
		NVT_ERR("i2c packet checksum not match. (point_data[%d]=0x%02X, checksum=0x%02X)\n", (length+1), buf[length+1], checksum);

		for (i = 0; i < 10; i++) {
			NVT_ERR("%02X %02X %02X %02X %02X %02X\n", buf[1+i*6], buf[2+i*6], buf[3+i*6], buf[4+i*6], buf[5+i*6], buf[6+i*6]);
		}

		for (i = 0; i < (length - 60); i++) {
			NVT_ERR("%02X ", buf[1+60+i]);
		}

		return -1;
	}

	return 0;
}
#endif /* POINT_DATA_CHECKSUM */

#define POINT_DATA_LEN 64
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 /* RTPM_PRIO_TPD 20170118 mask */ };

	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 2] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};

	int32_t i = 0;
	int32_t finger_cnt = 0;

	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

		mutex_lock(&ts->lock);
		memset(point_data, 0, POINT_DATA_LEN + 2);

		ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 2);
		if (ret < 0) {
			NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
			goto XFER_ERROR;
		}

/*
		//--- dump I2C buf ---
		for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ", point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
		}
		printk("\n");
*/
#if POINT_DATA_CHECKSUM
		ret = nvt_ts_point_data_checksum(ts->client, point_data, POINT_DATA_LEN);
		if (ret < 0) {
			goto XFER_ERROR;
		}
#endif /* POINT_DATA_CHECKSUM */

		finger_cnt = 0;
		input_id = (uint8_t)(point_data[1] >> 3);
		memset(press_id, 0, ts->max_touch_num);

#if WAKEUP_GESTURE
		if (bTouchIsAwake == 0) {
			nvt_ts_wakeup_gesture_report(input_id);
			enable_irq(ts->client->irq);
			mutex_unlock(&ts->lock);
			continue;
		}
#endif

		if (tpd_halt) {
			mutex_unlock(&ts->lock);
			NVT_LOG("return for interrupt after suspend...\n");
			continue;
		}

#if MT_PROTOCOL_B
		for (i = 0; i < ts->max_touch_num; i++) {
			position = 1 + 6 * i;
			input_id = (uint8_t)(point_data[position + 0] >> 3);
			if (input_id > ts->max_touch_num)
				continue;

			if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
				input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
				input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
				input_w = (uint32_t)(point_data[position + 4]) + 10;
				if (input_w > 255)
					input_w = 255;
				if (i < 2) {
					input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
					if (input_p > TOUCH_FORCE_NUM)
						input_p = TOUCH_FORCE_NUM;

					// Fix : Multi-touch fingers are filterred when input_p is 0
					if (input_p == 0)
						input_p = 1;
				} else {
					input_p = 1;
				}

				if ((input_x < 0) || (input_y < 0))
					continue;
				if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
					continue;

				press_id[input_id - 1] = 1;
				input_mt_slot(ts->input_dev, input_id - 1);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);
				finger_cnt++;
			}
		}

		for (i = 0; i < ts->max_touch_num; i++) {
			if (press_id[i] != 1) {
				input_mt_slot(ts->input_dev, i);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			}
		}

		input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));

#else /* #if MT_PROTOCOL_B */

		for (i = 0; i < ts->max_touch_num; i++) {
			position = 1 + 6 * i;
			input_id = (uint8_t)(point_data[position + 0] >> 3);

			if ((point_data[position] & 0x07) == 0x03) {	// finger up (break)
				continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
			} else if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
				input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
				input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
				input_w = (uint32_t)(point_data[position + 4]) + 10;
				if (input_w > 255)
					input_w = 255;
				if (i < 2) {
					input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
					if (input_p > TOUCH_FORCE_NUM)
						input_p = TOUCH_FORCE_NUM;

					// Fix : Multi-touch fingers are filterred when input_p is 0
					if (input_p == 0)
						input_p = 1;
				} else {
					input_p = 1;
				}

				if ((input_x < 0) || (input_y < 0))
					continue;
				if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
					continue;

				press_id[input_id - 1] = 1;
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);

				input_mt_sync(ts->input_dev);

				finger_cnt++;
			}
		}
		if (finger_cnt == 0) {
			input_report_key(ts->input_dev, BTN_TOUCH, 0);

			input_mt_sync(ts->input_dev);
		}
#endif /* #if MT_PROTOCOL_B */


#if TOUCH_KEY_NUM > 0
		if (point_data[61] == 0xF8) {
			for (i = 0; i < ts->max_button_num; i++) {
				input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
			}
		} else {
			for (i = 0; i < ts->max_button_num; i++) {
				input_report_key(ts->input_dev, touch_key_array[i], 0);
			}
		}
#endif

		input_sync(ts->input_dev);

XFER_ERROR:
		enable_irq(ts->client->irq);

		mutex_unlock(&ts->lock);

	} while (!kthread_should_stop());

	return 0;
}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
	tpd_flag = 1;
	disable_irq_nosync(ts->client->irq);

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		wake_lock_timeout(&gestrue_wakelock, msecs_to_jiffies(5000));
	}
#endif

	wake_up_interruptible(&waiter);

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Register interrupt handler

return:
	irq execute status.
*******************************************************/
static int nvt_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	NVT_LOG("Device Tree Tpd_irq_registration!\n");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		//gpio_set_debounce(ints[0], ints[1]);

		ts->client->irq = irq_of_parse_and_map(node, 0);
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
		ret = request_irq(ts->client->irq, nvt_ts_irq_handler, ts->int_trigger_type, ts->client->name, ts);
		if (ret > 0) {
			ret = -1;
			NVT_ERR("tpd request_irq IRQ LINE NOT AVAILABLE!.\n");
		}
	} else {
		NVT_ERR("tpd request_irq can not find touch eint device node!.\n");
		ret = -1;
	}
	NVT_LOG("irq:%d, debounce:%d-%d:\n", ts->client->irq, ints[0], ints[1]);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t ret = -1;

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {
		nvt_bootloader_reset();
		nvt_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		msleep(10);

		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF6;
		CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		if ((buf[1] == 0x55) && (buf[2] == 0x00) && (buf[4] == 0x00) && (buf[5] == 0x00) && (buf[6] == 0x00)) {
			NVT_LOG("this is NVT touch IC\n");
			ts->mmap = &NT36772_memory_map;
			ret = 0;
			break;
		} else if ((buf[1] == 0x55) && (buf[2] == 0x72) && (buf[4] == 0x00) && (buf[5] == 0x00) && (buf[6] == 0x00)) {
			NVT_LOG("this is NVT touch IC\n");
			ts->mmap = &NT36772_memory_map;
			ret = 0;
			break;
		} else if ((buf[1] == 0xAA) && (buf[2] == 0x00) && (buf[4] == 0x00) && (buf[5] == 0x00) && (buf[6] == 0x00)) {
			NVT_LOG("this is NVT touch IC\n");
			ts->mmap = &NT36772_memory_map;
			ret = 0;
			break;
		} else if ((buf[1] == 0xAA) && (buf[2] == 0x72) && (buf[4] == 0x00) && (buf[5] == 0x00) && (buf[6] == 0x00)) {
			NVT_LOG("this is NVT touch IC\n");
			ts->mmap = &NT36772_memory_map;
			ret = 0;
			break;
		} else if ((buf[4] == 0x72) && (buf[5] == 0x67) && (buf[6] == 0x03)) {
			NVT_LOG("this is NVT touch IC\n");
			ts->mmap = &NT36772_memory_map;
			ret = 0;
			break;
		} else if ((buf[4] == 0x70) && (buf[5] == 0x66) && (buf[6] == 0x03)) {
			NVT_LOG("this is NVT touch IC\n");
			ts->mmap = &NT36772_memory_map;
			ret = 0;
			break;
		} else if ((buf[4] == 0x70) && (buf[5] == 0x67) && (buf[6] == 0x03)) {
			NVT_LOG("this is NVT touch IC\n");
			ts->mmap = &NT36772_memory_map;
			ret = 0;
			break;
		} else if ((buf[4] == 0x72) && (buf[5] == 0x66) && (buf[6] == 0x03)) {
			NVT_LOG("this is NVT touch IC\n");
			ts->mmap = &NT36772_memory_map;
			ret = 0;
			break;
		} else {
			NVT_ERR("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
				buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
			ts->mmap = NULL;
			ret = -1;
		}

		msleep(10);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif

	NVT_LOG("start\n");

	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);

	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
	//---request INT-pin---
	NVT_GPIO_AS_INT(GTP_INT_PORT);
	// set output high due to display module reset pin RESX is global reset
/*
    ret = gpio_request(P_GPIO_LCD_RST_PIN, "lcd_rst");
	if (ret)
		NVT_ERR("gpio_request (%d) failed\n", P_GPIO_LCD_RST_PIN);

	ret = gpio_direction_output(P_GPIO_LCD_RST_PIN, 0);
	if (ret)
		NVT_ERR("gpio_direction_output (%d)failed\n", P_GPIO_LCD_RST_PIN);

    gpio_set_value(P_GPIO_LCD_RST_PIN, 1);
	gpio_free(P_GPIO_LCD_RST_PIN);

	msleep(50);
20170118 mask */
	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NVT_ERR("i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	mutex_init(&ts->lock);

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{
		ret = PTR_ERR(thread);
		NVT_ERR("failed to create kernel thread: %d\n", ret);
		goto err_create_kthread_failed;
	}

	//---check input device---
	if (tpd->dev == NULL) {
		NVT_LOG("input device tpd->dev is NULL, allocate for ts->input_dev\n");
		//---allocate input device---
		ts->input_dev = input_allocate_device();
		if (ts->input_dev == NULL) {
			NVT_ERR("allocate input device failed\n");
			ret = -ENOMEM;
			goto err_input_dev_alloc_failed;
		}

		//---register input device---
		ret = input_register_device(ts->input_dev);
		if (ret) {
			NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
			goto err_input_register_device_failed;
		}
	} else {
		ts->input_dev = tpd->dev;
	}

	ts->abs_x_max = TOUCH_MAX_WIDTH;
	ts->abs_y_max = TOUCH_MAX_HEIGHT;
	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
	wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	ret = nvt_irq_registration();
	if (ret != 0) {
		NVT_ERR("request irq failed. ret=%d\n", ret);
		goto err_int_request_failed;
	} else {
		disable_irq(client->irq);
		NVT_LOG("request irq %d succeed\n", client->irq);
	}

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
#endif

	mutex_lock(&ts->lock);
	nvt_bootloader_reset();
	mutex_unlock(&ts->lock);

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

	NVT_LOG("end\n");

	enable_irq(client->irq);

	tpd_load_status = 1;
	return 0;

err_init_NVT_ts:
	free_irq(client->irq, ts);
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
err_input_register_device_failed:
	if (tpd->dev == NULL)
		input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_create_kthread_failed:
	mutex_destroy(&ts->lock);
err_chipvertrim_failed:
err_check_functionality_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	mutex_destroy(&ts->lock);

	NVT_LOG("Removing driver...\n");

	free_irq(client->irq, ts);
	if (tpd->dev == NULL)
		input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;
}

static int nvt_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, NVT_I2C_NAME);
	return 0;
}

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id nvt_match_table[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
#endif

static struct i2c_driver nvt_i2c_driver = {
	.probe = nvt_ts_probe,
	.remove = nvt_ts_remove,
	.detect = nvt_i2c_detect,
	.driver.name = NVT_I2C_NAME,
	.driver = {
		.name = NVT_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
	.id_table = nvt_ts_id,
};

static int nvt_local_init(void)
{
	int ret = 0;

	NVT_LOG("start\n");

	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret) {
		NVT_ERR("unable to add i2c driver.\n");
		return -1;
	}

	if (tpd_load_status == 0) {
		NVT_ERR("add error touch panel driver.\n");
		i2c_del_driver(&nvt_i2c_driver);
		return -1;
	}

	if (tpd_dts_data.use_tpd_button) {
		/*initialize tpd button data*/
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;

	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	tpd_halt = 1;

#if WAKEUP_GESTURE
	bTouchIsAwake = 0;

	//---write i2c command to enter "wakeup gesture mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
#if 0 // Do not set 0xFF first, ToDo
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);
#else
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif
	enable_irq_wake(ts->client->irq);

	NVT_LOG("Enabled touch wakeup gesture\n");

#else // WAKEUP_GESTURE
	disable_irq(ts->client->irq);

	//---write i2c command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x12;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif // WAKEUP_GESTURE

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

	return;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_resume(struct device *dev)
{
	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_REK);

	tpd_halt = 0;
#if WAKEUP_GESTURE
	bTouchIsAwake = 1;
#else
	enable_irq(ts->client->irq);
#endif

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

	return;
}

static struct device_attribute *novatek_attrs[] = {
};

static struct tpd_driver_t nvt_device_driver =
{
	.tpd_device_name = NVT_I2C_NAME,
	.tpd_local_init = nvt_local_init,
	.suspend = nvt_ts_suspend,
	.resume = nvt_ts_resume,
	.attrs = {
		.attr = novatek_attrs,
		.num  = ARRAY_SIZE(novatek_attrs),
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("start\n");
	tpd_get_dts_info();

	ret = tpd_driver_add(&nvt_device_driver);
	if (ret < 0){
		NVT_ERR("failed to add i2c driver");
		goto err_driver;
	}

	NVT_LOG("end\n");

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	tpd_driver_remove(&nvt_device_driver);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif
}

module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
