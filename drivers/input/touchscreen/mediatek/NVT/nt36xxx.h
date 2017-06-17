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
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/i2c.h>
#include <linux/input.h>

#define NVT_DEBUG 1

#define NVT_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define NVT_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)

//---INT trigger mode---
//#define EINTF_TRIGGER_RISING     0x00000001
//#define EINTF_TRIGGER_FALLING    0x00000002
//#define EINTF_TRIGGER_HIGH       0x00000004
//#define EINTF_TRIGGER_LOW        0x00000008
#define INT_TRIGGER_TYPE IRQF_TRIGGER_RISING

//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define I2C_BLDR_Address 0x01
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"

//---Touch info.---
#define TOUCH_MAX_WIDTH 1080
#define TOUCH_MAX_HEIGHT 1920
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 1
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif

#define BOOT_UPDATE_FIRMWARE 0 /* 20170214 mask for factory */
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"
#define POINT_DATA_CHECKSUM 0

struct nvt_ts_mem_map {
	uint32_t EVENT_BUF_ADDR;
	uint32_t RAW_PIPE0_ADDR;
	uint32_t RAW_PIPE1_ADDR;
	uint32_t BASELINE_ADDR;
	uint32_t BASELINE_BTN_ADDR;
	uint32_t DIFF_PIPE0_ADDR;
	uint32_t DIFF_PIPE1_ADDR;
	uint32_t RAW_BTN_PIPE0_ADDR;
	uint32_t RAW_BTN_PIPE1_ADDR;
	uint32_t DIFF_BTN_PIPE0_ADDR;
	uint32_t DIFF_BTN_PIPE1_ADDR;
};

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct nvt_work;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
	struct i2c_client *client;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN	// normal run
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
} I2C_EVENT_MAP;

#endif /* _LINUX_NVT_TOUCH_H */
