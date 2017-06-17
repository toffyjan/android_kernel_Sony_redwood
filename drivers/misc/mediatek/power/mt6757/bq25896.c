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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include <mach/mtk_charging.h>
#include "bq25896.h"
#include "mtk_bif_intf.h"
#include "mtk_charger_intf.h"


/* == This section is added to make code the same as MTK == */
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0

#if defined(GPIO_PWR_AVAIL_WLC)
/*K.S.?*/
unsigned int wireless_charger_gpio_number = GPIO_PWR_AVAIL_WLC;
#else
unsigned int wireless_charger_gpio_number;
#endif

#endif
/* ========================================================  */


#ifdef CONFIG_RT_REGMAP
#include <mt-plat/rt-regmap.h>
#endif

#define I2C_ACCESS_MAX_RETRY	5

enum bq25896_charging_status {
	BQ25896_CHG_STATUS_NOT_CHARGING = 0,
	BQ25896_CHG_STATUS_PRECHARGE,
	BQ25896_CHG_STATUS_FAST_CHARGE,
	BQ25896_CHG_STATUS_DONE,
	BQ25896_CHG_STATUS_MAX,
};

enum bq25896_fault_status {
	BQ25896_FAULT_STATUS_NORMAL = 0,
	BQ25896_FAULT_STATUS_INPUT_ERR,
	BQ25896_FAULT_STATUS_THERMAL_SHUTDOWN,
	BQ25896_FAULT_STATUS_SAFETY_TIMER_EXP,
	BQ25896_FAULT_STATUS_MAX,
};

/* Charging status name */
static const char *bq25896_chg_status_name[BQ25896_CHG_STATUS_MAX] = {
	"not charging", "precharge", "fast charge", "done",
};

/* Charging fault status name */
static const char *bq25896_chg_fault_status_name[BQ25896_FAULT_STATUS_MAX] = {
	"normal", "Input Error", "Thermal Shutdown", "Safety Timer Exp",
};


/* BQ25896 REG0A BOOST_LIM[2:0], mA */
static const u32 bq25896_boost_current_limit[] = {
	500, 750, 1200, 1400, 1650,
};

/* BQ25896 REG07 WDT[5:4] ms */
static const u32 bq25896_wdt[] = {
	0, 40000, 80000, 160000,
};

static const u32 bq25896_chg_timer[] = {
	5, 8, 12, 20,
};

/* BQ25896 REG0A BOOST_LIM[2:0] mA */
static const u32 bq25896_boost_lim[] = {
	500, 750, 1200, 1400, 1650, 1875, 2150,
};

/* ========= */
/* RT Regmap */
/* ========= */

#ifdef CONFIG_RT_REGMAP
RT_REG_DECL(BQ25896_CON0, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON1, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON2, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON3, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON4, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON5, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON6, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON7, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON8, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON9, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CONA, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CONB, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CONC, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_COND, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CONE, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CONF, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON10, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON11, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON12, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON13, 1, RT_VOLATILE, {});
RT_REG_DECL(BQ25896_CON14, 1, RT_VOLATILE, {});

static rt_register_map_t bq25896_regmap_map[] = {
	RT_REG(BQ25896_CON0),
	RT_REG(BQ25896_CON1),
	RT_REG(BQ25896_CON2),
	RT_REG(BQ25896_CON3),
	RT_REG(BQ25896_CON4),
	RT_REG(BQ25896_CON5),
	RT_REG(BQ25896_CON6),
	RT_REG(BQ25896_CON7),
	RT_REG(BQ25896_CON8),
	RT_REG(BQ25896_CON9),
	RT_REG(BQ25896_CONA),
	RT_REG(BQ25896_CONB),
	RT_REG(BQ25896_CONC),
	RT_REG(BQ25896_COND),
	RT_REG(BQ25896_CONE),
	RT_REG(BQ25896_CONF),
	RT_REG(BQ25896_CON10),
	RT_REG(BQ25896_CON11),
	RT_REG(BQ25896_CON12),
	RT_REG(BQ25896_CON13),
	RT_REG(BQ25896_CON14),
};
#endif /* CONFIG_RT_REGMAP */

struct bq25896_info {
	struct mtk_charger_info mchr_info;
	int i2c_log_level;
	struct i2c_client *i2c;
	struct mutex i2c_access_lock;
	bool err_state;
	u32 input_current;
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *regmap_dev;
	struct rt_regmap_properties *regmap_prop;
#endif
};

//CEI comments start, identify which charegr IC is using
unsigned int bq25896_whoami  = 0;
//CEI comments end, identify which charegr IC is using

/* ========================= */
/* I2C operations            */
/* ========================= */

static int bq25896_device_read(void *client, u32 addr, int leng, void *dst)
{
	int ret = 0;
	struct i2c_client *i2c = NULL;

	i2c = (struct i2c_client *)client;
	ret = i2c_smbus_read_i2c_block_data(i2c, addr, leng, dst);

	return ret;
}

static int bq25896_device_write(void *client, u32 addr, int leng,
	const void *src)
{
	int ret = 0;
	struct i2c_client *i2c = NULL;

	i2c = (struct i2c_client *)client;
	ret = i2c_smbus_write_i2c_block_data(i2c, addr, leng, src);

	return ret;
}

#ifdef CONFIG_RT_REGMAP
static struct rt_regmap_fops bq25896_regmap_fops = {
	.read_device = bq25896_device_read,
	.write_device = bq25896_device_write,
};

static int bq25896_register_rt_regmap(struct bq25896_info *info)
{
	int ret = 0;
	struct i2c_client *i2c = info->i2c;
	struct rt_regmap_properties *prop = NULL;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);

	prop = devm_kzalloc(&i2c->dev, sizeof(struct rt_regmap_properties),
		GFP_KERNEL);
	if (!prop) {
		battery_log(BAT_LOG_CRTI, "%s: no enough memory\n", __func__);
		return -ENOMEM;
	}

	prop->name = "bq25896";
	prop->aliases = "bq25896";
	prop->register_num = ARRAY_SIZE(bq25896_regmap_map),
	prop->rm = bq25896_regmap_map,
	prop->rt_regmap_mode = RT_SINGLE_BYTE | RT_CACHE_DISABLE | RT_IO_PASS_THROUGH,
	prop->io_log_en = 0,

	info->regmap_prop = prop;
	info->regmap_dev = rt_regmap_device_register_ex(
		info->regmap_prop,
		&bq25896_regmap_fops,
		&i2c->dev,
		i2c,
		BQ25896_SLAVE_ADDR,
		info
	);

	if (!info->regmap_dev) {
		dev_err(&i2c->dev, "register regmap device failed\n");
		return -EIO;
	}

	battery_log(BAT_LOG_CRTI, "%s: ends\n", __func__);

	return ret;
}
#endif /* CONFIG_RT_REGMAP */

static inline int _bq25896_i2c_write_byte(struct bq25896_info *info, u8 cmd,
	u8 data)
{
	int ret = 0, retry = 0;

	do {
#ifdef CONFIG_RT_REGMAP
		ret = rt_regmap_block_write(info->regmap_dev, cmd, 1, &data);
#else
		ret = bq25896_device_write(info->i2c, cmd, 1, &data);
#endif
		retry++;
		if (ret < 0)
			mdelay(20);
	} while (ret < 0 && retry < I2C_ACCESS_MAX_RETRY);

	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: I2CW[0x%02X] = 0x%02X failed\n",
			__func__, cmd, data);
	else
		battery_log(info->i2c_log_level, "%s: I2CW[0x%02X] = 0x%02X\n",
			__func__, cmd, data);

	return ret;
}

#if 0
static int bq25896_i2c_write_byte(struct bq25896_info *info, u8 cmd, u8 data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _bq25896_i2c_write_byte(info, cmd, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}
#endif

static inline int _bq25896_i2c_read_byte(struct bq25896_info *info, u8 cmd)
{
	int ret = 0, ret_val = 0, retry = 0;

	do {
#ifdef CONFIG_RT_REGMAP
		ret = rt_regmap_block_read(info->regmap_dev, cmd, 1, &ret_val);
#else
		ret = bq25896_device_read(info->i2c, cmd, 1, &ret_val);
#endif
		retry++;
		if (ret < 0)
			msleep(20);
	} while (ret < 0 && retry < I2C_ACCESS_MAX_RETRY);

	if (ret < 0) {
		battery_log(BAT_LOG_CRTI, "%s: I2CR[0x%02X] failed\n",
			__func__, cmd);
		return ret;
	}

	ret_val = ret_val & 0xFF;

	battery_log(info->i2c_log_level, "%s: I2CR[0x%02X] = 0x%02X\n",
		__func__, cmd, ret_val);

	return ret_val;
}

static int bq25896_i2c_read_byte(struct bq25896_info *info, u8 cmd)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _bq25896_i2c_read_byte(info, cmd);
	mutex_unlock(&info->i2c_access_lock);

	if (ret < 0)
		return ret;

	return (ret & 0xFF);
}

#if 0
static inline int _bq25896_i2c_block_write(struct bq25896_info *info, u8 cmd,
	u32 leng, const u8 *data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_write(info->regmap_dev, cmd, leng, data);
#else
	ret = bq25896_device_write(info->i2c, cmd, leng, data);
#endif

	return ret;
}


static int bq25896_i2c_block_write(struct bq25896_info *info, u8 cmd, u32 leng,
	const u8 *data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _bq25896_i2c_block_write(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int _bq25896_i2c_block_read(struct bq25896_info *info, u8 cmd,
	u32 leng, u8 *data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(info->regmap_dev, cmd, leng, data);
#else
	ret = bq25896_device_read(info->i2c, cmd, leng, data);
#endif

	return ret;
}


static int bq25896_i2c_block_read(struct bq25896_info *info, u8 cmd, u32 leng,
	u8 *data)
{
	int ret = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _bq25896_i2c_block_read(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}
#endif


static int bq25896_i2c_test_bit(struct bq25896_info *info, u8 cmd, u8 shift)
{
	int ret = 0;

	ret = bq25896_i2c_read_byte(info, cmd);
	if (ret < 0)
		return ret;

	ret = ret & (1 << shift);

	return ret;
}

static int bq25896_i2c_update_bits(struct bq25896_info *info, u8 cmd, u8 data,
	u8 mask)
{
	int ret = 0;
	u8 reg_data = 0;

	mutex_lock(&info->i2c_access_lock);
	ret = _bq25896_i2c_read_byte(info, cmd);
	if (ret < 0) {
		mutex_unlock(&info->i2c_access_lock);
		return ret;
	}

	reg_data = ret & 0xFF;
	reg_data &= ~mask;
	reg_data |= (data & mask);

	ret = _bq25896_i2c_write_byte(info, cmd, reg_data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int bq25896_set_bit(struct bq25896_info *info, u8 reg, u8 mask)
{
	return bq25896_i2c_update_bits(info, reg, mask, mask);
}

static inline int bq25896_clr_bit(struct bq25896_info *info, u8 reg, u8 mask)
{
	return bq25896_i2c_update_bits(info, reg, 0x00, mask);
}


/* ================== */
/* Internal Functions */
/* ================== */
static int bq_charger_get_iinlim(struct mtk_charger_info *mchr_info,
	void *data);
static int bq_charger_set_iinlim(struct mtk_charger_info *mchr_info,
	void *data);
static int bq_charger_get_ichg(struct mtk_charger_info *mchr_info, void *data);
static int bq_charger_set_ichg(struct mtk_charger_info *mchr_info, void *data);
static int bq_charger_set_vindpm(struct mtk_charger_info *mchr_info,
	void *data);
static int bq_charger_enable_safety_timer(struct mtk_charger_info *mchr_info,
	void *data);
static int bq_charger_set_ircmp_resistor(struct mtk_charger_info *mchr_info,
	void *data);
static int bq_charger_set_ircmp_vclamp(struct mtk_charger_info *mchr_info,
	void *data);
static int bq_charger_enable_hz(struct mtk_charger_info *mchr_info,
	void *data);
static int bq_charger_set_cv_voltage(struct mtk_charger_info *mchr_info,
	void *data);
static int bq_charger_is_power_path_enable(struct mtk_charger_info *mchr_info,
	void *data);

static u8 bq25896_find_closest_reg_value(const u32 min, const u32 max,
	const u32 step, const u32 num, const u32 target)
{
	u32 i = 0, cur_val = 0, next_val = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target < min)
		return 0;

	for (i = 0; i < num - 1; i++) {
		cur_val = min + i * step;
		next_val = cur_val + step;

		if (cur_val > max)
			cur_val = max;

		if (next_val > max)
			next_val = max;

		if (target >= cur_val && target < next_val)
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return num - 1;
}

static u8 bq25896_find_closest_reg_value_via_table(const u32 *value_table,
	const u32 table_size, const u32 target_value)
{
	u32 i = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target_value < value_table[0])
		return 0;

	for (i = 0; i < table_size - 1; i++) {
		if (target_value >= value_table[i] &&
		    target_value < value_table[i + 1])
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return table_size - 1;
}

static u32 bq25896_find_closest_real_value(const u32 min, const u32 max,
	const u32 step, const u8 reg_val)
{
	u32 ret_val = 0;

	ret_val = min + reg_val * step;
	if (ret_val > max)
		ret_val = max;

	return ret_val;
}


/* CON1---------------------------------------------------- */

static int bq25896_set_vindpm_offset(struct bq25896_info *info, u32 offset)
{
	int ret = 0;
	u8 reg_offset = 0;

	/* Find corresponding reg value */
	reg_offset = bq25896_find_closest_reg_value(BQ25896_VINDPM_OS_MIN,
		BQ25896_VINDPM_OS_MAX, BQ25896_VINDPM_OS_STEP,
		BQ25896_VINDPM_OS_NUM, offset);

	battery_log(BAT_LOG_CRTI, "%s: vindpm os = %dmV\n", __func__, offset);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON1,
		reg_offset << CON1_VINDPM_OS_SHIFT,
		CON1_VINDPM_OS_MASK
	);

	return ret;
}


/* CON2---------------------------------------------------- */

static int bq25896_enable_ico(struct bq25896_info *info, bool enable)
{
	int ret = 0;

	ret = (enable ? bq25896_set_bit : bq25896_clr_bit)
		(info, BQ25896_CON2, CON2_ICO_EN_MASK);

	return ret;
}

static int bq25896_adc_start(struct bq25896_info *info, bool enable)
{
	int ret = 0;

	ret = (enable ? bq25896_set_bit : bq25896_clr_bit)
		(info, BQ25896_CON2, CON2_CONV_START_MASK);

	return ret;
}


/* CON3---------------------------------------------------- */

static int bq25896_is_charging_enable(struct bq25896_info *info, bool *enable)
{
	int ret = 0;

	ret = bq25896_i2c_read_byte(info, BQ25896_CON3);
	if (ret < 0)
		return ret;

	*enable = ((ret & CON3_CHG_CONFIG_MASK) >> CON3_CHG_CONFIG_SHIFT) > 0 ?
		true : false;

	return ret;
}

static int bq25896_set_sys_min(struct bq25896_info *info, u32 sys_min)
{
	int ret = 0;
	u8 reg_sys_min = 0;

	/* Find corresponding reg value */
	reg_sys_min = bq25896_find_closest_reg_value(BQ25896_SYS_MIN_MIN,
		BQ25896_SYS_MIN_MAX, BQ25896_SYS_MIN_STEP,
		BQ25896_SYS_MIN_NUM, sys_min);

	battery_log(BAT_LOG_CRTI, "%s: sys min = %dmV\n", __func__, sys_min);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON3,
		reg_sys_min << CON3_SYS_V_LIMIT_SHIFT,
		CON3_SYS_V_LIMIT_MASK
	);

	return ret;
}


/* CON4---------------------------------------------------- */

static int bq25896_enable_pumpx(struct bq25896_info *info, bool enable)
{
	int ret = 0;

	ret = (enable ? bq25896_set_bit : bq25896_clr_bit)
		(info, BQ25896_CON4, CON4_EN_PUMPX_MASK);

	return ret;
}

/* CON5---------------------------------------------------- */

static int bq25896_set_iprec(struct bq25896_info *info, u32 iprec)
{
	int ret = 0;
	u8 reg_iprec = 0;

	/* Find corresponding reg value */
	reg_iprec = bq25896_find_closest_reg_value(BQ25896_IPREC_MIN,
		BQ25896_IPREC_MAX, BQ25896_IPREC_STEP, BQ25896_IPREC_NUM, iprec);

	battery_log(BAT_LOG_CRTI, "%s: iprec = %dmA\n", __func__, iprec);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON5,
		reg_iprec << CON5_IPRECHG_SHIFT,
		CON5_IPRECHG_MASK
	);

	return ret;
}

static int bq25896_set_iterm(struct bq25896_info *info, u32 iterm)
{
	int ret = 0;
	u8 reg_iterm = 0;

	/* Find corresponding reg value */
	reg_iterm = bq25896_find_closest_reg_value(BQ25896_ITERM_MIN,
		BQ25896_ITERM_MAX, BQ25896_ITERM_STEP, BQ25896_ITERM_NUM, iterm);

	battery_log(BAT_LOG_CRTI, "%s: iterm = %dmA\n", __func__, iterm);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON5,
		reg_iterm << CON5_ITERM_SHIFT,
		CON5_ITERM_MASK
	);

	return ret;
}


/* CON7---------------------------------------------------- */
static int bq25896_set_wdt(struct bq25896_info *info, u32 wdt)
{
	int ret = 0;
	u8 reg_wdt = 0;

	/* Find corresponding reg value */
	reg_wdt = bq25896_find_closest_reg_value_via_table(
		bq25896_wdt,
		ARRAY_SIZE(bq25896_wdt),
		wdt
	);

	battery_log(BAT_LOG_CRTI, "%s: wdt = %dms\n", __func__, wdt);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON7,
		reg_wdt << CON7_WTG_TIM_SET_SHIFT,
		CON7_WTG_TIM_SET_MASK
	);

	return ret;
}

static int bq25896_set_chg_timer(struct bq25896_info *info, u32 chg_timer)
{
	int ret = 0;
	u8 reg_chg_timer = 0;

	/* Find corresponding reg value */
	reg_chg_timer = bq25896_find_closest_reg_value_via_table(
		bq25896_chg_timer,
		ARRAY_SIZE(bq25896_chg_timer),
		chg_timer
	);

	battery_log(BAT_LOG_CRTI, "%s: chg timer = %dhr\n", __func__, chg_timer);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON7,
		reg_chg_timer << CON7_SET_CHG_TIM_SHIFT,
		CON7_SET_CHG_TIM_MASK
	);

	return ret;
}

/* CON8---------------------------------------------------- */

static int bq25896_set_treg(struct bq25896_info *info, u32 treg)
{
	int ret = 0;
	u8 reg_treg = 0;

	/* Find corresponding reg value */
	reg_treg = bq25896_find_closest_reg_value(BQ25896_TREG_MIN,
		BQ25896_TREG_MAX, BQ25896_TREG_STEP,
		BQ25896_TREG_NUM, treg);

	battery_log(BAT_LOG_CRTI, "%s: treg = %ddegree\n", __func__, treg);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON8,
		reg_treg << CON8_TREG_SHIFT,
		CON8_TREG_MASK
	);

	return ret;
}


/* CON9---------------------------------------------------- */
static int bq25896_pumpx_up(struct bq25896_info *info, bool up)
{
	int ret = 0;
	u32 iinlim = 50000;	/* 10uA */
	u32 ichg = 204800;	/* 10uA */
	u8 mask = (up ? CON9_PUMPX_UP_MASK : CON9_PUMPX_DN_MASK);

	ret = bq25896_enable_pumpx(info, true);
	ret = bq25896_set_bit(info, BQ25896_CON9, mask);

	/* Input current limit = 500 mA, changes after PE+ detection */
	ret = bq_charger_set_iinlim(&info->mchr_info, &iinlim);

	/* CC mode current = 2048 mA */
	ret = bq_charger_set_ichg(&info->mchr_info, &ichg);

	msleep(3000);

	return ret;
}

//CEI comment start//
#ifdef CONFIG_BUILD_REDWOOD // redwood
#define COND_JEITA_VSET_MASK 0x10
static int bq25896_jeita_vset(struct bq25896_info *info, bool jeita_vset)
{
	int ret = 0;

	battery_log(BAT_LOG_FULL, "%s: reg_jeita_vset = %d\n", __func__, jeita_vset);
	ret = (jeita_vset ? bq25896_set_bit : bq25896_clr_bit)
		(info, BQ25896_CON9, COND_JEITA_VSET_MASK);

	return ret;
}
#endif
//CEI comment end//

/* CONA---------------------------------------------------- */
static int bq25896_set_boostv(struct bq25896_info *info, u32 boostv)
{
	int ret = 0;
	u8 reg_boostv = 0;

	/* Find corresponding reg value */
	reg_boostv = bq25896_find_closest_reg_value(BQ25896_BOOSTV_MIN,
		BQ25896_BOOSTV_MAX, BQ25896_BOOSTV_STEP, BQ25896_BOOSTV_NUM,
		boostv);

	battery_log(BAT_LOG_CRTI, "%s: boostv = %d\n", __func__, boostv);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CONA,
		reg_boostv << CONA_BOOST_VLIM_SHIFT,
		CONA_BOOST_VLIM_MASK
	);

	return ret;
}


static int bq25896_set_boost_lim(struct bq25896_info *info, u32 boost_lim)
{
	int ret = 0;
	u8 reg_boost_lim = 0;

	/* Find corresponding reg value */
	reg_boost_lim = bq25896_find_closest_reg_value_via_table(
		bq25896_boost_lim,
		ARRAY_SIZE(bq25896_boost_lim),
		boost_lim
	);

	battery_log(BAT_LOG_CRTI, "%s: boost lim = %dms\n", __func__, boost_lim);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CONA,
		reg_boost_lim << CONA_BOOST_ILIM_SHIFT,
		CONA_BOOST_ILIM_MASK
	);

	return ret;
}

/* CONB---------------------------------------------------- */


static int bq25896_get_charging_status(struct bq25896_info *info,
	enum bq25896_charging_status *chg_stat)
{
	int ret = 0;

	ret = bq25896_i2c_read_byte(info, BQ25896_CONB);
	if (ret < 0)
		return ret;

	*chg_stat = (ret & CONB_CHRG_STAT_MASK) >> CONB_CHRG_STAT_SHIFT;

	return ret;
}


/* CONC----------------------------------------------------- */

static int bq25896_get_chg_fault_state(struct bq25896_info *info,
	u8 *fault_state)
{
	int ret = 0;

	ret = bq25896_i2c_read_byte(info, BQ25896_CONC);
	if (ret < 0)
		return ret;

	*fault_state = (ret & CONC_CHRG_FAULT_MASK) >> CONC_CHRG_FAULT_SHIFT;

	return ret;
}


/* COND */
static int bq25896_set_force_vindpm(struct bq25896_info *info, bool force)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: force = %d\n", __func__, force);
	ret = (force ? bq25896_set_bit : bq25896_clr_bit)
		(info, BQ25896_COND, COND_FORCE_VINDPM_MASK);

	return ret;
}

static int bq25896_set_vindpm(struct bq25896_info *info, u32 vindpm)
{
	int ret = 0;
	u8 reg_vindpm = 0;

	/* Find corresponding reg value */
	reg_vindpm = bq25896_find_closest_reg_value(BQ25896_VINDPM_MIN,
		BQ25896_VINDPM_MAX, BQ25896_VINDPM_STEP, BQ25896_VINDPM_NUM,
		vindpm);

	battery_log(BAT_LOG_CRTI, "%s: vindpm = %d\n", __func__, vindpm);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_COND,
		reg_vindpm << COND_VINDPM_SHIFT,
		COND_VINDPM_MASK
	);

	return ret;
}

static int bq25896_get_vindpm(struct bq25896_info *info, u32 *vindpm)
{
	int ret = 0;
	u8 reg_vindpm = 0;

	ret = bq25896_i2c_read_byte(info, BQ25896_COND);
	if (ret < 0)
		return ret;

	reg_vindpm = (ret & COND_VINDPM_MASK) >> COND_VINDPM_SHIFT;
	*vindpm = bq25896_find_closest_real_value(BQ25896_VINDPM_MIN,
		BQ25896_VINDPM_MAX, BQ25896_VINDPM_STEP, reg_vindpm);

	return ret;
}

/* CONDE */
static int bq25896_get_adc_vbat(struct bq25896_info *info, u32 *adc_vbat)
{
	int ret = 0;
	u8 reg_adc_vbat = 0;

	ret = bq25896_i2c_read_byte(info, BQ25896_CONE);
	if (ret < 0)
		return ret;

	reg_adc_vbat = (ret & CONE_VBAT_MASK) >> CONE_VBAT_SHIFT;
	*adc_vbat = bq25896_find_closest_real_value(BQ25896_ADC_VBAT_MIN,
		BQ25896_ADC_VBAT_MAX, BQ25896_ADC_VBAT_STEP, reg_adc_vbat);

	return ret;
}

/* CON11 */
static int bq25896_get_adc_vbus(struct bq25896_info *info, u32 *adc_vbus)
{
	int ret = 0;
	u8 reg_adc_vbus = 0;

	ret = bq25896_i2c_read_byte(info, BQ25896_CON11);
	if (ret < 0)
		return ret;

	reg_adc_vbus = (ret & CON11_VBUS_MASK) >> CON11_VBUS_SHIFT;
	*adc_vbus = bq25896_find_closest_real_value(BQ25896_ADC_VBUS_MIN,
		BQ25896_ADC_VBUS_MAX, BQ25896_ADC_VBUS_STEP, reg_adc_vbus);

	return ret;
}

/* CON12 */
static int bq25896_get_adc_ibat(struct bq25896_info *info, u32 *adc_ibat)
{
	int ret = 0;
	u8 reg_adc_ibat = 0;

	ret = bq25896_i2c_read_byte(info, BQ25896_CON12);
	if (ret < 0)
		return ret;

	reg_adc_ibat = (ret & CONB_ICHG_STAT_MASK) >> CONB_ICHG_STAT_SHIFT;
	*adc_ibat = bq25896_find_closest_real_value(BQ25896_ADC_IBAT_MIN,
		BQ25896_ADC_IBAT_MAX, BQ25896_ADC_IBAT_STEP, reg_adc_ibat);

	return ret;
}



/* CON13 */


static int bq25896_is_in_vindpm(struct bq25896_info *info, bool *in_vindpm)
{
	int ret = 0;

	ret = bq25896_i2c_test_bit(info, BQ25896_CON13, CON13_VDPM_STAT_SHIFT);
	if (ret < 0)
		return ret;

	*in_vindpm = (ret > 0 ? true : false);

	return 0;
}


static bool bq25896_is_hw_exist(struct bq25896_info *info)
{
	int ret = 0;
	u8 revision = 0;

	ret = i2c_smbus_read_byte_data(info->i2c, BQ25896_CON14);
	if (ret < 0)
		return false;

	revision = ret & 0x03;
	if (revision == BQ25896_DEVICE_ID_E1)
		battery_log(BAT_LOG_CRTI, "%s: E1(0x%02X)", __func__, revision);
	else if (revision == BQ25896_DEVICE_ID_E2)
		battery_log(BAT_LOG_CRTI, "%s: E2(0x%02X)", __func__, revision);
	else if (revision == BQ25896_DEVICE_ID_E3)
		battery_log(BAT_LOG_CRTI, "%s: E3(0x%02X)", __func__, revision);
	else if (revision == BQ25896_DEVICE_ID_E4)
		battery_log(BAT_LOG_CRTI, "%s: E4(0x%02X)", __func__, revision);
	else
		return false;

	info->mchr_info.device_id = revision;
	return true;
}

/* ============================================================ */
/* The following is implementation for interface of bq_charger */
/* ============================================================ */
//CEI comment start//
#ifdef CONFIG_BUILD_REDWOOD // redwood
#include <../../../misc/mediatek/cei_hw_id/cei_hw_id.h>
#endif
//CEI comment end//

static int bq_charger_hw_init(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	ret = bq25896_set_force_vindpm(info, true);

#ifdef CONFIG_BUILD_REDWOOD // redwood
	if(get_cei_hw_id() == 0x2) //SP2
		bq25896_jeita_vset(info, true);
#endif

/* == This section is added to make code the same as MTK == */
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	if (wireless_charger_gpio_number != 0) {
#ifdef CONFIG_MTK_LEGACY
		mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
		mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
#else
/*K.S. way here*/
#endif
	}
#endif

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(vin_sel_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(vin_sel_gpio_number, 0);	/* 0: input, 1: output */
#else
/*K.S. way here*/
#endif
#endif
/* ========================================================= */

	return ret;
}

/* == Initialization is implemented here to make code the same as MTK == */
static int bq_charger_sw_init(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u32 iinlim = 325000;
	u32 ichg = 51200;
	u32 vindpm = 4500;
	u32 cv = 4352000;
	u32 ircmp_resistor = 0, ircmp_vclamp = 0;
	bool enable_safety_timer = false;//CEI comments, disable safety timer
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	battery_log(BAT_LOG_CRTI, "%s\n", __func__);

	/* BIF init */
	mtk_bif_init();

#ifdef CONFIG_BUILD_REDWOOD // redwood
	if(get_cei_hw_id() == 0x2) //SP2
		bq25896_jeita_vset(info, true);
#endif

	bq25896_set_bit(info, BQ25896_CON0, CON0_EN_ILIM_MASK);		/* enable ilimit Pin */
	bq25896_set_vindpm_offset(info, 600);				/* Vindpm offset  600MV */
	bq25896_set_force_vindpm(info, true);				/* vindpm vth 0:relative 1:absolute */

	bq_charger_set_ichg(&info->mchr_info, &ichg);			/* ICHG (0x08)512mA --> (0x20)2.048mA */
	bq_charger_set_iinlim(&info->mchr_info, &iinlim);		/* input current limit, IINLIM, 3.25A */

	/* absolute VINDPM = 2.6 + code x 0.1 =4.5V;K2 24261 4.452V */
	bq_charger_set_vindpm(&info->mchr_info, &vindpm);
	bq_charger_set_cv_voltage(&info->mchr_info, &cv);		/* VREG=CV 4.352V (default 4.208V) */

	/* The following setting is moved from HW_INIT */
	bq25896_enable_ico(info, true);					/* disable ico Algorithm -->bear:en */
	bq25896_clr_bit(info, BQ25896_CON2, CON2_HVDCP_EN_MASK);	/* disable HV DCP for gq25897 */
	bq25896_clr_bit(info, BQ25896_CON2, CON2_MAX_EN_MASK);		/* disbale MaxCharge for gq25897 */
	bq25896_clr_bit(info, BQ25896_CON2, CON2_FORCE_DPDM_MASK);	/* disable DPDM detection */

	bq25896_set_wdt(info, 40000);					/* enable  watch dog 40 secs 0x1 */
	/* enable charging timer safety timer */
	bq_charger_enable_safety_timer(&info->mchr_info, &enable_safety_timer);
//CEI comments, safty timer S
	//bq25896_set_chg_timer(info, 12);				/* charging timer 12h */
	bq25896_set_chg_timer(info, 0);
//CEI comments, safty timer E
	bq25896_clr_bit(info, BQ25896_CON2, CON2_BOOST_FREQ_MASK);	/* boost freq 1.5MHz when OTG_CONFIG=1 */
	bq25896_set_boostv(info, 4998);					/* boost voltagte 4.998V default */
	bq25896_set_boost_lim(info, 1400);				/* boost current limit 1.4A */

#ifdef CONFIG_MTK_BIF_SUPPORT
	bq_charger_set_ircmp_resistor(&info->mchr_info, &ircmp_resistor);	/* disable ir_comp_resistance */
	bq_charger_set_ircmp_vclamp(&info->mchr_info, &ircmp_vclamp);		/* disable ir_comp_vdamp */
#else
//CEI comments, disable ir_comp start
	ircmp_resistor = 0; //ori = 80
	ircmp_vclamp = 0; //ori = 192
//CEI comments, disable ir_comp start
	bq_charger_set_ircmp_resistor(&info->mchr_info, &ircmp_resistor);	/* enable ir_comp_resistance */
	bq_charger_set_ircmp_vclamp(&info->mchr_info, &ircmp_vclamp);		/* enable ir_comp_vdamp */
#endif

	bq25896_set_treg(info, 120);					/* thermal 120 default */

	bq25896_clr_bit(info, BQ25896_CON9, CON9_JEITA_VSET_MASK);	/* JEITA_VSET: VREG-200mV */
	bq25896_set_bit(info, BQ25896_CON7, CON7_JEITA_ISET_MASK);	/* JEITA_ISet : 20% x ICHG */

	bq25896_set_sys_min(info, 3500);				/* System min voltage default 3.5V */

	/*PreCC mode */
	bq25896_set_iprec(info, 128);					/* precharge current default 128mA */
	bq25896_set_bit(info, BQ25896_CON6, CON6_BATLOWV_MASK);		/* precharge2cc voltage,BATLOWV, 3.0V */
	/*CV mode */
	bq25896_clr_bit(info, BQ25896_CON6, CON6_VRECHG_MASK);		/* recharge voltage@VRECHG=CV-100MV */
	bq25896_set_bit(info, BQ25896_CON7, CON7_EN_TERM_CHG_MASK);	/* enable ICHG termination detect */
	bq25896_set_iterm(info, 128);					/* termianation current default 128mA */

	return ret;
}

static int bq_charger_dump_register(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0, i = 0;
	u32 ichg = 0, iinlim = 0, vindpm = 0;
	u32 adc_ibat = 0, adc_vbat = 0, adc_vbus = 0;
	bool chg_en = false, in_vindpm = false;
	u8 chg_fault = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;
	enum bq25896_charging_status chg_stat = BQ25896_CHG_STATUS_NOT_CHARGING;

//CEI comment start//
	ret = bq25896_get_chg_fault_state(info, &chg_fault);
	if (chg_fault != BQ25896_FAULT_STATUS_NORMAL) {
		info->i2c_log_level = BAT_LOG_CRTI;
		bq25896_adc_start(info, true);
		for (i = 0; i < BQ25896_REG_NUM; i++) {
			ret = bq25896_i2c_read_byte(info, i);
			if (ret < 0)
				continue;
		}
	} else
		info->i2c_log_level = BAT_LOG_FULL;

	bq25896_adc_start(info, true);

	for (i = 0; i < BQ25896_REG_NUM/7; i++) {
		int j;
		int bq25896_reg_buf[7];
		memset(bq25896_reg_buf, 0x0, sizeof(bq25896_reg_buf));
		for(j = 0; j < 7; j++)
		{
			bq25896_reg_buf[j] = bq25896_i2c_read_byte(info, (i*7)+j);
		}

		battery_log(BAT_LOG_CRTI, "LE(K)=> [bq25896 reg@][0x%x]=0x%x [0x%x]=0x%x [0x%x]=0x%x [0x%x]=0x%x [0x%x]=0x%x [0x%x]=0x%x [0x%x]=0x%x\n",
					(i*7)+0, bq25896_reg_buf[0], (i*7)+1, bq25896_reg_buf[1], (i*7)+2, bq25896_reg_buf[2],
					(i*7)+3, bq25896_reg_buf[3], (i*7)+4, bq25896_reg_buf[4], (i*7)+5, bq25896_reg_buf[5],
					(i*7)+6, bq25896_reg_buf[6]);
	}
//CEI comment end//

	ret = bq_charger_get_iinlim(&info->mchr_info, &iinlim);
	ret = bq25896_get_vindpm(info, &vindpm);
	ret = bq25896_is_in_vindpm(info, &in_vindpm);
	ret = bq25896_get_charging_status(info, &chg_stat);
	ret = bq25896_is_charging_enable(info, &chg_en);
	ret = bq_charger_get_ichg(&info->mchr_info, &ichg);
	ret = bq25896_get_adc_ibat(info, &adc_ibat);
	ret = bq25896_get_adc_vbat(info, &adc_vbat);
	ret = bq25896_get_adc_vbus(info, &adc_vbus);

	battery_log(BAT_LOG_CRTI,
		"%s: ICHG = %dmA, AICR = %dmA, MIVR = %dmV, MIVR_LOOP = %d\n",
		__func__, ichg / 100, iinlim / 100, vindpm, in_vindpm);

	battery_log(BAT_LOG_CRTI,
		"%s: VBUS = %dmV, VBAT = %dmV, IBAT = %dmA\n",
		__func__, adc_vbus, adc_vbat, adc_ibat);

	battery_log(BAT_LOG_CRTI,
		"%s: CHG_EN = %d, CHG_STATUS = %s, CHG_ERR = %s\n", __func__,
		chg_en, bq25896_chg_status_name[chg_stat],
		bq25896_chg_fault_status_name[chg_fault]);

	return ret;
}


static int bq_charger_enable_charging(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);
	u8 en_hz = !enable;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	battery_log(BAT_LOG_CRTI, "%s: enable = %d\n", __func__, enable);

	if (enable) {
		ret = bq25896_set_bit(info, BQ25896_CON3, CON3_CHG_CONFIG_MASK);
		ret = bq_charger_enable_hz(mchr_info, &en_hz);
	} else {
		ret = bq25896_clr_bit(info, BQ25896_CON3, CON3_CHG_CONFIG_MASK);
		if (info->err_state)
			battery_log(BAT_LOG_CRTI,
				"%s: under test mode: disable charging\n",
				__func__);
	}


	return ret;
}

static int bq_charger_enable_hz(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);
	u32 vindpm = (enable ? BQ25896_VINDPM_MAX : 4500);

	ret = bq_charger_set_vindpm(mchr_info, &vindpm);

	return ret;
}

static int bq_charger_enable_safety_timer(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	battery_log(BAT_LOG_CRTI, "%s: enable = %d\n", __func__, enable);
	ret = (enable ? bq25896_set_bit : bq25896_clr_bit)
		(info, BQ25896_CON7, CON7_EN_TIMER_MASK);

	return ret;
}

static int bq_charger_enable_otg(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	ret = (enable ? bq25896_set_bit : bq25896_clr_bit)
		(info, BQ25896_CON3, CON3_OTG_CONFIG_MASK);

	return ret;
}

static int bq_charger_enable_power_path(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);
	u32 vindpm = (enable ? 4500 : BQ25896_VINDPM_MAX);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	battery_log(BAT_LOG_CRTI, "%s: enable = %d\n", __func__, enable);
	ret = bq25896_set_force_vindpm(info, true);
	if (ret < 0)
		return ret;

	ret = bq25896_set_vindpm(info, vindpm);
	return ret;
}


static int bq_charger_set_ichg(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u8 reg_ichg = 0;
	u32 ichg = *((u32 *)data);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	ichg /= 100;

	/* Find corresponding reg value */
	reg_ichg = bq25896_find_closest_reg_value(BQ25896_ICHG_MIN,
		BQ25896_ICHG_MAX, BQ25896_ICHG_STEP, BQ25896_ICHG_NUM, ichg);

	battery_log(BAT_LOG_CRTI, "%s: ichg = %d\n", __func__, ichg);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON4,
		reg_ichg << CON4_ICHG_SHIFT,
		CON4_ICHG_MASK
	);

	return ret;
}

static int bq_charger_set_iinlim(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 reg_iinlim = 0;
	u32 iinlim = *((u32 *)data);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	iinlim /= 100;

	/* Find corresponding reg value */
	reg_iinlim = bq25896_find_closest_reg_value(BQ25896_IINLIM_MIN,
		BQ25896_IINLIM_MAX, BQ25896_IINLIM_STEP, BQ25896_IINLIM_NUM,
		iinlim);

	battery_log(BAT_LOG_CRTI, "%s: iinlim = %d\n", __func__, iinlim);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON0,
		reg_iinlim << CON0_IINLIM_SHIFT,
		CON0_IINLIM_MASK
	);

	/*
	 * For USB_IF compliance test only when USB is in suspend(Ibus < 2.5mA)
	 * or unconfigured(Ibus < 70mA) states
	 */
#ifdef CONFIG_USBIF_COMPLIANCE
	if (iinlim < 100)
		ret = bq25896_set_vindpm(info, BQ25896_VINDPM_MAX);
	else
		ret = bq25896_set_vindpm(info, 4500);
#endif

	return ret;
}

static int bq_charger_get_iinlim(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u8 reg_iinlim = 0;
	u32 iinlim = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	ret = bq25896_i2c_read_byte(info, BQ25896_CON0);
	if (ret < 0)
		return ret;

	reg_iinlim = (ret & CON0_IINLIM_MASK) >> CON0_IINLIM_SHIFT;
	iinlim = bq25896_find_closest_real_value(BQ25896_IINLIM_MIN,
		BQ25896_IINLIM_MAX, BQ25896_IINLIM_STEP, reg_iinlim);

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	iinlim *= 100;
	*((u32 *)data) = iinlim;

	return ret;
}

static int bq_charger_set_vindpm(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u32 vindpm = *((u32 *)data);
	bool enable = true;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	/*
	 * Since BQ25896 uses vindpm to turn off power path
	 * If power path is disabled, do not adjust mivr
	 */
	ret = bq_charger_is_power_path_enable(mchr_info, &enable);
	if (ret == 0 && !enable) {
		battery_log(BAT_LOG_CRTI,
			"%s: power path is disable, skip setting vindpm = %d\n",
			__func__, vindpm);
		return 0;
	}

	ret = bq25896_set_vindpm(info, vindpm);

	return ret;
}

static int bq_charger_set_cv_voltage(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 reg_cv = 0;
	u32 cv = *((u32 *)data);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	/* MTK's voltage unit : uV */
	/* Our voltage unit : mV */
	cv /= 1000;

	/* Find corresponding reg value */
	reg_cv = bq25896_find_closest_reg_value(BQ25896_CV_MIN, BQ25896_CV_MAX,
		BQ25896_CV_STEP, BQ25896_CV_NUM, cv);

	battery_log(BAT_LOG_CRTI, "%s: cv = %d\n", __func__, cv);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON6,
		reg_cv << CON6_VREG_SHIFT,
		CON6_VREG_MASK
	);

	return ret;
}

static int bq_charger_set_boost_current_limit(
	struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u8 reg_current_limit = 0;
	u32 current_limit = *((u32 *)data);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	reg_current_limit = bq25896_find_closest_reg_value_via_table(
		bq25896_boost_current_limit,
		ARRAY_SIZE(bq25896_boost_current_limit),
		current_limit
	);

	battery_log(BAT_LOG_CRTI, "%s: current limit = %d\n", __func__,
		current_limit);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CONA,
		reg_current_limit << CONA_BOOST_ILIM_SHIFT,
		CONA_BOOST_ILIM_MASK
	);

	return ret;
}

static int bq_charger_set_pep_current_pattern(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	kal_bool pumpup = *(kal_bool *)(data);
	bool up = (pumpup ? true : false);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	battery_log(BAT_LOG_CRTI, "%s: pump up = %d\n", __func__, up);
	ret = bq25896_pumpx_up(info, up);

	return ret;
}

static int bq_charger_set_pep20_reset(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u32 vindpm = 4500;	/* mA */
	u32 ichg = 51200;	/* 10uA */
	u32 iinlim = 10000;	/* 10uA */
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	ret = bq_charger_set_vindpm(mchr_info, &vindpm);
	ret = bq_charger_set_ichg(mchr_info, &ichg);

	ret = bq25896_enable_ico(info, false);
	ret = bq_charger_set_iinlim(mchr_info, &iinlim);

	msleep(250);

	iinlim = 70000; /* 10uA */
	ret = bq_charger_set_iinlim(mchr_info, &iinlim);
	ret = bq25896_enable_ico(info, true);

	return ret;
}

struct timespec ptime[13];
static int cptime[13][2];

static int dtime(int i)
{
	struct timespec time;

	time = timespec_sub(ptime[i], ptime[i-1]);
	return time.tv_nsec/1000000;
}

#define PEOFFTIME 40
#define PEONTIME 90

static int bq_charger_set_pep20_current_pattern(
	struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	int i, j = 0;
	int value;
	int flag;
	CHR_VOLTAGE_ENUM chr_vol = *(CHR_VOLTAGE_ENUM *)data;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;
	u32 vindpm = 4500;	/* mA */
	u32 ichg = 51200;	/* 10uA */
	u32 iinlim = 10000;	/* 10uA */

	ret = bq_charger_set_vindpm(mchr_info, &vindpm);
	ret = bq_charger_set_ichg(mchr_info, &ichg);
	ret = bq25896_enable_ico(info, false);

	usleep_range(1000, 1200);
	value = (chr_vol - CHR_VOLT_05_500000_V) / CHR_VOLT_00_500000_V;

	ret = bq_charger_set_iinlim(mchr_info, &iinlim);
	msleep(70);

	get_monotonic_boottime(&ptime[j++]);
	for (i = 4; i >= 0; i--) {
		flag = value & (1 << i);

		if (flag == 0) {
			iinlim = 70000;
			ret = bq_charger_set_iinlim(mchr_info, &iinlim);
			msleep(PEOFFTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
				battery_log(BAT_LOG_CRTI,
					"charging_set_ta20_current_pattern fail1: idx:%d target:%d actual:%d\n",
					i, PEOFFTIME, cptime[j][1]);
				return -EIO;
			}
			j++;
			iinlim = 10000;
			ret = bq_charger_set_iinlim(mchr_info, &iinlim);
			msleep(PEONTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
				battery_log(BAT_LOG_CRTI,
					"charging_set_ta20_current_pattern fail2: idx:%d target:%d actual:%d\n",
					i, PEOFFTIME, cptime[j][1]);
				return -EIO;
			}
			j++;

		} else {
			iinlim = 70000;
			ret = bq_charger_set_iinlim(mchr_info, &iinlim);
			msleep(PEONTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
				battery_log(BAT_LOG_CRTI,
					"charging_set_ta20_current_pattern fail3: idx:%d target:%d actual:%d\n",
					i, PEOFFTIME, cptime[j][1]);
				return -EIO;
			}
			j++;
			iinlim = 10000;
			ret = bq_charger_set_iinlim(mchr_info, &iinlim);
			msleep(PEOFFTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
				battery_log(BAT_LOG_CRTI,
					"charging_set_ta20_current_pattern fail4: idx:%d target:%d actual:%d\n",
					i, PEOFFTIME, cptime[j][1]);
				return -EIO;
			}
			j++;
		}
	}

	iinlim = 70000;
	ret = bq_charger_set_iinlim(mchr_info, &iinlim);
	msleep(160);
	get_monotonic_boottime(&ptime[j]);
	cptime[j][0] = 160;
	cptime[j][1] = dtime(j);
	if (cptime[j][1] < 150 || cptime[j][1] > 240) {
		battery_log(BAT_LOG_CRTI,
			"charging_set_ta20_current_pattern fail5: idx:%d target:%d actual:%d\n",
			i, PEOFFTIME, cptime[j][1]);
		return -EIO;
	}
	j++;

	iinlim = 10000;
	ret = bq_charger_set_iinlim(mchr_info, &iinlim);
	msleep(30);
	iinlim = 70000;
	ret = bq_charger_set_iinlim(mchr_info, &iinlim);

	battery_log(BAT_LOG_CRTI,
	"[charging_set_ta20_current_pattern]:chr_vol:%d bit:%d time:%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
	chr_vol, value,
	cptime[1][0], cptime[2][0], cptime[3][0], cptime[4][0], cptime[5][0],
	cptime[6][0], cptime[7][0], cptime[8][0], cptime[9][0], cptime[10][0], cptime[11][0]);

	battery_log(BAT_LOG_CRTI,
	"[charging_set_ta20_current_pattern2]:chr_vol:%d bit:%d time:%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
	chr_vol, value,
	cptime[1][1], cptime[2][1], cptime[3][1], cptime[4][1], cptime[5][1],
	cptime[6][1], cptime[7][1], cptime[8][1], cptime[9][1], cptime[10][1], cptime[11][1]);


	ret = bq25896_enable_ico(info, true);
	iinlim = 325000;
	ret = bq_charger_set_iinlim(mchr_info, &iinlim);

	return 0;
}

static int bq_charger_set_ircmp_resistor(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 reg_resistor = 0;
	u32 resistor = *((u32 *)data);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	/* Find corresponding reg value */
	reg_resistor = bq25896_find_closest_reg_value(BQ25896_BAT_COMP_MIN,
		BQ25896_BAT_COMP_MAX, BQ25896_BAT_COMP_STEP,
		BQ25896_BAT_COMP_NUM, resistor);


	battery_log(BAT_LOG_CRTI, "%s: resistor = %d\n", __func__,
		resistor);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON8,
		reg_resistor << CON8_BAT_COMP_SHIFT,
		CON8_BAT_COMP_MASK
	);

	return 0;
}

static int bq_charger_set_ircmp_vclamp(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 reg_vclamp = 0;
	u32 vclamp = *((u32 *)data);
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	/* Find corresponding reg value */
	reg_vclamp = bq25896_find_closest_reg_value(BQ25896_VCLAMP_MIN,
		BQ25896_VCLAMP_MAX, BQ25896_VCLAMP_STEP,
		BQ25896_VCLAMP_NUM, vclamp);

	battery_log(BAT_LOG_CRTI, "%s: vclamp = %d\n", __func__, vclamp);

	ret = bq25896_i2c_update_bits(
		info,
		BQ25896_CON8,
		reg_vclamp << CON8_VCLAMP_SHIFT,
		CON8_VCLAMP_MASK
	);

	return 0;
}

static int bq_charger_set_error_state(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u8 error = *(u8 *)data;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;


	info->err_state = error;
	ret = bq_charger_enable_hz(mchr_info, &error);

	return ret;
}

static int bq_charger_get_ichg(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u8 reg_ichg = 0;
	u32 ichg = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	ret = bq25896_i2c_read_byte(info, BQ25896_CON4);
	if (ret < 0)
		return ret;

	reg_ichg = (ret & CON4_ICHG_MASK) >> CON4_ICHG_SHIFT;
	ichg = bq25896_find_closest_real_value(BQ25896_ICHG_MIN,
		BQ25896_ICHG_MAX, BQ25896_ICHG_STEP, reg_ichg);

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	ichg *= 100;
	*((u32 *)data) = ichg;

	return ret;
}

static int bq_charger_is_charging_done(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;
	enum bq25896_charging_status chg_stat = BQ25896_CHG_STATUS_NOT_CHARGING;

	ret = bq25896_get_charging_status(info, &chg_stat);
	switch (chg_stat) {
	case BQ25896_CHG_STATUS_NOT_CHARGING:
	case BQ25896_CHG_STATUS_PRECHARGE:
	case BQ25896_CHG_STATUS_FAST_CHARGE:
		*((u32 *)data) = 0;
		break;
	case BQ25896_CHG_STATUS_DONE:
		*((u32 *)data) = 1;
		break;
	default:
		*((u32 *)data) = 0;
		break;
	}

	return ret;
}

static int bq_charger_is_power_path_enable(struct mtk_charger_info *mchr_info,
	void *data)
{
	int ret = 0;
	u32 vindpm = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	ret = bq25896_get_vindpm(info, &vindpm);
	*((bool *)data) = (vindpm == BQ25896_VINDPM_MAX) ? false : true;

	return ret;
}

static int bq_charger_is_safety_timer_enable(
	struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	ret = bq25896_i2c_test_bit(info, BQ25896_CON7, CON7_EN_TIMER_SHIFT);
	if (ret < 0)
		return ret;

	*((bool *)data) = (ret > 0) ? true : false;

	return ret;
}

static int bq_charger_reset_watchdog_timer(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	/* reset watchdog timer */
	ret = bq25896_set_bit(info, BQ25896_CON3, CON3_WD_MASK);

	return ret;
}

//CEI comments start, identify which charegr IC is using
static int bq_who_am_i(struct mtk_charger_info *mchr_info, void *data)
{
	return bq25896_whoami;
}
//CEI comments end, identify which charegr IC is using

//CEI comment start
//Solve IEOC reset by watchdog issue
static int bq_set_ieoc(struct mtk_charger_info *mchr_info, void *data)
{
	int ret = 0;
	u32 ieoc = 0;
	struct bq25896_info *info = (struct bq25896_info *)mchr_info;

	ieoc = *((u32 *)data);

	ret = bq25896_set_iterm(info, ieoc);

	return ret;
}
//CEI comment end

const static mtk_charger_intf bq25896_mchr_intf[CHARGING_CMD_NUMBER] = {
//CEI comments start, identify which charegr IC is using
	[CHARGING_CMD_WHO_AM_I] = bq_who_am_i,
//CEI comments end, identify which charegr IC is using
//CEI comment start, Solve IEOC reset by watchdog issue
	[CHARGING_CMD_SET_IEOC] = bq_set_ieoc,
//CEI comment end, Solve IEOC reset by watchdog issue
	[CHARGING_CMD_INIT] = bq_charger_hw_init,
	[CHARGING_CMD_SW_INIT] = bq_charger_sw_init,
	[CHARGING_CMD_DUMP_REGISTER] = bq_charger_dump_register,
	[CHARGING_CMD_ENABLE] = bq_charger_enable_charging,
	[CHARGING_CMD_SET_HIZ_SWCHR] = bq_charger_enable_hz,
	[CHARGING_CMD_ENABLE_SAFETY_TIMER] = bq_charger_enable_safety_timer,
	[CHARGING_CMD_ENABLE_OTG] = bq_charger_enable_otg,
	[CHARGING_CMD_ENABLE_POWER_PATH] = bq_charger_enable_power_path,
	[CHARGING_CMD_SET_CURRENT] = bq_charger_set_ichg,
	[CHARGING_CMD_SET_INPUT_CURRENT] = bq_charger_set_iinlim,
	[CHARGING_CMD_SET_VINDPM] = bq_charger_set_vindpm,
	[CHARGING_CMD_SET_CV_VOLTAGE] = bq_charger_set_cv_voltage,
	[CHARGING_CMD_SET_BOOST_CURRENT_LIMIT] = bq_charger_set_boost_current_limit,
	[CHARGING_CMD_SET_TA_CURRENT_PATTERN] = bq_charger_set_pep_current_pattern,
	[CHARGING_CMD_SET_TA20_RESET] = bq_charger_set_pep20_reset,
	[CHARGING_CMD_SET_TA20_CURRENT_PATTERN] = bq_charger_set_pep20_current_pattern,
	[CHARGING_CMD_SET_IRCMP_RESISTOR] = bq_charger_set_ircmp_resistor,
	[CHARGING_CMD_SET_IRCMP_VOLT_CLAMP] = bq_charger_set_ircmp_vclamp,
	[CHARGING_CMD_SET_ERROR_STATE] = bq_charger_set_error_state,
	[CHARGING_CMD_GET_CURRENT] = bq_charger_get_ichg,
	[CHARGING_CMD_GET_INPUT_CURRENT] = bq_charger_get_iinlim,
	[CHARGING_CMD_GET_CHARGING_STATUS] = bq_charger_is_charging_done,
	[CHARGING_CMD_GET_IS_POWER_PATH_ENABLE] = bq_charger_is_power_path_enable,
	[CHARGING_CMD_GET_IS_SAFETY_TIMER_ENABLE] = bq_charger_is_safety_timer_enable,
	[CHARGING_CMD_RESET_WATCH_DOG_TIMER] = bq_charger_reset_watchdog_timer,

	/*
	 * The following interfaces are not related to charger
	 * Define in bq_charger_intf.c
	 */
	[CHARGING_CMD_SET_HV_THRESHOLD] = mtk_charger_set_hv_threshold,
	[CHARGING_CMD_GET_HV_STATUS] = mtk_charger_get_hv_status,
	[CHARGING_CMD_GET_BATTERY_STATUS] = mtk_charger_get_battery_status,
	[CHARGING_CMD_GET_CHARGER_DET_STATUS] = mtk_charger_get_charger_det_status,
	[CHARGING_CMD_GET_CHARGER_TYPE] = mtk_charger_get_charger_type,
	[CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER] = mtk_charger_get_is_pcm_timer_trigger,
	[CHARGING_CMD_SET_PLATFORM_RESET] = mtk_charger_set_platform_reset,
	[CHARGING_CMD_GET_PLATFORM_BOOT_MODE] = mtk_charger_get_platform_boot_mode,
	[CHARGING_CMD_SET_POWER_OFF] = mtk_charger_set_power_off,
	[CHARGING_CMD_GET_POWER_SOURCE] = mtk_charger_get_power_source,
	[CHARGING_CMD_GET_CSDAC_FALL_FLAG] = mtk_charger_get_csdac_full_flag,
	[CHARGING_CMD_DISO_INIT] = mtk_charger_diso_init,
	[CHARGING_CMD_GET_DISO_STATE] = mtk_charger_get_diso_state,
	[CHARGING_CMD_SET_VBUS_OVP_EN] = mtk_charger_set_vbus_ovp_en,
	[CHARGING_CMD_GET_BIF_VBAT] = mtk_charger_get_bif_vbat,
	[CHARGING_CMD_SET_CHRIND_CK_PDN] = mtk_charger_set_chrind_ck_pdn,
	[CHARGING_CMD_GET_BIF_TBAT] = mtk_charger_get_bif_tbat,
	[CHARGING_CMD_SET_DP] = mtk_charger_set_dp,
	[CHARGING_CMD_GET_BIF_IS_EXIST] = mtk_charger_get_bif_is_exist,
};

static int bq25896_probe(struct i2c_client *i2c,
	const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct bq25896_info *info = NULL;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);

	info = devm_kzalloc(&i2c->dev, sizeof(struct bq25896_info), GFP_KERNEL);
	if (!info) {
		battery_log(BAT_LOG_CRTI, "%s: no enough memory\n", __func__);
		return -ENOMEM;
	}
	info->i2c = i2c;
	info->i2c_log_level = BAT_LOG_FULL;
	mutex_init(&info->i2c_access_lock);

	if (!bq25896_is_hw_exist(info)) {
		battery_log(BAT_LOG_CRTI, "%s: no bq25896 exist\n", __func__);
		ret = -ENODEV;
		goto err_nodev;
	}
	i2c_set_clientdata(i2c, info);

#ifdef CONFIG_RT_REGMAP
	ret = bq25896_register_rt_regmap(info);
	if (ret < 0)
		goto err_register_regmap;
#endif

	bq_charger_dump_register(&info->mchr_info, NULL);

	/* Hook chr_control_interface with battery's interface */
	info->mchr_info.mchr_intf = bq25896_mchr_intf;
	mtk_charger_set_info(&info->mchr_info);
	battery_charging_control = chr_control_interface;
//CEI comments start, identify which charegr IC is using
	bq25896_whoami = 2;
//CEI comments start, identify which charegr IC is using
	chargin_hw_init_done = true;
	return ret;

err_nodev:
err_register_regmap:
	mutex_destroy(&info->i2c_access_lock);

	return ret;
}

static int bq25896_remove(struct i2c_client *i2c)
{
	int ret = 0;
	struct bq25896_info *info = i2c_get_clientdata(i2c);

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);

	if (info) {
#ifdef CONFIG_RT_REGMAP
		rt_regmap_device_unregister(info->regmap_dev);
#endif
		mutex_destroy(&info->i2c_access_lock);
	}

	return ret;
}


static const struct i2c_device_id bq25896_i2c_id[] = {
	{"bq25896", 0},
	{},
};

#ifdef CONFIG_OF
static const struct of_device_id bq25896_of_match[] = {
	{.compatible = "mediatek,swithing_charger"},
	{},
};
#else

#define BQ25896_BUSNUM 1

static struct i2c_board_info i2c_bq25896 __initdata = {
	I2C_BOARD_INFO("bq25896", BQ25896_SLAVE_ADDR)
};
#endif

static struct i2c_driver bq25896_driver = {
	.driver = {
		   .name = "bq25896",
#ifdef CONFIG_OF
		   .of_match_table = bq25896_of_match,
#endif
		   },
	.probe = bq25896_probe,
	.remove = bq25896_remove,
	.id_table = bq25896_i2c_id,
};

static int __init bq25896_init(void)
{
	int ret = 0;

	/* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	battery_log(BAT_LOG_CRTI, "%s: with dts\n", __func__);
#else
	battery_log(BAT_LOG_CRTI, "%s: without dts\n", __func__);
	i2c_register_board_info(BQ25896_BUSNUM, &bq25896_i2c_board_info, 1);
#endif

	ret = i2c_add_driver(&bq25896_driver);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: register i2c driver failed\n",
			__func__);

	return ret;
}

static void __exit bq25896_exit(void)
{
	i2c_del_driver(&bq25896_driver);
}
module_init(bq25896_init);
module_exit(bq25896_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_DESCRIPTION("BQ25896 Charger Driver");
MODULE_VERSION("1.0.0_MTK");
