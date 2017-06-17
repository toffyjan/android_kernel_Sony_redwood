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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *    switch_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
 * Revision:   1.0
 * Modtime:   11 Aug 2005 10:28:16
 * Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc
 *
 * 03 05 2015 wy.chuang
 * [ALPS01921641] [L1_merge] for PMIC and charging
 * .
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/types.h>
#include <linux/kernel.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mach/mtk_battery_meter.h>//CEI comments, soft charger 3.0
#include <mt-plat/charging.h>
#include <mach/mtk_charging.h>
#include <mt-plat/mtk_boot.h>
#include "mtk_pep_intf.h"
#include "mtk_pep20_intf.h"

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mt-plat/diso.h>
#include <mach/mt_diso.h>
#endif


/* ============================================================ // */
/* define */
/* ============================================================ // */
/* cut off to full */
#define POST_CHARGING_TIME (30*60)	/* 30mins */
//CEI comment start//
//BAT_status_Full feature
#define FULL_CHECK_TIMES 3 //MTK ORG=6, since bat_thread time is 20s, we keep the design to wait 60s to check full
//CEI comment end//

#define STATUS_OK    0
#define STATUS_UNSUPPORTED    -1
#define STATUS_FAIL -2


/* ============================================================ // */
/* global variable */
/* ============================================================ // */
unsigned int g_bcct_flag;
unsigned int g_bcct_value;
/*input-output curent distinction*/
unsigned int g_bcct_input_flag;
unsigned int g_bcct_input_value;
unsigned int g_full_check_count;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
unsigned int g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited;
//CEI comment start, Alien battery support
extern int battery_id_invalid;
extern int g_fg_battery_id;
extern int id_voltage_read;
extern int g_battery_id_voltage[];
//CEI comment end, Alien battery support
#if (CONFIG_MTK_GAUGE_VERSION == 20)
BATTERY_VOLTAGE_ENUM g_cv_voltage = MTK_CV_VOLTAGE;
unsigned int get_cv_voltage(void)
{
	return g_cv_voltage;
}

//CEI comments, soft charger 3.0 start
#ifdef CUSTOM_SOFT_CHARGE_30
kal_bool g_sc30_low_cv_voltage = KAL_FALSE;
void set_sc30_low_cv_voltage(kal_bool low_cv)
{
	g_sc30_low_cv_voltage = low_cv;
}
#endif

//CEI comments, soft charger 3.0 end
#endif

DEFINE_MUTEX(g_ichg_aicr_access_mutex);
DEFINE_MUTEX(g_aicr_access_mutex);
DEFINE_MUTEX(g_ichg_access_mutex);
unsigned int g_aicr_upper_bound;
static bool g_enable_dynamic_cv = true;

 /* ///////////////////////////////////////////////////////////////////////////////////////// */
 /* // JEITA */
 /* ///////////////////////////////////////////////////////////////////////////////////////// */
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
int g_temp_status = TEMP_POS_10_TO_POS_45;
kal_bool temp_error_recovery_chr_flag = KAL_TRUE;
#endif

/* ============================================================ // */
/* function prototype */
/* ============================================================ // */
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\n");
#else
	if ((usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))) {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] BAT_SetUSBState Fail! Restore to default value\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_log(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\n",
			    usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}


unsigned int get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

int mtk_get_dynamic_cv(unsigned int *cv)
{
	int ret = 0;
#ifdef CONFIG_MTK_BIF_SUPPORT
	u32 _cv;
	u32 vbat_bif = 0, vbat_auxadc = 0, vbat = 0;
	u32 retry_cnt = 0;
	u32 ircmp_volt_clamp = 0, ircmp_resistor = 0;

	if (!g_enable_dynamic_cv) {
		_cv = MTK_CV_VOLTAGE / 1000;
		goto _out;
	}

	do {
		ret = battery_charging_control(CHARGING_CMD_GET_BIF_VBAT,
			&vbat_bif);
		vbat_auxadc = battery_meter_get_battery_voltage(KAL_TRUE);

		if (ret >= 0 && vbat_bif != 0 && vbat_bif < vbat_auxadc) {
			vbat = vbat_bif;
			battery_log(BAT_LOG_CRTI,
				"%s: use BIF vbat = %dmV, dV to auxadc = %dmV\n",
				__func__, vbat, vbat_auxadc - vbat_bif);
			break;
		}

		retry_cnt++;
	} while (retry_cnt < 5);

	if (retry_cnt == 5) {
		ret = 0;
		vbat = vbat_auxadc;
		battery_log(BAT_LOG_CRTI,
			"%s: use AUXADC vbat = %dmV, since BIF vbat = %d\n",
			__func__, vbat_auxadc, vbat_bif);
	}

	/* Adjust CV according to the obtained vbat */
	if (vbat >= 3400 && vbat < MTK_DYNAMIC_CV_THRESHOLD) {
		_cv = MTK_DYNAMIC_CV_VOLTAGE;
		battery_charging_control(CHARGING_CMD_SET_IRCMP_VOLT_CLAMP,
			&ircmp_volt_clamp);
		battery_charging_control(CHARGING_CMD_SET_IRCMP_RESISTOR,
			&ircmp_resistor);
	} else {
		_cv = MTK_CV_VOLTAGE / 1000;

		/* Turn on IR compensation */
		ircmp_volt_clamp = MTK_IRCMP_VOLT_CLAMP;
		ircmp_resistor = MTK_IRCMP_RESISTOR;
		battery_charging_control(CHARGING_CMD_SET_IRCMP_VOLT_CLAMP,
			&ircmp_volt_clamp);
		battery_charging_control(CHARGING_CMD_SET_IRCMP_RESISTOR,
			&ircmp_resistor);

		/* Disable dynamic CV */
		g_enable_dynamic_cv = false;
	}

_out:
	*cv = _cv;
	battery_log(BAT_LOG_CRTI, "%s: CV = %dmV, enable dynamic cv = %d\n",
		__func__, _cv, g_enable_dynamic_cv);
#else
	ret = -ENOTSUPP;
#endif
	return ret;
}

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)

static BATTERY_VOLTAGE_ENUM select_jeita_cv(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;

	if (g_temp_status == TEMP_ABOVE_POS_60) {
		cv_voltage = JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_45_TO_POS_60) {
		cv_voltage = JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_10_TO_POS_45) {
		cv_voltage = MTK_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_0_TO_POS_10) {
		cv_voltage = JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		cv_voltage = JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_BELOW_NEG_10) {
		cv_voltage = JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE;
	} else {
		cv_voltage = BATTERY_VOLT_04_200000_V;
	}

//CEI comments, JEITA parameters S
 	battery_log(BAT_LOG_CRTI, "[BATTERY]CEI(K)=> select_jeita_cv: %d\n", cv_voltage);
//CEI comments, JEITA parameters E

	return cv_voltage;
}

PMU_STATUS do_jeita_state_machine(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;
	PMU_STATUS jeita_status = PMU_STATUS_OK;

	/* JEITA battery temp Standard */

	if (BMT_status.temperature >= TEMP_POS_60_THRESHOLD) {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] Battery Over high Temperature(%d) !!\n",
			    TEMP_POS_60_THRESHOLD);

		g_temp_status = TEMP_ABOVE_POS_60;

		return PMU_STATUS_FAIL;
	} else if (BMT_status.temperature > TEMP_POS_45_THRESHOLD) {	/* control 45c to normal behavior */
		if ((g_temp_status == TEMP_ABOVE_POS_60)
		    && (BMT_status.temperature >= TEMP_POS_60_THRES_MINUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n",
				    TEMP_POS_60_THRES_MINUS_X_DEGREE, TEMP_POS_60_THRESHOLD);

			jeita_status = PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n",
				    TEMP_POS_45_THRESHOLD, TEMP_POS_60_THRESHOLD);

			g_temp_status = TEMP_POS_45_TO_POS_60;
		}
	} else if (BMT_status.temperature >= TEMP_POS_10_THRESHOLD) {
		if (((g_temp_status == TEMP_POS_45_TO_POS_60)
		     && (BMT_status.temperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE))
		    || ((g_temp_status == TEMP_POS_0_TO_POS_10)
			&& (BMT_status.temperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE))) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature not recovery to normal temperature charging mode yet!!\n");
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Normal Temperature between %d and %d !!\n",
				    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD);
			g_temp_status = TEMP_POS_10_TO_POS_45;
		}
	} else if (BMT_status.temperature >= TEMP_POS_0_THRESHOLD) {
		if ((g_temp_status == TEMP_NEG_10_TO_POS_0 || g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_POS_0_THRES_PLUS_X_DEGREE)) {
			if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n",
					    TEMP_POS_0_THRES_PLUS_X_DEGREE, TEMP_POS_10_THRESHOLD);
			}
			if (g_temp_status == TEMP_BELOW_NEG_10) {
				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n",
					    TEMP_POS_0_THRESHOLD, TEMP_POS_0_THRES_PLUS_X_DEGREE);
				return PMU_STATUS_FAIL;
			}
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n",
				    TEMP_POS_0_THRESHOLD, TEMP_POS_10_THRESHOLD);

			g_temp_status = TEMP_POS_0_TO_POS_10;
		}
	} else if (BMT_status.temperature >= TEMP_NEG_10_THRESHOLD) {
		if ((g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_NEG_10_THRES_PLUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n",
				    TEMP_NEG_10_THRESHOLD, TEMP_NEG_10_THRES_PLUS_X_DEGREE);

			jeita_status = PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n",
				    TEMP_NEG_10_THRESHOLD, TEMP_POS_0_THRESHOLD);

			g_temp_status = TEMP_NEG_10_TO_POS_0;
		}
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] Battery below low Temperature(%d) !!\n",
			    TEMP_NEG_10_THRESHOLD);
		g_temp_status = TEMP_BELOW_NEG_10;

		jeita_status = PMU_STATUS_FAIL;
	}

	/* set CV after temperature changed */
	/* In normal range, we adjust CV dynamically */
	if (g_temp_status != TEMP_POS_10_TO_POS_45) {
		cv_voltage = select_jeita_cv();
		//CEI comment start, Alien battery support
		if (battery_id_invalid) {
			cv_voltage = BATTERY_VOLT_04_000000_V;	
			battery_log(BAT_LOG_CRTI, "[Alien]do_jeita_state_machine(): set cv_voltage=%d\n", cv_voltage);
		}
		//CEI comment end, Alien battery support
		battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE,
			&cv_voltage);
#if (CONFIG_MTK_GAUGE_VERSION == 20)
		g_cv_voltage = cv_voltage;
#endif
	}
	
	//CEI comment start
	battery_log(BAT_LOG_CRTI,"[Battery][JEITA] BMT_status.temperature=%d, g_temp_status=%d, jeita_status=%s\n",
		BMT_status.temperature, g_temp_status, (jeita_status==PMU_STATUS_OK)? "OK":"NOK");
	
	battery_log(BAT_LOG_CRTI,
	    "[BATTERY][JEITA] g_cv_voltage=%d, g_fg_battery_id=%d, battery_id_invalid=%d, id_voltage_read=%d, g_battery_id_voltage=%d %d %d\n", 
	    g_cv_voltage, g_fg_battery_id, battery_id_invalid, id_voltage_read, g_battery_id_voltage[0], g_battery_id_voltage[1], g_battery_id_voltage[2]);
	//CEI comment end
	
	return jeita_status;
}


static void set_jeita_charging_current(void)
{
//CEI comments, JEITA parameters S
	int c_stop_charging = 0;
 	int c_decrease_current = 0;
//CEI comments, JEITA parameters E	
	
#ifdef CONFIG_USB_IF
	if (BMT_status.charger_type == STANDARD_HOST)
		return;
#endif

//CEI comments, JEITA parameters S
#if 0
	if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		g_temp_CC_value = CHARGE_CURRENT_350_00_MA;
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d\n",
			    g_temp_CC_value);
	}
#else
 	c_stop_charging = (g_temp_status == TEMP_BELOW_NEG_10) || (g_temp_status == TEMP_NEG_10_TO_POS_0) || (g_temp_status == TEMP_ABOVE_POS_60);
 	c_decrease_current = (g_temp_status == TEMP_POS_0_TO_POS_10) || (g_temp_status == TEMP_POS_45_TO_POS_60);
 
 	if (c_stop_charging) {
 		g_temp_CC_value = CHARGE_CURRENT_0_00_MA;/* for highest and lowest temp */
 		battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d, status: %d, for highest and lowest temp.n",
 			    g_temp_CC_value, g_temp_status);
 	} else if (c_decrease_current) {
 		if (BMT_status.charger_type != STANDARD_HOST && BMT_status.charger_type != APPLE_0_5A_CHARGER){
 			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;	/* for higher and lower temp */
 			battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d, status: %d, for higher and lower temp.\n",
 			    g_temp_CC_value, g_temp_status);
 		} else {
			battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : charger_type %d\n", BMT_status.charger_type);
 		}
 	} else {
 		battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d, status: %d, NORMAL\n",
			g_temp_CC_value, g_temp_status);
 	}
#endif
//CEI comments, JEITA parameters E
}

#endif /* CONFIG_MTK_JEITA_STANDARD_SUPPORT */

bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;

		return false;
}

void set_usb_current_unlimited(bool enable)
{
	usb_unlimited = enable;
}

//CEI comments, iusb limit for bcct S
struct chrlimt_table {
	int vchr_thres[4];
	int chrlimt_current[5][3];
};

static struct chrlimt_table chrlimit =
{
	{2500, 5500, 7500, 9500},

	{
		{1001, 501, 201}, //vchr is less than 2.5V 
		{1000, 500, 200}, //vchr is 5V 
		{867, 467, 351}, //vchr is 7V 
		{733, 434, 352}, //vchr is 9V 
		{600, 400, 353}, //vchr is 12V 
	},
};

struct bcct_log_table{
	int bcct_value[17];
	unsigned int bcct_count[17];
};

struct bcct_log_table bcct_log =
{
	{1001, 1000, 867, 733, 600, 501, 500, 467, 434, 400, 353, 352, 351, 201, 200, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

int bcct_level = 0;

int get_chrlimt_bcct_level(int chr_current)
{
	int ret = 0;

	switch (chr_current){
		case 200:
			ret=3;
			break;
		case 500:
			ret=2;
			break;
		case 1000:
			ret=1;
			break;
	}

	return ret;
}

void set_bcct_log(int bcct_now)
{
	int index = 0;

	for(index=0; index<(sizeof(bcct_log.bcct_value)/sizeof(int)); index++){
		if(bcct_now == bcct_log.bcct_value[index]){
			if(bcct_log.bcct_count[index] < 94608000)
				bcct_log.bcct_count[index]++;
			break;
		}
	}
}

int bcct_ever = 0;
//CEI comments, iusb limit for bcct E

unsigned int set_chr_input_current_limit(int current_limit)
{
#ifdef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	u32 power_path_enable = 1;
//CEI comments, iusb limit for bcct S
	//CHR_CURRENT_ENUM chr_type_aicr = 0; /* 10uA */
	//CHR_CURRENT_ENUM chr_type_ichg = 0;

	if(bypass_bcct){
		//return g_bcct_input_flag = 0 to disable bcct
		battery_log(BAT_LOG_CRTI, "[BATTERY][IUSB] BYPASS set_chr_input_current_limit, bypass_bcct = %d\r\n", bypass_bcct);
		return 0;
	}else{
		battery_log(BAT_LOG_CRTI, "[BATTERY][IUSB] RUN set_chr_input_current_limit, bypass_bcct = %d\r\n", bypass_bcct);
	}
//CEI comments, iusb limit for bcct E

	mutex_lock(&g_aicr_access_mutex);
	if (current_limit != -1) {
		g_bcct_input_flag = 1;

//CEI comments, iusb limit for bcct S
		bcct_level = get_chrlimt_bcct_level(current_limit);

		if(bcct_ever < 47304000)
			bcct_ever++;
#if 0

		if (current_limit < 100) {
			/* limit < 100, turn off power path */
			current_limit = 0;
			power_path_enable = 0;
			battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
				&power_path_enable);
		} else {
			/* Enable power path if it is disabled previously */
			if (g_bcct_input_value == 0) {
				power_path_enable = 1;
				battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
					&power_path_enable);
			}

			switch (BMT_status.charger_type) {
			case STANDARD_HOST:
				chr_type_aicr = batt_cust_data.usb_charger_current;
				break;
			case NONSTANDARD_CHARGER:
				chr_type_aicr = batt_cust_data.non_std_ac_charger_current;
				break;
			case STANDARD_CHARGER:
				chr_type_aicr = batt_cust_data.ac_charger_input_current;
				mtk_pep20_set_charging_current(&chr_type_ichg, &chr_type_aicr);
				mtk_pep_set_charging_current(&chr_type_ichg, &chr_type_aicr);
				break;
			case CHARGING_HOST:
				chr_type_aicr = batt_cust_data.charging_host_charger_current;
				break;
			case APPLE_2_1A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_2_1a_charger_current;
				break;
			case APPLE_1_0A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_1_0a_charger_current;
				break;
			case APPLE_0_5A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_0_5a_charger_current;
				break;
			default:
				chr_type_aicr = CHARGE_CURRENT_500_00_MA;
				break;
			}
			chr_type_aicr /= 100;
			if (current_limit > chr_type_aicr)
				current_limit = chr_type_aicr;
		}

		g_bcct_input_value = current_limit;
#endif
//CEI comments, iusb limit for bcct E

	} else {
		/* Enable power path if it is disabled previously */
		if (g_bcct_input_value == 0) {
			power_path_enable = 1;
			battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
				&power_path_enable);
		}

		/* Change to default current setting */
		g_bcct_input_flag = 0;

//CEI comments, iusb limit for bcct S
		bcct_level = 0;
//CEI comments, iusb limit for bcct E
	}

	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_chr_input_current_limit (%d)\n", current_limit);

//CEI comments, iusb limit for bcct S
	battery_log(BAT_LOG_CRTI, "[BATTERY][IUSB] CEI(K)=> set_chr_input_current_limit, g_bcct_input_value=%d, g_bcct_input_flag=%d, bcct_level=%d, bcct_ever=%d\r\n", g_bcct_input_value, g_bcct_input_flag, bcct_level, bcct_ever);
//CEI comments, iusb limit for bcct E

	mutex_unlock(&g_aicr_access_mutex);
	return g_bcct_input_flag;
#else
	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_chr_input_current_limit _NOT_ supported\n");
	return 0;
#endif
}

//CEI comments, iusb limit for bcct S
int adaptive_chrlimit_current(int vchr, int bcct_level_value)
{
	int index = 0;
	int ret = 0;	

	for(index=0; index<(sizeof(chrlimit.vchr_thres)/sizeof(int)); index++){	
		if(vchr < chrlimit.vchr_thres[index])
			break;
	}

	if ((bcct_level_value > 0) && (bcct_level_value <= 3))
		ret = chrlimit.chrlimt_current[index][bcct_level_value-1];

	battery_log(BAT_LOG_CRTI, "[BATTERY][adaptive_chrlimit_current][IUSB] index %d, ret %d, vchr %d, bcct_level_value %d\n", index, ret, vchr, bcct_level_value);

	return ret;
}
//CEI comments, iusb limit for bcct E

static void mtk_select_ichg_aicr(void);
unsigned int set_bat_charging_current_limit(int current_limit)
{
	CHR_CURRENT_ENUM chr_type_ichg = 0;
	CHR_CURRENT_ENUM chr_type_aicr = 0;

//CEI comments, bypass bcct S
	if(bypass_bcct){
		battery_log(BAT_LOG_CRTI, "[BATTERY] BYPASS set_chr_input_current_limit, bypass_bcct = %d\r\n", bypass_bcct);
		return 0;
	}else{
		battery_log(BAT_LOG_CRTI, "[BATTERY] RUN set_chr_input_current_limit, bypass_bcct = %d\r\n", bypass_bcct);
	}
//CEI comments, bypass bcct E

	mutex_lock(&g_ichg_access_mutex);
	if (current_limit != -1) {
		g_bcct_flag = 1;
		switch (BMT_status.charger_type) {
		case STANDARD_HOST:
			chr_type_ichg = batt_cust_data.usb_charger_current;
			break;
		case NONSTANDARD_CHARGER:
			chr_type_ichg = batt_cust_data.non_std_ac_charger_current;
			break;
		case STANDARD_CHARGER:
			chr_type_ichg = batt_cust_data.ac_charger_current;
			mtk_pep20_set_charging_current(&chr_type_ichg, &chr_type_aicr);
			mtk_pep_set_charging_current(&chr_type_ichg, &chr_type_aicr);
			break;
		case CHARGING_HOST:
			chr_type_ichg = batt_cust_data.charging_host_charger_current;
			break;
		case APPLE_2_1A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_2_1a_charger_current;
			break;
		case APPLE_1_0A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_1_0a_charger_current;
			break;
		case APPLE_0_5A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_0_5a_charger_current;
			break;
		default:
			chr_type_ichg = CHARGE_CURRENT_500_00_MA;
			break;
		}
		chr_type_ichg /= 100;
		if (current_limit > chr_type_ichg)
			current_limit = chr_type_ichg;
		g_bcct_value = current_limit;
	} else /* change to default current setting */
		g_bcct_flag = 0;

	mtk_select_ichg_aicr();

	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_bat_charging_current_limit (%d)\n",
		current_limit);

	mutex_unlock(&g_ichg_access_mutex);
	return g_bcct_flag;
}

int mtk_chr_reset_aicr_upper_bound(void)
{
	g_aicr_upper_bound = 0;
	return 0;
}

int set_chr_boost_current_limit(unsigned int current_limit)
{
	int ret = 0;

	ret = battery_charging_control(CHARGING_CMD_SET_BOOST_CURRENT_LIMIT,
		&current_limit);

	return ret;
}

int set_chr_enable_otg(unsigned int enable)
{
	int ret = 0;

	ret = battery_charging_control(CHARGING_CMD_ENABLE_OTG, &enable);

	return ret;
}

int mtk_chr_get_tchr(int *min_temp, int *max_temp)
{
	int ret = 0;
	int temp[2] = {0, 0};

	ret = battery_charging_control(CHARGING_CMD_GET_CHARGER_TEMPERATURE, temp);
	if (ret < 0)
		return ret;

	*min_temp = temp[0];
	*max_temp = temp[1];

	return ret;
}

int mtk_chr_get_soc(unsigned int *soc)
{
	if (BMT_status.SOC < 0) {
		*soc = 0;
		return -ENOTSUPP;
	}

	*soc = BMT_status.SOC;

	return 0;
}

int mtk_chr_get_ui_soc(unsigned int *ui_soc)
{
	/* UI_SOC2 is the one that shows on UI */
	if (BMT_status.UI_SOC2 < 0) {
		*ui_soc = 0;
		return -ENOTSUPP;
	}

	*ui_soc = BMT_status.UI_SOC2;

	return 0;
}

int mtk_chr_get_vbat(unsigned int *vbat)
{
	if (BMT_status.bat_vol < 0) {
		*vbat = 0;
		return -ENOTSUPP;
	}

	*vbat = BMT_status.bat_vol;

	return 0;
}

int mtk_chr_get_ibat(unsigned int *ibat)
{
	*ibat = BMT_status.IBattery / 10;
	return 0;
}

int mtk_chr_get_vbus(unsigned int *vbus)
{
	if (BMT_status.charger_vol < 0) {
		*vbus = 0;
		return -ENOTSUPP;
	}
	*vbus = BMT_status.charger_vol;

	return 0;
}

int mtk_chr_get_aicr(unsigned int *aicr)
{
	int ret = 0;
	u32 _aicr = 0; /* 10uA */

	ret = battery_charging_control(CHARGING_CMD_GET_INPUT_CURRENT, &_aicr);
	*aicr = _aicr / 100;

	return ret;
}

int mtk_chr_is_charger_exist(unsigned char *exist)
{
	*exist = (BMT_status.charger_exist ? 1 : 0);
	return 0;
}

static unsigned int charging_full_check(void)
{
	unsigned int status;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &status);
	if (status == KAL_TRUE) {
		g_full_check_count++;
		if (g_full_check_count >= FULL_CHECK_TIMES)
			return KAL_TRUE;
		else
			return KAL_FALSE;
	}

	g_full_check_count = 0;
	return status;
}

static bool mtk_is_pep_series_connect(void)
{
	if (mtk_pep20_get_is_connect() ||
	    mtk_pep_get_is_connect()) {
		return true;
	}

	return false;
}

static int mtk_check_aicr_upper_bound(void)
{
	u32 aicr_upper_bound = 0; /* 10uA */

	if (mtk_is_pep_series_connect())
		return -EPERM;

	/* Check AICR upper bound gererated by AICL */
	aicr_upper_bound = g_aicr_upper_bound * 100;
	if (g_temp_input_CC_value > aicr_upper_bound && aicr_upper_bound > 0)
		g_temp_input_CC_value = aicr_upper_bound;

	return 0;
}

void select_charging_current(void)
{
	if (g_ftm_battery_flag) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] FTM charging : %d\n",
			    charging_level_data[0]);
		g_temp_CC_value = charging_level_data[0];

		if (g_temp_CC_value == CHARGE_CURRENT_450_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_MAX;
			g_temp_CC_value = batt_cust_data.ac_charger_current;

			battery_log(BAT_LOG_CRTI, "[BATTERY] set_ac_current\n");
		}
	} else {
		if (BMT_status.charger_type == STANDARD_HOST) {
#ifdef CONFIG_USB_IF
			{
				g_temp_input_CC_value = CHARGE_CURRENT_MAX;
				if (g_usb_state == USB_SUSPEND)
					g_temp_CC_value = USB_CHARGER_CURRENT_SUSPEND;
				else if (g_usb_state == USB_UNCONFIGURED)
					g_temp_CC_value = batt_cust_data.usb_charger_current_unconfigured;
				else if (g_usb_state == USB_CONFIGURED)
					g_temp_CC_value = batt_cust_data.usb_charger_current_configured;
				else
					g_temp_CC_value = batt_cust_data.usb_charger_current_unconfigured;

				g_temp_input_CC_value = g_temp_CC_value;

				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\n",
					    g_temp_CC_value, g_usb_state);
			}
#else
			{
				g_temp_input_CC_value = batt_cust_data.usb_charger_current;
				//CEI comment start//				
				//Ibat is not Iusb, it can exceed 500mA, for sync to charging_hw_init CON04 ICHG setting				
				#if 0
				g_temp_CC_value = batt_cust_data.usb_charger_current;
				#else
				g_temp_CC_value = USB_IBAT_CURRENT;
				#endif
				//CEI comment end//	
			}
#endif
		} else if (BMT_status.charger_type == NONSTANDARD_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.non_std_ac_charger_current;
			g_temp_CC_value = batt_cust_data.non_std_ac_charger_current;

		} else if (BMT_status.charger_type == STANDARD_CHARGER) {
			if (batt_cust_data.ac_charger_input_current != 0)
				g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
			else
				g_temp_input_CC_value = batt_cust_data.ac_charger_current;

			g_temp_CC_value = batt_cust_data.ac_charger_current;
			mtk_pep_set_charging_current(&g_temp_CC_value, &g_temp_input_CC_value);
			mtk_pep20_set_charging_current(&g_temp_CC_value, &g_temp_input_CC_value);

		} else if (BMT_status.charger_type == CHARGING_HOST) {
			g_temp_input_CC_value = batt_cust_data.charging_host_charger_current;
			g_temp_CC_value = batt_cust_data.charging_host_charger_current;
		} else if (BMT_status.charger_type == APPLE_2_1A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_2_1a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_2_1a_charger_current;
		} else if (BMT_status.charger_type == APPLE_1_0A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_1_0a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_1_0a_charger_current;
		} else if (BMT_status.charger_type == APPLE_0_5A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_0_5a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_0_5a_charger_current;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE) {
			g_temp_input_CC_value = batt_cust_data.ac_charger_current;
			g_temp_CC_value = batt_cust_data.ac_charger_current;
		}
#endif

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
		set_jeita_charging_current();
#endif

	}

	mtk_check_aicr_upper_bound();
}

void select_charging_current_bcct(void)
{
/*BQ25896 is the first switch chrager separating input and charge current
* any switch charger can use this compile option which may be generalized
* to be CONFIG_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
*/
#ifndef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	if ((BMT_status.charger_type == STANDARD_HOST) ||
	    (BMT_status.charger_type == NONSTANDARD_CHARGER)) {
		if (g_bcct_value < 100)
			g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 500)
			g_temp_input_CC_value = CHARGE_CURRENT_100_00_MA;
		else if (g_bcct_value < 800)
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		else if (g_bcct_value == 800)
			g_temp_input_CC_value = CHARGE_CURRENT_800_00_MA;
		else
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	} else if ((BMT_status.charger_type == STANDARD_CHARGER) ||
		   (BMT_status.charger_type == CHARGING_HOST)) {
		g_temp_input_CC_value = CHARGE_CURRENT_MAX;

		/* --------------------------------------------------- */
		/* set IOCHARGE */
		if (g_bcct_value < 550)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 650)
			g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
		else if (g_bcct_value < 750)
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		else if (g_bcct_value < 850)
			g_temp_CC_value = CHARGE_CURRENT_750_00_MA;
		else if (g_bcct_value < 950)
			g_temp_CC_value = CHARGE_CURRENT_850_00_MA;
		else if (g_bcct_value < 1050)
			g_temp_CC_value = CHARGE_CURRENT_950_00_MA;
		else if (g_bcct_value < 1150)
			g_temp_CC_value = CHARGE_CURRENT_1050_00_MA;
		else if (g_bcct_value < 1250)
			g_temp_CC_value = CHARGE_CURRENT_1150_00_MA;
		else if (g_bcct_value == 1250)
			g_temp_CC_value = CHARGE_CURRENT_1250_00_MA;
		else
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		/* --------------------------------------------------- */

	} else {
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	}
#else
	if (g_bcct_flag == 1)
		g_temp_CC_value = g_bcct_value * 100;
	if (g_bcct_input_flag == 1)
		g_temp_input_CC_value = g_bcct_input_value * 100;
#endif

	mtk_check_aicr_upper_bound();
}

#ifdef CONFIG_CHARGER_QNS
extern int battery_get_qns_current(void);
#endif

//CEI comments, iusb limit for bcct S
extern int TM_enable;
//CEI comments, iusb limit for bcct E

static void mtk_select_ichg_aicr(void)
{
	kal_bool enable_charger = KAL_TRUE;

//CEI comments, iusb limit for bcct S
	int bcct_level_value = bcct_level;
	int chr_vol = BMT_status.charger_vol;
	int chr_current_limt = 0;
	int bcct_now = 0;
//CEI comments, iusb limit for bcct E
	int alien_current = CHARGE_CURRENT_400_00_MA;//CEI comment, Alien battery support
//CEI comment start, SW workaround for IEOC accuracy
	int RT9466_default_IEOC = 150;
	int RT9466_workaround_IEOC = 200;
	int ret = 0;
//CEI comment end, SW workaround for IEOC accuracy
//CEI comment start//
	static unsigned int chargeric_whoami = 0;
//CEI comment end//
#ifdef CONFIG_CHARGER_QNS
	int qns_current = battery_get_qns_current() * 100;
#endif

	mutex_lock(&g_ichg_aicr_access_mutex);

	/* Set Ichg, AICR */
	if (get_usb_current_unlimited()) {
		if (batt_cust_data.ac_charger_input_current != 0)
			g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
		else
			g_temp_input_CC_value = batt_cust_data.ac_charger_current;

		g_temp_CC_value = batt_cust_data.ac_charger_current;
		battery_log(BAT_LOG_FULL,
			"USB_CURRENT_UNLIMITED, use batt_cust_data.ac_charger_current\n");
	}
#ifndef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	else if (g_bcct_flag == 1) {
		select_charging_current_bcct();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_current_bcct !\n");
	} else {
		select_charging_current();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_current !\n");
	}
#else
//CEI comments, iusb limit for bcct S
	else if ((g_bcct_flag == 1 || g_bcct_input_flag == 1) && (TM_enable)) {
//CEI comments, iusb limit for bcct E
		select_charging_current();
//CEI comments, iusb limit for bcct S
#if 0
		select_charging_current_bcct();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_curret_bcct !\n");
#else

		chr_current_limt = adaptive_chrlimit_current(chr_vol, bcct_level_value);
		if (chr_current_limt != 0){
 			if ((BMT_status.charger_type == STANDARD_HOST) || (BMT_status.charger_type == NONSTANDARD_CHARGER)){
				if (chr_current_limt > CHARGE_CURRENT_500_00_MA / 100){
					g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
					g_bcct_input_value = CHARGE_CURRENT_500_00_MA / 100;
				} else {
					g_temp_input_CC_value = chr_current_limt *100;
					g_bcct_input_value = chr_current_limt;
				}
			} else if ((BMT_status.charger_type == STANDARD_CHARGER) || (BMT_status.charger_type == CHARGING_HOST)){
				if ((chr_current_limt < 350) && (chr_vol > 5500)){
						g_temp_input_CC_value = CHARGE_CURRENT_350_00_MA;
						g_bcct_input_value = CHARGE_CURRENT_350_00_MA / 100;
				} else {
						g_temp_input_CC_value = chr_current_limt *100;
						g_bcct_input_value = chr_current_limt;
				}
			} else {
				if(chr_current_limt > CHARGE_CURRENT_500_00_MA / 100) {
					g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
						g_bcct_input_value = CHARGE_CURRENT_500_00_MA / 100;
				} else {
					g_temp_input_CC_value = chr_current_limt *100;
					g_bcct_input_value = chr_current_limt;
				}
			}

			bcct_now = g_bcct_input_value;
			battery_log(BAT_LOG_CRTI, "[BATTERY][IUSB] LE(K)=> select_charging_curret_bcct! bcct_level_value=%d, chr_vol=%d, chr_current_limt=%d, bcct_now=%d\n", bcct_level_value, chr_vol, chr_current_limt, bcct_now);
		} else {
			bcct_now = 0;
			battery_log(BAT_LOG_CRTI, "[BATTERY][IUSB] LE(K)=> select_charging_curret! g_temp_input_CC_value = %d bcct_now = %d\n", g_temp_input_CC_value, bcct_now);
		}
#endif
//CEI comments, iusb limit for bcct E
	} else {
		select_charging_current();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_curret !\n");
	}
#endif

	battery_log(BAT_LOG_CRTI,
		"[BATTERY] Default CC mode charging : %d, input current = %d\n",
		g_temp_CC_value, g_temp_input_CC_value);

	battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
		&g_temp_input_CC_value);
#ifdef CONFIG_CHARGER_QNS
	if  ((qns_current != 0) && (qns_current < g_temp_CC_value)) {
		g_temp_CC_value = qns_current;
		battery_log(BAT_LOG_CRTI, "qns_current is less than current setting, use %dmA\r\n", qns_current/100);
	}
#endif

//CEI comment start, Alien battery support
	if( (g_temp_CC_value > CHARGE_CURRENT_400_00_MA) && (battery_id_invalid == 1)) {
		g_temp_CC_value = alien_current;
		battery_log(BAT_LOG_CRTI, "[Alien] current is less than current setting, use %dmA\r\n", alien_current);
	}
//CEI comment end, Alien battery support

//CEI comment start
	ret = battery_charging_control(CHARGING_CMD_SET_CURRENT,
		&g_temp_CC_value);

//Solve IEOC reset by watchdog issue
	if(chargeric_whoami == 0)
		chargeric_whoami = battery_charging_control(CHARGING_CMD_WHO_AM_I, NULL);

	if(chargeric_whoami == 1) // RT9466
	{
//CEI comment start, SW workaround for IEOC accuracy
		if (ret == 0) {
			if (g_temp_CC_value > CHARGE_CURRENT_800_00_MA) {
				battery_charging_control(CHARGING_CMD_SET_IEOC, &RT9466_default_IEOC);
				battery_log(BAT_LOG_CRTI, "RT9466 IEOC setting %dmA\n", RT9466_default_IEOC);
			} else {// Below 800mA, use 200mA value.
				battery_charging_control(CHARGING_CMD_SET_IEOC, &RT9466_workaround_IEOC);
				battery_log(BAT_LOG_CRTI, "RT9466  IEOC setting %dmA\r\n", RT9466_workaround_IEOC);
			}
		}
	}
//CEI comment end, SW workaround for IEOC accuracy
	else if (chargeric_whoami == 2) // BQ25896
	{

		int BQ25896_IEOC = 128;
		battery_charging_control(CHARGING_CMD_SET_IEOC, &BQ25896_IEOC);
		battery_log(BAT_LOG_CRTI, "CEI(K)=> BQ25896  IEOC (Iterm) setting %dmA\r\n", BQ25896_IEOC);
	}
//CEI comment end

	/* For thermal, they need to enable charger immediately */
	if (g_temp_CC_value > 0 && g_temp_input_CC_value > 0)
		battery_charging_control(CHARGING_CMD_ENABLE, &enable_charger);

	/* If AICR < 300mA, stop PE+/PE+20 */
	if (g_temp_input_CC_value < CHARGE_CURRENT_300_00_MA) {
		if (mtk_pep20_get_is_enable()) {
			mtk_pep20_set_is_enable(false);
			if (mtk_pep20_get_is_connect())
				mtk_pep20_reset_ta_vchr();
		}
		if (mtk_pep_get_is_enable()) {
			mtk_pep_set_is_enable(false);
			if (mtk_pep_get_is_connect())
				mtk_pep_reset_ta_vchr();
		}
	} else if (g_bcct_input_flag == 0 && g_bcct_flag == 0) {
		if (!mtk_pep20_get_is_enable()) {
			mtk_pep20_set_is_enable(true);
			mtk_pep20_set_to_check_chr_type(true);
		}
		if (!mtk_pep_get_is_enable()) {
			mtk_pep_set_is_enable(true);
			mtk_pep_set_to_check_chr_type(true);
		}
	}

	mutex_unlock(&g_ichg_aicr_access_mutex);

//CEI comments, iusb limit for bcct S
	set_bcct_log(bcct_now);
	battery_log(BAT_LOG_CRTI,
		    "[BATTERY][IUSB] LE(K)=> %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u\n",
		    bcct_log.bcct_value[0], bcct_log.bcct_count[0], bcct_log.bcct_value[1], bcct_log.bcct_count[1], bcct_log.bcct_value[2], bcct_log.bcct_count[2], bcct_log.bcct_value[3], bcct_log.bcct_count[3], bcct_log.bcct_value[4], bcct_log.bcct_count[4],
		    bcct_log.bcct_value[5], bcct_log.bcct_count[5], bcct_log.bcct_value[6], bcct_log.bcct_count[6], bcct_log.bcct_value[7], bcct_log.bcct_count[7], bcct_log.bcct_value[8], bcct_log.bcct_count[8]);
	battery_log(BAT_LOG_CRTI,
		    "[BATTERY][IUSB] LE(K)=> %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, %dmA=%u, normal=%u\n\n",
		    bcct_log.bcct_value[9], bcct_log.bcct_count[9], bcct_log.bcct_value[10], bcct_log.bcct_count[10], bcct_log.bcct_value[11], bcct_log.bcct_count[11], bcct_log.bcct_value[12], bcct_log.bcct_count[12], bcct_log.bcct_value[13], bcct_log.bcct_count[13],
		    bcct_log.bcct_value[14], bcct_log.bcct_count[14], bcct_log.bcct_value[15], bcct_log.bcct_count[15], bcct_log.bcct_count[16]);
//CEI comments, iusb limit for bcct E

}

static void mtk_select_cv(void)
{
	int ret = 0;
	u32 dynamic_cv = 0;
	BATTERY_VOLTAGE_ENUM cv_voltage;

#ifdef CONFIG_MTK_JEITA_STANDARD_SUPPORT
	/* If temperautre is abnormal, return not permitted */
	if (g_temp_status != TEMP_POS_10_TO_POS_45)
		return;
#endif

	cv_voltage = MTK_CV_VOLTAGE;

	ret = mtk_get_dynamic_cv(&dynamic_cv);
	if (ret == 0) {
		cv_voltage = dynamic_cv * 1000;
		battery_log(BAT_LOG_FULL, "%s: set dynamic cv = %dmV\n",
			__func__, dynamic_cv);
	}
//CEI comments, soft charger 3.0 start
#if (CONFIG_MTK_GAUGE_VERSION == 20) && defined(CUSTOM_SOFT_CHARGE_30)
	if (g_sc30_low_cv_voltage)
		cv_voltage = MTK_SC30_LOWER_CV_VOLTAGE;

	battery_log(BAT_LOG_CRTI, "[SC30] low_cv=%d cv_voltage=%d\n",
		g_sc30_low_cv_voltage, cv_voltage);
#endif
//CEI comments, soft charger 3.0 end

//CEI comment start, Alien battery support
	if (battery_id_invalid) {
		cv_voltage = BATTERY_VOLT_04_000000_V;
		battery_log(BAT_LOG_CRTI, "[Alien]mtk_select_cv(): set cv_voltage=%d\n", cv_voltage);
	}
//CEI comment end, Alien battery support

	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);

#if defined(CONFIG_MTK_HAFG_20)
	g_cv_voltage = cv_voltage;
#endif
}

static void pchr_turn_on_charging(void)
{
	u32 charging_enable = KAL_TRUE;

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
	if (BMT_status.charger_exist)
		charging_enable = KAL_TRUE;
	else
		charging_enable = KAL_FALSE;
#endif

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] Charger Error, turn OFF charging !\n");
		charging_enable = KAL_FALSE;
	} else if ((g_platform_boot_mode == META_BOOT) ||
		   (g_platform_boot_mode == ADVMETA_BOOT)) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = KAL_FALSE;
	} else {
		/* HW initialization */
		battery_charging_control(CHARGING_CMD_INIT, NULL);
		battery_log(BAT_LOG_FULL, "charging_hw_init\n");

		/* PE+/PE+20 algorithm */
		mtk_pep20_start_algorithm();
		mtk_pep_start_algorithm();

		/* Select ICHG/AICR */
		mtk_select_ichg_aicr();

		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA ||
		    g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {
			charging_enable = KAL_FALSE;
			battery_log(BAT_LOG_CRTI,
				"[BATTERY] charging current is set 0mA, turn off charging !\n");
		} else /* Set CV Voltage */
			mtk_select_cv();

	}

	/* enable/disable charging */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_log(BAT_LOG_CRTI,
		"[BATTERY] pchr_turn_on_charging(), enable =%d, ichg =%d, ibus =%d\n",
		charging_enable, g_temp_CC_value, g_temp_input_CC_value);
}


PMU_STATUS BAT_PreChargeModeAction(void)
{
#ifdef CONFIG_MTK_BIF_SUPPORT
	int ret = 0;
	bool bif_exist = false;
#endif
	unsigned int led_en = true;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%d on %d !!\n",
		    BMT_status.PRE_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

#ifdef CONFIG_MTK_BIF_SUPPORT
	/* If defined BIF but not BIF's battery, stop charging */
	ret = battery_charging_control(CHARGING_CMD_GET_BIF_IS_EXIST,
		&bif_exist);
	if (!bif_exist) {
		battery_log(BAT_LOG_CRTI,
			"%s: define BIF but no BIF battery, disable charging\n",
			__func__);
		BMT_status.bat_charging_state = CHR_ERROR;
		return PMU_STATUS_OK;
	}
#endif

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*  Enable charger */
	pchr_turn_on_charging();
#if (CONFIG_MTK_GAUGE_VERSION == 20)
// CEI comment start//
// BAT_status_Full feature, force go formal CHR_PRE -> CHR_CC (detect 6 times bat full of bq25896 status) -> CHR_BATFULL
//     if (BMT_status.UI_SOC2 == 100 && charging_full_check()) {
       if (0) {
// CEI comment end//
#else
	if (BMT_status.UI_SOC == 100) {
#endif
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	} else if (BMT_status.bat_vol > V_PRE2CC_THRES) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	/* If it is not disabled by throttling,
	 * enable PE+/PE+20, if it is disabled
	 */
	if (g_bcct_input_flag && g_bcct_input_value < 300)
		return PMU_STATUS_OK;

	if (!mtk_pep20_get_is_enable()) {
		mtk_pep20_set_is_enable(true);
		mtk_pep20_set_to_check_chr_type(true);
	}
	if (!mtk_pep_get_is_enable()) {
		mtk_pep_set_is_enable(true);
		mtk_pep_set_to_check_chr_type(true);
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	unsigned int led_en = true;

	battery_log(BAT_LOG_CRTI, "[BATTERY] CC mode charge, timer=%d on %d!!\n",
		    BMT_status.CC_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*  Enable charger */
	pchr_turn_on_charging();

	if (charging_full_check() == KAL_TRUE) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryFullAction(void)
{
	unsigned int led_en = false;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n");

	BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	battery_log(BAT_LOG_FULL, "Turn off PWRSTAT LED\n");
	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*
	 * If CV is set to lower value by JEITA,
	 * Reset CV to normal value if temperture is in normal zone
	 */
	mtk_select_cv();

	if (charging_full_check() == KAL_FALSE) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n");

		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_CC;
#if (CONFIG_MTK_GAUGE_VERSION != 20)
		battery_meter_reset();
#endif
		mtk_pep20_set_to_check_chr_type(true);
		mtk_pep_set_to_check_chr_type(true);
		g_enable_dynamic_cv = true;
	}


	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryHoldAction(void)
{
	unsigned int charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n");

	if (BMT_status.bat_vol < TALKING_RECHARGE_VOLTAGE || g_call_state == CALL_IDLE) {
		BMT_status.bat_charging_state = CHR_CC;
		battery_log(BAT_LOG_CRTI, "[BATTERY] Exit Hold mode and Enter CC mode !!\n");
	}

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	unsigned int charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n");

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	if ((g_temp_status == TEMP_ABOVE_POS_60) || (g_temp_status == TEMP_BELOW_NEG_10))
		temp_error_recovery_chr_flag = KAL_FALSE;

	if ((temp_error_recovery_chr_flag == KAL_FALSE) && (g_temp_status != TEMP_ABOVE_POS_60)
	    && (g_temp_status != TEMP_BELOW_NEG_10)) {
		temp_error_recovery_chr_flag = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_PRE;
	}
#endif

	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	/* Disable PE+/PE+20 */
	if (mtk_pep20_get_is_enable()) {
		mtk_pep20_set_is_enable(false);
		if (mtk_pep20_get_is_connect())
			mtk_pep20_reset_ta_vchr();
	}
	if (mtk_pep_get_is_enable()) {
		mtk_pep_set_is_enable(false);
		if (mtk_pep_get_is_connect())
			mtk_pep_reset_ta_vchr();
	}

	return PMU_STATUS_OK;
}


void mt_battery_charging_algorithm(void)
{
	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

	/* Generate AICR upper bound by AICL */
	if (!mtk_is_pep_series_connect()) {
		battery_charging_control(CHARGING_CMD_RUN_AICL,
			&g_aicr_upper_bound);
	}

	mtk_pep20_check_charger();
	mtk_pep_check_charger();
	switch (BMT_status.bat_charging_state) {
	case CHR_PRE:
		BAT_PreChargeModeAction();
		break;

	case CHR_CC:
		BAT_ConstantCurrentModeAction();
		break;

	case CHR_BATFULL:
		BAT_BatteryFullAction();
		break;

	case CHR_HOLD:
		BAT_BatteryHoldAction();
		break;

	case CHR_ERROR:
		BAT_BatteryStatusFailAction();
		break;
	}

	battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
}
