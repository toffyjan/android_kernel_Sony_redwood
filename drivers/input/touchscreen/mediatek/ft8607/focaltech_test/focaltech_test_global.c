/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)All Rights Reserved.
*
* File Name: focaltech_test_global.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: global function for test
*
************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/delay.h>//msleep

#include "../cei_touch_log.h"
#include "focaltech_test_ini.h"
#include "focaltech_test_global.h"
#include "focaltech_test_main.h"

#define DEVIDE_MODE_ADDR	0x00

#define FTS_MALLOC_TYPE		1 // 0: kmalloc, 1: vmalloc
enum enum_malloc_mode
{
	kmalloc_mode = 0,
	vmalloc_mode = 1,
};

struct StruScreenSeting g_ScreenSetParam;
struct stTestItem g_stTestItem[1][MAX_TEST_ITEM];
struct structSCapConfEx g_stSCapConfEx;
//struct structSCapConf g_stSCapConf;
int g_TestItemNum = 0;
char g_strIcName[20] = {0};
char *g_pStoreAllData = NULL;

int GetPrivateProfileString(char *section, char *ItemName,
		char *defaultvalue, char *returnValue, char *IniFile)
{
	char value[512] = {0};
	int len = 0;

	if (NULL == returnValue) {
		TP_TEST_LOGE("returnValue is NULL\n");
		return 0;
	}

	if (ini_get_key(IniFile, section, ItemName, value) < 0) {
		if (NULL != defaultvalue)
			memcpy(value, defaultvalue, strlen(defaultvalue));
		sprintf(returnValue, "%s", value);
		return 0;
	} else {
		len = sprintf(returnValue, "%s", value);
	}

	return len;
}

void focal_msleep(int ms)
{
	msleep(ms);
}

void SysDelay(int ms)
{
	msleep(ms);
}

int focal_abs(int value)
{
	if (value < 0)
		value = 0 - value;

	return value;
}

void *fts_malloc(size_t size)
{
	if (FTS_MALLOC_TYPE == kmalloc_mode)
		return kmalloc(size, GFP_ATOMIC);
	else if (FTS_MALLOC_TYPE == vmalloc_mode)
		return vmalloc(size);
	else
		TP_TEST_LOGE("invalid malloc.\n");

	return NULL;
}

void fts_free(void *p)
{
	if (FTS_MALLOC_TYPE == kmalloc_mode)
		return kfree(p);
	else if (FTS_MALLOC_TYPE == vmalloc_mode)
		return vfree(p);
	else
		TP_TEST_LOGE("invalid free.\n");
}

void OnInit_InterfaceCfg(char *strIniFile)
{
	char str[128] = {0};

	/* IC_Type */
	GetPrivateProfileString("Interface", "IC_Type", "FT8607", str, strIniFile);
	TP_TEST_LOGI("IC_Type = %s\n", str);

#if 0
	g_ScreenSetParam.iSelectedIC = fts_ic_table_get_ic_code_from_ic_name(str);
#else
	/*
	 * we force set 0xC0 (means FT8607) to iSelectedIC, because we
	 * did not merge focaltech_global in this driver.
	 */
	if (strncmp(str, "FT8607", 6) != 0)
		TP_TEST_LOGW("==== NOTE: you are not use FT8607's ini file !!! ===\n");

	g_ScreenSetParam.iSelectedIC = 0xC0;
#endif

	TP_TEST_LOGI("IC code :0x%02X\n", g_ScreenSetParam.iSelectedIC);

	/* Normalize Type */
	GetPrivateProfileString("Interface", "Normalize_Type", 0, str, strIniFile);
	TP_TEST_LOGI("Normalize_Type = %s\n", str);

	g_ScreenSetParam.isNormalize = fts_atoi(str);

}

/************************************************************************
* Name: ReadReg(Same function name as FT_MultipleTest)
* Brief:  Read Register
* Input: RegAddr
* Output: RegData
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
int ReadReg(u8 RegAddr, u8 *RegData)
{
	int iRet;

	if (NULL == fts_i2c_read_test) {
		TP_TEST_LOGE("fts_i2c_read_test is NULL !!!\n");
		return ERROR_CODE_INVALID_COMMAND;
	}

	iRet = fts_i2c_read_test(&RegAddr, 1, RegData, 1);

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

/************************************************************************
* Name: WriteReg(Same function name as FT_MultipleTest)
* Brief:  Write Register
* Input: RegAddr, RegData
* Output: null
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
int WriteReg(u8 RegAddr, u8 RegData)
{
	int iRet;
	unsigned char cmd[2] = {0};

	if (NULL == fts_i2c_write_test) {
		TP_TEST_LOGE("fts_i2c_write_test == NULL !!!\n");
		return ERROR_CODE_INVALID_COMMAND;
	}

	cmd[0] = RegAddr;
	cmd[1] = RegData;
	iRet = fts_i2c_write_test(cmd, 2);

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

/************************************************************************
* Name: Comm_Base_IIC_IO(Same function name as FT_MultipleTest)
* Brief:  Write/Read Data by IIC
* Input: pWriteBuffer, iBytesToWrite, iBytesToRead
* Output: pReadBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char Comm_Base_IIC_IO(u8 *pWriteBuffer, int iBytesToWrite,
		u8 *pReadBuffer, int iBytesToRead)
{
	int iRet;

	if (NULL == fts_i2c_read_test) {
		TP_TEST_LOGE("fts_i2c_read_test is NULL !!!\n");
		return ERROR_CODE_INVALID_COMMAND;
	}

	iRet = fts_i2c_read_test(pWriteBuffer, iBytesToWrite, pReadBuffer, iBytesToRead);

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

/************************************************************************
* Name: EnterWork(Same function name as FT_MultipleTest)
* Brief:  Enter Work Mode
* Input: null
* Output: null
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char EnterWork(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	TP_TEST_LOGI("start\n");

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if (ReCode == ERROR_CODE_OK) {
		if (((RunState >> 4) & 0x07) == 0x00) {
			/* already in Work mode */
			ReCode = ERROR_CODE_OK;
		} else {
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0);
			if (ReCode == ERROR_CODE_OK) {
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if (ReCode == ERROR_CODE_OK) {
					if (((RunState >> 4) & 0x07) == 0x00)
						ReCode = ERROR_CODE_OK;
					else
						ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	TP_TEST_LOGI("end\n");

	return ReCode;
}

/************************************************************************
* Name: EnterFactory
* Brief:  enter Fcatory Mode
* Input: null
* Output: null
* Return: Comm Code. Code = 0 is OK, else fail.
***********************************************************************/
unsigned char EnterFactory(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	TP_TEST_LOGI("start\n");

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if (ReCode == ERROR_CODE_OK) {
		if (((RunState >> 4) & 0x07) == 0x04) {
			/* already in Factory mode */
			ReCode = ERROR_CODE_OK;
		} else {
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0x40);
			if (ReCode == ERROR_CODE_OK) {
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if (ReCode == ERROR_CODE_OK) {
					if (((RunState >> 4) & 0x07) == 0x04)
						ReCode = ERROR_CODE_OK;
					else
						ReCode = ERROR_CODE_COMM_ERROR;
				} else
					TP_TEST_LOGE("EnterFactory read DEVIDE_MODE_ADDR error 3\n");
			} else
				TP_TEST_LOGE("EnterFactory write DEVIDE_MODE_ADDR error 2\n");
		}
	} else
		TP_TEST_LOGE("EnterFactory read DEVIDE_MODE_ADDR error 1\n");

	TP_TEST_LOGI("end\n");

	return ReCode;
}

/************************************************************************
* Name: fts_SetTestItemCodeName
* Brief:  set test item code and name
* Input: null
* Output: null
* Return:
**********************************************************************/
void fts_SetTestItemCodeName(unsigned char itemcode)
{
	g_stTestItem[0][g_TestItemNum].ItemCode = itemcode;
	g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
	g_stTestItem[0][g_TestItemNum].TestResult = RESULT_NULL;
	g_TestItemNum++;
}
