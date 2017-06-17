/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_test_main.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test entry for all IC
*
************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/slab.h>

#include "../focaltech_core.h"
#include "../cei_touch_log.h"

#include "focaltech_test_main.h"
#include "focaltech_test_global.h"

#include "focaltech_test_config_ft8607.h"
#include "focaltech_test_ft8607.h"

#include "focaltech_test_ini.h"

#define FTS_DRIVER_LIB_INFO  "Test_Lib_Version   V1.6.1 2016-05-19"

#define FTS_TEST_STORE_DATA_SIZE	80 * 1024

FTS_I2C_READ_FUNCTION fts_i2c_read_test;
FTS_I2C_WRITE_FUNCTION fts_i2c_write_test;

char *g_testparamstring = NULL;

/////////////////////IIC communication
int init_i2c_read_func(FTS_I2C_READ_FUNCTION fpI2C_Read)
{
	unsigned char value = 0;
	unsigned char recode = 0;

	fts_i2c_read_test = fpI2C_Read;
	if (NULL == fts_i2c_read_test) {
		TP_TEST_LOGE("fts_i2c_read_test is NULL\n");
	}

	// debug start
	recode = ReadReg(0xa6, &value);
	if (recode != ERROR_CODE_OK) {
		TP_TEST_LOGE("ReadReg Error, code: %d\n", recode);
	} else {
		TP_TEST_LOGI("ReadReg successed, Addr: 0xa6, value: 0x%02x\n", value);
	}
	// debug end

	return 0;
}

int init_i2c_write_func(FTS_I2C_WRITE_FUNCTION fpI2C_Write)
{
	fts_i2c_write_test = fpI2C_Write;
	if (NULL == fts_i2c_write_test) {
		TP_TEST_LOGE("fts_i2c_read_test is NULL\n");
	}

	return 0;
}

/************************************************************************
* Name: set_param_data
* Brief:  load Config. Set IC series, init test items, init basic threshold,
*         int detailThreshold, and set order of test items
* Input: TestParamData, from ini file.
* Output: none
* Return: 0. No sense, just according to the old format.
***********************************************************************/
int set_param_data(char *TestParamData)
{
	int ret = 0;

	TP_TEST_LOGI("Enter set_param_data.\n");

	g_testparamstring = TestParamData; // get param of ini file
	ret = ini_get_key_data(g_testparamstring); // get param to struct
	if (ret < 0) {
		TP_TEST_LOGE("ini_get_key_data error.\n");
		return ret;
	}

	/* set g_ScreenSetParam.iSelectedIC */
	OnInit_InterfaceCfg(g_testparamstring);

	/* Get IC Name */
#if 0
	fts_ic_table_get_ic_name_from_ic_code(g_ScreenSetParam.iSelectedIC, g_strIcName);
#else
	/*
	 * we force "FT8607" to g_strIcName, because we
	 * did not merge focaltech_global in this driver.
	 */
	sprintf(g_strIcName, "%s", "FT8607");
#endif

#if 0
	/* config test items */
	if (IC_FT5X46 >> 4 == g_ScreenSetParam.iSelectedIC >> 4)
	{
		OnInit_FT5X22_TestItem(g_testparamstring);
		OnInit_FT5X22_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT5X22();
	}
	else if(IC_FT8606 >> 4 == g_ScreenSetParam.iSelectedIC >> 4)
	{
		OnInit_FT8606_TestItem(g_testparamstring);
		OnInit_FT8606_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT8606();
	}
	else if(IC_FT5822>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT5822_TestItem(g_testparamstring);
		OnInit_FT5822_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT5822();
	}
	else if(IC_FT6X36>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT6X36_TestItem(g_testparamstring);
		OnInit_FT6X36_BasicThreshold(g_testparamstring);
		OnInit_SCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT6X36();
	}
	else if(IC_FT3C47U>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT3C47_TestItem(g_testparamstring);
		OnInit_FT3C47_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT3C47();
	}
	else if(IC_FT8716>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT8716_TestItem(g_testparamstring);
		OnInit_FT8716_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT8716();
	}
	else if(IC_FT8736>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT8736_TestItem(g_testparamstring);
		OnInit_FT8736_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT8736();
	}
	else if(IC_FT8607>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FT8607_TestItem(g_testparamstring);
		OnInit_FT8607_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT8607();
	}
	else if(IC_FTE716>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FTE716_TestItem(g_testparamstring);
		OnInit_FTE716_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FTE716();
	}
	else if(IC_FTE736>>4 == g_ScreenSetParam.iSelectedIC>>4)
	{
		OnInit_FTE736_TestItem(g_testparamstring);
		OnInit_FTE736_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FTE736();
	}
#else
	/*
	 * we force perform FT8607's tests, because we
	 * did not merge focaltech_global in this driver.
	 */
	OnInit_FT8607_TestItem(g_testparamstring);
	OnInit_FT8607_BasicThreshold(g_testparamstring);
	OnInit_MCap_DetailThreshold(g_testparamstring);
	SetTestItem_FT8607();
#endif

/*
	gettimeofday(&time_end, NULL); // End time
	time_use = (time_end.tv_sec - time_start.tv_sec) * 1000 +
		   (time_end.tv_usec - time_start.tv_usec) / 1000;
	TP_TEST_LOGI("Load Config, use time = %d ms\n", time_use);
*/

	TP_TEST_LOGI("end of set_param_data.\n");

	return 0;
}

/************************************************************************
* Name: start_test_tp
* Brief:  Test entry. Select test items based on IC series
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/

bool start_test_tp(void)
{
	bool bTestResult = false;

	TP_TEST_LOGI("start\n");
	TP_TEST_LOGI("IC_%s Test\n", g_strIcName);

#if 0
	switch (g_ScreenSetParam.iSelectedIC >> 4) {
	case IC_FT5X46 >> 4:
//#if FTS_AUTO_RESET_EN
		fts_auto_reset_suspend();
		fts_auto_reset_record_time();
//#endif

		bTestResult = FT5X46_StartTest();

//#if FTS_AUTO_RESET_EN
		fts_auto_reset_resume();
//#endif
		break;
	case IC_FT8606>>4:
		bTestResult = FT8606_StartTest();
		break;
	case IC_FT5822>>4:
		bTestResult = FT5822_StartTest();
		break;
	case IC_FT6X36>>4:
		bTestResult = FT6X36_StartTest();
		break;
	case IC_FT3C47U>>4:
		bTestResult = FT3C47_StartTest();
		break;
	case IC_FT8716>>4:
//#if FTS_AUTO_RESET_EN
		fts_auto_reset_suspend();
		fts_auto_reset_record_time();
//#endif

		bTestResult = FT8716_StartTest();

//#if FTS_AUTO_RESET_EN
		fts_auto_reset_resume();
//#endif
		break;
	case IC_FT8736>>4:
		bTestResult = FT8736_StartTest();
		break;
	case IC_FT8607>>4:
		bTestResult = FT8607_StartTest();
		break;
	case IC_FTE716>>4:
		bTestResult = FTE716_StartTest();
		break;
	case IC_FTE736>>4:
		bTestResult = FTE736_StartTest();
		break;
	default:
		TP_TEST_LOGE("Error IC, IC Name: %s, IC Code: %d\n",
				g_strIcName, g_ScreenSetParam.iSelectedIC);
		break;
	}
#else
	/*
	 * we force perform FT8607's tests, because we
	 * did not merge focaltech_global in this driver.
	 */
	bTestResult = FT8607_StartTest();
#endif

	/* back to work mode */
	EnterWork();

	return bTestResult;
}
/************************************************************************
* Name: get_test_data
* Brief:  Get test data based on IC series
* Input: none
* Output: pTestData, External application for memory, buff size >= 1024*8
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int get_test_data(char *pTestData)
{
	int iLen = 0;

	TP_TEST_LOGI("start\n");

#if 0
	switch (g_ScreenSetParam.iSelectedIC >> 4) {
	case IC_FT5X46>>4:
		iLen = FT5X46_get_test_data(pTestData);
		break;

	case IC_FT8606>>4:
		iLen = FT8606_get_test_data(pTestData);
		break;
	case IC_FT5822>>4:
		iLen = FT5822_get_test_data(pTestData);
		break;
	case IC_FT6X36>>4:
		iLen = FT6X36_get_test_data(pTestData);
		break;
	case IC_FT3C47U>>4:
		iLen = FT3C47_get_test_data(pTestData);
		break;
	case IC_FT8716>>4:
		iLen = FT8716_get_test_data(pTestData);
		break;
	case IC_FT8736>>4:
		iLen = FT8736_get_test_data(pTestData);
		break;
	case IC_FT8607>>4:
		iLen = FT8607_get_test_data(pTestData);
		break;
	case IC_FTE716>>4:
		iLen = FTE716_get_test_data(pTestData);
		break;
	case IC_FTE736>>4:
		iLen = FTE736_get_test_data(pTestData);
		break;
	default:
		TP_TEST_LOGE("Error IC, IC Name: %s, IC Code:  %d\n",
				g_strIcName, g_ScreenSetParam.iSelectedIC);
		break;
	}
#else
	iLen = FT8607_get_test_data(pTestData);
#endif

	return iLen;
}

int focaltech_test_main_init(void)
{
	int ret = 0;

	TP_TEST_LOGI("%s\n",  FTS_DRIVER_LIB_INFO); // show lib version

	/* allocate memory to save test result */
	g_pStoreAllData = NULL;
	if (NULL == g_pStoreAllData)
		g_pStoreAllData = fts_malloc(FTS_TEST_STORE_DATA_SIZE);
	if (NULL == g_pStoreAllData)
		return -1;

	ret = malloc_struct_DetailThreshold();
	if (ret < 0)
		return ret;

	return 0;
}
/************************************************************************
* Name: free_test_param_data
* Brief:  release printer memory
* Input: none
* Output: none
* Return: none.
***********************************************************************/
int focaltech_test_main_exit(void)
{

	TP_TEST_LOGI("release memory -start.\n");

	if (g_testparamstring)
		fts_free(g_testparamstring);
	g_testparamstring = NULL;

	if (g_pStoreAllData)
		fts_free(g_pStoreAllData);

	free_struct_DetailThreshold();

	release_key_data(); // release memory of key data for ini file

	TP_TEST_LOGI("release memory -end.\n");

	return 0;
}

