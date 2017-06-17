/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
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

 /************************************************************************
*
* File Name: focaltech_test.c
*
* Author:	  Software Department, FocalTech
*
* Created: 2016-03-24
*
* Modify:
*
* Abstract: create char device and proc node for  the comm between APK and TP
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>

#include <linux/i2c.h>
#include <linux/delay.h>

#include "../focaltech_core.h"
#include "../cei_touch_log.h"
#include "focaltech_test_main.h"
#include "focaltech_test_ini.h"
#include "focaltech_test_global.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_TEST_INFO "File Version of focaltech_test.c: 2016-06-28"

#define FTS_TEST_DEFAULT_READ_PATH "/etc/"
#define FTS_TEST_DEFAULT_SAVE_PATH "/data/"
//#define FTS_TEST_DEFAULT_INI_FILE_NAME "ft8607_self_test.ini"
#define FTS_TEST_AUO_INI_FILE_NAME "ft8607_AUO_self_test.ini"
#define FTS_TEST_SHARP_INI_FILE_NAME "ft8607_SHARP_self_test.ini"
#define FTS_TEST_INX_INI_FILE_NAME "ft8607_INX_self_test.ini"
#define FTS_TEST_AUO_RESULT_FILE_NAME "ft8607_AUO_self_test_data.csv" //(AUO)
#define FTS_TEST_SHARP_RESULT_FILE_NAME "ft8607_SHARP_self_test_data.csv" //(SHARP)
#define FTS_TEST_INX_RESULT_FILE_NAME "ft8607_INX_self_test_data.csv" //(INX)
#define FTS_TEST_BUFFER_SIZE	80 * 1024
#define FTS_TEST_PRINT_SIZE	256
/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int fts_test_get_ini_size(char *config_name);
static int fts_test_read_ini_data(char *config_name, char *config_buf);
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen);
static int fts_test_get_testparam_from_ini(char *config_name);
static int fts_test_entry(char *ini_file_name);

static int fts_test_i2c_read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen);
static int fts_test_i2c_write(unsigned char *writebuf, int writelen);

/*******************************************************************************
* functions body
*******************************************************************************/

static int fts_test_i2c_read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen)
{
	int iret = -1;

	iret = fts_i2c_read(fts_i2c_client, writebuf, writelen, readbuf, readlen);

	return iret;
}

static int fts_test_i2c_write(unsigned char *writebuf, int writelen)
{
	int iret = -1;

	iret = fts_i2c_write(fts_i2c_client, writebuf, writelen);

	return iret;
}

// get ini file size, for memory allocate
static int fts_test_get_ini_size(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
//	unsigned long magic;
	off_t fsize = 0;

	if (NULL == pfile)
		pfile = filp_open(config_name, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		TP_TEST_LOGE("error occured while opening file %s\n", config_name);
		return -EIO;
	}

	/* get file size from inode */
	inode = pfile->f_path.dentry->d_inode;
//	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;

	filp_close(pfile, NULL);

	return fsize;
}

// Read ini data into buffer
static int fts_test_read_ini_data(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
//	unsigned long magic;
	off_t fsize = 0;
	loff_t pos = 0;
	mm_segment_t old_fs;

	if (NULL == pfile)
		pfile = filp_open(config_name, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		TP_TEST_LOGE("error occured while opening file %s\n", config_name);
		return -EIO;
	}

	inode = pfile->f_path.dentry->d_inode;
//	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

// Save test data to storge
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen)
{
	struct file *pfile = NULL;

	loff_t pos;
	mm_segment_t old_fs;

	TP_TEST_LOGI("save test result to: %s\n", file_name);

	if (NULL == pfile)
		pfile = filp_open(file_name, O_CREAT | O_RDWR | O_TRUNC, 0);
	if (IS_ERR(pfile)) {
		TP_TEST_LOGE("error occured while opening file %s\n", file_name);
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_write(pfile, data_buf, iLen, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

// Read and parse test parameters
static int fts_test_get_testparam_from_ini(char *config_name)
{
	char *pcfiledata = NULL;
	int ret = 0;

	int inisize = fts_test_get_ini_size(config_name);

	TP_TEST_LOGI("ini_size = %d\n", inisize);

	if (inisize <= 0) {
		TP_TEST_LOGE("ERROR: Get firmware size failed\n");
		return -EIO;
	}

	pcfiledata = fts_malloc(inisize + 1);
	if (NULL == pcfiledata) {
		TP_TEST_LOGE("pcfiledata fts_malloc failed\n");
		return -1;
	}

	memset(pcfiledata, 0, inisize + 1);

	if (fts_test_read_ini_data(config_name, pcfiledata)) {
		TP_TEST_LOGE("ERROR: fts_test_read_ini_data failed\n");
		fts_free(pcfiledata);
		pcfiledata = NULL;

		return -EIO;
	} else {
		TP_TEST_LOGI("fts_test_read_ini_data successful\n");
	}

	ret = set_param_data(pcfiledata);

	fts_free(pcfiledata);
	pcfiledata = NULL;

	if (ret < 0)
		return ret;

	return 0;
}

/////////////////////////////////
// Test library entry
///////////////////////////////////
static int fts_test_entry(char *ini_file_name)
{
	/* place holder for future use */
	char testdataname[128] = {0};
	char *testdata = NULL;
	char *printdata = NULL;
	int iTestDataLen = 0; // test data length, for saving to document
	int ret = 0;
	int icycle = 0, i = 0;
	int print_index = 0;
	bool test_result = false;

	TP_TEST_LOGI("ini_file_name: %s\n", ini_file_name);

	/* for saving test data, notice the allocate size */
	TP_TEST_LOGI("Allocate memory, size: %d\n", FTS_TEST_BUFFER_SIZE);
	testdata = fts_malloc(FTS_TEST_BUFFER_SIZE);
	if (NULL == testdata) {
		TP_TEST_LOGE("testdata fts_malloc failed\n");
		fts_free(testdata);
		return -1;
	}

	printdata = fts_malloc(FTS_TEST_PRINT_SIZE);
	if (NULL == printdata) {
		TP_TEST_LOGE("printdata fts_malloc failed\n");
		fts_free(printdata);
		fts_free(testdata);
		return -1;
	}

	/* Initial i2c read & write function */
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);

	/* initial memories */
	ret = focaltech_test_main_init();
	if (ret < 0) {
		TP_TEST_LOGE("focaltech_test_main_init() error\n");
		goto TEST_ERR;
	}

	/* read and parse ini file */
	if (fts_test_get_testparam_from_ini(ini_file_name) < 0) {
		TP_TEST_LOGE("get testparam from ini failure\n");
		goto TEST_ERR;
	}

	/* start test */
	test_result = start_test_tp();

	if (true == test_result)
		TP_TEST_LOGI("tp test pass\n");
	else
		TP_TEST_LOGE("tp test failure\n");

	/* get the test data and save it */
	iTestDataLen = get_test_data(testdata);

	icycle = 0;

	/* print test data */
	TP_TEST_LOGD("print test data: \n");

	for (i = 0; i < iTestDataLen; i++) {
		if (('\0' == testdata[i]) ||
			(icycle == FTS_TEST_PRINT_SIZE - 2) ||
			(i == iTestDataLen - 1))
		{
			if (icycle == 0) {
				print_index++;
			} else {
				memcpy(printdata, testdata + print_index, icycle);
				printdata[FTS_TEST_PRINT_SIZE - 1] = '\0';
				TP_TEST_LOGD("%s\n", printdata);
				print_index += icycle;
				icycle = 0;
			}
		} else {
			icycle++;
		}
	}
	TP_TEST_LOGD("\n");

	memset(testdataname, 0, sizeof(testdataname));
	
	if ( tptype == 0x01 ) {
		sprintf(testdataname, "%s%s",
				FTS_TEST_DEFAULT_SAVE_PATH,
				FTS_TEST_AUO_RESULT_FILE_NAME);
	}else if(tptype == 0x02){
		sprintf(testdataname, "%s%s",
				FTS_TEST_DEFAULT_SAVE_PATH,
				FTS_TEST_SHARP_RESULT_FILE_NAME);
	}else if(tptype == 0x03){
		sprintf(testdataname, "%s%s",
				FTS_TEST_DEFAULT_SAVE_PATH,
				FTS_TEST_INX_RESULT_FILE_NAME);
		}

	fts_test_save_test_data(testdataname, testdata, iTestDataLen);

	focaltech_test_main_exit();

	if (NULL != testdata) fts_free(testdata);
	if (NULL != printdata) fts_free(printdata);
	return (test_result ? 0 : -1);

TEST_ERR:
	if (NULL != testdata) fts_free(testdata);
	if (NULL != printdata) fts_free(printdata);
	return -1;
}

/************************************************************************
* Name: fts_test_entry_show
* Brief:  no
* Input:
* Output:
* Return:
***********************************************************************/
static int fts_test_entry_show(char *ini_file_name)
{
	/* place holder for future use */
	char cfgname[128] = {0};
	char testdataname[128] = {0};
	char *testdata = NULL;
	char *printdata = NULL;
	int iTestDataLen = 0;
	int ret = 0;
	int icycle = 0, i = 0;
	int print_index = 0;
	bool test_result = false;

	TP_TEST_LOGI("ini_file_name: %s\n", ini_file_name);

	/* for saving test data, notice the allocate size */
	TP_TEST_LOGI("Allocate memory, size: %d\n", FTS_TEST_BUFFER_SIZE);
	testdata = fts_malloc(FTS_TEST_BUFFER_SIZE);
	if (NULL == testdata) {
		TP_TEST_LOGE("ftestdata fts_malloc failed\n");
		fts_free(testdata);
		return -1;
	}

	printdata = fts_malloc(FTS_TEST_PRINT_SIZE);
	if (NULL == printdata) {
		TP_TEST_LOGE("printdata fts_malloc failed\n");
		fts_free(printdata);
		fts_free(testdata);
		return -1;
	}

	/* Initial i2c read & write function */
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);

	/* initial memories */
	ret = focaltech_test_main_init();
	if (ret < 0) {
		TP_TEST_LOGE("focaltech_test_main_init() error.\n");
		goto TEST_ERR;
	}

	/* read and parse ini file */
	memset(cfgname, 0, sizeof(cfgname));
	sprintf(cfgname, "%s", ini_file_name);
	TP_TEST_LOGI("ini_file_name = %s\n", cfgname);

	if (fts_test_get_testparam_from_ini(cfgname) < 0) {
		TP_TEST_LOGE("get testparam from ini failure\n");
		goto TEST_ERR;
	}

	/* start test */
	test_result = start_test_tp();

	if (true == test_result)
		TP_TEST_LOGI("tp test pass\n");
	else
		TP_TEST_LOGE("tp test failure\n");

	/* get the test data and save it */
	iTestDataLen = get_test_data(testdata);
	TP_TEST_LOGI("iTestDataLen: %d\n", iTestDataLen);

	icycle = 0;

	/* print test data */
	TP_TEST_LOGD("=========== test data:\n");
	for (i = 0; i < iTestDataLen; i++) {
		if (('\0' == testdata[i]) ||
			(icycle == FTS_TEST_PRINT_SIZE - 2) ||
			(i == iTestDataLen - 1))
		{
			if (icycle == 0) {
				print_index++;
			} else {
				memcpy(printdata, testdata + print_index, icycle);
				printdata[FTS_TEST_PRINT_SIZE - 1] = '\0';
				TP_TEST_LOGD("%s\n", printdata);
				print_index += icycle;
				icycle = 0;
			}
		} else {
			icycle++;
		}
	}
	TP_TEST_LOGD("======================\n");

	memset(testdataname, 0, sizeof(testdataname));

	if ( tptype == 0x01 ) {
		sprintf(testdataname, "%s%s",
				FTS_TEST_DEFAULT_SAVE_PATH,
				FTS_TEST_AUO_RESULT_FILE_NAME);
	}else if(tptype == 0x02){
		sprintf(testdataname, "%s%s",
				FTS_TEST_DEFAULT_SAVE_PATH,
				FTS_TEST_SHARP_RESULT_FILE_NAME);
	}else if(tptype == 0x03){
		sprintf(testdataname, "%s%s",
				FTS_TEST_DEFAULT_SAVE_PATH,
				FTS_TEST_INX_RESULT_FILE_NAME);
		}
	fts_test_save_test_data(testdataname, testdata, iTestDataLen);

	focaltech_test_main_exit();

	if (NULL != testdata) fts_free(testdata);
	if (NULL != printdata) fts_free(printdata);

	return (test_result ? 0 : -1);

TEST_ERR:
	if (NULL != testdata) fts_free(testdata);
	if (NULL != printdata) fts_free(printdata);

	return -1;
}

/************************************************************************
* Name: fts_test_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	char ini_file_name[128] = {0};
	int test_result = -1;
	struct i2c_client *client = fts_i2c_client;

	memset(ini_file_name, 0, sizeof(ini_file_name));
	if ( tptype == 0x01 ) {
		sprintf(ini_file_name, "%s%s",
		FTS_TEST_DEFAULT_READ_PATH,
		FTS_TEST_AUO_INI_FILE_NAME);
	}else if(tptype == 0x02){
		sprintf(ini_file_name, "%s%s",
		FTS_TEST_DEFAULT_READ_PATH,
		FTS_TEST_SHARP_INI_FILE_NAME);
	}else if(tptype == 0x03){
		sprintf(ini_file_name, "%s%s",
		FTS_TEST_DEFAULT_READ_PATH,
		FTS_TEST_INX_INI_FILE_NAME);
	}
	
	mutex_lock(&fts_input_dev->mutex);
	disable_irq(client->irq);

	test_result = fts_test_entry_show(ini_file_name);

	if (0 != test_result) {
		ret = snprintf(buf, PAGE_SIZE, "fail\n");
	} else {
		ret = snprintf(buf, PAGE_SIZE, "pass\n");
	}


	enable_irq(client->irq);
	mutex_unlock(&fts_input_dev->mutex);

	return ret;
}

/************************************************************************
* Name: fts_test_store
* Brief: load *.ini and start test
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char ini_file_name[128] = {0};
	struct i2c_client *client = fts_i2c_client;
	int test_result = -1;
	
	memset(ini_file_name, 0, sizeof(ini_file_name));
	sprintf(ini_file_name, "%s", buf);
	ini_file_name[count - 1] = '\0';

	TP_TEST_LOGI("ini file name: %s\n", ini_file_name);

	mutex_lock(&fts_input_dev->mutex);

	disable_irq(client->irq);
	
	test_result =fts_test_entry(ini_file_name);

	if (0 != test_result) {
		TP_TEST_LOGI("TP selftest fail");
	} else {
		TP_TEST_LOGI("TP selftest pass");
	}

	enable_irq(client->irq);

	mutex_unlock(&fts_input_dev->mutex);

	return count;
}

/*
 * example:
 * 1. echo "***.ini" > fts_test (use specfic ini file)
 * 2. cat fts_test (use default ini file)
 */
static DEVICE_ATTR(fts_test, S_IRUGO | S_IWUSR, fts_test_show, fts_test_store);

/* add your attr in here*/
static struct attribute *fts_test_attributes[] = {
	&dev_attr_fts_test.attr,
	NULL
};

static struct attribute_group fts_test_attribute_group = {
	.attrs = fts_test_attributes
};

int fts_test_init(struct i2c_client *client)
{
	int err = 0;

	TP_TEST_LOGI("start\n");
	TP_TEST_LOGI("%s\n", FOCALTECH_TEST_INFO); // show version

	err = sysfs_create_group(&client->dev.kobj, &fts_test_attribute_group);
	if (0 != err) {
		TP_TEST_LOGE("ERROR: sysfs_create_group() failed\n");
		sysfs_remove_group(&client->dev.kobj, &fts_test_attribute_group);
		return -EIO;
	} else {
		TP_TEST_LOGI("sysfs_create_group() succeeded\n");
	}

	return err;
}

int fts_test_exit(struct i2c_client *client)
{
	TP_TEST_LOGI("start\n");

	sysfs_remove_group(&client->dev.kobj, &fts_test_attribute_group);

	return 0;
}

