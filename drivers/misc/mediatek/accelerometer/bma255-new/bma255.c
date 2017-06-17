/* BMA255 motion sensor driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 *
 * VERSION: V1.5
 * HISTORY: V1.0 --- Driver creation
 *          V1.1 --- Add share I2C address function
 *          V1.2 --- Fix the bug that sometimes sensor is stuck after system resume.
 *          V1.3 --- Add FIFO interfaces.
 *          V1.4 --- Use basic i2c function to read fifo data instead of i2c DMA mode.
 *          V1.5 --- Add compensated value performed by MTK acceleration calibration process.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <accel.h>
#include <hwmsensor.h>

#include <cust_acc.h>
#include "bma255.h"

/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#define SIG_MOTION_INT 1
#define RETRY 10

//#define BMA255_REG_POWER_CTL	0x11
//#define BMA255_MODE_CTRL_REG	0x11
#define MODE_NORMAL			(0 << 5)
#define MODE_DEEP_SUSPEND	(1 << 5)
#define MODE_LOW_POWER		(2 << 5)
#define MODE_SUSPEND			(4 << 5)
#define BMA255_INT_EN_0			0x16
//#define ALL_AXIS_SLOPE		(7 << 0)
#define XY_AXIS_SLOPE			(3 << 0)
#define BMA255_REG_INT_MAP_0		0x19
#define SLOPE_IRQ_INT1			(1 << 2)
#define BMA255_REG_INT_OUT_CTRL	0x20
#define INT_PUSH_PULL			(1 << 0)	// INT1 push-pull & active high
#define BMA255_REG_SLOPE_THRES	0x28
#define SLOPE_THRES			0x1e		// 7.81mg * 30 = 234.3mg
#define BMA255_REG_INT_STATUS		0x09

extern unsigned int mt_gpio_to_irq(unsigned int gpio);
#define IRQ_GPIO_NUM 4	// GPIO4 = EINT_A = BMA253's INT1; default active High , suspend Low
static int bma255_set_mode(struct i2c_client *client, unsigned char mode);
static int g_iIRQCnt;	// for factory
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */

/* [BY57] S-BUG#811 Grace_Chang G-sensor read miscTA for FRU */
int g_iMiscTaXyz[4] = {-1, -1, -1, -1};
/* [BY57] E-BUG#811 Grace_Chang G-sensor read miscTA for FRU */

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_BMA255 255
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_BMA255_LOWPASS   /*apply low pass filter on output*/       
#define SW_CALIBRATION
#define CONFIG_I2C_BASIC_FUNCTION
//tad3sgh add ++
#define BMM050_DEFAULT_DELAY	100
#define CALIBRATION_DATA_SIZE	12

/*----------------------------------------------------------------------------*/
/*
* Enable the driver to block e-compass daemon on suspend
*/
#define BMC050_BLOCK_DAEMON_ON_SUSPEND
#undef	BMC050_BLOCK_DAEMON_ON_SUSPEND
/*
* Enable gyroscope feature with BMC050
*/
#define BMC050_M4G	
#undef BMC050_M4G
/*
* Enable rotation vecter feature with BMC050
*/
#define BMC050_VRV	
#undef BMC050_VRV	

/*
* Enable virtual linear accelerometer feature with BMC050
*/
#define BMC050_VLA	
#undef BMC050_VLA

/*
* Enable virtual gravity feature with BMC050
*/
#define BMC050_VG	
#undef BMC050_VG

#ifdef BMC050_M4G
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_GFLAG			_IOR(MSENSOR, 0x30, short)
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_GDELAY			_IOR(MSENSOR, 0x31, int)
#endif //BMC050_M4G
#ifdef BMC050_VRV
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VRVFLAG			_IOR(MSENSOR, 0x32, short)
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VRVDELAY			_IOR(MSENSOR, 0x33, int)
#endif //BMC050_VRV
#ifdef BMC050_VLA
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VLAFLAG			_IOR(MSENSOR, 0x34, short)
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VLADELAY			_IOR(MSENSOR, 0x35, int)
#endif //BMC050_VLA
#ifdef BMC050_VG
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VGFLAG			_IOR(MSENSOR, 0x36, short)
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define ECOMPASS_IOC_GET_VGDELAY			_IOR(MSENSOR, 0x37, int)
#endif //BMC050_VG
/* !!! add a new definition in linux/sensors_io.h if possible !!! */
#define BMM_IOC_GET_EVENT_FLAG	ECOMPASS_IOC_GET_OPEN_STATUS
//add for non-block
#define BMM_IOC_GET_NONBLOCK_EVENT_FLAG _IOR(MSENSOR, 0x38, int)

// calibration msensor and orientation data
static int sensor_data[CALIBRATION_DATA_SIZE];
#if defined(BMC050_M4G) || defined(BMC050_VRV)
static int m4g_data[CALIBRATION_DATA_SIZE];
#endif //BMC050_M4G || BMC050_VRV
#if defined(BMC050_VLA)
static int vla_data[CALIBRATION_DATA_SIZE];
#endif //BMC050_VLA

#if defined(BMC050_VG)
static int vg_data[CALIBRATION_DATA_SIZE];
#endif //BMC050_VG

static struct mutex sensor_data_mutex;
static DECLARE_WAIT_QUEUE_HEAD(uplink_event_flag_wq);

static int bmm050d_delay = BMM050_DEFAULT_DELAY;
#ifdef BMC050_M4G
static int m4g_delay = BMM050_DEFAULT_DELAY;
#endif //BMC050_M4G
#ifdef BMC050_VRV
static int vrv_delay = BMM050_DEFAULT_DELAY;
#endif //BMC050_VRV
#ifdef BMC050_VLA
static int vla_delay = BMM050_DEFAULT_DELAY;
#endif //BMC050_VRV

#ifdef BMC050_VG
static int vg_delay = BMM050_DEFAULT_DELAY;
#endif //BMC050_VG

static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);
#ifdef BMC050_M4G
static atomic_t g_flag = ATOMIC_INIT(0);
#endif //BMC050_M4G
#ifdef BMC050_VRV
static atomic_t vrv_flag = ATOMIC_INIT(0);
#endif //BMC050_VRV
#ifdef BMC050_VLA
static atomic_t vla_flag = ATOMIC_INIT(0);
#endif //BMC050_VLA
#ifdef BMC050_VG
static atomic_t vg_flag = ATOMIC_INIT(0);
#endif //BMC050_VG

#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
static atomic_t driver_suspend_flag = ATOMIC_INIT(0);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND

static struct mutex uplink_event_flag_mutex;
/* uplink event flag */
static volatile u32 uplink_event_flag;
/* uplink event flag bitmap */
enum {
	/* active */
	BMMDRV_ULEVT_FLAG_O_ACTIVE = 0x0001,
	BMMDRV_ULEVT_FLAG_M_ACTIVE = 0x0002,
	BMMDRV_ULEVT_FLAG_G_ACTIVE = 0x0004,
	BMMDRV_ULEVT_FLAG_VRV_ACTIVE = 0x0008,/* Virtual Rotation Vector */
	BMMDRV_ULEVT_FLAG_FLIP_ACTIVE = 0x0010,
	BMMDRV_ULEVT_FLAG_VLA_ACTIVE = 0x0020,/* Virtual Linear Accelerometer */
	BMMDRV_ULEVT_FLAG_VG_ACTIVE = 0x0040,/* Virtual Gravity */
	
	/* delay */
	BMMDRV_ULEVT_FLAG_O_DELAY = 0x0100,
	BMMDRV_ULEVT_FLAG_M_DELAY = 0x0200,
	BMMDRV_ULEVT_FLAG_G_DELAY = 0x0400,
	BMMDRV_ULEVT_FLAG_VRV_DELAY = 0x0800,
	BMMDRV_ULEVT_FLAG_FLIP_DELAY = 0x1000,
	BMMDRV_ULEVT_FLAG_VLA_DELAY = 0x2000,
	BMMDRV_ULEVT_FLAG_VG_DELAY = 0x4000,

	/* all */
	BMMDRV_ULEVT_FLAG_ALL = 0xffff
};

//tad3sgh add --
#define MAX_FIFO_F_LEVEL 32
#define MAX_FIFO_F_BYTES 6

/*----------------------------------------------------------------------------*/
#define BMA255_AXIS_X          0
#define BMA255_AXIS_Y          1
#define BMA255_AXIS_Z          2
#define BMA255_AXES_NUM        3
#define BMA255_DATA_LEN        6
#define BMA255_DEV_NAME        "BMA255"

#define BMA255_MODE_NORMAL      0
#define BMA255_MODE_LOWPOWER    1
#define BMA255_MODE_SUSPEND     2
//for bma255 chip.
#define BMA255_ACC_X_LSB__POS           4
#define BMA255_ACC_X_LSB__LEN           4
#define BMA255_ACC_X_LSB__MSK           0xC0//0xF0
//#define BMA255_ACC_X_LSB__REG           BMA255_X_AXIS_LSB_REG

#define BMA255_ACC_X_MSB__POS           0
#define BMA255_ACC_X_MSB__LEN           8
#define BMA255_ACC_X_MSB__MSK           0xFF
//#define BMA255_ACC_X_MSB__REG           BMA255_X_AXIS_MSB_REG

#define BMA255_ACC_Y_LSB__POS           4
#define BMA255_ACC_Y_LSB__LEN           4
#define BMA255_ACC_Y_LSB__MSK           0xC0//0xF0
//#define BMA255_ACC_Y_LSB__REG           BMA255_Y_AXIS_LSB_REG

#define BMA255_ACC_Y_MSB__POS           0
#define BMA255_ACC_Y_MSB__LEN           8
#define BMA255_ACC_Y_MSB__MSK           0xFF
//#define BMA255_ACC_Y_MSB__REG           BMA255_Y_AXIS_MSB_REG

#define BMA255_ACC_Z_LSB__POS           4
#define BMA255_ACC_Z_LSB__LEN           4
#define BMA255_ACC_Z_LSB__MSK           0xC0//0xF0
//#define BMA255_ACC_Z_LSB__REG           BMA255_Z_AXIS_LSB_REG

#define BMA255_ACC_Z_MSB__POS           0
#define BMA255_ACC_Z_MSB__LEN           8
#define BMA255_ACC_Z_MSB__MSK           0xFF
//#define BMA255_ACC_Z_MSB__REG           BMA255_Z_AXIS_MSB_REG

#define BMA255_EN_LOW_POWER__POS          6
#define BMA255_EN_LOW_POWER__LEN          1
#define BMA255_EN_LOW_POWER__MSK          0x40
#define BMA255_EN_LOW_POWER__REG          BMA255_REG_POWER_CTL

#define BMA255_EN_SUSPEND__POS            7
#define BMA255_EN_SUSPEND__LEN            1
#define BMA255_EN_SUSPEND__MSK            0x80
#define BMA255_EN_SUSPEND__REG            BMA255_REG_POWER_CTL

#define BMA2X2_RESET_REG		0x14
#define BMA2X2_EN_SOFT_RESET__POS         0
#define BMA2X2_EN_SOFT_RESET__LEN         8
#define BMA2X2_EN_SOFT_RESET__MSK         0xFF
#define BMA2X2_EN_SOFT_RESET__REG         BMA2X2_RESET_REG

#define BMA2X2_EN_SOFT_RESET_VALUE        0xB6


/* fifo mode*/
#define BMA255_FIFO_MODE__POS                 6
#define BMA255_FIFO_MODE__LEN                 2
#define BMA255_FIFO_MODE__MSK                 0xC0
#define BMA255_FIFO_MODE__REG                 BMA255_FIFO_MODE_REG

#define BMA255_FIFO_FRAME_COUNTER_S__POS             0
#define BMA255_FIFO_FRAME_COUNTER_S__LEN             7
#define BMA255_FIFO_FRAME_COUNTER_S__MSK             0x7F
#define BMA255_FIFO_FRAME_COUNTER_S__REG             BMA255_STATUS_FIFO_REG

#define BMA255_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMA255_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/*----------------------------------------------------------------------------*/
/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
// add input device dedicated for shake event
static const char *shake_idev_name = "acc_shake_event";
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id bma255_i2c_id[] = {{BMA255_DEV_NAME,0},{}};

/*----------------------------------------------------------------------------*/
static int bma255_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int bma255_i2c_remove(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
typedef enum {
    BMA_TRC_FILTER  = 0x01,
    BMA_TRC_RAWDATA = 0x02,
    BMA_TRC_IOCTL   = 0x04,
    BMA_TRC_CALI	= 0X08,
    BMA_TRC_INFO	= 0X10,
} BMA_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][BMA255_AXES_NUM];
    int sum[BMA255_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct bma255_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    /* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
    struct input_dev *shake_idev;
    /* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[BMA255_AXES_NUM+1];
    struct mutex lock;

    /*data*/
    s8                      offset[BMA255_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[BMA255_AXES_NUM+1];
    u8			    fifo_count;

#if defined(CONFIG_BMA255_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
//static int bma255_suspend(struct i2c_client *client, pm_message_t msg);
//static int bma255_resume(struct i2c_client *client);
static int bma255_suspend(struct device *dev);
static int bma255_resume(struct device *dev);
#endif
/*----------------------------------------------------------------------------*/

static const struct dev_pm_ops bma255_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bma255_suspend, bma255_resume)
};

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,gsensor" },
	{},
};
#endif

static struct i2c_driver bma255_i2c_driver = {
    .driver = {
        .name           = BMA255_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = gsensor_of_match,
#endif
	.pm = &bma255_pm_ops,
    },
	.probe      		= bma255_i2c_probe,
	.remove    			= bma255_i2c_remove,
//#if !defined(CONFIG_HAS_EARLYSUSPEND)
//    .suspend            = bma255_suspend,
//    .resume             = bma255_resume,
//#endif
	.id_table = bma255_i2c_id,
};

/*----------------------------------------------------------------------------*/
//static struct i2c_client *bma255_i2c_client = NULL;
static struct i2c_client *bma255_i2c_client;
static struct acc_init_info bma255_init_info;
//static struct bma255_i2c_data *obj_i2c_data = NULL;
static struct bma255_i2c_data *obj_i2c_data;
static bool sensor_power = false;
static struct GSENSOR_VECTOR3D gsensor_gain;
//static char selftestRes[8]= {0}; 
//static int bma255_init_flag =-1; // 0<==>OK -1 <==> fail
//static int bma255_softreset_result = -1;	// 0<==>OK -1 <==> fail
static int bma255_init_flag; // 0<==>OK -1 <==> fail
static int bma255_softreset_result;	// 0<==>OK -1 <==> fail

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(GSE_TAG fmt, ##args)


#define COMPATIABLE_NAME "mediatek,bma2xx_new"

struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

/*----------------------------------------------------------------------------*/
static struct data_resolution bma255_data_resolution[1] = {
 /* combination by {FULL_RES,RANGE}*/
 //for bma255 12bit
    {{ 1, 95}, 512},   // dataformat +/-4g  in 12-bit resolution;  { 1, 95} = 1.95 = (2*4*1000)/(2^12);  512 = (2^12)/(2*4)
};
/*----------------------------------------------------------------------------*/
static struct data_resolution bma255_offset_resolution = {{ 1, 95}, 512};

static struct mutex bma255_i2c_mutex;	// Grace add for i2c protect

/* I2C operation functions */
static int bma_i2c_read_block(struct i2c_client *client,
			u8 addr, u8 *data, u8 len)
{
#ifdef CONFIG_I2C_BASIC_FUNCTION
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,		.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = len,		.buf = data,
		}
	};
	int err;

	mutex_lock(&bma255_i2c_mutex);	// Grace add for i2c protect
	if (!client) {
		mutex_unlock(&bma255_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&bma255_i2c_mutex);
		return -EINVAL;
	}

	//err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs) );
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;/*no error*/
	}

	mutex_unlock(&bma255_i2c_mutex);
	return err;
#else
	int err = 0;
	err = i2c_smbus_read_i2c_block_data(client, addr, len, data);
	if (err < 0)
		return -1;
	return 0;
#endif
}
#define I2C_BUFFER_SIZE 256
static int bma_i2c_write_block(struct i2c_client *client, u8 addr,
			u8 *data, u8 len)
{
#ifdef CONFIG_I2C_BASIC_FUNCTION
	/*
	*because address also occupies one byte,
	*the maximum length for write is 7 bytes
	*/
	int err, idx = 0, num = 0;
	char buf[32];

	mutex_lock(&bma255_i2c_mutex);	// Grace add for i2c protect
	if (!client) {
		mutex_unlock(&bma255_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&bma255_i2c_mutex);
		return -EINVAL;
	}

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		GSE_ERR("send command error!!\n");
		mutex_unlock(&bma255_i2c_mutex);
		return -EFAULT;
	} else {
		err = 0;/*no error*/
	}
	mutex_unlock(&bma255_i2c_mutex);
	return err;
#else
	int err = 0;
	err = i2c_smbus_write_i2c_block_data(client, addr, len, data);
	if (err < 0)
		return -1;
	return 0;
#endif
}


/*--------------------BMA255 power control function----------------------------------*/
/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#if SIG_MOTION_INT
static int BMA255_power(struct acc_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;
	power_on = on;
	return -ENOENT;
}
#else
static void BMA255_power(struct acc_hw *hw, unsigned int on) 
{
}
#endif	// end of SIG_MOTION_INT
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int BMA255_SetDataResolution(struct bma255_i2c_data *obj)
{

/*set g sensor dataresolution here*/

/*BMA255 only can set to 10-bit dataresolution, so do nothing in bma255 driver here*/

/*end of set dataresolution*/
 
 /*we set measure range from -2g to +2g in BMA255_SetDataFormat(client, BMA255_RANGE_2G), 
 *                                                   and set 10-bit dataresolution BMA255_SetDataResolution()
*/
                                                    
 /*so bma255_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here*/  

 	obj->reso = &bma255_data_resolution[0];
	return 0;
	
/*if you changed the measure range, for example call: BMA255_SetDataFormat(client, BMA255_RANGE_4G), 
* you must set the right value to bma255_data_resolution
*/

}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadData(struct i2c_client *client, s16 data[BMA255_AXES_NUM])
{
#ifdef CONFIG_BMA255_LOWPASS
	struct bma255_i2c_data *priv = i2c_get_clientdata(client);
#endif
	u8 addr = BMA255_REG_DATAXLOW;
	u8 buf[BMA255_DATA_LEN] = {0};
	int err = 0;

	if(client == NULL)
	{
		err = -EINVAL;
	}
	else if( (err = bma_i2c_read_block(client, addr, buf, BMA255_DATA_LEN)) )
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		/* Convert sensor raw data to 16-bit integer */
		data[BMA255_AXIS_X] = BMA255_GET_BITSLICE(buf[0], BMA255_ACC_X_LSB)
			|(BMA255_GET_BITSLICE(buf[1],
						BMA255_ACC_X_MSB)<<BMA255_ACC_X_LSB__LEN);
		data[BMA255_AXIS_X] = data[BMA255_AXIS_X] << (sizeof(short)*8-(BMA255_ACC_X_LSB__LEN
					+ BMA255_ACC_X_MSB__LEN));
		data[BMA255_AXIS_X] = data[BMA255_AXIS_X] >> (sizeof(short)*8-(BMA255_ACC_X_LSB__LEN
					+ BMA255_ACC_X_MSB__LEN));
		data[BMA255_AXIS_Y] = BMA255_GET_BITSLICE(buf[2], BMA255_ACC_Y_LSB)
			| (BMA255_GET_BITSLICE(buf[3],
						BMA255_ACC_Y_MSB)<<BMA255_ACC_Y_LSB__LEN);
		data[BMA255_AXIS_Y] = data[BMA255_AXIS_Y] << (sizeof(short)*8-(BMA255_ACC_Y_LSB__LEN
					+ BMA255_ACC_Y_MSB__LEN));
		data[BMA255_AXIS_Y] = data[BMA255_AXIS_Y] >> (sizeof(short)*8-(BMA255_ACC_Y_LSB__LEN
					+ BMA255_ACC_Y_MSB__LEN));
		data[BMA255_AXIS_Z] = BMA255_GET_BITSLICE(buf[4], BMA255_ACC_Z_LSB)
			| (BMA255_GET_BITSLICE(buf[5],
						BMA255_ACC_Z_MSB)<<BMA255_ACC_Z_LSB__LEN);
		data[BMA255_AXIS_Z] = data[BMA255_AXIS_Z] << (sizeof(short)*8-(BMA255_ACC_Z_LSB__LEN
					+ BMA255_ACC_Z_MSB__LEN));
		data[BMA255_AXIS_Z] = data[BMA255_AXIS_Z] >> (sizeof(short)*8-(BMA255_ACC_Z_LSB__LEN
					+ BMA255_ACC_Z_MSB__LEN));

#ifdef CONFIG_BMA255_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][BMA255_AXIS_X] = data[BMA255_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA255_AXIS_Y] = data[BMA255_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA255_AXIS_Z] = data[BMA255_AXIS_Z];
					priv->fir.sum[BMA255_AXIS_X] += data[BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] += data[BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] += data[BMA255_AXIS_Z];
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][BMA255_AXIS_X], priv->fir.raw[priv->fir.num][BMA255_AXIS_Y], priv->fir.raw[priv->fir.num][BMA255_AXIS_Z],
							priv->fir.sum[BMA255_AXIS_X], priv->fir.sum[BMA255_AXIS_Y], priv->fir.sum[BMA255_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA255_AXIS_X] -= priv->fir.raw[idx][BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] -= priv->fir.raw[idx][BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] -= priv->fir.raw[idx][BMA255_AXIS_Z];
					priv->fir.raw[idx][BMA255_AXIS_X] = data[BMA255_AXIS_X];
					priv->fir.raw[idx][BMA255_AXIS_Y] = data[BMA255_AXIS_Y];
					priv->fir.raw[idx][BMA255_AXIS_Z] = data[BMA255_AXIS_Z];
					priv->fir.sum[BMA255_AXIS_X] += data[BMA255_AXIS_X];
					priv->fir.sum[BMA255_AXIS_Y] += data[BMA255_AXIS_Y];
					priv->fir.sum[BMA255_AXIS_Z] += data[BMA255_AXIS_Z];
					priv->fir.idx++;
					data[BMA255_AXIS_X] = priv->fir.sum[BMA255_AXIS_X]/firlen;
					data[BMA255_AXIS_Y] = priv->fir.sum[BMA255_AXIS_Y]/firlen;
					data[BMA255_AXIS_Z] = priv->fir.sum[BMA255_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][BMA255_AXIS_X], priv->fir.raw[idx][BMA255_AXIS_Y], priv->fir.raw[idx][BMA255_AXIS_Z],
						priv->fir.sum[BMA255_AXIS_X], priv->fir.sum[BMA255_AXIS_Y], priv->fir.sum[BMA255_AXIS_Z],
						data[BMA255_AXIS_X], data[BMA255_AXIS_Y], data[BMA255_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadOffset(struct i2c_client *client, s8 ofs[BMA255_AXES_NUM])
{    
	int err=0;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#else
	if( (err = bma_i2c_read_block(client, BMA255_REG_OFSX, ofs, BMA255_AXES_NUM)) )
	{
		GSE_ERR("error: %d\n", err);
	}
#endif
	//printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);
	
	return err;    
}
/*----------------------------------------------------------------------------*/
static int BMA255_ResetCalibration(struct i2c_client *client)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	//u8 ofs[4]={0,0,0,0};
	int err=0;
	
	#ifdef SW_CALIBRATION
		
	#else
	u8 ofs[4]={0,0,0,0};

		if( (err = bma_i2c_write_block(client, BMA255_REG_OFSX, ofs, 4)) )
		{
			GSE_ERR("error: %d\n", err);
		}
	#endif

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadCalibration(struct i2c_client *client, int dat[BMA255_AXES_NUM])
{
    struct bma255_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;
    int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
	    if ((err = BMA255_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    	}    
    	mul = obj->reso->sensitivity/bma255_offset_resolution.sensitivity;
	#endif

    dat[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*(obj->offset[BMA255_AXIS_X]*mul + obj->cali_sw[BMA255_AXIS_X]);
    dat[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*(obj->offset[BMA255_AXIS_Y]*mul + obj->cali_sw[BMA255_AXIS_Y]);
    dat[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*(obj->offset[BMA255_AXIS_Z]*mul + obj->cali_sw[BMA255_AXIS_Z]);                        
                                       
    return err;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadCalibrationEx(struct i2c_client *client, int act[BMA255_AXES_NUM], int raw[BMA255_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	//int err;
	int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
	int err;

		if( (err = BMA255_ReadOffset(client, obj->offset)) )
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/bma255_offset_resolution.sensitivity;
	#endif
	
	raw[BMA255_AXIS_X] = obj->offset[BMA255_AXIS_X]*mul + obj->cali_sw[BMA255_AXIS_X];
	raw[BMA255_AXIS_Y] = obj->offset[BMA255_AXIS_Y]*mul + obj->cali_sw[BMA255_AXIS_Y];
	raw[BMA255_AXIS_Z] = obj->offset[BMA255_AXIS_Z]*mul + obj->cali_sw[BMA255_AXIS_Z];

	act[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*raw[BMA255_AXIS_X];
	act[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*raw[BMA255_AXIS_Y];
	act[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*raw[BMA255_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA255_WriteCalibration(struct i2c_client *client, int dat[BMA255_AXES_NUM])
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[BMA255_AXES_NUM], raw[BMA255_AXES_NUM];
#ifndef SW_CALIBRATION
	int lsb = bma255_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
#endif

	if( (err = BMA255_ReadCalibrationEx(client, cali, raw)) )	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[BMA255_AXIS_X], raw[BMA255_AXIS_Y], raw[BMA255_AXIS_Z],
		obj->offset[BMA255_AXIS_X], obj->offset[BMA255_AXIS_Y], obj->offset[BMA255_AXIS_Z],
		obj->cali_sw[BMA255_AXIS_X], obj->cali_sw[BMA255_AXIS_Y], obj->cali_sw[BMA255_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[BMA255_AXIS_X] += dat[BMA255_AXIS_X];
	cali[BMA255_AXIS_Y] += dat[BMA255_AXIS_Y];
	cali[BMA255_AXIS_Z] += dat[BMA255_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[BMA255_AXIS_X], dat[BMA255_AXIS_Y], dat[BMA255_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[BMA255_AXIS_X] = obj->cvt.sign[BMA255_AXIS_X]*(cali[obj->cvt.map[BMA255_AXIS_X]]);
	obj->cali_sw[BMA255_AXIS_Y] = obj->cvt.sign[BMA255_AXIS_Y]*(cali[obj->cvt.map[BMA255_AXIS_Y]]);
	obj->cali_sw[BMA255_AXIS_Z] = obj->cvt.sign[BMA255_AXIS_Z]*(cali[obj->cvt.map[BMA255_AXIS_Z]]);	
#else
	obj->offset[BMA255_AXIS_X] = (s8)(obj->cvt.sign[BMA255_AXIS_X]*(cali[obj->cvt.map[BMA255_AXIS_X]])/(divisor));
	obj->offset[BMA255_AXIS_Y] = (s8)(obj->cvt.sign[BMA255_AXIS_Y]*(cali[obj->cvt.map[BMA255_AXIS_Y]])/(divisor));
	obj->offset[BMA255_AXIS_Z] = (s8)(obj->cvt.sign[BMA255_AXIS_Z]*(cali[obj->cvt.map[BMA255_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMA255_AXIS_X] = obj->cvt.sign[BMA255_AXIS_X]*(cali[obj->cvt.map[BMA255_AXIS_X]])%(divisor);
	obj->cali_sw[BMA255_AXIS_Y] = obj->cvt.sign[BMA255_AXIS_Y]*(cali[obj->cvt.map[BMA255_AXIS_Y]])%(divisor);
	obj->cali_sw[BMA255_AXIS_Z] = obj->cvt.sign[BMA255_AXIS_Z]*(cali[obj->cvt.map[BMA255_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[BMA255_AXIS_X]*divisor + obj->cali_sw[BMA255_AXIS_X], 
		obj->offset[BMA255_AXIS_Y]*divisor + obj->cali_sw[BMA255_AXIS_Y], 
		obj->offset[BMA255_AXIS_Z]*divisor + obj->cali_sw[BMA255_AXIS_Z], 
		obj->offset[BMA255_AXIS_X], obj->offset[BMA255_AXIS_Y], obj->offset[BMA255_AXIS_Z],
		obj->cali_sw[BMA255_AXIS_X], obj->cali_sw[BMA255_AXIS_Y], obj->cali_sw[BMA255_AXIS_Z]);

	if( (err = bma_i2c_write_block(obj->client, BMA255_REG_OFSX, obj->offset, BMA255_AXES_NUM)) )
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA255_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);    

	res = bma_i2c_read_block(client, BMA255_REG_DEVID, databuf, 0x01);
	res = bma_i2c_read_block(client, BMA255_REG_DEVID, databuf, 0x01);
	if(res < 0)
		goto exit_BMA255_CheckDeviceID;

	if(databuf[0]!=BMA255_FIXED_DEVID)
	{
		printk("BMA255_CheckDeviceID %d failt!\n ", databuf[0]);
		return BMA255_ERR_IDENTIFICATION;
	}
	else
	{
		printk("BMA255_CheckDeviceID %d pass!\n ", databuf[0]);
	}

	exit_BMA255_CheckDeviceID:
	if (res < 0)
	{
		return BMA255_ERR_I2C;
	}
	
	return BMA255_SUCCESS;
}
/*----------------------------------------------------------------------------*/
/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#if SIG_MOTION_INT
struct motion_config {
	u8 mode;
	u8 int_en;
	u8 intmap;
	u8 int_out_ctrl;
	u8 slope_thres;
	u8 int_status;
};
#if 0
static int read_retry(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int rc, i;

	for (i = 0; i < RETRY; i++) {
		rc = bma_i2c_read_block(client, addr, data, len);
		if (!rc)
			break;
	}
	if (rc)
		dev_err(&client->dev, "%s: reg %02x, rc = %d", __func__, addr, rc);
	return rc;
}

static int write_retry(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int rc, i;

	for (i = 0; i < RETRY; i++) {
		rc = bma_i2c_write_block(client, addr, data, len);
		if (!rc)
			break;
	}
	if (rc)
		dev_err(&client->dev, "%s: reg %02x, val %02x, rc = %d", __func__, addr, data, rc);
	return rc;
}
#endif
static int bma255_setup_any_motion_detection(struct bma255_i2c_data *obj, bool enable)
{
	int rc;
	struct motion_config cfg;

	dev_info(&obj->client->dev, "%s, enable %d\n", __func__, enable);

	if (enable) {
		//cfg.int_en = ALL_AXIS_SLOPE;
		cfg.int_en = XY_AXIS_SLOPE;
	} else {
		cfg.int_en = 0;
	}
	cfg.intmap = SLOPE_IRQ_INT1;
	cfg.int_out_ctrl= INT_PUSH_PULL;
	cfg.slope_thres = SLOPE_THRES;

	rc = bma_i2c_write_block(obj->client, BMA255_INT_EN_0, &cfg.int_en, 1);
	if (rc)
		goto err;
	mdelay(1);
	rc = bma_i2c_write_block(obj->client, BMA255_REG_INT_MAP_0, &cfg.intmap, 1);
	if (rc)
		goto err;
	mdelay(1);
	rc = bma_i2c_write_block(obj->client, BMA255_REG_INT_OUT_CTRL, &cfg.int_out_ctrl, 1);
	if (rc)
		goto err;
	mdelay(1);
	rc = bma_i2c_write_block(obj->client, BMA255_REG_SLOPE_THRES, &cfg.slope_thres, 1);
	if (rc)
		goto err;
	mdelay(1);

	// To check register values
	rc = bma_i2c_read_block(obj->client, BMA255_MODE_CTRL_REG, &cfg.mode, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_INT_EN_0, &cfg.int_en, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_REG_INT_MAP_0, &cfg.intmap, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_REG_INT_OUT_CTRL, &cfg.int_out_ctrl, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_REG_SLOPE_THRES, &cfg.slope_thres, 1);
	if (rc)
		goto err;
	dev_info(&obj->client->dev,
		"%s, MODE %02x  INTEN %02x  INTMAP %02x  INTCTRL %02x  THRES %02x \n",
		__func__, cfg.mode, cfg.int_en, cfg.intmap, cfg.int_out_ctrl, cfg.slope_thres);
err:
	if (rc)
		dev_err(&obj->client->dev, "%s = %d\n", __func__, rc);
	return rc;
}

enum acc_client {
	REQ_ACC_DATA = 1 << 0,
	REQ_SHAKE_SENSOR = 1 << 1,
};

struct bma255_op_mode {
	int req;
	int saved_req;
	struct mutex lock;
};

static struct bma255_op_mode bma255_op_mode;

static int vote_op_mode_locked(struct i2c_client *client, bool enable,
	enum acc_client request)
{
	int req;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int rc = 0;

	req = bma255_op_mode.req;

	if (enable)
		bma255_op_mode.req |= request;
	else
		bma255_op_mode.req &= ~request;

	dev_info(&client->dev, "%s: request 0x%02x, was 0x%02x\n", __func__, bma255_op_mode.req, req);

	if (req != bma255_op_mode.req) {
		enable = !!bma255_op_mode.req;

		disable_irq(mt_gpio_to_irq(IRQ_GPIO_NUM));
		/*rc = write_retry(obj->client, BMA255_MODE_CTRL_REG, 0);
		if (rc) {
			dev_err(&client->dev, "%s: unable set mode 0\n",
					__func__);
			goto err;
		}*/
		bma255_set_mode(obj->client, BMA255_MODE_NORMAL);

		if (!(req & REQ_SHAKE_SENSOR) &&
				(bma255_op_mode.req & REQ_SHAKE_SENSOR)) {
			bma255_setup_any_motion_detection(obj, true);
		} else if ((req & REQ_SHAKE_SENSOR) &&
				!(bma255_op_mode.req & REQ_SHAKE_SENSOR)) {
			bma255_setup_any_motion_detection(obj, false);
		}

		if (enable) {
			//u8 mode = MODE_ACTIVE | ((req & REQ_SHAKE_SENSOR) ? INT_PUSH_PULL : 0);
			/*u8 mode = MODE_NORMAL;
			rc = write_retry(obj->client, BMA255_MODE_CTRL_REG, mode);
			if (rc) {
				dev_err(&client->dev, "%s: unable set mode 0x%02x\n", __func__, mode);
				goto err;
			}
			dev_dbg(&obj->client->dev, "%s mode 0x%02x\n", __func__, mode);*/
			//rc = STK831X_SetVD(obj->client);
			dev_info(&obj->client->dev, "%s: set BMA255_MODE_NORMAL\n", __func__);
			bma255_set_mode(obj->client, BMA255_MODE_NORMAL);
		}
		else {
			dev_info(&obj->client->dev, "%s: set BMA255_MODE_SUSPEND\n", __func__);
			bma255_set_mode(obj->client, BMA255_MODE_SUSPEND);
		}

		enable_irq(mt_gpio_to_irq(IRQ_GPIO_NUM));
		sensor_power = enable;
	}
//err:
	return rc;
}

static int vote_op_mode(struct i2c_client *client, bool enable,
	enum acc_client request)
{
	int rc;
	struct bma255_i2c_data *obj = obj_i2c_data;

	dev_info(&obj->client->dev, "%s: enable=%d, request=%d\n", __func__, enable, request);
	mutex_lock(&bma255_op_mode.lock);
	rc = vote_op_mode_locked(client, enable, request);
	mutex_unlock(&bma255_op_mode.lock);
	return rc;
}

static int op_mode_suspend(struct i2c_client *client, bool suspend)
{
	int rc;
	struct bma255_i2c_data *obj = obj_i2c_data;

	dev_info(&obj->client->dev, "%s: suspend=%d\n", __func__, suspend);
	mutex_lock(&bma255_op_mode.lock);
	if (suspend) {
		bma255_op_mode.saved_req = bma255_op_mode.req;
		rc = vote_op_mode_locked(client, false, REQ_ACC_DATA);
	} else {
		rc = vote_op_mode_locked(client, true, bma255_op_mode.saved_req);
	}
	mutex_unlock(&bma255_op_mode.lock);
	return rc;
}

static int BMA255_SetPowerMode(struct i2c_client *client, bool enable)
{
	return vote_op_mode(client, enable, REQ_ACC_DATA);
}
#else
/*----------------------------------------------------------------------------*/
static int BMA255_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;
	//u8 addr = BMA255_REG_POWER_CTL;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	u8 actual_power_mode = 0;
	
	if(enable == sensor_power )
	{
		GSE_LOG("Sensor power status is newest!\n");
		return BMA255_SUCCESS;
	}
	
	mutex_lock(&obj->lock);
	if(enable == true)
	{
		actual_power_mode = BMA255_MODE_NORMAL;
	}
	else
	{
		actual_power_mode = BMA255_MODE_SUSPEND;
	}
	
	res = bma_i2c_read_block(client,
			BMA255_MODE_CTRL_REG, &databuf[0], 1);
	res += bma_i2c_read_block(client,
		BMA255_LOW_POWER_CTRL_REG, &databuf[1], 1);

	switch (actual_power_mode) {
	case BMA255_MODE_NORMAL:
		databuf[0] = BMA255_SET_BITSLICE(databuf[0],
			BMA255_MODE_CTRL, 0);
		databuf[1] = BMA255_SET_BITSLICE(databuf[1],
			BMA255_LOW_POWER_MODE, 0);
		res += bma_i2c_write_block(client,
			BMA255_MODE_CTRL_REG, &databuf[0], 1);
		mdelay(1);
		res += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &databuf[1], 1);
		mdelay(1);
	break;
	case BMA255_MODE_SUSPEND:
		databuf[0] = BMA255_SET_BITSLICE(databuf[0],
			BMA255_MODE_CTRL, 4);
		databuf[1] = BMA255_SET_BITSLICE(databuf[1],
			BMA255_LOW_POWER_MODE, 0);
		res += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &databuf[1], 1);
		mdelay(1);
		res += bma_i2c_write_block(client,
			BMA255_MODE_CTRL_REG, &databuf[0], 1);
		mdelay(1);
	break;
	}

	if(res < 0)
	{
		GSE_ERR("set power mode failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA255_ERR_I2C;
	}
	sensor_power = enable;
	mutex_unlock(&obj->lock);
	
	return BMA255_SUCCESS;    
}
#endif	// end of SIG_MOTION_INT
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
/*----------------------------------------------------------------------------*/
static int BMA255_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[2] = {0};    
	int res = 0;

	mutex_lock(&obj->lock);
	res = bma_i2c_read_block(client,
		BMA255_RANGE_SEL_REG, &databuf[0], 1);
	databuf[0] = BMA255_SET_BITSLICE(databuf[0],
		BMA255_RANGE_SEL, dataformat);
	res += bma_i2c_write_block(client,
		BMA255_RANGE_SEL_REG, &databuf[0], 1);
	mdelay(1);

	if(res < 0)
	{
		GSE_ERR("set data format failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA255_ERR_I2C;
	}
	mutex_unlock(&obj->lock);
	
	return BMA255_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int BMA255_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);

	mutex_lock(&obj->lock);
	res = bma_i2c_read_block(client,
		BMA255_BANDWIDTH__REG, &databuf[0], 1);
	databuf[0] = BMA255_SET_BITSLICE(databuf[0],
		BMA255_BANDWIDTH, bwrate);
	res += bma_i2c_write_block(client,
		BMA255_BANDWIDTH__REG, &databuf[0], 1);
	mdelay(1);

	if(res < 0)
	{
		GSE_ERR("set bandwidth failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA255_ERR_I2C;
	}
	mutex_unlock(&obj->lock);

	return BMA255_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
// To prevent disable interrupt unexpectedly
#if 0
static int BMA255_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	int res = 0;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	
	mutex_lock(&obj->lock);
	res = bma_i2c_write_block(client, BMA255_INT_REG_1, &intenable, 0x01);
	mdelay(1);
	if(res != BMA255_SUCCESS) 
	{
		mutex_unlock(&obj->lock);
		return res;
	}

	res = bma_i2c_write_block(client, BMA255_INT_REG_2, &intenable, 0x01);
	mdelay(1);
	if(res != BMA255_SUCCESS) 
	{
		mutex_unlock(&obj->lock);
		return res;
	}
	mutex_unlock(&obj->lock);
	printk("BMA255 disable interrupt ...\n");

	/*for disable interrupt function*/

	return BMA255_SUCCESS;	  
}
#endif
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
/*----------------------------------------------------------------------------*/
static int bma255_init_client(struct i2c_client *client, int reset_cali)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	printk("bma255_init_client \n");

	res = BMA255_CheckDeviceID(client); 
	if(res != BMA255_SUCCESS)
	{
		return res;
	}	
	printk("BMA255_CheckDeviceID ok \n");
	
	res = BMA255_SetBWRate(client, BMA255_BW_100HZ);
	if(res != BMA255_SUCCESS ) 
	{
		return res;
	}
	printk("BMA255_SetBWRate OK!\n");
	
	res = BMA255_SetDataFormat(client, BMA255_RANGE_4G);
	if(res != BMA255_SUCCESS) 
	{
		return res;
	}
	printk("BMA255_SetDataFormat OK!\n");

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	// To prevent disable interrupt unexpectedly
	#if 0
	res = BMA255_SetIntEnable(client, 0x00);        
	if(res != BMA255_SUCCESS)
	{
		return res;
	}
	printk("BMA255 disable interrupt function!\n");
	#endif
	/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */

	res = BMA255_SetPowerMode(client, false);
	if(res != BMA255_SUCCESS)
	{
		return res;
	}
	printk("BMA255_SetPowerMode OK!\n");

	if(reset_cali != 0)
	{ 
		/*reset calibration only in power on*/
		res = BMA255_ResetCalibration(client);
		if(res != BMA255_SUCCESS)
		{
			return res;
		}
	}
	printk("bma255_init_client OK!\n");
#ifdef CONFIG_BMA255_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	mdelay(20);

	return BMA255_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((buf == NULL)||(bufsize<=30))
	{
		return -1;
	}
	
	if(client == NULL)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "BMA255 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA255_CompassReadData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);
	//u8 databuf[20];
	int acc[BMA255_AXES_NUM];
	int res = 0;
	s16 databuf[BMA255_AXES_NUM];
	//memset(databuf, 0, sizeof(u8)*10);

	if(buf == NULL)
	{
		return -1;
	}
	if(client == NULL)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == false)
	{
		res = BMA255_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on bma255 error %d!\n", res);
		}
	}

	if( (res = BMA255_ReadData(client, databuf)) )
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		/* Add compensated value performed by MTK calibration process*/
		databuf[BMA255_AXIS_X] += obj->cali_sw[BMA255_AXIS_X];
		databuf[BMA255_AXIS_Y] += obj->cali_sw[BMA255_AXIS_Y];
		databuf[BMA255_AXIS_Z] += obj->cali_sw[BMA255_AXIS_Z];

		/*remap coordinate*/
		acc[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*databuf[BMA255_AXIS_X];
		acc[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*databuf[BMA255_AXIS_Y];
		acc[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*databuf[BMA255_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA255_AXIS_X],obj->cvt.sign[BMA255_AXIS_Y],obj->cvt.sign[BMA255_AXIS_Z]);

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);

		sprintf(buf, "%d %d %d", (s16)acc[BMA255_AXIS_X], (s16)acc[BMA255_AXIS_Y], (s16)acc[BMA255_AXIS_Z]);
		if(atomic_read(&obj->trace) & BMA_TRC_IOCTL)
		{
			GSE_LOG("gsensor data for compass: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);
	//u8 databuf[20];
	int acc[BMA255_AXES_NUM];
	int res = 0;
	s16 databuf[BMA255_AXES_NUM];
	//memset(databuf, 0, sizeof(u8)*10);

	if(buf == NULL)
	{
		return -1;
	}
	if(client == NULL)
	{
		*buf = 0;
		return -2;
	}

/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#if SIG_MOTION_INT
	mutex_lock(&bma255_op_mode.lock);
	if(sensor_power == false)
	{
		res = vote_op_mode_locked(client, true, REQ_ACC_DATA);
		if(res)
		{
			GSE_ERR("Power on bma255 error %d!\n", res);
		}
	}

	res = BMA255_ReadData(client, databuf);
	mutex_unlock(&bma255_op_mode.lock);
	if(res)
#else
	if(sensor_power == false)
	{
		res = BMA255_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on bma255 error %d!\n", res);
		}
	}

	if( (res = BMA255_ReadData(client, databuf)) )
#endif	// end of SIG_MOTION_INT
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//printk("raw data x=%d, y=%d, z=%d \n",obj->data[BMA255_AXIS_X],obj->data[BMA255_AXIS_Y],obj->data[BMA255_AXIS_Z]);
		databuf[BMA255_AXIS_X] += obj->cali_sw[BMA255_AXIS_X];
		databuf[BMA255_AXIS_Y] += obj->cali_sw[BMA255_AXIS_Y];
		databuf[BMA255_AXIS_Z] += obj->cali_sw[BMA255_AXIS_Z];
		
		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[BMA255_AXIS_X],obj->cali_sw[BMA255_AXIS_Y],obj->cali_sw[BMA255_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*databuf[BMA255_AXIS_X];
		acc[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*databuf[BMA255_AXIS_Y];
		acc[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*databuf[BMA255_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA255_AXIS_X],obj->cvt.sign[BMA255_AXIS_Y],obj->cvt.sign[BMA255_AXIS_Z]);

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[BMA255_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[BMA255_AXIS_X] = acc[BMA255_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA255_AXIS_Y] = acc[BMA255_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA255_AXIS_Z] = acc[BMA255_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		

		sprintf(buf, "%04x %04x %04x", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);
		if(atomic_read(&obj->trace) & BMA_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA255_ReadRawData(struct i2c_client *client, char *buf)
{
	//struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);
	int res = 0;
	s16 databuf[BMA255_AXES_NUM];

	if (!buf || !client)
	{
		return EINVAL;
	}

/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#if SIG_MOTION_INT
	mutex_lock(&bma255_op_mode.lock);
	if(sensor_power == false)
	{
		res = vote_op_mode_locked(client, true, REQ_ACC_DATA);
		if(res)
		{
			GSE_ERR("Power on bma255 error %d!\n", res);
		}
	}

	res = BMA255_ReadData(client, databuf);
	mutex_unlock(&bma255_op_mode.lock);
	if(res)
#else
	if( (res = BMA255_ReadData(client, databuf)) )
#endif	// end of SIG_MOTION_INT
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "BMA255_ReadRawData %04x %04x %04x", databuf[BMA255_AXIS_X], 
			databuf[BMA255_AXIS_Y], databuf[BMA255_AXIS_Z]);
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma255_set_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data[2] = {0};
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);

	if ((client == NULL) || (mode >= 3))
	{
		return -1;
	}
	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMA255_EN_LOW_POWER__REG, &data[0], 1);
	comres += bma_i2c_read_block(client,
		BMA255_LOW_POWER_CTRL_REG, &data[1], 1);
	switch (mode) {
	case BMA255_MODE_NORMAL:
		data[0]  = BMA255_SET_BITSLICE(data[0],
				BMA255_MODE_CTRL, 0);
		data[1]  = BMA255_SET_BITSLICE(data[1],
				BMA255_LOW_POWER_MODE, 0);
		comres += bma_i2c_write_block(client,
				BMA255_MODE_CTRL_REG, &data[0], 0x01);
		mdelay(1);
		comres += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &data[1], 0x01);
		break;
	case BMA255_MODE_LOWPOWER:
		data[0]  = BMA255_SET_BITSLICE(data[0],
				BMA255_MODE_CTRL, 2);
		data[1]  = BMA255_SET_BITSLICE(data[1],
				BMA255_LOW_POWER_MODE, 0);
		comres += bma_i2c_write_block(client,
				BMA255_MODE_CTRL_REG, &data[0], 0x01);
		mdelay(1);
		comres += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &data[1], 0x01);
		break;
	case BMA255_MODE_SUSPEND:
		data[0]  = BMA255_SET_BITSLICE(data[0],
				BMA255_MODE_CTRL, 4);
		data[1]  = BMA255_SET_BITSLICE(data[1],
				BMA255_LOW_POWER_MODE, 0);
		comres += bma_i2c_write_block(client,
			BMA255_LOW_POWER_CTRL_REG, &data[1], 0x01);
		mdelay(1);
		comres += bma_i2c_write_block(client,
			BMA255_MODE_CTRL_REG, &data[0], 0x01);
		break;
	default:
		break;
	}

	mutex_unlock(&obj->lock);

	if(comres <= 0)
	{
		return BMA255_ERR_I2C;
	}
	else
	{
		return comres;
	}
}
/*----------------------------------------------------------------------------*/
static int bma255_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 0;

	if (client == NULL) 
	{
		return -1;
	}
	comres = bma_i2c_read_block(client,
			BMA255_EN_LOW_POWER__REG, mode, 1);
	*mode  = (*mode) >> 6;
		
	return comres;
}

/*----------------------------------------------------------------------------*/
static int bma255_set_range(struct i2c_client *client, unsigned char range)
{
	int comres = 0;
	unsigned char data[2] = {BMA255_RANGE_SEL__REG};
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);

	if (client == NULL)
	{
		return -1;
	}
	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMA255_RANGE_SEL__REG, data+1, 1);

	data[1]  = BMA255_SET_BITSLICE(data[1],
			BMA255_RANGE_SEL, range);

	mutex_lock(&bma255_i2c_mutex);	// Grace add for i2c protect
	comres = i2c_master_send(client, data, 2);
	mutex_unlock(&bma255_i2c_mutex);
	mutex_unlock(&obj->lock);
	if(comres <= 0)
	{
		return BMA255_ERR_I2C;
	}
	else
	{
		return comres;
	}
}
/*----------------------------------------------------------------------------*/
static int bma255_get_range(struct i2c_client *client, unsigned char *range)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) 
	{
		return -1;
	}

	comres = bma_i2c_read_block(client, BMA255_RANGE_SEL__REG,	&data, 1);
	*range = BMA255_GET_BITSLICE(data, BMA255_RANGE_SEL);

	return comres;
}
/*----------------------------------------------------------------------------*/
static int bma255_set_bandwidth(struct i2c_client *client, unsigned char bandwidth)
{
	int comres = 0;
	unsigned char data[2] = {BMA255_BANDWIDTH__REG};
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);

	if (client == NULL)
	{
		return -1;
	}

	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMA255_BANDWIDTH__REG, data+1, 1);

	data[1]  = BMA255_SET_BITSLICE(data[1],
			BMA255_BANDWIDTH, bandwidth);

	mutex_lock(&bma255_i2c_mutex);	// Grace add for i2c protect
	comres = i2c_master_send(client, data, 2);
	mutex_unlock(&bma255_i2c_mutex);
	mutex_unlock(&obj->lock);
	if(comres <= 0)
	{
		return BMA255_ERR_I2C;
	}
	else
	{
		return comres;
	}
}
/*----------------------------------------------------------------------------*/
static int bma255_get_bandwidth(struct i2c_client *client, unsigned char *bandwidth)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) 
	{
		return -1;
	}

	comres = bma_i2c_read_block(client, BMA255_BANDWIDTH__REG, &data, 1);
	data = BMA255_GET_BITSLICE(data, BMA255_BANDWIDTH);

	if (data < 0x08) //7.81Hz
	{
		*bandwidth = 0x08;
	}
	else if (data > 0x0f)	// 1000Hz
	{
		*bandwidth = 0x0f;
	}
	else
	{
		*bandwidth = data;
	}
	return comres;
}

/*----------------------------------------------------------------------------*/
static int bma255_set_fifo_mode(struct i2c_client *client, unsigned char fifo_mode)
{
	int comres = 0;
	unsigned char data[2] = {BMA255_FIFO_MODE__REG};
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);

	if (client == NULL || fifo_mode >= 4)
	{
		return -1;
	}

	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMA255_FIFO_MODE__REG, data+1, 1);

	data[1]  = BMA255_SET_BITSLICE(data[1],
			BMA255_FIFO_MODE, fifo_mode);

	mutex_lock(&bma255_i2c_mutex);	// Grace add for i2c protect
	comres = i2c_master_send(client, data, 2);
	mutex_unlock(&bma255_i2c_mutex);
	mutex_unlock(&obj->lock);
	if(comres <= 0)
	{
		return BMA255_ERR_I2C;
	}
	else
	{
		return comres;
	}
}
/*----------------------------------------------------------------------------*/
static int bma255_get_fifo_mode(struct i2c_client *client, unsigned char *fifo_mode)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) 
	{
		return -1;
	}

	comres = bma_i2c_read_block(client, BMA255_FIFO_MODE__REG, &data, 1);
	*fifo_mode = BMA255_GET_BITSLICE(data, BMA255_FIFO_MODE);

	return comres;
}

static int bma255_get_fifo_framecount(struct i2c_client *client, unsigned char *framecount)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) 
	{
		return -1;
	}

	comres = bma_i2c_read_block(client, BMA255_FIFO_FRAME_COUNTER_S__REG, &data, 1);
	*framecount = BMA255_GET_BITSLICE(data, BMA255_FIFO_FRAME_COUNTER_S);
	return comres;
}

static int bma255_soft_reset(struct i2c_client *client)
{
	int comres = 0;
	unsigned char data = BMA2X2_EN_SOFT_RESET_VALUE;

	GSE_FUN();
	comres = bma_i2c_write_block(client, BMA2X2_EN_SOFT_RESET__REG, &data, 1);
	if (comres!=0)
	{
		bma255_softreset_result = -1;
		GSE_LOG("[%s] fail\n", __func__);
	}
	else
	{
		bma255_softreset_result = 0;
		GSE_LOG("[%s] success\n", __func__);
	}

	return comres;
}


//tad3sgh add++
// Daemon application save the data
static int ECS_SaveData(int buf[CALIBRATION_DATA_SIZE])
{
#if DEBUG	
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif

	mutex_lock(&sensor_data_mutex);
	switch (buf[0])
	{
	case 2:	/* SENSOR_HANDLE_MAGNETIC_FIELD */
		memcpy(sensor_data+4, buf+1, 4*sizeof(int));	
		break;
	case 3:	/* SENSOR_HANDLE_ORIENTATION */
		memcpy(sensor_data+8, buf+1, 4*sizeof(int));	
		break;
#ifdef BMC050_M4G
	case 4:	/* SENSOR_HANDLE_GYROSCOPE */
		memcpy(m4g_data, buf+1, 4*sizeof(int));
		break;
#endif //BMC050_M4G
#ifdef BMC050_VRV
	case 11:	/* SENSOR_HANDLE_ROTATION_VECTOR */
		memcpy(m4g_data+4, buf+1, 4*sizeof(int));
		break;
#endif //BMC050_VRV
#ifdef BMC050_VLA
	case 10: /* SENSOR_HANDLE_LINEAR_ACCELERATION */
		memcpy(vla_data, buf+1, 4*sizeof(int));
		break;
#endif //BMC050_VLA
#ifdef BMC050_VG
	case 9: /* SENSOR_HANDLE_GRAVITY */
		memcpy(vg_data, buf+1, 4*sizeof(int));
		break;
#endif //BMC050_VG
	default:
		break;
	}
	mutex_unlock(&sensor_data_mutex);
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_LOG("Get daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],
			sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],
			sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
#if defined(BMC050_M4G) || defined(BMC050_VRV)
		GSE_LOG("Get m4g data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			m4g_data[0],m4g_data[1],m4g_data[2],m4g_data[3],
			m4g_data[4],m4g_data[5],m4g_data[6],m4g_data[7],
			m4g_data[8],m4g_data[9],m4g_data[10],m4g_data[11]);
#endif //BMC050_M4G || BMC050_VRV
#if defined(BMC050_VLA)
		GSE_LOG("Get vla data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			vla_data[0],vla_data[1],vla_data[2],vla_data[3],
			vla_data[4],vla_data[5],vla_data[6],vla_data[7],
			vla_data[8],vla_data[9],vla_data[10],vla_data[11]);
#endif //BMC050_VLA

#if defined(BMC050_VG)
		GSE_LOG("Get vg data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			vg_data[0],vg_data[1],vg_data[2],vg_data[3],
			vg_data[4],vg_data[5],vg_data[6],vg_data[7],
			vg_data[8],vg_data[9],vg_data[10],vg_data[11]);
#endif //BMC050_VG
	}	
#endif

	return 0;
}

//tad3sgh add--
/*----------------------------------------------------------------------------*/

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE];
	if(client == NULL)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	BMA255_ReadChipInfo(client, strbuf, BMA255_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
// Unused function , bypass for build
#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE];
	
	if(client == NULL)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	bma255_init_client(client, 1);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
}
#endif
/*----------------------------------------------------------------------------*/
/*
* g sensor opmode for compass tilt compensation
*/
static ssize_t show_cpsopmode_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_mode(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
/*
* g sensor opmode for compass tilt compensation
*/
static ssize_t store_cpsopmode_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	if ( (error = kstrtoul(buf, 10, &data)) )
	{
		return error;
	}
	if (data == BMA255_MODE_NORMAL)
	{
		BMA255_SetPowerMode(bma255_i2c_client, true);
	}
	else if (data == BMA255_MODE_SUSPEND)
	{
		BMA255_SetPowerMode(bma255_i2c_client, false);
	}
	else if (bma255_set_mode(bma255_i2c_client, (unsigned char) data) < 0)
	{
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
		GSE_ERR("invalid content: '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
/*
* g sensor range for compass tilt compensation
*/
static ssize_t show_cpsrange_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_range(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
/*
* g sensor range for compass tilt compensation
*/
static ssize_t store_cpsrange_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	if ( (error = kstrtoul(buf, 10, &data)) )
	{
		return error;
	}
	if (bma255_set_range(bma255_i2c_client, (unsigned char) data) < 0)
	{
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
		GSE_ERR("invalid content: '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
/*
* g sensor bandwidth for compass tilt compensation
*/
static ssize_t show_cpsbandwidth_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_bandwidth(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
/*
* g sensor bandwidth for compass tilt compensation
*/
static ssize_t store_cpsbandwidth_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	if ( (error = kstrtoul(buf, 10, &data)) )
	{
		return error;
	}
	if (bma255_set_bandwidth(bma255_i2c_client, (unsigned char) data) < 0)
	{
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
		GSE_ERR("invalid content: '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
/*
* g sensor data for compass tilt compensation
*/
static ssize_t show_cpsdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE];
	
	if(client == NULL)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA255_CompassReadData(client, strbuf, BMA255_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	char strbuf[BMA255_BUFSIZE];
	
	if(client == NULL)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA255_ReadSensorData(client, strbuf, BMA255_BUFSIZE);
	//BMA255_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[BMA255_AXES_NUM];

	if(client == NULL)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	if( (err = BMA255_ReadOffset(client, obj->offset)) )
	{
		return -EINVAL;
	}
	else if( (err = BMA255_ReadCalibration(client, tmp)) )
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/bma255_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[BMA255_AXIS_X], obj->offset[BMA255_AXIS_Y], obj->offset[BMA255_AXIS_Z],
			obj->offset[BMA255_AXIS_X], obj->offset[BMA255_AXIS_Y], obj->offset[BMA255_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[BMA255_AXIS_X], obj->cali_sw[BMA255_AXIS_Y], obj->cali_sw[BMA255_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[BMA255_AXIS_X]*mul + obj->cali_sw[BMA255_AXIS_X],
			obj->offset[BMA255_AXIS_Y]*mul + obj->cali_sw[BMA255_AXIS_Y],
			obj->offset[BMA255_AXIS_Z]*mul + obj->cali_sw[BMA255_AXIS_Z],
			tmp[BMA255_AXIS_X], tmp[BMA255_AXIS_Y], tmp[BMA255_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma255_i2c_client;  
	int err, x, y, z;
	int dat[BMA255_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if( (err = BMA255_ResetCalibration(client)) )
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z) == 3)
	{
		dat[BMA255_AXIS_X] = x;
		dat[BMA255_AXIS_Y] = y;
		dat[BMA255_AXIS_Z] = z;
		if( (err = BMA255_WriteCalibration(client, dat)) )
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_BMA255_LOWPASS
	struct i2c_client *client = bma255_i2c_client;
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][BMA255_AXIS_X], obj->fir.raw[idx][BMA255_AXIS_Y], obj->fir.raw[idx][BMA255_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[BMA255_AXIS_X], obj->fir.sum[BMA255_AXIS_Y], obj->fir.sum[BMA255_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[BMA255_AXIS_X]/len, obj->fir.sum[BMA255_AXIS_Y]/len, obj->fir.sum[BMA255_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_BMA255_LOWPASS
	struct i2c_client *client = bma255_i2c_client;  
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(sscanf(buf, "%d", &firlen) != 1)
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(firlen == NULL)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct bma255_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(sscanf(buf, "0x%x", &trace) == 1)
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
		GSE_ERR("invalid content: '%s'\n", buf);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct bma255_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	if(sensor_power)
	{
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
		return sprintf(buf, "work mode, sensor_power = %d\n", sensor_power);
	}
	else
	{
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);
		return sprintf(buf, "standby mode, sensor_power = %d\n", sensor_power);
	}
}

/*----------------------------------------------------------------------------*/
static ssize_t show_fifo_mode_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_fifo_mode(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
static ssize_t store_fifo_mode_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	if ( (error = kstrtoul(buf, 10, &data)) )
	{
		return error;
	}
	if (bma255_set_fifo_mode(bma255_i2c_client, (unsigned char) data) < 0)
	{
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
		GSE_ERR("invalid content: '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_fifo_framecount_value(struct device_driver *ddri, char *buf)
{
	unsigned char data;

	if (bma255_get_fifo_framecount(bma255_i2c_client, &data) < 0)
	{
		return sprintf(buf, "Read error\n");
	}
	else
	{
		return sprintf(buf, "%d\n", data);
	}
}

/*----------------------------------------------------------------------------*/
static ssize_t store_fifo_framecount_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct bma255_i2c_data *obj = obj_i2c_data;

	if ( (error = kstrtoul(buf, 10, &data)) )
	{
		return error;
	}
	mutex_lock(&obj->lock);
	obj->fifo_count = (unsigned char)data;
	mutex_unlock(&obj->lock);

	return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_fifo_data_out_frame_value(struct device_driver *ddri, char *buf)
{
	int err = 0, i, len = 0;
	//int addr = 0;
	u8 fifo_data_out[MAX_FIFO_F_BYTES] = {0};
	/* Select X Y Z axis data output for every fifo frame, not single axis data */
	unsigned char f_len = 6;/* FIXME: ONLY USE 3-AXIS */
	struct bma255_i2c_data *obj = obj_i2c_data;
	s16 acc[BMA255_AXES_NUM];
	s16 databuf[BMA255_AXES_NUM];

	if (obj->fifo_count == 0) {
		return -EINVAL;
	}

	for (i = 0; i < obj->fifo_count; i++) {
		if (bma_i2c_read_block(bma255_i2c_client,
			BMA255_FIFO_DATA_OUTPUT_REG, fifo_data_out, f_len) < 0)
		{
			GSE_ERR("[a]fatal error\n");
			return sprintf(buf, "Read byte block error\n");
		}
		/*data combination*/
		databuf[BMA255_AXIS_X] = ((s16)(((u16)fifo_data_out[1] << 8) |
						(u16)fifo_data_out[0])) >> 6;
		databuf[BMA255_AXIS_Y] = ((s16)(((u16)fifo_data_out[3] << 8) |
						(u16)fifo_data_out[2])) >> 6;
		databuf[BMA255_AXIS_Z] = ((s16)(((u16)fifo_data_out[5] << 8) |
						(u16)fifo_data_out[4])) >> 6;
		/*axis remap*/
		acc[obj->cvt.map[BMA255_AXIS_X]] = obj->cvt.sign[BMA255_AXIS_X]*databuf[BMA255_AXIS_X];
		acc[obj->cvt.map[BMA255_AXIS_Y]] = obj->cvt.sign[BMA255_AXIS_Y]*databuf[BMA255_AXIS_Y];
		acc[obj->cvt.map[BMA255_AXIS_Z]] = obj->cvt.sign[BMA255_AXIS_Z]*databuf[BMA255_AXIS_Z];
		
		len = sprintf(buf, "%d %d %d ", acc[BMA255_AXIS_X], acc[BMA255_AXIS_Y], acc[BMA255_AXIS_Z]);
		buf += len;
		err += len;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_softreset_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	bma255_softreset_result = -1;
	err = bma255_soft_reset(bma255_i2c_client);

	return count;
}
static ssize_t show_softreset_value(struct device_driver *ddri, char *buf)
{
	if (bma255_softreset_result==0)
	{
		return sprintf(buf, "%d - success\n", bma255_softreset_result);
	}
	else
	{
		return sprintf(buf, "%d - fail\n", bma255_softreset_result);
	}
}

/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
static int read_motion_config(struct bma255_i2c_data *obj, struct motion_config *cfg)
{
	int rc;

	memset(cfg, 0, sizeof(*cfg));

	rc = bma_i2c_read_block(obj->client, BMA255_MODE_CTRL_REG, &cfg->mode, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_INT_EN_0, &cfg->int_en, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_REG_INT_MAP_0, &cfg->intmap, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_REG_INT_OUT_CTRL, &cfg->int_out_ctrl, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_REG_SLOPE_THRES, &cfg->slope_thres, 1);
	if (rc)
		goto err;
	rc = bma_i2c_read_block(obj->client, BMA255_REG_INT_STATUS, &cfg->int_status, 1);
err:
	if (rc)
		dev_err(&obj->client->dev, "%s = %d\n", __func__, rc);
	return rc;
}

static int print_motion_config(struct motion_config *cfg, char *buf, size_t size)
{
	return scnprintf(buf, size,
		"MODE %02x  INTEN %02x  INTMAP %02x  INTCTRL %02x  THRES %02x  INTSTATUS %02x\n",
		cfg->mode, cfg->int_en, cfg->intmap, cfg->int_out_ctrl, cfg->slope_thres, cfg->int_status);
}

static ssize_t show_motion_config(struct device_driver *ddri, char *buf)
{
	int rc;
	struct motion_config cfg;
	struct bma255_i2c_data *obj = obj_i2c_data;

	rc = read_motion_config(obj, &cfg);
	if (rc)
		goto err;
	rc = print_motion_config(&cfg, buf, PAGE_SIZE);
err:
	return rc;
}
static ssize_t store_motion_config(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	int rc;

	//dev_info(&obj->client->dev, "%s: enable %c\n", __func__, *buf);
	dev_info(&obj->client->dev, "%s: enter.. *buf = %#x\n", __func__, (*buf));
	rc = vote_op_mode(obj->client, *buf == '1', REQ_SHAKE_SENSOR);
	return rc ? rc : count;
}
static ssize_t show_pm_relax(struct device_driver *ddri, char *buf)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	u8 x;
	int rc = hwmsen_read_byte(obj->client, BMA255_REG_INT_STATUS, &x);
	//int rc = bma_i2c_read_block(obj->client, BMA255_REG_INT_STATUS, &x, 1);

	pm_relax(&obj->client->dev);
	if (rc)
		goto err;
	dev_info(&obj->client->dev, "%s 0x%02x\n", __func__, x);
	rc = scnprintf(buf, PAGE_SIZE, "0x%02x\n", x);
err:
	return rc;
}
static ssize_t store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	unsigned int reg, val;
	u8 x;
	int rc = sscanf(buf, "%x,%x", &val, &reg);

	if (rc == 2) {
		dev_info(&obj->client->dev, "writing %02x to %02x\n", val, reg);
		rc = hwmsen_write_byte(obj->client, reg, val);
		if (rc)
			goto err;
		rc = hwmsen_read_byte(obj->client, reg, &x);
		dev_info(&obj->client->dev, "read back %02x from %02x\n", x, reg);
	} else {
		dev_err(&obj->client->dev, "%s: EINVAL", __func__);
		rc = -EINVAL;
	}
err:
	return rc ? rc : count;
}
// for factory
static ssize_t show_interrupt_test_result(struct device_driver *ddri, char *buf)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	int len = 0;

	dev_info(&obj->client->dev, "[%s] g_iIRQCnt = %d \n", __FUNCTION__, g_iIRQCnt);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", g_iIRQCnt);

	g_iIRQCnt = 0;  //read clear
	return len;
}
static ssize_t store_start_interrupt_test(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	int rc;

	dev_info(&obj->client->dev, "%s: %s\n", __func__, buf);
	if (*buf == '1')
	{
		g_iIRQCnt = 0;
		rc = vote_op_mode(obj->client, true, REQ_SHAKE_SENSOR);
		dev_info(&obj->client->dev, "[%s] rc = %d, count=%d \n", __func__, rc, (int)count);
		return rc ? rc : count;
	}
	else
	{
		rc = vote_op_mode(obj->client, false, REQ_SHAKE_SENSOR);
		dev_info(&obj->client->dev, "[%s] rc = %d, count=%d \n", __func__, rc, (int)count);
		return rc ? rc : count;
	}
}
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
/*----------------------------------------------------------------------------*/
/* [BY57] S-BUG#811 Grace_Chang G-sensor read miscTA for FRU */
static ssize_t show_cali_miscTa_value(struct device_driver *ddri, char *buf)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	int len = 0;
	dev_info(&obj->client->dev, "[%s] enter..\n", __FUNCTION__);

	len += snprintf(buf+len, PAGE_SIZE-len, "(X: %#x, Y: %#x, Z: %#x)\n", g_iMiscTaXyz[0], g_iMiscTaXyz[1], g_iMiscTaXyz[2]);
	dev_info(&obj->client->dev, "[%s] (X: %#x, Y: %#x, Z: %#x)\n", __FUNCTION__, g_iMiscTaXyz[0], g_iMiscTaXyz[1], g_iMiscTaXyz[2]);
	dev_info(&obj->client->dev, "[%s] len = %d\n", __FUNCTION__, len);

	return len;
}

static ssize_t store_cali_miscTa_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bma255_i2c_data *obj = obj_i2c_data;
	struct i2c_client *client = bma255_i2c_client;
	int iXyz[4];
	long err = 0;
	int cali[3];

	sscanf(buf, "%d %d %d\n", &iXyz[0], &iXyz[1], &iXyz[2]);
	dev_info(&obj->client->dev, "[%s] iXyz[0] = %#x, iXyz[1] = %#x, iXyz[2] = %#x\n", __FUNCTION__, iXyz[0], iXyz[1], iXyz[2]);
	g_iMiscTaXyz[0] =  iXyz[0];
	g_iMiscTaXyz[1] =  iXyz[1];
	g_iMiscTaXyz[2] =  iXyz[2];

	dev_info(&obj->client->dev, "[%s] call BMA255_ResetCalibration() \n", __FUNCTION__);
	err = BMA255_ResetCalibration(client);

	cali[BMA255_AXIS_X] = g_iMiscTaXyz[0] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
	cali[BMA255_AXIS_Y] = g_iMiscTaXyz[1] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
	cali[BMA255_AXIS_Z] = g_iMiscTaXyz[2] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
	dev_info(&obj->client->dev, "[%s] obj->reso->sensitivity = %#x \n", __FUNCTION__, obj->reso->sensitivity);
	dev_info(&obj->client->dev, "[%s] cali[x] = %#x cali[y] = %#x cali[z] = %#x\n", __FUNCTION__, cali[BMA255_AXIS_X], cali[BMA255_AXIS_Y], cali[BMA255_AXIS_Z]);
	dev_info(&obj->client->dev, "[%s] call BMA255_WriteCalibration() \n", __FUNCTION__);
	err = BMA255_WriteCalibration(client, cali);

	return count;
}
/* [BY57] E-BUG#811 Grace_Chang G-sensor read miscTA for FRU */
/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(cpsdata, 	 S_IWUSR | S_IRUGO, show_cpsdata_value,    NULL);
static DRIVER_ATTR(cpsopmode,  S_IWUSR | S_IRUGO, show_cpsopmode_value,    store_cpsopmode_value);
static DRIVER_ATTR(cpsrange, 	 S_IWUSR | S_IRUGO, show_cpsrange_value,     store_cpsrange_value);
static DRIVER_ATTR(cpsbandwidth, S_IWUSR | S_IRUGO, show_cpsbandwidth_value,    store_cpsbandwidth_value);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);
static DRIVER_ATTR(fifo_mode, S_IWUSR | S_IRUGO, show_fifo_mode_value,    store_fifo_mode_value);
static DRIVER_ATTR(fifo_framecount, S_IWUSR | S_IRUGO, show_fifo_framecount_value,    store_fifo_framecount_value);
static DRIVER_ATTR(fifo_data_frame, S_IRUGO, show_fifo_data_out_frame_value,    NULL);
static DRIVER_ATTR(softreset,   S_IWUSR | S_IRUGO, show_softreset_value,      store_softreset_value);
/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
static DRIVER_ATTR(tilt, S_IWUSR | S_IRUGO, show_motion_config, store_motion_config);
static DRIVER_ATTR(pmrelax, S_IRUGO, show_pm_relax, NULL);
static DRIVER_ATTR(reg, S_IWUSR, NULL, store_reg);
static DRIVER_ATTR(shaketest, S_IWUSR | S_IRUGO, show_interrupt_test_result, store_start_interrupt_test);	// for factory
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
/* [BY57] S-BUG#811 Grace_Chang G-sensor read miscTA for FRU */
static DRIVER_ATTR(cali_miscTa, S_IWUSR | S_IRUGO, show_cali_miscTa_value, store_cali_miscTa_value);
/* [BY57] E-BUG#811 Grace_Chang G-sensor read miscTA for FRU */

/*----------------------------------------------------------------------------*/
static struct driver_attribute *bma255_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_cpsdata,	/*g sensor data for compass tilt compensation*/
	&driver_attr_cpsopmode,	/*g sensor opmode for compass tilt compensation*/
	&driver_attr_cpsrange,	/*g sensor range for compass tilt compensation*/
	&driver_attr_cpsbandwidth,	/*g sensor bandwidth for compass tilt compensation*/
	&driver_attr_fifo_mode,
	&driver_attr_fifo_framecount,
	&driver_attr_fifo_data_frame,
	&driver_attr_softreset,
	/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	&driver_attr_tilt,
	&driver_attr_pmrelax,
	&driver_attr_reg,
	&driver_attr_shaketest,	// for factory
	/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	/* [BY57] S-BUG#811 Grace_Chang G-sensor read miscTA for FRU */
	&driver_attr_cali_miscTa,
	/* [BY57] E-BUG#811 Grace_Chang G-sensor read miscTA for FRU */
};
/*----------------------------------------------------------------------------*/
static int bma255_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	//int num = (int)(sizeof(bma255_attr_list)/sizeof(bma255_attr_list[0]));
	int num = (int)( ARRAY_SIZE(bma255_attr_list) );
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if( (err = driver_create_file(driver, bma255_attr_list[idx])) )
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", bma255_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma255_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	//int num = (int)(sizeof(bma255_attr_list)/sizeof(bma255_attr_list[0]));
	int num = (int)( ARRAY_SIZE(bma255_attr_list) );

	if(driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, bma255_attr_list[idx]);
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/
#ifdef BMC050_M4G
int bmm050_m4g_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* g_data;	
#if DEBUG	
	struct i2c_client *client = bma255_i2c_client;  
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_FUN();
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				m4g_delay = value;
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_DELAY;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&g_flag, 1);
				}
				else
				{
					atomic_set(&g_flag, 0);
				}	
				
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR( "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				g_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				g_data->values[0] = m4g_data[0];
				g_data->values[1] = m4g_data[1];
				g_data->values[2] = m4g_data[2];
				g_data->status = m4g_data[3];
				g_data->value_divide = CONVERT_G_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & BMA_TRC_INFO)
				{
					GSE_LOG("Hwm get m4g data: %d, %d, %d. divide %d, status %d!\n",
						g_data->values[0],g_data->values[1],g_data->values[2],
						g_data->value_divide,g_data->status);
				}	
#endif
			}
			break;
		default:
			GSE_ERR( "m4g operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif //BMC050_M4G
/*----------------------------------------------------------------------------*/
#ifdef BMC050_VRV
int bmm050_vrv_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* vrv_data;	
#if DEBUG	
	struct i2c_client *client = bma255_i2c_client;  
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_FUN();
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				vrv_delay = value;
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_DELAY;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&vrv_flag, 1);
				}
				else
				{
					atomic_set(&vrv_flag, 0);
				}	
				
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR( "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vrv_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				vrv_data->values[0] = m4g_data[4];
				vrv_data->values[1] = m4g_data[5];
				vrv_data->values[2] = m4g_data[6];
				vrv_data->status = m4g_data[7];
				vrv_data->value_divide = CONVERT_VRV_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & BMA_TRC_INFO)
				{
					GSE_LOG("Hwm get rotation vector data: %d, %d, %d. divide %d, status %d!\n",
						vrv_data->values[0],vrv_data->values[1],vrv_data->values[2],
						vrv_data->value_divide,vrv_data->status);
				}	
#endif
			}
			break;
		default:
			GSE_ERR( "rotation vector operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif //BMC050_VRV
/*----------------------------------------------------------------------------*/
#ifdef BMC050_VLA
int bmm050_vla_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* vla_value;	
#if DEBUG	
	struct i2c_client *client = bma255_i2c_client;  
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_FUN();
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				vla_delay = value;
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VLA_DELAY;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&vla_flag, 1);
				}
				else
				{
					atomic_set(&vla_flag, 0);
				}	
				
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VLA_ACTIVE;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR( "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vla_value = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				vla_value->values[0] = vla_data[0];
				vla_value->values[1] = vla_data[1];
				vla_value->values[2] = vla_data[2];
				vla_value->status = vla_data[3];
				vla_value->value_divide = CONVERT_VLA_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & BMA_TRC_INFO)
				{
					GSE_LOG("Hwm get virtual linear accelerometer data: %d, %d, %d. divide %d, status %d!\n",
						vla_value->values[0],vla_value->values[1],vla_value->values[2],
						vla_value->value_divide,vla_value->status);
				}	
#endif
			}
			break;
		default:
			GSE_ERR( "virtual linear accelerometer operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif //BMC050_VLA
/*----------------------------------------------------------------------------*/
#ifdef BMC050_VG
int bmm050_vg_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* vg_value;	
#if DEBUG	
	struct i2c_client *client = bma255_i2c_client;  
	struct bma255_i2c_data *data = i2c_get_clientdata(bma255_i2c_client);
#endif
	
#if DEBUG
	if(atomic_read(&data->trace) & BMA_TRC_INFO)
	{
		GSE_FUN();
	}	
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				vg_delay = value;
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_DELAY;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR( "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&vg_flag, 1);
				}
				else
				{
					atomic_set(&vg_flag, 0);
				}	
				
				/* set the flag */
				mutex_lock(&uplink_event_flag_mutex);
				uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
				mutex_unlock(&uplink_event_flag_mutex);
				/* wake up the wait queue */
				wake_up(&uplink_event_flag_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR( "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				vg_value = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				vg_value->values[0] = vg_data[0];
				vg_value->values[1] = vg_data[1];
				vg_value->values[2] = vg_data[2];
				vg_value->status = vg_data[3];
				vg_value->value_divide = CONVERT_VG_DIV;
					
				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & BMA_TRC_INFO)
				{
					GSE_LOG("Hwm get virtual gravity data: %d, %d, %d. divide %d, status %d!\n",
						vg_value->values[0],vg_value->values[1],vg_value->values[2],
						vg_value->value_divide,vg_value->status);
				}	
#endif
			}
			break;
		default:
			GSE_ERR( "virtual gravity operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif //BMC050_VG
//tad3sgh add --
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int bma255_open(struct inode *inode, struct file *file)
{
	file->private_data = bma255_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int bma255_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long bma255_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct bma255_i2c_data *obj = (struct bma255_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[BMA255_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];
	//tad3sgh add ++
	int status; 				/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	int value[CALIBRATION_DATA_SIZE];			/* for SET_YPR */
	//tad3sgh add --
	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			bma255_init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			BMA255_ReadChipInfo(client, strbuf, BMA255_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			BMA255_ReadSensorData(client, strbuf, BMA255_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			BMA255_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[BMA255_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA255_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA255_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = BMA255_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = BMA255_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if( (err = BMA255_ReadCalibration(client, cali)) )
			{
				break;
			}
			
			sensor_data.x = cali[BMA255_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[BMA255_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[BMA255_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		//tad3sgh add ++
		case BMM_IOC_GET_EVENT_FLAG:	// used by daemon only
			data = (void __user *) arg;
			/* block if no event updated */
			wait_event_interruptible(uplink_event_flag_wq, (uplink_event_flag != 0));
			mutex_lock(&uplink_event_flag_mutex);
			status = uplink_event_flag;
			mutex_unlock(&uplink_event_flag_mutex);
			if(copy_to_user(data, &status, sizeof(status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case BMM_IOC_GET_NONBLOCK_EVENT_FLAG:	// used by daemon only
			data = (void __user *) arg;
			/* nonblock daemon process */
			//wait_event_interruptible(uplink_event_flag_wq, (uplink_event_flag != 0));
			mutex_lock(&uplink_event_flag_mutex);
			status = uplink_event_flag;
			mutex_unlock(&uplink_event_flag_mutex);
			if(copy_to_user(data, &status, sizeof(status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			break;
			
		case ECOMPASS_IOC_GET_DELAY:			//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &bmm050d_delay, sizeof(bmm050d_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_M_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_M_DELAY;
			}
			else if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_O_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_O_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		
			
		case ECOMPASS_IOC_SET_YPR:				//used by daemon
			data = (void __user *) arg;
			if(data == NULL)
			{
				GSE_ERR("invalid argument.");
				return -EINVAL;
			}
			if(copy_from_user(value, data, sizeof(value)))
			{
				GSE_ERR("copy_from_user failed.");
				return -EFAULT;
			}
			ECS_SaveData(value);
			break;

		case ECOMPASS_IOC_GET_MFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&m_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active m-channel when driver suspend regardless of m_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_M_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_M_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;
			
		case ECOMPASS_IOC_GET_OFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&o_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active m-channel when driver suspend regardless of m_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_O_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_O_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
		                
#ifdef BMC050_M4G
		case ECOMPASS_IOC_GET_GDELAY:			//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &m4g_delay, sizeof(m4g_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_G_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_G_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		

		case ECOMPASS_IOC_GET_GFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&g_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active g-channel when driver suspend regardless of g_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_G_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_G_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
#endif //BMC050_M4G

#ifdef BMC050_VRV
		case ECOMPASS_IOC_GET_VRVDELAY:			//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &vrv_delay, sizeof(vrv_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VRV_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VRV_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		

		case ECOMPASS_IOC_GET_VRVFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&vrv_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active vrv-channel when driver suspend regardless of vrv_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VRV_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
#endif //BMC050_VRV

#ifdef BMC050_VLA
		case ECOMPASS_IOC_GET_VLADELAY: 		//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &vla_delay, sizeof(vla_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VLA_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VLA_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		

		case ECOMPASS_IOC_GET_VLAFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&vla_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active vla-channel when driver suspend regardless of vla_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VLA_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VLA_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
#endif //BMC050_VLA

#ifdef BMC050_VG
		case ECOMPASS_IOC_GET_VGDELAY: 		//used by daemon
			data = (void __user *) arg;
			if(copy_to_user(data, &vg_delay, sizeof(vg_delay)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VG_DELAY) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VG_DELAY;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;		

		case ECOMPASS_IOC_GET_VGFLAG:		//used by daemon
			data = (void __user *) arg;
			sensor_status = atomic_read(&vg_flag);
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
			if ((sensor_status == 1) && (atomic_read(&driver_suspend_flag) == 1))
			{
				/* de-active vla-channel when driver suspend regardless of vla_flag*/
				sensor_status = 0;
			}
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
			if(copy_to_user(data, &sensor_status, sizeof(sensor_status)))
			{
				GSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			/* clear the flag */
			mutex_lock(&uplink_event_flag_mutex);
			if ((uplink_event_flag & BMMDRV_ULEVT_FLAG_VG_ACTIVE) != 0)
			{
				uplink_event_flag &= ~BMMDRV_ULEVT_FLAG_VG_ACTIVE;
			}
			mutex_unlock(&uplink_event_flag_mutex);
			/* wake up the wait queue */
			wake_up(&uplink_event_flag_wq);
			break;			
#endif //BMC050_VG
		//tad3sgh add --
		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

#ifdef CONFIG_COMPAT
static long bma255_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	void __user *arg32 = compat_ptr(arg);

	if(!file->f_op || !file->f_op->unlocked_ioctl)
	{
		return -ENOTTY;
	}

	switch(cmd)
	{
		case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
			//GSE_LOG("[%s] fwq COMPAT_GSENSOR_IOCTL_READ_SENSORDATA\n", __FUNCTION__);
			if(arg32 == NULL)
			{
				ret = -EINVAL;
				break;	  
			}
			ret = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
			// == bma255_unlocked_ioctl()
			if(ret)
			{
				GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed!\n");
			}
			break;
#if 1   //Grace add for set cali
		case COMPAT_GSENSOR_IOCTL_SET_CALI:
			GSE_LOG("[%s] COMPAT_GSENSOR_IOCTL_SET_CALI \n", __FUNCTION__);
			if(arg32 == NULL)
			{
				ret = -EINVAL;
				break;	  
			}
			ret = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
			// == bma255_unlocked_ioctl()
			if(ret)
			{
				GSE_LOG("[%s] GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed!\n", __FUNCTION__);
			}
			break;
#endif
		default:
			GSE_ERR("[%s] unknown IOCTL: 0x%08x\n", __FUNCTION__, cmd);
 			ret = -ENOIOCTLCMD;
			break;
	}

	return ret;
}
#endif

/*----------------------------------------------------------------------------*/
static struct file_operations bma255_fops = {
	//.owner = THIS_MODULE,
	.open = bma255_open,
	.release = bma255_release,
	.unlocked_ioctl = bma255_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bma255_compat_ioctl,
#endif	
};
/*----------------------------------------------------------------------------*/
static struct miscdevice bma255_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bma255_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
#if 0
static int bma255_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	
	GSE_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		GSE_LOG("[%s] msg.event == PM_EVENT_SUSPEND\n", __FUNCTION__);
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
		err = op_mode_suspend(client, true);
		/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
		atomic_set(&obj->suspend, 1);
		//tad3sgh add ++
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
		/* set driver suspend flag */
		atomic_set(&driver_suspend_flag, 1);
		if (atomic_read(&m_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
		if (atomic_read(&o_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#ifdef BMC050_M4G
		if (atomic_read(&g_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#endif //BMC050_M4G
#ifdef BMC050_VRV
		if (atomic_read(&vrv_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#endif //BMC050_VRV
#ifdef BMC050_VLA
		if (atomic_read(&vla_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_VLA_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#endif //BMC050_VLA
#ifdef BMC050_VG
		if (atomic_read(&vg_flag) == 1)
		{
			/* set the flag to block e-compass daemon*/
			mutex_lock(&uplink_event_flag_mutex);
			uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
			mutex_unlock(&uplink_event_flag_mutex);
		}
#endif //BMC050_VG

		/* wake up the wait queue */
		wake_up(&uplink_event_flag_wq);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND

//tad3sgh add --
		/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
		#if 0
		if( (err = BMA255_SetPowerMode(obj->client, false)) )
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}
		#endif
		/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
		BMA255_power(obj->hw, 0);
	}
	/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	else{
		GSE_LOG("[%s] msg.event != PM_EVENT_SUSPEND\n", __FUNCTION__);
	}
	/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma255_resume(struct i2c_client *client)
{
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#if SIG_MOTION_INT
	if (!BMA255_power(obj->hw, 1)) {
		if( (err = bma255_init_client(client, 0)) )
		{
			GSE_ERR("initialize client fail!!\n");
			return err;
		}
	}
	op_mode_suspend(client, false);
#else
	BMA255_power(obj->hw, 1);
	if( (err = bma255_init_client(client, 0)) )
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
#endif
	/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */

	//tad3sgh add ++
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
	/* clear driver suspend flag */
	atomic_set(&driver_suspend_flag, 0);
	if (atomic_read(&m_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
	if (atomic_read(&o_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#ifdef BMC050_M4G
	if (atomic_read(&g_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_M4G
#ifdef BMC050_VRV
	if (atomic_read(&vrv_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VRV
#ifdef BMC050_VG
	if (atomic_read(&vg_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VG

	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
//tad3sgh add --

	atomic_set(&obj->suspend, 0);

	return 0;
}
#else
static int bma255_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	err = op_mode_suspend(client, true);
	/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	atomic_set(&obj->suspend, 1);
	/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	#if 0
	if( (err = BMA255_SetPowerMode(obj->client, false)) )
	{
		GSE_ERR("write power control fail!!\n");
		return err;
	}
	#endif
	/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
	BMA255_power(obj->hw, 0);

	return err;
}
/*----------------------------------------------------------------------------*/
static int bma255_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma255_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#if SIG_MOTION_INT
	if (!BMA255_power(obj->hw, 1)) {
		if( (err = bma255_init_client(client, 0)) )
		{
			GSE_ERR("initialize client fail!!\n");
			return err;
		}
	}
	op_mode_suspend(client, false);
#else
	BMA255_power(obj->hw, 1);
	if( (err = bma255_init_client(client, 0)) )
	{
		GSE_ERR("initialize client fail!!\n");
		return err;
	}
#endif
	/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */

	atomic_set(&obj->suspend, 0);

	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void bma255_early_suspend(struct early_suspend *h) 
{
	struct bma255_i2c_data *obj = container_of(h, struct bma255_i2c_data, early_drv);   
	int err;
	
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
//tad3sgh add ++
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
	/* set driver suspend flag */
	atomic_set(&driver_suspend_flag, 1);
	if (atomic_read(&m_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
	if (atomic_read(&o_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#ifdef BMC050_M4G
	if (atomic_read(&g_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_M4G
#ifdef BMC050_VRV
	if (atomic_read(&vrv_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VRV
#ifdef BMC050_VLA
	if (atomic_read(&vla_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VLA_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VLA
#ifdef BMC050_VG
	if (atomic_read(&vg_flag) == 1)
	{
		/* set the flag to block e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VG

	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND

//tad3sgh add --
	if( (err = BMA255_SetPowerMode(obj->client, false)) )
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	BMA255_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void bma255_late_resume(struct early_suspend *h)
{
	struct bma255_i2c_data *obj = container_of(h, struct bma255_i2c_data, early_drv);         
	int err;
	
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	BMA255_power(obj->hw, 1);
	if( (err = bma255_init_client(obj->client, 0)) )
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}
//tad3sgh add ++
#ifdef BMC050_BLOCK_DAEMON_ON_SUSPEND
	/* clear driver suspend flag */
	atomic_set(&driver_suspend_flag, 0);
	if (atomic_read(&m_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
	if (atomic_read(&o_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#ifdef BMC050_M4G
	if (atomic_read(&g_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_M4G
#ifdef BMC050_VRV
	if (atomic_read(&vrv_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VRV_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VRV
#ifdef BMC050_VG
	if (atomic_read(&vg_flag) == 1)
	{
		/* set the flag to unblock e-compass daemon*/
		mutex_lock(&uplink_event_flag_mutex);
		uplink_event_flag |= BMMDRV_ULEVT_FLAG_VG_ACTIVE;
		mutex_unlock(&uplink_event_flag_mutex);
	}
#endif //BMC050_VG

	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);
#endif //BMC050_BLOCK_DAEMON_ON_SUSPEND
//tad3sgh add --
	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int bma255_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int bma255_enable_nodata(int en)
{
	int err = 0;

	if(((en == 0) && (sensor_power == false))
			||((en == 1) && (sensor_power == true))) {
		GSE_LOG("Gsensor device have updated!\n");
	} else {
		err = BMA255_SetPowerMode(obj_i2c_data->client, !sensor_power);
	}

	return err;
}

static int bma255_set_delay(u64 ns)
{
	int err = 0;
	int value, sample_delay;

	value = (int)ns/1000/1000;
	if(value <= 5) {
		sample_delay = BMA255_BW_200HZ;
	} else if(value <= 10) {
		sample_delay = BMA255_BW_100HZ;
	} else {
		sample_delay = BMA255_BW_50HZ;
	}

	//err = BMA255_SetBWRate(obj_i2c_data->client, sample_delay);
	if(err != BMA255_SUCCESS ) {
		GSE_ERR("Set delay parameter error!\n");
	}

	if(value >= 50) {
		atomic_set(&obj_i2c_data->filter, 0);
	} else {
#if defined(CONFIG_BMA255_LOWPASS)
		obj_i2c_data->fir.num = 0;
		obj_i2c_data->fir.idx = 0;
		obj_i2c_data->fir.sum[BMA255_AXIS_X] = 0;
		obj_i2c_data->fir.sum[BMA255_AXIS_Y] = 0;
		obj_i2c_data->fir.sum[BMA255_AXIS_Z] = 0;
		atomic_set(&obj_i2c_data->filter, 1);
#endif
	}

	return 0;
}

static int bma255_get_data(int* x, int* y, int* z, int* status)
{
	char buff[BMA255_BUFSIZE];

	/* [BY57] S-BUG#1106 Grace_Chang [Sensor] Screen activation from sleep mode is slow */
	// if i2c object still suspend, skip this times get_data.
	if (atomic_read(&obj_i2c_data->suspend) == 1)
		return -1;
	/* [BY57] E-BUG#1106 Grace_Chang [Sensor] Screen activation from sleep mode is slow */

	/* use acc raw data for gsensor */
	BMA255_ReadSensorData(obj_i2c_data->client, buff, BMA255_BUFSIZE);

	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

// Unused functions , bypass for build
#if 0
int bsx_algo_m_enable(int en)
{
	if(en == 1) {
		atomic_set(&m_flag, 1);
	} else {
		atomic_set(&m_flag, 0);
	}

	/* set the flag */
	mutex_lock(&uplink_event_flag_mutex);
	uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_ACTIVE;
	mutex_unlock(&uplink_event_flag_mutex);
	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);

	return 0;
}

int bsx_algo_m_set_delay(u64 ns)
{
	int value = (int)ns/1000/1000;

	bmm050d_delay = value;
	/* set the flag */
	mutex_lock(&uplink_event_flag_mutex);
	uplink_event_flag |= BMMDRV_ULEVT_FLAG_M_DELAY;
	mutex_unlock(&uplink_event_flag_mutex);
	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);

	return 0;
}

int bsx_algo_m_open_report_data(int open)
{
	return 0;
}

int bsx_algo_m_get_data(int* x, int* y, int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);

	*x = sensor_data[4];
	*y = sensor_data[5];
	*z = sensor_data[6];
	*status = sensor_data[7];

	mutex_unlock(&sensor_data_mutex);

	return 0;
}

int bsx_algo_o_enable(int en)
{
	if(en == 1) {
		atomic_set(&o_flag, 1);
	} else {
		atomic_set(&o_flag, 0);
	}

	/* set the flag */
	mutex_lock(&uplink_event_flag_mutex);
	uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_ACTIVE;
	mutex_unlock(&uplink_event_flag_mutex);
	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);

	return 0;
}

int bsx_algo_o_set_delay(u64 ns)
{
	int value = (int)ns/1000/1000;

	bmm050d_delay = value;
	/* set the flag */
	mutex_lock(&uplink_event_flag_mutex);
	uplink_event_flag |= BMMDRV_ULEVT_FLAG_O_DELAY;
	mutex_unlock(&uplink_event_flag_mutex);
	/* wake up the wait queue */
	wake_up(&uplink_event_flag_wq);

	return 0;
}

int bsx_algo_o_open_report_data(int open)
{
	return 0;
}

int bsx_algo_o_get_data(int* x, int* y, int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);

	*x = sensor_data[8];
	*y = sensor_data[9];
	*z = sensor_data[10];
	*status = sensor_data[11];

	mutex_unlock(&sensor_data_mutex);

	return 0;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
int bsx_algo_gyro_open_report_data(int open)
{
        //should queuq work to report event if  is_report_input_direct=true
        return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

int bsx_algo_gyro_enable_nodata(int en)
{

        if(en == 1) {
                atomic_set(&g_flag, 1);
        } else {
                atomic_set(&g_flag, 0);
        }

        /* set the flag */
        mutex_lock(&uplink_event_flag_mutex);
        uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_ACTIVE;
        mutex_unlock(&uplink_event_flag_mutex);
        /* wake up the wait queue */
        wake_up(&uplink_event_flag_wq);

        return 0;
}

int bsx_algo_gyro_set_delay(u64 ns)
{
        int value = (int)ns/1000/1000 ;

        m4g_delay = value;

        /* set the flag */
        mutex_lock(&uplink_event_flag_mutex);
        uplink_event_flag |= BMMDRV_ULEVT_FLAG_G_DELAY;
        mutex_unlock(&uplink_event_flag_mutex);

        /* wake up the wait queue */
        wake_up(&uplink_event_flag_wq);

        return 0;
}

int bsx_algo_gyro_get_data(int* x, int* y, int* z, int* status)
{

        mutex_lock(&sensor_data_mutex);

        *x = m4g_data[0];
        *y = m4g_data[1];
        *z = m4g_data[2];
        *status = m4g_data[3];

        mutex_unlock(&sensor_data_mutex);

        return 0;
}
#endif	// Unused functions , bypass for build
/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#if SIG_MOTION_INT
static irqreturn_t bma255_irq_handler(int irq, void *handle)
{
	struct bma255_i2c_data *obj = (struct bma255_i2c_data *)handle;

	GSE_FUN();
	g_iIRQCnt++;	// for factory
	//pm_stay_awake(&obj->client->dev);
	pm_wakeup_event(&obj->client->dev, 5000);	// limit pm_stay_awake time by 5 seconds
	//acc_shake_report();
	input_report_rel(obj->shake_idev, REL_MISC, 1);
	input_sync(obj->shake_idev);
	return IRQ_HANDLED;
}

static int bma255_setup_irq(struct bma255_i2c_data *obj)
{
	int irqf;
	int rc;
	//int irq = gpio_to_irq(IRQ_GPIO_NUM);
	int irq =  mt_gpio_to_irq(IRQ_GPIO_NUM);
	struct device *dev = &obj->client->dev;

	//irqf = IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND;
	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND;

	rc = devm_request_threaded_irq(dev, irq, NULL, bma255_irq_handler,
			irqf, dev_name(dev), obj);
	if (rc) {
		dev_err(dev, "could not request irq %d\n", irq);
	} else {
		enable_irq_wake(irq);
		device_init_wakeup(dev, 1);
		dev_dbg(dev, "requested irq %d\n", irq);
	}
	return rc;
}
#endif
/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */
/*----------------------------------------------------------------------------*/
static int bma255_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct bma255_i2c_data *obj;
	//tad3sgh add ++
	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
#ifdef BMC050_M4G
		struct hwmsen_object sobj_g;
#endif //BMC050_M4G
#ifdef BMC050_VRV
		struct hwmsen_object sobj_vrv;
#endif //BMC050_VRV
#ifdef BMC050_VLA
		struct hwmsen_object sobj_vla;
#endif //BMC050_VLA
#ifdef BMC050_VG
		struct hwmsen_object sobj_vg;
#endif //BMC050_VG
//tad3sgh add --
	int err = 0;
	
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct bma255_i2c_data));

	obj->hw = hw;
	
	if( (err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)) )
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		/* [BY57] S-BUG#857 Grace_Chang [KPI][Sensor] coverity issue */
		#ifdef ORG_VER
		goto exit;
		#else
		goto exit_kfree;
		#endif
		/* [BY57] E-BUG#857 Grace_Chang [KPI][Sensor] coverity issue */
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	mutex_init(&obj->lock);
	mutex_init(&bma255_i2c_mutex);	// Grace add for i2c protect
	//tad3sgh add ++
	mutex_init(&sensor_data_mutex);
	mutex_init(&uplink_event_flag_mutex);
	
	init_waitqueue_head(&uplink_event_flag_wq);
	//tad3sgh add --
	
#ifdef CONFIG_BMA255_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	bma255_i2c_client = new_client;	

	/* [BY57] S-BUG#226 Grace_Chang Significant motion sensor type for Doze */
#if SIG_MOTION_INT
	obj->shake_idev = devm_input_allocate_device(&client->dev);
	if (!obj->shake_idev) {
		dev_err(&client->dev, "unable to allocate input device\n");
		err = -ENODEV;
		goto exit_init_failed;
	}
	obj->shake_idev->name = shake_idev_name;
	input_set_capability(obj->shake_idev, EV_REL, REL_MISC);
	input_set_drvdata(obj->shake_idev, obj);
	err = input_register_device(obj->shake_idev);
	if (err < 0) {
		dev_err(&client->dev, "unable to register input device: %d\n",
				err);
		goto exit_init_failed;
	}

	mutex_init(&bma255_op_mode.lock);
	bma255_op_mode.req = 0;
	bma255_setup_irq(obj);
#endif
	/* [BY57] E-BUG#226 Grace_Chang Significant motion sensor type for Doze */

	if( (err = bma255_init_client(new_client, 1)) )
	{
		goto exit_init_failed;
	}

	if( (err = misc_register(&bma255_device)) )
	{
		GSE_ERR("bma255_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = bma255_create_attr(&(bma255_init_info.platform_diver_addr->driver));
	if(err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data= bma255_open_report_data;
	ctl.enable_nodata = bma255_enable_nodata;
	ctl.set_delay  = bma255_set_delay;
	ctl.is_report_input_direct = false;

	err = acc_register_control_path(&ctl);
	if(err)
	{
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = bma255_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
		GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}

#ifdef BMC050_M4G
	sobj_g.self = obj;
	sobj_g.polling = 1;
	sobj_g.sensor_operate = bmm050_m4g_operate;
	if( (err = hwmsen_attach(ID_GYROSCOPE, &sobj_g)) )
	{
		GSE_ERR( "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif //BMC050_M4G

#ifdef BMC050_VRV
	sobj_vrv.self = obj;
	sobj_vrv.polling = 1;
	sobj_vrv.sensor_operate = bmm050_vrv_operate;
	if( (err = hwmsen_attach(ID_ROTATION_VECTOR, &sobj_vrv)) )
	{
		GSE_ERR( "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif //BMC050_VRV

#ifdef BMC050_VLA
	sobj_vla.self = obj;
	sobj_vla.polling = 1;
	sobj_vla.sensor_operate = bmm050_vla_operate;
	if( (err = hwmsen_attach(ID_LINEAR_ACCELERATION, &sobj_vla)) )
	{
		GSE_ERR( "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif //BMC050_VLA

#ifdef BMC050_VG
	sobj_vg.self = obj;
	sobj_vg.polling = 1;
	sobj_vg.sensor_operate = bmm050_vg_operate;
	if( (err = hwmsen_attach(ID_GRAVITY, &sobj_vg)) )
	{
		GSE_ERR( "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif //BMC050_VG

//tad3sgh add --
#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = bma255_early_suspend,
	obj->early_drv.resume   = bma255_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

	bma255_init_flag =0;
	GSE_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&bma255_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	bma255_init_flag =-1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma255_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	err = bma255_delete_attr(&(bma255_init_info.platform_diver_addr->driver));
	if(err) {
		GSE_ERR("bma150_delete_attr fail: %d\n", err);
	}
	
	misc_deregister(&bma255_device);
	bma255_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma255_local_init(void) 
{
	GSE_FUN();

	BMA255_power(hw, 1);
	if(i2c_add_driver(&bma255_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(bma255_init_flag == -1)
	{
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma255_remove(void)
{
    GSE_FUN();    
    BMA255_power(hw, 0);    
    i2c_del_driver(&bma255_i2c_driver);
    return 0;
}

static struct acc_init_info bma255_init_info = {
	.name = "bma255",
	.init = bma255_local_init,
	.uninit = bma255_remove,
};

/*----------------------------------------------------------------------------*/
static int __init bma255_init(void)
{
	hw = get_accel_dts_func(COMPATIABLE_NAME, hw);

	GSE_FUN();
	acc_driver_add(&bma255_init_info);
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit bma255_exit(void)
{
	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(bma255_init);
module_exit(bma255_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMA255 I2C driver");
MODULE_AUTHOR("hongji.zhou@bosch-sensortec.com");
