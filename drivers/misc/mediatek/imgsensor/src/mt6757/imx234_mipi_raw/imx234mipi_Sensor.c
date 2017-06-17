/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX234mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <linux/xlog.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx234mipi_Sensor.h"

#define PFX "imx234_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(fmt, args...)	pr_debug(PFX "[%s] " fmt, __FUNCTION__, ##args)

//#define THERMAL_AND_POWER_TEST

static DEFINE_SPINLOCK(imgsensor_drv_lock);



static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = IMX234_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0x22866f5f,//0xfe9e1a79,//0x22866f5f,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 288000000,				//record different mode's pclk
		.linelength = 6144,				//record different mode's linelength
		.framelength = 1560,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2656,//2656,   //2672		//record different mode's width of grabwindow
		.grabwindow_height = 1496,//1494,	 //1500 	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 573000000,
		.linelength = 6144,
		.framelength = 3106,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5312,//5312,  //5344
		.grabwindow_height = 2992,//2988,  //3000
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	}, 
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 462000000,
		.linelength = 6144,
		.framelength = 3106,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5344,//5312,
		.grabwindow_height = 3000,//2988,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 240,	//less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps  
	},
	.normal_video = {
#ifndef THERMAL_AND_POWER_TEST
		.pclk = 288000000,              //record different mode's pclk
		.linelength = 6144,             //record different mode's linelength
		.framelength = 1560,            //record different mode's framelength
		.startx = 0,                    //record different mode's startx of grabwindow
		.starty = 0,                    //record different mode's starty of grabwindow
		.grabwindow_width = 2672,//2656,     //2672 	//record different mode's width of grabwindow
		.grabwindow_height = 1500,//1494,    //1500 	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,     //unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
#else
		.pclk = 573000000,
		.linelength = 6144,
		.framelength = 3106,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5312,
		.grabwindow_height = 2992,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
#endif
	},
	.hs_video = {
		.pclk = 429000000,
		.linelength = 6144,
		.framelength = 580,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 288000000,
		.linelength = 6144,
		.framelength = 1560,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2656,
		.grabwindow_height = 1494,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.margin = 4,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0x7fff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 1,		//enter capture delay frame num
	.pre_delay_frame = 1, 		//enter preview delay frame num
	.video_delay_frame = 1,		//enter video delay frame num
	.hs_video_delay_frame = 1,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 1,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x34, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x4C00,					//current shutter
	.gain = 0x0200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x34,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 5344, 3000,	  0,	0, 5312, 2992, 2656, 1496, 0000, 0000, 2656,  1496,	  0,	0, 2656, 1496}, // Preview
 { 5344, 3000,	  0,	0, 5312, 2992, 5312, 2992, 0000, 0000, 5312,  2992,	  0,	0, 5312, 2992}, // capture
#ifndef THERMAL_AND_POWER_TEST
 { 5344, 3000,	  0,	0, 5344, 3000, 2672, 1500, 0000, 0000, 2672,  1500,	  0,	0, 2672, 1500}, // video
#else
 { 5344, 3000,	  0,    0, 5312, 2992, 5312, 2992, 0000, 0000, 5312,  2992,	  0,	0, 5312, 2992}, // video
#endif
 { 5344, 3000,	  0,  536, 5344, 1920, 1336,  480, 	348, 0000,  640,   480,	  0,	0, 	640,  480}, //hight speed video 
 { 5344, 3000,	  0,	0, 5344, 3000, 2672, 1500, 0000, 0000, 2672,  1500,	  0,	0, 2672, 1500}};// slim video 


#define IMX234MIPI_MaxGainIndex (119)
kal_uint16 IMX234MIPI_sensorGainMapping[IMX234MIPI_MaxGainIndex][2] ={	
	{64 ,0	},
	{65 ,8	},
	{66 ,16 },
	{67 ,25 },
	{68 ,30 },
	{69 ,37 },
	{70 ,45 },
	{71 ,51 },
	{72 ,57 },
	{73 ,63 },
	{74 ,67 },
	{75 ,75 },
	{76 ,81 },
	{77 ,85 },
	{78 ,92 },
	{79 ,96 },
	{80 ,103},
	{81 ,107},
	{82 ,112},
	{83 ,118},
	{84 ,122},
	{86 ,133},
	{88 ,140},
	{89 ,144},
	{90 ,148},
	{93 ,159},
	{96 ,171},
	{97 ,174},
	{99 ,182},
	{101,188},
	{102,192},
	{104,197},
	{106,202},
	{107,206},
	{109,211},
	{112,220},
	{113,222},
	{115,228},
	{118,235},
	{120,239},
	{125,250},
	{126,252},
	{128,256},
	{129,258},
	{130,260},
	{132,263},
	{133,266},
	{135,269},
	{136,271},
	{138,274},
	{139,276},
	{141,279},
	{142,282},
	{144,284},
	{145,286},
	{147,289},
	{149,292},
	{150,294},
	{152,296},
	{154,299},
	{155,301},
	{157,303},
	{158,305},
	{161,309},
	{163,311},
	{166,315},
	{170,319},
	{172,322},
	{174,324},
	{176,326},
	{179,329},
	{182,332},
	{185,335},
	{188,338},
	{192,341},
	{195,344},
	{196,345},
	{199,347},
	{200,348},
	{202,350},
	{205,352},
	{207,354},
	{210,356},
	{211,357},
	{214,359},
	{217,361},
	{218,362},
	{221,364},
	{224,366},
	{231,370},
	{237,374},
	{246,379},
	{250,381},
	{252,382},
	{256,384},
	{260,386},
	{262,387},
	{273,392},
	{275,393},
	{280,395},
	{290,399},
	{306,405},
	{312,407},
	{321,410},
	{331,413},
	{345,417},
	{352,419},
	{360,421},
	{364,422},
	{372,424},
	{386,427},
	{400,430},
	{410,432},
	{420,434},
	{431,436},
	{437,437},
	{449,439},
	{468,442},
	{512,448},
};



static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0104, 0x01); 
	   
	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0x00);
  
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
    unsigned long flags;
	kal_uint16 realtime_fps = 0;
	
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 247 && realtime_fps <= 250)
			set_max_framerate(246,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
        else {
        // Extend frame length
 		write_cmos_sensor(0x0104, 0x01); 
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
        }
	} else {
		// Extend frame length
		write_cmos_sensor(0x0104, 0x01); 
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	// Update Shutter
	write_cmos_sensor(0x0104, 0x01);      	
    write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor(0x0203, shutter  & 0xFF);	
    write_cmos_sensor(0x0104, 0x00);    
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;
    LOG_INF("[IMX234MIPI]enter IMX234MIPIGain2Reg function\n");
    for (iI = 0; iI < IMX234MIPI_MaxGainIndex; iI++)
    {
        if(gain <= IMX234MIPI_sensorGainMapping[iI][0])
        {
            break;
        }
    }
    if(iI == IMX234MIPI_MaxGainIndex)
    {
        iI--;
    }
    if(gain != IMX234MIPI_sensorGainMapping[iI][0])
    {
         LOG_INF("Gain mapping don't correctly:%d %d \n", gain, IMX234MIPI_sensorGainMapping[iI][0]);
    }
    LOG_INF("exit IMX234MIPIGain2Reg function\n");
    return IMX234MIPI_sensorGainMapping[iI][1];
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain, org_gain, enable_dg = 0;

	if (gain < BASEGAIN || gain > 64 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 64 * BASEGAIN)
			gain = 64 * BASEGAIN;
	}

	if (gain > 8 * BASEGAIN) {
		LOG_INF("gain > 8 * BASEGAIN");
		org_gain = gain;
		gain = 8 * BASEGAIN;
		enable_dg = 1;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);

	if (enable_dg)
		imgsensor.gain = reg_gain + IMX234MIPI_MaxGainIndex;
	else
		imgsensor.gain = reg_gain;

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	if (enable_dg)
	{
		LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
		gain = org_gain / 8;
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
		write_cmos_sensor(0x0205, reg_gain & 0xFF);
		/* digital gain */
		write_cmos_sensor(0x020e, (gain>>6)& 0xF);
		write_cmos_sensor(0x020f, ((gain & 0x3F)<<2) & 0xFF);
		write_cmos_sensor(0x0210, (gain>>6)& 0xF);
		write_cmos_sensor(0x0211, ((gain & 0x3F)<<2) & 0xFF);
		write_cmos_sensor(0x0212, (gain>>6)& 0xF);
		write_cmos_sensor(0x0213, ((gain & 0x3F)<<2) & 0xFF);
		write_cmos_sensor(0x0214, (gain>>6)& 0xF);
		write_cmos_sensor(0x0215, ((gain & 0x3F)<<2) & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
		LOG_INF("digital gain = %d , high_gain = %d , low_gain = 0x%x\n ", gain, (gain>>6), ((gain & 0x3F)<<2));
		LOG_INF("digital mode gain = %d , reg_gain = 0x%x\n ", org_gain, imgsensor.gain);
		gain = org_gain;
	}
	else
	{
		LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
		write_cmos_sensor(0x0205, reg_gain & 0xFF);
		/* 1x digital gain */
		write_cmos_sensor(0x020e, 0x01);
		write_cmos_sensor(0x020f, 0x00);
		write_cmos_sensor(0x0210, 0x01);
		write_cmos_sensor(0x0211, 0x00);
		write_cmos_sensor(0x0212, 0x01);
		write_cmos_sensor(0x0213, 0x00);
		write_cmos_sensor(0x0214, 0x01);
		write_cmos_sensor(0x0215, 0x00);
		write_cmos_sensor(0x0104, 0x00);
	}

#if 0
	write_cmos_sensor(0x0104, 0x01);
    write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
    write_cmos_sensor(0x0104, 0x00);
#endif

	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);

	write_cmos_sensor(0x3820, 0x81);   //enable ihdr
 	
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
				// Extend frame length first
				write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
				write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3508, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}


#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0101, 0x00);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101, 0x01);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101, 0x10);	
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101, 0x11);
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}
#endif

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/
#endif

static void sensor_init(void)
{
	LOG_INF("IMX234_Sensor_Init_2lane E\n");
	
	write_cmos_sensor(0x0101,0x00);
	write_cmos_sensor(0x0136,0x18);
	write_cmos_sensor(0x0137,0x00);

	write_cmos_sensor(0x301A,0x01);
	write_cmos_sensor(0x3021,0x00);
	write_cmos_sensor(0x3024,0x00);
	write_cmos_sensor(0x3025,0x00);
	write_cmos_sensor(0x4128,0x00);
	write_cmos_sensor(0x4129,0x0F);
	write_cmos_sensor(0x412E,0x00);
	write_cmos_sensor(0x412F,0x0E);
	write_cmos_sensor(0x4159,0x5D);
	write_cmos_sensor(0x4550,0x02);
	write_cmos_sensor(0x45D0,0x01);
	write_cmos_sensor(0x4612,0x30);
	write_cmos_sensor(0x4619,0x38);
	write_cmos_sensor(0x461A,0x7C);
	write_cmos_sensor(0x461B,0x2A);
	write_cmos_sensor(0x463F,0x18);
	write_cmos_sensor(0x4653,0x29);
	write_cmos_sensor(0x4657,0x29);
	write_cmos_sensor(0x465B,0x28);
	write_cmos_sensor(0x465F,0x23);
	write_cmos_sensor(0x4667,0x22);
	write_cmos_sensor(0x466B,0x22);
	write_cmos_sensor(0x4673,0x1A);
	write_cmos_sensor(0x4904,0x00);
	write_cmos_sensor(0x4905,0xA6);
	write_cmos_sensor(0x4906,0x00);
	write_cmos_sensor(0x4907,0x8E);
	write_cmos_sensor(0x4908,0x01);
	write_cmos_sensor(0x4909,0x18);
	write_cmos_sensor(0x490A,0x00);
	write_cmos_sensor(0x490B,0xD7);
	write_cmos_sensor(0x4A5C,0x01);
	write_cmos_sensor(0x4A5D,0x0A);
	write_cmos_sensor(0x4A62,0x01);
	write_cmos_sensor(0x4A63,0x0A);
	write_cmos_sensor(0x4A72,0x00);
	write_cmos_sensor(0x4A73,0x8C);
	write_cmos_sensor(0x4A76,0x01);
	write_cmos_sensor(0x4A77,0x07);
	write_cmos_sensor(0x4A7A,0x01);
	write_cmos_sensor(0x4A7B,0x07);
	write_cmos_sensor(0x4A84,0x01);
	write_cmos_sensor(0x4A85,0x0A);
	write_cmos_sensor(0x6227,0x11);
	write_cmos_sensor(0x6283,0x03);
	write_cmos_sensor(0xA900,0x7F);
	write_cmos_sensor(0xA908,0x00);
	write_cmos_sensor(0xA909,0x3D);
	write_cmos_sensor(0xAE1C,0x05);
	write_cmos_sensor(0xAE1D,0x30);
	write_cmos_sensor(0xAE1E,0x04);
	write_cmos_sensor(0xAE1F,0xA4);
	write_cmos_sensor(0xAE20,0x04);
	write_cmos_sensor(0xAE21,0xA4);
	write_cmos_sensor(0xAE3A,0x05);
	write_cmos_sensor(0xAE3B,0x30);
	


}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF(" IMX234PreviewSetting_4lane enter\n");

	write_cmos_sensor(0x0100,0x00);
	
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x18);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x02);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x0A);
	write_cmos_sensor(0x034D,0x60);
	write_cmos_sensor(0x034E,0x05);
	write_cmos_sensor(0x034F,0xD8);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x0A);
	write_cmos_sensor(0x040D,0x60);
	write_cmos_sensor(0x040E,0x05);
	write_cmos_sensor(0x040F,0xD8);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x60);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xFA);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x0E);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);										  
	
	write_cmos_sensor(0x0100,0x01);
	
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("IMX234CaptureSetting_4lane enter! currefps:%d\n",currefps);
	if (currefps == 240) { //24fps for PIP
		write_cmos_sensor(0x0100,0x00);
		
		write_cmos_sensor(0x0114,0x03);
		write_cmos_sensor(0x0340,0x0C);
		write_cmos_sensor(0x0341,0x22);
		write_cmos_sensor(0x0342,0x18);
		write_cmos_sensor(0x0343,0x00);
		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
		write_cmos_sensor(0x0347,0x00);
		write_cmos_sensor(0x0348,0x14);
		write_cmos_sensor(0x0349,0xDF);
		write_cmos_sensor(0x034A,0x0B);
		write_cmos_sensor(0x034B,0xB7);
		write_cmos_sensor(0x0381,0x01);
		write_cmos_sensor(0x0383,0x01);
		write_cmos_sensor(0x0385,0x01);
		write_cmos_sensor(0x0387,0x01);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x11);
		write_cmos_sensor(0x0902,0x00);
		write_cmos_sensor(0x3029,0x00);
		write_cmos_sensor(0x305C,0x11);

		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x034C,0x14);
		write_cmos_sensor(0x034D,0xC0);
		write_cmos_sensor(0x034E,0x0B);
		write_cmos_sensor(0x034F,0xB0);
		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0404,0x00);
		write_cmos_sensor(0x0405,0x10);
		write_cmos_sensor(0x0408,0x00);
		write_cmos_sensor(0x0409,0x00);
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x14);
		write_cmos_sensor(0x040D,0xC0);
		write_cmos_sensor(0x040E,0x0B);
		write_cmos_sensor(0x040F,0xB0);

		write_cmos_sensor(0x0301,0x04);
		write_cmos_sensor(0x0303,0x02);
		write_cmos_sensor(0x0305,0x04);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0x9A);
		write_cmos_sensor(0x0309,0x0A);
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x0F);
		write_cmos_sensor(0x030E,0x02);
		write_cmos_sensor(0x030F,0xA3);
		write_cmos_sensor(0x0310,0x01);

		write_cmos_sensor(0x0820,0x10);
		write_cmos_sensor(0x0821,0xE0);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);

		write_cmos_sensor(0x0202,0x0C);
		write_cmos_sensor(0x0203,0x18);

		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x00);
		write_cmos_sensor(0x020E,0x01);
		write_cmos_sensor(0x020F,0x00);
		write_cmos_sensor(0x0210,0x01);
		write_cmos_sensor(0x0211,0x00);
		write_cmos_sensor(0x0212,0x01);
		write_cmos_sensor(0x0213,0x00);
		write_cmos_sensor(0x0214,0x01);
		write_cmos_sensor(0x0215,0x00);

		write_cmos_sensor(0x0100,0x01);
		
	} else{ // for 30fps need ti update
		write_cmos_sensor(0x0100,0x00);
		
		write_cmos_sensor(0x0114,0x03);
		write_cmos_sensor(0x0340,0x0C);
		write_cmos_sensor(0x0341,0x22);
		write_cmos_sensor(0x0342,0x18);
		write_cmos_sensor(0x0343,0x00);
		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
		write_cmos_sensor(0x0347,0x00);
		write_cmos_sensor(0x0348,0x14);
		write_cmos_sensor(0x0349,0xDF);
		write_cmos_sensor(0x034A,0x0B);
		write_cmos_sensor(0x034B,0xB7);
		write_cmos_sensor(0x0381,0x01);
		write_cmos_sensor(0x0383,0x01);
		write_cmos_sensor(0x0385,0x01);
		write_cmos_sensor(0x0387,0x01);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x11);
		write_cmos_sensor(0x0902,0x00);
		write_cmos_sensor(0x3029,0x00);
		write_cmos_sensor(0x305C,0x11);
		
		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x034C,0x14);
		write_cmos_sensor(0x034D,0xE0);
		write_cmos_sensor(0x034E,0x0B);
		write_cmos_sensor(0x034F,0xB8);
		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0404,0x00);
		write_cmos_sensor(0x0405,0x10);
		write_cmos_sensor(0x0408,0x00);
		write_cmos_sensor(0x0409,0x00);
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x14);
		write_cmos_sensor(0x040D,0xE0);
		write_cmos_sensor(0x040E,0x0B);
		write_cmos_sensor(0x040F,0xB8);
		
		write_cmos_sensor(0x0301,0x04);
		write_cmos_sensor(0x0303,0x02);
		write_cmos_sensor(0x0305,0x04);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0xBF);
		write_cmos_sensor(0x0309,0x0A);
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x0F);
		write_cmos_sensor(0x030E,0x03);
		write_cmos_sensor(0x030F,0x39);
		write_cmos_sensor(0x0310,0x01);
		
		write_cmos_sensor(0x0820,0x14);
		write_cmos_sensor(0x0821,0xA0);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);
		
		write_cmos_sensor(0x0202,0x0C);
		write_cmos_sensor(0x0203,0x18);
		
		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x00);
		write_cmos_sensor(0x020E,0x01);
		write_cmos_sensor(0x020F,0x00);
		write_cmos_sensor(0x0210,0x01);
		write_cmos_sensor(0x0211,0x00);
		write_cmos_sensor(0x0212,0x01);
		write_cmos_sensor(0x0213,0x00);
		write_cmos_sensor(0x0214,0x01);
		write_cmos_sensor(0x0215,0x00);

		write_cmos_sensor(0x0100,0x01);
	} 
	
		
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting Enter! currefps:%d\n",currefps);
	write_cmos_sensor(0x0100,0x00);
		
	write_cmos_sensor(0x0114,0x03);
#ifndef THERMAL_AND_POWER_TEST
	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x18);
#else
	write_cmos_sensor(0x0340,0x0C);
	write_cmos_sensor(0x0341,0x22);
#endif
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
#ifndef THERMAL_AND_POWER_TEST
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x02);
#else
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
#endif
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
#ifndef THERMAL_AND_POWER_TEST
	write_cmos_sensor(0x034C,0x0A);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x05);
	write_cmos_sensor(0x034F,0xDC);
#else
	write_cmos_sensor(0x034C,0x14);
	write_cmos_sensor(0x034D,0xE0);
	write_cmos_sensor(0x034E,0x0B);
	write_cmos_sensor(0x034F,0xB8);
#endif
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
#ifndef THERMAL_AND_POWER_TEST
	write_cmos_sensor(0x040C,0x0A);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x05);
	write_cmos_sensor(0x040F,0xDC);
#else
	write_cmos_sensor(0x040C,0x14);
	write_cmos_sensor(0x040D,0xE0);
	write_cmos_sensor(0x040E,0x0B);
	write_cmos_sensor(0x040F,0xB8);
#endif

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
#ifndef THERMAL_AND_POWER_TEST
	write_cmos_sensor(0x0307,0x60);
#else
	write_cmos_sensor(0x0307,0xBF);
#endif
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
#ifndef THERMAL_AND_POWER_TEST
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xFA);
#else
	write_cmos_sensor(0x030E,0x03);
	write_cmos_sensor(0x030F,0x39);
#endif
	write_cmos_sensor(0x0310,0x01);

#ifndef THERMAL_AND_POWER_TEST
	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x40);
#else
	write_cmos_sensor(0x0820,0x14);
	write_cmos_sensor(0x0821,0xA0);
#endif
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

#ifndef THERMAL_AND_POWER_TEST
	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x0E);
#else
	write_cmos_sensor(0x0202,0x0C);
	write_cmos_sensor(0x0203,0x18);
#endif

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x0100,0x01);
}


static void hs_video_setting(void)
{ 
	LOG_INF("hs_video_setting enter!\n");
	write_cmos_sensor(0x0100,0x00);

//VGA 120fps
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x02);
	write_cmos_sensor(0x0341,0x44);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x02);
	write_cmos_sensor(0x0347,0x18);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x09);
	write_cmos_sensor(0x034B,0x97);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x44);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x02);
	write_cmos_sensor(0x034D,0x80);
	write_cmos_sensor(0x034E,0x01);
	write_cmos_sensor(0x034F,0xE0);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x01);
	write_cmos_sensor(0x0409,0x5C);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x02);
	write_cmos_sensor(0x040D,0x80);
	write_cmos_sensor(0x040E,0x01);
	write_cmos_sensor(0x040F,0xE0);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x8F);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xFA);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x02);
	write_cmos_sensor(0x0203,0x3A);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x0100,0x01);
}


static void slim_video_setting(void)
{
	LOG_INF("slim_video_setting enter!\n");
	write_cmos_sensor(0x0100,0x00);
	
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x18);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x0A);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x05);
	write_cmos_sensor(0x034F,0xDC);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x0A);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x05);
	write_cmos_sensor(0x040F,0xDC);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x60);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xFA);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x0E);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);										  
	
	write_cmos_sensor(0x0100,0x01);

}



/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id =  ((read_cmos_sensor(0x0016)&0x0F)<<8) + read_cmos_sensor(0x0017);
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, i2c write id: 0x%x id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	//kal_uint8 i = 0;
	//kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0; 

	//LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	get_imgsensor_id(&sensor_id);
#if 0
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id =  ((read_cmos_sensor(0x0016)&0x0F)<<8) + read_cmos_sensor(0x0017);
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
#endif
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x4C00;
	imgsensor.gain = 0x0200;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	
	
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 250) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 246;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x0601,0x02);
	} else {
		write_cmos_sensor(0x0601,0x00);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		case SENSOR_FEATURE_SET_GAIN:		
			set_gain((UINT16) *feature_data_16);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_16);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = (BOOL)*feature_data_16;
            imgsensor.ihdr_en = KAL_FALSE;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%lld\n", *feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		
			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;	  
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
            break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX234_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	IMX234_MIPI_RAW_SensorInit	*/
