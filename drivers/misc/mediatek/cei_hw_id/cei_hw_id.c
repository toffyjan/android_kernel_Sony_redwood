#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
//#include <mt-plat/mt_gpio.h>
//#include <mt-plat/mt_gpio_core.h>
#include <linux/gpio.h>
#include "cei_hw_id.h"
#include <linux/platform_device.h>

struct hwid_info {
	enum cei_hw_type hw_id;
	enum cei_project_type project_id;
	enum cei_lte_type lte_id;
	enum cei_sim_type sim_id;
	enum cei_rf_type rf_id;
	enum cei_customer_project_type customer_project_id;
};

static struct hwid_info cei_hwid_info;

#ifdef CONFIG_SONY_S1_SUPPORT
static unsigned int s1_bl_unlocked = 0;
#endif

/*** @OWNER NEED TO DEFINE ***/
static const char cei_hw_type_str[][CEI_HWID_STRING_LEN] = 
	{"EVT", "DVT1_1", "DVT1_2", "DVT2", "DVT3", "TP", "DVT4", "PVT"};

static const char customer_hw_type_str[][CEI_HWID_STRING_LEN] = 
	{"PDP", "DP1", "DP2", "SP","AP", "TP", "AP2", "PQ"};

static const char sm21_cei_hw_type_str[][CEI_HWID_STRING_LEN] = 
	{"EVT", "DVT1_1", "DVT1_2", "DVT2", "TP", "PVT", "MP"};

static const char sm21_customer_hw_type_str[][CEI_HWID_STRING_LEN] = 
	{"PDP", "SP1", "SP2", "AP", "TP", "PQ", "MP"};

static const char sm11l_cei_hw_type_str[][CEI_HWID_STRING_LEN] = 
	{"EVT", "DVT1_1", "DVT1_2", "DVT2", "TP", "PVT", "MP"};

static const char sm11l_customer_hw_type_str[][CEI_HWID_STRING_LEN] = 
	{"PDP", "SP1", "SP2", "AP", "TP", "PQ", "MP"};

static const char cei_project_type_str[][CEI_HWID_STRING_LEN] = 
{
	"BY57",      // 0 0 0 0
	"BY58",      // 0 0 0 1
	"BY59",      // 0 0 1 0
	"Unknow", // 0 0 1 1
	"Unknow", // 0 1 0 0
	"Unknow", // 0 1 0 1
	"Unknow", // 0 1 1 0
	"Unknow", // 0 1 1 1
	"BY66",      // 1 0 0 0 (8) 
	"Unknow", // 1 0 0 1
	"BY69",      // 1 0 1 0 (10)
	"Unknow", // 1 0 1 1 
	"BY67",      // 1 1 0 0 (12)
	"BY68",      // 1 1 0 1 (13)
	"Unknow", // 1 1 1 0
	"Unknow"  // 1 1 1 1
};

static const char sm21_project_type_str[][CEI_HWID_STRING_LEN] = 
{
	"BY86",      // 0 0 0 0 (0)
	"Unknow", // 0 0 0 1
	"BY88",      // 0 0 1 0 (2)
	"BY78",      // 0 0 1 1 (3)
	"Unknow", // 0 1 0 0
	"BY87",      // 0 1 0 1 (5)
	"Unknow", // 0 1 1 0
	"Unknow", // 0 1 1 1
	"Unknow", // 1 0 0 0
	"BY77",      // 1 0 0 1 (9)
	"BY78",      // 1 0 1 0 (10)
	"Unknow", // 1 0 1 1
	"BY76",      // 1 1 0 0 (12)
	"Unknow", // 1 1 0 1
	"Unknow", // 1 1 1 0
	"Unknow"  // 1 1 1 1
};

static const char sm11l_project_type_str[][CEI_HWID_STRING_LEN] = 
{
	"BY61",      // 0 0 0 0 (0)
	"Unknow", // 0 0 0 1
	"BY65",      // 0 0 1 0 (2)
	"Unknow",      // 0 0 1 1 (3)
	"Unknow", // 0 1 0 0
	"BY63",      // 0 1 0 1 (5)
	"Unknow", // 0 1 1 0
	"Unknow", // 0 1 1 1
	"Unknow", // 1 0 0 0
	"Unknow",      // 1 0 0 1 (9)
	"Unknow",      // 1 0 1 0 (10)
	"Unknow", // 1 0 1 1
	"BY62",      // 1 1 0 0 (12)
	"BY64", // 1 1 0 1
	"Unknow", // 1 1 1 0
	"Unknow"  // 1 1 1 1
};

static const char cei_lte_type_str[][CEI_HWID_STRING_LEN] = 
	{"CAT6","CAT4"};

static const char cei_sim_type_str[][CEI_HWID_STRING_LEN] = 
	{"SS","DS"};

static const char cei_rf_type_str[][CEI_HWID_STRING_LEN] = 
	{"GINA","APAC","REX"};

static const char cei_customer_project_type_str[][CEI_HWID_STRING_LEN] = 
	{"SM11","SM21","SM10N","SM11L"};

/**
  * SM11/21 HWID:
  *
  * HWID1 | HWID2 | HWID3 
  * GPIO90   GPIO89   GPIO58
  *
  * SM11: 000-001-010-011-100-101-110-111
  *            EVT-DVT1_1-DVT1_2-DVT2-DVT3-TP1-PVT-MP
  * SM21: 000
  *            EVT
  *
  * HWID4   | HWID8   | HWID9 | HWID10 |
  * GPIO59  | GPIO56  | GPIO57 | GPIO80 |
  * <-CAT->|<-SIM->| <-      SKU       -> |
  *
  * SM11: 0000-0001-0010-1000-1100-1101-1010
  * SM21: 0000-0101-1100-1001-1010
  *
  * HWID5 - GPIO60 - Distinguish SM11/SM21
  *
**/

static void  parse_projectid(void) {

	cei_hwid_info.lte_id = (cei_hwid_info.project_id & 0x8) >> 3;
	cei_hwid_info.sim_id = (cei_hwid_info.project_id & 0x4) >> 2;
	cei_hwid_info.rf_id= (cei_hwid_info.project_id & 0x3);

	if (cei_hwid_info.customer_project_id == CUSTOMER_PROJECT_SM21) {
		if (cei_hwid_info.project_id == CEI_PROJECT_BY78) {
			cei_hwid_info.lte_id = CEI_LTE_CAT4;  // change from 0 -> 1
			cei_hwid_info.rf_id = CEI_RF_REX;       // change from 3 -> 2
		}
		if (cei_hwid_info.project_id == CEI_PROJECT_BY78_EVT) {
			cei_hwid_info.project_id = CEI_PROJECT_BY78;
		}
	}

	printk(KERN_INFO "cei_lte_id=%d, cei_sim_id=%d, cei_rf_id=%d\n",
		cei_hwid_info.lte_id,
		cei_hwid_info.sim_id,
		cei_hwid_info.rf_id);
}

static int __init get_cei_hw_id_from_cmdline(char* cmdline)
{
	cei_hwid_info.hw_id = 0;
	if(cmdline == NULL || strlen(cmdline) == 0)
	{
		printk(KERN_WARNING"cei hw id is empty\n");
	}
	else
	{
		cei_hwid_info.hw_id = simple_strtoul(cmdline, NULL, 10);
	}
	printk(KERN_INFO "cei hw id = %d\n", cei_hwid_info.hw_id);

	return 0;
}
__setup("hwid=", get_cei_hw_id_from_cmdline);

static int __init get_cei_customer_proj_id_from_cmdline(char* cmdline)
{
	cei_hwid_info.customer_project_id = 0;
	if(cmdline == NULL || strlen(cmdline) == 0)
	{
		printk(KERN_WARNING "cei customer proj id is empty\n");
	}
	else
	{
		cei_hwid_info.customer_project_id = simple_strtoul(cmdline, NULL, 10);
	}
	printk(KERN_INFO "cei customer_project_id = %d\n", cei_hwid_info.customer_project_id);

	return 0;
}
__setup("customer_projid=", get_cei_customer_proj_id_from_cmdline);

static int __init get_cei_proj_id_from_cmdline(char* cmdline)
{
	cei_hwid_info.project_id = 0;
	if(cmdline == NULL || strlen(cmdline) == 0)
	{
		printk(KERN_WARNING "cei proj id is empty\n");
	}
	else
	{
		cei_hwid_info.project_id = simple_strtoul(cmdline, NULL, 10);
	}
	printk(KERN_INFO "cei project id = %d\n", cei_hwid_info.project_id);

	parse_projectid();
	return 0;
}
__setup("projectid=", get_cei_proj_id_from_cmdline);

/*
 * API to get CEI HWID information:
 *
 * get_cei_hw_id()-      return enum cei_hw_type
 * get_cei_project_id()- return enum cei_project_type
 *
 * Enum definition is defined in cei_hw_id.h
 */
enum cei_hw_type get_cei_hw_id(void)
{
	return cei_hwid_info.hw_id;
}
EXPORT_SYMBOL(get_cei_hw_id);

enum cei_project_type get_cei_project_id(void)
{
	return cei_hwid_info.project_id;
}
EXPORT_SYMBOL(get_cei_project_id);

enum cei_customer_project_type get_cei_customer_project_id(void)
{
	return cei_hwid_info.customer_project_id;
}
EXPORT_SYMBOL(get_cei_customer_project_id);

/* Below definition can be parse from project_id */
enum cei_lte_type get_cei_lte_id(void)
{
	return cei_hwid_info.lte_id;
}
EXPORT_SYMBOL(get_cei_lte_id);

enum cei_sim_type get_cei_sim_id(void)
{
	return cei_hwid_info.sim_id;
}
EXPORT_SYMBOL(get_cei_sim_id);

enum cei_rf_type get_cei_rf_id(void)
{
	return cei_hwid_info.rf_id;
}
EXPORT_SYMBOL(get_cei_rf_id);

#define FTM_PIN_GPIO 15
int get_ftm_pin(void)
{
	int ftm_pin = 0;

	printk("[FTM] get_ftm_pin E\n" );

	gpio_request(FTM_PIN_GPIO, "FTM_PIN_GPIO");
	gpio_direction_output(FTM_PIN_GPIO, 1);

	printk("[FTM] get_ftm_pin\n" );
	gpio_direction_input(FTM_PIN_GPIO);
	if (__gpio_get_value(FTM_PIN_GPIO) == 0)    ftm_pin= 1;
	gpio_free(FTM_PIN_GPIO);
	printk("[FTM] ftm_pin, %d\n", ftm_pin );

	return ftm_pin;
}

static int cei_hwid_info_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "projectid=%d hwid=%d lteid=%d simid=%d rfid=%d customer_projid=%d\n", 
				cei_hwid_info.project_id,
				cei_hwid_info.hw_id,
				cei_hwid_info.lte_id,
				cei_hwid_info.sim_id,
				cei_hwid_info.rf_id,
				cei_hwid_info.customer_project_id);
	return 0;
}

static int cei_ftm_pin_proc_show(struct seq_file *m, void *v)
{
	int ftm_pin;
	ftm_pin = get_ftm_pin();

	seq_printf(m, "%d\n", ftm_pin);

	return 0;
}

static int cei_hwid_info_string_proc_show(struct seq_file *m, void *v)
{
      if (cei_hwid_info.customer_project_id == CUSTOMER_PROJECT_SM11L)
      {
	seq_printf(m, "%s=%s %s=%s(%s) %s=%s %s=%s %s=%s %s=%s\n", 
				"projectid",  sm11l_project_type_str[cei_hwid_info.project_id], 
				"hwid",  sm11l_cei_hw_type_str[cei_hwid_info.hw_id] , 
					     sm11l_customer_hw_type_str[cei_hwid_info.hw_id],
				"lteid", cei_lte_type_str[cei_hwid_info.lte_id],
				"simid",cei_sim_type_str[cei_hwid_info.sim_id],
				"rfid",cei_rf_type_str[cei_hwid_info.rf_id],
				"customer_projid", cei_customer_project_type_str[cei_hwid_info.customer_project_id]);
      }
      else
      {
	seq_printf(m, "%s=%s %s=%s(%s) %s=%s %s=%s %s=%s %s=%s\n", 
				"projectid", cei_hwid_info.customer_project_id ? sm21_project_type_str[cei_hwid_info.project_id]:cei_project_type_str[cei_hwid_info.project_id], 
				"hwid", cei_hwid_info.customer_project_id ? sm21_cei_hw_type_str[cei_hwid_info.hw_id] : cei_hw_type_str[cei_hwid_info.hw_id], 
					    cei_hwid_info.customer_project_id ? sm21_customer_hw_type_str[cei_hwid_info.hw_id] : customer_hw_type_str[cei_hwid_info.hw_id],
				"lteid", cei_lte_type_str[cei_hwid_info.lte_id],
				"simid",cei_sim_type_str[cei_hwid_info.sim_id],
				"rfid",cei_rf_type_str[cei_hwid_info.rf_id],
				"customer_projid", cei_customer_project_type_str[cei_hwid_info.customer_project_id]);
      }
	return 0;
}

static int cci_hw_board_type_proc_show(struct seq_file *m, void *v)
{
      if (cei_hwid_info.customer_project_id == CUSTOMER_PROJECT_SM11L)
      {
	seq_printf(m, "%s(%s)\n", 
				sm11l_cei_hw_type_str[cei_hwid_info.hw_id], sm11l_customer_hw_type_str[cei_hwid_info.hw_id] );
      }
      else
      {
	seq_printf(m, "%s(%s)\n", 
				cei_hwid_info.customer_project_id ? sm21_cei_hw_type_str[cei_hwid_info.hw_id] : cei_hw_type_str[cei_hwid_info.hw_id], 
				cei_hwid_info.customer_project_id ? sm21_customer_hw_type_str[cei_hwid_info.hw_id] : customer_hw_type_str[cei_hwid_info.hw_id]);
      }
	return 0;
}

static int cei_hwid_info_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_hwid_info_proc_show, NULL);
}

static int cei_ftm_pin_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_ftm_pin_proc_show, NULL);
}

static int cei_hwid_info_string_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_hwid_info_string_proc_show, NULL);
}

static int cci_hw_board_type_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cci_hw_board_type_proc_show, NULL);
}

static const struct file_operations cei_hwid_info_string_proc_fops = {
	.open	= cei_hwid_info_string_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static const struct file_operations cei_hwid_info_proc_fops = {
	.open	= cei_hwid_info_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static const struct file_operations cei_ftm_pin_proc_fops = {
	.open	= cei_ftm_pin_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static const struct file_operations cci_hw_board_type_proc_fops = {
	.open	= cci_hw_board_type_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static const struct of_device_id cei_hw_id_of_match[] = {
	{ .compatible = "compal,cei_hwid", },
	{ },
};
MODULE_DEVICE_TABLE(of, cei_hw_id_of_match);

#ifdef CONFIG_SONY_S1_SUPPORT
static int __init get_s1_bl_unlocked_from_cmdline(char* cmdline)
{
	s1_bl_unlocked = 0;
	if(cmdline == NULL || strlen(cmdline) == 0)
	{
		printk(KERN_WARNING "s1 bl_unlocked is empty\n");
	}
	else
	{
		s1_bl_unlocked= (unsigned int)simple_strtoul(cmdline, NULL, 10);
	}
	printk(KERN_INFO "s1_bl_unlocked  = %d\n", s1_bl_unlocked);

	return 0;
}
__setup("s1_bl_unlocked=", get_s1_bl_unlocked_from_cmdline);

static int s1_bl_unlocked_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", s1_bl_unlocked);
	return 0;
}

static int s1_bl_unlocked_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, s1_bl_unlocked_proc_show, NULL);
}

static const struct file_operations s1_bl_unlocked_proc_fops = {
	.open	= s1_bl_unlocked_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};
#endif

static int cei_hw_id_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "cei_hw_id_probe\n");

	printk(KERN_INFO "cei_project_id=%d, customer_project_id=%d, cei_hw_id=%d\n",
				cei_hwid_info.project_id,
				cei_hwid_info.hw_id,
				cei_hwid_info.customer_project_id);

	printk(KERN_INFO "cei_hw_id_probe cei_lte_id=%d, cei_sim_id=%d, cei_rf_id=%d\n",
		cei_hwid_info.lte_id,
		cei_hwid_info.sim_id,
		cei_hwid_info.rf_id);

	/* Create proc node for userspace */
	proc_create("cei_hwid_info",0,NULL,&cei_hwid_info_proc_fops);
	proc_create("cei_hwid_info_string",0,NULL,&cei_hwid_info_string_proc_fops);
	proc_create("cei_ftm_pin",0,NULL,&cei_ftm_pin_proc_fops);
	proc_create("cci_hw_board_type",0,NULL,&cci_hw_board_type_proc_fops);

	#ifdef CONFIG_SONY_S1_SUPPORT
	proc_create("s1_bl_unlocked",0,NULL,&s1_bl_unlocked_proc_fops);
	#endif
	
	return 0;
}

static int cei_hw_id_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "CEI HWID remove\n");
	return 0;
}

static struct platform_driver cei_hw_id_driver = {
	.probe      = cei_hw_id_probe,
	.remove     = cei_hw_id_remove,
	.driver = {
		.name = "cei-hwid-driver",
		.owner = THIS_MODULE,
		.of_match_table = cei_hw_id_of_match,
	},
};

static int __init cei_hw_id_init(void)
{
	printk(KERN_INFO "cei_hw_id_init\n");
	return platform_driver_register(&cei_hw_id_driver);
}
static void __exit cei_hw_id_exit(void)
{
	printk(KERN_INFO "cei_hw_id_exit\n");
	platform_driver_unregister(&cei_hw_id_driver);
}

module_init(cei_hw_id_init);
module_exit(cei_hw_id_exit);

MODULE_DESCRIPTION("cei hardware ID driver");
MODULE_LICENSE("GPL");
