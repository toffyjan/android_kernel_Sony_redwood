#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/ctype.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ptrace.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/ioctl.h>
#include <asm/cputime.h>
#include <asm/io.h>
#include <linux/aio.h>
#include <linux/vmalloc.h>
//#include <mach/msm_iomap.h>
//#include <mach/subsystem_restart.h>
//#include <mach/mt_reg_base.h>
#include <linux/cciklog.h>
#ifdef IMEM_CERT_RECORD
#include <linux/ccistuff.h>
#endif // #ifdef IMEM_CERT_RECORD
#ifdef CCI_HW_ID
#include <../drivers/misc/mediatek/cei_hw_id/cei_hw_id.h>
#endif // #ifdef CCI_HW_ID
#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
#include <soc/qcom/smem.h>
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM

/*******************************************************************************
* Local Variable/Structure Declaration
*******************************************************************************/


#define KLOG_VERSION		"1.15.0.0"
#define KLOG_VERSION_HEX	0x010F0000

//make sure the category is able to write
#define CHK_CATEGORY(x) \
				do\
				{\
					if(unlikely(x < KLOG_KERNEL || x >= KLOG_IGNORE))\
					{\
						kprintk("%s:%s():%u --> Func Caller:<%p> sent an invalid KLog category:%u\n",\
							__FILE__, __func__, __LINE__,\
							__builtin_return_address(0),\
							x );\
						return;/* FIXME: Kevin_Chiang@CCI: Too coupling coding style */\
					}\
				}\
				while(0)

#define MSM_KLOG_HEADER		MSM_KLOG_BASE
#define MSM_KLOG_MAGIC		(MSM_KLOG_HEADER + KLOG_SIGNATURE_HEADER_LENGTH)
#define MSM_KLOG_MAIN		(MSM_KLOG_HEADER + CCI_KLOG_HEADER_SIZE)
#define MSM_KLOG_FOOTER		((char*)MSM_KLOG_MAIN + KLOG_CATEGORY_TOTAL_SIZE)

#ifdef CCI_KLOG_SUPPORT_ATTRIBUTE
#if 0
#define KLOG_RO_DEV_ATTR(name)	static DEVICE_ATTR(name, S_IRUSR, klog_show_##name, NULL)
#define KLOG_WO_DEV_ATTR(name)	static DEVICE_ATTR(name, S_IWUSR, NULL, klog_store_##name)
#define KLOG_RW_DEV_ATTR(name)	static DEVICE_ATTR(name, S_IRUSR | S_IWUSR, klog_show_##name, klog_store_##name)
#else
#define KLOG_RO_DEV_ATTR(name)	static DEVICE_ATTR(name, 0444, klog_show_##name, NULL)
#define KLOG_WO_DEV_ATTR(name)	static DEVICE_ATTR(name, 0222, NULL, klog_store_##name)
#define KLOG_RW_DEV_ATTR(name)	static DEVICE_ATTR(name, 0666, klog_show_##name, klog_store_##name)
#endif
#endif // #ifdef CCI_KLOG_SUPPORT_ATTRIBUTE

//Taylor CCIKlog-->B
/**
 * struct logger_log - represents a specific log, such as 'main' or 'radio'
 * @buffer:	The actual ring buffer
 * @misc:	The "misc" device representing the log
 * @wq:		The wait queue for @readers
 * @readers:	This log's readers
 * @mutex:	The mutex that protects the @buffer
 * @w_off:	The current write head offset
 * @head:	The head, or location that readers start reading at.
 * @size:	The size of the log
 * @logs:	The list of log channels
 *
 * This structure lives from module insertion until module removal, so it does
 * not need additional reference counting. The structure is protected by the
 * mutex 'mutex'.
 */
struct logger_log {
	unsigned char		*buffer;
	struct miscdevice	misc;
	wait_queue_head_t	wq;
	struct list_head	readers;
	struct mutex		mutex;
	size_t			w_off;
	size_t			head;
	size_t			size;
	struct list_head	logs;
};

struct iovec {
    const void*  iov_base;
    size_t       iov_len;
};

struct iov_iter {
	int type;
	size_t iov_offset;
	size_t count;
	const struct iovec *iov;
	unsigned long nr_segs;
};

static LIST_HEAD(log_list);

/**
 * struct logger_reader - a logging device open for reading
 * @log:	The associated log
 * @list:	The associated entry in @logger_log's list
 * @r_off:	The current read head offset.
 * @r_all:	Reader can read all entries
 * @r_ver:	Reader ABI version
 * @missing_bytes: android log missing warning
 *
 * This object lives from open to release, so we don't need additional
 * reference counting. The structure is protected by log->mutex.
 */
struct logger_reader {
	struct logger_log	*log;
	struct list_head	list;
	size_t			r_off;
	bool			r_all;
	int			r_ver;
    size_t      missing_bytes;
    size_t      wake_up_interval;          /* how many bytes does writer wake up reader.  0-android original, others-mtk used*/
    long long   wake_up_timer;      /* the timer to wake up reader unit:ns*/
};

/* logger_offset - returns index 'n' into the log via (optimized) modulus */
static size_t logger_offset(struct logger_log *log, size_t n)
{
	return n & (log->size - 1);
}
//Taylor CCIKlog<--E

static struct klog_magic_list kml[] =
{
	{KLOG_INDEX_FORCE_CLEAR, KLOG_MAGIC_FORCE_CLEAR, KLOG_PRIORITY_FORCE_CLEAR},
	{KLOG_INDEX_INIT, KLOG_MAGIC_INIT, KLOG_PRIORITY_KLOG_INIT},
	{KLOG_INDEX_MARM_FATAL, KLOG_MAGIC_MARM_FATAL, KLOG_PRIORITY_MARM_FATAL},
	{KLOG_INDEX_AARM_PANIC, KLOG_MAGIC_AARM_PANIC, KLOG_PRIORITY_AARM_PANIC},
	{KLOG_INDEX_RPM_CRASH, KLOG_MAGIC_RPM_CRASH, KLOG_PRIORITY_RPM_CRASH},
	{KLOG_INDEX_SUBSYS_CRASH, KLOG_MAGIC_SUBSYS_CRASH, KLOG_PRIORITY_SUBSYS_CRASH},
	{KLOG_INDEX_FIQ_HANG, KLOG_MAGIC_FIQ_HANG, KLOG_PRIORITY_FIQ_HANG},
	{KLOG_INDEX_UNKNOWN_CRASH, KLOG_MAGIC_UNKNOWN_CRASH, KLOG_PRIORITY_UNKNOWN_CRASH},
	{KLOG_INDEX_DOWNLOAD_MODE, KLOG_MAGIC_DOWNLOAD_MODE, KLOG_PRIORITY_DOWNLOAD_MODE},
	{KLOG_INDEX_POWER_OFF, KLOG_MAGIC_POWER_OFF, KLOG_PRIORITY_POWER_OFF},
	{KLOG_INDEX_REBOOT, KLOG_MAGIC_REBOOT, KLOG_PRIORITY_NATIVE_COMMAND},
	{KLOG_INDEX_BOOTLOADER, KLOG_MAGIC_BOOTLOADER, KLOG_PRIORITY_NATIVE_COMMAND},
	{KLOG_INDEX_RECOVERY, KLOG_MAGIC_RECOVERY, KLOG_PRIORITY_NATIVE_COMMAND},
	{KLOG_INDEX_OEM_COMMAND, KLOG_MAGIC_OEM_COMMAND, KLOG_PRIORITY_OEM_COMMAND},
};

static struct klog_category *pklog_category[] =
{
#if CCI_KLOG_CRASH_SIZE
	(struct klog_category*)MSM_KLOG_MAIN,
#endif // #if CCI_KLOG_CRASH_SIZE
#if CCI_KLOG_APPSBL_SIZE
	(struct klog_category*)	(MSM_KLOG_MAIN + CCI_KLOG_CRASH_SIZE),
#endif // #if CCI_KLOG_APPSBL_SIZE
#if CCI_KLOG_KERNEL_SIZE
	(struct klog_category*)	(MSM_KLOG_MAIN + CCI_KLOG_CRASH_SIZE + CCI_KLOG_APPSBL_SIZE),
#endif // #if CCI_KLOG_KERNEL_SIZE
#if CCI_KLOG_ANDROID_MAIN_SIZE
	(struct klog_category*)	(MSM_KLOG_MAIN + CCI_KLOG_CRASH_SIZE + CCI_KLOG_APPSBL_SIZE + CCI_KLOG_KERNEL_SIZE),
#endif // #if CCI_KLOG_ANDROID_MAIN_SIZE
#if CCI_KLOG_ANDROID_SYSTEM_SIZE
	(struct klog_category*)	(MSM_KLOG_MAIN + CCI_KLOG_CRASH_SIZE + CCI_KLOG_APPSBL_SIZE + CCI_KLOG_KERNEL_SIZE + CCI_KLOG_ANDROID_MAIN_SIZE),
#endif // #if CCI_KLOG_ANDROID_SYSTEM_SIZE
#if CCI_KLOG_ANDROID_RADIO_SIZE
	(struct klog_category*)	(MSM_KLOG_MAIN + CCI_KLOG_CRASH_SIZE + CCI_KLOG_APPSBL_SIZE + CCI_KLOG_KERNEL_SIZE + CCI_KLOG_ANDROID_MAIN_SIZE + CCI_KLOG_ANDROID_SYSTEM_SIZE),
#endif // #if CCI_KLOG_ANDROID_RADIO_SIZE
#if CCI_KLOG_ANDROID_EVENTS_SIZE
	(struct klog_category*)	(MSM_KLOG_MAIN + CCI_KLOG_CRASH_SIZE + CCI_KLOG_APPSBL_SIZE + CCI_KLOG_KERNEL_SIZE + CCI_KLOG_ANDROID_MAIN_SIZE + CCI_KLOG_ANDROID_SYSTEM_SIZE + CCI_KLOG_ANDROID_RADIO_SIZE),
#endif // #if CCI_KLOG_ANDROID_EVENTS_SIZE
};

static struct system_information sysinfo;
#ifdef CCI_KLOG_SUPPORT_CCI_ENGMODE
struct system_information *psysinfo = &sysinfo;
#endif // #ifdef CCI_KLOG_SUPPORT_CCI_ENGMODE
#ifdef IMEM_CERT_RECORD
static unsigned int *cci_imem_magic = (void*)MSM_IMEM_BASE + CCI_IMEM_OFFSET;
#endif // #ifdef IMEM_CERT_RECORD
#ifdef CCI_KLOG_SBL_BOOT_TIME_USE_IMEM
static unsigned int *sbl_bootup_time = (void*)MSM_IMEM_BASE + CCI_IMEM_OFFSET_SBL_BOOT_TIME;
#endif // #ifdef CCI_KLOG_SBL_BOOT_TIME_USE_IMEM
static struct klog_time system_bootup_time;
static struct klog_time system_shutdown_time;
unsigned int system_suspend_time = 0;
unsigned int system_resume_time = 0;
struct timespec suspend_timestamp;
struct timespec resume_timestamp;
int suspend_resume_state = 0;

static char *klog_header = (void *)MSM_KLOG_HEADER;	//klog header
static char *klog_magic = (void *)MSM_KLOG_MAGIC;	//klog state magic word
static int mem_ready = 0;				//0: memory is not ready, 1: memory is ready
static int mem_have_clean = 0;				//0: memory is not clean, 1: memory is clean
static int magic_priority = KLOG_PRIORITY_INVALID;
static int crash_state = CRASH_STATE_INIT;		//0: not crash, 0x01: crashing, 0x02: previous crash
static int rtc_synced = 0;

#ifdef CCI_HW_ID
static int device_info_update = 0;
#endif // #ifdef CCI_HW_ID

#ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION
static char rpm_version[KLOG_RPM_VERSION_LENGTH] = {0};
#endif // #ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION
static char bootloader_version[KLOG_BOOTLOADER_VERSION_LENGTH] = {0};

//for fault/exception record
#if CCI_KLOG_CRASH_SIZE
static int fault_log_level = 3;
static int fault_level = FAULT_LEVEL_INIT;
static int fault_type = FAULT_TYPE_INIT;
static char fault_msg[256] = {0};
static int kernel_log_level = -1;
#endif // #if CCI_KLOG_CRASH_SIZE

#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
#ifdef MSM_SHARED_RAM_BASE
#define MSM_SHARED_RAM_PHYS	0x0FA00000
static char *modem_log = (void *)MSM_SHARED_RAM_BASE + (MSM_SHARED_RAM_SIZE - 1024 * 2);
#else // #ifdef MSM_SHARED_RAM_BASE
static char *modem_log;
#endif // #ifdef MSM_SHARED_RAM_BASE
static struct mem_area smem_info;
static unsigned long modem_log_addr = 0;
static int modem_log_addr_inited = 0;
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM

static char previous_reboot_magic[KLOG_MAGIC_LENGTH + 1] = {0};
static int previous_normal_boot = -1;
#ifdef CONFIG_WARMBOOT_UNDEFINED
static unsigned int warmboot = CONFIG_WARMBOOT_UNDEFINED;
#else // #ifdef CONFIG_WARMBOOT_UNDEFINED
static unsigned int warmboot = 0xFFFFFFFF;
#endif // #ifdef CONFIG_WARMBOOT_UNDEFINED
static unsigned int startup = 0;
static int unknowncrashflag_inited = 0;
#ifdef CCI_KLOG_ALLOW_FORCE_PANIC
static int force_panic_when_suspend = 0;
static int force_panic_when_power_off = 0;
#endif // #ifdef CCI_KLOG_ALLOW_FORCE_PANIC

/*******************************************************************************
* Local Function Declaration
*******************************************************************************/

/*******************************************************************************
* External Variable/Structure Declaration
*******************************************************************************/

/*******************************************************************************
* External Function Declaration
*******************************************************************************/

/*** Functions ***/

#ifdef CCI_KLOG_ALLOW_FORCE_PANIC
int get_force_panic_when_suspend(void)
{
	return force_panic_when_suspend;
}

int get_force_panic_when_power_off(void)
{
	return force_panic_when_power_off;
}
#endif // #ifdef CCI_KLOG_ALLOW_FORCE_PANIC

#if CCI_KLOG_CRASH_SIZE
static const char *fault_level_str[] = { "NONE", "panic", "die/bug", "data abort", "prefetch abort", "subsys", "watchdog", "data abort(aarch64)" };
//level: 0x10:enable/disable, 0x01: panic, 0x02: die/bug, 0x03: data abort, 0x04: prefetch abort, 0x05: subsystem fatal error, 0x06: watchdog, 0x07: data abort(aarch64)
//type: depend on level, data abort: refer to fsr_info[], prefetch abort: ifsr_info[], subsystem: restart level
//msg: message from panic or die, subsystem name
int get_fault_state(void)
{
	return fault_level;
}

void set_fault_state(int level, int type, const char* msg)
{
#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
	int log_size = 0;
	int log_ok = 0;
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM

	if(level == FAULT_LEVEL_INIT)//reset the fault
	{
		fault_level = FAULT_LEVEL_INIT;
		fault_type = FAULT_TYPE_INIT;
		fault_msg[0] = 0;
		pklog_category[KLOG_CRASH]->index = 0;
		pklog_category[KLOG_CRASH]->overload = 0;
		memset(&pklog_category[KLOG_CRASH]->buffer[0], 0, CCI_KLOG_CRASH_SIZE - KLOG_CATEGORY_HEADER_SIZE);
	}
	else if(level > FAULT_LEVEL_INIT)//record the fault to crash(start)
	{
		if(fault_level <= FAULT_LEVEL_INIT)//allow to record
		{
			fault_level = level;
			fault_type = type;
			strncpy(fault_msg, msg, strlen(msg));
			kprintk("fault_level=0x%X(%s), fault_type=%d, fault_msg=%s\n", fault_level, fault_level_str[fault_level], fault_type, fault_msg);
#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
			if(fault_level == FAULT_LEVEL_SUBSYSTEM && strcmp(fault_msg, "modem") == 0)//modem subsystem
			{
//modem log address which provided by RIL
				if(modem_log_addr_inited == 1)
				{
					if(modem_log_addr >= smem_info.phys_addr && modem_log_addr < (smem_info.phys_addr + smem_info.size))//modem dynamic allocated physical address
					{
						kprintk("modem crash log address:0x%lX\n", modem_log_addr);
						modem_log = smem_info.virt_addr + (modem_log_addr - smem_info.phys_addr);
					}
					else if(modem_log_addr < smem_info.size)//offset
					{
						kprintk("modem crash log address offset:0x%lX\n", modem_log_addr);
						modem_log = smem_info.virt_addr + modem_log_addr;
					}
					else//invalid address
					{
						kprintk("invalid modem crash log address:0x%lX\n", modem_log_addr);
						modem_log_addr_inited = 0;
					}

					if(modem_log_addr_inited == 1 && strlen(modem_log) > 0)
					{
						log_ok = 1;
					}
				}

//modem log address from VENDOR1 by kernel
				if(modem_log_addr_inited == 0 || log_ok == 0)
				{
					modem_log = smem_get_entry(SMEM_ID_VENDOR1, &log_size, 0, SMEM_ANY_HOST_FLAG);
					if(IS_ERR_OR_NULL(modem_log))
					{
						kprintk("get modem crash log address failed\n");
					}
					else
					{
						kprintk("modem crash log address:0x%lX, size:%d\n", (unsigned long)modem_log, log_size);

						if(strlen(modem_log) > 0)
						{
							log_ok = 1;
						}
					}
				}

//default modem log address
				if(log_ok == 0)
				{
					kprintk("modem crash log address not available, try default address\n");
					modem_log = smem_info.virt_addr + (smem_info.size - 1024 * 2);

					if(strlen(modem_log) > 0)
					{
						log_ok = 1;
					}
				}

				if(log_ok > 0)
				{
					kprintk("modem crash log:%s\n", modem_log);
				}
				else
				{
					kprintk("modem crash log is empty\n");
				}
			}
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
		}
		else
		{
			kprintk("fault already exists:0x%X, ignore:0x%X\n", fault_level, level);
		}
	}
	else if(level < FAULT_LEVEL_INIT)//record the fault to crash(end)
	{
		if(fault_level > FAULT_LEVEL_INIT)//recording
		{
			fault_level |= FAULT_LEVEL_EXIST;
			kprintk("fault_level=0x%X, fault_type=%d, fault_msg=%s\n", fault_level, fault_type, fault_msg);
		}
		else
		{
			kprintk("fault invalid, ignore:0x%X\n", level);
		}
	}

	return;
}
#endif // #if CCI_KLOG_CRASH_SIZE

#ifdef CONFIG_CCI_KLOG_RECORD_KERNEL_TIMESTAMP
struct timespec klog_get_kernel_rtc_timestamp(const struct timespec *log_time)
{
	struct timespec current_time;

	if(log_time)
	{
		current_time.tv_sec = log_time->tv_sec;
		current_time.tv_nsec = log_time->tv_nsec;
	}
	else
	{
		current_time = current_kernel_time();
	}

	return current_time;
}

struct timespec klog_get_kernel_clock_timestamp(void)
{
	struct timespec current_time;
	unsigned long long now_clock;
	unsigned long now_clock_ns;

	now_clock = cpu_clock(UINT_MAX);
	now_clock_ns = do_div(now_clock, 1000000000);
	current_time.tv_sec = (time_t)now_clock;
	current_time.tv_nsec = (long)now_clock_ns;

	return current_time;
}

struct klog_time klog_record_kernel_timestamp(const struct timespec *log_time)
{
	struct klog_time current_time;

	current_time.clock = klog_get_kernel_clock_timestamp();
	current_time.rtc = klog_get_kernel_rtc_timestamp(log_time);

	if(mem_ready == 1)
	{
//update crash_state first
		update_priority();

		if((crash_state & CRASH_STATE_PREVIOUS) == 0)//only allow to record if not previous crash
		{
//record kernel clock time(uptime)
			snprintf(klog_magic + KLOG_MAGIC_TOTAL_LENGTH, KLOG_KERNEL_TIME_LENGTH, "[%08lX.%08lX]", (unsigned long) current_time.clock.tv_sec, current_time.clock.tv_nsec);

//record kernel first RTC sync time
			if(rtc_synced == 0 && current_time.clock.tv_sec > 30)
			{
				rtc_synced = 1;
				snprintf(klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH, KLOG_FIRST_RTC_TIMESTAMP_LENGTH, "[%08lX.%08lX]", current_time.rtc.tv_sec, current_time.rtc.tv_nsec);
			}

//record kernel RTC time
			snprintf(klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH, KLOG_LAST_RTC_TIMESTAMP_LENGTH, "[%08lX.%08lX]", current_time.rtc.tv_sec, current_time.rtc.tv_nsec);
		}
	}

	return current_time;
}
#endif // #ifdef CONFIG_CCI_KLOG_RECORD_KERNEL_TIMESTAMP

#ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION
void klog_record_rpm_version(const char *str)
{
	strncpy(rpm_version, str, KLOG_RPM_VERSION_LENGTH - 1);

	return;
}
EXPORT_SYMBOL(klog_record_rpm_version);
#endif // #ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION

static __inline__ void __cklc_append_char(unsigned int category, char c)
{
	if(mem_ready == 1)
	{
//update crash_state first
		update_priority();
	}

	if(mem_ready == 0 || (crash_state & CRASH_STATE_PREVIOUS) > 0)//not allow to record if previous crash
	{
		return;
	}

	if(!mem_have_clean)
	{
#if CCI_KLOG_CRASH_SIZE
		memset(&pklog_category[KLOG_CRASH]->name[0], 0, CCI_KLOG_CRASH_SIZE);
		snprintf(pklog_category[KLOG_CRASH]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_CRASH);
		pklog_category[KLOG_CRASH]->size = CCI_KLOG_CRASH_SIZE;
#endif // #if CCI_KLOG_CRASH_SIZE
#if CCI_KLOG_APPSBL_SIZE
		if(strncmp(pklog_category[KLOG_APPSBL]->name, KLOG_CATEGORY_NAME_APPSBL, strlen(KLOG_CATEGORY_NAME_APPSBL)) != 0
		|| (int)pklog_category[KLOG_APPSBL]->index >= (int)(CCI_KLOG_APPSBL_SIZE - KLOG_CATEGORY_HEADER_SIZE))//category name not match or index over than valid size, that means garbage, so we clean it
		{
			memset(&pklog_category[KLOG_APPSBL]->name[0], 0, CCI_KLOG_APPSBL_SIZE);
			snprintf(pklog_category[KLOG_APPSBL]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_APPSBL);
			pklog_category[KLOG_APPSBL]->size = CCI_KLOG_APPSBL_SIZE;
		}
#endif // #if CCI_KLOG_APPSBL_SIZE
		memset(&pklog_category[KLOG_KERNEL]->name[0], 0, KLOG_CATEGORY_TOTAL_SIZE - CCI_KLOG_CRASH_SIZE - CCI_KLOG_APPSBL_SIZE);
#if CCI_KLOG_KERNEL_SIZE
		snprintf(pklog_category[KLOG_KERNEL]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_KERNEL);
		pklog_category[KLOG_KERNEL]->size = CCI_KLOG_KERNEL_SIZE;
#endif // #if CCI_KLOG_KERNEL_SIZE
#if CCI_KLOG_ANDROID_MAIN_SIZE
		snprintf(pklog_category[KLOG_ANDROID_MAIN]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_ANDROID_MAIN);
		pklog_category[KLOG_ANDROID_MAIN]->size = CCI_KLOG_ANDROID_MAIN_SIZE;
#endif // #if CCI_KLOG_ANDROID_MAIN_SIZE
#if CCI_KLOG_ANDROID_SYSTEM_SIZE
		snprintf(pklog_category[KLOG_ANDROID_SYSTEM]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_ANDROID_SYSTEM);
		pklog_category[KLOG_ANDROID_SYSTEM]->size = CCI_KLOG_ANDROID_SYSTEM_SIZE;
#endif // #if CCI_KLOG_ANDROID_SYSTEM_SIZE
#if CCI_KLOG_ANDROID_RADIO_SIZE
		snprintf(pklog_category[KLOG_ANDROID_RADIO]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_ANDROID_RADIO);
		pklog_category[KLOG_ANDROID_RADIO]->size = CCI_KLOG_ANDROID_RADIO_SIZE;
#endif // #if CCI_KLOG_ANDROID_RADIO_SIZE
#if CCI_KLOG_ANDROID_EVENTS_SIZE
		snprintf(pklog_category[KLOG_ANDROID_EVENTS]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_ANDROID_EVENTS);
		pklog_category[KLOG_ANDROID_EVENTS]->size = CCI_KLOG_ANDROID_EVENTS_SIZE;
#endif // #if CCI_KLOG_ANDROID_EVENTS_SIZE
#if (CCI_KLOG_HEADER_SIZE + KLOG_CATEGORY_TOTAL_SIZE) + KLOG_CATEGORY_NAME_LENGTH < CCI_KLOG_SIZE
		snprintf(MSM_KLOG_FOOTER, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_RESERVE);
#endif // #if (CCI_KLOG_HEADER_SIZE + KLOG_CATEGORY_TOTAL_SIZE) + KLOG_CATEGORY_NAME_LENGTH < CCI_KLOG_SIZE

		mem_have_clean = 1;
	}
	pklog_category[category]->buffer[pklog_category[category]->index++] = c;
	if((int)pklog_category[category]->index >= (int)(pklog_category[category]->size - KLOG_CATEGORY_HEADER_SIZE))
	{
		if(pklog_category[category]->overload < 0xFF)
		{
			pklog_category[category]->overload++;
		}
		pklog_category[category]->index = 0;
	}

	return;
}

#if CCI_KLOG_CRASH_SIZE
//This function should only be called in printk.c
void set_kernel_log_level(int level)
{
	kernel_log_level = level;

	return;
}
#endif // #if CCI_KLOG_CRASH_SIZE

//Append a character into KLog[category]
//This function should only be called in printk.c
void cklc_append_kernel_raw_char(char c)
{
	__cklc_append_char(KLOG_KERNEL, c);
#if CCI_KLOG_CRASH_SIZE
	if(fault_level > 0 && fault_level < 0x10 && kernel_log_level >= 0 && kernel_log_level <= fault_log_level)
	{
		__cklc_append_char(KLOG_CRASH, c);
	}
#endif // #if CCI_KLOG_CRASH_SIZE

	return;
}

static __inline__ void __cklc_append_str_extend(unsigned int category, char *str, size_t len, int ignore_crash)
{
	int i = 0;
	char *c = NULL;

	for(i = 0; i < len; i++)
	{
		c = str + i;
		if(isprint(*c))
		{
			if(category == KLOG_KERNEL && ignore_crash == 0)
			{
				cklc_append_kernel_raw_char(*c);
			}
			else
			{
				__cklc_append_char(category, *c);
			}
		}
	}

	return;
}

static __inline__ void __cklc_append_str(unsigned int category, char *str, size_t len)
{
	__cklc_append_str_extend(category, str, len, 0);

	return;
}

//Append a string into KLog[category]
//This function should only be called in printk.c
void cklc_append_str(const char *str, size_t len)
{
	__cklc_append_str(KLOG_KERNEL, (char *)str, len);

	return;
}

static __inline__ void __cklc_append_newline(unsigned int category)
{
	__cklc_append_char(category, '\n');

	return;
}

//Append a new line character('\n') into KLog[category]
//This function should only be called in printk.c
void cklc_append_newline(void)
{
	cklc_append_kernel_raw_char('\n');

	return;
}

static __inline__ void __cklc_append_separator(unsigned int category)
{
	const char *buf = "| ";

	__cklc_append_str(category, (char *)buf, strlen(buf));

	return;
}

//Append a separator symbol '| ' into KLog[category]
//This function should only be called in printk.c
void cklc_append_separator(void)
{
	__cklc_append_separator(KLOG_KERNEL);

	return;
}

static __inline__ void __cklc_append_time_header(unsigned int category, struct timespec *log_time)
{
	char buf[32] = {0};
	unsigned len = 0;
	struct klog_time now;

#ifdef CONFIG_CCI_KLOG_RECORD_KERNEL_TIMESTAMP
	now = klog_record_kernel_timestamp(log_time);
#else // #ifdef CONFIG_CCI_KLOG_RECORD_KERNEL_TIMESTAMP
	if(log_time)
	{
		now.rtc.tv_sec = log_time->tv_sec;
		now.rtc.tv_nsec = log_time->tv_nsec;
	}
	else
	{
		now.rtc = current_kernel_time();
	}
#endif // #ifdef CONFIG_CCI_KLOG_RECORD_KERNEL_TIMESTAMP

	len = snprintf(buf, sizeof(buf), "[%8lx.%08lx] ", now.rtc.tv_sec, now.rtc.tv_nsec);

	__cklc_append_str_extend(category, buf, len, 1);

	return;
}

//Append a Unix Epoch timestamp as the line header
//This function should only be called in printk.c
void cklc_append_time_header(void)
{
	__cklc_append_time_header(KLOG_KERNEL, NULL);

	return;
}


//For Android Logger
//This function should only be called in logger.c
void cklc_append_android_log(unsigned int category,
				const struct logger_entry *header,
				const char *log_buf,
				size_t log_size,
				unsigned int priority,
				size_t tag_index,
				size_t tag_size,
				size_t msg_index,
				size_t msg_size)
{
	int len = 0;
	char buf[17] = {0};
	struct timespec log_time;

	log_time.tv_sec = header->sec;
	log_time.tv_nsec = header->nsec;
	__cklc_append_time_header(category, &log_time);

	len = snprintf(buf, sizeof(buf), "{%X,%X} <%u> ", (unsigned int)header->pid, (unsigned int)header->tid, priority);
	__cklc_append_str(category, buf, len);

	if((tag_index + tag_size) > log_size)
	{
		__cklc_append_str(category, (char *)log_buf + tag_index, log_size - tag_index);
		tag_size = tag_size - (log_size - tag_index);
		tag_index = 0;
	}
	__cklc_append_str(category, (char *)log_buf + tag_index, tag_size);
	__cklc_append_separator(category);

	if((msg_index + msg_size) > log_size)
	{
		__cklc_append_str(category, (char *)log_buf + msg_index, log_size - msg_index);
		msg_size = msg_size - (log_size - msg_index);
		msg_index = 0;
	}
	__cklc_append_str(category, (char *)log_buf + msg_index, msg_size);
	__cklc_append_newline(category);

	return;
}

int get_magic_index(char *magic)
{
	char buf[KLOG_MAGIC_LENGTH + 1] = {0};

	int i = 0;

	if(magic && strlen(magic) > 0)
	{
		strncpy(buf, magic, KLOG_MAGIC_LENGTH);

		for(i = 0; i < sizeof(kml) / sizeof(struct klog_magic_list); i++)
		{
			if(!strncmp(magic, kml[i].name, strlen(kml[i].name)))
			{
				return kml[i].index;
			}
		}
	}

	return KLOG_INDEX_INVALID;
}

int get_magic_priority(char *magic)
{
	char buf[KLOG_MAGIC_LENGTH + 1] = {0};

	int i = 0;

	if(magic && strlen(magic) > 0)
	{
		strncpy(buf, magic, KLOG_MAGIC_LENGTH);

		for(i = 0; i < sizeof(kml) / sizeof(struct klog_magic_list); i++)
		{
			if(!strncmp(magic, kml[i].name, strlen(kml[i].name)))
			{
				return kml[i].priority;
			}
		}
	}

	return KLOG_PRIORITY_INVALID;
}

int match_crash_priority(int priority)
{
	if(priority == KLOG_PRIORITY_MARM_FATAL || priority == KLOG_PRIORITY_AARM_PANIC || priority == KLOG_PRIORITY_RPM_CRASH || priority == KLOG_PRIORITY_SUBSYS_CRASH || priority == KLOG_PRIORITY_FIQ_HANG || priority == KLOG_PRIORITY_UNKNOWN_CRASH)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void update_priority(void)
{
	if(unknowncrashflag_inited == 0)
	{
		unknowncrashflag_inited = 1;
#ifdef STUFF_CRASH_DEFAULT
		crashflag = *(unsigned int *)(klog_header + CCI_KLOG_SIZE - sizeof(int) * 2);
		*(unsigned int *)(klog_header + CCI_KLOG_SIZE - sizeof(int) * 2) = STUFF_CRASH_DEFAULT;
#endif // #ifdef STUFF_CRASH_DEFAULT
#ifdef CONFIG_WARMBOOT_CRASH
		unknownrebootflag = *(unsigned int *)(klog_header + CCI_KLOG_SIZE - sizeof(int) * 4);
		*(unsigned int *)(klog_header + CCI_KLOG_SIZE - sizeof(int) * 4) = CONFIG_WARMBOOT_CRASH;
#endif // #ifdef CONFIG_WARMBOOT_CRASH

#ifdef CONFIG_WARMBOOT_CRASH
		if((warmboot == CONFIG_WARMBOOT_CRASH || (warmboot != CONFIG_WARMBOOT_UNDEFINED && warmboot != 0 && (crashflag == CONFIG_WARMBOOT_CRASH || unknownrebootflag == CONFIG_WARMBOOT_CRASH)))
#else // #ifdef CONFIG_WARMBOOT_CRASH
		if(warmboot == 0xC0DEDEAD
#endif // #ifdef CONFIG_WARMBOOT_CRASH
			&& match_crash_priority(get_magic_priority(klog_magic)) == 0)//crash happened before, but klog magic was not recorded as crashed, so overwrite the klog magic to unknown crash
		{
			strncpy(klog_magic, KLOG_MAGIC_UNKNOWN_CRASH, KLOG_MAGIC_LENGTH);
		}
	}

	if(magic_priority == KLOG_PRIORITY_INVALID)
	{
		if(klog_magic && strlen(klog_magic) > 0)
		{
			magic_priority = get_magic_priority(klog_magic);
//check if previous crash
			if(match_crash_priority(magic_priority) > 0)
			{
				crash_state = crash_state | CRASH_STATE_PREVIOUS;
			}
		}
	}
}

void cklc_save_magic(char *magic, int state)
{
	char value[KLOG_STATE_LENGTH + 1] = {0};
	char buf[KLOG_MAGIC_TOTAL_LENGTH] = {0};
	int priority = KLOG_PRIORITY_INVALID;
	int magic_update = 0;

	if(klog_magic && strlen(klog_magic) > 0)
	{
		strncpy(buf, klog_magic, KLOG_MAGIC_TOTAL_LENGTH - 1);
		kprintk("klog_magic=%s\n", buf);
	}

	if(magic_priority == KLOG_PRIORITY_INVALID)
	{
		update_priority();
	}

	if(state > 0xFFF || state < KLOG_STATE_INIT)
	{
		kprintk("Ignore invalid state %d\n", state);
		state = KLOG_STATE_NONE;
	}
	if(state == KLOG_STATE_INIT)
	{
		strncpy(value, KLOG_STATE_INIT_CODE, KLOG_STATE_LENGTH);
	}
	else
	{
		snprintf(value, KLOG_STATE_LENGTH, "%03X", state);
	}

	if(magic && strlen(magic) > 0)
	{
		strncpy(buf, magic, KLOG_MAGIC_LENGTH);
		strncpy(buf + KLOG_MAGIC_LENGTH, klog_magic + KLOG_MAGIC_LENGTH, KLOG_STATE_LENGTH);
#ifdef CCI_KLOG_DETAIL_LOG
		kprintk_set_magic("prepare", magic, value, buf);
#endif // #ifdef CCI_KLOG_DETAIL_LOG

		priority = get_magic_priority(buf);

		if(match_crash_priority(priority) > 0)//mARM fatal or aARM panic is happening, i.e., crashing
		{
			crash_state = crash_state | CRASH_STATE_CRASHING;
		}
	}
	else
	{
		priority = KLOG_PRIORITY_INVALID;
	}

#ifdef CCI_KLOG_DETAIL_LOG
	kprintk("magic_priority=%d, priority=%d, state=%d\n", magic_priority, priority, state);
#endif // #ifdef CCI_KLOG_DETAIL_LOG

//magic
	if(warmboot == 0 || magic_priority == KLOG_PRIORITY_INVALID || !strncmp(klog_magic, KLOG_MAGIC_POWER_OFF, KLOG_MAGIC_LENGTH))//cold-boot
	{
		state = 0;
#ifdef CONFIG_WARMBOOT_NORMAL
		warmboot = CONFIG_WARMBOOT_NORMAL;
#else // #ifdef CONFIG_WARMBOOT_NORMAL
		warmboot = 0x77665501;
#endif // #ifdef CONFIG_WARMBOOT_NORMAL
		if(magic_priority == KLOG_PRIORITY_INVALID)//invalid magic, init klog with default magic
		{
			magic_update = 1;
			magic_priority = KLOG_PRIORITY_KLOG_INIT;
			snprintf(buf, KLOG_MAGIC_TOTAL_LENGTH, "%s%s", KLOG_MAGIC_INIT, KLOG_STATE_INIT_CODE);
			kprintk_set_magic("cold:invalid", KLOG_MAGIC_INIT, KLOG_STATE_INIT_CODE, buf);
		}
		else//valid magic, init klog with specified magic
		{
			magic_update = 1;
			magic_priority = priority;
			strncpy(buf + KLOG_MAGIC_LENGTH, KLOG_STATE_INIT_CODE, KLOG_STATE_LENGTH);
			kprintk_set_magic("cold:valid", magic, KLOG_STATE_INIT_CODE, buf);
		}
	}
	else//warm-boot
	{
		if(priority == KLOG_PRIORITY_INVALID)//invalid magic, do not update magic
		{
			strncpy(buf, klog_magic, KLOG_MAGIC_LENGTH + KLOG_STATE_LENGTH);
			kprintk_set_magic("warm:invalid", KLOG_MAGIC_NONE, value, buf);
		}
		else//valid magic
		{
			if(!strncmp(buf, KLOG_MAGIC_FORCE_CLEAR, KLOG_MAGIC_LENGTH))//force clear
			{
				state = 0;
				if((crash_state & CRASH_STATE_CRASHING) > 0)//not allow force clear magic if crashing
				{
					strncpy(buf, klog_magic, KLOG_MAGIC_LENGTH + KLOG_STATE_LENGTH);
					kprintk_set_magic("!force", magic, value, buf);
				}
				else
				{
					magic_update = 1;
					crash_state = CRASH_STATE_INIT;
					magic_priority = KLOG_PRIORITY_KLOG_INIT;
					snprintf(buf, KLOG_MAGIC_TOTAL_LENGTH, "%s%s", KLOG_MAGIC_INIT, KLOG_STATE_INIT_CODE);
					kprintk_set_magic("force", KLOG_MAGIC_INIT, KLOG_STATE_INIT_CODE, buf);
				}
			}
			else if(!strncmp(buf, KLOG_MAGIC_INIT, KLOG_MAGIC_LENGTH))//klog init
			{
				state = 0;
				if(crash_state == CRASH_STATE_INIT)//only allow to clear if not any crash
				{
					magic_update = 1;
					magic_priority = KLOG_PRIORITY_KLOG_INIT;
					snprintf(buf, KLOG_MAGIC_TOTAL_LENGTH, "%s%s", KLOG_MAGIC_INIT, KLOG_STATE_INIT_CODE);
					kprintk_set_magic("init", KLOG_MAGIC_INIT, KLOG_STATE_INIT_CODE, buf);
				}
				else
				{
					strncpy(buf, klog_magic, KLOG_MAGIC_LENGTH + KLOG_STATE_LENGTH);
					kprintk_set_magic("!init", magic, value, buf);
				}
			}
			else
			{
				if(priority < magic_priority)//higher priority magic, update magic
				{
					magic_update = 1;
					magic_priority = priority;
					kprintk_set_magic("higher", magic, value, buf);
				}
				else//lower or same priority magic, do not update magic
				{
					strncpy(buf, klog_magic, KLOG_MAGIC_LENGTH + KLOG_STATE_LENGTH);
					kprintk_set_magic("lower", magic, value, buf);
				}
			}
		}
	}

//state
	if(state != 0)
	{
		if(state == KLOG_STATE_INIT)
		{
			if(crash_state == CRASH_STATE_INIT)//only allow to clear if not any crash
			{
				strncpy(buf + KLOG_MAGIC_LENGTH, KLOG_STATE_INIT_CODE, KLOG_STATE_LENGTH);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk_set_magic("init", magic, KLOG_STATE_INIT_CODE, buf);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
			else//crash, do not update state
			{
				strncpy(buf + KLOG_MAGIC_LENGTH, klog_magic + KLOG_MAGIC_LENGTH, KLOG_STATE_LENGTH);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk_set_magic("crash", magic, KLOG_STATE_INIT_CODE, buf);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
		}
		else
		{
			if(magic_priority != KLOG_PRIORITY_INVALID)//klog inited, the state should be a valid value
			{
				state = state | simple_strtoul(buf + KLOG_MAGIC_LENGTH, NULL, 10);
			}

			state = state & 0xFFF;
			snprintf(buf + KLOG_MAGIC_LENGTH, KLOG_STATE_LENGTH + 1, "%03X", state);
#ifdef CCI_KLOG_DETAIL_LOG
			kprintk_set_magic("update", magic, value, buf);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
		}
	}

//update
	snprintf(klog_magic, KLOG_MAGIC_TOTAL_LENGTH, "%s", buf);
	kprintk("new klog_magic:klog_magic=%s, buf=%s\n", klog_magic, buf);

//hw_id
#ifdef CCI_HW_ID
	if(magic_update == 1 && device_info_update == 0)
	{
		device_info_update = 1;
		snprintf(klog_magic + KLOG_INFO_LENGTH + KLOG_IMAGE_INFO_LENGTH, KLOG_HW_ID_LENGTH, "%02X", get_cei_hw_id());
		kprintk("hw_info:hw_id=%02X\n", get_cei_hw_id());
	}
#endif // #ifdef CCI_HW_ID

#ifdef CONFIG_CCI_KLOG_RECORD_KERNEL_TIMESTAMP
	klog_record_kernel_timestamp(NULL);
#endif // #ifdef CONFIG_CCI_KLOG_RECORD_KERNEL_TIMESTAMP
}
EXPORT_SYMBOL(cklc_save_magic);

void cklc_set_memory_ready(void)
{
	mem_ready = 1;
}
EXPORT_SYMBOL(cklc_set_memory_ready);

//Reinitialize categories for collecting logs again
void clear_klog(void)
{
//Backup KLOG status
	int org_mem_ready = mem_ready;
	int org_mem_have_clean = 1;

	int i = 0;

	mem_ready = 0;//temporary change to uninitialized

//reset crash state
	cklc_save_magic(KLOG_MAGIC_FORCE_CLEAR, KLOG_STATE_INIT);
	set_fault_state(FAULT_LEVEL_INIT, FAULT_TYPE_NONE, "");
	crash_state = CRASH_STATE_INIT;

//Clear Each KLOG Buffer
	for(i = 0; i < KLOG_IGNORE; i++)
	{
		switch(i)
		{
#if CCI_KLOG_CRASH_SIZE
			case KLOG_CRASH:
				snprintf(pklog_category[KLOG_CRASH]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_CRASH);
				pklog_category[KLOG_CRASH]->size = CCI_KLOG_CRASH_SIZE;
				break;
#endif // #if CCI_KLOG_CRASH_SIZE
#if CCI_KLOG_APPSBL_SIZE
			case KLOG_APPSBL:
				snprintf(pklog_category[KLOG_APPSBL]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_APPSBL);
				pklog_category[KLOG_APPSBL]->size = CCI_KLOG_APPSBL_SIZE;
				break;
#endif // #if CCI_KLOG_APPSBL_SIZE
#if CCI_KLOG_KERNEL_SIZE
			case KLOG_KERNEL:
				snprintf(pklog_category[KLOG_KERNEL]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_KERNEL);
				pklog_category[KLOG_KERNEL]->size = CCI_KLOG_KERNEL_SIZE;
				break;
#endif // #if CCI_KLOG_KERNEL_SIZE
#if CCI_KLOG_ANDROID_MAIN_SIZE
			case KLOG_ANDROID_MAIN:
				snprintf(pklog_category[KLOG_ANDROID_MAIN]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_ANDROID_MAIN);
				pklog_category[KLOG_ANDROID_MAIN]->size = CCI_KLOG_ANDROID_MAIN_SIZE;
				break;
#endif // #if CCI_KLOG_ANDROID_MAIN_SIZE
#if CCI_KLOG_ANDROID_SYSTEM_SIZE
			case KLOG_ANDROID_SYSTEM:
				snprintf(pklog_category[KLOG_ANDROID_SYSTEM]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_ANDROID_SYSTEM);
				pklog_category[KLOG_ANDROID_SYSTEM]->size = CCI_KLOG_ANDROID_SYSTEM_SIZE;
				break;
#endif // #if CCI_KLOG_ANDROID_SYSTEM_SIZE
#if CCI_KLOG_ANDROID_RADIO_SIZE
			case KLOG_ANDROID_RADIO:
				snprintf(pklog_category[KLOG_ANDROID_RADIO]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_ANDROID_RADIO);
				pklog_category[KLOG_ANDROID_RADIO]->size = CCI_KLOG_ANDROID_RADIO_SIZE;
				break;
#endif // #if CCI_KLOG_ANDROID_RADIO_SIZE
#if CCI_KLOG_ANDROID_EVENTS_SIZE
			case KLOG_ANDROID_EVENTS:
				snprintf(pklog_category[KLOG_ANDROID_EVENTS]->name, KLOG_CATEGORY_NAME_LENGTH, KLOG_CATEGORY_NAME_ANDROID_EVENTS);
				pklog_category[KLOG_ANDROID_EVENTS]->size = CCI_KLOG_ANDROID_EVENTS_SIZE;
				break;
#endif // #if CCI_KLOG_ANDROID_EVENTS_SIZE
			default:
				break;
		}
#if CCI_KLOG_APPSBL_SIZE
		if(i == KLOG_APPSBL)//skip APPSBL
		{
			continue;
		}
#endif // #if CCI_KLOG_APPSBL_SIZE
		pklog_category[i]->index = 0;
		pklog_category[i]->overload = 0;
		memset(&pklog_category[i]->buffer[0], 0, pklog_category[i]->size - KLOG_CATEGORY_HEADER_SIZE);
	}

//Restore KLOG status
	mem_have_clean = org_mem_have_clean;
	mem_ready = org_mem_ready;

	kprintk("clear all categories done\n");

	return;
}

void record_suspend_resume_time(int state, unsigned int time)
{
	if(state == 1)//suspending
	{
		sysinfo.suspend_time = system_suspend_time;
	}
	else if(state == 3)//resuming
	{
		sysinfo.resume_time = system_resume_time;
	}
	else//other
	{
		kprintk("invalid state %d for record suspend/resume time\n", state);
		return;
	}

//update suspend/resume time to klog header area
	memcpy(klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH + KLOG_NORMAL_BOOT_LENGTH + KLOG_SYSTEM_ON_OFF_STATE_LENGTH + KLOG_SBL_BOOTUP_TIME_LENGTH + KLOG_ABOOT_BOOTUP_TIME_LENGTH + KLOG_ANDROID_BOOTUP_TIME_LENGTH + KLOG_ANDROID_SHUTDOWN_TIME_LENGTH + KLOG_SYSFS_SYNC_TIME_LENGTH + KLOG_KERNEL_POWER_OFF_TIME_LENGTH, &sysinfo.suspend_time, KLOG_SUSPEND_TIME_LENGTH + KLOG_RESUME_TIME_LENGTH);

	return;
}

void record_shutdown_time(int state)
{
	struct klog_time current_time;

	switch(state % 0x10)
	{
		case 1://user confirmed to power-off
			if(sysinfo.system_on_off_state / 0x10 > 0)
			{
				sysinfo.system_on_off_state = state;
				system_shutdown_time = klog_record_kernel_timestamp(NULL);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk("sysinfo(shutdown:0x%X):timestamp=%u.%u\n", sysinfo.system_on_off_state, (unsigned int)system_shutdown_time.clock.tv_sec, (unsigned int)system_shutdown_time.clock.tv_nsec);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
			else
			{
				kprintk("sysinfo(shutdown:0x%X):invalid state 0x%X\n", sysinfo.system_on_off_state, state);
			}
			break;

		case 2://perform low-level power-off
			if(sysinfo.system_on_off_state == 0x01)
			{
				sysinfo.system_on_off_state = state;
				current_time = klog_record_kernel_timestamp(NULL);
//compute Android shutdown time
				sysinfo.android_shutdown_time = (unsigned int)((current_time.clock.tv_sec - system_shutdown_time.clock.tv_sec) * 1000 + (current_time.clock.tv_nsec - system_shutdown_time.clock.tv_nsec) / 1000000);//ms
//update Android shutdown time to klog header area
				memcpy(klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH + KLOG_NORMAL_BOOT_LENGTH, &sysinfo.system_on_off_state, KLOG_SYSTEM_ON_OFF_STATE_LENGTH + KLOG_SBL_BOOTUP_TIME_LENGTH + KLOG_ABOOT_BOOTUP_TIME_LENGTH + KLOG_ANDROID_BOOTUP_TIME_LENGTH + KLOG_ANDROID_SHUTDOWN_TIME_LENGTH + KLOG_SYSFS_SYNC_TIME_LENGTH + KLOG_KERNEL_POWER_OFF_TIME_LENGTH);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk("sysinfo(shutdown:0x%X):timestamp=%u.%u, android_shutdown_time=%u\n", sysinfo.system_on_off_state, (unsigned int)current_time.clock.tv_sec, (unsigned int)current_time.clock.tv_nsec, sysinfo.android_shutdown_time);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
			else
			{
				kprintk("sysinfo(shutdown:0x%X):invalid state 0x%X\n", sysinfo.system_on_off_state, state);
			}
			break;

		case 3://start sync, emergency remount
			if(sysinfo.system_on_off_state == 0x01)//in case that property_set is not fast enough
			{
				sysinfo.system_on_off_state = 0x02;
				current_time = klog_record_kernel_timestamp(NULL);
//compute Android shutdown time
				sysinfo.android_shutdown_time = (unsigned int)((current_time.clock.tv_sec - system_shutdown_time.clock.tv_sec) * 1000 + (current_time.clock.tv_nsec - system_shutdown_time.clock.tv_nsec) / 1000000);//ms
//update Android shutdown time to klog header area
				memcpy(klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH + KLOG_NORMAL_BOOT_LENGTH, &sysinfo.system_on_off_state, KLOG_SYSTEM_ON_OFF_STATE_LENGTH + KLOG_SBL_BOOTUP_TIME_LENGTH + KLOG_ABOOT_BOOTUP_TIME_LENGTH + KLOG_ANDROID_BOOTUP_TIME_LENGTH + KLOG_ANDROID_SHUTDOWN_TIME_LENGTH + KLOG_SYSFS_SYNC_TIME_LENGTH + KLOG_KERNEL_POWER_OFF_TIME_LENGTH);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk("sysinfo(shutdown:0x%X):timestamp=%u.%u, android_shutdown_time=%u\n", sysinfo.system_on_off_state, (unsigned int)current_time.clock.tv_sec, (unsigned int)current_time.clock.tv_nsec, sysinfo.android_shutdown_time);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
			if(sysinfo.system_on_off_state / 0x10 > 0 || sysinfo.system_on_off_state == 0x02)
			{
				sysinfo.system_on_off_state = state;
				current_time = klog_record_kernel_timestamp(NULL);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk("sysinfo(shutdown:0x%X):timestamp=%u.%u\n", sysinfo.system_on_off_state, (unsigned int)current_time.clock.tv_sec, (unsigned int)current_time.clock.tv_nsec);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
			else
			{
				kprintk("sysinfo(shutdown:0x%X):invalid state 0x%X\n", sysinfo.system_on_off_state, state);
			}
			break;

		case 4://sync, emergency remount done
			if(sysinfo.system_on_off_state == 0x03)
			{
				sysinfo.system_on_off_state = state;
				current_time = klog_record_kernel_timestamp(NULL);
//compute fs sync time(sync = system - android)
				sysinfo.sysfs_sync_time = (unsigned int)((current_time.clock.tv_sec - system_shutdown_time.clock.tv_sec) * 1000 + (current_time.clock.tv_nsec - system_shutdown_time.clock.tv_nsec) / 1000000) - sysinfo.android_shutdown_time;//ms
//update fs sync time to klog header area
				memcpy(klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH + KLOG_NORMAL_BOOT_LENGTH, &sysinfo.system_on_off_state, KLOG_SYSTEM_ON_OFF_STATE_LENGTH + KLOG_SBL_BOOTUP_TIME_LENGTH + KLOG_ABOOT_BOOTUP_TIME_LENGTH + KLOG_ANDROID_BOOTUP_TIME_LENGTH + KLOG_ANDROID_SHUTDOWN_TIME_LENGTH + KLOG_SYSFS_SYNC_TIME_LENGTH + KLOG_KERNEL_POWER_OFF_TIME_LENGTH);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk("sysinfo(shutdown:0x%X):timestamp=%u.%u, sysfs_sync_time=%u\n", sysinfo.system_on_off_state, (unsigned int)current_time.clock.tv_sec, (unsigned int)current_time.clock.tv_nsec, sysinfo.sysfs_sync_time);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
			else
			{
				kprintk("sysinfo(shutdown:0x%X):invalid state 0x%X\n", sysinfo.system_on_off_state, state);
			}
			break;

		case 5://kernel reboot
			if(sysinfo.system_on_off_state == 0x04)
			{
				sysinfo.system_on_off_state = state;
				current_time = klog_record_kernel_timestamp(NULL);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk("sysinfo(shutdown:0x%X):timestamp=%u.%u\n", sysinfo.system_on_off_state, (unsigned int)current_time.clock.tv_sec, (unsigned int)current_time.clock.tv_nsec);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
			else
			{
				kprintk("sysinfo(shutdown:0x%X):invalid state 0x%X\n", sysinfo.system_on_off_state, state);
			}
			break;

		case 6://soc power-off
			if(sysinfo.system_on_off_state == 0x05)
			{
				sysinfo.system_on_off_state = state;
				current_time = klog_record_kernel_timestamp(NULL);
//compute kernel power-off time(kernel = system - android - sync)
				system_shutdown_time.clock.tv_sec = current_time.clock.tv_sec - system_shutdown_time.clock.tv_sec;
				system_shutdown_time.clock.tv_nsec = current_time.clock.tv_nsec - system_shutdown_time.clock.tv_nsec;
				sysinfo.kernel_power_off_time = (unsigned int)(system_shutdown_time.clock.tv_sec * 1000 + system_shutdown_time.clock.tv_nsec / 1000000) - sysinfo.android_shutdown_time - sysinfo.sysfs_sync_time;//ms
//update kernel power-off time to klog header area
				memcpy(klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH + KLOG_NORMAL_BOOT_LENGTH, &sysinfo.system_on_off_state, KLOG_SYSTEM_ON_OFF_STATE_LENGTH + KLOG_SBL_BOOTUP_TIME_LENGTH + KLOG_ABOOT_BOOTUP_TIME_LENGTH + KLOG_ANDROID_BOOTUP_TIME_LENGTH + KLOG_ANDROID_SHUTDOWN_TIME_LENGTH + KLOG_SYSFS_SYNC_TIME_LENGTH + KLOG_KERNEL_POWER_OFF_TIME_LENGTH);
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk("sysinfo(shutdown:0x%X):timestamp=%u.%u, kernel_power_off_time=%u, system_shutdown_time=%u\n", sysinfo.system_on_off_state, (unsigned int)current_time.clock.tv_sec, (unsigned int)current_time.clock.tv_nsec, sysinfo.kernel_power_off_time, (unsigned int)(system_shutdown_time.clock.tv_sec * 1000 + system_shutdown_time.clock.tv_nsec / 1000000));
#endif // #ifdef CCI_KLOG_DETAIL_LOG
			}
			else
			{
				kprintk("sysinfo(shutdown:0x%X):invalid state 0x%X\n", sysinfo.system_on_off_state, state);
			}
			break;

		default:
			kprintk("sysinfo(shutdown:0x%X):invalid state 0x%X\n", sysinfo.system_on_off_state, state);
			break;
	}
}

static long klog_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int flag = 0;
	unsigned int version = 0;
	struct system_information si;
#ifdef CCI_KLOG_ALLOW_FORCE_PANIC
	char magic[KLOG_MAGIC_LENGTH + 1] = {0};
#endif // #ifdef CCI_KLOG_ALLOW_FORCE_PANIC
#ifdef CCI_KLOG_SUPPORT_RESTORATION
	char *klog_buffer = NULL;
#endif // #ifdef CCI_KLOG_SUPPORT_RESTORATION

	switch(cmd)
	{
		case KLOG_IOCTL_CHECK_OLD_LOG:
			flag = get_magic_index(klog_magic);
			if(copy_to_user((void *)arg, &flag ,sizeof(int)))
			{
				return -EFAULT;
			}
			break;

		case KLOG_IOCTL_CLEAR_LOG:
			if(copy_from_user(&flag, (void __user *)arg, sizeof(int)))
			{
				return -EFAULT;
			}
			if(flag == 1)
			{
				crash_state = CRASH_STATE_INIT;
			}
			clear_klog();
			break;

		case KLOG_IOCTL_RECORD_SYSINFO:
			if(copy_from_user(&si, (void __user *)arg, sizeof(struct system_information)))
			{
				return -EFAULT;
			}

//check system is on or off
			if(si.system_on_off_state / 0x10 > 0)//power-on
			{
				switch(si.system_on_off_state % 0x10)
				{
					case 0://klogcat init
//set sysinfo
						memcpy(&sysinfo, &si, sizeof(struct system_information));

//recover klog signature and klog versions
						strncpy(sysinfo.klog_signature, KLOG_SIGNATURE, KLOG_SIGNATURE_LENGTH);
						sysinfo.klog_header_version = KLOG_HEADER_VERSION;
						sysinfo.klog_version = KLOG_VERSION_HEX;

//recover rpm_version
#ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION
						strncpy(sysinfo.rpm_version, rpm_version, KLOG_RPM_VERSION_LENGTH - 1);
#endif // #ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION

//recover bootloader_version
						strncpy(sysinfo.bootloader_version, bootloader_version, KLOG_BOOTLOADER_VERSION_LENGTH - 1);

//recover hw_id
#ifdef CCI_HW_ID
						snprintf(sysinfo.hw_id, KLOG_HW_ID_LENGTH, "%02X", get_cei_hw_id());
#endif // #ifdef CCI_HW_ID

//recover bootup and previous shutdown info
						memcpy(&si, klog_header, sizeof(struct system_information));
#ifdef CCI_KLOG_SBL_BOOT_TIME_USE_IMEM
#ifdef IMEM_CERT_RECORD
//check it is valid or not
						if(*cci_imem_magic == IMEM_CERT_RECORD)
						{
							sysinfo.sbl_bootup_time = *sbl_bootup_time / 1000;//ms
						}
						else
						{
							sysinfo.sbl_bootup_time = 0;
						}
#else // #ifdef IMEM_CERT_RECORD
						sysinfo.sbl_bootup_time = *sbl_bootup_time / 1000;//ms
#endif // #ifdef IMEM_CERT_RECORD
#else // #ifdef CCI_KLOG_SBL_BOOT_TIME_USE_IMEM
						sysinfo.sbl_bootup_time = si.sbl_bootup_time / 1000;//ms
#endif // #ifdef CCI_KLOG_SBL_BOOT_TIME_USE_IMEM
						sysinfo.aboot_bootup_time = si.aboot_bootup_time;//ms
						if(si.system_on_off_state == 0x06)//completed power-off
						{
#ifdef CCI_KLOG_DETAIL_LOG
							kprintk("sysinfo:system_on_off_state=0x%X, android_shutdown_time=%u, sysfs_sync_time=%u, kernel_power_off_time=%u\n", si.system_on_off_state, si.android_shutdown_time, si.sysfs_sync_time, si.kernel_power_off_time);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
							sysinfo.android_shutdown_time = si.android_shutdown_time;
							sysinfo.sysfs_sync_time = si.sysfs_sync_time;
							sysinfo.kernel_power_off_time = si.kernel_power_off_time;
						}
						break;

					case 1://klogcat boot
						sysinfo.system_on_off_state = si.system_on_off_state;
//boot_time
						system_bootup_time = klog_record_kernel_timestamp(NULL);
//save bootup info
						sysinfo.android_bootup_time = (unsigned int)(system_bootup_time.clock.tv_sec * 1000 + system_bootup_time.clock.tv_nsec / 1000000);//ms
						break;

					case 2://klogcat sysinfo
//update sysinfo
						strncpy(sysinfo.modem_version, si.modem_version, KLOG_MODEM_VERSION_LENGTH - 1);
						strncpy(sysinfo.scaling_max_freq, si.scaling_max_freq, KLOG_SCALING_MAX_FREQ_LENGTH - 1);
						strncpy(sysinfo.sim_state, si.sim_state, KLOG_SIM_STATE_LENGTH - 1);
						break;
				}
			}
			else//power-off
			{
				record_shutdown_time(si.system_on_off_state);
			}

//record sysinfo to header
			if((crash_state & CRASH_STATE_PREVIOUS) > 0)//not allow to overwrite if previous crash
			{
				kprintk("sysinfo:not allow to record sysinfo\n");
			}
			else
			{
#ifdef CCI_KLOG_DETAIL_LOG
				kprintk("sysinfo:klog_userdata_count=%s, klog_internal_count=%s, klog_external_count=%s, normal_boot=%s, android_version=%s, modem_version=%s, flex_version=%s, rpm_version=%s, bootloader_version=%s, linux_version=%s, version_id=%s, build_version=%s, build_date=%s, build_type=%s, build_user=%s, build_host=%s, build_key=%s, secure_mode=%s, debug_mode=%s, cpuinfo_max_freq=%s, scaling_max_freq=%s, system_on_off_state=0x%X, android_shutdown_time=%u, kernel_power_off_time=%u\n", sysinfo.klog_userdata_count, sysinfo.klog_internal_count, sysinfo.klog_external_count, sysinfo.normal_boot, sysinfo.android_version, sysinfo.modem_version, sysinfo.flex_version, sysinfo.rpm_version, sysinfo.bootloader_version, sysinfo.linux_version, sysinfo.version_id, sysinfo.build_version, sysinfo.build_date, sysinfo.build_type, sysinfo.build_user, sysinfo.build_host, sysinfo.build_key, sysinfo.secure_mode, sysinfo.debug_mode, sysinfo.cpuinfo_max_freq, sysinfo.scaling_max_freq, sysinfo.system_on_off_state, sysinfo.android_shutdown_time, sysinfo.kernel_power_off_time);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
				memcpy(klog_header + KLOG_SIGNATURE_HEADER_LENGTH + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH, &sysinfo.normal_boot, sizeof(struct system_information) - (KLOG_SIGNATURE_HEADER_LENGTH + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH));
			}
			break;

		case KLOG_IOCTL_GET_KLOG_VERSION:
			version = KLOG_VERSION_HEX;
			if(copy_to_user((void *)arg, &version ,sizeof(unsigned int)))
			{
				return -EFAULT;
			}
			break;

#ifdef CCI_KLOG_SUPPORT_RESTORATION
		case KLOG_IOCTL_RESTORE_LOG:
			klog_buffer = kmalloc(CCI_KLOG_SIZE, GFP_KERNEL);
			if(copy_from_user(klog_buffer, (void __user *)arg, CCI_KLOG_SIZE))
			{
				return -EFAULT;
			}
			memcpy(&si, klog_buffer, sizeof(struct system_information));
			if(strcmp(si.klog_signature, KLOG_SIGNATURE) != 0 || si.klog_header_version != KLOG_HEADER_VERSION || si.klog_version != KLOG_VERSION_HEX)
			{
				kprintk("header mismatch");
				return -EFAULT;
			}
			mem_ready = 0;//temporary lock klog
			unknowncrashflag_inited = 0;
			magic_priority = KLOG_PRIORITY_INVALID;
			memcpy(klog_header, klog_buffer, CCI_KLOG_SIZE);
			update_priority();
//recover previous crash info
			previous_normal_boot = (klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH)[0] - 0x30;
			if(previous_normal_boot < 0 || previous_normal_boot > 3)
			{
				previous_normal_boot = -1;
			}
			strncpy(previous_reboot_magic, klog_magic, KLOG_MAGIC_LENGTH);
			mem_ready = 1;//unlock klog
			kfree(klog_buffer);
			kprintk("restore finished\n");
			break;
#endif // #ifdef CCI_KLOG_SUPPORT_RESTORATION

#ifdef CCI_KLOG_ALLOW_FORCE_PANIC
		case KLOG_IOCTL_FORCE_PANIC:
			panic("klog_panic");
			break;

		case KLOG_IOCTL_FORCE_CRASH:
			*((int*)0) = 0;
			break;

		case KLOG_IOCTL_SET_PANIC_WHEN_SUSPEND:
			if(copy_from_user(&flag, (void __user *)arg, sizeof(int)))
			{
				return -EFAULT;
			}
			force_panic_when_suspend = flag;
			break;

		case KLOG_IOCTL_SET_PANIC_WHEN_POWER_OFF:
			if(copy_from_user(&flag, (void __user *)arg, sizeof(int)))
			{
				return -EFAULT;
			}
			force_panic_when_power_off = flag;
			break;

		case KLOG_IOCTL_SET_MAGIC:
			if(copy_from_user(magic, (void __user *)arg, sizeof(magic)))
			{
				return -EFAULT;
			}
			if(get_magic_index(magic) == KLOG_INDEX_INVALID)
			{
				return -EFAULT;
			}
			cklc_save_magic(magic, KLOG_STATE_NONE);
			break;
#endif // #ifdef CCI_KLOG_ALLOW_FORCE_PANIC

		case KLOG_IOCTL_GET_HEADER:
			memcpy(&sysinfo, klog_header, sizeof(struct system_information));
			if(copy_to_user((void *)arg, &sysinfo, sizeof(struct system_information)))
			{
				return -EFAULT;
			}
			break;

#if CCI_KLOG_CRASH_SIZE
		case KLOG_IOCTL_GET_CRASH:
			if(copy_to_user((void *)arg, &pklog_category[KLOG_CRASH]->name[0], CCI_KLOG_CRASH_SIZE))
			{
				return -EFAULT;
			}
			break;
#endif // #if CCI_KLOG_CRASH_SIZE

#if CCI_KLOG_APPSBL_SIZE
		case KLOG_IOCTL_GET_APPSBL:
			if(copy_to_user((void *)arg, &pklog_category[KLOG_APPSBL]->name[0], CCI_KLOG_APPSBL_SIZE))
			{
				return -EFAULT;
			}
			break;
#endif // #if CCI_KLOG_APPSBL_SIZE

#if CCI_KLOG_KERNEL_SIZE
		case KLOG_IOCTL_GET_KERNEL:
			if(copy_to_user((void *)arg, &pklog_category[KLOG_KERNEL]->name[0], CCI_KLOG_KERNEL_SIZE))
			{
				return -EFAULT;
			}
			break;
#endif // #if CCI_KLOG_KERNEL_SIZE

#if CCI_KLOG_ANDROID_MAIN_SIZE
		case KLOG_IOCTL_GET_ANDROID_MAIN:
			if(copy_to_user((void *)arg, &pklog_category[KLOG_ANDROID_MAIN]->name[0], CCI_KLOG_ANDROID_MAIN_SIZE))
			{
				return -EFAULT;
			}
			break;
#endif // #if CCI_KLOG_ANDROID_MAIN_SIZE

#if CCI_KLOG_ANDROID_SYSTEM_SIZE
		case KLOG_IOCTL_GET_ANDROID_SYSTEM:
			if(copy_to_user((void *)arg, &pklog_category[KLOG_ANDROID_SYSTEM]->name[0], CCI_KLOG_ANDROID_SYSTEM_SIZE))
			{
				return -EFAULT;
			}
			break;
#endif // #if CCI_KLOG_ANDROID_SYSTEM_SIZE

#if CCI_KLOG_ANDROID_RADIO_SIZE
		case KLOG_IOCTL_GET_ANDROID_RADIO:
			if(copy_to_user((void *)arg, &pklog_category[KLOG_ANDROID_RADIO]->name[0], CCI_KLOG_ANDROID_RADIO_SIZE))
			{
				return -EFAULT;
			}
			break;
#endif // #if CCI_KLOG_ANDROID_RADIO_SIZE

#if CCI_KLOG_ANDROID_EVENTS_SIZE
		case KLOG_IOCTL_GET_ANDROID_EVENTS:
			if(copy_to_user((void *)arg, &pklog_category[KLOG_ANDROID_EVENTS]->name[0], CCI_KLOG_ANDROID_EVENTS_SIZE))
			{
				return -EFAULT;
			}
			break;
#endif // #if CCI_KLOG_ANDROID_EVENTS_SIZE

		case KLOG_IOCTL_GET_RESERVE:
/*
			if(copy_to_user((void *)arg, &pklog_category[KLOG_RESERVE]->name[0], CCI_KLOG_RESERVE_SIZE))
			{
				return -EFAULT;
			}
*/
			break;

		default:
			return -1;
	}

	return 0;
}

//Taylor CCIKlog--> B
/*
 * get_entry_header - returns a pointer to the logger_entry header within
 * 'log' starting at offset 'off'. A temporary logger_entry 'scratch' must
 * be provided. Typically the return value will be a pointer within
 * 'logger->buf'.  However, a pointer to 'scratch' may be returned if
 * the log entry spans the end and beginning of the circular buffer.
 */
static struct logger_entry *get_entry_header(struct logger_log *log,
		size_t off, struct logger_entry *scratch)
{
	size_t len = min(sizeof(struct logger_entry), log->size - off);

	if (len != sizeof(struct logger_entry)) {
		memcpy(((void *) scratch), log->buffer + off, len);
		memcpy(((void *) scratch) + len, log->buffer,
			sizeof(struct logger_entry) - len);
		return scratch;
	}

	return (struct logger_entry *) (log->buffer + off);
}

/*
 * get_entry_msg_len - Grabs the length of the message of the entry
 * starting from from 'off'.
 *
 * An entry length is 2 bytes (16 bits) in host endian order.
 * In the log, the length does not include the size of the log entry structure.
 * This function returns the size including the log entry structure.
 *
 * Caller needs to hold log->mutex.
 */
static __u32 get_entry_msg_len(struct logger_log *log, size_t off)
{
	struct logger_entry scratch;
	struct logger_entry *entry;

	entry = get_entry_header(log, off, &scratch);
	return entry->len;
}

/*
 * get_next_entry - return the offset of the first valid entry at least 'len'
 * bytes after 'off'.
 *
 * Caller must hold log->mutex.
 */
static size_t get_next_entry(struct logger_log *log, size_t off, size_t len)
{
	size_t count = 0;

	do {
		size_t nr = sizeof(struct logger_entry) +
			get_entry_msg_len(log, off);
		off = logger_offset(log, off + nr);
		count += nr;
	} while (count < len);

	return off;
}

/*
 * is_between - is a < c < b, accounting for wrapping of a, b, and c
 *    positions in the buffer
 *
 * That is, if a<b, check for c between a and b
 * and if a>b, check for c outside (not between) a and b
 *
 * |------- a xxxxxxxx b --------|
 *               c^
 *
 * |xxxxx b --------- a xxxxxxxxx|
 *    c^
 *  or                    c^
 */
static inline int is_between(size_t a, size_t b, size_t c)
{
	if (a < b) {
		/* is c between a and b? */
		if (a < c && c <= b)
			return 1;
	} else {
		/* is c outside of b through a? */
		if (c <= b || a < c)
			return 1;
	}

	return 0;
}


/*
 * fix_up_readers - walk the list of all readers and "fix up" any who were
 * lapped by the writer; also do the same for the default "start head".
 * We do this by "pulling forward" the readers and start head to the first
 * entry after the new write head.
 *
 * The caller needs to hold log->mutex.
 */
static void fix_up_readers(struct logger_log *log, size_t len)
{
	size_t old = log->w_off;
	size_t new = logger_offset(log, old + len);
	struct logger_reader *reader;

	if (is_between(old, new, log->head))
		log->head = get_next_entry(log, log->head, len);

	list_for_each_entry(reader, &log->readers, list)
		if (is_between(old, new, reader->r_off)) {
			size_t old_r_off = reader->r_off;
			reader->r_off = get_next_entry(log, reader->r_off, len);
			if (reader->r_off >= old_r_off) {
				reader->missing_bytes += (reader->r_off - old_r_off);
			}
			else {
				reader->missing_bytes += (reader->r_off + (log->size - old_r_off));
			}
		}
}

/*
 * file_get_log - Given a file structure, return the associated log
 *
 * This isn't aesthetic. We have several goals:
 *
 *	1) Need to quickly obtain the associated log during an I/O operation
 *	2) Readers need to maintain state (logger_reader)
 *	3) Writers need to be very fast (open() should be a near no-op)
 *
 * In the reader case, we can trivially go file->logger_reader->logger_log.
 * For a writer, we don't want to maintain a logger_reader, so we just go
 * file->logger_log. Thus what file->private_data points at depends on whether
 * or not the file was opened for reading. This function hides that dirtiness.
 */
static inline struct logger_log *file_get_log(struct file *file)
{

	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader = file->private_data;

		return reader->log;
	} else
		return file->private_data;
}

/*
 * do_write_log - writes 'len' bytes from 'buf' to 'log'
 *
 * The caller needs to hold log->mutex.
 */
static void do_write_log(struct logger_log *log, const void *buf, size_t count)
{
	size_t len;

	len = min(count, log->size - log->w_off);
	//memcpy(log->buffer + log->w_off, buf, len);

	if (count != len)
		//memcpy(log->buffer, buf + len, count - len);

	log->w_off = logger_offset(log, log->w_off + count);

}

/*
 * logger_aio_write - move from logger.c which kernel-3.18 doesn't have it anymore, 
 * implementing support for write(),
 * writev(), and aio_write(). Writes are our fast path, and we try to optimize
 * them above all else.
 */

/*
 * do_write_log_user - writes 'len' bytes from the user-space buffer 'buf' to
 * the log 'log'
 *
 * The caller needs to hold log->mutex.
 *
 * Returns 'count' on success, negative error code on failure.
 */
static ssize_t do_write_log_from_user(struct logger_log *log,
				      const void __user *buf, size_t count)
{
	size_t len;

	len = min(count, log->size - log->w_off);
	if (len && copy_from_user(log->buffer + log->w_off, buf, len))
		return -EFAULT;

	if (count != len)
		if (copy_from_user(log->buffer, buf + len, count - len))
			/*
			 * Note that by not updating w_off, this abandons the
			 * portion of the new entry that *was* successfully
			 * copied, just above.  This is intentional to avoid
			 * message corruption from missing fragments.
			 */
			return -EFAULT;

	log->w_off = logger_offset(log, log->w_off + count);

	return count;
}

static struct logger_log *get_log_from_minor(int minor)
{
	struct logger_log *log;

	list_for_each_entry(log, &log_list, logs)
		if (log->misc.minor == minor)
			return log;
	
	return NULL;
}

/*
 * logger_open - the log's open() file operation
 *
 * Note how near a no-op this is in the write-only case. Keep it that way!
 */
static int logger_open(struct inode *inode, struct file *file)
{
	struct logger_log *log;
	int ret;

	ret = nonseekable_open(inode, file);
	if (ret)
		return ret;

	log = get_log_from_minor(MINOR(inode->i_rdev));
	if (!log){
		printk("%s - logger open failed\n",__func__);
		return -ENODEV;
	}

	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader;

		reader = kmalloc(sizeof(struct logger_reader), GFP_KERNEL);
		if (!reader){
			printk("%s - reader alloc failed\n",__func__);
			return -ENOMEM;
		}

		reader->log = log;
		reader->r_ver = 1;
		reader->r_all = in_egroup_p(inode->i_gid) ||
			capable(CAP_SYSLOG);
		reader->missing_bytes = 0;

		INIT_LIST_HEAD(&reader->list);

		mutex_lock(&log->mutex);
		reader->r_off = log->head;
        reader->wake_up_interval = 0; 
        reader->wake_up_timer = 0;
		list_add_tail(&reader->list, &log->readers);
		mutex_unlock(&log->mutex);

		file->private_data = reader;
	} else
		file->private_data = log;

	return 0;
}

static ssize_t logger_write_iter(struct kiocb *iocb, struct iov_iter *iov_iter)
{
	ssize_t ret = 0;
	struct logger_log *log = file_get_log(iocb->ki_filp);
	size_t orig;
	struct logger_entry header;
	struct timespec now;
//[VY36] ==> CCI KLog, added by Jimmy@CCI
#ifdef CONFIG_CCI_KLOG
	unsigned int category = KLOG_IGNORE;
	unsigned int priority = 0;
	size_t tag_index = 0;
	size_t tag_size = 0;
	size_t msg_index = 0;
	size_t msg_size = 0;
	size_t log_offset = 0;
	int idx = 0;
	char klog_type[20];
#endif // #ifdef CONFIG_CCI_KLOG
//[VY36] <== CCI KLog, added by Jimmy@CCI
	const struct iovec *iov = iov_iter->iov;
	unsigned long nr_segs = iov_iter->nr_segs;

		// android default timestamp
		//now = current_kernel_time();
		getnstimeofday(&now);
		header.pid = current->tgid;
		header.tid = current->pid;
		header.sec = now.tv_sec;
		header.nsec = now.tv_nsec;
		header.euid = current_euid();
		//header.len = min_t(size_t, iocb->ki_left, LOGGER_ENTRY_MAX_PAYLOAD);
		header.len = LOGGER_ENTRY_MAX_PAYLOAD;
		header.hdr_size = sizeof(struct logger_entry);
        header.tz = sys_tz.tz_minuteswest * 60;
	/* null writes succeed, return zero */
	if (unlikely(!header.len))
		return 0;

	mutex_lock(&log->mutex);

	orig = log->w_off;

	/*
	 * Fix up any readers, pulling them forward to the first readable
	 * entry after (what will be) the new write offset. We do this now
	 * because if we partially fail, we can end up with clobbered log
	 * entries that encroach on readable buffer.
	 */
	fix_up_readers(log, sizeof(struct logger_entry) + header.len);

	do_write_log(log, &header, sizeof(struct logger_entry));

//Taylor CCIKlog--> B
if(copy_from_user(&klog_type,  iov[3].iov_base, sizeof(klog_type)) == 0){
	 if(!strncmp((char*)&klog_type, LOGGER_LOG_MAIN, sizeof(LOGGER_LOG_MAIN)))
	{
#if CCI_KLOG_ANDROID_MAIN_SIZE
		category = KLOG_ANDROID_MAIN;
#else // #if CCI_KLOG_ANDROID_MAIN_SIZE
		category = KLOG_IGNORE;
#endif // #if CCI_KLOG_ANDROID_MAIN_SIZE
	}
	else if(!strncmp((char*)&klog_type, LOGGER_LOG_SYSTEM, sizeof(LOGGER_LOG_SYSTEM)))
	{
#if CCI_KLOG_ANDROID_SYSTEM_SIZE
		category = KLOG_ANDROID_SYSTEM;
#else // #if CCI_KLOG_ANDROID_SYSTEM_SIZE
		category = KLOG_IGNORE;
#endif // #if CCI_KLOG_ANDROID_SYSTEM_SIZE
	}
		else if(!strncmp((char*)&klog_type, LOGGER_LOG_RADIO, sizeof(LOGGER_LOG_RADIO)))
	{
#if CCI_KLOG_ANDROID_RADIO_SIZE
		category = KLOG_ANDROID_RADIO;
#else // #if CCI_KLOG_ANDROID_RADIO_SIZE
		category = KLOG_IGNORE;
#endif // #if CCI_KLOG_ANDROID_RADIO_SIZE
	}
	else if(!strncmp((char*)&klog_type, LOGGER_LOG_EVENTS, sizeof(LOGGER_LOG_EVENTS)))
	{
#if CCI_KLOG_ANDROID_EVENTS_SIZE
		category = KLOG_ANDROID_EVENTS;
#else // #if CCI_KLOG_ANDROID_EVENTS_SIZE
		category = KLOG_IGNORE;
#endif // #if CCI_KLOG_ANDROID_EVENTS_SIZE
	}
	else
	{
		kprintk("%s():%d:Unable to identify the log name:%s\n", __func__, __LINE__, (char*)&klog_type);
	}
}
else
	kprintk("iov[3].iov_base is NULL bypass klog filter\n");
//Taylo CCIKlog<--E
	
	while (nr_segs-- > 0) {
		size_t len;
		ssize_t nr;

		/* figure out how much of this vector we can keep */
		len = min_t(size_t, iov->iov_len, header.len - ret);

//[VY36] ==> CCI KLog, added by Jimmy@CCI
#ifdef CONFIG_CCI_KLOG
		log_offset = log->w_off;
#endif // #ifdef CONFIG_CCI_KLOG
//[VY36] <== CCI KLog, added by Jimmy@CCI

		/* write out this segment's payload */
		nr = do_write_log_from_user(log, iov->iov_base, len);
		if (unlikely(nr < 0)) {
			log->w_off = orig;
			mutex_unlock(&log->mutex);
			return nr;
		}
//[VY36] ==> CCI KLog, added by Jimmy@CCI
#ifdef CONFIG_CCI_KLOG
		switch(idx)
		{
			case 0://priority
				priority = (unsigned int)*(log->buffer + log_offset);
				break;
			case 1://tag
				tag_index = log_offset;
				tag_size = nr;
				break;
			case 2://message
				msg_index = log_offset;
				msg_size = nr;
				break;
		}
		idx++;
#endif // #ifdef CONFIG_CCI_KLOG
//[VY36] <== CCI KLog, added by Jimmy@CCI

		iov++;
		ret += nr;
	}
//[VY36] ==> CCI KLog, added by Jimmy@CCI
#ifdef CONFIG_CCI_KLOG
	if(idx == 4)
	{
		
		if(category != KLOG_IGNORE)
		{
			cklc_append_android_log(category, &header, log->buffer, log->size, priority, tag_index, tag_size, msg_index, msg_size);
		}
	}
#endif // #ifdef CONFIG_CCI_KLOG
//[VY36] <== CCI KLog, added by Jimmy@CCI

	mutex_unlock(&log->mutex);

	/* wake up any blocked readers */
	wake_up_interruptible(&log->wq);
	return ret;
}
//Taylor CCIKlog<--E

#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
#ifndef MSM_SHARED_RAM_BASE
void set_smem_addr(struct mem_area *mem_area_info)
{
	smem_info.phys_addr = mem_area_info->phys_addr;
	smem_info.size = mem_area_info->size;
	smem_info.virt_addr = mem_area_info->virt_addr;

	return;
}
#endif // #ifndef MSM_SHARED_RAM_BASE
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM

#ifdef CCI_KLOG_SUPPORT_ATTRIBUTE
#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
static ssize_t klog_show_modem_log_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 12, "0x%lX\n", modem_log_addr);//12 = "0x" + 8-digit hexdecimal number + '\n' + '\0'
}

static ssize_t klog_store_modem_log_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);

#ifdef CCI_KLOG_DETAIL_LOG
	kprintk("%s():val=0x%lX, modem_log_addr=0x%lX\n", __func__, val, modem_log_addr);
#endif // #ifdef CCI_KLOG_DETAIL_LOG
	modem_log_addr = val;
	modem_log_addr_inited = 1;

	return count;
}

KLOG_RW_DEV_ATTR(modem_log_addr);
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM

static ssize_t klog_show_prev_reboot_reason(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 8, "%s\n", previous_reboot_magic);//8 = 6-character magic + '\n' + '\0'
}

KLOG_RO_DEV_ATTR(prev_reboot_reason);

static ssize_t klog_show_prev_normal_boot(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%d\n", previous_normal_boot);//4 = 2-digit decimal number (-1 ~ 3) + '\n' + '\0'
}

KLOG_RO_DEV_ATTR(prev_normal_boot);

static struct attribute *klog_attrs[] =
{
#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
	&dev_attr_modem_log_addr.attr,
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
	&dev_attr_prev_reboot_reason.attr,
	&dev_attr_prev_normal_boot.attr,
	NULL
};

static const struct attribute_group klog_attr_group =
{
	.attrs	= klog_attrs,
};
#endif // #ifdef CCI_KLOG_SUPPORT_ATTRIBUTE

static const struct file_operations klog_fops = {
	.owner		= THIS_MODULE,
#if 1
	.write_iter = logger_write_iter,
#else
	.aio_write = logger_aio_write,
#endif
	.unlocked_ioctl	= klog_ioctl,
	.open = logger_open,
};


static struct miscdevice klog_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= KLOG_DEV_ID,
	.fops	= &klog_fops,
	.parent	= NULL,
};

int klogmisc_init(void)
{
	int retval = 0;
//Taylor CCIKlog-->B
	struct logger_log *log;
	unsigned char *buffer;
	int size= 256*1024;
	
	buffer = vmalloc(size);
	if (buffer == NULL)
		return -ENOMEM;

	log = kzalloc(sizeof(struct logger_log), GFP_KERNEL);
	if (log == NULL) {
		retval = -ENOMEM;
		goto out_free_buffer;
	}
	log->buffer = buffer;
	
	
//Taylor CCIKlog--<E
//init miscdev
	retval = misc_register(&klog_miscdev);
	if(retval)
	{
		kprintk("cannot register as miscdev, err=0x%X\n", retval);
		goto error_init_miscdev;
	}
//Taylor CCIKlog-->B
	log->misc.minor = klog_miscdev.minor;
	log->misc.name = kstrdup(KLOG_DEV_ID, GFP_KERNEL);
	if (log->misc.name == NULL) {
		retval = -ENOMEM;
		goto error_init_miscdev;
	}

	log->misc.fops = &klog_fops;
	log->misc.parent = klog_miscdev.parent;

	init_waitqueue_head(&log->wq);
	INIT_LIST_HEAD(&log->readers);
	mutex_init(&log->mutex);
	log->w_off = 0;
	log->head = 0;
	log->size = size;

	INIT_LIST_HEAD(&log->logs);
	list_add_tail(&log->logs, &log_list);
//Taylor CCIKlog<--E
#ifdef CCI_KLOG_SUPPORT_ATTRIBUTE
//init sysfs
	retval = sysfs_create_group(&klog_miscdev.this_device->kobj, &klog_attr_group);
	if(retval < 0)
	{
		kprintk("init sysfs failed, err=0x%X\n", retval);
		goto error_init_sysfs;
	}
#endif // #ifdef CCI_KLOG_SUPPORT_ATTRIBUTE

	return 0;

#ifdef CCI_KLOG_SUPPORT_ATTRIBUTE
error_init_sysfs:
	sysfs_remove_group(&klog_miscdev.this_device->kobj, &klog_attr_group);
#endif // #ifdef CCI_KLOG_SUPPORT_ATTRIBUTE

error_init_miscdev:
	kfree(log);
	misc_deregister(&klog_miscdev);

out_free_buffer:
	vfree(buffer);

	return retval;
}

void klogmisc_exit(void)
{
#ifdef CCI_KLOG_SUPPORT_ATTRIBUTE
	sysfs_remove_group(&klog_miscdev.this_device->kobj, &klog_attr_group);
#endif // #ifdef CCI_KLOG_SUPPORT_ATTRIBUTE

	misc_deregister(&klog_miscdev);
}

static int __init get_warmboot(char* cmdline)
{
	if(cmdline == NULL || strlen(cmdline) == 0)
	{
		kprintk("warmboot is empty\n");
	}
	else
	{
		warmboot = simple_strtoul(cmdline, NULL, 16);
		kprintk("warmboot:0x%X\n", warmboot);
	}

	return 0;
}
__setup("warmboot=", get_warmboot);

static int __init get_startup(char* cmdline)
{
	if(cmdline == NULL || strlen(cmdline) == 0)
	{
		kprintk("startup is empty\n");
	}
	else
	{
		startup = simple_strtoul(cmdline, NULL, 16);
		kprintk("startup:0x%X\n", startup);
	}

	return 0;
}
__setup("startup=", get_startup);

static int __init get_bootloader_version(char* cmdline)
{
	if(cmdline == NULL || strlen(cmdline) == 0)
	{
		kprintk("oemandroidboot.s1boot is empty\n");
	}
	else
	{
		strncpy(bootloader_version, cmdline, KLOG_BOOTLOADER_VERSION_LENGTH - 1);
		kprintk("oemandroidboot.s1boot:%s\n", bootloader_version);
	}

	return 0;
}
__setup("oemandroidboot.s1boot=", get_bootloader_version);

static int __init cklc_init(void)
{
	int retval = 0;
	int magic_index = 0;
	unsigned int *tempval;

//record klog signature and versions
	snprintf(klog_header, KLOG_SIGNATURE_LENGTH, "%s", KLOG_SIGNATURE);
	tempval = (void *)MSM_KLOG_HEADER + KLOG_SIGNATURE_LENGTH;
	*tempval = KLOG_HEADER_VERSION;
	tempval++;
	*tempval = KLOG_VERSION_HEX;

//record previous normal boot
	previous_normal_boot = (klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH)[0] - 0x30;
	if(previous_normal_boot < 0 || previous_normal_boot > 3)
	{
		previous_normal_boot = -1;
	}

//record previous magic
	strncpy(previous_reboot_magic, klog_magic, KLOG_MAGIC_LENGTH);

//detect bootloader
	if(strlen(bootloader_version) == 0)
	{
		strncpy(bootloader_version, "CCI_bootloader", KLOG_BOOTLOADER_VERSION_LENGTH - 1);
	}

//detect charge-only mode
	magic_index = get_magic_index(klog_magic);
	if(startup == 0x4 && warmboot == 0)//charge-only mode
	{
		sysinfo.normal_boot[0] = '3';
		memcpy(klog_magic + KLOG_MAGIC_TOTAL_LENGTH + KLOG_KERNEL_TIME_LENGTH + KLOG_FIRST_RTC_TIMESTAMP_LENGTH + KLOG_LAST_RTC_TIMESTAMP_LENGTH, &sysinfo.normal_boot, sizeof(sysinfo.normal_boot));
	}

#ifdef STUFF_CRASH_DEFAULT
	kprintk("CCI KLog Init: prev_normal_boot=%d, crashflag=0x%X, unknownrebootflag=0x%X\n", previous_normal_boot, crashflag, unknownrebootflag);
#else // #ifdef STUFF_CRASH_DEFAULT
	kprintk("CCI KLog Init: prev_normal_boot=%d\n", previous_normal_boot);
#endif // #ifdef STUFF_CRASH_DEFAULT

#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
#ifdef MSM_SHARED_RAM_BASE
	smem_info.phys_addr = MSM_SHARED_RAM_PHYS;
	smem_info.size = MSM_SHARED_RAM_SIZE;
	smem_info.virt_addr = MSM_SHARED_RAM_BASE;
#endif // #ifdef MSM_SHARED_RAM_BASE
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM

//init klog magic
	cklc_save_magic(KLOG_MAGIC_INIT, KLOG_STATE_INIT);

	retval = klogmisc_init();

	return retval;
}

static void __exit cklc_exit(void)
{
	kprintk("CCI KLog Exit\n");
	klogmisc_exit();

	return;
}

module_init(cklc_init);
module_exit(cklc_exit);

MODULE_AUTHOR("Kevin Chiang <Kevin_Chiang@Compalcomm.com>");
MODULE_LICENSE("Proprietary");
MODULE_DESCRIPTION("CCI kernel and logcat log collector");
MODULE_VERSION(KLOG_VERSION);

/*
 * ------------------------------------------------------------------------------------
 * KLOG memory map
 *
 * address              : size      : field                     : note
 * ------------------------------------------------------------------------------------
 * 0x00000000-0x0000000F:(0x000010) : klog sig + h-ver + ver    : "CCIKLOG" + 0xAABBCCDD + 0xEEFFGGHH
 * 0x00000010-0x00000019:(0x00000A) : klog magic                : MAGIC:"CKLOGC###\0", please reference klog_magic_list
 * 0x0000001A-0x0000002D:(0x000014) : kernel up time            : [%08lX.%08lX]\0
 * 0x0000002E-0x00000041:(0x000014) : first RTC timestamp       : [%08lX.%08lX]\0
 * 0x00000042-0x00000055:(0x000014) : last RTC timestamp        : [%08lX.%08lX]\0
 * 0x00000056-0x000003FF:(0x0003BA) : other information         :
 * ------------------------------------------------------------------------------------
 * 0x00000400-0x000023FF:(0x002000) : KLOG_CRASH                :
 *                                  :                           : name             16
 *                                  :                           : size              4
 *                                  :                           : index             4
 *                                  :                           : overload          1
 *                                  :                           : buffer  0x002000-25
 * ------------------------------------------------------------------------------------
 * 0x00002400-0x00003BFF:(0x001800) : KLOG_APPSBL               :
 *                                  :                           : name             16
 *                                  :                           : size              4
 *                                  :                           : index             4
 *                                  :                           : overload          1
 *                                  :                           : buffer  0x001800-25
 * ------------------------------------------------------------------------------------
 * 0x00003C00-0x00102BFF:(0x0FF000) : KLOG_KERNEL               :
 *                                  :                           : name             16
 *                                  :                           : size              4
 *                                  :                           : index             4
 *                                  :                           : overload          1
 *                                  :                           : buffer  0x0FF000-25
 * ------------------------------------------------------------------------------------
 * 0x00102C00-0x00201BFF:(0x0FF000) : KLOG_ANDROID_MAIN         :
 *                                  :                           : name             16
 *                                  :                           : size              4
 *                                  :                           : index             4
 *                                  :                           : overload          1
 *                                  :                           : buffer  0x0FF000-25
 * ------------------------------------------------------------------------------------
 * 0x00201C00-0x00300BFF:(0x0FF000) : KLOG_ANDROID_SYSTEM       :
 *                                  :                           : name             16
 *                                  :                           : size              4
 *                                  :                           : index             4
 *                                  :                           : overload          1
 *                                  :                           : buffer  0x0FF000-25
 * ------------------------------------------------------------------------------------
 * 0x00300C00-0x003FFBFF:(0x0FF000) : KLOG_ANDROID_RADIO        :
 *                                  :                           : name             16
 *                                  :                           : size              4
 *                                  :                           : index             4
 *                                  :                           : overload          1
 *                                  :                           : buffer  0x0FF000-25
 * ------------------------------------------------------------------------------------
 * 0x---------0x--------:(0x000000) : KLOG_ANDROID_EVENTS       :
 *                                  :                           : name             16
 *                                  :                           : size              4
 *                                  :                           : index             4
 *                                  :                           : overload          1
 *                                  :                           : buffer  0x000000-25
 * ------------------------------------------------------------------------------------
 * 0x003FFC00-0x003FFFFF:(0x000400) : RESERVE                   :
 * ------------------------------------------------------------------------------------

Kernel Driver Version Description: (since 1.15.0.0)
ver A.B.C.D
A:main architecture
B:project serial
C:major function/feature implement/change
D:minor change

History:
version		author		description
1.0.0.0		Luke		KB62 latest version
1.1.0.0		Jimmy		porting to DA80 Gingerbread
1.2.0.0		Jimmy		implement KLog magic for power on/off reason
1.3.0.0		Jimmy		implement KLog header for system information
1.4.0.0		Jimmy		porting to DA80 IceCreamSandwich
1.5.0.0		Jimmy		implement KLog category size customization for all categories and support APPSBL category
1.6.0.0		Jimmy		implement KLog support CCI engmode tool
1.7.0.0		Jimmy		implement KLog crash category
1.8.0.0		Jimmy		porting to SA77 IceCreamSandwich
1.9.0.0		Jimmy		porting to SA77 JellyBean
1.10.0.0	Jimmy		implement KLog allocate memory in kernel and keep previous reboot reason
1.11.0.0	Jimmy		implement KLog support S1 boot cmdline and crashflag
1.12.0.0	Jimmy		implement KLog common header file and support subsystem silent restart
1.13.0.0	Jimmy		implement KLog header (signature + versions)
1.14.0.0	Jimmy		implement KLog support restoration
1.15.0.0	Jimmy		porting to LX16 KitKat with Linux kernel v3.10.28
1.16.0.0	Jimmy		porting to LY28 KitKat with Linux kernel v3.10.48
*/

