#ifndef __KLOG_DRIVER_H__
#define __KLOG_DRIVER_H__


#ifndef __KLOG_COMMON_H__
#include <generated/cciklog_common.h>
#endif // #ifndef __KLOG_COMMON_H__

#ifdef CONFIG_CCI_KLOG
#ifdef CONFIG_BUILD_REDWOOD
#define MSM_KLOG_BASE 0xFFFFFFC004640000
#else
#define MSM_KLOG_BASE 0xFFFFFFC07D800000
#endif
//#define CCI_KLOG_DETAIL_LOG
#define CCI_HW_ID
#ifdef IMEM_CERT_RECORD
#define CCI_KLOG_SBL_BOOT_TIME_USE_IMEM
#define CCI_IMEM_OFFSET						0xB00
#define CCI_IMEM_OFFSET_SBL_BOOT_TIME				0xB10
#define CCI_IMEM_OFFSET_POWER_ON_OFF				0xB20
#define CCI_IMEM_OFFSET_WARRANTY_DATA				(0xB20 + 84 * 4)
#define CCI_IMEM_OFFSET_FACTORY_RECORD				(0xB20 + 84 * 4 + 0x20 * 3)
#endif // #ifdef IMEM_CERT_RECORD
//#define CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
#define CCI_KLOG_SUPPORT_ATTRIBUTE
//#include <../drivers/staging/android/logger.h>

#define kprintk(fmt, args...)					printk(KERN_CRIT KLOG_LOG_TAG fmt, ##args)
#define kprintk_set_magic(reason, magic, state, final)		kprintk("set magic(%s):magic=%s, state=%s, final=%s\n", reason, magic, state, final)

enum klog_crash_state
{
	CRASH_STATE_INIT = 0,
	CRASH_STATE_CRASHING = 1,
	CRASH_STATE_PREVIOUS = 2,
};

enum klog_fault_level
{
	FAULT_LEVEL_FINISH = -1,
	FAULT_LEVEL_INIT = 0,
	FAULT_LEVEL_PANIC = 1,
	FAULT_LEVEL_DIE = 2,
	FAULT_LEVEL_DATA_ABORT = 3,
	FAULT_LEVEL_PREFETCH_ABORT = 4,
	FAULT_LEVEL_SUBSYSTEM = 5,
	FAULT_LEVEL_WATCHDOG = 6,
	FAULT_LEVEL_DATA_ABORT_64 = 7,
	FAULT_LEVEL_MAX = 7,
	FAULT_LEVEL_EXIST = 0x10,
};

enum klog_fault_type
{
	FAULT_TYPE_INIT = -1,
	FAULT_TYPE_NONE = 0,
};

struct klog_time
{
	struct timespec	clock;
	struct timespec	rtc;
};

struct mem_area
{
	phys_addr_t phys_addr;
	resource_size_t size;
	void __iomem *virt_addr;
};

//Taylor CCIKlog-->B
/**
 * struct logger_entry - defines a single entry that is given to a logger
 * @len:	The length of the payload
 * @hdr_size:	sizeof(struct logger_entry_v2)
 * @pid:	The generating process' process ID
 * @tid:	The generating process' thread ID
 * @sec:	The number of seconds that have elapsed since the Epoch
 * @nsec:	The number of nanoseconds that have elapsed since @sec
 * @euid:	Effective UID of logger
 * @msg:	The message that is to be logged
 *
 * The structure for version 2 of the logger_entry ABI.
 * This structure is returned to userspace if ioctl(LOGGER_SET_VERSION)
 * is called with version >= 2
 */
struct logger_entry {
	__u16		len;
	__u16		hdr_size;
	__s32		pid;
	__s32		tid;
	__s32		sec;
	__s32		nsec;
    __s32       tz;         /* timezone*/
	kuid_t		euid;
	char		msg[0];
};

#define LOGGER_ENTRY_MAX_PAYLOAD	4076

#define LOGGER_LOG_RADIO	"log_radio"	/* radio-related messages */
#define LOGGER_LOG_EVENTS	"log_events"	/* system/hardware events */
#define LOGGER_LOG_SYSTEM	"log_system"	/* system/framework messages */
#define LOGGER_LOG_MAIN		"log_main"	/* everything else */

//Taylor CCIKlog<--E

#ifdef CCI_KLOG_SUPPORT_CCI_ENGMODE
extern struct system_information *psysinfo;
#endif // #ifdef CCI_KLOG_SUPPORT_CCI_ENGMODE

void cklc_append_kernel_raw_char(char c);
void cklc_append_str(const char *str, size_t len);
void cklc_append_newline(void);
void cklc_append_separator(void);
void cklc_append_time_header(void);
void show_android_log_to_console(void);
//void cklc_append_android_log(unsigned int category, const struct logger_entry *header, const char *log_buf, size_t log_size, unsigned int priority, size_t tag_index, size_t tag_size, size_t msg_index, size_t msg_size);
void cklc_save_magic(char *magic, int state);
void cklc_set_memory_ready(void);
int match_crash_priority(int priority);
void update_priority(void);
#ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION
void klog_record_rpm_version(const char *str);
#endif // #ifdef CONFIG_CCI_KLOG_RECORD_RPM_VERSION
#ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
#ifndef MSM_SHARED_RAM_BASE
void set_smem_addr(struct mem_area *smem_info);
#endif // #ifndef MSM_SHARED_RAM_BASE
#endif // #ifdef CCI_KLOG_MODEM_CRASH_LOG_USE_SMEM
int get_fault_state(void);
void set_fault_state(int level, int type, const char* msg);
void set_kernel_log_level(int level);
void record_shutdown_time(int state);
struct timespec klog_get_kernel_clock_timestamp(void);
#ifdef CCI_KLOG_ALLOW_FORCE_PANIC
int get_force_panic_when_suspend(void);
int get_force_panic_when_power_off(void);
#endif // #ifdef CCI_KLOG_ALLOW_FORCE_PANIC

#else // #ifdef CONFIG_CCI_KLOG

#define cklc_append_kernel_raw_char(c)				do {} while (0)
#define cklc_append_str(str, len)				do {} while (0)
#define cklc_append_newline()					do {} while (0)
#define cklc_append_separator()					do {} while (0)
#define cklc_append_time_header()				do {} while (0)
#define cklc_save_magic						do {} while (0)
#define cklc_set_memory_ready					do {} while (0)
#define klog_record_rpm_version					do {} while (0)


#endif // #ifdef CONFIG_CCI_KLOG

#endif // #ifndef __KLOG_DRIVER_H__

