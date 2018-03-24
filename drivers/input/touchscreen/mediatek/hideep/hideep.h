/*************************************************************************
 * Copyright (C) 2012 Hideep, Inc.
 * kim.liao@hideep.com
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
 *************************************************************************/

#ifndef _LINUX_HIDEEP_H
#define _LINUX_HIDEEP_H

/*************************************************************************
 * this is include special HEAD file.
 *************************************************************************/
#ifdef CONFIG_FB
	#include <linux/fb.h>
	#include <linux/notifier.h>
#endif

/*************************************************************************
 * this is include normal HEAD file.
 *************************************************************************/
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/input/mt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gfp.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <linux/fcntl.h>
#include <linux/unistd.h>


/* For MTK platform */
#define MTK_PLATFORM
#ifdef MTK_PLATFORM
#define GTP_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define GTP_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)

#endif
/*************************************************************************
 * definition part.
 * define is (open, set, enable) if not, is (close, clear, disable)
 * some special switch of functions.
 *************************************************************************/
/* HIDEEP_DEBUG_DEVICE is for input output function for debugging. */
/* #define HIDEEP_DEBUG_DEVICE */

/* HIDEEP_AUTO_UPDATE is for FW automatically update function. */
/* #define HIDEEP_AUTO_UPDATE */
#ifdef HIDEEP_AUTO_UPDATE
/* UNIT ms */
#define HIDEEP_UPDATE_FW_THREAD_DELAY 10000
#endif

/* HIDEEP_VIRTUAL_KEY is for virtual key. */
#define HIDEEP_VIRTUAL_KEY

/* HIDEEP_PROTOCOL_2_0 is for protocol 2.0. */
#define HIDEEP_PROTOCOL_2_0

/* HIDEEP_TYPE_B_PROTOCOL is for input_dev, if define , using TYPE_B, otherwise TYPE_A. */
/* #define HIDEEP_TYPE_B_PROTOCOL */

/* HIDEEP_DWZ_VERSION_CHECK if define, it will check dwz version. */
/* #define HIDEEP_DWZ_VERSION_CHECK */

/* HIDEEP_KEYBUTTON if define, it will use key button. */
#define HIDEEP_KEYBUTTON

/* HIDEEP_PANEL_INFO if define, it will read dwz info. */
#define HIDEEP_PANEL_INFO

/* HIDEEP_SELFTEST_MODE if define, it will use self test. */
#define HIDEEP_SELFTEST_MODE

#ifdef HIDEEP_SELFTEST_MODE
#define SELF_TEST_DATA_ADDR 0x1008
#define TEST_MODE_COMMAND_ADDR 0x0804
#endif

/* HIDEEP_LPM_WAKE_UP if define, it will use double tab wake up. */
#define HIDEEP_LPM_WAKEUP

#define HIDEEP_DD_VERSION_MAJOR	1
#define HIDEEP_DD_VERSION_MINOR 1
#define HIDEEP_DD_DESCRIPTION	"HiDeep reference Device Driver"


/*************************************************************************
 * Firmware name.
 *************************************************************************/
#ifdef CONFIG_CRIMSON_TS
#define HIDEEP_AUTO_FW						"crimson.bin"
#define HIDEEP_MAN_FW						"crimson.bin"
#ifdef CONFIG_AULU_Z
#define FIRMWARE_SIZE (47 * 1024)
#define TEMP_SIZE 1024
#endif
#endif

#define CONFIG_LIME_TS
 #ifdef CONFIG_LIME_TS
#define HIDEEP_AUTO_FW						"lime.bin"
#define HIDEEP_MAN_FW						"lime.bin"
#ifdef CONFIG_AULU_Z
#define FIRMWARE_SIZE (62 * 1024)
#define TEMP_SIZE 2048
#endif
 #endif

/*************************************************************************
 * board porting config
 *************************************************************************/
#define HIDEEP_DEV_NAME						"HiDeep:"
#define HIDEEP_TS_NAME						"HiDeep"
#define HIDEEP_DEBUG_DEVICE_NAME			"hideep_debug"
#define HIDEEP_I2C_NAME						"hideep_ts"
#define HIDEEP_I2C_ADDR						0x6C

/*************************************************************************
 * register addr, touch & key event config
 *************************************************************************/
#define HIDEEP_MT_MAX						10
#define HIDEEP_KEY_MAX				3
#define HIDEEP_EVENT_COUNT_ADDR				0x0240
#define HIDEEP_TOUCH_DATA_ADDR				(HIDEEP_EVENT_COUNT_ADDR+2)
#define HIDEEP_KEY_DATA_ADDR		(HIDEEP_TOUCH_DATA_ADDR + (sizeof(struct hideep_mt_t)) * HIDEEP_MT_MAX)
#define HIDEEP_VR_IMAGE_ADDR				0x1000
#define HIDEEP_DWZ_ADDR						0x02C0
#define NVM_DWZ_LEN							(0x0400 - 0x280)	/* why 0x280? */

/*************************************************************************
 * multi-touch definitions.
 *************************************************************************/
/* multi touch event bit */
#define HIDEEP_MT_ALWAYS_REPORT				0
#define HIDEEP_MT_TOUCHED					1
#define HIDEEP_MT_FIRST_CONTACT				2
#define HIDEEP_MT_DRAG_MOVE					3
#define HIDEEP_MT_RELEASED					4
#define HIDEEP_MT_PINCH						5
#define HIDEEP_MT_PRESSURE					6

/* key event bit */
#define HIDEEP_KEY_RELEASED						0x20
#define HIDEEP_KEY_PRESSED						0x40
#define HIDEEP_KEY_FIRST_PRESSED				0x80
#define HIDEEP_KEY_PRESSED_MASK					0xC0

/*************************************************************************
 * command list
 *************************************************************************/
#define HIDEEP_EMPTY			(0x0000)
#define HIDEEP_RESET			(0x0802)
#define HIDEEP_TEST_MODE		(0x0804)
#define HIDEEP_SLEEP_MODE		(0x080F)
#define HIDEEP_SELF_TEST_MODE	(0x0810)
#define HIDEEP_LPM_MODE			(0x0815)


/*************************************************************************
 * HIDEEP PROTOCOL
 *************************************************************************/
#ifdef HIDEEP_PROTOCOL_2_0
struct hideep_mt_t {
	u16 x;
	u16 y;
	u16 z;
	u8 w;
	u8 flag;
	u8 type;
	u8 index;
};
#else
struct hideep_mt_t {
	u8 flag;
	u8 index;
	u16 x;
	u16 y;
	u8 z;
	u8 w;
};
#endif

struct hideep_key_t {
	u8 flag;
	u8 z;
};

enum e_dev_state {
	power_init = 1,
	power_normal,
	power_sleep,
	power_updating,
	power_debugging,
};

/*************************************************************************
 * panel info & firmware info
 *************************************************************************/
#ifdef HIDEEP_PANEL_INFO
struct panel_info_t {
	u16 vendor;
	u16 product;
	u16 version;
	u16 dp_w;	/* display width */
	u16 dp_h;	/* display height */
	u8 tx;
	u8 rx;
	u8 tx_stride;
	u8 key_nr;
	u16 key[10];
};
#endif

struct dwz_info_t {
	u32 c_begin;	/* code start address */
	u16 c_crc[6];	/* code crc */

	u32 d_begin;	/* custom code */
	u16 d_len;		/* custom code length */
	u16 rsv0;

	u32 v_begin;	/* vreg code */
	u16 v_len;		/* vreg code length */
	u16 rsv1;

	u32 f_begin;	/* vreg code */
	u16 f_len;		/* vreg code length */
	u16 rsv2;

	u16 ver_b;		/* version information */
	u16 ver_c;
	u16 ver_d;
	u16 ver_v;

	u8 factory_id;
	u8 panel_type;
	u8 model_name[6];	/* model name */
	u16 product_code;	/* product code */
	u16 extra_option;	/* extra option */

	u16 ver_ft_major;
	u16 ver_ft_minor;
#ifdef HIDEEP_PANEL_INFO
	struct panel_info_t panel;
#endif
};

/*************************************************************************
 * driver information for hideep_t
 *************************************************************************/
struct hideep_platform_data_t {
	u32 version;
	u32 max_x;		/* max coord x */
	u32 max_y;		/* max coord y */
	u32 max_z;		/* max coord z */
	u32 max_w;		/* max coord w */

	int irq_gpio;
	int reset_gpio;
	int scl_gpio;

	const char *regulator_vdd;	/* vdd regulator name */
	const char *regulator_vid;	/* vid regulator name */

	struct regulator *vcc_vdd;  /* main voltage */
	struct regulator *vcc_vid;  /* I/O voltage */

	const char *int_mode;		/* interrupt mode */

	struct pinctrl *pinctrl;
	struct pinctrl_state *reset_up;
	struct pinctrl_state *reset_down;
	struct pinctrl_state *intb_ctrl;
};

struct hideep_debug_dev_t {
	u8 *p_data;
	struct cdev cdev;
	wait_queue_head_t i_packet;
	u32 ready;
	u8 *vr_buff;
	u8 *im_buff;
	u16 im_size;
	u16 vr_size;
	u8 im_r_en;
	bool release_flag;
#ifdef HIDEEP_SELFTEST_MODE
	u8 *self_buff;
	u16 frame_size;
	u8 *rawdata;
	int tx_num;
	int rx_num;
#endif
	struct hideep_t *ts;
};

struct hideep_debug_cfg_t {
	u16 im_size;
	u16 vr_size;
};

struct hideep_t {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct hideep_platform_data_t *p_data;

	struct mutex dev_mutex;
	struct mutex i2c_mutex;

#ifdef MTK_PLATFORM
	/* struct tpd_device *tpd; */
#endif
#ifdef HIDEEP_DEBUG_DEVICE
	u32 debug_dev_no;
	struct hideep_debug_dev_t debug_dev;
	struct class *debug_class;
#endif

	u32 vr_addr;
	u32 vr_data;
	u32 vr_size;

#ifdef HIDEEP_LPM_WAKEUP
	bool lpm_mode;
#endif

	bool suspended;
	enum e_dev_state dev_state;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif

	s32 irq;
	long tch_bit;
	u32 tch_count;
	u32 key_count;
	u32 lpm_count;

	struct hideep_mt_t touch_evt[HIDEEP_MT_MAX];
	struct hideep_key_t key_evt[HIDEEP_KEY_MAX];

#ifdef CONFIG_AULU_Z
	u16 z_buffer;
	bool z_status;
	u32 z_index;

	u32 z_calib_start;
	u32 z_calib_end;
	bool z_flag_calib2;
	bool z_ready_flag;
	u16 z_data[4096];
	bool z_flag_ready;
	short z_value_buf;
#endif

	u8 i2c_buf[256];

	struct dwz_info_t *dwz_info;

	bool manually_update;
	struct workqueue_struct *p_workqueue_fw;
	struct delayed_work work_fwu;

	u16 addr;
	struct work_struct work;
	u32 flags;
};

extern int loglevel;
extern struct hideep_t *g_ts;

/*************************************************************************
 * function define
 *************************************************************************/
int hideep_sysfs_init(struct hideep_t *ts);
int hideep_sysfs_exit(struct hideep_t *ts);
int hideep_load_ucode(struct device *dev, const char *fn);
int hideep_load_dwz(struct hideep_t *ts);
int hideep_i2c_read(struct hideep_t *ts, u16 addr, u16 len, u8 *buf);
int hideep_i2c_write(struct hideep_t *ts, u16 addr, u16 len, u8 *buf);
#ifdef HIDEEP_DEBUG_DEVICE
int hideep_debug_init(struct hideep_t *ts);
void hideep_debug_uninit(struct hideep_t *ts);
#endif
int hideep_fuse_ucode(struct i2c_client *client, u8 *code, size_t len, int offset);
void hideep_reset_ic(struct hideep_t *ts);
int hideep_enter_pgm(struct i2c_client *client);
void hideep_power(struct hideep_t *ts, int on);

/*************************************************************************
 * debug message
 *************************************************************************/
#define HIDEEP_LOG_LEVEL_DEBUG	2
#define HIDEEP_LOG_LEVEL_XY		3
#define HIDEEP_LOG_LEVEL_I2C	4
#define HIDEEP_INFO(a, arg...) pr_warn(HIDEEP_DEV_NAME "info %s:%d " a "\n", __func__, __LINE__, ##arg)
#define HIDEEP_WARN(a, arg...) pr_warn(HIDEEP_DEV_NAME "warning %s:%d " a "\n", __func__, __LINE__, ##arg)
#define HIDEEP_ERR(a, arg...) pr_err(HIDEEP_DEV_NAME "error %s:%d " a "\n", __func__, __LINE__, ##arg)
#define HIDEEP_DBG(a, arg...)\
do {\
	if (loglevel > HIDEEP_LOG_LEVEL_DEBUG)\
		pr_err(HIDEEP_DEV_NAME "debug %s:%d "a"\n", __func__, __LINE__, ##arg);\
} while (0)

#define HIDEEP_XY(a, arg...)\
do {\
	if (loglevel > HIDEEP_LOG_LEVEL_XY)\
		pr_err(HIDEEP_DEV_NAME " %s:%d " a "\n", __func__, __LINE__, ##arg);\
} while (0)

#define HIDEEP_I2C(a, arg...)\
do {\
	if (loglevel > HIDEEP_LOG_LEVEL_I2C)\
		pr_warn(HIDEEP_DEV_NAME " %s:%d " a "\n", __func__, __LINE__, ##arg);\
} while (0)

#endif /* _LINUX_HIDEEP_TS_H */
