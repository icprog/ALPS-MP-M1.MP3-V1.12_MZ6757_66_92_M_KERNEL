/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#define pr_fmt(fmt) "memory-ssvp: " fmt
#define CONFIG_MTK_MEMORY_SSVP_DEBUG

#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/printk.h>
#include <linux/cma.h>
#include <linux/debugfs.h>
#include <linux/stat.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <asm/page.h>
#include <asm-generic/memory_model.h>

#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>
#include "sh_svp.h"

#define _SSVP_MBSIZE_ (CONFIG_MTK_SVP_RAM_SIZE + CONFIG_MTK_TUI_RAM_SIZE)
#define _SVP_MBSIZE CONFIG_MTK_SVP_RAM_SIZE
#define _TUI_MBSIZE CONFIG_MTK_TUI_RAM_SIZE

#define COUNT_DOWN_MS 4000
#define	COUNT_DOWN_INTERVAL 200

#include <mt-plat/aee.h>

#include "mt-plat/mtk_meminfo.h"

static unsigned long svp_usage_count;
static int ref_count;

#ifdef CONFIG_MTK_MEMORY_SSVP_WRAP
static struct page *wrap_svp_pages;
static struct page *wrap_tui_pages;
#endif


enum ssvp_subtype {
	SSVP_SVP,
	SSVP_TUI,
	__MAX_NR_SSVPSUBS,
};

enum svp_state {
	SVP_STATE_DISABLED,
	SVP_STATE_ONING_WAIT,
	SVP_STATE_ONING,
	SVP_STATE_ON,
	SVP_STATE_OFFING,
	SVP_STATE_OFF,
	NR_STATES,
};

const char *const svp_state_text[NR_STATES] = {
	[SVP_STATE_DISABLED]   = "[DISABLED]",
	[SVP_STATE_ONING_WAIT] = "[ONING_WAIT]",
	[SVP_STATE_ONING]      = "[ONING]",
	[SVP_STATE_ON]         = "[ON]",
	[SVP_STATE_OFFING]     = "[OFFING]",
	[SVP_STATE_OFF]        = "[OFF]",
};

struct SSVP_Region {
	unsigned int state;
	unsigned long count;
	struct page *page;
};

static struct task_struct *_svp_online_task; /* NULL */

static struct cma *cma;

static struct SSVP_Region _svpregs[__MAX_NR_SSVPSUBS];

void zmc_ssvp_init(struct cma *zmc_cma)
{
	phys_addr_t base = cma_get_base(zmc_cma), size = cma_get_size(zmc_cma);

	cma = zmc_cma;
	pr_info("%s, base: %pa, size: %pa\n", __func__, &base, &size);
}

struct single_cma_registration memory_ssvp_registration = {
	.align = 0x8000000,
	.size = (_SSVP_MBSIZE_ * SZ_1M),
	.name = "memory-ssvp",
	.init = zmc_ssvp_init,
	.prio = ZMC_SSVP,
};

static int __init memory_ssvp_init(struct reserved_mem *rmem)
{
	int ret;

	pr_info("%s, name: %s, base: 0x%pa, size: 0x%pa\n",
		 __func__, rmem->name,
		 &rmem->base, &rmem->size);

	/* init cma area */
	ret = cma_init_reserved_mem(rmem->base, rmem->size , 0, &cma);

	if (ret) {
		pr_err("%s cma failed, ret: %d\n", __func__, ret);
		return 1;
	}

	return 0;
}
RESERVEDMEM_OF_DECLARE(memory_ssvp, "mediatek,memory-ssvp",
			memory_ssvp_init);

/*
 * Check whether memory_ssvp is initialized
 */
bool memory_ssvp_inited(void)
{
	return (cma != NULL);
}

/*
 * memory_ssvp_cma_base - query the cma's base
 */
phys_addr_t memory_ssvp_cma_base(void)
{
	return cma_get_base(cma);
}

/*
 * memory_ssvp_cma_size - query the cma's size
 */
phys_addr_t memory_ssvp_cma_size(void)
{
	return cma_get_size(cma);
}

int tui_region_offline(phys_addr_t *pa, unsigned long *size)
{
	struct page *page;
	int retval = 0;

	pr_alert("%s %d: tui to offline enter state: %s\n", __func__, __LINE__,
			svp_state_text[_svpregs[SSVP_TUI].state]);

	if (_svpregs[SSVP_TUI].state == SVP_STATE_ON) {
		_svpregs[SSVP_TUI].state = SVP_STATE_OFFING;

#ifdef CONFIG_MTK_MEMORY_SSVP_WRAP
		page = wrap_tui_pages;
#else
		page = zmc_cma_alloc(cma, _svpregs[SSVP_TUI].count,
				fls((memory_ssvp_registration.align - 1) >> PAGE_SHIFT), &memory_ssvp_registration);
		if (page)
			svp_usage_count += _svpregs[SSVP_TUI].count;
#endif

		if (page) {
			_svpregs[SSVP_TUI].page = page;
			_svpregs[SSVP_TUI].state = SVP_STATE_OFF;

			if (pa)
				*pa = page_to_phys(page);
			if (size)
				*size = _svpregs[SSVP_TUI].count << PAGE_SHIFT;

			pr_alert("%s %d: pa %llx, size 0x%lx\n",
					__func__, __LINE__,
					page_to_phys(page),
					_svpregs[SSVP_TUI].count << PAGE_SHIFT);
		} else {
			_svpregs[SSVP_TUI].state = SVP_STATE_ON;
			retval = -EAGAIN;
		}
	} else {
		retval = -EBUSY;
	}

	pr_alert("%s %d: tui to offline leave state: %s, retval: %d\n",
			__func__, __LINE__,
			svp_state_text[_svpregs[SSVP_TUI].state], retval);

	return retval;
}
EXPORT_SYMBOL(tui_region_offline);

int tui_region_online(void)
{
	int retval = 0;
	bool retb;

	pr_alert("%s %d: tui to online enter state: %s\n", __func__, __LINE__,
			svp_state_text[_svpregs[SSVP_TUI].state]);

	if (_svpregs[SSVP_TUI].state == SVP_STATE_OFF) {
		_svpregs[SSVP_TUI].state = SVP_STATE_ONING_WAIT;

#ifdef CONFIG_MTK_MEMORY_SSVP_WRAP
		retb = true;
#else
		retb = cma_release(cma, _svpregs[SSVP_TUI].page, _svpregs[SSVP_TUI].count);

		if (retb == true)
			svp_usage_count -= _svpregs[SSVP_TUI].count;
#endif

		if (retb == true) {
			_svpregs[SSVP_TUI].page = NULL;
			_svpregs[SSVP_TUI].state = SVP_STATE_ON;
		}
	} else {
		retval = -EBUSY;
	}

	pr_alert("%s %d: tui to online leave state: %s, retval: %d\n",
			__func__, __LINE__,
			svp_state_text[_svpregs[SSVP_TUI].state], retval);

	return retval;
}
EXPORT_SYMBOL(tui_region_online);

static ssize_t
tui_cma_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	pr_alert("%s %d: tui state: %s\n", __func__, __LINE__,
			svp_state_text[_svpregs[SSVP_TUI].state]);

	return 0;
}

static ssize_t
tui_cma_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned char val;
	int retval = count;

	pr_alert("%s %d: count: %zu\n", __func__, __LINE__, count);

	if (count >= 2) {
		if (copy_from_user(&val, &buf[0], 1)) {
			retval = -EFAULT;
			goto out;
		}

		if (count == 2 && val == '0')
			tui_region_offline(NULL, NULL);
		else
			tui_region_online();
	}

out:
	return retval;
}

static const struct file_operations tui_cma_fops = {
	.owner =    THIS_MODULE,
	.read  =    tui_cma_read,
	.write =    tui_cma_write,
};

static int _svp_wdt_kthread_func(void *data)
{
	int count_down = COUNT_DOWN_MS / COUNT_DOWN_INTERVAL;

	pr_info("[START COUNT DOWN]: %dms/%dms\n", COUNT_DOWN_MS, COUNT_DOWN_INTERVAL);

	for (; count_down > 0; --count_down) {
		msleep(COUNT_DOWN_INTERVAL);

		/*
		 * some component need ssvp memory,
		 * and stop count down watch dog
		 */
		if (ref_count > 0) {
			_svp_online_task = NULL;
			pr_info("[STOP COUNT DOWN]: new request for ssvp\n");
			return 0;
		}

		if (_svpregs[SSVP_SVP].state == SVP_STATE_ON) {
			pr_info("[STOP COUNT DOWN]: SSVP has online\n");
			return 0;
		}
		pr_info("[COUNT DOWN]: %d\n", count_down);

	}
	pr_alert("[COUNT DOWN FAIL]\n");

	pr_alert("Shareable SVP trigger kernel warnin");
	aee_kernel_warning_api("SVP", 0, DB_OPT_DEFAULT|DB_OPT_DUMPSYS_ACTIVITY|DB_OPT_LOW_MEMORY_KILLER
			| DB_OPT_PID_MEMORY_INFO /*for smaps and hprof*/
			| DB_OPT_PROCESS_COREDUMP
			| DB_OPT_DUMPSYS_SURFACEFLINGER
			| DB_OPT_DUMPSYS_GFXINFO
			| DB_OPT_DUMPSYS_PROCSTATS,
			"SVP wdt: SSVP online Failed\nCRDISPATCH_KEY:SVP_SS1",
			"please contact SS memory module owner\n");

	return 0;
}

int svp_start_wdt(void)
{
	_svp_online_task = kthread_create(_svp_wdt_kthread_func, NULL, "svp_online_kthread");
	wake_up_process(_svp_online_task);
	return 0;
}

int svp_region_offline(phys_addr_t *pa, unsigned long *size)
{
	struct page *page;
	int retval = 0;
	int res = 0;

	pr_alert("%s %d: svp to offline enter state: %s\n", __func__, __LINE__,
			svp_state_text[_svpregs[SSVP_SVP].state]);

	if (_svpregs[SSVP_SVP].state == SVP_STATE_ON) {
		_svpregs[SSVP_SVP].state = SVP_STATE_OFFING;

#ifdef CONFIG_MTK_MEMORY_SSVP_WRAP
		page = wrap_svp_pages;
#else
		page = zmc_cma_alloc(cma, _svpregs[SSVP_SVP].count,
				fls((memory_ssvp_registration.align - 1) >> PAGE_SHIFT), &memory_ssvp_registration);

		if (page)
			svp_usage_count += _svpregs[SSVP_SVP].count;
#endif

		if (page) {
			_svpregs[SSVP_SVP].page = page;
			_svpregs[SSVP_SVP].state = SVP_STATE_OFF;

			if (pa)
				*pa = page_to_phys(page);
			if (size)
				*size = _svpregs[SSVP_SVP].count << PAGE_SHIFT;

			pr_alert("%s %d: secmem_enable, res %d pa 0x%llx, size 0x%lx\n",
					__func__, __LINE__, res,
					page_to_phys(page),
					_svpregs[SSVP_SVP].count << PAGE_SHIFT);
		} else {
			_svpregs[SSVP_SVP].state = SVP_STATE_ON;
			retval = -EAGAIN;
		}
	} else {
		retval = -EBUSY;
	}

	pr_alert("%s %d: svp to offline leave state: %s, retval: %d\n",
			__func__, __LINE__, svp_state_text[_svpregs[SSVP_SVP].state], retval);

	return retval;
}
EXPORT_SYMBOL(svp_region_offline);

int svp_region_online(void)
{
	int retval = 0;
	int retb;

	pr_alert("%s %d: svp to online enter state: %s\n", __func__, __LINE__,
			svp_state_text[_svpregs[SSVP_SVP].state]);

	if (_svpregs[SSVP_SVP].state == SVP_STATE_OFF) {
#ifdef CONFIG_MTK_MEMORY_SSVP_WRAP
		retb = true;
#else
		retb = cma_release(cma, _svpregs[SSVP_SVP].page, _svpregs[SSVP_SVP].count);

		if (retb == true)
			svp_usage_count -= _svpregs[SSVP_SVP].count;
#endif

		if (retb == true) {
			_svpregs[SSVP_SVP].page = NULL;
			_svpregs[SSVP_SVP].state = SVP_STATE_ON;
		}
	} else {
		retval = -EBUSY;
	}

	pr_alert("%s %d: svp to online leave state: %s, retval: %d\n",
			__func__, __LINE__, svp_state_text[_svpregs[SSVP_SVP].state], retval);

	return retval;
}
EXPORT_SYMBOL(svp_region_online);

/* any read request will free coherent memory, eg.
 * cat /dev/svp_region
 */
static ssize_t
svp_cma_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	pr_alert("%s %d:svp state: %s\n", __func__, __LINE__,
			svp_state_text[_svpregs[SSVP_SVP].state]);

	if (cma)
		pr_alert("%s %d: svp cma base_pfn: 0x%lx, count %ld\n",
				__func__, __LINE__, (unsigned long)cma_get_base(cma), cma_get_size(cma));

	return 0;
}

/*
 * any write request will alloc coherent memory, eg.
 * echo 0 > /dev/cma_test
 */
static ssize_t
svp_cma_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned char val;
	int retval = count;

	pr_alert("%s %d: count: %zu\n", __func__, __LINE__, count);

	if (count >= 2) {
		if (copy_from_user(&val, &buf[0], 1)) {
			retval = -EFAULT;
			goto out;
		}

		if (count == 2 && val == '0')
			svp_region_offline(NULL, NULL);
		else
			svp_region_online();
	}

out:
	return retval;
}

static long svp_cma_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	pr_alert("%s %d: cmd: %x\n", __func__, __LINE__, cmd);

	switch (cmd) {
	case SVP_REGION_IOC_ONLINE:
		pr_info("Called phased out ioctl: %s\n", "SVP_REGION_IOC_ONLINE");
		/* svp_region_online(); */
		break;
	case SVP_REGION_IOC_OFFLINE:
		pr_info("Called phased out ioctl: %s\n", "SVP_REGION_IOC_OFFLINE");
		/* svp_region_offline(NULL, NULL); */
		break;
	case SVP_REGION_ACQUIRE:
		if (ref_count == 0 && _svp_online_task != NULL)
			_svp_online_task = NULL;

		ref_count++;
		break;
	case SVP_REGION_RELEASE:
		ref_count--;

		if (ref_count == 0)
			svp_start_wdt();
		break;
	default:
		/* IONMSG("ion_ioctl : No such command!! 0x%x\n", cmd); */
		return -ENOTTY;
	}

	return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long svp_cma_COMPAT_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	pr_alert("%s %d: cmd: %x\n", __func__, __LINE__, cmd);

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	return filp->f_op->unlocked_ioctl(filp, cmd,
							(unsigned long)compat_ptr(arg));
}

#else

#define svp_cma_COMPAT_ioctl  NULL

#endif

static const struct file_operations svp_cma_fops = {
	.owner =    THIS_MODULE,
	.read  =    svp_cma_read,
	.write =    svp_cma_write,
	.unlocked_ioctl = svp_cma_ioctl,
	.compat_ioctl   = svp_cma_COMPAT_ioctl,
};

static int memory_ssvp_show(struct seq_file *m, void *v)
{
	phys_addr_t cma_base = cma_get_base(cma);
	phys_addr_t cma_end = cma_base + cma_get_size(cma);

	seq_printf(m, "cma info: [%pa-%pa] (0x%lx)\n",
			&cma_base, &cma_end,
			cma_get_size(cma));

	seq_printf(m, "cma info: base %pa pfn [%lu-%lu] count %lu\n",
			&cma_base,
			__phys_to_pfn(cma_base), __phys_to_pfn(cma_end),
			cma_get_size(cma) >> PAGE_SHIFT);

#ifdef CONFIG_MTK_MEMORY_SSVP_WRAP
	seq_printf(m, "cma info: svp wrap: %pa\n", &wrap_svp_pages);
	seq_printf(m, "cma info: tui wrap: %pa\n", &wrap_tui_pages);
#endif

	seq_printf(m, "svp region base:0x%llx, count %lu, state %s\n",
			_svpregs[SSVP_SVP].page == NULL ? 0 : page_to_phys(_svpregs[SSVP_SVP].page),
			_svpregs[SSVP_SVP].count,
			svp_state_text[_svpregs[SSVP_SVP].state]);

	seq_printf(m, "tui region count %lu, state %s\n",
			_svpregs[SSVP_TUI].count,
			svp_state_text[_svpregs[SSVP_TUI].state]);

	seq_printf(m, "cma usage: %lu pages\n", svp_usage_count);

	return 0;
}

static int memory_ssvp_open(struct inode *inode, struct file *file)
{
	return single_open(file, &memory_ssvp_show, NULL);
}

static const struct file_operations memory_ssvp_fops = {
	.open		= memory_ssvp_open,
	.read		= seq_read,
	.release	= single_release,
};

static int __init memory_ssvp_debug_init(void)
{
	int ret = 0;
	struct proc_dir_entry *procfs_entry;

	struct dentry *dentry;

	if (!cma) {
		pr_err("memory-ssvp cma is not inited\n");
		return 1;
	}

#ifdef CONFIG_MTK_MEMORY_SSVP_WRAP
	if (_SVP_MBSIZE > 0) {
		wrap_svp_pages = zmc_cma_alloc(cma, _SVP_MBSIZE * SZ_1M >> PAGE_SHIFT,
				fls((memory_ssvp_registration.align - 1) >> PAGE_SHIFT), &memory_ssvp_registration);
		svp_usage_count = _SVP_MBSIZE * SZ_1M >> PAGE_SHIFT;
	} else
		pr_err("wrap_svp_pages is not inited\n");

	if (_TUI_MBSIZE > 0) {
		wrap_tui_pages = zmc_cma_alloc(cma, _TUI_MBSIZE * SZ_1M >> PAGE_SHIFT,
				fls((memory_ssvp_registration.align - 1) >> PAGE_SHIFT), &memory_ssvp_registration);
		svp_usage_count = _TUI_MBSIZE * SZ_1M >> PAGE_SHIFT;
	} else
		pr_err("wrap_tui_pages is not inited\n");

#endif

	dentry = debugfs_create_file("memory-ssvp", S_IRUGO, NULL, NULL,
					&memory_ssvp_fops);
	if (!dentry)
		pr_warn("Failed to create debugfs memory_ssvp file\n");


	/* svp is enabled with a given size */
	if (_SVP_MBSIZE > 0) {
		procfs_entry = proc_create("svp_region", 0, NULL, &svp_cma_fops);

		if (!procfs_entry)
			pr_warn("Failed to create procfs svp_region file\n");

		_svpregs[SSVP_SVP].count = (_SVP_MBSIZE * SZ_1M) >> PAGE_SHIFT;
		_svpregs[SSVP_SVP].state = SVP_STATE_ON;

		pr_alert("%s %d: SVP region is enable with size: %d mB",
			__func__, __LINE__, _SVP_MBSIZE);
	} else
		_svpregs[SSVP_SVP].state = SVP_STATE_DISABLED;


	/* TUI is enabled with a given size */
	if (_TUI_MBSIZE > 0) {
		proc_create("tui_region", 0, NULL, &tui_cma_fops);

		if (!procfs_entry)
			pr_warn("Failed to create procfs tui_region file\n");

		_svpregs[SSVP_TUI].count = (_TUI_MBSIZE * SZ_1M) >> PAGE_SHIFT;
		_svpregs[SSVP_TUI].state = SVP_STATE_ON;

		pr_alert("%s %d: TUI region is enable with size: %d mB",
			__func__, __LINE__, _TUI_MBSIZE);
	} else
		_svpregs[SSVP_TUI].state = SVP_STATE_DISABLED;

	return ret;
}
late_initcall(memory_ssvp_debug_init);