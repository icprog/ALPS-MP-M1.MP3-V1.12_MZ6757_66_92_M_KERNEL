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

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/suspend.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <linux/reboot.h>

#include "tcpci_core.h"
#include "mtk_direct_charge_vdm.h"
#include "tcpm.h"

#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/upmu_common.h>
#include <mach/mt_charging.h>

#include "mtk_pdc_intf.h"
#include "mtk_pep30_intf.h"

static struct tcpc_device *tcpc;

bool mtk_pdc_check_charger(void)
{
	if (mtk_check_pe_ready_snk() == true && mtk_is_TA_support_pep30() == false)
		return true;

	return false;
}

void mtk_pdc_plugout_reset(void)
{


}

void mtk_pdc_init_table(void)
{
	struct tcpm_remote_power_cap cap;
	int i;

	cap.nr = 0;
	if (mtk_check_pe_ready_snk() == true && mtk_is_TA_support_pep30() == false) {
		tcpm_get_remote_power_cap(tcpc, &cap);
		if (cap.nr != 0)
			for (i = 0; i < cap.nr; i++)
				battery_log(BAT_LOG_CRTI, "[%s]:%d %d %d\n",
					__func__, i, cap.mv[i], cap.ma[i]);
	}

	battery_log(BAT_LOG_CRTI, "[%s]:=>%d\n", __func__, cap.nr);
}




void mtk_pdc_init(void)
{
	tcpc = tcpc_dev_get_by_name("type_c_port0");
}




