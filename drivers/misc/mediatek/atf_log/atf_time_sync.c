/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/sched.h>        /* local_clock */
#include <linux/syscore_ops.h>
#include <mach/mt_secure_api.h>

#ifdef MTK_SIP_KERNEL_TIME_SYNC
static void atf_time_sync_resume(void)
{
	/* Get local_clock and sync to ATF */
	u64 time_to_sync = local_clock();

#ifdef CONFIG_ARM64
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC, time_to_sync, 0, 0);
#else
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC, (u32)time_to_sync, (u32)(time_to_sync >> 32), 0);
#endif
	pr_notice("atf_time_sync: resume sync");
}

static struct syscore_ops atf_time_sync_syscore_ops = {
	.resume = atf_time_sync_resume,
};

static int __init atf_time_sync_init(void)
{
	/* register module driver */
	u64 time_to_sync;

	register_syscore_ops(&atf_time_sync_syscore_ops);

	/* Get local_clock and sync to ATF */
	time_to_sync = local_clock();

#ifdef CONFIG_ARM64
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC, time_to_sync, 0, 0);
#else
	mt_secure_call(MTK_SIP_KERNEL_TIME_SYNC, (u32)time_to_sync, (u32)(time_to_sync >> 32), 0);
#endif
	pr_notice("atf_time_sync: inited");

	return 0;
}

static void __exit atf_time_sync_exit(void)
{
	pr_notice("atf_time_sync: exited");
}

module_init(atf_time_sync_init);
module_exit(atf_time_sync_exit);

#endif
