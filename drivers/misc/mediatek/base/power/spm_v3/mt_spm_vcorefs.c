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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/of_fdt.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include "mt_spm_misc.h"
#include "mt_spm_vcorefs.h"
#include "mt_spm_internal.h"
#include "mt_vcorefs_governor.h"
#include "mt_dvfsrc_reg.h"

/*
 * DVFS level mask and request
 */
#if 1 /* For Elbrus */
#define LV4_MSK ((1U << 0) | (1U << 1) | (1U << 2))
#define LV3_REQ (0x3 << 4)
#define LV2_MSK ((1U << 0) | (1U << 1))
#define LV1_MSK ((1U << 0))
#define LV0_MSK (0x0)
u32 level_mask[NUM_OPP] = { LV4_MSK, LV3_REQ, LV2_MSK, LV1_MSK, LV0_MSK};
#else /* For Whitney */
#define LV4_MSK ((1U << 0) | (1U << 1) | (1U << 2))
#define LV3_MSK ((1U << 0) | (1U << 1))
#define LV2_REQ (0x2 << 4)
#define LV1_MSK ((1U << 0))
#define LV0_MSK (0x0)
u32 level_mask[NUM_OPP] = { LV4_MSK, LV3_MSK, LV2_REQ, LV1_MSK, LV0_MSK};
#endif

/*
 * only for internal debug
 */
#define SPM_VCOREFS_TAG	"[VcoreFS] "
#define spm_vcorefs_err(fmt, args...)	pr_err(SPM_VCOREFS_TAG fmt, ##args)
#define spm_vcorefs_warn(fmt, args...)	pr_warn(SPM_VCOREFS_TAG fmt, ##args)
#define spm_vcorefs_debug(fmt, args...)	pr_debug(SPM_VCOREFS_TAG fmt, ##args)

void __iomem *dvfsrc_base;

#ifdef CONFIG_MTK_RAM_CONSOLE
#define SPM_AEE_RR_REC	1
#else
#define SPM_AEE_RR_REC	0
#endif

#if SPM_AEE_RR_REC
enum spm_vcorefs_step {
	SPM_VCOREFS_ENTER = 0,
	SPM_VCOREFS_B1,
	SPM_VCOREFS_DVFS_START,
	SPM_VCOREFS_B3,
	SPM_VCOREFS_B4,
	SPM_VCOREFS_B5,
	SPM_VCOREFS_DVFS_END,
	SPM_VCOREFS_B7,
	SPM_VCOREFS_B8,
	SPM_VCOREFS_B9,
	SPM_VCOREFS_LEAVE,
};
#endif

static void set_aee_vcore_dvfs_status(int state)
{
#if SPM_AEE_RR_REC /* FIXME: */
	u32 value = aee_rr_curr_vcore_dvfs_status();

	value &= ~(0xFF);
	value |= (state & 0xFF);
	aee_rr_rec_vcore_dvfs_status(value);
#endif
}

static const u32 vcorefs_binary[] = {
	0xa1d58407, 0xa1d70407, 0x81f68407, 0x80358400, 0x1b80001f, 0x20000000,
	0x80300400, 0x80328400, 0xa1d28407, 0x81f20407, 0x81f88407, 0xe8208000,
	0x108c0610, 0x20000000, 0x81431801, 0xd8000245, 0x17c07c1f, 0x80318400,
	0x81f00407, 0xa1dd0407, 0x1b80001f, 0x20000424, 0x81fd0407, 0x18c0001f,
	0x108c0604, 0x1910001f, 0x108c0604, 0x813f8404, 0xe0c00004, 0xc28055c0,
	0x1290041f, 0xa19d8406, 0xa1dc0407, 0x1880001f, 0x00000006, 0xc0c05fc0,
	0x17c07c1f, 0xa0800c02, 0x1300081f, 0xf0000000, 0x17c07c1f, 0x81fc0407,
	0x1b00001f, 0xffffffff, 0x81fc8407, 0x1b80001f, 0x20000004, 0x88c0000c,
	0xffffffff, 0xd8200703, 0x17c07c1f, 0xa1dc0407, 0x1b00001f, 0x00000000,
	0xd0000ba0, 0x17c07c1f, 0xe8208000, 0x108c0610, 0x10000000, 0x1880001f,
	0x108c0028, 0xc0c054a0, 0xe080000f, 0xd82008e3, 0x17c07c1f, 0x81f00407,
	0xa1dc0407, 0x1b00001f, 0x00000006, 0xd0000ba0, 0x17c07c1f, 0xe080001f,
	0x81431801, 0xd8000985, 0x17c07c1f, 0xa0118400, 0x81bd8406, 0xa0128400,
	0xa1dc0407, 0x1b00001f, 0x00000001, 0xc28055c0, 0x129f841f, 0xc28055c0,
	0x1290841f, 0xa1d88407, 0xa1d20407, 0x81f28407, 0xa1d68407, 0xa0100400,
	0xa0158400, 0x81f70407, 0x81f58407, 0xf0000000, 0x17c07c1f, 0x81409801,
	0xd8000d45, 0x17c07c1f, 0x18c0001f, 0x108c0318, 0xc0c04b40, 0x1200041f,
	0x80310400, 0x1b80001f, 0x2000000e, 0xa0110400, 0xe8208000, 0x108c0610,
	0x40000000, 0xe8208000, 0x108c0610, 0x00000000, 0xc28055c0, 0x1291041f,
	0x18c0001f, 0x108c01d0, 0x1910001f, 0x108c01d0, 0xa1100404, 0xe0c00004,
	0xa19e0406, 0xa1dc0407, 0x1880001f, 0x00000058, 0xc0c05fc0, 0x17c07c1f,
	0xa0800c02, 0x1300081f, 0xf0000000, 0x17c07c1f, 0x18c0001f, 0x108c01d0,
	0x1910001f, 0x108c01d0, 0x81300404, 0xe0c00004, 0x81409801, 0xd80011e5,
	0x17c07c1f, 0x18c0001f, 0x108c0318, 0xc0c04fe0, 0x17c07c1f, 0xc28055c0,
	0x1291841f, 0x81be0406, 0xa1dc0407, 0x1880001f, 0x00000006, 0xc0c05fc0,
	0x17c07c1f, 0xa0800c02, 0x1300081f, 0xf0000000, 0x17c07c1f, 0xc0804240,
	0x17c07c1f, 0xc0803f80, 0x17c07c1f, 0xe8208000, 0x102101a4, 0x00010000,
	0x18c0001f, 0x10230068, 0x1910001f, 0x10230068, 0x89000004, 0xffffff0f,
	0xe0c00004, 0x1910001f, 0x10230068, 0xe8208000, 0x108c0408, 0x00001fff,
	0xa1d80407, 0xc28055c0, 0x1292041f, 0xa19e8406, 0x80cf1801, 0xa1dc0407,
	0xd8001743, 0x17c07c1f, 0x1880001f, 0x00010060, 0xd0001780, 0x17c07c1f,
	0x1880001f, 0x000100a0, 0xc0c05fc0, 0x17c07c1f, 0xa0800c02, 0x1300081f,
	0xf0000000, 0x17c07c1f, 0x1b80001f, 0x20000fdf, 0x81f80407, 0xe8208000,
	0x108c0408, 0x0000ffff, 0x1910001f, 0x108c0170, 0xd8201903, 0x80c41001,
	0x80cf9801, 0xd8001a23, 0x17c07c1f, 0xc0c03f80, 0x17c07c1f, 0x18c0001f,
	0x10230068, 0x1910001f, 0x10230068, 0xa9000004, 0x000000f0, 0xe0c00004,
	0x1910001f, 0x10230068, 0x1880001f, 0x108c0028, 0xc0c051a0, 0xe080000f,
	0xd8201c63, 0x17c07c1f, 0xa1d80407, 0xd0002100, 0x17c07c1f, 0xe080001f,
	0x1890001f, 0x108c0604, 0x80c68801, 0x81429801, 0xa1400c05, 0xd8001de5,
	0x17c07c1f, 0xc0c05780, 0x17c07c1f, 0xc28055c0, 0x1296841f, 0xe8208000,
	0x102101a8, 0x00070000, 0xc0803c00, 0x17c07c1f, 0xc0803d20, 0x17c07c1f,
	0xc28055c0, 0x1292841f, 0x81be8406, 0xa19f8406, 0x80cf1801, 0xa1dc0407,
	0xd8002043, 0x17c07c1f, 0x1880001f, 0x00000058, 0xd0002080, 0x17c07c1f,
	0x1880001f, 0x00000090, 0xc0c05fc0, 0x17c07c1f, 0xa0800c02, 0x1300081f,
	0xf0000000, 0x17c07c1f, 0x814e9801, 0xd82021e5, 0x17c07c1f, 0xc0c03f80,
	0x17c07c1f, 0x18c0001f, 0x108c038c, 0xe0e00011, 0xe0e00031, 0xe0e00071,
	0xe0e000f1, 0xe0e001f1, 0xe0e003f1, 0xe0e007f1, 0xe0e00ff1, 0xe0e01ff1,
	0xe0e03ff1, 0xe0e07ff1, 0xe0f07ff1, 0x1b80001f, 0x20000020, 0xe0f07ff3,
	0xe0f07ff2, 0x80350400, 0x1b80001f, 0x2000001a, 0x80378400, 0x1b80001f,
	0x20000208, 0x80338400, 0x1b80001f, 0x2000001a, 0x81f98407, 0xc28055c0,
	0x1293041f, 0xa19f0406, 0x80ce9801, 0xa1dc0407, 0xd80026c3, 0x17c07c1f,
	0x1880001f, 0x00000090, 0xd00027e0, 0x17c07c1f, 0x80cf9801, 0xd80027a3,
	0x17c07c1f, 0x1880001f, 0x000100a0, 0xd00027e0, 0x17c07c1f, 0x1880001f,
	0x000080a0, 0xc0c05fc0, 0x17c07c1f, 0xa0800c02, 0x1300081f, 0xc0c04780,
	0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x814e9801, 0xd8202985, 0x17c07c1f,
	0xc0c03f80, 0x17c07c1f, 0xa1d98407, 0xa0138400, 0xa0178400, 0xa0150400,
	0x18c0001f, 0x108c038c, 0xe0f07ff3, 0xe0f07ff1, 0xe0e00ff1, 0xe0e000f1,
	0xe0e00001, 0xc28055c0, 0x1293841f, 0x81bf0406, 0x80ce9801, 0xa1dc0407,
	0xd8002c43, 0x17c07c1f, 0x1880001f, 0x00000058, 0xd0002d60, 0x17c07c1f,
	0x80cf9801, 0xd8202d23, 0x17c07c1f, 0x1880001f, 0x00010060, 0xd0002d60,
	0x17c07c1f, 0x1880001f, 0x00008060, 0xc0c05fc0, 0x17c07c1f, 0xa0800c02,
	0x1300081f, 0xc0c04780, 0x17c07c1f, 0xf0000000, 0x17c07c1f, 0xc0c03f80,
	0x17c07c1f, 0xc28055c0, 0x1294041f, 0xa19f8406, 0x80cf1801, 0xa1dc0407,
	0xd8003003, 0x17c07c1f, 0x1880001f, 0x00010060, 0xd0003040, 0x17c07c1f,
	0x1880001f, 0x000100a0, 0xc0c05fc0, 0x17c07c1f, 0xa0800c02, 0x1300081f,
	0xf0000000, 0x17c07c1f, 0x1880001f, 0x108c0028, 0xc0c051a0, 0xe080000f,
	0xd8003443, 0x17c07c1f, 0xe080001f, 0xc0803c00, 0x17c07c1f, 0xc28055c0,
	0x1294841f, 0x81bf8406, 0x80cf1801, 0xa1dc0407, 0xd8003383, 0x17c07c1f,
	0x1880001f, 0x00008060, 0xd00033c0, 0x17c07c1f, 0x1880001f, 0x000080a0,
	0xc0c05fc0, 0x17c07c1f, 0xa0800c02, 0x1300081f, 0xf0000000, 0x17c07c1f,
	0x814e9801, 0xd8203525, 0x17c07c1f, 0xc0c03f80, 0x17c07c1f, 0xc0c05cc0,
	0x17c07c1f, 0x18c0001f, 0x108c0404, 0x1910001f, 0x108c0404, 0xa1108404,
	0xe0c00004, 0x1b80001f, 0x20000104, 0xc0c05ea0, 0x17c07c1f, 0xc28055c0,
	0x1295041f, 0xa19d0406, 0xa1dc0407, 0x1890001f, 0x108c0148, 0xa0978402,
	0x80b70402, 0x1300081f, 0xc0c04780, 0x17c07c1f, 0xf0000000, 0x17c07c1f,
	0x814e9801, 0xd82038e5, 0x17c07c1f, 0xc0c03f80, 0x17c07c1f, 0xc0c05cc0,
	0x17c07c1f, 0x18c0001f, 0x108c0404, 0x1910001f, 0x108c0404, 0x81308404,
	0xe0c00004, 0x1b80001f, 0x20000104, 0xc0c05ea0, 0x17c07c1f, 0xc28055c0,
	0x1295841f, 0x81bd0406, 0xa1dc0407, 0x1890001f, 0x108c0148, 0x80b78402,
	0xa0970402, 0x1300081f, 0xc0c04780, 0x17c07c1f, 0xf0000000, 0x17c07c1f,
	0xa0188400, 0xa0190400, 0xa0110400, 0x80398400, 0x803a0400, 0x803a8400,
	0x803b0400, 0xf0000000, 0x17c07c1f, 0xa1da0407, 0xa1dd8407, 0x803b8400,
	0xa0168400, 0xa0140400, 0xe8208000, 0x108c0320, 0x00000f0d, 0xe8208000,
	0x108c0320, 0x00000f0f, 0xe8208000, 0x108c0320, 0x00000f1e, 0xe8208000,
	0x108c0320, 0x00000f12, 0xf0000000, 0x17c07c1f, 0xa01b0400, 0xa01a8400,
	0xa01c0400, 0xa01a0400, 0x1b80001f, 0x20000004, 0xa0198400, 0x1b80001f,
	0x20000004, 0x80388400, 0x803c0400, 0x80310400, 0x1b80001f, 0x20000004,
	0x80390400, 0x81f08407, 0x808ab401, 0xd8004182, 0x17c07c1f, 0x80380400,
	0xf0000000, 0x17c07c1f, 0xe8208000, 0x108c0320, 0x00000f16, 0xe8208000,
	0x108c0320, 0x00000f1e, 0xe8208000, 0x108c0320, 0x00000f0e, 0x1b80001f,
	0x2000001b, 0xe8208000, 0x108c0320, 0x00000f0c, 0xe8208000, 0x108c0320,
	0x00000f0d, 0xe8208000, 0x108c0320, 0x00000e0d, 0xe8208000, 0x108c0320,
	0x00000c0d, 0xe8208000, 0x108c0320, 0x0000080d, 0xe8208000, 0x108c0320,
	0x0000000d, 0x80340400, 0x80368400, 0x1b80001f, 0x20000209, 0xa01b8400,
	0x1b80001f, 0x20000209, 0x1b80001f, 0x20000209, 0x81fa0407, 0x81fd8407,
	0xf0000000, 0x17c07c1f, 0x816f9801, 0x814e9805, 0xd8204b05, 0x17c07c1f,
	0x1880001f, 0x108c0028, 0xe080000f, 0x1111841f, 0xa1d08407, 0xd8204924,
	0x80eab401, 0xd80048a3, 0x01200404, 0x81f08c07, 0x1a00001f, 0x108c00b0,
	0xe2000003, 0xd8204aa3, 0x17c07c1f, 0xa1dc0407, 0x1b00001f, 0x00000000,
	0x81fc0407, 0xd0004b00, 0x17c07c1f, 0xe080001f, 0xc0c03c00, 0x17c07c1f,
	0xf0000000, 0x17c07c1f, 0xe0f07f16, 0x1380201f, 0xe0f07f1e, 0x1380201f,
	0xe0f07f0e, 0xe0f07f0c, 0x1900001f, 0x108c0374, 0xe120003e, 0xe120003c,
	0xe1200038, 0xe1200030, 0xe1200020, 0xe1200000, 0x1880001f, 0x108c03a4,
	0x1900001f, 0x0400fffc, 0xe0800004, 0x1900001f, 0x0000fffc, 0xe0800004,
	0x1b80001f, 0x20000104, 0xe0f07f0d, 0xe0f07e0d, 0x1b80001f, 0x20000104,
	0xe0f07c0d, 0x1b80001f, 0x20000104, 0xe0f0780d, 0x1b80001f, 0x20000104,
	0xe0f0700d, 0xf0000000, 0x17c07c1f, 0x1900001f, 0x108c0374, 0xe120003f,
	0x1900001f, 0x108c03a4, 0x1880001f, 0x0c00fffc, 0xe1000002, 0xe0f07f0d,
	0xe0f07f0f, 0xe0f07f1e, 0xe0f07f12, 0xf0000000, 0x17c07c1f, 0xa0180400,
	0x1111841f, 0xa1d08407, 0xd8205284, 0x80eab401, 0xd8005203, 0x01200404,
	0xd82053c3, 0x17c07c1f, 0xa1dc0407, 0x1b00001f, 0x00000000, 0x81fc0407,
	0x81f08407, 0xe8208000, 0x108c00b0, 0x00000001, 0xf0000000, 0x17c07c1f,
	0xa1d10407, 0x1b80001f, 0x20000020, 0xf0000000, 0x17c07c1f, 0xa1d00407,
	0x1b80001f, 0x20000208, 0x10c07c1f, 0x1900001f, 0x108c00b0, 0xe1000003,
	0xf0000000, 0x17c07c1f, 0x18c0001f, 0x108c0604, 0x1910001f, 0x108c0604,
	0xa1002804, 0xe0c00004, 0xf0000000, 0x17c07c1f, 0xa1d40407, 0x1391841f,
	0xa1d90407, 0x1392841f, 0xf0000000, 0x17c07c1f, 0x18c0001f, 0x100040f4,
	0x19300003, 0x17c07c1f, 0x813a0404, 0xe0c00004, 0x1b80001f, 0x20000003,
	0x18c0001f, 0x10004110, 0x19300003, 0x17c07c1f, 0xa11e8404, 0xe0c00004,
	0x1b80001f, 0x20000004, 0x18c0001f, 0x100041ec, 0x19300003, 0x17c07c1f,
	0x81380404, 0xe0c00004, 0x18c0001f, 0x100040f4, 0x19300003, 0x17c07c1f,
	0xa11a8404, 0xe0c00004, 0x18c0001f, 0x100041dc, 0x19300003, 0x17c07c1f,
	0x813d0404, 0xe0c00004, 0x18c0001f, 0x10004110, 0x19300003, 0x17c07c1f,
	0x813e8404, 0xe0c00004, 0xf0000000, 0x17c07c1f, 0x814f1801, 0xd8005e65,
	0x17c07c1f, 0x80350400, 0x1b80001f, 0x2000001a, 0x80378400, 0x1b80001f,
	0x20000208, 0x80338400, 0x1b80001f, 0x2000001a, 0x81f98407, 0xf0000000,
	0x17c07c1f, 0x814f1801, 0xd8005f85, 0x17c07c1f, 0xa1d98407, 0xa0138400,
	0xa0178400, 0xa0150400, 0xf0000000, 0x17c07c1f, 0x10c07c1f, 0x810d1801,
	0x810a1804, 0x816d1801, 0x814a1805, 0xa0d79003, 0xa0d71403, 0xf0000000,
	0x17c07c1f, 0x1b80001f, 0x20000300, 0xf0000000, 0x17c07c1f, 0xe8208000,
	0x110e0014, 0x00000002, 0xe8208000, 0x110e0020, 0x00000001, 0xe8208000,
	0x110e0004, 0x000000d6, 0x1a00001f, 0x110e0000, 0x1880001f, 0x110e0024,
	0x18c0001f, 0x20000152, 0xd820658a, 0x17c07c1f, 0xe220000a, 0xe22000f6,
	0xe8208000, 0x110e0024, 0x00000001, 0x1b80001f, 0x20000152, 0xe220008a,
	0xe2200001, 0xe8208000, 0x110e0024, 0x00000001, 0x1b80001f, 0x20000152,
	0xd0006740, 0x17c07c1f, 0xe220008a, 0xe2200000, 0xe8208000, 0x110e0024,
	0x00000001, 0x1b80001f, 0x20000152, 0xe220000a, 0xe22000f4, 0xe8208000,
	0x110e0024, 0x00000001, 0x1b80001f, 0x20000152, 0xf0000000, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x1840001f, 0x00000001, 0xa1d48407, 0xe8208000,
	0x108c0620, 0x0000000e, 0x1990001f, 0x108c0600, 0xa9800006, 0xfe000000,
	0x1890001f, 0x108c061c, 0x80800402, 0xa19c0806, 0xe8208000, 0x108c0604,
	0x00000000, 0x81fc0407, 0x1b00001f, 0xffffffff, 0xa1dc0407, 0x1b00001f,
	0x00000000, 0x81401801, 0xd80103c5, 0x17c07c1f, 0x1b80001f, 0x5000aaaa,
	0xd00104a0, 0x17c07c1f, 0x1b80001f, 0xd00f0000, 0x81fc8407, 0x8880000c,
	0xffffffff, 0xd8011b02, 0x17c07c1f, 0xe8208000, 0x108c0408, 0x00001fff,
	0xe8208000, 0x108c040c, 0xe0003fff, 0xc0c05400, 0x81401801, 0xd8010885,
	0x17c07c1f, 0x80c41801, 0xd8010663, 0x17c07c1f, 0x81f60407, 0x18d0001f,
	0x108c0254, 0xc0c11f20, 0x17c07c1f, 0x18d0001f, 0x108c0250, 0xc0c12000,
	0x17c07c1f, 0x18c0001f, 0x108c0200, 0xc0c120c0, 0x17c07c1f, 0xe8208000,
	0x108c0260, 0x00000007, 0xc28055c0, 0x1296041f, 0x81401801, 0xd8010a25,
	0x17c07c1f, 0xa1dc0407, 0x1b00001f, 0x00000000, 0x1b80001f, 0x30000004,
	0x81fc8407, 0x8880000c, 0xffffffff, 0xd80117c2, 0x17c07c1f, 0x81489801,
	0xd8010b25, 0x17c07c1f, 0x18c0001f, 0x108c037c, 0xe0e00013, 0xe0e00011,
	0xe0e00001, 0x81481801, 0xd8010c65, 0x17c07c1f, 0x18c0001f, 0x108c0370,
	0xe0f07ff3, 0xe0f07ff1, 0xe0e00ff1, 0xe0e000f1, 0xe0e00001, 0x81449801,
	0xd8010cc5, 0x17c07c1f, 0xc0c056c0, 0x17c07c1f, 0xa1d38407, 0xa0108400,
	0xa0120400, 0xa0130400, 0xa0170400, 0xa0148400, 0xe8208000, 0x108c0080,
	0x0000fff3, 0xa1dc0407, 0x18c0001f, 0x000100a0, 0x814a1801, 0xa0d79403,
	0x814a9801, 0xa0d89403, 0x1b00001f, 0xffffffdf, 0x1b80001f, 0x90100000,
	0x80cd9801, 0xc8e00003, 0x17c07c1f, 0x80ce1801, 0xc8e00be3, 0x17c07c1f,
	0x80cf1801, 0xc8e02143, 0x17c07c1f, 0x80cf9801, 0xc8e02e63, 0x17c07c1f,
	0x80ce9801, 0xc8e01363, 0x17c07c1f, 0x80cd1801, 0xc8e03483, 0x17c07c1f,
	0x81481801, 0xd8011465, 0x17c07c1f, 0x18c0001f, 0x108c0370, 0xe0e00011,
	0xe0e00031, 0xe0e00071, 0xe0e000f1, 0xe0e001f1, 0xe0e003f1, 0xe0e007f1,
	0xe0e00ff1, 0xe0e01ff1, 0xe0e03ff1, 0xe0e07ff1, 0xe0f07ff1, 0x1b80001f,
	0x20000020, 0xe0f07ff3, 0xe0f07ff2, 0x81489801, 0xd80115a5, 0x17c07c1f,
	0x18c0001f, 0x108c037c, 0xe0e00011, 0x1b80001f, 0x20000020, 0xe0e00013,
	0xe0e00012, 0x80348400, 0x1b80001f, 0x20000300, 0x80370400, 0x1b80001f,
	0x20000300, 0x80330400, 0x1b80001f, 0x20000104, 0x80308400, 0x80320400,
	0x81f38407, 0x81f90407, 0x81f40407, 0x81449801, 0xd80117c5, 0x17c07c1f,
	0x81401801, 0xd8011b05, 0x17c07c1f, 0x18d0001f, 0x108e0e10, 0x1890001f,
	0x108c0260, 0x81400c01, 0x81020c01, 0x80b01402, 0x80b09002, 0x18c0001f,
	0x108c0260, 0xe0c00002, 0x18c0001f, 0x108c0200, 0xc0c128a0, 0x17c07c1f,
	0x18d0001f, 0x108c0250, 0xc0c12760, 0x17c07c1f, 0x18d0001f, 0x108c0254,
	0xc0c12620, 0x17c07c1f, 0xe8208000, 0x108c0034, 0x000f0000, 0x81f48407,
	0xa1d60407, 0x81f10407, 0xe8208000, 0x108c0620, 0x0000000f, 0x18c0001f,
	0x108c041c, 0x1910001f, 0x108c0164, 0xe0c00004, 0x18c0001f, 0x108c0420,
	0x1910001f, 0x108c0150, 0xe0c00004, 0x18c0001f, 0x108c0614, 0x1910001f,
	0x108c0614, 0x09000004, 0x00000001, 0xe0c00004, 0x1ac0001f, 0x55aa55aa,
	0xe8208000, 0x108e0e00, 0x00000001, 0xd0013040, 0x17c07c1f, 0x1900001f,
	0x108c0278, 0xe100001f, 0x17c07c1f, 0xe2e01041, 0xf0000000, 0x17c07c1f,
	0x1900001f, 0x108c026c, 0xe100001f, 0xe2e01041, 0xf0000000, 0x17c07c1f,
	0x1900001f, 0x108c0204, 0x1940001f, 0x0000104d, 0xa1528405, 0xe1000005,
	0x81730405, 0xe1000005, 0xa1540405, 0xe1000005, 0x1950001f, 0x108c0204,
	0x808c1401, 0xd8212202, 0x17c07c1f, 0xa1538405, 0xe1000005, 0xa1508405,
	0xe1000005, 0x81700405, 0xe1000005, 0xa1520405, 0xe1000005, 0x81710405,
	0xe1000005, 0x81718405, 0xe1000005, 0x1880001f, 0x0000104d, 0xa0908402,
	0xe2c00002, 0x80b00402, 0xe2c00002, 0xa0920402, 0xe2c00002, 0x80b10402,
	0xe2c00002, 0x80b18402, 0xe2c00002, 0x1b80001f, 0x2000001a, 0xf0000000,
	0x17c07c1f, 0xe2e01045, 0x1910001f, 0x108e0e10, 0x1950001f, 0x108c0188,
	0x81001404, 0xd8212644, 0x17c07c1f, 0xf0000000, 0x17c07c1f, 0xe2e01045,
	0x1910001f, 0x108e0e14, 0x1950001f, 0x108c0188, 0x81001404, 0xd8212784,
	0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x18b0000b, 0x17c07c1f, 0xa0910402,
	0xe2c00002, 0xa0918402, 0xe2c00002, 0x80b08402, 0xe2c00002, 0x1900001f,
	0x108c0204, 0x1950001f, 0x108c0204, 0xa1510405, 0xe1000005, 0xa1518405,
	0xe1000005, 0x81708405, 0xe1000005, 0x1950001f, 0x108c0188, 0x81081401,
	0x81079404, 0x1950001f, 0x108c018c, 0x81081404, 0x81079404, 0xd8212ae4,
	0x17c07c1f, 0x1900001f, 0x108c0204, 0x1950001f, 0x108c0204, 0x81740405,
	0xe1000005, 0x1950001f, 0x108c0204, 0x814c1401, 0xd8012ce5, 0x17c07c1f,
	0x1b80001f, 0x2000001a, 0x1950001f, 0x108c0204, 0xa1530405, 0xe1000005,
	0x1b80001f, 0x20000003, 0x81728405, 0xe1000005, 0x80b20402, 0xe2c00002,
	0xa0900402, 0xe2c00002, 0x81720405, 0xe1000005, 0xa1500405, 0xe1000005,
	0x81738405, 0xe1000005, 0xf0000000, 0x17c07c1f, 0xf0000000, 0x17c07c1f
};
static struct pcm_desc vcorefs_pcm = {
	.version	= "pcm_suspend_v42.0_r12_0xffffffff",
	.base		= vcorefs_binary,
	.size		= 2436,
	.sess		= 2,
	.replace	= 0,
	.addr_2nd	= 0,
	.vec0		= EVENT_VEC(32, 1, 0, 0),	/* FUNC_26M_WAKEUP */
	.vec1		= EVENT_VEC(33, 1, 0, 41),	/* FUNC_26M_SLEEP */
	.vec4		= EVENT_VEC(34, 1, 0, 95),	/* FUNC_INFRA_WAKEUP */
	.vec5		= EVENT_VEC(35, 1, 0, 130),	/* FUNC_INFRA_SLEEP */
	.vec6		= EVENT_VEC(46, 1, 0, 420),	/* FUNC_NFC_CLK_BUF_ON */
	.vec7		= EVENT_VEC(47, 1, 0, 450),	/* FUNC_NFC_CLK_BUF_OFF */
	.vec8		= EVENT_VEC(36, 1, 0, 155),	/* FUNC_APSRC_WAKEUP */
	.vec9		= EVENT_VEC(37, 1, 0, 194),	/* FUNC_APSRC_SLEEP */
	.vec12		= EVENT_VEC(38, 1, 0, 266),	/* FUNC_VRF18_WAKEUP */
	.vec13		= EVENT_VEC(39, 1, 0, 327),	/* FUNC_VRF18_SLEEP */
	.vec14		= EVENT_VEC(47, 1, 0, 371),	/* FUNC_DDREN_WAKEUP */
	.vec15		= EVENT_VEC(48, 1, 0, 392),	/* FUNC_DDREN_SLEEP */
};

static struct pwr_ctrl vcorefs_ctrl = {
	.wake_src		= WAKE_SRC_R12_PCM_TIMER,

	/* default VCORE DVFS is disabled */
	.pcm_flags		= (SPM_FLAG_DIS_VCORE_DVS | SPM_FLAG_DIS_VCORE_DFS),

	/* Auto-gen Start */

	/* SPM_CLK_CON */
	.reg_srcclken0_ctl = 0,
	.reg_srcclken1_ctl = 0,
	.reg_spm_lock_infra_dcm = 0,
	.reg_srcclken_mask = 0,
	.reg_md1_c32rm_en = 0,
	.reg_md2_c32rm_en = 0,
	.reg_clksq0_sel_ctrl = 0,
	.reg_clksq1_sel_ctrl = 0,
	.reg_srcclken0_en = 0,
	.reg_srcclken1_en = 0,
	.reg_sysclk0_src_mask_b = 0,
	.reg_sysclk1_src_mask_b = 0,

	/* SPM_AP_STANDBY_CON */
	.reg_mpwfi_op = WFI_OP_AND,
	.reg_mp0_cputop_idle_mask = 1,
	.reg_mp1_cputop_idle_mask = 1,
	.reg_debugtop_idle_mask = 1,
	.reg_mp_top_idle_mask = 1,
	.reg_mcusys_idle_mask = 1,
	.reg_md_ddr_en_0_dbc_en = 0,
	.reg_md_ddr_en_1_dbc_en = 0,
	.reg_conn_ddr_en_dbc_en = 0,
	.reg_md32_mask_b = 0,
	.reg_md_0_mask_b = 0,
	.reg_md_1_mask_b = 0,
	.reg_scp_mask_b = 0,
	.reg_srcclkeni0_mask_b = 0,
	.reg_srcclkeni1_mask_b = 0,
	.reg_md_apsrc_1_sel = 0,
	.reg_md_apsrc_0_sel = 0,
	.reg_conn_mask_b = 0,
	.reg_conn_apsrc_sel = 0,

	/* SPM_SRC_REQ */
	.reg_spm_apsrc_req = 0,
	.reg_spm_f26m_req = 0,
	.reg_spm_infra_req = 0,
	.reg_spm_ddren_req = 0,
	.reg_spm_vrf18_req = 0,
	.reg_spm_dvfs_level0_req = 0,
	.reg_spm_dvfs_level1_req = 0,
	.reg_spm_dvfs_level2_req = 0,
	.reg_spm_dvfs_level3_req = 0,
	.reg_spm_dvfs_level4_req = 0,
	.reg_spm_pmcu_mailbox_req = 0,
	.reg_spm_sw_mailbox_req = 0,
	.reg_spm_cksel2_req = 0,
	.reg_spm_cksel3_req = 0,

	/* SPM_SRC_MASK */
	.reg_csyspwreq_mask = 0,
	.reg_md_srcclkena_0_infra_mask_b = 0,
	.reg_md_srcclkena_1_infra_mask_b = 0,
	.reg_md_apsrc_req_0_infra_mask_b = 0,
	.reg_md_apsrc_req_1_infra_mask_b = 0,
	.reg_conn_srcclkena_infra_mask_b = 0,
	.reg_conn_infra_req_mask_b = 0,
	.reg_md32_srcclkena_infra_mask_b = 0,
	.reg_md32_infra_req_mask_b = 0,
	.reg_scp_srcclkena_infra_mask_b = 0,
	.reg_scp_infra_req_mask_b = 0,
	.reg_srcclkeni0_infra_mask_b = 0,
	.reg_srcclkeni1_infra_mask_b = 0,
	.reg_ccif0_md_event_mask_b = 0,
	.reg_ccif0_ap_event_mask_b = 0,
	.reg_ccif1_md_event_mask_b = 0,
	.reg_ccif1_ap_event_mask_b = 0,
	.reg_ccif2_md_event_mask_b = 0,
	.reg_ccif2_ap_event_mask_b = 0,
	.reg_ccif3_md_event_mask_b = 0,
	.reg_ccif3_ap_event_mask_b = 0,
	.reg_ccifmd_md1_event_mask_b = 0,
	.reg_ccifmd_md2_event_mask_b = 0,
	.reg_c2k_ps_rccif_wake_mask_b = 0,
	.reg_c2k_l1_rccif_wake_mask_b = 0,
	.reg_ps_c2k_rccif_wake_mask_b = 0,
	.reg_l1_c2k_rccif_wake_mask_b = 0,
	.reg_dqssoc_req_mask_b = 0,
	.reg_disp2_req_mask_b = 0,
	.reg_md_ddr_en_0_mask_b = 0,
	.reg_md_ddr_en_1_mask_b = 0,
	.reg_conn_ddr_en_mask_b = 0,

	/* SPM_SRC2_MASK */
	.reg_disp0_req_mask_b = 0,
	.reg_disp1_req_mask_b = 0,
	.reg_disp_od_req_mask_b = 0,
	.reg_mfg_req_mask_b = 0,
	.reg_vdec0_req_mask_b = 0,
	.reg_gce_vrf18_req_mask_b = 0,
	.reg_gce_req_mask_b = 0,
	.reg_lpdma_req_mask_b = 0,
	.reg_srcclkeni1_cksel2_mask_b = 0,
	.reg_conn_srcclkena_cksel2_mask_b = 0,
	.reg_srcclkeni0_cksel3_mask_b = 0,
	.reg_md32_apsrc_req_ddren_mask_b = 0,
	.reg_scp_apsrc_req_ddren_mask_b = 0,
	.reg_md_vrf18_req_0_mask_b = 0,
	.reg_md_vrf18_req_1_mask_b = 0,
	.reg_next_dvfs_level0_mask_b = 0,
	.reg_next_dvfs_level1_mask_b = 0,
	.reg_next_dvfs_level2_mask_b = 0,
	.reg_next_dvfs_level3_mask_b = 0,
	.reg_next_dvfs_level4_mask_b = 0,
	.reg_msdc1_dvfs_halt_mask = 0,
	.reg_msdc2_dvfs_halt_mask = 0,
	.reg_msdc3_dvfs_halt_mask = 0,
	.reg_sw2spm_int0_mask_b = 0,
	.reg_sw2spm_int1_mask_b = 0,
	.reg_sw2spm_int2_mask_b = 0,
	.reg_sw2spm_int3_mask_b = 0,
	.reg_pmcu2spm_int0_mask_b = 0,
	.reg_pmcu2spm_int1_mask_b = 0,
	.reg_pmcu2spm_int2_mask_b = 0,
	.reg_pmcu2spm_int3_mask_b = 0,

	/* SPM_WAKEUP_EVENT_MASK */
	.reg_wakeup_event_mask = 0,

	/* SPM_EXT_WAKEUP_EVENT_MASK */
	.reg_ext_wakeup_event_mask = 0,

	/* MP0_CPU0_WFI_EN */
	.mp0_cpu0_wfi_en = 0,

	/* MP0_CPU1_WFI_EN */
	.mp0_cpu1_wfi_en = 0,

	/* MP0_CPU2_WFI_EN */
	.mp0_cpu2_wfi_en = 0,

	/* MP0_CPU3_WFI_EN */
	.mp0_cpu3_wfi_en = 0,

	/* MP1_CPU0_WFI_EN */
	.mp1_cpu0_wfi_en = 0,

	/* MP1_CPU1_WFI_EN */
	.mp1_cpu1_wfi_en = 0,

	/* MP1_CPU2_WFI_EN */
	.mp1_cpu2_wfi_en = 0,

	/* MP1_CPU3_WFI_EN */
	.mp1_cpu3_wfi_en = 0,

	/* DEBUG0_WFI_EN */
	.debug0_wfi_en = 0,

	/* DEBUG1_WFI_EN */
	.debug1_wfi_en = 0,

	/* DEBUG2_WFI_EN */
	.debug2_wfi_en = 0,

	/* DEBUG3_WFI_EN */
	.debug3_wfi_en = 0,

	/* Auto-gen End */
};

struct spm_lp_scen __spm_vcorefs = {
#ifdef CONFIG_MTK_FPGA
	.pcmdesc	= &vcorefs_pcm,
#endif
	.pwrctrl	= &vcorefs_ctrl,
};

static void spm_fw_version(void)
{
	struct pcm_desc *pcmdesc;

#if DYNAMIC_LOAD
	pcmdesc = &(dyna_load_pcm[DYNA_LOAD_PCM_VCOREFS].desc);
#else
	pcmdesc = __spm_vcorefs.pcmdesc;
#endif

	spm_vcorefs_warn("VCOREFS_VER: %s\n", pcmdesc->version);
}

static void __check_dvfs_ultra_source(void)
{
	u32 val;

	val = spm_read(DRAMC_DPY_CLK_SW_CON2);

	spm_vcorefs_warn("DVFS ULTRA SOURCE      : 0x%x, REQ(0x%x, 0x%x), ACK(0x%x, 0x%x, 0x%x)\n",
						val,
						val & SPM2MM_ULTRAREQ_LSB,
						val & SPM2MD_ULTRAREQ_LSB,
						val & SPM2ISP_ULTRAACK_D2T_LSB,
						val & SPM2MM_ULTRAACK_D2T_LSB,
						val & SPM2MD_ULTRAACK_D2T_LSB);
}

char *spm_vcorefs_dump_dvfs_regs(char *p)
{
	if (p) {
		#if 1
		p += sprintf(p, "DVFSRC_DEBUG_EN        : 0x%x\n", spm_read(DVFSRC_DEBUG_EN));
		p += sprintf(p, "DVFSRC_ENABLE          : 0x%x\n", spm_read(DVFSRC_ENABLE));
		p += sprintf(p, "DVFSRC_MD32_LEVEL_MASK : 0x%x\n", spm_read(DVFSRC_MD32_LEVEL_MASK));
		p += sprintf(p, "DVFSRC_RECORD          : 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
				spm_read(DVFSRC_RECORD_0), spm_read(DVFSRC_RECORD_1), spm_read(DVFSRC_RECORD_2),
				spm_read(DVFSRC_RECORD_3), spm_read(DVFSRC_RECORD_4), spm_read(DVFSRC_RECORD_5),
				spm_read(DVFSRC_RECORD_6), spm_read(DVFSRC_RECORD_7), spm_read(DVFSRC_RECORD_8));
		p += sprintf(p, "DVFSRC_SPM_LEVEL_MASK  : 0x%x\n", spm_read(DVFSRC_SPM_LEVEL_MASK));
		__check_dvfs_ultra_source();
		p += sprintf(p, "SPM_SW_FLAG     : 0x%x\n", spm_read(SPM_SW_FLAG));
		p += sprintf(p, "DVFS_LEVEL      : 0x%x\n", spm_read(DVFS_LEVEL));
		p += sprintf(p, "PCM_IM_PTR      : 0x%x (%u)\n", spm_read(PCM_IM_PTR), spm_read(PCM_IM_LEN));
		#endif
	} else {
		#if 1
		spm_fw_version();
		spm_vcorefs_warn("DVFSRC_DEBUG_EN        : 0x%x\n", spm_read(DVFSRC_DEBUG_EN));
		spm_vcorefs_warn("DVFSRC_ENABLE          : 0x%x\n", spm_read(DVFSRC_ENABLE));
		spm_vcorefs_warn("DVFSRC_MD32_LEVEL_MASK : 0x%x\n", spm_read(DVFSRC_MD32_LEVEL_MASK));
		spm_vcorefs_warn("DVFSRC_RECORD          : 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
				spm_read(DVFSRC_RECORD_0), spm_read(DVFSRC_RECORD_1), spm_read(DVFSRC_RECORD_2),
				spm_read(DVFSRC_RECORD_3), spm_read(DVFSRC_RECORD_4), spm_read(DVFSRC_RECORD_5),
				spm_read(DVFSRC_RECORD_6), spm_read(DVFSRC_RECORD_7), spm_read(DVFSRC_RECORD_8));
		spm_vcorefs_warn("DVFSRC_SPM_LEVEL_MASK  : 0x%x\n", spm_read(DVFSRC_SPM_LEVEL_MASK));
		__check_dvfs_ultra_source();
		spm_vcorefs_warn("SPM_SW_FLAG     : 0x%x\n", spm_read(SPM_SW_FLAG));
		spm_vcorefs_warn("DVFS_LEVEL      : 0x%x\n", spm_read(DVFS_LEVEL));
		spm_vcorefs_warn("CLK_SW_CON_SEL  : 0x%x\n", spm_read(DRAMC_DPY_CLK_SW_CON_SEL));
		spm_vcorefs_warn("PCM_IM_PTR      : 0x%x (%u)\n", spm_read(PCM_IM_PTR), spm_read(PCM_IM_LEN));
		#endif
	}

	return p;
}

/*
 * condition: false will loop for check
 */
#define wait_spm_complete_by_condition(condition, timeout)	\
({								\
	int i = 0;						\
	while (!(condition)) {					\
		if (i >= (timeout)) {				\
			i = -EBUSY;				\
			break;					\
		}						\
		udelay(1);					\
		i++;						\
	}							\
	i;							\
})

#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT)
/*
 * powerMCU
 */
#if SPM_VCORE_DVFS_EN
static void spm_dvfsfw_init(void)
{
	unsigned long flags;
	int ret;
	u32 opp = 0, pll = 0;
	struct spm_data spm_d;

	memset(&spm_d, 0, sizeof(struct spm_data));
	opp = vcorefs_get_curr_opp();
	spm_d.u.args.arg0 = opp;

	if (1)
		pll = 0x1; /* PHYPLL */
	else
		pll = 0x0; /* CLRPLL */

	spm_d.u.args.arg1 = pll;

	spin_lock_irqsave(&__spm_lock, flags);

	ret = spm_to_sspm_command(SPM_VCORE_DVFS_FWINIT, &spm_d);
	if (ret < 0)
		BUG();

	spin_unlock_irqrestore(&__spm_lock, flags);

	spm_vcorefs_warn("DVFS_LEVEL: 0x%x(%u), DRAMC_DPY: 0x%x, RSV_5: 0x%x(%u)\n",
						spm_read(DVFS_LEVEL), opp,
						spm_read(DRAMC_DPY_CLK_SW_CON_SEL),
						spm_read(SPM_SW_RSV_5), pll);
	spm_vcorefs_warn("[%s] done\n", __func__);
}
#endif

static void
spm_vcorefs_pcm_setup_for_kick(struct pcm_desc *pcmdesc, struct pwr_ctrl *pwrctrl)
{
	int ret;
	struct spm_data spm_d;

	memset(&spm_d, 0, sizeof(struct spm_data));
	spm_d.u.vcorefs.pcm_flags = pwrctrl->pcm_flags;
	ret = spm_to_sspm_command(SPM_VCORE_DVFS_ENTER, &spm_d);
	if (ret < 0)
		BUG();
}

void spm_to_sspm_pwarp_cmd(void)
{
	unsigned long flags;
	int ret;
	struct spm_data spm_d;

	spin_lock_irqsave(&__spm_lock, flags);

	memset(&spm_d, 0, sizeof(struct spm_data));
	ret = spm_to_sspm_command(SPM_VCORE_PWARP_CMD, &spm_d);
	if (ret < 0)
		BUG();

	spin_unlock_irqrestore(&__spm_lock, flags);
}

void spm_msdc_setting(int msdc, bool msdc_sta)
{
	unsigned long flags;
	int ret;
	struct spm_data spm_d;

	spin_lock_irqsave(&__spm_lock, flags);

	memset(&spm_d, 0, sizeof(struct spm_data));
	spm_d.u.args.arg0 = msdc;
	spm_d.u.args.arg1 = msdc_sta;
	ret = spm_to_sspm_command(SPM_PWR_CTRL_MSDC, &spm_d);
	if (ret < 0)
		BUG();

	spin_unlock_irqrestore(&__spm_lock, flags);

	spm_vcorefs_warn("msdc: %d, msdc_sta: %d, SPM_SW_NONSERSV_3: 0x%x\n",
							msdc, msdc_sta, spm_read(SPM_SW_NONSERSV_3));
	spm_vcorefs_warn("[%s] done\n", __func__);
}

#else
/*
 * legacy
 */
#if SPM_VCORE_DVFS_EN
static void spm_dvfsfw_init(void)
{
	unsigned long flags;
	u32 opp = 0, pll = 0, level = 0;
	u32 dvfs_level[NUM_OPP] = { 0x8810, 0x4808, 0x4404, 0x2202, 0x1101};

	opp = vcorefs_get_curr_opp();

	if (1)
		pll = 0x1; /* PHYPLL */
	else
		pll = 0x0; /* CLRPLL */

	if (opp < 0 || opp >= NUM_OPP) {
		spm_vcorefs_err("NOT SUPPORT OPP: %u\n", opp);
		BUG();
	}
	if (pll < 0 || pll >= NUM_PLL) {
		spm_vcorefs_err("NOT SUPPORT PLL: %u\n", pll);
		BUG();
	}

	level = dvfs_level[opp];

	spin_lock_irqsave(&__spm_lock, flags);

	spm_write(DVFS_LEVEL, level);

	spm_write(DRAMC_DPY_CLK_SW_CON_SEL, 0xBFFFF);

	spm_write(SPM_SW_RSV_5, (spm_read(SPM_SW_RSV_5) & ~(1U << 0)) | pll);

	spin_unlock_irqrestore(&__spm_lock, flags);

	spm_vcorefs_warn("DVFS_LEVEL: 0x%x(%u), DRAMC_DPY: 0x%x, RSV_5: 0x%x(%u)\n",
						spm_read(DVFS_LEVEL), opp,
						spm_read(DRAMC_DPY_CLK_SW_CON_SEL),
						spm_read(SPM_SW_RSV_5), pll);
	spm_vcorefs_warn("[%s] done\n", __func__);
}
#endif

static void
spm_vcorefs_pcm_setup_for_kick(struct pcm_desc *pcmdesc, struct pwr_ctrl *pwrctrl)
{
	__spm_reset_and_init_pcm(pcmdesc);

	__spm_kick_im_to_fetch(pcmdesc);

	__spm_init_pcm_register();

	__spm_init_event_vector(pcmdesc);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	__spm_kick_pcm_to_run(pwrctrl);
}

void spm_msdc_setting(int msdc, bool msdc_sta)
{
	unsigned long flags;

	spin_lock_irqsave(&__spm_lock, flags);

	spm_write(SPM_SW_NONSERSV_3, (spm_read(SPM_SW_NONSERSV_3) & ~SPM_SW_NONSERSV_3_LSB));

	if (msdc == MSDC1)
		spm_write(SPM_SW_NONSERSV_3, (spm_read(SPM_SW_NONSERSV_3) & ~(1U << MSDC1)) | (msdc_sta << MSDC1));

	if (msdc == MSDC3)
		spm_write(SPM_SW_NONSERSV_3, (spm_read(SPM_SW_NONSERSV_3) & ~(1U << MSDC3)) | (msdc_sta << MSDC3));

	spin_unlock_irqrestore(&__spm_lock, flags);

	spm_vcorefs_warn("msdc: %d, msdc_sta: %d, SPM_SW_NONSERSV_3: 0x%x\n",
							msdc, msdc_sta, spm_read(SPM_SW_NONSERSV_3));
	spm_vcorefs_warn("[%s] done\n", __func__);
}

void __spm_sync_vcore_dvfs_power_control(struct pwr_ctrl *dest_pwr_ctrl, const struct pwr_ctrl *src_pwr_ctrl)
{
	/* SPM_SRC_REQ */
	dest_pwr_ctrl->reg_spm_dvfs_level0_req = src_pwr_ctrl->reg_spm_dvfs_level0_req;
	dest_pwr_ctrl->reg_spm_dvfs_level1_req = src_pwr_ctrl->reg_spm_dvfs_level1_req;
	dest_pwr_ctrl->reg_spm_dvfs_level2_req = src_pwr_ctrl->reg_spm_dvfs_level2_req;
	dest_pwr_ctrl->reg_spm_dvfs_level3_req = src_pwr_ctrl->reg_spm_dvfs_level3_req;
	dest_pwr_ctrl->reg_spm_dvfs_level4_req = src_pwr_ctrl->reg_spm_dvfs_level4_req;

	/* SPM_SRC2_MASK */
	dest_pwr_ctrl->reg_next_dvfs_level0_mask_b = src_pwr_ctrl->reg_next_dvfs_level0_mask_b;
	dest_pwr_ctrl->reg_next_dvfs_level1_mask_b = src_pwr_ctrl->reg_next_dvfs_level1_mask_b;
	dest_pwr_ctrl->reg_next_dvfs_level2_mask_b = src_pwr_ctrl->reg_next_dvfs_level2_mask_b;
	dest_pwr_ctrl->reg_next_dvfs_level3_mask_b = src_pwr_ctrl->reg_next_dvfs_level3_mask_b;
	dest_pwr_ctrl->reg_next_dvfs_level4_mask_b = src_pwr_ctrl->reg_next_dvfs_level4_mask_b;
	dest_pwr_ctrl->reg_msdc1_dvfs_halt_mask = src_pwr_ctrl->reg_msdc1_dvfs_halt_mask;
	dest_pwr_ctrl->reg_msdc2_dvfs_halt_mask = src_pwr_ctrl->reg_msdc2_dvfs_halt_mask;
	dest_pwr_ctrl->reg_msdc3_dvfs_halt_mask = src_pwr_ctrl->reg_msdc3_dvfs_halt_mask;

	dest_pwr_ctrl->pcm_flags = (dest_pwr_ctrl->pcm_flags & (~dvfs_mask)) |
					(src_pwr_ctrl->pcm_flags & dvfs_mask);

	if (dest_pwr_ctrl->pcm_flags_cust)
		dest_pwr_ctrl->pcm_flags_cust = (dest_pwr_ctrl->pcm_flags_cust & (~dvfs_mask)) |
						(src_pwr_ctrl->pcm_flags & dvfs_mask);
}
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

static int spm_trigger_dvfs(int opp)
{
	u32 timeout;

	timeout = spm_read(DVFSRC_SPM_LEVEL_MASK) & DVFS_REQ_TO_SPM_TIMEOUT;
	if (timeout) {
		spm_vcorefs_err("PRE-DVFS TIMEOUT: %dus\n", SPM_DVFS_TIMEOUT);
		spm_vcorefs_dump_dvfs_regs(NULL);
		BUG();
	}

#if 1 /* for Elbrus */
	if (opp == OPP_1) {
		spm_write(DVFSRC_DEBUG_EN, (spm_read(DVFSRC_DEBUG_EN) & ~RG_REQUEST_VOLT_ONLY) | level_mask[opp]);
		spm_write(DVFSRC_MD32_LEVEL_MASK, (spm_read(DVFSRC_MD32_LEVEL_MASK) & ~MD32_LEVEL_MASK) | LV2_MSK);
#else /* for Whitney */
	if (opp == OPP_2) {
		spm_write(DVFSRC_DEBUG_EN, (spm_read(DVFSRC_DEBUG_EN) & ~RG_REQUEST_VOLT_ONLY) | level_mask[opp]);
		spm_write(DVFSRC_MD32_LEVEL_MASK, (spm_read(DVFSRC_MD32_LEVEL_MASK) & ~MD32_LEVEL_MASK) | LV1_MSK);
#endif
	} else if (opp >= OPP_0 && opp < NUM_OPP) {
		spm_write(DVFSRC_MD32_LEVEL_MASK, (spm_read(DVFSRC_MD32_LEVEL_MASK) & ~MD32_LEVEL_MASK) |
												level_mask[opp]);
		spm_write(DVFSRC_DEBUG_EN, (spm_read(DVFSRC_DEBUG_EN) & ~RG_REQUEST_VOLT_ONLY));
	} else {
		return -EINVAL;
	}

	udelay(SPM_DVFS_TIMEOUT);

	timeout = spm_read(DVFSRC_SPM_LEVEL_MASK) & DVFS_REQ_TO_SPM_TIMEOUT;
	if (timeout) {
		spm_vcorefs_err("POST-DVFS TIMEOUT: %dus\n", SPM_DVFS_TIMEOUT);
		spm_vcorefs_dump_dvfs_regs(NULL);
		BUG();
	}

	spm_vcorefs_warn("opp: %d, DVFSRC_DEBUG_EN: 0x%x, DVFSRC_MD32_LEVEL_MASK: 0x%x\n",
					opp, spm_read(DVFSRC_DEBUG_EN), spm_read(DVFSRC_MD32_LEVEL_MASK));

	return SPM_DVFS_TIMEOUT;
}

int spm_dvfs_flag_init(void)
{
	int flag = SPM_FLAG_RUN_COMMON_SCENARIO;

	if (!vcorefs_vcore_dvs_en())
		flag |= SPM_FLAG_DIS_VCORE_DVS;
	if (!vcorefs_dram_dfs_en())
		flag |= SPM_FLAG_DIS_VCORE_DFS;

	spm_vcorefs_warn("pcm_flag: 0x%x\n", flag);

	return flag;
}

#if SPM_VCORE_DVFS_EN
static void spm_dvfsrc_init(void)
{
	unsigned long flags;

	spin_lock_irqsave(&__spm_lock, flags);

	spm_write(DVFSRC_DEBUG_EN, 0x1);
	spm_write(DVFSRC_ENABLE, 0x5);

	spin_unlock_irqrestore(&__spm_lock, flags);

	spm_vcorefs_warn("DVFSRC_DEBUG_EN: 0x%x, DVFSRC_ENABLE: 0x%x\n",
							spm_read(DVFSRC_DEBUG_EN),
							spm_read(DVFSRC_ENABLE));
	spm_vcorefs_warn("[%s] done\n", __func__);
}
#endif

static void dvfsrc_register_init(void)
{
	struct device_node *node;

	/* dvfsrc */
	node = of_find_compatible_node(NULL, NULL, "mediatek,dvfsrc");
	if (!node) {
		spm_vcorefs_err("[DVFSRC] find node failed\n");
		goto dvfsrc_exit;
	}

	dvfsrc_base = of_iomap(node, 0);
	if (!dvfsrc_base) {
		spm_vcorefs_err("[DVFSRC] base failed\n");
		goto dvfsrc_exit;
	}

dvfsrc_exit:

	spm_vcorefs_warn("spm_dvfsrc_register_init: dvfsrc_base = %p\n", dvfsrc_base);
}

int spm_set_vcore_dvfs(int opp)
{
	unsigned long flags;
	int t_dvfs = 0;

	spin_lock_irqsave(&__spm_lock, flags);

	set_aee_vcore_dvfs_status(SPM_VCOREFS_ENTER);

	set_aee_vcore_dvfs_status(SPM_VCOREFS_DVFS_START);

	t_dvfs = spm_trigger_dvfs(opp);

	set_aee_vcore_dvfs_status(SPM_VCOREFS_DVFS_END);

	set_aee_vcore_dvfs_status(SPM_VCOREFS_LEAVE);

	spin_unlock_irqrestore(&__spm_lock, flags);

	return t_dvfs;
}

void spm_go_to_vcorefs(int spm_flags)
{
	unsigned long flags;
	struct pcm_desc *pcmdesc;
	struct pwr_ctrl *pwrctrl;

#if DYNAMIC_LOAD
	if (dyna_load_pcm[DYNA_LOAD_PCM_VCOREFS].ready) {
		pcmdesc = &(dyna_load_pcm[DYNA_LOAD_PCM_VCOREFS].desc);
		pwrctrl = __spm_vcorefs.pwrctrl;
	} else {
		spm_vcorefs_err("[%s] dyna load F/W fail\n", __func__);
		BUG();
	}
#else
	pcmdesc = __spm_vcorefs.pcmdesc;
	pwrctrl = __spm_vcorefs.pwrctrl;
#endif

	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

	spin_lock_irqsave(&__spm_lock, flags);

	spm_vcorefs_pcm_setup_for_kick(pcmdesc, pwrctrl);

	spm_vcorefs_warn("[%s] done\n", __func__);

	spin_unlock_irqrestore(&__spm_lock, flags);
}

void spm_vcorefs_init(void)
{
	int flag;

	spm_fw_version();
	flag = spm_dvfs_flag_init();
	dvfsrc_register_init();

#if SPM_VCORE_DVFS_EN
	if (is_vcorefs_feature_enable()) {
		vcorefs_module_init();
		spm_dvfsfw_init();
		spm_go_to_vcorefs(flag);
		spm_dvfsrc_init();
		vcorefs_late_init_dvfs();
	} else {
		spm_vcorefs_warn("[%s] NOT SUPPOT VCORE DVFS\n", __func__);
	}
#endif
	spm_vcorefs_warn("[%s] done\n", __func__);
}

MODULE_DESCRIPTION("SPM VCORE-DVFS DRIVER");
