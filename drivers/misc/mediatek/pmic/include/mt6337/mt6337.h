/*
 * Copyright (C) 2016 MediaTek Inc.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef _MT6337_PMIC_H_
#define _MT6337_PMIC_H_


#include "mt6337_upmu_hw.h"

#define PMIC_DEBUG

#include <linux/smp.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>

/*
 * The CHIP INFO
 */
#define PMIC6337_E1_CID_CODE    0x3710
#define PMIC6337_E2_CID_CODE    0x3720
#define PMIC6337_E3_CID_CODE    0x3730

#define mt6337_emerg(fmt, args...)		pr_emerg("[SPM-MT6337] " fmt, ##args)
#define mt6337_alert(fmt, args...)		pr_alert("[SPM-MT6337] " fmt, ##args)
#define mt6337_crit(fmt, args...)		pr_crit("[SPM-MT6337] " fmt, ##args)
#define mt6337_err(fmt, args...)		pr_err("[SPM-MT6337] " fmt, ##args)
#define mt6337_warn(fmt, args...)		pr_warn("[SPM-MT6337] " fmt, ##args)
#define mt6337_notice(fmt, args...)	pr_notice("[SPM-MT6337] " fmt, ##args)
#define mt6337_info(fmt, args...)		pr_info("[SPM-MT6337] " fmt, ##args)
#define mt6337_debug(fmt, args...)		pr_info("[SPM-MT6337] " fmt, ##args)	/* pr_debug show nothing */

/* just use in suspend flow for important log due to console suspend */
#if defined PMIC_DEBUG_PR_DBG
#define mt6337_spm_crit2(fmt, args...)		\
do {					\
	aee_sram_printk(fmt, ##args);	\
	mt6337_crit(fmt, ##args);		\
} while (0)
#else
#define mt6337_spm_crit2(fmt, args...)		\
do {					\
	aee_sram_printk(fmt, ##args);	\
	mt6337_debug(fmt, ##args);		\
} while (0)
#endif

#define PMIC_DEBUG_PR_DBG

#define MT6337TAG                "[MT6337] "
#ifdef PMIC_DEBUG
#define PMICDEB(fmt, arg...) pr_debug(MT6337TAG "cpuid=%d, " fmt, raw_smp_processor_id(), ##arg)
#define PMICFUC(fmt, arg...) pr_debug(MT6337TAG "cpuid=%d, %s\n", raw_smp_processor_id(), __func__)
#endif  /*-- defined PMIC_DEBUG --*/
#if defined PMIC_DEBUG_PR_DBG
#define PMICLOG(fmt, arg...)   pr_err(MT6337TAG fmt, ##arg)
#else
#define PMICLOG(fmt, arg...)
#endif  /*-- defined PMIC_DEBUG_PR_DBG --*/
#define PMICERR(fmt, arg...)   pr_debug(MT6337TAG "ERROR,line=%d " fmt, __LINE__, ##arg)
#define PMICREG(fmt, arg...)   pr_debug(MT6337TAG fmt, ##arg)

#define MT6337_INT_WIDTH 16

#define MT6337_INTERRUPT_WIDTH 16

#define MT6337_S_INT_GEN(_name)	\
	{	\
		.name =  #_name,	\
	}

#define MT6337_M_INTS_GEN(adr, enA, maskA, interrupt)	\
	{	\
		.address =  adr,	\
		.en = enA,	\
		.set = (enA) + 2,	\
		.clear = (enA) + 4,	\
		.mask = maskA,	\
		.mask_set =	(maskA) + 2, \
		.mask_clear = (maskA) + 4,	\
		.interrupts = interrupt,	\
	}

struct mt6337_interrupt_bit {
	const char *name;
	void (*callback)(void);
	unsigned int times;
};

struct mt6337_interrupts {
	unsigned int address;
	unsigned int en;
	unsigned int set;
	unsigned int clear;
	unsigned int mask;
	unsigned int mask_set;
	unsigned int mask_clear;
	struct mt6337_interrupt_bit *interrupts;
};

extern unsigned int mt6337_read_interface(unsigned int RegNum, unsigned int *val,
	unsigned int MASK, unsigned int SHIFT);
extern unsigned int mt6337_config_interface(unsigned int RegNum, unsigned int val,
	unsigned int MASK, unsigned int SHIFT);
extern unsigned int mt6337_read_interface_nolock(unsigned int RegNum, unsigned int *val,
	unsigned int MASK, unsigned int SHIFT);
extern unsigned int mt6337_config_interface_nolock(unsigned int RegNum, unsigned int val,
	unsigned int MASK, unsigned int SHIFT);
extern unsigned int mt6337_upmu_get_reg_value(unsigned int reg);
void mt6337_upmu_set_reg_value(unsigned int reg, unsigned int reg_val);
extern unsigned short mt6337_set_register_value(MT6337_PMU_FLAGS_LIST_ENUM flagname, unsigned int val);
extern unsigned short mt6337_get_register_value(MT6337_PMU_FLAGS_LIST_ENUM flagname);
#endif				/* _MT6337_PMIC_H_ */