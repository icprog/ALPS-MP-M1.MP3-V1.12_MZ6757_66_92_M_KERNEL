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

#if defined(__KERNEL__)
#include <linux/init.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cpumask.h>
#include <linux/cpu.h>
#include <asm/bug.h>
#include <mt-plat/mt_io.h>
#include <mt-plat/sync_write.h>
#include <mach/mt_secure_api.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <mt_dramc.h>
#endif

#include "mt_dcm.h"
#include "mt_dcm_autogen.h"

/* #define DCM_DEFAULT_ALL_OFF */

#if defined(__KERNEL__)
#if defined(CONFIG_OF)
unsigned long dcm_topckgen_base;
unsigned long dcm_mcucfg_base;
unsigned long dcm_mcucfg_phys_base;
unsigned long dcm_dramc_ch0_top0_base;
unsigned long dcm_dramc_ch0_top1_base;
unsigned long dcm_dramc_ch0_top3_base;
unsigned long dcm_dramc_ch1_top0_base;
unsigned long dcm_dramc_ch1_top1_base;
unsigned long dcm_dramc_ch1_top3_base;
unsigned long dcm_emi_base;
unsigned long dcm_infracfg_ao_base;

#define MCUCFG_NODE "mediatek,mcucfg"
#define INFRACFG_AO_NODE "mediatek,infracfg_ao"
#define TOPCKGEN_NODE "mediatek,topckgen"
#define DRAMC_CH0_TOP0_NODE "mediatek,dramc_ch0_top0"
#define DRAMC_CH0_TOP1_NODE "mediatek,dramc_ch0_top1"
#define DRAMC_CH0_TOP3_NODE "mediatek,dramc_ch0_top3"
#define DRAMC_CH1_TOP0_NODE "mediatek,dramc_ch1_top0"
#define DRAMC_CH1_TOP1_NODE "mediatek,dramc_ch1_top1"
#define DRAMC_CH1_TOP3_NODE "mediatek,dramc_ch1_top3"
#define EMI_NODE "mediatek,emi"
#endif /* #if defined(CONFIG_OF) */
#endif /* defined(__KERNEL__) */

#define MCSI_A_DCM_PHYS					(CCI_BASE + 0x0)

#if defined(__KERNEL__)
#define USING_PR_LOG
#endif /* defined(__KERNEL__) */

#ifdef USING_PR_LOG /* __KERNEL__ */
#define TAG	"[Power/dcm] "
/* #define DCM_ENABLE_DCM_CFG */
#define dcm_err(fmt, args...)	pr_err(TAG fmt, ##args)
#define dcm_warn(fmt, args...)	pr_warn(TAG fmt, ##args)
#define dcm_info(fmt, args...)	pr_notice(TAG fmt, ##args)
#define dcm_dbg(fmt, args...)	pr_info(TAG fmt, ##args)
#define dcm_ver(fmt, args...)	pr_debug(TAG fmt, ##args)
#endif

/** macro **/
#define and(v, a) ((v) & (a))
#define or(v, o) ((v) | (o))
#define aor(v, a, o) (((v) & (a)) | (o))

#define reg_read(addr)	 __raw_readl(IOMEM(addr))
#define reg_write(addr, val)   mt_reg_sync_writel((val), ((void *)addr))

#if defined(__KERNEL__) /* for KERNEL */
#if defined(CONFIG_ARM_PSCI) || defined(CONFIG_MTK_PSCI)
#define MCUSYS_SMC_WRITE(addr, val)  mcusys_smc_write_phy(addr##_PHYS, val)
#define MCSI_A_SMC_WRITE(addr, val)  mcsi_a_smc_write_phy(addr##_PHYS, val)
#define MCSI_A_SMC_READ(addr)  mcsi_a_smc_read_phy(addr##_PHYS)
#else
#define MCUSYS_SMC_WRITE(addr, val)  mcusys_smc_write(addr, val)
#define MCSI_A_SMC_WRITE(addr, val)  mcusys_smc_write(addr, val)
#define MCSI_A_SMC_READ(addr)  reg_read(addr)
#endif
#endif /* defined(__KERNEL__) */

#ifdef USING_PR_LOG
#define REG_DUMP(addr) dcm_info("%-30s(0x%08lx): 0x%08x\n", #addr, addr, reg_read(addr))
#define SECURE_REG_DUMP(addr) dcm_info("%-30s(0x%08x): 0x%08x\n", #addr, addr, mcsi_a_smc_read_phy(addr))
#endif

/** global **/
#if defined(__KERNEL__)
static DEFINE_MUTEX(dcm_lock);
#endif
static unsigned int dcm_initiated;

/*****************************************
 * following is implementation per DCM module.
 * 1. per-DCM function is 1-argu with ON/OFF/MODE option.
 *****************************************/
typedef int (*DCM_FUNC)(int);
typedef void (*DCM_PRESET_FUNC)(void);

int dcm_armcore(ENUM_ARMCORE_DCM mode)
{
	if (mode == ARMCORE_DCM_MODE1) {
		dcm_infracfg_ao_mcu_armpll_bus_mode1(mode);
		dcm_infracfg_ao_mcu_armpll_ca7l_mode1(mode);
		dcm_infracfg_ao_mcu_armpll_ca7ll_mode1(mode);
	} else if (mode == ARMCORE_DCM_MODE2) {
		dcm_infracfg_ao_mcu_armpll_bus_mode2(mode);
		dcm_infracfg_ao_mcu_armpll_ca7l_mode2(mode);
		dcm_infracfg_ao_mcu_armpll_ca7ll_mode2(mode);
	} else {
		dcm_infracfg_ao_mcu_armpll_bus_mode2(mode);
		dcm_infracfg_ao_mcu_armpll_ca7l_mode2(mode);
		dcm_infracfg_ao_mcu_armpll_ca7ll_mode2(mode);
	}
	return 0;
}

int dcm_infra(ENUM_INFRA_DCM on)
{
	/* ASSERT_INFRA_DCMCTL(); */

	dcm_infracfg_ao_dcm_infra_bus(on);
	dcm_infracfg_ao_p2p_rxclk_ctrl(on);
	dcm_infracfg_ao_infra_md_fmem(on);
	/* dcm_infracfg_ao_dcm_mem_ctrl(on); Controlled by DRAMC driver in preloader */
	/* dcm_infracfg_ao_dcm_dfs_mem_ctrl(on); No used, removed from infra-ao */
	return 0;
}

int dcm_infra_dbc(int cnt)
{
#if 0
	reg_write(INFRA_BUS_DCM_CTRL, aor(reg_read(INFRA_BUS_DCM_CTRL),
					  ~(0x1f << 15), (cnt << 15)));
#endif
	return 0;
}

/*
 * input argument
 * 0: 1/1
 * 1: 1/2
 * 2: 1/4
 * 3: 1/8
 * 4: 1/16
 * 5: 1/32
 * default: 0, 0
 */
int dcm_infra_rate(unsigned int fsel, unsigned int sfsel)
{
#if 0
	BUG_ON(fsel > 5 || sfsel > 5);

	fsel = 0x10 >> fsel;
	sfsel = 0x10 >> sfsel;

	reg_write(INFRA_BUS_DCM_CTRL, aor(reg_read(INFRA_BUS_DCM_CTRL),
					 ~INFRA_BUS_DCM_CTRL_SEL_MASK,
					 ((fsel << 5) | (sfsel << 10))));
#endif
	return 0;
}

int dcm_peri(ENUM_PERI_DCM on)
{
	dcm_infracfg_ao_dcm_peri_bus(on);
	return 0;
}


int dcm_peri_dbc(int cnt)
{
#if 0
	reg_write(PERI_BUS_DCM_CTRL, aor(reg_read(PERI_BUS_DCM_CTRL),
						  ~(0x1f << 15),
						  (cnt << 15)));
#endif
	return 0;
}

/*
 * input argument
 * 0: 1/1
 * 1: 1/2
 * 2: 1/4
 * 3: 1/8
 * 4: 1/16
 * 5: 1/32
 * default: 0, 0
 */
int dcm_peri_rate(unsigned int fsel, unsigned int sfsel)
{
#if 0
	BUG_ON(fsel > 5 || sfsel > 5);

	fsel = 0x10 >> fsel;
	sfsel = 0x10 >> sfsel;

	reg_write(PERI_BUS_DCM_CTRL, aor(reg_read(PERI_BUS_DCM_CTRL),
					 ~PERI_DCM_SEL_MASK,
					 ((fsel << 5) | (sfsel << 10))));
#endif
	return 0;
}

int dcm_mcusys(ENUM_MCUSYS_DCM on)
{
	/* Bypass for ICE cannot attach issue */
	/* dcm_mp0_cpucfg_dcm_mcu_bus(on); */
	/* dcm_mp1_cpucfg_dcm_mcu_bus(on); */

	dcm_mcu_misccfg_dcm_mcu_bus(on);
	dcm_mcu_misccfg_cci_sync_dcm(on);
	dcm_mcu_misccfg_mp0_sync_dcm(on);
	dcm_mcu_misccfg_mp1_sync_dcm(on);
	dcm_mcsi_a(on);
	return 0;
}

int dcm_mcusys_cci_stall(ENUM_MCUSYS_DCM on)
{
	return 0;
}

int dcm_mcusys_sync_dcm(ENUM_MCUSYS_DCM on)
{
	return 0;
}

int dcm_dramc_ao(ENUM_DRAMC_AO_DCM on)
{
	dcm_dramc_ch0_top1_dcm_dramc_ch0_pd_ctrl(on);
	dcm_chn0_emi_dcm_emi_group(on);
	dcm_dramc_ch1_top1_dcm_dramc_ch1_pd_ctrl(on);
	dcm_chn1_emi_dcm_emi_group(on);
	return 0;
}

int dcm_ddrphy(ENUM_DDRPHY_DCM on)
{
	dcm_dramc_ch0_top0_ddrphy(on);
	dcm_dramc_ch1_top0_ddrphy(on);
	return 0;
}

int dcm_emi(ENUM_EMI_DCM on)
{
	dcm_emi_dcm_emi_group(on);
	return 0;
}

int dcm_stall(ENUM_STALL_DCM on)
{
	dcm_mcu_misccfg_mp0_stall_dcm(on);
	dcm_mcu_misccfg_mp1_stall_dcm(on);
	return 0;
}

/*****************************************************/
typedef struct _dcm {
	int current_state;
	int saved_state;
	int disable_refcnt;
	int default_state;
	DCM_FUNC func;
	DCM_PRESET_FUNC preset_func;
	int typeid;
	char *name;
} DCM;

static DCM dcm_array[NR_DCM_TYPE] = {
	{
	 .typeid = ARMCORE_DCM_TYPE,
	 .name = "ARMCORE_DCM",
	 .func = (DCM_FUNC) dcm_armcore,
	 .current_state = ARMCORE_DCM_MODE1,
	 .default_state = ARMCORE_DCM_MODE1,
	 .disable_refcnt = 0,
	 },
	{
	 .typeid = MCUSYS_DCM_TYPE,
	 .name = "MCUSYS_DCM",
	 .func = (DCM_FUNC) dcm_mcusys,
	 .current_state = MCUSYS_DCM_ON,
	 .default_state = MCUSYS_DCM_ON,
	 .disable_refcnt = 0,
	 },
	{
	 .typeid = INFRA_DCM_TYPE,
	 .name = "INFRA_DCM",
	 .func = (DCM_FUNC) dcm_infra,
	 /*.preset_func = (DCM_PRESET_FUNC) dcm_infra_preset,*/
	 .current_state = INFRA_DCM_ON,
	 .default_state = INFRA_DCM_ON,
	 .disable_refcnt = 0,
	 },
	{
	 .typeid = PERI_DCM_TYPE,
	 .name = "PERI_DCM",
	 .func = (DCM_FUNC) dcm_peri,
	 /*.preset_func = (DCM_PRESET_FUNC) dcm_peri_preset,*/
	 .current_state = PERI_DCM_ON,
	 .default_state = PERI_DCM_ON,
	 .disable_refcnt = 0,
	 },
	{
	 .typeid = EMI_DCM_TYPE,
	 .name = "EMI_DCM",
	 .func = (DCM_FUNC) dcm_emi,
	 .current_state = EMI_DCM_ON,
	 .default_state = EMI_DCM_ON,
	 .disable_refcnt = 0,
	 },
	{
	 .typeid = DRAMC_DCM_TYPE,
	 .name = "DRAMC_DCM",
	 .func = (DCM_FUNC) dcm_dramc_ao,
	 .current_state = DRAMC_AO_DCM_ON,
	 .default_state = DRAMC_AO_DCM_ON,
	 .disable_refcnt = 0,
	 },
	{
	 .typeid = DDRPHY_DCM_TYPE,
	 .name = "DDRPHY_DCM",
	 .func = (DCM_FUNC) dcm_ddrphy,
	 .current_state = DDRPHY_DCM_ON,
	 .default_state = DDRPHY_DCM_ON,
	 .disable_refcnt = 0,
	 },
	{
	 .typeid = STALL_DCM_TYPE,
	 .name = "STALL_DCM",
	 .func = (DCM_FUNC) dcm_stall,
	 .current_state = STALL_DCM_ON,
	 .default_state = STALL_DCM_ON,
	 .disable_refcnt = 0,
	 },
};

/*****************************************
 * DCM driver will provide regular APIs :
 * 1. dcm_restore(type) to recovery CURRENT_STATE before any power-off reset.
 * 2. dcm_set_default(type) to reset as cold-power-on init state.
 * 3. dcm_disable(type) to disable all dcm.
 * 4. dcm_set_state(type) to set dcm state.
 * 5. dcm_dump_state(type) to show CURRENT_STATE.
 * 6. /sys/power/dcm_state interface:  'restore', 'disable', 'dump', 'set'. 4 commands.
 *
 * spsecified APIs for workaround:
 * 1. (definitely no workaround now)
 *****************************************/
void dcm_set_default(unsigned int type)
{
	int i;
	DCM *dcm;

#ifndef ENABLE_DCM_IN_LK
#ifdef USING_PR_LOG
	dcm_info("[%s]type:0x%08x\n", __func__, type);
#endif
#else
#ifdef USING_PR_LOG
	dcm_info("[%s]type:0x%08x,INIT_DCM_TYPE_BY_K=0x%x\n", __func__, type,
		 INIT_DCM_TYPE_BY_K);
#endif
#endif

	mutex_lock(&dcm_lock);

	for (i = 0, dcm = &dcm_array[0]; i < NR_DCM_TYPE; i++, dcm++) {
		if (type & dcm->typeid) {
			dcm->saved_state = dcm->default_state;
			dcm->current_state = dcm->default_state;
			dcm->disable_refcnt = 0;
#ifdef ENABLE_DCM_IN_LK
			if (INIT_DCM_TYPE_BY_K & dcm->typeid) {
#endif
				if (dcm->preset_func)
					dcm->preset_func();
				dcm->func(dcm->current_state);
#ifdef ENABLE_DCM_IN_LK
			}
#endif

#ifdef USING_PR_LOG
			dcm_info("[%16s 0x%08x] current state:%d (%d)\n",
				 dcm->name, dcm->typeid, dcm->current_state,
				 dcm->disable_refcnt);
#endif
		}
	}
	mutex_unlock(&dcm_lock);
}

void dcm_set_state(unsigned int type, int state)
{
	int i;
	DCM *dcm;

#ifdef USING_PR_LOG
	dcm_info("[%s]type:0x%08x, set:%d\n", __func__, type, state);
#endif

	mutex_lock(&dcm_lock);

	for (i = 0, dcm = &dcm_array[0]; type && (i < NR_DCM_TYPE); i++, dcm++) {
		if (type & dcm->typeid) {
			type &= ~(dcm->typeid);

			dcm->saved_state = state;
			if (dcm->disable_refcnt == 0) {
				dcm->current_state = state;
				dcm->func(dcm->current_state);
			}

#ifdef USING_PR_LOG
			dcm_info("[%16s 0x%08x] current state:%d (%d)\n",
				 dcm->name, dcm->typeid, dcm->current_state,
				 dcm->disable_refcnt);
#endif

		}
	}

	mutex_unlock(&dcm_lock);
}


void dcm_disable(unsigned int type)
{
	int i;
	DCM *dcm;

#ifdef USING_PR_LOG
	dcm_info("[%s]type:0x%08x\n", __func__, type);
#endif

	mutex_lock(&dcm_lock);

	for (i = 0, dcm = &dcm_array[0]; type && (i < NR_DCM_TYPE); i++, dcm++) {
		if (type & dcm->typeid) {
			type &= ~(dcm->typeid);

			dcm->current_state = DCM_OFF;
			dcm->disable_refcnt++;
			dcm->func(dcm->current_state);

#ifdef USING_PR_LOG
			dcm_info("[%16s 0x%08x] current state:%d (%d)\n",
				 dcm->name, dcm->typeid, dcm->current_state,
				 dcm->disable_refcnt);
#endif

		}
	}

	mutex_unlock(&dcm_lock);

}

void dcm_restore(unsigned int type)
{
	int i;
	DCM *dcm;

#ifdef USING_PR_LOG
	dcm_info("[%s]type:0x%08x\n", __func__, type);
#endif

	mutex_lock(&dcm_lock);

	for (i = 0, dcm = &dcm_array[0]; type && (i < NR_DCM_TYPE); i++, dcm++) {
		if (type & dcm->typeid) {
			type &= ~(dcm->typeid);

			if (dcm->disable_refcnt > 0)
				dcm->disable_refcnt--;
			if (dcm->disable_refcnt == 0) {
				dcm->current_state = dcm->saved_state;
				dcm->func(dcm->current_state);
			}

#ifdef USING_PR_LOG
			dcm_info("[%16s 0x%08x] current state:%d (%d)\n",
				 dcm->name, dcm->typeid, dcm->current_state,
				 dcm->disable_refcnt);
#endif

		}
	}

	mutex_unlock(&dcm_lock);
}


void dcm_dump_state(int type)
{
	int i;
	DCM *dcm;

	dcm_info("\n******** dcm dump state *********\n");
	for (i = 0, dcm = &dcm_array[0]; i < NR_DCM_TYPE; i++, dcm++) {
		if (type & dcm->typeid) {
#ifdef USING_PR_LOG
			dcm_info("[%-16s 0x%08x] current state:%d (%d)\n",
				 dcm->name, dcm->typeid, dcm->current_state,
				 dcm->disable_refcnt);
#endif
		}
	}
}

void dcm_dump_regs(void)
{
	dcm_info("\n******** dcm dump register *********\n");
	REG_DUMP(INFRA_TOPCKGEN_DCMCTL);
	REG_DUMP(INFRA_BUS_DCM_CTRL);
	REG_DUMP(PERI_BUS_DCM_CTRL);
	REG_DUMP(MEM_DCM_CTRL);
	REG_DUMP(DFS_MEM_DCM_CTRL);
	REG_DUMP(INFRA_MISC);
	REG_DUMP(MP0_DBG_PWR_CTRL);
	REG_DUMP(MP1_DBG_PWR_CTRL);
	REG_DUMP(L2C_SRAM_CTRL);
	REG_DUMP(CCI_CLK_CTRL);
	REG_DUMP(BUS_FABRIC_DCM_CTRL);
	REG_DUMP(CCI_ADB400_DCM_CONFIG);
	REG_DUMP(SYNC_DCM_CONFIG);
	REG_DUMP(SYNC_DCM_CLUSTER_CONFIG);
	REG_DUMP(BIG_DBG_PWR_CTRL);
	REG_DUMP(EMI_CONM);
	REG_DUMP(EMI_CONN);
	REG_DUMP(DRAMC_CH0_TOP0_MISC_CG_CTRL0);
	REG_DUMP(DRAMC_CH0_TOP0_MISC_CG_CTRL2);
	REG_DUMP(DRAMC_CH0_TOP1_DRAMC_PD_CTRL);
	REG_DUMP(DRAMC_CH0_TOP1_CLKAR);
	REG_DUMP(CHN0_EMI_CHN_EMI_CONB);
	REG_DUMP(DRAMC_CH1_TOP0_MISC_CG_CTRL0);
	REG_DUMP(DRAMC_CH1_TOP0_MISC_CG_CTRL2);
	REG_DUMP(DRAMC_CH1_TOP1_DRAMC_PD_CTRL);
	REG_DUMP(DRAMC_CH1_TOP1_CLKAR);
	REG_DUMP(CHN1_EMI_CHN_EMI_CONB);
	SECURE_REG_DUMP(MCSI_A_DCM);
}


#if defined(CONFIG_PM)
static ssize_t dcm_state_show(struct kobject *kobj, struct kobj_attribute *attr,
				  char *buf)
{
	int len = 0;
	char *p = buf;
	int i;
	DCM *dcm;

	/* dcm_dump_state(ALL_DCM_TYPE); */
	p += sprintf(p, "\n******** dcm dump state *********\n");
	for (i = 0, dcm = &dcm_array[0]; i < NR_DCM_TYPE; i++, dcm++)
		p += sprintf(p, "[%-16s 0x%08x] current state:%d (%d)\n",
				 dcm->name, dcm->typeid, dcm->current_state,
				 dcm->disable_refcnt);

	p += sprintf(p, "\n********** dcm_state help *********\n");
	p += sprintf(p, "set:		echo set [mask] [mode] > /sys/power/dcm_state\n");
	p += sprintf(p, "disable:	echo disable [mask] > /sys/power/dcm_state\n");
	p += sprintf(p, "restore:	echo restore [mask] > /sys/power/dcm_state\n");
	p += sprintf(p, "dump:		echo dump [mask] > /sys/power/dcm_state\n");
	p += sprintf(p, "***** [mask] is hexl bit mask of dcm;\n");
	p += sprintf(p, "***** [mode] is type of DCM to set and retained\n");

	len = p - buf;
	return len;
}

#define MCUSYS_STALL_DCM_MP0_WR_DEL_SEL_MASK (0x1F << 0)
#define MCUSYS_STALL_DCM_MP1_WR_DEL_SEL_MASK (0x1F << 8)

static int dcm_convert_stall_wr_del_sel(unsigned int val)
{
	if (val < 0 || val > 0x1F)
		return 0;
	else
		return val;
}

int dcm_set_stall_wr_del_sel(unsigned int mp0, unsigned int mp1)
{
	mutex_lock(&dcm_lock);

	reg_write(SYNC_DCM_CLUSTER_CONFIG,
			aor(reg_read(SYNC_DCM_CLUSTER_CONFIG),
				~(MCUSYS_STALL_DCM_MP0_WR_DEL_SEL_MASK |
				MCUSYS_STALL_DCM_MP1_WR_DEL_SEL_MASK),
				((dcm_convert_stall_wr_del_sel(mp0) << 0) |
				(dcm_convert_stall_wr_del_sel(mp1) << 8))));

	mutex_unlock(&dcm_lock);
	return 0;
}

static ssize_t dcm_state_store(struct kobject *kobj,
				   struct kobj_attribute *attr, const char *buf,
				   size_t n)
{
	char cmd[16];
	unsigned int mask;
	unsigned int mp0, mp1;
	int ret, mode;

	if (sscanf(buf, "%15s %x", cmd, &mask) == 2) {
		mask &= ALL_DCM_TYPE;

		if (!strcmp(cmd, "restore")) {
			/* dcm_dump_regs(); */
			dcm_restore(mask);
			/* dcm_dump_regs(); */
		} else if (!strcmp(cmd, "disable")) {
			/* dcm_dump_regs(); */
			dcm_disable(mask);
			/* dcm_dump_regs(); */
		} else if (!strcmp(cmd, "dump")) {
			dcm_dump_state(mask);
			dcm_dump_regs();
		} else if (!strcmp(cmd, "set_stall_sel")) {
			if (sscanf(buf, "%15s %x %x", cmd, &mp0, &mp1) == 3) {
				dcm_set_stall_wr_del_sel(mp0, mp1);
			}
		} else if (!strcmp(cmd, "set")) {
			if (sscanf(buf, "%15s %x %d", cmd, &mask, &mode) == 3) {
				mask &= ALL_DCM_TYPE;

				dcm_set_state(mask, mode);

				/* Log for stallDCM switching in Performance/Normal mode */
				if (mask & 0x80) {
					if (mode)
						dcm_info("stall dcm is enabled for Default(Normal) mode started\n");
					else
						dcm_info("stall dcm is disabled for Performance(Sports) mode started\n");
				}
			}
		} else {
			dcm_info("SORRY, do not support your command: %s\n", cmd);
		}
		ret = n;
	} else {
		dcm_info("SORRY, do not support your command.\n");
		ret = -EINVAL;
	}

	return ret;
}

static struct kobj_attribute dcm_state_attr = {
	.attr = {
		 .name = "dcm_state",
		 .mode = 0644,
		 },
	.show = dcm_state_show,
	.store = dcm_state_store,
};
#endif				/* #if defined (CONFIG_PM) */

#if defined(CONFIG_OF)
static int mt_dcm_dts_map(void)
{
	struct device_node *node;
	struct resource r;

	/* topckgen */
	node = of_find_compatible_node(NULL, NULL, TOPCKGEN_NODE);
	if (!node) {
		dcm_info("error: cannot find node " TOPCKGEN_NODE);
		BUG();
	}
	dcm_topckgen_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_topckgen_base) {
		dcm_info("error: cannot iomap " TOPCKGEN_NODE);
		BUG();
	}

	/* mcucfg */
	node = of_find_compatible_node(NULL, NULL, MCUCFG_NODE);
	if (!node) {
		dcm_info("error: cannot find node " MCUCFG_NODE);
		BUG();
	}
	if (of_address_to_resource(node, 0, &r)) {
		dcm_info("error: cannot get phys addr" MCUCFG_NODE);
		BUG();
	}
	dcm_mcucfg_phys_base = r.start;

	dcm_mcucfg_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_mcucfg_base) {
		dcm_info("error: cannot iomap " MCUCFG_NODE);
		BUG();
	}

	/* dramc_ch0_top0 */
	node = of_find_compatible_node(NULL, NULL, DRAMC_CH0_TOP0_NODE);
	if (!node) {
		dcm_info("error: cannot find node " DRAMC_CH0_TOP0_NODE);
		BUG();
	}
	dcm_dramc_ch0_top0_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_dramc_ch0_top0_base) {
		dcm_info("error: cannot iomap " DRAMC_CH0_TOP0_NODE);
		BUG();
	}

	/* dramc_ch0_top1 */
	node = of_find_compatible_node(NULL, NULL, DRAMC_CH0_TOP1_NODE);
	if (!node) {
		dcm_info("error: cannot find node " DRAMC_CH0_TOP1_NODE);
		BUG();
	}
	dcm_dramc_ch0_top1_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_dramc_ch0_top1_base) {
		dcm_info("error: cannot iomap " DRAMC_CH0_TOP1_NODE);
		BUG();
	}

	/* dramc_ch0_top3 */
	node = of_find_compatible_node(NULL, NULL, DRAMC_CH0_TOP3_NODE);
	if (!node) {
		dcm_info("error: cannot find node " DRAMC_CH0_TOP3_NODE);
		BUG();
	}
	dcm_dramc_ch0_top3_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_dramc_ch0_top3_base) {
		dcm_info("error: cannot iomap " DRAMC_CH0_TOP3_NODE);
		BUG();
	}

	/* dramc_ch1_top0 */
	node = of_find_compatible_node(NULL, NULL, DRAMC_CH1_TOP0_NODE);
	if (!node) {
		dcm_info("error: cannot find node " DRAMC_CH1_TOP0_NODE);
		BUG();
	}
	dcm_dramc_ch1_top0_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_dramc_ch1_top0_base) {
		dcm_info("error: cannot iomap " DRAMC_CH1_TOP0_NODE);
		BUG();
	}

	/* dramc_ch1_top1 */
	node = of_find_compatible_node(NULL, NULL, DRAMC_CH1_TOP1_NODE);
	if (!node) {
		dcm_info("error: cannot find node " DRAMC_CH1_TOP1_NODE);
		BUG();
	}
	dcm_dramc_ch1_top1_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_dramc_ch1_top1_base) {
		dcm_info("error: cannot iomap " DRAMC_CH1_TOP1_NODE);
		BUG();
	}

	/* dramc_ch1_top3 */
	node = of_find_compatible_node(NULL, NULL, DRAMC_CH1_TOP3_NODE);
	if (!node) {
		dcm_info("error: cannot find node " DRAMC_CH1_TOP3_NODE);
		BUG();
	}
	dcm_dramc_ch1_top3_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_dramc_ch1_top3_base) {
		dcm_info("error: cannot iomap " DRAMC_CH1_TOP3_NODE);
		BUG();
	}

	/* emi */
	node = of_find_compatible_node(NULL, NULL, EMI_NODE);
	if (!node) {
		dcm_info("error: cannot find node " EMI_NODE);
		BUG();
	}
	dcm_emi_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_emi_base) {
		dcm_info("error: cannot iomap " EMI_NODE);
		BUG();
	}

	/* infracfg_ao */
	node = of_find_compatible_node(NULL, NULL, INFRACFG_AO_NODE);
	if (!node) {
		dcm_info("error: cannot find node " INFRACFG_AO_NODE);
		BUG();
	}
	dcm_infracfg_ao_base = (unsigned long)of_iomap(node, 0);
	if (!dcm_infracfg_ao_base) {
		dcm_info("error: cannot iomap " INFRACFG_AO_NODE);
		BUG();
	}

	return 0;
}
#else
static int mt_dcm_dts_map(void)
{
	return 0;
}
#endif

int mt_dcm_init(void)
{
	if (dcm_initiated)
		return 0;

	mt_dcm_dts_map();

#if !defined(DCM_DEFAULT_ALL_OFF)
	/** enable all dcm **/
	dcm_set_default(INIT_DCM_TYPE);
#else /* #if !defined (DCM_DEFAULT_ALL_OFF) */
	dcm_set_state(ALL_DCM_TYPE, DCM_OFF);
#endif /* #if !defined (DCM_DEFAULT_ALL_OFF) */

	dcm_dump_regs();

#if defined(CONFIG_PM)
	{
		int err = 0;

		err = sysfs_create_file(power_kobj, &dcm_state_attr.attr);
		if (err)
			dcm_err("[%s]: fail to create sysfs\n", __func__);
	}

#if defined(DCM_DEBUG_MON)
	{
		int err = 0;

		err = sysfs_create_file(power_kobj, &dcm_debug_mon_attr.attr);
		if (err)
			dcm_err("[%s]: fail to create sysfs\n", __func__);
	}
#endif /* #if defined (DCM_DEBUG_MON) */
#endif /* #if defined (CONFIG_PM) */

	dcm_initiated = 1;

	return 0;
}
late_initcall(mt_dcm_init);

/**** public APIs *****/
void mt_dcm_disable(void)
{
	mt_dcm_init();
	dcm_disable(ALL_DCM_TYPE);
}

void mt_dcm_restore(void)
{
	mt_dcm_init();
	dcm_restore(ALL_DCM_TYPE);
}

unsigned int sync_dcm_convert_freq2div(unsigned int freq)
{
	unsigned int div = 0;

	if (freq < 65)
		return 0;

	/* max divided ratio = Floor (CPU Frequency / 5 * system timer Frequency) */
	div = freq / 65;
	if (div > 31)
		return 31;

	return div;
}

#define MCUCFG_SYNC_DCM_TOGMASK     ((0x1 << 6) | (0x1 << 14) | (0x1 << 22))
#define MCUCFG_SYNC_DCM_CCI_TOGMASK (0x1 << 6)
#define MCUCFG_SYNC_DCM_MP0_TOGMASK (0x1 << 14)
#define MCUCFG_SYNC_DCM_MP1_TOGMASK (0x1 << 22)
#define MCUCFG_SYNC_DCM_TOG1        ((0x1 << 6) | (0x1 << 14) | (0x1 << 22))
#define MCUCFG_SYNC_DCM_CCI_TOG1    MCUCFG_SYNC_DCM_CCI_TOGMASK
#define MCUCFG_SYNC_DCM_MP0_TOG1    MCUCFG_SYNC_DCM_MP0_TOGMASK
#define MCUCFG_SYNC_DCM_MP1_TOG1    MCUCFG_SYNC_DCM_MP1_TOGMASK
#define MCUCFG_SYNC_DCM_TOG0        ((0x0 << 6) | (0x0 << 14) | (0x0 << 22))
#define MCUCFG_SYNC_DCM_CCI_TOG0    (0x0 << 6)
#define MCUCFG_SYNC_DCM_MP0_TOG0    (0x0 << 14)
#define MCUCFG_SYNC_DCM_MP1_TOG0    (0x0 << 22)
#define MCUCFG_SYNC_DCM_SEL_MASK	((0x1F << 1) | (0x1F << 9) | (0x1F << 17))
#define MCUCFG_SYNC_DCM_SEL_CCI_MASK ((0x1F << 1))
#define MCUCFG_SYNC_DCM_SEL_MP0_MASK ((0x1F << 9))
#define MCUCFG_SYNC_DCM_SEL_MP1_MASK ((0x1F << 17))

/* unit of frequency is MHz */
int sync_dcm_set_cpu_freq(unsigned int cci, unsigned int mp0, unsigned int mp1)
{
#if 1
	sync_dcm_set_cci_freq(cci);
	sync_dcm_set_mp0_freq(mp0);
	sync_dcm_set_mp1_freq(mp1);
#else
	mt_dcm_init();
	/* set xxx_sync_dcm_div and xxx_sync_dcm_tog as 0 first */
	/* set xxx_sync_dcm_tog as 1 */
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_SEL_MASK,
				((sync_dcm_convert_freq2div(cci) << 1) |
				 (sync_dcm_convert_freq2div(mp0) << 9) |
				 (sync_dcm_convert_freq2div(mp1) << 17))));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_TOGMASK,
				MCUCFG_SYNC_DCM_TOG0));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_TOGMASK,
				MCUCFG_SYNC_DCM_TOG1));
#ifdef DCM_DEBUG
#ifdef USING_PR_LOG
	dcm_info("%s: SYNC_DCM_CONFIG=0x%08x, cci=%u/%u,%u, mp0=%u/%u,%u, mp1=%u/%u,%u\n",
#endif
		 __func__, reg_read(SYNC_DCM_CONFIG), cci,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 1)) >> 1),
		 sync_dcm_convert_freq2div(cci), mp0,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 9)) >> 9),
		 sync_dcm_convert_freq2div(mp0), mp1,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 17)) >> 17),
		 sync_dcm_convert_freq2div(mp1));
#endif
#endif
	return 0;
}

int sync_dcm_set_cpu_div(unsigned int cci, unsigned int mp0, unsigned int mp1)
{
	if (cci > 31 || mp0 > 31 || mp1 > 31)
		return -1;

	mt_dcm_init();
	/* set xxx_sync_dcm_div and xxx_sync_dcm_tog as 0 first */
	/* set xxx_sync_dcm_tog as 1 */
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_SEL_MASK,
				((cci << 1) |
				 (mp0 << 9) |
				 (mp1 << 17))));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_TOGMASK,
				MCUCFG_SYNC_DCM_TOG0));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_TOGMASK,
				MCUCFG_SYNC_DCM_TOG1));
#ifdef DCM_DEBUG
#ifdef USING_PR_LOG
	dcm_info("%s: SYNC_DCM_CONFIG=0x%08x, cci=%u/%u, mp0=%u/%u, mp1=%u/%u\n",
#endif
		 __func__, reg_read(SYNC_DCM_CONFIG), cci,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 1)) >> 1), mp0,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 9)) >> 9), mp1,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 17)) >> 17));
#endif
	return 0;
}

int sync_dcm_set_cci_freq(unsigned int cci)
{
	mt_dcm_init();
	/* set xxx_sync_dcm_div and xxx_sync_dcm_tog as 0 first */
	/* set xxx_sync_dcm_tog as 1 */
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_SEL_CCI_MASK,
				sync_dcm_convert_freq2div(cci) << 1));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_CCI_TOGMASK,
				MCUCFG_SYNC_DCM_CCI_TOG0));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_CCI_TOGMASK,
				MCUCFG_SYNC_DCM_CCI_TOG1));
#ifdef DCM_DEBUG
#ifdef USING_PR_LOG
	dcm_info("%s: SYNC_DCM_CONFIG=0x%08x, cci=%u, cci_div_sel=%u,%u\n",
#endif
		 __func__, reg_read(SYNC_DCM_CONFIG), cci,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 1)) >> 1),
		 sync_dcm_convert_freq2div(cci));
#endif
	return 0;
}

int sync_dcm_set_mp0_freq(unsigned int mp0)
{
	mt_dcm_init();
	/* set xxx_sync_dcm_div and xxx_sync_dcm_tog as 0 first */
	/* set xxx_sync_dcm_tog as 1 */
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_SEL_MP0_MASK,
				sync_dcm_convert_freq2div(mp0) << 9));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_MP0_TOGMASK,
				MCUCFG_SYNC_DCM_MP0_TOG0));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_MP0_TOGMASK,
				MCUCFG_SYNC_DCM_MP0_TOG1));
#ifdef DCM_DEBUG
#ifdef USING_PR_LOG
	dcm_info("%s: SYNC_DCM_CONFIG=0x%08x, mp0=%u, mp0_div_sel=%u,%u\n",
#endif
		 __func__, reg_read(SYNC_DCM_CONFIG), mp0,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 9)) >> 9),
		 sync_dcm_convert_freq2div(mp0));
#endif
	return 0;
}

int sync_dcm_set_mp1_freq(unsigned int mp1)
{
	mt_dcm_init();
	/* set xxx_sync_dcm_div and xxx_sync_dcm_tog as 0 first */
	/* set xxx_sync_dcm_tog as 1 */
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_SEL_MP1_MASK,
				sync_dcm_convert_freq2div(mp1) << 17));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_MP1_TOGMASK,
				MCUCFG_SYNC_DCM_MP1_TOG0));
	reg_write(SYNC_DCM_CONFIG,
			aor(reg_read(SYNC_DCM_CONFIG),
				~MCUCFG_SYNC_DCM_MP1_TOGMASK,
				MCUCFG_SYNC_DCM_MP1_TOG1));
#ifdef DCM_DEBUG
#ifdef USING_PR_LOG
	dcm_info("%s: SYNC_DCM_CONFIG=0x%08x, mp1=%u, mp1_div_sel=%u,%u\n",
#endif
		 __func__, reg_read(SYNC_DCM_CONFIG), mp1,
		 (and(reg_read(SYNC_DCM_CONFIG), (0x1F << 17)) >> 17),
		 sync_dcm_convert_freq2div(mp1));
#endif
	return 0;
}

/* mt_dcm_topckg_disable/enable is used for slow idle */
void mt_dcm_topckg_disable(void)
{
#if 0
#if !defined(DCM_DEFAULT_ALL_OFF)
	reg_write(TOPCKG_DCM_CFG, aor(reg_read(TOPCKG_DCM_CFG),
			~TOPCKG_DCM_CFG_QMASK, TOPCKG_DCM_CFG_QOFF));
#endif /* #if !defined (DCM_DEFAULT_ALL_OFF) */
#endif /* 0 */
}

/* mt_dcm_topckg_disable/enable is used for slow idle */
void mt_dcm_topckg_enable(void)
{
#if 0
#if !defined(DCM_DEFAULT_ALL_OFF)
	if (dcm_array[TOPCKG_DCM].current_state != DCM_OFF) {
		reg_write(TOPCKG_DCM_CFG, aor(reg_read(TOPCKG_DCM_CFG),
				~TOPCKG_DCM_CFG_QMASK, TOPCKG_DCM_CFG_QON));
	}
#endif /* #if !defined (DCM_DEFAULT_ALL_OFF) */
#endif /* 0 */
}

void mt_dcm_topck_off(void)
{
#if 0
	mt_dcm_init();
	dcm_set_state(TOPCKG_DCM_TYPE, DCM_OFF);
#endif /* 0 */
}

void mt_dcm_topck_on(void)
{
#if 0
	mt_dcm_init();
	dcm_set_state(TOPCKG_DCM_TYPE, DCM_ON);
#endif /* 0 */
}

void mt_dcm_peri_off(void)
{
}

void mt_dcm_peri_on(void)
{
}

