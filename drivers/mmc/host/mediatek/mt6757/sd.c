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

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/blkdev.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/pagemap.h>
#include <linux/highmem.h>
#include <linux/printk.h>
#include <asm/page.h>
#include <linux/gpio.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/mt_lpae.h>

#ifdef MTK_MSDC_DMA_RETRY
#include <mt-plat/mt_chip.h>
#endif

#include "mt_sd.h"
#include <mmc/core/core.h>
#include <mmc/card/queue.h>

#ifdef MTK_MSDC_BRINGUP_DEBUG
#include <mach/mt_pmic_wrap.h>
#endif
#ifdef CONFIG_MTK_AEE_FEATURE
#include <mt-plat/aee.h>
#endif
#ifdef CONFIG_MTK_HIBERNATION
#include <mtk_hibernate_dpm.h>
#endif

#ifdef CONFIG_PWR_LOSS_MTK_TEST
#include <mach/power_loss_test.h>
#else
#define MVG_EMMC_CHECK_BUSY_AND_RESET(...)
#define MVG_EMMC_SETUP(...)
#define MVG_EMMC_RESET(...)
#define MVG_EMMC_WRITE_MATCH(...)
#define MVG_EMMC_ERASE_MATCH(...)
#define MVG_EMMC_ERASE_RESET(...)
#define MVG_EMMC_DECLARE_INT32(...)
#endif

#include "dbg.h"
#include "msdc_tune.h"
#include "autok.h"
#include "autok_dvfs.h"

#define CAPACITY_2G             (2 * 1024 * 1024 * 1024ULL)

u32 g_emmc_mode_switch; /*Check if its reference in sd_misc.h can be removed */

#define MSDC_MAX_FLUSH_COUNT    (3)
#define CACHE_UN_FLUSHED        (0)
#define CACHE_FLUSHED           (1)
#ifdef MTK_MSDC_USE_CACHE
static unsigned int g_cache_status = CACHE_UN_FLUSHED;
#endif
static unsigned long long g_flush_data_size;
static unsigned int g_flush_error_count;
static int g_flush_error_happend;

/* if disable cache by vendor fill CID.MID to g_emmc_cache_quirk[i] */
unsigned char g_emmc_cache_quirk[256];
#define CID_MANFID_SANDISK		0x2
#define CID_MANFID_TOSHIBA		0x11
#define CID_MANFID_MICRON		0x13
#define CID_MANFID_SAMSUNG		0x15
#define CID_MANFID_SANDISK_NEW		0x45
#define CID_MANFID_HYNIX		0x90
#define CID_MANFID_KSI			0x70

#if (MSDC_DATA1_INT == 1)
static u16 u_sdio_irq_counter;
static u16 u_msdc_irq_counter;
/*static int int_sdio_irq_enable;*/
#endif

struct msdc_host *ghost;
int src_clk_control;

bool emmc_sleep_failed;
static int emmc_do_sleep_awake;
static struct workqueue_struct *wq_init;

static bool sdio_use_dvfs;

#define DRV_NAME                "mtk-msdc"

#define MSDC_COOKIE_PIO         (1<<0)
#define MSDC_COOKIE_ASYNC       (1<<1)

#define msdc_use_async(x)       (x & MSDC_COOKIE_ASYNC)
#define msdc_use_async_dma(x)   (msdc_use_async(x) && (!(x & MSDC_COOKIE_PIO)))
#define msdc_use_async_pio(x)   (msdc_use_async(x) && ((x & MSDC_COOKIE_PIO)))

#define HOST_MAX_BLKSZ          (2048)

#define MSDC_OCR_AVAIL          (MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 \
				| MMC_VDD_31_32 | MMC_VDD_32_33)
#define DEFAULT_DTOC            (3)     /* data timeout counter.
					1048576 * 3 sclk. */

#define MAX_DMA_CNT             (64 * 1024 - 512)
				/* a WIFI transaction may be 50K */
#define MAX_DMA_CNT_SDIO        (0xFFFFFFFF - 255)
				/* a LTE  transaction may be 128K */

#define MAX_HW_SGMTS            (MAX_BD_NUM)
#define MAX_PHY_SGMTS           (MAX_BD_NUM)
#define MAX_SGMT_SZ             (MAX_DMA_CNT)
#define MAX_SGMT_SZ_SDIO        (MAX_DMA_CNT_SDIO)

u8 g_emmc_id;
unsigned int cd_gpio = 0;

struct msdc_host *mtk_msdc_host[] = { NULL, NULL, NULL};
EXPORT_SYMBOL(mtk_msdc_host);

int g_dma_debug[HOST_MAX_NUM] = { 0, 0, 0};
u32 latest_int_status[HOST_MAX_NUM] = { 0, 0, 0};

unsigned int msdc_latest_transfer_mode[HOST_MAX_NUM] = {
	/* 0 for PIO; 1 for DMA; 2 for nothing */
	TRAN_MOD_NUM,
	TRAN_MOD_NUM,
	TRAN_MOD_NUM
};

unsigned int msdc_latest_op[HOST_MAX_NUM] = {
	/* 0 for read; 1 for write; 2 for nothing */
	OPER_TYPE_NUM,
	OPER_TYPE_NUM,
	OPER_TYPE_NUM
};

/* for debug zone */
unsigned int sd_debug_zone[HOST_MAX_NUM] = {
	0,
	0,
	0
};
/* for enable/disable register dump */
unsigned int sd_register_zone[HOST_MAX_NUM] = {
	1,
	1,
	1
};
/* mode select */
u32 dma_size[HOST_MAX_NUM] = {
	512,
	8,
	512
};

u32 drv_mode[HOST_MAX_NUM] = {
	MODE_SIZE_DEP, /* using DMA or not depend on the size */
	MODE_SIZE_DEP,
	MODE_SIZE_DEP
};

u8 msdc_clock_src[HOST_MAX_NUM] = {
	0,
	0,
	0
};

u32 msdc_host_mode[HOST_MAX_NUM] = {
	0,
	0,
	0
};

u32 msdc_host_mode2[HOST_MAX_NUM] = {
	0,
	0,
	0
};

int msdc_rsp[] = {
	0,                      /* RESP_NONE */
	1,                      /* RESP_R1 */
	2,                      /* RESP_R2 */
	3,                      /* RESP_R3 */
	4,                      /* RESP_R4 */
	1,                      /* RESP_R5 */
	1,                      /* RESP_R6 */
	1,                      /* RESP_R7 */
	7,                      /* RESP_R1b */
};

/* For Inhanced DMA */
#define msdc_init_gpd_ex(gpd, extlen, cmd, arg, blknum) \
	do { \
		((struct gpd_t *)gpd)->extlen = extlen; \
		((struct gpd_t *)gpd)->cmd    = cmd; \
		((struct gpd_t *)gpd)->arg    = arg; \
		((struct gpd_t *)gpd)->blknum = blknum; \
	} while (0)

#define msdc_init_bd(bd, blkpad, dwpad, dptr, dlen) \
	do { \
		BUG_ON(dlen > 0xFFFFFFUL); \
		((struct bd_t *)bd)->blkpad = blkpad; \
		((struct bd_t *)bd)->dwpad = dwpad; \
		((struct bd_t *)bd)->ptrh4 = (u32)((dptr >> 32) & 0xF); \
		((struct bd_t *)bd)->ptr = (u32)(dptr & 0xFFFFFFFFUL); \
		((struct bd_t *)bd)->buflen = dlen; \
	} while (0)

#ifdef CONFIG_NEED_SG_DMA_LENGTH
#define msdc_sg_len(sg, dma)    ((dma) ? (sg)->dma_length : (sg)->length)
#else
#define msdc_sg_len(sg, dma)    sg_dma_len(sg)
#endif

#define msdc_dma_on()           MSDC_CLR_BIT32(MSDC_CFG, MSDC_CFG_PIO)
#define msdc_dma_off()          MSDC_SET_BIT32(MSDC_CFG, MSDC_CFG_PIO)
#define msdc_dma_status()       ((MSDC_READ32(MSDC_CFG) & MSDC_CFG_PIO) >> 3)

/***************************************************************
* BEGIN register dump functions
***************************************************************/
#define PRINTF_REGISTER_BUFFER_SIZE 512
#define ONE_REGISTER_STRING_SIZE    14

u16 msdc_offsets[] = {
	OFFSET_MSDC_CFG,
	OFFSET_MSDC_IOCON,
	OFFSET_MSDC_PS,
	OFFSET_MSDC_INT,
	OFFSET_MSDC_INTEN,
	OFFSET_MSDC_FIFOCS,
	OFFSET_SDC_CFG,
	OFFSET_SDC_CMD,
	OFFSET_SDC_ARG,
	OFFSET_SDC_STS,
	OFFSET_SDC_RESP0,
	OFFSET_SDC_RESP1,
	OFFSET_SDC_RESP2,
	OFFSET_SDC_RESP3,
	OFFSET_SDC_BLK_NUM,
	OFFSET_SDC_VOL_CHG,
	OFFSET_SDC_CSTS,
	OFFSET_SDC_CSTS_EN,
	OFFSET_SDC_DCRC_STS,
	OFFSET_SDC_CMD_STS,
	OFFSET_EMMC_CFG0,
	OFFSET_EMMC_CFG1,
	OFFSET_EMMC_STS,
	OFFSET_EMMC_IOCON,
	OFFSET_SDC_ACMD_RESP,
	OFFSET_SDC_ACMD19_TRG,
	OFFSET_SDC_ACMD19_STS,
	OFFSET_MSDC_DMA_SA_HIGH,
	OFFSET_MSDC_DMA_SA,
	OFFSET_MSDC_DMA_CA,
	OFFSET_MSDC_DMA_CTRL,
	OFFSET_MSDC_DMA_CFG,
	OFFSET_MSDC_DMA_LEN,
	OFFSET_MSDC_DBG_SEL,
	OFFSET_MSDC_DBG_OUT,
	OFFSET_MSDC_PATCH_BIT0,
	OFFSET_MSDC_PATCH_BIT1,
	OFFSET_MSDC_PATCH_BIT2,

	OFFSET_DAT0_TUNE_CRC,
	OFFSET_DAT1_TUNE_CRC,
	OFFSET_DAT2_TUNE_CRC,
	OFFSET_DAT3_TUNE_CRC,
	OFFSET_CMD_TUNE_CRC,
	OFFSET_SDIO_TUNE_WIND,

	OFFSET_MSDC_PAD_TUNE0,
	OFFSET_MSDC_PAD_TUNE1,
	OFFSET_MSDC_DAT_RDDLY0,
	OFFSET_MSDC_DAT_RDDLY1,
	OFFSET_MSDC_DAT_RDDLY2,
	OFFSET_MSDC_DAT_RDDLY3,
	OFFSET_MSDC_HW_DBG,
	OFFSET_MSDC_VERSION,

	OFFSET_EMMC50_PAD_DS_TUNE,
	OFFSET_EMMC50_PAD_CMD_TUNE,
	OFFSET_EMMC50_PAD_DAT01_TUNE,
	OFFSET_EMMC50_PAD_DAT23_TUNE,
	OFFSET_EMMC50_PAD_DAT45_TUNE,
	OFFSET_EMMC50_PAD_DAT67_TUNE,
	OFFSET_EMMC51_CFG0,
	OFFSET_EMMC50_CFG0,
	OFFSET_EMMC50_CFG1,
	OFFSET_EMMC50_CFG2,
	OFFSET_EMMC50_CFG3,
	OFFSET_EMMC50_CFG4,
	OFFSET_SDC_FIFO_CFG,

	OFFSET_MSDC_IOCON_1,
	OFFSET_MSDC_PATCH_BIT0_1,
	OFFSET_MSDC_PATCH_BIT1_1,
	OFFSET_MSDC_PATCH_BIT2_1,
	OFFSET_MSDC_PAD_TUNE0_1,
	OFFSET_MSDC_PAD_TUNE1_1,
	OFFSET_MSDC_DAT_RDDLY0_1,
	OFFSET_MSDC_DAT_RDDLY1_1,
	OFFSET_MSDC_DAT_RDDLY2_1,
	OFFSET_MSDC_DAT_RDDLY3_1,
	OFFSET_MSDC_IOCON_2,
	OFFSET_MSDC_PATCH_BIT0_2,
	OFFSET_MSDC_PATCH_BIT1_2,
	OFFSET_MSDC_PATCH_BIT2_2,
	OFFSET_MSDC_PAD_TUNE0_2,
	OFFSET_MSDC_PAD_TUNE1_2,
	OFFSET_MSDC_DAT_RDDLY0_2,
	OFFSET_MSDC_DAT_RDDLY1_2,
	OFFSET_MSDC_DAT_RDDLY2_2,
	OFFSET_MSDC_DAT_RDDLY3_2,

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	OFFSET_MTKCQ_CFG0,
	OFFSET_MTKCQ_CFG1,
	OFFSET_MTKCQ_CFG2,
	OFFSET_MTKCQ_ERR_ST,
	OFFSET_MTKCQ_CMD45_READY,
	OFFSET_MTKCQ_TASK_READY_ST,
	OFFSET_MTKCQ_TASK_DONE_ST,
	OFFSET_MTKCQ_ERR_ST_CLR,
	OFFSET_MTKCQ_CMD_DONE_CLR,
	OFFSET_MTKCQ_SW_CTL_CQ,
	OFFSET_MTKCQ_CMD44_RESP,
	OFFSET_MTKCQ_CMD45_RESP,
	OFFSET_MTKCQ_CMD13_RCA,
	OFFSET_EMMC51_CQCB_CFG3,
	OFFSET_EMMC51_CQCB_CMD44,
	OFFSET_EMMC51_CQCB_CMD45,
	OFFSET_EMMC51_CQCB_TIDMAP,
	OFFSET_EMMC51_CQCB_TIDMAPCLR,
	OFFSET_EMMC51_CQCB_CURCMD,
#endif

	0xFFFF /*as mark of end */
};

void msdc_dump_register_core(u32 id, void __iomem *base)
{
	u32 msg_size = 0;
	u16 i;
	char buffer[PRINTF_REGISTER_BUFFER_SIZE + 1];
	char *buffer_cur_ptr = buffer;

	memset(buffer, 0, PRINTF_REGISTER_BUFFER_SIZE);
	pr_err("MSDC%d normal register\n", id);
	for (i = 0; msdc_offsets[i] != 0xFFFF; i++) {

		if (((id != 2) && (id != 3))
		 && (msdc_offsets[i] >= OFFSET_DAT0_TUNE_CRC)
		 && (msdc_offsets[i] <= OFFSET_SDIO_TUNE_WIND))
			continue;

		msg_size += ONE_REGISTER_STRING_SIZE;
		if (msg_size >= PRINTF_REGISTER_BUFFER_SIZE) {
			pr_err("%s", buffer);
			memset(buffer, 0, PRINTF_REGISTER_BUFFER_SIZE);
			msg_size = ONE_REGISTER_STRING_SIZE;
			buffer_cur_ptr = buffer;
		}
		/* the size of one register string is 15 */
		snprintf(buffer_cur_ptr, ONE_REGISTER_STRING_SIZE+1, "[%.3hx:%.8x]",
			msdc_offsets[i], MSDC_READ32(base + msdc_offsets[i]));
		buffer_cur_ptr += ONE_REGISTER_STRING_SIZE;
	}

	pr_err("%s\n", buffer);
}

void msdc_dump_register(struct msdc_host *host)
{
	msdc_dump_register_core(host->id, host->base);
}

void msdc_dump_dbg_register_core(u32 id, void __iomem *base)
{
	u32 msg_size = 0;
	u16 i;
	char buffer[PRINTF_REGISTER_BUFFER_SIZE + 1];
	char *buffer_cur_ptr = buffer;

	memset(buffer, 0, PRINTF_REGISTER_BUFFER_SIZE);
	pr_err("MSDC debug register [set:out]\n");
	for (i = 0; i < MSDC_DEBUG_REGISTER_COUNT + 1; i++) {
		msg_size += ONE_REGISTER_STRING_SIZE;
		if (msg_size >= PRINTF_REGISTER_BUFFER_SIZE) {
			pr_err("%s", buffer);
			memset(buffer, 0, PRINTF_REGISTER_BUFFER_SIZE);
			msg_size = ONE_REGISTER_STRING_SIZE;
			buffer_cur_ptr = buffer;
		}
		MSDC_WRITE32(MSDC_DBG_SEL, i);
		snprintf(buffer_cur_ptr, ONE_REGISTER_STRING_SIZE+1, "[%.3hx:%.8x]",
			i, MSDC_READ32(MSDC_DBG_OUT));
		buffer_cur_ptr += ONE_REGISTER_STRING_SIZE;
	}

	pr_err("%s\n", buffer);

	MSDC_WRITE32(MSDC_DBG_SEL, 0);
}

static void msdc_dump_dbg_register(struct msdc_host *host)
{
	msdc_dump_dbg_register_core(host->id, host->base);
}

void msdc_dump_info(u32 id)
{
	struct msdc_host *host = mtk_msdc_host[id];
	void __iomem *base;

	if (host == NULL) {
		pr_err("msdc host<%d> null\n", id);
		return;
	}

	if (host->tuning_in_progress == true)
		return;

	/* when detect card, cmd13 will be sent which timeout log is not needed */
	if (!sd_register_zone[id]) {
		pr_err("msdc host<%d> is timeout when detect, so don't dump register\n", id);
		return;
	}

	host->prev_cmd_cause_dump++;
	if (host->prev_cmd_cause_dump > 1)
		return;

	base = host->base;

	msdc_dump_register(host);
	INIT_MSG("latest_INT_status<0x%.8x>", latest_int_status[id]);

	mdelay(10);

	msdc_dump_clock_sts(host);

	msdc_dump_ldo_sts(host);

	msdc_dump_padctl(host);

	mdelay(10);
	msdc_dump_dbg_register(host);
}

void msdc_raw_dump_info(u32 id)
{
	struct msdc_host *host = mtk_msdc_host[id];
	void __iomem *base;
	int ret;

	if (host == NULL) {
		pr_err("msdc host<%d> null\n", id);
		return;
	}

	pr_err("====== msdc%d dump start ======\n", host->id);

	pr_err("msdc%d need_tune = %d, %s\n",
		host->id,
		host->need_tune,
		host->tuning_in_progress ? "tuning":"not tuning");

	base = host->base;

	if (0 == host->core_clkon) {
		pr_err("host%d enable clock\n", host->id);
		ret = msdc_clk_enable(host);
		if (ret < 0)
			pr_err("host%d enable clock fail\n", host->id);
		host->core_clkon = 1;
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_SDMMC);
	}

	msdc_dump_register(host);
	INIT_MSG("latest_INT_status<0x%.8x>", latest_int_status[id]);

#ifdef MTK_MSDC_DUMP_STATUS_DEBUG
	pr_err("host%d [%5d.%06lu] dma start\n",
		host->id,
		(int)host->start_dma_time[id].tv_sec,
		host->start_dma_time[id].tv_nsec/1000);
	pr_err("host%d [%5d.%06lu] dma stop\n",
		host->id,
		(int)host->stop_dma_time[id].tv_sec,
		host->stop_dma_time[id].tv_nsec/1000);
#else
	pr_err("no last dma time stamp\n");
#endif

	mdelay(10);

	msdc_dump_clock_sts(host);

	msdc_dump_ldo_sts(host);

	msdc_dump_padctl(host);

	mdelay(10);
	msdc_dump_dbg_register(host);

	pr_err("======= msdc%d dump end =======\n", host->id);
}

/* for hang_detect call back */
void msdc_dump_status(void)
{
	/* dump msdc0 */
	msdc_raw_dump_info(0);
	/* dump msdc1 */
	msdc_raw_dump_info(1);
}
EXPORT_SYMBOL(msdc_dump_status);

/***************************************************************
* END register dump functions
***************************************************************/

/*
 * for AHB read / write debug
 * return DMA status.
 */
int msdc_get_dma_status(int host_id)
{
	if (host_id < 0 || host_id >= HOST_MAX_NUM) {
		pr_err("[%s] failed to get dma status, invalid host_id %d\n",
			__func__, host_id);
	} else if (msdc_latest_transfer_mode[host_id] == TRAN_MOD_DMA) {
		if (msdc_latest_op[host_id] == OPER_TYPE_READ)
			return 1;       /* DMA read */
		else if (msdc_latest_op[host_id] == OPER_TYPE_WRITE)
			return 2;       /* DMA write */
	} else if (msdc_latest_transfer_mode[host_id] == TRAN_MOD_PIO) {
		return 0;               /* PIO mode */
	}

	return -1;
}
EXPORT_SYMBOL(msdc_get_dma_status);

void msdc_clr_fifo(unsigned int id)
{
	int retry = 3, cnt = 1000;
	void __iomem *base;

	if (id < 0 || id >= HOST_MAX_NUM)
		return;
	base = mtk_msdc_host[id]->base;

	if (MSDC_READ32(MSDC_DMA_CFG) & MSDC_DMA_CFG_STS) {
		pr_err("<<<WARN>>>: msdc%d, clear FIFO when DMA active, MSDC_DMA_CFG=0x%x\n",
			id, MSDC_READ32(MSDC_DMA_CFG));
		show_stack(current, NULL);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_STOP, 1);
		msdc_retry((MSDC_READ32(MSDC_DMA_CFG) & MSDC_DMA_CFG_STS),
			retry, cnt, id);
		if (retry == 0) {
			pr_err("<<<WARN>>>: msdc%d, faield to stop DMA before clear FIFO, MSDC_DMA_CFG=0x%x\n",
				id, MSDC_READ32(MSDC_DMA_CFG));
			return;
		}
	}

	retry = 3;
	cnt = 1000;
	MSDC_SET_BIT32(MSDC_FIFOCS, MSDC_FIFOCS_CLR);
	msdc_retry(MSDC_READ32(MSDC_FIFOCS) & MSDC_FIFOCS_CLR, retry, cnt, id);
}

int msdc_clk_stable(struct msdc_host *host, u32 mode, u32 div,
	u32 hs400_div_dis)
{
	void __iomem *base = host->base;
	int retry = 0;
	int cnt = 1000;
	int retry_cnt = 1;
	int ret;

	do {
		retry = 3;
		MSDC_SET_FIELD(MSDC_CFG,
			MSDC_CFG_CKMOD_HS400 | MSDC_CFG_CKMOD | MSDC_CFG_CKDIV,
			(hs400_div_dis << 14) | (mode << 12) |
				((div + retry_cnt) % 0xfff));
		/* MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, mode); */
		msdc_retry(!(MSDC_READ32(MSDC_CFG) & MSDC_CFG_CKSTB), retry,
			cnt, host->id);
		if (retry == 0) {
			pr_err("msdc%d host->onclock(%d)\n", host->id,
				host->core_clkon);
			pr_err("msdc%d on clock failed ===> retry twice\n",
				host->id);

			msdc_clk_disable(host);
			ret = msdc_clk_enable(host);
			if (ret < 0)
				pr_err("msdc%d enable clock fail\n", host->id);

			msdc_dump_info(host->id);
			host->prev_cmd_cause_dump = 0;
		}
		retry = 3;
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKDIV, div);
		msdc_retry(!(MSDC_READ32(MSDC_CFG) & MSDC_CFG_CKSTB), retry,
			cnt, host->id);
		if (retry == 0)
			msdc_dump_info(host->id);
		msdc_reset_hw(host->id);
		if (retry_cnt == 2)
			break;
		retry_cnt++;
	} while (!retry);

	return 0;
}

#define msdc_irq_save(val) \
	do { \
		val = MSDC_READ32(MSDC_INTEN); \
		MSDC_CLR_BIT32(MSDC_INTEN, val); \
	} while (0)

#define msdc_irq_restore(val) \
	MSDC_SET_BIT32(MSDC_INTEN, val)

/* set the edge of data sampling */
void msdc_set_smpl(struct msdc_host *host, u32 clock_mode, u8 mode, u8 type, u8 *edge)
{
	void __iomem *base = host->base;
	int i = 0;

	switch (type) {
	case TYPE_CMD_RESP_EDGE:
		if (clock_mode == 3) {
			MSDC_SET_FIELD(EMMC50_CFG0,
				MSDC_EMMC50_CFG_PADCMD_LATCHCK, 0);
			MSDC_SET_FIELD(EMMC50_CFG0,
				MSDC_EMMC50_CFG_CMD_RESP_SEL, 0);
		}

		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_RSPL, mode);
		} else {
			ERR_MSG("invalid resp parameter: type=%d, mode=%d\n",
				type, mode);
		}
		break;
	case TYPE_WRITE_CRC_EDGE:
		if (clock_mode == 3) {
			/*latch write crc status at DS pin*/
			MSDC_SET_FIELD(EMMC50_CFG0,
				MSDC_EMMC50_CFG_CRC_STS_SEL, 1);
		} else {
			/*latch write crc status at CLK pin*/
			MSDC_SET_FIELD(EMMC50_CFG0,
				MSDC_EMMC50_CFG_CRC_STS_SEL, 0);
		}
		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			if (clock_mode == 3) {
				MSDC_SET_FIELD(EMMC50_CFG0,
					MSDC_EMMC50_CFG_CRC_STS_EDGE, mode);
			} else {
				MSDC_SET_FIELD(MSDC_PATCH_BIT2,
					MSDC_PB2_CFGCRCSTSEDGE, mode);
			}
		} else if ((mode == MSDC_SMPL_SEPARATE) &&
			   (edge != NULL) &&
			   (sizeof(edge) == 8)) {
			pr_err("%s: Shall not enter here\n", __func__);

		} else {
			ERR_MSG("invalid crc parameter: type=%d, mode=%d\n",
				type, mode);
		}
		break;
	case TYPE_READ_DATA_EDGE:
		if (clock_mode == 3) {
			/*for HS400, start bit is output on both edge*/
			MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_START_BIT,
				START_AT_RISING_AND_FALLING);
		} else {
			/*for the other modes, start bit is only output on
			 rising edge; but DDR50 can try falling edge
			 if error casued by pad delay*/
			MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_START_BIT,
				START_AT_RISING);
		}
		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_R_D_SMPL_SEL, 0);
			MSDC_SET_FIELD(MSDC_PATCH_BIT0,
				MSDC_PB0_RD_DAT_SEL, mode);
		} else if ((mode == MSDC_SMPL_SEPARATE) &&
			   (edge != NULL) &&
			   (sizeof(edge) == 8)) {
			pr_err("%s: Shall not enter here\n", __func__);
		} else {
			ERR_MSG("invalid read parameter: type=%d, mode=%d\n",
				type, mode);
		}
		break;
	case TYPE_WRITE_DATA_EDGE:
		/*latch write crc status at CLK pin*/
		MSDC_SET_FIELD(EMMC50_CFG0, MSDC_EMMC50_CFG_CRC_STS_SEL, 0);

		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_W_D_SMPL_SEL, 0);
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_W_D_SMPL, mode);
		} else if ((mode == MSDC_SMPL_SEPARATE) &&
			   (edge != NULL) &&
			   (sizeof(edge) >= 4)) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_W_D_SMPL_SEL, 1);
			for (i = 0; i < 4; i++) {
				/*dat0~4 is for SDIO card*/
				MSDC_SET_FIELD(MSDC_IOCON,
					(MSDC_IOCON_W_D0SPL << i), edge[i]);
			}
		} else {
			ERR_MSG("invalid write parameter: type=%d, mode=%d\n",
				type, mode);
		}
		break;
	default:
		ERR_MSG("invalid parameter: type=%d, mode=%d\n", type, mode);
		break;
	}
	/*pr_err("finished, HS400=%d, type=%d, mode=%d\n", HS400, type, mode);*/

}

void msdc_set_smpl_all(struct msdc_host *host, u32 clock_mode)
{
	msdc_set_smpl(host, clock_mode, host->hw->cmd_edge,
		TYPE_CMD_RESP_EDGE, NULL);
	msdc_set_smpl(host, clock_mode, host->hw->rdata_edge,
		TYPE_READ_DATA_EDGE, NULL);
	msdc_set_smpl(host, clock_mode, host->hw->wdata_edge,
		TYPE_WRITE_CRC_EDGE, NULL);
}

/*sd card change voltage wait time= (1/freq) * SDC_VOL_CHG_CNT(default 0x145)*/
#define msdc_set_vol_change_wait_count(count) \
	MSDC_SET_FIELD(SDC_VOL_CHG, SDC_VOL_CHG_CNT, (count))

void msdc_set_check_endbit(struct msdc_host *host, bool enable)
{
	void __iomem *base = host->base;

	if (enable) {
		MSDC_SET_BIT32(SDC_CMD_STS, SDC_CMD_STS_INDEX_CHECK);
		MSDC_SET_BIT32(SDC_CMD_STS, SDC_CMD_STS_ENDBIT_CHECK);
	} else {
		MSDC_CLR_BIT32(SDC_CMD_STS, SDC_CMD_STS_INDEX_CHECK);
		MSDC_CLR_BIT32(SDC_CMD_STS, SDC_CMD_STS_ENDBIT_CHECK);
	}
}

/*
 * Check the dat pin 1 to 3 is high when power up.
 * If any pin is low, removing cap 4 bit width.
 */
static void msdc_check_dat_1to3_high(struct msdc_host *host)
{
	void __iomem *base = host->base;
	unsigned long polling_tmo = 0;

	polling_tmo = jiffies + POLLING_PINS;
	while ((MSDC_READ32(MSDC_PS) & 0xE0000) != 0xE0000) {
		if (time_after(jiffies, polling_tmo)) {
			pr_err("msdc%d, some of device's pin, dat1~3 are stuck!\n", host->id);
			host->mmc->caps &= ~MMC_CAP_4_BIT_DATA;
			return;
		}
	}
	host->mmc->caps |= MMC_CAP_4_BIT_DATA;
}

/*
 * Check the dat pin 0 is high when power up.
 * If the pin is low, the card is bad and don't power up
 */
static int msdc_check_dat_0_high(struct msdc_host *host)
{
	void __iomem *base = host->base;
	unsigned long polling_tmo = 0;

	polling_tmo = jiffies + POLLING_PINS;
	while ((MSDC_READ32(MSDC_PS) & 0x10000) != 0x10000) {
		if (time_after(jiffies, polling_tmo)) {
			pr_err("msdc%d, device's pin dat0 is stuck!\n", host->id);
			host->block_bad_card = 1;
			host->power_control(host, 0);
			return 0;
		}
	}
	return 1;
}

static void msdc_clksrc_onoff(struct msdc_host *host, u32 on)
{
	void __iomem *base = host->base;
	u32 div, mode, hs400_div_dis;
	int ret;

	if ((on) && (0 == host->core_clkon)) {

		ret = msdc_clk_enable(host);
		if (ret < 0)
			pr_err("msdc%d enable clock fail\n", host->id);

		host->core_clkon = 1;
		udelay(10);

		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_SDMMC);

		MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, mode);
		MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKDIV, div);
		MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD_HS400, hs400_div_dis);
		msdc_clk_stable(host, mode, div, hs400_div_dis);

		/* Enable DVFS handshake need check restore done */
		if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
			if (sdio_use_dvfs == 1)
				spm_msdc_dvfs_setting(MSDC2_DVFS, 1);
		}
	} else if ((!on) && (host->core_clkon == 1) &&
		 (!((host->hw->flags & MSDC_SDIO_IRQ) && src_clk_control))) {

		/* Disable DVFS handshake */
		if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
			if (sdio_use_dvfs == 1)
				spm_msdc_dvfs_setting(MSDC2_DVFS, 0);
		}

		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_MS);

		msdc_clk_disable(host);

		host->core_clkon = 0;

	}
}

/*host doesn't need the clock on*/
void msdc_gate_clock(struct msdc_host *host, int delay)
{
	unsigned long flags;
	unsigned int suspend;

	/*Use delay<0 for suspend purpose*/
	if (delay < 0) {
		suspend = 1;
		delay = 0;
	} else {
		suspend = 0;
	}

	spin_lock_irqsave(&host->clk_gate_lock, flags);
	if ((suspend == 0) && (host->clk_gate_count > 0))
		host->clk_gate_count--;
	if (delay) {
		mod_timer(&host->timer, jiffies + CLK_TIMEOUT);
		N_MSG(CLK, "[%s]: msdc%d, clk_gate_count=%d, delay=%d",
			__func__, host->id, host->clk_gate_count, delay);
	} else if (host->clk_gate_count == 0) {
		del_timer(&host->timer);
		msdc_clksrc_onoff(host, 0);
		N_MSG(CLK, "[%s]: msdc%d, clock gated done",
			__func__, host->id);
	} else {
		if (is_card_sdio(host))
			host->error = -EBUSY;
		ERR_MSG("[%s]: msdc%d, clock is needed, clk_gate_count=%d",
			__func__, host->id, host->clk_gate_count);
	}
	spin_unlock_irqrestore(&host->clk_gate_lock, flags);
}

/* host does need the clock on */
void msdc_ungate_clock(struct msdc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_gate_lock, flags);
	host->clk_gate_count++;
	N_MSG(CLK, "[%s]: msdc%d, clk_gate_count=%d", __func__, host->id,
		host->clk_gate_count);
	if (host->clk_gate_count == 1)
		msdc_clksrc_onoff(host, 1);
	spin_unlock_irqrestore(&host->clk_gate_lock, flags);
}

#if 0
static void msdc_dump_card_status(struct msdc_host *host, u32 status)
{
	static const char const *state[] = {
		"Idle",         /* 0 */
		"Ready",        /* 1 */
		"Ident",        /* 2 */
		"Stby",         /* 3 */
		"Tran",         /* 4 */
		"Data",         /* 5 */
		"Rcv",          /* 6 */
		"Prg",          /* 7 */
		"Dis",          /* 8 */
		"Reserved",     /* 9 */
		"Reserved",     /* 10 */
		"Reserved",     /* 11 */
		"Reserved",     /* 12 */
		"Reserved",     /* 13 */
		"Reserved",     /* 14 */
		"I/O mode",     /* 15 */
	};
	if (status & R1_OUT_OF_RANGE)
		N_MSG(RSP, "[CARD_STATUS] Out of Range");
	if (status & R1_ADDRESS_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Address Error");
	if (status & R1_BLOCK_LEN_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Block Len Error");
	if (status & R1_ERASE_SEQ_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Erase Seq Error");
	if (status & R1_ERASE_PARAM)
		N_MSG(RSP, "[CARD_STATUS] Erase Param");
	if (status & R1_WP_VIOLATION)
		N_MSG(RSP, "[CARD_STATUS] WP Violation");
	if (status & R1_CARD_IS_LOCKED)
		N_MSG(RSP, "[CARD_STATUS] Card is Locked");
	if (status & R1_LOCK_UNLOCK_FAILED)
		N_MSG(RSP, "[CARD_STATUS] Lock/Unlock Failed");
	if (status & R1_COM_CRC_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Command CRC Error");
	if (status & R1_ILLEGAL_COMMAND)
		N_MSG(RSP, "[CARD_STATUS] Illegal Command");
	if (status & R1_CARD_ECC_FAILED)
		N_MSG(RSP, "[CARD_STATUS] Card ECC Failed");
	if (status & R1_CC_ERROR)
		N_MSG(RSP, "[CARD_STATUS] CC Error");
	if (status & R1_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Error");
	if (status & R1_UNDERRUN)
		N_MSG(RSP, "[CARD_STATUS] Underrun");
	if (status & R1_OVERRUN)
		N_MSG(RSP, "[CARD_STATUS] Overrun");
	if (status & R1_CID_CSD_OVERWRITE)
		N_MSG(RSP, "[CARD_STATUS] CID/CSD Overwrite");
	if (status & R1_WP_ERASE_SKIP)
		N_MSG(RSP, "[CARD_STATUS] WP Eraser Skip");
	if (status & R1_CARD_ECC_DISABLED)
		N_MSG(RSP, "[CARD_STATUS] Card ECC Disabled");
	if (status & R1_ERASE_RESET)
		N_MSG(RSP, "[CARD_STATUS] Erase Reset");
	if ((status & R1_READY_FOR_DATA) == 0)
		N_MSG(RSP, "[CARD_STATUS] Not Ready for Data");
	if (status & R1_SWITCH_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Switch error");
	if (status & R1_APP_CMD)
		N_MSG(RSP, "[CARD_STATUS] App Command");

	N_MSG(RSP, "[CARD_STATUS] '%s' State", state[R1_CURRENT_STATE(status)]);
}
#endif

static void msdc_set_timeout(struct msdc_host *host, u32 ns, u32 clks)
{
	void __iomem *base = host->base;
	u32 timeout, clk_ns;
	u32 mode = 0;

	host->timeout_ns = ns;
	host->timeout_clks = clks;
	if (host->sclk == 0) {
		timeout = 0;
	} else {
		clk_ns  = 1000000000UL / host->sclk;
		timeout = (ns + clk_ns - 1) / clk_ns + clks;
		/* in 1048576 sclk cycle unit */
		timeout = (timeout + (1 << 20) - 1) >> 20;
		MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, mode);
		/*DDR mode will double the clk cycles for data timeout*/
		timeout = mode >= 2 ? timeout * 2 : timeout;
		timeout = timeout > 1 ? timeout - 1 : 0;
		timeout = timeout > 255 ? 255 : timeout;
	}
	MSDC_SET_FIELD(SDC_CFG, SDC_CFG_DTOC, timeout);

	N_MSG(OPS, "msdc%d, Set read data timeout: %dns %dclks -> %d x 1048576 cycles, mode:%d, clk_freq=%dKHz\n",
		host->id, ns, clks, timeout + 1, mode, (host->sclk / 1000));
}

/* msdc_eirq_sdio() will be called when EIRQ(for WIFI) */
static void msdc_eirq_sdio(void *data)
{
	struct msdc_host *host = (struct msdc_host *)data;

	N_MSG(INT, "SDIO EINT");
#ifdef SDIO_ERROR_BYPASS
	if (host->sdio_error != -EILSEQ) {
#endif
		mmc_signal_sdio_irq(host->mmc);
#ifdef SDIO_ERROR_BYPASS
	}
#endif
}

void msdc_set_mclk(struct msdc_host *host, unsigned char timing, u32 hz)
{
	void __iomem *base = host->base;
	u32 mode;
	u32 flags;
	u32 div;
	u32 sclk;
	u32 hclk = host->hclk;
	u32 hs400_div_dis = 0; /* FOR MSDC_CFG.HS400CKMOD */

	if (!hz) { /* set mmc system clock to 0*/
		if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
			host->saved_para.hz = hz;
#ifdef SDIO_ERROR_BYPASS
			host->sdio_error = 0;
#endif
		}
		host->mclk = 0;
		msdc_reset_hw(host->id);
		return;
	}

	msdc_irq_save(flags);

	if (timing == MMC_TIMING_MMC_HS400) {
		mode = 0x3; /* HS400 mode */
		if (hz >= hclk/2) {
			hs400_div_dis = 1;
			div = 0;
			sclk = hclk/2;
		} else {
			hs400_div_dis = 0;
			if (hz >= (hclk >> 2)) {
				div  = 0;         /* mean div = 1/4 */
				sclk = hclk >> 2; /* sclk = clk / 4 */
			} else {
				div  = (hclk + ((hz << 2) - 1)) / (hz << 2);
				sclk = (hclk >> 2) / div;
				div  = (div >> 1);
			}
		}
	} else if ((timing == MMC_TIMING_UHS_DDR50)
		|| (timing == MMC_TIMING_MMC_DDR52)) {
		mode = 0x2; /* ddr mode and use divisor */
		if (hz >= (hclk >> 2)) {
			div  = 0;         /* mean div = 1/4 */
			sclk = hclk >> 2; /* sclk = clk / 4 */
		} else {
			div  = (hclk + ((hz << 2) - 1)) / (hz << 2);
			sclk = (hclk >> 2) / div;
			div  = (div >> 1);
		}
#if !defined(FPGA_PLATFORM)
	} else if (hz >= hclk) {
		mode = 0x1; /* no divisor */
		div  = 0;
		sclk = hclk;
#endif
	} else {
		mode = 0x0; /* use divisor */
		if (hz >= (hclk >> 1)) {
			div  = 0;         /* mean div = 1/2 */
			sclk = hclk >> 1; /* sclk = clk / 2 */
		} else {
			div  = (hclk + ((hz << 2) - 1)) / (hz << 2);
			sclk = (hclk >> 2) / div;
		}
	}

	msdc_clk_stable(host, mode, div, hs400_div_dis);

	host->sclk = sclk;
	host->mclk = hz;
	host->timing = timing;

	/* need because clk changed.*/
	msdc_set_timeout(host, host->timeout_ns, host->timeout_clks);

	msdc_set_smpl_all(host, mode);

	pr_err("msdc%d -> !!! Set<%dKHz> Source<%dKHz> -> sclk<%dKHz> timing<%d> mode<%d> div<%d> hs400_div_dis<%d>",
		host->id, hz/1000, hclk/1000, sclk/1000, (int)timing, mode, div,
		hs400_div_dis);

	msdc_irq_restore(flags);
}

void msdc_send_stop(struct msdc_host *host)
{
	struct mmc_command stop = {0};
	struct mmc_request mrq = {0};
	u32 err;

	stop.opcode = MMC_STOP_TRANSMISSION;
	stop.arg = 0;
	stop.flags = MMC_RSP_R1B | MMC_CMD_AC;

	mrq.cmd = &stop;
	stop.mrq = &mrq;
	stop.data = NULL;

	err = msdc_do_command(host, &stop, 0, CMD_TIMEOUT);
}

#if 0
static int msdc_app_cmd(struct mmc_host *mmc, struct msdc_host *host)
{
	struct mmc_command cmd = { 0 };
	struct mmc_request mrq = { 0 };
	u32 err;

	cmd.opcode = MMC_APP_CMD;
	cmd.arg = host->app_cmd_arg;    /* meet mmc->card is null when ACMD6 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;

	mrq.cmd = &cmd;
	cmd.mrq = &mrq;
	cmd.data = NULL;

	err = msdc_do_command(host, &cmd, 0, CMD_TIMEOUT);
	return err;
}
#endif

int msdc_get_card_status(struct mmc_host *mmc, struct msdc_host *host,
	u32 *status)
{
	struct mmc_command cmd = { 0 };
	struct mmc_request mrq = { 0 };
	u32 err;

	cmd.opcode = MMC_SEND_STATUS;   /* CMD13 */
	cmd.arg = host->app_cmd_arg;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;

	mrq.cmd = &cmd;
	cmd.mrq = &mrq;
	cmd.data = NULL;

	/* tune until CMD13 pass. */
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (host->mmc->card->ext_csd.cmdq_mode_en)
		err = msdc_do_cmdq_command(host, &cmd, 0, CMD_TIMEOUT);
	else
#endif
		err = msdc_do_command(host, &cmd, 0, CMD_TIMEOUT);

	if (status)
		*status = cmd.resp[0];

	return err;
}

static void msdc_pin_reset(struct msdc_host *host, int mode, int force_reset)
{
	struct msdc_hw *hw = (struct msdc_hw *)host->hw;
	void __iomem *base = host->base;

	/* Config reset pin */
	if ((hw->flags & MSDC_RST_PIN_EN) || force_reset) {
		if (mode == MSDC_PIN_PULL_UP)
			MSDC_CLR_BIT32(EMMC_IOCON, EMMC_IOCON_BOOTRST);
		else
			MSDC_SET_BIT32(EMMC_IOCON, EMMC_IOCON_BOOTRST);
	}
}

/* called by ops.set_ios */
static void msdc_set_buswidth(struct msdc_host *host, u32 width)
{
	void __iomem *base = host->base;
	u32 val = MSDC_READ32(SDC_CFG);

	val &= ~SDC_CFG_BUSWIDTH;

	switch (width) {
	default:
	case MMC_BUS_WIDTH_1:
		val |= (MSDC_BUS_1BITS << 16);
		break;
	case MMC_BUS_WIDTH_4:
		val |= (MSDC_BUS_4BITS << 16);
		break;
	case MMC_BUS_WIDTH_8:
		val |= (MSDC_BUS_8BITS << 16);
		break;
	}

	MSDC_WRITE32(SDC_CFG, val);
}

static void msdc_init_hw(struct msdc_host *host)
{
	void __iomem *base = host->base;

	/* Power on */
	/* msdc_pin_reset(host, MSDC_PIN_PULL_UP, 0); */

	msdc_ungate_clock(host);

	/* Configure to MMC/SD mode */
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_SDMMC);

	/* Reset */
	msdc_reset_hw(host->id);

	/* Disable card detection */
	MSDC_CLR_BIT32(MSDC_PS, MSDC_PS_CDEN);

	/* Disable and clear all interrupts */
	MSDC_CLR_BIT32(MSDC_INTEN, MSDC_READ32(MSDC_INTEN));
	MSDC_WRITE32(MSDC_INT, MSDC_READ32(MSDC_INT));

	/* reset tuning parameter */
	msdc_init_tune_setting(host);

	/* disable endbit check */
	if (host->id == 1)
		msdc_set_check_endbit(host, 0);

	/* for safety, should clear SDC_CFG.SDIO_INT_DET_EN & set SDC_CFG.SDIO
	   in pre-loader,uboot,kernel drivers.
	   SDC_CFG.SDIO_INT_DET_EN will be only set when kernel driver wants
	   to use SDIO bus interrupt */
	/* Enable SDIO mode. it's must otherwise sdio cmd5 failed */
	MSDC_SET_BIT32(SDC_CFG, SDC_CFG_SDIO);

	/* disable detect SDIO device interrupt function */
	if (host->hw->flags & MSDC_SDIO_IRQ) {
		ghost = host;
		/* enable sdio detection */
		MSDC_SET_BIT32(SDC_CFG, SDC_CFG_SDIOIDE);
	} else {
		MSDC_CLR_BIT32(SDC_CFG, SDC_CFG_SDIOIDE);
	}

	msdc_set_smt(host, 1);

	msdc_set_driving(host, host->hw, 0);

	/* Defalut SDIO GPIO mode is worng */
	if (host->hw->host_function == MSDC_SDIO)
		msdc_set_pin_mode(host);

	/*msdc_set_ies(host, 1);*/

	/* write crc timeout detection */
	MSDC_SET_FIELD(MSDC_PATCH_BIT0, MSDC_PB0_DETWR_CRCTMO, 1);

	/* Configure to default data timeout */
	MSDC_SET_FIELD(SDC_CFG, SDC_CFG_DTOC, DEFAULT_DTOC);

	msdc_set_buswidth(host, MMC_BUS_WIDTH_1);

	/* Configure support 64G */
	if (enable_4G())
		MSDC_CLR_BIT32(MSDC_PATCH_BIT2, MSDC_PB2_SUPPORT64G);
	else
		MSDC_SET_BIT32(MSDC_PATCH_BIT2, MSDC_PB2_SUPPORT64G);

	host->need_tune = TUNE_NONE;
	host->tune_smpl_times = 0;
	host->reautok_times = 0;

	msdc_gate_clock(host, 1);

	N_MSG(FUC, "init hardware done!");
}

/*
 * card hw reset if error
 * 1. reset pin:    DONW => 2us    => UP  => 200us
 * 2. power:        OFF     => 10us  => ON => 200us
*/
static void msdc_card_reset(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	pr_err("XXX msdc%d reset card\n", host->id);

	if (host->power_control) {
		host->power_control(host, 0);
		udelay(10);
		host->power_control(host, 1);
	}
	usleep_range(200, 500);

	msdc_pin_reset(host, MSDC_PIN_PULL_DOWN, 1);
	udelay(2);
	msdc_pin_reset(host, MSDC_PIN_PULL_UP, 1);
	usleep_range(200, 500);
	msdc_init_hw(host);
}

static void msdc_set_power_mode(struct msdc_host *host, u8 mode)
{
	N_MSG(CFG, "Set power mode(%d)", mode);
	if (host->power_mode == MMC_POWER_OFF && mode != MMC_POWER_OFF) {
		msdc_pin_reset(host, MSDC_PIN_PULL_UP, 0);
		msdc_pin_config(host, MSDC_PIN_PULL_UP);

		if (host->power_control)
			host->power_control(host, 1);

		mdelay(10);

		if (msdc_oc_check(host))
			return;
		if (host->id == 1) {
			/*
			 * check the dat pin 0 is high or not
			 * If dat0 is low, the card is bad and don't power up
			 */
			if (!msdc_check_dat_0_high(host))
				return;
			/*
			 * check the dat pin 1~3 is high or not
			 * If any pin is low, use one bit mode
			 */
			msdc_check_dat_1to3_high(host);
			msdc_set_check_endbit(host, 1);
		}

	} else if (host->power_mode != MMC_POWER_OFF && mode == MMC_POWER_OFF) {

		if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
			msdc_pin_config(host, MSDC_PIN_PULL_UP);
		} else {

			if (host->power_control)
				host->power_control(host, 0);

			msdc_pin_config(host, MSDC_PIN_PULL_DOWN);
		}
		mdelay(10);
		msdc_pin_reset(host, MSDC_PIN_PULL_DOWN, 0);
	}
	host->power_mode = mode;
}

#ifdef CONFIG_PM
static void msdc_pm(pm_message_t state, void *data)
{
	struct msdc_host *host = (struct msdc_host *)data;
	void __iomem *base = host->base;

	int evt = state.event;

	msdc_ungate_clock(host);
	if (host->hw->host_function == MSDC_EMMC)
		emmc_do_sleep_awake = 1;

	if (evt == PM_EVENT_SUSPEND || evt == PM_EVENT_USER_SUSPEND) {
		if (host->suspend)
			goto end;

		if (evt == PM_EVENT_SUSPEND &&
		     host->power_mode == MMC_POWER_OFF)
			goto end;

		host->suspend = 1;
		host->pm_state = state;

		N_MSG(PWR, "msdc%d -> %s Suspend", host->id,
			evt == PM_EVENT_SUSPEND ? "PM" : "USR");
		if (host->hw->flags & MSDC_SYS_SUSPEND) {
			if (host->hw->host_function == MSDC_EMMC) {
				msdc_save_timing_setting(host, 1);
				msdc_set_power_mode(host, MMC_POWER_OFF);
			}
			msdc_set_tdsel(host, 1, 0);
		} else {
			host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
			mmc_remove_host(host->mmc);
		}
	} else if (evt == PM_EVENT_RESUME || evt == PM_EVENT_USER_RESUME) {
		if (!host->suspend)
			goto end;

		if (evt == PM_EVENT_RESUME
			&& host->pm_state.event == PM_EVENT_USER_SUSPEND) {
			ERR_MSG("PM Resume when in USR Suspend");
			goto end;
		}

		host->suspend = 0;
		host->pm_state = state;

		N_MSG(PWR, "msdc%d -> %s Resume", host->id,
			evt == PM_EVENT_RESUME ? "PM" : "USR");

		if (!(host->hw->flags & MSDC_SYS_SUSPEND)) {
			host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
			mmc_add_host(host->mmc);
			goto end;
		}

		/* Begin for host->hw->flags & MSDC_SYS_SUSPEND*/
		msdc_set_tdsel(host, 1, 0);

		if (host->hw->host_function == MSDC_EMMC) {
			msdc_reset_hw(host->id);
			msdc_set_power_mode(host, MMC_POWER_ON);
			msdc_restore_timing_setting(host);

			if (emmc_sleep_failed) {
				msdc_card_reset(host->mmc);
				mdelay(200);
				mmc_card_clr_sleep(host->mmc->card);
				emmc_sleep_failed = 0;
				host->mmc->ios.timing = MMC_TIMING_LEGACY;
				mmc_set_clock(host->mmc, 260000);
			}
		}
	}

end:
#ifdef SDIO_ERROR_BYPASS
	if (is_card_sdio(host))
		host->sdio_error = 0;
#endif
	if ((evt == PM_EVENT_SUSPEND) || (evt == PM_EVENT_USER_SUSPEND)) {
		if ((host->hw->host_function == MSDC_SDIO) &&
		    (evt == PM_EVENT_USER_SUSPEND)) {
			pr_err("msdc%d -> MSDC Device Request Suspend",
				host->id);
		}
		msdc_gate_clock(host, 0);
	} else {
		msdc_gate_clock(host, 1);
	}
	if (host->hw->host_function == MSDC_EMMC)
		emmc_do_sleep_awake = 0;

	if (host->hw->host_function == MSDC_SDIO) {
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
		host->mmc->rescan_entered = 0;
	}
}
#endif

struct msdc_host *msdc_get_host(int host_function, bool boot, bool secondary)
{
	int host_index = 0;
	struct msdc_host *host = NULL, *host2;

	for (; host_index < HOST_MAX_NUM; ++host_index) {
		host2 = mtk_msdc_host[host_index];
		if (!host2)
			continue;
		if ((host_function == host2->hw->host_function) &&
		    (boot == host2->hw->boot)) {
			host = host2;
			break;
		}
	}
	if (secondary && (host_function == MSDC_SD))
		host = mtk_msdc_host[2];
	if (host == NULL) {
		pr_err("[MSDC] This host(<host_function:%d> <boot:%d><secondary:%d>) isn't in MSDC host config list",
			 host_function, boot, secondary);
		/* BUG(); */
	}

	return host;
}
EXPORT_SYMBOL(msdc_get_host);

int msdc_switch_part(struct msdc_host *host, char part_id)
{
	int ret = 0;
	/* FIX ME: use allocate, instead of declare big array */
	char l_buf[512];

	if (!host || !host->mmc || !host->mmc->card)
		return -ENOMEDIUM;

	ret = mmc_send_ext_csd(host->mmc->card, l_buf);
	if (ret)
		return ret;

	if ((part_id >= 0) && (part_id != (l_buf[EXT_CSD_PART_CONFIG] & 0x7))) {
		l_buf[EXT_CSD_PART_CONFIG] &= ~0x7;
		l_buf[EXT_CSD_PART_CONFIG] |= 0x0;
		ret = mmc_switch(host->mmc->card, 0, EXT_CSD_PART_CONFIG,
			l_buf[EXT_CSD_PART_CONFIG], 1000);
	}

	return ret;
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
#ifdef MTK_MSDC_DMA_RETRY
static void msdc_cq_onoff(struct mmc_data *data)
{
	u8 *ptr;
	struct scatterlist *sg;
	unsigned int ver;

	ver = mt_get_chip_hw_ver();
	sg = data->sg;
	ptr = (u8 *) sg_virt(sg);

	/* Turn of CQ */
	if (ver == 0xca00) {
		pr_err("%s: turn off cq 0x%x\n", __func__, ver);
		*(ptr + EXT_CSD_CMDQ_SUPPORT) = 0;

	}
}
#endif
#endif

static int msdc_cache_onoff(struct mmc_data *data)
{
	u8 *ptr;
#if defined(MTK_MSDC_USE_CACHE) && !defined(FPGA_PLATFORM)
	int i;
	enum boot_mode_t mode;
#endif
	struct scatterlist *sg;

	sg = data->sg;
	ptr = (u8 *) sg_virt(sg);

	/*
	 * if not defined MTK_MSDC_USE_CACHE disable cache
	 */
#if !defined(MTK_MSDC_USE_CACHE) || defined(FPGA_PLATFORM)
	*(ptr + 252) = *(ptr + 251) = *(ptr + 250) = *(ptr + 249) = 0;
	return 0;
#else
	/*
	 * Enable cache by boot mode
	 * only enable the emmc cache for normal boot up, alarm, and sw reboot
	 * Other mode will disable it
	 */
	mode = get_boot_mode();
	if ((mode != NORMAL_BOOT) && (mode != ALARM_BOOT)
		&& (mode != SW_REBOOT)) {
		/* Set cache_size is 0, mmc layer will not enable cache */
		*(ptr + 252) = *(ptr + 251) = *(ptr + 250) = *(ptr + 249) = 0;
		return 0;
	}
	/*
	 * Enable cache by eMMC vendor
	 * If eMMC in emmc_cache_quirk[], Don't enable it.
	 */

	for (i = 0; i < sizeof(g_emmc_cache_quirk); i++) {
		if (g_emmc_cache_quirk[i] == g_emmc_id) {
			/* Set cache_size is 0,
			 * mmc layer will not enable cache
			 */
			*(ptr + 252) = *(ptr + 251) = 0;
			*(ptr + 250) = *(ptr + 249) = 0;
		}
	}
	return 0;
#endif
}

static unsigned int msdc_get_sectors(struct mmc_data *data)
{
	u8 *ptr;
	struct scatterlist *sg;
	unsigned int sectors;

	sg = data->sg;
	ptr = (u8 *) sg_virt(sg);

	sectors = *(ptr + EXT_CSD_SEC_CNT + 0) << 0 |
		 *(ptr + EXT_CSD_SEC_CNT + 1) << 8 |
		 *(ptr + EXT_CSD_SEC_CNT + 2) << 16 |
		 *(ptr + EXT_CSD_SEC_CNT + 3) << 24;

	return sectors;
}

#ifdef MTK_MSDC_USE_CACHE
static void msdc_set_cache_quirk(struct msdc_host *host)
{
	int i;
	/*
	 * if need disable emmc cache feature by vendor, Add quirk here
	 * exmple:
	 * g_emmc_cache_quirk[0] = CID_MANFID_HYNIX;
	 * g_emmc_cache_quirk[1] = CID_MANFID_SAMSUNG;
	 */


	for (i = 0; i < sizeof(g_emmc_cache_quirk); i++) {
		if (g_emmc_cache_quirk[i] == 0) {
			/* g_emmc_cache_quirk[i] = eMMC id; */
			pr_debug("msdc%d total emmc cache quirk count=%d\n",
				host->id, i);
			break;
		}
		pr_debug("msdc%d,add emmc cache quirk[%d]=0x%x\n",
			host->id, i, g_emmc_cache_quirk[i]);
	 }
}
#endif

int msdc_cache_ctrl(struct msdc_host *host, unsigned int enable,
	u32 *status)
{
	struct mmc_command cmd = { 0 };
	struct mmc_request mrq = { 0 };
	u32 err;

	cmd.opcode = MMC_SWITCH;        /* CMD6 */
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24)
		| (EXT_CSD_CACHE_CTRL << 16) | (!!enable << 8)
		| EXT_CSD_CMD_SET_NORMAL;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

	mrq.cmd = &cmd;
	cmd.mrq = &mrq;
	cmd.data = NULL;

	ERR_MSG("do disable Cache, cmd=0x%x, arg=0x%x\n", cmd.opcode, cmd.arg);
	/* tune until CMD13 pass. */
	err = msdc_do_command(host, &cmd, 0, CMD_TIMEOUT);

	if (status)
		*status = cmd.resp[0];
	if (!err) {
		host->mmc->card->ext_csd.cache_ctrl = !!enable;
		host->autocmd |= MSDC_AUTOCMD23;
		N_MSG(CHE, "enable AUTO_CMD23 because Cache feature is disabled\n");
	}

	return err;
}

#ifdef MTK_MSDC_USE_CACHE
static void msdc_update_cache_flush_status(struct msdc_host *host,
	struct mmc_request *mrq, struct mmc_data *data,
	u32 l_bypass_flush)
{
	struct mmc_command *cmd = mrq->cmd;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	struct mmc_command *sbc;
	unsigned int task_id;
#endif

	if (!check_mmc_cache_ctrl(host->mmc->card))
		return;

	if (check_mmc_cmd2425(cmd->opcode)) {
		if ((host->error == 0)
		 && mrq->sbc
		 && (((mrq->sbc->arg >> 24) & 0x1) ||
		     ((mrq->sbc->arg >> 31) & 0x1))) {
			/* if reliable write, or force prg write succeed,
			   do set cache flushed status */
			if (g_cache_status == CACHE_UN_FLUSHED) {
				g_cache_status = CACHE_FLUSHED;
				N_MSG(CHE, "reliable/force prg write happend, update g_cache_status = %d",
					g_cache_status);
				N_MSG(CHE, "reliable/force prg write happend, update g_flush_data_size=%lld",
					g_flush_data_size);
				g_flush_data_size = 0;
			}
		} else if (host->error == 0) {
			/* if normal write succee,
			   do clear the cache flushed status */
			if (g_cache_status == CACHE_FLUSHED) {
				g_cache_status = CACHE_UN_FLUSHED;
				N_MSG(CHE, "normal write happend, update g_cache_status = %d",
					g_cache_status);
			}
			g_flush_data_size += data->blocks;
		} else if (host->error) {
			g_flush_data_size += data->blocks;
			ERR_MSG("write error happend, g_flush_data_size=%lld",
				g_flush_data_size);
		}
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	} else if (cmd->opcode == MMC_WRITE_REQUESTED_QUEUE) {
		if (host->error == 0) {
			task_id = (cmd->arg >> 16) & 0x1f;
			sbc = host->mmc->areq_que[task_id]->mrq_que->sbc;
			if (sbc	&& (((sbc->arg >> 24) & 0x1) ||
				((sbc->arg >> 31) & 0x1))) {
				/* if reliable write, or force prg write succeed,
					do set cache flushed status */
				if (g_cache_status == CACHE_UN_FLUSHED) {
					g_cache_status = CACHE_FLUSHED;
					N_MSG(CHE, "reliable/force prg write happend, update g_cache_status = %d",
						g_cache_status);
					N_MSG(CHE, "reliable/force prg write happend, update g_flush_data_size=%lld",
						g_flush_data_size);
					g_flush_data_size = 0;
				}
			} else {
				/* if normal write succee,
					do clear the cache flushed status */
				if (g_cache_status == CACHE_FLUSHED) {
					g_cache_status = CACHE_UN_FLUSHED;
					N_MSG(CHE, "normal write happend, update g_cache_status = %d",
						g_cache_status);
				}
				g_flush_data_size += data->blocks;
			}
		} else if (host->error) {
			g_flush_data_size += data->blocks;
			ERR_MSG("write error happend, g_flush_data_size=%lld",
				g_flush_data_size);
		}
#endif
	} else if (l_bypass_flush == 0) {
		if (host->error == 0) {
			/* if flush cache of emmc device successfully,
			   do set the cache flushed status */
			g_cache_status = CACHE_FLUSHED;
			N_MSG(CHE, "flush happend, update g_cache_status = %d, g_flush_data_size=%lld",
				g_cache_status, g_flush_data_size);
			g_flush_data_size = 0;
		} else {
			g_flush_error_happend = 1;
		}
	}
}
#endif

void msdc_check_cache_flush_error(struct msdc_host *host,
	struct mmc_command *cmd)
{
	if (g_flush_error_happend &&
	    check_mmc_cache_ctrl(host->mmc->card) &&
	    check_mmc_cache_flush_cmd(cmd)) {
		g_flush_error_count++;
		g_flush_error_happend = 0;
		ERR_MSG("the %d time flush error happned, g_flush_data_size=%lld",
			g_flush_error_count, g_flush_data_size);
		/*
		 * if reinit emmc at resume, cache should not be enabled
		 * because too much flush error, so add cache quirk for
		 * this emmmc.
		 * if awake emmc at resume, cache should not be enabled
		 * because too much flush error, so force set cache_size = 0
		 */
		if (g_flush_error_count >= MSDC_MAX_FLUSH_COUNT) {
			if (!msdc_cache_ctrl(host, 0, NULL)) {
				g_emmc_cache_quirk[0] = g_emmc_id;
				host->mmc->card->ext_csd.cache_size = 0;
			}
			pr_err("msdc%d:flush cache error count=%d,Disable cache\n",
				host->id, g_flush_error_count);
		}
	}
}

/*--------------------------------------------------------------------------*/
/* mmc_host_ops members                                                     */
/*--------------------------------------------------------------------------*/
static u32 wints_cmd = MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO |
		       MSDC_INT_ACMDRDY | MSDC_INT_ACMDCRCERR |
		       MSDC_INT_ACMDTMO;
static unsigned int msdc_command_start(struct msdc_host   *host,
	struct mmc_command *cmd,
	int                 tune,   /* not used */

	unsigned long       timeout)
{
	void __iomem *base = host->base;
	u32 opcode = cmd->opcode;
	u32 rawcmd;
	u32 rawarg;
	u32 resp;
	unsigned long tmo;
	struct mmc_command *sbc = NULL;
	char *str;

	if (host->data && host->data->mrq && host->data->mrq->sbc)
		sbc = host->data->mrq->sbc;

	/* Protocol layer does not provide response type, but our hardware needs
	 * to know exact type, not just size!
	 */
	switch (opcode) {
	case MMC_SEND_OP_COND:
	case SD_APP_OP_COND:
		resp = RESP_R3;
		break;
	case MMC_SET_RELATIVE_ADDR:
	/*case SD_SEND_RELATIVE_ADDR:*/
		/* Since SD_SEND_RELATIVE_ADDR=MMC_SET_RELATIVE_ADDR=3,
		   only one is allowed in switch case.*/
		resp = (mmc_cmd_type(cmd) == MMC_CMD_BCR) ? RESP_R6 : RESP_R1;
		break;
	case MMC_FAST_IO:
		resp = RESP_R4;
		break;
	case MMC_GO_IRQ_STATE:
		resp = RESP_R5;
		break;
	case MMC_SELECT_CARD:
		resp = (cmd->arg != 0) ? RESP_R1 : RESP_NONE;
		host->app_cmd_arg = cmd->arg;
		N_MSG(PWR, "msdc%d select card<0x%.8x>", host->id, cmd->arg);
		break;
	case SD_IO_RW_DIRECT:
	case SD_IO_RW_EXTENDED:
		/* SDIO workaround. */
		resp = RESP_R1;
		break;
	case SD_SEND_IF_COND:
		resp = RESP_R1;
		break;
	/* Ignore crc errors when sending status cmd to poll for busy
	 * MMC_RSP_CRC will be set, then mmc_resp_type will return
	 * MMC_RSP_NONE. CMD13 will not receive resp
	 */
	case MMC_SEND_STATUS:
		resp = RESP_R1;
		break;
	default:
		switch (mmc_resp_type(cmd)) {
		case MMC_RSP_R1:
			resp = RESP_R1;
			break;
		case MMC_RSP_R1B:
			resp = RESP_R1B;
			break;
		case MMC_RSP_R2:
			resp = RESP_R2;
			break;
		case MMC_RSP_R3:
			resp = RESP_R3;
			break;
		case MMC_RSP_NONE:
		default:
			resp = RESP_NONE;
			break;
		}
	}

	cmd->error = 0;
	/* rawcmd :
	 * vol_swt << 30 | auto_cmd << 28 | blklen << 16 | go_irq << 15 |
	 * stop << 14 | rw << 13 | dtype << 11 | rsptyp << 7 | brk << 6 |
	 * opcode
	 */

	rawcmd = opcode | msdc_rsp[resp] << 7 | host->blksz << 16;

	switch (opcode) {
	case MMC_READ_MULTIPLE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
		rawcmd |= (2 << 11);
		if (opcode == MMC_WRITE_MULTIPLE_BLOCK)
			rawcmd |= (1 << 13);
		if (host->autocmd & MSDC_AUTOCMD12) {
			rawcmd |= (1 << 28);
			N_MSG(CMD, "AUTOCMD12 is set, addr<0x%x>", cmd->arg);
#ifdef MTK_MSDC_USE_CMD23
		} else if ((host->autocmd & MSDC_AUTOCMD23)) {
			unsigned int reg_blk_num;

			rawcmd |= (1 << 29);
			if (sbc) {
				/* if block number is greater than 0xFFFF,
				   CMD23 arg will fail to set it.*/
				reg_blk_num = MSDC_READ32(SDC_BLK_NUM);
				if (reg_blk_num != (sbc->arg & 0xFFFF))
					pr_err("msdc%d: acmd23 arg(0x%x) fail to match block num(0x%x), SDC_BLK_NUM(0x%x)\n",
						host->id, sbc->arg,
						host->mrq->cmd->data->blocks,
						reg_blk_num);
				else
					MSDC_WRITE32(SDC_BLK_NUM, sbc->arg);
				N_MSG(CMD, "AUTOCMD23 addr<0x%x>, arg<0x%x> ",
					cmd->arg, sbc->arg);
			}
#endif /* end of MTK_MSDC_USE_CMD23 */
		}
		break;

	case MMC_READ_SINGLE_BLOCK:
	case MMC_SEND_TUNING_BLOCK:
	case MMC_SEND_TUNING_BLOCK_HS200:
		rawcmd |= (1 << 11);
		break;
	case MMC_WRITE_BLOCK:
		rawcmd |= ((1 << 11) | (1 << 13));
		break;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	case MMC_READ_REQUESTED_QUEUE:
		rawcmd |= (2 << 11);
		break;
	case MMC_WRITE_REQUESTED_QUEUE:
		rawcmd |= ((2 << 11) | (1 << 13));
		break;
	case MMC_CMDQ_TASK_MGMT:
		break;
#endif
	case SD_IO_RW_EXTENDED:
		if (cmd->data->flags & MMC_DATA_WRITE)
			rawcmd |= (1 << 13);
		if (cmd->data->blocks > 1)
			rawcmd |= (2 << 11);
		else
			rawcmd |= (1 << 11);
		break;
	case SD_IO_RW_DIRECT:
		if (cmd->flags == (unsigned int)-1)
			rawcmd |= (1 << 14);
		break;
	case SD_SWITCH_VOLTAGE:
		rawcmd |= (1 << 30);
		break;
	case SD_APP_SEND_SCR:
	case SD_APP_SEND_NUM_WR_BLKS:
	case MMC_SEND_WRITE_PROT:
	/*case MMC_SEND_WRITE_PROT_TYPE:*/
		rawcmd |= (1 << 11);
		break;
	case SD_SWITCH:
	case SD_APP_SD_STATUS:
	case MMC_SEND_EXT_CSD:
		if (mmc_cmd_type(cmd) == MMC_CMD_ADTC)
			rawcmd |= (1 << 11);
		break;
	case MMC_STOP_TRANSMISSION:
		rawcmd |= (1 << 14);
		rawcmd &= ~(0x0FFF << 16);
		break;
	}

	N_MSG(CMD, "CMD<%d><0x%.8x> Arg<0x%.8x>", opcode, rawcmd, cmd->arg);

	tmo = jiffies + timeout;

	if (opcode == MMC_SEND_STATUS) {
		for (;;) {
			if (!sdc_is_cmd_busy())
				break;
			if (time_after(jiffies, tmo)) {
				str = "cmd_busy";
				goto err;
			}
		}
	} else {
		for (;;) {
			if (!sdc_is_busy())
				break;
			if (time_after(jiffies, tmo)) {
				str = "sdc_busy";
				goto err;
			}
		}
	}

	host->cmd = cmd;
	host->cmd_rsp = resp;

	/* use polling way */
	MSDC_CLR_BIT32(MSDC_INTEN, wints_cmd);
	rawarg = cmd->arg;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	dbg_add_host_log(host->mmc, 0, cmd->opcode, cmd->arg);
#endif

	sdc_send_cmd(rawcmd, rawarg);

	return 0;

err:
	ERR_MSG("XXX %s timeout: before CMD<%d>", str, opcode);
	cmd->error = (unsigned int)-ETIMEDOUT;
	msdc_dump_register(host);
	msdc_reset_hw(host->id);

	return cmd->error;
}

static u32 msdc_command_resp_polling(struct msdc_host *host,
	struct mmc_command *cmd,
	int                 tune,
	unsigned long       timeout)
{
	void __iomem *base = host->base;
	u32 intsts;
	u32 resp;
	unsigned long tmo;

	u32 cmdsts = MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO;
#ifdef MTK_MSDC_USE_CMD23
	struct mmc_command *sbc = NULL;

	if (host->autocmd & MSDC_AUTOCMD23) {
		if (host->data && host->data->mrq && host->data->mrq->sbc)
			sbc = host->data->mrq->sbc;

		/* autocmd interrupt disabled, used polling way */
		cmdsts |= MSDC_INT_ACMDCRCERR | MSDC_INT_ACMDTMO;
	}
#endif

	resp = host->cmd_rsp;

	/*polling */
	tmo = jiffies + timeout;
	while (1) {
		intsts = MSDC_READ32(MSDC_INT);
		if ((intsts & cmdsts) != 0) {
			/* clear all int flag */
#ifdef MTK_MSDC_USE_CMD23
			/* need clear autocmd23 command ready interrupt */
			intsts &= (cmdsts | MSDC_INT_ACMDRDY);
#else
			intsts &= cmdsts;
#endif
			MSDC_WRITE32(MSDC_INT, intsts);
			break;
		}

		if (time_after(jiffies, tmo)) {
			pr_err("[%s]: msdc%d CMD<%d> polling_for_completion timeout ARG<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
			cmd->error = (unsigned int)-ETIMEDOUT;
			host->sw_timeout++;
			msdc_dump_info(host->id);
			msdc_reset_hw(host->id);
			goto out;
		}
	}

#ifdef MTK_MSDC_ERROR_TUNE_DEBUG
	msdc_error_tune_debug1(host, cmd, sbc, &intsts);
#endif

	/* command interrupts */
	if  (!(intsts & cmdsts))
		goto out;

#ifdef MTK_MSDC_USE_CMD23
	if (intsts & (MSDC_INT_CMDRDY | MSDC_INT_ACMD19_DONE)) {
#else
	if (intsts & (MSDC_INT_CMDRDY | MSDC_INT_ACMD19_DONE
		| MSDC_INT_ACMDRDY)) {
#endif
		u32 *rsp = NULL;

		rsp = &cmd->resp[0];
		switch (host->cmd_rsp) {
		case RESP_NONE:
			break;
		case RESP_R2:
			*rsp++ = MSDC_READ32(SDC_RESP3);
			*rsp++ = MSDC_READ32(SDC_RESP2);
			*rsp++ = MSDC_READ32(SDC_RESP1);
			*rsp++ = MSDC_READ32(SDC_RESP0);
			break;
		default: /* Response types 1, 3, 4, 5, 6, 7(1b) */
			*rsp = MSDC_READ32(SDC_RESP0);
			if ((cmd->opcode == 13) || (cmd->opcode == 25)) {
				/* Only print msg on this error */
				if (*rsp & R1_WP_VIOLATION) {
					pr_err("[%s]: msdc%d XXX CMD<%d> resp<0x%.8x>, write protection violation\n",
						__func__, host->id, cmd->opcode,
						*rsp);
				}

				if ((*rsp & R1_OUT_OF_RANGE)
					 && (host->hw->host_function != MSDC_SDIO)) {
					pr_err("[%s]: msdc%d XXX CMD<%d> resp<0x%.8x>,bit31=1,force make crc error\n",
						__func__, host->id, cmd->opcode,
						*rsp);
					/*cmd->error = (unsigned int)-EILSEQ;*/
				}
			}
			break;
		}
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		dbg_add_host_log(host->mmc, 1, cmd->opcode, cmd->resp[0]);
#endif
	} else if (intsts & MSDC_INT_RSPCRCERR) {
		cmd->error = (unsigned int)-EILSEQ;
		if ((cmd->opcode != 19) && (cmd->opcode != 21))
			pr_err("[%s]: msdc%d CMD<%d> MSDC_INT_RSPCRCERR Arg<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
		if (((MMC_RSP_R1B == mmc_resp_type(cmd)) || (cmd->opcode == 13))
			&& (host->hw->host_function != MSDC_SDIO)) {
			pr_err("[%s]: msdc%d CMD<%d> ARG<0x%.8X> is R1B, CRC not reset hw\n",
				__func__, host->id, cmd->opcode, cmd->arg);
		} else {
			msdc_reset_hw(host->id);
		}
	} else if (intsts & MSDC_INT_CMDTMO) {
		cmd->error = (unsigned int)-ETIMEDOUT;
		if ((cmd->opcode != 19) && (cmd->opcode != 21))
			pr_err("[%s]: msdc%d CMD<%d> MSDC_INT_CMDTMO Arg<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
		if ((cmd->opcode != 52) && (cmd->opcode != 8) &&
		    (cmd->opcode != 5) && (cmd->opcode != 55) &&
		    (cmd->opcode != 19) && (cmd->opcode != 21) &&
		    (cmd->opcode != 1) &&
		    ((cmd->opcode != 13) || (g_emmc_mode_switch == 0))) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
			mmc_cmd_dump(host->mmc);
#endif
			msdc_dump_info(host->id);
		}
		if (cmd->opcode == MMC_STOP_TRANSMISSION) {
			cmd->error = 0;
			pr_err("msdc%d: send stop TMO, device status: %x\n",
					host->id, host->device_status);
		}
		if ((cmd->opcode == 5) && emmc_do_sleep_awake)
			msdc_dump_info(host->id);

		if (((MMC_RSP_R1B == mmc_resp_type(cmd)) || (cmd->opcode == 13))
			&& (host->hw->host_function != MSDC_SDIO)) {
			pr_err("[%s]: msdc%d XXX CMD<%d> ARG<0x%.8X> is R1B, TMO not reset hw\n",
				__func__, host->id, cmd->opcode, cmd->arg);
		} else {
			msdc_reset_hw(host->id);
		}
	}
#ifdef MTK_MSDC_USE_CMD23
	if ((sbc != NULL) && (host->autocmd & MSDC_AUTOCMD23)) {
		if (intsts & MSDC_INT_ACMDRDY) {
			u32 *arsp = &sbc->resp[0];
			*arsp = MSDC_READ32(SDC_ACMD_RESP);
		} else if (intsts & MSDC_INT_ACMDCRCERR) {
			pr_err("[%s]: msdc%d, autocmd23 crc error\n",
				__func__, host->id);
			sbc->error = (unsigned int)-EILSEQ;
			/* record the error info in current cmd struct */
			cmd->error = (unsigned int)-EILSEQ;
			/* host->error |= REQ_CMD23_EIO; */
			msdc_reset_hw(host->id);
		} else if (intsts & MSDC_INT_ACMDTMO) {
			pr_err("[%s]: msdc%d, autocmd23 tmo error\n",
				__func__, host->id);
			sbc->error = (unsigned int)-ETIMEDOUT;
			/* record the error info in current cmd struct */
			cmd->error = (unsigned int)-ETIMEDOUT;
			msdc_dump_info(host->id);
			/* host->error |= REQ_CMD23_TMO; */
			msdc_reset_hw(host->id);
		}
	}
#endif /* end of MTK_MSDC_USE_CMD23 */

out:
	host->cmd = NULL;

	if (!cmd->data && !cmd->error)
		host->prev_cmd_cause_dump = 0;

	return cmd->error;
}

unsigned int msdc_do_command(struct msdc_host *host,
	struct mmc_command *cmd,
	int                 tune,
	unsigned long       timeout)
{
	MVG_EMMC_DECLARE_INT32(delay_ns);
	MVG_EMMC_DECLARE_INT32(delay_us);
	MVG_EMMC_DECLARE_INT32(delay_ms);

	if ((cmd->opcode == MMC_GO_IDLE_STATE) &&
	    (host->hw->host_function == MSDC_SD)) {
		mdelay(10);
	}

	MVG_EMMC_ERASE_MATCH(host, (u64)cmd->arg, delay_ms, delay_us,
		delay_ns, cmd->opcode);

	if (msdc_command_start(host, cmd, tune, timeout))
		goto end;

	MVG_EMMC_ERASE_RESET(delay_ms, delay_us, cmd->opcode);

	if (msdc_command_resp_polling(host, cmd, tune, timeout))
		goto end;

end:
	if (cmd->opcode == MMC_SEND_STATUS && cmd->error == 0)
		host->device_status = cmd->resp[0];

	N_MSG(CMD, "        return<%d> resp<0x%.8x>", cmd->error, cmd->resp[0]);

	return cmd->error;
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static unsigned int msdc_cmdq_command_start(struct msdc_host *host,
	struct mmc_command *cmd,
	int tune,
	unsigned long timeout)
{
	void __iomem *base = host->base;
	u32 opcode = cmd->opcode;
	u32 rawarg;
	u32 resp;
	unsigned long tmo;
	u32 wints_cq_cmd = MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO;

	switch (opcode) {
	case MMC_SET_QUEUE_CONTEXT:
	case MMC_QUEUE_READ_ADDRESS:
	case MMC_SEND_STATUS:
		break;
	default:
		pr_err("[%s]: ERROR, only CMD44/CMD45/CMD13 can issue\n",
			__func__);
		break;
	}

	resp = RESP_R1;
	cmd->error = 0;

	N_MSG(CMD, "CMD<%d>        Arg<0x%.8x>", opcode, cmd->arg);

	tmo = jiffies + timeout;

	for (;;) {
		if (!sdc_is_cmd_busy())
			break;

		if (time_after(jiffies, tmo)) {
			ERR_MSG("[%s]: XXX cmd_busy timeout: before CMD<%d>",
				__func__ , opcode);
			cmd->error = (unsigned int)-ETIMEDOUT;
			msdc_reset_hw(host->id);
			return cmd->error;
		}
	}

	host->cmd	  = cmd;
	host->cmd_rsp = resp;

	/* use polling way */
	MSDC_CLR_BIT32(MSDC_INTEN, wints_cq_cmd);
	rawarg = cmd->arg;

	dbg_add_host_log(host->mmc, 0, cmd->opcode, cmd->arg);
	sdc_send_cmdq_cmd(opcode, rawarg);

	return 0;
}

static unsigned int msdc_cmdq_command_resp_polling(struct msdc_host *host,
	struct mmc_command *cmd,
	int tune,
	unsigned long timeout)
{
	void __iomem *base = host->base;
	u32 intsts;
	u32 resp;
	unsigned long tmo;
	u32 cmdsts = MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO;

	resp = host->cmd_rsp;

	/* polling */
	tmo = jiffies + timeout;
	while (1) {
		intsts = MSDC_READ32(MSDC_INT);
		if ((intsts & cmdsts) != 0) {
			/* clear all int flag */
			intsts &= cmdsts;
			MSDC_WRITE32(MSDC_INT, intsts);
			break;
		}

		if (time_after(jiffies, tmo)) {
			pr_err("[%s]: msdc%d CMD<%d> polling_for_completion timeout ARG<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
			cmd->error = (unsigned int)-ETIMEDOUT;
			host->sw_timeout++;
			msdc_dump_info(host->id);
			goto out;
		}
	}

	/* command interrupts */
	if (intsts & cmdsts) {
		if (intsts & MSDC_INT_CMDRDY) {
			u32 *rsp = NULL;

			rsp = &cmd->resp[0];
			switch (host->cmd_rsp) {
			case RESP_NONE:
				break;
			case RESP_R2:
				*rsp++ = MSDC_READ32(SDC_RESP3);
				*rsp++ = MSDC_READ32(SDC_RESP2);
				*rsp++ = MSDC_READ32(SDC_RESP1);
				*rsp++ = MSDC_READ32(SDC_RESP0);
				break;
			default: /* Response types 1, 3, 4, 5, 6, 7(1b) */
				*rsp = MSDC_READ32(SDC_RESP0);
				break;
			}
			dbg_add_host_log(host->mmc, 1, cmd->opcode, cmd->resp[0]);
		} else if (intsts & MSDC_INT_RSPCRCERR) {
			cmd->error = (unsigned int)-EILSEQ;
			pr_err("[%s]: msdc%d XXX CMD<%d> MSDC_INT_RSPCRCERR Arg<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
			msdc_dump_info(host->id);
		} else if (intsts & MSDC_INT_CMDTMO) {
			cmd->error = (unsigned int)-ETIMEDOUT;
			pr_err("[%s]: msdc%d XXX CMD<%d> MSDC_INT_CMDTMO Arg<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
			mmc_cmd_dump(host->mmc);
			msdc_dump_info(host->id);
		}
	}
out:
	host->cmd = NULL;
	MSDC_SET_FIELD(EMMC51_CFG0, MSDC_EMMC51_CFG_CMDQEN, (0));

	if (!cmd->data && !cmd->error)
		host->prev_cmd_cause_dump = 0;

	return cmd->error;
}

/* do command queue command
	- CMD44, CMD45, CMD13 - QSR
	Use another register set */
unsigned int msdc_do_cmdq_command(struct msdc_host *host,
	struct mmc_command *cmd,
	int tune,
	unsigned long timeout)
{
	if (msdc_cmdq_command_start(host, cmd, tune, timeout))
		goto end;

	if (msdc_cmdq_command_resp_polling(host, cmd, tune, timeout))
		goto end;
end:
	N_MSG(CMD, "		return<%d> resp<0x%.8x>", cmd->error, cmd->resp[0]);
	return cmd->error;
}
#endif

/* For DMA retry check */
#ifdef MTK_MSDC_DMA_RETRY
#ifdef MTK_MSDC_FIX_VCORE_WHEN_FAIL
static int msdc_wr_do_fix_vcore(void)
{
	static int b_do_fix_vcore = -1;

	if (b_do_fix_vcore != -1)
		return b_do_fix_vcore;

	b_do_fix_vcore = 0;

#if defined(CONFIG_ARM64)
	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_64_om_lwctg_lp", 20) == 0)
		b_do_fix_vcore = 1;

	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_64_op01_lwctg_lp", 22) == 0)
		b_do_fix_vcore = 1;

#elif defined(CONFIG_ARM)
	if (strncmp(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_om_lwctg_lp", 17) == 0)
		b_do_fix_vcore = 1;

	if (strncmp(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_op01_lwctg_lp", 19) == 0)
		b_do_fix_vcore = 1;
#endif
	return b_do_fix_vcore;
}
#endif /*endif of MTK_MSDC_FIX_VCORE_WHEN_FAIL*/

static int msdc_wr_retry_cnt(struct msdc_host *host)
{
	static int retry_cnt = -1;

	if (retry_cnt != -1)
		return retry_cnt;

	retry_cnt = MTK_MAX_RETRY_CNT;

#if defined(CONFIG_ARM64)
	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_64_om_lwctg_lp", 20) == 0)
		retry_cnt = MTK_MAX_RETRY_CNT_LP;

	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_64_op01_lwctg_lp", 22) == 0)
		retry_cnt = MTK_MAX_RETRY_CNT_LP;

#elif defined(CONFIG_ARM)
	if (strncmp(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_om_lwctg_lp", 17) == 0)
		retry_cnt = MTK_MAX_RETRY_CNT_LP;

	if (strncmp(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_op01_lwctg_lp", 19) == 0)
		retry_cnt = MTK_MAX_RETRY_CNT_LP;
#endif
	host->dma_max_retry_cnt = retry_cnt;

	return retry_cnt;
}

static int msdc_wr_do_retry(struct msdc_host *host)
{
	static int b_do_retry = -1;

	if (!host->dma_retry) {
		/* 0: default, 1: disable, 2:enable */
		if (host->dma_retry_force_dis == 1)
			return 0;
		else if (host->dma_retry_force_dis == 2)
			return 1;
	}

	if (b_do_retry != -1)
		return b_do_retry;

#ifdef MTK_MSDC_DMA_RETRY_ALL_FLAVOR
	b_do_retry = 1;
#else
	b_do_retry = 0;
#if defined(CONFIG_ARM64)
	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_64_om_lwctg_lp", 20) == 0)
		b_do_retry = 1;

	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_64_op01_lwctg_lp", 22) == 0)
		b_do_retry = 1;

#elif defined(CONFIG_ARM)
	if (strncmp(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_om_lwctg_lp", 17) == 0)
		b_do_retry = 1;

	if (strncmp(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES,
		"k57v1_op01_lwctg_lp", 19) == 0)
		b_do_retry = 1;
#endif
#endif /*endif of MTK_MSDC_DMA_RETRY_ALL_FLAVOR*/

	if (host->chip_hw_ver >= 0xCA01)
		b_do_retry = 0;

	host->dma_retry_en = b_do_retry;

	return b_do_retry;
}

static void msdc_wr_dump_status(struct msdc_host *host)
{
	int i, retry;

	retry = host->dma_retry;
	for (i = 0; i <= retry; i++) {
		pr_err("msdc%d checksum (%d) 0x%x\n",
			host->id, i, host->dma_checksum[i]);
	}
}

static int msdc_wr_check_result(struct msdc_host *host)
{
	int i, j, retry, max;

	retry = host->dma_retry;
	max = msdc_wr_retry_cnt(host);

	if (retry == max)
		return 0; /*PASS for last req*/

	for (i = 0; i < retry; i++) {
		for (j = i + 1; j <= retry; j++) {
			if (host->dma_checksum[i] ==
				host->dma_checksum[j])
				return 0; /*PASS*/
		}
	}

	if (retry == (max - 1)) {
		msdc_wr_dump_status(host);
		return 2; /*FAIL*/
	}

	return 1; /*RETRY*/
}

static void msdc_wr_cal_chksum(void *data, u32 len, u32 *result)
{
	u32 sum = 0, i;
	u8 *u8ptr = (u8 *)data;

	for (i = 0; i < len; i++) {
		sum += *u8ptr;
		u8ptr++;
	}
	*result += sum;
}

static void msdc_wr_read_data(struct msdc_host *host, u32 *checksum)
{
	struct mmc_data *data = host->data;
	struct scatterlist *sg;
	u32 num;
	u32 *ptr;
	u32 left = 0;
	u32 size = 0, result = 0;
	struct page *hmpage = NULL;
	int i = 0, totalpages = 0;
	int flag, subpage = 0;
	ulong kaddr[DIV_ROUND_UP(MAX_SGMT_SZ, PAGE_SIZE)];

	sg = data->sg;
	num = data->sg_len;

	while (sg) {
		if ((num == 0) && (left == 0))
			break;

		left = msdc_sg_len(sg, host->dma_xfer);
		ptr = sg_virt(sg);

		flag = 0;

		/* High memory must kmap, if already mapped,
		   only add counter */
		if	((ptr != NULL) &&
			 !(PageHighMem((struct page *)(sg->page_link & ~0x3))))
			goto check_1;

		hmpage = (struct page *)(sg->page_link & ~0x3);
		totalpages = DIV_ROUND_UP(left + sg->offset, PAGE_SIZE);
		subpage = (left + sg->offset) % PAGE_SIZE;

		if ((subpage != 0) || (sg->offset != 0))
			N_MSG(OPS, "msdc%d: size or start not align %x, %x, hmpage %lx,sg offset %x\n",
				host->id, subpage, left, (ulong)hmpage,
				sg->offset);

		/* Kmap all need pages, */
		for (i = 0; i < totalpages; i++) {
			kaddr[i] = (ulong) kmap_atomic(hmpage + i);
			if ((i > 0) && ((kaddr[i] - kaddr[i - 1]) != PAGE_SIZE))
				flag = 1; /* kmap not continuous */
			if (!kaddr[i])
				pr_err("msdc0:kmap failed %lx (%d)\n", kaddr[i], i);
		}

		if (flag == 0)
			goto check_1; /* kmap continuous */

		/* High memory and more than 1 va address va
		   may be not continuous */
		/*pr_err(ERR "msdc0:w kmap not continuous %x %x %x\n",
			left, kaddr[i], kaddr[i-1]);*/
		for (i = 0; i < totalpages; i++) {
			left = PAGE_SIZE;
			ptr = (u32 *) kaddr[i];

			if (i == 0) {
				left = PAGE_SIZE - sg->offset;
				ptr = (u32 *) (kaddr[i] + sg->offset);
			}
			if (subpage != 0 && (i == (totalpages - 1)))
				left = subpage;
check_1:
			if ((flag == 1) && (left == 0))
				continue;
			else if ((flag == 0) && (left == 0))
				goto check_end;

			if (!ptr)
				goto check_end;

			msdc_wr_cal_chksum(ptr, left, &result);
			left = 0;

			goto check_1;
		}

check_end:
		if (hmpage != NULL) {
			for (i = 0; i < totalpages; i++)
				/*kunmap_atomic should pass address*/
				/*kunmap(hmpage + i);*/
				kunmap_atomic((void *)kaddr[i]);

			hmpage = NULL;
		}
		size += msdc_sg_len(sg, host->dma_xfer);
		sg = sg_next(sg);
		num--;
	}

	*checksum = result;
}

void msdc_wr_check(struct msdc_host *host)
{
	struct mmc_host *mmc = host->mmc;
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *data = host->data;
	struct _mmc_blk_data *main_md;
	int i, ret, dir = DMA_FROM_DEVICE;
	u32 checksum;
	char str[128];

	if (!msdc_wr_do_retry(host))
		return;

	if (host->id != 0)
		return;

	if (!host->dma_xfer)
		return;

	if (!mrq || !data)
		return;

	if (!check_mmc_cmd1718(mrq->cmd->opcode))
		return;

	if (mmc->card) {
		main_md = mmc_get_drvdata(mmc->card);
		if (main_md->part_curr == 0x3) /*bypass rpmb*/
			return;
	}

	dir = data->flags & MMC_DATA_READ ?
		DMA_FROM_DEVICE : DMA_TO_DEVICE;
	dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len, dir);

	host->dma_sg_unmaped = 1;

	msdc_wr_read_data(host, &checksum);

	i = host->dma_retry;
	host->dma_checksum[i] = checksum;

	ret = msdc_wr_check_result(host);
	host->dma_retry++;
	switch (ret) {
	case 1: /*RETRY*/
		mrq->cmd->error = (unsigned int)-EILSEQ;
		break;
	case 2: /*FAIL*/
		host->dma_chksum_fail_cnt++;
		snprintf(str, sizeof(str),
			"msdc0 dma checksum fail!!! 0x%x\n",
			host->dma_checksum[i]);
		aee_kernel_warning_api(__FILE__, __LINE__,
				DB_OPT_DEFAULT, str, "msdc0 dma sram warning!");
#ifdef MTK_MSDC_FIX_VCORE_WHEN_FAIL
		/* fix hpm once compare fail */
		if (msdc_wr_do_fix_vcore()) {
			vcorefs_fix_hpm_req();
			pr_err("msdc0 dma checksum fail fix vcore hpm\n");
			host->fix_vcore = 1;
			mrq->cmd->error = (unsigned int)-EILSEQ;
			break;
		}
#endif
		/*fall through*/
	case 0: /*PASS*/
	default:
		host->dma_retry_stat[i]++;
		host->dma_retry = 0;
		break;
	}
}
#endif

/* The abort condition when PIO read/write
   tmo:
*/
static int msdc_pio_abort(struct msdc_host *host, struct mmc_data *data,
	unsigned long tmo)
{
	int  ret = 0;
	void __iomem *base = host->base;

	if (atomic_read(&host->abort))
		ret = 1;

	if (time_after(jiffies, tmo)) {
		data->error = (unsigned int)-ETIMEDOUT;
		ERR_MSG("XXX PIO Data Timeout: CMD<%d>",
			host->mrq->cmd->opcode);
		msdc_dump_info(host->id);
		ret = 1;
	}

	if (ret) {
		msdc_reset_hw(host->id);
		ERR_MSG("msdc pio find abort");
	}

	return ret;
}

/*
   Need to add a timeout, or WDT timeout, system reboot.
*/
/* pio mode data read/write */
int msdc_pio_read(struct msdc_host *host, struct mmc_data *data)
{
	struct scatterlist *sg = data->sg;
	void __iomem *base = host->base;
	u32 num = data->sg_len;
	u32 *ptr;
	u8 *u8ptr;
	u32 left = 0;
	u32 count, size = 0;
	u32 wints = MSDC_INTEN_DATTMO | MSDC_INTEN_DATCRCERR
		| MSDC_INTEN_XFER_COMPL;
	u32 ints = 0;
	bool get_xfer_done = 0;
	unsigned long tmo = jiffies + DAT_TIMEOUT;
	struct page *hmpage = NULL;
	int i = 0, subpage = 0, totalpages = 0;
	int flag = 0;
	ulong kaddr[DIV_ROUND_UP(MAX_SGMT_SZ, PAGE_SIZE)];

	BUG_ON(sg == NULL);
	/*MSDC_CLR_BIT32(MSDC_INTEN, wints);*/
	while (1) {
		if (!get_xfer_done) {
			ints = MSDC_READ32(MSDC_INT);
			latest_int_status[host->id] = ints;
			ints &= wints;
			MSDC_WRITE32(MSDC_INT, ints);
		}
		if (ints & MSDC_INT_DATTMO) {
			data->error = (unsigned int)-ETIMEDOUT;
			msdc_dump_info(host->id);
			msdc_reset_hw(host->id);
			break;
		} else if (ints & MSDC_INT_DATCRCERR) {
			data->error = (unsigned int)-EILSEQ;
			/* msdc_dump_info(host->id); */
			msdc_reset_hw(host->id);
			/* left = msdc_sg_len(sg, host->dma_xfer); */
			/* ptr = sg_virt(sg); */
			break;
		} else if (ints & MSDC_INT_XFER_COMPL) {
			get_xfer_done = 1;
		}
		if (get_xfer_done && (num == 0) && (left == 0))
			break;
		if (msdc_pio_abort(host, data, tmo))
			goto end;
		if ((num == 0) && (left == 0))
			continue;
		left = msdc_sg_len(sg, host->dma_xfer);
		ptr = sg_virt(sg);
		flag = 0;

		if  ((ptr != NULL) &&
		     !(PageHighMem((struct page *)(sg->page_link & ~0x3))))
			goto check_fifo1;

		hmpage = (struct page *)(sg->page_link & ~0x3);
		totalpages = DIV_ROUND_UP((left + sg->offset), PAGE_SIZE);
		subpage = (left + sg->offset) % PAGE_SIZE;

		if (subpage != 0 || (sg->offset != 0))
			N_MSG(OPS, "msdc%d: read size or start not align %x, %x, hmpage %lx,sg offset %x\n",
				host->id, subpage, left, (ulong)hmpage,
				sg->offset);

		for (i = 0; i < totalpages; i++) {
			kaddr[i] = (ulong) kmap(hmpage + i);
			if ((i > 0) && ((kaddr[i] - kaddr[i - 1]) != PAGE_SIZE))
				flag = 1;
			if (!kaddr[i])
				ERR_MSG("msdc0:kmap failed %lx", kaddr[i]);
		}

		ptr = sg_virt(sg);

		if (ptr == NULL)
			ERR_MSG("msdc0:sg_virt %p", ptr);

		if (flag == 0)
			goto check_fifo1;

		/* High memory and more than 1 va address va
		   and not continuous */
		/* pr_err("msdc0: kmap not continuous %x %x %x\n",
			left,kaddr[i],kaddr[i-1]); */
		for (i = 0; i < totalpages; i++) {
			left = PAGE_SIZE;
			ptr = (u32 *) kaddr[i];

			if (i == 0) {
				left = PAGE_SIZE - sg->offset;
				ptr = (u32 *) (kaddr[i] + sg->offset);
			}
			if ((subpage != 0) && (i == (totalpages-1)))
				left = subpage;

check_fifo1:
			if ((flag == 1) && (left == 0))
				continue;
			else if ((flag == 0) && (left == 0))
				goto check_fifo_end;

			if ((msdc_rxfifocnt() >= MSDC_FIFO_THD) &&
			    (left >= MSDC_FIFO_THD)) {
				count = MSDC_FIFO_THD >> 2;
				do {
#ifdef MTK_MSDC_DUMP_FIFO
					pr_debug("0x%x ", msdc_fifo_read32());
#else
					*ptr++ = msdc_fifo_read32();
#endif
				} while (--count);
				left -= MSDC_FIFO_THD;
			} else if ((left < MSDC_FIFO_THD) &&
				    msdc_rxfifocnt() >= left) {
				while (left > 3) {
#ifdef MTK_MSDC_DUMP_FIFO
					pr_debug("0x%x ", msdc_fifo_read32());
#else
					*ptr++ = msdc_fifo_read32();
#endif
					left -= 4;
				}

				u8ptr = (u8 *) ptr;
				while (left) {
#ifdef MTK_MSDC_DUMP_FIFO
					pr_debug("0x%x ", msdc_fifo_read8());
#else
					*u8ptr++ = msdc_fifo_read8();
#endif
					left--;
				}
			} else {
				ints = MSDC_READ32(MSDC_INT);
				latest_int_status[host->id] = ints;

				if (ints & MSDC_INT_DATCRCERR) {
					ERR_MSG("[msdc%d] DAT CRC error (0x%x), Left DAT: %d bytes\n",
						host->id, ints, left);
					data->error = (unsigned int)-EILSEQ;
				} else if (ints & MSDC_INT_DATTMO) {
					ERR_MSG("[msdc%d] DAT TMO error (0x%x), Left DAT: %d bytes\n",
						host->id, ints, left);
					data->error = (unsigned int)-ETIMEDOUT;
				} else {
					goto skip_msdc_dump_and_reset1;
				}

				if (ints & MSDC_INT_DATTMO)
					msdc_dump_info(host->id);

				MSDC_WRITE32(MSDC_INT, ints);
				msdc_reset_hw(host->id);
				goto end;
			}

skip_msdc_dump_and_reset1:
			if (msdc_pio_abort(host, data, tmo))
				goto end;

			goto check_fifo1;
		}

check_fifo_end:
		if (hmpage != NULL) {
			/* pr_err("read msdc0:unmap %x\n", hmpage); */
			for (i = 0; i < totalpages; i++)
				kunmap(hmpage + i);

			hmpage = NULL;
		}
		size += msdc_sg_len(sg, host->dma_xfer);
		sg = sg_next(sg);
		num--;
	}
 end:
	if (hmpage != NULL) {
		for (i = 0; i < totalpages; i++)
			kunmap(hmpage + i);
		/* pr_err("msdc0 read unmap:\n"); */
	}
	data->bytes_xfered += size;
	N_MSG(FIO, "        PIO Read<%d>bytes", size);

	if (data->error)
		ERR_MSG("read pio data->error<%d> left<%d> size<%d>",
			data->error, left, size);

	if (!data->error)
		host->prev_cmd_cause_dump = 0;

	return data->error;
}

/* please make sure won't using PIO when size >= 512
   which means, memory card block read/write won't using pio
   then don't need to handle the CMD12 when data error.
*/
int msdc_pio_write(struct msdc_host *host, struct mmc_data *data)
{
	void __iomem *base = host->base;
	struct scatterlist *sg = data->sg;
	u32 num = data->sg_len;
	u32 *ptr;
	u8 *u8ptr;
	u32 left = 0;
	u32 count, size = 0;
	u32 wints = MSDC_INTEN_DATTMO | MSDC_INTEN_DATCRCERR
		| MSDC_INTEN_XFER_COMPL;
	bool get_xfer_done = 0;
	unsigned long tmo = jiffies + DAT_TIMEOUT;
	u32 ints = 0;
	struct page *hmpage = NULL;
	int i = 0, totalpages = 0;
	int flag, subpage = 0;
	ulong kaddr[DIV_ROUND_UP(MAX_SGMT_SZ, PAGE_SIZE)];

	/* MSDC_CLR_BIT32(MSDC_INTEN, wints); */
	while (1) {
		if (!get_xfer_done) {
			ints = MSDC_READ32(MSDC_INT);
			latest_int_status[host->id] = ints;
			ints &= wints;
			MSDC_WRITE32(MSDC_INT, ints);
		}
		if (ints & MSDC_INT_DATTMO) {
			data->error = (unsigned int)-ETIMEDOUT;
			msdc_dump_info(host->id);
			msdc_reset_hw(host->id);
			break;
		} else if (ints & MSDC_INT_DATCRCERR) {
			data->error = (unsigned int)-EILSEQ;
			/* msdc_dump_info(host->id); */
			msdc_reset_hw(host->id);
			break;
		} else if (ints & MSDC_INT_XFER_COMPL) {
			get_xfer_done = 1;
		}
		if ((get_xfer_done == 1) && (num == 0) && (left == 0))
			break;
		if (msdc_pio_abort(host, data, tmo))
			goto end;
		if ((num == 0) && (left == 0))
			continue;
		left = msdc_sg_len(sg, host->dma_xfer);
		ptr = sg_virt(sg);

		flag = 0;

		/* High memory must kmap, if already mapped,
		   only add counter */
		if  ((ptr != NULL) &&
		     !(PageHighMem((struct page *)(sg->page_link & ~0x3))))
			goto check_fifo1;

		hmpage = (struct page *)(sg->page_link & ~0x3);
		totalpages = DIV_ROUND_UP(left + sg->offset, PAGE_SIZE);
		subpage = (left + sg->offset) % PAGE_SIZE;

		if ((subpage != 0) || (sg->offset != 0))
			N_MSG(OPS, "msdc%d: write size or start not align %x, %x, hmpage %lx,sg offset %x\n",
				host->id, subpage, left, (ulong)hmpage,
				sg->offset);

		/* Kmap all need pages, */
		for (i = 0; i < totalpages; i++) {
			kaddr[i] = (ulong) kmap(hmpage + i);
			if ((i > 0) && ((kaddr[i] - kaddr[i - 1]) != PAGE_SIZE))
				flag = 1;
			if (!kaddr[i])
				ERR_MSG("msdc0:kmap failed %lx\n", kaddr[i]);
		}

		ptr = sg_virt(sg);

		if (ptr == NULL)
			ERR_MSG("msdc0:write sg_virt %p\n", ptr);

		if (flag == 0)
			goto check_fifo1;

		/* High memory and more than 1 va address va
		   may be not continuous */
		/*pr_err(ERR "msdc0:w kmap not continuous %x %x %x\n",
			left, kaddr[i], kaddr[i-1]);*/
		for (i = 0; i < totalpages; i++) {
			left = PAGE_SIZE;
			ptr = (u32 *) kaddr[i];

			if (i == 0) {
				left = PAGE_SIZE - sg->offset;
				ptr = (u32 *) (kaddr[i] + sg->offset);
			}
			if (subpage != 0 && (i == (totalpages - 1)))
				left = subpage;

check_fifo1:
			if ((flag == 1) && (left == 0))
				continue;
			else if ((flag == 0) && (left == 0))
				goto check_fifo_end;

			if (left >= MSDC_FIFO_SZ && msdc_txfifocnt() == 0) {
				count = MSDC_FIFO_SZ >> 2;
				do {
					msdc_fifo_write32(*ptr);
					ptr++;
				} while (--count);
				left -= MSDC_FIFO_SZ;
			} else if (left < MSDC_FIFO_SZ &&
				   msdc_txfifocnt() == 0) {
				while (left > 3) {
					msdc_fifo_write32(*ptr);
					ptr++;
					left -= 4;
				}
				u8ptr = (u8 *) ptr;
				while (left) {
					msdc_fifo_write8(*u8ptr);
					u8ptr++;
					left--;
				}
			} else {
				ints = MSDC_READ32(MSDC_INT);
				latest_int_status[host->id] = ints;

				if (ints & MSDC_INT_DATCRCERR) {
					ERR_MSG("[msdc%d] DAT CRC error (0x%x), Left DAT: %d bytes\n",
						host->id, ints, left);
					data->error = (unsigned int)-EILSEQ;
				} else if (ints & MSDC_INT_DATTMO) {
					ERR_MSG("[msdc%d] DAT TMO error (0x%x), Left DAT: %d bytes\n",
						host->id, ints, left);
					data->error = (unsigned int)-ETIMEDOUT;
				} else {
					goto skip_msdc_dump_and_reset1;
				}

				msdc_dump_info(host->id);
				MSDC_WRITE32(MSDC_INT, ints);
				msdc_reset_hw(host->id);
				goto end;
			}

skip_msdc_dump_and_reset1:
			if (msdc_pio_abort(host, data, tmo))
				goto end;

			goto check_fifo1;
		}

check_fifo_end:
		if (hmpage != NULL) {
			for (i = 0; i < totalpages; i++)
				kunmap(hmpage + i);

			hmpage = NULL;

		}
		size += msdc_sg_len(sg, host->dma_xfer);
		sg = sg_next(sg);
		num--;
	}
 end:
	if (hmpage != NULL) {
		for (i = 0; i < totalpages; i++)
			kunmap(hmpage + i);
		pr_err("msdc0 write unmap 0x%x:\n", left);
	}
	data->bytes_xfered += size;
	N_MSG(FIO, "        PIO Write<%d>bytes", size);

	if (data->error)
		ERR_MSG("write pio data->error<%d> left<%d> size<%d>",
			data->error, left, size);

	if (!data->error)
		host->prev_cmd_cause_dump = 0;

	/*MSDC_CLR_BIT32(MSDC_INTEN, wints);*/

	return data->error;
}

static void msdc_dma_start(struct msdc_host *host)
{
	void __iomem *base = host->base;
	u32 wints = MSDC_INTEN_XFER_COMPL | MSDC_INTEN_DATTMO
		| MSDC_INTEN_DATCRCERR;

	if (host->autocmd & MSDC_AUTOCMD12)
		wints |= MSDC_INT_ACMDCRCERR | MSDC_INT_ACMDTMO
			| MSDC_INT_ACMDRDY;
	MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_START, 1);

	MSDC_SET_BIT32(MSDC_INTEN, wints);

	N_MSG(DMA, "DMA start");
	/* Schedule delayed work to check if data0 keeps busy */
	if (host->data && host->data->flags & MMC_DATA_WRITE) {
		if (host->hw->host_function == MSDC_EMMC)
			/* insure eMMC write time */
			host->write_timeout_ms = 20 * 1000;
		else
			host->write_timeout_ms = min_t(u32, max_t(u32,
				host->data->blocks * 500,
				host->data->timeout_ns / 1000000), 10 * 1000);
		schedule_delayed_work(&host->write_timeout,
			msecs_to_jiffies(host->write_timeout_ms));
		N_MSG(DMA, "DMA Data Busy Timeout:%u ms, schedule_delayed_work",
			host->write_timeout_ms);
	}
#ifdef MTK_MSDC_DUMP_STATUS_DEBUG
	get_monotonic_boottime(&host->start_dma_time[host->id]);
#endif
}

static void msdc_dma_stop(struct msdc_host *host)
{
	void __iomem *base = host->base;
	int retry = 500;
	int count = 1000;
	u32 wints = MSDC_INTEN_XFER_COMPL | MSDC_INTEN_DATTMO
		| MSDC_INTEN_DATCRCERR;

	/* Clear DMA data busy timeout */
	if (host->data && (host->data->flags & MMC_DATA_WRITE)) {
		cancel_delayed_work(&host->write_timeout);
		N_MSG(DMA, "DMA Data Busy Timeout:%u ms, cancel_delayed_work",
			host->write_timeout_ms);
		host->write_timeout_ms = 0; /* clear timeout */
	}

	/* handle autocmd12 error in msdc_irq */
	if (host->autocmd & MSDC_AUTOCMD12)
		wints |= MSDC_INT_ACMDCRCERR | MSDC_INT_ACMDTMO
			| MSDC_INT_ACMDRDY;
	N_MSG(DMA, "DMA status: 0x%.8x", MSDC_READ32(MSDC_DMA_CFG));

	MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_STOP, 1);
	msdc_retry((MSDC_READ32(MSDC_DMA_CFG) & MSDC_DMA_CFG_STS), retry,
		count, host->id);
	if (retry == 0)
		ERR_MSG("DMA stop retry till count 0");

	MSDC_CLR_BIT32(MSDC_INTEN, wints); /* Not just xfer_comp */

#ifdef MTK_MSDC_DUMP_STATUS_DEBUG
	get_monotonic_boottime(&host->stop_dma_time[host->id]);
#endif

	N_MSG(DMA, "DMA stop");
}

/* calc checksum */
static u8 msdc_dma_calcs(u8 *buf, u32 len)
{
	u32 i, sum = 0;

	for (i = 0; i < len; i++)
		sum += buf[i];
	return 0xFF - (u8) sum;
}

/* gpd bd setup + dma registers */
static int msdc_dma_config(struct msdc_host *host, struct msdc_dma *dma)
{
	void __iomem *base = host->base;
	u32 sglen = dma->sglen;
	u32 j, num, bdlen;
	dma_addr_t dma_address;
	u32 dma_len;
	u8  blkpad, dwpad, chksum;
	struct scatterlist *sg = dma->sg;
	struct gpd_t *gpd;
	struct bd_t *bd, vbd = {0};

	switch (dma->mode) {
	case MSDC_MODE_DMA_BASIC:
		if (host->hw->host_function == MSDC_SDIO)
			BUG_ON(dma->xfersz > 0xFFFFFFFF);
		else
			BUG_ON(dma->xfersz > 65535);

		BUG_ON(dma->sglen != 1);
		dma_address = sg_dma_address(sg);
		dma_len = msdc_sg_len(sg, host->dma_xfer);

		N_MSG(DMA, "BASIC DMA len<%x> dma_address<%llx>",
			dma_len, (u64)dma_address);

		MSDC_SET_FIELD(MSDC_DMA_SA_HIGH, MSDC_DMA_SURR_ADDR_HIGH4BIT,
			(u32) ((dma_address >> 32) & 0xF));
		MSDC_WRITE32(MSDC_DMA_SA, (u32) (dma_address & 0xFFFFFFFFUL));

		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_LASTBUF, 1);
		MSDC_WRITE32(MSDC_DMA_LEN, dma_len);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_BRUSTSZ,
			dma->burstsz);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_MODE, 0);
		break;
	case MSDC_MODE_DMA_DESC:
		blkpad = (dma->flags & DMA_FLAG_PAD_BLOCK) ? 1 : 0;
		dwpad  = (dma->flags & DMA_FLAG_PAD_DWORD) ? 1 : 0;
		chksum = (dma->flags & DMA_FLAG_EN_CHKSUM) ? 1 : 0;

		/* calculate the required number of gpd */
		num = (sglen + MAX_BD_PER_GPD - 1) / MAX_BD_PER_GPD;
		BUG_ON(num != 1);

		gpd = dma->gpd;
		bd  = dma->bd;
		bdlen = sglen;

		/* modify gpd */
		gpd->hwo = 1;   /* hw will clear it */
		gpd->bdp = 1;
		gpd->chksum = 0;        /* need to clear first. */
		gpd->chksum = (chksum ? msdc_dma_calcs((u8 *) gpd, 16) : 0);

		/* modify bd */
		for (j = 0; j < bdlen; j++) {
#ifdef MSDC_DMA_VIOLATION_DEBUG
			if (g_dma_debug[host->id] &&
			    (msdc_latest_op[host->id] == OPER_TYPE_READ)) {
				pr_debug("[%s] msdc%d do write 0x10000\n",
					__func__, host->id);
				dma_address = 0x10000;
			} else {
				dma_address = sg_dma_address(sg);
			}
#else
			dma_address = sg_dma_address(sg);
#endif

			dma_len = msdc_sg_len(sg, host->dma_xfer);

			N_MSG(DMA, "DESC DMA len<%x> dma_address<%llx>",
				dma_len, (u64)dma_address);

			memcpy(&vbd, &bd[j], sizeof(struct bd_t));

			msdc_init_bd(&vbd, blkpad, dwpad, dma_address,
				dma_len);

			if (j == bdlen - 1)
				vbd.eol = 1;  /* the last bd */
			else
				vbd.eol = 0;

			/* checksume need to clear first */
			vbd.chksum = 0;
			vbd.chksum = (chksum ?
				msdc_dma_calcs((u8 *) (&vbd), 16) : 0);

			memcpy(&bd[j], &vbd, sizeof(struct bd_t));

			sg++;
		}
#ifdef MSDC_DMA_VIOLATION_DEBUG
		if (g_dma_debug[host->id] &&
		    (msdc_latest_op[host->id] == OPER_TYPE_READ))
			g_dma_debug[host->id] = 0;
#endif

		dma->used_gpd += 2;
		dma->used_bd += bdlen;

		MSDC_SET_FIELD(MSDC_DMA_CFG, MSDC_DMA_CFG_DECSEN, chksum);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_BRUSTSZ,
			dma->burstsz);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_MODE, 1);

		MSDC_SET_FIELD(MSDC_DMA_SA_HIGH, MSDC_DMA_SURR_ADDR_HIGH4BIT,
			(u32) ((dma->gpd_addr >> 32) & 0xF));
		MSDC_WRITE32(MSDC_DMA_SA, (u32) (dma->gpd_addr & 0xFFFFFFFFUL));
		break;

	default:
		break;
	}

	N_MSG(DMA, "DMA_CTRL = 0x%x", MSDC_READ32(MSDC_DMA_CTRL));
	N_MSG(DMA, "DMA_CFG  = 0x%x", MSDC_READ32(MSDC_DMA_CFG));
	N_MSG(DMA, "DMA_SA_HIGH   = 0x%x", MSDC_READ32(MSDC_DMA_SA_HIGH));
	N_MSG(DMA, "DMA_SA   = 0x%x", MSDC_READ32(MSDC_DMA_SA));

	return 0;
}

static void msdc_dma_setup(struct msdc_host *host, struct msdc_dma *dma,
	struct scatterlist *sg, unsigned int sglen)
{
	u32 max_dma_len = 0;

	BUG_ON(sglen > MAX_BD_NUM);     /* not support currently */

	dma->sg = sg;
	dma->flags = DMA_FLAG_EN_CHKSUM;
	/* dma->flags = DMA_FLAG_NONE; */ /* CHECKME */
	dma->sglen = sglen;
	dma->xfersz = host->xfer_size;
	dma->burstsz = MSDC_BRUST_64B;

	if (host->hw->host_function == MSDC_SDIO)
		max_dma_len = MAX_DMA_CNT_SDIO;
	else
		max_dma_len = MAX_DMA_CNT;

	if (sglen == 1 &&
	     msdc_sg_len(sg, host->dma_xfer) <= max_dma_len)
		dma->mode = MSDC_MODE_DMA_BASIC;
	else
		dma->mode = MSDC_MODE_DMA_DESC;

	N_MSG(DMA, "DMA mode<%d> sglen<%d> xfersz<%d>", dma->mode, dma->sglen,
		dma->xfersz);

	msdc_dma_config(host, dma);
}

static void msdc_dma_clear(struct msdc_host *host)
{
	void __iomem *base = host->base;

	host->data = NULL;
	host->mrq = NULL;
	host->dma_xfer = 0;
	msdc_dma_off();
	host->dma.used_bd = 0;
	host->dma.used_gpd = 0;
	host->blksz = 0;
}

/* set block number before send command */
static void msdc_set_blknum(struct msdc_host *host, u32 blknum)
{
	void __iomem *base = host->base;

	MSDC_WRITE32(SDC_BLK_NUM, blknum);
}

static void msdc_log_cmd(struct msdc_host *host, struct mmc_command *cmd,
	struct mmc_data *data)
{
	N_MSG(OPS, "CMD<%d> data<%s %s> blksz<%d> block<%d> error<%d>",
		cmd->opcode, (host->dma_xfer ? "dma" : "pio"),
		((data->flags & MMC_DATA_READ) ? "read " : "write"),
		data->blksz, data->blocks, data->error);

	if (!(is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ))) {
		if (!check_mmc_cmd2425(cmd->opcode) &&
		    !check_mmc_cmd1718(cmd->opcode)) {
			N_MSG(NRW, "CMD<%3d> arg<0x%8x> Resp<0x%8x> data<%s> size<%d>",
				cmd->opcode, cmd->arg, cmd->resp[0],
				((data->flags & MMC_DATA_READ)
					? "read " : "write"),
				data->blksz * data->blocks);
		} else if (cmd->opcode != 13) { /* by pass CMD13 */
			N_MSG(NRW, "CMD<%3d> arg<0x%8x> resp<%8x %8x %8x %8x>",
				cmd->opcode, cmd->arg, cmd->resp[0],
				cmd->resp[1], cmd->resp[2], cmd->resp[3]);
		} else {
			N_MSG(RW, "CMD<%3d> arg<0x%8x> Resp<0x%8x> block<%d>",
				cmd->opcode, cmd->arg, cmd->resp[0],
				data->blocks);
		}
	}
}


int msdc_if_send_stop(struct msdc_host *host,
	struct mmc_command *cmd, struct mmc_data *data)
{
	if (!data || !data->stop)
		return 0;

	if ((cmd->error != 0)
	 || (data->error != 0)
	 || !(host->autocmd & MSDC_AUTOCMD12)
	 || !(check_mmc_cmd1825(cmd->opcode))) {
		if (msdc_do_command(host, data->stop, 0, CMD_TIMEOUT) != 0)
			return 1;
	}

	return 0;
}
int msdc_rw_cmd_dma(struct mmc_host *mmc, struct mmc_command *cmd,
	struct mmc_data *data, struct mmc_request *mrq, int tune)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	int map_sg = 0;
	int dir;

	msdc_dma_on();  /* enable DMA mode first!! */

	init_completion(&host->xfer_done);

	if (msdc_command_start(host, cmd, 0, CMD_TIMEOUT) != 0)
		return -1;

	if (tune == 0) {
		dir = data->flags & MMC_DATA_READ ?
			DMA_FROM_DEVICE : DMA_TO_DEVICE;
		(void)dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len, dir);
		map_sg = 1;
	}

	/* then wait command done */
	if (msdc_command_resp_polling(host, cmd, 0, CMD_TIMEOUT) != 0)
		return -2;

	/* for read, the data coming too fast, then CRC error
	   start DMA no business with CRC. */
	msdc_dma_setup(host, &host->dma, data->sg, data->sg_len);
	msdc_dma_start(host);

	spin_unlock(&host->lock);
	if (!wait_for_completion_timeout(&host->xfer_done, DAT_TIMEOUT)) {
		ERR_MSG("XXX CMD<%d> ARG<0x%x> wait xfer_done<%d> timeout!!",
			cmd->opcode, cmd->arg, data->blocks * data->blksz);

		host->sw_timeout++;

		msdc_dump_info(host->id);
		data->error = (unsigned int)-ETIMEDOUT;
		msdc_reset(host->id);
	}
	spin_lock(&host->lock);

	msdc_dma_stop(host);

	if (((host->autocmd & MSDC_AUTOCMD12) && mrq->stop && mrq->stop->error)
	 || (mrq->data && mrq->data->error)
	 || (mrq->sbc && (mrq->sbc->error != 0) &&
	    (host->autocmd & MSDC_AUTOCMD23))) {
		msdc_clr_fifo(host->id);
		msdc_clr_int();
	}

	/* Commented out since tune is always 0
	if (tune)
		return 0;
	else*/
		return map_sg;
}

#define PREPARE_NON_ASYNC       0
#define PREPARE_ASYNC	        1
#define PREPARE_TUNE            2
int msdc_do_request_prepare(struct msdc_host *host,
	struct mmc_request *mrq,
	struct mmc_command *cmd,
	struct mmc_data *data,
	u32 *l_force_prg,
	u32 *l_bypass_flush,
	int prepare_case)
{
#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	void __iomem *base = host->base;
#endif

	atomic_set(&host->abort, 0);

#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	/* check msdc work ok: RX/TX fifocnt must be zero after last request
	 * if find abnormal, try to reset msdc first
	 */
	if (msdc_txfifocnt() || msdc_rxfifocnt()) {
		pr_err("[SD%d] register abnormal,please check!\n", host->id);
		msdc_reset_hw(host->id);
	}
#endif

	if ((prepare_case == PREPARE_NON_ASYNC) && !data) {

#ifdef MTK_MSDC_USE_CACHE
		if ((host->hw->host_function == MSDC_EMMC) &&
		    check_mmc_cache_flush_cmd(cmd)) {
			if (g_cache_status == CACHE_FLUSHED) {
				N_MSG(CHE, "bypass flush command, g_cache_status=%d",
					g_cache_status);
				*l_bypass_flush = 1;
				return 1;
			}
			*l_bypass_flush = 0;
		}
#endif

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (check_mmc_cmd13_sqs(cmd)) {
			if (msdc_do_cmdq_command(host, cmd, 0, CMD_TIMEOUT) != 0)
				return 1;
		} else {
#endif
		if (msdc_do_command(host, cmd, 0, CMD_TIMEOUT) != 0)
			return 1;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		}
#endif

		/* Get emmc_id when send ALL_SEND_CID command */
		if ((host->hw->host_function == MSDC_EMMC) &&
			(cmd->opcode == MMC_ALL_SEND_CID))
			g_emmc_id = UNSTUFF_BITS(cmd->resp, 120, 8);
		return 1;
	}

	BUG_ON(data->blksz > HOST_MAX_BLKSZ);

	data->error = 0;
	msdc_latest_op[host->id] = (data->flags & MMC_DATA_READ)
		? OPER_TYPE_READ : OPER_TYPE_WRITE;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	/* if CMDQ CMD13 QSR, host->data may be data of mrq - CMD46,47 */
	if (!check_mmc_cmd13_sqs(cmd))
		host->data = data;
#else
	host->data = data;
#endif

	host->xfer_size = data->blocks * data->blksz;
	host->blksz = data->blksz;
	if (prepare_case != PREPARE_NON_ASYNC) {
		host->dma_xfer = 1;
	} else {
		/* deside the transfer mode */
		if (drv_mode[host->id] == MODE_PIO) {
			host->dma_xfer = 0;
			msdc_latest_transfer_mode[host->id] = TRAN_MOD_PIO;
		} else if (drv_mode[host->id] == MODE_DMA) {
			host->dma_xfer = 1;
			msdc_latest_transfer_mode[host->id] = TRAN_MOD_DMA;
		} else if (drv_mode[host->id] == MODE_SIZE_DEP) {
			host->dma_xfer = (host->xfer_size >= dma_size[host->id])
				? 1 : 0;
			msdc_latest_transfer_mode[host->id] =
				host->dma_xfer ? TRAN_MOD_DMA : TRAN_MOD_PIO;
		}
	}

	if (data->flags & MMC_DATA_READ) {
		if ((host->timeout_ns != data->timeout_ns) ||
		    (host->timeout_clks != data->timeout_clks)) {
			msdc_set_timeout(host, data->timeout_ns,
				data->timeout_clks);
		}
	}

	msdc_set_blknum(host, data->blocks);
	/* msdc_clr_fifo();  */ /* no need */

#ifdef MTK_MSDC_USE_CACHE
	/* Currently, tuning does not use CMD23, so force programming
	   cannot be applied */
	if (prepare_case != PREPARE_TUNE
	 && check_mmc_cache_ctrl(host->mmc->card)
	 && (cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK))
		*l_force_prg = !msdc_can_apply_cache(cmd->arg, data->blocks);
#endif

	return 0;
}

int msdc_do_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	u32 l_autocmd23_is_set = 0;
#ifdef MTK_MSDC_USE_CMD23
	u32 l_card_no_cmd23 = 0;
#endif
#ifdef MTK_MSDC_USE_CACHE
	u32 l_force_prg = 0;
	/* 0: flush need, 1: flush bypass, 2: not switch cmd*/
	u32 l_bypass_flush = 2;
#endif
	void __iomem *base = host->base;
	/* u32 intsts = 0; */
	unsigned int left = 0;
	int read = 1, dir = DMA_FROM_DEVICE;
	u32 map_sg = 0;
	unsigned long pio_tmo;

	if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ))
		msdc_sdio_restore_after_resume(host);

#if (MSDC_DATA1_INT == 1)
	if (host->hw->flags & MSDC_SDIO_IRQ) {
		if ((u_sdio_irq_counter > 0) && ((u_sdio_irq_counter%800) == 0))
			ERR_MSG("sdio_irq=%d, msdc_irq=%d  SDC_CFG=%x MSDC_INTEN=%x MSDC_INT=%x ",
				u_sdio_irq_counter, u_msdc_irq_counter,
				MSDC_READ32(SDC_CFG), MSDC_READ32(MSDC_INTEN),
				MSDC_READ32(MSDC_INT));
	}
#endif

	BUG_ON(mmc == NULL || mrq == NULL);

	cmd = mrq->cmd;
	data = mrq->cmd->data;

#ifdef MTK_MSDC_USE_CACHE
	if (msdc_do_request_prepare(host, mrq, cmd, data, &l_force_prg,
		&l_bypass_flush, PREPARE_NON_ASYNC))
#else
	if (msdc_do_request_prepare(host, mrq, cmd, data, NULL,
		NULL, PREPARE_NON_ASYNC))
#endif
		goto done;

#ifdef MTK_MSDC_USE_CMD23
	if (0 == (host->autocmd & MSDC_AUTOCMD23)) {
		/* start the cmd23 first,
		   mrq->sbc is NULL with single r/w */
		if (mrq->sbc) {
			host->autocmd &= ~MSDC_AUTOCMD12;

			if (host->hw->host_function == MSDC_EMMC) {
#ifdef MTK_MSDC_USE_CACHE
				/* We use force programming to make sure data
				 * integrity when enable cache except "userdata"
				 * and "cache" partition
				 */
				if (!((mrq->sbc->arg >> 31) & 0x1) &&
				    l_force_prg)
					mrq->sbc->arg |= (1 << 24);
#endif
			}

			if (msdc_command_start(host, mrq->sbc, 0,
				CMD_TIMEOUT) != 0)
				goto done;

			/* then wait command done */
			if (msdc_command_resp_polling(host, mrq->sbc, 0,
				CMD_TIMEOUT) != 0) {
				goto stop;
			}
		} else {
			/* some sd card may not support cmd23,
			 * some emmc card have problem with cmd23,
			   so use cmd12 here */
			if (host->hw->host_function != MSDC_SDIO)
				host->autocmd |= MSDC_AUTOCMD12;
		}
	} else {
		/* enable auto cmd23 */
		if (mrq->sbc) {
			host->autocmd &= ~MSDC_AUTOCMD12;
			if (host->hw->host_function == MSDC_EMMC) {
#ifdef MTK_MSDC_USE_CACHE
				if (!((mrq->sbc->arg >> 31) & 0x1) &&
				    l_force_prg)
					mrq->sbc->arg |= (1 << 24);
#endif
			}
		} else {
			/* some sd card may not support cmd23,
			 * some emmc card have problem with cmd23,
			   so use cmd12 here */
			if (host->hw->host_function != MSDC_SDIO) {
				host->autocmd &= ~MSDC_AUTOCMD23;
				host->autocmd |= MSDC_AUTOCMD12;
				l_card_no_cmd23 = 1;
			}
		}
	}
#endif /* end of MTK_MSDC_USE_CMD23 */

	read = data->flags & MMC_DATA_READ ? 1 : 0;
	if (host->dma_xfer) {
#ifndef MTK_MSDC_USE_CMD23
		/* start the command first */
		if (host->hw->host_function != MSDC_SDIO)
			host->autocmd |= MSDC_AUTOCMD12;
#endif
		map_sg = msdc_rw_cmd_dma(mmc, cmd, data, mrq, 0);
		if (map_sg == -1)
			goto done;
		else if (map_sg == -2)
			goto stop;

	} else {
		/* Turn off dma */
		if (is_card_sdio(host)) {
			msdc_reset_hw(host->id);
			msdc_dma_off();
			data->error = 0;
		}
		/* Firstly: send command */
		host->autocmd &= ~MSDC_AUTOCMD12;

		l_autocmd23_is_set = 0;
		if (host->autocmd & MSDC_AUTOCMD23) {
			l_autocmd23_is_set = 1;
			host->autocmd &= ~MSDC_AUTOCMD23;
		}

		host->dma_xfer = 0;
		if (msdc_do_command(host, cmd, 0, CMD_TIMEOUT) != 0)
			goto stop;

		/* Secondly: pio data phase */
		if (read) {
#ifdef MTK_MSDC_DUMP_FIFO
			pr_debug("[%s]: start pio read\n", __func__);
#endif
			if (msdc_pio_read(host, data)) {
				msdc_gate_clock(host, 0);
				msdc_ungate_clock(host);
				goto stop;      /* need cmd12 */
			}
		} else {
#ifdef MTK_MSDC_DUMP_FIFO
			pr_debug("[%s]: start pio write\n", __func__);
#endif
			if (msdc_pio_write(host, data)) {
				msdc_gate_clock(host, 0);
				msdc_ungate_clock(host);
				goto stop;
			}

			/* For write case: make sure contents in fifo
			   flushed to device */

			pio_tmo = jiffies + DAT_TIMEOUT;
			while (1) {
				left = msdc_txfifocnt();
				if (left == 0)
					break;

				if (msdc_pio_abort(host, data, pio_tmo))
					break;
			}
		}
	}

stop:
	/* pio mode will disable autocmd23 */
	if (l_autocmd23_is_set == 1) {
		l_autocmd23_is_set = 0;
		host->autocmd |= MSDC_AUTOCMD23;
	}

#ifndef MTK_MSDC_USE_CMD23
	/* Last: stop transfer */
	if (msdc_if_send_stop(host, cmd, data))
		goto done;

#else

	if (host->hw->host_function == MSDC_EMMC) {
		/* multi r/w with no cmd23 and no autocmd12,
		   need send cmd12 manual */
		/* if PIO mode and autocmd23 enable, cmd12 need send,
		   because autocmd23 is disable under PIO */
		if (!check_mmc_cmd1825(cmd->opcode))
			goto done;
		if (((mrq->sbc == NULL) &&
		     !(host->autocmd & MSDC_AUTOCMD12))
		 || (!host->dma_xfer && mrq->sbc &&
		     (host->autocmd & MSDC_AUTOCMD23))) {
			if (msdc_do_command(host, data->stop, 0,
				CMD_TIMEOUT) != 0)
				goto done;
		}
	} else {
		if (msdc_if_send_stop(host, cmd, data))
			goto done;
	}
#endif
done:

#ifdef MTK_MSDC_USE_CMD23
	/* for msdc use cmd23, but card not supported(sbc is NULL),
	   need enable autocmd23 for next request */
	if (1 == l_card_no_cmd23) {
		if (host->hw->host_function != MSDC_SDIO) {
			host->autocmd |= MSDC_AUTOCMD23;
			host->autocmd &= ~MSDC_AUTOCMD12;
			l_card_no_cmd23 = 0;
		}
	}
#endif

	if (data != NULL) {
		host->data = NULL;

		if (host->dma_xfer != 0) {
			host->dma_xfer = 0;
			msdc_dma_off();
			host->dma.used_bd = 0;
			host->dma.used_gpd = 0;
			if (map_sg == 1) {
				/*if (data->error == 0) {
					int retry = 3;
					int count = 1000;
					msdc_retry(host->dma.gpd->hwo, retry,
						count, host->id);
				} */
				dma_unmap_sg(mmc_dev(mmc), data->sg,
					data->sg_len, dir);
			}
		}

		/* If eMMC we use is in g_emmc_cache_quirk[] or
		 * MTK_MSDC_USE_CACHE is closed. Driver should return
		 * cache_size = 0 in exd_csd to mmc layer
		 * So, mmc_init_card can disable cache
		 */
		if ((cmd->opcode == MMC_SEND_EXT_CSD) &&
			(host->hw->host_function == MSDC_EMMC)) {
			msdc_cache_onoff(data);
			#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
			#ifdef MTK_MSDC_DMA_RETRY
			msdc_cq_onoff(data);
			#endif
			#endif
			host->total_sectors = msdc_get_sectors(data);
		}

		host->blksz = 0;

		msdc_log_cmd(host, cmd, data);
	}

	if (mrq->cmd->error == (unsigned int)-EILSEQ) {
		if (((cmd->opcode == MMC_SELECT_CARD) ||
		     (cmd->opcode == MMC_SLEEP_AWAKE))
		 && ((host->hw->host_function == MSDC_EMMC) ||
		     (host->hw->host_function == MSDC_SD))) {
			/* should be deleted in new platform,
			   as the state verify function has applied*/
			mrq->cmd->error = 0x0;
		} else {
			host->error |= REQ_CMD_EIO;

			if (mrq->cmd->opcode == SD_IO_RW_EXTENDED)
				sdio_tune_flag |= 0x1;
		}
	}

	if (mrq->cmd->error == (unsigned int)-ETIMEDOUT) {
		if (mrq->cmd->opcode == MMC_SLEEP_AWAKE) {
			if (mrq->cmd->arg & 0x8000) {
				pr_err("Sleep_Awake CMD timeout, MSDC_PS %0x\n",
					MSDC_READ32(MSDC_PS));
				emmc_sleep_failed = 1;
				mrq->cmd->error = 0x0;
				pr_err("eMMC sleep CMD5 TMO will reinit\n");
			} else {
				host->error |= REQ_CMD_TMO;
			}
		} else {
			host->error |= REQ_CMD_TMO;
		}
	}

	if (mrq->data && mrq->data->error) {
		if (mrq->data->flags & MMC_DATA_WRITE)
			host->error |= REQ_CRC_STATUS_ERR;
		else
			host->error |= REQ_DAT_ERR;
		/* FIXME: return cmd error for retry if data CRC error */
		mrq->cmd->error = (unsigned int)-EILSEQ;
		sdio_tune_flag |= 0x10;

		if (mrq->data->flags & MMC_DATA_READ)
			sdio_tune_flag |= 0x80;
		else
			sdio_tune_flag |= 0x40;
	}

#ifdef MTK_MSDC_USE_CMD23
	if (mrq->sbc && (mrq->sbc->error == (unsigned int)-EILSEQ))
		host->error |= REQ_CMD_EIO;
	if (mrq->sbc && (mrq->sbc->error == (unsigned int)-ETIMEDOUT)) {
#ifdef CONFIG_MTK_AEE_FEATURE
		aee_kernel_warning_api(__FILE__, __LINE__,
			DB_OPT_NE_JBT_TRACES|DB_OPT_DISPLAY_HANG_DUMP,
			"\n@eMMC FATAL ERROR@\n", "eMMC fatal error ");
#endif
		host->error |= REQ_CMD_TMO;
	}
#endif

	if (mrq->stop && (mrq->stop->error == (unsigned int)-EILSEQ))
		host->error |= REQ_STOP_EIO;
	if (mrq->stop && (mrq->stop->error == (unsigned int)-ETIMEDOUT))
		host->error |= REQ_STOP_TMO;

	/* mmc_blk_err_check will also do legacy request
	 * So, use '|=' instead '=' if command error occur
	 * to avoid clearing data error flag
	 */
	if (host->error & REQ_CMD_EIO)
		host->need_tune |= TUNE_LEGACY_CMD;
	else if (host->error & REQ_DAT_ERR)
		host->need_tune = TUNE_LEGACY_DATA_READ;
	else if (host->error & REQ_CRC_STATUS_ERR)
		host->need_tune = TUNE_LEGACY_DATA_WRITE;

	if (host->error & REQ_CMD_EIO || host->error & REQ_DAT_ERR ||
		host->error & REQ_CRC_STATUS_ERR)
		host->err_cmd = mrq->cmd->opcode;
#ifdef SDIO_ERROR_BYPASS
	if (is_card_sdio(host) && !host->error)
		host->sdio_error = 0;
#endif

#ifdef MTK_MSDC_USE_CACHE
	msdc_update_cache_flush_status(host, mrq, data, l_bypass_flush);
#endif

	return host->error;
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static int msdc_do_discard_task_cq(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 task_id;

	task_id = (mrq->sbc->arg >> 16) & 0x1f;
	memset(&mmc->deq_cmd, 0, sizeof(struct mmc_command));
	mmc->deq_cmd.opcode = MMC_CMDQ_TASK_MGMT;
	mmc->deq_cmd.arg = 2 | (task_id << 16);
	mmc->deq_cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1B | MMC_CMD_AC;
	mmc->deq_cmd.data = NULL;
	msdc_do_command(host, &mmc->deq_cmd, 0, CMD_TIMEOUT);

	pr_debug("[%s]: msdc%d, discard task id %d, CMD<%d> arg<0x%08x> rsp<0x%08x>",
		__func__, host->id, task_id, mmc->deq_cmd.opcode, mmc->deq_cmd.arg, mmc->deq_cmd.resp[0]);

	return mmc->deq_cmd.error;
}

static int msdc_do_request_cq(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
#ifdef MTK_MSDC_USE_CACHE
	u32 l_force_prg = 0;
#endif

	BUG_ON(mmc == NULL);
	BUG_ON(mrq == NULL);
	BUG_ON(mrq->data);

	host->error = 0;
	atomic_set(&host->abort, 0);

	cmd  = mrq->sbc;
	data = mrq->data;

	mrq->sbc->error = 0;
	mrq->cmd->error = 0;

#ifdef MTK_MSDC_USE_CACHE
	/* check cache enabled, write direction */
	if (check_mmc_cache_ctrl(host->mmc->card) &&
		!((cmd->arg >> 30) & 0x1)) {
		l_force_prg = !msdc_can_apply_cache(mrq->cmd->arg, cmd->arg & 0xffff);
		/* check not reliable write */
		if (!((cmd->arg >> 31) & 0x1) &&
			l_force_prg)
			cmd->arg |= (1 << 24);
	}
#endif

	if (msdc_do_cmdq_command(host, cmd, 0, CMD_TIMEOUT) != 0)
		goto done1;

done1:
	if (cmd->error == (unsigned int)-EILSEQ)
		host->error |= REQ_CMD_EIO;
	else if (cmd->error == (unsigned int)-ETIMEDOUT)
		host->error |= REQ_CMD_TMO;

	cmd  = mrq->cmd;
	data = mrq->cmd->data;

	if (msdc_do_cmdq_command(host, cmd, 0, CMD_TIMEOUT) != 0)
		goto done2;

done2:
	if (cmd->error == (unsigned int)-EILSEQ)
		host->error |= REQ_CMD_EIO;
	else if (cmd->error == (unsigned int)-ETIMEDOUT)
		host->error |= REQ_CMD_TMO;

	return host->error;
}

static int tune_cmdq_cmdrsp(struct mmc_host *mmc,
	struct mmc_request *mrq, int *retry)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	unsigned long polling_tmo = 0, polling_status_tmo;

	u32 err = 0, status = 0;

	/* time for wait device to return to trans state
	when needed to send CMD48 */
	polling_status_tmo = jiffies + 30 * HZ;

	do {
		err = msdc_get_card_status(mmc, host, &status);
		if (err) {
			/* wait for transfer done */
			polling_tmo = jiffies + 10 * HZ;
			pr_err("msdc%d waiting data transfer done\n",
				host->id);
			while (mmc->is_data_dma) {
				if (time_after(jiffies, polling_tmo)) {
					ERR_MSG("waiting data transfer done TMO");
					msdc_dump_info(host->id);
					msdc_dma_stop(host);
					msdc_dma_clear(host);
					msdc_reset_hw(host->id);
					return -1;
				}
			}

			ERR_MSG("get card status, err = %d", err);

			if (msdc_execute_tuning(mmc, MMC_SEND_STATUS)) {
				ERR_MSG("failed to updata cmd para");
				return 1;
			}

			continue;
		}

		if (status & (1 << 22)) {
			/* illegal command */
			 (*retry)--;
			ERR_MSG("status = %x, illegal command, retry = %d",
				status, *retry);
			if ((mrq->cmd->error || mrq->sbc->error) && *retry)
				return 0;
			else
				return 1;
		} else {
			if (R1_CURRENT_STATE(status) != R1_STATE_TRAN) {
				if (time_after(jiffies, polling_status_tmo))
					ERR_MSG("wait transfer state timeout\n");
				else {
					err = 1;
					continue;
				}
			}
			ERR_MSG("status = %x, discard task, re-send command",
				status);
			err = msdc_do_discard_task_cq(mmc, mrq);
			if (err == (unsigned int)-EIO)
				continue;
			else
				break;
		}
	} while (err);

	/* wait for transfer done */
	polling_tmo = jiffies + 10 * HZ;
	pr_err("msdc%d waiting data transfer done\n", host->id);
	while (mmc->is_data_dma) {
		if (time_after(jiffies, polling_tmo)) {
			ERR_MSG("waiting data transfer done TMO");
			msdc_dump_info(host->id);
			msdc_dma_stop(host);
			msdc_dma_clear(host);
			msdc_reset_hw(host->id);
			return -1;
		}
	}

	if (msdc_execute_tuning(mmc, MMC_SEND_STATUS)) {
		pr_err("msdc%d autok failed\n", host->id);
		return 1;
	}

	return 0;
}
#endif

static void msdc_dump_trans_error(struct msdc_host   *host,
	struct mmc_command *cmd,
	struct mmc_data    *data,
	struct mmc_command *stop,
	struct mmc_command *sbc)
{
	if ((cmd->opcode == 52) && (cmd->arg == 0xc00))
		return;
	if ((cmd->opcode == 52) && (cmd->arg == 0x80000c08))
		return;

	/* SDcard bypass SDIO init CMD5 command*/
	if (host->hw->host_function == MSDC_SD) {
		if (cmd->opcode == 5)
			return;
	/* eMMC bypss SDIO init command, SDcard init command */
	} else if (host->hw->host_function == MSDC_EMMC) {
		if ((cmd->opcode == 5) || (cmd->opcode == 55))
			return;
	} else {
		if (cmd->opcode == 8)
			return;
	}

	ERR_MSG("XXX CMD<%d><0x%x> Error<%d> Resp<0x%x>", cmd->opcode, cmd->arg,
		cmd->error, cmd->resp[0]);

	if (data) {
		ERR_MSG("XXX DAT block<%d> Error<%d>", data->blocks,
			data->error);
	}
	if (stop) {
		ERR_MSG("XXX STOP<%d><0x%x> Error<%d> Resp<0x%x>",
			stop->opcode, stop->arg, stop->error, stop->resp[0]);
	}

	if (sbc) {
		ERR_MSG("XXX SBC<%d><0x%x> Error<%d> Resp<0x%x>",
			sbc->opcode, sbc->arg, sbc->error, sbc->resp[0]);
	}

#ifdef SDIO_ERROR_BYPASS
	if (is_card_sdio(host) &&
	    (host->sdio_error != -EILSEQ) &&
	    (cmd->opcode == 53) &&
	    (msdc_sg_len(data->sg, host->dma_xfer) > 4)) {
		host->sdio_error = -EILSEQ;
		ERR_MSG("XXX SDIO Error ByPass");
	}
#endif
}

static void msdc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq,
	bool is_first_req)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data;
	struct mmc_command *cmd = mrq->cmd;
	int read = 1, dir = DMA_FROM_DEVICE;

	BUG_ON(!cmd);
	data = mrq->data;

	if (!data)
		return;

	data->host_cookie = MSDC_COOKIE_ASYNC;
	if (check_mmc_cmd1718(cmd->opcode) ||
	    check_mmc_cmd2425(cmd->opcode)) {
		host->xfer_size = data->blocks * data->blksz;
		read = data->flags & MMC_DATA_READ ? 1 : 0;
		if (drv_mode[host->id] == MODE_PIO) {
			data->host_cookie |= MSDC_COOKIE_PIO;
			msdc_latest_transfer_mode[host->id] = TRAN_MOD_PIO;
		} else if (drv_mode[host->id] == MODE_DMA) {
			msdc_latest_transfer_mode[host->id] = TRAN_MOD_DMA;
		} else if (drv_mode[host->id] == MODE_SIZE_DEP) {
			if (host->xfer_size < dma_size[host->id]) {
				data->host_cookie |= MSDC_COOKIE_PIO;
				msdc_latest_transfer_mode[host->id] =
					TRAN_MOD_PIO;
			} else {
				msdc_latest_transfer_mode[host->id] =
					TRAN_MOD_DMA;
			}
		}
		if (msdc_use_async_dma(data->host_cookie)) {
			dir = read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
			(void)dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len,
				dir);
		}
		N_MSG(OPS, "CMD<%d> ARG<0x%x> data<%s %s> blksz<%d> block<%d> error<%d>",
			mrq->cmd->opcode, mrq->cmd->arg,
			(data->host_cookie ? "dma" : "pio"),
			(read ? "read " : "write"), data->blksz,
			data->blocks, data->error);
	}
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	else if (data && check_mmc_cmd4647(cmd->opcode)) {
		read = data->flags & MMC_DATA_READ ? 1 : 0;
		dir = read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
		(void)dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len, dir);
	}
#endif
}

static void msdc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
	int err)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data;
	/* struct mmc_command *cmd = mrq->cmd; */
	int dir = DMA_FROM_DEVICE;

	data = mrq->data;
	if (data && (msdc_use_async_dma(data->host_cookie))) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (!mmc->card->ext_csd.cmdq_mode_en)
#endif
			host->xfer_size = data->blocks * data->blksz;
		dir = data->flags & MMC_DATA_READ ?
			DMA_FROM_DEVICE : DMA_TO_DEVICE;
		dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len, dir);
		data->host_cookie = 0;
		N_MSG(OPS, "CMD<%d> ARG<0x%x> blksz<%d> block<%d> error<%d>",
			mrq->cmd->opcode, mrq->cmd->arg, data->blksz,
			data->blocks, data->error);
	}
	data->host_cookie = 0;

}

static int msdc_do_request_async(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	void __iomem *base = host->base;

#ifdef MTK_MSDC_USE_CMD23
	u32 l_card_no_cmd23 = 0;
#endif

#ifdef MTK_MSDC_USE_CACHE
	u32 l_force_prg = 0;
#endif
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	u32 task_id;
#endif

	MVG_EMMC_DECLARE_INT32(delay_ns);
	MVG_EMMC_DECLARE_INT32(delay_us);
	MVG_EMMC_DECLARE_INT32(delay_ms);

	BUG_ON(mrq == NULL);

	if (!is_card_present(host) || host->power_mode == MMC_POWER_OFF) {
		ERR_MSG("cmd<%d> arg<0x%x> card<%d> power<%d>",
			mrq->cmd->opcode, mrq->cmd->arg,
			is_card_present(host), host->power_mode);
		mrq->cmd->error = (unsigned int)-ENOMEDIUM;
		if (mrq->done)
			mrq->done(mrq); /* call done directly. */
		return 0;
	}
	msdc_ungate_clock(host);

	host->error = 0;

	spin_lock(&host->lock);

	cmd = mrq->cmd;
	data = mrq->cmd->data;

	host->mrq = mrq;

#ifdef MTK_MSDC_USE_CACHE
	if (msdc_do_request_prepare(host, mrq, cmd, data, &l_force_prg,
		NULL, PREPARE_ASYNC))
		goto done;
#else
	if (msdc_do_request_prepare(host, mrq, cmd, data, NULL,
		NULL, PREPARE_ASYNC))
		goto done;
#endif

#ifdef MTK_MSDC_USE_CMD23
	/* start the cmd23 first */
	if (mrq->sbc) {
		host->autocmd &= ~MSDC_AUTOCMD12;

		if (host->hw->host_function == MSDC_EMMC) {
#ifdef MTK_MSDC_USE_CACHE
			if (l_force_prg && !((mrq->sbc->arg >> 31) & 0x1))
				mrq->sbc->arg |= (1 << 24);
#endif
		}

		if (0 == (host->autocmd & MSDC_AUTOCMD23)) {
			if (msdc_command_start(host, mrq->sbc, 0,
				CMD_TIMEOUT) != 0)
				goto done;

			/* then wait command done */
			if (msdc_command_resp_polling(host, mrq->sbc, 0,
				CMD_TIMEOUT) != 0) {
				goto stop;
			}
		}
	} else {
		/* some sd card may not support cmd23,
		 * some emmc card have problem with cmd23,
		   so use cmd12 here */
		if (host->hw->host_function != MSDC_SDIO) {
			host->autocmd |= MSDC_AUTOCMD12;
			if (0 != (host->autocmd & MSDC_AUTOCMD23)) {
				host->autocmd &= ~MSDC_AUTOCMD23;
				l_card_no_cmd23 = 1;
			}
		}
	}

#else
	/* start the command first*/
	if (host->hw->host_function != MSDC_SDIO)
		host->autocmd |= MSDC_AUTOCMD12;
#endif /* end of MTK_MSDC_USE_CMD23 */

	msdc_dma_on();          /* enable DMA mode first!! */
	/* init_completion(&host->xfer_done); */

	if (msdc_command_start(host, cmd, 0, CMD_TIMEOUT) != 0)
		goto done;

	/* then wait command done */
	if (msdc_command_resp_polling(host, cmd, 0, CMD_TIMEOUT) != 0)
		goto stop;

	/* for read, the data coming too fast, then CRC error
	   start DMA no business with CRC. */
	msdc_dma_setup(host, &host->dma, data->sg, data->sg_len);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	mmc->is_data_dma = 1;
#endif

	msdc_dma_start(host);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (check_mmc_cmd4647(cmd->opcode)) {
		task_id = (cmd->arg >> 16) & 0x1f;
		MVG_EMMC_WRITE_MATCH(host,
			(u64)mmc->areq_que[task_id]->mrq_que->cmd->arg,
			delay_ms, delay_us, delay_ns,
			cmd->opcode, host->xfer_size);
	} else
#endif
	MVG_EMMC_WRITE_MATCH(host, (u64)cmd->arg, delay_ms, delay_us, delay_ns,
		cmd->opcode, host->xfer_size);

	spin_unlock(&host->lock);

#ifdef MTK_MSDC_USE_CMD23
	/* for msdc use cmd23, but card not supported(sbc is NULL),
	   need enable autocmd23 for next request */
	if (1 == l_card_no_cmd23) {
		if (host->hw->host_function != MSDC_SDIO) {
			host->autocmd |= MSDC_AUTOCMD23;
			host->autocmd &= ~MSDC_AUTOCMD12;
			l_card_no_cmd23 = 0;
		}
	}
#endif

#ifdef MTK_MSDC_USE_CACHE
	msdc_update_cache_flush_status(host, mrq, data, 1);
#endif

	return 0;


stop:
	/* Last: stop transfer */
	if (msdc_if_send_stop(host, cmd, data))
		pr_err("%s send stop error\n", __func__);
done:
#ifdef MTK_MSDC_USE_CMD23
	/* for msdc use cmd23, but card not supported(sbc is NULL),
	   need enable autocmd23 for next request */
	if (1 == l_card_no_cmd23) {
		if (host->hw->host_function != MSDC_SDIO) {
			host->autocmd |= MSDC_AUTOCMD23;
			host->autocmd &= ~MSDC_AUTOCMD12;
			l_card_no_cmd23 = 0;
		}
	}
#endif

	msdc_dma_clear(host);

	msdc_log_cmd(host, cmd, data);

#ifdef MTK_MSDC_USE_CMD23
	if (mrq->sbc && (mrq->sbc->error == (unsigned int)-EILSEQ))
		host->error |= REQ_CMD_EIO;
	if (mrq->sbc && (mrq->sbc->error == (unsigned int)-ETIMEDOUT)) {
#ifdef CONFIG_MTK_AEE_FEATURE
		aee_kernel_warning_api(__FILE__, __LINE__,
			DB_OPT_NE_JBT_TRACES|DB_OPT_DISPLAY_HANG_DUMP,
			"\n@eMMC FATAL ERROR@\n", "eMMC fatal error ");
#endif
		host->error |= REQ_CMD_TMO;
	}
#endif

	if (mrq->cmd->error == (unsigned int)-EILSEQ) {
		if (((cmd->opcode == MMC_SELECT_CARD) ||
		     (cmd->opcode == MMC_SLEEP_AWAKE))
		 && ((host->hw->host_function == MSDC_EMMC) ||
		     (host->hw->host_function == MSDC_SD)))
			/* CMD5 CMD7 set error to 0*/
			mrq->cmd->error = 0x0;
		else
			host->error |= REQ_CMD_EIO;
	}

	if (mrq->cmd->error == (unsigned int)-ETIMEDOUT)
		host->error |= REQ_CMD_TMO;
	if (mrq->data && (mrq->data->error)) {
		if (mrq->data->flags & MMC_DATA_WRITE)
			host->error |= REQ_CRC_STATUS_ERR;
		else
			host->error |= REQ_DAT_ERR;
	}
	if (mrq->stop && (mrq->stop->error == (unsigned int)-EILSEQ))
		host->error |= REQ_STOP_EIO;
	if (mrq->stop && (mrq->stop->error == (unsigned int)-ETIMEDOUT))
		host->error |= REQ_STOP_TMO;

	/* re-autok or try smpl except TMO */
	if (host->error & REQ_CMD_EIO)
		host->need_tune = TUNE_ASYNC_CMD;

#ifdef MTK_MSDC_USE_CACHE
	msdc_update_cache_flush_status(host, mrq, data, 1);
#endif

	if (mrq->done)
		mrq->done(mrq);

	msdc_gate_clock(host, 1);
	spin_unlock(&host->lock);

	return host->error;
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static int msdc_do_cmdq_request_with_retry(struct msdc_host *host,
	struct mmc_request *mrq)
{
	struct mmc_host *mmc;
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_command *stop = NULL;
	int ret = 0, retry;

	mmc = host->mmc;
	cmd = mrq->cmd;
	data = mrq->cmd->data;
	if (data)
		stop = data->stop;

	retry = 5;
	while (msdc_do_request_cq(mmc, mrq)) {
		msdc_dump_trans_error(host, cmd, data, stop, mrq->sbc);
		if ((cmd->error == (unsigned int)-EILSEQ) ||
			(cmd->error == (unsigned int)-ETIMEDOUT) ||
			(mrq->sbc->error == (unsigned int)-EILSEQ) ||
			(mrq->sbc->error == (unsigned int)-ETIMEDOUT)) {
			ret = tune_cmdq_cmdrsp(mmc, mrq, &retry);
			if (ret)
				return ret;
		} else {
			ERR_MSG("CMD44 and CMD45 error - error %d %d",
				mrq->sbc->error, cmd->error);
			break;
		}
	}

	return ret;
}
#endif
/* ops.request */
static void msdc_ops_request_legacy(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_command *stop = NULL;
	struct mmc_command *sbc = NULL;
	/* === for sdio profile === */
	u32 old_H32 = 0, old_L32 = 0, new_H32 = 0, new_L32 = 0;
	u32 ticks = 0, opcode = 0, sizes = 0, bRx = 0;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	int ret;
#endif

	host->error = 0;

#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (host->mrq) {
		ERR_MSG("XXX host->mrq<0x%p> cmd<%d>arg<0x%x>", host->mrq,
			host->mrq->cmd->opcode, host->mrq->cmd->arg);
		BUG();
	}
#endif

	if (is_card_present(host)
		&& host->power_mode == MMC_POWER_OFF
		&& mrq->cmd->opcode == 7 && mrq->cmd->arg == 0
		&& host->hw->host_function == MSDC_SD) {
		ERR_MSG("cmd<7> arg<0x0> card<1> power<0>, bypass return -ENOMEDIUM");
	} else if (!is_card_present(host) || host->power_mode == MMC_POWER_OFF) {
		ERR_MSG("cmd<%d> arg<0x%x> card<%d> power<%d>",
			mrq->cmd->opcode, mrq->cmd->arg,
			is_card_present(host), host->power_mode);
		mrq->cmd->error = (unsigned int)-ENOMEDIUM;
		if (mrq->done)
			mrq->done(mrq); /* call done directly. */

		return;
	}

	/* start to process */
	spin_lock(&host->lock);
	cmd = mrq->cmd;
	data = mrq->cmd->data;
	if (data)
		stop = data->stop;

#ifdef MTK_MSDC_USE_CMD23
	if (data)
		sbc = mrq->sbc;
#endif

	msdc_ungate_clock(host);  /* set sw flag */

	if (sdio_pro_enable) {
		/*=== for sdio profile ===*/
		if (mrq->cmd->opcode == 52 || mrq->cmd->opcode == 53)
			; /*GPT_GetCounter64(&old_L32, &old_H32);*/
	}

#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	host->mrq = mrq;
#endif

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (check_mmc_cmd44(mrq->sbc)) {
		ret = msdc_do_cmdq_request_with_retry(host, mrq);
		goto cq_req_done;
	} else {
		/* only CMD0/CMD12/CMD13 can be send
			when non-empty queue @ CMDQ on */
		if (mmc->card && mmc->card->ext_csd.cmdq_mode_en
			&& atomic_read(&mmc->areq_cnt)
			&& !check_mmc_cmd01213(cmd->opcode)
			&& !check_mmc_cmd48(cmd->opcode)) {
			ERR_MSG("[%s][WARNING] CMDQ on, sending CMD%d\n",
				__func__, cmd->opcode);
		}
		if (!check_mmc_cmd13_sqs(mrq->cmd))
			host->mrq = mrq;
	}
#endif

	if (msdc_do_request(host->mmc, mrq)) {
		msdc_dump_trans_error(host, cmd, data, stop, sbc);
	} else {
		/* mmc_blk_err_check will do legacy request without data */
		if (host->need_tune & TUNE_LEGACY_CMD)
			host->need_tune &= ~TUNE_LEGACY_CMD;
		/* Retry legacy data read pass, clear autok pass flag */
		if ((host->need_tune & TUNE_LEGACY_DATA_READ) &&
			mrq->cmd->data) {
			host->need_tune &= ~TUNE_LEGACY_DATA_READ;
			host->need_tune &= ~TUNE_AUTOK_PASS;
			host->reautok_times = 0;
			host->tune_smpl_times = 0;
		}
		/* Retry legacy data write pass, clear autok pass flag */
		if ((host->need_tune & TUNE_LEGACY_DATA_WRITE) &&
			mrq->cmd->data) {
			host->need_tune &= ~TUNE_LEGACY_DATA_WRITE;
			host->need_tune &= ~TUNE_AUTOK_PASS;
			host->reautok_times = 0;
			host->tune_smpl_times = 0;
		}
		/* legacy command err, tune pass, clear autok pass flag */
		if (host->need_tune == TUNE_AUTOK_PASS) {
			host->reautok_times = 0;
			host->tune_smpl_times = 0;
			host->need_tune = TUNE_NONE;
		}
		if (host->need_tune == TUNE_NONE)
			host->err_cmd = -1;
	}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
cq_req_done:
#endif

#ifdef MTK_MSDC_USE_CACHE
	msdc_check_cache_flush_error(host, cmd);
#endif

	/* ==== when request done, check if app_cmd ==== */
	if (mrq->cmd->opcode == MMC_APP_CMD) {
		host->app_cmd = 1;
		host->app_cmd_arg = mrq->cmd->arg;      /* save the RCA */
	} else {
		host->app_cmd = 0;
		/* host->app_cmd_arg = 0; */
	}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (!(check_mmc_cmd13_sqs(mrq->cmd)
		|| check_mmc_cmd44(mrq->sbc))) {
		/* if not CMDQ CMD44/45 or CMD13, follow orignal flow to clear host->mrq
		if it's CMD44/45 or CMD13 QSR, host->mrq may be CMD46,47 */
		host->mrq = NULL;
	}
#else
	host->mrq = NULL;
#endif

	/* === for sdio profile === */
	if (sdio_pro_enable) {
		if (mrq->cmd->opcode == 52 || mrq->cmd->opcode == 53) {
			/* GPT_GetCounter64(&new_L32, &new_H32); */
			ticks = msdc_time_calc(old_L32, old_H32,
				new_L32, new_H32);

			opcode = mrq->cmd->opcode;
			if (mrq->cmd->data) {
				sizes = mrq->cmd->data->blocks *
					mrq->cmd->data->blksz;
				bRx = mrq->cmd->data->flags & MMC_DATA_READ ?
					1 : 0;
			} else {
				bRx = mrq->cmd->arg & 0x80000000 ? 1 : 0;
			}

			if (!mrq->cmd->error)
				msdc_performance(opcode, sizes, bRx, ticks);
		}
	}

	msdc_gate_clock(host, 1);       /* clear flag. */

	spin_unlock(&host->lock);

	mmc_request_done(mmc, mrq);
}

int msdc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret = 0;
	int sdio_res_exist = 0;

	msdc_init_tune_path(host, mmc->ios.timing);
	host->tuning_in_progress = true;

#ifndef FPGA_PLATFORM
	pmic_force_vcore_pwm(true); /* set PWM mode for MT6351 */
#endif
	msdc_ungate_clock(host);
	if (host->hw->host_function == MSDC_SD) {
		if (mmc->ios.timing == MMC_TIMING_UHS_SDR104 ||
		    mmc->ios.timing == MMC_TIMING_UHS_SDR50) {
			if (host->is_autok_done == 0) {
				pr_err("[AUTOK]SDcard autok\n");
				ret = autok_execute_tuning(host, sd_autok_res[AUTOK_VCORE_HIGH]);
				host->is_autok_done = 1;
			} else {
				autok_init_sdr104(host);
				autok_tuning_parameter_init(host, sd_autok_res[AUTOK_VCORE_HIGH]);
			}
		}
	} else if (host->hw->host_function == MSDC_EMMC) {
		if (mmc->ios.timing == MMC_TIMING_MMC_HS200) {
			if (opcode == MMC_SEND_STATUS) {
				pr_err("[AUTOK]eMMC HS200 Tune CMD only\n");
				ret = hs200_execute_tuning_cmd(host, NULL);
			} else {
				pr_err("[AUTOK]eMMC HS200 Tune\n");
				ret = hs200_execute_tuning(host, NULL);
			}
		} else if (mmc->ios.timing == MMC_TIMING_MMC_HS400) {
			if (opcode == MMC_SEND_STATUS) {
				pr_err("[AUTOK]eMMC HS400 Tune CMD only\n");
				ret = hs400_execute_tuning_cmd(host, NULL);
			} else {
				pr_err("[AUTOK]eMMC HS400 Tune\n");
				ret = hs400_execute_tuning(host, NULL);
			}
		}
	} else if (host->hw->host_function == MSDC_SDIO) {
		if (host->is_autok_done) {
			autok_init_sdr104(host);
			/* Check which vcore setting to apply */
			if (vcorefs_get_hw_opp() == OPPI_PERF)
				autok_tuning_parameter_init(host, sdio_autok_res[AUTOK_VCORE_HIGH]);
			else
				autok_tuning_parameter_init(host, sdio_autok_res[AUTOK_VCORE_LOW]);

			if (sdio_use_dvfs == 0) {
				pr_err("[AUTOK] No need change para when dvfs\n");
			} else {
				pr_err("[AUTOK] Need change para when dvfs\n");

				/* Use HW DVFS */
				sdio_dvfs_reg_restore(host);
				/* Enable DVFS handshake */
				spm_msdc_dvfs_setting(MSDC2_DVFS, 1);
			}

			goto out;
		}
		/* HQA need read autok setting from file */
		sdio_res_exist = sdio_autok_res_exist(host);

		pr_err("[AUTOK]SDIO SDR104 Tune\n");
		/* Wait DFVS ready for excute autok here */
		sdio_autok_wait_dvfs_ready();

		/* Performance mode, return 0 pass */
		if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, OPPI_PERF) != 0)
			pr_err("vcorefs_request_dvfs_opp@OPPI_PERF fail!\n");
		if (sdio_res_exist)
			sdio_autok_res_apply(host, AUTOK_VCORE_HIGH);
		else
			autok_execute_tuning(host, sdio_autok_res[AUTOK_VCORE_HIGH]);
		sdio_set_hw_dvfs(AUTOK_VCORE_HIGH, 0, host);

		/* Low power mode, return 0 pass */
		if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, OPPI_LOW_PWR) != 0)
			pr_err("vcorefs_request_dvfs_opp@OPPI_LOW_PWR fail!\n");
		if (sdio_res_exist)
			sdio_autok_res_apply(host, AUTOK_VCORE_LOW);
		else
			autok_execute_tuning(host, sdio_autok_res[AUTOK_VCORE_LOW]);
		/* Enable HW DVFS, but setting used now is at register offset <=0x104.
		 * Setting at register offset >=0x300 will effect after SPM handshakes
		 * with MSDC.
		 */
		sdio_set_hw_dvfs(AUTOK_VCORE_LOW, 1, host);

		if (autok_res_check(sdio_autok_res[AUTOK_VCORE_HIGH],
				sdio_autok_res[AUTOK_VCORE_LOW]) == 0) {
			pr_err("[AUTOK] No need change para when dvfs\n");
		} else {
			pr_err("[AUTOK] Need change para when dvfs\n");

			/* Use HW DVFS */
			sdio_use_dvfs = 1;
			/* Enable DVFS handshake */
			spm_msdc_dvfs_setting(MSDC2_DVFS, 1);
		}

		/* Un-request, return 0 pass */
		if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, OPPI_UNREQ) != 0)
			pr_err("vcorefs_request_dvfs_opp@OPPI_UNREQ fail!\n");

		host->is_autok_done = 1;
		complete(&host->autok_done);
	}

out:
	if (ret) {
		/* FIX ME, consider to use msdc_dump_info() to replace all */
		msdc_dump_clock_sts(host);
		msdc_dump_ldo_sts(host);
		pr_err("msdc%d latest_INT_status<0x%.8x>\n", host->id,
				latest_int_status[host->id]);
		msdc_dump_register(host);
		msdc_dump_dbg_register(host);
	}

	host->tuning_in_progress = false;
#ifndef FPGA_PLATFORM
	pmic_force_vcore_pwm(false); /* set non-PWM mode for MT6351 */
#endif
	msdc_gate_clock(host, 1);

	return 0;
}

int msdc_stop_and_wait_busy(struct msdc_host *host, struct mmc_request *mrq)
{
	void __iomem *base = host->base;
	unsigned long polling_tmo = 0;

	msdc_send_stop(host);
	polling_tmo = jiffies + POLLING_BUSY;
	pr_err("msdc%d, waiting device is not busy\n", host->id);
	while ((MSDC_READ32(MSDC_PS) & 0x10000) != 0x10000) {
		if (time_after(jiffies, polling_tmo)) {
			pr_err("msdc%d, device stuck in PRG!\n",
				host->id);
			return -1;
		}
	}

	return 0;
}

/* FIX ME, consider to move it to msdc_tune.c */
int msdc_error_tuning(struct mmc_host *mmc,  struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret = 0;
	int autok_err_type = -1;
	unsigned int tune_smpl = 0;

	msdc_ungate_clock(host);
	host->tuning_in_progress = true;

	/* autok_err_type is used for switch edge tune and CMDQ tune */
	if ((host->need_tune & TUNE_ASYNC_CMD) ||
		(host->need_tune & TUNE_LEGACY_CMD))
		autok_err_type = CMD_ERROR;
	else if ((host->need_tune & TUNE_ASYNC_DATA_READ) ||
		(host->need_tune & TUNE_LEGACY_DATA_READ))
		autok_err_type = DATA_ERROR;
	else if ((host->need_tune & TUNE_ASYNC_DATA_WRITE) ||
		(host->need_tune & TUNE_LEGACY_DATA_WRITE))
		autok_err_type = CRC_STATUS_ERROR;

	/* 1. mmc_blk_err_check will send CMD13 to check device status
	 * Don't autok/switch edge here, or it will cause CMD19 send when
	 * device is not in transfer status
	 * 2. mmc_blk_err_check will send CMD12 to stop transition if device is
	 * in RCV and DATA status.
	 * Don't autok/switch edge here, or it will cause switch edge failed
	 */
	if ((mrq->cmd->opcode == MMC_SEND_STATUS &&
		host->err_cmd != MMC_SEND_STATUS) ||
		(mrq->cmd->opcode == MMC_STOP_TRANSMISSION &&
		host->err_cmd != MMC_STOP_TRANSMISSION)) {
		goto end;
	}

	pr_err("msdc%d saved device status: %x", host->id, host->device_status);
	/* clear device status */
	host->device_status = 0x0;

	/* force send stop command to turn device to transfer status */
	if (msdc_stop_and_wait_busy(host, mrq))
		goto recovery;

	if (host->hw->host_function == MSDC_EMMC ||
		host->hw->host_function == MSDC_SD) {

#ifndef FPGA_PLATFORM
		pmic_force_vcore_pwm(true); /* set PWM mode for MT6351 */
#endif
		switch (mmc->ios.timing) {
		case MMC_TIMING_UHS_SDR104:
			pr_err("msdc%d: SD UHS_SDR104 re-autok %d times\n",
				host->id, ++host->reautok_times);
			ret = autok_execute_tuning(host, NULL);
			break;
		case MMC_TIMING_UHS_SDR50:
			pr_err("msdc%d: SD UHS_SDR50 re-autok %d times\n",
				host->id, ++host->reautok_times);
			ret = autok_execute_tuning(host, NULL);
			break;
		case MMC_TIMING_MMC_HS200:
			pr_err("msdc%d: eMMC HS200 re-autok %d times\n",
				host->id, ++host->reautok_times);
			ret = hs200_execute_tuning(host, NULL);
			break;
		case MMC_TIMING_MMC_HS400:
			pr_err("msdc%d: eMMC HS400 re-autok %d times\n",
				host->id, ++host->reautok_times);
			ret = hs400_execute_tuning(host, NULL);
			break;
		/* Other speed mode will tune smpl */
		default:
			tune_smpl = 1;
			pr_err("msdc%d: tune smpl %d times timing:%d err: %d\n",
				host->id, ++host->tune_smpl_times,
				mmc->ios.timing, autok_err_type);
			autok_low_speed_switch_edge(host, &mmc->ios, autok_err_type);
			break;
		}
#ifndef FPGA_PLATFORM
		pmic_force_vcore_pwm(false); /* set PWM mode for MT6351 */
#endif
		if (ret) {
			/* FIX ME, consider to use msdc_dump_info() to replace
			   all */
			msdc_dump_clock_sts(host);
			msdc_dump_ldo_sts(host);
			pr_err("msdc%d latest_INT_status<0x%.8x>\n", host->id,
					latest_int_status[host->id]);
			msdc_dump_register(host);
			msdc_dump_dbg_register(host);
		}

		/* autok failed three times will try reinit tuning */
		if ((host->reautok_times >= 4) || (host->tune_smpl_times >= 4)) {
recovery:
			pr_err("msdc%d autok error\n", host->id);
			/* eMMC will chang to HS200 and lower frequence */
			if (host->hw->host_function == MSDC_EMMC) {
				msdc_dump_register(host);
#ifdef MSDC_SWITCH_MODE_WHEN_ERROR
				ret = emmc_reinit_tuning(mmc);
#endif
			}
			/* SDcard will change speed mode and power reset sdcard*/
			if (host->hw->host_function == MSDC_SD)
				ret = sdcard_reset_tuning(mmc);
		} else if (!tune_smpl) {
			pr_err("msdc%d autok pass\n", host->id);
			host->need_tune |= TUNE_AUTOK_PASS;
		}
	}

end:
	host->tuning_in_progress = false;
	msdc_gate_clock(host, 1);

	return ret;
}

static void msdc_ops_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_data *data;
	int host_cookie = 0;
	struct msdc_host *host = mmc_priv(mmc);
	BUG_ON(mrq == NULL);

	if ((host->hw->host_function == MSDC_SDIO) &&
	    !(host->trans_lock.active))
		__pm_stay_awake(&host->trans_lock);

	data = mrq->data;
	if (data)
		host_cookie = data->host_cookie;

	if (!(host->tuning_in_progress) && host->need_tune &&
		!(host->need_tune & TUNE_AUTOK_PASS))
		msdc_error_tuning(mmc, mrq);

	/* Async only support  DMA and asyc CMD flow */
	if (msdc_use_async_dma(host_cookie))
		msdc_do_request_async(mmc, mrq);
	else
		msdc_ops_request_legacy(mmc, mrq);

	if ((host->hw->host_function == MSDC_SDIO) &&
	    (host->trans_lock.active))
		__pm_relax(&host->trans_lock);
}

/* ops.set_ios */
void msdc_ops_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);

	spin_lock(&host->lock);
	msdc_ungate_clock(host);

	if (host->power_mode != ios->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
		case MMC_POWER_UP:
			spin_unlock(&host->lock);
			msdc_init_hw(host);
			msdc_set_power_mode(host, ios->power_mode);
			spin_lock(&host->lock);
			break;
		case MMC_POWER_ON:
		default:
			break;
		}
		host->power_mode = ios->power_mode;
	}

	if (host->bus_width != ios->bus_width) {
		msdc_set_buswidth(host, ios->bus_width);
		host->bus_width = ios->bus_width;
	}

	if (msdc_clock_src[host->id] != host->hw->clk_src) {
		host->hw->clk_src = msdc_clock_src[host->id];
		msdc_select_clksrc(host, host->hw->clk_src);
	}

	if (host->timing != ios->timing) {
		/* msdc setting TX parameter */
		msdc_ios_tune_setting(host, ios);
		if (ios->timing == MMC_TIMING_MMC_DDR52)
			msdc_set_mclk(host, ios->timing, ios->clock);
		host->timing = ios->timing;
	}

	if (host->mclk != ios->clock) {
		msdc_set_mclk(host, ios->timing, ios->clock);
		if (ios->timing == MMC_TIMING_MMC_HS400)
			msdc_execute_tuning(host->mmc, MMC_SEND_TUNING_BLOCK_HS200);
	}

	msdc_gate_clock(host, 1);
	spin_unlock(&host->lock);
}

/* ops.get_ro */
static int msdc_ops_get_ro(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	unsigned long flags;
	int ro = 0;

	spin_lock_irqsave(&host->lock, flags);
	msdc_ungate_clock(host);
	if (host->hw->flags & MSDC_WP_PIN_EN)
		ro = (MSDC_READ32(MSDC_PS) >> 31);
	msdc_gate_clock(host, 1);
	spin_unlock_irqrestore(&host->lock, flags);

	return ro;
}

/* ops.get_cd */
static int msdc_ops_get_cd(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	/* unsigned long flags; */
	int level = 0;

	/* spin_lock_irqsave(&host->lock, flags); */

	/* for sdio, depends on USER_RESUME */
	if (is_card_sdio(host) && !(host->hw->flags & MSDC_SDIO_IRQ)) {
		host->card_inserted =
			(host->pm_state.event == PM_EVENT_USER_RESUME) ? 1 : 0;
		goto end;
	}

	/* for emmc, MSDC_REMOVABLE not set, always return 1 */
	if (mmc->caps & MMC_CAP_NONREMOVABLE) {
		host->card_inserted = 1;
		goto end;
	} else {
#ifdef CONFIG_GPIOLIB
		level = __gpio_get_value(cd_gpio);
#else
		ERR_MSG("Cannot get gpio %d level", cd_gpio);
#endif
		host->card_inserted = (host->hw->cd_level == level) ? 1 : 0;
	}

	if (host->block_bad_card)
		host->card_inserted = 0;
 end:
	/* enable msdc register dump */
	sd_register_zone[host->id] = 1;
	INIT_MSG("Card insert<%d> Block bad card<%d>",
		host->card_inserted, host->block_bad_card);
	/* spin_unlock_irqrestore(&host->lock, flags); */

	return host->card_inserted;
}

static void msdc_ops_card_event(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	host->block_bad_card = 0;
	host->is_autok_done = 0;
	msdc_ops_get_cd(mmc);
	/* when detect card, cmd13 will be sent which timeout log is not needed */
	sd_register_zone[host->id] = 0;
}

/* ops.enable_sdio_irq */
static void msdc_ops_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct msdc_hw *hw = host->hw;
	void __iomem *base = host->base;
	unsigned long flags;

	if (hw->flags & MSDC_EXT_SDIO_IRQ) {    /* yes for sdio */
		if (enable)
			hw->enable_sdio_eirq(); /* combo_sdio_enable_eirq */
		else
			hw->disable_sdio_eirq(); /* combo_sdio_disable_eirq */
	} else if (hw->flags & MSDC_SDIO_IRQ) {

#if (MSDC_DATA1_INT == 1)
		spin_lock_irqsave(&host->sdio_irq_lock, flags);

		if (enable) {
			while (1) {
				MSDC_SET_BIT32(MSDC_INTEN, MSDC_INT_SDIOIRQ);
				pr_debug("@#0x%08x @e >%d<\n",
					(MSDC_READ32(MSDC_INTEN)),
					host->mmc->sdio_irq_pending);
				if ((MSDC_READ32(MSDC_INTEN) & MSDC_INT_SDIOIRQ)
					== 0) {
					pr_debug("Should never ever get into this >%d<\n",
						host->mmc->sdio_irq_pending);
				} else {
					break;
				}
			}
		} else {
			MSDC_CLR_BIT32(MSDC_INTEN, MSDC_INT_SDIOIRQ);
			pr_debug("@#0x%08x @d\n", (MSDC_READ32(MSDC_INTEN)));
		}

		spin_unlock_irqrestore(&host->sdio_irq_lock, flags);
#endif
	}
}

static int msdc_ops_switch_volt(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	int err = 0;
	u32 timeout = 100;
	u32 retry = 10;
	u32 status;

	if (host->hw->host_function == MSDC_EMMC)
		return 0;

	if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330) {
		/* make sure SDC is not busy (TBC) */
		/* WAIT_COND(!SDC_IS_BUSY(), timeout, timeout); */
		err = (unsigned int)-EILSEQ;
		msdc_retry(sdc_is_busy(), retry, timeout, host->id);
		if (timeout == 0 && retry == 0) {
			err = (unsigned int)-ETIMEDOUT;
			pr_err("msdc%d sdc busy timeout 100ms\n", host->id);
			goto out;
		}

		/* pull up disabled in CMD and DAT[3:0]
		   to allow card drives them to low */
		/* check if CMD/DATA lines both 0 */
		if ((MSDC_READ32(MSDC_PS) & ((1 << 24) | (0xF << 16))) == 0) {
			/* pull up disabled in CMD and DAT[3:0] */
			msdc_pin_config(host, MSDC_PIN_PULL_NONE);

			if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {

				if (host->power_switch)
					host->power_switch(host, 1);
			}
			/* wait at least 5ms for card to switch to 1.8v signal*/
			mdelay(10);

			/* config clock to 10~12MHz mode for
			   volt switch detection by host. */

			/*For FPGA 13MHz clock, this not work*/
			msdc_set_mclk(host, MMC_TIMING_LEGACY, 260000);

			/* pull up enabled in CMD and DAT[3:0] */
			msdc_pin_config(host, MSDC_PIN_PULL_UP);
			mdelay(105);

			/* start to detect volt change
			   by providing 1.8v signal to card */
			MSDC_SET_BIT32(MSDC_CFG, MSDC_CFG_BV18SDT);

			/* wait at max. 1ms */
			mdelay(1);
			/* ERR_MSG("before read status"); */

			while ((status =
				MSDC_READ32(MSDC_CFG)) & MSDC_CFG_BV18SDT)
				;

			if (status & MSDC_CFG_BV18PSS)
				err = 0;
		}
	}
 out:

	return err;
}

static int msdc_card_busy(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	u32 status = MSDC_READ32(MSDC_PS);

	/* check if any pin between dat[0:3] is low */
	if (((status >> 16) & 0xf) != 0xf)
		return 1;

	return 0;
}

/* Add this function to check if no interrupt back after write.         *
 * It may occur when write crc revice, but busy over data->timeout_ns   */
static void msdc_check_write_timeout(struct work_struct *work)
{
	struct msdc_host *host =
		container_of(work, struct msdc_host, write_timeout.work);
	void __iomem *base = host->base;
	struct mmc_data  *data = host->data;
	struct mmc_request *mrq = host->mrq;
	struct mmc_host *mmc = host->mmc;
	u32 status = 0;
	u32 state = 0;
	u32 err = 0;
	unsigned long tmo;
	u32 intsts;

	if (!data || !mrq || !mmc)
		return;
	pr_err("[%s]: XXX DMA Data Write Busy Timeout: %u ms, CMD<%d>",
		__func__, host->write_timeout_ms, mrq->cmd->opcode);

	intsts = MSDC_READ32(MSDC_INT);
	/* MSDC have received int, but delay by system. Just print warning */
	if (intsts) {
		pr_err("[%s]: Warning msdc%d ints are delayed by system, ints: %x\n",
			__func__, host->id, intsts);
		msdc_dump_info(host->id);
		return;
	}

	if (msdc_use_async_dma(data->host_cookie) && (host->need_tune == TUNE_NONE)) {
		msdc_dump_info(host->id);

		msdc_dma_stop(host);
		msdc_dma_clear(host);
		msdc_reset_hw(host->id);

		tmo = jiffies + POLLING_BUSY;

		/* check the card state, try to bring back to trans state */
		spin_lock(&host->lock);
		do {
			/* if anything wrong, let block driver do error
			   handling. */
			err = msdc_get_card_status(mmc, host, &status);
			if (err) {
				ERR_MSG("CMD13 ERR<%d>", err);
				break;
			}

			state = R1_CURRENT_STATE(status);
			ERR_MSG("check card state<%d>", state);
			if (state == R1_STATE_DATA || state == R1_STATE_RCV) {
				ERR_MSG("state<%d> need cmd12 to stop", state);
				msdc_send_stop(host);
			} else if (state == R1_STATE_PRG) {
				ERR_MSG("state<%d> card is busy", state);
				spin_unlock(&host->lock);
				msleep(100);
				spin_lock(&host->lock);
			}

			if (time_after(jiffies, tmo)) {
				ERR_MSG("card stuck in %d state, remove such bad card!" , state);
				spin_unlock(&host->lock);
				if (host->hw->host_function == MSDC_SD)
					msdc_set_bad_card_and_remove(host);
				spin_lock(&host->lock);
				break;
			}
		} while (state != R1_STATE_TRAN);
		spin_unlock(&host->lock);

		data->error = (unsigned int)-ETIMEDOUT;
		host->sw_timeout++;

		if (mrq->done)
			mrq->done(mrq);

		msdc_gate_clock(host, 1);
		host->error |= REQ_DAT_ERR;
	} else {
		/* do nothing, since legacy mode or async tuning
		   have it own timeout. */
		/* complete(&host->xfer_done); */
	}
}

static struct mmc_host_ops mt_msdc_ops = {
	.post_req                      = msdc_post_req,
	.pre_req                       = msdc_pre_req,
	.request                       = msdc_ops_request,
	.set_ios                       = msdc_ops_set_ios,
	.get_ro                        = msdc_ops_get_ro,
	.get_cd                        = msdc_ops_get_cd,
	.card_event                    = msdc_ops_card_event,
	.enable_sdio_irq               = msdc_ops_enable_sdio_irq,
	.start_signal_voltage_switch   = msdc_ops_switch_volt,
	.execute_tuning                = msdc_execute_tuning,
	.hw_reset                      = msdc_card_reset,
	.card_busy                     = msdc_card_busy,
};

static void msdc_irq_data_complete(struct msdc_host *host,
	struct mmc_data *data, int error)
{
	void __iomem *base = host->base;
	struct mmc_request *mrq;

	if ((msdc_use_async_dma(data->host_cookie)) &&
	    (!host->tuning_in_progress)) {
		msdc_dma_stop(host);
		if (error) {
			msdc_clr_fifo(host->id);
			msdc_clr_int();
		}

		mrq = host->mrq;
		msdc_dma_clear(host);

		if (error) {
			if (mrq->data->flags & MMC_DATA_WRITE) {
				host->error |= REQ_CRC_STATUS_ERR;
				host->need_tune = TUNE_ASYNC_DATA_WRITE;
			} else {
				host->error |= REQ_DAT_ERR;
				host->need_tune = TUNE_ASYNC_DATA_READ;
			}
			host->err_cmd = mrq->cmd->opcode;
			/* FIXME: return cmd error for retry if data CRC error */
			mrq->cmd->error = (unsigned int)-EILSEQ;
		} else {
			host->error &= ~REQ_DAT_ERR;
			host->need_tune = TUNE_NONE;
			host->reautok_times = 0;
			host->tune_smpl_times = 0;
			host->err_cmd = -1;
			host->prev_cmd_cause_dump = 0;
		}
		if (mrq->done)
			mrq->done(mrq);
		msdc_gate_clock(host, 1);
	} else {
		/* Autocmd12 issued but error, data transfer done INT will not issue,
		* so cmplete is need here
		*/
		complete(&host->xfer_done);
	}
}

static irqreturn_t msdc_irq(int irq, void *dev_id)
{
	struct msdc_host *host = (struct msdc_host *)dev_id;
	struct mmc_data *data = host->data;
	struct mmc_command *cmd = host->cmd;
	struct mmc_command *stop = NULL;
	void __iomem *base = host->base;
	int ret;

	u32 cmdsts = MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO | MSDC_INT_CMDRDY |
		     MSDC_INT_ACMDCRCERR | MSDC_INT_ACMDTMO | MSDC_INT_ACMDRDY |
		     MSDC_INT_ACMD19_DONE;
	u32 datsts = MSDC_INT_DATCRCERR | MSDC_INT_DATTMO;
	u32 intsts, inten;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	u32 cmdqsts = MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO | MSDC_INT_CMDRDY;
#endif

	if (host->hw->flags & MSDC_SDIO_IRQ)
		spin_lock(&host->sdio_irq_lock);

	if (0 == host->core_clkon) {
		ret = msdc_clk_enable(host);
		if (ret < 0)
			pr_err("host%d enable clock fail\n", host->id);
		host->core_clkon = 1;
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_SDMMC);
	}
	intsts = MSDC_READ32(MSDC_INT);

	latest_int_status[host->id] = intsts;
	inten = MSDC_READ32(MSDC_INTEN);
#if (MSDC_DATA1_INT == 1)
	if (host->hw->flags & MSDC_SDIO_IRQ) {
		intsts &= inten;
	} else
#endif
	{
		inten &= intsts;
	}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	/* don't clear command related interrupt bits,
		these will be cleared by command polling response */
	intsts &= ~cmdqsts;
#endif

	MSDC_WRITE32(MSDC_INT, intsts); /* clear interrupts */

	/* sdio interrupt */
	if (host->hw->flags & MSDC_SDIO_IRQ) {
		spin_unlock(&host->sdio_irq_lock);

		#if (MSDC_DATA1_INT == 1)
		if (intsts & MSDC_INT_SDIOIRQ)
			mmc_signal_sdio_irq(host->mmc);
		#endif
	}

	/* transfer complete interrupt */
	if (data == NULL)
		goto skip_data_interrupts;

#ifdef MTK_MSDC_ERROR_TUNE_DEBUG
	msdc_error_tune_debug2(host, stop, &intsts);
#endif

	stop = data->stop;
#if (MSDC_DATA1_INT == 1)
	if ((host->hw->flags & MSDC_SDIO_IRQ) && (intsts & MSDC_INT_XFER_COMPL))
		goto done;
#endif

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	host->mmc->is_data_dma = 0;
#endif

	if (intsts & MSDC_INT_XFER_COMPL) {
#ifdef MTK_MSDC_DMA_RETRY
		if (host->chip_hw_ver == 0xca00)
			msdc_wr_check(host);
#endif
		goto done;
	}

	if (intsts & datsts) {
		/* do basic reset, or stop command will sdc_busy */
		if (intsts & MSDC_INT_DATTMO)
			msdc_dump_info(host->id);

		if (host->dma_xfer)
			msdc_reset(host->id);
		else
			msdc_reset_hw(host->id);

		atomic_set(&host->abort, 1);    /* For PIO mode exit */

		if (intsts & MSDC_INT_DATTMO) {
			data->error = (unsigned int)-ETIMEDOUT;
			ERR_MSG("XXX CMD<%d> Arg<0x%.8x> MSDC_INT_DATTMO",
				host->mrq->cmd->opcode, host->mrq->cmd->arg);
		} else if (intsts & MSDC_INT_DATCRCERR) {
			data->error = (unsigned int)-EILSEQ;
			ERR_MSG("XXX CMD<%d> Arg<0x%.8x> MSDC_INT_DATCRCERR, SDC_DCRC_STS<0x%x>",
				host->mrq->cmd->opcode, host->mrq->cmd->arg,
				MSDC_READ32(SDC_DCRC_STS));
			if (host->hw->host_function == MSDC_SDIO)
				msdc_dump_info(host->id);
			if (vcorefs_get_hw_opp() == OPPI_PERF)
				pr_err("%s: Vcore 0.8V\n", __func__);
			else
				pr_err("%s: Vcore 0.7V\n", __func__);
		}

		goto tune;

	}

	if ((stop != NULL) &&
	    (host->autocmd & MSDC_AUTOCMD12) &&
	    (intsts & cmdsts)) {
		if (intsts & MSDC_INT_ACMDRDY) {
			u32 *arsp = &stop->resp[0];
			*arsp = MSDC_READ32(SDC_ACMD_RESP);
		} else if (intsts & MSDC_INT_ACMDCRCERR) {
			stop->error = (unsigned int)-EILSEQ;
			host->error |= REQ_STOP_EIO;
			if (host->dma_xfer)
				msdc_reset(host->id);
			else
				msdc_reset_hw(host->id);
		} else if (intsts & MSDC_INT_ACMDTMO) {
			stop->error = (unsigned int)-ETIMEDOUT;
			host->error |= REQ_STOP_TMO;
			if (host->dma_xfer)
				msdc_reset(host->id);
			else
				msdc_reset_hw(host->id);
		}
		if ((intsts & MSDC_INT_ACMDCRCERR) ||
		    (intsts & MSDC_INT_ACMDTMO)) {
			goto tune;
		}
	}

skip_data_interrupts:

	/* command interrupts */
	if ((cmd == NULL) || !(intsts & cmdsts))
		goto skip_cmd_interrupts;

#ifdef MTK_MSDC_ERROR_TUNE_DEBUG
	msdc_error_tune_debug3(host, cmd, &intsts);
#endif

#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (intsts & MSDC_INT_CMDRDY) {
		u32 *rsp = NULL;

		rsp = &cmd->resp[0];
		switch (host->cmd_rsp) {
		case RESP_NONE:
			break;
		case RESP_R2:
			*rsp++ = MSDC_READ32(SDC_RESP3);
			*rsp++ = MSDC_READ32(SDC_RESP2);
			*rsp++ = MSDC_READ32(SDC_RESP1);
			*rsp++ = MSDC_READ32(SDC_RESP0);
			break;
		default: /* Response types 1, 3, 4, 5, 6, 7(1b) */
			*rsp = MSDC_READ32(SDC_RESP0);
			break;
		}
	} else if (intsts & MSDC_INT_RSPCRCERR) {
		cmd->error = (unsigned int)-EILSEQ;
		ERR_MSG("XXX CMD<%d> MSDC_INT_RSPCRCERR Arg<0x%.8x>",
			cmd->opcode, cmd->arg);
		msdc_reset_hw(host->id);
	} else if (intsts & MSDC_INT_CMDTMO) {
		cmd->error = (unsigned int)-ETIMEDOUT;
		ERR_MSG("XXX CMD<%d> MSDC_INT_CMDTMO Arg<0x%.8x>",
			cmd->opcode, cmd->arg);
		msdc_reset_hw(host->id);
	}
	if (intsts & (MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO))
		complete(&host->cmd_done);
#endif

skip_cmd_interrupts:
	/* mmc irq interrupts */
	if (intsts & MSDC_INT_MMCIRQ) {
		/* pr_debug("msdc[%d] MMCIRQ: SDC_CSTS=0x%.8x\r\n",
			host->id, MSDC_READ32(SDC_CSTS)); */
	}

	if (host->dma_xfer)
		msdc_irq_data_complete(host, data, 1);

	latest_int_status[host->id] = 0;
	return IRQ_HANDLED;

done:   /* Finished data transfer */
	data->bytes_xfered = host->dma.xfersz;
	msdc_irq_data_complete(host, data, 0);
	return IRQ_HANDLED;

tune:   /* DMA DATA transfer crc error */
	/* PIO mode can't do complete, because not init */
	if (host->dma_xfer)
		msdc_irq_data_complete(host, data, 1);

	if (data && !data->error)
		host->prev_cmd_cause_dump = 0;

	return IRQ_HANDLED;
}

/* init gpd and bd list in msdc_drv_probe */
static void msdc_init_gpd_bd(struct msdc_host *host, struct msdc_dma *dma)
{
	struct gpd_t *gpd = dma->gpd;
	struct bd_t *bd = dma->bd;
	struct bd_t *ptr, *prev;
	dma_addr_t dma_addr;

	/* we just support one gpd */
	int bdlen = MAX_BD_PER_GPD;

	/* init the 2 gpd */
	memset(gpd, 0, sizeof(struct gpd_t) * 2);
	dma_addr = dma->gpd_addr + sizeof(struct gpd_t);
	gpd->nexth4 = (u32) ((dma_addr >> 32) & 0xF);
	gpd->next = (u32) (dma_addr & 0xFFFFFFFFUL);

	/* gpd->intr = 0; */
	gpd->bdp = 1;           /* hwo, cs, bd pointer */
	/* gpd->ptr  = (void*)virt_to_phys(bd); */
	dma_addr = dma->bd_addr;
	gpd->ptrh4 = (u32) ((dma_addr >> 32) & 0xF); /* physical address */
	gpd->ptr = (u32) (dma_addr & 0xFFFFFFFFUL); /* physical address */

	memset(bd, 0, sizeof(struct bd_t) * bdlen);
	ptr = bd + bdlen - 1;
	while (ptr != bd) {
		prev = ptr - 1;
		dma_addr = dma->bd_addr + sizeof(struct bd_t) * (ptr - bd);
		prev->nexth4 = (u32) ((dma_addr >> 32) & 0xF);
		prev->next = (u32) (dma_addr & 0xFFFFFFFFUL);
		ptr = prev;
	}
}

/* This is called by run_timer_softirq */
static void msdc_timer_pm(unsigned long data)
{
	struct msdc_host *host = (struct msdc_host *)data;
	unsigned long flags;

	spin_lock_irqsave(&host->clk_gate_lock, flags);
	if (host->clk_gate_count == 0) {
		msdc_clksrc_onoff(host, 0);
		N_MSG(CLK, "time out, dsiable clock, clk_gate_count=%d",
			host->clk_gate_count);
	}

	spin_unlock_irqrestore(&host->clk_gate_lock, flags);
}

/* FIX ME : consider if this function can be moved to msdc_io.c */
static void msdc_set_host_power_control(struct msdc_host *host)
{
	if (MSDC_EMMC == host->hw->host_function) {
		host->power_control = msdc_emmc_power;
	} else if (MSDC_SD == host->hw->host_function) {
		host->power_control = msdc_sd_power;
		host->power_switch = msdc_sd_power_switch;
	} else if (MSDC_SDIO == host->hw->host_function) {
		host->power_control = msdc_sdio_power;
	}

	if (host->power_control != NULL) {
		msdc_power_calibration_init(host);
	} else {
		ERR_MSG("Host function defination error for msdc%d", host->id);
		BUG();
	}
}

void SRC_trigger_signal(int i_on)
{
	if ((ghost != NULL) && (ghost->hw->flags & MSDC_SDIO_IRQ)) {
		pr_debug("msdc2 SRC_trigger_signal %d\n", i_on);
		src_clk_control = i_on;
		if (src_clk_control) {
			msdc_clksrc_onoff(ghost, 1);
			/* mb(); */
			if (ghost->mmc->sdio_irq_thread &&
			    (atomic_read(&ghost->mmc->sdio_irq_thread_abort)
				== 0)) {/* if (ghost->mmc->sdio_irq_thread) */
				mmc_signal_sdio_irq(ghost->mmc);
				if (u_msdc_irq_counter < 3)
					pr_debug("msdc2 SRC_trigger_signal mmc_signal_sdio_irq\n");
			}
			/* pr_debug("msdc2 SRC_trigger_signal ghost->id=%d\n",
				ghost->id); */
		}
	}
}
EXPORT_SYMBOL(SRC_trigger_signal);

#ifdef CONFIG_MTK_HIBERNATION
int msdc_drv_pm_restore_noirq(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmc_host *mmc = NULL;
	struct msdc_host *host = NULL;

	BUG_ON(pdev == NULL);
	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);
	if (host->hw->host_function == MSDC_SD) {
		if ((host->id == 1) && !(mmc->caps & MMC_CAP_NONREMOVABLE)) {
#ifdef CONFIG_GPIOLIB
			if ((host->hw->cd_level == __gpio_get_value(cd_gpio))
			 && host->mmc->card) {
				mmc_card_set_removed(host->mmc->card);
				host->card_inserted = 0;
			}
#endif
		} else if ((host->id == 2) &&
			   !(mmc->caps & MMC_CAP_NONREMOVABLE)) {
			/* sdio need handle here */
		}
		host->block_bad_card = 0;
	}

	return 0;
}
#endif

static void msdc_deinit_hw(struct msdc_host *host)
{
	void __iomem *base = host->base;

	/* Disable and clear all interrupts */
	MSDC_CLR_BIT32(MSDC_INTEN, MSDC_READ32(MSDC_INTEN));
	MSDC_WRITE32(MSDC_INT, MSDC_READ32(MSDC_INT));

	/* make sure power down */
	msdc_set_power_mode(host, MMC_POWER_OFF);
}

static void msdc_add_host(struct work_struct *work)
{
	int ret;
	struct msdc_host *host = NULL;
	struct mmc_host *mmc = NULL;

	host = container_of(work, struct msdc_host, work_init.work);
	BUG_ON(!host || !host->mmc);
	mmc = host->mmc;

	ret = mmc_add_host(mmc);
	if (ret) {
		free_irq(host->irq, host);
		platform_set_drvdata(host->pdev, NULL);
		msdc_deinit_hw(host);
		kfree(host->hw);
		mmc_free_host(mmc);
	}
}

static int msdc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc = NULL;
	struct msdc_host *host = NULL;
	struct msdc_hw *hw = NULL;
	void __iomem *base = NULL;
	u32 *hclks = NULL;
	int ret = 0;

	/* Allocate MMC host for this device */
	mmc = mmc_alloc_host(sizeof(struct msdc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	ret = msdc_dt_init(pdev, mmc);
	if (ret) {
		mmc_free_host(mmc);
		return ret;
	}

	host = mmc_priv(mmc);
	base = host->base;
	hw = host->hw;

	/* Set host parameters to mmc */
	mmc->ops        = &mt_msdc_ops;
	mmc->f_min      = HOST_MIN_MCLK;
	mmc->f_max      = HOST_MAX_MCLK;
	mmc->ocr_avail  = MSDC_OCR_AVAIL;

	if ((hw->flags & MSDC_SDIO_IRQ) || (hw->flags & MSDC_EXT_SDIO_IRQ))
		mmc->caps |= MMC_CAP_SDIO_IRQ;  /* yes for sdio */
#ifdef MTK_MSDC_USE_CMD23
	if (host->hw->host_function == MSDC_EMMC)
		mmc->caps |= MMC_CAP_ERASE | MMC_CAP_CMD23;
	else
		mmc->caps |= MMC_CAP_ERASE;
#else
	mmc->caps |= MMC_CAP_ERASE;
#endif

	/* If 0  < mmc->max_busy_timeout < cmd.busy_timeout,
	 * R1B will change to R1, host will not detect DAT0 busy,
	 * next CMD may send to eMMC at busy state.
	 */
	mmc->max_busy_timeout = 0;

	/* MMC core transfer sizes tunable parameters */
	mmc->max_segs = MAX_HW_SGMTS;
	/*mmc->max_phys_segs = MAX_PHY_SGMTS;*/
	if (hw->host_function == MSDC_SDIO)
		mmc->max_seg_size  = MAX_SGMT_SZ_SDIO;
	else
		mmc->max_seg_size  = MAX_SGMT_SZ;
	mmc->max_blk_size  = HOST_MAX_BLKSZ;
	mmc->max_req_size  = MAX_REQ_SZ;
	mmc->max_blk_count = MAX_REQ_SZ / 512; /*mmc->max_req_size;*/

	hclks = msdc_get_hclks(pdev->id);

	host->error             = 0;
	host->mclk              = 0;    /* request clock of mmc */
	host->hclk              = hclks[hw->clk_src];
					/* clocksource to msdc */
	host->sclk              = 0;    /* sd/sdio/emmc bus clock */
	host->pm_state          = PMSG_RESUME;
	host->suspend           = 0;
	host->core_clkon        = 0;
	host->clk_gate_count    = 0;
	host->power_mode        = MMC_POWER_OFF;
	host->power_control     = NULL;
	host->power_switch      = NULL;

	host->dma_mask          = DMA_BIT_MASK(33);
	mmc_dev(mmc)->dma_mask  = &host->dma_mask;

#ifndef FPGA_PLATFORM
	/* FIX ME, consider to move it into msdc_io.c */
	if (msdc_get_ccf_clk_pointer(pdev, host))
		return 1;
#endif

	msdc_set_host_power_control(host);
	if ((host->hw->host_function == MSDC_SD) &&
	    !(host->mmc->caps & MMC_CAP_NONREMOVABLE)) {
		/* Since SD card power is default on,
		   it shall be turned off so that removalbe card slot won't
		    keep power when there is no card plugged */
		msdc_sd_power(host, 1); /* turn on first to match HW/SW state*/
		msdc_sd_power(host, 0);
	}
	/*
	 * mmc_rescan if check host->caps & NONREMOVABLE not call host->ops->get_cd
	 * host->card_inserted cat not be set to 1 for eMMC, so set this flag here
	 */
	host->card_inserted = host->mmc->caps & MMC_CAP_NONREMOVABLE ? 1 : 0;
	host->timeout_ns = 0;
	host->timeout_clks = DEFAULT_DTOC * 1048576;

#ifndef MTK_MSDC_USE_CMD23
	if (host->hw->host_function != MSDC_SDIO)
		host->autocmd |= MSDC_AUTOCMD12;
	else
		host->autocmd &= ~MSDC_AUTOCMD12;
#else
	if (host->hw->host_function == MSDC_EMMC) {
		host->autocmd &= ~MSDC_AUTOCMD12;

#if (1 == MSDC_USE_AUTO_CMD23)
		host->autocmd |= MSDC_AUTOCMD23;
#endif

	} else if (host->hw->host_function == MSDC_SD) {
		host->autocmd |= MSDC_AUTOCMD12;
	} else {
		host->autocmd &= ~MSDC_AUTOCMD12;
	}
#endif  /* end of MTK_MSDC_USE_CMD23 */

	host->mrq = NULL;

#ifdef MTK_MSDC_USE_CACHE
	if (host->hw->host_function == MSDC_EMMC)
		msdc_set_cache_quirk(host);
#endif

	host->dma.used_gpd = 0;
	host->dma.used_bd = 0;

	/* using dma_alloc_coherent *//* todo: using 1, for all 4 slots */
	host->dma.gpd = dma_alloc_coherent(&pdev->dev,
			MAX_GPD_NUM * sizeof(struct gpd_t),
			&host->dma.gpd_addr, GFP_KERNEL);
	host->dma.bd = dma_alloc_coherent(&pdev->dev,
			MAX_BD_NUM * sizeof(struct bd_t),
			&host->dma.bd_addr, GFP_KERNEL);
	BUG_ON((!host->dma.gpd) || (!host->dma.bd));
	msdc_init_gpd_bd(host, &host->dma);
	msdc_clock_src[host->id] = hw->clk_src;
	msdc_host_mode[host->id] = mmc->caps;
	msdc_host_mode2[host->id] = mmc->caps2;
	/*for emmc */
	mtk_msdc_host[host->id] = host;
	host->sw_timeout = 0;

	/* for re-autok */
	host->tuning_in_progress = false;
	init_completion(&host->autok_done);
	host->is_autok_done = 0;
	host->need_tune	= TUNE_NONE;
	host->reautok_times = 0;
	host->tune_smpl_times = 0;
	host->timing = 0;
	host->device_status = 0;
	host->err_cmd = -1;
	host->block_bad_card = 0;

#ifdef MTK_MSDC_DMA_RETRY
	host->chip_hw_ver = mt_get_chip_hw_ver();
#endif

	if (host->hw->host_function == MSDC_SDIO)
		wakeup_source_init(&host->trans_lock, "MSDC Transfer Lock");

	INIT_DELAYED_WORK(&host->write_timeout, msdc_check_write_timeout);
	INIT_DELAYED_WORK(&host->work_init, msdc_add_host);
	/*INIT_DELAYED_WORK(&host->remove_card, msdc_remove_card);*/

	spin_lock_init(&host->lock);
	spin_lock_init(&host->clk_gate_lock);
	spin_lock_init(&host->remove_bad_card);
	spin_lock_init(&host->sdio_irq_lock);
	/* init dynamtic timer */
	init_timer(&host->timer);
	/*host->timer.expires = jiffies + HZ;*/
	host->timer.function = msdc_timer_pm;
	host->timer.data = (unsigned long)host;

	ret = request_irq((unsigned int)host->irq, msdc_irq, IRQF_TRIGGER_NONE,
		DRV_NAME, host);
	if (ret)
		goto release;

	MVG_EMMC_SETUP(host);

	if (hw->request_sdio_eirq)
		/* set to combo_sdio_request_eirq() for WIFI */
		/* msdc_eirq_sdio() will be called when EIRQ */
		hw->request_sdio_eirq(msdc_eirq_sdio, (void *)host);

#ifdef CONFIG_PM
	if (hw->register_pm) {/* only for sdio */
		/* function pointer to combo_sdio_register_pm() */
		hw->register_pm(msdc_pm, (void *)host);
		if (hw->flags & MSDC_SYS_SUSPEND) {
			/* will not set for WIFI */
			ERR_MSG("MSDC_SYS_SUSPEND and register_pm both set");
		}
		/* pm not controlled by system but by client. */
		mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
	}
#endif

	if (host->hw->host_function == MSDC_EMMC)
		mmc->pm_flags |= MMC_PM_KEEP_POWER;

	platform_set_drvdata(pdev, mmc);

#ifdef CONFIG_MTK_HIBERNATION
	if (pdev->id == 1)
		register_swsusp_restore_noirq_func(ID_M_MSDC,
			msdc_drv_pm_restore_noirq, &(pdev->dev));
#endif

	/* Config card detection pin and enable interrupts */
	if (!(host->mmc->caps & MMC_CAP_NONREMOVABLE)) {
		MSDC_CLR_BIT32(MSDC_PS, MSDC_PS_CDEN);
		MSDC_CLR_BIT32(MSDC_INTEN, MSDC_INTEN_CDSC);
		MSDC_CLR_BIT32(SDC_CFG, SDC_CFG_INSWKUP);
	}

	/* Use ordered workqueue to reduce msdc moudle init time */
	if (!queue_delayed_work(wq_init, &host->work_init, 0)) {
		pr_err("msdc%d queue delay work failed BUG_ON,[%s]L:%d\n",
			host->id, __func__, __LINE__);
		BUG();
	}

#ifdef MTK_MSDC_BRINGUP_DEBUG
	pr_debug("[%s]: msdc%d, mmc->caps=0x%x, mmc->caps2=0x%x\n",
		__func__, host->id, mmc->caps, mmc->caps2);
	msdc_dump_clock_sts(host);
#endif

	return 0;

release:
	platform_set_drvdata(pdev, NULL);
	msdc_deinit_hw(host);
	pr_err("[%s]: msdc%d init fail release!\n", __func__, host->id);
	kfree(host->hw);
	mmc_free_host(mmc);

	return ret;
}

/* 4 device share one driver, using "drvdata" to show difference */
static int msdc_drv_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;
	struct resource *mem;

	mmc = platform_get_drvdata(pdev);
	BUG_ON(!mmc);

	host = mmc_priv(mmc);
	BUG_ON(!host);

	ERR_MSG("msdc_drv_remove");
#ifndef FPGA_PLATFORM
	/* clock unprepare */
	if (host->clock_control)
		clk_unprepare(host->clock_control);
#endif
	platform_set_drvdata(pdev, NULL);
	mmc_remove_host(host->mmc);
	msdc_deinit_hw(host);

	tasklet_kill(&host->card_tasklet);
	free_irq(host->irq, host);

	dma_free_coherent(NULL, MAX_GPD_NUM * sizeof(struct gpd_t),
		host->dma.gpd, host->dma.gpd_addr);
	dma_free_coherent(NULL, MAX_BD_NUM * sizeof(struct bd_t),
		host->dma.bd, host->dma.bd_addr);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (mem)
		release_mem_region(mem->start, mem->end - mem->start + 1);

	kfree(host->hw);

	mmc_free_host(host->mmc);

	return 0;
}

#ifdef CONFIG_PM
static int msdc_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct msdc_host *host;
	void __iomem *base;

	if (mmc == NULL)
		return 0;

	host = mmc_priv(mmc);
	base = host->base;

	if (state.event == PM_EVENT_SUSPEND) {
		if  (host->hw->flags & MSDC_SYS_SUSPEND) {
			/* will set for card */
			msdc_pm(state, (void *)host);
		} else {
			/* WIFI slot should be off when enter suspend */
			msdc_gate_clock(host, -1);
			if (host->error == -EBUSY) {
				ret = host->error;
				host->error = 0;
			}
		}
	}

	if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
		if (host->clk_gate_count > 0) {
			host->error = 0;
			return -EBUSY;
		}
		if (host->saved_para.suspend_flag == 0) {
			host->saved_para.hz = host->mclk;
			if (host->saved_para.hz) {
				host->saved_para.suspend_flag = 1;
				/* mb(); */
				msdc_ungate_clock(host);
				msdc_save_timing_setting(host, 2);
				msdc_gate_clock(host, 0);
				if (host->error == -EBUSY) {
					ret = host->error;
					host->error = 0;
				}
			}
			ERR_MSG("msdc suspend cur_cfg=%x, save_cfg=%x, cur_hz=%d",
				MSDC_READ32(MSDC_CFG),
				host->saved_para.msdc_cfg, host->mclk);
		}
	}
	return ret;
}

static int msdc_drv_resume(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct msdc_host *host = mmc_priv(mmc);
	struct pm_message state;

	if (host->hw->flags & MSDC_SDIO_IRQ)
		pr_err("msdc msdc_drv_resume\n");
	state.event = PM_EVENT_RESUME;
	if (mmc && (host->hw->flags & MSDC_SYS_SUSPEND)) {
		/* will set for card;
		   WIFI not controller by PM */
		msdc_pm(state, (void *)host);
	}

	/* This mean WIFI not controller by PM */
	if (host->hw->host_function == MSDC_SDIO) {
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
		host->mmc->rescan_entered = 0;
	}

	return 0;
}
#endif

static struct platform_driver mt_msdc_driver = {
	.probe = msdc_drv_probe,
	.remove = msdc_drv_remove,
#ifdef CONFIG_PM
	.suspend = msdc_drv_suspend,
	.resume = msdc_drv_resume,
#endif
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msdc_of_ids,
	},
};

/*--------------------------------------------------------------------------*/
/* module init/exit                                                         */
/*--------------------------------------------------------------------------*/
static int __init mt_msdc_init(void)
{
	int ret;

	/* Alloc init workqueue */
	wq_init = alloc_ordered_workqueue("msdc-init", 0);
	if (!wq_init) {
		pr_err("msdc create work_queue failed.[%s]:%d", __func__, __LINE__);
		BUG();
	}

	ret = platform_driver_register(&mt_msdc_driver);
	if (ret) {
		pr_err(DRV_NAME ": Can't register driver");
		return ret;
	}

#ifdef CONFIG_PWR_LOSS_MTK_TEST
	msdc_proc_emmc_create();
#endif
	msdc_debug_proc_init();

	pr_debug(DRV_NAME ": MediaTek MSDC Driver\n");

	return 0;
}

static void __exit mt_msdc_exit(void)
{
	platform_driver_unregister(&mt_msdc_driver);

	if (wq_init) {
		destroy_workqueue(wq_init);
		wq_init = NULL;
	}

#ifdef CONFIG_MTK_HIBERNATION
	unregister_swsusp_restore_noirq_func(ID_M_MSDC);
#endif
}
module_init(mt_msdc_init);
module_exit(mt_msdc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek SD/MMC Card Driver");
