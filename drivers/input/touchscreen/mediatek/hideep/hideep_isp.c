/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
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
 *******************************************************************************/

#include "hideep.h"
#include "hideep_isp.h"

static struct pgm_packet packet_w;
static struct pgm_packet packet_r;

#define SET_FLASH_PIO(CE)		hideep_pgm_w_reg(client, FLASH_CON, 0x01 | ((CE) << 1))
#define SET_PIO_SIG(X, Y)		hideep_pgm_w_reg(client, (FLASH_BASE + 0x400000) + (X), Y)
#define SET_FLASH_HWCONTROL()	hideep_pgm_w_reg(client, FLASH_CON, 0x00)

#define NVM_DEFAULT_PAGE		0
#define NVM_SFR_WPAGE			1

#define NVM_W_SFR(x, y)	{ \
	SET_FLASH_PIO(1); \
	SET_PIO_SIG(x, y); \
	SET_FLASH_PIO(0); \
}

int hideep_enter_pgm(struct i2c_client *client)
{
	s32 ret = 0;
	u32 status;
	u32 pattern = 0xDF9DAF39;


	i2c_master_send(client, (u8 *)&pattern, 4);
	mdelay(1);

	/* flush invalid Tx load register */
	hideep_pgm_w_reg(client, ESI_TX_INVALID, 0x01);

	hideep_pgm_r_reg(client, SYSCON_PGM_ID, &status);

	HIDEEP_INFO("enter_pgm : return value(0x%08x)", status);

	if (status != htonl(pattern)) {
		HIDEEP_ERR("enter_pgm : error(%08x):", status);
		return -1;
	}

	hideep_pgm_w_reg(client, SYSCON_WDT_CON, 0x00);
	hideep_pgm_w_reg(client, SYSCON_SPC_CON, 0x00);		/* remap */
	hideep_pgm_w_reg(client, SYSCON_CLK_ENA, 0xFF);		/* Clock Enable for YRAM/DEMOD/ACU/MSP/CM0 */
	hideep_pgm_w_reg(client, SYSCON_CLK_CON, 0x01);		/* Select MOSC */
	hideep_pgm_w_reg(client, SYSCON_PWR_CON, 0x01);
	hideep_pgm_w_reg(client, FLASH_TIM, 0x03);
	hideep_pgm_w_reg(client, FLASH_CACHE_CFG, 0x00);	/* cache disable */
	hideep_pgm_w_reg(client, FLASH_CACHE_CFG, 0x02);	/* cache flush.. */

	mdelay(1);

	return ret;
}

static s32 hideep_program_page(struct i2c_client *client, u16 addr, struct pgm_packet *packet_w)
{
	u32 pio_cmd = PROG;
	u32 status;
#ifndef PGM_BURST_WR
	s32 i;
#endif

	hideep_pgm_r_reg(client, FLASH_STA, &status);	/* flash status */
	if (status == 0)
		return -1;

	addr = addr & ~(NVM_PAGE_SIZE - 1);		/* address mask */

	SET_FLASH_PIO(0);
	SET_FLASH_PIO(1);

	SET_PIO_SIG(pio_cmd+addr, htonl(packet_w->payload[0]));

#ifdef PGM_BURST_WR
	hideep_pgm_w_mem(client, (FLASH_BASE + 0x400000) + pio_cmd, packet_w, NVM_PAGE_SIZE);
#else
	for (i = 0; i < NVM_PAGE_SIZE / 4; i++)
		SET_PIO_SIG(pio_cmd + (i<<2), packet_w->payload[i]);
#endif
	SET_PIO_SIG(124, htonl(packet_w->payload[31]));

	SET_FLASH_PIO(0);

	mdelay(1);	/* optimize....... */

	while (1) {
		hideep_pgm_r_reg(client, FLASH_STA, &status);
		if ((status) != 0)
			break;
	}

	hideep_pgm_w_reg(client, FLASH_CON, 0);

	return 0;
}

int hideep_read_trim_data(struct i2c_client *client)
{
	unsigned int tmp;

	hideep_pgm_w_reg(client, FLASH_CFG, 2);
	hideep_pgm_r_reg(client, 0x00000000, (unsigned int *)&tmp);
	tmp = (tmp & 0x0000F000) >> 12;
	hideep_pgm_w_reg(client, FLASH_CFG, 0);

	return tmp;
}

int hideep_nvm_unlock(struct i2c_client *client)
{
	int ret = 0;
	unsigned char trim_data = 0x08;

	trim_data = hideep_read_trim_data(client);

	hideep_pgm_w_reg(client, FLASH_TIM, 3);
	hideep_pgm_w_reg(client, FLASH_CFG, NVM_SFR_WPAGE);
	SET_FLASH_PIO(0);

	NVM_W_SFR(0, NVM_MASK0 | ((trim_data & 0x0f) << 12));
	NVM_W_SFR(4, NVM_MASK1);
	NVM_W_SFR(8, NVM_MASK2);
	NVM_W_SFR(12, NVM_MASK3);
	SET_FLASH_HWCONTROL();
	hideep_pgm_w_reg(client, FLASH_CFG, NVM_DEFAULT_PAGE);

	return ret;
}

static int hideep_program_nvm(struct i2c_client *client, const u8 *ucode, size_t len, int offset)
{
	size_t i;
	s32 ret;
	u32 pages;
	u32 addr;
	s32 retry = 4;
	s32 len_r;
	s32 len_w;

	while (retry--) {
		HIDEEP_DBG("enter_pgm : %d", retry);
		ret = hideep_enter_pgm(client);
		if (ret >= 0)
			break;
	}

	if (retry <= 0) {
		HIDEEP_ERR("enter_pgm : failed");
		return -1;
	}

	hideep_nvm_unlock(client);

	pages = (len + NVM_PAGE_SIZE - 1) / NVM_PAGE_SIZE;
	addr = offset;
	len_r = len;
	len_w = len_r;

	HIDEEP_INFO("pages : %d", pages);
	for (i = 0; i < pages; i++) {
		if (len_r >= NVM_PAGE_SIZE)
			len_w = NVM_PAGE_SIZE;

		memcpy(packet_w.payload, &(ucode[addr]), len_w);

	ret = hideep_program_page(client, i * NVM_PAGE_SIZE + offset, &packet_w);

		if (ret < 0)
			HIDEEP_ERR("hideep_program_nvm : error(%08x):", addr);

		addr += NVM_PAGE_SIZE;
		len_r -= NVM_PAGE_SIZE;
		len_w = len_r;
	}

	return ret;
}

static int hideep_verify_nvm(struct i2c_client *client, const u8 *ucode, size_t len, int offset)
{
	s32 i;
	s32 j;
	s32 ret = 0;
	u32 addr = offset;
	u32 pages = (len + NVM_PAGE_SIZE - 1) / NVM_PAGE_SIZE;
	s32 len_r = len;
	s32 len_v = len_r;

	for (i = 0; i < pages; i++) {
		if (len_r >= NVM_PAGE_SIZE)
			len_v = NVM_PAGE_SIZE;

#ifdef PGM_BURST_WR
		hideep_pgm_r_mem(client, 0x00000000 + addr, &packet_r, NVM_PAGE_SIZE);
#else
		for (j = 0; j < NVM_PAGE_SIZE / 4; j++)
			hideep_pgm_r_reg(client, addr + (j << 2), &(packet_r.payload[j]));
#endif
		ret = memcmp(&(ucode[addr]), packet_r.payload, len_v);

		if (ret != 0) {
			u8 *read = (u8 *)packet_r.payload;

			for (j = 0; j < NVM_PAGE_SIZE; j++)
				HIDEEP_ERR("%02x : %02x", ucode[addr+j], read[j]);

			HIDEEP_ERR("verify : error(addr : %d)", addr);

			ret = -1;
		}

		addr += NVM_PAGE_SIZE;
		len_r -= NVM_PAGE_SIZE;
		len_v = len_r;
	}

	return ret;
}

static void get_dwz_from_binary(unsigned char *pres, struct dwz_info_t *dwz_info)
{
	memcpy(dwz_info, pres + HIDEEP_DWZ_ADDR, sizeof(struct dwz_info_t));
}

static void set_dwz_from_binary(unsigned char *pres, struct dwz_info_t *dwz_info)
{
	memcpy(pres + HIDEEP_DWZ_ADDR, dwz_info, sizeof(struct dwz_info_t));
}

int hideep_fuse_ucode(struct i2c_client *client, u8 *code, size_t len, int offset)
{
	s32 ret;
	s32 retry = 3;
	s16 b, c, d, v;
	int firm_len;
	struct hideep_t *ts = (struct hideep_t *)i2c_get_clientdata(client);
	struct dwz_info_t bin_dwz_info;

	firm_len = len;
#ifdef CONFIG_AULU_Z
	if (firm_len > FIRMWARE_SIZE)
		firm_len = FIRMWARE_SIZE;
#endif

	HIDEEP_INFO("enter");
	if (offset == 0) {
		get_dwz_from_binary(code, &bin_dwz_info);

		b = bin_dwz_info.ver_b;
		c = bin_dwz_info.ver_c;
		d = bin_dwz_info.ver_d;
		v = bin_dwz_info.ver_v;

		/* reset dwz info */
		bin_dwz_info.ver_b = 0;
		bin_dwz_info.ver_c = 0;
		bin_dwz_info.ver_d = 0;
		bin_dwz_info.ver_v = 0;

		set_dwz_from_binary(code, &bin_dwz_info);

		hideep_reset_ic(ts);

		ret = hideep_program_nvm(client, code, firm_len, offset);
		if (ret != 0)
			return ret;

#ifdef PGM_VERIFY
		while (retry--) {
			ret = hideep_verify_nvm(client, code, firm_len, offset);
			if (ret == 0) {
				HIDEEP_INFO("update success");
				break;
			}
			HIDEEP_ERR(" download uc failed(%d)", retry);
		}

		if (retry > 0)
			HIDEEP_INFO("download uc success");
#endif

		/* restore dwz info */
		bin_dwz_info.ver_b = b;
		bin_dwz_info.ver_c = c;
		bin_dwz_info.ver_d = d;
		bin_dwz_info.ver_v = v;

		set_dwz_from_binary(code, &bin_dwz_info);
		hideep_sw_reset(ts->client, 10);
		hideep_reset_ic(ts);

		ret = hideep_program_nvm(client, code, NVM_DWZ_LEN, 0x280);
	} else {
		hideep_reset_ic(ts);

		ret = hideep_program_nvm(client, code, firm_len, offset);
		if (ret != 0)
			return ret;

#ifdef PGM_VERIFY
		while (retry--) {
			ret = hideep_verify_nvm(client, code, firm_len, offset);
			if (ret == 0) {
				HIDEEP_INFO("update success");
				break;
			}
			HIDEEP_ERR(" download uc failed(%d)", retry);
		}

		if (retry > 0)
			HIDEEP_INFO("download uc success");
#endif
	}

	if (ret != 0)
		return ret;

	hideep_sw_reset(client, 1000);	/* enable wdr reset */

	return ret;
}

static int hideep_is_update_necessary(struct hideep_t *ts, unsigned char *pres)
{
	int ver_cmp = 0;
	struct dwz_info_t bin_dwz_info;

	HIDEEP_INFO("enter");

	get_dwz_from_binary(pres, &bin_dwz_info);

	HIDEEP_INFO("binary boot version : %04x", bin_dwz_info.ver_b);
	HIDEEP_INFO("binary core version : %04x", bin_dwz_info.ver_c);
	HIDEEP_INFO("binary custom version : %04x", bin_dwz_info.ver_d);
	HIDEEP_INFO("binary vr version : %04x", bin_dwz_info.ver_v);

	ver_cmp |= (ts->dwz_info->ver_b < bin_dwz_info.ver_b);
	ver_cmp |= ((ts->dwz_info->ver_c < bin_dwz_info.ver_c) << 1);
	ver_cmp |= ((ts->dwz_info->ver_d < bin_dwz_info.ver_d) << 2);
	ver_cmp |= ((ts->dwz_info->ver_v < bin_dwz_info.ver_v) << 3);

	if (ver_cmp != 0)
		HIDEEP_INFO("the version info is not same, FW update necessary : %04x", ver_cmp);

	HIDEEP_INFO("ver_cmp = 0x%x", ver_cmp);
	return ver_cmp;
}

int hideep_load_ucode(struct device *dev, const char *fn)
{
	struct hideep_t *ts_drv = dev_get_drvdata(dev);
	const struct firmware *fw_entry;
	unsigned char *fw_buf;
	unsigned int fw_length;
	int ret;

	HIDEEP_INFO("enter");
	hideep_load_dwz(ts_drv);
	ret = request_firmware(&fw_entry, fn, dev);

	if (ret != 0) {
		HIDEEP_ERR("request_firmware : fail(%d)", ret);
		return ret;
	}

	fw_buf = (unsigned char *)fw_entry->data;
	fw_length = (unsigned int)fw_entry->size;

	ret = hideep_is_update_necessary(ts_drv, fw_buf);

	if ((!ts_drv->manually_update) && (false == ret)) {
		/*
		 * no need to update firmware,
		 * because the version between firmware and binary is the same.
		 */
		HIDEEP_INFO("no need to update.");
		release_firmware(fw_entry);
		return 0;
	}

	/* chip specific code for flash fuse */
	mutex_lock(&ts_drv->dev_mutex);
	mutex_lock(&ts_drv->i2c_mutex);

	ts_drv->dev_state = power_updating;

	ret = hideep_fuse_ucode(ts_drv->client, fw_buf, fw_length, 0);

	ts_drv->dev_state = power_normal;

	mutex_unlock(&ts_drv->i2c_mutex);
	mutex_unlock(&ts_drv->dev_mutex);

	release_firmware(fw_entry);
	return ret;
}
