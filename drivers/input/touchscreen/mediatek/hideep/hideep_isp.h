#ifndef _LINUX_HIDEEP_ISP_H
#define _LINUX_HIDEEP_ISP_H

#define PGM_BURST_WR
#define PGM_VERIFY

/*************************************************************************
 * register map
 *************************************************************************/
#define YRAM_BASE			(0x40000000)
#define PERIPHERAL_BASE		(0x50000000)
#define ESI_BASE			(PERIPHERAL_BASE + 0x00000000)
#define FLASH_BASE			(PERIPHERAL_BASE + 0x01000000)
#define SYSCON_BASE			(PERIPHERAL_BASE + 0x02000000)

#define SYSCON_MOD_CON		(SYSCON_BASE + 0x0000)
#define SYSCON_SPC_CON		(SYSCON_BASE + 0x0004)
#define SYSCON_CLK_CON		(SYSCON_BASE + 0x0008)
#define SYSCON_CLK_ENA		(SYSCON_BASE + 0x000C)
#define SYSCON_RST_CON		(SYSCON_BASE + 0x0010)
#define SYSCON_WDT_CON		(SYSCON_BASE + 0x0014)
#define SYSCON_WDT_CNT		(SYSCON_BASE + 0x0018)
#define SYSCON_PWR_CON		(SYSCON_BASE + 0x0020)
#define SYSCON_PGM_ID		(SYSCON_BASE + 0x00F4)

#define FLASH_CON			(FLASH_BASE + 0x0000)
#define FLASH_STA			(FLASH_BASE + 0x0004)
#define FLASH_CFG			(FLASH_BASE + 0x0008)
#define FLASH_TIM			(FLASH_BASE + 0x000C)
#define FLASH_CACHE_CFG		(FLASH_BASE + 0x0010)

#define ESI_TX_INVALID		(ESI_BASE + 0x0008)

/*************************************************************************
 * flash commands
 *************************************************************************/
#define MERASE				(0x00010000)
#define SERASE				(0x00020000)
#define PERASE				(0x00040000)
#define PROG				(0x00080000)
#define WRONLY				(0x00100000)
#define INF					(0x00200000)

#define CONFIG_LIME_TS
#ifdef CONFIG_CRIMSON_TS
#define NVM_MASK0 0x27270698
#define NVM_MASK1 0x0E5203FF
#define NVM_MASK2 0xFC623800
#define NVM_MASK3 0x00310000
#endif
#ifdef CONFIG_LIME_TS
#define NVM_MASK0 0x28170EA0
#define NVM_MASK1 0x0A0E03FF
#define NVM_MASK2 0x8C203D0C
#define NVM_MASK3 0x0030027B
#endif

#define NVM_PAGE_SIZE		(128)

struct pgm_packet {
	union {
		u8 b[8];
		u32 w[2];
	} header;

	u32 payload[NVM_PAGE_SIZE / sizeof(u32)];
};

#ifdef PGM_BURST_WR
int hideep_pgm_w_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len);
int hideep_pgm_r_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len);
#endif
int hideep_pgm_w_reg(struct i2c_client *client, u32 addr, u32 data);
int hideep_pgm_r_reg(struct i2c_client *client, u32 addr, u32 *val);
int hideep_load_dwz(struct hideep_t *ts);
void hideep_sw_reset(struct i2c_client *client, u32 food);

#endif /* _LINUX_HIDEEP_ISP_H */
