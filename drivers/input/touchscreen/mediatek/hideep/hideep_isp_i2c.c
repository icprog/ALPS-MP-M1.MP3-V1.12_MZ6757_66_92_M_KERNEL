#include "hideep.h"
#include "hideep_isp.h"

int hideep_pgm_r_reg(struct i2c_client *client, u32 addr, u32 *val)
{
	int ret = 0;
	u32 packet[3];
	u8 *bulk = (u8 *)packet + 3;

	packet[0] = htonl(0x00);
	packet[1] = htonl(addr);

	ret = i2c_master_send(client, bulk, 5);
	if (ret < 0)
		goto err;

	mdelay(1);
	ret = i2c_master_recv(client, (u8 *)&(packet[2]), 4);

	if (ret < 0)
		goto err;

	*val = ntohl(packet[2]);

err:
	return ret;
}

int hideep_pgm_w_reg(struct i2c_client *client, u32 addr, u32 data)
{
	int ret = 0;
	u32 packet[3];
	u8 *bulk = (u8 *)packet + 3;

	packet[0] = htonl(0x80);
	packet[1] = htonl(addr);
	packet[2] = htonl(data);

    /* i2c_master_send */
	ret = i2c_master_send(client, bulk+0, 5);

	if (ret < 0)
		goto err;

	ret = i2c_master_send(client, bulk+5, 4);

	if (ret < 0)
		goto err;

err:
	return ret;
}

#ifdef PGM_BURST_WR
int hideep_pgm_w_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len)
{
	int ret = 0;
	int i;

	if ((len % 4) != 0)
		return -1;

	packet->header.w[0] = htonl((0x80 | (len / 4-1)));
	packet->header.w[1] = htonl(addr);

	for (i = 0; i < NVM_PAGE_SIZE / sizeof(u32); i++)
		packet->payload[i] = htonl(packet->payload[i]);

	ret = i2c_master_send(client, (u8 *)&packet->header.b[3], (len+5));

	if (ret < 0)
		goto err;

err:
	return ret;
}

int hideep_pgm_r_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len)
{
	int ret = 0;
	int i;

	if ((len % 4) != 0)
		return -1;

	packet->header.w[0] = htonl((0x00 | (len / 4-1)));
	packet->header.w[1] = htonl(addr);

	ret = i2c_master_send(client, (u8 *)&packet->header.b[3], 5);

	if (ret < 0)
		goto err;

	ret = i2c_master_recv(client, (u8 *)packet->payload, len);

	if (ret < 0)
		goto err;

	for (i = 0; i < NVM_PAGE_SIZE / sizeof(u32); i++)
		packet->payload[i] = htonl(packet->payload[i]);

err:
	return ret;
}
#endif

void hideep_sw_reset(struct i2c_client *client, u32 food)
{
	hideep_pgm_w_reg(client, SYSCON_WDT_CNT, food);
	hideep_pgm_w_reg(client, SYSCON_WDT_CON, 0x03);
	hideep_pgm_w_reg(client, SYSCON_WDT_CON, 0x01);

	HIDEEP_INFO("sw reset");
}

int hideep_load_dwz(struct hideep_t *ts)
{
	int ret = 0;
	int retry = 4;
	struct pgm_packet packet_r;

	while (retry--) {
		HIDEEP_INFO("enter_pgm : %d", retry);
		ret = hideep_enter_pgm(ts->client);
		if (ret >= 0)
			break;
	}

	if (retry <= 0) {
		HIDEEP_ERR("dwz enter_pgm : failed");
		return -1;
	}

	mdelay(50);

	ret = hideep_pgm_r_mem(ts->client, HIDEEP_DWZ_ADDR, &packet_r, sizeof(struct dwz_info_t));
	if (ret < 0) {
		HIDEEP_ERR("i2c failed");
		goto i2c_err;
	}

	memcpy((u8 *)ts->dwz_info, packet_r.payload, sizeof(struct dwz_info_t));
	hideep_sw_reset(ts->client, 10);

	HIDEEP_INFO("firmware boot version   : %04x", ts->dwz_info->ver_b);
	HIDEEP_INFO("firmware core version   : %04x", ts->dwz_info->ver_c);
	HIDEEP_INFO("firmware custom version : %04x", ts->dwz_info->ver_d);
	HIDEEP_INFO("firmware vr version     : %04x", ts->dwz_info->ver_v);

	mdelay(50);

	return 0;
i2c_err:
	return -1;
}
