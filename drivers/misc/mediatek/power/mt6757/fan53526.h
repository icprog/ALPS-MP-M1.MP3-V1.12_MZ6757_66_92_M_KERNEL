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

/*****************************************************************************
*
* Filename:
* ---------
*   fan53526.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   fan53526 header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _fan53526_SW_H_
#define _fan53526_SW_H_

#define fan53526_REG_NUM 6

extern unsigned int g_fan53526_logger;
extern unsigned int g_fan53526_dbgaddr;

#define FAN53526LOGLV     4

#define FAN53526LOGGER(addr, wdata, val, mask, shift) do { \
	if (g_fan53526_logger >= FAN53526LOGLV) \
		if (g_fan53526_dbgaddr == addr) { \
			pr_err("fan53526 addr = 0x%x wdata = 0x%x\n", addr, wdata); \
			pr_err("fan53526 val = 0x%x, mask = 0x%x, shift =0x%x\n", val, mask, shift); \
			WARN_ON(1); \
		} \
} while (0)


extern int is_fan53526_exist(void);
extern int is_fan53526_sw_ready(void);
extern void fan53526_dump_register(void);
extern unsigned int fan53526_read_interface(unsigned char RegNum, unsigned char *val,
					    unsigned char MASK, unsigned char SHIFT);
extern unsigned int fan53526_config_interface(unsigned char RegNum, unsigned char val,
					      unsigned char MASK, unsigned char SHIFT);
/*extern int fan53526_vosel(unsigned long val);*/
extern int fan53526_is_enabled(void);
extern int fan53526_enable(unsigned char en);
extern int fan53526_set_voltage(unsigned long val);
extern int fan53526_set_mode(unsigned char mode);
extern unsigned int fan53526_get_voltage(void);
extern unsigned int fan53526_read_byte(unsigned char cmd, unsigned char *returnData);
extern int is_fan53526_sw_ready(void);
extern int get_fan53526_i2c_ch_num(void);

#endif				/* _fan53526_SW_H_ */
