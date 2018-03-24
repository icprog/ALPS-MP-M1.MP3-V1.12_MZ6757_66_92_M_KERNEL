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

#ifndef _MT_POWER_GS_V1_H
#define _MT_POWER_GS_V1_H

void mt_power_gs_dump_suspend(void);
void mt_power_gs_dump_dpidle(void);
void mt_power_gs_dump_sodi3(void);

void __weak mt_power_gs_dump_suspend(void) { }
void __weak mt_power_gs_dump_dpidle(void) { }
void __weak mt_power_gs_dump_sodi3(void) { }

extern bool slp_chk_golden_suspend;
extern bool slp_chk_golden_dpidle;
extern bool slp_chk_golden_sodi3;

#endif /* ifndef _MT_POWER_GS_V1_H */
