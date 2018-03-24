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

#ifndef __MTK_PE_30_INTF_H
#define __MTK_PE_30_INTF_H

#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT


struct mtk_pep30_cap {
	int cur;
	int vol;
};


extern bool mtk_pep30_init(void);
extern bool mtk_is_TA_support_pep30(void);
extern bool mtk_is_pep30_running(void);
extern void mtk_pep30_plugout_reset(void);
extern bool mtk_pep30_check_charger(void);
extern void mtk_pep30_set_pd_rdy(bool rdy);
extern bool pep30_enable_jeita(bool en);
extern bool is_pep30_enable_jeita(void);
extern void mtk_pep30_set_charging_current_limit(int cur);
extern int mtk_pep30_get_charging_current_limit(void);


extern void chrdet_int_handler(void);
#else
static inline bool mtk_is_TA_support_pep30(void)
{
	return false;
}

static inline bool mtk_is_pep30_running(void)
{
	return false;
}


static inline void mtk_pep30_plugout_reset(void)
{
}

static inline bool mtk_pep30_init(void)
{
	return false;
}

static inline bool pep30_enable_jeita(bool en)
{
	return true;
}

static inline bool is_pep30_enable_jeita(void)
{
	return false;
}

static inline void mtk_pep30_set_pd_rdy(bool rdy)
{
}

static inline bool mtk_pep30_check_charger(void)
{
	return false;
}

#ifdef THERMAL_READY
static inline void mtk_pep30_set_charging_current_limit(int cur)
{
}

static int mtk_pep30_get_charging_current_limit(void)
{
	return 0;
}
#endif

#endif /* __MTK_PE_30_INTF_H */

#endif
