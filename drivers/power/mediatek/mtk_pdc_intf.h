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

#ifndef __MTK_PDC_INTF_H
#define __MTK_PDC_INTF_H

#ifdef CONFIG_MTK_POWER_DELIVERY_CHARGING_SUPPORT
extern bool mtk_pdc_check_charger(void);
extern void mtk_pdc_plugout_reset(void);
extern void mtk_pdc_init_table(void);
extern void mtk_pdc_init(void);
#else
static inline  bool mtk_pdc_check_charger(void)
{
	return false;
}
static inline  void mtk_pdc_plugout_reset(void)
{}
static inline  void mtk_pdc_init_table(void)
{}
static inline  void mtk_pdc_init(void)
{}

#endif /* CONFIG_MTK_POWER_DELIVERY_CHARGING_SUPPORT */

#endif
