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

#ifndef __MT6337_IRQ_H
#define __MT6337_IRQ_H

typedef enum {
	INT_THR_H,
	INT_THR_L,
	INT_AUDIO,
	INT_MAD,
	INT_ACCDET,
	INT_ACCDET_EINT,
	INT_ACCDET_EINT1,
	INT_ACCDET_NEGV,
	INT_PMU_THR,
	INT_LDO_VA18_OC,
	INT_LDO_VA25_OC,
	INT_LDO_VA18_PG
} MT6337_IRQ_ENUM;

/* pmic irq extern variable */
extern struct task_struct *mt6337_thread_handle;
#if !defined CONFIG_HAS_WAKELOCKS
extern struct wakeup_source pmicThread_lock_mt6337;
#else
extern struct wake_lock pmicThread_lock_mt6337;
#endif
extern struct mt6337_interrupts sub_interrupts[];
/* pmic irq extern functions */
extern void MT6337_EINT_SETTING(void);
extern int mt6337_thread_kthread(void *x);

extern void mt6337_enable_interrupt(MT6337_IRQ_ENUM intNo, unsigned int en, char *str);
extern void mt6337_mask_interrupt(MT6337_IRQ_ENUM intNo, char *str);
extern void mt6337_unmask_interrupt(MT6337_IRQ_ENUM intNo, char *str);
extern void mt6337_register_interrupt_callback(MT6337_IRQ_ENUM intNo, void (EINT_FUNC_PTR) (void));

#endif /*--MT6337_IRQ_H--*/
