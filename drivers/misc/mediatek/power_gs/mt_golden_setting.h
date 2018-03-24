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

#ifndef _MT_GOLDEN_SETTING_H
#define _MT_GOLDEN_SETTING_H

int snapshot_golden_setting(const char *func, const unsigned int line);

int __weak snapshot_golden_setting(const char *func, const unsigned int line) { return 0; }

#endif /* ifndef _MT_GOLDEN_SETTING_H */
