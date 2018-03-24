#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include

obj-$(CONFIG_MTK_LEDS) += leds_drv.o

ifneq ($(filter y,$(CONFIG_ARCH_MT8127) $(CONFIG_ARCH_MT7623) $(CONFIG_ARCH_MT8590)),)
CONFIG_MTK_PLATFORM := mt8127
ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/$(CONFIG_MTK_PLATFORM)
endif
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

