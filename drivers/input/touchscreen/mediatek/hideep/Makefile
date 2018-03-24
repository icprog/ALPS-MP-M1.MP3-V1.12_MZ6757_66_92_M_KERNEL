#
# Makefile for the HiDeep touchscreen drivers.
#

# Each configuration option enables a list of files.
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
# obj-$(CONFIG_HIDEEP_TS)   += hideep_ts.o
obj-y   += hideep_ts.o
hideep_ts-objs := hideep_core.o hideep_sysfs.o hideep_isp.o hideep_dbg.o hideep_isp_i2c.o

