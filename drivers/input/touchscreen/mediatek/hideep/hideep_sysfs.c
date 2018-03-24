/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *******************************************************************************/

#include "hideep.h"

static ssize_t fuse_ucode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep_t *ts_drv = dev_get_drvdata(dev);
	int ret;

	disable_irq(ts_drv->irq);
	ts_drv->manually_update = true;
	ret = hideep_load_ucode(dev, HIDEEP_MAN_FW);

	if (ret) {
		HIDEEP_ERR("The firmware update failed(%d)", ret);
		count = ret;
	}

	ts_drv->manually_update = false;
	enable_irq(ts_drv->irq);

	return count;
}

static ssize_t read_ucode(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct hideep_t *ts = dev_get_drvdata(dev);

	len = scnprintf(buf, PAGE_SIZE, "%d\n", ts->dwz_info->ver_c);
	return len;
}

static ssize_t loglevel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len;

	HIDEEP_INFO("enter");
	len = scnprintf(buf, PAGE_SIZE, "loglevel = %d, [0~2(normal log), 3(debug), 4(xy), 5(i2c)]\n", loglevel);
	HIDEEP_INFO("loglevel =%d ", loglevel);

	return len;
}

static ssize_t loglevel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep_t *ts = dev_get_drvdata(dev);

	HIDEEP_INFO("enter, loglevel =%d", loglevel);

	mutex_lock(&ts->dev_mutex);
	if (kstrtoint(buf, 10, &loglevel) != 0)
		return -EINVAL;
	mutex_unlock(&ts->dev_mutex);

	return count;
}

static ssize_t r_vr_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 len = 0;
	u32 vr_data = 0;
	struct hideep_t *ts = dev_get_drvdata(dev);

	hideep_i2c_read(ts, ts->vr_addr, ts->vr_size, (u8 *)&vr_data);

	len = scnprintf(buf, PAGE_SIZE, "vr : %d %d(%02x)\n", ts->vr_addr, vr_data, vr_data);
	return len;
}

static ssize_t w_vr_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep_t *ts = dev_get_drvdata(dev);
	u32 vr_addr = 0;
	u32 vr_data = 0;
	u32 vr_size = 0;
	u32 vr_flag = 0;
	int ret;

	ret = sscanf(buf, "%d %d %d %d", &vr_addr, &vr_data, &vr_size, &vr_flag);
	if (ret != 1)
		return -EINVAL;

	if (vr_addr >= HIDEEP_EVENT_COUNT_ADDR)
		return 0;
	if (vr_size >  sizeof(vr_data))
		return 0;

	ts->vr_addr = vr_addr;
	ts->vr_size = vr_size;

	if (vr_flag != 0)
		hideep_i2c_write(ts, vr_addr, vr_size, (u8 *)&vr_data);

	return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 len = 0;
	char *panel_name = "";
	struct hideep_t *ts = dev_get_drvdata(dev);

	hideep_load_dwz(ts);

	HIDEEP_INFO("boot version : %04x", ts->dwz_info->ver_b);
	HIDEEP_INFO("core version : %04x", ts->dwz_info->ver_c);
	HIDEEP_INFO("custom version : %04x", ts->dwz_info->ver_d);
	HIDEEP_INFO("vr version : %04x", ts->dwz_info->ver_v);
	HIDEEP_INFO("factory ID : %02x", ts->dwz_info->factory_id);

	panel_name = "None";

#ifdef HIDEEP_PANEL_INFO
	HIDEEP_INFO("panel : (%d, %d)", ts->dwz_info->panel.dp_w, ts->dwz_info->panel.dp_h);
#endif
	len = scnprintf(buf, PAGE_SIZE,
		"boot ver: %04x\ncore ver: %04x\ncustom ver: %04x\nvr ver: %04x\nfactory ID : %02x\npanel company : %s\nD/D ver : %d.%02d\nDescription : %s\n",
		ts->dwz_info->ver_b, ts->dwz_info->ver_c, ts->dwz_info->ver_d,
		ts->dwz_info->ver_v, ts->dwz_info->factory_id, panel_name, HIDEEP_DD_VERSION_MAJOR,
				HIDEEP_DD_VERSION_MINOR, HIDEEP_DD_DESCRIPTION);
	return len;
}

static ssize_t power_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hideep_t *ts = dev_get_drvdata(dev);
	int len;

	len = scnprintf(buf, PAGE_SIZE, "power status : %s\n", (ts->dev_state == power_init)?"off":"on");

	return len;
}

static ssize_t power_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep_t *ts = dev_get_drvdata(dev);
	int on;

	if (kstrtoint(buf, 10, &on) != 0)
		return -EINVAL;

	if (on) {
		hideep_power(ts, on);
		ts->dev_state = power_normal;
		hideep_reset_ic(ts);
	} else {
		hideep_power(ts, on);
		ts->dev_state = power_init;
	}

	return count;
}

static DEVICE_ATTR(update, 0660, read_ucode, fuse_ucode);
static DEVICE_ATTR(vr_data,	0660, r_vr_data, w_vr_data);
static DEVICE_ATTR(version, 0440, version_show, NULL);
static DEVICE_ATTR(loglevel, 0660,	loglevel_show, loglevel_store);
static DEVICE_ATTR(power_en, 0660, power_status, power_control);

static struct attribute *hideep_ts_sysfs_entries[] = {
	&dev_attr_update.attr,
	&dev_attr_vr_data.attr,
	&dev_attr_version.attr,
	&dev_attr_loglevel.attr,
	&dev_attr_power_en.attr,
	NULL
};

static struct attribute_group hideep_ts_attr_group = {
	.attrs  = hideep_ts_sysfs_entries,
};

int hideep_sysfs_init(struct hideep_t *ts)
{
	int ret;
	struct  i2c_client *client = ts->client;

	/* Create the files associated with this kobject */
	ret = sysfs_create_group(&client->dev.kobj, &hideep_ts_attr_group);

	HIDEEP_INFO("device : %s ", client->dev.kobj.name);
	ret = sysfs_create_link(NULL, &client->dev.kobj, "hideep");

	if (ret)
		HIDEEP_ERR("%s: Fail create link error = %d\n", __func__, ret);

	return ret;
}

int hideep_sysfs_exit(struct hideep_t *ts)
{
	struct  i2c_client *client = ts->client;

	sysfs_remove_link(&client->dev.kobj, "hideep");
	sysfs_remove_group(&client->dev.kobj, &hideep_ts_attr_group);

	return 0;
}
