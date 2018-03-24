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
#include "hideep_dbg.h"
#include <linux/poll.h>

#ifdef HIDEEP_DEBUG_DEVICE

static int hideep_get_vreg(struct hideep_t *ts, u32 addr, u32 len)
{
	int32_t ret = 0;
	struct hideep_debug_dev_t *debug_dev = &ts->debug_dev;

	ret = hideep_i2c_read(ts, addr, len, debug_dev->vr_buff);
	if (ret < 0)
		goto i2c_err;

	HIDEEP_INFO("hideep_get_vreg(0x%02x:%d)", addr, len);
	return ret;

i2c_err:
	HIDEEP_ERR("%s(%d) : i2c_err", __func__, __LINE__);
	return ret;
}

static int hideep_set_vreg(struct hideep_t *ts, u32 addr, u32 len)
{
	int32_t ret = 0;
	struct hideep_debug_dev_t *debug_dev = &ts->debug_dev;
	int32_t wr_remain = len;
	u32 vr_addr = addr;
	int32_t wr_len = len;
	uint8_t *buff = debug_dev->vr_buff;

	do {
		if (wr_remain >=  MAX_VR_BUFF)
			wr_len = MAX_VR_BUFF;
		else
			wr_len = wr_remain;

		ret = hideep_i2c_write(ts, vr_addr, wr_len, buff);
		if (ret < 0)
			goto i2c_err;

		wr_remain -= MAX_VR_BUFF;
		vr_addr += MAX_VR_BUFF;
		buff += MAX_VR_BUFF;
	} while (wr_remain > 0);

	HIDEEP_INFO("hideep_set_vreg(0x%02x:%d)", addr, len);
	return ret;

i2c_err:
	HIDEEP_ERR("i2c_err");
	return ret;
}

static size_t
hideep_download_uc(struct hideep_t *ts, const char __user *uc, size_t count, int offset)
{
	int ret;
	unsigned char *ucode;


	HIDEEP_INFO("%s count = %d", __func__, (int)count);

#ifdef CONFIG_AULU_Z
	len += count;
	len -= FIRMWARE_SIZE;
	if (len > TEMP_SIZE)
		return -1;
#endif

#ifdef CONFIG_AULU_Z
	ucode = kmalloc(FIRMWARE_SIZE+len+count, GFP_KERNEL);
#else
	ucode = kmalloc(count, GFP_KERNEL);
#endif
	ret = copy_from_user(ucode+offset, uc, count);
	if (ret < 0) {
		HIDEEP_ERR("ADDR_UC : copy_to_user");
		kfree(ucode);
		return 0;
	}

	disable_irq(ts->irq);
	hideep_fuse_ucode(ts->client, ucode, count, offset);
	enable_irq(ts->irq);
	kfree(ucode);

	HIDEEP_INFO("Download_uc(%d)", (int)count);

	return count;
}

static int hideep_debug_open(struct inode *inode, struct file *file)
{
	struct hideep_debug_dev_t *dev_info;

	dev_info = container_of(inode->i_cdev, struct hideep_debug_dev_t, cdev);
	if (dev_info == NULL) {
		HIDEEP_ERR("No such char device node");
		return -ENODEV;
	}

	dev_info->release_flag = false;
#ifdef CONFIG_AULU_Z
	dev_info->ts->z_flag_calib2 = false;
#endif

	file->private_data = dev_info;
	HIDEEP_INFO("hideep_debug_open");

	return 0;
}

static int hideep_debug_release(struct inode *inode, struct file *file)
{
	struct hideep_debug_dev_t *dev_info;

	dev_info = container_of(inode->i_cdev, struct hideep_debug_dev_t, cdev);

	HIDEEP_INFO("%s", __func__);
	if (!dev_info->release_flag)
		return -1;
	dev_info->release_flag = false;
	file->private_data = NULL;
	return 0;
}

static unsigned int hideep_debug_poll(struct file *file, struct poll_table_struct *wait)
{
	u32 mask = 0;
	struct hideep_debug_dev_t *dev_info = file->private_data;

	HIDEEP_INFO("%s", __func__);
	if (file->private_data == NULL)
		return 0;

	poll_wait(file, &dev_info->i_packet, wait);
	if (dev_info->ready) {
		mask |= POLLIN | POLLRDNORM;
		dev_info->ready = 0;
	}

	return mask;
}

static ssize_t hideep_debug_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct hideep_debug_dev_t *drv_info = file->private_data;
	ssize_t rd_len = 0;
	uint8_t *rd_buffer = NULL;
	unsigned short ver = 0x0;

	HIDEEP_INFO("%s count = %d", __func__, (int)count);
	if (file->private_data == NULL)
		return 0;

	drv_info->vr_size = count;
	rd_buffer = drv_info->vr_buff;
	rd_len = count;

	if ((*offset) < ADDR_VR_END) {
		if ((*offset) == ADDR_IMG) {
			drv_info->im_size = count;
			rd_buffer = drv_info->im_buff;
			rd_len = count;
			ret = 0;
#ifdef CONFIG_AULU_Z
		} else if ((HIDEEP_Z_VALUE == (*offset)) && (count == 2) && (drv_info->ts->z_status)) {
			drv_info->ts->z_status = false;
			*rd_buffer = drv_info->ts->z_buffer;
			rd_len = 2;
			ret = 0;
		} else if ((HIDEEP_Z_CALIB2 == (*offset)) &&
				(((drv_info->ts->z_calib_end-drv_info->ts->z_calib_start+1)*2+2) == count) &&
					(drv_info->ts->z_flag_ready)) {
			rd_buffer = drv_info->vr_buff;
			drv_info->ts->z_flag_calib2 = false;
			drv_info->ts->z_flag_ready = false;
			memcpy(rd_buffer+0, &drv_info->ts->z_index, 2);
			memcpy(rd_buffer+2, &drv_info->ts->z_data[drv_info->ts->z_calib_start], count-2);
			rd_len = count;
			ret = 0;
#endif
		} else {
			ret = hideep_get_vreg(drv_info->ts, *offset, rd_len);
		}
		if (ret < 0)
			rd_len = 0;
	} else if ((*offset) == HIDEEP_VERSION_INFO) {
		rd_buffer = drv_info->vr_buff;
		/* DD, boot, core, custom, vr :total 10 byte */
		/* vr */
		memcpy(rd_buffer+0, &drv_info->ts->dwz_info->ver_v, 2);
		/* custom */
		memcpy(rd_buffer+2, &drv_info->ts->dwz_info->ver_d, 2);
		/* core */
		memcpy(rd_buffer+4, &drv_info->ts->dwz_info->ver_c, 2);
		/* boot */
		memcpy(rd_buffer+6, &drv_info->ts->dwz_info->ver_b, 2);
		/* DD */
		ver = (HIDEEP_DD_VERSION_MAJOR & 0xff) << 8;
		ver = ver | (HIDEEP_DD_VERSION_MINOR & 0xf);
		memcpy(rd_buffer+8, &ver, 2);
		rd_len = count;
		ret = 0;
	} else {
		HIDEEP_ERR("hideep_read : undefined address");
		return 0;
	}

	ret = copy_to_user(buf, rd_buffer, rd_len);
	if (ret < 0) {
		HIDEEP_ERR("error : copy_to_user");
		return -EFAULT;
	}

	return rd_len;
}

static ssize_t hideep_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct hideep_debug_dev_t *drv_info = file->private_data;
	int wr_len = 0;

	HIDEEP_INFO("%s count = %d", __func__, (int)count);
	if (file->private_data == NULL)
		return 0;

	if ((*offset) < ADDR_VR_END) {
		wr_len = count;

		ret = copy_from_user(drv_info->vr_buff, buf, wr_len);
		if (ret < 0) {
			HIDEEP_ERR("error : copy_to_user");
			return -EFAULT;
		}

		ret = hideep_set_vreg(drv_info->ts, *offset, wr_len);
		if (ret < 0)
			wr_len = 0;
	} else if ((*offset & ADDR_UC) == ADDR_UC) {
		wr_len = hideep_download_uc(drv_info->ts, buf, count, *offset&0xffff);
	} else {
		HIDEEP_ERR("hideep_write : undefined address, 0x%08x", (int)*offset);

		return 0;
	}

	return wr_len;
}

static loff_t hideep_debug_llseek(struct file *file, loff_t off, int whence)
{
	loff_t newpos;
	struct hideep_debug_dev_t *drv_info = file->private_data;

	HIDEEP_INFO("%s off = 0x%08x, whence = %d", __func__, (unsigned int)off, whence);
	if (file->private_data == NULL)
		return -EFAULT;

		switch (whence) {
		/* SEEK_SET */
		case 0:
			newpos = off;
			break;
		/* SEEK_CUR */
		case 1:
			HIDEEP_INFO("%s set mode off = 0x%08x", __func__, (unsigned int)off);
			if (off == 0x1000) {
				drv_info->im_r_en = 1;
				drv_info->vr_buff[0] = HIDEEP_OPM_MOD_CAP;  /* select frame mode... */
				hideep_set_vreg(drv_info->ts, 0x00, 1);
				newpos = file->f_pos;
			} else if (off == 0x240) {
				drv_info->im_r_en = 0;
				drv_info->vr_buff[0] = HIDEEP_OPM_TOUCH_A;
				hideep_set_vreg(drv_info->ts, 0x00, 1);
				newpos = file->f_pos;
			} else if (off == HIDEEP_RELEASE_FLAG) {
				HIDEEP_DBG("set release flag");
				drv_info->release_flag = true;
				newpos = file->f_pos;
#ifdef CONFIG_AULU_Z
			} else if ((off & HIDEEP_Z_CALIB2_READ) == HIDEEP_Z_CALIB2_READ) {
				drv_info->ts->z_calib_start = off & 0x0fff;
				drv_info->ts->z_calib_end = (off >> 16) & 0x0fff;
				if ((drv_info->ts->z_calib_end - drv_info->ts->z_calib_start+1) > (1024*4)) {
					HIDEEP_ERR(" set the frames is oversize\n");
					return -EINVAL;
				}
				HIDEEP_DBG("set calib2 start = %d, end = %d\n",
						drv_info->ts->z_calib_start, drv_info->ts->z_calib_end);
				drv_info->ts->z_flag_calib2 = true;
				drv_info->ts->z_flag_ready = false;
				drv_info->ts->z_index = 0;
				memset(drv_info->ts->z_data, 0x0, 4096*2);
				newpos = off;
#endif
			} else {
				newpos = file->f_pos;
			}
			break;
		/* SEEK_END */
		case 2:
			drv_info->im_size = off & 0xffff;
			drv_info->vr_size = (off >> 16) & 0xffff;
			HIDEEP_INFO("%s HIDEEP_DEBUG_CFG : %d", __func__, drv_info->im_size);
			HIDEEP_INFO("%s HIDEEP_DEBUG_CFG : %d", __func__, drv_info->vr_size);
			newpos = file->f_pos;
			break;
		default:
			return -EINVAL;
	}

	if (newpos < 0)
		return -EINVAL;

	file->f_pos = newpos;

	return newpos;
}

static long hideep_debug_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct hideep_debug_dev_t *drv_info = file->private_data;
	struct hideep_debug_cfg_t config;
	unsigned char *buff;
	int err = 0;
	int ret = 0;
	int size;
	unsigned char mode;

	void __user *argp = (void __user *) arg;

	buff = drv_info->self_buff;

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/
	HIDEEP_INFO("%s", __func__);
	if (file->private_data == NULL)
		return 0;

	if (_IOC_TYPE(cmd) != HIDEEP_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HIDEEP_IOC_MAXNR)
		return -ENOTTY;

	/*
	* the direction is a bitmask, and VERIFY_WRITE catches R/W
	* transfers. `Type' is user-oriented, while
	* access_ok is kernel-oriented, so the concept of "read" and
	* "write" is reversed
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

		switch (cmd) {
		case HIDEEP_CFG:
			ret = copy_from_user(&config, argp, sizeof(struct hideep_debug_cfg_t));

			drv_info->im_size = config.im_size;
			drv_info->vr_size = config.vr_size;

			HIDEEP_INFO("HIDEEP_IFACE_CFG : %d", drv_info->im_size);
			HIDEEP_INFO("HIDEEP_IFACE_CFG : %d", drv_info->vr_size);
			break;
		case HIDEEP_RDIM:
			ret = copy_from_user(&drv_info->im_r_en, argp, sizeof(int));

			if (drv_info->im_r_en != 0)
				drv_info->vr_buff[0] = HIDEEP_OPM_MOD_CAP;	/* select frame mode... */
			else
				drv_info->vr_buff[0] = HIDEEP_OPM_TOUCH_A;

			hideep_set_vreg(drv_info->ts, 0x00, 1);
			break;
		/* redundant, as cmd was checked against MAXNR */
		case HIDEEP_SELF_TEST:
			ret = copy_from_user(buff, argp, sizeof(argp));
			mode = buff[0];
			size = (buff[1] << 8) | (buff[2]);

			HIDEEP_INFO("self mode : 0x%02x, size : %d", mode, size);

			mutex_lock(&drv_info->ts->dev_mutex);
			hideep_reset_ic(drv_info->ts);
			mdelay(100);
			ret = hideep_i2c_write(drv_info->ts, TEST_MODE_COMMAND_ADDR, 1, &mode);
			mdelay(500);
			memset(buff, 0x0, size);
			ret = hideep_i2c_read(drv_info->ts, SELF_TEST_DATA_ADDR, size, buff);

			ret = copy_to_user(argp, buff, size);
			hideep_reset_ic(drv_info->ts);
			mdelay(100);
			mutex_unlock(&drv_info->ts->dev_mutex);

			break;
		default:
			return -ENOTTY;
	}

	return ret;
}

static const struct file_operations hideep_debug_fops = {
	.owner = THIS_MODULE,
	.open = hideep_debug_open,
	.poll = hideep_debug_poll,
	.release = hideep_debug_release,
	.read = hideep_debug_read,
	.write = hideep_debug_write,
	.llseek = hideep_debug_llseek,
	.unlocked_ioctl = hideep_debug_ioctl,
};

static void hideep_debug_unregister(struct hideep_t *ts)
{
	struct hideep_debug_dev_t *dev  = &ts->debug_dev;

	device_destroy(ts->debug_class, ts->debug_dev_no);
	cdev_del(&dev->cdev);
}

static int hideep_debug_register(struct hideep_t *ts, u32 minor)
{
	int err = 0;
	struct device *device = NULL;
	struct hideep_debug_dev_t *dev = &ts->debug_dev;
	dev_t devno = ts->debug_dev_no;

	cdev_init(&dev->cdev, &hideep_debug_fops);
	dev->cdev.owner = THIS_MODULE;

	err = cdev_add(&dev->cdev, ts->debug_dev_no, 1);
	if (err)
		goto err;

	device = device_create(ts->debug_class, NULL, devno, NULL, HIDEEP_DEBUG_DEVICE_NAME);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		cdev_del(&dev->cdev);
		goto err;
	}

	return 0;

err:
	HIDEEP_ERR("hideep_debug_register failed");
	return err;
}

void hideep_debug_uninit(struct hideep_t *ts)
{
	/* Get rid of character devices (if any exist) */
	hideep_debug_unregister(ts);

	if (ts->debug_class)
		class_destroy(ts->debug_class);

	unregister_chrdev_region(ts->debug_dev_no, 1);
}

int hideep_debug_init(struct hideep_t *ts)
{
	int ret = 0;
	dev_t dev_id = 0;
	struct hideep_debug_dev_t *debug_drv = &ts->debug_dev;

	ret = alloc_chrdev_region(&dev_id, 0, 1, HIDEEP_DEBUG_DEVICE_NAME);
	if (ret < 0)
		return ret;

	ts->debug_dev_no = dev_id;
	ts->debug_class  = class_create(THIS_MODULE, HIDEEP_DEBUG_DEVICE_NAME);
	if (IS_ERR(ts->debug_class)) {
		ret = PTR_ERR(ts->debug_class);
		goto fail;
	}

	ret = hideep_debug_register(ts, 0);
	if (ret)
		goto fail;

	init_waitqueue_head(&debug_drv->i_packet);

	debug_drv->ts = ts;
	debug_drv->im_r_en = 0;	/* disable */
	debug_drv->im_size = 4096;
	debug_drv->vr_size = 4096;
#ifdef HIDEEP_SELFTEST_MODE
	hideep_i2c_read(ts, 0x8000, 1, (unsigned char *)&debug_drv->tx_num);
	hideep_i2c_read(ts, 0x8001, 1, (unsigned char *)&debug_drv->rx_num);
	HIDEEP_INFO("TX_NUM : %d, RX_NUM : %d", debug_drv->tx_num, debug_drv->rx_num);

	debug_drv->frame_size = debug_drv->tx_num * debug_drv->rx_num * 2;
	debug_drv->self_buff = kmalloc(debug_drv->frame_size, GFP_KERNEL);
	debug_drv->rawdata = kmalloc(debug_drv->frame_size, GFP_KERNEL);
#endif
	debug_drv->im_buff = kmalloc(debug_drv->im_size, GFP_KERNEL);
	debug_drv->vr_buff = kmalloc(debug_drv->vr_size, GFP_KERNEL);

	if (!debug_drv->im_buff || !debug_drv->vr_buff)
		goto fail;

	HIDEEP_INFO("hideep_debug_init....");
	return 0;

fail:
	hideep_debug_uninit(ts);

	pr_err("%s failed", __func__);

	return ret;
}
#endif
