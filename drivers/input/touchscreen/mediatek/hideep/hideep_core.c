/*******************************************************************************************
 * Copyright (C) 2012 Hideep, Inc.
 * kim.liao@hideep.com
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
#include <linux/of.h>
#include <linux/of_irq.h>

#include "hideep.h"
#include "tpd.h"

static unsigned int touch_irq;

static DECLARE_WAIT_QUEUE_HEAD(waiter);

int tpd_flag = 0;
int loglevel = 0;
struct hideep_t *g_ts;

#ifdef HIDEEP_KEYBUTTON
static int hideep_keycodes[] = {KEY_MENU, KEY_HOMEPAGE, KEY_BACK};
#endif

#ifdef CONFIG_FB
static int hideep_suspend(struct device *dev);
static int hideep_resume(struct device *dev);
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

static int hideep_hw_pwron(struct hideep_t *ts)
{
	int ret = 0;

	/*enable regulator*/
	ret = regulator_enable(tpd->reg);
	if (ret)
		HIDEEP_ERR("regulator_enable() failed!\n");

	mdelay(3);

	/* rst pin output high */
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

	return ret;
}

static int hideep_hw_pwroff(struct hideep_t *ts)
{
	int ret = 0;

	/* rst pin output low */
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);

	/* disable regulator */
	ret = regulator_disable(tpd->reg);
	if (ret)
		HIDEEP_ERR("regulator_disable() failed!\n");

	return ret;
}

void hideep_power(struct hideep_t *ts, int on)
{
	int ret = 0;

	if (on) {
		HIDEEP_INFO("power on");
		ret = hideep_hw_pwron(ts);
	} else {
		HIDEEP_INFO("power off");
		ret = hideep_hw_pwroff(ts);
	}
}

int hideep_i2c_read(struct hideep_t *ts, u16 addr, u16 len, u8 *buf)
{
	int ret = -1;
	struct i2c_client *client = ts->client;

	HIDEEP_I2C("addr=0x%02x, len=%d", addr, len);
	mutex_lock(&ts->i2c_mutex);
	ret = i2c_master_send(client, (char *) &addr, 2);

	if (ret < 0)
		goto i2c_err;

	ret = i2c_master_recv(client, (char *) buf, len);

	if (ret < 0)
		goto i2c_err;

	mutex_unlock(&ts->i2c_mutex);
	return  0;

i2c_err:
	mutex_unlock(&ts->i2c_mutex);
	return -1;
}

int hideep_i2c_write(struct hideep_t *ts, u16 addr, u16 len, u8 *buf)
{
	int ret = -1;
	struct i2c_client *client = ts->client;

	HIDEEP_I2C("addr=0x%02x, len=%d", addr, len);
	mutex_lock(&ts->i2c_mutex);

	/* data mangling..*/
	ts->i2c_buf[0] = (addr >> 0) & 0xFF;
	ts->i2c_buf[1] = (addr >> 8) & 0xFF;
	memcpy(&ts->i2c_buf[2], buf, len);

	ret = i2c_master_send(client, (char *)ts->i2c_buf, len + 2);

	if (ret < 0)
		goto i2c_err;

	mutex_unlock(&ts->i2c_mutex);
	return  0;

i2c_err:
	mutex_unlock(&ts->i2c_mutex);
	return -1;
}

static void pops_mt(struct input_dev *input_dev, struct hideep_mt_t *finger, s32 nr)
{
#ifdef HIDEEP_TYPE_B_PROTOCOL
	s32 id;
	s32 i;

	for (i = 0; i < nr; i++) {
		id = (finger[i].index >> 0) & 0x0F;
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
		input_report_key(input_dev, BTN_TOUCH, false);
	}
#else
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
#endif

	input_sync(input_dev);
}

static void push_mt(struct hideep_t *ts, struct input_dev *input_dev, struct hideep_mt_t *finger, s32 nr)
{
	s32 id;
	s32 i;
	bool btn_up;
	bool btn_dn;
	bool btn_mv;
	int evt = 0;

#ifdef CONFIG_AULU_Z
	if ((ts->z_flag_calib2) && (nr) && (!ts->z_flag_ready)) {
		HIDEEP_XY("z = 0x%02x, index = %d", finger[0].z, ts->z_index);
		if (ts->z_index >= ts->z_calib_start) {
			HIDEEP_XY("reading");
			ts->z_data[ts->z_index] = finger[0].z;
			if (ts->z_index >= ts->z_calib_end) {
				HIDEEP_XY("ready");
				ts->z_flag_ready = true;
			}
		}
		if (!ts->z_flag_ready)
			ts->z_index++;
	} else if (nr) {
		HIDEEP_XY("z_buffer, %d", finger[0].z);
		ts->z_buffer = finger[0].z;
		ts->z_status = true;
	}
#endif

	/* load multi-touch event to input system*/
	for (i = 0; i < nr; i++) {
		id = (finger[i].index >> 0) & 0x0F;
		btn_up = (finger[i].flag >> HIDEEP_MT_RELEASED) & 0x01;
		btn_dn = (finger[i].flag >> HIDEEP_MT_FIRST_CONTACT) & 0x01;
		btn_mv = (finger[i].flag >> HIDEEP_MT_DRAG_MOVE) & 0x01;

		if (btn_up)
			clear_bit(id, &ts->tch_bit);
		else
			__set_bit(id, &ts->tch_bit);

		HIDEEP_XY("id=%d, i=%d, x=%d, y=%d, z=%d", finger[i].index, i, finger[i].x, finger[i].y, finger[i].z);

#ifdef HIDEEP_TYPE_B_PROTOCOL
		input_mt_slot(input_dev, id);
		if (btn_up) {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
		} else {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
#else
		if (btn_up) {
			input_mt_sync(input_dev);
		} else {
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
#endif
			/* because android system. must z value upper zero.*/
			if (finger[i].z < 10)
					finger[i].z = 10;

			input_report_abs(input_dev, ABS_MT_POSITION_X, finger[i].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[i].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE, finger[i].z);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[i].w);
			evt++;
#ifndef HIDEEP_TYPE_B_PROTOCOL
			input_mt_sync(input_dev);
#endif
			}
		}

	if (ts->tch_bit == 0)
		evt = 0;

	input_report_key(input_dev, BTN_TOUCH, evt);
	input_sync(input_dev);
}

static void pops_ky(struct input_dev *input_dev, struct hideep_key_t *key, s32 nr)
{
	s32 i;

	for (i = 0; i < nr; i++) {
		input_report_key(input_dev, hideep_keycodes[i], false);
		input_report_key(input_dev, BTN_TOUCH, false);
	}

	input_sync(input_dev);
}

static void push_ky(struct input_dev *input_dev, struct hideep_key_t *key, s32 nr)
{
	s32 i;
	s32 pressed;
	s32 key_code;
	s32 key_status;

	for (i = 0; i < nr; i++) {
		key_code   = key[i].flag & 0x0F;
		key_status = key[i].flag & 0xF0;
		pressed    = false;

		if (key_status & HIDEEP_KEY_PRESSED_MASK)
			pressed = true;
		else
			pressed = false;

		input_report_key(input_dev, hideep_keycodes[key_code], pressed);
		input_report_key(input_dev, BTN_TOUCH, pressed);
	}

	input_sync(input_dev);
}

static void hideep_put_event(struct hideep_t *ts)
{
	/* mangling touch information */
	if (ts->tch_count > 0)
		push_mt(ts, ts->input_dev, ts->touch_evt, ts->tch_count);

	if (ts->key_count > 0)
		push_ky(ts->input_dev, ts->key_evt, ts->key_count);
}

static s32 hideep_get_event(struct hideep_t *ts)
{
	s32 ret;
	u8 i2c_buff[2];
	s32 touch_count;
	u8 cmd;

	/* get touch event count */
	HIDEEP_XY();
	ts = g_ts;
	ret = hideep_i2c_read(ts, HIDEEP_EVENT_COUNT_ADDR, 2, (u8 *)i2c_buff);
	if (ret < 0)
		goto i2c_err;

	ts->tch_count = i2c_buff[0];
	ts->key_count = i2c_buff[1] & 0x0f;
	ts->lpm_count = i2c_buff[1] & 0xf0;

	HIDEEP_XY("mt = %d, key = %d, lpm = %02x", ts->tch_count, ts->key_count, ts->lpm_count);

	/* get touch event information */
	if (ts->tch_count > HIDEEP_MT_MAX)
		ts->tch_count = 0;

	if (ts->key_count > HIDEEP_KEY_MAX)
		ts->key_count = 0;

	touch_count = ts->tch_count + ts->key_count;

	if (ts->dev_state == power_sleep && ts->lpm_mode == true) {
		if (ts->lpm_count == 0x10) {
			/* POWER_KEY push */
			input_report_key(ts->input_dev, KEY_WAKEUP, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_WAKEUP, 0);
			input_sync(ts->input_dev);
		} else {
			/* Enter again lpm mode */
			cmd = 1;
			hideep_i2c_write(ts, HIDEEP_LPM_MODE, 1, &cmd);
		}
		return 0;
	}

	if (ts->tch_count > 0) {
		uint32_t info_size = ts->tch_count * sizeof(struct hideep_mt_t);

		ret = hideep_i2c_read(ts, HIDEEP_TOUCH_DATA_ADDR, info_size, (u8 *)ts->touch_evt);
		if (ret < 0) {
			HIDEEP_ERR("read I2C error.");
			goto i2c_err;
		}
	}

	if (ts->key_count > 0) {
		uint32_t info_size = ts->key_count * sizeof(struct hideep_key_t);

		ret = hideep_i2c_read(ts, HIDEEP_KEY_DATA_ADDR, info_size, (u8 *)ts->key_evt);
		if (ret < 0) {
			HIDEEP_ERR("read I2C error.");
			goto i2c_err;
		}
	}

	return touch_count;

i2c_err:
	HIDEEP_ERR("I2C error.");
	return -1;
}


#ifdef HIDEEP_DEBUG_DEVICE
static s32 hideep_get_image(struct hideep_t *ts)
{
	s32 ret = 0;
	struct hideep_debug_dev_t *debug_dev = &ts->debug_dev;

	ret = hideep_i2c_read(ts, HIDEEP_VR_IMAGE_ADDR, debug_dev->im_size, debug_dev->im_buff);

	if (ret < 0)
		goto i2c_err;

	HIDEEP_INFO("load image from sensor(%d)", debug_dev->im_size);
	return ret;

i2c_err:
	HIDEEP_ERR("i2c error.");
	return ret;
}
#endif

static irqreturn_t hideep_irq_task(int irq, void *handle)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	disable_irq_nosync(touch_irq);
	return IRQ_HANDLED;
}

static int hideep_capability(struct hideep_t *ts)
{
	int i;
#ifdef HIDEEP_PANEL_INFO
	ts->input_dev->id.vendor = ts->dwz_info->panel.vendor;
	ts->input_dev->id.product = ts->dwz_info->panel.product;
	ts->input_dev->id.version = ts->dwz_info->panel.version;
#endif

	ts->input_dev->name = HIDEEP_TS_NAME;
	ts->input_dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
#ifdef HIDEEP_LPM_WAKEUP
	set_bit(KEY_WAKEUP, ts->input_dev->keybit);
#endif
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef HIDEEP_KEYBUTTON
	for (i = 0; i < ARRAY_SIZE(hideep_keycodes); i++)
		set_bit(hideep_keycodes[i], ts->input_dev->keybit);
#endif

#ifdef HIDEEP_TYPE_B_PROTOCOL
	input_mt_init_slots(ts->input_dev, HIDEEP_MT_MAX, INPUT_MT_DIRECT);
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 1080-1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 1920-1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 65535, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);

	return 0;
}

void hideep_reset_ic(struct hideep_t *ts)
{
	struct hideep_platform_data_t *pdata;

	pdata = ts->p_data;

	HIDEEP_INFO("start!!");

	/* rst pin output  up, down, up */
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
	mdelay(1);
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	mdelay(20);
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

	mdelay(20);
	HIDEEP_INFO("end!!");
}

void hideep_init_ic(struct hideep_t *ts)
{
	int ret = 0;

	/* regulator get */
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	/*set 2.8v*/
	if (ret)
		HIDEEP_ERR("regulator_set_voltage(%d) failed!\n", ret);

	/* power on */
	hideep_power(ts, true);
	ts->dev_state = power_init;
	mdelay(30);

	/* interrupt pin set */
	GTP_GPIO_AS_INT(GTP_INT_PORT);

	/* ic reset */
	hideep_reset_ic(ts);
}


static int hideep_i2c_suspend(struct device *dev)
{
	struct hideep_t *ts = dev_get_drvdata(dev);
	u8 sleep_cmd = 1;

	HIDEEP_INFO("enter");

	mutex_lock(&ts->dev_mutex);
	if (ts->dev_state == power_sleep)
		goto hideep_i2c_suspend_exit;

	HIDEEP_INFO("not waiting.");
	ts->dev_state = power_sleep;

	/* send sleep command.. */
	pops_mt(ts->input_dev, ts->touch_evt, ts->tch_count);
	pops_ky(ts->input_dev, ts->key_evt, ts->key_count);

	if (ts->lpm_mode) {
		hideep_i2c_write(ts, HIDEEP_LPM_MODE, 1, &sleep_cmd);
	} else {
		hideep_i2c_write(ts, HIDEEP_SLEEP_MODE, 1, &sleep_cmd);
		disable_irq(ts->client->irq);
	}

hideep_i2c_suspend_exit:
	mutex_unlock(&ts->dev_mutex);
	HIDEEP_INFO("exit.");
	return 0;
}

static int hideep_i2c_resume(struct device *dev)
{
	struct hideep_t *ts = dev_get_drvdata(dev);
	u8 sleep_cmd = 0;

	HIDEEP_INFO("enter");

	mutex_lock(&ts->dev_mutex);

	if (ts->dev_state == power_normal)
		goto hideep_i2c_resume_exit;

	HIDEEP_INFO("not waiting.");
	ts->dev_state = power_normal;

	if (!ts->lpm_mode) {
		/* send wake up command.. */
		hideep_i2c_read(ts, HIDEEP_SLEEP_MODE, 1, &sleep_cmd);
		enable_irq(ts->client->irq);
	}

hideep_i2c_resume_exit:
	mdelay(10);
	hideep_reset_ic(ts);

	mutex_unlock(&ts->dev_mutex);
	HIDEEP_INFO("exit.");
	return 0;
}

#ifdef CONFIG_FB
static int hideep_suspend(struct device *dev)
{
	struct hideep_t *ts = dev_get_drvdata(dev);
	u8 sleep_cmd = 1;

	HIDEEP_INFO("enter");

	mutex_lock(&ts->dev_mutex);
	if (ts->dev_state == power_sleep)
		goto hideep_suspend_exit;

	HIDEEP_INFO("not waiting.");
	ts->dev_state = power_sleep;

	/* send sleep command.. */
	pops_mt(ts->input_dev, ts->touch_evt, ts->tch_count);
	pops_ky(ts->input_dev, ts->key_evt, ts->key_count);

	if (ts->lpm_mode) {
		hideep_i2c_write(ts, HIDEEP_LPM_MODE, 1, &sleep_cmd);
	} else {
		hideep_i2c_write(ts, HIDEEP_SLEEP_MODE, 1, &sleep_cmd);
		disable_irq(ts->client->irq);
	}

hideep_suspend_exit:
	mutex_unlock(&ts->dev_mutex);
	HIDEEP_INFO("exit.");
	return 0;
}

static int hideep_resume(struct device *dev)
{
	struct hideep_t *ts = dev_get_drvdata(dev);
	u8 sleep_cmd = 0;

	HIDEEP_INFO("enter");

	mutex_lock(&ts->dev_mutex);

	if (ts->dev_state == power_normal)
		goto hideep_resume_exit;

	HIDEEP_INFO("not waiting.");
	ts->dev_state = power_normal;

	if (!ts->lpm_mode) {
		/* send wake up command.. */
		hideep_i2c_read(ts, HIDEEP_SLEEP_MODE, 1, &sleep_cmd);
		enable_irq(ts->client->irq);
	}

hideep_resume_exit:
	mdelay(10);
	hideep_reset_ic(ts);

	mutex_unlock(&ts->dev_mutex);
	HIDEEP_INFO("exit.");
	return 0;
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	struct hideep_t *ts = container_of(self, struct hideep_t, fb_notif);

	if ((evdata) && (evdata->data) && (event == FB_EVENT_BLANK) && (ts) && (ts->client)) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			HIDEEP_INFO("resume");
			if (ts->suspended == 1) {
				hideep_resume(&ts->client->dev);
				ts->suspended = 0;
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			HIDEEP_INFO("suspend");
			if (ts->suspended == 0) {
				ts->suspended = 1;
				hideep_suspend(&ts->client->dev);
			}
		}
	}

	return 0;
}
#endif

#ifdef HIDEEP_AUTO_UPDATE
static void fwu_startup_fw_update_work(struct work_struct *work)
{
	int ret;

	disable_irq(g_ts->client->irq);
	g_ts->manually_update = false;
	HIDEEP_INFO("hideep starting fw[%s] update...", HIDEEP_AUTO_FW);
	ret = hideep_load_ucode(&g_ts->client->dev, HIDEEP_AUTO_FW);

	if (ret)
		HIDEEP_ERR("The firmware update failed(%d)", ret);

	enable_irq(&g_ts->client->irq);
	HIDEEP_INFO("hideep done fw[%s] update.", HIDEEP_AUTO_FW);
}
#endif


static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	HIDEEP_INFO("Device Tree Tpd_irq_registration!");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		ret =
			    request_irq(touch_irq, (irq_handler_t) hideep_irq_task, IRQF_TRIGGER_FALLING,
					"TOUCH_PANEL-eint", NULL);
		if (ret > 0) {
			ret = -1;
			HIDEEP_ERR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
		}

	} else {
		HIDEEP_ERR("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}

	HIDEEP_INFO("[%s]irq:%d, debounce:%d-%d:", __func__, touch_irq, ints[0], ints[1]);
	return ret;
}

static int tpd_event_handler(void *unused)
{
	struct hideep_t *ts = g_ts;
#ifdef HIDEEP_DEBUG_DEVICE
	struct hideep_debug_dev_t *debug_dev = &ts->debug_dev;
#endif
	s32 t_evt;

	struct sched_param param = {.sched_priority = 4};

	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

		HIDEEP_INFO("enter %s\n", __func__);
#ifdef HIDEEP_DEBUG_DEVICE
		HIDEEP_XY("hideep_irq_task, state = 0x%x, im_r_en = %d", ts->dev_state, debug_dev->im_r_en);
#endif
		mutex_lock(&ts->dev_mutex);

		if (ts->dev_state != power_normal) {
#ifdef HIDEEP_LPM_WAKEUP
			if (!ts->lpm_mode) {
				enable_irq(touch_irq);
				mutex_unlock(&ts->dev_mutex);
				return 0;
			}
#endif
		}
#ifdef HIDEEP_DEBUG_DEVICE
		if (debug_dev->im_r_en == 1) {
			/* for debug.... */
			hideep_get_image(ts);
			ts->debug_dev.ready = 1;
			wake_up_interruptible(&ts->debug_dev.i_packet);
		} else
#endif
		{
			t_evt = hideep_get_event(ts);
			if (t_evt > 0)
				hideep_put_event(ts);
		}
		/* Enable irq */
		enable_irq(touch_irq);
		mutex_unlock(&ts->dev_mutex);
		HIDEEP_XY("end.");

	} while (!kthread_should_stop());

	return 0;
}

static int hideep_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct dwz_info_t *dwz;
	struct hideep_t *ts;
	struct task_struct *tpd_data_thread;

	HIDEEP_WARN("enter [%s]", __func__);
	/* check i2c bus */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		HIDEEP_ERR("check i2c device error");
		ret = -ENODEV;
		return ret;
	}

	/* init hideep_t */
	ts = devm_kzalloc(&client->dev, sizeof(struct hideep_t), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	dwz = devm_kzalloc(&client->dev, sizeof(struct dwz_info_t), GFP_KERNEL);
	if (!dwz)
		return -ENOMEM;

	ts->client = client;
	ts->dwz_info = dwz;
	i2c_set_clientdata(client, ts);

	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->dev_mutex);

	hideep_init_ic(ts);


#ifdef HIDEEP_LPM_WAKEUP
	ts->lpm_mode = true;
#endif

#ifdef HIDEEP_DWZ_VERSION_CHECK
	/* read info */
	ret = hideep_load_dwz(ts);
	if (ret < 0) {
		HIDEEP_ERR("fail to load dwz, ret = 0x%x", ret);
		goto hideep_probe_read_dwz_err;
	}
#endif


	/* init input device */
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		HIDEEP_ERR("can't allocate memory for input_dev");
		ret = -ENOMEM;
		goto hideep_probe_input_dev_memory_err;
	}

	hideep_capability(ts);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		HIDEEP_ERR("can't register input_dev");
		ret = -ENOMEM;
		goto hideep_probe_register_input_dev_err;
	}

	input_set_drvdata(ts->input_dev, ts);


#ifdef HIDEEP_AUTO_UPDATE
	ts->p_workqueue_fw = create_singlethread_workqueue("hideep_fw_auto_workqueue");
	INIT_DELAYED_WORK(&ts->work_fwu, fwu_startup_fw_update_work);
	queue_delayed_work(ts->p_workqueue_fw, &ts->work_fwu, msecs_to_jiffies(HIDEEP_UPDATE_FW_THREAD_DELAY));
#endif

#ifdef HIDEEP_DEBUG_DEVICE
	ret = hideep_debug_init(ts);
	if (ret) {
		HIDEEP_ERR("fail init debug, ret = 0x%x", ret);
		ret = -1;
		goto hideep_probe_debug_init_err;
	}
#endif

	ret = hideep_sysfs_init(ts);
	if (ret) {
		HIDEEP_ERR("fail init sys, ret = 0x%x", ret);
		ret = -1;
		goto hideep_probe_sysfs_init_err;
	}

#ifdef CONFIG_FB
	ts->suspended = 0;
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret) {
		HIDEEP_ERR("Unable to register fb_notifier: ret = %d", ret);
		ret = -1;
		goto hideep_probe_register_fb_err;
	}
#endif

	ts->dev_state = power_normal;
	/* save ts data to g_ts */
	g_ts = ts;

	/* Create thread to report data */
	tpd_data_thread = kthread_run(tpd_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(tpd_data_thread)) {
		ret = PTR_ERR(tpd_data_thread);
		HIDEEP_ERR(TPD_DEVICE " failed to create kernel thread: %d\n", ret);
		goto hideep_probe_register_fb_err;
	}


#ifdef MTK_PLATFORM
	tpd_load_status = 1;
#endif

	/* request irq */
	ret = tpd_irq_registration();
	if (ret < 0) {
		HIDEEP_ERR("tpd_irq_registration fail\n");
		goto hideep_probe_register_fb_err;
	}
	ts->client->irq = touch_irq;

	HIDEEP_INFO("probe is ok!");
	return 0;

hideep_probe_register_fb_err:
	hideep_sysfs_exit(ts);

hideep_probe_sysfs_init_err:
#ifdef HIDEEP_DEBUG_DEVICE
hideep_probe_debug_init_err:
#endif
	input_unregister_device(ts->input_dev);
hideep_probe_register_input_dev_err:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

#ifdef HIDEEP_DWZ_VERSION_CHECK
hideep_probe_read_dwz_err:
#endif
hideep_probe_input_dev_memory_err:

	HIDEEP_ERR("probe err!");
	return ret;
}

static int hideep_remove(struct i2c_client *client)
{
	struct hideep_t *ts = i2c_get_clientdata(client);

#ifdef CONFIG_FB
	if (fb_unregister_client(&ts->fb_notif))
		HIDEEP_INFO("Error occurred while unregistering fb_notifier");
#endif

#ifdef DO_STARTUP_FW_UPDATE
	cancel_delayed_work_sync(&ts->fwu_work);
	flush_workqueue(ts->fwu_workqueue);
	destroy_workqueue(ts->fwu_workqueue);
#endif

	if (ts->p_data->regulator_vid != NULL || ts->p_data->regulator_vdd != NULL)
		hideep_power(ts, false);
	hideep_reset_ic(ts);
	free_irq(client->irq, ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	input_unregister_device(ts->input_dev);
	hideep_sysfs_exit(ts);

#ifdef HIDEEP_DEBUG_DEVICE
	hideep_debug_uninit(ts);
#endif
	kfree(ts);
	return 0;
}

static const struct i2c_device_id hideep_dev_idtable[] = {
	{ HIDEEP_I2C_NAME, 0 },
	{}
};

static struct of_device_id hideep_match_table[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

#ifdef CONFIG_PM
static const struct dev_pm_ops hideep_pm_ops = {
	.suspend = hideep_i2c_suspend,
	.resume = hideep_i2c_resume,
};
#endif

static struct i2c_driver hideep_driver = {
	.probe = hideep_probe,
	.remove = hideep_remove,
	.id_table = hideep_dev_idtable,
	.driver = {
		.name = HIDEEP_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hideep_match_table),
#ifdef CONFIG_PM
		.pm = &hideep_pm_ops,
#endif
	},
};

static void tpd_suspend(struct device *h)
{
	/* use i2c_driver->driver->pm now */
}

static void tpd_resume(struct device *h)
{
	/* use i2c_driver->driver->pm now */
}

static int tpd_local_init(void)
{
	int ret;

	ret = i2c_add_driver(&hideep_driver);
	if (ret != 0)
		HIDEEP_ERR("unable to add i2c driver.");

	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = HIDEEP_I2C_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};

static int hideep_init(void)
{
	HIDEEP_INFO("enter");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		HIDEEP_ERR("add generic driver failed\n");
	return 0;
}

static void hideep_exit(void)
{
	HIDEEP_INFO("enter");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(hideep_init);
module_exit(hideep_exit);

MODULE_DESCRIPTION("Driver for HiDeep Touchscreen Controller");
MODULE_AUTHOR("anthony.kim@hideep.com");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
