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

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/suspend.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <linux/reboot.h>

#include "tcpci_core.h"
#include "mtk_direct_charge_vdm.h"

#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/upmu_common.h>
#include <mach/mt_charging.h>

#include "mtk_pep30_intf.h"

#define  DC_INIT                        (0x00)
#define  DC_MEASURE_R					(0x01)
#define  DC_SOFT_START                  (0x02)
#define  DC_CC							(0x03)
#define  DC_STOP	                    (0x04)


#define BAT_MS_TO_NS(x) (x * 1000 * 1000)


static struct hrtimer mtk_charger_pep30_timer;
static struct task_struct *mtk_charger_pep30_thread;
static bool mtk_charger_pep30_thread_flag;
static DECLARE_WAIT_QUEUE_HEAD(mtk_charger_pep30_thread_waiter);

static bool is_pep30_done; /* one plug in only do pep30 once */
static int pep30_charging_state = DC_STOP;
static struct tcpc_device *tcpc;
static ktime_t ktime;
static int r_total, r_cable, r_sw, r_vbat;
static int batteryTemperature = -1000;

static int is_vdm_rdy;
#define VDM_UNDET 0
#define VDM_VALID 1
#define VDM_INVALID -1

/* CONFIG_MTK_JEITA_STANDARD_SUPPORT */
static bool is_jeita_enable;
static int jeita_state = TEMP_POS_10_TO_POS_45;

struct mtk_vdm_ta_cap current_cap;

static int charging_current_limit = CC_INIT;

/* VDM cmd */
static int mtk_pep30_set_ta_boundary_cap(int cur, int voltage)
{
	struct mtk_vdm_ta_cap cap;

	cap.cur = cur;
	cap.vol = voltage;

	return mtk_set_ta_boundary_cap(tcpc, &cap);
}

static int mtk_pep30_get_ta_boundary_cap(struct mtk_pep30_cap *cap)
{
	struct mtk_vdm_ta_cap ta_cap;
	int ret;

	ret = mtk_get_ta_boundary_cap(tcpc, &ta_cap);
	cap->cur = ta_cap.cur;
	cap->vol = ta_cap.vol;

	return ret;
}

static int mtk_pep30_set_ta_cap(int cur, int voltage)
{
	current_cap.cur = cur;
	current_cap.vol = voltage;

	return mtk_set_ta_cap(tcpc, &current_cap);
}

static int mtk_pep30_get_ta_cap(struct mtk_pep30_cap *cap)
{
	struct mtk_vdm_ta_cap ta_cap;
	int ret;

	ret = mtk_get_ta_cap(tcpc, &ta_cap);
	cap->cur = ta_cap.cur;
	cap->vol = ta_cap.vol;

	return ret;
}

static int mtk_pep30_get_ta_current_cap(struct mtk_pep30_cap *cap)
{
	struct mtk_vdm_ta_cap ta_cap;
	int ret;

	ret = mtk_get_ta_current_cap(tcpc, &ta_cap);
	cap->cur = ta_cap.cur;
	cap->vol = ta_cap.vol;

	return ret;
}

static int mtk_pep30_enable_direct_charge(bool en)
{
	int ret = 0;

	ret = mtk_enable_direct_charge(tcpc, en);
	mtk_chr_enable_direct_charge(en);
	return ret;
}
/* VDM cmd end*/



/*
static void dump(void)
{
	signed int cur;
	kal_bool sign;
	struct mtk_vdm_ta_cap cap, cap_max, cap_b;
	int vbus, vbat, bifvbat, biftmp;
	int ret1, ret2;

	cur = battery_meter_get_battery_current();
	sign = battery_meter_get_battery_current_sign();
	vbus = battery_meter_get_charger_voltage();

	vbat = battery_meter_get_battery_voltage(KAL_TRUE);
	battery_meter_get_bif_battery_voltage(&bifvbat);
	battery_meter_get_bif_battery_temperature(&biftmp);

	ret1 = mtk_get_ta_current_cap(tcpc, &cap);
	ret2 = mtk_get_ta_cap(tcpc, &cap_max);
	ret2 = mtk_get_ta_boundary_cap(tcpc, &cap_b);

	battery_log(BAT_LOG_CRTI,
	"[%d][fg]:%d:%d: diff:%d: [bus_cur]:%d:%d: [bus_vol]:%d:%d: vbus:%d: vbat:%d: bifvbat:%d: soc:%d: uisoc2:%d: tmp:%d: b:%d:%d:\n",
		pep30_charging_state, sign, cur, cap.cur - cur/10,
		cap.cur, cap_max.cur, cap.vol, cap_max.vol, vbus, vbat, bifvbat, BMT_status.SOC, BMT_status.UI_SOC2,
		biftmp, cap_b.cur, cap_b.vol);

}
*/

static void wake_up_pep30_thread(void)
{
	mtk_charger_pep30_thread_flag = true;
	wake_up_interruptible(&mtk_charger_pep30_thread_waiter);

	battery_log(BAT_LOG_CRTI, "[wake_up_pep30_thread]\n");
}

/* CONFIG_MTK_JEITA_STANDARD_SUPPORT */
bool pep30_enable_jeita(bool en)
{
	is_jeita_enable = en;
	battery_log(BAT_LOG_CRTI, "[pep30_enable_jeita= %d]\n", is_jeita_enable);
	return true;
}

bool is_pep30_enable_jeita(void)
{
	return is_jeita_enable;
}


static int pep30_select_jeita_cv(void)
{
	int cv_voltage;

	if (jeita_state == TEMP_ABOVE_POS_60)
		cv_voltage = JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE / 1000;
	else if (jeita_state == TEMP_POS_45_TO_POS_60)
		cv_voltage = JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE / 1000;
	else if (jeita_state == TEMP_POS_10_TO_POS_45)
			cv_voltage = BAT_UPPER_BOUND;
	else if (jeita_state == TEMP_POS_0_TO_POS_10)
		cv_voltage = JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE / 1000;
	else if (jeita_state == TEMP_NEG_10_TO_POS_0)
		cv_voltage = JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE / 1000;
	else if (jeita_state == TEMP_BELOW_NEG_10)
		cv_voltage = JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE / 1000;
	else
		cv_voltage = BATTERY_VOLT_04_200000_V / 1000;

	return cv_voltage;
}

static bool pe30_do_jeita_state_machine(void)
{

	battery_log(BAT_LOG_CRTI, "[pe30_do_jeita_state_machine]tmp:%d state:%d\n", batteryTemperature, jeita_state);

	/* JEITA battery temp Standard */
	if (batteryTemperature >= TEMP_POS_60_THRESHOLD) {
		battery_log(BAT_LOG_CRTI,
			    "[PE3.0] Battery Over high Temperature(%d) !!\n\r",
			    TEMP_POS_60_THRESHOLD);

		jeita_state = TEMP_ABOVE_POS_60;

		return false;
	} else if (batteryTemperature > TEMP_POS_45_THRESHOLD) {	/* control 45c to normal behavior */
		if ((jeita_state == TEMP_ABOVE_POS_60)
		    && (batteryTemperature >= TEMP_POS_60_THRES_MINUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[PE3.0] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
				    TEMP_POS_60_THRES_MINUS_X_DEGREE, TEMP_POS_60_THRESHOLD);

			return false;
		}
			battery_log(BAT_LOG_CRTI,
				    "[PE3.0] Battery Temperature between %d and %d !!\n\r",
				    TEMP_POS_45_THRESHOLD, TEMP_POS_60_THRESHOLD);

			jeita_state = TEMP_POS_45_TO_POS_60;

	} else if (batteryTemperature >= TEMP_POS_10_THRESHOLD) {
		if (((jeita_state == TEMP_POS_45_TO_POS_60)
		     && (batteryTemperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE))
		    || ((jeita_state == TEMP_POS_0_TO_POS_10)
			&& (batteryTemperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE))) {
			battery_log(BAT_LOG_CRTI,
				    "[PE3.0] Battery Temperature not recovery to normal temperature charging mode yet!!\n\r");
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[PE3.0] Battery Normal Temperature between %d and %d !!\n\r",
				    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD);
			jeita_state = TEMP_POS_10_TO_POS_45;
		}
	} else if (batteryTemperature >= TEMP_POS_0_THRESHOLD) {
		if ((jeita_state == TEMP_NEG_10_TO_POS_0 || jeita_state == TEMP_BELOW_NEG_10)
		    && (batteryTemperature <= TEMP_POS_0_THRES_PLUS_X_DEGREE)) {
			if (jeita_state == TEMP_NEG_10_TO_POS_0) {
				battery_log(BAT_LOG_CRTI,
					    "[PE3.0] Battery Temperature between %d and %d !!\n\r",
					    TEMP_POS_0_THRES_PLUS_X_DEGREE, TEMP_POS_10_THRESHOLD);
			}
			if (jeita_state == TEMP_BELOW_NEG_10) {
				battery_log(BAT_LOG_CRTI,
					    "[PE3.0] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
					    TEMP_POS_0_THRESHOLD, TEMP_POS_0_THRES_PLUS_X_DEGREE);
				return PMU_STATUS_FAIL;
			}
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[PE3.0] Battery Temperature between %d and %d !!\n\r",
				    TEMP_POS_0_THRESHOLD, TEMP_POS_10_THRESHOLD);

			jeita_state = TEMP_POS_0_TO_POS_10;
		}
	} else if (batteryTemperature >= TEMP_NEG_10_THRESHOLD) {
		if ((jeita_state == TEMP_BELOW_NEG_10)
		    && (batteryTemperature <= TEMP_NEG_10_THRES_PLUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[PE3.0] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
				    TEMP_NEG_10_THRESHOLD, TEMP_NEG_10_THRES_PLUS_X_DEGREE);

			return false;
		}
			battery_log(BAT_LOG_CRTI,
				    "[PE3.0] Battery Temperature between %d and %d !!\n\r",
				    TEMP_NEG_10_THRESHOLD, TEMP_POS_0_THRESHOLD);

			jeita_state = TEMP_NEG_10_TO_POS_0;

	} else {
		battery_log(BAT_LOG_CRTI,
			    "[PE3.0] Battery below low Temperature(%d) !!\n\r",
			    TEMP_NEG_10_THRESHOLD);
		jeita_state = TEMP_BELOW_NEG_10;

		return false;
	}

	return true;
}

static int mtk_pep30_get_cv(void)
{
	int cv = BAT_UPPER_BOUND;

	if (is_jeita_enable == true) {

		if (batteryTemperature == -1000)
			batteryTemperature = battery_meter_get_battery_temperature();
		cv = pep30_select_jeita_cv();
	}

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_get_cv]cv = %d, jeita:%d\n", cv, is_jeita_enable);

	return cv;
}

static int mtk_pep30_get_cali_vbus(int charging_current)
{
	int vbus = mtk_pep30_get_cv() + 70 + 110 * charging_current / 100 * r_total / 1000;

	return vbus;
}

bool mtk_pep30_safety_check(void)
{
	int vbus, vbus_cv, bifvbat;
	int pmic_vbat = 0, pmic_vbat_ovp;
	struct mtk_pep30_cap cap, setting, cap_b;
	int tmp_min, tmp_max;
	int ret;
	signed int cur;
	kal_bool sign;

	setting.vol = 0;
	setting.cur = 0;
	cap.vol = 0;
	cap.cur = 0;
	cur = battery_meter_get_battery_current();
	sign = battery_meter_get_battery_current_sign();
	ret = mtk_pep30_get_ta_boundary_cap(&cap_b);

	/*vbus ov: auxadc by PMIC */
	vbus = battery_meter_get_charger_voltage();
	if (pep30_charging_state != DC_MEASURE_R && pep30_charging_state != DC_INIT) {
		vbus_cv = BAT_UPPER_BOUND + 70 + 110 * CC_INIT / 100 * r_cable;
		if (vbus > vbus_cv) {
			battery_log(BAT_LOG_CRTI, "[%s]%d vbus %d > vbus_cv %d , r_cable:%d end pep30\n",
			__func__, pep30_charging_state, vbus, vbus_cv, r_cable);
			goto _fail;
		}
	}

	/*vbat ovp: auxadc by pmic */
	pmic_vbat = battery_meter_get_battery_voltage(KAL_TRUE);
	ret = battery_meter_get_bif_battery_voltage(&bifvbat);
	if (pep30_charging_state != DC_MEASURE_R && pep30_charging_state != DC_INIT) {
		if (ret >= 0)
			pmic_vbat_ovp = BAT_UPPER_BOUND + 50 + 110 * current_cap.cur / 100 * r_vbat;
		else
			pmic_vbat_ovp = BAT_UPPER_BOUND + 50;
		if (pmic_vbat > pmic_vbat_ovp) {
			battery_log(BAT_LOG_CRTI, "[%s]pmic_vbat:%d pmic_vbat_ovp:%d vbatCv:%d current:%d r_vbat:%d\n",
			__func__, pmic_vbat, pmic_vbat_ovp, BAT_UPPER_BOUND, current_cap.cur, r_vbat);
			goto _fail;
		}

		if (ret >= 0 && bifvbat >= BAT_UPPER_BOUND + 50) {
			battery_log(BAT_LOG_CRTI, "[%s]bifvbat %d >= BAT_UPPER_BOUND:%d + 50\n",
			__func__, bifvbat, BAT_UPPER_BOUND + 50);
			goto _fail;
		}
	}

	/* ibus oc:TA ADC current*/
	mtk_pep30_get_ta_current_cap(&cap);
	mtk_pep30_get_ta_cap(&setting);

#ifdef PE30_CHECK_TA
	if (cap.cur >= CC_MAX) {
		battery_log(BAT_LOG_CRTI, "[%s]cur %d >= CC_MAX %d , end pep30\n", __func__, cap.cur, CC_MAX);
		goto _fail;
	}

	if ((cap.vol - setting.vol) > VBUS_OV_GAP) {
		battery_log(BAT_LOG_CRTI, "[%s]vbus:%d > vbuus_b:%d gap:%d, end pep30\n", __func__, cap.vol,
		setting.vol, VBUS_OV_GAP);
		goto _fail;
	}
#endif

	ret = mtk_chr_get_tchr(&tmp_min, &tmp_max);
	if (ret < 0 || tmp_max >= CHARGER_TEMP_MAX) {
		battery_log(BAT_LOG_CRTI, "[%s]chr tmp %d >= %d ,ret:%d  end pep30\n",
		__func__, tmp_max, CHARGER_TEMP_MAX, ret);
		goto _fail;
	}

	if (get_battery_status() == false) {
		battery_log(BAT_LOG_CRTI, "[%s]no battery, end pep30\n",
		__func__);
		goto _fail;
	}

	if (is_jeita_enable == false && batteryTemperature >= BATTERY_TEMP_MAX) {
		battery_log(BAT_LOG_CRTI, "[%s]battery temperature is too high %d, end pep30\n",
		__func__, batteryTemperature);
		goto _fail;
	}

	if (is_jeita_enable == false && batteryTemperature <= PE30_MIN_CHARGE_TEMPERATURE) {
		battery_log(BAT_LOG_CRTI, "[%s]battery temperature is too low %d, end pep30\n",
		__func__, batteryTemperature);
		goto _fail;
	}


	battery_log(BAT_LOG_CRTI,
	"[%d]fg:%d:%d:dif:%d:[cur]:%d:%d:[vol]:%d:%d:vbus:%d:vbat:%d:bifvbat:%d:soc:%d:uisoc2:%d:tmp:%d:b:%d:%d:\n",
		pep30_charging_state, sign, cur, cap.cur - cur/10,
		cap.cur, setting.cur, cap.vol, setting.vol, vbus, pmic_vbat, bifvbat,
		BMT_status.SOC, BMT_status.UI_SOC2,
		batteryTemperature, cap_b.cur, cap_b.vol);

	return true;

_fail:

	battery_log(BAT_LOG_CRTI,
	"[%d]fg:%d:%d:dif:%d:[cur]:%d:%d:[vol]:%d:%d:vbus:%d:vbat:%d:bifvbat:%d:soc:%d:uisoc2:%d:tmp:%d:b:%d:%d:\n",
		pep30_charging_state, sign, cur, cap.cur - cur/10,
		cap.cur, setting.cur, cap.vol, setting.vol, vbus, pmic_vbat, bifvbat,
		BMT_status.SOC, BMT_status.UI_SOC2,
		batteryTemperature, cap_b.cur, cap_b.vol);

	return false;

}

static bool mtk_pep30_is_vdm_rdy(void)
{
	int ret;
	int i = 0;

	do {
		ret = mtk_vdm_config_dfp();
		battery_log(BAT_LOG_CRTI, "[mtk_vdm_config_dfp]ret = %d\n", ret);
		msleep(1000);
		i++;
		if (i >= 10)
			break;
	} while (ret != 0);

	if (ret != 0)
		return false;

	for (i = 0; i < 3; i++)
		if (mtk_get_ta_id(tcpc) == MANUFACTUREID)
			return true;

	return false;

}

static bool is_mtk_pep30_rdy(void)
{
	int vbat = 0;
	int ret = false;

	if (is_pep30_done == true)
		goto _fail;

	if (pep30_charging_state != DC_STOP)
		goto _fail;

	if (mtk_check_pe_ready_snk() == false)
		goto _fail;
	else {
		if (is_vdm_rdy == VDM_UNDET) {
			if (mtk_pep30_is_vdm_rdy() == true)
				is_vdm_rdy = VDM_VALID;
			else {
				is_vdm_rdy = VDM_INVALID;
				goto _fail;
			}
		} else if (is_vdm_rdy == VDM_INVALID)
			goto _fail;
	}

	vbat = battery_meter_get_battery_voltage(KAL_TRUE);
	if (vbat < BAT_UPPER_BOUND && vbat > BAT_LOWER_BOUND)
		ret = true;

	battery_log(BAT_LOG_CRTI,
	"[is_mtk_pep30_rdy]vbat=%d max/min=%d %d ret=%d is_pep30_done:%d is_pd_rdy:%d is_vdm_rdy:%d\n",
		vbat, BAT_UPPER_BOUND, BAT_LOWER_BOUND, ret, is_pep30_done, mtk_check_pe_ready_snk(), is_vdm_rdy);

	return ret;

_fail:

	battery_log(BAT_LOG_CRTI,
	"[is_mtk_pep30_rdy]vbat=%d max/min=%d %d ret=%d is_pep30_done:%d is_pd_rdy:%d is_vdm_rdy:%d\n",
		vbat, BAT_UPPER_BOUND, BAT_LOWER_BOUND, ret, is_pep30_done, mtk_check_pe_ready_snk(), is_vdm_rdy);


	return ret;
}

static void mtk_pep30_start(void)
{
	is_pep30_done = true;
	pep30_charging_state = DC_INIT;
	wake_up_pep30_thread();
}

bool mtk_is_TA_support_pep30(void)
{
	return is_vdm_rdy;
}

bool mtk_is_pep30_running(void)
{
	if (pep30_charging_state == DC_STOP)
		return false;
		return true;
}

void mtk_pep30_set_charging_current_limit(int cur)
{
	charging_current_limit = cur;
}

int mtk_pep30_get_charging_current_limit(void)
{
	return charging_current_limit;
}

void mtk_pep30_plugout_reset(void)
{
	is_pep30_done = false;
	is_vdm_rdy = VDM_UNDET;
	pep30_charging_state = DC_STOP;
	tcpm_set_direct_charge_en(tcpc, false);
	mtk_pep30_enable_direct_charge(false);
	mtk_pep30_set_charging_current_limit(CC_INIT);
}

void mtk_pep30_end(void)
{
	int ret;
	struct mtk_vdm_ta_cap cap;

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_end]state = %d\n", pep30_charging_state);

	if (pep30_charging_state == DC_STOP)
		return;

	tcpm_set_direct_charge_en(tcpc, false);

	cap.cur = 3000;
	cap.vol = 5000;
	ret = mtk_set_ta_cap(tcpc, &cap);
	if (ret != 0)
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_end]reset vbus fail,err = %d\n", ret);

	mtk_pep30_enable_direct_charge(false);
	is_pep30_done = true;
	pep30_charging_state = DC_STOP;

	wake_up_bat3();
}

bool mtk_pep30_check_charger(void)
{
	if (is_mtk_pep30_rdy()) {
		mtk_chr_enable_charge(false);
		mtk_pep30_start();
		return true;
	}
	return false;
}


static enum hrtimer_restart mtk_charger_pep30_timer_callback(struct hrtimer *timer)
{
	mtk_charger_pep30_thread_flag = true;
	wake_up_interruptible(&mtk_charger_pep30_thread_waiter);

	battery_log(BAT_LOG_FULL, "[mtk_charger_pep30_timer_callback]\n");

	return HRTIMER_NORESTART;
}

static void mtk_pep30_DC_init(void)
{
	int vbat;
	int ret;
	struct mtk_pep30_cap cap_now;

	tcpm_set_direct_charge_en(tcpc, true);

	vbat = battery_meter_get_battery_voltage(KAL_TRUE);

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_init]state = %d ,vbat = %d\n", pep30_charging_state, vbat);

	mtk_chr_enable_power_path(false);
	msleep(100);
	ret = mtk_pep30_get_ta_current_cap(&cap_now);
	mtk_chr_enable_power_path(true);
	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_init]FOD cur:%d vol:%d\n", cap_now.cur, cap_now.vol);
	if (cap_now.cur > FOD_CURRENT)
		goto _fail;

	ret = mtk_pep30_set_ta_boundary_cap(CC_INIT, vbat + 50);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_init]err1 = %d\n", ret);
		goto _fail;
	}

	ret = mtk_pep30_set_ta_cap(CC_SS_INIT, vbat + 50);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_init]err2 = %d\n", ret);
		goto _fail;
	}

	msleep(CC_SS_BLANKING);

	mtk_pep30_enable_direct_charge(true);
	pep30_charging_state = DC_MEASURE_R;
	ktime = ktime_set(0, BAT_MS_TO_NS(CC_SS_BLANKING));

	return;
_fail:
	mtk_pep30_end();

}

static void mtk_pep30_DC_measure_R(void)
{
	struct mtk_pep30_cap cap1, cap2;
	int vbus1, vbat1, vbus2, vbat2, bifvbat1 = 0, bifvbat2 = 0;
	int fgcur1, fgcur2;
	int ret;

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_measure_R]vbus = %d\n", battery_meter_get_charger_voltage());

	/* set boundary */
	ret = mtk_pep30_set_ta_boundary_cap(CC_INIT, CV_END);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_measure_R]err1 = %d\n", ret);
		goto _fail;
	}

	/* measure 1 */
	ret = mtk_pep30_set_ta_cap(1500, CV_END);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_measure_R]err2 = %d\n", ret);
		goto _fail;
	}
	msleep(300);

	vbus1 = battery_meter_get_charger_voltage();
	vbat1 = battery_meter_get_battery_voltage(KAL_TRUE);
	battery_meter_get_bif_battery_voltage(&bifvbat1);
	fgcur1 = battery_meter_get_battery_current() / 10;
	ret = mtk_pep30_get_ta_current_cap(&cap1);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_measure_R]err3 = %d\n", ret);
		goto _fail;
	}

	if (bifvbat1 != 0 && bifvbat1 >= mtk_pep30_get_cv()) {
			battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_measure_R]end bifvbat1=%d cur:%d vol:%d\n",
			bifvbat1, cap1.cur, cap1.vol);
			goto _fail;
	}

	/* measure 2 */
	ret = mtk_pep30_set_ta_cap(2000, CV_END);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_measure_R]err2 = %d\n", ret);
		goto _fail;
	}
	msleep(300);

	vbus2 = battery_meter_get_charger_voltage();
	vbat2 = battery_meter_get_battery_voltage(KAL_TRUE);
	battery_meter_get_bif_battery_voltage(&bifvbat2);
	fgcur2 = battery_meter_get_battery_current() / 10;
	ret = mtk_pep30_get_ta_current_cap(&cap2);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_measure_R]err3 = %d\n", ret);
		goto _fail;
	}

	if (bifvbat2 != 0 && bifvbat2 >= mtk_pep30_get_cv()) {
			battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_measure_R]end bifvbat2=%d cur:%d vol:%d\n",
			bifvbat2, cap2.cur, cap2.vol);
			goto _fail;
	}

	if (cap2.cur == cap1.cur)
		return;

	if (bifvbat1 != 0 && bifvbat2 != 0)
		r_vbat = abs((vbat2 - bifvbat2) - (vbat1 - bifvbat1)) * 1000 / abs(fgcur2 - fgcur1);

	r_sw = abs((vbus2 - vbus1) - (vbat2 - vbat1)) * 1000 / abs(cap2.cur - cap1.cur);
	r_cable = abs((cap2.vol - cap1.vol)-(vbus2 - vbus1)) * 1000 / abs(cap2.cur - cap1.cur);

	r_total = r_cable + r_sw + r_vbat;

	battery_log(BAT_LOG_CRTI,
	"[mtk_pep30_DC_measure_R]Tibus:%d %d Tvbus:%d %d vbus:%d %d vbat:%d %d bifvbat:%d %d fgcur:%d %dr_cable:%d r_sw:%d r_vbat:%d\n",
	cap2.cur, cap1.cur, cap2.vol, cap1.vol, vbus2, vbus1, vbat2,
	vbat1, bifvbat2, bifvbat1, fgcur2, fgcur1, r_cable, r_sw, r_vbat);

	pep30_charging_state = DC_SOFT_START;

	return;
_fail:
	mtk_pep30_end();

}


static void mtk_pep30_DC_soft_start(void)
{
	int ret;
	int vbat;
	int cv;
	struct mtk_pep30_cap cap, setting, cap_now;
	int current_setting;

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]vbus = %d\n", battery_meter_get_charger_voltage());

	ret = mtk_pep30_get_ta_current_cap(&cap_now);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]err1 = %d\n", ret);
		goto _fail;
	}

	ret = mtk_pep30_get_ta_cap(&setting);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]err2 = %d\n", ret);
		goto _fail;
	}

	current_setting = setting.cur;

	vbat = battery_meter_get_battery_voltage(KAL_TRUE);

	if (vbat >= mtk_pep30_get_cv()) {

		if (cap_now.cur >= CC_END) {
			battery_log(BAT_LOG_CRTI,
				"[mtk_pep30_DC_soft_start]go to CV vbat:%d current:%d setting current:%d\n",
				vbat, cap_now.cur, current_setting);
			pep30_charging_state = DC_CC;
			ktime = ktime_set(0, BAT_MS_TO_NS(CC_BLANKING));
			return;
		}
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]end vbat:%d current:%d setting current:%d\n",
			vbat, cap_now.cur, current_setting);
		goto _fail;

	}

	if (setting.cur >= CC_INIT) {
		pep30_charging_state = DC_CC;
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]go to CV2 vbat:%d current:%d setting current:%d\n",
			vbat, cap_now.cur, current_setting);
		ktime = ktime_set(0, BAT_MS_TO_NS(CC_BLANKING));
		return;
	}

	current_setting = current_setting + CC_SS_STEP;

	cv = mtk_pep30_get_cali_vbus(current_setting);

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]vbat:%d cv = %d cur:%d r:%d cap_now.cur:%d\n",
		vbat, cv , current_setting, r_total,
		cap_now.cur);

	ret = mtk_pep30_set_ta_boundary_cap(CC_INIT, CV_LIMIT);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]err4 = %d\n", ret);
		goto _fail;
	}

	if (cv >= CV_LIMIT)
		cv = CV_LIMIT;

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]before mtk_set_ta_cap, cur:%d vol:%d:%d\n",
		cap.cur, cap.vol, cv);
	ret = mtk_pep30_set_ta_cap(current_setting, cv);


	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]cur = %d vol = %d\n", current_setting, cv);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]err5 = %d\n", ret);
		goto _fail;
	}

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_soft_start]setting:%d %d now:%d %d boundary:%d %d cap:%d %d\n",
		setting.cur, setting.vol,
		cap_now.cur, cap_now.vol, CC_INIT, CV_LIMIT, current_setting, cv);


	return;
_fail:
	mtk_pep30_end();


}

static void mtk_pep30_DC_cc(void)
{
	struct mtk_pep30_cap setting, cap_now;
	int vbat = 0;
	int ret;
	int TAVbus, current_setting, cv;
	int tmp;
	int tmp_min, tmp_max;

	battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]state = %d\n", pep30_charging_state);

	ret = battery_meter_get_bif_battery_voltage(&vbat);
	if (ret < 0)
		vbat = battery_meter_get_battery_voltage(KAL_TRUE);

	ret = mtk_pep30_get_ta_current_cap(&cap_now);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]err1 = %d\n", ret);
		goto _fail;
	}

	if (cap_now.cur <= CC_END) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]pe30 end , current:%d\n", cap_now.cur);
		goto _fail;
	}

	ret = mtk_pep30_get_ta_cap(&setting);
	if (ret != 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]err2 = %d\n", ret);
		goto _fail;
	}

	current_setting = setting.cur;
	tmp = battery_meter_get_battery_temperature();

	ret = mtk_chr_get_tchr(&tmp_min, &tmp_max);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]err3 = %d\n", ret);
		goto _fail;
	}

	cv = mtk_pep30_get_cv();
	if (vbat >= cv || tmp_max >= CHARGER_TEMP_MAX) {

		current_setting -= CC_STEP;
		TAVbus = mtk_pep30_get_cali_vbus(current_setting);

		if (TAVbus >= CV_LIMIT)
			TAVbus = CV_LIMIT;

		if (TAVbus <= 5500)
			TAVbus = 5500;

		ret = mtk_pep30_set_ta_boundary_cap(CC_INIT, TAVbus);
		if (ret != 0) {
			battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]err6 = %d\n", ret);
			goto _fail;
		}

		if (current_setting >= mtk_pep30_get_charging_current_limit())
			current_setting = mtk_pep30_get_charging_current_limit();

		ret = mtk_pep30_set_ta_cap(current_setting, TAVbus);
		if (ret != 0) {
			battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]err7 = %d\n", ret);
			goto _fail;
		}

		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]- vbat:%d current:%d:%d r:%d TAVBUS:%d cv:%d tmp_max:%d\n",
			vbat, current_setting, mtk_pep30_get_charging_current_limit(), r_total, TAVbus, cv, tmp_max);
	} else {

		current_setting += CC_STEP;
		TAVbus = mtk_pep30_get_cali_vbus(current_setting);

		if (TAVbus >= CV_LIMIT)
			TAVbus = CV_LIMIT;

		if (TAVbus <= 5500)
			TAVbus = 5500;

		ret = mtk_pep30_set_ta_boundary_cap(CC_INIT, TAVbus);
		if (ret != 0) {
			battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]err8 = %d\n", ret);
			goto _fail;
		}

		if (current_setting >= mtk_pep30_get_charging_current_limit())
			current_setting = mtk_pep30_get_charging_current_limit();

		ret = mtk_pep30_set_ta_cap(current_setting, TAVbus);
		if (ret != 0) {
			battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]err9 = %d\n", ret);
			goto _fail;
		}

		battery_log(BAT_LOG_CRTI, "[mtk_pep30_DC_CC]+ vbat:%d current:%d:%d r:%d TAVBUS:%d cv:%d\n",
			vbat, current_setting, mtk_pep30_get_charging_current_limit(), r_total, TAVbus, cv);
	}

	return;

_fail:
	mtk_pep30_end();

}

static int mtk_charger_pep30_thread_handler(void *unused)
{


	ktime = ktime_set(0, BAT_MS_TO_NS(500));
	do {
		wait_event_interruptible(mtk_charger_pep30_thread_waiter,
					 (mtk_charger_pep30_thread_flag == true));

		mtk_charger_pep30_thread_flag = false;

		batteryTemperature = battery_meter_get_battery_temperature();

		battery_log(BAT_LOG_CRTI, "[mtk_charger_pep30_thread_handler]state = %d tmp:%d jstate:%d\n",
		pep30_charging_state, batteryTemperature, jeita_state);


		if (is_jeita_enable == true)
			if (pe30_do_jeita_state_machine() == false)
				mtk_pep30_end();



		switch (pep30_charging_state) {
		case DC_INIT:
			mtk_pep30_DC_init();
			break;

		case DC_MEASURE_R:
			mtk_pep30_DC_measure_R();
			break;

		case DC_SOFT_START:
			mtk_pep30_DC_soft_start();
			break;

		case DC_CC:
			mtk_pep30_DC_cc();
			break;

		case DC_STOP:
			break;
		}

		if (pep30_charging_state != DC_STOP) {
			mtk_pep30_safety_check();
			hrtimer_start(&mtk_charger_pep30_timer, ktime, HRTIMER_MODE_REL);
		}
	} while (!kthread_should_stop());

	return 0;
}

bool mtk_pep30_init(void)
{
	ktime_t ktime;

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	is_jeita_enable = true;
#else
	is_jeita_enable = false;
#endif

	mtk_direct_charge_vdm_init();
	tcpc = tcpc_dev_get_by_name("type_c_port0");
	ktime = ktime_set(0, BAT_MS_TO_NS(2000));
	hrtimer_init(&mtk_charger_pep30_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mtk_charger_pep30_timer.function = mtk_charger_pep30_timer_callback;

	mtk_charger_pep30_thread =
	    kthread_run(mtk_charger_pep30_thread_handler, 0,
			"bat_mtk_charger_pep30");
	if (IS_ERR(mtk_charger_pep30_thread)) {
		battery_log(BAT_LOG_FULL,
			    "[%s]: failed to create mtk_charger_pep30 thread\n",
			    __func__);
	}

	battery_log(BAT_LOG_CRTI, "mtk_charger_pep30 : done\n");

	return true;

}
