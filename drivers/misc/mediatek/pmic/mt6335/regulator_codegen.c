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

#include <mt-plat/upmu_common.h>
#include "include/regulator_codegen.h"
#include "include/pmic.h"
#include "include/pmic_api.h"
#include "include/pmic_api_ldo.h"
#include "include/pmic_api_buck.h"

static const int vio28_voltages[] = {
	2800000,
};

static const int vio18_voltages[] = {
	1800000,
};

static const int vufs18_voltages[] = {
	1800000,
};

static const int va10_voltages[] = {
	800000,
	850000,
	875000,
	900000,
	950000,
	1000000,
};

static const int va12_voltages[] = {
	1200000,
};

static const int va18_voltages[] = {
	1800000,
};

static const int vusb33_voltages[] = {
	3300000,
};

static const int vemc_voltages[] = {
	2900000, /*--dummy--*/
	2900000,
	3000000,
	3300000,
};

static const int vxo22_voltages[] = {
	2200000,
};

static const int vefuse_voltages[] = {
	1200000, /*---dummy ---*/
	1200000, /*---dummy ---*/
	1200000, /*---dummy ---*/
	1200000, /*---dummy ---*/
	1200000, /*---dummy ---*/
	1200000, /*---dummy ---*/
	1200000, /*---dummy ---*/
	1200000, /*---dummy ---*/
	1200000,
	1300000,
	1700000,
	1800000,
	1900000,
	2000000,
	2100000,
	2200000,
};

static const int vsim1_voltages[] = {
	1200000,
	1300000,
	1700000,
	1800000,
	1860000,
	2760000,
	3000000,
	3100000,
};

static const int vsim2_voltages[] = {
	1200000,
	1300000,
	1700000,
	1800000,
	1860000,
	2760000,
	3000000,
	3100000,
};

static const int vcamaf_voltages[] = {
	2800000,
};

static const int vtouch_voltages[] = {
	2800000,
};

static const int vcamd1_voltages[] = {
	900000,
	950000,
	1000000,
	1050000,
	1100000,
	1200000,
	1210000,
};

static const int vcamd2_voltages[] = {
	900000,
	1000000,
	1050000,
	1100000,
	1210000,
	1300000,
	1500000,
	1800000,
};

static const int vcamio_voltages[] = {
	1800000,
};

static const int vmipi_voltages[] = {
	1800000,
};

static const int vgp3_voltages[] = {
	1000000,
	1050000,
	1100000,
	1220000,
	1300000,
	1500000,
	1800000,
};

static const int vcn33_voltages[] = {
	3300000,
	3400000,
	3500000,
	3600000,
};

static const int vcn18_voltages[] = {
	1800000,
};

static const int vcn28_voltages[] = {
	2800000,
};

static const int vibr_voltages[] = {
	1200000,
	1300000,
	1500000,
	1800000,
	2000000,
	2800000,
	3000000,
	3300000,
};

static const int vbif28_voltages[] = {
	2800000,
};

static const int vfe28_voltages[] = {
	2800000,
};

static const int vmch_voltages[] = {
	2900000, /*--dummy--*/
	2900000,
	3000000,
	3300000,
};

static const int vmc_voltages[] = {
	1860000,
	2900000,
	3000000,
	3300000,
};

static const int vrf18_voltages[] = {
	1810000,
};

static const int vrf12_voltages[] = {
	1200000,
};

static const int vcama1_voltages[] = {
	1800000,
	2800000,
	2900000,
	2500000,
};

static const int vcama2_voltages[] = {
	1800000,
	2800000,
	2900000,
	3000000,
};

/* Regulator vio28 enable */
static int pmic_ldo_vio28_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vio28 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vio28_sw_en(1);
	return 0;
}

/* Regulator vio28 disable */
static int pmic_ldo_vio28_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vio28 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vio28_sw_en(0);
	return ret;
}

/* Regulator vio28 is_enabled */
static int pmic_ldo_vio28_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vio28 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vio28_en();
}

/* Regulator vio28 set_voltage_sel */
static int pmic_ldo_vio28_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vio28 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vio28_vosel(selector);*/
	return 0;
}

/* Regulator vio28 get_voltage_sel */
static int pmic_ldo_vio28_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vio28 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vio28_vosel();*/
	return 0;
}

/* Regulator vio28 list_voltage */
static int pmic_ldo_vio28_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vio28_voltages[selector];
	pr_debug("ldo vio28 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vio18 enable */
static int pmic_ldo_vio18_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vio18 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vio18_sw_en(1);
	return 0;
}

/* Regulator vio18 disable */
static int pmic_ldo_vio18_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vio18 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vio18_sw_en(0);
	return ret;
}

/* Regulator vio18 is_enabled */
static int pmic_ldo_vio18_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vio18 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vio18_en();
}

/* Regulator vio18 set_voltage_sel */
static int pmic_ldo_vio18_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("ldo vio18 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vio18_vosel(selector);*/
	return 0;
}

/* Regulator vio18 get_voltage_sel */
static int pmic_ldo_vio18_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vio18 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vio18_vosel();*/
	return 0;
}

/* Regulator vio18 list_voltage */
static int pmic_ldo_vio18_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vio18_voltages[selector];
	pr_debug("ldo vio18 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vufs18 enable */
static int pmic_ldo_vufs18_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vufs18 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vufs18_sw_en(1);
	return 0;
}

/* Regulator vufs18 disable */
static int pmic_ldo_vufs18_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vufs18 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vufs18_sw_en(0);
	return ret;
}

/* Regulator vufs18 is_enabled */
static int pmic_ldo_vufs18_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vufs18 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vufs18_en();
}

/* Regulator vufs18 set_voltage_sel */
static int pmic_ldo_vufs18_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("ldo vufs18 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vufs18_vosel(selector);*/
	return 0;
}

/* Regulator vufs18 get_voltage_sel */
static int pmic_ldo_vufs18_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vufs18 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vufs18_vosel();*/
	return 0;
}

/* Regulator vufs18 list_voltage */
static int pmic_ldo_vufs18_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vufs18_voltages[selector];
	pr_debug("ldo vufs18 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator va10 enable */
static int pmic_ldo_va10_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo va10 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_va10_sw_en(1);
	return 0;
}

/* Regulator va10 disable */
static int pmic_ldo_va10_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo va10 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_va10_sw_en(0);
	return ret;
}

/* Regulator va10 is_enabled */
static int pmic_ldo_va10_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo va10 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_va10_en();
}

/* Regulator va10 set_voltage_sel */
static int pmic_ldo_va10_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo va10 set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_va10_vosel(selector);
	return 0;
}

/* Regulator va10 get_voltage_sel */
static int pmic_ldo_va10_get_voltage_sel(struct regulator_dev *rdev)
{
	unsigned int regVal = 0;
	/*pr_debug("ldo va10 get_voltage_sel\n");*/

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	regVal = mt6335_upmu_get_rg_va10_vosel();
	pr_debug("ldo va10 get_voltage_sel %d\n", regVal);
	return regVal;
}

/* Regulator va10 list_voltage */
static int pmic_ldo_va10_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = va10_voltages[selector];
	pr_debug("ldo va10 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator va12 enable */
static int pmic_ldo_va12_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo va12 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_va12_sw_en(1);
	return 0;
}

/* Regulator va12 disable */
static int pmic_ldo_va12_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo va12 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_va12_sw_en(0);
	return ret;
}

/* Regulator va12 is_enabled */
static int pmic_ldo_va12_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo va12 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_va12_en();
}

/* Regulator va12 set_voltage_sel */
static int pmic_ldo_va12_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo va12 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_va12_vosel(selector);*/
	return 0;
}

/* Regulator va12 get_voltage_sel */
static int pmic_ldo_va12_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo va12 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_va12_vosel();*/
	return 0;
}

/* Regulator va12 list_voltage */
static int pmic_ldo_va12_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = va12_voltages[selector];
	pr_debug("ldo va12 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator va18 enable */
static int pmic_ldo_va18_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo va18 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_va18_sw_en(1);
	return 0;
}

/* Regulator va18 disable */
static int pmic_ldo_va18_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo va18 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_va18_sw_en(0);
	return ret;
}

/* Regulator va18 is_enabled */
static int pmic_ldo_va18_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo va18 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_va18_en();
}

/* Regulator va18 set_voltage_sel */
static int pmic_ldo_va18_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo va18 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_va18_vosel(selector);*/
	return 0;
}

/* Regulator va18 get_voltage_sel */
static int pmic_ldo_va18_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo va18 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_va18_vosel();*/
	return 0;
}

/* Regulator va18 list_voltage */
static int pmic_ldo_va18_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = va18_voltages[selector];
	pr_debug("ldo va18 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vusb33 enable */
static int pmic_ldo_vusb33_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vusb33 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vusb33_sw_en(1);
	return 0;
}

/* Regulator vusb33 disable */
static int pmic_ldo_vusb33_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vusb33 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vusb33_sw_en(0);
	return ret;
}

/* Regulator vusb33 is_enabled */
static int pmic_ldo_vusb33_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vusb33 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vusb33_en();
}

/* Regulator vusb33 set_voltage_sel */
static int pmic_ldo_vusb33_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("ldo vusb33 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vusb33_vosel(selector);*/
	return 0;
}

/* Regulator vusb33 get_voltage_sel */
static int pmic_ldo_vusb33_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vusb33 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vusb33_vosel();*/
	return 0;
}

/* Regulator vusb33 list_voltage */
static int pmic_ldo_vusb33_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vusb33_voltages[selector];
	pr_debug("ldo vusb33 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vemc enable */
static int pmic_ldo_vemc_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vemc enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vemc_sw_en(1);
	return 0;
}

/* Regulator vemc disable */
static int pmic_ldo_vemc_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vemc disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vemc_sw_en(0);
	return ret;
}

/* Regulator vemc is_enabled */
static int pmic_ldo_vemc_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vemc is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vemc_en();
}

/* Regulator vemc set_voltage_sel */
static int pmic_ldo_vemc_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	pr_debug("ldo vemc set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	if (selector == 0)
		selector = 1;

	mreg->vosel.cur_sel = selector;

	mt6335_upmu_set_rg_vemc_vosel(selector);
	return 0;
}

/* Regulator vemc get_voltage_sel */
static int pmic_ldo_vemc_get_voltage_sel(struct regulator_dev *rdev)
{
	unsigned char regVal = 0;

	regVal = mt6335_upmu_get_rg_vemc_vosel();
	pr_debug("ldo vemc get_voltage_sel %d\n", regVal);
	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return regVal;
}

/* Regulator vemc list_voltage */
static int pmic_ldo_vemc_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vemc_voltages[selector];
	pr_debug("ldo vemc list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vxo22 enable */
static int pmic_ldo_vxo22_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vxo22 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vxo22_sw_en(1);
	return 0;
}

/* Regulator vxo22 disable */
static int pmic_ldo_vxo22_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vxo22 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vxo22_sw_en(0);
	return ret;
}

/* Regulator vxo22 is_enabled */
static int pmic_ldo_vxo22_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vxo22 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vxo22_en();
}

/* Regulator vxo22 set_voltage_sel */
static int pmic_ldo_vxo22_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vxo22 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vxo22_vosel(selector);*/
	return 0;
}

/* Regulator vxo22 get_voltage_sel */
static int pmic_ldo_vxo22_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vxo22 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vxo22_vosel();*/
	return 0;
}

/* Regulator vxo22 list_voltage */
static int pmic_ldo_vxo22_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vxo22_voltages[selector];
	pr_debug("ldo vxo22 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vefuse enable */
static int pmic_ldo_vefuse_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vefuse enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vefuse_sw_en(1);
	return 0;
}

/* Regulator vefuse disable */
static int pmic_ldo_vefuse_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vefuse disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vefuse_sw_en(0);
	return ret;
}

/* Regulator vefuse is_enabled */
static int pmic_ldo_vefuse_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vefuse is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vefuse_en();
}

/* Regulator vefuse set_voltage_sel */
static int pmic_ldo_vefuse_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	if (selector >= 0 && selector < 8)
		selector = 9;

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vefuse set_voltage_sel: %d\n", selector);

	mt6335_upmu_set_rg_vefuse_vosel(selector);
	return 0;
}

/* Regulator vefuse get_voltage_sel */
static int pmic_ldo_vefuse_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vefuse get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vefuse_vosel();
}

/* Regulator vefuse list_voltage */
static int pmic_ldo_vefuse_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vefuse_voltages[selector];
	pr_debug("ldo vefuse list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vsim1 enable */
static int pmic_ldo_vsim1_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vsim1 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vsim1_sw_en(1);
	return 0;
}

/* Regulator vsim1 disable */
static int pmic_ldo_vsim1_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vsim1 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vsim1_sw_en(0);
	return ret;
}

/* Regulator vsim1 is_enabled */
static int pmic_ldo_vsim1_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vsim1 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vsim1_en();
}

/* Regulator vsim1 set_voltage_sel */
static int pmic_ldo_vsim1_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vsim1 set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vsim1_vosel(selector);
	return 0;
}

/* Regulator vsim1 get_voltage_sel */
static int pmic_ldo_vsim1_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vsim1 get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vsim1_vosel();
}

/* Regulator vsim1 list_voltage */
static int pmic_ldo_vsim1_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vsim1_voltages[selector];
	pr_debug("ldo vsim1 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vsim2 enable */
static int pmic_ldo_vsim2_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vsim2 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vsim2_sw_en(1);
	return 0;
}

/* Regulator vsim2 disable */
static int pmic_ldo_vsim2_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vsim2 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vsim2_sw_en(0);
	return ret;
}

/* Regulator vsim2 is_enabled */
static int pmic_ldo_vsim2_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vsim2 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vsim2_en();
}

/* Regulator vsim2 set_voltage_sel */
static int pmic_ldo_vsim2_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vsim2 set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vsim2_vosel(selector);
	return 0;
}

/* Regulator vsim2 get_voltage_sel */
static int pmic_ldo_vsim2_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vsim2 get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vsim2_vosel();
}

/* Regulator vsim2 list_voltage */
static int pmic_ldo_vsim2_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vsim2_voltages[selector];
	pr_debug("ldo vsim2 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcamaf enable */
static int pmic_ldo_vcamaf_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamaf enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcamaf_sw_en(1);
	return 0;
}

/* Regulator vcamaf disable */
static int pmic_ldo_vcamaf_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcamaf disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcamaf_sw_en(0);
	return ret;
}

/* Regulator vcamaf is_enabled */
static int pmic_ldo_vcamaf_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamaf is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcamaf_en();
}

/* Regulator vcamaf set_voltage_sel */
static int pmic_ldo_vcamaf_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("ldo vcamaf dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vcamaf_vosel(selector);*/
	return 0;
}

/* Regulator vcamaf get_voltage_sel */
static int pmic_ldo_vcamaf_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamaf dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vcamaf_vosel();*/
	return 0;
}

/* Regulator vcamaf list_voltage */
static int pmic_ldo_vcamaf_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcamaf_voltages[selector];
	pr_debug("ldo vcamaf list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vtouch enable */
static int pmic_ldo_vtouch_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vtouch enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vtouch_sw_en(1);
	return 0;
}

/* Regulator vtouch disable */
static int pmic_ldo_vtouch_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vtouch disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vtouch_sw_en(0);
	return ret;
}

/* Regulator vtouch is_enabled */
static int pmic_ldo_vtouch_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vtouch is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vtouch_en();
}

/* Regulator vtouch set_voltage_sel */
static int pmic_ldo_vtouch_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("ldo vtouch set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vtouch_vosel(selector);*/
	return 0;
}

/* Regulator vtouch get_voltage_sel */
static int pmic_ldo_vtouch_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vtouch get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vtouch_vosel();*/
	return 0;
}

/* Regulator vtouch list_voltage */
static int pmic_ldo_vtouch_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vtouch_voltages[selector];
	pr_debug("ldo vtouch list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcamd1 enable */
static int pmic_ldo_vcamd1_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamd1 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcamd1_sw_en(1);
	return 0;
}

/* Regulator vcamd1 disable */
static int pmic_ldo_vcamd1_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcamd1 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcamd1_sw_en(0);
	return ret;
}

/* Regulator vcamd1 is_enabled */
static int pmic_ldo_vcamd1_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamd1 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcamd1_en();
}

/* Regulator vcamd1 set_voltage_sel */
static int pmic_ldo_vcamd1_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vcamd1 set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vcamd1_vosel(selector);
	return 0;
}

/* Regulator vcamd1 get_voltage_sel */
static int pmic_ldo_vcamd1_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamd1 get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vcamd1_vosel();
}

/* Regulator vcamd1 list_voltage */
static int pmic_ldo_vcamd1_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcamd1_voltages[selector];
	pr_debug("ldo vcamd1 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcamd2 enable */
static int pmic_ldo_vcamd2_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamd2 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcamd2_sw_en(1);
	return 0;
}

/* Regulator vcamd2 disable */
static int pmic_ldo_vcamd2_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcamd2 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcamd2_sw_en(0);
	return ret;
}

/* Regulator vcamd2 is_enabled */
static int pmic_ldo_vcamd2_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamd2 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcamd2_en();
}

/* Regulator vcamd2 set_voltage_sel */
static int pmic_ldo_vcamd2_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vcamd2 set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vcamd2_vosel(selector);
	return 0;
}

/* Regulator vcamd2 get_voltage_sel */
static int pmic_ldo_vcamd2_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamd2 get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vcamd2_vosel();
}

/* Regulator vcamd2 list_voltage */
static int pmic_ldo_vcamd2_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcamd2_voltages[selector];
	pr_debug("ldo vcamd2 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcamio enable */
static int pmic_ldo_vcamio_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamio enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcamio_sw_en(1);
	return 0;
}

/* Regulator vcamio disable */
static int pmic_ldo_vcamio_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcamio disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcamio_sw_en(0);
	return ret;
}

/* Regulator vcamio is_enabled */
static int pmic_ldo_vcamio_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcamio is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcamio_en();
}

/* Regulator vcamio set_voltage_sel */
static int pmic_ldo_vcamio_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vcamio dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vcamio_vosel(selector);*/
	return 0;
}

/* Regulator vcamio get_voltage_sel */
static int pmic_ldo_vcamio_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vcamio dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vcamio_vosel();*/
	return 0;
}

/* Regulator vcamio list_voltage */
static int pmic_ldo_vcamio_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcamio_voltages[selector];
	pr_debug("ldo vcamio list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vmipi enable */
static int pmic_ldo_vmipi_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vmipi enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vmipi_sw_en(1);
	return 0;
}

/* Regulator vmipi disable */
static int pmic_ldo_vmipi_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vmipi disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vmipi_sw_en(0);
	return ret;
}

/* Regulator vmipi is_enabled */
static int pmic_ldo_vmipi_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vmipi is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vmipi_en();
}

/* Regulator vmipi set_voltage_sel */
static int pmic_ldo_vmipi_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vmipi dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vmipi_vosel(selector);*/
	return 0;
}

/* Regulator vmipi get_voltage_sel */
static int pmic_ldo_vmipi_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vmipi dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vmipi_vosel();*/
	return 0;
}

/* Regulator vmipi list_voltage */
static int pmic_ldo_vmipi_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vmipi_voltages[selector];
	pr_debug("ldo vmipi list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vgp3 enable */
static int pmic_ldo_vgp3_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vgp3 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vgp3_sw_en(1);
	return 0;
}

/* Regulator vgp3 disable */
static int pmic_ldo_vgp3_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vgp3 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vgp3_sw_en(0);
	return ret;
}

/* Regulator vgp3 is_enabled */
static int pmic_ldo_vgp3_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vgp3 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vgp3_en();
}

/* Regulator vgp3 set_voltage_sel */
static int pmic_ldo_vgp3_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vgp3 set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vgp3_vosel(selector);
	return 0;
}

/* Regulator vgp3 get_voltage_sel */
static int pmic_ldo_vgp3_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vgp3 get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vgp3_vosel();
}

/* Regulator vgp3 list_voltage */
static int pmic_ldo_vgp3_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vgp3_voltages[selector];
	pr_debug("ldo vgp3 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcn33_bt enable */
static int pmic_ldo_vcn33_bt_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn33_bt enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcn33_bt_sw_en(1);
	return 0;
}

/* Regulator vcn33_bt disable */
static int pmic_ldo_vcn33_bt_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcn33_bt disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcn33_bt_sw_en(0);
	return ret;
}

/* Regulator vcn33_bt is_enabled */
static int pmic_ldo_vcn33_bt_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn33_bt is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcn33_en();
}

/* Regulator vcn33_bt set_voltage_sel */
static int pmic_ldo_vcn33_bt_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vcn33_bt set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vcn33_vosel(selector);
	return 0;
}

/* Regulator vcn33_bt get_voltage_sel */
static int pmic_ldo_vcn33_bt_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn33_bt get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vcn33_vosel();
}

/* Regulator vcn33_bt list_voltage */
static int pmic_ldo_vcn33_bt_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcn33_voltages[selector];
	pr_debug("ldo vcn33_bt list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcn33_wifi enable */
static int pmic_ldo_vcn33_wifi_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn33_wifi enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcn33_wifi_sw_en(1);
	return 0;
}

/* Regulator vcn33_wifi disable */
static int pmic_ldo_vcn33_wifi_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcn33_wifi disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcn33_wifi_sw_en(0);
	return ret;
}

/* Regulator vcn33_wifi is_enabled */
static int pmic_ldo_vcn33_wifi_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn33_wifi is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcn33_en();
}

/* Regulator vcn33_wifi set_voltage_sel */
static int pmic_ldo_vcn33_wifi_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vcn33_wifi set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vcn33_vosel(selector);
	return 0;
}

/* Regulator vcn33_wifi get_voltage_sel */
static int pmic_ldo_vcn33_wifi_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn33_wifi get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vcn33_vosel();
}

/* Regulator vcn33_wifi list_voltage */
static int pmic_ldo_vcn33_wifi_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcn33_voltages[selector];
	pr_debug("ldo vcn33_wifi list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcn18_bt enable */
static int pmic_ldo_vcn18_bt_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn18_bt enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcn18_bt_sw_en(1);
	return 0;
}

/* Regulator vcn18_bt disable */
static int pmic_ldo_vcn18_bt_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcn18_bt disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcn18_bt_sw_en(0);
	return ret;
}

/* Regulator vcn18_bt is_enabled */
static int pmic_ldo_vcn18_bt_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn18_bt is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcn18_en();
}

/* Regulator vcn18_bt set_voltage_sel */
static int pmic_ldo_vcn18_bt_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vcn18_bt dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vcn18_vosel(selector);*/
	return 0;
}

/* Regulator vcn18_bt get_voltage_sel */
static int pmic_ldo_vcn18_bt_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vcn18_bt dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vcn18_vosel();*/
	return 0;
}

/* Regulator vcn18_bt list_voltage */
static int pmic_ldo_vcn18_bt_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcn18_voltages[selector];
	pr_debug("ldo vcn18_bt list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcn18_wifi enable */
static int pmic_ldo_vcn18_wifi_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn18_wifi enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcn18_wifi_sw_en(1);
	return 0;
}

/* Regulator vcn18_wifi disable */
static int pmic_ldo_vcn18_wifi_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcn18_wifi disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcn18_wifi_sw_en(0);
	return ret;
}

/* Regulator vcn18_wifi is_enabled */
static int pmic_ldo_vcn18_wifi_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn18_wifi is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcn18_en();
}

/* Regulator vcn18_wifi set_voltage_sel */
static int pmic_ldo_vcn18_wifi_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vcn18_wifi dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vcn18_vosel(selector);*/
	return 0;
}

/* Regulator vcn18_wifi get_voltage_sel */
static int pmic_ldo_vcn18_wifi_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vcn18_wifi dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vcn18_vosel();*/
	return 0;
}

/* Regulator vcn18_wifi list_voltage */
static int pmic_ldo_vcn18_wifi_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcn18_voltages[selector];
	pr_debug("ldo vcn18_wifi list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcn28 enable */
static int pmic_ldo_vcn28_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn28 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcn28_sw_en(1);
	return 0;
}

/* Regulator vcn28 disable */
static int pmic_ldo_vcn28_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcn28 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcn28_sw_en(0);
	return ret;
}

/* Regulator vcn28 is_enabled */
static int pmic_ldo_vcn28_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcn28 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcn28_en();
}

/* Regulator vcn28 set_voltage_sel */
static int pmic_ldo_vcn28_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vcn28 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vcn28_vosel(selector);*/
	return 0;
}

/* Regulator vcn28 get_voltage_sel */
static int pmic_ldo_vcn28_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vcn28 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vcn28_vosel();*/
	return 0;
}

/* Regulator vcn28 list_voltage */
static int pmic_ldo_vcn28_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcn28_voltages[selector];
	pr_debug("ldo vcn28 list_voltage: %d\n", voltage);
	return voltage;
}


/* Regulator vibr enable */
static int pmic_ldo_vibr_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vibr enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vibr_sw_en(1);
	return 0;
}

/* Regulator vibr disable */
static int pmic_ldo_vibr_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vibr disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vibr_sw_en(0);
	return ret;
}

/* Regulator vibr is_enabled */
static int pmic_ldo_vibr_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vibr is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vibr_en();
}

/* Regulator vibr set_voltage_sel */
static int pmic_ldo_vibr_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vibr set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vibr_vosel(selector);
	return 0;
}

/* Regulator vibr get_voltage_sel */
static int pmic_ldo_vibr_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vibr get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vibr_vosel();
}

/* Regulator vibr list_voltage */
static int pmic_ldo_vibr_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vibr_voltages[selector];
	pr_debug("ldo vibr list_voltage: %d\n", voltage);
	return voltage;
}


/* Regulator vbif28 enable */
static int pmic_ldo_vbif28_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vbif28 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vbif28_sw_en(1);
	return 0;
}

/* Regulator vbif28 disable */
static int pmic_ldo_vbif28_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vbif28 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vbif28_sw_en(0);
	return ret;
}

/* Regulator vbif28 is_enabled */
static int pmic_ldo_vbif28_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vbif28 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vbif28_en();
}

/* Regulator vbif28 set_voltage_sel */
static int pmic_ldo_vbif28_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vbif28 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vbif28_vosel(selector);*/
	return 0;
}

/* Regulator vbif28 get_voltage_sel */
static int pmic_ldo_vbif28_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vbif28 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vbif28_vosel();*/
	return 0;
}

/* Regulator vbif28 list_voltage */
static int pmic_ldo_vbif28_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vbif28_voltages[selector];
	pr_debug("ldo vbif28 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vfe28 enable */
static int pmic_ldo_vfe28_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vfe28 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vfe28_sw_en(1);
	return 0;
}

/* Regulator vfe28 disable */
static int pmic_ldo_vfe28_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vfe28 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vfe28_sw_en(0);
	return ret;
}

/* Regulator vfe28 is_enabled */
static int pmic_ldo_vfe28_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vfe28 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vfe28_en();
}

/* Regulator vfe28 set_voltage_sel */
static int pmic_ldo_vfe28_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vfe28 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vfe28_vosel(selector);*/
	return 0;
}

/* Regulator vfe28 get_voltage_sel */
static int pmic_ldo_vfe28_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vfe28 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vfe28_vosel();*/
	return 0;
}

/* Regulator vfe28 list_voltage */
static int pmic_ldo_vfe28_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vfe28_voltages[selector];
	pr_debug("ldo vfe28 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vmch enable */
static int pmic_ldo_vmch_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vmch enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vmch_sw_en(1);
	return 0;
}

/* Regulator vmch disable */
static int pmic_ldo_vmch_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vmch disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vmch_sw_en(0);
	return ret;
}

/* Regulator vmch is_enabled */
static int pmic_ldo_vmch_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vmch is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vmch_en();
}

/* Regulator vmch set_voltage_sel */
static int pmic_ldo_vmch_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	pr_debug("ldo vmch set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	if (selector == 0)
		selector = 1;

	mreg->vosel.cur_sel = selector;
	mt6335_upmu_set_rg_vmch_vosel(selector);
	return 0;
}

/* Regulator vmch get_voltage_sel */
static int pmic_ldo_vmch_get_voltage_sel(struct regulator_dev *rdev)
{
	unsigned char regVal = 0;

	pr_debug("ldo vmch get_voltage_sel\n");
	/*return mtk_regulator_get_voltage_sel(rdev);*/
	regVal = mt6335_upmu_get_rg_vmch_vosel();
	return regVal;
}

/* Regulator vmch list_voltage */
static int pmic_ldo_vmch_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	/*---Since spec not 0,1,2 but 1,2,3---*/
	voltage = vmch_voltages[selector];
	pr_debug("ldo vmch list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vmc enable */
static int pmic_ldo_vmc_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vmc enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vmc_sw_en(1);
	return 0;
}

/* Regulator vmc disable */
static int pmic_ldo_vmc_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vmc disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vmc_sw_en(0);
	return ret;
}

/* Regulator vmc is_enabled */
static int pmic_ldo_vmc_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vmc is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vmc_en();
}

/* Regulator vmc set_voltage_sel */
static int pmic_ldo_vmc_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vmc set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vmc_vosel(selector);
	return 0;
}

/* Regulator vmc get_voltage_sel */
static int pmic_ldo_vmc_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vmc get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vmc_vosel();
}

/* Regulator vmc list_voltage */
static int pmic_ldo_vmc_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vmc_voltages[selector];
	pr_debug("ldo vmc list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vrf18_1 enable */
static int pmic_ldo_vrf18_1_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vrf18_1 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vrf18_1_sw_en(1);
	return 0;
}

/* Regulator vrf18_1 disable */
static int pmic_ldo_vrf18_1_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vrf18_1 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vrf18_1_sw_en(0);
	return ret;
}

/* Regulator vrf18_1 is_enabled */
static int pmic_ldo_vrf18_1_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vrf18_1 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vrf18_1_en();
}

/* Regulator vrf18_1 set_voltage_sel */
static int pmic_ldo_vrf18_1_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vrf18_1 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vrf18_1_vosel(selector);*/
	return 0;
}

/* Regulator vrf18_1 get_voltage_sel */
static int pmic_ldo_vrf18_1_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vrf18_1 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vrf18_1_vosel();*/
	return 0;
}

/* Regulator vrf18_1 list_voltage */
static int pmic_ldo_vrf18_1_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vrf18_voltages[selector];
	pr_debug("ldo vrf18_1 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vrf18_2 enable */
static int pmic_ldo_vrf18_2_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vrf18_2 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vrf18_2_sw_en(1);
	return 0;
}

/* Regulator vrf18_2 disable */
static int pmic_ldo_vrf18_2_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vrf18_2 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vrf18_2_sw_en(0);
	return ret;
}

/* Regulator vrf18_2 is_enabled */
static int pmic_ldo_vrf18_2_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vrf18_2 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vrf18_2_en();
}

/* Regulator vrf18_2 set_voltage_sel */
static int pmic_ldo_vrf18_2_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vrf18_2 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vrf18_2_vosel(selector);*/
	return 0;
}

/* Regulator vrf18_2 get_voltage_sel */
static int pmic_ldo_vrf18_2_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vrf18_2 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vrf18_2_vosel();*/
	return 0;
}

/* Regulator vrf18_2 list_voltage */
static int pmic_ldo_vrf18_2_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vrf18_voltages[selector];
	pr_debug("ldo vrf18_2 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vrf12 enable */
static int pmic_ldo_vrf12_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vrf12 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vrf12_sw_en(1);
	return 0;
}

/* Regulator vrf12 disable */
static int pmic_ldo_vrf12_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vrf12 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vrf12_sw_en(0);
	return ret;
}

/* Regulator vrf12 is_enabled */
static int pmic_ldo_vrf12_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vrf12 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vrf12_en();
}

/* Regulator vrf12 set_voltage_sel */
static int pmic_ldo_vrf12_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	pr_debug("sldo vrf12 dont support set_voltage_sel\n");
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	/*mt6335_upmu_set_rg_vrf12_vosel(selector);*/
	return 0;
}

/* Regulator vrf12 get_voltage_sel */
static int pmic_ldo_vrf12_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("sldo vrf12 dont support get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	/*return mt6335_upmu_get_rg_vrf12_vosel();*/
	return 0;
}

/* Regulator vrf12 list_voltage */
static int pmic_ldo_vrf12_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vrf12_voltages[selector];
	pr_debug("ldo vrf12 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcama1 enable */
static int pmic_ldo_vcama1_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcama1 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcama1_sw_en(1);
	return 0;
}

/* Regulator vcama1 disable */
static int pmic_ldo_vcama1_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcama1 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcama1_sw_en(0);
	return ret;
}

/* Regulator vcama1 is_enabled */
static int pmic_ldo_vcama1_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcama1 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcama1_en();
}

/* Regulator vcama1 set_voltage_sel */
static int pmic_ldo_vcama1_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vcama1 set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vcama1_vosel(selector);
	return 0;
}

/* Regulator vcama1 get_voltage_sel */
static int pmic_ldo_vcama1_get_voltage_sel(struct regulator_dev *rdev)
{
	unsigned char regVal = 0;

	regVal = mt6335_upmu_get_rg_vcama1_vosel();
	pr_debug("ldo vcama1 get_voltage_sel %d\n", regVal);
	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return regVal;
}

/* Regulator vcama1 list_voltage */
static int pmic_ldo_vcama1_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcama1_voltages[selector];
	pr_debug("ldo vcama1 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vcama2 enable */
static int pmic_ldo_vcama2_enable(struct regulator_dev *rdev)
{
	pr_debug("ldo vcama2 enable\n");
	/*return mtk_regulator_enable(rdev);*/
	pmic_ldo_vcama2_sw_en(1);
	return 0;
}

/* Regulator vcama2 disable */
static int pmic_ldo_vcama2_disable(struct regulator_dev *rdev)
{
	int ret = 0;

	pr_debug("ldo vcama2 disable\n");
	/*return mtk_regulator_disable(rdev);*/
	if (rdev->use_count == 0) {
		pr_debug("regulator name=%s should not disable( use_count=%d)\n", rdev->desc->name,
			rdev->use_count);
		ret = -1;
	} else
		pmic_ldo_vcama2_sw_en(0);
	return ret;
}

/* Regulator vcama2 is_enabled */
static int pmic_ldo_vcama2_is_enabled(struct regulator_dev *rdev)
{
	pr_debug("ldo vcama2 is_enabled\n");
	/*return mtk_regulator_is_enabled(rdev);*/
	return mt6335_upmu_get_da_qi_vcama2_en();
}

/* Regulator vcama2 set_voltage_sel */
static int pmic_ldo_vcama2_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg;

	mreg = container_of(rdesc, struct mtk_regulator, desc);

	mreg->vosel.cur_sel = selector;
	pr_debug("ldo vcama2 set_voltage_sel: %d\n", selector);
	/*return mtk_regulator_set_voltage_sel(rdev, selector);*/
	mt6335_upmu_set_rg_vcama2_vosel(selector);
	return 0;
}

/* Regulator vcama2 get_voltage_sel */
static int pmic_ldo_vcama2_get_voltage_sel(struct regulator_dev *rdev)
{
	pr_debug("ldo vcama2 get_voltage_sel\n");

	/*return mtk_regulator_get_voltage_sel(rdev);*/
	return mt6335_upmu_get_rg_vcama2_vosel();
}

/* Regulator vcama2 list_voltage */
static int pmic_ldo_vcama2_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int voltage;

	voltage = vcama2_voltages[selector];
	pr_debug("ldo vcama2 list_voltage: %d\n", voltage);
	return voltage;
}

/* Regulator vio28 ops */
static struct regulator_ops pmic_ldo_vio28_ops = {
	.enable = pmic_ldo_vio28_enable,
	.disable = pmic_ldo_vio28_disable,
	.is_enabled = pmic_ldo_vio28_is_enabled,
	.get_voltage_sel = pmic_ldo_vio28_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vio28_set_voltage_sel,
	.list_voltage = pmic_ldo_vio28_list_voltage,
	/* .enable_time = pmic_ldo_vio28_enable_time, */
};

/* Regulator vio18 ops */
static struct regulator_ops pmic_ldo_vio18_ops = {
	.enable = pmic_ldo_vio18_enable,
	.disable = pmic_ldo_vio18_disable,
	.is_enabled = pmic_ldo_vio18_is_enabled,
	.get_voltage_sel = pmic_ldo_vio18_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vio18_set_voltage_sel,
	.list_voltage = pmic_ldo_vio18_list_voltage,
	/* .enable_time = pmic_ldo_vio18_enable_time, */
};

/* Regulator vufs18 ops */
static struct regulator_ops pmic_ldo_vufs18_ops = {
	.enable = pmic_ldo_vufs18_enable,
	.disable = pmic_ldo_vufs18_disable,
	.is_enabled = pmic_ldo_vufs18_is_enabled,
	.get_voltage_sel = pmic_ldo_vufs18_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vufs18_set_voltage_sel,
	.list_voltage = pmic_ldo_vufs18_list_voltage,
	/* .enable_time = pmic_ldo_vufs18_enable_time, */
};

/* Regulator va10 ops */
static struct regulator_ops pmic_ldo_va10_ops = {
	.enable = pmic_ldo_va10_enable,
	.disable = pmic_ldo_va10_disable,
	.is_enabled = pmic_ldo_va10_is_enabled,
	.get_voltage_sel = pmic_ldo_va10_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_va10_set_voltage_sel,
	.list_voltage = pmic_ldo_va10_list_voltage,
	/* .enable_time = pmic_ldo_va10_enable_time, */
};

/* Regulator va12 ops */
static struct regulator_ops pmic_ldo_va12_ops = {
	.enable = pmic_ldo_va12_enable,
	.disable = pmic_ldo_va12_disable,
	.is_enabled = pmic_ldo_va12_is_enabled,
	.get_voltage_sel = pmic_ldo_va12_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_va12_set_voltage_sel,
	.list_voltage = pmic_ldo_va12_list_voltage,
	/* .enable_time = pmic_ldo_va12_enable_time, */
};

/* Regulator va18 ops */
static struct regulator_ops pmic_ldo_va18_ops = {
	.enable = pmic_ldo_va18_enable,
	.disable = pmic_ldo_va18_disable,
	.is_enabled = pmic_ldo_va18_is_enabled,
	.get_voltage_sel = pmic_ldo_va18_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_va18_set_voltage_sel,
	.list_voltage = pmic_ldo_va18_list_voltage,
	/* .enable_time = pmic_ldo_va18_enable_time, */
};

/* Regulator vusb33 ops */
static struct regulator_ops pmic_ldo_vusb33_ops = {
	.enable = pmic_ldo_vusb33_enable,
	.disable = pmic_ldo_vusb33_disable,
	.is_enabled = pmic_ldo_vusb33_is_enabled,
	.get_voltage_sel = pmic_ldo_vusb33_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vusb33_set_voltage_sel,
	.list_voltage = pmic_ldo_vusb33_list_voltage,
	/* .enable_time = pmic_ldo_vusb33_enable_time, */
};

/* Regulator vemc ops */
static struct regulator_ops pmic_ldo_vemc_ops = {
	.enable = pmic_ldo_vemc_enable,
	.disable = pmic_ldo_vemc_disable,
	.is_enabled = pmic_ldo_vemc_is_enabled,
	.get_voltage_sel = pmic_ldo_vemc_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vemc_set_voltage_sel,
	.list_voltage = pmic_ldo_vemc_list_voltage,
	/* .enable_time = pmic_ldo_vemc_enable_time, */
};

/* Regulator vxo22 ops */
static struct regulator_ops pmic_ldo_vxo22_ops = {
	.enable = pmic_ldo_vxo22_enable,
	.disable = pmic_ldo_vxo22_disable,
	.is_enabled = pmic_ldo_vxo22_is_enabled,
	.get_voltage_sel = pmic_ldo_vxo22_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vxo22_set_voltage_sel,
	.list_voltage = pmic_ldo_vxo22_list_voltage,
	/* .enable_time = pmic_ldo_vxo22_enable_time, */
};

/* Regulator vefuse ops */
static struct regulator_ops pmic_ldo_vefuse_ops = {
	.enable = pmic_ldo_vefuse_enable,
	.disable = pmic_ldo_vefuse_disable,
	.is_enabled = pmic_ldo_vefuse_is_enabled,
	.get_voltage_sel = pmic_ldo_vefuse_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vefuse_set_voltage_sel,
	.list_voltage = pmic_ldo_vefuse_list_voltage,
	/* .enable_time = pmic_ldo_vefuse_enable_time, */
};

/* Regulator vsim1 ops */
static struct regulator_ops pmic_ldo_vsim1_ops = {
	.enable = pmic_ldo_vsim1_enable,
	.disable = pmic_ldo_vsim1_disable,
	.is_enabled = pmic_ldo_vsim1_is_enabled,
	.get_voltage_sel = pmic_ldo_vsim1_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vsim1_set_voltage_sel,
	.list_voltage = pmic_ldo_vsim1_list_voltage,
	/* .enable_time = pmic_ldo_vsim1_enable_time, */
};

/* Regulator vsim2 ops */
static struct regulator_ops pmic_ldo_vsim2_ops = {
	.enable = pmic_ldo_vsim2_enable,
	.disable = pmic_ldo_vsim2_disable,
	.is_enabled = pmic_ldo_vsim2_is_enabled,
	.get_voltage_sel = pmic_ldo_vsim2_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vsim2_set_voltage_sel,
	.list_voltage = pmic_ldo_vsim2_list_voltage,
	/* .enable_time = pmic_ldo_vsim2_enable_time, */
};

/* Regulator vcamaf ops */
static struct regulator_ops pmic_ldo_vcamaf_ops = {
	.enable = pmic_ldo_vcamaf_enable,
	.disable = pmic_ldo_vcamaf_disable,
	.is_enabled = pmic_ldo_vcamaf_is_enabled,
	.get_voltage_sel = pmic_ldo_vcamaf_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcamaf_set_voltage_sel,
	.list_voltage = pmic_ldo_vcamaf_list_voltage,
	/* .enable_time = pmic_ldo_vcamaf_enable_time, */
};

/* Regulator vtouch ops */
static struct regulator_ops pmic_ldo_vtouch_ops = {
	.enable = pmic_ldo_vtouch_enable,
	.disable = pmic_ldo_vtouch_disable,
	.is_enabled = pmic_ldo_vtouch_is_enabled,
	.get_voltage_sel = pmic_ldo_vtouch_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vtouch_set_voltage_sel,
	.list_voltage = pmic_ldo_vtouch_list_voltage,
	/* .enable_time = pmic_ldo_vtouch_enable_time, */
};

/* Regulator vcamd1 ops */
static struct regulator_ops pmic_ldo_vcamd1_ops = {
	.enable = pmic_ldo_vcamd1_enable,
	.disable = pmic_ldo_vcamd1_disable,
	.is_enabled = pmic_ldo_vcamd1_is_enabled,
	.get_voltage_sel = pmic_ldo_vcamd1_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcamd1_set_voltage_sel,
	.list_voltage = pmic_ldo_vcamd1_list_voltage,
	/* .enable_time = pmic_ldo_vcamd1_enable_time, */
};

/* Regulator vcamd2 ops */
static struct regulator_ops pmic_ldo_vcamd2_ops = {
	.enable = pmic_ldo_vcamd2_enable,
	.disable = pmic_ldo_vcamd2_disable,
	.is_enabled = pmic_ldo_vcamd2_is_enabled,
	.get_voltage_sel = pmic_ldo_vcamd2_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcamd2_set_voltage_sel,
	.list_voltage = pmic_ldo_vcamd2_list_voltage,
	/* .enable_time = pmic_ldo_vcamd2_enable_time, */
};

/* Regulator vcamio ops */
static struct regulator_ops pmic_ldo_vcamio_ops = {
	.enable = pmic_ldo_vcamio_enable,
	.disable = pmic_ldo_vcamio_disable,
	.is_enabled = pmic_ldo_vcamio_is_enabled,
	.get_voltage_sel = pmic_ldo_vcamio_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcamio_set_voltage_sel,
	.list_voltage = pmic_ldo_vcamio_list_voltage,
	/* .enable_time = pmic_ldo_vcamio_enable_time, */
};

/* Regulator vmipi ops */
static struct regulator_ops pmic_ldo_vmipi_ops = {
	.enable = pmic_ldo_vmipi_enable,
	.disable = pmic_ldo_vmipi_disable,
	.is_enabled = pmic_ldo_vmipi_is_enabled,
	.get_voltage_sel = pmic_ldo_vmipi_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vmipi_set_voltage_sel,
	.list_voltage = pmic_ldo_vmipi_list_voltage,
	/* .enable_time = pmic_ldo_vmipi_enable_time, */
};

/* Regulator vgp3 ops */
static struct regulator_ops pmic_ldo_vgp3_ops = {
	.enable = pmic_ldo_vgp3_enable,
	.disable = pmic_ldo_vgp3_disable,
	.is_enabled = pmic_ldo_vgp3_is_enabled,
	.get_voltage_sel = pmic_ldo_vgp3_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vgp3_set_voltage_sel,
	.list_voltage = pmic_ldo_vgp3_list_voltage,
	/* .enable_time = pmic_ldo_vgp3_enable_time, */
};

/* Regulator vcn33_bt ops */
static struct regulator_ops pmic_ldo_vcn33_bt_ops = {
	.enable = pmic_ldo_vcn33_bt_enable,
	.disable = pmic_ldo_vcn33_bt_disable,
	.is_enabled = pmic_ldo_vcn33_bt_is_enabled,
	.get_voltage_sel = pmic_ldo_vcn33_bt_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcn33_bt_set_voltage_sel,
	.list_voltage = pmic_ldo_vcn33_bt_list_voltage,
	/* .enable_time = pmic_ldo_vcn33_bt_enable_time, */
};

/* Regulator vcn33_wifi ops */
static struct regulator_ops pmic_ldo_vcn33_wifi_ops = {
	.enable = pmic_ldo_vcn33_wifi_enable,
	.disable = pmic_ldo_vcn33_wifi_disable,
	.is_enabled = pmic_ldo_vcn33_wifi_is_enabled,
	.get_voltage_sel = pmic_ldo_vcn33_wifi_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcn33_wifi_set_voltage_sel,
	.list_voltage = pmic_ldo_vcn33_wifi_list_voltage,
	/* .enable_time = pmic_ldo_vcn33_wifi_enable_time, */
};

/* Regulator vcn18_bt ops */
static struct regulator_ops pmic_ldo_vcn18_bt_ops = {
	.enable = pmic_ldo_vcn18_bt_enable,
	.disable = pmic_ldo_vcn18_bt_disable,
	.is_enabled = pmic_ldo_vcn18_bt_is_enabled,
	.get_voltage_sel = pmic_ldo_vcn18_bt_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcn18_bt_set_voltage_sel,
	.list_voltage = pmic_ldo_vcn18_bt_list_voltage,
	/* .enable_time = pmic_ldo_vcn18_bt_enable_time, */
};

/* Regulator vcn18_wifi ops */
static struct regulator_ops pmic_ldo_vcn18_wifi_ops = {
	.enable = pmic_ldo_vcn18_wifi_enable,
	.disable = pmic_ldo_vcn18_wifi_disable,
	.is_enabled = pmic_ldo_vcn18_wifi_is_enabled,
	.get_voltage_sel = pmic_ldo_vcn18_wifi_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcn18_wifi_set_voltage_sel,
	.list_voltage = pmic_ldo_vcn18_wifi_list_voltage,
	/* .enable_time = pmic_ldo_vcn18_wifi_enable_time, */
};

/* Regulator vcn28 ops */
static struct regulator_ops pmic_ldo_vcn28_ops = {
	.enable = pmic_ldo_vcn28_enable,
	.disable = pmic_ldo_vcn28_disable,
	.is_enabled = pmic_ldo_vcn28_is_enabled,
	.get_voltage_sel = pmic_ldo_vcn28_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcn28_set_voltage_sel,
	.list_voltage = pmic_ldo_vcn28_list_voltage,
	/* .enable_time = pmic_ldo_vcn28_enable_time, */
};

/* Regulator vibr ops */
static struct regulator_ops pmic_ldo_vibr_ops = {
	.enable = pmic_ldo_vibr_enable,
	.disable = pmic_ldo_vibr_disable,
	.is_enabled = pmic_ldo_vibr_is_enabled,
	.get_voltage_sel = pmic_ldo_vibr_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vibr_set_voltage_sel,
	.list_voltage = pmic_ldo_vibr_list_voltage,
	/* .enable_time = pmic_ldo_vibr_enable_time, */
};

/* Regulator vbif28 ops */
static struct regulator_ops pmic_ldo_vbif28_ops = {
	.enable = pmic_ldo_vbif28_enable,
	.disable = pmic_ldo_vbif28_disable,
	.is_enabled = pmic_ldo_vbif28_is_enabled,
	.get_voltage_sel = pmic_ldo_vbif28_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vbif28_set_voltage_sel,
	.list_voltage = pmic_ldo_vbif28_list_voltage,
	/* .enable_time = pmic_ldo_vbif28_enable_time, */
};

/* Regulator vfe28 ops */
static struct regulator_ops pmic_ldo_vfe28_ops = {
	.enable = pmic_ldo_vfe28_enable,
	.disable = pmic_ldo_vfe28_disable,
	.is_enabled = pmic_ldo_vfe28_is_enabled,
	.get_voltage_sel = pmic_ldo_vfe28_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vfe28_set_voltage_sel,
	.list_voltage = pmic_ldo_vfe28_list_voltage,
	/* .enable_time = pmic_ldo_vfe28_enable_time, */
};

/* Regulator vmch ops */
static struct regulator_ops pmic_ldo_vmch_ops = {
	.enable = pmic_ldo_vmch_enable,
	.disable = pmic_ldo_vmch_disable,
	.is_enabled = pmic_ldo_vmch_is_enabled,
	.get_voltage_sel = pmic_ldo_vmch_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vmch_set_voltage_sel,
	.list_voltage = pmic_ldo_vmch_list_voltage,
	/* .enable_time = pmic_ldo_vmch_enable_time, */
};

/* Regulator vmc ops */
static struct regulator_ops pmic_ldo_vmc_ops = {
	.enable = pmic_ldo_vmc_enable,
	.disable = pmic_ldo_vmc_disable,
	.is_enabled = pmic_ldo_vmc_is_enabled,
	.get_voltage_sel = pmic_ldo_vmc_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vmc_set_voltage_sel,
	.list_voltage = pmic_ldo_vmc_list_voltage,
	/* .enable_time = pmic_ldo_vmc_enable_time, */
};

/* Regulator vrf18_1 ops */
static struct regulator_ops pmic_ldo_vrf18_1_ops = {
	.enable = pmic_ldo_vrf18_1_enable,
	.disable = pmic_ldo_vrf18_1_disable,
	.is_enabled = pmic_ldo_vrf18_1_is_enabled,
	.get_voltage_sel = pmic_ldo_vrf18_1_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vrf18_1_set_voltage_sel,
	.list_voltage = pmic_ldo_vrf18_1_list_voltage,
	/* .enable_time = pmic_ldo_vrf18_1_enable_time, */
};

/* Regulator vrf18_2 ops */
static struct regulator_ops pmic_ldo_vrf18_2_ops = {
	.enable = pmic_ldo_vrf18_2_enable,
	.disable = pmic_ldo_vrf18_2_disable,
	.is_enabled = pmic_ldo_vrf18_2_is_enabled,
	.get_voltage_sel = pmic_ldo_vrf18_2_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vrf18_2_set_voltage_sel,
	.list_voltage = pmic_ldo_vrf18_2_list_voltage,
	/* .enable_time = pmic_ldo_vrf18_2_enable_time, */
};

/* Regulator vrf12 ops */
static struct regulator_ops pmic_ldo_vrf12_ops = {
	.enable = pmic_ldo_vrf12_enable,
	.disable = pmic_ldo_vrf12_disable,
	.is_enabled = pmic_ldo_vrf12_is_enabled,
	.get_voltage_sel = pmic_ldo_vrf12_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vrf12_set_voltage_sel,
	.list_voltage = pmic_ldo_vrf12_list_voltage,
	/* .enable_time = pmic_ldo_vrf12_enable_time, */
};

/* Regulator vcama1 ops */
static struct regulator_ops pmic_ldo_vcama1_ops = {
	.enable = pmic_ldo_vcama1_enable,
	.disable = pmic_ldo_vcama1_disable,
	.is_enabled = pmic_ldo_vcama1_is_enabled,
	.get_voltage_sel = pmic_ldo_vcama1_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcama1_set_voltage_sel,
	.list_voltage = pmic_ldo_vcama1_list_voltage,
	/* .enable_time = pmic_ldo_vcama1_enable_time, */
};

/* Regulator vcama2 ops */
static struct regulator_ops pmic_ldo_vcama2_ops = {
	.enable = pmic_ldo_vcama2_enable,
	.disable = pmic_ldo_vcama2_disable,
	.is_enabled = pmic_ldo_vcama2_is_enabled,
	.get_voltage_sel = pmic_ldo_vcama2_get_voltage_sel,
	.set_voltage_sel = pmic_ldo_vcama2_set_voltage_sel,
	.list_voltage = pmic_ldo_vcama2_list_voltage,
	/* .enable_time = pmic_ldo_vcama2_enable_time, */
};

/*------LDO ATTR------*/
static ssize_t show_LDO_STATUS(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_regulator *mreg;
	unsigned int ret_value = 0;

	mreg = container_of(attr, struct mtk_regulator, en_att);

	ret_value = pmic_get_register_value(mreg->en_reg);

	pr_debug("[EM] LDO_%s_STATUS : %d\n", mreg->desc.name, ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_LDO_STATUS(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t size)
{
	PMICLOG("[EM] Not Support Write Function\n");
	return size;
}

static ssize_t show_LDO_VOLTAGE(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_regulator *mreg;
	const int *pVoltage;

	unsigned short regVal;
	unsigned int ret_value = 0;

	mreg = container_of(attr, struct mtk_regulator, voltage_att);

	if (mreg->desc.n_voltages != 1) {
		if (mreg->vol_reg != 0) {
			regVal = pmic_get_register_value(mreg->vol_reg);
			if (mreg->pvoltages != NULL) {
				pVoltage = (const int *)mreg->pvoltages;
				/*HW LDO sequence issue, we need to change it */
				ret_value = pVoltage[regVal];
			} else {
				ret_value = mreg->desc.min_uV + mreg->desc.uV_step * regVal;
			}
		} else {
			PMICLOG("[EM][ERROR] LDO_%s_VOLTAGE : voltage=0 vol_reg=0\n",
				mreg->desc.name);
		}
	} else {
		pVoltage = (const int *)mreg->pvoltages;
		ret_value = pVoltage[0];
	}

	ret_value = ret_value / 1000;
	pr_debug("[EM] LDO_%s_VOLTAGE : %d\n", mreg->desc.name, ret_value);
	return sprintf(buf, "%u\n", ret_value);

}

static ssize_t store_LDO_VOLTAGE(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t size)
{
	PMICLOG("[EM] Not Support Write Function\n");
	return size;
}


/* Regulator: LDO */
#define LDO_EN	REGULATOR_CHANGE_STATUS
#define LDO_VOL REGULATOR_CHANGE_VOLTAGE
#define LDO_VOL_EN (REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE)
struct mtk_regulator mtk_ldos[] = {
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vio28, ldo, vio28_voltages,
		LDO_EN, 1, PMIC_RG_VIO28_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vio18, ldo, vio18_voltages,
		LDO_EN, 1, PMIC_RG_VIO18_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vufs18, ldo, vufs18_voltages,
		LDO_EN, 1, PMIC_RG_VUFS18_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(va10, ldo, va10_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VA10_SW_EN, PMIC_RG_VA10_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(va12, ldo, va12_voltages,
		LDO_EN, 1, PMIC_RG_VA12_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(va18, ldo, va18_voltages,
		LDO_EN, 1, PMIC_RG_VA18_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vusb33, ldo, vusb33_voltages,
		LDO_EN, 1, PMIC_RG_VUSB33_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vemc, ldo, vemc_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VEMC_SW_EN, PMIC_RG_VEMC_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vxo22, ldo, vxo22_voltages,
		LDO_EN, 1, PMIC_RG_VXO22_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vefuse, ldo, vefuse_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VEFUSE_SW_EN, PMIC_RG_VEFUSE_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vsim1, ldo, vsim1_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VSIM1_SW_EN, PMIC_RG_VSIM1_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vsim2, ldo, vsim2_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VSIM2_SW_EN, PMIC_RG_VSIM2_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcamaf, ldo, vcamaf_voltages,
		LDO_EN, 1, PMIC_RG_VCAMAF_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vtouch, ldo, vtouch_voltages,
		LDO_EN, 1, PMIC_RG_VTOUCH_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcamd1, ldo, vcamd1_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VCAMD1_SW_EN, PMIC_RG_VCAMD1_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcamd2, ldo, vcamd2_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VCAMD2_SW_EN, PMIC_RG_VCAMD2_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcamio, ldo, vcamio_voltages,
		LDO_EN, 1, PMIC_RG_VCAMIO_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vmipi, ldo, vmipi_voltages,
		LDO_EN, 1, PMIC_RG_VMIPI_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vgp3, ldo, vgp3_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VGP3_SW_EN, PMIC_RG_VGP3_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcn33_bt, ldo, vcn33_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VCN33_SW_EN_BT, PMIC_RG_VCN33_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcn33_wifi, ldo, vcn33_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VCN33_SW_EN_WIFI, PMIC_RG_VCN33_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcn18_bt, ldo, vcn18_voltages,
		LDO_EN, 1, PMIC_RG_VCN18_SW_EN_BT, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcn18_wifi, ldo, vcn18_voltages,
		LDO_EN, 1, PMIC_RG_VCN18_SW_EN_WIFI, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcn28, ldo, vcn28_voltages,
		LDO_EN, 1, PMIC_RG_VCN28_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vibr, ldo, vibr_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VIBR_SW_EN, PMIC_RG_VIBR_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vbif28, ldo, vbif28_voltages,
		LDO_EN, 1, PMIC_RG_VBIF28_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vfe28, ldo, vfe28_voltages,
		LDO_EN, 1, PMIC_RG_VFE28_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vmch, ldo, vmch_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VMCH_SW_EN, PMIC_RG_VMCH_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vmc, ldo, vmc_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VMC_SW_EN, PMIC_RG_VMC_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vrf18_1, ldo, vrf18_voltages,
		LDO_EN, 1, PMIC_RG_VRF18_1_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vrf18_2, ldo, vrf18_voltages,
		LDO_EN, 1, PMIC_RG_VRF18_2_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vrf12, ldo, vrf12_voltages,
		LDO_EN, 1, PMIC_RG_VRF12_SW_EN, NULL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcama1, ldo, vcama1_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VCAMA1_SW_EN, PMIC_RG_VCAMA1_VOSEL),
	NON_REGULAR_VOLTAGE_REGULATOR_GEN(vcama2, ldo, vcama2_voltages,
		LDO_VOL_EN, 1, PMIC_RG_VCAMA2_SW_EN, PMIC_RG_VCAMA2_VOSEL),
};

int mtk_ldos_size = ARRAY_SIZE(mtk_ldos);


/* -------Code Gen End-------*/

#ifdef CONFIG_OF
#if !defined CONFIG_MTK_LEGACY

#define PMIC_REGULATOR_OF_MATCH(_name, _id)			\
	{						\
		.name = #_name,						\
		.driver_data = &mtk_ldos[MT6335_POWER_LDO_##_id],	\
	}

struct of_regulator_match pmic_regulator_matches[] = {
	PMIC_REGULATOR_OF_MATCH(ldo_vio28, VIO28),
	PMIC_REGULATOR_OF_MATCH(ldo_vio18, VIO18),
	PMIC_REGULATOR_OF_MATCH(ldo_vufs18, VUFS18),
	PMIC_REGULATOR_OF_MATCH(ldo_va10, VA10),
	PMIC_REGULATOR_OF_MATCH(ldo_va12, VA12),
	PMIC_REGULATOR_OF_MATCH(ldo_va18, VA18),
	PMIC_REGULATOR_OF_MATCH(ldo_vusb33, VUSB33),
	PMIC_REGULATOR_OF_MATCH(ldo_vemc, VEMC),
	PMIC_REGULATOR_OF_MATCH(ldo_vxo22, VXO22),
	PMIC_REGULATOR_OF_MATCH(ldo_vefuse, VEFUSE),
	PMIC_REGULATOR_OF_MATCH(ldo_vsim1 , VSIM1),
	PMIC_REGULATOR_OF_MATCH(ldo_vsim2 , VSIM2),
	PMIC_REGULATOR_OF_MATCH(ldo_vcamaf, VCAMAF),
	PMIC_REGULATOR_OF_MATCH(ldo_vtouch, VTOUCH),
	PMIC_REGULATOR_OF_MATCH(ldo_vcamd1, VCAMD1),
	PMIC_REGULATOR_OF_MATCH(ldo_vcamd2, VCAMD2),
	PMIC_REGULATOR_OF_MATCH(ldo_vcamio, VCAMIO),
	PMIC_REGULATOR_OF_MATCH(ldo_vmipi, VMIPI),
	PMIC_REGULATOR_OF_MATCH(ldo_vgp3, VGP3),
	PMIC_REGULATOR_OF_MATCH(ldo_vcn33_bt, VCN33_BT),
	PMIC_REGULATOR_OF_MATCH(ldo_vcn33_wifi, VCN33_WIFI),
	PMIC_REGULATOR_OF_MATCH(ldo_vcn18_bt, VCN18_BT),
	PMIC_REGULATOR_OF_MATCH(ldo_vcn18_wifi, VCN18_WIFI),
	PMIC_REGULATOR_OF_MATCH(ldo_vcn28, VCN28),
	PMIC_REGULATOR_OF_MATCH(ldo_vibr, VIBR),
	PMIC_REGULATOR_OF_MATCH(ldo_vbif28, VBIF28),
	PMIC_REGULATOR_OF_MATCH(ldo_vfe28, VFE28),
	PMIC_REGULATOR_OF_MATCH(ldo_vmch, VMCH),
	PMIC_REGULATOR_OF_MATCH(ldo_vmc, VMC),
	PMIC_REGULATOR_OF_MATCH(ldo_vrf18_1, VRF18_1),
	PMIC_REGULATOR_OF_MATCH(ldo_vrf18_2, VRF18_2),
	PMIC_REGULATOR_OF_MATCH(ldo_vrf12, VRF12),
	PMIC_REGULATOR_OF_MATCH(ldo_vcama1, VCAMA1),
	PMIC_REGULATOR_OF_MATCH(ldo_vcama2, VCAMA2),
};

int pmic_regulator_matches_size = ARRAY_SIZE(pmic_regulator_matches);

#endif				/* End of #ifdef CONFIG_OF */
#endif				/* End of #if !defined CONFIG_MTK_LEGACY */


