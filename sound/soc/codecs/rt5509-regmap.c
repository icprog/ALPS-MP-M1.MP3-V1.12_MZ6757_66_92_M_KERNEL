/*
 *  sound/soc/samsung/rt5509-regmap.c
 *  Driver to Richtek RT5509 SPKAMP IC
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/device.h>

#include "rt5509.h"

#ifdef CONFIG_RT_REGMAP
/* ---------------------------------------------------------------------
 * Create RT5509 register map
 *
 * RT_REG_DECL(_addr, _reg_length, _reg_type, _mask_...)
 * @ _addr : reigster address
 * @ _reg_length : register byte length
 * @ _reg_type : reigster type (RT_NORMAL, RT_VOLATILE, RT_WBITS)
 * @ _mask : register write bits mask
 */

/* WBITS will and mask, to check the writable bits */
/* NORMAL ignore mask */
/* VOLATILE directly write through */
RT_REG_DECL(RT5509_REG_CHIPREV, 1, RT_WBITS_WR_ONCE, {0x00});
RT_REG_DECL(RT5509_REG_EVENTINFO, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_DMGFLAG, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_CHIPEN, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_AUDFMT, 1, RT_WBITS_WR_ONCE, {0x1f});
RT_REG_DECL(RT5509_REG_AUDSR, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_I2SSEL, 1, RT_WBITS_WR_ONCE, {0x0f});
RT_REG_DECL(RT5509_REG_I2SDOLRSEL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_I2SDOSEL, 1, RT_WBITS_WR_ONCE, {0x03});
RT_REG_DECL(RT5509_REG_FUNCEN, 1, RT_WBITS_WR_ONCE, {0x7f});
RT_REG_DECL(RT5509_REG_CLIP_THR, 1, RT_WBITS_WR_ONCE, {0x07});
RT_REG_DECL(RT5509_REG_CLIP_CTRL, 1, RT_WBITS_WR_ONCE, {0xbf});
RT_REG_DECL(RT5509_REG_CLIP_SLOPE, 2, RT_WBITS_WR_ONCE, {0x03, 0xff});
RT_REG_DECL(RT5509_REG_CLIP_VOMIN, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CLIP_SIGMAX, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_AMPCONF, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DACRNKGAIN, 1, RT_WBITS_WR_ONCE, {0xcf});
RT_REG_DECL(RT5509_REG_SAMPOFFS, 1, RT_WBITS_WR_ONCE, {0x07});
RT_REG_DECL(RT5509_REG_SAMPCONF, 1, RT_WBITS_WR_ONCE, {0x7f});
RT_REG_DECL(RT5509_REG_DAGAIN, 2, RT_WBITS_WR_ONCE, {0x07, 0xff});
RT_REG_DECL(RT5509_REG_FFGAIN, 2, RT_WBITS_WR_ONCE, {0x03, 0xff});
RT_REG_DECL(RT5509_REG_VBATGAIN, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_RLDCOEF1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_RLDCOEF2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BST_MODE, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BST_TH1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BST_TH2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BST_TH3, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BST_CONF1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BST_SIG_GAIN, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BST_CONF2, 1, RT_WBITS_WR_ONCE, {0xf3});
RT_REG_DECL(RT5509_REG_BST_CONF3, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_OCPOTPEN, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_IDAC1TST, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_IDAC2TST, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_IDAC3TST, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_IDACTSTEN, 1, RT_WBITS_WR_ONCE, {0x83});
RT_REG_DECL(RT5509_REG_CCMAX, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_OCPMAX, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_INTERRUPT, 2, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_INTRMASK, 2, RT_WBITS_WR_ONCE, {0x07, 0xff});
RT_REG_DECL(RT5509_REG_DEGLITCH, 2, RT_WBITS_WR_ONCE, {0x0F, 0xff});
RT_REG_DECL(RT5509_REG_SICRTNSTHACT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_TIMEDET, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_TDELAY, 2, RT_WBITS_WR_ONCE, {0x01, 0xff});
RT_REG_DECL(RT5509_REG_TATKSEL, 1, RT_WBITS_WR_ONCE, {0x03});
RT_REG_DECL(RT5509_REG_TREL, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_THOLDREL, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_STHLMT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_XTHLMT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_STHALC, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_XTHALC, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_INITUDT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_UDT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DNHALFT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_ALCGAIN, 1, RT_WBITS_WR_ONCE, {0xf7});
RT_REG_DECL(RT5509_REG_ADAPTCONF, 1, RT_WBITS_WR_ONCE, {0x7f});
RT_REG_DECL(RT5509_REG_INITIMPLDMU, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_IMPLDMU, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_GPILOT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PILOTEN, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PILOTNISENSE, 1, RT_WBITS_WR_ONCE, {0x7f});
RT_REG_DECL(RT5509_REG_ISENSEGAIN, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_RAPP, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DCR_MAX, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DCR_KD, 2, RT_WBITS_WR_ONCE, {0x07, 0xff});
RT_REG_DECL(RT5509_REG_DCR_KP, 2, RT_WBITS_WR_ONCE, {0x03, 0xff});
RT_REG_DECL(RT5509_REG_DCR_KI, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_INITDCRIDMU, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DCRIDMU, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CALIB_DCR, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CALIB_BL, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CALIB_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_CALIB_REQ, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CALIB_GAIN, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CALIB_OUT0, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CALIB_OUT1, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_XTHLMTDAM, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_RMAXDAM, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_TSCALEDAM, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_RECOVERT, 1, RT_WBITS_WR_ONCE, {0x0f});
RT_REG_DECL(RT5509_REG_SETRESFREQ, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_GETRESFREQ, 2, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_VOLCTL, 1, RT_WBITS_WR_ONCE, {0x07});
RT_REG_DECL(RT5509_REG_VOLUME, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CALIB_OUTX, 4, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_CALIB_OUTY, 4, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_BQ1, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ2, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ3, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ4, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ5, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ6, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ7, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ8, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ9, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BQ10, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ1, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ2, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ3, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ4, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ5, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ6, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ7, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ8, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBBQ9, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBFCN, 24, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN1, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN2, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN3, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN4, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN5, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN6, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN7, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN8, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN9, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBGAIN10, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_SLOPCONST, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BWCOEFF, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_SWRESET, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_SPKGAIN, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKCONF1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKCONF2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKCONF3, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKCONF4, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKVMID, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKZCBOOST, 1, RT_WBITS_WR_ONCE, {0x1f});
RT_REG_DECL(RT5509_REG_ISENSE_CTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DIMADC, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKEN1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBATDATA, 2, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_VTHRMDATA, 2, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_VBATSENSE, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_IDACTSTNINFO, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_IDACBOOST, 2, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_DSPKEN2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKIBCONF1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKIBCONF2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKIBCONF3, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKCONF5, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DSPKCONF6, 1, RT_WBITS_WR_ONCE, {0x7f});
RT_REG_DECL(RT5509_REG_OVPUVPCTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PLLCONF1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PLLCONF2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PLLCONF3, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PLLCONF4, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_PLLINFO, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_PLLDIVISOR, 4, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_ZCCONF, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DCADJ, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_I2CBCKLRCKCONF, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_TDEN, 1, RT_WBITS_WR_ONCE, {0x38});
RT_REG_DECL(RT5509_REG_ALPHACONF, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_SPKRPTSEL, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_SPKRPT, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_NDELAY, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DELAYRES, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PHI1, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PHI2, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PHI3, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PHI4, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PHI5, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_ADAPTB0, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_ADAPTB1, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_ADAPTB2, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_ADAPTB3, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_ADAPTB4, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_ADAPTB5, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_COEFSIERA, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_COEFHPF, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MIMATC_CTRL, 1, RT_WBITS_WR_ONCE, {0x1f});
RT_REG_DECL(RT5509_REG_TDM_CTRL, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_ECO_CTRL, 1, RT_WBITS_WR_ONCE, {0x01});
RT_REG_DECL(RT5509_REG_BSTTM, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_OTPCONF, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_OTPDIN, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VBG_TRIM, 1, RT_WBITS_WR_ONCE, {0x1f});
RT_REG_DECL(RT5509_REG_VTEMP_TRIM, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_TCOEFF, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_SPSCONF, 1, RT_WBITS_WR_ONCE, {0x87});
RT_REG_DECL(RT5509_REG_SPSTHR, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_VTHERMBATEN, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DBGADS, 2, RT_WBITS_WR_ONCE, {0x1f, 0x1f});
RT_REG_DECL(RT5509_REG_TESTDAC, 3, RT_WBITS_WR_ONCE, {0xcf, 0xff, 0xff});
RT_REG_DECL(RT5509_REG_SPKDCS, 2, RT_WBITS_WR_ONCE, {0x80, 0xff});
RT_REG_DECL(RT5509_REG_MSKFLAG, 1, RT_WBITS_WR_ONCE, {0x3f});
RT_REG_DECL(RT5509_REG_DRCMINGAIN, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRC_SEL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRC_ATTACK, 16, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_DRC_PARAM, 7, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_DRCBQ1, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ2, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ3, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ4, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ5, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ6, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ7, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ8, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ9, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ10, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ11, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCBQ12, 20, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_DRCEN, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW3, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW4, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW5, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW6, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW7, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW8, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_MTPFLOW9, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_MTPFLOWA, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOWB, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOWC, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOWD, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOWE, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_MTPFLOWF, 3, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_TESTMODE1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_RAMIND1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_RAMIND2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT5509_REG_SCANMODE, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CLKEN1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_CLKEN2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_PADDRV, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_TESTMODE2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_SLEWRATE1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_SLEWRATE2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BIASRESISTOR, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_SPKDRV, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BLOCKREF1, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BLOCKREF2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BIASCURRENT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(RT5509_REG_BIASOPTION, 1, RT_NORMAL_WR_ONCE, {});

static const rt_register_map_t rt5509_regmap[] = {
	RT_REG(RT5509_REG_CHIPREV),
	RT_REG(RT5509_REG_EVENTINFO),
	RT_REG(RT5509_REG_DMGFLAG),
	RT_REG(RT5509_REG_CHIPEN),
	RT_REG(RT5509_REG_AUDFMT),
	RT_REG(RT5509_REG_AUDSR),
	RT_REG(RT5509_REG_I2SSEL),
	RT_REG(RT5509_REG_I2SDOLRSEL),
	RT_REG(RT5509_REG_I2SDOSEL),
	RT_REG(RT5509_REG_FUNCEN),
	RT_REG(RT5509_REG_CLIP_THR),
	RT_REG(RT5509_REG_CLIP_CTRL),
	RT_REG(RT5509_REG_CLIP_SLOPE),
	RT_REG(RT5509_REG_CLIP_VOMIN),
	RT_REG(RT5509_REG_CLIP_SIGMAX),
	RT_REG(RT5509_REG_AMPCONF),
	RT_REG(RT5509_REG_DACRNKGAIN),
	RT_REG(RT5509_REG_SAMPOFFS),
	RT_REG(RT5509_REG_SAMPCONF),
	RT_REG(RT5509_REG_DAGAIN),
	RT_REG(RT5509_REG_FFGAIN),
	RT_REG(RT5509_REG_VBATGAIN),
	RT_REG(RT5509_REG_RLDCOEF1),
	RT_REG(RT5509_REG_RLDCOEF2),
	RT_REG(RT5509_REG_BST_MODE),
	RT_REG(RT5509_REG_BST_TH1),
	RT_REG(RT5509_REG_BST_TH2),
	RT_REG(RT5509_REG_BST_TH3),
	RT_REG(RT5509_REG_BST_CONF1),
	RT_REG(RT5509_REG_BST_SIG_GAIN),
	RT_REG(RT5509_REG_BST_CONF2),
	RT_REG(RT5509_REG_BST_CONF3),
	RT_REG(RT5509_REG_OCPOTPEN),
	RT_REG(RT5509_REG_IDAC1TST),
	RT_REG(RT5509_REG_IDAC2TST),
	RT_REG(RT5509_REG_IDAC3TST),
	RT_REG(RT5509_REG_IDACTSTEN),
	RT_REG(RT5509_REG_CCMAX),
	RT_REG(RT5509_REG_OCPMAX),
	RT_REG(RT5509_REG_INTERRUPT),
	RT_REG(RT5509_REG_INTRMASK),
	RT_REG(RT5509_REG_DEGLITCH),
	RT_REG(RT5509_REG_SICRTNSTHACT),
	RT_REG(RT5509_REG_TIMEDET),
	RT_REG(RT5509_REG_TDELAY),
	RT_REG(RT5509_REG_TATKSEL),
	RT_REG(RT5509_REG_TREL),
	RT_REG(RT5509_REG_THOLDREL),
	RT_REG(RT5509_REG_STHLMT),
	RT_REG(RT5509_REG_XTHLMT),
	RT_REG(RT5509_REG_STHALC),
	RT_REG(RT5509_REG_XTHALC),
	RT_REG(RT5509_REG_INITUDT),
	RT_REG(RT5509_REG_UDT),
	RT_REG(RT5509_REG_DNHALFT),
	RT_REG(RT5509_REG_ALCGAIN),
	RT_REG(RT5509_REG_ADAPTCONF),
	RT_REG(RT5509_REG_INITIMPLDMU),
	RT_REG(RT5509_REG_IMPLDMU),
	RT_REG(RT5509_REG_GPILOT),
	RT_REG(RT5509_REG_PILOTEN),
	RT_REG(RT5509_REG_PILOTNISENSE),
	RT_REG(RT5509_REG_ISENSEGAIN),
	RT_REG(RT5509_REG_RAPP),
	RT_REG(RT5509_REG_DCR_MAX),
	RT_REG(RT5509_REG_DCR_KD),
	RT_REG(RT5509_REG_DCR_KP),
	RT_REG(RT5509_REG_DCR_KI),
	RT_REG(RT5509_REG_INITDCRIDMU),
	RT_REG(RT5509_REG_DCRIDMU),
	RT_REG(RT5509_REG_CALIB_DCR),
	RT_REG(RT5509_REG_CALIB_BL),
	RT_REG(RT5509_REG_CALIB_CTRL),
	RT_REG(RT5509_REG_CALIB_REQ),
	RT_REG(RT5509_REG_CALIB_GAIN),
	RT_REG(RT5509_REG_CALIB_OUT0),
	RT_REG(RT5509_REG_CALIB_OUT1),
	RT_REG(RT5509_REG_XTHLMTDAM),
	RT_REG(RT5509_REG_RMAXDAM),
	RT_REG(RT5509_REG_TSCALEDAM),
	RT_REG(RT5509_REG_RECOVERT),
	RT_REG(RT5509_REG_SETRESFREQ),
	RT_REG(RT5509_REG_GETRESFREQ),
	RT_REG(RT5509_REG_VOLCTL),
	RT_REG(RT5509_REG_VOLUME),
	RT_REG(RT5509_REG_CALIB_OUTX),
	RT_REG(RT5509_REG_CALIB_OUTY),
	RT_REG(RT5509_REG_BQ1),
	RT_REG(RT5509_REG_BQ2),
	RT_REG(RT5509_REG_BQ3),
	RT_REG(RT5509_REG_BQ4),
	RT_REG(RT5509_REG_BQ5),
	RT_REG(RT5509_REG_BQ6),
	RT_REG(RT5509_REG_BQ7),
	RT_REG(RT5509_REG_BQ8),
	RT_REG(RT5509_REG_BQ9),
	RT_REG(RT5509_REG_BQ10),
	RT_REG(RT5509_REG_VBBQ1),
	RT_REG(RT5509_REG_VBBQ2),
	RT_REG(RT5509_REG_VBBQ3),
	RT_REG(RT5509_REG_VBBQ4),
	RT_REG(RT5509_REG_VBBQ5),
	RT_REG(RT5509_REG_VBBQ6),
	RT_REG(RT5509_REG_VBBQ7),
	RT_REG(RT5509_REG_VBBQ8),
	RT_REG(RT5509_REG_VBBQ9),
	RT_REG(RT5509_REG_VBFCN),
	RT_REG(RT5509_REG_VBGAIN1),
	RT_REG(RT5509_REG_VBGAIN2),
	RT_REG(RT5509_REG_VBGAIN3),
	RT_REG(RT5509_REG_VBGAIN4),
	RT_REG(RT5509_REG_VBGAIN5),
	RT_REG(RT5509_REG_VBGAIN6),
	RT_REG(RT5509_REG_VBGAIN7),
	RT_REG(RT5509_REG_VBGAIN8),
	RT_REG(RT5509_REG_VBGAIN9),
	RT_REG(RT5509_REG_VBGAIN10),
	RT_REG(RT5509_REG_SLOPCONST),
	RT_REG(RT5509_REG_BWCOEFF),
	RT_REG(RT5509_REG_SWRESET),
	RT_REG(RT5509_REG_SPKGAIN),
	RT_REG(RT5509_REG_DSPKCONF1),
	RT_REG(RT5509_REG_DSPKCONF2),
	RT_REG(RT5509_REG_DSPKCONF3),
	RT_REG(RT5509_REG_DSPKCONF4),
	RT_REG(RT5509_REG_DSPKVMID),
	RT_REG(RT5509_REG_DSPKZCBOOST),
	RT_REG(RT5509_REG_ISENSE_CTRL),
	RT_REG(RT5509_REG_DIMADC),
	RT_REG(RT5509_REG_DSPKEN1),
	RT_REG(RT5509_REG_VBATDATA),
	RT_REG(RT5509_REG_VTHRMDATA),
	RT_REG(RT5509_REG_VBATSENSE),
	RT_REG(RT5509_REG_IDACTSTNINFO),
	RT_REG(RT5509_REG_IDACBOOST),
	RT_REG(RT5509_REG_DSPKEN2),
	RT_REG(RT5509_REG_DSPKIBCONF1),
	RT_REG(RT5509_REG_DSPKIBCONF2),
	RT_REG(RT5509_REG_DSPKIBCONF3),
	RT_REG(RT5509_REG_DSPKCONF5),
	RT_REG(RT5509_REG_DSPKCONF6),
	RT_REG(RT5509_REG_OVPUVPCTRL),
	RT_REG(RT5509_REG_PLLCONF1),
	RT_REG(RT5509_REG_PLLCONF2),
	RT_REG(RT5509_REG_PLLCONF3),
	RT_REG(RT5509_REG_PLLCONF4),
	RT_REG(RT5509_REG_PLLINFO),
	RT_REG(RT5509_REG_PLLDIVISOR),
	RT_REG(RT5509_REG_ZCCONF),
	RT_REG(RT5509_REG_DCADJ),
	RT_REG(RT5509_REG_I2CBCKLRCKCONF),
	RT_REG(RT5509_REG_TDEN),
	RT_REG(RT5509_REG_ALPHACONF),
	RT_REG(RT5509_REG_SPKRPTSEL),
	RT_REG(RT5509_REG_SPKRPT),
	RT_REG(RT5509_REG_NDELAY),
	RT_REG(RT5509_REG_DELAYRES),
	RT_REG(RT5509_REG_PHI1),
	RT_REG(RT5509_REG_PHI2),
	RT_REG(RT5509_REG_PHI3),
	RT_REG(RT5509_REG_PHI4),
	RT_REG(RT5509_REG_PHI5),
	RT_REG(RT5509_REG_ADAPTB0),
	RT_REG(RT5509_REG_ADAPTB1),
	RT_REG(RT5509_REG_ADAPTB2),
	RT_REG(RT5509_REG_ADAPTB3),
	RT_REG(RT5509_REG_ADAPTB4),
	RT_REG(RT5509_REG_ADAPTB5),
	RT_REG(RT5509_REG_COEFSIERA),
	RT_REG(RT5509_REG_COEFHPF),
	RT_REG(RT5509_REG_MIMATC_CTRL),
	RT_REG(RT5509_REG_TDM_CTRL),
	RT_REG(RT5509_REG_ECO_CTRL),
	RT_REG(RT5509_REG_BSTTM),
	RT_REG(RT5509_REG_OTPCONF),
	RT_REG(RT5509_REG_OTPDIN),
	RT_REG(RT5509_REG_VBG_TRIM),
	RT_REG(RT5509_REG_VTEMP_TRIM),
	RT_REG(RT5509_REG_TCOEFF),
	RT_REG(RT5509_REG_SPSCONF),
	RT_REG(RT5509_REG_SPSTHR),
	RT_REG(RT5509_REG_VTHERMBATEN),
	RT_REG(RT5509_REG_DBGADS),
	RT_REG(RT5509_REG_TESTDAC),
	RT_REG(RT5509_REG_SPKDCS),
	RT_REG(RT5509_REG_MSKFLAG),
	RT_REG(RT5509_REG_DRCMINGAIN),
	RT_REG(RT5509_REG_DRC_SEL),
	RT_REG(RT5509_REG_DRC_ATTACK),
	RT_REG(RT5509_REG_DRC_PARAM),
	RT_REG(RT5509_REG_DRCBQ1),
	RT_REG(RT5509_REG_DRCBQ2),
	RT_REG(RT5509_REG_DRCBQ3),
	RT_REG(RT5509_REG_DRCBQ4),
	RT_REG(RT5509_REG_DRCBQ5),
	RT_REG(RT5509_REG_DRCBQ6),
	RT_REG(RT5509_REG_DRCBQ7),
	RT_REG(RT5509_REG_DRCBQ8),
	RT_REG(RT5509_REG_DRCBQ9),
	RT_REG(RT5509_REG_DRCBQ10),
	RT_REG(RT5509_REG_DRCBQ11),
	RT_REG(RT5509_REG_DRCBQ12),
	RT_REG(RT5509_REG_DRCEN),
	RT_REG(RT5509_REG_MTPFLOW1),
	RT_REG(RT5509_REG_MTPFLOW2),
	RT_REG(RT5509_REG_MTPFLOW3),
	RT_REG(RT5509_REG_MTPFLOW4),
	RT_REG(RT5509_REG_MTPFLOW5),
	RT_REG(RT5509_REG_MTPFLOW6),
	RT_REG(RT5509_REG_MTPFLOW7),
	RT_REG(RT5509_REG_MTPFLOW8),
	RT_REG(RT5509_REG_MTPFLOW9),
	RT_REG(RT5509_REG_MTPFLOWA),
	RT_REG(RT5509_REG_MTPFLOWB),
	RT_REG(RT5509_REG_MTPFLOWC),
	RT_REG(RT5509_REG_MTPFLOWD),
	RT_REG(RT5509_REG_MTPFLOWE),
	RT_REG(RT5509_REG_MTPFLOWF),
	RT_REG(RT5509_REG_TESTMODE1),
	RT_REG(RT5509_REG_RAMIND1),
	RT_REG(RT5509_REG_RAMIND2),
	RT_REG(RT5509_REG_SCANMODE),
	RT_REG(RT5509_REG_CLKEN1),
	RT_REG(RT5509_REG_CLKEN2),
	RT_REG(RT5509_REG_PADDRV),
	RT_REG(RT5509_REG_TESTMODE2),
	RT_REG(RT5509_REG_SLEWRATE1),
	RT_REG(RT5509_REG_SLEWRATE2),
	RT_REG(RT5509_REG_BIASRESISTOR),
	RT_REG(RT5509_REG_SPKDRV),
	RT_REG(RT5509_REG_BLOCKREF1),
	RT_REG(RT5509_REG_BLOCKREF2),
	RT_REG(RT5509_REG_BIASCURRENT),
	RT_REG(RT5509_REG_BIASOPTION),
};
#define REGISTER_NUM ARRAY_SIZE(rt5509_regmap)

static struct rt_regmap_properties rt5509_regmap_props = {
	.register_num = REGISTER_NUM,
	.rm = rt5509_regmap,
	.rt_regmap_mode = RT_MULTI_BYTE | RT_CACHE_DISABLE,
	.aliases = "rt5509",
};

#if RT5509_SIMULATE_DEVICE
int rt5509_calculate_offset(int reg)
{
	int i = 0;
	int offset = 0;
	int ret = -EINVAL;

	for (i = 0; i < REGISTER_NUM; i++,
		offset += (rt5509_regmap[i]->size)) {
		if (rt5509_regmap[i]->addr == reg) {
			ret = offset;
			break;
		}
	}
	return ret;
}
EXPORT_SYMBOL(rt5509_calculate_offset);

int rt5509_calculate_total_size(void)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < REGISTER_NUM; i++)
		ret += rt5509_regmap[i]->size;
	return ret;
}
EXPORT_SYMBOL(rt5509_calculate_total_size);
#endif /* #if RT5509_SIMULATE_DEVICE */
#endif /* #ifdef CONFIG_RT_REGMAP */

/* ---------------------------------------------------------------------
 * RT5509 register map related function
 *
 */
struct rt_regmap_device *rt5509_regmap_register(
	struct rt_regmap_fops *regmap_ops,
	struct device *parent, void *client, void *drvdata)
{
	rt5509_regmap_props.name = kasprintf(GFP_KERNEL,
		"rt5509.%s", dev_name(parent));
#ifdef CONFIG_RT_REGMAP
	return rt_regmap_device_register(
		&rt5509_regmap_props, regmap_ops, parent, client, drvdata);
#else
	return 0;
#endif /* #ifdef CONFIG_RT_REGMAP */
}
EXPORT_SYMBOL_GPL(rt5509_regmap_register);
