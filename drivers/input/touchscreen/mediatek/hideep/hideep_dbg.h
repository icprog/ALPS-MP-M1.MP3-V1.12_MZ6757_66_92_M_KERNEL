/*************************************************************************
 * Copyright (C) 2015 Hideep, Inc.
 * anthony.kim@hideep.com
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
 *************************************************************************/

#ifndef _LINUX_HIDEEP_DBG_H
#define _LINUX_HIDEEP_DBG_H

/*************************************************************************
 * VR OP Mode register
 *************************************************************************/
#define HIDEEP_OPM_RAW_FLAG					(0x80)
#define HIDEEP_OPM_TOUCH_A					(0x00)
#define HIDEEP_OPM_MOD						(0x80)
#define HIDEEP_OPM_MOD_CAP					(0x81)
#define HIDEEP_OPM_MOD_PRS					(0x82)
#define HIDEEP_OPM_FRAME_PRS				(0x84)
#define HIDEEP_OPM_DEMOD					(0x85)
#define HIDEEP_OPM_DEMOD_CAP				(0x86)
#define HIDEEP_OPM_DEMOD_PRS				(0x87)
#define HIDEEP_OPM_FRAME					(0x88)
#define HIDEEP_OPM_FRAME_CAP				(0x89)
#define HIDEEP_OPM_DIFF						(0x8A)
#define HIDEEP_OPM_DIFF_CAP					(0x8B)
#define HIDEEP_OPM_DIFF_PRS					(0x8C)
#define HIDEEP_OPM_BASELINE_CAP				(0x8D)
#define HIDEEP_OPM_BASELINE_PRS				(0x8E)
#define HIDEEP_OPM_SCDIFF					(0x8F)
#define HIDEEP_OPM_STATUS					(0x90)
#define HIDEEP_OPM_LASTDIFF					(0x91)
#define HIDEEP_OPM_PARM0					(0x92)
#define HIDEEP_OPM_PARM1					(0x93)
#define HIDEEP_OPM_PARM2					(0x94)
#define HIDEEP_OPM_PARM3					(0x95)

/*************************************************************************
 * VR info
 *************************************************************************/
#define ADDR_VR     (0x0000)
#define ADDR_VR_END (0xFFFF)
#define ADDR_IMG    (0x1000)
#define ADDR_UC     (0x10000000)
#define MAX_VR_BUFF  1024

#define HIDEEP_4RCF_DATA	0x1001
#define HIDEEP_EOP_DATA		0x1011
#define HIDEEP_4RC_DATA		0x1021
#define HIDEEP_1RC_DATA		0x1011

#define HIDEEP_Z_VALUE		0x246

#define HIDEEP_Z_CALIB2			0x1004
#define HIDEEP_Z_CALIB2_READ	0x70007000
#define HIDEEP_VERSION_INFO	0x10000001

#define HIDEEP_RELEASE_FLAG		0x8000

/* IOCTL command */
#define HIDEEP_IOC_MAGIC  'k'
#define HIDEEP_CFG     _IOW(HIDEEP_IOC_MAGIC,  0x01, struct hideep_debug_cfg_t)
#define HIDEEP_RDIM    _IOW(HIDEEP_IOC_MAGIC,  0x02, int)
#define HIDEEP_SELF_TEST	_IOWR(HIDEEP_IOC_MAGIC, 0x03, unsigned char*)
#define HIDEEP_IOC_MAXNR 0xff

#endif /* _LINUX_HIDEEP_DBG_H */
