
/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name	: lis2dh_acc.h
* Authors	: MH - C&I BU - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
* Version	: V.1.0.12
* Date		: 2012/Feb/29
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/
/*******************************************************************************
Version History.

 Revision 1.0.10: 2011/Aug/16

 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
  renamed field g_range to fs_range in lis2dh_acc_platform_data;
  replaced defines SA0L and SA0H with LIS2DH_SAD0x
*******************************************************************************/

#ifndef	__LIS2DH12_H__
#define	__LIS2DH12_H__

/* Accelerometer Sensor Full Scale */
#define	LIS2DH_ACC_FS_MASK		(0x30)
#define LIS2DH_ACC_G_2G			(0x00)
#define LIS2DH_ACC_G_4G			(0x10)
#define LIS2DH_ACC_G_8G			(0x20)
#define LIS2DH_ACC_G_16G		(0x30)


#include <stdbool.h>

bool lis2dh12_init(void);
void lis2dh12_powerDown(void);
void lis2dh12_acc_data(float acc_g_xyz[3], float cal_acc_g_xyz[3]);
void lis2dh12_acc_data_calibrate(float acc_g_xyz[3], float cal_acc_g_xyz[3]);

#endif	/* __LIS2DH12_H__ */
