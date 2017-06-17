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

/*********************************************
* HAL API
**********************************************/

void vibr_Enable_HW(void);
void vibr_Disable_HW(void);
void vibr_power_set(void);
/* [BY57] S- BUG#11 Grace_Chang Setup vibrator */
void vibr_vosel_cal_set(unsigned int vosel, unsigned int cal);
void vibr_vosel_cal_get(unsigned int *vosel, unsigned int *cal);
/* [BY57] E- BUG#11 Grace_Chang Setup vibrator */
struct vibrator_hw *mt_get_cust_vibrator_hw(void);
