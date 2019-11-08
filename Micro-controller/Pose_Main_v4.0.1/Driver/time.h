#ifndef _TIME_H
#define	_TIME_H

#include "stm32f10x.h"
#include "POSE_Drv_MPU9250.h"
#include "DL_LN3X.h"

void TIM3_INIT(void);
void sys_time(void);
u16 Get_Time(u8,u16,u16);
extern s16 time_1h,time_1m,time_1s,time_1ms;
extern u8 acc_ng_cali;
extern u8 imu_ready;
#endif



