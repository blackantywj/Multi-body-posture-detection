#ifndef FUNCTION_H
#define FUNCTION_H

#include "stm32f10x.h"
#include "ANO_Drv_MPU6050.h"
#include "imu.h"
#include "ANO_Data_Transfer.h"

typedef struct
{
	u8 check_flag;
	u16 err_flag;
	s16 cnt_1ms;
	s16 cnt_2ms;
	s16 cnt_6ms;
	s16 cnt_10ms;
	s16 cnt_20ms;
	s16 cnt_50ms;
	u16 time;
}loop_t;

void main_loop(void);
void Loop_check(void);
#endif
