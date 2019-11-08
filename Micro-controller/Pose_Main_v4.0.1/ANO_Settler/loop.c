#include "loop.h"


loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_6ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag >= 2)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期2ms内没有跑完。
	}
	else
	{	
		loop.check_flag += 1;	//该标志位在循环的最后被清零
	}
	//LED_1ms_DRV(); 				//1ms中断里边，1ms执行一次
}

u32 test_time[10];
void Duty_2ms()
{
	test_time[0] = GetSysTime_us();

	ANO_DT_Data_Exchange(); 		//数据发送
	
	//LED_Flash_Control(1);
	



		
	test_time[1] = GetSysTime_us();
	test_time[2] = test_time[1] - test_time[0];	
}


void Duty_6ms()
{
	test_time[3] = GetSysTime_us();	

	test_time[4] = GetSysTime_us();
	test_time[5] = test_time[4] - test_time[3];
	
	
}
void Duty_10ms()
{

}

void Duty_20ms()
{
	//LED_Dyty(0.02f);
}

void Duty_50ms()
{
	//VL53L0X_Error Status = VL53L0X_ERROR_NONE;//工作状态
    DL_DT_Data_Exchange();			/*zigbee发送函数*/
	//Status = vl53l0x_start_SingleMeasure(&vl53l0x_dev, &vl53l0x_data);//执行一次测量(Distance_data)
}


void main_loop()
{
	if( loop.check_flag >= 1 )
	{
		
		//Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_6ms >= 6 )
		{
			loop.cnt_6ms = 0;
			Duty_6ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		} 
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}

}
/************************END OF FILE************/


