#include "time.h"

void TIM3_CONF()
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		/* 设置TIM3CLK 为 72MHZ */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
    //TIM_DeInit(TIM3);
	
	/* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period=1000;
	
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
	  /* 时钟预分频数为72 */
    TIM_TimeBaseStructure.TIM_Prescaler= 72 - 1;
	
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
		
    TIM_Cmd(TIM3, ENABLE);																		
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , DISABLE);		/*先关闭等待使用*/  
}
void TIM3_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM3_INIT()
{
    TIM3_CONF();
    TIM3_NVIC();
	
		/* TIM3 重新开时钟，开始计时 */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
}

s16 time_1h,time_1m,time_1s,time_1ms;

void sys_time()
{

	
  if(time_1ms < 999)
	{
		time_1ms++;
	}
	else
	{
		
		time_1ms = 0;
		if(time_1s<59)
		{
			//static u8 cnt = 0;
			time_1s++;
			//cnt++;
			//if (cnt == 1)
			//{
			//	DL_DT_Send_ReadCommand(RED_PORT, SELF_MODULE, 100);
			//	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_INTERNET_ID, 0x2021);
			//}
			//if (cnt == 2)
			//{
			//	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_ADDRESS);
			//}
			//if (cnt == 3)
			//{
			//	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_INTERNET_ID);
			//}
			//if (cnt == 4)
			//{
			//	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_CHANNEL);
			//}
			//if (cnt == 5)
			//{
			//	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_UART);
			//}
			/*0h:0m:3s*/
			if ((time_1h == 0) && (time_1m == 0) && (time_1s == 3))
			{
				//mpu9250.Acc_CALIBRATE = 1;
				//mpu9250.Gyro_CALIBRATE = 1;
				//acc_ng_cali = mpu9250.Gyro_CALIBRATE = 2;	/*解锁后矫正*/
			}
			/*0h:0m:5s*/
			if ((time_1h == 0) && (time_1m == 0) && (time_1s == 5))
			{
				imu_ready = 1;
			}
		}
		else
		{
			time_1s = 0;
			if(time_1m<59)
			{
				time_1m++;
			}
			else
			{
				time_1m = 0;
				if(time_1h<23)
				{
					time_1h++;
				}
				else
				{
					time_1h = 0;
				}
			}
		}
	}
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/


