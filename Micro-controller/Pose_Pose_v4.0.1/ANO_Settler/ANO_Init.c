#include "ANO_Init.h"



void sys_init()
{
	//中断优先级组别设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);			//中断优先级组别设置	 (NVIC_PriorityGroup_4: 4 bits for pre-emption priority and 0 bits for subpriority)
	
	//初始化系统滴答定时器
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);				   /*cnt = 72000:1ms滴答一次中断*/
	
	//led初始化
	POSE_LED_Init();
	LED_Flash_Duty(led_duty);
	//i2c初始化
	I2c_Soft_Init();
	
#if POSE_DT_USE_USART1
	ANO_Uart1_Init(115200);					   /*作为数传接口*/
#endif // POSE_DT_USE_USART1

#if DL_DT_USE_USART3
	ANO_UART3_Init(500000);					   /*zigbee模块接口*/
#endif // DL_DT_USE_USART3

	Delay_ms(100);

#if MPU9250
	Init_MPU9250(20, 20);
	//参数初始化
	POSE_Param_Read();
#endif // MPU9250

	Delay_ms(200);
	
#if DL_DT_USE_USART3
//	DL_LN3X_INIT();
	DL_LN3X_Check();
#endif // DL_DT_USE_USART3

	Kalman_IMMU_Init();

}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/


