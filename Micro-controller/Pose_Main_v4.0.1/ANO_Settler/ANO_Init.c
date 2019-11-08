#include "ANO_Init.h"

u8 NRF_ENABLE;
void sys_init()
{
	//中断优先级组别设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);			//中断优先级组别设置	 (NVIC_PriorityGroup_4: 4 bits for pre-emption priority and 0 bits for subpriority)
	
	//初始化系统滴答定时器
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);				   /*cnt = 72000:1ms滴答一次中断*/
	
	//led初始化
	//POSE_LED_Init();
	//LED_Flash_Duty(40);
	//i2c初始化
	//I2c_Soft_Init();
	
	ANO_Uart1_Init(115200);					   /*作为数传接口*/
	ANO_UART3_Init(500000);					   /*zigbee模块接口*/
	
	
	Delay_ms(100);
	
	//Init_MPU9250(20,20);

	Delay_ms(200);
	
//	DL_LN3X_INIT();
	DL_LN3X_Check();
	
	//vl53l0x_init(&vl53l0x_dev);//vl53l0x初始化
	//Delay_ms(100);
	//vl53l0x_calibration(&vl53l0x_dev);
	//Delay_ms(100);
	//vl53l0x_general(&vl53l0x_dev);

}

/************************END OF FILE************/


