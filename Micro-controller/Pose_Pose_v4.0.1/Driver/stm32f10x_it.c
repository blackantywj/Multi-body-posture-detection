#include "stm32f10x_it.h"
#include "ANO_Drv_Uart.h"
#include "bsp_SysTick.h"

void USART1_IRQHandler(void)  //串口1中断函数
{
	ANO_Uart1_IRQ();
}
void USART3_IRQHandler(void)  //串口3中断函数
{
	ANO_UART3_IRQ();
}

void SysTick_Handler(void)
{
	SysTick_IRQ();
}

