#include "ANO_Drv_Uart3.h"

static u8 Usart3_TxBuffer[DMA_TX_BUFFER_SIZE];
//static u8 Usart3_RxBuffer[DMA_RX_BUFFER_SIZE];
static u8 Flag_Tx_Uart3_Busy = 0;

void ANO_UART3_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART3时钟
	RCC_APB2PeriphClockCmd(ANO_RCC_UART3, ENABLE);
	/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* USART3_TX */
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART3->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart3_TxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = DMA_TX_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	//Tx  PB10
	GPIO_InitStructure.GPIO_Pin = ANO_UART3_Pin_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(ANO_GPIO_UART3, &GPIO_InitStructure);
	//Rx  PB11
	GPIO_InitStructure.GPIO_Pin = ANO_UART3_Pin_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ANO_GPIO_UART3, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = br_num;                 //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		 //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;			 //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;				    //发送、接收使能
																					//配置USART3时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;     //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;			 //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;		 //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

	USART_Init(USART3, &USART_InitStructure);
	USART_ClockInit(USART3, &USART_ClockInitStruct);

	/* DMA */
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

	//使能USART3接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	//使能USART3
	USART_Cmd(USART3, ENABLE);
	//DMA_Cmd(DMA1_Channel2, ENABLE);	 /*注意：没使能DMA1_Channel2(软件主动发送)*/

	//DMA发送中断设置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  	  //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  		  //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  			  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART3_P;	   /*4级 抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART3_S;			   /*应该没有响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void ANO_UART3_DeInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_DeInit(USART3);
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void ANO_UART3_IRQ(void)
{
	if (USART3->SR & USART_SR_ORE)//ORE中断   (数据溢出)
	{
		u8 com_data = USART3->DR;
		//USART_ClearFlag(USART3,USART_IT_ORE);
	}

	//发送完成中断(DMA发送完成后执行)
	if (USART_GetITStatus(USART3, USART_IT_TC) == SET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_TC);
		USART_ITConfig(USART3, USART_IT_TC, DISABLE);   //打开串口3发送完成中断
		Flag_Tx_Uart3_Busy = 0;  			            //【串口3发送】空闲
	}
	//接收中断 (接收寄存器非空) 
	if (USART3->SR & (1 << 5))//if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    
	{
		u8 com_data = USART3->DR;

		//ANO_Uart3_Send(&com_data, 1);			    /*接收一个字节发一个字节,测试用*/
		//ANO_DT_Data_Receive_Prepare(com_data);
		DL_DT_Data_Receive_Prepare(com_data);
	}
}

void Uart3_Tx(u8 *data, u16 size)
{
	while (Flag_Tx_Uart3_Busy);  		 //等待【串口3发送】空闲
	Flag_Tx_Uart3_Busy = 1;  			 //【串口3发送】忙碌
										 //复制数据
	memcpy(Usart3_TxBuffer, data, size);
	//设置传输数据长度
	DMA_SetCurrDataCounter(DMA1_Channel2, size);
	//打开DMA1_数据流2,开始发送(UART3_TX) 
	DMA_Cmd(DMA1_Channel2, ENABLE);
}

//DMA_TX 中断，配合串口TC中断使用
void DMA1_Channel2_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TC2) == SET)	       //DMA传输完成
	{
		DMA_ClearITPendingBit(DMA1_IT_TC2);
		DMA_Cmd(DMA1_Channel2, DISABLE);

		USART_ITConfig(USART3, USART_IT_TC, ENABLE);   //打开串口3发送完成中断
	}
}

