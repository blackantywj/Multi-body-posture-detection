#include "ANO_Drv_Uart3.h"


void ANO_UART3_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART3时钟
	RCC_APB2PeriphClockCmd(ANO_RCC_UART3,ENABLE);	
	
	//Tx  PB10
	GPIO_InitStructure.GPIO_Pin =  ANO_UART3_Pin_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(ANO_GPIO_UART3 , &GPIO_InitStructure);
	//Rx  PB11
	GPIO_InitStructure.GPIO_Pin =  ANO_UART3_Pin_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ANO_GPIO_UART3 , &GPIO_InitStructure);

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

	//使能USART3接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART3
	USART_Cmd(USART3, ENABLE); 
	
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
u8 TxBuffer3[256];
u8 TxCounter3=0;
u8 count3=0;

void ANO_UART3_IRQ(void)
{
	if(USART3->SR & USART_SR_ORE)//ORE中断   (数据溢出)
	{
		u8 com_data = USART3->DR;
		//USART_ClearFlag(USART3,USART_IT_ORE);
	}

	//发送中断
	if((USART3->SR & (1<<7))&&(USART3->CR1 & USART_CR1_TXEIE))
	{
		USART3->DR = TxBuffer3[TxCounter3++];       //写DR清除中断标志(发送一个字节) 
		if(TxCounter3 == count3)					/*判断是否发完全部有效字节(count3在ANO_Uart3_Send函数中更新)*/
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
	//接收中断 (接收寄存器非空) 
	if(USART3->SR & (1<<5))//if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    
	{
		u8 com_data = USART3->DR;

		//ANO_Uart3_Send(&com_data, 1);			    /*接收一个字节发一个字节,测试用*/
	    //ANO_DT_Data_Receive_Prepare(com_data);
		DL_DT_Data_Receive_Prepare(com_data);
	}
}

void ANO_UART3_Put_Char(unsigned char DataToSend)
{
	TxBuffer3[count3++] = DataToSend;
  //if(!(USART3->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE); 	//打开发送中断(串口3)
}
void ANO_Uart3_Send(unsigned char *DataToSend, u8 data_num)
{
	u8 i;
	for (i = 0;i<data_num;i++)
	{
		TxBuffer3[count3++] = *(DataToSend + i);		  /*加载需要发送的数据字节，并记录数据长度(单位：字节)*/
	}
	//TxBuffer[count++] = DataToSend;
	//if(!(USART1->CR1 & USART_CR1_TXEIE))
	//USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	if (!(USART3->CR1 & USART_CR1_TXEIE))			     /*如果串口3发送中断尚未使能(未占用)*/
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE);    //打开发送中断
}
void ANO_UART3_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str)
	{
		//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
		if(*Str=='\r')
			ANO_UART3_Put_Char(0x0d);
		else if(*Str=='\n')
				ANO_UART3_Put_Char(0x0a);
			else 
				ANO_UART3_Put_Char(*Str);
		//指针++ 指向下一个字节.
		Str++;
	}
}
//void ANO_UART3_Put_Buf(unsigned char *DataToSend , u8 data_num)
//{
//	for(u8 i=0;i<data_num;i++)
//		TxBuffer3[count3++] = *(DataToSend+i);
//	if(!(USART3->CR1 & USART_CR1_TXEIE))
//		USART_ITConfig(USART3, USART_IT_TXE, ENABLE);  
//}


