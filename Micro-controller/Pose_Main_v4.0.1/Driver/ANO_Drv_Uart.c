#include "ANO_Drv_Uart.h"

void ANO_Uart1_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   //复用IO时钟
	RCC_APB2PeriphClockCmd(ANO_RCC_UART1,ENABLE);	
	
	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin =  ANO_UART1_Pin_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(ANO_GPIO_UART1 , &GPIO_InitStructure);
	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin =  ANO_UART1_Pin_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ANO_GPIO_UART1 , &GPIO_InitStructure);
	
	//GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	
	//配置USART1
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;				 //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		 //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;			 //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;				    //发送、接收使能
	//配置USART1时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;	 //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;			 //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;		 //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
	
	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStruct);

	//使能USART1接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART1
	USART_Cmd(USART1, ENABLE); 
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART1_P;	/*5级 抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART1_S;			/*应该没有响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void ANO_Uart1_DeInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART1);
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}
u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0;
void ANO_Uart1_IRQ(void)
{
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//ORE中断   (数据溢出)
    {
        USART_ReceiveData(USART1);
    }
		
	//发送中断
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++];                   //写DR清除中断标志(发送一个字节)          
		if(TxCounter == count)								  /*判断是否发完全部有效字节(count在ANO_Uart1_Send函数中更新)*/
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		          //关闭TXE（发送中断）中断
			//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	//接收中断 (接收寄存器非空) 
	if(USART1->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{	//不需要清除中断标志	???
		u8 com_data = USART1->DR;

		//ANO_Uart1_Send(&com_data,1);						  /*接收一个字节发一个字节,测试用*/
	    ANO_DT_Data_Receive_Prepare(com_data);

	}
}
void ANO_Uart1_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
	//if(!(USART1->CR1 & USART_CR1_TXEIE))
		//USART_ITConfig(USART3, USART_IT_TXE, ENABLE); 
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开发送中断(串口1)
}
void ANO_Uart1_Send(unsigned char *DataToSend, u8 data_num)
{
	u8 i;
	for (i = 0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend + i);		  /*加载需要发送的数据字节，并记录数据长度(单位：字节)*/
	}
	//TxBuffer[count++] = DataToSend;
	//if(!(USART1->CR1 & USART_CR1_TXEIE))
	//USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	if (!(USART1->CR1 & USART_CR1_TXEIE))			  /*如果串口1发送中断尚未使能(未占用)*/
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开发送中断
}

void ANO_Uart1_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str)
	{
		//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
		if(*Str=='\r')
			ANO_Uart1_Put_Char(0x0d);
		else if(*Str=='\n')
				ANO_Uart1_Put_Char(0x0a);
			else 
				ANO_Uart1_Put_Char(*Str);
		//指针++ 指向下一个字节.
		Str++;
	}
}
//void ANO_Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num)
//{
//	for(u8 i=0;i<data_num;i++)
//		TxBuffer[count++] = *(DataToSend+i);
//	if(!(USART1->CR1 & USART_CR1_TXEIE))
//		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
//}


