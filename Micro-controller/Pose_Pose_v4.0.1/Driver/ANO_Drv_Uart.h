#ifndef __ANO_DRV_UART_H__
#define __ANO_DRV_UART_H__

#include "include.h"
#include "ANO_Data_Transfer.h"

void ANO_Uart1_Init(u32 br_num);
void ANO_Uart1_DeInit(void);
void ANO_Uart1_IRQ(void);
void ANO_Uart1_Put_Char(unsigned char DataToSend);
void ANO_Uart1_Send(unsigned char *DataToSend, u8 data_num);
void ANO_Uart1_Put_String(unsigned char *Str);
//void ANO_Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
