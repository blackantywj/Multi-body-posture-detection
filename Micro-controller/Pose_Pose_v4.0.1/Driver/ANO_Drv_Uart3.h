#ifndef __ANO_DRV_UART3_H__
#define __ANO_DRV_UART3_H__

#include "include.h"
#include "ANO_Data_Transfer.h"

extern u8 RxState3;

void ANO_UART3_Init(u32 br_num);
void ANO_UART3_DeInit(void);
void ANO_UART3_IRQ(void);

void ANO_UART3_Put_Char(unsigned char DataToSend);
void ANO_Uart3_Send(unsigned char *DataToSend, u8 data_num);
void ANO_UART3_Put_String(unsigned char *Str);
//void ANO_UART3_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
