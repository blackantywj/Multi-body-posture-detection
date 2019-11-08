#ifndef __ANO_DRV_UART3_H__
#define __ANO_DRV_UART3_H__

#include "include.h"
#include "ANO_Data_Transfer.h"


#define DMA_TX_BUFFER_SIZE   128

void ANO_UART3_Init(u32 br_num);
void ANO_UART3_DeInit(void);
void ANO_UART3_IRQ(void);
extern void Uart3_Tx(u8 *data, u16 size);

#endif

