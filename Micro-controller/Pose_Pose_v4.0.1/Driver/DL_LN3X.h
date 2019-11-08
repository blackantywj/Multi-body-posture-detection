#ifndef __DL_LN3X_H__
#define	__DL_LN3X_H__

#include "include.h"
#include "ANO_Data_Transfer.h"
#include "bsp_SysTick.h"
#include "imu.h"

#if DL_LN3X

#define MCU_PORT     (0x80)	   /*内部端口:0x00~0x7F 开放端口:0x80~0xFF*/
#define MCU_PORT1	 (0x81)
#define MCU_PORT2	 (0x82)
/*-------------------本模块配置-------------------*/
#define DL_ADDRESS             (0x0002)					 /*姿态模块1*/
#define DL_ADDRESS_H		   (0x00)
#define DL_ADDRESS_L		   (0x02)
#define DL_Main_ADDRESS_H	   (0x00)
#define DL_Main_ADDRESS_L	   (0x01)
#define DL6_ADDRESS_H		   (0x00)					 /*气压模块*/
#define DL6_ADDRESS_L		   (0x06)
#define DL_INTERNET_ID		   (0x2021)					 //(0x0001~0xfffe)
#define DL_CHANNEL			   (0x0F)					 //(0x0B~0x1A)
#define DL_BAUDRATE			   (BaudRate_500000)
/*-------------------端口-------------------------*/
#define RED_PORT               (0x20)
#define CONFIG_PORT            (0x21)
/*-------------------地址-------------------------*/	 //(0x0001~0xfffe)
#define SELF_MODULE            (0x0000)
#define ALL_NET                (0xFFFF)
/*-------------------读取命令---------------------*/
#define READ_SELF_ADDRESS      (0x01)
#define READ_SELF_INTERNET_ID  (0x02)
#define READ_SELF_CHANNEL      (0x03)
#define READ_SELF_UART         (0x04)
/*-------------------设置命令---------------------*/
#define SET_ADDRESS			   (0x11)
#define SET_INTERNET_ID        (0x12)
#define SET_CHANNEL			   (0x13)
#define SET_BAUDRATE		   (0x14)

#define SET_REBOOT			   (0x10)
/*------------------返回命令----------------------*/
#define OPERATION_SUCCESS         (0x00)

#define	REMOTE_ACCESS_FORBIDDEN	  (0xF0)
#define	ERROR_COMMAND             (0xF8)
#define	ERROR_LENGTH			  (0xF9)
#define	ERROR_VALUE				  (0xFA)

#define	RETURN_ADDRESS			  (0x21)
#define	RETURN_INTERNET_ID		  (0x22)
#define	RETURN_CHANNEL			  (0x23)
#define RETURN_UART				  (0x24)
/*-------------------波特率------------------------*/
#define BaudRate_2400             (0x00)
#define BaudRate_4800             (0x01)
#define BaudRate_9600             (0x02)
#define BaudRate_14400            (0x03)
#define BaudRate_19200            (0x04)
#define BaudRate_28800            (0x05)
#define BaudRate_38400            (0x06)
#define BaudRate_57600            (0x07)
#define BaudRate_115200           (0x08)
#define BaudRate_230400           (0x09)
#define BaudRate_125000           (0x0A)
#define BaudRate_250000           (0x0B)
#define BaudRate_500000           (0x0C)

typedef struct
{
	u8  BaudRate;				  /*波特率*/
	u8  Channel;				  /*信道*/
	u16 Address;				  /*地址*/
	u16 ID;						  /*网络ID*/
}DL_LN3X_STRUCT;

extern DL_LN3X_STRUCT dl_ln3x;

void DL_LN3X_INIT(void);
void DL_LN3X_Check(void);
void DL_DT_Data_Exchange(void);
void DL_DT_Send_ConfigCommand(u8 target_port, u16 target_addr, u8 com, u16 _value);
void DL_DT_Send_ReadCommand(u8 target_port, u16 target_addr, u8 com);
void DL_DT_Send_Status(u16 target_addr, float Roll, float  Pitch, float  Yaw);
void DL_DT_Send_Senser(u16 target_addr, s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z);
void DL_DT_Data_Receive_Prepare(u8 data);
void DL_DT_Data_Receive_Anl(u8 *data_buf, u8 num);




#endif


#endif
