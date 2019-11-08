#ifndef _include_H_
#define _include_H_

#include "ANO_Config.h"
#include "stm32f10x.h"
#include "arm_math.h"
#include "mymath.h"
#include "ANO_Data.h"
#include "ANO_Param.h"
#include "i2c_soft.h"
#include "ANO_Init.h"


#include "ANO_Drv_Uart.h"
#include "ANO_Drv_Uart3.h"
#include "ANO_Drv_SPI.h"
#include "ANO_Drv_LED.h"
#include "ANO_Drv_MPU6050.h"
#include "POSE_Drv_MPU9250.h"
#include "DL_LN3X.h"
#include "vl53l0x.h"
#include "ms5611.h"

#include "bsp_SysTick.h"



#include "loop.h"

//#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)  /*!< JTAG-DP Disabled and SW-DP Enabled */
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
/*******************IO口地址映射**************/
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 

/**********IO口操作,只对单一的IO口!*************/
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define HW_TYPE	1
#define HW_VER	3
#define BL_VER	100
#define PT_VER	400

/***************LED GPIO定义******************/
#define ANO_RCC_LED_M				RCC_APB2Periph_GPIOB
#define ANO_GPIO_LED_M				GPIOB
#define ANO_Pin_LED_M				GPIO_Pin_8
#define ANO_RCC_LED_0				RCC_APB2Periph_GPIOB
#define ANO_GPIO_LED_0				GPIOB
#define ANO_Pin_LED_0				GPIO_Pin_3
#define ANO_RCC_LED_1				RCC_APB2Periph_GPIOA
#define ANO_GPIO_LED_1				GPIOA
#define ANO_Pin_LED_1				GPIO_Pin_15
#define POSE_RCC_LED_0				RCC_APB2Periph_GPIOC
#define POSE_GPIO_LED_0				GPIOC
#define POSE_Pin_LED_0				GPIO_Pin_13
/*********************************************/

/***************UART1 GPIO定义******************/
#define ANO_RCC_UART1		RCC_APB2Periph_GPIOA
#define ANO_GPIO_UART1		GPIOA
#define ANO_UART1_Pin_TX	GPIO_Pin_9
#define ANO_UART1_Pin_RX	GPIO_Pin_10
/*********************************************/
/***************UART3 GPIO定义******************/
#define ANO_RCC_UART3		RCC_APB2Periph_GPIOB
#define ANO_GPIO_UART3		GPIOB
#define ANO_UART3_Pin_TX	GPIO_Pin_10
#define ANO_UART3_Pin_RX	GPIO_Pin_11
/*********************************************/
/***************SPI GPIO定义******************/
#define ANO_GPIO_SPI		GPIOB
#define ANO_GPIO_CE			GPIOA
#define RCC_GPIO_SPI		RCC_APB2Periph_GPIOB
#define RCC_GPIO_CE			RCC_APB2Periph_GPIOA
#define SPI_Pin_CE			GPIO_Pin_8
#define SPI_Pin_CSN			GPIO_Pin_12
#define SPI_Pin_SCK			GPIO_Pin_13
#define SPI_Pin_MISO		GPIO_Pin_14
#define SPI_Pin_MOSI		GPIO_Pin_15
/*********************************************/
/***************硬件中断优先级******************/
#define NVIC_UART1_P	5
#define NVIC_UART1_S	1
#define NVIC_UART3_P	4
#define NVIC_UART3_S	1
/***********************************************/
//================传感器===================
#define ACC_ADJ_EN 									//是否允许校准加速度计,(定义则允许)
#define TO_ANGLE 				0.06103f 		    //0.061036 //   4000/65536  +-2000   (陀螺仪:最大量程 +-2000度每秒  (65536/4000=16.4LSB/(°/S)))
#define TO_M_S2 				0.23926f   	        //   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN         0.01745329f         //*0.01745 = /57.3	角度转弧度

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_Y ,
 G_X ,
 G_Z ,
 TEM ,
 ITEMS ,
};

// CH_filter[],0横滚，1俯仰，2油门，3航向		
enum
{
 ROL= 0,
 PIT ,
 THR ,
 YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
// AUX5 ,
// AUX6 ,
// AUX7 ,
// AUX8 ,
// AUX9 ,
// AUX10 ,
// AUX11 ,
// AUX12 ,
 CH_NUM,
};

enum
{
	m1=0,
	m2,
	m3,
	m4,
	m5,
	m6,
	m7,
	m8,

};
#if MPU6050
enum
{
	MPU_6050_0 = 0,
	MPU_6050_1,
	
};
#endif //MPU6050

#endif


