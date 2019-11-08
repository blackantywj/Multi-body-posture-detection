#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "ANO_Config.h"
#include "stm32f10x.h"
#include "i2c_soft.h"
#include "bsp_SysTick.h"
#include "mymath.h"

#if VL15310x

#include "vl53L0X_def.h"
#include "vl53l0x_api.h"

//使能2.8V IO电平模式
#define USE_I2C_2V8  1

//测量模式
#define Default_Mode   0// 默认
#define HIGH_ACCURACY  1//高精度
#define LONG_RANGE     2//长距离
#define HIGH_SPEED     3//高速

//vl53l0x模式配置参数集
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal极限数值 
	FixPoint1616_t sigmaLimit;     //Sigmal极限数值
	uint32_t timingBudget;         //采样时间周期
	uint8_t preRangeVcselPeriod;  //VCSEL脉冲周期
	uint8_t finalRangeVcselPeriod;//VCSEL脉冲周期范围

}mode_data;
//vl53l0x传感器校准信息结构体定义
typedef  struct
{
	uint8_t  adjustok;                    //校准成功标志，0XAA，已校准;其他，未校准
	uint8_t  isApertureSpads;             //ApertureSpads值
	uint8_t  VhvSettings;                 //VhvSettings值
	uint8_t  PhaseCal;                    //PhaseCal值
	uint32_t XTalkCalDistance;            //XTalkCalDistance值
	uint32_t XTalkCompensationRateMegaCps;//XTalkCompensationRateMegaCps值
	uint32_t CalDistanceMilliMeter;       //CalDistanceMilliMeter值
	int32_t  OffsetMicroMeter;            //OffsetMicroMeter值
	uint32_t refSpadCount;                //refSpadCount值

}_vl53l0x_adjust;


//VL53L0X传感器上电默认IIC地址为0X52(不包含最低位)
#define VL53L0X_Addr     (0x52>>1)
#define SET_VL53L0X_Addr (0x54)		  /*设置模块地址*/

extern VL53L0X_Dev_t vl53l0x_dev;

extern _vl53l0x_adjust Vl53l0x_data;

extern VL53L0X_RangingMeasurementData_t vl53l0x_data;
extern vu16 Distance_data;			  /*测距数据(单位:mm)*/
extern vu8  Distance_state;

VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr);
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev);

void vl53l0x_general(VL53L0X_Dev_t *dev);
void vl53l0x_general_start(VL53L0X_Dev_t *dev, u8 mode);
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev, u8 mode);
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *pdata, char *buf);
VL53L0X_Error vl53l0x_start_SingleMeasure(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *pdata);
void vl53l0x_reset(VL53L0X_Dev_t *dev);

void vl53l0x_calibration(VL53L0X_Dev_t *dev);
VL53L0X_Error vl53l0x_adjust(VL53L0X_Dev_t *dev);

#endif

#endif
