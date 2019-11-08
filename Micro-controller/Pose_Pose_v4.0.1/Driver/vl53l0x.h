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

//ʹ��2.8V IO��ƽģʽ
#define USE_I2C_2V8  1

//����ģʽ
#define Default_Mode   0// Ĭ��
#define HIGH_ACCURACY  1//�߾���
#define LONG_RANGE     2//������
#define HIGH_SPEED     3//����

//vl53l0xģʽ���ò�����
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal������ֵ 
	FixPoint1616_t sigmaLimit;     //Sigmal������ֵ
	uint32_t timingBudget;         //����ʱ������
	uint8_t preRangeVcselPeriod;  //VCSEL��������
	uint8_t finalRangeVcselPeriod;//VCSEL�������ڷ�Χ

}mode_data;
//vl53l0x������У׼��Ϣ�ṹ�嶨��
typedef  struct
{
	uint8_t  adjustok;                    //У׼�ɹ���־��0XAA����У׼;������δУ׼
	uint8_t  isApertureSpads;             //ApertureSpadsֵ
	uint8_t  VhvSettings;                 //VhvSettingsֵ
	uint8_t  PhaseCal;                    //PhaseCalֵ
	uint32_t XTalkCalDistance;            //XTalkCalDistanceֵ
	uint32_t XTalkCompensationRateMegaCps;//XTalkCompensationRateMegaCpsֵ
	uint32_t CalDistanceMilliMeter;       //CalDistanceMilliMeterֵ
	int32_t  OffsetMicroMeter;            //OffsetMicroMeterֵ
	uint32_t refSpadCount;                //refSpadCountֵ

}_vl53l0x_adjust;


//VL53L0X�������ϵ�Ĭ��IIC��ַΪ0X52(���������λ)
#define VL53L0X_Addr     (0x52>>1)
#define SET_VL53L0X_Addr (0x54)		  /*����ģ���ַ*/

extern VL53L0X_Dev_t vl53l0x_dev;

extern _vl53l0x_adjust Vl53l0x_data;

extern VL53L0X_RangingMeasurementData_t vl53l0x_data;
extern vu16 Distance_data;			  /*�������(��λ:mm)*/
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
