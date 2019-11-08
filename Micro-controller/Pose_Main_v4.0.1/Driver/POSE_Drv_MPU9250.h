#ifndef __POSE_DRV_MPU9250_H__
#define	__POSE_DRV_MPU9250_H__

#include "stm32f10x.h"
#include "ANO_Data.h"
#include "ANO_Param.h"
#include "include.h"
#include "ANO_Config.h"
#include "ANO_Data_Transfer.h"

#if MPU9250

#define OFFSET_AV_NUM 50
#define FILTER_NUM 10
#define CALIBRATING_MAG_CYCLES              2000  //校准时间持续20s

// 定义MPU9250内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU9250_CONFIG	0x1A	//低通滤波频率，典型值：0x04(20Hz)	 (CONFIG)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
//新加
#define ACCEL_CONFIG2   0X1D    //加速度计低通滤波器 0x06 5hz
#define PWR_MGMT_1      0X6B    //电源管理1 典型值为0x00
#define PWR_MGMT_2      0X6C    //电源管理2 典型值为0X00

//#define WHO_AM_I        0X75    //器件ID MPU9250默认ID为0X71
#define USER_CTRL       0X6A    //用户配置当为0X10时使用SPI模式

#define MPU9250_CS      PDout(3)//MPU9250片选信号
#define I2C_ADDR        0X68    //i2c的地址
//新加结束
#define MPU9250_XG_OFFSET_H         (0x13)
#define MPU9250_XG_OFFSET_L         (0x14)
#define MPU9250_YG_OFFSET_H         (0x15)
#define MPU9250_YG_OFFSET_L         (0x16)
#define MPU9250_ZG_OFFSET_H         (0x17)
#define MPU9250_ZG_OFFSET_L         (0x18)

#define MPU9250_XA_OFFSET_H         (0x77)
#define MPU9250_XA_OFFSET_L         (0x78)
#define MPU9250_YA_OFFSET_H         (0x7A)
#define MPU9250_YA_OFFSET_L         (0x7B)
#define MPU9250_ZA_OFFSET_H         (0x7D)
#define MPU9250_ZA_OFFSET_L         (0x7E)

#define	MPU9250_ACCEL_XOUT_H	0x3B
#define	MPU9250_ACCEL_XOUT_L	0x3C
#define	MPU9250_ACCEL_YOUT_H	0x3D
#define	MPU9250_ACCEL_YOUT_L	0x3E
#define	MPU9250_ACCEL_ZOUT_H	0x3F
#define	MPU9250_ACCEL_ZOUT_L	0x40

#define	MPU9250_TEMP_OUT_H		0x41
#define	MPU9250_TEMP_OUT_L		0x42

#define	MPU9250_GYRO_XOUT_H		0x43
#define	MPU9250_GYRO_XOUT_L		0x44	
#define	MPU9250_GYRO_YOUT_H		0x45
#define	MPU9250_GYRO_YOUT_L		0x46
#define	MPU9250_GYRO_ZOUT_H		0x47
#define	MPU9250_GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define MPU9250_INT_PIN_CFG         (0x37)   //INT引脚配置和Bypass模式配置寄存器
#define MPU9250_INT_ENABLE          (0x38)
#define MPU9250_INT_STATUS          (0x3A)
#define	WHO_AM_I		0x75	             //IIC地址寄存器(默认数值0x73，只读)
//****************采样速率****************
#define	MPU9250_SMPLRT_DIV_125HZ      0x07
#define	MPU9250_SMPLRT_DIV_250HZ      0x03
#define	MPU9250_SMPLRT_DIV_333HZ      0x02
#define	MPU9250_SMPLRT_DIV_500HZ      0x01
#define	MPU9250_SMPLRT_DIV_1KHZ       0x00
//****************************************
//****************滤波参数****************
#define	MPU9250_GYRO_DLPF_BW_250HZ    0x00
#define	MPU9250_GYRO_DLPF_BW_184HZ    0x01
#define	MPU9250_GYRO_DLPF_BW_92HZ     0x02
#define	MPU9250_GYRO_DLPF_BW_41HZ     0x03
#define	MPU9250_GYRO_DLPF_BW_20HZ     0x04
#define	MPU9250_GYRO_DLPF_BW_10HZ     0x05
#define	MPU9250_GYRO_DLPF_BW_5HZ      0x06
#define	MPU9250_GYRO_DLPF_BW_DISABLE  0x07

#define	MPU9250_ACCE_DLPF_BW_460HZ    0x00
#define	MPU9250_ACCE_DLPF_BW_184HZ    0x01
#define	MPU9250_ACCE_DLPF_BW_92HZ     0x02
#define	MPU9250_ACCE_DLPF_BW_41HZ     0x03
#define	MPU9250_ACCE_DLPF_BW_20HZ     0x04
#define	MPU9250_ACCE_DLPF_BW_10HZ     0x05
#define	MPU9250_ACCE_DLPF_BW_5HZ      0x06
#define	MPU9250_ACCE_DLPF_BW_DISABLE  0x08
//****************************************
//****************加速度量程**************
#define	MPU9250_ACCE_FS_2G     0x00		  //16384   LSB/g
#define	MPU9250_ACCE_FS_4G     0x08		  //8192    LSB/g
#define	MPU9250_ACCE_FS_8G     0x10		  //4096    LSB/g
#define	MPU9250_ACCE_FS_16G    0x18		  //2048    LSB/g
//****************************************
//****************陀螺仪量程**************
#define	MPU9250_GYRO_FS_250    0x00		  //131     LSB/(°/s)
#define	MPU9250_GYRO_FS_500    0x08		  //65.5    LSB/(°/s)
#define	MPU9250_GYRO_FS_1000   0x10		  //32.8    LSB/(°/s)
#define	MPU9250_GYRO_FS_2000   0x18		  //16.4    LSB/(°/s)
//****************************************

#define	GYRO_ADDRESS     (0xD0>>1)  //陀螺地址
#define MAG_ADDRESS       0x0C//0x18      //磁场地址
#define ACCEL_ADDRESS    (0xD0>>1) 
#define MPU9250_ADDRESS	 (0xD0>>1)
/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
#define AK8963_OFFSET_ADDRESS		 0x00
#define AK8963_I2C_ADDR             /*(0x18>>1)*/(0x0C)  //指南针设备地址
#define AK8963_WIA                  (AK8963_OFFSET_ADDRESS+0x00)  //指南针设备ID
#define AK8963_CONTROL				(AK8963_OFFSET_ADDRESS+0x0A)

/*共125字节,19个成员变量*/
typedef struct
{
	char Acc_CALIBRATE;			   /*上位机:01(ACC校准)*/
	char Gyro_CALIBRATE;		   /*上位机:02(GYRO校准)*/
	char Cali_3d;
	xyz_s16_t Acc_I16;			   /*加速度原始值*/
	xyz_s16_t Gyro_I16;			   /*陀螺仪原始值*/

	xyz_f_t Acc;				   /*加速度滤波后原始值*/
	xyz_f_t Gyro;				   /*陀螺仪滤波后原始值*/

	xyz_f_t Gyro_deg;			   /*陀螺仪滤波后实际值(°/S)*/

	xyz_f_t Acc_Offset;			   /*加速度偏移量，对应sensor_setup.Offset.Accel*/
	xyz_f_t Gyro_Offset;		   /*陀螺仪偏移量，对应sensor_setup.Offset.Gyro*/
	xyz_f_t Gyro_Auto_Offset;
	xyz_f_t vec_3d_cali;		   /*sensor_setup.Offset.vec_3d_cali*/
	float Acc_Temprea_Offset;	   /*加速度的温度偏移量，对应sensor_setup.Offset.Acc_Temperature*/
	float Gyro_Temprea_Offset;	   /*陀螺仪的温度偏移量，对应sensor_setup.Offset.Gyro_Temperature*/

	float Gyro_Temprea_Adjust;
	float ACC_Temprea_Adjust;

	s16   Tempreature;			   /*温度原始值*/
	float TEM_LPF;				   /*温度原始值低通滤波*/
	float Ftempreature;			   /*温度实际值：°C*/
	/***********************磁力计**********************************/
	xyz_s16_t  Mag_Adc;			//采样值
	xyz_f_t    Mag_Offset;		//偏移值 (sensor_setup.Offset.Mag)
	xyz_f_t    Mag_Gain;		//比例缩放	
	xyz_f_t    Mag_Val;			//纠正后的值
}MPU9250_STRUCT;

u8 Init_MPU9250(u16 lpf, u16 lpf_2);
void MPU9250_Read_ACCEL_GYRO(void);
void MPU9250_Read_MAG(void);
void MPU9250_Data_Prepare(float T);
void MPU9250_Data_Offset(void);
void Param_SaveAccelOffset(xyz_f_t offset);
void Param_SaveGyroOffset(xyz_f_t offset);
void ANO_AK8963_CalOffset_Mag(void);

//void READ_MPU9250_ACCEL(void);
//void READ_MPU9250_GYRO(void);
//void READ_MPU9250_MAG(void);

//extern short T_X, T_Y, T_Z, T_T;
//extern short T2_X, T2_Y, T2_Z;
//extern char  test;
extern u8    test2;
extern MPU9250_STRUCT mpu9250;
extern u8 Mag_CALIBRATED;
#endif

#endif

