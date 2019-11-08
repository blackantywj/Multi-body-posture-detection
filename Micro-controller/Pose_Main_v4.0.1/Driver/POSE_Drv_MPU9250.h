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
#define CALIBRATING_MAG_CYCLES              2000  //У׼ʱ�����20s

// ����MPU9250�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	MPU9250_CONFIG	0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x04(20Hz)	 (CONFIG)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
//�¼�
#define ACCEL_CONFIG2   0X1D    //���ٶȼƵ�ͨ�˲��� 0x06 5hz
#define PWR_MGMT_1      0X6B    //��Դ����1 ����ֵΪ0x00
#define PWR_MGMT_2      0X6C    //��Դ����2 ����ֵΪ0X00

//#define WHO_AM_I        0X75    //����ID MPU9250Ĭ��IDΪ0X71
#define USER_CTRL       0X6A    //�û����õ�Ϊ0X10ʱʹ��SPIģʽ

#define MPU9250_CS      PDout(3)//MPU9250Ƭѡ�ź�
#define I2C_ADDR        0X68    //i2c�ĵ�ַ
//�¼ӽ���
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

#define MPU9250_INT_PIN_CFG         (0x37)   //INT�������ú�Bypassģʽ���üĴ���
#define MPU9250_INT_ENABLE          (0x38)
#define MPU9250_INT_STATUS          (0x3A)
#define	WHO_AM_I		0x75	             //IIC��ַ�Ĵ���(Ĭ����ֵ0x73��ֻ��)
//****************��������****************
#define	MPU9250_SMPLRT_DIV_125HZ      0x07
#define	MPU9250_SMPLRT_DIV_250HZ      0x03
#define	MPU9250_SMPLRT_DIV_333HZ      0x02
#define	MPU9250_SMPLRT_DIV_500HZ      0x01
#define	MPU9250_SMPLRT_DIV_1KHZ       0x00
//****************************************
//****************�˲�����****************
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
//****************���ٶ�����**************
#define	MPU9250_ACCE_FS_2G     0x00		  //16384   LSB/g
#define	MPU9250_ACCE_FS_4G     0x08		  //8192    LSB/g
#define	MPU9250_ACCE_FS_8G     0x10		  //4096    LSB/g
#define	MPU9250_ACCE_FS_16G    0x18		  //2048    LSB/g
//****************************************
//****************����������**************
#define	MPU9250_GYRO_FS_250    0x00		  //131     LSB/(��/s)
#define	MPU9250_GYRO_FS_500    0x08		  //65.5    LSB/(��/s)
#define	MPU9250_GYRO_FS_1000   0x10		  //32.8    LSB/(��/s)
#define	MPU9250_GYRO_FS_2000   0x18		  //16.4    LSB/(��/s)
//****************************************

#define	GYRO_ADDRESS     (0xD0>>1)  //���ݵ�ַ
#define MAG_ADDRESS       0x0C//0x18      //�ų���ַ
#define ACCEL_ADDRESS    (0xD0>>1) 
#define MPU9250_ADDRESS	 (0xD0>>1)
/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
#define AK8963_OFFSET_ADDRESS		 0x00
#define AK8963_I2C_ADDR             /*(0x18>>1)*/(0x0C)  //ָ�����豸��ַ
#define AK8963_WIA                  (AK8963_OFFSET_ADDRESS+0x00)  //ָ�����豸ID
#define AK8963_CONTROL				(AK8963_OFFSET_ADDRESS+0x0A)

/*��125�ֽ�,19����Ա����*/
typedef struct
{
	char Acc_CALIBRATE;			   /*��λ��:01(ACCУ׼)*/
	char Gyro_CALIBRATE;		   /*��λ��:02(GYROУ׼)*/
	char Cali_3d;
	xyz_s16_t Acc_I16;			   /*���ٶ�ԭʼֵ*/
	xyz_s16_t Gyro_I16;			   /*������ԭʼֵ*/

	xyz_f_t Acc;				   /*���ٶ��˲���ԭʼֵ*/
	xyz_f_t Gyro;				   /*�������˲���ԭʼֵ*/

	xyz_f_t Gyro_deg;			   /*�������˲���ʵ��ֵ(��/S)*/

	xyz_f_t Acc_Offset;			   /*���ٶ�ƫ��������Ӧsensor_setup.Offset.Accel*/
	xyz_f_t Gyro_Offset;		   /*������ƫ��������Ӧsensor_setup.Offset.Gyro*/
	xyz_f_t Gyro_Auto_Offset;
	xyz_f_t vec_3d_cali;		   /*sensor_setup.Offset.vec_3d_cali*/
	float Acc_Temprea_Offset;	   /*���ٶȵ��¶�ƫ��������Ӧsensor_setup.Offset.Acc_Temperature*/
	float Gyro_Temprea_Offset;	   /*�����ǵ��¶�ƫ��������Ӧsensor_setup.Offset.Gyro_Temperature*/

	float Gyro_Temprea_Adjust;
	float ACC_Temprea_Adjust;

	s16   Tempreature;			   /*�¶�ԭʼֵ*/
	float TEM_LPF;				   /*�¶�ԭʼֵ��ͨ�˲�*/
	float Ftempreature;			   /*�¶�ʵ��ֵ����C*/
	/***********************������**********************************/
	xyz_s16_t  Mag_Adc;			//����ֵ
	xyz_f_t    Mag_Offset;		//ƫ��ֵ (sensor_setup.Offset.Mag)
	xyz_f_t    Mag_Gain;		//��������	
	xyz_f_t    Mag_Val;			//�������ֵ
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

