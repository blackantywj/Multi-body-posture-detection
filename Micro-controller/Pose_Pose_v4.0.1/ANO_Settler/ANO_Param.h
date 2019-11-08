#ifndef __ANO_PARAM_H
#define __ANO_PARAM_H

#include "stm32f10x.h"
#include "ANO_Drv_Flash.h"
#include "ANO_PID.h"
#include "ANO_Data.h"

/////////////////////////////////////////////////
#define FIRST_INIT_FLAG 		0XAA


/*共12字节*/
typedef struct
{
	float x;
	float y;
	float z;
}xyz_f_t;

/*共6字节*/
typedef struct
{
	s16 x;
	s16 y;
	s16 z;

}xyz_s16_t;

/*共64字节*/
typedef union
{
	uint8_t raw_data[4/*64*/];
	struct
	{
		//xyz_f_t Accel;		 /*mpu6050.Acc_Offset*/
		//xyz_f_t Gyro;			 /*mpu6050.Gyro_Offset*/
		//xyz_f_t Mag;			 /*ak8975.Mag_Offset*/
		//xyz_f_t vec_3d_cali;	 /*mpu6050.vec_3d_cali*/
		uint32_t mpu_flag;
		//float Acc_Temperature;	 /*mpu6050.Acc_Temprea_Offset*/
		//float Gyro_Temperature;	 /*mpu6050.Gyro_Temprea_Offset*/
	}Offset;	/*共60字节*/
}sensor_setup_t; //__attribute__((packed))

typedef struct
{
	s32 kp;			 //比例系数
	s32 ki;			 //积分系数
	s32 kd;		 	 //微分系数

}PID_param_st_pk; 

struct _save_param_st_pk
{	
	u8  firstintiflag;
    u8  sensor_type;
	u16 hardware;
	u16 software;
	
	float gyr_temprea_offset;
	float acc_temprea_offset;
	
	PID_param_st_pk PID_ct4;
	PID_param_st_pk PID_ct3;
	
	PID_param_st_pk PID_rol;   //12字节，3个float
	PID_param_st_pk PID_pit;
	PID_param_st_pk PID_yaw;
	
	PID_param_st_pk PID_rol_s; //12字节，3个float
	PID_param_st_pk PID_pit_s;
	PID_param_st_pk PID_yaw_s;
	
	PID_param_st_pk PID_hs;
	
};
struct _save_param_senser 
{
	u8 firstintiflag;

	xyz_f_t acc_offset;//3个float 12字节
	xyz_f_t gyr_offset;
	xyz_f_t mag_offset;

	float gyr_temprea_offset;
	float acc_temprea_offset;

};
/////////////////////////////////////////////////
extern struct _save_param_st_pk ANO_Param;
extern sensor_setup_t sensor_setup;

void ANO_Param_Init(void);
void POSE_Param_Init(void);
void ANO_Param_Read(void);
void POSE_Param_Read(void);
void ANO_Param_Save(void);
void POSE_Param_Save(u8 _mode);
void PID_Save_Overtime(u16 ms,u16 dTms);//PID参数超时写入

#include "POSE_Drv_MPU9250.h"

#endif

