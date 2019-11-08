#include "ANO_Param.h"

struct _save_param_senser POSE_Param;
sensor_setup_t sensor_setup;

/*-----------Data_Init-----------*/
void POSE_Param_Init(void)
{
	POSE_Param.firstintiflag = FIRST_INIT_FLAG;

	POSE_Param.acc_offset.x = 0;
	POSE_Param.acc_offset.y = 0;
	POSE_Param.acc_offset.z = 0;

	POSE_Param.gyr_offset.x = 0;
	POSE_Param.gyr_offset.y = 0;
	POSE_Param.gyr_offset.z = 0;

	POSE_Param.mag_offset.x = 0;
	POSE_Param.mag_offset.y = 0;
	POSE_Param.mag_offset.z = 0;

	POSE_Param.acc_temprea_offset = 0;
	POSE_Param.gyr_temprea_offset = 0;

	ANO_Flash_Write((u8 *)(&POSE_Param), sizeof(POSE_Param));
}
/*-----------Data_Read-----------*/
void POSE_Param_Read(void)
{
	ANO_Flash_Read((u8 *)(&POSE_Param), sizeof(POSE_Param));
	if (POSE_Param.firstintiflag != FIRST_INIT_FLAG)//板子从未初始化
	{
		POSE_Param_Init();
	}
	else  //读取偏移量
	{
		mpu9250.Acc_Offset.x = POSE_Param.acc_offset.x;
		mpu9250.Acc_Offset.y = POSE_Param.acc_offset.y;
		mpu9250.Acc_Offset.z = POSE_Param.acc_offset.z;
		mpu9250.Acc_Temprea_Offset = POSE_Param.acc_temprea_offset;

		mpu9250.Gyro_Offset.x = POSE_Param.gyr_offset.x;
		mpu9250.Gyro_Offset.y = POSE_Param.gyr_offset.y;
		mpu9250.Gyro_Offset.z = POSE_Param.gyr_offset.z;
		mpu9250.Gyro_Temprea_Offset = POSE_Param.gyr_temprea_offset;

		mpu9250.Mag_Offset.x = POSE_Param.mag_offset.x;
		mpu9250.Mag_Offset.y = POSE_Param.mag_offset.y;
		mpu9250.Mag_Offset.z = POSE_Param.mag_offset.z;
	}
}
/*-----------Data_Save-----------*/
void POSE_Param_Save(u8 _mode)
{
	switch (_mode)
	{
	case 1:  //Acc
	{
		POSE_Param.acc_offset.x = mpu9250.Acc_Offset.x;
		POSE_Param.acc_offset.y = mpu9250.Acc_Offset.y;
		POSE_Param.acc_offset.z = mpu9250.Acc_Offset.z;
		POSE_Param.acc_temprea_offset = mpu9250.Acc_Temprea_Offset;
	} break;
	case 2:  //Gryo
	{
		POSE_Param.gyr_offset.x = mpu9250.Gyro_Offset.x;
		POSE_Param.gyr_offset.y = mpu9250.Gyro_Offset.y;
		POSE_Param.gyr_offset.z = mpu9250.Gyro_Offset.z;
		POSE_Param.gyr_temprea_offset = mpu9250.Gyro_Temprea_Offset;
	}break;
	case 3:  //Mag
	{
		POSE_Param.mag_offset.x = mpu9250.Mag_Offset.x;
		POSE_Param.mag_offset.y = mpu9250.Mag_Offset.y;
		POSE_Param.mag_offset.z = mpu9250.Mag_Offset.z;
	} break;
	default:
		break;
	}
	ANO_Flash_Write((u8 *)(&POSE_Param), sizeof(POSE_Param));
}








