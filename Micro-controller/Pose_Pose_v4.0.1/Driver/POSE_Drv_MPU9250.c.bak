#include "POSE_Drv_MPU9250.h"
#include "i2c_soft.h"

#if MPU9250

MPU9250_STRUCT mpu9250;

//unsigned char TX_DATA[4];  	 //显示据缓存区
//unsigned char BUF[10];       //接收数据缓存区
u8 mpu9250_buffer[20];
//char  test=0; 				 //IIC用到
u8    test2 = 0;
//short T_X,T_Y,T_Z,T_T;		 //X,Y,Z轴，温度
//short T2_X,T2_Y,T2_Z;		 //X,Y,Z轴

/*初始化MPU9250，根据需要请参考pdf进行修改************************
*  返回：
*  1：初始化失败
*  0：初始化成功
*/
u8 Init_MPU9250(u16 lpf, u16 lpf_2)
{
	u8  value = 0;
	u8 default_filter = 1, default_filter_2 = 1;

	I2C_FastMode = 1;
	switch (lpf)
	{
	case 5:
		default_filter = MPU9250_GYRO_DLPF_BW_5HZ;
		break;
	case 10:
		default_filter = MPU9250_GYRO_DLPF_BW_10HZ;
		break;
	case 20:
		default_filter = MPU9250_GYRO_DLPF_BW_20HZ;
		break;
	case 41:
		default_filter = MPU9250_GYRO_DLPF_BW_41HZ;
		break;
	case 92:
		default_filter = MPU9250_GYRO_DLPF_BW_92HZ;
		break;
	case 184:
		default_filter = MPU9250_GYRO_DLPF_BW_184HZ;
		break;
	case 250:
		default_filter = MPU9250_GYRO_DLPF_BW_250HZ;
		break;
	default:
		default_filter = MPU9250_GYRO_DLPF_BW_DISABLE;
		break;
	}
	switch (lpf_2)
	{
	case 5:
		default_filter_2 = MPU9250_ACCE_DLPF_BW_5HZ;
		break;
	case 10:
		default_filter_2 = MPU9250_ACCE_DLPF_BW_10HZ;
		break;
	case 20:
		default_filter_2 = MPU9250_ACCE_DLPF_BW_20HZ;
		break;
	case 41:
		default_filter_2 = MPU9250_ACCE_DLPF_BW_41HZ;
		break;
	case 92:
		default_filter_2 = MPU9250_ACCE_DLPF_BW_92HZ;
		break;
	case 184:
		default_filter_2 = MPU9250_ACCE_DLPF_BW_184HZ;
		break;
	case 460:
		default_filter_2 = MPU9250_ACCE_DLPF_BW_460HZ;
		break;
	default:
		default_filter_2 = MPU9250_ACCE_DLPF_BW_DISABLE;
		break;
	}

	/*软件IIC初始化*/
	I2c_Soft_Init();
	Delay_ms(10);

	IIC_Write_1Byte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);	//电源管理,复位MPU9250,内部晶振20MHz
	Delay_ms(10);
	//IIC_Read_1Byte(MPU9250_ADDRESS, PWR_MGMT_1, &test2);
	IIC_Write_1Byte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);	//解除休眠状态
	Delay_ms(10);
	//IIC_Read_1Byte(MPU9250_ADDRESS, PWR_MGMT_1, &test2);
	IIC_Read_1Byte(MPU9250_ADDRESS, WHO_AM_I, &value);
/*
   Single_Write(GYRO_ADDRESS,PWR_M, 0x80);   //
   Single_Write(GYRO_ADDRESS,SMPL, 0x07);    //
   Single_Write(GYRO_ADDRESS,DLPF, 0x1E);    //±2000°
   Single_Write(GYRO_ADDRESS,INT_C, 0x00 );  //
   Single_Write(GYRO_ADDRESS,PWR_M, 0x00);   //
*/
	if (value == 0x73) /*设备ID：MPU9250:0x71  MPU9255:0x73*/
	{
		IIC_Write_1Byte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);	                          //打开三轴加速度、三轴陀螺仪
		Delay_ms(10);
		IIC_Write_1Byte(MPU9250_ADDRESS, SMPLRT_DIV, MPU9250_SMPLRT_DIV_333HZ);	      //陀螺仪采样率1000/(1+7)=125HZ (SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV))
		Delay_ms(10);
		IIC_Write_1Byte(GYRO_ADDRESS, MPU9250_CONFIG, default_filter);	              //陀螺仪低通滤波器 
		Delay_ms(10);
		IIC_Write_1Byte(ACCEL_ADDRESS, ACCEL_CONFIG, MPU9250_ACCE_FS_8G);	          //加速度度最大量程 +-8G	    4096    LSB/g
		Delay_ms(10);
		IIC_Write_1Byte(ACCEL_ADDRESS, ACCEL_CONFIG2, default_filter_2);              //加速度低通滤波器
		Delay_ms(10);
		IIC_Write_1Byte(GYRO_ADDRESS, GYRO_CONFIG, MPU9250_GYRO_FS_2000);	          //陀螺仪最大量程 +-2000度每秒	16.4    LSB/(°/s)
		Delay_ms(10);
		IIC_Write_1Byte(MPU9250_ADDRESS, USER_CTRL, 0x00);							  //初始化I2C
		Delay_ms(10);
		IIC_Write_1Byte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x00);					  //禁止中断
		Delay_ms(10);
		IIC_Write_1Byte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x02);				  //进入Bypass模式，用于控制电子指南针
		Delay_ms(10);

		I2C_FastMode = 0;
		IIC_Read_1Byte(AK8963_I2C_ADDR, AK8963_WIA, &value);
		if (value == 0x48)
		{

		}
		//IIC_Read_1Byte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, &test2);
		//IIC_Read_1Byte(MPU9250_ADDRESS, GYRO_CONFIG, &test2);
		//IIC_Read_1Byte(MPU9250_ADDRESS, ACCEL_CONFIG, &test2);
		//IIC_Read_1Byte(MPU9250_ADDRESS, MPU9250_INT_STATUS, &test2);																									   

		//----------------
		//Single_Write(GYRO_ADDRESS,0x6A,0x00);//close Master Mode	
		return 0;
	}
	return 1;
}
s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
u8  filter_cnt = 0, filter_cnt_old = 0;

//******读取MPU9250数据****************************************
//给三个变量赋值
void Transform(float itx, float ity, float itz, float *it_x, float *it_y, float *it_z)
{
	*it_x = itx;
	*it_y = ity;
	*it_z = itz;

}
/*读取加速度、温度、陀螺仪原始数据*/
void MPU9250_Read_ACCEL_GYRO(void)
{
	I2C_FastMode = 1;
	IIC_Read_nByte(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 14, mpu9250_buffer);	
}
/*读取磁力计原始数据*/
void MPU9250_Read_MAG(void)
{
	int16_t mag_temp[3];

	I2C_FastMode = 0;
	IIC_Read_nByte(AK8963_I2C_ADDR, MAG_XOUT_L, 6, mpu9250_buffer + 14);

	mag_temp[1] = ((((int16_t)mpu9250_buffer[15]) << 8) | mpu9250_buffer[14]);  //磁力计X轴
	mag_temp[0] = ((((int16_t)mpu9250_buffer[17]) << 8) | mpu9250_buffer[16]);  //磁力计Y轴
	mag_temp[2] = -((((int16_t)mpu9250_buffer[19]) << 8) | mpu9250_buffer[18]);  //磁力计Z轴

	mpu9250.Mag_Adc.x = mag_temp[0];
	mpu9250.Mag_Adc.y = mag_temp[1];
	mpu9250.Mag_Adc.z = mag_temp[2];

	mpu9250.Mag_Val.x = (mpu9250.Mag_Adc.x - mpu9250.Mag_Offset.x);
	mpu9250.Mag_Val.y = (mpu9250.Mag_Adc.y - mpu9250.Mag_Offset.y);
	mpu9250.Mag_Val.z = (mpu9250.Mag_Adc.z - mpu9250.Mag_Offset.z);
	//磁力计中点矫正
	ANO_AK8963_CalOffset_Mag();
	//AK8963采样触发
	IIC_Write_1Byte(AK8963_I2C_ADDR, AK8963_CONTROL, 0x11);
}
float mpu9250_tmp[ITEMS];
float mpu_fil_tmp[ITEMS];
/*------------------------------------------------------
加速度方向：(从芯片正面向下看,芯片左上为黑点)
			x		| z
			<-------+---	(总感觉是这样的)
					|
					|
				   \/ y

陀螺仪方向：(从芯片正面向下看,芯片左上为黑点)
				   个 y
				    |
				    |		x
			  ------+------->	  (旋转方向为右手坐标系)
			     z  |
				    |
磁力计方向：(从芯片正面向下看,芯片左上为黑点)
				   个 y
				    |
				    |		x
			  ------+------->	 （感觉） 
			     z  |
				    |
--------------------------------------------------------*/
void MPU9250_Data_Prepare(float T)	 /*参数T:内环准确的执行周期,单位：s(大约2ms)*/
{
	u8 i;
	s32 FILT_TMP[ITEMS] = { 0,0,0,0,0,0,0 };
	//	float auto_offset_temp[3];
	float Gyro_tmp[3];


	MPU9250_Data_Offset(); //校准函数(上位机指令允许才真正进行校准)

						   /*读取buffer原始数据*/
	mpu9250.Acc_I16.x = ((((int16_t)mpu9250_buffer[0]) << 8) | mpu9250_buffer[1]);   /*加速度:最大量程 +-8G		  (65536/16=4096 LSB/g)(N/kg)*/
	mpu9250.Acc_I16.y = ((((int16_t)mpu9250_buffer[2]) << 8) | mpu9250_buffer[3]);
	mpu9250.Acc_I16.z = ((((int16_t)mpu9250_buffer[4]) << 8) | mpu9250_buffer[5]);

	mpu9250.Gyro_I16.x = ((((int16_t)mpu9250_buffer[8]) << 8) | mpu9250_buffer[9]);  /*陀螺仪:最大量程 +-2000度每秒  (65536/4000=16.4LSB/(°/S))*/
	mpu9250.Gyro_I16.y = ((((int16_t)mpu9250_buffer[10]) << 8) | mpu9250_buffer[11]);
	mpu9250.Gyro_I16.z = ((((int16_t)mpu9250_buffer[12]) << 8) | mpu9250_buffer[13]);

	Gyro_tmp[0] = mpu9250.Gyro_I16.x;//
	Gyro_tmp[1] = mpu9250.Gyro_I16.y;//
	Gyro_tmp[2] = mpu9250.Gyro_I16.z;//

	mpu9250.Tempreature = ((((int16_t)mpu9250_buffer[6]) << 8) | mpu9250_buffer[7]);   //tempreature(温度)
	mpu9250.TEM_LPF += 2 * 3.14f *T *(mpu9250.Tempreature - mpu9250.TEM_LPF);		   /*温度低通滤波*/
	mpu9250.Ftempreature = mpu9250.TEM_LPF / 333.87f + 21.0f;						   /*温度换算公式:Temperature  = 21 + regval/333.87;*/

/////////////////////////////////////开始滤波(加速度、陀螺仪)///////////////////////////////////
/*======================================================================
(0) (1) (2) (3) (4) (5) (6) (7) (8) (9) (10)
(11)
filter_cnt    :	 1   2   3   4   5   6   7   8   9   10   0	  //实际值

filter_cnt_old:  2   3   4   5   6   7   8   9   10   0	  1	  //实际值
======================================================================*/
	if (++filter_cnt > FILTER_NUM)
	{
		filter_cnt = 0;
		filter_cnt_old = 1;
	}
	else
	{
		filter_cnt_old = (filter_cnt == FILTER_NUM) ? 0 : (filter_cnt + 1);
	}
	//10 170 4056
	/* 得出校准后的数据 */
	if (sensor_setup.Offset.mpu_flag == 0)  /*与MPU9250_Data_Offset()有关*/
	{
		mpu9250_tmp[A_X] = (mpu9250.Acc_I16.x - mpu9250.Acc_Offset.x);
		mpu9250_tmp[A_Y] = (mpu9250.Acc_I16.y - mpu9250.Acc_Offset.y);
		mpu9250_tmp[A_Z] = (mpu9250.Acc_I16.z - mpu9250.Acc_Offset.z);
	}
	else
	{
		mpu9250_tmp[A_X] = 2 * (mpu9250.Acc_I16.x - mpu9250.Acc_Offset.x);
		mpu9250_tmp[A_Y] = 2 * (mpu9250.Acc_I16.y - mpu9250.Acc_Offset.y);
		mpu9250_tmp[A_Z] = 2 * (mpu9250.Acc_I16.z - mpu9250.Acc_Offset.z - 2048);
	}

	mpu9250_tmp[G_X] = Gyro_tmp[0] - mpu9250.Gyro_Offset.x;//mpu9250.Gyro_I16.x - 	mpu9250.Gyro_Offset.x
	mpu9250_tmp[G_Y] = Gyro_tmp[1] - mpu9250.Gyro_Offset.y;//mpu9250.Gyro_I16.y - 	mpu9250.Gyro_Offset.y
	mpu9250_tmp[G_Z] = Gyro_tmp[2] - mpu9250.Gyro_Offset.z;//mpu9250.Gyro_I16.z - 	mpu9250.Gyro_Offset.z


														   /* 更新滤波滑动窗口数组 ,一次1组，循环记录11组*/
	FILT_BUF[A_X][filter_cnt] = mpu9250_tmp[A_X];           /*有两种*/
	FILT_BUF[A_Y][filter_cnt] = mpu9250_tmp[A_Y];           /*有两种*/
	FILT_BUF[A_Z][filter_cnt] = mpu9250_tmp[A_Z];           /*有两种*/
	FILT_BUF[G_X][filter_cnt] = mpu9250_tmp[G_X];           /*mpu9250.Gyro_I16.x - 	mpu9250.Gyro_Offset.x*/
	FILT_BUF[G_Y][filter_cnt] = mpu9250_tmp[G_Y];           /*mpu9250.Gyro_I16.y - 	mpu9250.Gyro_Offset.y*/
	FILT_BUF[G_Z][filter_cnt] = mpu9250_tmp[G_Z];           /*mpu9250.Gyro_I16.z - 	mpu9250.Gyro_Offset.z*/

	for (i = 0; i<FILTER_NUM; i++)								/*一次执行11次*/
	{
		FILT_TMP[A_X] += FILT_BUF[A_X][i];
		FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
		FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
		FILT_TMP[G_X] += FILT_BUF[G_X][i];
		FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
		FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
	}

	/*求滤波滑动窗口数组(加速度)的平均值*/
	mpu_fil_tmp[A_X] = (float)(FILT_TMP[A_X]) / (float)FILTER_NUM;
	mpu_fil_tmp[A_Y] = (float)(FILT_TMP[A_Y]) / (float)FILTER_NUM;
	mpu_fil_tmp[A_Z] = (float)(FILT_TMP[A_Z]) / (float)FILTER_NUM;

	/*求滤波滑动窗口数组(陀螺仪)的平均值*/
	mpu_fil_tmp[G_X] = (float)(FILT_TMP[G_X]) / (float)FILTER_NUM;
	mpu_fil_tmp[G_Y] = (float)(FILT_TMP[G_Y]) / (float)FILTER_NUM;
	mpu_fil_tmp[G_Z] = (float)(FILT_TMP[G_Z]) / (float)FILTER_NUM;


	/*坐标转换,将滤波后的值(加速度、陀螺仪)传递到mpu9250结构体中去*/
	Transform(mpu_fil_tmp[A_X], mpu_fil_tmp[A_Y], mpu_fil_tmp[A_Z], &mpu9250.Acc.x, &mpu9250.Acc.y, &mpu9250.Acc.z);
	Transform(mpu_fil_tmp[G_X], mpu_fil_tmp[G_Y], mpu_fil_tmp[G_Z], &mpu9250.Gyro.x, &mpu9250.Gyro.y, &mpu9250.Gyro.z);

	/*得到陀螺仪滤波后实际值(°/S)*/
	mpu9250.Gyro_deg.x = mpu9250.Gyro.x *TO_ANGLE;
	mpu9250.Gyro_deg.y = mpu9250.Gyro.y *TO_ANGLE;
	mpu9250.Gyro_deg.z = mpu9250.Gyro.z *TO_ANGLE;



	//======================================================================
}
s32 sum_temp[7] = { 0,0,0,0,0,0,0 };			 /*临时数据*/
u16 acc_sum_cnt = 0, gyro_sum_cnt = 0;		     /*计数据用*/

void MPU9250_Data_Offset(void)
{
#ifdef ACC_ADJ_EN							 /*是否使能了加速度校准*/

	if (mpu9250.Acc_CALIBRATE == 1)			 /*上位机选择是否校准加速度*/
	{
		if (my_sqrt(my_pow(mpu9250.Acc_I16.x) + my_pow(mpu9250.Acc_I16.y) + my_pow(mpu9250.Acc_I16.z)) < 2500)
		{
			sensor_setup.Offset.mpu_flag = 1; /*在MPU9250_Data_Prepare(float T)用到*/
		}
		else if (my_sqrt(my_pow(mpu9250.Acc_I16.x) + my_pow(mpu9250.Acc_I16.y) + my_pow(mpu9250.Acc_I16.z)) > 2600)
		{
			sensor_setup.Offset.mpu_flag = 0; /*在MPU9250_Data_Prepare(float T)用到*/
		}

		acc_sum_cnt++;                        /*记录收集数据组个数*/
		sum_temp[A_X] += mpu9250.Acc_I16.x;
		sum_temp[A_Y] += mpu9250.Acc_I16.y;
		sum_temp[A_Z] += mpu9250.Acc_I16.z - 65536 / 16;   // +-8G	(65536 / 16 = 4096)
		sum_temp[TEM] += mpu9250.Tempreature;

		if (acc_sum_cnt >= OFFSET_AV_NUM)	 /*收集了50组加速度和温度的数据后计算偏移量*/
		{
			mpu9250.Acc_Offset.x = sum_temp[A_X] / OFFSET_AV_NUM;
			mpu9250.Acc_Offset.y = sum_temp[A_Y] / OFFSET_AV_NUM;
			mpu9250.Acc_Offset.z = sum_temp[A_Z] / OFFSET_AV_NUM;
			mpu9250.Acc_Temprea_Offset = sum_temp[TEM] / OFFSET_AV_NUM;
			acc_sum_cnt = 0;					/*清零计数*/
			mpu9250.Acc_CALIBRATE = 0;		/*清零，校准完毕*/
			f.msg_id = 1;
			f.msg_data = 1;
			//Param_SaveAccelOffset(&mpu9250.Acc_Offset);	                          /*保存偏移量*/
			//Param_SaveAccelOffset(mpu9250.Acc_Offset);
			POSE_Param_Save(1);

			sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;	  /*清零临时数据*/
		}
	}

#endif

	if (mpu9250.Gyro_CALIBRATE)			 /*上位机选择是否校准陀螺仪( = 1/2)*/
	{
		gyro_sum_cnt++;                  /*记录收集数据组个数*/
		sum_temp[G_X] += mpu9250.Gyro_I16.x;
		sum_temp[G_Y] += mpu9250.Gyro_I16.y;
		sum_temp[G_Z] += mpu9250.Gyro_I16.z;
		sum_temp[TEM] += mpu9250.Tempreature;

		if (gyro_sum_cnt >= OFFSET_AV_NUM)	 /*收集了50组加速度和温度的数据后计算偏移量*/
		{
			mpu9250.Gyro_Offset.x = (float)sum_temp[G_X] / OFFSET_AV_NUM;
			mpu9250.Gyro_Offset.y = (float)sum_temp[G_Y] / OFFSET_AV_NUM;
			mpu9250.Gyro_Offset.z = (float)sum_temp[G_Z] / OFFSET_AV_NUM;
			mpu9250.Gyro_Temprea_Offset = sum_temp[TEM] / OFFSET_AV_NUM;
			gyro_sum_cnt = 0;				 /*清零计数*/
			if (mpu9250.Gyro_CALIBRATE == 1)
			{
				//Param_SaveGyroOffset(&mpu9250.Gyro_Offset);	                   /*保存偏移量*/
				//Param_SaveGyroOffset(mpu9250.Gyro_Offset);	                   /*保存偏移量*/
				POSE_Param_Save(2);
				f.msg_id = 2;
				f.msg_data = 1;
			}
			mpu9250.Gyro_CALIBRATE = 0;			                               /*清零，校准完毕*/
			sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0; /*清零临时数据*/
		}
	}
}
void Param_SaveAccelOffset(xyz_f_t offset)
{
	u8 buffer[6];
	s16 tmp_buf[3];
	tmp_buf[0] = ((s16)(-offset.x));	  /*结果似乎不对*/
	tmp_buf[1] = ((s16)(-offset.y));
	tmp_buf[2] = ((s16)(-offset.z));
	tmp_buf[0] <<= 1;
	tmp_buf[1] <<= 1;
	tmp_buf[2] <<= 1;
	buffer[0] = BYTE1(tmp_buf[0]);
	buffer[1] = BYTE0(tmp_buf[0]);
	buffer[2] = BYTE1(tmp_buf[1]);
	buffer[3] = BYTE0(tmp_buf[1]);
	buffer[4] = BYTE1(tmp_buf[2]);
	buffer[5] = BYTE0(tmp_buf[2]);
	IIC_Write_nByte(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 6, buffer);
	//memcpy(&mpu6050.Acc_Offset, offset, sizeof(xyz_f_t));
	//memcpy(&sensor_setup.Offset.Accel, offset, sizeof(xyz_f_t));

	//sensor_setup.Offset.Acc_Temperature = mpu6050.Acc_Temprea_Offset;

	//Para_WriteSettingToFile();
}
void Param_SaveGyroOffset(xyz_f_t offset)
{
	u8 buffer[6];
	s16 tmp_buf[3];
	offset.x = -offset.x * 2 /*  * 8 / 4*/;
	offset.y = -offset.y * 2 /*  * 8 / 4*/;
	offset.z = -offset.z * 2 /*  * 8 / 4*/;
	tmp_buf[0] = (s16)(offset.x);
	tmp_buf[1] = (s16)(offset.y);
	tmp_buf[2] = (s16)(offset.z);
	buffer[0] = BYTE1(tmp_buf[0]);
	buffer[1] = BYTE0(tmp_buf[0]);
	buffer[2] = BYTE1(tmp_buf[1]);
	buffer[3] = BYTE0(tmp_buf[1]);
	buffer[4] = BYTE1(tmp_buf[2]);
	buffer[5] = BYTE0(tmp_buf[2]);
	IIC_Write_nByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, 6, buffer);
	//memcpy(&mpu6050.Gyro_Offset, offset, sizeof(xyz_f_t));
	//memcpy(&sensor_setup.Offset.Gyro, offset, sizeof(xyz_f_t));

	//sensor_setup.Offset.Gyro_Temperature = mpu6050.Gyro_Temprea_Offset;

	//Para_WriteSettingToFile();
}

u8 Mag_CALIBRATED = 0;			 /*上位机:04(MAG校准) 或者 左边摇杆左下，右边摇杆右上保持2s以上(RC_Duty)*/
//磁力计中点矫正
void ANO_AK8963_CalOffset_Mag(void)
{
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m = 0;

	if (Mag_CALIBRATED)
	{

		if (ABS(mpu9250.Mag_Adc.x)<400 && ABS(mpu9250.Mag_Adc.y)<400 && ABS(mpu9250.Mag_Adc.z)<400)
		{
			//平面校准方法教程:http://www.dzsc.com/data/html/2010-11-29/87454.html
			//得到磁力三轴最大值
			MagMAX.x = _MAX(mpu9250.Mag_Adc.x, MagMAX.x);
			MagMAX.y = _MAX(mpu9250.Mag_Adc.y, MagMAX.y);
			MagMAX.z = _MAX(mpu9250.Mag_Adc.z, MagMAX.z);
			//得到磁力三轴最小值
			MagMIN.x = _MIN(mpu9250.Mag_Adc.x, MagMIN.x);
			MagMIN.y = _MIN(mpu9250.Mag_Adc.y, MagMIN.y);
			MagMIN.z = _MIN(mpu9250.Mag_Adc.z, MagMIN.z);

			if (cnt_m == CALIBRATING_MAG_CYCLES)											  //计时:2000*10ms = 20s
			{
				//计算磁场干扰矢量γ(γx, γy, γz)
				mpu9250.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				mpu9250.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				mpu9250.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);

				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;

				mpu9250.Mag_Gain.y = MagSum.x / MagSum.y;
				mpu9250.Mag_Gain.z = MagSum.x / MagSum.z;

				//Param_SaveMagOffset(&mpu9250.Mag_Offset);//param_Save();//保存数据
				POSE_Param_Save(3);

				cnt_m = 0;
				Mag_CALIBRATED = 0;
				f.msg_id = 3;
				f.msg_data = 1;
			}
		}
		cnt_m++;

	}
	else
	{

	}
}
//void READ_MPU9250_ACCEL(void)
//{ 
//
//   //BUF[0]=
//   IIC_Read_1Byte(ACCEL_ADDRESS, MPU9250_ACCEL_XOUT_L,&(BUF[0]));
//   //BUF[1]=
//   IIC_Read_1Byte(ACCEL_ADDRESS, MPU9250_ACCEL_XOUT_H,&(BUF[1]));
//    
//   T_X=	(BUF[1]<<8)|BUF[0];
//   //T_X/=16; 						   //读取计算X轴数据
//
//   //BUF[2]=
//   IIC_Read_1Byte(ACCEL_ADDRESS, MPU9250_ACCEL_YOUT_L,&(BUF[2]));
//   //BUF[3]=
//   IIC_Read_1Byte(ACCEL_ADDRESS, MPU9250_ACCEL_YOUT_H,&(BUF[3]));
//   T_Y=	(BUF[3]<<8)|BUF[2];
//   //T_Y/=16; 						   //读取计算Y轴数据
//   //BUF[4]=
//   IIC_Read_1Byte(ACCEL_ADDRESS, MPU9250_ACCEL_ZOUT_L,&(BUF[4]));
//   //BUF[5]=
//   IIC_Read_1Byte(ACCEL_ADDRESS, MPU9250_ACCEL_ZOUT_H,&(BUF[5]));
//   T_Z=	(BUF[5]<<8)|BUF[4];
//   //T_Z/=16; 					       //读取计算Z轴数据
// 
//}

//void READ_MPU9250_GYRO(void)
//{ 
//
//   //BUF[0]=Single_Read(GYRO_ADDRESS,GYRO_XOUT_L); 
//   //BUF[1]=Single_Read(GYRO_ADDRESS,GYRO_XOUT_H);
//   //BUF[0]=
//   IIC_Read_1Byte(GYRO_ADDRESS, MPU9250_GYRO_XOUT_L,&(BUF[0]));
//   //BUF[1]=
//   IIC_Read_1Byte(GYRO_ADDRESS, MPU9250_GYRO_XOUT_H,&(BUF[1]));
//    
//   T2_X=	(BUF[1]<<8)|BUF[0];
//   //T2_X/=16.4; 						   //读取计算X轴数据
//
//   //BUF[2]=Single_Read(GYRO_ADDRESS,GYRO_YOUT_L);
//   //BUF[3]=Single_Read(GYRO_ADDRESS,GYRO_YOUT_H);
//   //BUF[2]=
//   IIC_Read_1Byte(GYRO_ADDRESS, MPU9250_GYRO_YOUT_L,&(BUF[2]));
//   //BUF[3]=
//   IIC_Read_1Byte(GYRO_ADDRESS, MPU9250_GYRO_YOUT_H,&(BUF[3]));
//    
//   T2_Y=	(BUF[3]<<8)|BUF[2];
//   //T2_Y/=16.4; 						   //读取计算Y轴数据
//   //BUF[4]=Single_Read(GYRO_ADDRESS,GYRO_ZOUT_L);
//   //BUF[5]=Single_Read(GYRO_ADDRESS,GYRO_ZOUT_H);
//   //BUF[4]=
//   IIC_Read_1Byte(GYRO_ADDRESS, MPU9250_GYRO_ZOUT_L,&(BUF[4]));
//   //BUF[5]=
//   IIC_Read_1Byte(GYRO_ADDRESS, MPU9250_GYRO_ZOUT_H,&(BUF[5]));
//   
//   T2_Z=	(BUF[5]<<8)|BUF[4];
//   //T2_Z/=16.4; 					       //读取计算Z轴数据
// 
// 
//  // BUF[6]=Single_Read(GYRO_ADDRESS,TEMP_OUT_L); 
//  // BUF[7]=Single_Read(GYRO_ADDRESS,TEMP_OUT_H); 
//  // T_T=(BUF[7]<<8)|BUF[6];
//  // T_T = 35+ ((double) (T_T + 13200)) / 280;// 读取计算出温度
//}

//void READ_MPU9250_MAG(void)
//{ 
//   I2C_FastMode = 0;
//
//   IIC_Write_1Byte(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
//   //Delay_ms(10);
//   
//   //Delay_ms(10);
//   //BUF[0]=Single_Read (MAG_ADDRESS,MAG_XOUT_L);
//   //BUF[1]=Single_Read (MAG_ADDRESS,MAG_XOUT_H);
//   //BUF[0]=
//   IIC_Read_1Byte(MAG_ADDRESS,MAG_XOUT_L,&(BUF[0])); 
//   //BUF[1]=
//   IIC_Read_1Byte(MAG_ADDRESS,MAG_XOUT_H,&(BUF[1]));
//    
//   T_X=(BUF[1]<<8)|BUF[0];
//
//   //BUF[2]=Single_Read(MAG_ADDRESS,MAG_YOUT_L);
//   //BUF[3]=Single_Read(MAG_ADDRESS,MAG_YOUT_H);
//   //BUF[2]=
//   IIC_Read_1Byte(MAG_ADDRESS,MAG_YOUT_L,&(BUF[2]));
//   //BUF[3]=
//   IIC_Read_1Byte(MAG_ADDRESS,MAG_YOUT_H,&(BUF[3]));
//    
//   T_Y=	(BUF[3]<<8)|BUF[2];
//   						   //读取计算Y轴数据
//	 
//   //BUF[4]=Single_Read(MAG_ADDRESS,MAG_ZOUT_L);
//   //BUF[5]=Single_Read(MAG_ADDRESS,MAG_ZOUT_H);
//   //BUF[4]=
//   IIC_Read_1Byte(MAG_ADDRESS,MAG_ZOUT_L,&(BUF[4]));
//   //BUF[5]=
//   IIC_Read_1Byte(MAG_ADDRESS,MAG_ZOUT_H,&(BUF[5]));
//   
//   T_Z=	(BUF[5]<<8)|BUF[4];
// 					       //读取计算Z轴数据
//   IIC_Write_1Byte(MAG_ADDRESS, 0x0A, 0x01);
//}

#endif
