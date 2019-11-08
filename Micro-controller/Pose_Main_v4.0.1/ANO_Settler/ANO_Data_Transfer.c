#include "ANO_Data_Transfer.h"

#define  SIGN_CHANGE(X)     {if(X < 0)\
                            {\
                                _temp = (s16)(-100.0f * X);\
                                _sign = 1;\
                            }\
                            else\
                            {\
                                _temp = (s16)(100.0f * X);\
                                _sign = 0;\
                            }}\

dt_flag_t f;					    //需要发送数据的标志
static u8 data_to_send[50];			//发送数据缓存
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每6ms发送一次传感器数据至上位机，即在此函数内实现(cnt = 6/2 = 3)
//此函数应由用户每2ms调用一次
void ANO_DT_Data_Exchange(void) //2ms一次
{
	static u16 cnt           =  0;
	static u16 senser_cnt 	 = 10;	   //10*2 = 20ms
	static u16 senser2_cnt   = 50;
	static u16 user_cnt 	 = 250;	   //500ms
	static u16 status_cnt 	 = 15;	   //30ms
//	static u8 rcdata_cnt 	= 20;
//	static u8 motopwm_cnt	= 20;
//	static u8 power_cnt		= 50;
//	static u8 speed_cnt     = 50;
//	static u8 location_cnt  = 200;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.send_senser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))
		f.send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	//if((cnt % speed_cnt) == (speed_cnt-3))
	//	f.send_speed = 1;		

	
	if(++cnt>1000) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.msg_id)
	{
		//ANO_DT_Send_Msg(f.msg_id,f.msg_data);
		f.msg_id = 0;
	}
	if(f.send_status)
	{
		f.send_status = 0;
//		ANO_DT_Send_Status(  Roll,  Pitch, Yaw,      1,            0, 0/*fly_ready*/);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_speed)
	{
		f.send_speed = 0;
		//ANO_DT_Send_Speed(airframe_x_sp,airframe_y_sp,wz_speed);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_user)
	{
		f.send_user = 0;
		ANO_DT_Send_User();
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_senser)
	{
		f.send_senser = 0;
//		ANO_DT_Send_Senser(mpu9250.Acc.x, mpu9250.Acc.y, mpu9250.Acc.z,
//			               mpu9250.Gyro.x, mpu9250.Gyro.y, mpu9250.Gyro.z,
//			               mpu9250.Mag_Val.x, mpu9250.Mag_Val.y, mpu9250.Mag_Val.z);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_senser2)
	{
		f.send_senser2 = 0;
//		ANO_DT_Send_Senser2(baro.relative_height, Distance_data);    //mm
	}	
				
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
#ifdef POSE_DT_USE_USART1
	ANO_Uart1_Send(data_to_send, length);
#endif
}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用


void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	/////////////////////////////////校验//////////////////////////////////////
	for (u8 i = 0;i<(num - 1);i++)
		sum += *(data_buf + i);
	if (!(sum == *(data_buf + num - 1)))		            return;	    //判断sum
	if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF)) return;		//判断帧头

																		//////////////////////////////////CONMAND//////////////////////////////////
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
			mpu9250.Acc_CALIBRATE = 1;
			//sensor.acc_CALIBRATE = 1;
			//sensor.Cali_3d = 1;
		}
		else if(*(data_buf+4)==0X02)
		{
			mpu9250.Gyro_CALIBRATE = 1;
			//sensor.gyr_CALIBRATE = 1;
		}
		else if(*(data_buf+4)==0X03)
		{
			mpu9250.Acc_CALIBRATE = 1;						    /*ACC校准*/
			mpu9250.Gyro_CALIBRATE = 1;						    /*GYRO校准*/
			//sensor.acc_CALIBRATE = 1;		
			//sensor.gyr_CALIBRATE = 1;			
		}
		else if (*(data_buf + 4) == 0X04)							/*04：MAG校准*/
		{
			Mag_CALIBRATED = 1;
		}
		else if(*(data_buf+4)==0X31)
		{
			ANO_Uart1_DeInit();
			ANO_UART3_DeInit();
			Delay_us(100);
			ANO_Uart1_Init(576000);
			ANO_UART3_Init(576000);
            
            #ifdef ANO_DT_USE_WIFI
			Esp8266_Init();
			Esp8266_SetGpio0(0);
			Esp8266_SetEnable(0);
			Delay_us(100);
			Esp8266_SetEnable(1);
            #endif //ANO_DT_USE_WIFI
            
			flag.espDownloadMode = 1;
			//ANO_LED_0_ON();
			//ANO_LED_1_ON();
			POSE_LED_0_ON();
		}
		else if(*(data_buf+4)==0XA0)
		{
			//fly_ready = 0;			
		}
		else if(*(data_buf+4)==0XA1)
		{
			//fly_ready = 1;			
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{

		}
	}

	if(*(data_buf+2)==0X03)
	{
		flag.NS = 2;
		#if RC_Module
		RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
		RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
		RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
		RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
		RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
		RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
		RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
		RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
        #endif //RC_Module
	}

	if(*(data_buf+2)==0X10)								//PID1
    {

		
        ANO_DT_Send_Check(*(data_buf+2),sum);
        //Param_SavePID();
 
    }
    if(*(data_buf+2)==0X11)								//PID2
    {

		
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
//			
//        pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//			
//        pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
//		    pid_setup.groups.ctrl4.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        pid_setup.groups.ctrl4.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        pid_setup.groups.ctrl4.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			
//         pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//         pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//         pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
// 			
//         pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//         pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//         pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);		   //4
	data_to_send[_cnt++]=BYTE0(_temp);		   //5
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);		   //6
	data_to_send[_cnt++]=BYTE0(_temp);		   //7
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);		   //8
	data_to_send[_cnt++]=BYTE0(_temp);		   //9
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

#if PID_Module
extern PID_val_t val_1_rol;
#endif //PID_Module

void ANO_DT_Send_User()
{
	u8 _cnt=0;
    u8 _sign = 0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; //用户数据
	data_to_send[_cnt++]=0;
	
//	_temp = (s16)123;           //1
//	data_to_send[_cnt++] = BYTE1(_temp);
//	data_to_send[_cnt++] = BYTE0(_temp);

//	_temp = (s16)321;           //2
//	data_to_send[_cnt++] = BYTE1(_temp);
//	data_to_send[_cnt++] = BYTE0(_temp);

#if MPU9250
    
    SIGN_CHANGE(Roll);
    //_temp = (s16)Roll/*T_Z*/;           //3
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = _sign;

    SIGN_CHANGE(Pitch);
	//_temp = (s16)Pitch;           //4
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = _sign;
    
    SIGN_CHANGE(Yaw);
    //_temp = (s16)Yaw;           //5
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = _sign;
    
	SIGN_CHANGE(Roll_2);           //6
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = _sign;
    
	SIGN_CHANGE(Pitch_2);           //7
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = _sign;
    
	SIGN_CHANGE(Yaw_2);           //8
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = _sign;
    
	//SIGN_CHANGE(Distance_data);    //9
	#if VL15310x
    _temp = (s16)Distance_data;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = 0;
	#endif
	
	
#endif //MPU9250
	

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

