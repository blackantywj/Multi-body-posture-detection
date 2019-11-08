#include "DL_LN3X.h"

#if DL_LN3X

#define CHECK_FF_FE(x)		 {if(x == 0xFF)\
							{data_to_send[_cnt++] = 0xFE;data_to_send[_cnt++] = 0xFD;_check_cnt ++;}\
                          else if(x == 0xFE)\
							{data_to_send[_cnt++] = 0xFE;data_to_send[_cnt++] = 0xFC;_check_cnt ++;}\
                          else\
							{data_to_send[_cnt++] = x;}\
                         }

#define CHECK_LENGTH(x)	 {if((x) > 65)\
							{x = 65;}\
                         }

float Roll_2, Pitch_2, Yaw_2;    				//姿态角

static u8 data_to_send[50];			//发送数据缓存
DL_LN3X_STRUCT dl_ln3x;
dt_flag_t f2;					    //需要发送数据的标志

/*本模块初始化函数
* 无输入
* 无输出
*/
void DL_LN3X_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	ANO_UART3_Init(115200);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; //PB1设置成推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIOB->BSRR = GPIO_Pin_1;	  //接地

	DL_DT_Send_ReadCommand(RED_PORT, SELF_MODULE, 50);									 /*点亮红灯5s*/
	//Delay_ms(10);
	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_ADDRESS, DL_ADDRESS);		 /*设置地址*/
	//Delay_ms(10);
	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_INTERNET_ID, DL_INTERNET_ID); /*设置网络ID*/
	//Delay_ms(10);
	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_CHANNEL, DL_CHANNEL);		 /*设置信道*/
	//Delay_ms(10);
	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_BAUDRATE, DL_BAUDRATE);	     /*设置波特率*/
	//Delay_ms(10);
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, SET_REBOOT);						 /*重启命令*/
	Delay_ms(2000);																		 /*重启要多花时间*/
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_ADDRESS);				 /*读取地址*/
	//Delay_ms(10);
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_INTERNET_ID);			 /*读取网络ID*/
	//Delay_ms(10);
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_CHANNEL);				 /*读取信道编号*/
	//Delay_ms(10);
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_UART);					 /*读取Uart的波特率*/
	//Delay_ms(10);
}

/* 本模块检查函数
*  无输入
*  无输出
*  备注：最多需2s执行时间
*/
void DL_LN3X_Check(void)
{
	u8 check_flag = 0;
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_ADDRESS);				 /*读取地址*/
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_INTERNET_ID);			 /*读取网络ID*/
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_CHANNEL);				 /*读取信道编号*/
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_UART);					 /*读取Uart的波特率*/

	Delay_ms(10);
	if (dl_ln3x.Channel != DL_CHANNEL)
	{
		DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_CHANNEL, DL_CHANNEL);		 /*设置信道*/
		check_flag = 1;
	}
	if (dl_ln3x.ID != DL_INTERNET_ID)
	{
		DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_INTERNET_ID, DL_INTERNET_ID); /*设置网络ID*/
		check_flag = 1;
	}
	if (dl_ln3x.Address != DL_ADDRESS)
	{
		DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_ADDRESS, DL_ADDRESS);		 /*设置地址*/
		check_flag = 1;
	}
	if (dl_ln3x.BaudRate != DL_BAUDRATE)
	{
		DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_BAUDRATE, DL_BAUDRATE);	     /*设置波特率*/
		check_flag = 1;
	}
	if (check_flag == 1)
	{
		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, SET_REBOOT);						 /*重启命令*/
		Delay_ms(2000);																		 /*重启要多花时间*/

		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_ADDRESS);				 /*读取地址*/
		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_INTERNET_ID);			 /*读取网络ID*/
		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_CHANNEL);				 /*读取信道编号*/
		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_UART);					 /*读取Uart的波特率*/
	}
}

/*---------------------------------发送-------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每6ms发送一次传感器数据至上位机，即在此函数内实现(cnt = 6/2 = 3)
//此函数应由用户每2ms调用一次
void DL_DT_Data_Exchange(void) //2ms一次
{
	static u8 cnt = 0;
	static u8 senser_cnt = 2;	   //2*2 = 4ms
	static u8 user_cnt = 2;	   //4ms
	static u8 status_cnt = 40;	   //80ms
	static u8 speed_cnt = 50;

	if ((cnt % senser_cnt) == (senser_cnt - 1))
		f2.send_senser = 1;

	if ((cnt % user_cnt) == (user_cnt - 1))
		f2.send_user = 1;

	if ((cnt % status_cnt) == (status_cnt - 1))
		f2.send_status = 1;

	if ((cnt % speed_cnt) == (speed_cnt - 1))
		f2.send_speed = 1;

	if (++cnt>200) cnt = 0;
	/////////////////////////////////////////////////////////////////////////////////////
	if (f2.send_senser)
	{
		f2.send_senser = 0;

	}
	if (f2.send_user)
	{
		f2.send_user = 0;
	}
	if (f2.send_status)
	{
		f2.send_status = 0;
		if (mpu9250.Acc_CALIBRATE == 1)						    /*ACC校准*/
		{
			DL_DT_Send_CALIBRATE1(0x0002,1,1);
			DL_DT_Send_CALIBRATE1(0x0004, 1, 1);
			mpu9250.Acc_CALIBRATE  = 0;						 
			mpu9250.Gyro_CALIBRATE = 0;
		}
		if (Mag_CALIBRATED == 1)						   
		{
			DL_DT_Send_CALIBRATE2(0x0002, 1);
			DL_DT_Send_CALIBRATE2(0x0004, 1);
			Mag_CALIBRATED = 0;
		}

	}
	/////////////////////////////////////////////////////////////////////////////////////
	if (f2.send_speed)
	{
		f2.send_speed = 0;
	}
	/////////////////////////////////////////////////////////////////////////////////////
	

}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void DL_DT_Send_Data(/*u8 *dataToSend,*/ u8 length)
{
#ifdef ANO_DT_USE_USB_HID
	Usb_Hid_Adddata(dataToSend, length);
#endif
#ifdef ANO_DT_USE_NRF24l01
	ANO_NRF_TxPacket(dataToSend, length);
#endif
#ifdef ANO_DT_USE_WIFI
	if (!flag.espDownloadMode)
	{
		ANO_UART3_Put_Buf(data_to_send, length);
	}
#endif
#ifdef DL_DT_USE_USART3
	//ANO_Uart3_Send(data_to_send, length);
	Uart3_Tx(data_to_send, length);
#endif
}

/* 读取命令函数(可点亮红灯)
*  输入：
*  target_port:	 目标端口号
*  target_addr:	 目标地址
*  com:	         命令字
*  无返回
*/
void DL_DT_Send_ReadCommand(u8 target_port, u16 target_addr, u8 com)
{
	u8 _cnt = 0;
	data_to_send[_cnt++] = 0xFE;			/*0:包头*/
	data_to_send[_cnt++] = 0;				/*1:长度*/
	data_to_send[_cnt++] = MCU_PORT;		/*2:源端口号*/
	data_to_send[_cnt++] = target_port;		/*3:目的端口号*/

	/*远程地址*/
	data_to_send[_cnt++] = BYTE0(target_addr);	/*4*/
	data_to_send[_cnt++] = BYTE1(target_addr);	/*5*/
	data_to_send[_cnt++] = com;					/*6:命令*/
	data_to_send[_cnt++] = 0xFF;				/*7:包尾*/


	data_to_send[1] = _cnt - 3;

	DL_DT_Send_Data(_cnt);
}
void DL_DT_Send_Status(u16 target_addr,float Roll, float  Pitch, float  Yaw)
{
	u8 _cnt = 0;
	u8 _check_cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xFE;			/*0:包头*/
	data_to_send[_cnt++] = 0;				/*1:长度*/
	CHECK_FF_FE(MCU_PORT);		            /*2:源端口号*/
	CHECK_FF_FE(MCU_PORT1);		            /*3:目的端口号*/

											/*远程地址*/
	CHECK_FF_FE(BYTE0(target_addr));	/*4*/
	CHECK_FF_FE(BYTE1(target_addr));	/*5*/
	_temp = (int)(Roll * 100);
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));
	_temp = (int)(Pitch * 100);
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));
	_temp = (int)(Yaw * 100);
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));

	CHECK_LENGTH(_cnt);

	data_to_send[_cnt++] = 0xFF;				/*7:包尾*/

	data_to_send[1] = _cnt - 3 - _check_cnt;

	DL_DT_Send_Data(_cnt);
}
void DL_DT_Send_Senser(u16 target_addr, s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z)
{
	u8 _cnt = 0;
	u8 _check_cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xFE;			/*0:包头*/
	data_to_send[_cnt++] = 0;				/*1:长度*/
	CHECK_FF_FE(MCU_PORT);		/*2:源端口号*/
	CHECK_FF_FE(MCU_PORT2);		/*3:目的端口号*/

											/*远程地址*/
	CHECK_FF_FE(BYTE0(target_addr));	/*4*/
	CHECK_FF_FE(BYTE1(target_addr));	/*5*/
	_temp = a_x;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));
	_temp = a_y;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));
	_temp = a_z;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));

	_temp = g_x;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));
	_temp = g_y;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));
	_temp = g_z;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));

	_temp = m_x;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));
	_temp = m_y;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));
	_temp = m_z;
	CHECK_FF_FE(BYTE0(_temp));
	CHECK_FF_FE(BYTE1(_temp));

	CHECK_LENGTH(_cnt);

	data_to_send[_cnt++] = 0xFF;				/*7:包尾*/

	data_to_send[1] = _cnt - 3 - _check_cnt;

	DL_DT_Send_Data(_cnt);
}
void DL_DT_Send_CALIBRATE1(u16 target_addr, u8 cal_1, u8  cal_2)
{
	u8 _cnt = 0;
	u8 _check_cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xFE;			/*0:包头*/
	data_to_send[_cnt++] = 0;				/*1:长度*/
	CHECK_FF_FE(MCU_PORT);		/*2:源端口号*/
	CHECK_FF_FE(MCU_PORT1);		/*3:目的端口号*/

	/*远程地址*/
	CHECK_FF_FE(BYTE0(target_addr));	/*4*/
	CHECK_FF_FE(BYTE1(target_addr));	/*5*/
	_temp = cal_1;
	CHECK_FF_FE(BYTE0(_temp));
	_temp = cal_2;
	CHECK_FF_FE(BYTE0(_temp));

	CHECK_LENGTH(_cnt);

	data_to_send[_cnt++] = 0xFF;				/*7:包尾*/


	data_to_send[1] = _cnt - 3 - _check_cnt;

	DL_DT_Send_Data(_cnt);
}
void DL_DT_Send_CALIBRATE2(u16 target_addr, u8 cal_1)
{
	u8 _cnt = 0;
	u8 _check_cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xFE;			/*0:包头*/
	data_to_send[_cnt++] = 0;				/*1:长度*/
	CHECK_FF_FE(MCU_PORT);		/*2:源端口号*/
	CHECK_FF_FE(MCU_PORT2);		/*3:目的端口号*/

	/*远程地址*/
	CHECK_FF_FE(BYTE0(target_addr));	/*4*/
	CHECK_FF_FE(BYTE1(target_addr));	/*5*/
	_temp = cal_1;
	CHECK_FF_FE(BYTE0(_temp));

	CHECK_LENGTH(_cnt);

	data_to_send[_cnt++] = 0xFF;				/*7:包尾*/


	data_to_send[1] = _cnt - 3 - _check_cnt;

	DL_DT_Send_Data(_cnt);
}
/* 修改命令函数
*  输入：
*  target_port:	 目标端口号
*  target_addr:	 目标地址
*  com:	         命令字
*  _value:       值
*  无返回
*/
void DL_DT_Send_ConfigCommand(u8 target_port, u16 target_addr, u8 com, u16 _value)
{
	u8 _cnt = 0;
	data_to_send[_cnt++] = 0xFE;			/*0:包头*/
	data_to_send[_cnt++] = 0;				/*1:长度*/
	data_to_send[_cnt++] = MCU_PORT;		/*2:源端口号*/
	data_to_send[_cnt++] = target_port;		/*3:目的端口号*/

	/*远程地址*/
	data_to_send[_cnt++] = BYTE0(target_addr);	/*4*/
	data_to_send[_cnt++] = BYTE1(target_addr);	/*5*/
	data_to_send[_cnt++] = com;					/*6:命令*/
	if ((com == SET_ADDRESS) || (com == SET_INTERNET_ID))
	{
		data_to_send[_cnt++] = BYTE0(_value);	/*7*/
		data_to_send[_cnt++] = BYTE1(_value);	/*8*/
	}
	else
	{
		data_to_send[_cnt++] = BYTE0(_value);	/*7*/
	}
	data_to_send[_cnt++] = 0xFF;				/*9/8:包尾*/

	data_to_send[1] = _cnt - 3;

	DL_DT_Send_Data(_cnt);
}

/*----------------------------接收-------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void DL_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 state = 0;
	static u8 DL_change = 0;

	if (state == 0 && data == 0xFE)
	{
		state = 1;
		RxBuffer[0] = data;
	}
	else if (state == 1 && data < 64)
	{
		state = 2;
		RxBuffer[1] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (state == 2 && _data_len>0)
	{
		_data_len--;
		/*=============转义字符=============*/
		if (data == 0xFE)
		{
			_data_len++;
			DL_change = 1;
		}
		else if (DL_change == 1)
		{
			if (data == 0xFD)
			{
				_data_cnt--;
				data = 0xFF;
			}
			else if (data == 0xFC)
			{
				_data_cnt--;
				data = 0xFE;
			}

			DL_change = 0;
		}
		/*=================================*/
		RxBuffer[2 + _data_cnt++] = data;
		if (_data_len == 0)
			state = 3;
	}
	else if (state == 3)
	{
		if (data == 0xFF)
			state = 0;
		else
		{
			state = 0;
			/*---ERROR---*/
		}

		RxBuffer[2 + _data_cnt] = data;
		DL_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 3);
	}
	else
		state = 0;
}

/*
* 输入:
* num:数据总长度(字节)
*
*/
void DL_DT_Data_Receive_Anl(u8 *data_buf, u8 num)
{
	//u8 sum = 0;
	/////////////////////////////////校验//////////////////////////////////////
	if (!(*(data_buf) == 0xFE && *(data_buf + num - 1) == 0xFF)) return;		//判断帧头

	//////////////////////////////////CONMAND//////////////////////////////////
	if ((*(data_buf + 4) == 0X00)&& (*(data_buf + 5) == 0X00))					/*本模块*/
	{
		if (*(data_buf + 6) == OPERATION_SUCCESS)
		{
			//mpu9250.Acc_CALIBRATE = 1;
		}
		else if (*(data_buf + 6) == RETURN_ADDRESS)
		{
			dl_ln3x.Address = (*(data_buf + 7) + (*(data_buf + 8)) * 256);
		}
		else if (*(data_buf + 6) == RETURN_INTERNET_ID)
		{
			dl_ln3x.ID = (*(data_buf + 7) + (*(data_buf + 8)) * 256);
		}
		else if (*(data_buf + 6) == RETURN_CHANNEL)
		{
			dl_ln3x.Channel = *(data_buf + 7);
		}
		else if (*(data_buf + 6) == RETURN_UART)
		{
			dl_ln3x.BaudRate = *(data_buf + 7);
		}
		else if (*(data_buf + 6) == REMOTE_ACCESS_FORBIDDEN)
		{
			/*---ERROR---*/
		}
		else if (*(data_buf + 6) == ERROR_COMMAND)
		{
			/*---ERROR---*/
		}
		else if (*(data_buf + 6) == ERROR_LENGTH)
		{
			/*---ERROR---*/
		}
		else if (*(data_buf + 6) == ERROR_VALUE)
		{
			/*---ERROR---*/
		}
	}
	else if ((*(data_buf + 4) == DL2_ADDRESS_L) && (*(data_buf + 5) == DL2_ADDRESS_H))	/*收到姿态模块的数据*/
	{
		if (*(data_buf + 3) == MCU_PORT1)  /*功能1*/
		{
			Roll  = ((float)((short int)(*(data_buf + 6) + (*(data_buf + 7)) *256 )))/100.0f;
			Pitch = ((float)((short int)(*(data_buf + 8) + (*(data_buf + 9)) * 256))) / 100.0f;
			Yaw   = ((float)((short int)(*(data_buf + 10) + (*(data_buf + 11)) * 256))) / 100.0f;
		}
		else if (*(data_buf + 3) == MCU_PORT2)  /*功能2*/
		{
			mpu9250.Acc.x = (*(data_buf + 6) + (*(data_buf + 7)) * 256);
			mpu9250.Acc.y = (*(data_buf + 8) + (*(data_buf + 9)) * 256);
			mpu9250.Acc.z = (*(data_buf + 10) + (*(data_buf + 11)) * 256);

			mpu9250.Gyro.x = (*(data_buf + 12) + (*(data_buf + 13)) * 256);
			mpu9250.Gyro.y = (*(data_buf + 14) + (*(data_buf + 15)) * 256);
			mpu9250.Gyro.z = (*(data_buf + 16) + (*(data_buf + 17)) * 256);

			mpu9250.Mag_Val.x = (*(data_buf + 18) + (*(data_buf + 19)) * 256);
			mpu9250.Mag_Val.y = (*(data_buf + 20) + (*(data_buf + 21)) * 256);
			mpu9250.Mag_Val.z = (*(data_buf + 22) + (*(data_buf + 23)) * 256);
		}
	}
	else if ((*(data_buf + 4) == DL3_ADDRESS_L) && (*(data_buf + 5) == DL3_ADDRESS_H))	/*收到高度模块的数据*/
	{
		if (*(data_buf + 3) == MCU_PORT1)  /*功能1*/
		{
			Distance_data  = (*(data_buf + 6) + (*(data_buf + 7)) * 256);
			Distance_state = *(data_buf + 8);
		}
		else if (*(data_buf + 3) == MCU_PORT2)  /*功能2*/
		{

		}
	}
	else if ((*(data_buf + 4) == DL4_ADDRESS_L) && (*(data_buf + 5) == DL4_ADDRESS_H))	/*收到姿态模块2的数据*/
	{
		if (*(data_buf + 3) == MCU_PORT1)  /*功能1*/
		{
			Roll_2  = ((float)((short int)(*(data_buf + 6) + (*(data_buf + 7)) * 256))) / 100.0f;
			Pitch_2 = ((float)((short int)(*(data_buf + 8) + (*(data_buf + 9)) * 256))) / 100.0f;
			Yaw_2   = ((float)((short int)(*(data_buf + 10) + (*(data_buf + 11)) * 256))) / 100.0f;
		}
		else if (*(data_buf + 3) == MCU_PORT2)  /*功能2*/
		{

		}
	}
	else if ((*(data_buf + 4) == DL6_ADDRESS_L) && (*(data_buf + 5) == DL6_ADDRESS_H))	/*收到气压模块的数据*/
	{
		if (*(data_buf + 3) == MCU_PORT1)  /*功能1*/
		{
			baro.relative_height = (*(data_buf + 6) + (*(data_buf + 7)) * 256);			//cm
			baro.measure_ok      = *(data_buf + 8);
		}
		else if (*(data_buf + 3) == MCU_PORT2)  /*功能2*/
		{

		}
	}
}

#endif
