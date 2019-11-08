#include "DL_LN3X.h"

#if DL_LN3X

static u8 data_to_send[50];			//�������ݻ���
DL_LN3X_STRUCT dl_ln3x;
dt_flag_t f2;					    //��Ҫ�������ݵı�־

/*��ģ���ʼ������
* ������
* �����
*/
void DL_LN3X_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	ANO_UART3_Init(115200);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //PB1���ó��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIOB->BSRR = GPIO_Pin_1;	  //�ӵ�

	DL_DT_Send_ReadCommand(RED_PORT, SELF_MODULE, 50);									 /*�������5s*/
	//Delay_ms(10);
	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_ADDRESS, DL_ADDRESS);		 /*���õ�ַ*/
	//Delay_ms(10);
	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_INTERNET_ID, DL_INTERNET_ID); /*��������ID*/
	//Delay_ms(10);
	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_CHANNEL, DL_CHANNEL);		 /*�����ŵ�*/
	//Delay_ms(10);
	DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_BAUDRATE, DL_BAUDRATE);	     /*���ò�����*/
	//Delay_ms(10);
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, SET_REBOOT);						 /*��������*/
	Delay_ms(2000);																		 /*����Ҫ�໨ʱ��*/
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_ADDRESS);				 /*��ȡ��ַ*/
	//Delay_ms(10);
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_INTERNET_ID);			 /*��ȡ����ID*/
	//Delay_ms(10);
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_CHANNEL);				 /*��ȡ�ŵ����*/
	//Delay_ms(10);
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_UART);					 /*��ȡUart�Ĳ�����*/
	//Delay_ms(10);
}

/* ��ģ���麯��
*  ������
*  �����
*  ��ע�������2sִ��ʱ��
*/
void DL_LN3X_Check(void)
{
	u8 check_flag = 0;
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_ADDRESS);				 /*��ȡ��ַ*/
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_INTERNET_ID);			 /*��ȡ����ID*/
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_CHANNEL);				 /*��ȡ�ŵ����*/
	DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_UART);					 /*��ȡUart�Ĳ�����*/

	Delay_ms(10);
	if (dl_ln3x.Channel != DL_CHANNEL)
	{
		DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_CHANNEL, DL_CHANNEL);		 /*�����ŵ�*/
		check_flag = 1;
	}
	if (dl_ln3x.ID != DL_INTERNET_ID)
	{
		DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_INTERNET_ID, DL_INTERNET_ID); /*��������ID*/
		check_flag = 1;
	}
	if (dl_ln3x.Address != DL_ADDRESS)
	{
		DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_ADDRESS, DL_ADDRESS);		 /*���õ�ַ*/
		check_flag = 1;
	}
	if (dl_ln3x.BaudRate != DL_BAUDRATE)
	{
		DL_DT_Send_ConfigCommand(CONFIG_PORT, SELF_MODULE, SET_BAUDRATE, DL_BAUDRATE);	     /*���ò�����*/
		check_flag = 1;
	}
	if (check_flag == 1)
	{
		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, SET_REBOOT);						 /*��������*/
		Delay_ms(2000);																		 /*����Ҫ�໨ʱ��*/

		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_ADDRESS);				 /*��ȡ��ַ*/
		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_INTERNET_ID);			 /*��ȡ����ID*/
		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_CHANNEL);				 /*��ȡ�ŵ����*/
		DL_DT_Send_ReadCommand(CONFIG_PORT, SELF_MODULE, READ_SELF_UART);					 /*��ȡUart�Ĳ�����*/
	}
}

/*---------------------------------����-------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange��������������ݷ������󣬱�����ʵ��ÿ6ms����һ�δ�������������λ�������ڴ˺�����ʵ��(cnt = 6/2 = 3)
//�˺���Ӧ���û�ÿ2ms����һ��
void DL_DT_Data_Exchange(void) //2msһ��
{
	static u8 cnt = 0;
	static u8 senser_cnt = 10;	   //10*2 = 20ms
	static u8 user_cnt = 10;	   //20ms
	static u8 status_cnt = 40;	   //80ms
	static u8 power_cnt = 50;
	static u8 speed_cnt = 50;

	if ((cnt % senser_cnt) == (senser_cnt - 1))
		f2.send_senser = 1;

	if ((cnt % user_cnt) == (user_cnt - 2))
		f2.send_user = 1;

	if ((cnt % status_cnt) == (status_cnt - 1))
		f2.send_status = 1;

	if ((cnt % power_cnt) == (power_cnt - 2))
		f2.send_power = 1;

	if ((cnt % speed_cnt) == (speed_cnt - 3))
		f2.send_speed = 1;

	if (++cnt>200) cnt = 0;
	/////////////////////////////////////////////////////////////////////////////////////
	if (f2.msg_id)
	{
		f2.msg_id = 0;
	}
	/////////////////////////////////////////////////////////////////////////////////////
	else if (f2.send_version)
	{
		f2.send_version = 0;
		//ANO_DT_Send_Version(1,ANO_Param.hardware,ANO_Param.software,400,0);
	}
	/////////////////////////////////////////////////////////////////////////////////////
	else if (f2.send_status)
	{
		f2.send_status = 0;
		//ANO_DT_Send_Status(imu_data.rol,imu_data.pit,imu_data.yaw,0,0,fly_ready);	
		//�����,������,ƫ���ǣ��ɻ��߶� ����ģʽ����״̬
		DL_DT_Send_Status(0x0001, Roll, Pitch, Yaw);
        #if VL15310x
		if (Distance_state != 0)
		{
			Distance_data = 0;
		}

		DL_DT_Send_Status(0x0001, Distance_data, Distance_state);
        #endif
	}
	else if (f2.send_senser)
	{
		f2.send_senser = 0;
		//DL_DT_Send_Senser(0x0001, mpu9250.Acc.x, mpu9250.Acc.y, mpu9250.Acc.z,
		//	mpu9250.Gyro.x, mpu9250.Gyro.y, mpu9250.Gyro.z,
		//	mpu9250.Mag_Val.x, mpu9250.Mag_Val.y, mpu9250.Mag_Val.z);
	}
	/////////////////////////////////////////////////////////////////////////////////////
	else if (f2.send_speed)
	{
		f2.send_speed = 0;
	}
	/////////////////////////////////////////////////////////////////////////////////////
	else if (f2.send_user)
	{
		f2.send_user = 0;
		//ANO_DT_Send_User();
	}

}
/////////////////////////////////////////////////////////////////////////////////////
//Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
//��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���
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
	ANO_Uart3_Send(data_to_send, length);
#endif
}

/* ��ȡ�����(�ɵ������)
*  ���룺
*  target_port:	 Ŀ��˿ں�
*  target_addr:	 Ŀ���ַ
*  com:	         ������
*  �޷���
*/
void DL_DT_Send_ReadCommand(u8 target_port, u16 target_addr, u8 com)
{
	u8 _cnt = 0;
	data_to_send[_cnt++] = 0xFE;			/*0:��ͷ*/
	data_to_send[_cnt++] = 0;				/*1:����*/
	data_to_send[_cnt++] = MCU_PORT;		/*2:Դ�˿ں�*/
	data_to_send[_cnt++] = target_port;		/*3:Ŀ�Ķ˿ں�*/

	/*Զ�̵�ַ*/
	data_to_send[_cnt++] = BYTE0(target_addr);	/*4*/
	data_to_send[_cnt++] = BYTE1(target_addr);	/*5*/
	data_to_send[_cnt++] = com;					/*6:����*/
	data_to_send[_cnt++] = 0xFF;				/*7:��β*/


	data_to_send[1] = _cnt - 3;

	DL_DT_Send_Data(_cnt);
}

//void DL_DT_Send_Status(u16 target_addr,float Roll, float  Pitch, float  Yaw)
//{
//	u8 _cnt = 0;
//	vs16 _temp;
//
//	data_to_send[_cnt++] = 0xFE;			/*0:��ͷ*/
//	data_to_send[_cnt++] = 0;				/*1:����*/
//	data_to_send[_cnt++] = MCU_PORT;		/*2:Դ�˿ں�*/
//	data_to_send[_cnt++] = MCU_PORT1;		/*3:Ŀ�Ķ˿ں�*/
//
//	/*Զ�̵�ַ*/
//	data_to_send[_cnt++] = BYTE0(target_addr);	/*4*/
//	data_to_send[_cnt++] = BYTE1(target_addr);	/*5*/
//	_temp = (int)(Roll * 100);
//	data_to_send[_cnt++] = BYTE0(_temp);
//	data_to_send[_cnt++] = BYTE1(_temp);
//	_temp = (int)(Pitch * 100);
//	data_to_send[_cnt++] = BYTE0(_temp);
//	data_to_send[_cnt++] = BYTE1(_temp);
//	_temp = (int)(Yaw * 100);
//	data_to_send[_cnt++] = BYTE0(_temp);
//	data_to_send[_cnt++] = BYTE1(_temp);
//	
//	data_to_send[_cnt++] = 0xFF;				/*7:��β*/
//
//
//	data_to_send[1] = _cnt - 3;
//
//	DL_DT_Send_Data(_cnt);
//}
void DL_DT_Send_Status(u16 target_addr,float Roll, float  Pitch, float  Yaw)
{
	u8 _cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xFE;			/*0:��ͷ*/
	data_to_send[_cnt++] = 0;				/*1:����*/
	data_to_send[_cnt++] = MCU_PORT;		/*2:Դ�˿ں�*/
	data_to_send[_cnt++] = MCU_PORT1;		/*3:Ŀ�Ķ˿ں�*/

	/*Զ�̵�ַ*/
	data_to_send[_cnt++] = BYTE0(target_addr);	/*4*/
	data_to_send[_cnt++] = BYTE1(target_addr);	/*5*/
	_temp = (int)(Roll * 100);
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	_temp = (int)(Pitch * 100);
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	_temp = (int)(Yaw * 100);
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	
	data_to_send[_cnt++] = 0xFF;				/*7:��β*/


	data_to_send[1] = _cnt - 3;

	DL_DT_Send_Data(_cnt);
}
void DL_DT_Send_Senser(u16 target_addr, s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z)
{
	u8 _cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xFE;			/*0:��ͷ*/
	data_to_send[_cnt++] = 0;				/*1:����*/
	data_to_send[_cnt++] = MCU_PORT;		/*2:Դ�˿ں�*/
	data_to_send[_cnt++] = MCU_PORT2;		/*3:Ŀ�Ķ˿ں�*/

											/*Զ�̵�ַ*/
	data_to_send[_cnt++] = BYTE0(target_addr);	/*4*/
	data_to_send[_cnt++] = BYTE1(target_addr);	/*5*/
	_temp = a_x;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	_temp = a_y;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	_temp = a_z;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);

	_temp = g_x;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	_temp = g_y;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	_temp = g_z;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);

	_temp = m_x;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	_temp = m_y;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	_temp = m_z;
	data_to_send[_cnt++] = BYTE0(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);

	data_to_send[_cnt++] = 0xFF;				/*7:��β*/


	data_to_send[1] = _cnt - 3;

	DL_DT_Send_Data(_cnt);
}
/* �޸������
*  ���룺
*  target_port:	 Ŀ��˿ں�
*  target_addr:	 Ŀ���ַ
*  com:	         ������
*  _value:       ֵ
*  �޷���
*/
void DL_DT_Send_ConfigCommand(u8 target_port, u16 target_addr, u8 com, u16 _value)
{
	u8 _cnt = 0;
	data_to_send[_cnt++] = 0xFE;			/*0:��ͷ*/
	data_to_send[_cnt++] = 0;				/*1:����*/
	data_to_send[_cnt++] = MCU_PORT;		/*2:Դ�˿ں�*/
	data_to_send[_cnt++] = target_port;		/*3:Ŀ�Ķ˿ں�*/

	/*Զ�̵�ַ*/
	data_to_send[_cnt++] = BYTE0(target_addr);	/*4*/
	data_to_send[_cnt++] = BYTE1(target_addr);	/*5*/
	data_to_send[_cnt++] = com;					/*6:����*/
	if ((com == SET_ADDRESS) || (com == SET_INTERNET_ID))
	{
		data_to_send[_cnt++] = BYTE0(_value);	/*7*/
		data_to_send[_cnt++] = BYTE1(_value);	/*8*/
	}
	else
	{
		data_to_send[_cnt++] = BYTE0(_value);	/*7*/
	}
	data_to_send[_cnt++] = 0xFF;				/*9/8:��β*/

	data_to_send[1] = _cnt - 3;

	DL_DT_Send_Data(_cnt);
}

/*----------------------------����-------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare������Э��Ԥ����������Э��ĸ�ʽ�����յ������ݽ���һ�θ�ʽ�Խ�������ʽ��ȷ�Ļ��ٽ������ݽ���
//��ֲʱ���˺���Ӧ���û���������ʹ�õ�ͨ�ŷ�ʽ���е��ã����紮��ÿ�յ�һ�ֽ����ݣ�����ô˺���һ��
//�˺������������ϸ�ʽ������֡�󣬻����е������ݽ�������
void DL_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 state = 0;

	if (state == 0 && data == 0xFE)
	{
		state = 1;
		RxBuffer[0] = data;
	}
	else if (state == 1 && data<50)
	{
		state = 2;
		RxBuffer[1] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (state == 2 && _data_len>0)
	{
		_data_len--;
		RxBuffer[2 + _data_cnt++] = data;
		if (_data_len == 0)
			state = 3;
	}
	else if (state == 3)
	{
		if(data == 0xFF)
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
* ����:
* num:�����ܳ���(�ֽ�)
*
*/
void DL_DT_Data_Receive_Anl(u8 *data_buf, u8 num)
{
	//u8 sum = 0;
	/////////////////////////////////У��//////////////////////////////////////
	if (!(*(data_buf) == 0xFE && *(data_buf + num - 1) == 0xFF)) return;		//�ж�֡ͷ

	//////////////////////////////////CONMAND//////////////////////////////////
	if ((*(data_buf + 4) == 0X00)&& (*(data_buf + 5) == 0X00))					/*��ģ��*/
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
	else if ((*(data_buf + 4) == DL_Main_ADDRESS_L) && (*(data_buf + 5) == DL_Main_ADDRESS_H))	/*�յ�����ģ�������*/
	{
		if (*(data_buf + 3) == MCU_PORT1)  /*����1*/
		{
			mpu9250.Acc_CALIBRATE  = (*(data_buf + 6));
			mpu9250.Gyro_CALIBRATE = (*(data_buf + 7));
		}
		else if (*(data_buf + 3) == MCU_PORT2)  /*����2*/
		{
			Mag_CALIBRATED = (*(data_buf + 6));
		}
	}
	else if ((*(data_buf + 4) == DL_ADDRESS_L) && (*(data_buf + 5) == DL_ADDRESS_H))	/*�յ�����ģ�������*/
	{
		if (*(data_buf + 3) == MCU_PORT1)  /*����1*/
		{
//			Roll  = ((float)(*(data_buf + 6) + (*(data_buf + 7)) << 8 ))/100.0f;
//			Pitch = ((float)(*(data_buf + 8) + (*(data_buf + 9)) << 8 )) / 100.0f;
//			Yaw   = ((float)(*(data_buf + 10) + (*(data_buf + 11)) << 8 )) / 100.0f;
		}
	}
	else if ((*(data_buf + 4) == DL6_ADDRESS_L) && (*(data_buf + 5) == DL6_ADDRESS_H))	/*�յ���ѹģ�������*/
	{
		if (*(data_buf + 3) == MCU_PORT1)  /*����1*/
		{
			//baro.relative_height = (*(data_buf + 6) + (*(data_buf + 7)) * 256);			//cm
			//baro.measure_ok = *(data_buf + 8);
		}
		else if (*(data_buf + 3) == MCU_PORT2)  /*����2*/
		{

		}
	}




}

#endif
