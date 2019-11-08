#include "vl53l0x.h"

#if VL15310x

VL53L0X_Dev_t vl53l0x_dev;                //�豸I2C���ݲ���
VL53L0X_DeviceInfo_t vl53l0x_dev_info;    //�豸ID�汾��Ϣ
uint8_t AjustOK = 0;//У׼��־λ

VL53L0X_RangingMeasurementData_t vl53l0x_data;  //�������ṹ��
vu16 Distance_data = 0;                         //����������
//(0:Range valid / 1 : Sigma fail / 2 : Signal fail / 3 : min range fail / 4 : phase fail / 5 : hardware fail)
vu8	 Distance_state = 0;

_vl53l0x_adjust Vl53l0x_adjust; //У׼����24c02д������(������У׼ģʽУ׼����д��24c02)
_vl53l0x_adjust Vl53l0x_data;   //У׼����24c02��������������ϵͳ��ʼ��ʱ��24C02��ȡ���ݣ�

#define adjust_num 5//У׼�������

//VL53L0X������ģʽ����
//0��Ĭ��;1:�߾���;2:������;3:����
mode_data Mode_data[] =
{
	{ (FixPoint1616_t)(0.25 * 65536),
	(FixPoint1616_t)(18 * 65536),
	33000,
	14,
	10 },//Ĭ��

	{ (FixPoint1616_t)(0.25 * 65536) ,
	(FixPoint1616_t)(18 * 65536),
	200000,
	14,
	10 },//�߾���

	{ (FixPoint1616_t)(0.1 * 65536) ,
	(FixPoint1616_t)(60 * 65536),
	33000,
	18,
	14 },//������

	{ (FixPoint1616_t)(0.25 * 65536) ,
	(FixPoint1616_t)(32 * 65536),
	20000,
	14,
	10 },//����

};


//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	uint8_t status_uchar = 0;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta = 0x00;

	

	FinalAddress = newaddr;

	if (FinalAddress == dev->I2cDevAddr)//���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
		return VL53L0X_ERROR_NONE;
	//�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
	Status = VL53L0X_WrByte(dev, 0x88, 0x00);
	//Status = IIC_Write_1Byte(dev->I2cDevAddr, 0x88, 0x00);

	if (Status != VL53L0X_ERROR_NONE)
	{
		sta = 0x01;//����I2C��׼ģʽ����
		goto set_error;
	}

	//����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	//Status = IIC_Read_nByte(dev->I2cDevAddr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, 2, (uint8_t *)(&Id));
	//if (Id == 0xAAEE)
	//{
	//	Id = 0xEEAA;
	//}
	/*���������ֽ�*/
	//Id = SWAP_8x8(*(uint8_t *)(&Id), *((uint8_t *)(&Id) + 1));
	//Id = SWAP32(Id);

	if (Status != VL53L0X_ERROR_NONE)
	{
		sta = 0x02;//��ȡ�Ĵ�������
		goto set_error;
	}
	if (Id == 0xEEAA)
	{
		//�����豸�µ�I2C��ַ
		Status = VL53L0X_SetDeviceAddress(dev, FinalAddress);
		//VL53L0X_WrByte(dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, FinalAddress / 2);
		//Status = IIC_Write_1Byte(dev->I2cDevAddr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, FinalAddress /*/ 2*/);

		if (Status != VL53L0X_ERROR_NONE)
		{
			sta = 0x03;//����I2C��ַ����
			goto set_error;
		}

		//�޸Ĳ����ṹ���I2C��ַ
		dev->I2cDevAddr = FinalAddress;
		//����µ�I2C��ַ��д�Ƿ�����
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		//Status = IIC_Read_nByte(dev->I2cDevAddr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, 2, (uint8_t *)(&Id));

		if (Status != VL53L0X_ERROR_NONE)
		{
			sta = 0x04;//��I2C��ַ��д����
			goto set_error;
		}
	}
set_error:
	if (Status != VL53L0X_ERROR_NONE)
	{
		//print_pal_error(Status);//��ӡ������Ϣ
	}
	if (sta != 0)
	{
		/*printf("sta:0x%x\r\n",sta)*/;
	}
	return Status;
}
//��ʼ��vl53l0x
//dev:�豸I2C�����ṹ��
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = dev;
	GPIO_InitTypeDef  GPIO_InitStructure;
	//   RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ�� 
	//GPIOA->CRH&=0X0FFFFFFF;	//PA15���ó��������	  
	//GPIOA->CRH|=0X30000000; 
	//JTAG_Set(SWD_ENABLE);	//��ֹJTAG,�Ӷ�PA15��������ͨIOʹ��,����PA15��������ͨIO!!!
	/*-------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //PB8���ó��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*-------------------------------------------------------------------*/
	pMyDevice->I2cDevAddr = VL53L0X_Addr;//I2C��ַ(�ϵ�Ĭ��0x52)
	pMyDevice->comms_type = 1;           //I2Cͨ��ģʽ
	pMyDevice->comms_speed_khz = 400;    //I2Cͨ������

										 //VL53L0X_i2c_init();//��ʼ��IIC����
	I2c_Soft_Init();

	I2C_FastMode = 0;

	//VL53L0X_Xshut=0;//ʧ��VL53L0X
	GPIOB->BRR = GPIO_Pin_8;
	Delay_ms(30);
	//VL53L0X_Xshut=1;//ʹ��VL53L0X,�ô��������ڹ���
	GPIOB->BSRR = GPIO_Pin_8;
	Delay_ms(30);

	Status = vl53l0x_Addr_set(pMyDevice, (SET_VL53L0X_Addr >>1));//����VL53L0X������I2C��ַ
	if (Status != VL53L0X_ERROR_NONE) goto error;

	Status = VL53L0X_DataInit(pMyDevice);//�豸��ʼ��
	if (Status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);

	Status = VL53L0X_GetDeviceInfo(pMyDevice, &vl53l0x_dev_info);//��ȡ�豸ID��Ϣ
	if (Status != VL53L0X_ERROR_NONE) goto error;

	//AT24CXX_Read(0,(u8*)&Vl53l0x_data,sizeof(_vl53l0x_adjust));//��ȡ24c02�����У׼����,����У׼ Vl53l0x_data.adjustok==0xAA

	if (Vl53l0x_data.adjustok == 0xAA)//��У׼
		AjustOK = 1;
	else //ûУ׼	
		AjustOK = 0;

error:
	if (Status != VL53L0X_ERROR_NONE)
	{
		//print_pal_error(Status);//��ӡ������Ϣ
		return Status;
	}

	return Status;
}

/*------------------------------------����--------------------------------------*/
//vl53l0x��ͨ����ģʽ����
//dev:�豸I2C�����ṹ��
void vl53l0x_general(VL53L0X_Dev_t *dev)
{
	u8 mode = 0;
	//mode = 0 ;"Default        "Ĭ��
	//mode = 1 ;"High Accuracy  "�߾���
	//mode = 2 ;"Long Range     "������
	//mode = 3 ;"High Speed     "����

	vl53l0x_general_start(dev, mode);
    //delay_ms(50);

}
//������ͨ����
//dev���豸I2C�����ṹ��
//modeģʽ���� 0:Ĭ��;1:�߾���;2:������
void vl53l0x_general_start(VL53L0X_Dev_t *dev, u8 mode)
{
	static char buf[VL53L0X_MAX_STRING_LENGTH];//����ģʽ�ַ����ַ�������
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;//����״̬


	//mode_string(mode, buf);//��ʾ��ǰ���õ�ģʽ
	while (vl53l0x_set_mode(dev, mode))//���ò���ģʽ
	{

	} 

	if (Status == VL53L0X_ERROR_NONE)
	{
		Status = vl53l0x_start_single_test(dev, &vl53l0x_data, buf);//ִ��һ�β���

		//LCD_ShowxNum(110, 140 + 90, Distance_data, 4, 16, 0);
	
		//printf("d: %4imm\r\n", Distance_data);//��ӡ��������
	}
	//delay_ms(50);

}

//vl53l0x��λ����
//dev:�豸I2C�����ṹ��
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//�����豸ԭI2C��ַ
	//VL53L0X_Xshut = 0;//ʧ��VL53L0X
	GPIOB->BRR = GPIO_Pin_8;
	Delay_ms(30);
	//VL53L0X_Xshut = 1;//ʹ��VL53L0X,�ô��������ڹ���(I2C��ַ��ָ�Ĭ��0X52)
	GPIOB->BSRR = GPIO_Pin_8;
	Delay_ms(30);
	dev->I2cDevAddr = VL53L0X_Addr;
	vl53l0x_Addr_set(dev, addr);//����VL53L0X������ԭ���ϵ�ǰԭI2C��ַ
	VL53L0X_DataInit(dev);
}
//VL53L0X ����ģʽ����
//dev:�豸I2C�����ṹ��
//mode: 0:Ĭ��;1:�߾���;2:������
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev, u8 mode)
{

	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;

	vl53l0x_reset(dev);//��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)

	status = VL53L0X_StaticInit(dev);

	if (AjustOK != 0)//��У׼����,д��У׼ֵ
	{
		status = VL53L0X_SetReferenceSpads(dev, Vl53l0x_data.refSpadCount, Vl53l0x_data.isApertureSpads);//�趨SpadsУ׼ֵ
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
		status = VL53L0X_SetRefCalibration(dev, Vl53l0x_data.VhvSettings, Vl53l0x_data.PhaseCal);//�趨RefУ׼ֵ
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
		status = VL53L0X_SetOffsetCalibrationDataMicroMeter(dev, Vl53l0x_data.OffsetMicroMeter);//�趨ƫ��У׼ֵ
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
		status = VL53L0X_SetXTalkCompensationRateMegaCps(dev, Vl53l0x_data.XTalkCompensationRateMegaCps);//�趨����У׼ֵ
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);

	}
	else
	{
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref�ο�У׼
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//ִ�вο�SPAD����
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
	}
	status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);//ʹ�ܵ��β���ģʽ
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);//ʹ��SIGMA��Χ���
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);//ʹ���ź����ʷ�Χ���
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Mode_data[mode].sigmaLimit);//�趨SIGMA��Χ
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Mode_data[mode].signalLimit);//�趨�ź����ʷ�Χ��Χ
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, Mode_data[mode].timingBudget);//�趨��������ʱ��
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//�趨VCSEL��������
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//�趨VCSEL�������ڷ�Χ

error://������Ϣ
	if (status != VL53L0X_ERROR_NONE)
	{
		//print_pal_error(status);
		//LCD_Fill(30, 140 + 20, 300, 300, WHITE);
		return status;
	}
	return status;

}
//VL53L0X ���ξ����������
//dev:�豸I2C�����ṹ��
//pdata:����������ݽṹ��
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *pdata, char *buf)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;

	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//ִ�е��β�ಢ��ȡ����������
	if (status != VL53L0X_ERROR_NONE) return status;

	RangeStatus = pdata->RangeStatus;//��ȡ��ǰ����״̬
	memset(buf, 0x00, VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus, buf);//���ݲ���״̬��ȡ״̬�ַ���

	Distance_data = pdata->RangeMilliMeter;//�������һ�β���������

	return status;
}

//VL53L0X ���ξ���������� 2
//dev:�豸I2C�����ṹ��
//pdata:����������ݽṹ��
VL53L0X_Error vl53l0x_start_SingleMeasure(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *pdata)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;

	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//ִ�е��β�ಢ��ȡ����������
	if (status != VL53L0X_ERROR_NONE) return status;

	RangeStatus = pdata->RangeStatus;                           //��ȡ��ǰ����״̬(0:Range valid / 1:Sigma fail / 2:Signal fail / 3:min range fail / 4:phase fail / 5:hardware fail)

	Distance_data = pdata->RangeMilliMeter;                     //�������һ�β���������

	Distance_state = RangeStatus;								//�������һ�β��������ݵ�״̬

	return status;
}
/*----------------------------------------------------------------------------------*/
/*-------------------------------------У׼-----------------------------------------*/
//vl53l0xУ׼����
//dev:�豸I2C�����ṹ��
void vl53l0x_calibration(VL53L0X_Dev_t *dev)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	status = vl53l0x_adjust(dev);     //����У׼
	if (status != VL53L0X_ERROR_NONE) //У׼ʧ��
	{

	}

	//delay_ms(500);

}

//VL53L0XУ׼����
//dev:�豸I2C�����ṹ��
VL53L0X_Error vl53l0x_adjust(VL53L0X_Dev_t *dev)
{

	VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount = 7;
	uint8_t  isApertureSpads = 0;
	uint32_t XTalkCalDistance = 100;
	uint32_t XTalkCompensationRateMegaCps;
	uint32_t CalDistanceMilliMeter = 100 << 16;
	int32_t  OffsetMicroMeter = 30000;
	uint8_t VhvSettings = 23;
	uint8_t PhaseCal = 1;
	u8 i = 0;

	VL53L0X_StaticInit(dev);//��ֵ�ָ�Ĭ��,���������ڿ���״̬
/*-------------------------------SPADSУ׼------------------------------*/
spads:
	Delay_ms(10);

	Status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//ִ�вο�Spad����
	if (Status == VL53L0X_ERROR_NONE)
	{
		//printf("refSpadCount = %d\r\n", refSpadCount);
		Vl53l0x_adjust.refSpadCount = refSpadCount;
		//printf("isApertureSpads = %d\r\n", isApertureSpads);
		Vl53l0x_adjust.isApertureSpads = isApertureSpads;
		//printf("The SPADS Calibration Finish...\r\n\r\n");
		i = 0;
	}
	else
	{
		i++;
		if (i == adjust_num) return Status;
		/*printf("SPADS Calibration Error,Restart this step\r\n")*/;
		goto spads;
	}
/*-------------------------------------------�豸�ο�У׼---------------------------------------------------*/
ref:
	Delay_ms(10);
	Status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref�ο�У׼
	if (Status == VL53L0X_ERROR_NONE)
	{
		//printf("VhvSettings = %d\r\n", VhvSettings);
		Vl53l0x_adjust.VhvSettings = VhvSettings;
		//printf("PhaseCal = %d\r\n", PhaseCal);
		Vl53l0x_adjust.PhaseCal = PhaseCal;
		//printf("The Ref Calibration Finish...\r\n\r\n");
		i = 0;
	}
	else
	{
		i++;
		if (i == adjust_num) return Status;
		/*printf("Ref Calibration Error,Restart this step\r\n")*/;
		goto ref;
	}
/*---------------------------------------------ƫ��У׼------------------------------------------------*/
offset:
	Delay_ms(10);
	//printf("Offset Calibration:need a white target,in black space,and the distance keep 100mm!\r\n");
	//printf("The Offset Calibration Start...\r\n");
	Status = VL53L0X_PerformOffsetCalibration(dev, CalDistanceMilliMeter, &OffsetMicroMeter);//ƫ��У׼
	if (Status == VL53L0X_ERROR_NONE)
	{
		//printf("CalDistanceMilliMeter = %d mm\r\n", CalDistanceMilliMeter);
		Vl53l0x_adjust.CalDistanceMilliMeter = CalDistanceMilliMeter;
		//printf("OffsetMicroMeter = %d mm\r\n", OffsetMicroMeter);
		Vl53l0x_adjust.OffsetMicroMeter = OffsetMicroMeter;
		//printf("The Offset Calibration Finish...\r\n\r\n");
		i = 0;
	}
	else
	{
		i++;
		if (i == adjust_num) return Status;
		/*printf("Offset Calibration Error,Restart this step\r\n")*/;
		goto offset;
	}
/*----------------------------------------------����У׼-----------------------------------------------------*/
xtalk:
	Delay_ms(20);
	//printf("Cross Talk Calibration:need a grey target\r\n");
	//printf("The Cross Talk Calibration Start...\r\n");
	Status = VL53L0X_PerformXTalkCalibration(dev, XTalkCalDistance, &XTalkCompensationRateMegaCps);//����У׼
	if (Status == VL53L0X_ERROR_NONE)
	{
		//printf("XTalkCalDistance = %d mm\r\n", XTalkCalDistance);
		Vl53l0x_adjust.XTalkCalDistance = XTalkCalDistance;
		//printf("XTalkCompensationRateMegaCps = %d\r\n", XTalkCompensationRateMegaCps);
		Vl53l0x_adjust.XTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
		//printf("The Cross Talk Calibration Finish...\r\n\r\n");
		i = 0;
	}
	else
	{
		i++;
		if (i == adjust_num) return Status;
		/*printf("Cross Talk Calibration Error,Restart this step\r\n")*/;
		goto xtalk;
	}
	//LED1 = 1;
	//printf("All the Calibration has Finished!\r\n");
	//printf("Calibration is successful!!\r\n");

	Vl53l0x_adjust.adjustok = 0xAA;//У׼�ɹ�
	//AT24CXX_Write(0, (u8*)&Vl53l0x_adjust, sizeof(_vl53l0x_adjust));//��У׼���ݱ��浽24c02
	memcpy(&Vl53l0x_data, &Vl53l0x_adjust, sizeof(_vl53l0x_adjust));//��У׼���ݸ��Ƶ�Vl53l0x_data�ṹ��
	return Status;
}
/*----------------------------------------------------------------------------------*/


#endif

