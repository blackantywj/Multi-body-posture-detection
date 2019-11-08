#include "vl53l0x.h"

#if VL15310x

VL53L0X_Dev_t vl53l0x_dev;                //设备I2C数据参数
VL53L0X_DeviceInfo_t vl53l0x_dev_info;    //设备ID版本信息
uint8_t AjustOK = 0;//校准标志位

VL53L0X_RangingMeasurementData_t vl53l0x_data;  //测距测量结构体
vu16 Distance_data = 0;                         //保存测距数据
//(0:Range valid / 1 : Sigma fail / 2 : Signal fail / 3 : min range fail / 4 : phase fail / 5 : hardware fail)
vu8	 Distance_state = 0;

_vl53l0x_adjust Vl53l0x_adjust; //校准数据24c02写缓存区(用于在校准模式校准数据写入24c02)
_vl53l0x_adjust Vl53l0x_data;   //校准数据24c02读缓存区（用于系统初始化时向24C02读取数据）

#define adjust_num 5//校准错误次数

//VL53L0X各测量模式参数
//0：默认;1:高精度;2:长距离;3:高速
mode_data Mode_data[] =
{
	{ (FixPoint1616_t)(0.25 * 65536),
	(FixPoint1616_t)(18 * 65536),
	33000,
	14,
	10 },//默认

	{ (FixPoint1616_t)(0.25 * 65536) ,
	(FixPoint1616_t)(18 * 65536),
	200000,
	14,
	10 },//高精度

	{ (FixPoint1616_t)(0.1 * 65536) ,
	(FixPoint1616_t)(60 * 65536),
	33000,
	18,
	14 },//长距离

	{ (FixPoint1616_t)(0.25 * 65536) ,
	(FixPoint1616_t)(32 * 65536),
	20000,
	14,
	10 },//高速

};


//配置VL53L0X设备I2C地址
//dev:设备I2C参数结构体
//newaddr:设备新I2C地址
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	uint8_t status_uchar = 0;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta = 0x00;

	

	FinalAddress = newaddr;

	if (FinalAddress == dev->I2cDevAddr)//新设备I2C地址与旧地址一致,直接退出
		return VL53L0X_ERROR_NONE;
	//在进行第一个寄存器访问之前设置I2C标准模式(400Khz)
	Status = VL53L0X_WrByte(dev, 0x88, 0x00);
	//Status = IIC_Write_1Byte(dev->I2cDevAddr, 0x88, 0x00);

	if (Status != VL53L0X_ERROR_NONE)
	{
		sta = 0x01;//设置I2C标准模式出错
		goto set_error;
	}

	//尝试使用默认的0x52地址读取一个寄存器
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	//Status = IIC_Read_nByte(dev->I2cDevAddr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, 2, (uint8_t *)(&Id));
	//if (Id == 0xAAEE)
	//{
	//	Id = 0xEEAA;
	//}
	/*交换两个字节*/
	//Id = SWAP_8x8(*(uint8_t *)(&Id), *((uint8_t *)(&Id) + 1));
	//Id = SWAP32(Id);

	if (Status != VL53L0X_ERROR_NONE)
	{
		sta = 0x02;//读取寄存器出错
		goto set_error;
	}
	if (Id == 0xEEAA)
	{
		//设置设备新的I2C地址
		Status = VL53L0X_SetDeviceAddress(dev, FinalAddress);
		//VL53L0X_WrByte(dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, FinalAddress / 2);
		//Status = IIC_Write_1Byte(dev->I2cDevAddr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, FinalAddress /*/ 2*/);

		if (Status != VL53L0X_ERROR_NONE)
		{
			sta = 0x03;//设置I2C地址出错
			goto set_error;
		}

		//修改参数结构体的I2C地址
		dev->I2cDevAddr = FinalAddress;
		//检查新的I2C地址读写是否正常
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		//Status = IIC_Read_nByte(dev->I2cDevAddr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, 2, (uint8_t *)(&Id));

		if (Status != VL53L0X_ERROR_NONE)
		{
			sta = 0x04;//新I2C地址读写出错
			goto set_error;
		}
	}
set_error:
	if (Status != VL53L0X_ERROR_NONE)
	{
		//print_pal_error(Status);//打印错误信息
	}
	if (sta != 0)
	{
		/*printf("sta:0x%x\r\n",sta)*/;
	}
	return Status;
}
//初始化vl53l0x
//dev:设备I2C参数结构体
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = dev;
	GPIO_InitTypeDef  GPIO_InitStructure;
	//   RCC->APB2ENR|=1<<2;     //使能PORTA时钟 
	//GPIOA->CRH&=0X0FFFFFFF;	//PA15设置成推挽输出	  
	//GPIOA->CRH|=0X30000000; 
	//JTAG_Set(SWD_ENABLE);	//禁止JTAG,从而PA15可以做普通IO使用,否则PA15不能做普通IO!!!
	/*-------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //PB8设置成推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*-------------------------------------------------------------------*/
	pMyDevice->I2cDevAddr = VL53L0X_Addr;//I2C地址(上电默认0x52)
	pMyDevice->comms_type = 1;           //I2C通信模式
	pMyDevice->comms_speed_khz = 400;    //I2C通信速率

										 //VL53L0X_i2c_init();//初始化IIC总线
	I2c_Soft_Init();

	I2C_FastMode = 0;

	//VL53L0X_Xshut=0;//失能VL53L0X
	GPIOB->BRR = GPIO_Pin_8;
	Delay_ms(30);
	//VL53L0X_Xshut=1;//使能VL53L0X,让传感器处于工作
	GPIOB->BSRR = GPIO_Pin_8;
	Delay_ms(30);

	Status = vl53l0x_Addr_set(pMyDevice, (SET_VL53L0X_Addr >>1));//设置VL53L0X传感器I2C地址
	if (Status != VL53L0X_ERROR_NONE) goto error;

	Status = VL53L0X_DataInit(pMyDevice);//设备初始化
	if (Status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);

	Status = VL53L0X_GetDeviceInfo(pMyDevice, &vl53l0x_dev_info);//获取设备ID信息
	if (Status != VL53L0X_ERROR_NONE) goto error;

	//AT24CXX_Read(0,(u8*)&Vl53l0x_data,sizeof(_vl53l0x_adjust));//读取24c02保存的校准数据,若已校准 Vl53l0x_data.adjustok==0xAA

	if (Vl53l0x_data.adjustok == 0xAA)//已校准
		AjustOK = 1;
	else //没校准	
		AjustOK = 0;

error:
	if (Status != VL53L0X_ERROR_NONE)
	{
		//print_pal_error(Status);//打印错误信息
		return Status;
	}

	return Status;
}

/*------------------------------------测量--------------------------------------*/
//vl53l0x普通测量模式测试
//dev:设备I2C参数结构体
void vl53l0x_general(VL53L0X_Dev_t *dev)
{
	u8 mode = 0;
	//mode = 0 ;"Default        "默认
	//mode = 1 ;"High Accuracy  "高精度
	//mode = 2 ;"Long Range     "长距离
	//mode = 3 ;"High Speed     "高速

	vl53l0x_general_start(dev, mode);
    //delay_ms(50);

}
//启动普通测量
//dev：设备I2C参数结构体
//mode模式配置 0:默认;1:高精度;2:长距离
void vl53l0x_general_start(VL53L0X_Dev_t *dev, u8 mode)
{
	static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;//工作状态


	//mode_string(mode, buf);//显示当前配置的模式
	while (vl53l0x_set_mode(dev, mode))//配置测量模式
	{

	} 

	if (Status == VL53L0X_ERROR_NONE)
	{
		Status = vl53l0x_start_single_test(dev, &vl53l0x_data, buf);//执行一次测量

		//LCD_ShowxNum(110, 140 + 90, Distance_data, 4, 16, 0);
	
		//printf("d: %4imm\r\n", Distance_data);//打印测量距离
	}
	//delay_ms(50);

}

//vl53l0x复位函数
//dev:设备I2C参数结构体
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//保存设备原I2C地址
	//VL53L0X_Xshut = 0;//失能VL53L0X
	GPIOB->BRR = GPIO_Pin_8;
	Delay_ms(30);
	//VL53L0X_Xshut = 1;//使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)
	GPIOB->BSRR = GPIO_Pin_8;
	Delay_ms(30);
	dev->I2cDevAddr = VL53L0X_Addr;
	vl53l0x_Addr_set(dev, addr);//设置VL53L0X传感器原来上电前原I2C地址
	VL53L0X_DataInit(dev);
}
//VL53L0X 测量模式配置
//dev:设备I2C参数结构体
//mode: 0:默认;1:高精度;2:长距离
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev, u8 mode)
{

	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;

	vl53l0x_reset(dev);//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)

	status = VL53L0X_StaticInit(dev);

	if (AjustOK != 0)//已校准好了,写入校准值
	{
		status = VL53L0X_SetReferenceSpads(dev, Vl53l0x_data.refSpadCount, Vl53l0x_data.isApertureSpads);//设定Spads校准值
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
		status = VL53L0X_SetRefCalibration(dev, Vl53l0x_data.VhvSettings, Vl53l0x_data.PhaseCal);//设定Ref校准值
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
		status = VL53L0X_SetOffsetCalibrationDataMicroMeter(dev, Vl53l0x_data.OffsetMicroMeter);//设定偏移校准值
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
		status = VL53L0X_SetXTalkCompensationRateMegaCps(dev, Vl53l0x_data.XTalkCompensationRateMegaCps);//设定串扰校准值
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);

	}
	else
	{
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref参考校准
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//执行参考SPAD管理
		if (status != VL53L0X_ERROR_NONE) goto error;
		Delay_ms(2);
	}
	status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);//使能单次测量模式
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);//使能SIGMA范围检查
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);//使能信号速率范围检查
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Mode_data[mode].sigmaLimit);//设定SIGMA范围
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Mode_data[mode].signalLimit);//设定信号速率范围范围
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, Mode_data[mode].timingBudget);//设定完整测距最长时间
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//设定VCSEL脉冲周期
	if (status != VL53L0X_ERROR_NONE) goto error;
	Delay_ms(2);
	status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//设定VCSEL脉冲周期范围

error://错误信息
	if (status != VL53L0X_ERROR_NONE)
	{
		//print_pal_error(status);
		//LCD_Fill(30, 140 + 20, 300, 300, WHITE);
		return status;
	}
	return status;

}
//VL53L0X 单次距离测量函数
//dev:设备I2C参数结构体
//pdata:保存测量数据结构体
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *pdata, char *buf)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;

	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//执行单次测距并获取测距测量数据
	if (status != VL53L0X_ERROR_NONE) return status;

	RangeStatus = pdata->RangeStatus;//获取当前测量状态
	memset(buf, 0x00, VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus, buf);//根据测量状态读取状态字符串

	Distance_data = pdata->RangeMilliMeter;//保存最近一次测距测量数据

	return status;
}

//VL53L0X 单次距离测量函数 2
//dev:设备I2C参数结构体
//pdata:保存测量数据结构体
VL53L0X_Error vl53l0x_start_SingleMeasure(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *pdata)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;

	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//执行单次测距并获取测距测量数据
	if (status != VL53L0X_ERROR_NONE) return status;

	RangeStatus = pdata->RangeStatus;                           //获取当前测量状态(0:Range valid / 1:Sigma fail / 2:Signal fail / 3:min range fail / 4:phase fail / 5:hardware fail)

	Distance_data = pdata->RangeMilliMeter;                     //保存最近一次测距测量数据

	Distance_state = RangeStatus;								//保存最近一次测距测量数据的状态

	return status;
}
/*----------------------------------------------------------------------------------*/
/*-------------------------------------校准-----------------------------------------*/
//vl53l0x校准测试
//dev:设备I2C参数结构体
void vl53l0x_calibration(VL53L0X_Dev_t *dev)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	status = vl53l0x_adjust(dev);     //进入校准
	if (status != VL53L0X_ERROR_NONE) //校准失败
	{

	}

	//delay_ms(500);

}

//VL53L0X校准函数
//dev:设备I2C参数结构体
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

	VL53L0X_StaticInit(dev);//数值恢复默认,传感器处于空闲状态
/*-------------------------------SPADS校准------------------------------*/
spads:
	Delay_ms(10);

	Status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//执行参考Spad管理
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
/*-------------------------------------------设备参考校准---------------------------------------------------*/
ref:
	Delay_ms(10);
	Status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref参考校准
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
/*---------------------------------------------偏移校准------------------------------------------------*/
offset:
	Delay_ms(10);
	//printf("Offset Calibration:need a white target,in black space,and the distance keep 100mm!\r\n");
	//printf("The Offset Calibration Start...\r\n");
	Status = VL53L0X_PerformOffsetCalibration(dev, CalDistanceMilliMeter, &OffsetMicroMeter);//偏移校准
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
/*----------------------------------------------串扰校准-----------------------------------------------------*/
xtalk:
	Delay_ms(20);
	//printf("Cross Talk Calibration:need a grey target\r\n");
	//printf("The Cross Talk Calibration Start...\r\n");
	Status = VL53L0X_PerformXTalkCalibration(dev, XTalkCalDistance, &XTalkCompensationRateMegaCps);//串扰校准
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

	Vl53l0x_adjust.adjustok = 0xAA;//校准成功
	//AT24CXX_Write(0, (u8*)&Vl53l0x_adjust, sizeof(_vl53l0x_adjust));//将校准数据保存到24c02
	memcpy(&Vl53l0x_data, &Vl53l0x_adjust, sizeof(_vl53l0x_adjust));//将校准数据复制到Vl53l0x_data结构体
	return Status;
}
/*----------------------------------------------------------------------------------*/


#endif

