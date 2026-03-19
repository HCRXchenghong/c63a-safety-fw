#include "robot_select_init.h"
#include "bsp_adc.h"

//定义机器人参数
RobotParmentType_t RobotHardWareParam = { 0 };

//机器人的控制软件参数
RobotControlParmentType_t RobotControlParam = { 
	.en_flag = 1,          //小车使能标志位
	.ErrNum = 0,           //小车报错码
	.defalutSpeed = 500,   //小车默认遥控速度,单位mm/s
	.ChargeMode = 0,       //自动回充模式状态,默认关闭
	.EmergencyMode = 0,    //应急模式,默认关闭
	.softwareEnflag = 0,   //软件使能位,默认使能
	.LineDiffParam = 50,   //纠偏系数,用于调整走直线效果
	.RangerAvoidEN = 0 ,   //超声波安全保护位
	.RangerHardStopState = 0,
	.RangerHardStopMode = 0,
	.RangerDangerPending = 0,
	.RangerDangerMask = 0,
	.SecurityLevel = 0 ,   //安全等级,0为最高
	.LowPowerFlag = 0  ,   //小车电量低标志位 1表示低电量
	.ImuAssistedFlag = 1,  //默认启用小车走直线时用IMU辅助功能
	.DebugLevel = 0        //调试等级
};

static void Robot_Init(float wheelspacing,float axlespacing,float tyre_diameter,uint8_t WheelType);

void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
	//ADC值分段变量，取决于小车型号数量
	uint16_t Divisor_Mode = 4095/(Number_of_CAR-1);
	uint8_t selectedCarType = USER_ADC_Get_AdcBufValue(userconfigADC_CARMODE_CHANNEL)/Divisor_Mode;
	
	//先写入安全默认值,再根据车型补充实际参数
	RobotHardWareParam.CarType = selectedCarType;
	RobotHardWareParam.CarTypeSupported = 0;
	RobotHardWareParam.driveCounts = 0;
	RobotHardWareParam.WheelSpacing = 0.0f;
	RobotHardWareParam.AxleSpacing = 0.0f;
	RobotHardWareParam.WheelPerimeter = 1.0f;
	RobotHardWareParam.wheeltype = SingleAxis_5inch;
	if( selectedCarType >= Number_of_CAR )
		RobotHardWareParam.CarType = Number_of_CAR;
	
	//根据车型配置设置不同的参数:轮距、轴距、轮子直径、轮子型号
	if( RobotHardWareParam.CarType == S300 )
	{
		Robot_Init(S300_Wheelspacing,0,S300_Diameter,DoubleAxis_8inch);
		RobotHardWareParam.CarTypeSupported = 1;
	}
	else if( RobotHardWareParam.CarType == S200 )
	{
		Robot_Init(S200_Wheelspacing,S200_axlespacing,S200_Diameter,SingleAxis_8inch);
		RobotHardWareParam.CarTypeSupported = 1;
	}
	else if( RobotHardWareParam.CarType == S300Mini )
	{
		Robot_Init(S150_Wheelspacing,0,S150_Diameter,DoubleAxis_5inch);
		RobotHardWareParam.CarTypeSupported = 1;
	}
	else if( RobotHardWareParam.CarType == S100 )
	{
		Robot_Init(S100_Wheelspacing,0,S100_Diameter,SingleAxis_5inch);
		RobotHardWareParam.CarTypeSupported = 1;
	}
	
	//S260 6轮车,模型为差速车型
	else if( RobotHardWareParam.CarType == S260 )
	{
		Robot_Init(S260_Wheelspacing,S260_axlespacing,S260_Diameter,SingleAxis_8inch);
		RobotHardWareParam.CarTypeSupported = 1;
	}
	
	//户外版S200
	else if( RobotHardWareParam.CarType == S200_OUTDOOR )
	{
		Robot_Init(S200_OUTDOOR_Wheelspacing,S200_OUTDOOR_axlespacing,S200_OUTDOOR_Diameter,SingleAxis_8inch);
		RobotHardWareParam.CarTypeSupported = 1;
	}
	
	
	//驱动器的个数选择,根据车型不同初始化不同个数的驱动器
	switch( RobotHardWareParam.CarType )
	{
		case S300:case S300Mini:case S100:
			RobotHardWareParam.driveCounts=1;
			break;
		case S200:case S200_OUTDOOR:
			RobotHardWareParam.driveCounts=2;
			break;
		case S260:
			RobotHardWareParam.driveCounts=3;
			break;
		default:
			RobotHardWareParam.driveCounts=0;
			break;
	}

	// 默认仅为S100开启超声波安全保护，其余车型保持原有默认行为
	RobotControlParam.RangerAvoidEN = (RobotHardWareParam.CarTypeSupported == 1U && RobotHardWareParam.CarType == S100) ? 1 : 0;
	RobotControlParam.RangerHardStopState = 0;
	RobotControlParam.RangerHardStopMode = 0;
	RobotControlParam.RangerDangerPending = 0;
	RobotControlParam.RangerDangerMask = 0;
}

static void Robot_Init(float wheelspacing,float axlespacing,float tyre_diameter,uint8_t WheelType) 
{
	RobotHardWareParam.WheelSpacing = wheelspacing;
	RobotHardWareParam.AxleSpacing = axlespacing;
	RobotHardWareParam.WheelPerimeter = tyre_diameter*PI;
	RobotHardWareParam.wheeltype = WheelType;
}

