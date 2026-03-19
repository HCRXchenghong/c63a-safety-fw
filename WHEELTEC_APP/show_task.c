#include "show_task.h"

//C Include File

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_oled.h"
#include "bsp_gamepad.h"
#include "bsp_key.h"
#include "bsp_RTOSdebug.h"
#include "bsp_adc.h"
#include "bsp_icm20948.h"
#include "bsp_RGBLight.h"
#include "bsp_ServoDrive.h"

//APP Include File
#include "sensor_ranger.h"
#include "xbox360_gamepad.h"
#include "rc_joystick.h"
#include "RobotControl_task.h"
#include "robot_select_init.h"
#include "AutoRecharge_task.h"

static uint8_t page = 0;
extern uint8_t Received_IP[4];           // 确认接收的IP
static pOLEDInterface_t oled = &UserOLED;


void oled_page_up(void)
{
	page++;
}

void oled_page_down(void)
{
	page--;
}

static void ServoDriverInfo_Show(ServoDriverHWType_t* driver)
{
	oled->ShowString(0,0,"  id:");
	oled->ShowNumber(40,0,driver->NodeID,2,12);
	
	oled->ShowString(70,0,"OL:");
	oled->ShowNumber(96,0,driver->onlineFlag,1,12);
	
	oled->ShowString(0,12," ver:");
	oled->ShowNumber(45,12,driver->SoftWareVer,5,12);
	
	oled->ShowString(96,12,"M");
	oled->ShowNumber(104,12,driver->motorEnflag,1,12);
	
	oled->ShowString(0,24," err:");
	oled->ShowNumber(45,24,driver->errFlag,1,12);
	
	oled->ShowString(60,24,"can:");
	oled->ShowNumber(92,24,driver->comm_qua,3,12);
	
	oled->ShowString(0,36,"Lerr:");
	oled->ShowNumber(50,36,driver->LeftMotorErr,5,12);
	
	oled->ShowString(0,48,"Rerr:");
	oled->ShowNumber(50,48,driver->RightMotorErr,5,12);
	
	oled->RefreshGram();
	
}

static void HardWare_Info_show(void)
{
	//车型
	oled->ShowString(0,0,"CarType:");
	     if( RobotHardWareParam.CarType == S300 ) oled->ShowString(70,0,"S300  ");
	else if( RobotHardWareParam.CarType == S200 ) oled->ShowString(70,0,"S200  ");
	else if( RobotHardWareParam.CarType == S200_OUTDOOR ) oled->ShowString(70,0,"ODS200");
	else if( RobotHardWareParam.CarType == S300Mini ) oled->ShowString(70,0,"MNS300");
	else if( RobotHardWareParam.CarType == S100 ) oled->ShowString(70,0,"S100  ");
	else if( RobotHardWareParam.CarType == S260 ) oled->ShowString(70,0,"S260  ");
	else if( RobotHardWareParam.CarType == SX04 ) oled->ShowString(70,0,"SX04  ");
	else oled->ShowString(70,0,"UKOW  ");
	
	//硬件版本
	oled->ShowString(0,12," HW_Ver:");
	if( get_HardWareVersion() == HW_1_0 )
		oled->ShowString(70,12,"1.0");
	else if( get_HardWareVersion() == HW_1_1 )
		oled->ShowString(70,12,"1.1");
	
	//驱动器个数
	oled->ShowString(0,24," DriveN:");
	oled->ShowNumber(70,24,RobotHardWareParam.driveCounts,3,12);
	
	oled->ShowString(0,36,"wheelTp:");
	switch(RobotHardWareParam.wheeltype)
	{
		case SingleAxis_5inch:
			oled->ShowString(70,36,"5-S");
			break;
		case DoubleAxis_5inch:
			oled->ShowString(70,36,"5-D");
			break;
		case SingleAxis_8inch:
			oled->ShowString(70,36,"8-S");
			break;
		case DoubleAxis_8inch:
			oled->ShowString(70,36,"8-D");
			break;
	}
	//IP显示
	oled->ShowString(0,48,"IP:");
	oled->ShowNumber(24,48,Received_IP[0],3,12);
	oled->ShowNumber(48,48,Received_IP[1],3,12);
	oled->ShowNumber(72,48,Received_IP[2],3,12);
	oled->ShowNumber(96,48,Received_IP[3],3,12);
	
	oled->RefreshGram();
}

static void ChargerDev_Info_show(void)
{
	oled->ShowString(07,00,"LA  LB  RB  RA");
	oled->ShowNumber(0+9,10,ChargeDev.L_A,1,12);
	oled->ShowNumber(30+9,10,ChargeDev.L_B,1,12);
	oled->ShowNumber(60+9,10,ChargeDev.R_B,1,12);
	oled->ShowNumber(90+9,10,ChargeDev.R_A,1,12);
	
	oled->ShowString(0,23,"cur:"); 
	oled->ShowString(75,23,"A"); 
	oled->ShowFloat(30,23,ChargeDev.ChargingCur/1000.0f,2,2);
	
	oled->ShowString(0,36,"st:");
	oled->ShowNumber(30,36,ChargeDev.ChargingFlag,1,12);
	
	oled->ShowString(70,36,"OL:");
	oled->ShowNumber(96,36,ChargeDev.online,1,12);
	
	oled->ShowString(0,50,"rcm:");
	oled->ShowNumber(35,50,RobotControlParam.ChargeMode,1,12);
	
	oled->RefreshGram();
}

static void RobotMainInfoShow(void)
{
	//首行,所有车型固定显示
	uint16_t Divisor_Mode = 4095/(Number_of_CAR-1);
	uint8_t cartypeshow = USER_ADC_Get_AdcBufValue(userconfigADC_CARMODE_CHANNEL)/Divisor_Mode;
	
	      if(RobotControlParam.EmergencyMode) oled->ShowString(0,0,"EMC on");
	else if( RobotControlParam.ChargeMode ) oled->ShowString(0,0,"RCM   ");
	else if( cartypeshow == S300 ) oled->ShowString(0,0,"S300  ");
	else if( cartypeshow == S200 ) oled->ShowString(0,0,"S200  ");
	else if( cartypeshow == S200_OUTDOOR ) oled->ShowString(0,0,"ODS200");
	else if( cartypeshow == S300Mini ) oled->ShowString(0,0,"MNS300");
	else if( cartypeshow == S100 ) oled->ShowString(0,0,"S100  ");
	else if( cartypeshow == S260 ) oled->ShowString(0,0,"S260  ");
	else if( cartypeshow == SX04 ) oled->ShowString(0,0,"SX04  ");
	else oled->ShowString(0,0,"UKOW  ");
	
	oled->ShowString(55,0,"Cur:");
	oled->ShowFloat(87,0,ChargeDev.ChargingCur/1000.0f,1,2);
	oled->ShowString(120,0,"A");
	
	//第2-4行,不同车型显示不一样的信息
	//六轮车
	if( RobotHardWareParam.CarType == S260 )
	{
		//左右轮目标值
		oled->ShowString(0,10,"L");
		oled->ShowShort(15,10,rpm_to_linearVel(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter)*1000,4,12);
		oled->ShowString(60,10,"R");
		oled->ShowShort(70,10,rpm_to_linearVel(-RobotControlParam.MotorD.target,RobotHardWareParam.WheelPerimeter)*1000,4,12);
		
		//6轮实际值
		oled->ShowString(0,20,"C");
		oled->ShowString(0,30,"B");
		oled->ShowString(0,40,"A");
		oled->ShowString(60,20,"D");
		oled->ShowString(60,30,"E");
		oled->ShowString(60,40,"F");
		oled->ShowShort(15,20,RobotControlParam.MotorC.feedback*1000,4,12);
		oled->ShowShort(15,30,RobotControlParam.MotorB.feedback*1000,4,12);
		oled->ShowShort(15,40,RobotControlParam.MotorA.feedback*1000,4,12);

		oled->ShowShort(70,20,RobotControlParam.MotorD.feedback*1000,4,12);
		oled->ShowShort(70,30,RobotControlParam.MotorE.feedback*1000,4,12);
		oled->ShowShort(70,40,RobotControlParam.MotorF.feedback*1000,4,12);
	}
	
	//四驱车
	else if( RobotHardWareParam.CarType == S200 || RobotHardWareParam.CarType == S200_OUTDOOR )
	{
		oled->ShowString(0,10,"A:");
		oled->ShowString(0,20,"B:");
		oled->ShowString(0,30,"C:");
		oled->ShowString(0,40,"D:");
		
		oled->ShowShort(15,10,rpm_to_linearVel(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter)*1000,5,12);
		oled->ShowShort(75,10,RobotControlParam.MotorA.feedback*1000,5,12);
		oled->ShowShort(15,20,rpm_to_linearVel(RobotControlParam.MotorB.target,RobotHardWareParam.WheelPerimeter)*1000,5,12);
		oled->ShowShort(75,20,RobotControlParam.MotorB.feedback*1000,5,12);
		oled->ShowShort(15,30,rpm_to_linearVel(-RobotControlParam.MotorC.target,RobotHardWareParam.WheelPerimeter)*1000,5,12);
		oled->ShowShort(75,30,RobotControlParam.MotorC.feedback*1000,5,12);
		oled->ShowShort(15,40,rpm_to_linearVel(-RobotControlParam.MotorD.target,RobotHardWareParam.WheelPerimeter)*1000,5,12);
		oled->ShowShort(75,40,RobotControlParam.MotorD.feedback*1000,5,12);
	}

	//差速车
	else if( RobotHardWareParam.CarType == S300 || RobotHardWareParam.CarType == S300Mini || 
		     RobotHardWareParam.CarType == S100 )
	{
		oled->ShowString(0,10,"GYRO_Z");
		oled->ShowShort(65,10,axis_9ValOri.gyro.z,5,12);
		
		oled->ShowString(0,20,"L:");
		oled->ShowShort(15,20,rpm_to_linearVel(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter)*1000,5,12);
		oled->ShowShort(75,20,RobotControlParam.MotorA.feedback*1000,5,12);
		
		oled->ShowString(0,30,"R:");
		oled->ShowShort(15,30,rpm_to_linearVel(-RobotControlParam.MotorB.target,RobotHardWareParam.WheelPerimeter)*1000,5,12);
		oled->ShowShort(75,30,RobotControlParam.MotorB.feedback*1000,5,12);
		
		oled->ShowString(0,40,"RPM");
		oled->ShowShort(30,40,RobotControlParam.MotorA.target,4,12);
		oled->ShowShort(80,40,-RobotControlParam.MotorB.target,4,12);
	}
	

	//最后一行,所有车型固定显示
	//控制方式,使能状态,电池电压
	if( RobotControl_CMDsource==NONE_CMD )          oled->ShowString(0,50,"ROS ");
	else if( RobotControl_CMDsource==GamePad_CMD )  oled->ShowString(0,50,"PS2 ");
	else if( RobotControl_CMDsource==ROS_CMD )      oled->ShowString(0,50,"ROS ");
	else if( RobotControl_CMDsource==RCJOY_CMD )    oled->ShowString(0,50,"R-C ");
	else if( RobotControl_CMDsource==APP_CMD )      oled->ShowString(0,50,"APP ");
	else if( RobotControl_CMDsource==BootLoader )   oled->ShowString(0,50,"boot");
	else if( RobotControl_CMDsource==Charger_CMD )   oled->ShowString(0,50,"RCM ");
	else if( RobotControl_CMDsource==CAN_CMD )   oled->ShowString(0,50,"CAN ");
	else oled->ShowString(0,50,"UKOW");
	
	if( RobotControlParam.en_flag ) oled->ShowString(45,50," ON");
	else oled->ShowString(45,50,"OFF");
	
	oled->ShowFloat(70,50,RobotControlParam.Vol,2,2);
	oled->ShowString(112,50,"V");
	oled->ShowString(70,50," ");
	oled->RefreshGram();
	
}

//超声波显示
static void RangerSensorShow(void)
{
	oled->ShowString(0,0,"A");
	oled->ShowString(0,10,"B");
	oled->ShowString(0,20,"C");
	oled->ShowString(0,30,"D");
	oled->ShowString(0,40,"E");
	oled->ShowString(0,50,"F");
	
	oled->ShowFloat(15,0,RangerHAL_A.dis,3,2);
	oled->ShowFloat(15,10,RangerHAL_B.dis,3,2);
	oled->ShowFloat(15,20,RangerHAL_C.dis,3,2);
	oled->ShowFloat(15,30,RangerHAL_D.dis,3,2);
	oled->ShowFloat(15,40,RangerHAL_E.dis,3,2);
	oled->ShowFloat(15,50,RangerHAL_F.dis,3,2);

	oled->ShowNumber(80,0,RangerHAL_A.timeoutCount,3,12);
	oled->ShowNumber(80,10,RangerHAL_B.timeoutCount,3,12);
	oled->ShowNumber(80,20,RangerHAL_C.timeoutCount,3,12);
	oled->ShowNumber(80,30,RangerHAL_D.timeoutCount,3,12);
	oled->ShowNumber(80,40,RangerHAL_E.timeoutCount,3,12);
	oled->ShowNumber(80,50,RangerHAL_F.timeoutCount,3,12);

	oled->RefreshGram();
}

static void JoyShow(void)
{
	oled->ShowString(0,0,"LX:");
	oled->ShowString(0,10,"LY:");
	oled->ShowString(0,20,"RX:");
	oled->ShowString(0,30,"RY:");
	
	oled->ShowNumber(30,0,GamePadInterface->LX,3,12);
	oled->ShowNumber(30,10,GamePadInterface->LY,3,12);
	oled->ShowNumber(30,20,GamePadInterface->RX,3,12);
	oled->ShowNumber(30,30,GamePadInterface->RY,3,12);
	
	oled->ShowNumber(70,0,rc_remote.LX,4,12);
	oled->ShowNumber(70,10,rc_remote.LY,4,12);
	oled->ShowNumber(70,20,rc_remote.RX,4,12);
	oled->ShowNumber(70,30,rc_remote.RY,4,12);
	oled->ShowNumber(70,40,rc_remote.freq,4,12);
	
	oled->ShowString(0,50,"ENkey:");
	if( get_EnKeyState()==1 )
		oled->ShowString(50,50," ON");
	else
		oled->ShowString(50,50,"OFF");

	oled->RefreshGram();
}

static void ImuShow(void)
{
	extern IMU_DATA_t axis_9Val ;      
	extern IMU_DATA_t axis_9ValOri;  
	extern ATTITUDE_DATA_t AttitudeVal;
	
	oled->ShowFloat(0,0,axis_9Val.gyro.x*57.29578f,3,2);
	oled->ShowFloat(0,10,axis_9Val.gyro.y*57.29578f,3,2);
	oled->ShowFloat(0,20,axis_9Val.gyro.z*57.29578f,3,2);

	oled->ShowFloat(70,0,axis_9Val.accel.x,3,2);
	oled->ShowFloat(70,10,axis_9Val.accel.y,3,2);
	oled->ShowFloat(70,20,axis_9Val.accel.z,3,2);
	
	oled->ShowFloat(0,35,AttitudeVal.pitch,3,2);
	oled->ShowFloat(70,35,AttitudeVal.roll,3,2);
	oled->ShowFloat(0,45,AttitudeVal.yaw,3,2);
	
	oled->RefreshGram();
}

void show_task(void* param)
{
	uint8_t taskfreq = 0;
	
	uint8_t page_max = 9;
	
	pRTOSDebugInterface_t debug = &RTOSDebugTimer;
	RTOSDebugPrivateVar debugvar = { 0 };
	
	//不同车型驱动个数不一样,对应不同的可显示OLED页数
	switch(RobotHardWareParam.CarType)
	{
		case S260:
			page_max = 9;
			break;
		case S200: case S200_OUTDOOR:
			page_max = 8;
			break;
		case S300:case S300Mini:case S100:
			page_max = 7;
			break;
	}
	
	static uint8_t LastPage = 0;
	static uint32_t LedTickCnt = 0;
	while(1)
	{
		//OLED页数更变时刷新屏幕
		if( page!=LastPage ) oled->Clear();
		if( page >= page_max )
		{
			page = 0;
			continue;
		}
		LastPage = page;
		
		taskfreq = debug->UpdateFreq(&debugvar);
		
		pKeyInterface_t key = &UserKey;
		UserKeyState_t ks = key->getKeyState(taskfreq);
		if( ks == single_click ) page++;
		else if( ks == double_click ) page--;
		
		if( page==0 ) RobotMainInfoShow();
		else if(page==1) ImuShow();
		else if( page==2 ) ChargerDev_Info_show();
		else if( page==3 ) RangerSensorShow();
		else if( page==4 ) JoyShow();
		else if( page==5 ) HardWare_Info_show();
		else if( page==6 ) ServoDriverInfo_Show(&Driver1);
		else if( page==7 ) ServoDriverInfo_Show(&Driver2);
		else if( page==8 ) ServoDriverInfo_Show(&Driver3);
		
		if( RobotControlParam.LedTickState==2 )
		{	//标零完成时LED灯慢速闪烁
			LedTickCnt++;
			if( LedTickCnt>= taskfreq/2 ) LedTickCnt=0,HAL_GPIO_TogglePin(UserLED_GPIO_Port,UserLED_Pin);
		}
		//未标零时快速闪烁
		else HAL_GPIO_TogglePin(UserLED_GPIO_Port,UserLED_Pin);
		
		vTaskDelay(50);
	}
}
