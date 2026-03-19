/**
 * @file    RobotControl_task.c
 * @brief   小车控制逻辑文件.
 * @author  WHEELTEC
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - 本文件是小车运行的核心内容,由任务RobotControl_task，从控制队列中接收各种遥控或串口的
 *   速度指令信息，使用速度信息进行运动学分析解算后，获得小车各个轮子的转速，进而下发转速
 *   指令到驱动器，完成小车的驱动
 * @note
 * 
 * 
 */
#include "RobotControl_task.h"

//HAL Lib Include File
#include "main.h"

//C Include File
#include <stdio.h>
#include <math.h>

//BSP Include File
#include "bsp_adc.h"
#include "bsp_ServoDrive.h"
#include "bsp_RTOSdebug.h"
#include "bsp_icm20948.h"
#include "bsp_RGBLight.h"
#include "bsp_buzzer.h"


//APP Include File
#include "robot_select_init.h"
#include "sensor_ranger.h"
#include "xbox360_gamepad.h"
#include "show_task.h"

//用于存放小车的控制队列数据
QueueHandle_t  g_xQueueRobotControl = NULL;

//用于记录小车当前的控制来源
volatile uint8_t RobotControl_CMDsource = NONE_CMD; 

//内部使用函数
static float linearVel_to_rpm(float linearVel,float wheelper);
static void Robot_DriverRecovery(void);
static void robot_en_check(void);
static void Robot_DriverDisable(void);
static void Robot_DriverEnable(void);
static void Robot_DriverSetParkingMode(uint8_t setflag);
static void Robot_DriverCheckError(void);
static void Robot_DriverStop(void);
static void Drive_Motor(const RobotControlCMDType_t* target);
static void Robot_Forwardkinematics(RobotControlParmentType_t* robot,const RobotParmentType_t* robot_hw);
static void Smooth_control(RobotControlCMDType_t* cmd,const RobotControlCMDType_t* target,float step);
static void Linear_IMUHelp(RobotControlCMDType_t* target);
static uint8_t Ranger_GetMonitorMode(const RobotControlCMDType_t* target);
static uint8_t Ranger_GetFreshDangerMask(uint8_t mode,uint8_t* updatedMask);
static uint8_t Ranger_GetMonitorMask(uint8_t mode);
static uint8_t Ranger_IsModeSafe(uint8_t mode,float threshold);
static uint8_t Robot_WheelsStopped(void);
static void Ranger_ResetHardStop(void);
static void Ranger_TriggerHardStop(uint8_t mode,uint8_t dangerMask);
static void RangerHardStop_Update(const RobotControlCMDType_t* request,RobotControlCMDType_t* applied);
static uint8_t CountBits_u8(uint8_t value);

//IMU辅助小车走直线变量
static RobotControlCMDType_t LinearHelp_Target = { 0 };

#define RANGER_HARDSTOP_TRIGGER_DIS 0.20f
#define RANGER_HARDSTOP_RELEASE_DIS 0.25f
#define RANGER_WHEEL_STOP_DIS       0.03f
#define RANGER_WHEEL_STOP_FILTER    5U
#define RANGER_LINEAR_EPS           0.02f
#define RANGER_ANGULAR_EPS          0.05f
#define RANGER_HARDSTOP_BUZZER_MS   1000U
#define RANGER_FRONT_MASK           ((uint8_t)0x0F)
#define RANGER_BACK_MASK            ((uint8_t)0x30)
#define RANGER_ALL_MASK             ((uint8_t)0x3F)

static RangerHALObj* const g_RangerSensorList[] = {
	&RangerHAL_A,&RangerHAL_B,&RangerHAL_C,
	&RangerHAL_D,&RangerHAL_E,&RangerHAL_F,
};

static uint32_t g_RangerSampleShadow[sizeof(g_RangerSensorList)/sizeof(g_RangerSensorList[0])] = { 0 };

void RobotControl_task(void* param)
{
	vTaskDelay(1000);
	
	//创建控制指令队列
	g_xQueueRobotControl = xQueueCreate(1,sizeof(RobotControlCMDType_t));
	
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//本任务的控制频率,单位为Hz
	const uint16_t TaskFreq = 100;

	//调试变量
//	pRTOSDebugInterface_t debug = &RTOSDebugTimer;
//	RTOSDebugPrivateVar priv = { 0 };
	
	//伺服电机自检变量
	uint8_t driverCheck = 0;
	
	//空闲计数
	TickType_t ActiveTick = xTaskGetTickCount();
	
	//队列无数据统计
	uint8_t LostCmdCounts = 0;
	
	//蜂鸣器接口
	pBuzzerInterface_t buzzer = &UserBuzzer;
	
	RobotControlCMDType_t cmd = { 0 };
	RobotControlCMDType_t applied_cmd = { 0 };
	while(1)
	{
		//从队列中读取控制指令
		BaseType_t control_queue_state = pdFAIL;
		control_queue_state = xQueueReceive(g_xQueueRobotControl,&cmd,0);
		if( control_queue_state == pdPASS )
			LostCmdCounts=0;
		else 
			LostCmdCounts++;

		// 小车失控保护需要在当前控制周期内立即生效
		if( LostCmdCounts > TaskFreq )
		{
			LostCmdCounts=TaskFreq;
			
			switch( RobotControl_CMDsource )
			{
				//非上位机类型的控制默认需要保护
				//此处原本APP控制也应执行,但为了兼容IOS APP(数据非连续发送),保护无法开启.
				case GamePad_CMD:case RCJOY_CMD: case Charger_CMD:
					cmd.Vx=0;cmd.Vy=0;cmd.Vz=0;
					break;
				
				//上位机类型,根据用户设置的安全级别启用保护
				case ROS_CMD:case CAN_CMD:
					if( RobotControlParam.SecurityLevel==0 )
					{
						cmd.Vx=0;
						cmd.Vy=0;
						cmd.Vz=0;
					}
					break;
			}
		}

		applied_cmd = cmd;
		RangerHardStop_Update(&cmd,&applied_cmd);
		
		//陀螺仪直线补偿
		if( RobotControlParam.ImuAssistedFlag == 1 )
			Linear_IMUHelp(&applied_cmd);
		else
		{
			LinearHelp_Target.Vx=0;LinearHelp_Target.Vy=0;LinearHelp_Target.Vz=0;
		}
			
		//运动学逆解,获取车轮线速度m/s
		Drive_Motor(&applied_cmd);
		
		//小车使能状态检查,根据自检参数判断是否允许启动小车
		robot_en_check();

		//小车允许使能时,根据不同的车型控制电机运动
		static uint8_t ErrTipsFlag = 0;
		if( RobotControlParam.en_flag == 1 )
		{	
			switch( RobotHardWareParam.CarType )
			{
				case S260: //6轮车
					set_MotorRPM(&Driver1,RobotControlParam.MotorA.target,RobotControlParam.MotorF.target);
					set_MotorRPM(&Driver2,RobotControlParam.MotorB.target,RobotControlParam.MotorE.target);
					set_MotorRPM(&Driver3,RobotControlParam.MotorC.target,RobotControlParam.MotorD.target);
					break;
				case S200:case S200_OUTDOOR: //四驱车
					set_MotorRPM(&Driver1,RobotControlParam.MotorA.target,RobotControlParam.MotorD.target);
					set_MotorRPM(&Driver2,RobotControlParam.MotorB.target,RobotControlParam.MotorC.target);
					break;
				case S300: case S100: //两轮差速
					set_MotorRPM(&Driver1,RobotControlParam.MotorA.target,RobotControlParam.MotorB.target);
					break;
				case S300Mini:
					set_MotorRPM(&Driver1,-RobotControlParam.MotorA.target,-RobotControlParam.MotorB.target);
					break;
			}
			
			//允许使能后,关闭报错提示
			if( ErrTipsFlag==1 )
			{
				ErrTipsFlag = 0;
				//可加入取消报错后操作
			}
		}
		else //小车被失能,发送0转速停止小车
		{
			Robot_DriverStop();
			
			//小车被失能时收到控制命令,蜂鸣器提醒报错
			//if( control_queue_state == pdPASS ) buzzer->AddTask(3,100);
			
			//被失能后,启动报错提示.
			if( ErrTipsFlag == 0 )
			{
				ErrTipsFlag=1;
				//可加入报错后操作
			}
		}
		
		//运动学正解,根据车轮速度计算三轴速度
		Robot_Forwardkinematics(&RobotControlParam,&RobotHardWareParam);
		
		//判断当前是否有活跃的控制量
		if( cmd.Vx!=0 || cmd.Vy!=0 || cmd.Vz!=0 )
		{
			ActiveTick = xTaskGetTickCount();
			if( RobotControlParam.ParkingMode==1 )
			{
				RobotControlParam.ParkingMode=0;
				//驻车模式解除
				Robot_DriverSetParkingMode(0);
			}
		}
		
		//根据控制量是否活跃来判断是否进入驻车模式,若小车停止10秒后无控制量,将进入驻车模式省电
		if( RobotControlParam.ParkingMode==0 && xTaskGetTickCount() - ActiveTick > 10000 )
		{
			//根据imu角度判断是否位于斜坡上
			if(fabs(AttitudeVal.pitch) < 10.0f && fabs(AttitudeVal.roll)<10.0f )
			{
				RobotControlParam.ParkingMode=1;
				//发送驻车指令
				Robot_DriverSetParkingMode(1);
			}
			else
			{	//小车位于斜坡上,刷新用于判断的时间
				ActiveTick = xTaskGetTickCount();
			}
		}
		
		//驱动器低频获取自检参数,包含电机报错码,驱动报错情况
		driverCheck++;
		if( driverCheck==10 )
		{
			driverCheck=0;
			Robot_DriverCheckError();
		}
		
		//驱动报错恢复请求,驱动器报错后,用户可通过此标志位请求尝试清除报错
		if( RobotControlParam.DriveErrRecovery )
		{
			RobotControlParam.DriveErrRecovery = 0;
			Robot_DriverRecovery();
		}
		
		//小车低电量蜂鸣器提示,仅无灯带车型使用蜂鸣器提示
		static uint32_t LowPowerTipsCnt = 0;
		switch( RobotHardWareParam.CarType )
		{
			case S200:case S200_OUTDOOR:case S260:
				if( RobotControlParam.LowPowerFlag )
				{
					LowPowerTipsCnt++;
					if( LowPowerTipsCnt>TaskFreq*4 )
					{
						LowPowerTipsCnt=0;
						buzzer->AddTask(1,750);
					}
				}
				break;
			default:
				break;
		}
		
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

//多模智控手柄按键回调函数,均与小车功能相关
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	pBuzzerInterface_t buzzer = &UserBuzzer;
	
	//LB左上按键：减速
	if( keyid == Xbox360KEY_LB )
	{
		RobotControlParam.defalutSpeed -= 50;
		if( RobotControlParam.defalutSpeed<0 ) RobotControlParam.defalutSpeed=0;
		buzzer->AddTask(1,200);
	}
	
	//RB右上按键：加速
	if( keyid == Xbox360KEY_RB )
	{
		RobotControlParam.defalutSpeed += 50;
		if( RobotControlParam.defalutSpeed>3000 ) RobotControlParam.defalutSpeed=3000;
		buzzer->AddTask(1,200);
	}
	
	//右摇杆Rjoy:IMU标零
	if( keyid == Xbox360KEY_RJoy && event == GamePadKeyEvent_LONGCLICK)
	{
		pIMUInterface_t imu = &UserICM20948;
		imu->UpdateZeroPoint_axis();
		imu->UpdateZeroPoint_attitude();
		buzzer->AddTask(1,200);//使用蜂鸣器提示操作成功
	}
		
	//Y:自动回充开启/关闭
	if( keyid == Xbox360KEY_Y && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.ChargeMode = !RobotControlParam.ChargeMode;
		buzzer->AddTask(1,200);
	}
	
	//B:清除电机报错请求
	if( keyid == Xbox360KEY_B && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.DriveErrRecovery=1;
		buzzer->AddTask(1,200);
	}
	
	//A:避障请求
	if( keyid == Xbox360KEY_A && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.RangerAvoidEN = !RobotControlParam.RangerAvoidEN;
		buzzer->AddTask(1,200);
	}
	
	//X:IMU辅助走直线开关
	if( keyid==Xbox360KEY_X && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.ImuAssistedFlag = !RobotControlParam.ImuAssistedFlag;
		buzzer->AddTask(1,200);
	}
	
	//左摇杆LJoy:应急模式
	if( keyid == Xbox360KEY_LJoy && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.EmergencyMode = !RobotControlParam.EmergencyMode;
		buzzer->AddTask(1,200);
	}
	
	//方向按键,左右对应OLED翻页
	if( keyid == Xbox360KEY_LEFT )
	{
		oled_page_down();
		buzzer->AddTask(1,200);
	}
	
	if( keyid == Xbox360KEY_RIGHT )
	{
		oled_page_up();
		buzzer->AddTask(1,200);
	}
}

//线速度m/s转RPM转速
static float linearVel_to_rpm(float linearVel,float wheelper)
{
	return linearVel*60.0f/wheelper;
}

//驱动报错恢复函数
static void Robot_DriverRecovery(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		//如果电机存在报错,则清除
		if(g_ServoDriveList[i]->errFlag==1)
		{
			ServoDriver_ClearError(g_ServoDriveList[i]);
		}
			
		//根据使能开关的状态确定是否要使能电机
		if( RobotControlParam.Enkeystate==1 ) ServoDriver_Enable(g_ServoDriveList[i]);
		else ServoDriver_Disable(g_ServoDriveList[i]);
		
		//转速清零
		set_MotorRPM(g_ServoDriveList[i],0,0);
	}
}

//设置驻车模式
static void Robot_DriverSetParkingMode(uint8_t setflag)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_ParkingMode(g_ServoDriveList[i],setflag);
	}
}

//获取驱动报错信息
static void Robot_DriverCheckError(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_CheckError(g_ServoDriveList[i]);
	}
}

//失能电机
static void Robot_DriverDisable(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Disable(g_ServoDriveList[i]);
	}
}

//使能电机
static void Robot_DriverEnable(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Enable(g_ServoDriveList[i]);
	}
}

//电机自锁停转
static void Robot_DriverStop(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		set_MotorRPM(g_ServoDriveList[i],0,0);
	}
}

//小车使能状态检查
static void robot_en_check(void)
{
	//报错码默认为0,根据下列报错情况而设置
	RobotControlParam.ErrNum = 0;
	
	//计算机器人电池电压,电量不足时将禁止小车的控制
	RobotControlParam.Vol = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL)/4095.0f*3.3f*11;

	if( RobotHardWareParam.CarTypeSupported == 0U )
		Set_RobotErrorCode(errCode_UnsupportedCarType);
	
	//软件急停开关状态
	if( RobotControlParam.softwareEnflag == 1 ) Set_RobotErrorCode(errCode_SoftWareStop);
	if( RobotControlParam.RangerHardStopState != RangerStopState_Idle ) Set_RobotErrorCode(errCode_RangerHardStop);
	
	//读取机器人急停开关的状态,按键切换时作出对应操作
	static uint8_t last_EnKey = 1;
	RobotControlParam.Enkeystate = get_EnKeyState();
	if( RobotControlParam.Enkeystate==0 ) Set_RobotErrorCode(errCode_StopKeyEn);
	
	//使能开关按下
	if( RobotControlParam.Enkeystate==0 && last_EnKey==1 )
	{
		Robot_DriverDisable();
	}
	
	//使能开关弹起
	else if( last_EnKey==0 && RobotControlParam.Enkeystate==1 )
	{
		Robot_DriverEnable();
	}
	
	last_EnKey = RobotControlParam.Enkeystate;
	
	//机器人电量低状态获取
	static uint32_t LowPowerCnt=0;
	if( RobotControlParam.Vol>18.0f && RobotControlParam.Vol < 20.5f )
	{
		LowPowerCnt++;
		if( LowPowerCnt==100*10 ) 
		{
			RobotControlParam.LowPowerFlag=1;
		}
	}
	else
	{
		LowPowerCnt=0;
		if( RobotControlParam.Vol>21.5f) RobotControlParam.LowPowerFlag=0;
	}

	//电压不足.(若开启了自动回充模式,则忽略电压)
	if( RobotControlParam.Vol < 20.0f && RobotControlParam.ChargeMode==0 ) Set_RobotErrorCode(errCode_LowPower);
	
	//根据不同的驱动个数生成不同的报错码
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( g_ServoDriveList[i]->errFlag==1 )
		{
			switch(g_ServoDriveList[i]->NodeID)
			{
				case 1: Set_RobotErrorCode(errCode_Driver1_Err); break;
				case 2: Set_RobotErrorCode(errCode_Driver2_Err);break;
				case 3: Set_RobotErrorCode(errCode_Driver3_Err);break;
			}
		}
		
		if( g_ServoDriveList[i]->onlineFlag==0 )
		{
			switch(g_ServoDriveList[i]->NodeID)
			{
				case 1: Set_RobotErrorCode(errCode_Driver1_offline); break;
				case 2: Set_RobotErrorCode(errCode_Driver2_offline); break;
				case 3: Set_RobotErrorCode(errCode_Driver3_offline); break;
			}
		}
	}

	if( RobotHardWareParam.CarTypeSupported == 0U ) RobotControlParam.en_flag = 0;
	else if( RobotControlParam.RangerHardStopState != RangerStopState_Idle ) RobotControlParam.en_flag = 0;
	else if( RobotControlParam.ErrNum!=0 && RobotControlParam.EmergencyMode==0 ) RobotControlParam.en_flag = 0;
	else RobotControlParam.en_flag = 1;
}


/**
 * @brief  此函数用于写控制队列,所有需要控制小车的任务均需要通过此任务写入控制量.
 * @param  cmd:参数需要包含指令来源,以及指定小车的三轴目标速度大小
           woken:如果是在中断中调用,需要传入此参数。非中断中调用直接写入0
 * @retval 0写入成功,1写入失败.写入失败原因：未指定控制指令来源或当前有更高优先级控制在活跃
 *
 * @details
 * - 用户需要新增控制方式时,需要先在RobotControl_task.h头文件新增一个控制类型的枚举并确定优先级,
 *   然后通过 WriteControlQueue 写控制队列进行小车的控制
 */
uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd,BaseType_t* woken)
{
	static uint32_t lastTick = 0;
	uint8_t writeflag = 0;
	
	//队列未创建或未指定的控制源,忽略写入
	if(g_xQueueRobotControl == NULL || cmd->cmdsource == NONE_CMD || 
		cmd->cmdsource >= UnKnownCMD ) return 1;
	
	//高优先级的控制指令,可直接抢占控制
	if( cmd->cmdsource <= RobotControl_CMDsource )
	{
		RobotControl_CMDsource = cmd->cmdsource;
		writeflag = 1;
	}
	else
	{
		//接受到低优先级控制指令,检查活跃时间戳
		uint32_t tick = 0;
		if( __get_IPSR()!=0 ) tick = xTaskGetTickCountFromISR();
		else tick = xTaskGetTickCount();
		
		//高优先级指令超2000ms未活跃,则允许低优先级指令执行
		if( tick - lastTick >= 2000 )
		{
			RobotControl_CMDsource = cmd->cmdsource;
			writeflag = 1;
		}
	}
	
	//允许写入队列
	if( writeflag )
	{
		if( __get_IPSR()!=0 ) //中断上下文判断
		{
			lastTick = xTaskGetTickCountFromISR();
			xQueueOverwriteFromISR(g_xQueueRobotControl,cmd,woken);
		}			
		else 
		{
			lastTick = xTaskGetTickCount();
			xQueueOverwrite(g_xQueueRobotControl,cmd);
		}
		return 0;
	}
	
	return 1;
}

/**
 * @brief  此函数不同车型的运动学正解。根据不同的车型执行不同的正解公式，最终获得小车3轴速度
 * @param  robot:小车控制相关对象  robot_hw:小车硬件参数相关对象
 * @retval 无
 *
 * @details
 * 
 *   
 */
static void Robot_Forwardkinematics(RobotControlParmentType_t* robot,const RobotParmentType_t* robot_hw)
{
	switch( robot_hw->CarType )
	{
		case S260:
		{
			robot->feedbackVx = (robot->MotorA.feedback + robot->MotorB.feedback + robot->MotorC.feedback + robot->MotorD.feedback + robot->MotorE.feedback + robot->MotorF.feedback )/6.0f;
			robot->feedbackVy = 0;
			robot->feedbackVz = (-robot->MotorA.feedback - robot->MotorB.feedback - robot->MotorC.feedback + robot->MotorD.feedback + robot->MotorE.feedback + robot->MotorF.feedback)/6.0f/(robot_hw->WheelSpacing+robot_hw->AxleSpacing);
			break;
		}
		case S200:case S200_OUTDOOR:
		{
			robot->feedbackVx = (robot->MotorA.feedback + robot->MotorB.feedback + robot->MotorC.feedback + robot->MotorD.feedback )/4.0f;
			robot->feedbackVy = 0;
			robot->feedbackVz = (-robot->MotorA.feedback - robot->MotorB.feedback + robot->MotorC.feedback + robot->MotorD.feedback )/4.0f/(robot_hw->WheelSpacing+robot_hw->AxleSpacing);
			break;
		}
		case S300:case S300Mini:case S100:
		{
			robot->feedbackVx = (robot->MotorA.feedback+robot->MotorB.feedback)/2.0f;
			robot->feedbackVy = 0;
			robot->feedbackVz = (robot->MotorB.feedback-robot->MotorA.feedback)/robot_hw->WheelSpacing;
			break;
		}
	}
}

/**
 * @brief  此函数用于计算纠偏系数,用于纠正小车走直线的效果
 * @param  纠偏系数值,左轮/右轮
 * @retval 纠偏倍数
 *
 * @details
 * 当小车无法走直线时,可通过调整纠偏系数来改善走直线的效果. 
 * 0-100为可调值,50表示无纠偏效果.大于50小车主动往右偏,
 * 小于50小车主动往左偏. 
 *   
 */
float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //左轮纠偏,对应50~100对应1.0~1.2倍的纠偏系数
	{
		if( diffparam>50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //右轮纠偏,50~0对应1.0~1.2倍的纠偏系数
	{
		if( diffparam<50 )
			return 1.0f + 0.004f*(50-diffparam);
	}
	
	return 1.0f;//不满足条件时,默认是1.
}


/**
 * @brief  此函数用于速度平滑控制以及运动学逆解。根据不同的车型执行不同的逆解公式，最终获得车轮速度
 * @param  Vx、Vy、Vz:小车的X轴、Y轴、Z轴三轴速度,单位 m/s
 * @retval 无
 *
 * @details
 * 
 *   
 */
static void Drive_Motor(const RobotControlCMDType_t* target)
{
	//速度平滑值
	static RobotControlCMDType_t smooth = { 0 }; 
	
	//Wheel target speed limit 
	//车轮目标速度限幅 单位m/s
	float amplitude=3.5;
	
	//实际传入的控制为用户控制+陀螺仪补偿
	RobotControlCMDType_t control_val = { 0 };
	control_val.Vx = target->Vx ;
	control_val.Vy = target->Vy ;
	control_val.Vz = target->Vz + LinearHelp_Target.Vz;
	
	//在自动回充模式下,只接收回充设备与ROS上位机的命令,其他命令设置速度为0
	if( RobotControlParam.ChargeMode==1 )
	{
		if( target->cmdsource!=Charger_CMD && target->cmdsource!=ROS_CMD )
		{
			control_val.Vx=0;control_val.Vy=0;control_val.Vz=0;
		}
	}
	
	//速度平滑
	Smooth_control(&smooth,&control_val,0.02f);
	
	//纠偏系数计算
	float LeftWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,0);
	
	//6轮车运动学解算
	if( RobotHardWareParam.CarType == S260 )
	{
		//四驱车模型 轴距不是真正意义上的轴距,需要根据imu的数据来联合调试到一个合适的值
		RobotControlParam.MotorA.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorB.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorC.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorD.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorE.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorF.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);

		//纠偏系数赋值
		RobotControlParam.MotorA.target *= LeftWheelDiff;
		RobotControlParam.MotorB.target *= LeftWheelDiff;
		RobotControlParam.MotorC.target *= LeftWheelDiff;
		RobotControlParam.MotorD.target *= RightWheelDiff;
		RobotControlParam.MotorE.target *= RightWheelDiff;
		RobotControlParam.MotorF.target *= RightWheelDiff;
		
		//最大轮速限幅
		RobotControlParam.MotorA.target = target_limit_float(RobotControlParam.MotorA.target,-amplitude,amplitude);
		RobotControlParam.MotorB.target = target_limit_float(RobotControlParam.MotorB.target,-amplitude,amplitude);
		RobotControlParam.MotorC.target = target_limit_float(RobotControlParam.MotorC.target,-amplitude,amplitude);
		RobotControlParam.MotorD.target = target_limit_float(RobotControlParam.MotorD.target,-amplitude,amplitude);
		RobotControlParam.MotorE.target = target_limit_float(RobotControlParam.MotorE.target,-amplitude,amplitude);
		RobotControlParam.MotorF.target = target_limit_float(RobotControlParam.MotorF.target,-amplitude,amplitude);
		
		//轮速转为rpm
		RobotControlParam.MotorA.target = linearVel_to_rpm(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorB.target = linearVel_to_rpm(RobotControlParam.MotorB.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorC.target = linearVel_to_rpm(RobotControlParam.MotorC.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorD.target = -linearVel_to_rpm(RobotControlParam.MotorD.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorE.target = -linearVel_to_rpm(RobotControlParam.MotorE.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorF.target = -linearVel_to_rpm(RobotControlParam.MotorF.target,RobotHardWareParam.WheelPerimeter);
	}
	
	//四驱车
	else if( RobotHardWareParam.CarType == S200 || RobotHardWareParam.CarType == S200_OUTDOOR )
	{
		//获得线速度
		RobotControlParam.MotorA.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorB.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorC.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorD.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		
		//纠偏系数赋值
		RobotControlParam.MotorA.target *= LeftWheelDiff;
		RobotControlParam.MotorB.target *= LeftWheelDiff;
		RobotControlParam.MotorC.target *= RightWheelDiff;
		RobotControlParam.MotorD.target *= RightWheelDiff;
		
		//最大轮速限幅
		RobotControlParam.MotorA.target = target_limit_float(RobotControlParam.MotorA.target,-amplitude,amplitude);
		RobotControlParam.MotorB.target = target_limit_float(RobotControlParam.MotorB.target,-amplitude,amplitude);
		RobotControlParam.MotorC.target = target_limit_float(RobotControlParam.MotorC.target,-amplitude,amplitude);
		RobotControlParam.MotorD.target = target_limit_float(RobotControlParam.MotorD.target,-amplitude,amplitude);
		
		//轮速转为rpm
		RobotControlParam.MotorA.target = linearVel_to_rpm(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorB.target = linearVel_to_rpm(RobotControlParam.MotorB.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorC.target = -linearVel_to_rpm(RobotControlParam.MotorC.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorD.target = -linearVel_to_rpm(RobotControlParam.MotorD.target,RobotHardWareParam.WheelPerimeter);
	}
	
	//两轮差速
	else if( RobotHardWareParam.CarType == S300 || RobotHardWareParam.CarType == S300Mini ||
              RobotHardWareParam.CarType == S100 )
	{
		RobotControlParam.MotorA.target = smooth.Vx - smooth.Vz * RobotHardWareParam.WheelSpacing / 2.0f; 
		RobotControlParam.MotorB.target = smooth.Vx + smooth.Vz * RobotHardWareParam.WheelSpacing / 2.0f; 
		RobotControlParam.MotorA.target *= LeftWheelDiff;
		RobotControlParam.MotorB.target *= RightWheelDiff;
		RobotControlParam.MotorA.target = target_limit_float(RobotControlParam.MotorA.target,-amplitude,amplitude);
		RobotControlParam.MotorB.target = target_limit_float(RobotControlParam.MotorB.target,-amplitude,amplitude);
		RobotControlParam.MotorA.target = -linearVel_to_rpm(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorB.target = linearVel_to_rpm(RobotControlParam.MotorB.target,RobotHardWareParam.WheelPerimeter);
		
		//S300轮子安装方向为反向
		if( RobotHardWareParam.CarType == S300 || RobotHardWareParam.CarType == S300Mini)
		{
			RobotControlParam.MotorA.target=-RobotControlParam.MotorA.target;
			RobotControlParam.MotorB.target=-RobotControlParam.MotorB.target;
		}
	}
	
}


//速度平滑控制
static void Smooth_control(RobotControlCMDType_t* cmd,const RobotControlCMDType_t* target,float step)
{
	//X轴速度平滑
	if(target->Vx>cmd->Vx)
	{
		cmd->Vx+=step;
		if(cmd->Vx>target->Vx) cmd->Vx=target->Vx;
	}
	else if (target->Vx<cmd->Vx)
	{
		cmd->Vx-=step;
		if(cmd->Vx<target->Vx) cmd->Vx=target->Vx;
	}
	else
		 cmd->Vx=target->Vx;

	//Y轴速度平滑
	if(target->Vy>cmd->Vy)
	{
		cmd->Vy+=step;
		if(cmd->Vy>target->Vy) cmd->Vy=target->Vy;
	}
	else if (target->Vy<cmd->Vy)
	{
		cmd->Vy-=step;
		if(cmd->Vy<target->Vy) cmd->Vy=target->Vy;
	}
	else
		 cmd->Vy=target->Vy;
	
	//Z轴速度平滑
//	if(target->Vz>cmd->Vz)
//	{
//		cmd->Vz+=step*2;
//		if(cmd->Vz>target->Vz) cmd->Vz=target->Vz;
//	}
//	else if (target->Vz<cmd->Vz)
//	{
//		cmd->Vz-=step*2;
//		if(cmd->Vz<target->Vz) cmd->Vz=target->Vz;
//	}
//	else
	
	//Z轴不使用平滑
	cmd->Vz=target->Vz;
}

static uint8_t CountBits_u8(uint8_t value)
{
	uint8_t count = 0;
	while( value != 0U )
	{
		count += value & 0x01U;
		value >>= 1U;
	}
	return count;
}

static uint8_t Ranger_GetMonitorMask(uint8_t mode)
{
	switch( mode )
	{
		case RangerMonitor_Front: return RANGER_FRONT_MASK;
		case RangerMonitor_Back:  return RANGER_BACK_MASK;
		case RangerMonitor_All:   return RANGER_ALL_MASK;
		default:                  return 0U;
	}
}

static uint8_t Ranger_GetMonitorMode(const RobotControlCMDType_t* target)
{
	if( fabs(target->Vz) > RANGER_ANGULAR_EPS || fabs(target->Vy) > RANGER_LINEAR_EPS )
		return RangerMonitor_All;
	if( target->Vx > RANGER_LINEAR_EPS )
		return RangerMonitor_Front;
	if( target->Vx < -RANGER_LINEAR_EPS )
		return RangerMonitor_Back;
	return RangerMonitor_None;
}

static uint8_t Ranger_GetFreshDangerMask(uint8_t mode,uint8_t* updatedMask)
{
	uint8_t monitorMask = Ranger_GetMonitorMask(mode);
	uint8_t localUpdatedMask = 0;
	uint8_t dangerMask = 0;

	for(uint8_t i=0;i<sizeof(g_RangerSensorList)/sizeof(g_RangerSensorList[0]);i++)
	{
		uint32_t sampleSeq = g_RangerSensorList[i]->sampleSeq;
		uint8_t sensorMask = (uint8_t)(1U<<i);
		if( sampleSeq != g_RangerSampleShadow[i] )
		{
			g_RangerSampleShadow[i] = sampleSeq;
			if( (monitorMask & sensorMask) != 0U )
			{
				localUpdatedMask |= sensorMask;
				if( g_RangerSensorList[i]->dis < RANGER_HARDSTOP_TRIGGER_DIS )
					dangerMask |= sensorMask;
			}
		}
	}

	if( updatedMask != NULL ) *updatedMask = localUpdatedMask;
	return dangerMask;
}

static uint8_t Ranger_IsModeSafe(uint8_t mode,float threshold)
{
	uint8_t monitorMask = Ranger_GetMonitorMask(mode);
	if( monitorMask == 0U ) return 1U;

	for(uint8_t i=0;i<sizeof(g_RangerSensorList)/sizeof(g_RangerSensorList[0]);i++)
	{
		uint8_t sensorMask = (uint8_t)(1U<<i);
		if( (monitorMask & sensorMask) != 0U && g_RangerSensorList[i]->dis < threshold )
			return 0U;
	}
	return 1U;
}

static uint8_t Robot_WheelsStopped(void)
{
	switch( RobotHardWareParam.CarType )
	{
		case S260:
			return (fabs(RobotControlParam.MotorA.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorB.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorC.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorD.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorE.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorF.feedback) < RANGER_WHEEL_STOP_DIS);
		case S200:
		case S200_OUTDOOR:
			return (fabs(RobotControlParam.MotorA.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorB.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorC.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorD.feedback) < RANGER_WHEEL_STOP_DIS);
		case S300:
		case S300Mini:
		case S100:
			return (fabs(RobotControlParam.MotorA.feedback) < RANGER_WHEEL_STOP_DIS &&
			        fabs(RobotControlParam.MotorB.feedback) < RANGER_WHEEL_STOP_DIS);
		default:
			return 1U;
	}
}

static void Ranger_ResetHardStop(void)
{
	RobotControlParam.RangerHardStopState = RangerStopState_Idle;
	RobotControlParam.RangerHardStopMode = RangerMonitor_None;
	RobotControlParam.RangerDangerPending = 0;
	RobotControlParam.RangerDangerMask = 0;
}

static void Ranger_TriggerHardStop(uint8_t mode,uint8_t dangerMask)
{
	pBuzzerInterface_t tips = &UserBuzzer;
	RobotControlParam.RangerHardStopState = RangerStopState_Braking;
	RobotControlParam.RangerHardStopMode = mode;
	RobotControlParam.RangerDangerPending = 0;
	RobotControlParam.RangerDangerMask = dangerMask;
	tips->AddHoldTask(RANGER_HARDSTOP_BUZZER_MS);
}

static void RangerHardStop_Update(const RobotControlCMDType_t* request,RobotControlCMDType_t* applied)
{
	static uint8_t wheelStopStableCnt = 0;
	uint8_t currentMode = Ranger_GetMonitorMode(request);

	if( RobotControlParam.RangerAvoidEN == 0 )
	{
		wheelStopStableCnt = 0;
		Ranger_ResetHardStop();
		return;
	}

	if( RobotControlParam.RangerHardStopState == RangerStopState_Idle )
	{
		uint8_t updatedMask = 0;
		uint8_t dangerMask = 0;

		if( RobotControlParam.RangerHardStopMode != currentMode )
		{
			RobotControlParam.RangerDangerPending = 0;
			RobotControlParam.RangerDangerMask = 0;
		}

		RobotControlParam.RangerHardStopMode = currentMode;
		if( currentMode == RangerMonitor_None )
		{
			RobotControlParam.RangerDangerPending = 0;
			RobotControlParam.RangerDangerMask = 0;
			return;
		}

		dangerMask = Ranger_GetFreshDangerMask(currentMode,&updatedMask);
		if( dangerMask != 0U )
		{
			if( CountBits_u8(dangerMask) >= 2U || RobotControlParam.RangerDangerPending != 0U )
			{
				wheelStopStableCnt = 0;
				Ranger_TriggerHardStop(currentMode,dangerMask);
				applied->Vx = 0;
				applied->Vy = 0;
				applied->Vz = 0;
				return;
			}

			RobotControlParam.RangerDangerPending = 1;
			RobotControlParam.RangerDangerMask = dangerMask;
		}
		else if( updatedMask != 0U )
		{
			RobotControlParam.RangerDangerPending = 0;
			RobotControlParam.RangerDangerMask = 0;
		}
		return;
	}

	applied->Vx = 0;
	applied->Vy = 0;
	applied->Vz = 0;

	if( Robot_WheelsStopped() )
	{
		if( wheelStopStableCnt < 0xFFU ) wheelStopStableCnt++;
	}
	else wheelStopStableCnt = 0;

	if( RobotControlParam.RangerHardStopState == RangerStopState_Braking )
	{
		if( wheelStopStableCnt >= RANGER_WHEEL_STOP_FILTER )
			RobotControlParam.RangerHardStopState = RangerStopState_Hold;
		return;
	}

	if( wheelStopStableCnt == 0U )
	{
		RobotControlParam.RangerHardStopState = RangerStopState_Braking;
		return;
	}

	currentMode = Ranger_GetMonitorMode(request);
	if( currentMode == RangerMonitor_None || Ranger_IsModeSafe(currentMode,RANGER_HARDSTOP_RELEASE_DIS) )
	{
		wheelStopStableCnt = 0;
		Ranger_ResetHardStop();
	}
}



/**
 * @brief  此函数执行结合IMU数据走直线功能
 * @param  小车目标速度值
 * @retval 无
 *
 * @details
 *  IMU和小车控制方向：逆时针旋转时,yaw为正,yaw速度为正. Vz为正时小车逆时针旋转
 *   
 */
static void Linear_IMUHelp(RobotControlCMDType_t* target)
{
	//用于延迟获取目标角度
	static uint8_t TargetCnt = 0;
	#define DelayCNT 70
	
	//目标角度
	static float target_angle = 0;
	
	//PID参数
	float linear_helpkp = 0.08f;
	float linear_helpkd = 0.05f;
	
	switch( RobotHardWareParam.CarType )
	{
		//不同车型在直线调节时PID参数不相同,与车身重量、车身结构、以及电机都有关系
		case S100: linear_helpkp = 0.03f; linear_helpkd = 0.007f; break;
		case S300Mini: linear_helpkp = 0.04f; linear_helpkd = 0.005f; break;
		case S300: linear_helpkp = 0.06f; linear_helpkd = 0.015f; break;
		default:break;
	}
	
	static uint8_t giveup_flag = 0;

	//仅在Vx有速度时补偿,其余情况不补偿并更新目标角度
	if( target->Vx!=0 && target->Vy==0 && target->Vz==0 && giveup_flag==0 ) 
	{
		//延迟获得目标角度
		TargetCnt++;
		if( TargetCnt==DelayCNT )  target_angle = AttitudeVal.yaw;

		//获取到目标角度后再执行PID控制
		if( TargetCnt >= DelayCNT )
		{
			//锁住计数值防止循环
			TargetCnt=DelayCNT+1;
			
			//误差计算
			float error = target_angle-AttitudeVal.yaw;
			
			//对误差角度归一化
			if( error >  180.0f ) error -= 360.0f;
			if( error < -180.0f ) error += 360.0f;
			
			//当偏差过大时,放弃调整(如悬空控制).
			if( fabs(error)> 30 ) 
			{
				giveup_flag = 1;
				LinearHelp_Target.Vz=0;
				return;
			}
			
			//PID调整小车走直线效果（仅超过一定速度才调整）
			static uint8_t AllowFilterCnt;
			if( fabs(RobotControlParam.feedbackVx)>=0.05f )
			{
				AllowFilterCnt++;
				if( AllowFilterCnt>80 )
				{
					AllowFilterCnt=81;
					LinearHelp_Target.Vz = linear_helpkp * error - linear_helpkd * axis_9Val.gyro.z*57.3f;
				}
				else LinearHelp_Target.Vz=0;
			}
			else
			{
				LinearHelp_Target.Vz=0;
				AllowFilterCnt=0;
			}
				
			
			//限制调整值
			LinearHelp_Target.Vz = target_limit_float(LinearHelp_Target.Vz,-1.57f,1.57f);
		}

	}
	else if( target->Vx==0 && target->Vy==0 && target->Vz==0 )
	{
		//速度全0时静止一段时间则刷新目标角度
		TargetCnt++;
		if( TargetCnt>100 )  target_angle = AttitudeVal.yaw,TargetCnt=101;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
		giveup_flag=0;
	}
	else
	{	
		//其他情况则延迟目标角度的刷新,防止动态切换的过程目标角度刷新不准确
		TargetCnt=0;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
	}
	
}


float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

float rpm_to_linearVel(float rpm,float wheelper)
{
	return rpm/60.0f*wheelper;
}

//复位进入bootloader函数
void _System_Reset_FromAPP_RTOS(char uart_recv)
{
	static char res_buf[5];
	static uint8_t res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//接受到上位机请求的复位字符“reset”，执行软件复位
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			RobotControlCMDType_t cmd = {
				.cmdsource = BootLoader,
				0,0,0
			};
			
			//写入最高优先级指令停止小车运动
			for(uint8_t i=0;i<50;i++)
			{
				WriteRobotControlQueue(&cmd,0);
				vTaskDelay( pdMS_TO_TICKS(10) );
			}
			
			//复位系统
			NVIC_SystemReset();
		}
	}
}

//调试模式
void RobotControl_SetDebugLevel(char uart_recv)
{
	//调试任务句柄
	extern TaskHandle_t g_reportErrTaskHandle;
	
	static char res_buf[4];
	static uint8_t res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='L'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==4)
	{
		res_count = 0;
		
		if( res_buf[0]=='L'&&res_buf[1]=='O'&&res_buf[2]=='G')
		{
			switch( res_buf[3] )
			{
				case '0': RobotControlParam.DebugLevel = 0; break;
				case '1': RobotControlParam.DebugLevel = 1; break;
				case '2': RobotControlParam.DebugLevel = 2; break;
				case '3': RobotControlParam.DebugLevel = 3; break;
				default: RobotControlParam.DebugLevel = 0; break;
			}
			
			//操作提示音
			pBuzzerInterface_t tips = &UserBuzzer;
			tips->AddTask(1,200);
			
			//发送任务通知
			xTaskNotifyGive(g_reportErrTaskHandle);
		}
	}
}


