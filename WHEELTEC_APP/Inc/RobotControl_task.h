#ifndef __ROBOTCONTROL_TASK_H
#define __ROBOTCONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_compiler.h"

#include "robot_select_init.h"

//控制指令优先级,数值越小优先级越高
//高优先级控制可打断低优先级控制
enum{
	NONE_CMD   = 0,  //用户不可修改.0表示忽略控制量,以防用户未指定控制源就写入的异常情况。
	BootLoader ,  //用户不可修改,BootLoader为最高优先级的系统控制量,用于小车无线更新程序使用
	               //程序更新前最高优先级控制量将介入控制小车停止。
	
	//用户自定义区域,优先级由高到低
	GamePad_CMD , //游戏手柄
	RCJOY_CMD   , //航模遥控
	APP_CMD     , //手机APP
	ROS_CMD     , //ROS或串口上位机
	CAN_CMD     , //CAN1接口控制
	Charger_CMD , //自动回充装备控制
	
	UnKnownCMD     //用户不可修改,此枚举需要一直保留在最后一位.目的同样为禁止未指定控制源就写入的异常情况
};

//小车控制队列的数据格式
typedef struct{
	uint8_t cmdsource;
	float Vx;
	float Vy;
	float Vz;
}RobotControlCMDType_t;

//小车最新的控制来源
extern volatile uint8_t RobotControl_CMDsource;

//小车主动失能的条件标志位,满足任意1个条件都会失能禁止控制
enum{
	errCode_LowPower        = (1<<0), //电量不足
	errCode_StopKeyEn       = (1<<1), //急停开关按下
	errCode_SoftWareStop    = (1<<2), //软件急停被设置
	errCode_Driver1_offline = (1<<3), //1号驱动器离线
	errCode_Driver1_Err     = (1<<4), //1号驱动器报错
	errCode_Driver2_offline = (1<<5), //2号驱动器离线
	errCode_Driver2_Err     = (1<<6), //2号驱动器报错
	errCode_Driver3_offline = (1<<7), //3号驱动器离线
	errCode_Driver3_Err     = (1<<8), //3号驱动器报错
	errCode_RangerHardStop  = (1<<9), //超声波硬急停
	errCode_UnsupportedCarType = (1<<10), //当前车型未被固件支持
};

enum{
	RangerStopState_Idle = 0,
	RangerStopState_Braking,
	RangerStopState_Hold,
};

enum{
	RangerMonitor_None = 0,
	RangerMonitor_Front,
	RangerMonitor_Back,
	RangerMonitor_All,
};

#define Clear_RobotErrorCode(mask)  (RobotControlParam.ErrNum &= ~(mask))    //清除报错标志位
#define Set_RobotErrorCode(mask)    (RobotControlParam.ErrNum |= (mask))     //设置报错标志位
#define Get_RobotErrorCode(mask)    (RobotControlParam.ErrNum & (mask))      //读取报错标志位


//对外公开使用
uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd,BaseType_t* woken);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel);
void _System_Reset_FromAPP_RTOS(char uart_recv);
float rpm_to_linearVel(float rpm,float wheelper);
void RobotControl_SetDebugLevel(char uart_recv);

#endif


