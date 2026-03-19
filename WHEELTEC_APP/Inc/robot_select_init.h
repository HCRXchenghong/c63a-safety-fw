#ifndef __ROBOT_SELECT_INIT_H
#define __ROBOT_SELECT_INIT_H

#include <stdint.h>

#define PI  3.14159265358979f

enum{
	SingleAxis_5inch = 0,  //5寸单出轴电机,适配型号S100
	DoubleAxis_5inch ,     //5寸双出轴电机,适配型号S300Mini(S150)
	SingleAxis_8inch ,     //8寸单出轴电机,适配型号S200、S200户外版本(S200_OUTDOOR)、S260
	DoubleAxis_8inch ,     //8寸双出轴电机,适配型号S300
};

typedef struct{
	uint8_t CarType;     //车型
	uint8_t CarTypeSupported; //车型是否被当前固件支持
	uint8_t driveCounts; //驱动器的数量
	uint8_t wheeltype;   //轮子的型号：5寸单出轴、5寸双出轴、8寸单出轴、8寸双出轴
	float WheelSpacing; //轮距
	float AxleSpacing;  //轴距
	float WheelPerimeter;//轮子周长
}RobotParmentType_t;

//S300轮距、轮径
#define S300_Wheelspacing 0.398
#define S300_Diameter 0.20

//S150轮距、轮径
#define S150_Wheelspacing 0.26
#define S150_Diameter 0.13

//S100轮距、轮距
#define S100_Wheelspacing 0.2272f
#define S100_Diameter 0.13

//S200轮距、轮径
#define S200_Wheelspacing 0.23 //半轮距
#define S200_axlespacing  0.15//半轴距
#define S200_Diameter 0.205

#define S200_OUTDOOR_Wheelspacing 0.23 //半轮距
#define S200_OUTDOOR_axlespacing  0.15//半轴距
#define S200_OUTDOOR_Diameter 0.205

//S260轮距、轮径
#define S260_Wheelspacing 0.29  //半轮距

//S260轴距不是真正意义上的轴距,需要根据imu的数据来联合调试到一个合适的值
#define S260_axlespacing  0.08  //半轴距
#define S260_Diameter 0.205

//车型枚举
enum CAR_MODE{
	S300,
	S200,
	S200_OUTDOOR, //S200户外版
	S300Mini,
	S100,
	S260,
	SX04,
	Number_of_CAR
};

//机器人硬件参数
extern RobotParmentType_t RobotHardWareParam;

//车轮变量,包含车轮目标速度和车轮实际速度
typedef struct{
	float target;
	float feedback;
}RobotMotorType_t;

//机器人控制参数
typedef struct{
	uint8_t en_flag;         //小车全局使能标志位,1使能可控制 0失能禁止控制
	uint32_t ErrNum ;        //小车报错码
	short defalutSpeed;     //小车默认遥控速度,单位mm/s
	uint8_t Enkeystate;      //小车硬件使能开关的状态,1开关弹起 0开关按下
	float Vol;              //小车电池电压
	uint8_t DriveErrRecovery;//小车请求清除驱动报错,设置1请求清除驱动器的报错,此位会自动置0,
	uint8_t ChargeMode;      //自动回充模式, 1开启自动回充,0关闭自动回充
	uint8_t SecurityLevel;   //安全等级 0:若小车没有持续收到目标速度,则会主动停止 1:小车将保持最后一次的目标速度响应  
	uint8_t EmergencyMode;   //应急模式,此位为1时,非电机无法驱动的报错,都可以强行控制小车,以应对在室外突发情况.若存在还可驱动的电机,也能拖动无法驱动的电机行驶
	uint8_t softwareEnflag;  //软件急停标志位,设1可使小车进入急停状态
	uint8_t RangerAvoidEN;   //超声波安全保护使能位,沿用旧字段做兼容
	uint8_t RangerHardStopState;
	uint8_t RangerHardStopMode;
	uint8_t RangerDangerPending;
	uint8_t RangerDangerMask;
	uint32_t LineDiffParam;  //机器人纠偏系数，0-100可调整
	uint8_t ImuAssistedFlag; //imu辅助小车走直线标志位. 0无赋值,设置1时,小车被控制时,将通过imu的数据辅助小车走直线
	uint8_t DebugLevel;      //小车报错调试等级. 0正常控制, 置1时小车将通过串口1、蓝牙串口汇报错误信息
	uint8_t LedTickState;    //小车板载LED提示状态
	uint8_t LowPowerFlag;    //小车电量低标志位
	
	uint8_t ParkingMode;     //私有变量,小车驻车模式标志位（用户勿操作）
	
	//机器人车轮目标值与反馈值
	RobotMotorType_t MotorA;  //车轮的目标速度和反馈速度存放于此
	RobotMotorType_t MotorB;
	RobotMotorType_t MotorC;
	RobotMotorType_t MotorD;
	RobotMotorType_t MotorE;
	RobotMotorType_t MotorF;
	
	//机器人三轴实际速度存放于此,由运动学正解获得
	float feedbackVx;
	float feedbackVy;
	float feedbackVz;
	
}RobotControlParmentType_t;

void Robot_Select(void);
extern RobotControlParmentType_t RobotControlParam;

#endif

