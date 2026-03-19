#include "AutoRecharge_task.h"

//C Include File
#include <stdio.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_can.h"
#include "bsp_RTOSdebug.h"

//APP Include File
#include "RobotControl_task.h"
#include "sensor_ranger.h"

ChargeDevType_t ChargeDev = { 
	.dockerX = -0.15f,
	.dockerY = 0,
	.dockerZ = 0.2f
};

void AutoRechargeTask(void* param)
{
	extern QueueHandle_t g_xQueueAutoRecharge;
	
//	pRTOSDebugInterface_t debug = &RTOSDebugTimer;
//	RTOSDebugPrivateVar priv = { 0 };
	
	while(1)
	{
		CANmsgType_t msg;
		
		if( pdPASS == xQueueReceive(g_xQueueAutoRecharge,&msg,pdMS_TO_TICKS(500)) ) 
		{
			//标注控制源,自动回充设备
			RobotControlCMDType_t cmd = {
				.cmdsource = Charger_CMD,
				0,0,0
			};
			
			//充电装备在线
			ChargeDev.online=1;

			//Target speed from charging device
			//来自充电设备的目标速度
			cmd.Vx = (float)((short)(msg.buffer[0]<<8|msg.buffer[1]))/1000.0f;
			cmd.Vz = (float)((short)(msg.buffer[4]<<8|msg.buffer[5]))/1000.0f;
			//带后部超声波的车型可根据后部超声波距离减速
			if( RobotHardWareParam.CarType==S200_OUTDOOR || RobotHardWareParam.CarType==S260 )
			{
				if( RangerHAL_E.dis < 0.5f || RangerHAL_F.dis < 0.5f )
					cmd.Vx/=2.0f;
			}
			
			//充电设备的内部状态值
			ChargeDev.devState = msg.buffer[2];

			//充电设备上检测到的电压
			ChargeDev.ChargingVol= (msg.buffer[3]*30)/100.0f;

			//充电状态标志位
			ChargeDev.ChargingFlag = msg.buffer[6]&0x01; 

			//A:39ms红外情况（正面面向充电桩左边的红外）
			//B:52ms红外情况（正面面向充电桩右边的红外）
			ChargeDev.L_A = (msg.buffer[6]>>5)&0x01;
			ChargeDev.L_B = (msg.buffer[6]>>4)&0x01;
			ChargeDev.R_B = (msg.buffer[6]>>3)&0x01;
			ChargeDev.R_A = (msg.buffer[6]>>2)&0x01;
			
			//红外管信号个数
			ChargeDev.RedNum = ChargeDev.L_A+ChargeDev.L_B+ChargeDev.R_A+ChargeDev.R_B;
	
			//未识别到红外信号、已经在充电 -> 速度置0
			if(ChargeDev.RedNum==0 || ChargeDev.ChargingFlag==1 ) cmd.Vx = cmd.Vy = cmd.Vz = 0;

			//接近满电状态，清空速度防止出现充满后小车运动
			if( ChargeDev.devState==0xAB && RobotControlParam.Vol>24.5f ) cmd.Vx = 0 , cmd.Vy = 0;

			//充电电流换算
			if(msg.buffer[7]>128) ChargeDev.ChargingCur =-(256-msg.buffer[7])*30;
			else ChargeDev.ChargingCur = (msg.buffer[7]*30);
			
			//速度空闲判断
			uint8_t writeflag=1;
			static uint8_t idleCount = 0;
			if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
			else idleCount=0;
			if( idleCount>10 ) writeflag=0,idleCount=10;
			
			//自动回充开启,且存在速度,则写入队列
			if( RobotControlParam.ChargeMode==1 && writeflag==1 )
			{
				WriteRobotControlQueue(&cmd,0);
			}
			
		}
		else //自动回充装备离线
		{
			ChargeDev.online=0;
			ChargeDev.ChargingCur=0;
			ChargeDev.ChargingVol=0;
			ChargeDev.ChargingFlag=0;
			ChargeDev.L_A=0,ChargeDev.L_B=0,ChargeDev.R_A=0,ChargeDev.R_B=0;
			ChargeDev.RedNum=0;
		}

	}
}
