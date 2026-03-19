#include "bsp_icm20948.h"

#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "robot_select_init.h"


void ImuTask(void* param)
{
	//获取时基,用于辅助任务能指定固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//控制任务的频率,单位HZ
	const uint16_t TaskFreq = 100;
	
	pIMUInterface_t imu = &UserICM20948;
	
	//陀螺仪初始化
	while(imu->Init())
	{
		vTaskDelay(1000);
	}
	
	//开机计时
	uint32_t SystemStartCnt = 0;
	
	//开机前10秒失能小车,等待imu标零
	RobotControlParam.softwareEnflag = 1;
	
	while(1)
	{
		imu->Update_9axisVal(&axis_9Val,&axis_9ValOri);//陀螺仪数据更新,获取的数据均为原始数据.软件iic耗时1.0 ms
		imu->UpdateAttitude(axis_9Val,&AttitudeVal);//更新姿态角
		
		//前30秒计时
		if( SystemStartCnt < TaskFreq*30 ) SystemStartCnt++;
		
		//10秒后执行imu标零,使能小车
		if( SystemStartCnt == TaskFreq*10 )
		{
			//小车相对静止的情况下执行标零
			if( fabs(axis_9Val.gyro.x)<0.05f && fabs(axis_9Val.gyro.y)<0.05f && fabs(axis_9Val.gyro.z)<0.05f )
			{
				RobotControlParam.LedTickState++;
				imu->UpdateZeroPoint_axis();
			}			
			
			//小车姿态角较为水平的正常情况下执行标零
			if( fabs(AttitudeVal.pitch) < 5.0f && fabs(AttitudeVal.roll) < 5.0f )
			{
				RobotControlParam.LedTickState++;
				imu->UpdateZeroPoint_attitude();
			}

			//检查标零是否全部成功
			if( RobotControlParam.LedTickState!=2 )
			{
				RobotControlParam.LedTickState=0;
				SystemStartCnt=TaskFreq*8;//不满足条件,重新等待
			}
			else
			{
				//标零成功,使能小车
				RobotControlParam.softwareEnflag=0;
			}				
		}
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

