/**
 * @file    tim_capture_callback.c
 * @brief   定时器输入捕获中断回调函数.
 * @author  WHEELTEC
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - 本文件主要内容为定时器输入捕获中断回调函数,在回调函数内处理航模手柄的底层信号,
 *   以及处理超声波的底层信号。航模遥控信号与超声波测距均依赖此回调函数获得数据。
   - 回调函数处理内容兼容FreeRTOS,由xHigherPriorityTaskWoken确定是否需要发起任
     务调度
 *      
 * @note
 * 
 * 
 */

#include "sensor_ranger.h"
#include "rc_joystick.h"

//定时器捕获回调函数,超声波捕获、航模遥控器通道捕获均使用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//此中断涉及操作系统的API,仅在调度器启动时才执行中断内容
	BaseType_t schedulerState = xTaskGetSchedulerState();
	if( schedulerState!=taskSCHEDULER_RUNNING ) return;
	
	if( htim == &htim4 )
	{
		//航模手柄信号处理逻辑
		RCJoystick_IC_CaptureCallback(htim,&xHigherPriorityTaskWoken);
	}
	else
	{
		//超声波信号逻辑处理
		Ranger_IC_CaptureCallback(htim,&xHigherPriorityTaskWoken);
	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

