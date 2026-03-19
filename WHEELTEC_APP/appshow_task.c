
//HAL Lib Include File
#include "usart.h"

//C Include File
#include <stdio.h>
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_ServoDrive.h"
#include "bsp_icm20948.h"

//APP Include File
#include "BlueTooth_task.h"
#include "robot_select_init.h"
#include "RobotControl_task.h"

static char page1_paramshow[100];   //APP首页数据
static char page2_paramshow[512];  //APP波形页面数据
static char page3_paramshow[512];  //APP参数页面数据
static uint8_t g_appshowTxBuffer[sizeof(page3_paramshow)] = { 0 };

static void APPshow_TransmitDMA(const char *buffer)
{
	size_t length = 0U;

	if( buffer == NULL )
		return;

	length = strnlen(buffer,sizeof(g_appshowTxBuffer));
	if( length == 0U || length >= sizeof(g_appshowTxBuffer) )
		return;

	if( huart2.gState != HAL_UART_STATE_READY )
		return;

	memcpy(g_appshowTxBuffer,buffer,length);
	if( HAL_UART_Transmit_DMA(&huart2,g_appshowTxBuffer,(uint16_t)length) != HAL_OK )
	{
		// 串口忙时直接跳过当前显示帧，避免覆盖仍在发送中的 DMA 缓冲区
	}
}

static int voltage_to_percentage(float voltage) {
    const float V_MIN = 20.3f;  // 最小电压 (0%)
    const float V_MAX = 23.45f; // 最大电压 (100%)
    
    // 确保电压在有效范围内
    if (voltage < V_MIN) return 0;
    if (voltage > V_MAX) return 100;
    
    // 线性插值计算百分比并取整
    int percentage = (int)(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f + 0.5f); // 四舍五入
	
	if( percentage > 100 ) percentage = 100;
	if( percentage <= 0 ) percentage = 0;
	
    return percentage;
}

void APPshow_task(void* param)
{
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//本任务的控制频率,单位为Hz
	const uint16_t TaskFreq = 20;
	
	//实现APP数据分时显示
	uint8_t appshowNum = 0;
	const char* NeedShowBuf = NULL;
	
	while(1)
	{	
		int showLeftWheel=0,showRightWheel=0,VolPercen=0;
		
		switch(RobotHardWareParam.CarType)
		{
			case S300:case S300Mini:case S100:
				showLeftWheel = RobotControlParam.MotorA.feedback*1000;
				showRightWheel = RobotControlParam.MotorB.feedback*1000;
				break;
			case S200:case S200_OUTDOOR:
				showLeftWheel = RobotControlParam.MotorA.feedback*1000;
				showRightWheel = RobotControlParam.MotorD.feedback*1000;
				break;
			case S260:
				showLeftWheel = RobotControlParam.MotorA.feedback*1000;
				showRightWheel = RobotControlParam.MotorF.feedback*1000;
				break;
			default:
				break;
		}
		
		//电池电压百分比
		VolPercen = voltage_to_percentage(RobotControlParam.Vol);
		
		//APP显示数据
		appshowNum++; //分时显示
		if( 1 == appshowNum )
		{
			snprintf(page1_paramshow,sizeof(page1_paramshow),"{A%d:%d:%d:%d}$",showLeftWheel,showRightWheel,VolPercen,(int)AttitudeVal.yaw); //左码盘,右码盘,电量百分比,角度
			NeedShowBuf = page1_paramshow;
		}
		else if( 2 == appshowNum )
		{
			appshowNum = 0;
			snprintf(page2_paramshow,sizeof(page2_paramshow),"{B%d:%d:%d}$",(int)AttitudeVal.pitch,(int)AttitudeVal.roll,(int)AttitudeVal.yaw); //波形显示
			NeedShowBuf = page2_paramshow;
		}
		
		//APP获取数据请求
		if( wheeltecApp.reportparam == 1 )
		{
			wheeltecApp.reportparam = 0;
			snprintf(page3_paramshow,sizeof(page3_paramshow),"{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
			 (int)RobotControlParam.defalutSpeed,
			 (int)RobotControlParam.LineDiffParam,
			 (int)DrivePid.SpeedKp,(int)DrivePid.SpeedKi,
			 (int)DrivePid.CurKp,(int)DrivePid.CurKi,
			  0,0,0);
			NeedShowBuf = page3_paramshow;
		}
		
		//常规数据发送
		if( RobotControlParam.DebugLevel == 0 )
		{
			APPshow_TransmitDMA(NeedShowBuf);
		}
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
	
}