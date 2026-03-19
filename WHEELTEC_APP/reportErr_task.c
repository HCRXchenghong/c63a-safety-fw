/**
 * @file    reportErr_task.c
 * @brief   小车自检调试文件
 * @author  WHEELTEC
 * @date    2025-08-08
 * @version 1.0.0
 *
 * @details
 * - 本文件是小车调试相关内容，由任务 ReportErrTask 管理.当用户在串口或者蓝牙端输入 "LOG1~3"设置调试等级后，
 * 调试任务才会被触发，小车将自主上报报错信息，底盘的状态等内容.
 * @note
 * 
 * 
 */
//HAL Include File
#include "usart.h"

//C include File
#include <stdio.h>
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_ServoDrive.h"

//APP Include File
#include "RobotControl_task.h"
#include "AutoRecharge_task.h"

//调试任务句柄
extern TaskHandle_t g_reportErrTaskHandle;


// 通用错误信息映射表
typedef struct {
    uint32_t errCode;         // 错误码
    const char* errMessage;  // 错误信息
} ErrorMap;

//小车的错误码以及对应的报错信息
static const ErrorMap errorMap[] = {
    {errCode_LowPower,        "因小车电压过低，小车已失能\r\n"},
    {errCode_StopKeyEn,       "因急停开关被按下，小车已失能\r\n"},
    {errCode_SoftWareStop,    "因软件急停被设置，小车已失能\r\n"},
    {errCode_RangerHardStop,  "因超声波硬急停被触发，小车已失能\r\n"},
    {errCode_UnsupportedCarType, "因当前车型未被此固件支持，小车已失能\r\n"},
    {errCode_Driver1_offline, "因 1号驱动器离线，小车已失能\r\n"},
    {errCode_Driver1_Err,     "因 1号驱动器报错，小车已失能\r\n"},
    {errCode_Driver2_offline, "因 2号驱动器离线，小车已失能\r\n"},
    {errCode_Driver2_Err,     "因 2号驱动器报错，小车已失能\r\n"},
    {errCode_Driver3_offline, "因 3号驱动器离线，小车已失能\r\n"},
    {errCode_Driver3_Err,     "因 3号驱动器报错，小车已失能\r\n"},
};


//伺服驱动器内部定义报错码
enum{
	DriverOverVol =          (1<<0), //驱动过压
	DriverUnderVol =         (1<<1), //驱动欠压
	ServoMotorOverCur =      (1<<2), //电机过流
	ServoMotorOverLoad =     (1<<3), //电机过载
	ServoMotorCurOverD =     (1<<4), //电流超差
	ServoMotorEncoderOverD = (1<<5), //编码器超差
	ServoMotorSpeedErr =     (1<<6), //速度超差
	ServoMotorVolErr =       (1<<7), //参考电压出错
	DrvierEEPROMErr =        (1<<8), //驱动器EEPROM读写错误
	ServoMotorHALLErr =      (1<<9), //霍尔错误
	ServoMotorOverTemp =     (1<<10), //电机超温
	ServoMotorEncoderErr =   (1<<11)  //编码器错误
};

//驱动器的错误码以及对应的报错信息
static const ErrorMap ServoErrMap[] = {
    {DriverOverVol,          "驱动器过压\r\n"},
    {DriverUnderVol,         "驱动器欠压\r\n"},
    {ServoMotorOverCur,      "电机过流\r\n"},
    {ServoMotorOverLoad,     "电机过载\r\n"},
    {ServoMotorCurOverD,     "电流超差\r\n"},
    {ServoMotorEncoderOverD, "编码器值超差\r\n"},
    {ServoMotorSpeedErr,     "速度超差\r\n"},
    {ServoMotorVolErr,       "参考电压出错\r\n"},
    {DrvierEEPROMErr,        "驱动器EEPROM读写出错\r\n"},
	{ServoMotorHALLErr,      "霍尔线未插或接触不良\r\n"},
	{ServoMotorOverTemp,     "电机温度超过保护阈值\r\n"},
	{ServoMotorEncoderErr,   "编码器数据错误\r\n"},
};


//对外数据汇报的接口，用于调试
static void report_interface(char* buffer,uint16_t size)
{
	for(uint32_t k=0;k<size;k+=2)
	{
		HAL_UART_Transmit(&huart2,(uint8_t*)&buffer[k],2,500);
		HAL_UART_Transmit(&huart1,(uint8_t*)&buffer[k],2,500);
		vTaskDelay( pdMS_TO_TICKS(50) );
	}
//	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)buffer,size);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)buffer,size);
//	vTaskDelay( pdMS_TO_TICKS(10) );
}

//用于临时保存输出的数据
static char err_info[512] = { 0 };

void ReportErrTask(void* param)
{
	
	//调试等级
	// 1:仅汇报小车失能的原因(APP或串口1,发送"LOG1"设置调试等级1)
	// 2:显示小车以及自动回充相关信息(APP或串口1,发送"LOG2"设置调试等级2)
	// 3:显示小车、自动回充、以及所有伺服驱动器的状态信息(APP或串口1,发送"LOG3"设置调试等级3)
	// 因使用中文汇报,需要保证字符串和中文过渡时,始终保持偶数个数,以免APP或串口助手解析中文字符错误
	
	while(1)
	{
		//阻塞等待任务通知
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		
		while( RobotControlParam.DebugLevel!=0 )
		{
			extern uint8_t Received_IP[4];       
			extern uint8_t IP_Frame_Valid;   
			//若小车有主动失能的情况,则优先进行原因提示
			const char* head = "{#\r\n";
			const char* end = "}$";
			// IP上报逻辑移植
			if(IP_Frame_Valid)  // IP请求已关闭且IP帧有效
			{
				// 第一步：发送统一的head（{#\r\n）
				HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
				vTaskDelay(2);
				// 第二步：发送IP内容（仅保留核心信息，不带head/end）
				sprintf(err_info,"IP:%d.%d.%d.%d\r\n", 
						(uint8_t)Received_IP[0], 
						(uint8_t)Received_IP[1], 
						(uint8_t)Received_IP[2], 
						(uint8_t)Received_IP[3]);
				report_interface(err_info, strlen(err_info));
				vTaskDelay(2);
				// 第三步：发送统一的end（}$）
				HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
				vTaskDelay(500);
			}
			else  // IP未知
			{
				// 同样对齐格式：head + 内容 + end
				HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
				vTaskDelay(2);
				sprintf(err_info,"IP:unknow..\r\n");
				report_interface(err_info, strlen(err_info));
				vTaskDelay(2);
				HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
				vTaskDelay(500);
			}
			if( RobotControlParam.LowPowerFlag )
			{
				sprintf(err_info,"%s","{#\r\n小车电量低，请充电}$");
				report_interface(err_info,strlen(err_info));
				vTaskDelay(500);
				if( RobotControlParam.DebugLevel==0 ) break;
			}
			
			if( RobotControlParam.ErrNum!=0 )
			{
				HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
				for(uint8_t i=0;i<sizeof(errorMap)/sizeof(errorMap[0]);i++)
				{
					if( Get_RobotErrorCode(errorMap[i].errCode) )
					{
						sprintf(err_info,"%s",errorMap[i].errMessage);
						report_interface(err_info,strlen(err_info));
					}
				}
				vTaskDelay(2);
				HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
				vTaskDelay(3000);
				if( RobotControlParam.DebugLevel==0 ) break;
			}
			else if( RobotControlParam.ErrNum==0 && RobotControlParam.DebugLevel == 1 )
			{	//仅在调试等级为1时提示状态正常的情况
				sprintf(err_info,"%s","{#\r\n小车使能状态正常}$");
				report_interface(err_info,strlen(err_info));
				vTaskDelay(3000);
				if( RobotControlParam.DebugLevel==0 ) break;
			}

			//小车的详细自检参数汇报
			if( RobotControlParam.DebugLevel > 1 )
			{
				HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
				
				sprintf(err_info,"小车状态\r\n");
				report_interface(err_info,strlen(err_info));
				
				//车型信息
				sprintf(err_info,"    当前车型：");
				report_interface(err_info,strlen(err_info));
				switch( RobotHardWareParam.CarType )
				{
					case S300: sprintf(err_info,"S300\r\n"); break;
					case S300Mini: sprintf(err_info,"S300 Mini \r\n"); break;
					case S200: sprintf(err_info,"S200\r\n"); break;
					case S200_OUTDOOR: sprintf(err_info,"S200 OUTDOOR\r\n"); break;
					case S260: sprintf(err_info,"S260\r\n"); break;
					case S100: sprintf(err_info,"S100\r\n"); break;
				}
				report_interface(err_info,strlen(err_info));
				
				//硬件版本信息
				sprintf(err_info,"    C63A版本：");
				report_interface(err_info,strlen(err_info));
				if( get_HardWareVersion() == HW_1_0 )
					sprintf(err_info,"V1.0\r\n");
				else if( get_HardWareVersion() == HW_1_1  )
					sprintf(err_info,"V1.1\r\n");
				report_interface(err_info,strlen(err_info));
				
				//电压信息
				sprintf(err_info,"    当前电压：%.2f V",RobotControlParam.Vol);
				size_t len = strlen(err_info);
				if (len % 2 != 0) {
					strcat(err_info, " \r\n"); // 如果长度为奇数，追加一个空格
				}
				else strcat(err_info, "\r\n");
				report_interface(err_info,strlen(err_info));
				
				//急停开关信息
				sprintf(err_info,"    急停开关：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.Enkeystate == 0 )
					sprintf(err_info,"按下\r\n");
				else
					sprintf(err_info,"弹起\r\n");
				report_interface(err_info,strlen(err_info));
				
				//使能状态信息
				sprintf(err_info,"    使能状态：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.en_flag == 1 )
					sprintf(err_info,"使能");
				else
					sprintf(err_info,"失能，错误码：%lu",(unsigned long)RobotControlParam.ErrNum);
				len = strlen(err_info);
				if (len % 2 != 0) {
					strcat(err_info, " \r\n"); // 如果长度为奇数，追加一个空格
				}
				else strcat(err_info, "\r\n");
				report_interface(err_info,strlen(err_info));
				
				//驻车模式信息
				sprintf(err_info,"    驻车模式：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.ParkingMode == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//IMU直线补偿
				sprintf(err_info,"    IMU 直线补偿：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.ImuAssistedFlag == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//超声波安全保护信息
				sprintf(err_info,"    超声波安全保护：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.RangerAvoidEN == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				sprintf(err_info,"    超声波急停状态：");
				report_interface(err_info,strlen(err_info));
				switch( RobotControlParam.RangerHardStopState )
				{
					case RangerStopState_Braking: sprintf(err_info,"BRAKING\r\n"); break;
					case RangerStopState_Hold: sprintf(err_info,"HOLD\r\n"); break;
					default: sprintf(err_info,"IDLE\r\n"); break;
				}
				report_interface(err_info,strlen(err_info));
				
				//自动回充模式
				sprintf(err_info,"    自动回充模式：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.ChargeMode == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//紧急模式
				sprintf(err_info,"    紧急模式：");
				report_interface(err_info,strlen(err_info));
				if( RobotControlParam.EmergencyMode == 0 )
					sprintf(err_info,"关闭\r\n");
				else
					sprintf(err_info,"启用\r\n");
				report_interface(err_info,strlen(err_info));
				
				//控制状态
				sprintf(err_info,"    控制方式：");
				report_interface(err_info,strlen(err_info));
				switch( RobotControl_CMDsource )
				{
					case NONE_CMD: sprintf(err_info,"无\r\n");break;
					case BootLoader: sprintf(err_info,"系统控制\r\n");break;
					case GamePad_CMD: sprintf(err_info,"PS2 \r\n");break;
					case ROS_CMD: sprintf(err_info,"ROS \r\n");break;
					case RCJOY_CMD: sprintf(err_info," R-C航模\r\n");break;
					case APP_CMD: sprintf(err_info," APP\r\n");break;
					case CAN_CMD: sprintf(err_info," CAN\r\n");break;
					case Charger_CMD: sprintf(err_info,"回充装备\r\n");break;
					case UnKnownCMD: sprintf(err_info,"未经允许的控制\r\n");break;
				}
				report_interface(err_info,strlen(err_info));
				
				vTaskDelay(2);
				HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
				vTaskDelay(1500);
				if( RobotControlParam.DebugLevel==0 ) break;
				
				HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
				sprintf(err_info,"自动回充设备状态\r\n");
				report_interface(err_info,strlen(err_info));
				
				if( ChargeDev.online==0 )
				{
					sprintf(err_info,"    设备已离线\r\n");
					report_interface(err_info,strlen(err_info));
				}
				else
				{
					//在线状态
					sprintf(err_info,"    在线状态：");
					report_interface(err_info,strlen(err_info));
					if( ChargeDev.online == 1 )
						sprintf(err_info,"在线\r\n");
					else
						sprintf(err_info,"离线\r\n");
					report_interface(err_info,strlen(err_info));
					
					//充电状态
					sprintf(err_info,"    充电状态：");
					report_interface(err_info,strlen(err_info));
					if( ChargeDev.ChargingFlag==1 )
						sprintf(err_info,"充电中\r\n");
					else
						sprintf(err_info,"未充电\r\n");
					report_interface(err_info,strlen(err_info));
					
					//电流值
					sprintf(err_info,"    测量电流值：%.2fA",(float)ChargeDev.ChargingCur/1000.0f);
					len = strlen(err_info);
					if (len % 2 != 0) {
						strcat(err_info, " \r\n"); // 如果长度为奇数，追加一个空格
					}
					else strcat(err_info, "\r\n");
					report_interface(err_info,strlen(err_info));
					
					//电压值
					sprintf(err_info,"    测量电压值：%.2fV",ChargeDev.ChargingVol);
					len = strlen(err_info);
					if (len % 2 != 0) {
						strcat(err_info, " \r\n"); // 如果长度为奇数，追加一个空格
					}
					else strcat(err_info, "\r\n");
					report_interface(err_info,strlen(err_info));
					
					//红外信号状态
					sprintf(err_info,"    红外信号状态：%d 个信号，%d %d %d %d ",ChargeDev.RedNum,
						ChargeDev.L_A,ChargeDev.L_B,ChargeDev.R_B,ChargeDev.R_A);
					len = strlen(err_info);
					if (len % 2 != 0) {
						strcat(err_info, " \r\n"); // 如果长度为奇数，追加一个空格
					}
					else strcat(err_info, "\r\n");
					report_interface(err_info,strlen(err_info));
				}

				vTaskDelay(2);
				HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
				vTaskDelay(1500);
				if( RobotControlParam.DebugLevel==0 ) break;
			}
			
			//最高调试等级,提示轮毂电机的所有状态
			if( RobotControlParam.DebugLevel > 2 )
			{
				for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
				{
					HAL_UART_Transmit(&huart2,(uint8_t*)head,3,500);
					
					sprintf(err_info,"%2d号驱动器状态：\r\n",g_ServoDriveList[i]->NodeID);
					report_interface(err_info,strlen(err_info));
					
					//离线时简化报错内容
					if( g_ServoDriveList[i]->onlineFlag==0 )
					{
						sprintf(err_info,"    驱动已离线\r\n");
						report_interface(err_info,strlen(err_info));
					}
					else
					{
						sprintf(err_info,"    驱动器地址：%2d\r\n",g_ServoDriveList[i]->NodeID);
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    软件版本号：%6d\r\n",g_ServoDriveList[i]->SoftWareVer);
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    CAN 通信质量：%.2f%%",g_ServoDriveList[i]->comm_qua);
						size_t len = strlen(err_info);
						if (len % 2 != 0) {
							strcat(err_info, " \r\n"); // 如果长度为奇数，追加一个空格
						}
						else strcat(err_info, "\r\n");
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    在线状态：");
						report_interface(err_info,strlen(err_info));
						if( g_ServoDriveList[i]->onlineFlag==1 )
							sprintf(err_info,"在线\r\n");
						else
							sprintf(err_info,"离线\r\n");
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    使能状态：");
						report_interface(err_info,strlen(err_info));
						if( g_ServoDriveList[i]->motorEnflag==1 )
							sprintf(err_info,"使能\r\n");
						else
							sprintf(err_info,"失能\r\n");
						report_interface(err_info,strlen(err_info));
						
						sprintf(err_info,"    报错状态：");
						report_interface(err_info,strlen(err_info));
						if( g_ServoDriveList[i]->errFlag==1 )
							sprintf(err_info,"存在报错\r\n");
						else
							sprintf(err_info,"无报错\r\n");
						report_interface(err_info,strlen(err_info));
						
						//报错时显示报错内容
						if( g_ServoDriveList[i]->errFlag==1 )
						{
							if( g_ServoDriveList[i]->LeftMotorErr!=0 )
							{
								sprintf(err_info,"    左电机存在以下错误：\r\n");
								report_interface(err_info,strlen(err_info));
								for(uint8_t kk=0;kk<sizeof(ServoErrMap)/sizeof(ServoErrMap[0]);kk++)
								{
									if( (g_ServoDriveList[i]->LeftMotorErr)&(ServoErrMap[kk].errCode) )
									{
										sprintf(err_info,"    * %s",ServoErrMap[kk].errMessage);
										report_interface(err_info,strlen(err_info));
									}
								}
							}
							
							if( g_ServoDriveList[i]->RightMotorErr!=0 )
							{
								sprintf(err_info,"    右电机存在以下错误：\r\n");
								report_interface(err_info,strlen(err_info));
								for(uint8_t kk=0;kk<sizeof(ServoErrMap)/sizeof(ServoErrMap[0]);kk++)
								{
									if( (g_ServoDriveList[i]->RightMotorErr)&(ServoErrMap[kk].errCode) )
									{
										sprintf(err_info,"    * %s",ServoErrMap[kk].errMessage);
										report_interface(err_info,strlen(err_info));
									}
								}
							}
						}
					}

					vTaskDelay(2);
					HAL_UART_Transmit(&huart2,(uint8_t*)end,2,500);
					vTaskDelay(1500);
					if( RobotControlParam.DebugLevel==0 ) break;
				}
			}

		}
	}
}


