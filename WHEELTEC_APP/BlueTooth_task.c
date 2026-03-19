/**
 * @file    BlueTooth_task.c
 * @brief   蓝牙APP数据解析.
 * @author  WHEELTEC
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - 文件包含对蓝牙模块的AT反馈指令过滤,以防止在连接和断开蓝牙时收到AT指令的干扰
 * - 主要任务是BlueToothControlTask,在串口接受到指令后将会写队列,由调度器唤醒此
 *   任务进行数据处理。
 * 
 *
 * @note
 * 
 * 
 */
#include "BlueTooth_task.h"

//HAL Lib Include File
#include "usart.h"

//C Include File
#include <string.h>
#include <math.h>
#include <stdio.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_ServoDrive.h"
#include "bsp_flash.h"
#include "bsp_buzzer.h"
#include "bsp_icm20948.h"


//APP Include File
#include "RobotControl_task.h"
#include "show_task.h"

enum{
	AppKeyNone = 0,
	AppKeyFront = 1,
	AppKeyFR=2,
	AppKeyRight=3,
	AppKeyBR=4,
	AppKeyBack=5,
	AppKeyBL = 6,
	AppKeyLeft=7,
	AppKeyFL = 8,
	APPKeyUp = 0x18,
	APPKeyDown = 0x19,
	APPKeyStop = 0x1A,
};


//蓝牙AT指令抓包
static uint8_t ATCommandFeedBack_BT04A(uint8_t recv);
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv);

//WHEELTEC 手机APP
WHEELTEC_APPKey_t wheeltecApp = { 0 };

//蓝牙任务
void BlueToothControlTask(void* param)
{
	uint8_t recv = 0;     //用于接收队列数据
	uint8_t lastrecv = 0;
	uint8_t app_page = 1; //手机APP控制页的页数. 0:重力页面 1:摇杆页面 2:按键页面
	uint8_t controlKey = 0; //控制值
	
	//APP调参页面辅助参数
	static uint8_t paramFlag=0,param_i=0,param_j=0,paramReceive[50]={0};
	float paramData=0;
	
	//写控制队列
	extern QueueHandle_t g_xQueueBlueTooth;
	
	//APP控制解锁
	uint8_t AppUnlock = 0;
	
	pBuzzerInterface_t tips = &UserBuzzer;
	while( 1 )
	{
		if( pdPASS == xQueueReceive(g_xQueueBlueTooth,&recv,portMAX_DELAY) ) 
		{
			uint8_t writeflag = 1;
			
			_System_Reset_FromAPP_RTOS(recv);
			
			//APP控制解锁
			if( lastrecv == 'A' && recv=='A' && AppUnlock==0)
				AppUnlock=1;
			
			RobotControlCMDType_t cmd = {
				.cmdsource = APP_CMD,//标注控制源,APP控制
				0,0,0
			};
			
			//过滤蓝牙的AT指令反馈
			uint8_t ATFilter = 0;
			ATFilter += ATCommandFeedBack_JDY33(recv);
			ATFilter += ATCommandFeedBack_BT04A(recv);
			if( ATFilter > 0 ) continue; //任意存在一个蓝牙的过滤信息,则过滤
				
			//调试等级设置
			RobotControl_SetDebugLevel(recv);
			
			/* APP按键页面切换 */
			if( recv == 'K' ) app_page = 2;      //按键页面
			else if( recv == 'J' ) app_page = 1; //摇杆页面
			else if( recv == 'I' ) app_page = 0; //重力球页面
			
			//页面
			wheeltecApp.page = app_page;
			
			//读取控制信息
			uint8_t Tmpcontrolkey = recv-0x40;
			
			//对于控制的键值,需要进行二次确认.收到连续的键值才使能控制
			if( Tmpcontrolkey >= AppKeyFront && Tmpcontrolkey <= AppKeyFL )
			{
				if( lastrecv-0x40 == Tmpcontrolkey ) controlKey = Tmpcontrolkey;
			}
			else controlKey = Tmpcontrolkey;
			
			//保存上次接收数据情况
			lastrecv = recv;
			
			//方向按键
			wheeltecApp.dirkey = controlKey;
			
			float base_vz = PI/4.0f;
				
			switch( controlKey )
			{
				case AppKeyFront:  
					cmd.Vx = RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = 0; 
				break;
				
				case AppKeyFR:     
					cmd.Vx = RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = -base_vz;
				break;
				
				case AppKeyRight:  
					cmd.Vx = 0; cmd.Vy=0; cmd.Vz = -base_vz;
				break; 
				
				case AppKeyBR:     
					cmd.Vx = -RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = +base_vz;
				break;
				
				case AppKeyBack:  
					cmd.Vx = -RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = 0;
				break;
				
				case AppKeyBL:     
					cmd.Vx = -RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = -base_vz;
				break;
				
				case AppKeyLeft:   
					cmd.Vx = 0; cmd.Vy=0; cmd.Vz = +base_vz;
				break;
				
				case AppKeyFL:     
					cmd.Vx = RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = +base_vz;
				break;
				
				case APPKeyStop:   
					cmd.Vx = 0; cmd.Vy=0; cmd.Vz = 0;
				break;
				
				default: writeflag = 0;  break;
			}
			
			//写入控制队列,控制小车
			if( writeflag && AppUnlock )
			{
				if(WriteRobotControlQueue(&cmd,0))
				{
					AppUnlock = 0;//若写入不成功,则上锁APP控制
				}
			}
			
			//加减速设置
			if( recv == 'X' )
			{
				RobotControlParam.defalutSpeed+=100;
				tips->AddTask(1,200);
			}	
			else if( recv == 'Y' )
			{
				RobotControlParam.defalutSpeed-=100;
				tips->AddTask(1,200);
			}
			
			////////////// APP 自定义按键区域 /////////////////
			if( recv == 'a' ) 
			{
				//IMU零点标定
				pIMUInterface_t imu = &UserICM20948;
				imu->UpdateZeroPoint_axis();
				imu->UpdateZeroPoint_attitude();
				tips->AddTask(1,200);//使用蜂鸣器提示操作成功
			}
			else if( recv == 'b' ) 
			{
				//自动回充功能启动/关闭
				RobotControlParam.ChargeMode = !RobotControlParam.ChargeMode;
				tips->AddTask(1,200);
			}
			else if( recv == 'c' )
			{
				//请求清除驱动器报错
				RobotControlParam.DriveErrRecovery=1;
				tips->AddTask(1,200);
			}
			else if( recv == 'd' )
			{
				//应急模式进入与退出.进入按3下,蜂鸣器长鸣.退出按1下,蜂鸣器短鸣
				static uint8_t entryCnt = 0;
				if( RobotControlParam.EmergencyMode==1 )
				{
					entryCnt=0;
					RobotControlParam.EmergencyMode=0;
					tips->AddTask(1,200);
				}
				else
				{
					entryCnt++;
					if( entryCnt==3 ) 
					{	//强制使能所有电机
						for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
						{
							ServoDriver_Enable(g_ServoDriveList[i]);
						}
						RobotControlParam.EmergencyMode=1,tips->AddTask(1,1000);
					}
				}
			}
			else if( recv == 'e' )
			{
				//开启/关闭超声波避障功能
				RobotControlParam.RangerAvoidEN = !RobotControlParam.RangerAvoidEN;
				tips->AddTask(1,200);
			}
			else if( recv == 'f' )
			{	//开启/关闭 imu辅助走直线功能
				RobotControlParam.ImuAssistedFlag = !RobotControlParam.ImuAssistedFlag;
				tips->AddTask(1,200);
			}
			else if( recv == 'g' )
			{	
				oled_page_down();
				tips->AddTask(1,200);
			}
			else if( recv == 'h' )
			{	
				oled_page_up();
				tips->AddTask(1,200);
			}
			//////////////      END          /////////////////
			
			//APP参数页面 数据格式: {?:?}
			if(recv==0x7B) paramFlag=1;        //The start bit of the APP parameter instruction //APP参数指令起始位
			else if(recv==0x7D) paramFlag=2;   //The APP parameter instruction stops the bit    //APP参数指令停止位
			
			if(paramFlag==1) //Collect data //采集数据
			{
				paramReceive[(param_i%50)]=recv;
				param_i++;
			}
			else if(paramFlag==2) //Analyze the data //分析数据
			{
				if(paramReceive[3]==0x50)  // {Q:P} 获取设备参数
				{
					wheeltecApp.reportparam = 1;//获取参数请求
				}
					
				else if( paramReceive[3]==0x57 ) // {Q:W} 设置掉电保存参数
				{
					wheeltecApp.saveflash = 1;//保存Flash参数请求
					//保存驱动器的参数
					ServoDriver_SaveEEPROM(&Driver1);
					ServoDriver_SaveEEPROM(&Driver2);
					ServoDriver_SaveEEPROM(&Driver3);
					
					//写入flash前先保证小车停止
					cmd.cmdsource = BootLoader;
					cmd.Vx=0,cmd.Vy=0,cmd.Vz=0;
					WriteRobotControlQueue(&cmd,0);
					vTaskDelay(200);
					
					//小车初始速度的参数保存
					taskENTER_CRITICAL(); //进入临界区,所有的任务以及中断挂起
					uint32_t writeFlashbuf[2] = { RobotControlParam.defalutSpeed,RobotControlParam.LineDiffParam };
					int8_t res = User_Flash_SaveParam((uint32_t*)writeFlashbuf,2); //写数据
					taskEXIT_CRITICAL(); //写入完毕,退出临界	
					
					if( res==0 ) tips->AddTask(1,1000);
				}					
				
				else  if(paramReceive[1]!=0x23)  // {0:xxx} {1:xxx} {2:xxx} 单通道数值设置
				{
					for(param_j=param_i; param_j>=4; param_j--)
					{
						paramData+=(paramReceive[param_j-1]-48)*pow(10,param_i-param_j);
					}
					switch(paramReceive[1])
					{
						case 0x30: RobotControlParam.defalutSpeed = paramData; tips->AddTask(1,200);break;
						case 0x31: RobotControlParam.LineDiffParam = paramData;tips->AddTask(1,200);break;
						case 0x32: ServoDrive_SetSpeedkp(paramData); tips->AddTask(1,200);break;
						case 0x33: ServoDrive_SetSpeedki(paramData); tips->AddTask(1,200);break;
						case 0x34: ServoDrive_SetCurkp(paramData);   tips->AddTask(1,200);break;
						case 0x35: ServoDrive_SetCurki(paramData);   tips->AddTask(1,200);break;
						case 0x36: break;
						case 0x37: break;
						case 0x38: break;
					}
				}
				else if( paramReceive[1]==0x23 ) //APP上点击“发送所有数据”处理方法  // {#xxx:xxx:xxx...xxx}
				{
					float num=0;
					uint8_t dataIndex=0;
					float dataArray[9]={0};

					if( param_i<=50 ) //数据在可接受范围
					{
						paramReceive[param_i]='}'; //补充帧尾

						for(uint8_t kk=0; paramReceive[kk]!='}'; kk++)
						{
							if( paramReceive[kk]>='0' && paramReceive[kk]<='9' )
							{
								num = num*10 + ( paramReceive[kk] - '0' );
							}
							else if( paramReceive[kk]==':' )
							{
								dataArray[dataIndex++] = num;
								num = 0;
							}

						}
						//处理最后一个数据
						dataArray[dataIndex] = num;
						
						//数据获取
						RobotControlParam.defalutSpeed = dataArray[0];
						RobotControlParam.LineDiffParam = dataArray[1];
						ServoDrive_SetSpeedkp(dataArray[2]);
						ServoDrive_SetSpeedki(dataArray[3]);
						ServoDrive_SetCurkp(dataArray[4]);
						ServoDrive_SetCurki(dataArray[5]);
						
						//蜂鸣器提示操作成功
						tips->AddTask(1,200);
					}
				}				
				//Relevant flag position is cleared
				//相关标志位清零
				paramFlag=0;param_i=0;param_j=0;paramData=0;
				memset(paramReceive, 0, sizeof(uint8_t)*50); //Clear the array to zero//数组清零
			}
			
			//小车设置速度限制
			if( RobotControlParam.defalutSpeed>3000 ) RobotControlParam.defalutSpeed=3000;
			if( RobotControlParam.defalutSpeed<0 ) RobotControlParam.defalutSpeed=0;
			
			//小车纠偏系数限制
			if( RobotControlParam.LineDiffParam > 100 ) RobotControlParam.LineDiffParam = 100;
		}
	}
}


//BT04-A蓝牙的AT指令反馈过滤函数.包含连接反馈与断开反馈
static uint8_t ATCommandFeedBack_BT04A(uint8_t recv)
{
	#define DEBUG_BT04ACommand 0
	
	uint8_t isFilter = 0;//是否允许该字符通过,1过滤,0不处理
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	const char* BT04AConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\n+CONNECTED\r\n";
	const char* BT04ADisConnect = "+DISC:SUCCESS\r\n+READY\r\n+PAIRABLE\r\n";
	enum{
		BT04A_NORMAL=  0,
		BT04A_CONNECTSTART,
		BT04A_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = BT04A_NORMAL;
	
	switch( statemachine )
	{
		case BT04A_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = BT04A_CONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = BT04A_DISCONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			break;
		case BT04A_CONNECTSTART:
			if( BT04AConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
       				  ((recv>='0'&&recv<='9')||(recv>='a'&&recv<='z')) )//进入到MAC地址匹配阶段.该阶段匹配 0~9 、a~z字段
			{
				isFilter = 1;//MAC地址过滤
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				//都不满足,允许字符通过.并退出过滤模式
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand 
				printf("No:get:%c,but:%c\r\n",recv,BT04AConnect[filterIndex]);
				#endif
			}
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(BT04AConnect) )
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand 
				printf("filter con done!\r\n");
				#endif
			}
			break;
		case BT04A_DISCONNECTSTART:
			if( BT04ADisConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand
				printf("No:get:%c,but:%c\r\n",recv,BT04ADisConnect[filterIndex]);
				#endif
			}				
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(BT04ADisConnect) ) 
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand
				printf("filter dis done!\r\n");
				#endif
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}


//JDY-33蓝牙AT指令集过滤
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv)
{
	#define DEBUG_JDY33Command 0
	
	uint8_t isFilter = 0;//是否允许该字符通过,1过滤,0允许
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	const char* JDY33_SPPConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\nCONNECTED\r\r\n";
	const char* JDY33_BLEConnect = "CONNECTED\r\r\n";
	const char* JDY33_DisConnect = "+DISC:SUCCESS\r\r\n\0"; //注意\0不会被统计字长,需要自行把字长+1
	enum{
		JDY33_NORMAL=  0,
		JDY33_SPPCONNECTSTART,
		JDY33_BLECONNECTSTART,
		JDY33_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = JDY33_NORMAL;
	
	switch( statemachine )
	{
		case JDY33_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = JDY33_SPPCONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='O'&&lastrecv=='C' )
			{
				statemachine = JDY33_BLECONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = JDY33_DISCONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='C'&&lastrecv!='C' )//进入状态的判断优先,再到歧义的判断
			{
				//有关C的歧义,"+C"、"CO"不能通过. "?C也不允许通过",才能保证连接时不存在控制命令,若需要控制小车,需要连续的C
				isFilter = 1;//禁止通过
			}
			break;
		case JDY33_SPPCONNECTSTART:
			if( JDY33_SPPConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
       				  ((recv>='0'&&recv<='9')||(recv>='A'&&recv<='Z')) )//进入到MAC地址匹配阶段.该阶段匹配 0~9 、a~z字段
			{
				isFilter = 1;//MAC地址过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				//都不满足,允许字符通过.并退出过滤模式
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP->No:get:%c,but:%c\r\n",recv,JDY33_SPPConnect[filterIndex]);
				#endif
			}
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_SPPConnect) )
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP filter con done!\r\n");
				#endif
			}
			break;
		case JDY33_BLECONNECTSTART:
			if( JDY33_BLEConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("BLE->No:get:%c,but:%c\r\n",recv,JDY33_BLEConnect[filterIndex]);
				#endif
			}				
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_BLEConnect) ) 
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("ble filter dis done!\r\n");
				#endif
			}
			break;
		case JDY33_DISCONNECTSTART:
			if( JDY33_DisConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("dis->No:get:%c,but:%c\r\n",recv,JDY33_DisConnect[filterIndex]);
				#endif
			}				
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_DisConnect)+1 ) //+1为补充空字符'\0'
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("filter dis done!\r\n");
				#endif
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}

