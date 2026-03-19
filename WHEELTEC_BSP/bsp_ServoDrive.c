#include "bsp_ServoDrive.h"

//C Include File
#include <stdio.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_can.h"

//APP Include File
#include "robot_select_init.h"

extern QueueHandle_t g_xQueueCANopenCallback;

//设置调试等级
#define DRIVER_DEBUG_LEVEL 0

//调试日志输出方法
#if (DRIVER_DEBUG_LEVEL > 0U)
#define  Driver_UsrLog(...)   do { \
                            printf(__VA_ARGS__); \
} while (0)
#else
#define Driver_UsrLog(...) do {} while (0)
#endif

//驱动器定义,每个驱动带2个轮毂电机
ServoDriverHWType_t Driver1 = {
	.NodeID = 1,
	.comm_qua = 100,
	.comm_cnt = 0,
	.comm_Errcnt = 0,
	.time_tick = 0,
	.onlineFlag = 0,
	.errFlag = 0    
};

ServoDriverHWType_t Driver2 = {
	.NodeID = 2,
	.comm_qua = 100,
	.comm_cnt = 0,
	.comm_Errcnt = 0,
	.time_tick = 0,
	.onlineFlag = 0,
	.errFlag = 0  
};

ServoDriverHWType_t Driver3 = {
	.NodeID = 3,
	.comm_qua = 100,
	.comm_cnt = 0,
	.comm_Errcnt = 0,
	.time_tick = 0,
	.onlineFlag = 0,
	.errFlag = 0  
};

//可用驱动列表
ServoDriverHWType_t* g_ServoDriveList[] = {&Driver1,&Driver2,&Driver3};

//所有伺服电机PID参数
ServoDriverPIDType_t DrivePid = { 0 };

//复位驱动器的状态数据
static void DriverInfoReset(ServoDriverHWType_t* driver)
{
	driver->comm_cnt=0;
	driver->comm_Errcnt=0;
	driver->comm_qua=0;
	driver->errFlag=0;
	driver->LeftMotorErr=0;
	driver->onlineFlag=0;
	driver->RightMotorErr=0;
	driver->time_tick=0;
	driver->motorEnflag=0;
}

//比较CAN通信内容是否符合预期值
static uint8_t compare_canopen_msg(CANmsgType_t* a,CANmsgType_t* b)
{
	if( a->id == b->id )
	{
		for(uint8_t i=0;i<8;i++)
		{
			if( a->buffer[i]!=b->buffer[i] ) return 0;
		}
		
		return 1;
	}
	
	return 0;
}

//获取CANopen-RSDO服务器端写数据时内容
static void CANopen_getSendList(uint8_t id,uint16_t addr,uint8_t index,uint8_t byteNum,uint32_t writedata,CANmsgType_t* sendlist,CANmsgType_t* fblist)
{
	if( id<1 || id > 127 ) return;
	
	sendlist->id = 0x600 + id;
	uint8_t i=0;
	
	//设置命令
	if( byteNum==1 ) sendlist->buffer[0]=0x2F;
	else if( byteNum==2 ) sendlist->buffer[0]=0x2B;
	else if( byteNum==3 ) sendlist->buffer[0]=0x27;
	else sendlist->buffer[0]=0x23;
	
	//寄存器地址,低位在前
	sendlist->buffer[1]=addr;
	sendlist->buffer[2]=addr>>8;
	
	//索引值
	sendlist->buffer[3]=index;
	
    //要写入的数据从低位向高位排序取出
    for(i=0;i<4;i++)
    {
        sendlist->buffer[4+i] = ( writedata>>(8*i) )&0xff;
    }
	
	fblist->id = 0x580 + id;
	fblist->buffer[0]=0x60;
	fblist->buffer[1]=sendlist->buffer[1];
	fblist->buffer[2]=sendlist->buffer[2];
	fblist->buffer[3]=sendlist->buffer[3];
	fblist->buffer[4]=0;
	fblist->buffer[5]=0;
	fblist->buffer[6]=0;
	fblist->buffer[7]=0;
}

//获取CANopen-RSDO服务器端读数据时内容
static void CANopen_getReadList(uint8_t id,uint16_t addr,uint8_t index,uint8_t byteNum,CANmsgType_t* sendlist)
{
	if( id<1 || id > 127 ) return;
	
	sendlist->id = 0x600 + id;
	uint8_t i=0;
	
	//读取命令
	sendlist->buffer[0]=0x40;

	//寄存器地址,低位在前
	sendlist->buffer[1]=addr;
	sendlist->buffer[2]=addr>>8;
	
	//索引值
	sendlist->buffer[3]=index;
	
    //读取时末尾无数值
    for(i=0;i<4;i++)
    {
        sendlist->buffer[4+i] = 0;
    }
}

//在FreeRTOS环境下读数据
int8_t CANopenRSDO_Read_RTOS(ServoDriverHWType_t* driver,uint16_t addr,uint8_t index,uint8_t byteNum,uint32_t* readdata,uint8_t retrytime)
{
	int8_t res = -1;
	
	//指定使用的CAN设备
	pCANInterface_t candev = &UserCAN2Dev;
	
	//获取数据发送列表
	CANmsgType_t send,fbreal;
	CANopen_getReadList(driver->NodeID,addr,index,byteNum,&send);
	
	//发送数据并等待处理反馈
	uint8_t times = 1;
	if( retrytime!=0 ) times = retrytime;
	while( times-- )
	{
		driver->comm_cnt++;//统计通信的次数
		
		BaseType_t queue_state;
		
		//发送数据
		candev->sendStd(send.id,send.buffer,8);
		
		//读取反馈
		queue_state = xQueueReceive(g_xQueueCANopenCallback,&fbreal,pdMS_TO_TICKS(5));
		
		if( queue_state==pdTRUE )
		{
//			printf("id=%X\r\n",fbreal.id);
//			for(uint8_t i=0;i<8;i++)
//				printf("%02X\t",fbreal.buffer[i]);
//			printf("\r\n");
			
			//比较反馈结果与预期结果
			if( fbreal.id == driver->NodeID+0x580 && (fbreal.buffer[0]==0x4F || fbreal.buffer[0]==0x4B || fbreal.buffer[0]==0x47 ||fbreal.buffer[0]==0x43) )
			{
				*readdata = fbreal.buffer[7]<<24 | fbreal.buffer[6]<<16 | fbreal.buffer[5]<<8 | fbreal.buffer[4];
				res = 0;
				break;		
			}
			else
			{
//				printf("read:err\r\n");
				driver->comm_Errcnt++;
				res = -2;//反馈内容错误
			}
		}
		else 
		{
//			printf("read:timeout\r\n");
//			printf("id=%X\r\n",send.id);
//			for(uint8_t i=0;i<8;i++)
//				printf("%02X\t",send.buffer[i]);
//			printf("\r\n");
	
			res = -1 , driver->comm_Errcnt++;//超时	
		}
	}

	//检测10秒内驱动的通信质量
	float err = ((float)driver->comm_Errcnt/(float)driver->comm_cnt)*100;
	driver->comm_qua = 100.0f-err;
	if( xTaskGetTickCount()-driver->time_tick > 10000 )
	{
		driver->comm_Errcnt=0;
		driver->comm_cnt=0;
		driver->time_tick=xTaskGetTickCount();
	}
	
	//通信质量小于50.0%,判定为驱动器离线
//	if( driver->comm_qua <= 50.0f )
//	{
//		driver->onlineFlag = 0;
//	}
	
	return res;
}

//在FreeRTOS环境下写数据
int8_t CANopenRSDO_Write_RTOS(ServoDriverHWType_t* driver,uint16_t addr,uint8_t index,uint8_t byteNum,uint32_t writedata,uint8_t retrytime)
{
	int8_t res = -1;
	
	//指定使用的CAN设备
	pCANInterface_t candev = &UserCAN2Dev;
	
	//获取数据发送列表
	CANmsgType_t send,fbwait,fbreal;
	CANopen_getSendList(driver->NodeID,addr,index,byteNum,writedata,&send,&fbwait);
	
	//发送数据并等待处理反馈
	uint8_t times = 1;
	if( retrytime!=0 ) times = retrytime;
	while( times-- )
	{
		driver->comm_cnt++;//统计通信的次数
		
		BaseType_t queue_state;
		
		//发送数据
		candev->sendStd(send.id,send.buffer,8);
		
		//读取反馈
		queue_state = xQueueReceive(g_xQueueCANopenCallback,&fbreal,pdMS_TO_TICKS(5));
		
		//比较反馈结果与预期结果
		uint8_t compare_state = compare_canopen_msg(&fbwait,&fbreal);
		
//		printf("id=%X\r\n",fbreal.id);
//		for(uint8_t i=0;i<8;i++)
//			printf("%02X\t",fbreal.buffer[i]);
//		printf("\r\n");
		
		//实际反馈的内容与预期相同,本次通信成功
		if( queue_state==pdTRUE )
		{
			if( compare_state == 1 )
			{
				res=0;
				break;
			}
			else 
			{
//				printf("write:err\r\n");
				res = -2,driver->comm_Errcnt++;//反馈内容错误
			}
		}
		else 
		{
//			printf("write:timeout\r\n");
			res = -1,driver->comm_Errcnt++;//超时	
		}
	}
	
	//检测10秒内驱动的通信质量
	float err = ((float)driver->comm_Errcnt/(float)driver->comm_cnt)*100;
	driver->comm_qua = 100.0f-err;
	if( xTaskGetTickCount()-driver->time_tick > 10000 )
	{
		driver->comm_Errcnt=0;
		driver->comm_cnt=0;
		driver->time_tick=xTaskGetTickCount();
	}
	
	//通信质量小于50.0%,判定为驱动器离线
//	if( driver->comm_qua <= 50.0f )
//	{
//		driver->onlineFlag = 0;
//	}
	
	return res;
}

//编码器参数设置
int8_t set_encoder_accuracy(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;
	
	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x200E,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200E,1,2,param,10);
	
	Driver_UsrLog("Lencoder:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x200E,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200E,2,2,param,10);
	
	Driver_UsrLog("Rencoder:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_HallOffset(ServoDriverHWType_t* driver,short param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;
	
	//限幅
	if(param<-360) param=-360;
	if(param> 360) param=360;
	
	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2011,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2011,1,2,param,10);
	
	Driver_UsrLog("L_Hall:%d\r\n",(short)tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2011,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2011,2,2,param,10);
	
	Driver_UsrLog("R_Hall:%d\r\n",(short)tmpRead);
	
	return comm_state;
}

int8_t set_poles(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param<4) param=4;
	if(param>64) param=64;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x200C,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200C,1,2,param,10);
	
	Driver_UsrLog("L_poles:%d\r\n",(short)tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x200C,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200C,2,2,param,10);
	
	Driver_UsrLog("R_poles:%d\r\n",(short)tmpRead);
	
	return comm_state;
}


int8_t set_rated_current(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>150) param=150;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2014,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2014,1,2,param,10);
	
	Driver_UsrLog("L_cur:%d\r\n",(short)tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2014,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2014,2,2,param,10);
	
	Driver_UsrLog("R_cur:%d\r\n",(short)tmpRead);
	
	return comm_state;
}


int8_t set_max_current(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>300) param=300;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2015,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2015,1,2,param,10);
	
	Driver_UsrLog("L_MAXcur:%d\r\n",(short)tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2015,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2015,2,2,param,10);
	
	Driver_UsrLog("R_MAXcur:%d\r\n",(short)tmpRead);
	
	return comm_state;
}

int8_t set_max_rpm(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param<1)    param=1;
	if(param>3000) param=3000;

	comm_state = CANopenRSDO_Read_RTOS(driver,0x2008,0,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2008,0,2,param,10);
	
	Driver_UsrLog("MAXRpm:%d\r\n",(short)tmpRead);
	
	return comm_state;
}

int8_t set_overload_factor(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>300) param=300;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2012,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2012,1,2,param,10);
	
	Driver_UsrLog("L_OverLoad:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2012,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2012,2,2,param,10);
	
	Driver_UsrLog("R_OverLoad:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_overload_time(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>6553) param=6553;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2016,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2016,1,2,param,10);
	
	Driver_UsrLog("L_OverLoadTime:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2016,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2016,2,2,param,10);
	
	Driver_UsrLog("R_OverLoadTime:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_encoder_ErrorAlarm(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>6553) param=6553;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2017,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2017,1,2,param,10);
	
	Driver_UsrLog("L_Encoderdiff:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2017,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2017,2,2,param,10);
	
	Driver_UsrLog("R_Encoderdiff:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_maxTemp(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>1200) param=1200;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2013,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2013,1,2,param,10);
	
	Driver_UsrLog("L_MaxTemp:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2013,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2013,2,2,param,10);
	
	Driver_UsrLog("R_MaxTemp:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_pos_kp(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2020,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2020,1,2,param,10);
	
	Driver_UsrLog("L_PosKp:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2020,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2020,2,2,param,10);
	
	Driver_UsrLog("R_PosKp:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_pos_kf(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2021,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2021,1,2,param,10);
	
	Driver_UsrLog("L_PosKf:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2021,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2021,2,2,param,10);
	
	Driver_UsrLog("R_PosKf:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t get_speed_kp(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

//	//左轮
//	comm_state = CANopenRSDO_Read_RTOS(driver,0x201D,1,2,&tmpRead,10);
//	if( tmpRead!=param )
//		comm_state = CANopenRSDO_Write_RTOS(driver,0x201D,1,2,param,10);
//	
//	Driver_UsrLog("L_speedKp:%d\r\n",tmpRead);
//	
//	//右轮
//	comm_state = CANopenRSDO_Read_RTOS(driver,0x201D,2,2,&tmpRead,10);
//	if( tmpRead!=param )
//		comm_state = CANopenRSDO_Write_RTOS(driver,0x201D,2,2,param,10);
//	
//	Driver_UsrLog("R_speedKp:%d\r\n",tmpRead);
	
	//读取PID参数,仅用单边轮子的数据参考
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201D,1,2,&tmpRead,10);
	DrivePid.SpeedKp = tmpRead;
	
	Driver_UsrLog("speedKp:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t ServoDrive_SetSpeedkp(uint16_t param)
{
	int8_t comm_state = 0;

	//限幅
	if(param>30000) param=30000;

	#define DriveNum 3

	for(uint8_t i=0;i<DriveNum;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag==1)
		{
			//设置参数
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201D,1,2,param,10);
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201D,2,2,param,10);
		}
	}
	
	//仅在全部驱动写入成功时更新参数
	if( comm_state==0 ) DrivePid.SpeedKp = param;
	
	return comm_state;
}

int8_t get_speed_ki(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

//	//左轮
//	comm_state = CANopenRSDO_Read_RTOS(driver,0x201E,1,2,&tmpRead,10);
//	if( tmpRead!=param )
//		comm_state = CANopenRSDO_Write_RTOS(driver,0x201E,1,2,param,10);
//	
//	Driver_UsrLog("L_speedKi:%d\r\n",tmpRead);
//	
//	//右轮
//	comm_state = CANopenRSDO_Read_RTOS(driver,0x201E,2,2,&tmpRead,10);
//	if( tmpRead!=param )
//		comm_state = CANopenRSDO_Write_RTOS(driver,0x201E,2,2,param,10);
//	
//	Driver_UsrLog("R_speedKi:%d\r\n",tmpRead);
	
	//读取PID参数,仅用单边轮子的数据参考
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201E,1,2,&tmpRead,10);
	DrivePid.SpeedKi = tmpRead;
	
	Driver_UsrLog("speedKi:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t ServoDrive_SetSpeedki(uint16_t param)
{
	int8_t comm_state = 0;

	//限幅
	if(param>30000) param=30000;

	#define DriveNum 3

	for(uint8_t i=0;i<DriveNum;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag==1)
		{
			//设置参数
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201E,1,2,param,10);
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201E,2,2,param,10);
		}
	}
	
	//仅在全部驱动写入成功时更新参数
	if( comm_state==0 ) DrivePid.SpeedKi = param;
	
	return comm_state;
}


int8_t set_speed_kf(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201F,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201F,1,2,param,10);
	
	Driver_UsrLog("L_speedKf:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201F,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201F,2,2,param,10);
	
	Driver_UsrLog("R_speedKf:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_speed_smooth(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2018,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2018,1,2,param,10);
	
	Driver_UsrLog("L_speed_smooth:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2018,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2018,2,2,param,10);
	
	Driver_UsrLog("R_speed_smooth:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_smooth_kf(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201B,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201B,1,2,param,10);
	
	Driver_UsrLog("L_smoothKf:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201B,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201B,2,2,param,10);
	
	Driver_UsrLog("R_smoothKf:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_tor_smooth(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201C,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201C,1,2,param,10);
	
	Driver_UsrLog("L_torKf:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201C,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201C,2,2,param,10);
	
	Driver_UsrLog("R_torKf:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t get_curloop_kp(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

//	//左轮
//	comm_state = CANopenRSDO_Read_RTOS(driver,0x2019,1,2,&tmpRead,10);
//	if( tmpRead!=param )
//		comm_state = CANopenRSDO_Write_RTOS(driver,0x2019,1,2,param,10);
//	
//	Driver_UsrLog("L_CurKp:%d\r\n",tmpRead);
//	
//	//右轮
//	comm_state = CANopenRSDO_Read_RTOS(driver,0x2019,2,2,&tmpRead,10);
//	if( tmpRead!=param )
//		comm_state = CANopenRSDO_Write_RTOS(driver,0x2019,2,2,param,10);
//	
//	Driver_UsrLog("R_CurKp:%d\r\n",tmpRead);
	
	//读取PID参数,仅用单边轮子的数据参考
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2019,1,2,&tmpRead,10);
	DrivePid.CurKp = tmpRead;
	
	Driver_UsrLog("CurKp:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t ServoDrive_SetCurkp(uint16_t param)
{
	int8_t comm_state = 0;

	//限幅
	if(param>30000) param=30000;

	#define DriveNum 3

	for(uint8_t i=0;i<DriveNum;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag==1)
		{
			//设置参数
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x2019,1,2,param,10);
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x2019,2,2,param,10);
		}
	}
	
	//仅在全部驱动写入成功时更新参数
	if( comm_state==0 ) DrivePid.CurKp = param;

	
	return comm_state;
}


int8_t get_curloop_ki(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//限幅
	if(param>30000) param=30000;

//	//左轮
//	comm_state = CANopenRSDO_Read_RTOS(driver,0x201A,1,2,&tmpRead,10);
//	if( tmpRead!=param )
//		comm_state = CANopenRSDO_Write_RTOS(driver,0x201A,1,2,param,10);
//	
//	Driver_UsrLog("L_CurKi:%d\r\n",tmpRead);
//	
//	//右轮
//	comm_state = CANopenRSDO_Read_RTOS(driver,0x201A,2,2,&tmpRead,10);
//	if( tmpRead!=param )
//		comm_state = CANopenRSDO_Write_RTOS(driver,0x201A,2,2,param,10);
//	
//	Driver_UsrLog("R_CurKi:%d\r\n",tmpRead);
	
	//读取PID参数,仅用单边轮子的数据参考
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201A,1,2,&tmpRead,10);
	DrivePid.CurKi = tmpRead;
	
	Driver_UsrLog("CurKi:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t ServoDrive_SetCurki(uint16_t param)
{
	int8_t comm_state = 0;

	//限幅
	if(param>30000) param=30000;

	#define DriveNum 3

	for(uint8_t i=0;i<DriveNum;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag==1)
		{
			//设置参数
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201A,1,2,param,10);
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201A,2,2,param,10);
		}
	}
	
	//仅在全部驱动写入成功时更新参数
	if( comm_state==0 ) DrivePid.CurKi = param;
	
	return comm_state;
}

int8_t set_SynCon_state(ServoDriverHWType_t* driver,uint8_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	if(param!=0) param=1;

	comm_state = CANopenRSDO_Read_RTOS(driver,0x200F,0,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200F,0,2,param,10);
	
	Driver_UsrLog("SynMode:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_acctime(ServoDriverHWType_t* driver,uint32_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	if(param>32767) param=32767;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6083,1,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6083,1,4,param,10);
	
	Driver_UsrLog("L_AccTime:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6083,2,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6083,2,4,param,10);
	
	Driver_UsrLog("R_AccTime:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_dectime(ServoDriverHWType_t* driver,uint32_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	if(param>32767) param=32767;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6084,1,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6084,1,4,param,10);
	
	Driver_UsrLog("L_DecTime:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6084,2,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6084,2,4,param,10);
	
	Driver_UsrLog("R_DecTime:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_stoptime(ServoDriverHWType_t* driver,uint32_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	if(param>32767) param=32767;

	//左轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6085,1,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6085,1,4,param,10);
	
	Driver_UsrLog("L_StopTime:%d\r\n",tmpRead);
	
	//右轮
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6085,2,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6085,2,4,param,10);
	
	Driver_UsrLog("R_StopTime:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_hearbeatTime(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//每个节点设置有一定间隔的心跳时间
	param = param + driver->NodeID*150;

	comm_state = CANopenRSDO_Read_RTOS(driver,0x1017,0,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x1017,0,2,param,10);
	
	Driver_UsrLog("HearTime:%d\r\n",tmpRead);
	
	return comm_state;
}

//设置电机控制模式
/*
	1:相对位置模式
	2:绝对位置模式
	3:速度模式
	4:转矩模式
*/
int8_t set_ControlMode(ServoDriverHWType_t* driver,uint8_t param)
{
	int8_t comm_state = 0;
	if(param>4) param=0;
	
	comm_state = CANopenRSDO_Write_RTOS(driver,0x6060,0,1,param,10);
	
	return comm_state;
}

int8_t set_MotorRPM(ServoDriverHWType_t* driver,short Lm,short Rm)
{
	if( driver->onlineFlag==0 ) return 1;
	
	int8_t comm_state = 0;

	uint32_t TmpWrite = (uint32_t)Rm<<16|(uint16_t)Lm;
	
	comm_state = CANopenRSDO_Write_RTOS(driver,0x60FF,3,4,TmpWrite,5);
	
	return comm_state;
}


int8_t ServoDriver_Enable(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;
	
	//使能步骤：对控制字写入06、07、0F
	uint32_t param = 0x06;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	param = 0x07;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	param = 0x0F;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	return comm_state;
}

int8_t ServoDriver_Disable(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;
	
	uint32_t param = 0;
	comm_state = CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	return comm_state;
}

//释放电机
int8_t ServoDriver_ReleaseMotor(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;
	
	//电机位于使能状态,且驱动器无报错,才允许释放电机
	if( driver->motorEnflag==1 && driver->errFlag==0 )
	{
		comm_state += ServoDriver_Disable(driver);
		comm_state += ServoDriver_Enable(driver);
	}

	return comm_state;
}

//驻车模式设置
int8_t ServoDriver_ParkingMode(ServoDriverHWType_t* driver,uint8_t set)
{
	int8_t comm_state = 0;
	
	if(set==1)
	{
		//释放电机
		comm_state += ServoDriver_ReleaseMotor(driver);
	}

	//驻车模式设置
	comm_state += CANopenRSDO_Write_RTOS(driver,0x2026,4,2,set,5);
	
	return comm_state;
}


//清除驱动报错
int8_t ServoDriver_ClearError(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;

	//bit7由 0->1 复位故障
	uint32_t param = 0x00;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	param = 0x80;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	return comm_state;
}

//TPDO0映射配置
int8_t ServoDriver_MappingTPDO0(ServoDriverHWType_t* driver,uint32_t COBID)
{
	int8_t comm_state = 0;

	uint32_t param = 0;
	
	//清空TPDO0上的映射
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,0,1,param,10);
	
	//映射1,内容为 0x606C 地址,索引01,字节数0x20（32）
	param = 0x606C0120;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,1,4,param,10);
	
	//映射2,内容为 0x606C 地址,索引02,字节数0x20（32）
	param = 0x606C0220;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,2,4,param,10);
	
	//设置TPDO上报时的帧ID
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,1,4,COBID,10);
	
	//设置254事件,数据改变立即上报
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,2,1,254,10);
	
	//设置禁止时间,若两次数据间隔没超过此时间则不会触发
	param = 100;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,3,2,param,10);
	
	//开启2个TPDO0映射
	param = 2;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,0,1,param,10);
	
	return comm_state;
}

//保存参数
int8_t ServoDriver_SaveEEPROM(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;

	uint32_t param = 1;

	if( driver->onlineFlag==1 )
		comm_state += CANopenRSDO_Write_RTOS(driver,0x2010,0,2,param,10);
	
	return comm_state;
}

//驱动器自检获取报错信息
int8_t ServoDriver_CheckError(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;
	uint32_t readtmp;
	uint16_t LeftMotor;
	uint16_t RightMotor;
	
	//仅在驱动在线时,才执行自检
	if( driver->onlineFlag == 1 )
	{
		comm_state = CANopenRSDO_Read_RTOS(driver,0x6041,0,4,&readtmp,10);
		if( comm_state==0 )
		{
			RightMotor = readtmp>>16;
			LeftMotor = readtmp;
			
			//检查电机使能或释放状态
			if( (LeftMotor&0x07)==7 && (RightMotor&0x07)==7 )
			{
				driver->motorEnflag = 1;
			}
			else driver->motorEnflag = 0;

			//检查电机的报错码
			if( 1 == ((RightMotor>>7)&0x01) || 1==((LeftMotor>>7)&0x01) ) 
			{
				driver->errFlag = 1;
				CANopenRSDO_Read_RTOS(driver,0x603F,0,4,&readtmp,10);
				driver->LeftMotorErr = readtmp>>16;
				driver->RightMotorErr = readtmp;
			}
			else 
			{
				driver->errFlag = 0;
				driver->LeftMotorErr = 0;
				driver->RightMotorErr = 0;
			}
		}
	}
	
	//驱动离线,复位该驱动器的信息
	else
	{
		DriverInfoReset(driver);
	}
	
	return comm_state;

}

//根据反馈的NodeID激活对应的在线标志位,用于心跳检测
void ServoDriver_OnLineCheck(uint8_t nodeid)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( nodeid == g_ServoDriveList[i]->NodeID )
		{
			g_ServoDriveList[i]->online_tick++;
			if( g_ServoDriveList[i]->onlineFlag==0 )
			{
				g_ServoDriveList[i]->onlineFlag = 1;
				g_ServoDriveList[i]->comm_Errcnt=0;
				g_ServoDriveList[i]->comm_qua=100;
			}
		}		
	}
}

//伺服电机初始化
uint8_t ServoDriver_Init(ServoDriverHWType_t* driver)
{
	int8_t comm_state;
	uint32_t readdata = 0;
	
	//进入预操作状态,停止映射,开始配置驱动器(关闭已经被配置过的映射,防止影响初始化进程)
	pCANInterface_t candev = &UserCAN2Dev;
	uint8_t PreOp[8]= {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	candev->sendStd(0x00,PreOp,8);
	
	//读取驱动器软件版本号,并判断驱动是否在线
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2031,0,2,&readdata,10);
	if( comm_state==0 )
	{
		driver->SoftWareVer = readdata;
		driver->onlineFlag = 1;
	}
	else 
	{
		driver->onlineFlag=0;
		return 0;
	}
	
	//驱动器参数初始化
	#if 1
	Driver_UsrLog("======= driver param init %d ========\r\n",driver->NodeID);
	
	//二次确认预操作
	candev->sendStd(0x00,PreOp,8);
	
	//电机参数,根据车轮型号执行不同的初始化
	if( RobotHardWareParam.wheeltype == SingleAxis_8inch )
	{
		comm_state += set_encoder_accuracy(driver,4096); //设置编码器线数
		comm_state += set_HallOffset(driver,0);          //设置角度偏移
		comm_state += set_poles(driver,15);              //设置电机极对数
	}
	else if( RobotHardWareParam.wheeltype == DoubleAxis_8inch )
	{
		comm_state += set_encoder_accuracy(driver,1024);
		comm_state += set_HallOffset(driver,0);         
		comm_state += set_poles(driver,15);             
	}
	else if( RobotHardWareParam.wheeltype == SingleAxis_5inch )
	{
		comm_state += set_encoder_accuracy(driver,4096);
		comm_state += set_HallOffset(driver,240);       
		comm_state += set_poles(driver,10);             
	}
	else if( RobotHardWareParam.wheeltype == DoubleAxis_5inch )
	{
		comm_state += set_encoder_accuracy(driver,1024);
		comm_state += set_HallOffset(driver,240);       
		comm_state += set_poles(driver,10);             
	}
	
	comm_state += set_rated_current(driver,150);     //设置额定电流
	comm_state += set_max_current(driver,300);       //设置最大电流
	comm_state += set_max_rpm(driver,1000);          //设置额定转速
	comm_state += set_overload_factor(driver,300);   //过载系数
	comm_state += set_overload_time(driver,6553);    //过载时间
	comm_state += set_encoder_ErrorAlarm(driver,6553);//超差报警值
	comm_state += set_maxTemp(driver,1200);           //温度保护阈值
	
	//位置环参数
//	comm_state += set_pos_kp(driver,200);
//	comm_state += set_pos_kf(driver,200);
	
	//速度环参数
	comm_state += get_speed_kp(driver,420);
	comm_state += get_speed_ki(driver,200);
//	comm_state += set_speed_kf(driver,800);
	
	//输出平滑系数
//	comm_state += set_speed_smooth(driver,50);
//	comm_state += set_smooth_kf(driver,100);
//	comm_state += set_tor_smooth(driver,100);
	
	//电流环参数
	comm_state += get_curloop_kp(driver,300);
	comm_state += get_curloop_ki(driver,200);
	
	comm_state += set_SynCon_state(driver,1);//设置同步控制
	comm_state += set_acctime(driver,30);    //设置加速时间
	comm_state += set_dectime(driver,30);    //设置减速时间
	comm_state += set_stoptime(driver,30);   //设置急停减速时间
	comm_state += set_ControlMode(driver,3); //设置速度控制模式
	comm_state += ServoDriver_SaveEEPROM(driver);//保存上述参数到EEPROM
	
	comm_state += set_MotorRPM(driver,0,0);              //设置目标速度为0
	comm_state += ServoDriver_MappingTPDO0(driver,(0x190+driver->NodeID-1));//TPDO0映射转速
	comm_state += ServoDriver_Enable(driver);            //使能电机
	comm_state += set_hearbeatTime(driver,5000);         //设置心跳时间
	
	//配置完成,启动NMT,所有的TPDO和RPDO将开启映射
	uint8_t start[8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	candev->sendStd(0x00,start,8);
	
	Driver_UsrLog("comm_state:%d\r\n",comm_state);
	Driver_UsrLog("Qua:%.3f \r\n",driver->comm_qua);
	#else //通信测试,无意义数据
	comm_state += set_encoder_accuracy(driver,10); //设置编码器线数
	comm_state += set_HallOffset(driver,11);          //设置角度偏移
	comm_state += set_poles(driver,12);              //设置电机极对数
	comm_state += set_rated_current(driver,13);     //设置额定电流
	comm_state += set_max_current(driver,14);       //设置最大电流
	comm_state += set_max_rpm(driver,15);          //设置额定转速
	comm_state += set_overload_factor(driver,16);   //过载系数
	comm_state += set_overload_time(driver,17);    //过载时间
	comm_state += set_encoder_ErrorAlarm(driver,18);//超差报警值
	comm_state += set_maxTemp(driver,19);           //温度保护阈值
	
	//位置环
	comm_state += set_pos_kp(driver,20);
	comm_state += set_pos_kf(driver,21);
	
	//速度环
	comm_state += set_speed_kp(driver,22);
	comm_state += set_speed_ki(driver,23);
	comm_state += set_speed_kf(driver,24);
	
	//输出平滑系数
	comm_state += set_speed_smooth(driver,25);
	comm_state += set_smooth_kf(driver,26);
	comm_state += set_tor_smooth(driver,27);
	
	//电流环
	comm_state += set_curloop_kp(driver,28);
	comm_state += set_curloop_ki(driver,29);
	#endif
	Driver_UsrLog("======= driver param init End ========\r\n\r\n");
	return comm_state;
}


//伺服驱动心跳检测,用于判断驱动器是否在线
void HeartbeatTask(void* param)
{
	while(1)
	{
		//心跳报文设置online_tick自增
		Driver1.online_tick=0;
		Driver2.online_tick=0;
		Driver3.online_tick=0;
		vTaskDelay(5000);
		
		//若无心跳报文,说明驱动已经掉线
		if( Driver1.online_tick==0 ) Driver1.onlineFlag = 0;
		if( Driver2.online_tick==0 ) Driver2.onlineFlag = 0;
		if( Driver3.online_tick==0 ) Driver3.onlineFlag = 0;
	}
}



