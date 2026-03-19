#include "usart.h"

//C Include File
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_icm20948.h"
#include "bsp_can.h"

//APP Include File
#include "robot_select_init.h"
#include "sensor_ranger.h"
#include "AutoRecharge_task.h"

//校验函数
extern uint8_t Calculate_BCC(const uint8_t* checkdata,uint16_t datalen);
//指定与ROS通信的串口
static UART_HandleTypeDef *serial = &huart4;

//定义各个数据包的帧头帧尾以及数据包长度
#define BaseFRAME_HEAD 0x7B
#define BaseFRAME_TAIL 0x7D
#define BaseFRAME_LEN   24

#define RangerFRAME_HEAD 0xFA
#define RangerFRAME_TAIL 0xFC
#define RangerFRAME_LEN 19

#define ChargeFRAME_HEAD 0x7C
#define ChargeFRAME_TAIL 0x7F
#define ChargeFRAME_LEN  8
#define RobotTX_MAX_LEN  (BaseFRAME_LEN+RangerFRAME_LEN+ChargeFRAME_LEN)

static uint8_t g_rosTxBuffer[RobotTX_MAX_LEN] = { 0 };
static uint8_t g_uart1TxBuffer[RobotTX_MAX_LEN] = { 0 };

static void RobotDataTransmitDMA(UART_HandleTypeDef *uart,uint8_t *dmaBuffer,const uint8_t *frame,uint16_t length)
{
	if( uart == NULL || dmaBuffer == NULL || frame == NULL || length == 0U )
		return;

	if( uart->gState != HAL_UART_STATE_READY )
		return;

	memcpy(dmaBuffer,frame,length);
	if( HAL_UART_Transmit_DMA(uart,dmaBuffer,length) != HAL_OK )
	{
		// 串口忙时直接丢弃本帧，避免覆盖仍在发送中的 DMA 缓冲区
	}
}

void RobotDataTransmitTask(void* param)
{
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//本任务的控制频率,单位为Hz
	const uint16_t TaskFreq = 20;

	//定义需要发送的数据
	uint8_t basebuffer[BaseFRAME_LEN];
	uint8_t rangerbuffer[RangerFRAME_LEN];
	uint8_t autorechargerbuffer[ChargeFRAME_LEN];
	//自动回充CAN帧数据包
	uint8_t chargerDevCANFrame[8] = { 0 };
	
	pCANInterface_t candev = &UserCAN1Dev;
	while(1)
	{
		//小车基础24字节数据包
		basebuffer[0] = BaseFRAME_HEAD;
		basebuffer[BaseFRAME_LEN-1] = BaseFRAME_TAIL;
		basebuffer[1] = RobotControlParam.en_flag;
		basebuffer[2] = (short)(RobotControlParam.feedbackVx*1000)>>8;
		basebuffer[3] = (short)(RobotControlParam.feedbackVx*1000);
		basebuffer[4] = (short)(RobotControlParam.feedbackVy*1000)>>8;
		basebuffer[5] = (short)(RobotControlParam.feedbackVy*1000);
		basebuffer[6] = (short)(RobotControlParam.feedbackVz*1000)>>8;
		basebuffer[7] = (short)(RobotControlParam.feedbackVz*1000);
		basebuffer[8] = (short)(axis_9ValOri.accel.y)>>8;
		basebuffer[9] = (short)(axis_9ValOri.accel.y);
		basebuffer[10] = (short)(-axis_9ValOri.accel.x)>>8;
		basebuffer[11] = (short)(-axis_9ValOri.accel.x);
		basebuffer[12] = (short)(axis_9ValOri.accel.z)>>8;
		basebuffer[13] = (short)(axis_9ValOri.accel.z);
		basebuffer[14] = (short)(axis_9ValOri.gyro.y)>>8;
		basebuffer[15] = (short)(axis_9ValOri.gyro.y);
		basebuffer[16] = (short)(-axis_9ValOri.gyro.x)>>8;
		basebuffer[17] = (short)(-axis_9ValOri.gyro.x);
		if( RobotControlParam.en_flag==0 )
		{
			basebuffer[18]=0;
			basebuffer[19]=0;
		}
		else
		{
			basebuffer[18] = (short)(axis_9ValOri.gyro.z)>>8;
			basebuffer[19] = (short)(axis_9ValOri.gyro.z);
		}
		basebuffer[20] = (short)(RobotControlParam.Vol*1000)>>8;
		basebuffer[21] = (short)(RobotControlParam.Vol*1000);
		basebuffer[22] = Calculate_BCC(basebuffer,22);
		
		//超声波数据包
		rangerbuffer[0] = RangerFRAME_HEAD;
		rangerbuffer[RangerFRAME_LEN-1] = RangerFRAME_TAIL;
		rangerbuffer[1] = (short)(RangerHAL_A.dis*1000)>>8;
		rangerbuffer[2] = (short)(RangerHAL_A.dis*1000);
		rangerbuffer[3] = (short)(RangerHAL_B.dis*1000)>>8;
		rangerbuffer[4] = (short)(RangerHAL_B.dis*1000);
		rangerbuffer[5] = (short)(RangerHAL_C.dis*1000)>>8;
		rangerbuffer[6] = (short)(RangerHAL_C.dis*1000);
		rangerbuffer[7] = (short)(RangerHAL_D.dis*1000)>>8;
		rangerbuffer[8] = (short)(RangerHAL_D.dis*1000);
		rangerbuffer[9] = (short)(RangerHAL_E.dis*1000)>>8;
		rangerbuffer[10] = (short)(RangerHAL_E.dis*1000);
		rangerbuffer[11] = (short)(RangerHAL_F.dis*1000)>>8;
		rangerbuffer[12] = (short)(RangerHAL_F.dis*1000);
		rangerbuffer[13] = 0;
		rangerbuffer[14] = 0;
		rangerbuffer[15] = 0;
		rangerbuffer[16] = 0;
		rangerbuffer[17] = Calculate_BCC(rangerbuffer,17);
		
		//自动回充数据包
		autorechargerbuffer[0] = ChargeFRAME_HEAD;
		autorechargerbuffer[ChargeFRAME_LEN-1] = ChargeFRAME_TAIL;
		autorechargerbuffer[1] = (short)ChargeDev.ChargingCur>>8;
		autorechargerbuffer[2] = (short)ChargeDev.ChargingCur;
		autorechargerbuffer[3] = ChargeDev.RedNum;
		autorechargerbuffer[4] = ChargeDev.ChargingFlag;
		autorechargerbuffer[5] = RobotControlParam.ChargeMode;
		autorechargerbuffer[6] = Calculate_BCC(autorechargerbuffer,6);
		
		//根据车型的不同发送对应的数据
		if( RobotHardWareParam.CarType == S300 || RobotHardWareParam.CarType == S300Mini ||
 			RobotHardWareParam.CarType == S200_OUTDOOR || RobotHardWareParam.CarType == S260 ||
			RobotHardWareParam.CarType == S100 )
		{
			uint8_t tmp[BaseFRAME_LEN+RangerFRAME_LEN+ChargeFRAME_LEN]={0};
			uint8_t *ptr = tmp;
			
			//基础24字节数据包
			memcpy(ptr,basebuffer,BaseFRAME_LEN);
			ptr+=BaseFRAME_LEN;
			
			//超声波数据包
			memcpy(ptr,rangerbuffer,RangerFRAME_LEN);
			ptr+=RangerFRAME_LEN;
			
			//自动回充数据包
			memcpy(ptr,autorechargerbuffer,ChargeFRAME_LEN);
			
			//启动DMA发送
			RobotDataTransmitDMA(serial,g_rosTxBuffer,tmp,sizeof(tmp));
			
			//串口1发送
			if( RobotControlParam.DebugLevel == 0 )
				RobotDataTransmitDMA(&huart1,g_uart1TxBuffer,tmp,sizeof(tmp));
			
			
			//模拟无超声波情况
//			uint8_t tmp[BaseFRAME_LEN+ChargeFRAME_LEN]={0};
//			uint8_t *ptr = tmp;
//			
//			//基础24字节数据包
//			memcpy(ptr,basebuffer,BaseFRAME_LEN);
//			ptr+=BaseFRAME_LEN;
//			
//			//自动回充数据包
//			memcpy(ptr,autorechargerbuffer,ChargeFRAME_LEN);
//			
//			//启动DMA发送
//			HAL_UART_Transmit_DMA(serial,tmp,sizeof(tmp)/sizeof(tmp[0]));
//			
//			//串口1发送
//			if( RobotControlParam.DebugLevel == 0 )
//				HAL_UART_Transmit_DMA(&huart1,tmp,sizeof(tmp)/sizeof(tmp[0]));

			//模拟只有24字节数据包
//			uint8_t tmp[BaseFRAME_LEN]={0};
//			uint8_t *ptr = tmp;
//			
//			//基础24字节数据包
//			memcpy(ptr,basebuffer,BaseFRAME_LEN);
//			
//			//启动DMA发送
//			HAL_UART_Transmit_DMA(serial,tmp,sizeof(tmp)/sizeof(tmp[0]));
//			
//			//串口1发送
//			if( RobotControlParam.DebugLevel == 0 )
//				HAL_UART_Transmit_DMA(&huart1,tmp,sizeof(tmp)/sizeof(tmp[0]));
		}
		else if( RobotHardWareParam.CarType == S200 )
		{
			uint8_t tmp[BaseFRAME_LEN+ChargeFRAME_LEN]={0};
			uint8_t *ptr = tmp;
			
			//基础24字节数据包
			memcpy(ptr,basebuffer,BaseFRAME_LEN);
			ptr+=BaseFRAME_LEN;
			
			//自动回充数据包
			memcpy(ptr,autorechargerbuffer,ChargeFRAME_LEN);

			//ROS串口发送
			RobotDataTransmitDMA(serial,g_rosTxBuffer,tmp,sizeof(tmp));
			
			//串口1发送
			if( RobotControlParam.DebugLevel == 0 )
				RobotDataTransmitDMA(&huart1,g_uart1TxBuffer,tmp,sizeof(tmp));
		}
		
		//CAN数据发送.通过帧id 0x101-0x103 发送基础24字节数据
		for(uint8_t i=0;i<3;i++)
		{
			candev->sendStd(0x100+(i+1),basebuffer+i*8,8);
		}
		
		//自动回充数据包发送.通过帧id 0x105 反馈标志位以及设置对接速度
		chargerDevCANFrame[0] = RobotControlParam.ChargeMode;
		chargerDevCANFrame[1] = (short)(ChargeDev.dockerX*1000)>>8;
		chargerDevCANFrame[2] = (short)(ChargeDev.dockerX*1000);
		chargerDevCANFrame[3] = (short)(ChargeDev.dockerY*1000)>>8;
		chargerDevCANFrame[4] = (short)(ChargeDev.dockerY*1000);
		chargerDevCANFrame[5] = (short)(ChargeDev.dockerZ*1000)>>8;
		chargerDevCANFrame[6] = (short)(ChargeDev.dockerZ*1000);
		chargerDevCANFrame[7] = 0;
		candev->sendStd(0x105,chargerDevCANFrame,8);
		
		//自动回充控制特征发送(TODO:移除此部分内容并优化)
		//用途:用于充电装备识别小车运动特征,在对接充电过程中进行一定的减速
		uint8_t chargerControlSend[8]={0};
		switch(RobotHardWareParam.CarType)
		{
			case S300:case S300Mini:case S100:
				if( RobotControlParam.MotorA.feedback>0 ) chargerControlSend[0]=1;
				else if( RobotControlParam.MotorA.feedback<0 ) chargerControlSend[0]=2;
				else chargerControlSend[0]=0;
			
				if( RobotControlParam.MotorB.feedback>0 ) chargerControlSend[1]=1;
				else if( RobotControlParam.MotorB.feedback<0 ) chargerControlSend[1]=2;
				else chargerControlSend[1]=0;
				break;
			case S200:
				if( RobotControlParam.MotorA.feedback>0 ) chargerControlSend[0]=1;
				else if( RobotControlParam.MotorA.feedback<0 ) chargerControlSend[0]=2;
				else chargerControlSend[0]=0;
			
				if( RobotControlParam.MotorD.feedback>0 ) chargerControlSend[1]=1;
				else if( RobotControlParam.MotorD.feedback<0 ) chargerControlSend[1]=2;
				else chargerControlSend[1]=0;
				break;
			//带后置超声波的车型,无需充电装备识别轮子运动情况.自主减速
			case S260:case S200_OUTDOOR:
				chargerControlSend[0]=0;
				chargerControlSend[1]=0;
				break;
		}
		candev->sendStd(0x185,chargerControlSend,8);
		
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
	
}

