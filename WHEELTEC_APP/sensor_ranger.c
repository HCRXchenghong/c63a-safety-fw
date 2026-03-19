/**
 * @file    sensor_ranger.c
 * @brief   超声波传感器数据获取逻辑文件.
 * @author  WHEELTEC
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - 本文件主要是超声波传感器数据获取的内容,需要依赖定时器的输入捕获功能。超声波数据获取流程：
     由任务ultrasonic_task触发超声波工作,并最终在定时器的捕获中断里计算距离。
 * - 此文件用户主要关注GetRangerGroup函数,此函数可以给超声波分组,同一组的超声波将会同时被
 *   触发测距,需要注意相邻太近的超声波不可分配到同一组,否则将会出现两个超声波信号互相干扰
 * - 需要修改超声波触发顺序或者分组,请按GetRangerGroup里面的格式定义分组即可。如需要屏蔽某个
 *   个超声波，直接将其移除分组即可
 * @note
 * 
 * 
 */

#include "sensor_ranger.h"

#include "robot_select_init.h"

//超声波传感器事件组
EventGroupHandle_t g_xEventRangerSensor = NULL;

//设置调试等级
#define SENSOR_DEBUG_LEVEL 0

#if (SENSOR_DEBUG_LEVEL > 0U)
#define  SENSOR_UsrLog(...)   do { \
                            printf(__VA_ARGS__); \
                            printf("\r\n"); \
} while (0)
#else
#define SENSOR_UsrLog(...) do {} while (0)
#endif

//超声波距离过滤配置
#define RANGER_INVALID_DISTANCE  5.1f
#define RANGER_FILTER_THRESHOLD  1.5f

static float RangerApplyDistanceFilter(float distance)
{
	if( distance > RANGER_FILTER_THRESHOLD )
		return RANGER_INVALID_DISTANCE;

	return distance;
}


//超声波完成采集回调函数_运行环境为中断环境
BaseType_t RangerSensor_CaptureCallBack(uint8_t index)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	//设置采集成功的标志位
	xEventGroupSetBitsFromISR(g_xEventRangerSensor,1<<index,&xHigherPriorityTaskWoken);
	
	return xHigherPriorityTaskWoken;
}


enum {
	WaitUp=0,
	WaitDown,
};

//接口绑定
RangerHALObj RangerHAL_A = {
	.index = 1,      //索引值,A对1,B-2,C-3....
	.sm = WaitUp, //内置状态机
	.dis = RANGER_INVALID_DISTANCE,  //距离值
	.upvalue = 0,    //内部采集变量
	.downvalue = 0,  //内部采集变量
	.CaptureCompleted = RangerSensor_CaptureCallBack,//采集完成回调函数
	.timeoutflag = 1,//采集超时标志位
	.timeoutCount = 0,//采集超时次数
	
	//绑定硬件接口
	.tim = &htim3,
	.ch = TIM_CHANNEL_4,
	.TrigPort = TriA_GPIO_Port,
	.TrigPin = TriA_Pin,
};

RangerHALObj RangerHAL_B = {
	.index = 2,
	.sm = WaitUp,
	.dis = RANGER_INVALID_DISTANCE,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim3,
	.ch = TIM_CHANNEL_3,
	.TrigPort = TriB_GPIO_Port,
	.TrigPin = TriB_Pin,
};

RangerHALObj RangerHAL_C = {
	.index = 3,
	.sm = WaitUp,
	.dis = RANGER_INVALID_DISTANCE,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim3,
	.ch = TIM_CHANNEL_1,
	.TrigPort = TriC_GPIO_Port,
	.TrigPin = TriC_Pin,
};

RangerHALObj RangerHAL_D = {
	.index = 4,
	.sm = WaitUp,
	.dis = RANGER_INVALID_DISTANCE,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim2,
	.ch = TIM_CHANNEL_1,
	.TrigPort = TriD_GPIO_Port,
	.TrigPin = TriD_Pin,
};

RangerHALObj RangerHAL_E = {
	.index = 5,
	.sm = WaitUp,
	.dis = RANGER_INVALID_DISTANCE,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim2,
	.ch = TIM_CHANNEL_2,
	.TrigPort = TriE_GPIO_Port,
	.TrigPin = TriE_Pin,
};

RangerHALObj RangerHAL_F = {
	.index = 6,
	.sm = WaitUp,
	.dis = RANGER_INVALID_DISTANCE,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim3,
	.ch = TIM_CHANNEL_2,
	.TrigPort = TriF_GPIO_Port,
	.TrigPin = TriF_Pin,
};

//支持的超声波个数列表,此内容用于中断使用
#define RANGER_NUM 6
static RangerHALObj* rangerlist[] = { 
	&RangerHAL_A,&RangerHAL_B,&RangerHAL_C,
	&RangerHAL_D,&RangerHAL_E,&RangerHAL_F 
};

//计数器的计数频率,与定时器配置相关
#define TIM_COUNT_FREQ 1000000.0f

//传感器复位,采集数据超时时使用
static void RangerReset(RangerHALObj* ranger)
{
	ranger->sm = WaitUp;
	ranger->dis = RANGER_INVALID_DISTANCE;
	ranger->upvalue = 0;
	ranger->downvalue = 0;
	ranger->timeoutflag = 1;
	TIM_RESET_CAPTUREPOLARITY(ranger->tim,ranger->ch);
	TIM_SET_CAPTUREPOLARITY(ranger->tim,ranger->ch,TIM_ICPOLARITY_RISING);
	HAL_GPIO_WritePin(ranger->TrigPort,ranger->TrigPin,GPIO_PIN_SET);
}

//根据活跃的定时器通道返回对应的通道值,中断使用
static uint32_t get_tim_activate_ch(HAL_TIM_ActiveChannel active_ch)
{
	switch (active_ch) {
		case HAL_TIM_ACTIVE_CHANNEL_1: return TIM_CHANNEL_1;
		case HAL_TIM_ACTIVE_CHANNEL_2: return TIM_CHANNEL_2;
		case HAL_TIM_ACTIVE_CHANNEL_3: return TIM_CHANNEL_3;
		case HAL_TIM_ACTIVE_CHANNEL_4: return TIM_CHANNEL_4;
		default: return 0xFF;
	}
}

//超声波触发,用于驱动超声波传感器工作
static void RangerTrigger(RangerHALObj* ranger)
{
	//低电平触发超声波工作
	HAL_GPIO_WritePin(ranger->TrigPort,ranger->TrigPin,GPIO_PIN_RESET);
	delay_us(15);
	HAL_GPIO_WritePin(ranger->TrigPort,ranger->TrigPin,GPIO_PIN_SET);
	
	//设置超时标志位,若改标志位没有被置0代表数据超时
	ranger->timeoutflag = 1;
	
	//清空历史事件组标志位
	xEventGroupClearBits(g_xEventRangerSensor,1<<ranger->index);
	
	//检查元素访问是否正确
//	SENSOR_UsrLog("sensor %c trigger!",0x40+ranger->index);
}

/*
 * 中断内部状态机,中断使用.
 * 超声波工作方式：trig 下降沿触发工作, 测距信号由低变高,然后由高变低,统计高电平时间测出距离
*/
static void RangerHandle_Result(RangerHALObj* ranger,BaseType_t* wokenTask)
{
	switch (ranger->sm)
	{
		case WaitUp:
		{
			//读出计数值
			ranger->upvalue = HAL_TIM_ReadCapturedValue(ranger->tim,ranger->ch);
			
			//设置下降沿捕获
			TIM_RESET_CAPTUREPOLARITY(ranger->tim,ranger->ch);
			TIM_SET_CAPTUREPOLARITY(ranger->tim,ranger->ch,TIM_ICPOLARITY_FALLING);
			
			//状态机切换
			ranger->sm = WaitDown;
			break;
		}
		
		case WaitDown:
		{
			float distance = 0.0f;

			//读取计数值
			ranger->downvalue = HAL_TIM_ReadCapturedValue(ranger->tim,ranger->ch);
			
			//计算距离
			if( ranger->upvalue > ranger->downvalue )//溢出计算
				distance = ranger->downvalue+0xFFFF - ranger->upvalue;
			else
				distance = ranger->downvalue - ranger->upvalue;
			
			// S = V*t/2
			distance = (distance/TIM_COUNT_FREQ)/2.0f * 340.0f;
			ranger->dis = RangerApplyDistanceFilter(distance);
			ranger->sampleSeq++;
			
			//设置上升沿捕获
			TIM_RESET_CAPTUREPOLARITY(ranger->tim,ranger->ch);
			TIM_SET_CAPTUREPOLARITY(ranger->tim,ranger->ch,TIM_ICPOLARITY_RISING);
			
			//触发完成采集回调函数
			*wokenTask = ranger->CaptureCompleted(ranger->index);
			
			//成功采集数据,清除超时标志位
			ranger->timeoutflag = 0;
			
			//状态机切换
			ranger->sm = WaitUp;
			
			break;
		}
	}		
}

//超声波捕获回调函数处理
void Ranger_IC_CaptureCallback(TIM_HandleTypeDef *htim,BaseType_t* woken)
{
	//根据硬件接口匹配对应的超声波
	for(uint8_t i=0;i<RANGER_NUM;i++)
	{
		//通过匹配对应的定时器+定时器通道,完成不同的超声波数据采集
		if( rangerlist[i]->tim == htim && rangerlist[i]->ch == get_tim_activate_ch(htim->Channel) )
		{
			RangerHandle_Result(rangerlist[i],woken);
		}
	}
}

// 组信息结构体
typedef struct {
    RangerHALObj** list; // 指向组的指针
    uint8_t size;        // 组的元素个数
} RangerGroup_t;

//////////// 用户配置区,可修改超声波的分组情况,以及定义超声波的组分配情况,配置后无需修改其他代码 ////////////
////定义使用哪些超声波作为组1
//RangerHALObj* RangerGroup1_List[] = { &RangerHAL_E,&RangerHAL_F }; 

////定义使用哪些超声波作为组2
//RangerHALObj* RangerGroup2_List[] = { &RangerHAL_A,&RangerHAL_B }; 

////定义使用哪些超声波作为组X

////将上述定义的组别，一一添加到下列超声波集合
//static RangerGroup_t GroupManager_List[] = {
//	{ RangerGroup1_List,sizeof(RangerGroup1_List)/sizeof(RangerGroup1_List[0]) } , 
//	{ RangerGroup2_List,sizeof(RangerGroup2_List)/sizeof(RangerGroup2_List[0]) } , 
//};
//////////// 用户配置区,可修改超声波的分组情况,以及定义超声波的组分配情况,配置后无需修改其他代码 ////////////


//根据不同的车型对超声波进行分组
void GetRangerGroup(RangerGroup_t** group,uint8_t* groupNum)
{
	//S260六轮车、S200户外自动驾驶机器人,均适用本超声波分组
//	static RangerHALObj* RangerGroup1_List_S260[] = { &RangerHAL_A,&RangerHAL_D,&RangerHAL_F };
//	static RangerHALObj* RangerGroup2_List_S260[] = { &RangerHAL_B,&RangerHAL_E}; 
//	static RangerHALObj* RangerGroup3_List_S260[] = { &RangerHAL_C}; 
//	static RangerGroup_t GroupManager_List_S260[] = {
//		{ RangerGroup1_List_S260,sizeof(RangerGroup1_List_S260)/sizeof(RangerGroup1_List_S260[0]) } , 
//		{ RangerGroup2_List_S260,sizeof(RangerGroup2_List_S260)/sizeof(RangerGroup2_List_S260[0]) } , 
//		{ RangerGroup3_List_S260,sizeof(RangerGroup3_List_S260)/sizeof(RangerGroup3_List_S260[0]) } , 
//	};
	
	//分组版本2
	static RangerHALObj* RangerGroup1_List_S260[] = { &RangerHAL_A,&RangerHAL_E };
	static RangerHALObj* RangerGroup2_List_S260[] = { &RangerHAL_C,&RangerHAL_F}; 
	static RangerHALObj* RangerGroup3_List_S260[] = { &RangerHAL_B}; 
	static RangerHALObj* RangerGroup4_List_S260[] = { &RangerHAL_D}; 
	static RangerGroup_t GroupManager_List_S260[] = {
		{ RangerGroup1_List_S260,sizeof(RangerGroup1_List_S260)/sizeof(RangerGroup1_List_S260[0]) } , 
		{ RangerGroup2_List_S260,sizeof(RangerGroup2_List_S260)/sizeof(RangerGroup2_List_S260[0]) } , 
		{ RangerGroup3_List_S260,sizeof(RangerGroup3_List_S260)/sizeof(RangerGroup3_List_S260[0]) } ,
		{ RangerGroup4_List_S260,sizeof(RangerGroup4_List_S260)/sizeof(RangerGroup4_List_S260[0]) } ,

	};
	
	//S300超声波分组
	static RangerHALObj* RangerGroup1_List_S300[] = { &RangerHAL_A,&RangerHAL_C,&RangerHAL_E };
	static RangerHALObj* RangerGroup2_List_S300[] = { &RangerHAL_B,&RangerHAL_D,&RangerHAL_F}; 
	static RangerGroup_t GroupManager_List_S300[] = {
		{ RangerGroup1_List_S300,sizeof(RangerGroup1_List_S300)/sizeof(RangerGroup1_List_S300[0]) } , 
		{ RangerGroup2_List_S300,sizeof(RangerGroup2_List_S300)/sizeof(RangerGroup2_List_S300[0]) } , 
	};
	
	//S300 Mini超声波分组
	static RangerHALObj* RangerGroup1_List_S150[] = { &RangerHAL_A,&RangerHAL_C,&RangerHAL_E };
	static RangerHALObj* RangerGroup2_List_S150[] = { &RangerHAL_B,&RangerHAL_D}; 
	static RangerGroup_t GroupManager_List_S150[] = {
		{ RangerGroup1_List_S150,sizeof(RangerGroup1_List_S150)/sizeof(RangerGroup1_List_S150[0]) } , 
		{ RangerGroup2_List_S150,sizeof(RangerGroup2_List_S150)/sizeof(RangerGroup2_List_S150[0]) } , 
	};

	//S100超声波分组
	static RangerHALObj* RangerGroup1_List_S100[] = { &RangerHAL_A,&RangerHAL_C,&RangerHAL_E };
	static RangerHALObj* RangerGroup2_List_S100[] = { &RangerHAL_B,&RangerHAL_D,&RangerHAL_F};
	static RangerGroup_t GroupManager_List_S100[] = {
		{ RangerGroup1_List_S100,sizeof(RangerGroup1_List_S100)/sizeof(RangerGroup1_List_S100[0]) } ,
		{ RangerGroup2_List_S100,sizeof(RangerGroup2_List_S100)/sizeof(RangerGroup2_List_S100[0]) } ,
	};
	
	//根据车型不同选择不同的分组
	if( RobotHardWareParam.CarType == S200_OUTDOOR || RobotHardWareParam.CarType == S260 )
	{
		*group = GroupManager_List_S260;
		*groupNum = sizeof(GroupManager_List_S260)/sizeof(GroupManager_List_S260[0]);
	}
	else if( RobotHardWareParam.CarType == S300 )
	{
		*group = GroupManager_List_S300;
		*groupNum = sizeof(GroupManager_List_S300)/sizeof(GroupManager_List_S300[0]);
	}
	else if( RobotHardWareParam.CarType == S300Mini )
	{
		*group = GroupManager_List_S150;
		*groupNum = sizeof(GroupManager_List_S150)/sizeof(GroupManager_List_S150[0]);
	}
	else if( RobotHardWareParam.CarType == S100 )
	{
		*group = GroupManager_List_S100;
		*groupNum = sizeof(GroupManager_List_S100)/sizeof(GroupManager_List_S100[0]);
	}
	
	//对于无超声波的车型不会执行分组,此时会删除超声波任务以及关闭对应的中断节省系统资源

}

//超声波测距任务
void ultrasonic_task(void* param)
{
	static uint8_t MAX_Group=0;
	static RangerGroup_t* GroupManager_List = NULL;
	
	//获取超声波的分组和组别数量
	GetRangerGroup(&GroupManager_List,&MAX_Group);
	
    // 检查分组是否有效,无效的分组将不执行此任务
    if (GroupManager_List == NULL || MAX_Group == 0)
	{
		//没有分到组别的车型,删除超声波任务、关闭超声波中断,节省系统资源
		HAL_TIM_IC_Stop_IT(RangerHAL_A.tim,RangerHAL_A.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_B.tim,RangerHAL_B.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_C.tim,RangerHAL_C.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_D.tim,RangerHAL_D.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_E.tim,RangerHAL_E.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_F.tim,RangerHAL_F.ch);
		vTaskDelete(NULL);
    }
	
	//事件组实例
	g_xEventRangerSensor = xEventGroupCreate();
	EventBits_t waitbits = 0 , resultbits = 0;
	
	//开启超声波相关定时器捕获功能
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	
	uint8_t i=0,k=0;
	while(1)
	{
		for(i=0;i<MAX_Group;i++)
		{
			//同一个组内的超声波同时触发
			for(k=0;k<GroupManager_List[i].size;k++)
			{
				waitbits |= (1<<GroupManager_List[i].list[k]->index);//计算需要等待的事件
				RangerTrigger(GroupManager_List[i].list[k]);//触发超声波工作
			}

			//等待超声波处理结果,超时时间为30ms
			resultbits = xEventGroupWaitBits(g_xEventRangerSensor,waitbits,pdTRUE,pdTRUE,pdMS_TO_TICKS(50));
			
			//检查是否有超声波超时,并执行超时处理
			if( (resultbits&waitbits) != waitbits )
			{
				for(k=0;k<GroupManager_List[i].size;k++)
				{
					//处理超时的超声波
					if( GroupManager_List[i].list[k]->timeoutflag == 1 ) 
					{
						GroupManager_List[i].list[k]->timeoutCount++;//统计超时次数
						RangerReset(GroupManager_List[i].list[k]);
						GroupManager_List[i].list[k]->sampleSeq++;
						SENSOR_UsrLog("sensor %c timeout...",0x40+GroupManager_List[i].list[k]->index);
					}
				}
			}
			
			waitbits = 0;
			//下一组触发...
		}
	}
}



