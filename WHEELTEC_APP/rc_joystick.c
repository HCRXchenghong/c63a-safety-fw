#include "rc_joystick.h"

typedef struct{
	uint8_t sm;
	uint32_t upvalue;
	uint32_t downvalue;
	TIM_HandleTypeDef* tim;
	uint64_t ch;
	uint16_t val;
}RCJoystickChannelType_t;

enum {
	WaitUp=0,
	WaitDown,
};

RC_REMOTE_t rc_remote = {
	.LX = 1500,
	.LY = 1500,
	.RX = 1500,
	.RY = 1000
};

//硬件挂载
static RCJoystickChannelType_t JoyRX = {
	.sm = WaitUp,
	.upvalue = 0,
	.downvalue = 0,
	.tim = &htim4,
	.ch = TIM_CHANNEL_1
};

static RCJoystickChannelType_t JoyLY = {
	.sm = WaitUp,
	.upvalue = 0,
	.downvalue = 0,
	.tim = &htim4,
	.ch = TIM_CHANNEL_2
};

static RCJoystickChannelType_t JoyRY = {
	.sm = WaitUp,
	.upvalue = 0,
	.downvalue = 0,
	.tim = &htim4,
	.ch = TIM_CHANNEL_3
};

static RCJoystickChannelType_t JoyLX = {
	.sm = WaitUp,
	.upvalue = 0,
	.downvalue = 0,
	.tim = &htim4,
	.ch = TIM_CHANNEL_4
};

#include "bsp_RTOSdebug.h"
static pRTOSDebugInterface_t debug = &RTOSDebugTimer;
static RTOSDebugPrivateVar var = { 0 };

//处理通道捕获的内容
static void RCJoystickHandle_Result(RCJoystickChannelType_t* hal,BaseType_t* wokenTask)
{
	switch (hal->sm)
	{
		case WaitUp:
		{
			//读出计数值
			hal->upvalue = HAL_TIM_ReadCapturedValue(hal->tim,hal->ch);
			
			//设置下降沿捕获
			TIM_RESET_CAPTUREPOLARITY(hal->tim,hal->ch);
			TIM_SET_CAPTUREPOLARITY(hal->tim,hal->ch,TIM_ICPOLARITY_FALLING);
			
			//状态机切换
			hal->sm = WaitDown;
			break;
		}
		
		case WaitDown:
		{
			//读取计数值
			hal->downvalue = HAL_TIM_ReadCapturedValue(hal->tim,hal->ch);
			
			//计算距离
			if( hal->upvalue > hal->downvalue )//溢出计算
				hal->val = hal->downvalue+9999 - hal->upvalue;
			else
				hal->val = hal->downvalue - hal->upvalue;
			
			if( hal->ch == TIM_CHANNEL_1 ) rc_remote.RX = hal->val;
			else if( hal->ch == TIM_CHANNEL_2 ) rc_remote.LY = hal->val;
			else if( hal->ch == TIM_CHANNEL_3 ) rc_remote.RY = hal->val;
			else if( hal->ch == TIM_CHANNEL_4 ) rc_remote.LX = hal->val;
			
			//设置上升沿捕获
			TIM_RESET_CAPTUREPOLARITY(hal->tim,hal->ch);
			TIM_SET_CAPTUREPOLARITY(hal->tim,hal->ch,TIM_ICPOLARITY_RISING);
			
			//状态机切换
			hal->sm = WaitUp;
			
			break;
		}
	}		
}

static RCJoystickChannelType_t* RCJoylist[] = { 
	&JoyLX,&JoyLY,&JoyRX,&JoyRY
};

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

#include "RobotControl_task.h"

//航模手柄
void RCJoystick_IC_CaptureCallback(TIM_HandleTypeDef *htim,BaseType_t* woken)
{
	//根据硬件接口匹配对应的超声波
	for(uint8_t i=0;i<4;i++)
	{
		//通过匹配对应的定时器+定时器通道,完成不同的超声波数据采集
		if( RCJoylist[i]->tim == htim && RCJoylist[i]->ch == get_tim_activate_ch(htim->Channel) )
		{
			RCJoystickHandle_Result(RCJoylist[i],woken);
		}
	}
	
	//降低更新频率,仅在某个摇杆通道采集完毕时才更新
	if( htim != JoyRX.tim || get_tim_activate_ch(htim->Channel)!=JoyRX.ch || 
		JoyRX.sm!=WaitDown ) return;
		
	rc_remote.freq = debug->UpdateFreq(&var);
	
	RobotControlCMDType_t cmd = { 
		.cmdsource = RCJOY_CMD,
		0,0,0
	};

    //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity;
	float base_vz = PI/4.0f ;//Z轴速度基准

	//打杆阈值
	const uint8_t Yuzhi=100;
	
	//4个通道原始值限幅
	rc_remote.LX = target_limit_int(rc_remote.LX,1000,2000);
	rc_remote.LY = target_limit_int(rc_remote.LY,1000,2000);
	rc_remote.RX = target_limit_int(rc_remote.RX,1000,2000);
	rc_remote.RY = target_limit_int(rc_remote.RY,1000,2000);

    //Front and back direction of left rocker. Control forward and backward.
    //左摇杆前后方向。控制前进后退。
    LX=rc_remote.LY-1500;
    
	//The left joystick's horizontal directions control lateral movement
	//for omnidirectional mobile vehicles.
    //左摇杆左右方向,对于全向移动小车可控制左右横移
    LY=rc_remote.LX-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
    //右摇杆前后方向。油门/加减速。
    RX=rc_remote.RY-1500;
	
    //Right stick left and right. To control the rotation.
    //右摇杆左右方向。控制自转。
    RY=-(rc_remote.RX-1500);//自转

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
    if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;

	//油门调整速度 Remote_RCvelocity取值:0~1000
    Remote_RCvelocity= RobotControlParam.defalutSpeed + RX;
    if(Remote_RCvelocity<0)Remote_RCvelocity=0;

    //The remote control command of model aircraft is processed
    //对航模遥控控制命令进行处理
	cmd.Vx = LX * (float)Remote_RCvelocity/500.0f;
	//cmd.Vy =-LY * (float)Remote_RCvelocity/500.0f;
	cmd.Vy = 0;//暂无使用Vy的车型
	//基准:base_vz,自身倍数:RY/500,取值[-1~1],油门倍数:(Remote_RCvelocity/500.0f),取值[0,2]
	cmd.Vz = base_vz*((float)RY/500.0f) * ((float)Remote_RCvelocity/500.0f) ; 

    //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
	cmd.Vx/=1000.0f;
	cmd.Vy/=1000.0f;
	
	//后退时转换Z轴速度,符合直觉的控制逻辑
	if(cmd.Vx<0)cmd.Vz=-cmd.Vz;
	
	//航模遥控解锁条件：持续向前推【前进】摇杆保持2秒
	static uint16_t filterCounts = 0;
	static uint32_t LastTick = 0;
	static uint8_t RCUnlock = 0;
	if( RCUnlock==0 )
	{
		if( cmd.Vx>0 && ((xTaskGetTickCountFromISR()-LastTick)<50) )
		{
			filterCounts++;
		}
		else filterCounts=0;
		
		if( filterCounts > rc_remote.freq*2 )
		{
			filterCounts = rc_remote.freq*2+10;
			RCUnlock = 1;
		}
		
		//记录当前时刻
		LastTick = xTaskGetTickCountFromISR();
	}
	
	//判断是否要执行队列写入,空闲时不写入
	uint8_t writeflag=1;
	static uint8_t idleCount = 0;
	if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
	else idleCount=0;
	if( idleCount>10 ) writeflag=0,idleCount=10;
	
	//满足控制条件,写入数据
	if( writeflag && RCUnlock )
	{
		if( WriteRobotControlQueue(&cmd,woken) )
		{
			filterCounts=0;RCUnlock=0; //若写入失败,则需要重新解锁航模
		}
	}
    
}




