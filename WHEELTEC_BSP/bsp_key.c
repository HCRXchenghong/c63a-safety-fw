#include "bsp_key.h"
#include "gpio.h"

static uint8_t keyValue(void)
{
	if( get_HardWareVersion() == HW_1_1 )
		return HAL_GPIO_ReadPin(UserKey_GPIO_Port,UserKey_Pin);
	else
		return HAL_GPIO_ReadPin(UserKey_V1_0_Port,UserKey_V1_0_Pin);
}

static UserKeyState_t key_scan(uint16_t freq)
{
    static uint16_t time_core;//走时核心
    static uint16_t long_press_time;//长按识别
    static uint8_t press_flag=0;//按键按下标记
    static uint8_t check_once=0;//是否已经识别1次标记
	
    float Count_time = (((float)(1.0f/(float)freq))*1000.0f);//算出计1需要多少个毫秒

    if(check_once)//完成了识别，则清空所有变量
    {
        press_flag=0;//完成了1次识别，标记清零
        time_core=0;//完成了1次识别，时间清零
        long_press_time=0;//完成了1次识别，时间清零
    }
    if(check_once&&1 == keyValue()) check_once=0; //完成扫描后按键被弹起，则开启下一次扫描

    if(0==keyValue()&&check_once==0)//按键扫描
    {
        press_flag=1;//标记被按下1次
        long_press_time++;		  
    }

    if(long_press_time>(uint16_t)(500.0f/Count_time))// 长按1秒
    {	
        check_once=1;//标记已被识别
        return long_click; //长按
    }

    //按键被按下1次又弹起后，开启内核走时
    if(press_flag&&1==keyValue())
    {
        time_core++; 
    }		
	
    if(press_flag&&(time_core>(uint16_t)(50.0f/Count_time)&&time_core<(uint16_t)(300.0f/Count_time)))//50~700ms内被再次按下
    {
        if(0==keyValue()) //如果再次按下
        {
            check_once=1;//标记已被识别
            return double_click; //标记为双击
        }
    }
    else if(press_flag&&time_core>(uint16_t)(300.0f/Count_time))
    {
        check_once=1;//标记已被识别
        return single_click; //800ms后还没被按下，则是单击
    }

    return key_stateless;
}

//驱动挂载
KeyInterface_t UserKey = {
    .getKeyState = key_scan,
};

