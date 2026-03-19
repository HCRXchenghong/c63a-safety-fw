#ifndef __SENSOR_RANGER_H
#define __SENSOR_RANGER_H

#include <stdio.h>
#include <stdlib.h>

#include "tim.h"
#include "gpio.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h" 

#include "bsp_dwt.h"

//与硬件绑定的超声波对象
typedef struct{
	uint8_t sm;
	float dis;
	uint32_t upvalue;
	uint32_t downvalue;
	TIM_HandleTypeDef* tim;
	uint64_t ch;
	GPIO_TypeDef *TrigPort;
	uint16_t TrigPin;
	BaseType_t (*CaptureCompleted)(uint8_t index);
	uint8_t index ;
	uint8_t timeoutflag;
	uint8_t timeoutCount;
	volatile uint32_t sampleSeq;
}RangerHALObj;

extern RangerHALObj RangerHAL_A,RangerHAL_B,RangerHAL_C,RangerHAL_D,RangerHAL_E,RangerHAL_F;

void Ranger_IC_CaptureCallback(TIM_HandleTypeDef *htim,BaseType_t* woken);

#endif /* __SENSOR_RANGER_H */

