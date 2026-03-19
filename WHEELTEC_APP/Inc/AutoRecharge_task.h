#ifndef __AUTORECHARGE_TASK_H
#define __AUTORECHARGE_TASK_H

#include <stdint.h>

typedef struct{
	uint8_t online;
	uint8_t R_A;
	uint8_t R_B;
	uint8_t L_A;
	uint8_t L_B;
	uint8_t RedNum;
	uint8_t ChargingFlag;//是否已经在充电的标志位
	uint8_t devState;    //充电装备内部状态机
	float  ChargingCur;  //充电电流
	float  ChargingVol;  //充电电压
	
	float dockerX;
	float dockerY;
	float dockerZ;
}ChargeDevType_t;

extern ChargeDevType_t ChargeDev;

#endif

