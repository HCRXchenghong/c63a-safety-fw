#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include <stdint.h>

typedef struct{
	void (*init)(void);                                        //CAN初始化
	uint8_t (*sendStd)(uint32_t id,uint8_t *pData,uint8_t Len); //发送标准帧数据
	uint8_t (*sendExt)(uint32_t id,uint8_t *pData,uint8_t Len); //发送扩展帧数据
}CANInterface_t,*pCANInterface_t; 

typedef struct{
	uint32_t id;
	uint8_t buffer[8];
}CANmsgType_t;

extern CANInterface_t UserCAN1Dev , UserCAN2Dev;

#endif /* __BSP_CAN_H */
