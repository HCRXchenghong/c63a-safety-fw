#ifndef __BSP_sIIC_H
#define __BSP_sIIC_H

#include <stdint.h>

#include "gpio.h"

typedef enum 
{
	IIC_OK       = 0x00U,
	IIC_ERR      = 0x01U,
	IIC_BUSY     = 0x02U,
	IIC_TIMEOUT  = 0x03U,
	I2C_BUS_ERROR,
	I2C_ARBITRATION_LOST,
	I2C_ADDR_NACK
} IIC_Status_t;

typedef struct {
	uint16_t pin;
	GPIO_TypeDef* port;
}GPIOConfigType_t;

typedef struct {
	GPIOConfigType_t* SCL;
	GPIOConfigType_t* SDA;
}sIIC_IOType_t;

typedef struct {
	sIIC_IOType_t iic_io;
	void (*init)(void);
	//参数：设备地址，要写入的数据，要写入的数据长度，超时时间
    IIC_Status_t (*write)(sIIC_IOType_t* io,uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    IIC_Status_t (*read)(sIIC_IOType_t* io,uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	
	//参数：设备地址，要访问的寄存器地址，要写入的数据，要写入的数据长度，超时时间
    IIC_Status_t (*write_reg)(sIIC_IOType_t* io,uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	IIC_Status_t (*read_reg)(sIIC_IOType_t* io,uint16_t DevAddress, uint16_t MemAddress,  uint8_t *pData, uint16_t Size, uint32_t Timeout);
	
	void (*delay_ms)(uint16_t ms);
}IICInterface_t,*pIICInterface_t;


extern IICInterface_t UserIMU_sIICDev;

#endif /* __BSP_IIC_H */

