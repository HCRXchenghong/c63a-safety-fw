/**
 * @file    bsp_eeprom.c
 * @brief   BL24C02F-RRRC器件驱动(256bytes空间的EEPROM)
 * @author  WHEELTEC
 * @date    2025-08-05
 * @version 1.0.0
 *
 * @details
 * - C63X - V2.0版本才有此器件,V1.0版本没有此器件.软件通过此特征来区分当前的硬件运行环境
 * BL24C02F共有256bytes存储空间,分别是地址 0-255,支持单字节写入、多字节写入(最多一次性写入1页)
 * 一页为16bytes.
 * @note
 * 
 * 
 */
 
#include "bsp_eeprom.h"
#include "bsp_siic.h"

//IIC驱动
static pIICInterface_t eeprom = &UserVer_sIICDev;
	
//EEPROM写入数据
//1次最多写入16字节
uint8_t eeprom_writebytes(uint8_t addr,uint8_t *data,uint8_t bufferLen)
{
	if( bufferLen > 16 ) return 1;
	
	return eeprom->write_reg(&eeprom->iic_io,0x50<<1,addr,data,bufferLen,0xFFFF);
}
	
//EEPROM读数据
uint8_t eeprom_readbytes(uint8_t addr,uint8_t* data,uint8_t readLen)
{
	uint8_t state = 0;
	for(uint8_t r=addr;r<addr+readLen;r++)
	{
		state+=eeprom->read_reg(&eeprom->iic_io,0x50<<1,r,data++,1,0xFFFF);
		eeprom->delay_ms(5);
	}
	return state;
}

//版本确认示例
//C63X硬件版本确定(通过EEPROM数据操作,确认硬件版本)
// HAL_Delay(500);
// uint8_t write_ver = 1;
// uint8_t iic_state = 0;
// iic_state += eeprom_writebytes(0,&write_ver,1);

// HAL_Delay(5);
// uint8_t read_ver = 0xff;
// iic_state += eeprom_readbytes(0,&read_ver,1);

// //IIC通信正常,版本号能正确写入与读出,则为V2.0版本硬件
// if( iic_state==0 && read_ver == 1 )
// {
// 	HardWareVersion = HW_1_1;
// }
// else
// {
// 	HardWareVersion = HW_1_0;
	
// 	//反初始化V2.0版本的CAN引脚
// 	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);
	
// 	//初始化V1.0版本CAN引脚
// 	GPIO_InitTypeDef GPIO_InitStruct = {0};
// 	__HAL_RCC_GPIOA_CLK_ENABLE();
// 	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
// 	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
// 	GPIO_InitStruct.Pull = GPIO_NOPULL;
// 	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
// 	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
// 	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
// 	//V1.0版本急停开关
// 	HAL_GPIO_DeInit(ENKey_GPIO_Port,ENKey_Pin);
// 	__HAL_RCC_GPIOD_CLK_ENABLE();
// 	GPIO_InitStruct.Pin = ENKey_V1_0_Pin;
// 	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
// 	GPIO_InitStruct.Pull = GPIO_PULLUP;
// 	HAL_GPIO_Init(ENKey_V1_0_GPIO_Port, &GPIO_InitStruct);
// }
	