#ifndef __BSP_EEPROM_H
#define __BSP_EEPROM_H

#include <stdint.h>

uint8_t eeprom_writebytes(uint8_t addr,uint8_t *data,uint8_t bufferLen);
uint8_t eeprom_readbytes(uint8_t addr,uint8_t* data,uint8_t readLen);

#endif


