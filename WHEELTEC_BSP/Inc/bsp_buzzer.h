#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include <stdint.h>

typedef struct {
	void (*init)(void);
    void (*on)(void);
    void (*off)(void);
	void (*toggle)(void);
	void (*AddTask)(uint8_t cnt,uint16_t time);
	void (*AddHoldTask)(uint16_t time);
}BuzzerInterface_t,*pBuzzerInterface_t;

extern BuzzerInterface_t UserBuzzer;

#endif