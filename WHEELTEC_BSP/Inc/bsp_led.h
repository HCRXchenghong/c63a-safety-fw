#ifndef __BSP_LED_H
#define __BSP_LED_H

#include <stdint.h>

typedef struct {
	void (*init)(void);
    void (*on)(void);
    void (*off)(void);
	void (*toggle)(void);
}LEDInterface_t,*pLEDInterface_t;

extern LEDInterface_t UserLED;

#endif
