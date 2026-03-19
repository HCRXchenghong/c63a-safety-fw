#include "bsp_led.h"
#include "gpio.h"

static void led_init(void)
{
	
}

static void led_on(void)
{
	HAL_GPIO_WritePin(UserLED_GPIO_Port,UserLED_Pin,GPIO_PIN_SET);
}

static void led_off(void)
{
	HAL_GPIO_WritePin(UserLED_GPIO_Port,UserLED_Pin,GPIO_PIN_RESET);
}

static void led_toggle(void)
{
	HAL_GPIO_TogglePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin);
}

LEDInterface_t UserLED = {
	.init = led_init,
	.on = led_on,
	.off = led_off,
	.toggle = led_toggle
};


