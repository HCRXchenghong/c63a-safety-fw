#include "bsp_buzzer.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

static uint8_t TaskTipsCount = 0;
static uint16_t TaskTipsTimeInt = 0;
static volatile uint8_t taskRunning = 0;
static TaskHandle_t g_buzzerTaskHandle = NULL;

enum{
	BuzzerTaskMode_Beep = 0,
	BuzzerTaskMode_Hold,
};

static uint8_t g_buzzerTaskMode = BuzzerTaskMode_Beep;

static void buzzer_init(void)
{
	
}

static void buzzer_on(void)
{
	HAL_GPIO_WritePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin,GPIO_PIN_SET);
}

static void buzzer_off(void)
{
	HAL_GPIO_WritePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin,GPIO_PIN_RESET);
}

static void buzzer_toggle(void)
{
	HAL_GPIO_TogglePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin);
}

static void buzzer_start_task(void* param)
{
	if( g_buzzerTaskMode == BuzzerTaskMode_Hold )
	{
		buzzer_on();
		vTaskDelay( pdMS_TO_TICKS(TaskTipsTimeInt) );
		buzzer_off();
	}
	else
	{
		for(uint8_t i=0;i<TaskTipsCount;i++)
		{
			buzzer_on();
			vTaskDelay( pdMS_TO_TICKS(TaskTipsTimeInt) );
			buzzer_off();
			vTaskDelay( pdMS_TO_TICKS(TaskTipsTimeInt) );
		}
	}
	
	taskRunning = 0;
	g_buzzerTaskHandle = NULL;
	vTaskDelete(NULL);
}

static void buzzer_SetTask(uint8_t tipscnt,uint16_t time_int )
{
	if( taskRunning==1 ) return;
	TaskTipsCount = tipscnt;
	TaskTipsTimeInt = time_int;
	g_buzzerTaskMode = BuzzerTaskMode_Beep;
	taskRunning = 1;
	if( xTaskCreate(buzzer_start_task,"buzzerTask",64,NULL,osPriorityNormal,&g_buzzerTaskHandle) != pdPASS )
	{
		taskRunning = 0;
		g_buzzerTaskHandle = NULL;
	}
}

static void buzzer_SetHoldTask(uint16_t time_int)
{
	if( g_buzzerTaskHandle != NULL )
	{
		vTaskDelete(g_buzzerTaskHandle);
		g_buzzerTaskHandle = NULL;
		taskRunning = 0;
		buzzer_off();
	}

	TaskTipsCount = 1;
	TaskTipsTimeInt = time_int;
	g_buzzerTaskMode = BuzzerTaskMode_Hold;
	taskRunning = 1;
	if( xTaskCreate(buzzer_start_task,"buzzerTask",64,NULL,osPriorityNormal,&g_buzzerTaskHandle) != pdPASS )
	{
		taskRunning = 0;
		g_buzzerTaskHandle = NULL;
		buzzer_off();
	}
}

BuzzerInterface_t UserBuzzer = {
	.init = buzzer_init,
	.on = buzzer_on,
	.off = buzzer_off,
	.toggle = buzzer_toggle,
	.AddTask = buzzer_SetTask,
	.AddHoldTask = buzzer_SetHoldTask
};