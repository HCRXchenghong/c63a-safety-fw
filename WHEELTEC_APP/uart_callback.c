#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//串口控制互斥锁
static uint8_t serial_control_lock = 0;
static uint32_t serial_control_tick = 0;

//串口接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	extern uint8_t BlueToothBuffer;
	extern uint8_t rosbuffer;
	extern uint8_t usart1_buffer;
	extern uint8_t rs485_buffer;
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//蓝牙使用的串口
	if( huart == &huart2 )
	{
		/* APP写入指令 */
		extern QueueHandle_t g_xQueueBlueTooth;
		
		//中断写队列必须做非空判断,以防开机瞬间被写入导致异常
		if( g_xQueueBlueTooth!=NULL )
			xQueueSendFromISR(g_xQueueBlueTooth,&BlueToothBuffer,&xHigherPriorityTaskWoken);
		
		HAL_UART_Receive_IT(&huart2,&BlueToothBuffer,1);
	}
	
	//ROS使用的串口
	else if( huart == &huart4 )
	{
		extern QueueHandle_t g_xQueueROSserial;
		
		//数值1为最高优先级互斥锁
		serial_control_lock = 1;
		
		if( g_xQueueROSserial!=NULL )
		{
			xQueueSendFromISR(g_xQueueROSserial,&rosbuffer,&xHigherPriorityTaskWoken);
			serial_control_tick = xTaskGetTickCountFromISR();
		}
			
		HAL_UART_Receive_IT(&huart4,&rosbuffer,1);
	}
	
	//串口1接收缓冲区
	else if( huart == &huart1 )
	{
		extern QueueHandle_t g_xQueueROSserial;
		
		//带优先级的互斥锁判断,避免多个串口同时写1个队列
		// 2为人为规定的优先级,数值越低优先级越高
		if( 2 <= serial_control_lock )
		{
			serial_control_lock=2;
			if( g_xQueueROSserial!=NULL )
			{
				xQueueSendFromISR(g_xQueueROSserial,&usart1_buffer,&xHigherPriorityTaskWoken);
				serial_control_tick = xTaskGetTickCountFromISR();
			}
		}
		else
		{
			if((xTaskGetTickCountFromISR() - serial_control_tick) > 2000)
				serial_control_lock = 2;
		}

		HAL_UART_Receive_IT(&huart1,&usart1_buffer,1);
	}
	
	//485接收缓冲区
	else if( huart == &huart3 )
	{
		//HAL_UART_Transmit(&huart3,&rs485_buffer,1,200);
		HAL_UART_Receive_IT(&huart3,&rs485_buffer,1);
	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
