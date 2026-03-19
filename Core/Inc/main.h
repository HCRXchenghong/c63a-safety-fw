/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
uint8_t get_HardWareVersion(void);
GPIO_PinState get_EnKeyState(void);
enum{
	HW_UnKnown = 0, //Ӳ���汾
	HW_1_0 , 
	HW_1_1
};

//��ͣ����
#define ENKey_V1_0_Pin       GPIO_PIN_0
#define ENKey_V1_0_GPIO_Port GPIOD

#define UserKey_V1_0_Pin  GPIO_PIN_3
#define UserKey_V1_0_Port GPIOD

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_DC_Pin GPIO_PIN_4
#define OLED_DC_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOE
#define OLED_RES_Pin GPIO_PIN_13
#define OLED_RES_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_14
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_15
#define OLED_SCL_GPIO_Port GPIOC
#define EchoD_Pin GPIO_PIN_0
#define EchoD_GPIO_Port GPIOA
#define EchoE_Pin GPIO_PIN_1
#define EchoE_GPIO_Port GPIOA
#define BlueToothTX_Pin GPIO_PIN_2
#define BlueToothTX_GPIO_Port GPIOA
#define BlueToothRX_Pin GPIO_PIN_3
#define BlueToothRX_GPIO_Port GPIOA
#define TriD_Pin GPIO_PIN_4
#define TriD_GPIO_Port GPIOA
#define TriE_Pin GPIO_PIN_5
#define TriE_GPIO_Port GPIOA
#define EchoC_Pin GPIO_PIN_6
#define EchoC_GPIO_Port GPIOA
#define EchoF_Pin GPIO_PIN_7
#define EchoF_GPIO_Port GPIOA
#define TriC_Pin GPIO_PIN_4
#define TriC_GPIO_Port GPIOC
#define TriF_Pin GPIO_PIN_5
#define TriF_GPIO_Port GPIOC
#define EchoB_Pin GPIO_PIN_0
#define EchoB_GPIO_Port GPIOB
#define EchoA_Pin GPIO_PIN_1
#define EchoA_GPIO_Port GPIOB
#define TriB_Pin GPIO_PIN_7
#define TriB_GPIO_Port GPIOE
#define TriA_Pin GPIO_PIN_8
#define TriA_GPIO_Port GPIOE
#define VersionBit0_Pin GPIO_PIN_12
#define VersionBit0_GPIO_Port GPIOC
#define UserKey_Pin GPIO_PIN_3
#define UserKey_GPIO_Port GPIOD
#define UserBuzzer_Pin GPIO_PIN_4
#define UserBuzzer_GPIO_Port GPIOD
#define UserLED_Pin GPIO_PIN_5
#define UserLED_GPIO_Port GPIOD
#define VersionBit1_Pin GPIO_PIN_6
#define VersionBit1_GPIO_Port GPIOD
#define VersionBit2_Pin GPIO_PIN_7
#define VersionBit2_GPIO_Port GPIOD
#define IIC_SCL_Pin GPIO_PIN_6
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_7
#define IIC_SDA_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_9
#define LED_G_GPIO_Port GPIOB
#define ENKey_Pin GPIO_PIN_0
#define ENKey_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
//全局调试使用
typedef struct{
	uint8_t powerlost;
	uint8_t chargingfull;
	uint8_t chargingON;
}DebugType_t;

//extern DebugType_t g_sys_debug;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
