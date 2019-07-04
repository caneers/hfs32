/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define BUFFER_SIZE  50
extern volatile uint8_t rx_len;           //接收一帧数据的长度
extern volatile uint8_t recv_end_flag;    //一帧数据接收完成标志
extern uint8_t rx_buffer[50];            //接收数据缓存


#define ADC_CHANNEL_CNT     10


 
extern uint32_t AD_Buf[ADC_CHANNEL_CNT];
extern uint32_t DMA_CNT;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define BOOST_PWM_Pin GPIO_PIN_3
#define BOOST_PWM_GPIO_Port GPIOA
#define COUNTIN_Pin GPIO_PIN_4
#define COUNTIN_GPIO_Port GPIOA
#define SET_3V_Pin GPIO_PIN_5
#define SET_3V_GPIO_Port GPIOA
#define OUT_LED_Pin GPIO_PIN_7
#define OUT_LED_GPIO_Port GPIOA
#define COM_REF_Pin GPIO_PIN_1
#define COM_REF_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
 



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
