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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define USB_IN_MAX_PACKET_SIZE 64
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOC
#define MOT_RA_DIR_Pin GPIO_PIN_0
#define MOT_RA_DIR_GPIO_Port GPIOA
#define MOT_RA_STEP_Pin GPIO_PIN_1
#define MOT_RA_STEP_GPIO_Port GPIOA
#define MOT_RA_ENABLE_Pin GPIO_PIN_2
#define MOT_RA_ENABLE_GPIO_Port GPIOA
#define MOT_RA_M2_Pin GPIO_PIN_3
#define MOT_RA_M2_GPIO_Port GPIOA
#define MOT_RA_M1_Pin GPIO_PIN_4
#define MOT_RA_M1_GPIO_Port GPIOA
#define MOT_RA_M0_Pin GPIO_PIN_5
#define MOT_RA_M0_GPIO_Port GPIOA
#define MOT_DEC_DIR_Pin GPIO_PIN_6
#define MOT_DEC_DIR_GPIO_Port GPIOA
#define MOT_DEC_STEP_Pin GPIO_PIN_7
#define MOT_DEC_STEP_GPIO_Port GPIOA
#define MOT_DEC_ENABLE_Pin GPIO_PIN_0
#define MOT_DEC_ENABLE_GPIO_Port GPIOB
#define MOT_DEC_M2_Pin GPIO_PIN_1
#define MOT_DEC_M2_GPIO_Port GPIOB
#define MOT_DEC_M1_Pin GPIO_PIN_10
#define MOT_DEC_M1_GPIO_Port GPIOB
#define MOT_DEC_M0_Pin GPIO_PIN_11
#define MOT_DEC_M0_GPIO_Port GPIOB
#define ENC_RA_CS_Pin GPIO_PIN_12
#define ENC_RA_CS_GPIO_Port GPIOB
#define ENC_GEN_Pin GPIO_PIN_10
#define ENC_GEN_GPIO_Port GPIOA
#define ENC_DEC_CS_Pin GPIO_PIN_4
#define ENC_DEC_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
