/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define MOT_RA_PERIOD_SIDEREAL 3217
#define USB_IN_MAX_PACKET_SIZE 32
#define MOT_RA_PSC_SIDEREAL 279
#define MOT_DEC_PERIOD_SIDEREAL 2476
#define MOT_DEC_PSC_SIDEREAL 208

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
#define DEC_LIMIT_FORWARD_Pin GPIO_PIN_15
#define DEC_LIMIT_FORWARD_GPIO_Port GPIOA
#define DEC_LIMIT_REVERSE_Pin GPIO_PIN_3
#define DEC_LIMIT_REVERSE_GPIO_Port GPIOB
#define ENC_DEC_CS_Pin GPIO_PIN_4
#define ENC_DEC_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/