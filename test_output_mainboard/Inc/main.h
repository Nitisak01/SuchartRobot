/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Relay4_Pin GPIO_PIN_2
#define Relay4_GPIO_Port GPIOE
#define Relay3_Pin GPIO_PIN_0
#define Relay3_GPIO_Port GPIOA
#define Relay2_Pin GPIO_PIN_0
#define Relay2_GPIO_Port GPIOB
#define Relay8_Pin GPIO_PIN_2
#define Relay8_GPIO_Port GPIOB
#define Select3_Pin GPIO_PIN_12
#define Select3_GPIO_Port GPIOF
#define Select4_Pin GPIO_PIN_13
#define Select4_GPIO_Port GPIOF
#define Start_Pin GPIO_PIN_0
#define Start_GPIO_Port GPIOG
#define Start_EXTI_IRQn EXTI0_IRQn
#define Select5_Pin GPIO_PIN_9
#define Select5_GPIO_Port GPIOE
#define Select6_Pin GPIO_PIN_11
#define Select6_GPIO_Port GPIOE
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOB
#define Relay5_Pin GPIO_PIN_11
#define Relay5_GPIO_Port GPIOD
#define Relay6_Pin GPIO_PIN_12
#define Relay6_GPIO_Port GPIOD
#define Relay7_Pin GPIO_PIN_13
#define Relay7_GPIO_Port GPIOD
#define Select1_Pin GPIO_PIN_14
#define Select1_GPIO_Port GPIOD
#define Select2_Pin GPIO_PIN_15
#define Select2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOC
#define LED6_Pin GPIO_PIN_7
#define LED6_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_15
#define LED5_GPIO_Port GPIOA
#define LED8_Pin GPIO_PIN_3
#define LED8_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_5
#define LED7_GPIO_Port GPIOB
#define Relay1_Pin GPIO_PIN_0
#define Relay1_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
