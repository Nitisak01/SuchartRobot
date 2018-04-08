/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "dwt_stm32_delay.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int angle;
float pul;
float gear_ratio ;
uint8_t datalenght = 4;
uint8_t dataRx[4];
uint8_t dataTx[4];
int i = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  dataRx[0] = 0;
	dataRx[1] = 0;
	dataRx[2] = 0;
	dataRx[3] = 0;
	dataTx[0] = 5;
	dataTx[1] = 5;
	dataTx[2] = 5;
	dataTx[3] = 5;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  DWT_Delay_Init ();
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	TIM2->CCR3 = 0 ; /// period 20000
	HAL_SPI_Receive_IT(&hspi1,(uint8_t*)dataRx,datalenght);
	

  int speed = 2000;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
 
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	

  #if 0 //Joint6
		TIM2->CCR3 = 600;
		HAL_Delay(2000);
		TIM2->CCR3 = 2100;
		HAL_Delay(2000);
		
  #endif		
		
	
	#if 0 /// Joint5
		angle = 100;
		gear_ratio = 22/10;
	 HAL_GPIO_WritePin(Dir_step_GPIO_Port,Dir_step_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_RESET);
		pul  = (angle*4*5.18*gear_ratio)/(1.8); 
	for(int i = 0;i<=pul ;i++){
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_RESET);
		DWT_Delay_us(1);
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
		DWT_Delay_us(500);
	}
  HAL_Delay(1000); 
	HAL_GPIO_WritePin(Dir_step_GPIO_Port,Dir_step_Pin,GPIO_PIN_RESET);
	for(int i = 0;i<=pul ;i++){
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_RESET);
		DWT_Delay_us(1);
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
		DWT_Delay_us(500);
	}
	HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_SET);
  HAL_Delay(1000);
	#endif
	
	#if 0 /// Joint4
	angle = 275;
		gear_ratio = 22/12;
	 HAL_GPIO_WritePin(Dir_step_GPIO_Port,Dir_step_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_RESET);
		pul  = (angle*8*gear_ratio)/(1.8); 
	for(int i = 0;i<=pul ;i++){
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_RESET);
		DWT_Delay_us(1);
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
		DWT_Delay_us(1000);
	}
  HAL_Delay(1000); 
	HAL_GPIO_WritePin(Dir_step_GPIO_Port,Dir_step_Pin,GPIO_PIN_RESET);
	for(int i = 0;i<=pul ;i++){
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_RESET);
		DWT_Delay_us(1);
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
		DWT_Delay_us(1000);
	}
	HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_SET);
  HAL_Delay(1000);
	#endif
	
	#if 0 /// Joint3
	angle = 10000;
		gear_ratio = 22/10;
	 HAL_GPIO_WritePin(Dir_step_GPIO_Port,Dir_step_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_RESET);
		pul  = (angle*4*gear_ratio)/(1.8); 
	for(int i = 0;i<=pul ;i++){
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_RESET);
		DWT_Delay_us(1);
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
		DWT_Delay_us(3500);
	}
	HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
  HAL_Delay(1000); 
	HAL_GPIO_WritePin(Dir_step_GPIO_Port,Dir_step_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_RESET);
	for(int i = 0;i<=pul ;i++){
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_RESET);
		DWT_Delay_us(1);
		HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
		DWT_Delay_us(3500);
	}
	HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
  HAL_Delay(1000);
	#endif
	
	#if 1 /// Joint2
	angle = 10;
		gear_ratio = 20/60;
	 HAL_GPIO_WritePin(Dir_step_GPIO_Port,Dir_step_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_RESET);
		pul  = (angle*2*gear_ratio)/(1.8); 
	for(int i = 0;i<=100/3;i++){
		for(int j=0; j<3; j++){
      HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_RESET);
      DWT_Delay_us(1);
      HAL_GPIO_WritePin(Pul_step_GPIO_Port,Pul_step_Pin,GPIO_PIN_SET);
      DWT_Delay_us(speed);   
    }
    if(speed > 300){
      speed--;
    }
	}
	HAL_GPIO_WritePin(En_step_GPIO_Port,En_step_Pin,GPIO_PIN_SET);
	#endif
	
	#if 0 //Joint 1
	  HAL_GPIO_WritePin(Dir_Dc_GPIO_Port,Dir_Dc_Pin,GPIO_PIN_SET);
	  TIM2->CCR3 = 4000;
		HAL_Delay(10000);
		HAL_GPIO_WritePin(Dir_Dc_GPIO_Port,Dir_Dc_Pin,GPIO_PIN_RESET);
		HAL_Delay(10000);
	#endif
	
	
	
	
	
	}
	
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Dir_Dc_Pin|En_step_Pin|Pul_step_Pin|Dir_step_Pin 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Dir_Dc_Pin En_step_Pin Pul_step_Pin Dir_step_Pin 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = Dir_Dc_Pin|En_step_Pin|Pul_step_Pin|Dir_step_Pin 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == hspi1.Instance){
		HAL_SPI_Receive_IT(&hspi1,dataRx,datalenght);
		HAL_SPI_Transmit_IT(&hspi1,dataTx,datalenght);
	}
}

void SendPackageSPI (SPI_HandleTypeDef *hspi, uint8_t *package,uint8_t lenght_data){
	    HAL_SPI_Transmit(hspi,(uint8_t*)package,lenght_data,100);
	    while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
