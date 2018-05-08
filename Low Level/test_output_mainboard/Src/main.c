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
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t spiRx[50];
uint16_t CurrentJ[10];
uint16_t J[10];
uint8_t run_mode;
uint8_t Uarttxbuffer[50];
uint8_t spitxbuffer[50];
uint8_t Spitxbuffer[50];
uint8_t error = 1;
uint8_t Jerror[6];
int n=0;
int MaxLimitJ[10];
typedef struct{
	uint8_t Address;
	uint8_t value;
}Pos;

typedef struct{
	uint8_t ID;
	uint8_t Lenght;
	uint8_t Inst;
	uint8_t Address;
	uint8_t NumberOfRead;
	uint8_t NumberOfWrite;
	uint8_t Parameter[50];
	uint8_t Pingerror;
	
	Pos GoalPosJ1L;
	Pos GoalPosJ1H;
	Pos GoalPosJ2L;
	Pos GoalPosJ2H;
	Pos GoalPosJ3L;
	Pos GoalPosJ3H;
	Pos GoalPosJ4L;
	Pos GoalPosJ4H;
	Pos GoalPosJ5L;
	Pos GoalPosJ5H;
	Pos GoalPosJ6L;
	Pos GoalPosJ6H;
	
	Pos CurrentPosJ1L;
	Pos CurrentPosJ1H;
	Pos CurrentPosJ2L;
	Pos CurrentPosJ2H;
	Pos CurrentPosJ3L;
	Pos CurrentPosJ3H;
	Pos CurrentPosJ4L;
	Pos CurrentPosJ4H;
	Pos CurrentPosJ5L;
	Pos CurrentPosJ5H;
	Pos CurrentPosJ6L;
	Pos CurrentPosJ6H;
	
	Pos AllJComplete;
	Pos Gripper_Status;
	Pos GameState;
	
	
}DataPackageStruct;

typedef struct{
	uint8_t Buffer[50];
	uint8_t Lenght;
	uint8_t checksum;
	uint8_t BufferIndex;
	uint8_t rxBuff;
	uint8_t EndLenght;
	uint8_t RxData[50];
}RxStruct;

typedef enum{
	FRAME_WAIT,
	FRAME_PROGRESS
}FRAME_STATE;

typedef enum{
	Ping,
	READ_BIT,
	Write
}Machine_STATE;

FRAME_STATE Framestate;
RxStruct UartRxBuffer;
RxStruct spiRxBuffer;
DataPackageStruct UartData;
Machine_STATE BotState;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
void ClearBuffer(RxStruct *RxBuffer);
void BufferSerial(RxStruct *RxBuffer);
void variableInit();
void CheckPackage(DataPackageStruct *Data,uint8_t *Buffer);
void CreatePackage(uint8_t *Tx,uint8_t ID,uint8_t Lenght,uint8_t *Data);
void Vaccumm(int Switch);
uint8_t InsRead(uint8_t Address);
uint8_t InsWrite(uint8_t Address,uint8_t* Data);
uint8_t SetPosJoint1(uint8_t posLow,uint8_t posHigh);
uint8_t SetPosJoint2(uint8_t posLow,uint8_t posHigh);
uint8_t SetPosJoint3(uint8_t posLow,uint8_t posHigh);
uint8_t SetPosJoint4(uint8_t posLow,uint8_t posHigh);
uint8_t SetPosJoint5(uint8_t posLow,uint8_t posHigh);
uint8_t SetPosJoint6(uint8_t posLow,uint8_t posHigh);
uint8_t SetGripper(uint8_t cmd);
_Bool checksum(uint8_t *package);
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define SpiEn1 HAL_GPIO_WritePin (Select1_GPIO_Port,Select1_Pin, GPIO_PIN_RESET);
#define SpiDis1 HAL_GPIO_WritePin(Select1_GPIO_Port,Select1_Pin, GPIO_PIN_SET);
#define SpiEn2 HAL_GPIO_WritePin (Select2_GPIO_Port,Select2_Pin, GPIO_PIN_RESET);
#define SpiDis2 HAL_GPIO_WritePin(Select2_GPIO_Port,Select2_Pin, GPIO_PIN_SET);
#define SpiEn3 HAL_GPIO_WritePin (Select3_GPIO_Port,Select3_Pin, GPIO_PIN_RESET);
#define SpiDis3 HAL_GPIO_WritePin(Select3_GPIO_Port,Select3_Pin, GPIO_PIN_SET);
#define SpiEn4 HAL_GPIO_WritePin (Select4_GPIO_Port,Select4_Pin, GPIO_PIN_RESET);
#define SpiDis4 HAL_GPIO_WritePin(Select4_GPIO_Port,Select4_Pin, GPIO_PIN_SET);
#define SpiEn5 HAL_GPIO_WritePin (Select5_GPIO_Port,Select5_Pin, GPIO_PIN_RESET);
#define SpiDis5 HAL_GPIO_WritePin(Select5_GPIO_Port,Select5_Pin, GPIO_PIN_SET);
#define SpiEn6 HAL_GPIO_WritePin (Select6_GPIO_Port,Select6_Pin, GPIO_PIN_RESET);
#define SpiDis6 HAL_GPIO_WritePin(Select6_GPIO_Port,Select6_Pin, GPIO_PIN_SET);

	/////Config Adress Serial UART
#define GoalPosJ1LAddress 0x00;
#define GoalPosJ1HAddress 0x01;
#define GoalPosJ2LAddress 0x02;
#define GoalPosJ2HAddress 0x03;
#define GoalPosJ3LAddress 0x04;
#define GoalPosJ3HAddress 0x05;
#define GoalPosJ4LAddress 0x06;
#define GoalPosJ4HAddress 0x07;
#define GoalPosJ5LAddress 0x08;
#define GoalPosJ5HAddress 0x09;
#define GoalPosJ6LAddress 0x0A;
#define GoalPosJ6HAddress 0x0B;

#define CurrentPosJ1LAddress 0x0C;
#define CurrentPosJ1HAddress 0x0D;
#define CurrentPosJ2LAddress 0x0E;
#define CurrentPosJ2HAddress 0x0F;
#define CurrentPosJ3LAddress 0x10;
#define CurrentPosJ3HAddress 0x11;
#define CurrentPosJ4LAddress 0x12;
#define CurrentPosJ4HAddress 0x13;
#define CurrentPosJ5LAddress 0x14;
#define CurrentPosJ5HAddress 0x15;
#define CurrentPosJ6LAddress 0x16;
#define CurrentPosJ6HAddress 0x17;

#define AllJCompleteAddress  0x18;
#define Gripper_StatusAddress 0x19;
#define GameStateAddress     0x1A;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
	variableInit();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	//HAL_SPI_Receive_IT(&hspi1,&SpiRxBuffer.rxBuff,1);
	HAL_UART_Receive_IT(&huart3,&UartRxBuffer.rxBuff,1);
	SpiDis1;
	SpiDis2;
	SpiDis3;
	SpiDis4;
	SpiDis5;
	SpiDis6;
	
	while(error == 1){
		spitxbuffer[0] = 0xff;
		spitxbuffer[1] = 0xff;
		spitxbuffer[2] = 0x01;
		spitxbuffer[3] = 0x05;
		spitxbuffer[4] = 0x01;
		spitxbuffer[5] = 0x00;
		spitxbuffer[6] = 0x00;
		spitxbuffer[7] = 0x00;
		spitxbuffer[8] = 0xf8;
			////////////////1/////////////////
		for (int i = 0;i<2;i++){
			n = 1;
				while(n == 1){
					SpiEn1;
					if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
						SpiDis1;
						n = 0;
					}
			}
				HAL_Delay(10);
			}
			if(checksum(spiRxBuffer.Buffer)){
				for (int n = 0; n < 9 ;n++){
						spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
				}
				Jerror[0] = spiRxBuffer.RxData[7];
			}
			if(Jerror[0] == 1){ HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);}
			else  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
			////////////////2/////////////////
			for (int i = 0;i<2;i++){
			n = 1;
				while(n == 1){
					SpiEn2;
					if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
						SpiDis2;
						n = 0;
					}
			}
				HAL_Delay(10);
			}
			if(checksum(spiRxBuffer.Buffer)){
				for (int n = 0; n < 9 ;n++){
						spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
				}
				Jerror[1] = spiRxBuffer.RxData[7];
			}
			if(Jerror[1] == 1){ HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);}
			else  HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
			
			//////////////////3////////////////
			for (int i = 0;i<2;i++){
			n = 1;
				while(n == 1){
					SpiEn3;
					if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
						SpiDis3;
						n = 0;
					}
			}
				HAL_Delay(10);
			}
			if(checksum(spiRxBuffer.Buffer)){
				for (int n = 0; n < 9 ;n++){
						spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
				}
				Jerror[2] = spiRxBuffer.RxData[7];
			}
			if(Jerror[2] == 1){ HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);}
			else  HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
			/////////////////4////////////////
			for (int i = 0;i<2;i++){
			n = 1;
				while(n == 1){
					SpiEn4;
					if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
						SpiDis4;
						n = 0;
					}
			}
				HAL_Delay(10);
			}
			if(checksum(spiRxBuffer.Buffer)){
				for (int n = 0; n < 9 ;n++){
						spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
				}
				Jerror[3] = spiRxBuffer.RxData[7];
			}
			if(Jerror[3] == 1){ HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET);}
			else  HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET);
			
			//////////////////5/////////////////////
			for (int i = 0;i<2;i++){
			n = 1;
				while(n == 1){
					SpiEn5;
					if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
						SpiDis5;
						n = 0;
					}
			}
				HAL_Delay(10);
			}
			if(checksum(spiRxBuffer.Buffer)){
				for (int n = 0; n < 9 ;n++){
						spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
				}
				Jerror[4] = spiRxBuffer.RxData[7];
			}
			if(Jerror[4] == 1){ HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_SET);}
			else  HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_RESET);
			
			///////////////////6////////////////////
			for (int i = 0;i<2;i++){
			n = 1;
				while(n == 1){
					SpiEn6;
					if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
						SpiDis6;
						n = 0;
					}
			}
				HAL_Delay(10);
			}
			if(checksum(spiRxBuffer.Buffer)){
				for (int n = 0; n < 9 ;n++){
						spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
				}
				Jerror[5] = spiRxBuffer.RxData[7];
			}
			if(Jerror[5] == 1){ HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);}
			else  HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
			
			HAL_Delay(200);
			if(Jerror[0] == 1 && Jerror[1] == 1 && Jerror[2] == 1 && Jerror[3] == 1 && Jerror[4] == 1 && Jerror[5] == 1){
				UartData.Pingerror = 0x00;
				error = 0;
				for (int i = 0; i<4; i ++){
					HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
					HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
					HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
					HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
					HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin);
					HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin);
					HAL_Delay(500);
				}
			}
			else{
				UartData.Pingerror = 0x01; 
			}
			

	}
	HAL_TIM_Base_Stop_IT(&htim2);
	Jerror[0] = 0;
	Jerror[1] = 0;
	Jerror[2] = 0;
	Jerror[3] = 0;
	Jerror[4] = 0;
	Jerror[5] = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		J[0] = ((uint16_t)UartData.CurrentPosJ1H.value << 8) | UartData.CurrentPosJ1L.value; 
		J[1] = ((uint16_t)UartData.CurrentPosJ2H.value << 8) | UartData.CurrentPosJ2L.value; 
		J[2] = ((uint16_t)UartData.CurrentPosJ3H.value << 8) | UartData.CurrentPosJ3L.value; 
		J[3] = ((uint16_t)UartData.CurrentPosJ4H.value << 8) | UartData.CurrentPosJ4L.value; 
		J[4] = ((uint16_t)UartData.CurrentPosJ5H.value << 8) | UartData.CurrentPosJ5L.value; 
		J[5] = ((uint16_t)UartData.CurrentPosJ6H.value << 8) | UartData.CurrentPosJ6L.value; 

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	switch (run_mode){
		case 1:
		UartData.GameState.value = 1;
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
		spitxbuffer[0] = 0xff;
		spitxbuffer[1] = 0xff;
		spitxbuffer[2] = 0x01;
		spitxbuffer[3] = 0x05;
		spitxbuffer[4] = 0x02;
		spitxbuffer[5] = 0x00;
		spitxbuffer[6] = 0x00;
		spitxbuffer[7] = 0x00;
		spitxbuffer[8] = 0xf7;
		HAL_GPIO_TogglePin(LED_B_2_GPIO_Port, LED_B_2_Pin);
	////////////////1/////////////////
		for (int i = 0;i<2;i++){
		n = 1;
			while(n == 1){
				SpiEn1;
				if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
					SpiDis1;
					n = 0;
				}
		}
		if(checksum(spiRxBuffer.Buffer)){
			for (int n = 0; n < 9 ;n++){
					spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
			}
			UartData.CurrentPosJ1L.value = spiRxBuffer.RxData[5];
			UartData.CurrentPosJ1H.value = spiRxBuffer.RxData[6];
			Jerror[0] = spiRxBuffer.RxData[7];
		}
			HAL_Delay(10);
		}
		
		
		////////////////2/////////////////
		for (int i = 0;i<2;i++){
		n = 1;
			while(n == 1){
				SpiEn2;
				if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
					SpiDis2;
					n = 0;
				}
		}
		if(checksum(spiRxBuffer.Buffer)){
			for (int n = 0; n < 9 ;n++){
				spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
			}
			UartData.CurrentPosJ2L.value = spiRxBuffer.RxData[5];
			UartData.CurrentPosJ2H.value = spiRxBuffer.RxData[6];
			Jerror[1] = spiRxBuffer.RxData[7];
		}
			HAL_Delay(10);
		}
		
		
		
		//////////////////3////////////////
		for (int i = 0;i<2;i++){
		n = 1;
			while(n == 1){
				SpiEn3;
				if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
					SpiDis3;
					n = 0;
				}
		}
		if(checksum(spiRxBuffer.Buffer)){
			for (int n = 0; n < 9 ;n++){
					spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
			}
			UartData.CurrentPosJ3L.value = spiRxBuffer.RxData[5];
			UartData.CurrentPosJ3H.value = spiRxBuffer.RxData[6];
			Jerror[2] = spiRxBuffer.RxData[7];
		}
			HAL_Delay(10);
		}
		
		
		
		/////////////////4////////////////
		for (int i = 0;i<2;i++){
		n = 1;
			while(n == 1){
				SpiEn4;
				if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
					SpiDis4;
					n = 0;
				}
		}
		if(checksum(spiRxBuffer.Buffer)){
			for (int n = 0; n < 9 ;n++){
					spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
			}
			UartData.CurrentPosJ4L.value = spiRxBuffer.RxData[5];
			UartData.CurrentPosJ4H.value = spiRxBuffer.RxData[6];
			Jerror[3] = spiRxBuffer.RxData[7];
		}
			HAL_Delay(10);
		}
		
		
		
		//////////////////5/////////////////////
		for (int i = 0;i<2;i++){
		n = 1;
			while(n == 1){
				SpiEn5;
				if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
					SpiDis5;
					n = 0;
				}
		}
		if(checksum(spiRxBuffer.Buffer)){
			for (int n = 0; n < 9 ;n++){
					spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
			}
			UartData.CurrentPosJ5L.value = spiRxBuffer.RxData[5];
			UartData.CurrentPosJ5H.value = spiRxBuffer.RxData[6];
			Jerror[4] = spiRxBuffer.RxData[7];
		}
			HAL_Delay(10);
		}
		
		
		
		///////////////////6////////////////////
		for (int i = 0;i<2;i++){
		n = 1;
			while(n == 1){
				SpiEn6;
				if(HAL_SPI_TransmitReceive(&hspi1,spitxbuffer,spiRxBuffer.Buffer,9,HAL_MAX_DELAY) == HAL_OK){
					SpiDis6;
					n = 0;
				}
		}
		if(checksum(spiRxBuffer.Buffer)){
			for (int n = 0; n < 9 ;n++){
					spiRxBuffer.RxData[n] = spiRxBuffer.Buffer[n];
			}
			UartData.CurrentPosJ6L.value = spiRxBuffer.RxData[5];
			UartData.CurrentPosJ6H.value = spiRxBuffer.RxData[6];
			if(UartData.CurrentPosJ6L.value == UartData.GoalPosJ6L.value && UartData.CurrentPosJ6H.value == UartData.GoalPosJ6H.value)Jerror[5] = 0;
			else Jerror[5] = 1;
		}
			HAL_Delay(10);
		}
		
		
		if(Jerror[0] == 0 && Jerror[1] == 0 && Jerror[2] == 0 && Jerror[3] == 0 && Jerror[4] == 0 && Jerror[5] == 0){UartData.AllJComplete.value = 0x01;}
		else{UartData.AllJComplete.value = 0x00;}
		
		
		break;
		case 0:
			n = 0;
		UartData.GameState.value = 0;
		UartData.Gripper_Status.value = 0;
		//HAL_UART_Transmit(&huart3,Uart_tx,sizeof(Uart_tx),10);
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Relay1_GPIO_Port,Relay1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay2_GPIO_Port,Relay2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay3_GPIO_Port,Relay3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay4_GPIO_Port,Relay4_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay5_GPIO_Port,Relay5_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay6_GPIO_Port,Relay6_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay7_GPIO_Port,Relay7_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay8_GPIO_Port,Relay8_Pin,GPIO_PIN_SET);
		HAL_Init();
		variableInit();
		break;
		default:
			break;
	 }
	HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 600;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 119999;
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

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Relay4_Pin|Select5_Pin|Select6_Pin|Relay1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Relay3_GPIO_Port, Relay3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay2_Pin|Relay8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Select3_Pin|Select4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin|LED_B_3_Pin|LED2_Pin 
                          |LED8_Pin|LED7_Pin|LED_B_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Relay5_Pin|Relay6_Pin|Relay7_Pin|Select1_Pin 
                          |Select2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Relay4_Pin Select5_Pin Select6_Pin Relay1_Pin */
  GPIO_InitStruct.Pin = Relay4_Pin|Select5_Pin|Select6_Pin|Relay1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Relay3_Pin */
  GPIO_InitStruct.Pin = Relay3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Relay3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay2_Pin Relay8_Pin LED_B_3_Pin LED_B_2_Pin */
  GPIO_InitStruct.Pin = Relay2_Pin|Relay8_Pin|LED_B_3_Pin|LED_B_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Select3_Pin Select4_Pin */
  GPIO_InitStruct.Pin = Select3_Pin|Select4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_Pin */
  GPIO_InitStruct.Pin = Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin LED2_Pin LED8_Pin 
                           LED7_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|LED2_Pin|LED8_Pin 
                          |LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay5_Pin Relay6_Pin Relay7_Pin Select1_Pin 
                           Select2_Pin */
  GPIO_InitStruct.Pin = Relay5_Pin|Relay6_Pin|Relay7_Pin|Select1_Pin 
                          |Select2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED6_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED5_Pin */
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
  UNUSED(huart);
		if(huart->Instance == huart3.Instance){
			HAL_UART_Receive_IT(&huart3,&UartRxBuffer.rxBuff,1);
			BufferSerial(&UartRxBuffer);
		}
}
_Bool checksum(uint8_t *package){
	uint8_t checksum = 0;
	uint8_t lenght = 0;
	lenght = package[3];
	for (int i = 2;i<lenght+4;i++){
			checksum += package[i];
	}
	checksum = (~checksum) & 0x00ff;
	if(checksum == package[lenght+4]){
		return 1;
	}
	else{
		return 0;
	}
}
void BufferSerial(RxStruct *RxBuffer){
		switch(Framestate){
			case FRAME_WAIT:
				if (RxBuffer->rxBuff == 0xff){
					RxBuffer->Buffer[RxBuffer->BufferIndex] = RxBuffer->rxBuff; 
					if(RxBuffer->BufferIndex == 1){
						Framestate = FRAME_PROGRESS;
					}
					RxBuffer->BufferIndex++;
				}else{
					RxBuffer->checksum = 0;
					RxBuffer->BufferIndex = 0;
					RxBuffer->EndLenght = NULL;
					ClearBuffer(&UartRxBuffer);
					Framestate = FRAME_WAIT;
				}
				break;
			case FRAME_PROGRESS:
					if(RxBuffer->BufferIndex == RxBuffer->EndLenght){
						RxBuffer->Buffer[RxBuffer->BufferIndex] = RxBuffer->rxBuff;
						RxBuffer->Lenght = RxBuffer->Buffer[3];
						for (int i = 2 ; i< RxBuffer->Lenght+3; i++){ 
							RxBuffer->checksum += RxBuffer->Buffer[i];
						}
						RxBuffer->checksum = (~RxBuffer->checksum) & 0x00ff;
						if(RxBuffer->checksum != RxBuffer->Buffer[RxBuffer->BufferIndex]){
							RxBuffer->checksum = 0;
							RxBuffer->BufferIndex = 0;
							RxBuffer->EndLenght = NULL;
							ClearBuffer(&UartRxBuffer);
							Framestate = FRAME_WAIT;
						}
						else{
							
							for (int n = 0; n <= RxBuffer->EndLenght;n++){
								RxBuffer->RxData[n] = RxBuffer->Buffer[n];
							}
							CheckPackage(&UartData,RxBuffer->RxData);
							RxBuffer->checksum = 0;
							RxBuffer->BufferIndex++;
							RxBuffer->Buffer[RxBuffer->BufferIndex] = 0;
							RxBuffer->BufferIndex = 0;
							RxBuffer->EndLenght = NULL;
							ClearBuffer(&UartRxBuffer);
							Framestate = FRAME_WAIT;
						}
					}
					else{
						RxBuffer->Buffer[RxBuffer->BufferIndex] = RxBuffer->rxBuff; 
						if(RxBuffer->BufferIndex == 3){
						  RxBuffer->EndLenght = RxBuffer->BufferIndex + RxBuffer->Buffer[RxBuffer->BufferIndex];
						}
						RxBuffer->BufferIndex++;
					}
				break;
			default:
				break;
		}
		
		
}

void ClearBuffer(RxStruct *RxBuffer){
	for (int i=0;i < sizeof(RxBuffer->Buffer);i++){
		RxBuffer->Buffer[i] = 0;
	}
}

void variableInit(){
	ClearBuffer(&UartRxBuffer);
	Framestate = FRAME_WAIT;
	UartRxBuffer.checksum = 0;
	UartRxBuffer.BufferIndex = 0;
	UartRxBuffer.EndLenght = NULL;
	run_mode = 0; 
	
	/////Config Adress Serial UART
  UartData.GoalPosJ1L.Address = GoalPosJ1LAddress;
	UartData.GoalPosJ1H.Address = GoalPosJ1HAddress;
	UartData.GoalPosJ2L.Address = GoalPosJ2LAddress;
	UartData.GoalPosJ2H.Address = GoalPosJ2HAddress;
	UartData.GoalPosJ3L.Address = GoalPosJ3LAddress;
	UartData.GoalPosJ3H.Address = GoalPosJ3HAddress;
	UartData.GoalPosJ4L.Address = GoalPosJ4LAddress;
	UartData.GoalPosJ4H.Address = GoalPosJ4HAddress;
	UartData.GoalPosJ5L.Address = GoalPosJ5LAddress;
	UartData.GoalPosJ5H.Address = GoalPosJ5HAddress;
	UartData.GoalPosJ6L.Address = GoalPosJ6LAddress;
	UartData.GoalPosJ6H.Address = GoalPosJ6HAddress;
	
	UartData.CurrentPosJ1L.Address = CurrentPosJ1LAddress;
	UartData.CurrentPosJ1H.Address = CurrentPosJ1HAddress;
	UartData.CurrentPosJ2L.Address = CurrentPosJ2LAddress;
	UartData.CurrentPosJ2H.Address = CurrentPosJ2HAddress;
	UartData.CurrentPosJ3L.Address = CurrentPosJ3LAddress;
	UartData.CurrentPosJ3H.Address = CurrentPosJ3HAddress;
	UartData.CurrentPosJ4L.Address = CurrentPosJ4LAddress;
	UartData.CurrentPosJ4H.Address = CurrentPosJ4HAddress;
	UartData.CurrentPosJ5L.Address = CurrentPosJ5LAddress;
	UartData.CurrentPosJ5H.Address = CurrentPosJ5HAddress;
	UartData.CurrentPosJ6L.Address = CurrentPosJ6LAddress;
	UartData.CurrentPosJ6H.Address = CurrentPosJ6HAddress;
	
	UartData.GoalPosJ1L.value = 0x00;
	UartData.GoalPosJ1H.value = 0x00;
	UartData.GoalPosJ2L.value = 0x00;
	UartData.GoalPosJ2H.value = 0x00;
	UartData.GoalPosJ3L.value = 0x00;
	UartData.GoalPosJ3H.value = 0x00;
	UartData.GoalPosJ4L.value = 0x00;
	UartData.GoalPosJ4H.value = 0x00;
	UartData.GoalPosJ5L.value = 0x00;
	UartData.GoalPosJ5H.value = 0x00;
	UartData.GoalPosJ6L.value = 0x00;
	UartData.GoalPosJ6H.value = 0x00;
	
	UartData.CurrentPosJ1L.value = 0x00;
	UartData.CurrentPosJ1H.value = 0x00;
	UartData.CurrentPosJ2L.value = 0x00;
	UartData.CurrentPosJ2H.value = 0x00;
	UartData.CurrentPosJ3L.value = 0x00;
	UartData.CurrentPosJ3H.value = 0x00;
	UartData.CurrentPosJ4L.value = 0x00;
	UartData.CurrentPosJ4H.value = 0x00;
	UartData.CurrentPosJ5L.value = 0x00;
	UartData.CurrentPosJ5H.value = 0x00;
	UartData.CurrentPosJ6L.value = 0x00;
	UartData.CurrentPosJ6H.value = 0x00;
	
	UartData.AllJComplete.Address  = AllJCompleteAddress;
	UartData.Gripper_Status.Address= Gripper_StatusAddress;
	UartData.GameState.Address     = GameStateAddress;

	//JointLimit 
	MaxLimitJ[0] = 180;
	MaxLimitJ[1] = 700;
	MaxLimitJ[2] = 300;
	MaxLimitJ[3] = 300;
	MaxLimitJ[4] = 90;
	MaxLimitJ[5] = 90;
}


void CheckPackage(DataPackageStruct *Data,uint8_t *RxData){
	Data->ID = RxData[2];
	Data->Lenght = RxData[3];
	Data->Inst = RxData[4];
	uint8_t Lenght;
	uint8_t ParameterTx[50];
	uint8_t j_eiei = 0x56;
	switch(Data->Inst){
		case 0x01: //ping
			Lenght = 2;
			ParameterTx[0] = UartData.Pingerror;
			CreatePackage(Uarttxbuffer,Data->ID,Lenght,ParameterTx);
			HAL_UART_Transmit_IT(&huart3,Uarttxbuffer,Lenght+4);
			break;
		case 0x02: //read
			Data->Address = RxData[5];
			Data->NumberOfRead = RxData[6];
			Data -> Parameter[0] = 0x00;
			Lenght = Data -> NumberOfRead + 2;
			for(uint8_t i=1 ; i <= Data->NumberOfRead ;i++){
				Data->Parameter[i] = InsRead(Data->Address + i-1);
			}
			CreatePackage(Uarttxbuffer,Data->ID,Lenght,Data->Parameter);
			HAL_UART_Transmit_IT(&huart3,Uarttxbuffer,Lenght+4);
			break;
		case 0x03: //write
			Data->Address = RxData[5];
			Data->NumberOfWrite = RxData[3]-3;
			Lenght = RxData[3];
			uint8_t error = 0;
			for(uint8_t i=0 ; i < Data->NumberOfWrite ;i++){
				Data->Parameter[i] = RxData[6+i];
			}
			for( uint8_t i=0 ; i< Data->NumberOfWrite ;i++){
				if( Data->Address == 0x00 ||  Data->Address == 0x02 ||  Data->Address == 0x04 ||  Data->Address == 0x06 ||  Data->Address == 0x08 ||  Data->Address == 0x0A){
					error = InsWrite(Data->Address,&Data->Parameter[i]);
					i++;
					Data->Address += 2 ;
				}else error = InsWrite(Data->Address,&Data->Parameter[i]);
			}
			Lenght = 2;
			ParameterTx[0] = error;
			CreatePackage(Uarttxbuffer,Data->ID,Lenght,ParameterTx);
			HAL_UART_Transmit_IT(&huart3,Uarttxbuffer,Lenght+4);
			break;
		case 0x04:
			break;
		case 0x05:
			break;
		default:
			break;
	}
	
	
}

void CreatePackage(uint8_t *Tx,uint8_t ID,uint8_t Lenght,uint8_t *Data){
	
	uint8_t checksum = 0;
	for (int i = 0 ; i<2;i++){Tx[i] = 0xff;}
	Tx[2] = ID;
	Tx[3] = Lenght;
	for (int i = 1;i<Lenght;i++){
		Tx[i+3] = Data[i-1];
	}
	for (int i = 2 ; i< Lenght+3; i++){ 
		checksum += Tx[i];
	}
	checksum = (~checksum) & 0x00ff;
	Tx[Lenght+3] = checksum;

}

void Vaccumm(int Switch){
	if(Switch == 1){
		UartData.Gripper_Status.value = 1;
		HAL_GPIO_WritePin(Relay6_GPIO_Port,Relay6_Pin,GPIO_PIN_RESET);
	}
	else{
		UartData.Gripper_Status.value = 0;
		HAL_GPIO_WritePin(Relay6_GPIO_Port,Relay6_Pin,GPIO_PIN_SET);
	}
}

uint8_t InsRead(uint8_t Address){
	uint16_t Goalpos1, GoalposMap1,Goalpos2, GoalposMap2,Goalpos3, GoalposMap3,Goalpos4, GoalposMap4,Goalpos5, GoalposMap5,Goalpos6, GoalposMap6;
	uint16_t Currentpos1, CurrentposMap1,Currentpos2, CurrentposMap2,Currentpos3, CurrentposMap3,Currentpos4, CurrentposMap4,Currentpos5, CurrentposMap5,Currentpos6, CurrentposMap6;
	Goalpos1 = ((uint16_t)UartData.GoalPosJ1H.value << 8) | UartData.GoalPosJ1L.value;
	Goalpos2 = ((uint16_t)UartData.GoalPosJ2H.value << 8) | UartData.GoalPosJ2L.value;
	Goalpos3 = ((uint16_t)UartData.GoalPosJ3H.value << 8) | UartData.GoalPosJ3L.value;
	Goalpos4 = ((uint16_t)UartData.GoalPosJ4H.value << 8) | UartData.GoalPosJ4L.value;
	Goalpos5 = ((uint16_t)UartData.GoalPosJ5H.value << 8) | UartData.GoalPosJ5L.value;
	Goalpos6 = ((uint16_t)UartData.GoalPosJ6H.value << 8) | UartData.GoalPosJ6L.value;

	GoalposMap1 = (uint32_t)((Goalpos1*65535)/MaxLimitJ[0]);
	GoalposMap2 = (uint32_t)((Goalpos2*65535)/MaxLimitJ[1]);
	GoalposMap3 = (uint32_t)((Goalpos3*65535)/MaxLimitJ[2]);
	GoalposMap4 = (uint32_t)((Goalpos4*65535)/MaxLimitJ[3]);
	GoalposMap5 = (uint32_t)((Goalpos5*65535)/MaxLimitJ[4]);
	GoalposMap6 = (uint32_t)((Goalpos6*65535)/MaxLimitJ[5]);

	Currentpos1 = ((uint16_t)UartData.CurrentPosJ1H.value << 8) | UartData.CurrentPosJ1L.value;
	Currentpos2 = ((uint16_t)UartData.CurrentPosJ2H.value << 8) | UartData.CurrentPosJ2L.value;
	Currentpos3 = ((uint16_t)UartData.CurrentPosJ3H.value << 8) | UartData.CurrentPosJ3L.value;
	Currentpos4 = ((uint16_t)UartData.CurrentPosJ4H.value << 8) | UartData.CurrentPosJ4L.value;
	Currentpos5 = ((uint16_t)UartData.CurrentPosJ5H.value << 8) | UartData.CurrentPosJ5L.value;
	Currentpos6 = ((uint16_t)UartData.CurrentPosJ6H.value << 8) | UartData.CurrentPosJ6L.value;

	CurrentposMap1 = (uint32_t)((Currentpos1*65535)/MaxLimitJ[0]);
	CurrentposMap2 = (uint32_t)((Currentpos2*65535)/MaxLimitJ[1]);
	CurrentposMap3 = (uint32_t)((Currentpos3*65535)/MaxLimitJ[2]);
	CurrentposMap4 = (uint32_t)((Currentpos4*65535)/MaxLimitJ[3]);
	CurrentposMap5 = (uint32_t)((Currentpos5*65535)/MaxLimitJ[4]);
	CurrentposMap6 = (uint32_t)((Currentpos6*65535)/MaxLimitJ[5]);

	switch(Address){
		case 0x00: return (uint8_t)(GoalposMap1 & 0x00ff); break;
		case 0x01: return (uint8_t)((GoalposMap1 & 0xff00) >> 8); break;
		case 0x02: return (uint8_t)(GoalposMap2 & 0x00ff); break;
		case 0x03: return (uint8_t)((GoalposMap2 & 0xff00) >> 8); break;
		case 0x04: return (uint8_t)(GoalposMap3 & 0x00ff); break;
		case 0x05: return (uint8_t)((GoalposMap3 & 0xff00) >> 8); break;
		case 0x06: return (uint8_t)(GoalposMap4 & 0x00ff); break;
		case 0x07: return (uint8_t)((GoalposMap4 & 0xff00) >> 8); break;
		case 0x08: return (uint8_t)(GoalposMap5 & 0x00ff); break;
		case 0x09: return (uint8_t)((GoalposMap5 & 0xff00) >> 8); break;
		case 0x0A: return (uint8_t)(GoalposMap6 & 0x00ff); break;
		case 0x0B: return (uint8_t)((GoalposMap6 & 0xff00) >> 8); break;
		case 0x0C: return (uint8_t)(CurrentposMap1 & 0x00ff); break;
		case 0x0D: return (uint8_t)((CurrentposMap1 & 0xff00) >> 8); break;
		case 0x0E: return (uint8_t)(CurrentposMap2 & 0x00ff); break;
		case 0x0F: return (uint8_t)((CurrentposMap2 & 0xff00) >> 8); break;
		case 0x10: return (uint8_t)(CurrentposMap3 & 0x00ff); break;
		case 0x11: return (uint8_t)((CurrentposMap3 & 0xff00) >> 8); break;
		case 0x12: return (uint8_t)(CurrentposMap4 & 0x00ff); break;
		case 0x13: return (uint8_t)((CurrentposMap4 & 0xff00) >> 8); break;
		case 0x14: return (uint8_t)(CurrentposMap5 & 0x00ff); break;
		case 0x15: return (uint8_t)((CurrentposMap5 & 0xff00) >> 8); break;
		case 0x16: return (uint8_t)(CurrentposMap6 & 0x00ff); break;
		case 0x17: return (uint8_t)((CurrentposMap6 & 0xff00) >> 8); break;
		case 0x18: return UartData.AllJComplete.value; break;
		case 0x19: return UartData.Gripper_Status.value; break;
		case 0x1A: return UartData.GameState.value; break;
	}
}

uint8_t InsWrite(uint8_t Address,uint8_t* Data){
	uint8_t error=0,posLow,posHigh,posLowMap,posHighMap;
	uint16_t pos,posMap;
	switch(Address){
		case 0x00:
			posLow = *Data;
			Data++;
			posHigh = *Data;
			pos = ((uint16_t)posHigh << 8) | posLow;
			posMap = (uint32_t)((MaxLimitJ[0]*pos) / 65535);
			posLowMap = posMap & 0x00ff;
			posHighMap = (uint16_t)(posMap & 0xff00) >> 8 ;
			error = SetPosJoint1(posLowMap,posHighMap); break; 

		case 0x02:
			posLow = *Data;
			Data++;
			posHigh = *Data; 
			pos = ((uint16_t)posHigh << 8) | posLow;
			posMap = (uint32_t)((MaxLimitJ[1]*pos) / 65535);
			posLowMap = posMap & 0x00ff;
			posHighMap = (uint16_t)(posMap & 0xff00) >> 8 ;
			error = SetPosJoint2(posLowMap,posHighMap); break; 

		case 0x04: 
			posLow = *Data;
			Data++;
			posHigh = *Data;
			pos = ((uint16_t)posHigh << 8) | posLow;
			posMap = (uint32_t)((MaxLimitJ[2]*pos) / 65535);
			posLowMap = posMap & 0x00ff;
			posHighMap = (uint16_t)(posMap & 0xff00) >> 8 ;
			error = SetPosJoint3(posLowMap,posHighMap); break; 

		case 0x06: 
			posLow = *Data;
			Data++;
			posHigh = *Data;
			pos = ((uint16_t)posHigh << 8) | posLow;
			posMap = (uint32_t)((MaxLimitJ[3]*pos) / 65535);
			posLowMap = posMap & 0x00ff;
			posHighMap = (uint16_t)(posMap & 0xff00) >> 8 ;
			error = SetPosJoint4(posLowMap,posHighMap); break; 

		case 0x08: 
			posLow = *Data;
			Data++;
			posHigh = *Data;
			pos = ((uint16_t)posHigh << 8) | posLow;
			posMap = (uint32_t)((MaxLimitJ[4]*pos) / 65535);
			posLowMap = posMap & 0x00ff;
			posHighMap = (uint16_t)(posMap & 0xff00) >> 8 ;
			error = SetPosJoint5(posLowMap,posHighMap); break; 

		case 0x0A: 
			posLow = *Data;
			Data++;
			posHigh = *Data;
			pos = ((uint16_t)posHigh << 8) | posLow;
			posMap = (uint32_t)((MaxLimitJ[5]*pos) / 65535);
			posLowMap = posMap & 0x00ff;
			posHighMap = (uint16_t)(posMap & 0xff00) >> 8 ;
			error = SetPosJoint6(posLowMap,posHighMap); break; 

		case 0x19: error = SetGripper(*Data); break;
	}
	return error;
}

uint8_t SetPosJoint1(uint8_t posLow,uint8_t posHigh){
	UartData.GoalPosJ1H.value = posHigh;
	UartData.GoalPosJ1L.value = posLow;
	uint8_t ParameterTx[50];
	ParameterTx[0] = 0x03;
	ParameterTx[1] = posLow;
	ParameterTx[2] = posHigh;
	ParameterTx[3] = 0x00;
	CreatePackage(Spitxbuffer,0x01,0x05,ParameterTx);
	for (int i = 0;i<2;i++){
		SpiEn1;		
		HAL_SPI_Transmit(&hspi1,Spitxbuffer,9,1000);
		SpiDis1;
	}

	return 0x00;
}

uint8_t SetPosJoint2(uint8_t posLow,uint8_t posHigh){
	uint16_t PosGoal;
	uint16_t PosCurrent;
	uint8_t Dir;
	uint16_t Dis;
	UartData.GoalPosJ2H.value = posHigh;
	UartData.GoalPosJ2L.value = posLow;
	PosGoal = ((uint16_t)posHigh << 8 ) | posLow;
	PosCurrent = ((uint16_t)UartData.CurrentPosJ2H.value << 8) | UartData.CurrentPosJ2L.value; 
	if(PosCurrent != PosGoal){
		if((PosGoal - PosCurrent) < 0){
			Dir = 0x00;
			Dis = PosCurrent - PosGoal;
		}
		else{
			Dir = 0x01;
			Dis = PosGoal - PosCurrent;
		}
	}
	else{
		Dir = 0x02;
	}

	uint8_t ParameterTx[50];
	ParameterTx[0] = 0x03;
	ParameterTx[1] = Dis & 0x00ff; //0xc8
	ParameterTx[2] = (Dis & 0xff00) >> 8;
	ParameterTx[3] = Dir;
	CreatePackage(Spitxbuffer,0x01,0x05,(uint8_t *)ParameterTx);
	
	for (int i = 0;i<2;i++){
		SpiEn2;		
		HAL_SPI_Transmit(&hspi1,Spitxbuffer,9,1000);
		SpiDis2;
	}
	return 0x00;
}

uint8_t SetPosJoint3(uint8_t posLow,uint8_t posHigh){
	uint16_t PosGoal;
	uint16_t PosCurrent;
	uint8_t Dir;
	uint16_t Dis;
	UartData.GoalPosJ3H.value = posHigh;
	UartData.GoalPosJ3L.value = posLow;
	PosGoal = ((uint16_t)posHigh << 8 ) | posLow;
	PosCurrent = ((uint16_t)UartData.CurrentPosJ3H.value << 8) | UartData.CurrentPosJ3L.value; 
	if(PosCurrent != PosGoal){
		if((PosGoal - PosCurrent) < 0){
			Dir = 0x00;
			Dis = PosCurrent - PosGoal;
		}
		else{
			Dir = 0x01;
			Dis = PosGoal - PosCurrent;
		}
	}
	else{
		Dir = 0x02;
	}

	uint8_t ParameterTx[50];
	ParameterTx[0] = 0x03;
	ParameterTx[1] = Dis & 0x00ff; //0xc8
	ParameterTx[2] = (Dis & 0xff00) >> 8;
	ParameterTx[3] = Dir;
	CreatePackage(Spitxbuffer,0x01,0x05,(uint8_t *)ParameterTx);
	
	for (int i = 0;i<2;i++){
		SpiEn3;		
		HAL_SPI_Transmit(&hspi1,Spitxbuffer,9,1000);
		SpiDis3;
	}
	return 0x00;
}

uint8_t SetPosJoint4(uint8_t posLow,uint8_t posHigh){
	uint16_t PosGoal;
	uint16_t PosCurrent;
	uint8_t Dir;
	uint16_t Dis;
	UartData.GoalPosJ4H.value = posHigh;
	UartData.GoalPosJ4L.value = posLow;
	PosGoal = ((uint16_t)posHigh << 8 ) | posLow;
	PosCurrent = ((uint16_t)UartData.CurrentPosJ4H.value << 8) | UartData.CurrentPosJ4L.value; 
	if(PosCurrent != PosGoal){
		if((PosGoal - PosCurrent) < 0){
			Dir = 0x00;
			Dis = PosCurrent - PosGoal;
		}
		else{
			Dir = 0x01;
			Dis = PosGoal - PosCurrent;
		}
	}
	else{
		Dir = 0x02;
	}

	uint8_t ParameterTx[50];
	ParameterTx[0] = 0x03;
	ParameterTx[1] = Dis & 0x00ff; //0xc8
	ParameterTx[2] = (Dis & 0xff00) >> 8;
	ParameterTx[3] = Dir;
	CreatePackage(Spitxbuffer,0x01,0x05,(uint8_t *)ParameterTx);
	
	for (int i = 0;i<2;i++){
		SpiEn4;		
		HAL_SPI_Transmit(&hspi1,Spitxbuffer,9,1000);
		SpiDis4;
	}
	return 0x00;
}

uint8_t SetPosJoint5(uint8_t posLow,uint8_t posHigh){
	uint16_t PosGoal;
	uint16_t PosCurrent;
	uint8_t Dir;
	uint16_t Dis;
	UartData.GoalPosJ5H.value = posHigh;
	UartData.GoalPosJ5L.value = posLow;
	PosGoal = ((uint16_t)posHigh << 8 ) | posLow;
	PosCurrent = ((uint16_t)UartData.CurrentPosJ5H.value << 8) | UartData.CurrentPosJ5L.value; 
	if(PosCurrent != PosGoal){
		if((PosGoal - PosCurrent) < 0){
			Dir = 0x00;
			Dis = PosCurrent - PosGoal;
		}
		else{
			Dir = 0x01;
			Dis = PosGoal - PosCurrent;
		}
	}
	else{
		Dir = 0x02;
	}

	uint8_t ParameterTx[50];
	ParameterTx[0] = 0x03;
	ParameterTx[1] = Dis & 0x00ff; //0xc8
	ParameterTx[2] = (Dis & 0xff00) >> 8;
	ParameterTx[3] = Dir;
	CreatePackage(Spitxbuffer,0x01,0x05,(uint8_t *)ParameterTx);
	
	for (int i = 0;i<2;i++){
		SpiEn5;		
		HAL_SPI_Transmit(&hspi1,Spitxbuffer,9,1000);
		SpiDis5;
	}
	return 0x00;
}

uint8_t SetPosJoint6(uint8_t posLow,uint8_t posHigh){
		UartData.GoalPosJ6H.value = posHigh;
	UartData.GoalPosJ6L.value = posLow;
		uint8_t ParameterTx[50];
	ParameterTx[0] = 0x03;
	ParameterTx[1] = posLow;
	ParameterTx[2] = posHigh;
	ParameterTx[3] = 0x00;
	CreatePackage(Spitxbuffer,0x01,0x05,ParameterTx);
	for (int i = 0;i<2;i++){
		SpiEn6;		
		HAL_SPI_Transmit(&hspi1,Spitxbuffer,9,1000);
		SpiDis6;
	}

	return 0x00;
}
uint8_t SetGripper(uint8_t cmd){
	Vaccumm(cmd);
	return 0x00;
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
