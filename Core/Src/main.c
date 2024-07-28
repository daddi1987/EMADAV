/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CR_SIZE 4
#define BUFFER_SIZE 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId AmplifierTaskHandle;
osThreadId SerialTaskHandle;
/* USER CODE BEGIN PV */

bool StateAmplifier = false;
bool OldStateAmplifier = false;
bool MuteButtonState = false;
uint8_t HEADER1[40] = {'\0'};
char CR[CR_SIZE];

uint8_t Rx_data[1];
uint8_t Rx_buffer[BUFFER_SIZE];
uint16_t Rx_index = 0;
uint8_t SerialCommand = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartAmplifierTask(void const * argument);
void StartSerialTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf(HEADER1, "Initialized USB Serial Comunication \n");
  HAL_UART_Transmit(&huart1, HEADER1, sizeof(HEADER1), 38);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of AmplifierTask */
  osThreadDef(AmplifierTask, StartAmplifierTask, osPriorityNormal, 0, 128);
  AmplifierTaskHandle = osThreadCreate(osThread(AmplifierTask), NULL);

  /* definition and creation of SerialTask */
  osThreadDef(SerialTask, StartSerialTask, osPriorityNormal, 0, 128);
  SerialTaskHandle = osThreadCreate(osThread(SerialTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STANDBY__AMPLIFIER_GPIO_Port, STANDBY__AMPLIFIER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUTE_AMPLIFIER_GPIO_Port, MUTE_AMPLIFIER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : STANDBY__AMPLIFIER_Pin MUTE_AMPLIFIER_Pin */
  GPIO_InitStruct.Pin = STANDBY__AMPLIFIER_Pin|MUTE_AMPLIFIER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUTE_BUTTON_Pin */
  GPIO_InitStruct.Pin = MUTE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUTE_BUTTON_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart1, Rx_data, 1);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAmplifierTask */
/**
* @brief Function implementing the AmplifierTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAmplifierTask */
void StartAmplifierTask(void const * argument)
{
  /* USER CODE BEGIN StartAmplifierTask */
  /* Infinite loop */
  for(;;)
  {
	osDelay(1);
	MuteButtonState = HAL_GPIO_ReadPin(MUTE_BUTTON_GPIO_Port, MUTE_BUTTON_Pin); // GET STATE MUTE BUTTON 'CURRENT STATE IS NEGETIVE'
	MuteButtonState = !MuteButtonState; 	// REVERSE NEGATIVE TO POSITIVE SIGNAL

	if ((MuteButtonState == true && StateAmplifier == false)||(SerialCommand == 1 ))	//CHECK STATUS BUTTON
	{
		StateAmplifier = true;
		HAL_GPIO_WritePin(STANDBY__AMPLIFIER_GPIO_Port, STANDBY__AMPLIFIER_Pin, GPIO_PIN_SET);
		osDelay(200);
		HAL_GPIO_WritePin(MUTE_AMPLIFIER_GPIO_Port, MUTE_AMPLIFIER_Pin, GPIO_PIN_RESET);
		MuteButtonState = false;
		SerialCommand = 0;
		osDelay(1000);
	}
	if ((MuteButtonState == true && StateAmplifier == true)||(SerialCommand == 2 ))	//CHECK STATUS BUTTON
	{
		StateAmplifier = false;
		HAL_GPIO_WritePin(MUTE_AMPLIFIER_GPIO_Port, MUTE_AMPLIFIER_Pin, GPIO_PIN_SET);
		osDelay(200);
		HAL_GPIO_WritePin(STANDBY__AMPLIFIER_GPIO_Port, STANDBY__AMPLIFIER_Pin, GPIO_PIN_RESET);
		MuteButtonState = false;
		SerialCommand = 0;
		osDelay(1000);
	}

   }

  /* USER CODE END StartAmplifierTask */
}

/* USER CODE BEGIN Header_StartSerialTask */
/**
* @brief Function implementing the SerialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialTask */
void StartSerialTask(void const * argument)
{
  /* USER CODE BEGIN StartSerialTask */
  /* Infinite loop */
  for(;;)
  {
	//--------------------FOR SERIAL COMUNICATION---------------------------------------------
	osDelay(1);
	//----------------------------TX COMUNICATION---------------------------------------------
	MuteButtonState = HAL_GPIO_ReadPin(MUTE_BUTTON_GPIO_Port, MUTE_BUTTON_Pin); // GET STATE MUTE BUTTON 'CURRENT STATE IS NEGETIVE'
	MuteButtonState = !MuteButtonState; 	// REVERSE NEGATIVE TO POSITIVE SIGNAL

	if ((MuteButtonState == true)||(SerialCommand != 0))
	{
		if ((StateAmplifier == true)&&(StateAmplifier != OldStateAmplifier))
		{
			  sprintf(HEADER1, "Px_AMPLIFIER ON_Sx");
			  sprintf(CR, "\r\n");
			  HAL_UART_Transmit(&huart1, HEADER1, sizeof(HEADER1), 18);
			  HAL_UART_Transmit(&huart1, (uint8_t *)CR, sizeof(CR), 0xFFFF);
			  OldStateAmplifier = StateAmplifier;
			  SerialCommand = 0;
		}
		else if ((StateAmplifier == false)&&(StateAmplifier != OldStateAmplifier))
		{
			  sprintf(HEADER1, "Px_AMPLIFIER OFF_Sx");
			  sprintf(CR, "\r\n");
			  HAL_UART_Transmit(&huart1, HEADER1, sizeof(HEADER1), 18);
			  HAL_UART_Transmit(&huart1, (uint8_t *)CR, sizeof(CR), 0xFFFF);
			  OldStateAmplifier = StateAmplifier;
			  SerialCommand = 0;
		}
	}
	//----------------------------RX COMUNICATION---------------------------------------------
	HAL_UART_Receive_IT(&huart1, Rx_data, 1);
	if (Rx_data[0] == 49) // COMMAND 1
	{
		sprintf(HEADER1, "Px_Command ON Amplifier Received_Sx");
		sprintf(CR, "\r\n");
		HAL_UART_Transmit(&huart1, HEADER1, sizeof(HEADER1), 35);
		HAL_UART_Transmit(&huart1, (uint8_t *)CR, sizeof(CR), 0xFFFF);
		SerialCommand = 1; 			//COMMAND START AMPIFIER AND SOUND
		Rx_data[0] = 0;
	}
	else if ((Rx_data[0] == 50))    // COMMAND 2
	{
		sprintf(HEADER1, "Px_Command OFF Amplifier Received_Sx");
		sprintf(CR, "\r\n");
		HAL_UART_Transmit(&huart1, HEADER1, sizeof(HEADER1), 36);
		HAL_UART_Transmit(&huart1, (uint8_t *)CR, sizeof(CR), 0xFFFF);
		SerialCommand = 2;		   //COMMAND STOP AMPIFIER AND SOUND
		Rx_data[0] = 0;
	}

  }
  /* USER CODE END StartSerialTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
