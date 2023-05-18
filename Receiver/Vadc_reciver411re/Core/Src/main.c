/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RxBuf_SIZE 60
#define Rx_data_size 24
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
char RxBuffer[RxBuf_SIZE] = { 0 };
char TxBuffer[RxBuf_SIZE] = { 0 };
int counter = 0, Vadc_counter = 0;
char Vadc1[8];
char Vadc2[8];
char Vadc3[8];
char vadc1[6] = "Vadc1:";
char vadc2[6] = "Vadc2:";
char vadc3[6] = "Vadc3:";
char test_value[50] = { "kbdA1.245ZB3.453Zsdfdsfas" };
float Vadc1_float_value = 0;
float Vadc2_float_value = 0;
float Vadc3_float_value = 0;
int cnt = 0;
char Rxcuffer[1];
int Counter_Dma = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
		if (Counter_Dma == 0) {
			RxBuffer[cnt++] = Rxcuffer[0];


			if (cnt > 23) {
				Counter_Dma = 1;
				cnt = 0;
				//HAL_UART_Transmit(&huart2, RxBuffer,sizeof(RxBuffer),100);
				//
			}

		}
		HAL_UART_Receive_DMA(&huart1,  Rxcuffer, 1);

	}

}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1,  Rxcuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (Counter_Dma == 1) {
			for (counter; counter < 60; counter++) {
				if (RxBuffer[counter] == 'A') {
					counter++;
					for ( counter; RxBuffer[counter] != 'Z'; counter++) {
						Vadc1[Vadc_counter++] = RxBuffer[counter];
					}
				}

				if (RxBuffer[counter] == 'B') {
					counter++;
					Vadc_counter = 0;
					for (counter; RxBuffer[counter] != 'Z'; counter++) {
						Vadc2[Vadc_counter++] = RxBuffer[counter];
					}

					Vadc_counter = 0;
					//Counter_Dma = 0;
				}
				if (RxBuffer[counter] == 'C') {
					counter++;
					Vadc_counter = 0;
					for (counter; RxBuffer[counter] != 'Z'; counter++) {
						Vadc3[Vadc_counter++] = RxBuffer[counter];
					}
					Vadc_counter = 0;
				}
			}
			sscanf(Vadc1, "%f", &Vadc1_float_value);
			sscanf(Vadc2, "%f", &Vadc2_float_value);
			sscanf(Vadc3, "%f", &Vadc3_float_value);


			if (counter == 60) {
				counter = 0;

			}
			Counter_Dma = 0;


			if (Vadc1_float_value < 0.50) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			}

			if (Vadc1_float_value > 0.50 && Vadc1_float_value < 2) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			}

			if (Vadc1_float_value > 2 && Vadc1_float_value < 3) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			}

			if (Vadc1_float_value > 3) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
			}

			if (Vadc2_float_value < 0.50) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
			}

			if (Vadc2_float_value > 0.50 && Vadc2_float_value < 2) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
			}

			if (Vadc2_float_value > 2 && Vadc2_float_value < 3) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
			}

			if (Vadc2_float_value > 3) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);
			}


			if (Vadc3_float_value < 0.50) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
			}

			if (Vadc3_float_value > 0.50 && Vadc3_float_value < 2) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
			}

			if (Vadc3_float_value > 2 && Vadc3_float_value < 3) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
			}

			if (Vadc3_float_value > 3) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
			}



		}

		/*
		 HAL_Delay(500);*/

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
