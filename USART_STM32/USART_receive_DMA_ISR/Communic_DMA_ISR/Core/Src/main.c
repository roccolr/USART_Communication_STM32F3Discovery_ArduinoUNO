/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_fft.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_FINESTRE 105
#define N_CAMPIONI 128
#define LED_1 GPIO_PIN_9
#define LED_2 GPIO_PIN_8
#define LED_3 GPIO_PIN_15
#define LED_4 GPIO_PIN_14
#define LED_5 GPIO_PIN_13
#define EMB_LEDS GPIOE
#define COM_PINS GPIOB
#define REQ GPIO_PIN_0
#define ACK GPIO_PIN_1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  uint8_t signal[N_CAMPIONI*N_FINESTRE];
  uint8_t metadata[8];
  int32_t massimo = 0x00000000;
  int32_t minimo = 0x00000000;
  uint8_t indici[N_CAMPIONI];
  volatile uint8_t trasmissione_completata = 1;
  volatile uint8_t metadati_ricevuti = 0;
  volatile uint8_t dati_ricevuti = 0;
  volatile uint8_t fine = 0;
  volatile uint8_t finestra_corrente = 0;
  volatile uint8_t finestra_calcolata = 0;
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
  MX_USB_PCD_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  // LEGENDA PIN:
  // GPIOC10 -> UART TX
  // GPIOC11 -> UART RX
  // GPIOB0  -> REQ
  // GPIOB1  -> ACK  -> scatena interrupt
  // GPIOE9  -> LED 1 Rosso
  // GPIOE8  -> LED 2 Blue
  // GPIOE15 -> LED 3 Green
  // GPIOE14 -> Led 4 Orange
  // GPIOE13 -> Led 5 Red


  HAL_UART_Receive_DMA(&huart4, metadata, 8);
  while(metadati_ricevuti == 0){
  }
  HAL_UART_Receive_DMA(&huart4, signal, N_FINESTRE*N_CAMPIONI);
  while(dati_ricevuti == 0){
  }

  // calcolo massimo e minimo
  massimo = (int32_t)(metadata[0] | metadata[1]<<8 | metadata[2]<<16 | metadata[3]<<24);
  minimo = (int32_t)(metadata[4] | metadata[5]<<8 | metadata[6]<<16 | metadata[7]<<24);


  for(int i=0; i<N_FINESTRE; i++){
	  HAL_GPIO_WritePin(EMB_LEDS, LED_3, GPIO_PIN_SET); //inizio elaborazione finestra i-esima


	  // CALCOLI FFT
	  Complesso x[N_CAMPIONI];
	  Complesso X[N_CAMPIONI];
	  float magnitude[N_CAMPIONI];
	  for(int j=0; j<N_CAMPIONI; j++){
		  x[j].re = ((float)signal[i*N_CAMPIONI + j]) / 255.0f * (massimo - minimo) + minimo;
		  x[j].im = 0.0f;
	  }
	  arm_fft(x,X,N_CAMPIONI);
	  fft_to_spectrogram(X, magnitude, N_CAMPIONI, -65.0f);	// [0,1]
	  for(int l=0; l<N_CAMPIONI; l++){
		  signal[i*N_CAMPIONI+l] = (uint8_t)(magnitude[l]*255);
	  }
	  finestra_calcolata++;
	  if(trasmissione_completata == 1){
		  HAL_GPIO_WritePin(COM_PINS, REQ, GPIO_PIN_SET); // alzo la richiesta
		  trasmissione_completata = 0;
	  }
	  HAL_GPIO_WritePin(EMB_LEDS, LED_3, GPIO_PIN_RESET); //fine elaborazione finestra i-esima
//	  HAL_Delay(1000);
  }
  fine = 1;

//   DATA READY CAUSA SUL MASTER UN'INTERRUZIONE CHE TRIGGERA IL SEGNALE ACK E FA IN MODO CHE LA COMUNICAZIONE SIA
//   SINCRONA E ORDINATA

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(EMB_LEDS, LED_5, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(EMB_LEDS, LED_5, GPIO_PIN_RESET);
	  HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_UART4;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT4_Pin MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin PE8 PE9 PE13
                           PE14 PE15 */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(metadati_ricevuti == 0){
		metadati_ricevuti = 1;
		HAL_GPIO_WritePin(EMB_LEDS, LED_1, GPIO_PIN_SET);
	}
	else{
		dati_ricevuti = 1;
		HAL_GPIO_WritePin(EMB_LEDS, LED_2, GPIO_PIN_SET);
	    if (HAL_UART_DeInit(huart) != HAL_OK) {
	        Error_Handler();
	    }
	    if (HAL_UART_Init(huart) != HAL_OK) {
	        Error_Handler(); 	    }
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(finestra_corrente < finestra_calcolata){
//		HAL_GPIO_WritePin(COM_PINS, REQ, GPIO_PIN_SET);
//		stop = 1;
//		HAL_GPIO_TogglePin(EMB_LEDS, LED_4);
//	}else{
//		stop = 0;
//	}
	trasmissione_completata = 1;
	if(fine == 1){
		if(finestra_corrente<finestra_calcolata){
			HAL_GPIO_WritePin(COM_PINS, REQ, GPIO_PIN_SET);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if (GPIO_Pin == ACK){
//		HAL_GPIO_WritePin(COM_PINS, REQ, GPIO_PIN_RESET);
//		stop = 0;
//		finestra_corrente++;
//		HAL_UART_Transmit_DMA(&huart4, &signal[(finestra_corrente-1)*N_CAMPIONI], 128);
//	}
	HAL_GPIO_WritePin(COM_PINS, REQ, GPIO_PIN_RESET);
	finestra_corrente++;
	HAL_UART_Transmit_DMA(&huart4, &signal[(finestra_corrente-1)*N_CAMPIONI], 128);
}
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
