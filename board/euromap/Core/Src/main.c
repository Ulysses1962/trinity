/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
//==============================================================================
// MACHINE STATE DEFINITION STRUCTURE
//==============================================================================
typedef struct {
    // Machine state definitions
    bool MACHINE_EMGS           : 1;
    bool MOULD_OPEN             : 1;
    bool MACHINE_SAFETY         : 1;
    bool REJECT                 : 1;
    bool DEVICE_OP_ENA          : 1;
    bool MOULD_CLOSED           : 1;
    bool INTER_MOULD_OPEN       : 1;
    // Ejectors state definitions
    bool EJECTOR_IN_BCK_POS     : 1;
    bool EJECTOR_IN_FWD_POS     : 1;
    // Core pullers state definitions
    bool CORE_PULLERS_IN_POS1   : 1;
    bool CORE_PULLERS_IN_POS2   : 1;
} MACHINE_STATE;

//==============================================================================
// MACHINE STATE INTEGRAL INDICATOR
//==============================================================================
static MACHINE_STATE emap_state;
static uint8_t machine_state_string[64];
static uint8_t cmd_ack_string[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
static void emap_state_check(void) {
    GPIO_PinState line_state;
    
    // MACHINE_EMGS state check
    line_state = HAL_GPIO_ReadPin(MACHINE_EMGS_GPIO_Port, MACHINE_EMGS_Pin);
    if (line_state) emap_state.MACHINE_EMGS = true;
    else            emap_state.MACHINE_EMGS = false;
    
    // MOULD_OPEN state check
    line_state = HAL_GPIO_ReadPin(MOULD_OPEN_POS_GPIO_Port, MOULD_OPEN_POS_Pin);
    if (line_state) emap_state.MOULD_OPEN = false;
    else            emap_state.MOULD_OPEN = true;

    // MACHINE_SAFETY state check
    line_state = HAL_GPIO_ReadPin(MACHINE_SAFETY_GPIO_Port, MACHINE_SAFETY_Pin);
    if (line_state) emap_state.MACHINE_SAFETY = false;
    else            emap_state.MACHINE_SAFETY = true;

    // REJECT state check
    line_state = HAL_GPIO_ReadPin(REJECT_GPIO_Port, REJECT_Pin);
    if (line_state) emap_state.REJECT = false;
    else            emap_state.REJECT = true;

    // DEVICE_OP_ENA state check
    line_state = HAL_GPIO_ReadPin(DEVICE_OP_ENA_GPIO_Port, DEVICE_OP_ENA_Pin);
    if (line_state) emap_state.DEVICE_OP_ENA = false;
    else            emap_state.DEVICE_OP_ENA = true;
    
    // MOULD_CLOSED state check
    line_state = HAL_GPIO_ReadPin(MOULD_CLOSED_GPIO_Port, MOULD_CLOSED_Pin);
    if (line_state) emap_state.MOULD_CLOSED = false;
    else            emap_state.MOULD_CLOSED = true;
    
    // INTER_MOULD_OPEN state check
    line_state = HAL_GPIO_ReadPin(INTER_OPEN_POS_GPIO_Port, INTER_OPEN_POS_Pin);
    if (line_state) emap_state.INTER_MOULD_OPEN = false;
    else            emap_state.INTER_MOULD_OPEN = true;
    
    // EJECTOR_IN_BCK_POS state check
    line_state = HAL_GPIO_ReadPin(EJECT_IN_BACK_POS_GPIO_Port, EJECT_IN_BACK_POS_Pin);
    if (line_state) emap_state.EJECTOR_IN_BCK_POS = false;
    else            emap_state.EJECTOR_IN_BCK_POS = true;
    
    // EJECTOR_IN_FWD_POS state check
    line_state = HAL_GPIO_ReadPin(EJECT_IN_FWD_POS_GPIO_Port, EJECT_IN_FWD_POS_Pin);
    if (line_state) emap_state.EJECTOR_IN_FWD_POS = false;
    else            emap_state.EJECTOR_IN_FWD_POS = true;
    
    // CORE_PULLERS_IN_POS1 state check
    line_state = HAL_GPIO_ReadPin(COREPULLER_POS1_GPIO_Port, COREPULLER_POS1_Pin);
    if (line_state) emap_state.CORE_PULLERS_IN_POS1 = false;
    else            emap_state.CORE_PULLERS_IN_POS1 = true;
    
    // CORE_PULLERS_IN_POS2 state check
    line_state = HAL_GPIO_ReadPin(COREPULLER_POS2_GPIO_Port, COREPULLER_POS2_Pin);
    if (line_state) emap_state.CORE_PULLERS_IN_POS2 = false;
    else            emap_state.CORE_PULLERS_IN_POS2 = true;
}

static void emap_send_state(void) {
    memset(machine_state_string, 0x00, 64);
    sprintf((char*)machine_state_string, "%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\r\n",
            emap_state.MACHINE_EMGS, emap_state.MOULD_OPEN, emap_state.MACHINE_SAFETY, emap_state.REJECT, emap_state.DEVICE_OP_ENA,
            emap_state.MOULD_CLOSED, emap_state.INTER_MOULD_OPEN, emap_state.EJECTOR_IN_BCK_POS, emap_state.EJECTOR_IN_FWD_POS,
            emap_state.CORE_PULLERS_IN_POS1, emap_state.CORE_PULLERS_IN_POS2);   
    HAL_UART_Transmit_DMA(&huart1, machine_state_string, strlen((char*)machine_state_string));    
}

static void emap_cmd_ack(bool cmd_result) {
    memset(cmd_ack_string, 0x00, 10);
    if (cmd_result) sprintf((char*)cmd_ack_string, "OK\r\n");
    else            sprintf((char*)cmd_ack_string, "ERROR\r\n");
    HAL_UART_Transmit_DMA(&huart1, cmd_ack_string, strlen((char*)cmd_ack_string));    
}

static void emap_init(void) {
    emap_state_check();
    HAL_TIM_Base_Start_IT(&htim6);    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) emap_state_check();
}

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  
  /* USER CODE BEGIN 2 */
  emap_init();
  /* USER CODE END 2 */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 479;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 4000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  __HAL_UART_DISABLE(&huart1);
    huart1.Instance->CR2 |= 0x0a000000;
  __HAL_UART_ENABLE(&huart1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_CM);
  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOLD_CLOSE_ENA_Pin|MOLD_AREA_FREE_Pin|DEVICE_EMGS_Pin|EJECT_BACK_ENA_Pin 
                          |EJECT_FORWARD_ENA_Pin|CPP2_MOV_ENA_Pin|CPP1_MOV_ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEVICE_OP_MODE_GPIO_Port, DEVICE_OP_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FULL_MOLD_OPEN_ENA_GPIO_Port, FULL_MOLD_OPEN_ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOLD_CLOSE_ENA_Pin MOLD_AREA_FREE_Pin DEVICE_EMGS_Pin EJECT_BACK_ENA_Pin 
                           EJECT_FORWARD_ENA_Pin CPP2_MOV_ENA_Pin CPP1_MOV_ENA_Pin */
  GPIO_InitStruct.Pin = MOLD_CLOSE_ENA_Pin|MOLD_AREA_FREE_Pin|DEVICE_EMGS_Pin|EJECT_BACK_ENA_Pin 
                          |EJECT_FORWARD_ENA_Pin|CPP2_MOV_ENA_Pin|CPP1_MOV_ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DEVICE_OP_MODE_Pin */
  GPIO_InitStruct.Pin = DEVICE_OP_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEVICE_OP_MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FULL_MOLD_OPEN_ENA_Pin */
  GPIO_InitStruct.Pin = FULL_MOLD_OPEN_ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FULL_MOLD_OPEN_ENA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INTER_OPEN_POS_Pin DEVICE_OP_ENA_Pin COREPULLER_POS1_Pin */
  GPIO_InitStruct.Pin = INTER_OPEN_POS_Pin|DEVICE_OP_ENA_Pin|COREPULLER_POS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MACHINE_SAFETY_Pin */
  GPIO_InitStruct.Pin = MACHINE_SAFETY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MACHINE_SAFETY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : REJECT_Pin EJECT_IN_FWD_POS_Pin MOULD_OPEN_POS_Pin MOULD_CLOSED_Pin 
                           COREPULLER_POS2_Pin EJECT_IN_BACK_POS_Pin MACHINE_EMGS_Pin */
  GPIO_InitStruct.Pin = REJECT_Pin|EJECT_IN_FWD_POS_Pin|MOULD_OPEN_POS_Pin|MOULD_CLOSED_Pin 
                          |COREPULLER_POS2_Pin|EJECT_IN_BACK_POS_Pin|MACHINE_EMGS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
