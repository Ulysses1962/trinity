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
#define MOLD_CLOSE_ENA_Pin GPIO_PIN_13
#define MOLD_CLOSE_ENA_GPIO_Port GPIOC
#define MOLD_AREA_FREE_Pin GPIO_PIN_14
#define MOLD_AREA_FREE_GPIO_Port GPIOC
#define DEVICE_EMGS_Pin GPIO_PIN_15
#define DEVICE_EMGS_GPIO_Port GPIOC
#define DEVICE_OP_MODE_Pin GPIO_PIN_1
#define DEVICE_OP_MODE_GPIO_Port GPIOF
#define EJECT_BACK_ENA_Pin GPIO_PIN_0
#define EJECT_BACK_ENA_GPIO_Port GPIOC
#define EJECT_FORWARD_ENA_Pin GPIO_PIN_1
#define EJECT_FORWARD_ENA_GPIO_Port GPIOC
#define CPP2_MOV_ENA_Pin GPIO_PIN_2
#define CPP2_MOV_ENA_GPIO_Port GPIOC
#define CPP1_MOV_ENA_Pin GPIO_PIN_3
#define CPP1_MOV_ENA_GPIO_Port GPIOC
#define FULL_MOLD_OPEN_ENA_Pin GPIO_PIN_0
#define FULL_MOLD_OPEN_ENA_GPIO_Port GPIOA
#define EMAP_TX_Pin GPIO_PIN_9
#define EMAP_TX_GPIO_Port GPIOA
#define EMAP_RX_Pin GPIO_PIN_10
#define EMAP_RX_GPIO_Port GPIOA
#define INTER_OPEN_POS_Pin GPIO_PIN_10
#define INTER_OPEN_POS_GPIO_Port GPIOC
#define INTER_OPEN_POS_EXTI_IRQn EXTI4_15_IRQn
#define DEVICE_OP_ENA_Pin GPIO_PIN_11
#define DEVICE_OP_ENA_GPIO_Port GPIOC
#define DEVICE_OP_ENA_EXTI_IRQn EXTI4_15_IRQn
#define COREPULLER_POS1_Pin GPIO_PIN_12
#define COREPULLER_POS1_GPIO_Port GPIOC
#define COREPULLER_POS1_EXTI_IRQn EXTI4_15_IRQn
#define MACHINE_SAFETY_Pin GPIO_PIN_2
#define MACHINE_SAFETY_GPIO_Port GPIOD
#define MACHINE_SAFETY_EXTI_IRQn EXTI2_3_IRQn
#define REJECT_Pin GPIO_PIN_3
#define REJECT_GPIO_Port GPIOB
#define REJECT_EXTI_IRQn EXTI2_3_IRQn
#define EJECT_IN_FWD_POS_Pin GPIO_PIN_4
#define EJECT_IN_FWD_POS_GPIO_Port GPIOB
#define EJECT_IN_FWD_POS_EXTI_IRQn EXTI4_15_IRQn
#define MOULD_OPEN_POS_Pin GPIO_PIN_5
#define MOULD_OPEN_POS_GPIO_Port GPIOB
#define MOULD_OPEN_POS_EXTI_IRQn EXTI4_15_IRQn
#define MOULD_CLOSED_Pin GPIO_PIN_6
#define MOULD_CLOSED_GPIO_Port GPIOB
#define MOULD_CLOSED_EXTI_IRQn EXTI4_15_IRQn
#define COREPULLER_POS2_Pin GPIO_PIN_7
#define COREPULLER_POS2_GPIO_Port GPIOB
#define COREPULLER_POS2_EXTI_IRQn EXTI4_15_IRQn
#define EJECT_IN_BACK_POS_Pin GPIO_PIN_8
#define EJECT_IN_BACK_POS_GPIO_Port GPIOB
#define EJECT_IN_BACK_POS_EXTI_IRQn EXTI4_15_IRQn
#define MACHINE_EMGS_Pin GPIO_PIN_9
#define MACHINE_EMGS_GPIO_Port GPIOB
#define MACHINE_EMGS_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
