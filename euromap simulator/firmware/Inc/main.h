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
#include "stm32f0xx_hal.h"
#include "stdbool.h"
#include "stdint.h"

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
#define USB_RESET_Pin GPIO_PIN_2
#define USB_RESET_GPIO_Port GPIOA
#define MACHINE_EMGS_Pin GPIO_PIN_4
#define MACHINE_EMGS_GPIO_Port GPIOA
#define MOULD_OPEN_POS_Pin GPIO_PIN_5
#define MOULD_OPEN_POS_GPIO_Port GPIOA
#define MACHINE_SAFETY_Pin GPIO_PIN_6
#define MACHINE_SAFETY_GPIO_Port GPIOA
#define EJECT_IN_BACK_POS_Pin GPIO_PIN_7
#define EJECT_IN_BACK_POS_GPIO_Port GPIOA
#define EJECT_IN_FWD_POS_Pin GPIO_PIN_4
#define EJECT_IN_FWD_POS_GPIO_Port GPIOC
#define CP_IN_POS1_Pin GPIO_PIN_5
#define CP_IN_POS1_GPIO_Port GPIOC
#define CP_IN_POS2_Pin GPIO_PIN_0
#define CP_IN_POS2_GPIO_Port GPIOB
#define REJECT_Pin GPIO_PIN_1
#define REJECT_GPIO_Port GPIOB
#define DEVICE_OP_ENA_Pin GPIO_PIN_2
#define DEVICE_OP_ENA_GPIO_Port GPIOB
#define MOULD_CLOSED_Pin GPIO_PIN_10
#define MOULD_CLOSED_GPIO_Port GPIOB
#define INTER_MOULD_OPEN_POS_Pin GPIO_PIN_11
#define INTER_MOULD_OPEN_POS_GPIO_Port GPIOB
#define MOULD_CLOSE_ENA_Pin GPIO_PIN_10
#define MOULD_CLOSE_ENA_GPIO_Port GPIOC
#define MOULD_CLOSE_ENA_EXTI_IRQn EXTI4_15_IRQn
#define DEVICE_EMGS_Pin GPIO_PIN_11
#define DEVICE_EMGS_GPIO_Port GPIOC
#define DEVICE_EMGS_EXTI_IRQn EXTI4_15_IRQn
#define EJECT_FWD_ENA_Pin GPIO_PIN_12
#define EJECT_FWD_ENA_GPIO_Port GPIOC
#define EJECT_FWD_ENA_EXTI_IRQn EXTI4_15_IRQn
#define MOULD_AREA_FREE_Pin GPIO_PIN_2
#define MOULD_AREA_FREE_GPIO_Port GPIOD
#define MOULD_AREA_FREE_EXTI_IRQn EXTI2_3_IRQn
#define DEVICE_OP_MODE_Pin GPIO_PIN_3
#define DEVICE_OP_MODE_GPIO_Port GPIOB
#define DEVICE_OP_MODE_EXTI_IRQn EXTI2_3_IRQn
#define CP_POS1_ENA_Pin GPIO_PIN_4
#define CP_POS1_ENA_GPIO_Port GPIOB
#define CP_POS1_ENA_EXTI_IRQn EXTI4_15_IRQn
#define FULL_MOULD_OPEN_ENA_Pin GPIO_PIN_5
#define FULL_MOULD_OPEN_ENA_GPIO_Port GPIOB
#define FULL_MOULD_OPEN_ENA_EXTI_IRQn EXTI4_15_IRQn
#define EJECT_BACK_ENA_Pin GPIO_PIN_6
#define EJECT_BACK_ENA_GPIO_Port GPIOB
#define EJECT_BACK_ENA_EXTI_IRQn EXTI4_15_IRQn
#define CP_POS2_ENA_Pin GPIO_PIN_7
#define CP_POS2_ENA_GPIO_Port GPIOB
#define CP_POS2_ENA_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
