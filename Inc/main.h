/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENS_ADDR0_Pin GPIO_PIN_13
#define SENS_ADDR0_GPIO_Port GPIOC
#define SENS_ADDR1_Pin GPIO_PIN_14
#define SENS_ADDR1_GPIO_Port GPIOC
#define SENS_ADDR2_Pin GPIO_PIN_15
#define SENS_ADDR2_GPIO_Port GPIOC
#define TS_MOSI_Pin GPIO_PIN_1
#define TS_MOSI_GPIO_Port GPIOC
#define SENS_STROBE_Pin GPIO_PIN_0
#define SENS_STROBE_GPIO_Port GPIOA
#define SENS_CHANGED_A_Pin GPIO_PIN_1
#define SENS_CHANGED_A_GPIO_Port GPIOA
#define DATA_CS_Pin GPIO_PIN_4
#define DATA_CS_GPIO_Port GPIOA
#define DATA_CLK_Pin GPIO_PIN_5
#define DATA_CLK_GPIO_Port GPIOA
#define DATA_MISO_Pin GPIO_PIN_6
#define DATA_MISO_GPIO_Port GPIOA
#define DATA_MOSI_Pin GPIO_PIN_7
#define DATA_MOSI_GPIO_Port GPIOA
#define MACHINE_EMGS_Pin GPIO_PIN_4
#define MACHINE_EMGS_GPIO_Port GPIOC
#define MOLD_OPEN_POS_Pin GPIO_PIN_5
#define MOLD_OPEN_POS_GPIO_Port GPIOC
#define SENS_CHANGED_Pin GPIO_PIN_0
#define SENS_CHANGED_GPIO_Port GPIOB
#define RESET_SENSE_Pin GPIO_PIN_1
#define RESET_SENSE_GPIO_Port GPIOB
#define EMAP_MOSI_Pin GPIO_PIN_2
#define EMAP_MOSI_GPIO_Port GPIOB
#define MACHINE_SAFETY_Pin GPIO_PIN_7
#define MACHINE_SAFETY_GPIO_Port GPIOE
#define SENS_A_IN_Pin GPIO_PIN_8
#define SENS_A_IN_GPIO_Port GPIOE
#define TRO_0_PULSE_Pin GPIO_PIN_9
#define TRO_0_PULSE_GPIO_Port GPIOE
#define SENS_B_IN_Pin GPIO_PIN_10
#define SENS_B_IN_GPIO_Port GPIOE
#define TRO_1_PULSE_Pin GPIO_PIN_11
#define TRO_1_PULSE_GPIO_Port GPIOE
#define TRO_2_PULSE_Pin GPIO_PIN_13
#define TRO_2_PULSE_GPIO_Port GPIOE
#define TRO_3_PULSE_Pin GPIO_PIN_14
#define TRO_3_PULSE_GPIO_Port GPIOE
#define TS_CLK_Pin GPIO_PIN_10
#define TS_CLK_GPIO_Port GPIOB
#define TS_ENA_Pin GPIO_PIN_11
#define TS_ENA_GPIO_Port GPIOB
#define WIFI_RESET_Pin GPIO_PIN_15
#define WIFI_RESET_GPIO_Port GPIOB
#define EJECT_BACK_POS_Pin GPIO_PIN_8
#define EJECT_BACK_POS_GPIO_Port GPIOD
#define EJECT_FORWARD_POS_Pin GPIO_PIN_9
#define EJECT_FORWARD_POS_GPIO_Port GPIOD
#define CORE_PULL_POS1_Pin GPIO_PIN_10
#define CORE_PULL_POS1_GPIO_Port GPIOD
#define CORE_PULL_POS2_Pin GPIO_PIN_11
#define CORE_PULL_POS2_GPIO_Port GPIOD
#define REJECT_Pin GPIO_PIN_12
#define REJECT_GPIO_Port GPIOD
#define DEVICE_OP_ENA_Pin GPIO_PIN_13
#define DEVICE_OP_ENA_GPIO_Port GPIOD
#define MOLD_CLOSE_Pin GPIO_PIN_14
#define MOLD_CLOSE_GPIO_Port GPIOD
#define INTER_OPEN_POS_Pin GPIO_PIN_15
#define INTER_OPEN_POS_GPIO_Port GPIOD
#define TRO_4_PULSE_Pin GPIO_PIN_6
#define TRO_4_PULSE_GPIO_Port GPIOC
#define TRO_5_PULSE_Pin GPIO_PIN_7
#define TRO_5_PULSE_GPIO_Port GPIOC
#define TRO_6_PULSE_Pin GPIO_PIN_8
#define TRO_6_PULSE_GPIO_Port GPIOC
#define TRO_7_PULSE_Pin GPIO_PIN_9
#define TRO_7_PULSE_GPIO_Port GPIOC
#define SYNCHRO_Pin GPIO_PIN_8
#define SYNCHRO_GPIO_Port GPIOA
#define WIFI_TX_Pin GPIO_PIN_9
#define WIFI_TX_GPIO_Port GPIOA
#define WIFI_RX_Pin GPIO_PIN_10
#define WIFI_RX_GPIO_Port GPIOA
#define CONS_DM_Pin GPIO_PIN_11
#define CONS_DM_GPIO_Port GPIOA
#define CONS_DP_Pin GPIO_PIN_12
#define CONS_DP_GPIO_Port GPIOA
#define EMAP_ENA_Pin GPIO_PIN_15
#define EMAP_ENA_GPIO_Port GPIOA
#define EMAP_CLK_Pin GPIO_PIN_10
#define EMAP_CLK_GPIO_Port GPIOC
#define CAN_RX_Pin GPIO_PIN_0
#define CAN_RX_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_TX_GPIO_Port GPIOD
#define RESET_A_Pin GPIO_PIN_7
#define RESET_A_GPIO_Port GPIOD
#define RESET_Z_Pin GPIO_PIN_3
#define RESET_Z_GPIO_Port GPIOB
#define RESET_Y_Pin GPIO_PIN_4
#define RESET_Y_GPIO_Port GPIOB
#define RESET_X_Pin GPIO_PIN_5
#define RESET_X_GPIO_Port GPIOB
#define I_SCL_Pin GPIO_PIN_6
#define I_SCL_GPIO_Port GPIOB
#define I_SDA_Pin GPIO_PIN_7
#define I_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
