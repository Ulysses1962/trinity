
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

#define SERVO_A_PULSE_Pin GPIO_PIN_5
#define SERVO_A_PULSE_GPIO_Port GPIOE
#define SENS_IN_A_Pin GPIO_PIN_13
#define SENS_IN_A_GPIO_Port GPIOC
#define SENS_IN_B_Pin GPIO_PIN_14
#define SENS_IN_B_GPIO_Port GPIOC
#define SENS_INTERRUPT_Pin GPIO_PIN_15
#define SENS_INTERRUPT_GPIO_Port GPIOC
#define SENS_INTERRUPT_EXTI_IRQn EXTI15_10_IRQn
#define SENS_STROBE_Pin GPIO_PIN_0
#define SENS_STROBE_GPIO_Port GPIOC
#define SENS_ADDR0_Pin GPIO_PIN_1
#define SENS_ADDR0_GPIO_Port GPIOC
#define SENS_ADDR1_Pin GPIO_PIN_2
#define SENS_ADDR1_GPIO_Port GPIOC
#define SENS_ADDR2_Pin GPIO_PIN_3
#define SENS_ADDR2_GPIO_Port GPIOC
#define RS485_DE_Pin GPIO_PIN_1
#define RS485_DE_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA
#define MEM_NSS_Pin GPIO_PIN_4
#define MEM_NSS_GPIO_Port GPIOA
#define MEM_SCK_Pin GPIO_PIN_5
#define MEM_SCK_GPIO_Port GPIOA
#define MEM_MISO_Pin GPIO_PIN_6
#define MEM_MISO_GPIO_Port GPIOA
#define MEM_MOSI_Pin GPIO_PIN_7
#define MEM_MOSI_GPIO_Port GPIOA
#define SERVO_X_PULSE_Pin GPIO_PIN_9
#define SERVO_X_PULSE_GPIO_Port GPIOE
#define SERVO_X_SRDY_INT_Pin GPIO_PIN_10
#define SERVO_X_SRDY_INT_GPIO_Port GPIOE
#define SERVO_X_SRDY_INT_EXTI_IRQn EXTI15_10_IRQn
#define SERVO_Y_SRDY_INT_Pin GPIO_PIN_11
#define SERVO_Y_SRDY_INT_GPIO_Port GPIOE
#define SERVO_Y_SRDY_INT_EXTI_IRQn EXTI15_10_IRQn
#define SERVO_Z_SRDY_INT_Pin GPIO_PIN_12
#define SERVO_Z_SRDY_INT_GPIO_Port GPIOE
#define SERVO_Z_SRDY_INT_EXTI_IRQn EXTI15_10_IRQn
#define SERVO_A_SRDY_INT_Pin GPIO_PIN_13
#define SERVO_A_SRDY_INT_GPIO_Port GPIOE
#define SERVO_A_SRDY_INT_EXTI_IRQn EXTI15_10_IRQn
#define SERVO_ALARM_INT_Pin GPIO_PIN_14
#define SERVO_ALARM_INT_GPIO_Port GPIOE
#define SERVO_ALARM_INT_EXTI_IRQn EXTI15_10_IRQn
#define SERVO_Z_PULSE_Pin GPIO_PIN_10
#define SERVO_Z_PULSE_GPIO_Port GPIOB
#define SERVO_NSS_Pin GPIO_PIN_12
#define SERVO_NSS_GPIO_Port GPIOB
#define SERVO_SCK_Pin GPIO_PIN_13
#define SERVO_SCK_GPIO_Port GPIOB
#define SERVO_MOSI_Pin GPIO_PIN_15
#define SERVO_MOSI_GPIO_Port GPIOB
#define WIFI_TX_Pin GPIO_PIN_8
#define WIFI_TX_GPIO_Port GPIOD
#define WIFI_RX_Pin GPIO_PIN_9
#define WIFI_RX_GPIO_Port GPIOD
#define WIFI_RESET_Pin GPIO_PIN_10
#define WIFI_RESET_GPIO_Port GPIOD
#define SERVO_Y_PULSE_Pin GPIO_PIN_6
#define SERVO_Y_PULSE_GPIO_Port GPIOC
#define USB_RESET_Pin GPIO_PIN_8
#define USB_RESET_GPIO_Port GPIOC
#define EMAP_RESET_Pin GPIO_PIN_9
#define EMAP_RESET_GPIO_Port GPIOC
#define EMAP_SYNC_Pin GPIO_PIN_8
#define EMAP_SYNC_GPIO_Port GPIOA
#define EMAP_TX_Pin GPIO_PIN_9
#define EMAP_TX_GPIO_Port GPIOA
#define EMAP_RX_Pin GPIO_PIN_10
#define EMAP_RX_GPIO_Port GPIOA
#define CONSOLE_DM_Pin GPIO_PIN_11
#define CONSOLE_DM_GPIO_Port GPIOA
#define CONSOLE_DP_Pin GPIO_PIN_12
#define CONSOLE_DP_GPIO_Port GPIOA
#define TS_NSS_Pin GPIO_PIN_15
#define TS_NSS_GPIO_Port GPIOA
#define TS_SCK_Pin GPIO_PIN_10
#define TS_SCK_GPIO_Port GPIOC
#define TS_MOSI_Pin GPIO_PIN_12
#define TS_MOSI_GPIO_Port GPIOC


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

