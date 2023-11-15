/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define PWM_R_Pin GPIO_PIN_6
#define PWM_R_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_7
#define R1_GPIO_Port GPIOE
#define R2_Pin GPIO_PIN_8
#define R2_GPIO_Port GPIOE
#define R3_Pin GPIO_PIN_9
#define R3_GPIO_Port GPIOE
#define R4_Pin GPIO_PIN_10
#define R4_GPIO_Port GPIOE
#define C1_Pin GPIO_PIN_11
#define C1_GPIO_Port GPIOE
#define C2_Pin GPIO_PIN_12
#define C2_GPIO_Port GPIOE
#define C3_Pin GPIO_PIN_13
#define C3_GPIO_Port GPIOE
#define C4_Pin GPIO_PIN_14
#define C4_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define EL_1_Pin GPIO_PIN_12
#define EL_1_GPIO_Port GPIOD
#define EL_2_Pin GPIO_PIN_13
#define EL_2_GPIO_Port GPIOD
#define ER_2_Pin GPIO_PIN_6
#define ER_2_GPIO_Port GPIOC
#define ER_1_Pin GPIO_PIN_7
#define ER_1_GPIO_Port GPIOC
#define PWM_L_Pin GPIO_PIN_15
#define PWM_L_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
