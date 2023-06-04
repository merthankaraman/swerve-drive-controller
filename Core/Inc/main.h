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
#include "stm32f1xx_hal.h"

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
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define STP_PDN_Pin GPIO_PIN_2
#define STP_PDN_GPIO_Port GPIOA
#define ENC_Z_Pin GPIO_PIN_4
#define ENC_Z_GPIO_Port GPIOA
#define MOTOR_EN_Pin GPIO_PIN_5
#define MOTOR_EN_GPIO_Port GPIOA
#define MOTOR_PWM_P_Pin GPIO_PIN_6
#define MOTOR_PWM_P_GPIO_Port GPIOA
#define MOTOR_PWM_M_Pin GPIO_PIN_7
#define MOTOR_PWM_M_GPIO_Port GPIOA
#define STP_MS2_Pin GPIO_PIN_15
#define STP_MS2_GPIO_Port GPIOA
#define STP_MS1_Pin GPIO_PIN_3
#define STP_MS1_GPIO_Port GPIOB
#define STP_EN_Pin GPIO_PIN_4
#define STP_EN_GPIO_Port GPIOB
#define STP_DIR_Pin GPIO_PIN_5
#define STP_DIR_GPIO_Port GPIOB
#define STP_STEP_Pin GPIO_PIN_6
#define STP_STEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
