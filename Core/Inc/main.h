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
#define drive_m_en_Pin GPIO_PIN_3
#define drive_m_en_GPIO_Port GPIOA
#define steer_m_en_Pin GPIO_PIN_4
#define steer_m_en_GPIO_Port GPIOA
#define steer_ENC_A_Pin GPIO_PIN_6
#define steer_ENC_A_GPIO_Port GPIOA
#define steer_ENC_B_Pin GPIO_PIN_7
#define steer_ENC_B_GPIO_Port GPIOA
#define AIN0_Pin GPIO_PIN_12
#define AIN0_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_13
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_14
#define AIN2_GPIO_Port GPIOB
#define drive_m_p_Pin GPIO_PIN_8
#define drive_m_p_GPIO_Port GPIOA
#define drive_m_n_Pin GPIO_PIN_9
#define drive_m_n_GPIO_Port GPIOA
#define steer_m_p_Pin GPIO_PIN_10
#define steer_m_p_GPIO_Port GPIOA
#define steer_m_n_Pin GPIO_PIN_11
#define steer_m_n_GPIO_Port GPIOA
#define drive_ENC_A_Pin GPIO_PIN_15
#define drive_ENC_A_GPIO_Port GPIOA
#define drive_ENC_B_Pin GPIO_PIN_3
#define drive_ENC_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
