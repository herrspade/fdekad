/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define DEC0_NC_P7_17_Pin GPIO_PIN_0
#define DEC0_NC_P7_17_GPIO_Port GPIOA
#define DEK1_NO_p7_8_Pin GPIO_PIN_1
#define DEK1_NO_p7_8_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define DEK2_NO_p7_9_Pin GPIO_PIN_3
#define DEK2_NO_p7_9_GPIO_Port GPIOA
#define DEK3_NO_p7_10_Pin GPIO_PIN_4
#define DEK3_NO_p7_10_GPIO_Port GPIOA
#define DEK4_NO_p7_11_Pin GPIO_PIN_5
#define DEK4_NO_p7_11_GPIO_Port GPIOA
#define DEK5_NO_p7_12_Pin GPIO_PIN_6
#define DEK5_NO_p7_12_GPIO_Port GPIOA
#define DEK6_NO_p7_13_Pin GPIO_PIN_7
#define DEK6_NO_p7_13_GPIO_Port GPIOA
#define DEK_manual_Pin GPIO_PIN_8
#define DEK_manual_GPIO_Port GPIOA
#define DEK_pulse_inp_p1_Pin GPIO_PIN_9
#define DEK_pulse_inp_p1_GPIO_Port GPIOA
#define DEK_reset_Pin GPIO_PIN_10
#define DEK_reset_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define REL_NC_p5_6_Pin GPIO_PIN_4
#define REL_NC_p5_6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
