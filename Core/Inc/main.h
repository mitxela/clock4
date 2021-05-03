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

#define cSegDP 0b00100000

#define cSegDecode0 0b00111111
#define cSegDecode1 0b00000110
#define cSegDecode2 0b01011011
#define cSegDecode3 0b01001111
#define cSegDecode4 0b01100110
#define cSegDecode5 0b01101101
#define cSegDecode6 0b01111101
#define cSegDecode7 0b00000111
#define cSegDecode8 0b01111111
#define cSegDecode9 0b01101111

#define bSegDecode0 0b0011111100
#define bSegDecode1 0b0000011000
#define bSegDecode2 0b0101101100
#define bSegDecode3 0b0100111100
#define bSegDecode4 0b0110011000
#define bSegDecode5 0b0110110100
#define bSegDecode6 0b0111110100
#define bSegDecode7 0b0000011100
#define bSegDecode8 0b0111111100
#define bSegDecode9 0b0110111100

#define bCat0 0b1111000000000000
#define bCat1 0b1110001000000000
#define bCat2 0b1101001000000000
#define bCat3 0b1011001000000000
#define bCat4 0b0111001000000000

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
