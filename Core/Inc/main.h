/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l4xx_ll_lptim.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_dma.h"

#include "stm32l4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
  uint8_t tenYears;
  uint8_t years;
  uint8_t tenMonths;
  uint8_t months;
  uint8_t tenDays;
  uint8_t days;

  uint8_t tenHours;
  uint8_t hours;
  uint8_t tenMinutes;
  uint8_t minutes;
  uint8_t tenSeconds;
  uint8_t seconds;
} bcdStamp_t;

typedef struct {
  uint8_t low;
  uint8_t high;
} buffer_c_t;

extern buffer_c_t buffer_c[];

extern uint16_t buffer_b[];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define RULES_FILENAME  "/TZRULES.BIN"
#define MAP_FILENAME    "/TZMAP.BIN"
#define CONFIG_FILENAME "/CONFIG.TXT"

#define cSegDP 0b00010000

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


// ADC timer is 1000Hz, interrupt at TC
// DAC timer is 100Hz, interrupt at HT and TC
#define DAC_BUFFER_SIZE 20
#define ADC_BUFFER_SIZE 50


#define CMD_LOAD_TEXT          0x90
#define CMD_SET_FREQUENCY      0x91
#define CMD_RELOAD_TEXT        0x92

enum {
  MODE_ISO8601_STD =0,
  MODE_ISO_ORDINAL,
  MODE_ISO_WEEK,
  MODE_UNIX,
  MODE_JULIAN_DATE,
  MODE_MODIFIED_JD,
  MODE_SHOW_OFFSET,
  MODE_SHOW_TZ_NAME,
  MODE_STANDBY,

  NUM_DISPLAY_MODES
};

enum {
  COUNT_NORMAL =0,
  COUNT_HIDDEN,
  COUND_DOWN
};

enum {
  COLON_MODE_SLOWFADE = 0,
  COLON_MODE_HEARTBEAT,
  COLON_MODE_1PPS_SAWTOOTH,
  COLON_MODE_ALT_SAWTOOTH,
  COLON_MODE_TOGGLE,
  COLON_MODE_SOLID
};


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void decodeRMC(void);
void setDisplayPWM(uint32_t bright);
void write_rtc(void);
void displayOff(void);
void button1pressed(void);
void setPrecision(void);
void sendDate( _Bool now );
void readConfigFile(void);
void generateDACbuffer(uint16_t * buf);

#define latchSegments() \
  buffer_c[0].low = next7seg.c; \
  buffer_b[0] = next7seg.b[0]; \
  buffer_b[1] = next7seg.b[1]; \
  buffer_b[2] = next7seg.b[2]; \
  buffer_b[3] = next7seg.b[3]; \
  buffer_b[4] = next7seg.b[4];

#define triggerPendSV() \
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

#define loadNextTimestamp() \
  latchSegments() \
  huart2.Instance->TDR = 0xFE; \
  triggerPendSV()

extern uint32_t __VECTORS_FLASH[];
extern uint32_t __VECTORS_RAM[];
#define SetSysTick(x) __VECTORS_RAM[ 16 + SysTick_IRQn ] = (uint32_t)x
#define SetPPS(x)     __VECTORS_RAM[ 16 + EXTI9_5_IRQn ] = (uint32_t)x

#define SetVector(x,y) __VECTORS_RAM[ 16 + x ] = (uint32_t)y
#define GetVector(x)   ((void (*)(void)) __VECTORS_RAM[ 16 + x ]

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
