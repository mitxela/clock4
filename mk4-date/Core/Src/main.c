/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define byteswap32(x) \
   ( ((x & 0xff000000) >> 24) | ((x & 0x00ff0000) >> 8) \
   | ((x & 0x0000ff00) <<  8) | ((x & 0x000000ff) << 24))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


const uint8_t lut_7seg[] = {
  0,
  64, // !
  0b00100010,// "
  64, // #
  64, // $
  64, // %
  64, // &
  0b00000010,// '
  0b00111001,// (
  0b00001111,// )
  64, // *
  64, // +
  64, // ,
  0b01000000,// -
  64, // .
  0b01010010,// /
  0b00111111,// 0
  0b00000110,// 1
  0b01011011,// 2
  0b01001111,// 3
  0b01100110,// 4
  0b01101101,// 5
  0b01111101,// 6
  0b00000111,// 7
  0b01111111,// 8
  0b01101111,// 9
  64, // :
  64, // ;
  64, // <
  64, // =
  64, // >
  64, // ?
  64, // @
  0b01110111,// A
  0b01111100,// B
  0b00111001,// C
  0b01011110,// D
  0b01111001,// E
  0b01110001,// F
  0b00111101,// G
  0b01110100,// H
  0b00000110,// I
  0b00011110,// J
  0b01110101,// K
  0b00111000,// L
  0b00010101,// M
  0b01010100,// N
  0b00111111,// O
  0b01110011,// P
  0b01100111,// Q
  0b01010000,// R
  0b01101101,// S
  0b01111000,// T
  0b00111110,// U
  0b01100010,// V
  0b00101010,// W
  0b01110110,// X
  0b01101110,// Y
  0b01011011,// Z
  0b00111001,// [
  0b01100100,// \ //
  0b00001111,// ]
  0b00100011,// ^
  0b00001000,// _
  0b00100000,// `
  0b01110111,// a
  0b01111100,// b
  0b01011000,// c
  0b01011110,// d
  0b01111001,// e
  0b01110001,// f
  0b00111101,// g
  0b01110100,// h
  0b00000100,// i
  0b00011110,// j
  0b01110101,// k
  0b00111000,// l
  0b01010101,// m
  0b01010100,// n
  0b01011100,// o
  0b01110011,// p
  0b01100111,// q
  0b01010000,// r
  0b01101101,// s
  0b01111000,// t
  0b00011100,// u
  0b01100010,// v
  0b01101010,// w
  0b01110110,// x
  0b01101110,// y
  0b01011011,// z
  0b00111001,// {
  64, // |
  0b00001111,// }
  64, // ~
};

const uint8_t lut_7seg_inv[] = {
  0,
  64, // !
  0b00010100,// "
  64, // #
  64, // $
  64, // %
  64, // &
  0b00010000,// '
  0b00001111,// (
  0b00111001,// )
  64, // *
  64, // +
  64, // ,
  0b01000000,// -
  64, // .
  0b01010010,// /
  0b00111111,// 0
  0b00110000,// 1
  0b01011011,// 2
  0b01111001,// 3
  0b01110100,// 4
  0b01101101,// 5
  0b01101111,// 6
  0b00111000,// 7
  0b01111111,// 8
  0b01111101,// 9
  64, // :
  64, // ;
  64, // <
  64, // =
  64, // >
  64, // ?
  64, // @
  0b01111110,// A
  0b01100111,// B
  0b00001111,// C
  0b01110011,// D
  0b01001111,// E
  0b01001110,// F
  0b00101111,// G
  0b01100110,// H
  0b00110000,// I
  0b00110011,// J
  0b01101110,// K
  0b00000111,// L
  0b00101010,// M
  0b01100010,// N
  0b00111111,// O
  0b01011110,// P
  0b01111100,// Q
  0b01000010,// R
  0b01101101,// S
  0b01000111,// T
  0b00110111,// U
  0b01010100,// V
  0b00010101,// W
  0b01110110,// X
  0b01110101,// Y
  0b01011011,// Z
  0b00001111,// [
  0b01100100,// \ //
  0b00111001,// ]
  0b00011100,// ^
  0b00000001,// _
  0b00000100,// `
  0b01111110,// a
  0b01100111,// b
  0b01000011,// c
  0b01110011,// d
  0b01001111,// e
  0b01001110,// f
  0b00101111,// g
  0b01100110,// h
  0b00100000,// i
  0b00110011,// j
  0b01101110,// k
  0b00000111,// l
  0b01101010,// m
  0b01100010,// n
  0b01100011,// o
  0b01011110,// p
  0b01111100,// q
  0b01000010,// r
  0b01101101,// s
  0b01000111,// t
  0b00100011,// u
  0b01010100,// v
  0b01010101,// w
  0b01110110,// x
  0b01110101,// y
  0b01011011,// z
  0b00001111,// {
  64, // |
  0b00111001,// }
  64, // ~
};

#define CMD_LOAD_TEXT          0x90
#define CMD_SET_FREQUENCY      0x91
#define CMD_RELOAD_TEXT        0x92
#define CMD_SET_SCROLL_SPEED   0x93

#define CMD_SHOW_CRC           0x9D
#define CMD_REPORT_CRC         0x9E
#define CMD_START_BOOTLOADER   0x9F

#define CMD_SET_FREQUENCY_B2   0xA1
#define CMD_SET_FREQUENCY_B3   0xA2

// 32e6/5/50 = 128000 Hz
// on -O0, ARR_MIN 66 => 95522.388Hz
#define ARR_MIN                49
#define ARR_MAX                6399
// 32e6/5/6400 = 1000Hz

uint16_t pre_buffer_a[5] ={0};
uint16_t pre_buffer_b[5] ={0};
uint8_t status = 0;

uint16_t buffer_a[5] ={0};
uint16_t buffer_b[5] ={0};
uint8_t buffer_idx=0;

#define MAX_TEXT_LEN 32
uint8_t text[MAX_TEXT_LEN] ={0};
uint8_t text_idx=0;
uint8_t dp_pos=0;


uint32_t target_freq=0;

const uint16_t cathodes_a[5]={
    0b1001100000000010,
    0b1001100000000001,
    0b1001000000000011,
    0b1000100000000011,
    0b0001100000000011
};
const uint16_t cathodes_b[5]={
    0b1111000000000000,
    0b1110100000000000,
    0b1101100000000000,
    0b1011100000000000,
    0b0111100000000000
};

uint8_t inverted=0;
uint8_t b1_held =0;
uint8_t b2_held =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

__attribute__((naked,noreturn))
void triggerBootloader(void){


  LL_TIM_DisableIT_UPDATE(TIM2);
  LL_TIM_DisableCounter(TIM2);
  LL_TIM_DeInit(TIM2);

  LL_TIM_DisableIT_UPDATE(TIM21);
  LL_TIM_DisableCounter(TIM21);
  LL_TIM_DeInit(TIM21);

  LL_USART_Disable(USART2);
  LL_USART_DeInit(USART2);

  GPIOA->ODR=0;
  GPIOB->ODR=0;

  LL_GPIO_DeInit(GPIOA);
  LL_GPIO_DeInit(GPIOB);

  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_ALL);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_ALL);

  LL_RCC_DeInit();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

#define SYSMEM 0x1FF00000

  __set_MSP(*(volatile uint32_t*) SYSMEM);
  ((void (*)(void)) (*((volatile uint32_t *)(SYSMEM + 4))))();

  __builtin_unreachable();
}


void setDigitPre(uint8_t digit, uint8_t val){
  if (val<32 || val>127) val=32;

  if (inverted == 1) {
    if (digit>=5) {
      pre_buffer_b[9-digit] = (lut_7seg_inv[val-32]<<4) | cathodes_b[9-digit];
    } else {
      pre_buffer_a[9-digit-5] = (lut_7seg_inv[val-32]<<4) | cathodes_a[9-digit-5];
    }
  } else {
    if (digit>=5) {
      pre_buffer_a[digit-5] = (lut_7seg[val-32]<<4) | cathodes_a[digit-5];
    } else {
      pre_buffer_b[digit] = (lut_7seg[val-32]<<4) | cathodes_b[digit];
    }
  }
}
void setDigitDirect(uint8_t digit, uint8_t val){
  if (val<32 || val>127) val=32;

  if (inverted == 1) {
    if (digit>=5) {
      buffer_b[9-digit] = (lut_7seg_inv[val-32]<<4) | cathodes_b[9-digit];
    } else {
      buffer_a[9-digit-5] = (lut_7seg_inv[val-32]<<4) | cathodes_a[9-digit-5];
    }
  } else {
    if (digit>=5) {
      buffer_a[digit-5] = (lut_7seg[val-32]<<4) | cathodes_a[digit-5];
    } else {
      buffer_b[digit] = (lut_7seg[val-32]<<4) | cathodes_b[digit];
    }
  }
}

// Main display matrix routine
void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_UIF){

    GPIOA->ODR = buffer_a[buffer_idx];
    GPIOB->ODR = buffer_b[buffer_idx];

    buffer_idx ++;
    if (buffer_idx>=5) buffer_idx=0;

    TIM2->SR = ~TIM_DIER_UIE;
    return;
  }
}

void TIM21_IRQHandler(void){
  if (TIM21->SR & TIM_SR_UIF){

    uint8_t i = (LL_GPIO_ReadInputPort(GPIOC) & 1)?1:0;

    //if (i!=inverted)...

    inverted=i;

    TIM21->SR = ~TIM_DIER_UIE;

   //if (!latched) return; //don't intervene while waiting for latch
    //if (inverted)...

#define btn_debounce 2
#define btn_delay 42
#define btn_repeat 10

    if ((LL_GPIO_ReadInputPort(GPIOB) & LL_GPIO_PIN_3)==0) {
      if (++b1_held == btn_debounce || b1_held == btn_delay) {
          if ( (USART2->ISR & USART_ISR_TXE) ) {
            USART2->TDR = 0x91 + inverted;
          }
          if (b1_held==btn_delay) b1_held -= btn_repeat;
      }
    } else b1_held=0;
    if ((LL_GPIO_ReadInputPort(GPIOC) & LL_GPIO_PIN_13)==0) {
      if (++b2_held == btn_debounce || b2_held == btn_delay) {
          if ( (USART2->ISR & USART_ISR_TXE) ) {
            USART2->TDR = 0x92 - inverted;
          }
          if (b2_held==btn_delay) b2_held -= btn_repeat;
      }
    } else b2_held=0;
  }
}

static inline void setFrequency(void){

  if (target_freq<1 || target_freq>100000) return;

  uint32_t arr = round(6400000.0 / (float)target_freq) -1.0;
  if (arr > ARR_MAX) arr = ARR_MAX;
  if (arr < ARR_MIN) arr = ARR_MIN;
  TIM2->ARR= arr;
}

static inline void latchDisplay(void){
  buffer_b[0] = pre_buffer_b[0];
  buffer_b[1] = pre_buffer_b[1];
  buffer_b[2] = pre_buffer_b[2];
  buffer_b[3] = pre_buffer_b[3];
  buffer_b[4] = pre_buffer_b[4];
  buffer_a[0] = pre_buffer_a[0];
  buffer_a[1] = pre_buffer_a[1];
  buffer_a[2] = pre_buffer_a[2];
  buffer_a[3] = pre_buffer_a[3];
  buffer_a[4] = pre_buffer_a[4];

  if (!dp_pos) return;

  if (inverted){
    if (dp_pos>=5) {
      buffer_b[9-dp_pos] |=1 | cathodes_b[9-dp_pos];
    } else {
      buffer_a[9-dp_pos-5]  |=(1<<14) | cathodes_a[9-dp_pos-5];
    }
  } else {
    if (dp_pos>5) {
      buffer_a[dp_pos-6] |=1<<14;
    } else {
      buffer_b[dp_pos-1] |=1;
    }
  }
}
static inline uint8_t waitForByte(void){
  while( !( USART2->ISR & USART_ISR_RXNE ) ) {};
  return USART2->RDR;
}

void transmitBlocking(uint8_t * c, size_t n){
  while (n--){
    while( !( USART2->ISR & USART_ISR_TXE ) ) {};
    USART2->TDR = *c++;
  }
}

static inline void waitForLatch(void){
  LL_USART_DisableDirectionRx(USART2);
  LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT );

  while (GPIOA->IDR & LL_GPIO_PIN_3) {}

  latchDisplay();

  // The latch byte is 0xFE with even parity, so as soon as the line returns high we can re-enable uart
  while (!(GPIOA->IDR & LL_GPIO_PIN_3)) {}

  LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE );
  LL_USART_EnableDirectionRx(USART2);
}

static inline void parseByte(uint8_t x){

  if (x & 0x80) { // command byte
    status = x;
    switch (x) {
      case CMD_SHOW_CRC:
      case CMD_LOAD_TEXT:
        text_idx=0;
        dp_pos=0;
        memset(text, 0, MAX_TEXT_LEN);

        pre_buffer_b[0]=0;
        pre_buffer_b[1]=0;
        pre_buffer_b[2]=0;
        pre_buffer_b[3]=0;
        pre_buffer_b[4]=0;
        pre_buffer_a[0]=0;
        pre_buffer_a[1]=0;
        pre_buffer_a[2]=0;
        pre_buffer_a[3]=0;
        pre_buffer_a[4]=0;

        break;
      case CMD_RELOAD_TEXT:
        latchDisplay();
        break;
      case CMD_SET_SCROLL_SPEED:
        break;

      case CMD_SET_FREQUENCY:
      case CMD_SET_FREQUENCY_B2:
      case CMD_SET_FREQUENCY_B3:
        target_freq=0;
        break;

      case CMD_REPORT_CRC:
        transmitBlocking( (uint8_t*)0x8007ffc, 4);
        break;

      case CMD_START_BOOTLOADER:
        triggerBootloader();
        break;

      default:
        status=0;
    }

    if (x==CMD_SHOW_CRC) {
      uint32_t* crc = (uint32_t*)0x8007ffc;
      sprintf(text, "d %08lx", byteswap32(crc[0]));
      for (text_idx=0; text_idx<10; text_idx++)
        setDigitPre(text_idx, text[text_idx]);
    }

    return;
  }

  // Process data
  switch(status){

  case CMD_SHOW_CRC:
  case CMD_LOAD_TEXT:
    if (x=='\n' || x==0) {
      waitForLatch();
      return;
    }
    if (x=='.') {
      dp_pos = text_idx;
      return;
    }
    if(text_idx > MAX_TEXT_LEN) return;

    if (text_idx < 10) setDigitPre(text_idx, x);
    text[text_idx++] = x;
    return;

  case CMD_SET_SCROLL_SPEED:
    return;

  case CMD_SET_FREQUENCY:
    status=CMD_SET_FREQUENCY_B2;
    target_freq |= x<<14;
    return;
  case CMD_SET_FREQUENCY_B2:
    status=CMD_SET_FREQUENCY_B3;
    target_freq |= x<<7;
    return;
  case CMD_SET_FREQUENCY_B3:
    status=0;
    target_freq |= x;
    setFrequency();
    return;
  }


}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */


  //  (HAL_TIM_Base_Start_IT(&htim2)


  LL_TIM_EnableIT_UPDATE(TIM2); //TIM2->DIER |= TIM_DIER_UIE;
  LL_TIM_EnableCounter(TIM2); //TIM2->CR1 |= TIM_CR1_CEN;

  LL_TIM_EnableIT_UPDATE(TIM21);
  LL_TIM_EnableCounter(TIM21);

//  setDigitDirect(0, 'l');
//  setDigitDirect(1, 'o');
//  setDigitDirect(2, 'l');
//  setDigitDirect(3, 'o');
//  setDigitDirect(4, 'l');
//  setDigitDirect(5, 'o');
//  setDigitDirect(6, 'l');
//  setDigitDirect(7, 'o');
//  setDigitDirect(8, 'l');
//  setDigitDirect(9, 'o');


  //buffer_a[2] |= 1<<14;
  //buffer_b[2] |= 1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    uint8_t x = waitForByte();
    parseByte(x);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }

  LL_Init1msTick(32000000);

  LL_SetSystemCoreClock(32000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 319;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM21);

  /* TIM21 interrupt Init */
  NVIC_SetPriority(TIM21_IRQn, 0);
  NVIC_EnableIRQ(TIM21_IRQn);

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  TIM_InitStruct.Prescaler = 31;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 19999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM21, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM21);
  LL_TIM_SetClockSource(TIM21, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM21, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM21);
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  // Disable overrun detection, for two reasons
  // 1. The command structure should always sort itself out
  // 2. It makes interactive debugging the assembled clock a lot easier
  USART2->CR3 = USART_CR3_OVRDIS_Msk;

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#define DISABLE_SWCLK

#ifdef DISABLE_SWCLK

  if ( (LL_GPIO_ReadInputPort(GPIOC) & LL_GPIO_PIN_13)!=0 && ((LL_GPIO_ReadInputPort(GPIOB) & LL_GPIO_PIN_3)!=0) ){
    GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
#endif
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
