/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_tim5_ch1;
extern DMA_HandleTypeDef hdma_tim5_ch2;
extern DMA_HandleTypeDef hdma_tim7_up;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern uint8_t nmea[90];
extern uint8_t GPS_sv, GLONASS_sv;
extern uint16_t buffer_adc[ADC_BUFFER_SIZE];
extern uint16_t buffer_dac[DAC_BUFFER_SIZE];
extern DAC_HandleTypeDef hdac1;
extern ADC_HandleTypeDef hadc1;
extern float dac_target;
extern uint8_t uart2_tx_buffer[32];
extern _Bool data_valid, had_pps;
extern uint8_t decisec, centisec, millisec;
extern uint8_t displayMode, countMode, nmea_cdc_level;

extern uint32_t qspi_write_time;
extern uint32_t qspi_usb_read_time;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  buffer_c[3].low=cSegDecode0;

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  buffer_c[3].low=cSegDecode1;

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  buffer_c[3].low=cSegDecode2;

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  buffer_c[3].low=cSegDecode3;

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  // Writing to the RTC is normally very fast, but if something goes wrong
  // the HAL functions will fail to time out if it's running with the same
  // preemption priority as systick
  if (had_pps) write_rtc();

  //if (buffer_c[3].high & cSegDP) buffer_c[3].high&=~cSegDP; else buffer_c[3].high|=cSegDP;

  setPrecision();

  if (resendDate) {sendDate(1); resendDate=0;}

  GLONASS_sv = 255; GPS_sv = 255;

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  // 10Hz interrupt, preemption priority 1, same as USB
  // (DAC DMA still runs with display off)
  if (qspi_usb_read_time && uwTick - qspi_usb_read_time > 10) {
    qspi_usb_read_time=0;
  }
  if (qspi_write_time && uwTick - qspi_write_time > 100) {
    delayedReadConfigFile=1;
    qspi_write_time=0;
  }

  if (DMA1->ISR & DMA_FLAG_HT3) {

    generateDACbuffer(&buffer_dac[DAC_BUFFER_SIZE/2]);

    DMA1->IFCR = DMA_ISR_HTIF3;
    return;

  } else if (DMA1->ISR & DMA_FLAG_TC3) {

    generateDACbuffer(&buffer_dac[0]);

    DMA1->IFCR = DMA_ISR_TCIF3;
    return;
  }


  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac_ch1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim7_up);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
  if (DMA1->ISR & DMA_FLAG_TC5) {
    // data buffer full
    if (nmea_cdc_level==NMEA_ALL)
      CDC_Copy_Transmit(&nmea[0], sizeof(nmea));
  }

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */


  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_up);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  uint8_t rec = sizeof(nmea) - huart1.hdmarx->Instance->CNDTR;

  if (nmea_cdc_level==NMEA_ALL) CDC_Copy_Transmit(&nmea[0], rec);

  // look for $GxRMC
  if (nmea[0]=='$'
   && nmea[1]=='G'
   && nmea[3]=='R'
   && nmea[4]=='M'
   && nmea[5]=='C') {
    if (nmea_cdc_level==NMEA_RMC) CDC_Copy_Transmit(&nmea[0], rec);
    decodeRMC();
  } else if (nmea[0]=='$'
   && nmea[1]=='G'
   && nmea[3]=='G'
   && nmea[4]=='S'
   && nmea[5]=='V')
    decodeGSV();


  HAL_UART_AbortReceive(&huart1);
  HAL_UART_Receive_DMA(&huart1, nmea, sizeof(nmea));

  USART1->ICR = USART_ICR_CMCF;

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  if ((USART2->ISR & (USART_ISR_PE|USART_ISR_FE|USART_ISR_ORE|USART_ISR_NE|USART_ISR_RTOF|USART_ISR_RXNE)) == USART_ISR_RXNE ) {
  //if (USART2->ISR & USART_ISR_RXNE ) {

    //if (buffer_c[2].high & cSegDP) buffer_c[2].high&=~cSegDP; else buffer_c[2].high|=cSegDP;

    uint8_t x = (USART2->RDR &0xFF);
    if (x == 0x91) {
        button1pressed();
    } else if (x == 0x92) {
        button2pressed();
    }
    return;
  }

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel4 global interrupt.
  */
void DMA2_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel4_IRQn 0 */

  /* USER CODE END DMA2_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim5_ch2);
  /* USER CODE BEGIN DMA2_Channel4_IRQn 1 */

  /* USER CODE END DMA2_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel5 global interrupt.
  */
void DMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

  /* USER CODE END DMA2_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim5_ch1);
  /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */

  /* USER CODE END DMA2_Channel5_IRQn 1 */
}

/**
  * @brief This function handles LPTIM1 global interrupt.
  */
void LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM1_IRQn 0 */

  /* USER CODE END LPTIM1_IRQn 0 */
  /* USER CODE BEGIN LPTIM1_IRQn 1 */

  LL_LPTIM_ClearFLAG_ARRM(LPTIM1);

  extern uint32_t LPTIM1_high;
  LPTIM1_high++;

  // potentially wait for flag to clear
  // __DSB();

  /* USER CODE END LPTIM1_IRQn 1 */
}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
