/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "qspi_drv.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim4_up;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Firmware image on file system fails CRC check
#define ERR_FS_IMG_CRC_INVALID    0b0011111100 // 0

// Loaded firmware image is invalid and no replacement found in file system
#define ERR_INVALID_NO_FW         0b0000011000 // 1

// Flash page erase failed
#define ERR_ERASE_FAILED          0b0101101100 // 2

// Data written to flash didn't match data read back
#define ERR_WRITE_INVALID         0b0100111100 // 3

// Write data to flash returned failure
#define ERR_WRITE_FAILED          0b0110011000 // 4

//0b0110110100 // 5
//0b0111110100 // 6
//0b0000011100 // 7
//0b0111111100 // 8
//0b0110111100 // 9

#define byteswap32(x) \
   ( ((x & 0xff000000) >> 24) | ((x & 0x00ff0000) >> 8) \
   | ((x & 0x0000ff00) <<  8) | ((x & 0x000000ff) << 24))

struct {
  uint8_t low;
  uint8_t high;
} buffer_c[1280] = {0};

uint16_t buffer_b[1280] = {0};

char animationFrame=99;
#define startAnimation() animationFrame=0
#define stopAnimation() animationFrame=99

// This method of passing variables from the linker script assumes everything is an address.
// boot size is obviously not an address, but this still works, so whatever.
extern uint32_t _boot_size[];
extern uint32_t _app_start[];
extern uint32_t _app_size[];

#define BOOT_SIZE (int)_boot_size
#define APP_SIZE (int)_app_size



void hang_error(uint16_t errno){
  stopAnimation();

  buffer_b[0] = bCat0 | 0b0111110000; //b
  buffer_b[1] = bCat1 | errno;

  buffer_b[4] = bCat4 | 0b0111100100; //E
  buffer_c[0].low=0b01010000; //r
  buffer_c[1].low=0b01010000; //r
  buffer_c[2].low=0b01011100; //o
  buffer_c[3].low=0b01010000; //r

  HAL_FLASH_Lock();
  while(1){}
}



// Get the CRC of the firmware file on FATFS
// If the CRC is wrong, immediately throw an error.
// The firmware file is padded to a fixed length, we don't need to worry about lengths not divisible by 4.
uint32_t f_crc(FIL* fp)
{
#define READ_BLOCK_SIZE (4096*2)
  unsigned int rc;
  char buf[READ_BLOCK_SIZE];
  uint32_t result;

  if ((fp)->fptr !=0) // pointless but just in case
    f_rewind(fp);

  if ((fp)->obj.objsize != APP_SIZE)
    hang_error(ERR_FS_IMG_CRC_INVALID);

  hcrc.State = HAL_CRC_STATE_BUSY;
  __HAL_CRC_DR_RESET(&hcrc);

  while ((fp)->fptr < APP_SIZE -READ_BLOCK_SIZE) {
    f_read(fp, &buf, READ_BLOCK_SIZE, &rc);
    for (int j=0; j<READ_BLOCK_SIZE; j+=4)
      hcrc.Instance->DR = buf[j+3] | (buf[j+2]<<8) | (buf[j+1]<<16) | (buf[j]<<24);
  }
  f_read(fp, &buf, READ_BLOCK_SIZE-4, &rc);
  for (int j=0; j<READ_BLOCK_SIZE-4; j+=4)
    hcrc.Instance->DR = buf[j+3] | (buf[j+2]<<8) | (buf[j+1]<<16) | (buf[j]<<24);

  result = ~(hcrc.Instance->DR);
  hcrc.State = HAL_CRC_STATE_READY;

  f_read(fp, &buf, 4, &rc);
  if (result != (buf[3] | (buf[2]<<8) | (buf[1]<<16) | (buf[0]<<24)) )
    hang_error(ERR_FS_IMG_CRC_INVALID);

  return result;
#undef READ_BLOCK_SIZE
}

// Get CRC of loaded image
uint32_t app_crc()
{
  uint32_t * buf = _app_start;
  hcrc.State = HAL_CRC_STATE_BUSY;
  __HAL_CRC_DR_RESET(&hcrc);

  int j;
  uint32_t t;
  for (j=0; j<(APP_SIZE/4)-1; j++) {
    t = byteswap32(buf[j]);
    hcrc.Instance->DR = t;
  }

  uint32_t result = ~(hcrc.Instance->DR);
  hcrc.State = HAL_CRC_STATE_READY;
  return result;
}


void launch_app(){
  HAL_RCC_DeInit();
  HAL_DeInit();

//  SysTick->CTRL = 0;
//  SysTick->LOAD = 0;
//  SysTick->VAL = 0;

  //__disable_irq();
  //__DSB();
  //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  //__DSB();
  //__ISB();

  // 1st entry in the vector table is stack pointer
  // 2nd entry in the vector table is the application entry point

  __set_MSP(_app_start[0]);
  ((void (*)(void)) _app_start[1])();
}


//#define progress1() buffer_b[0] = bCat0 | 0b0100000000
//#define progress2() buffer_b[1] = bCat1 | 0b0100000000


void progressBar(uint32_t addr){
#define CHUNK (0x30000 / 8) /* ( APP_SIZE / 8)*/
  static uint32_t threshold = ((uint32_t)_app_start) + CHUNK;
  static char progress = 2;
  if (addr>threshold) {
    switch (++progress){
      case 3: buffer_b[2] = bCat2 | 0b0111111100; break;
      case 4: buffer_b[3] = bCat3 | 0b0111111100; break;
      case 5: buffer_b[4] = bCat4 | 0b0111111100; break;
      case 6: buffer_c[0].low =     0b01111111  ; break;
      case 7: buffer_c[1].low =     0b01111111  ; break;
      case 8: buffer_c[2].low =     0b01111111  ; break;
      case 9: buffer_c[3].low =     0b01111111  ; break;
    }
    threshold += CHUNK;
  }
#undef CHUNK
}


void doAnimation(){
  switch (animationFrame++){
    case 0:  buffer_b[0] = bCat0 | 0b0000110000; buffer_b[1] = bCat1 | 0b0000000000; break;
    case 1:  buffer_b[0] = bCat0 | 0b0001100000; buffer_b[1] = bCat1 | 0b0000000000; break;
    case 2:  buffer_b[0] = bCat0 | 0b0011000000; buffer_b[1] = bCat1 | 0b0000000000; break;
    case 3:  buffer_b[0] = bCat0 | 0b0010000100; buffer_b[1] = bCat1 | 0b0000000000; break;
    case 4:  buffer_b[0] = bCat0 | 0b0000001100; buffer_b[1] = bCat1 | 0b0000000000; break;
    case 5:  buffer_b[0] = bCat0 | 0b0000001000; buffer_b[1] = bCat1 | 0b0001000000; break;
    case 6:  buffer_b[0] = bCat0 | 0b0000000000; buffer_b[1] = bCat1 | 0b0001100000; break;
    case 7:  buffer_b[0] = bCat0 | 0b0000000000; buffer_b[1] = bCat1 | 0b0000110000; break;
    case 8:  buffer_b[0] = bCat0 | 0b0000000000; buffer_b[1] = bCat1 | 0b0000011000; break;
    case 9:  buffer_b[0] = bCat0 | 0b0000000000; buffer_b[1] = bCat1 | 0b0000001100; break;
    case 10: buffer_b[0] = bCat0 | 0b0000000000; buffer_b[1] = bCat1 | 0b0010000100; break;
    case 11: buffer_b[0] = bCat0 | 0b0000010000; buffer_b[1] = bCat1 | 0b0010000000; animationFrame=0; break;
    case 99: animationFrame=99; // disable animation
  }
}

void flash_erase(){
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PAGEError = 0;

#define FLASH_PAGES_PER_BANK (FLASH_BANK_SIZE / FLASH_PAGE_SIZE)
#define APP_START_PAGE (((int)_app_start - FLASH_BASE) / FLASH_PAGE_SIZE )

  // Erase from app start up to end of bank 1
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = FLASH_BANK_1;
  EraseInitStruct.Page        = APP_START_PAGE;
  EraseInitStruct.NbPages     = FLASH_PAGES_PER_BANK - APP_START_PAGE;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    hang_error(ERR_ERASE_FAILED);

  //progress1();

  // Erase bank 2
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = FLASH_BANK_2;
  EraseInitStruct.Page        = 0;
  EraseInitStruct.NbPages     = FLASH_PAGES_PER_BANK;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    hang_error(ERR_ERASE_FAILED);

  //progress2();
}

void flash_write(FIL* fp){
  uint32_t addr = (uint32_t)_app_start;

  unsigned int rc;
  char buf[8];

  if ((fp)->fptr !=0)
    f_rewind(fp);

  while (addr < FLASH_BASE + FLASH_SIZE){
    f_read(fp, &buf, 8, &rc);
    uint32_t low = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | (buf[0]);
    uint32_t high= (buf[7] << 24) | (buf[6] << 16) | (buf[5] << 8) | (buf[4]);
    uint64_t data = ((uint64_t)high << 32) | low;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data ) == HAL_OK) {
      if (*(uint64_t*)addr != data) hang_error(ERR_WRITE_INVALID);
      addr += 8;
    } else hang_error(ERR_WRITE_FAILED);

    progressBar(addr);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_QUADSPI_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  buffer_c[0].high=0b11011110;
  buffer_c[1].high=0b11011101;
  buffer_c[2].high=0b11011011;
  buffer_c[3].high=0b11010111;
  buffer_c[4].high=0b11001111;

//  buffer_c[0].low=0b01011100; //o
//  buffer_c[1].low=0b01011100; //o
//  buffer_c[2].low=0b01111000; //t
//  buffer_c[3].low=0;
//  buffer_c[4].low=0;
//
//  buffer_b[0] = 0;//bCat0 | bSegDecode1;
//  buffer_b[1] = 0;//bCat1 | bSegDecode2;
//  buffer_b[2] = 0;//bCat2 | bSegDecode3;
//  buffer_b[3] = 0;//bCat3 | bSegDecode4;
//  buffer_b[4] = bCat4 | 0b0111110000; //b

  if (HAL_DMA_Start(&hdma_tim1_up, (uint32_t)buffer_c, (uint32_t)&GPIOC->ODR, 5) != HAL_OK)
    Error_Handler();

  if (HAL_DMA_Start(&hdma_tim4_up, (uint32_t)buffer_b, (uint32_t)&GPIOB->ODR, 5) != HAL_OK)
    Error_Handler();

  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE(&htim1);

  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE(&htim4);

  _Bool new_fw_present;
  uint32_t new_fw_crc;

  // Calculate CRC of loaded image
  uint32_t loaded_fw_crc = app_crc();

  // Look for new firmware file and calculate its crc
  FIL file;

  if (f_open(&file, "/FWT.BIN", FA_READ) != FR_OK)
    new_fw_present = 0;
  else {
    new_fw_present = 1;
    new_fw_crc = f_crc(&file);
  }


// debug force reload
// new_fw_crc=0;


  if (loaded_fw_crc == byteswap32(*(uint32_t*)(FLASH_BASE + FLASH_SIZE -4)) ) { // loaded firmware valid

    if (!new_fw_present) launch_app();

    if (loaded_fw_crc == new_fw_crc) launch_app();

  } else { // loaded firmware invalid

    if (!new_fw_present) hang_error(ERR_INVALID_NO_FW);

  }

  startAnimation();

  HAL_FLASH_Unlock();
  flash_erase();
  flash_write(&file);
  HAL_FLASH_Lock();

  launch_app();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 0;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 256;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 256;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 
                           PC3 PC4 PC5 PC6 
                           PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  while(1){}
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
