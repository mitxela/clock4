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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "retarget.h"
#include "qspi_drv.h"
#include "zonedetect.h"
#include "chainloader.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim7_up;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM7_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void tmToBcd(struct tm *in, bcdStamp_t *out );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t cLut[]= { cSegDecode0, cSegDecode1, cSegDecode2, cSegDecode3, cSegDecode4, cSegDecode5, cSegDecode6, cSegDecode7, cSegDecode8, cSegDecode9 };

buffer_c_t buffer_c[80] = {0};

uint16_t buffer_b[80] = {0};

uint8_t uart2_tx_buffer[32];

volatile uint16_t buffer_adc[ADC_BUFFER_SIZE] = {0};
uint16_t buffer_dac[DAC_BUFFER_SIZE] = {[0 ... DAC_BUFFER_SIZE-1] = 4095};
float dac_target=4095;

// NMEA 0183 messages have a max length of 82 characters
uint8_t nmea[90];

time_t currentTime;

bcdStamp_t nextBcd;
struct {
  uint8_t c;
  uint16_t b[5];
} next7seg;

uint8_t decisec=0, centisec=0, millisec=0;

float longitude=-9999, latitude=-9999;
uint8_t data_valid =0, had_pps=0;

struct {
  uint32_t t;
  int32_t offset;
} rules[162];
#define MAX_RULES (sizeof rules / sizeof rules[0])

char loadedRulesString[32];


// memcpy() appears to move data by bytes, which doesn't work with the word-accessed backup registers
// here we explicitly move data a word at a time
void memcpyword(volatile uint32_t *dest, volatile uint32_t *src, size_t n){
  while (n--){
    dest[n] = src[n];
  }
}

void setNextTimestamp(time_t nextTime){

  int32_t offset = 0;
  for (uint8_t i=0; i< MAX_RULES; i++) {
    if (rules[i].t <= nextTime) offset=rules[i].offset;
    else break;
  }

  nextTime += offset;

  struct tm * nextTm = gmtime( &nextTime );

  tmToBcd( nextTm, &nextBcd );

  next7seg.c = cLut[nextBcd.seconds];

  next7seg.b[0] = bCat0 | cLut[nextBcd.tenHours]<<2;
  next7seg.b[1] = bCat1 | cLut[nextBcd.hours]<<2;
  next7seg.b[2] = bCat2 | cLut[nextBcd.tenMinutes]<<2;
  next7seg.b[3] = bCat3 | cLut[nextBcd.minutes]<<2;
  next7seg.b[4] = bCat4 | cLut[nextBcd.tenSeconds]<<2;

  if (1){
    uart2_tx_buffer[0] =0x90;
    uart2_tx_buffer[1] ='0'+nextBcd.seconds;
    uart2_tx_buffer[2] ='0';
    uart2_tx_buffer[3] ='0'+nextBcd.tenYears;
    uart2_tx_buffer[4] ='0'+nextBcd.years;
    uart2_tx_buffer[5] ='-';
    uart2_tx_buffer[6] ='0'+nextBcd.tenMonths;
    uart2_tx_buffer[7] ='0'+nextBcd.months;
    uart2_tx_buffer[8] ='-';
    uart2_tx_buffer[9] ='0'+nextBcd.tenDays;
    uart2_tx_buffer[10]='0'+nextBcd.days;
  }else if (1){
    sprintf(&uart2_tx_buffer[1], "%010ld", currentTime);
    uart2_tx_buffer[0] =0x90;
  }else{
    uart2_tx_buffer[0] =0x90;
    uart2_tx_buffer[1] ='2';
    uart2_tx_buffer[2] ='0';
    uart2_tx_buffer[3] ='0'+nextBcd.tenYears;
    uart2_tx_buffer[4] ='0'+nextBcd.years;
    uart2_tx_buffer[5] ='-';
    uart2_tx_buffer[6] ='0'+nextBcd.tenMonths;
    uart2_tx_buffer[7] ='0'+nextBcd.months;
    uart2_tx_buffer[8] ='-';
    uart2_tx_buffer[9] ='0'+nextBcd.tenDays;
    uart2_tx_buffer[10]='0'+nextBcd.days;
  }
  HAL_UART_AbortTransmit(&huart2);
  HAL_UART_Transmit_DMA(&huart2, uart2_tx_buffer, 11);
}

// Store UTC on RTC
// need to also write zone into backup registers
// Only called at the start of a second, don't attempt to write subseconds.
void write_rtc(void){

  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;
  bcdStamp_t cBcd;
  struct tm * cTm = gmtime( &currentTime );

  tmToBcd( cTm, &cBcd );

  sdatestructure.Year    = (cBcd.tenYears<<4)  | cBcd.years;
  sdatestructure.Month   = (cBcd.tenMonths<<4) | cBcd.months;
  sdatestructure.Date    = (cBcd.tenDays<<4)   | cBcd.days;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD);

  stimestructure.Hours          = (cBcd.tenHours<<4)  | cBcd.hours;
  stimestructure.Minutes        = (cBcd.tenMinutes<<4)  | cBcd.minutes;
  stimestructure.Seconds        = (cBcd.tenSeconds<<4)  | cBcd.seconds;
  stimestructure.SubSeconds     = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  HAL_RTC_SetTime(&hrtc,&stimestructure,RTC_FORMAT_BCD);

  // Write zone info to backup registers
  // There are 32 words of memory, 128 bytes
  // First 8 words are the zone string including separator and null byte (always less than 32 bytes)
  // Remaining 24 words is a chunk of the ruleset in use, i.e. 12 years

  uint8_t i;
  for (i=0; i< MAX_RULES; i++) {
    if (rules[i].t > currentTime) break;
  }
  if (i==0) return; //something has gone wrong, data invalid
  i--; //include currently active rule

  char numRulesToStore = (i+12>=MAX_RULES-1)? (MAX_RULES-i)*2 : 24;

  memcpyword( (uint32_t*)&(RTC->BKP0R), (uint32_t*)loadedRulesString, 8 );
  memcpyword( (uint32_t*)&(RTC->BKP8R), (uint32_t*)&rules[i], numRulesToStore );

}

time_t bcdToTm(bcdStamp_t *in, struct tm *out ) {
  out->tm_isdst = 0;
  out->tm_sec = in->seconds + in->tenSeconds*10;
  out->tm_min = in->minutes + in->tenMinutes*10;
  out->tm_hour = in->hours + in->tenHours*10;
  out->tm_mday = in->days + in->tenDays*10;
  out->tm_mon = in->months + in->tenMonths*10 -1;
  out->tm_year = in->years + in->tenYears*10 + 100; //Years since 1900

  return mktime(out);
}
void tmToBcd(struct tm *in, bcdStamp_t *out ) {
  out->tenYears   = (in->tm_year-100) / 10;
  out->years      = (in->tm_year-100) % 10;
  out->tenMonths  = (in->tm_mon+1) / 10;
  out->months     = (in->tm_mon+1) % 10;
  out->tenDays    = in->tm_mday / 10;
  out->days       = in->tm_mday % 10;
  out->tenHours   = in->tm_hour / 10;
  out->hours      = in->tm_hour % 10;
  out->tenMinutes = in->tm_min / 10;
  out->minutes    = in->tm_min % 10;
  out->tenSeconds = in->tm_sec / 10;
  out->seconds    = in->tm_sec % 10;
}

void decodeRMC(void){

  // do checksum
  uint8_t *c = &nmea[1], *end = &nmea[sizeof(nmea)];
  uint8_t sum=0;

  bcdStamp_t rmcBcd;
  struct tm rmcTm;

  while (*c !='*') {
    sum ^= *c;
    if (*c==',') *c=0;
    c++;
    if(c==end) return; //checksum not found
  }

  sprintf(nmea, "%02X", sum);
  if (nmea[0] != c[1] || nmea[1]!=c[2]) return; //checksum error

#define nextField() while (*c && c!=end) c++; c++;

  c=&nmea[7]; // Time

  if (*c==0) return; // time not present

  rmcBcd.tenHours   = *c++ -'0';
  rmcBcd.hours      = *c++ -'0';
  rmcBcd.tenMinutes = *c++ -'0';
  rmcBcd.minutes    = *c++ -'0';
  rmcBcd.tenSeconds = *c++ -'0';
  rmcBcd.seconds    = *c++ -'0';

  if (*c++ =='.') { // subseconds not always present
    if (*c!='0') printf("subseconds non-zero: %s\n", c);
  }
  nextField() // Navigation receiver warning
  data_valid = (*c=='A'?1:0);

  nextField() // Latitude deg
  if (*c){
    latitude =  (float)(*c++ -'0')*10.0;
    latitude += (float)(*c++ -'0');
    latitude += (float)atof(c) / 60.0;
  }
  nextField() // Latitude N/S
  if (*c =='S') latitude =-latitude;

  nextField() // Longitude  deg
  if (*c){
    longitude =  (float)(*c++ -'0')*100.0;
    longitude += (float)(*c++ -'0')*10.0;
    longitude += (float)(*c++ -'0');
    longitude += (float)atof(c) / 60.0;
  }
  nextField() // Longitude  E/W
  if (*c == 'W') longitude =-longitude;

  nextField() // Speed over ground, Knots
  nextField() // Course Made Good, True
  nextField() // Date

  if (*c==0) return; // date not present

  rmcBcd.tenDays    = *c++ -'0';
  rmcBcd.days       = *c++ -'0';
  rmcBcd.tenMonths  = *c++ -'0';
  rmcBcd.months     = *c++ -'0';
  rmcBcd.tenYears   = *c++ -'0';
  rmcBcd.years      = *c++ -'0';


  if ( data_valid || !had_pps ) {
    currentTime = bcdToTm( &rmcBcd, &rmcTm );

    if (decisec >= 9) {
      currentTime++;
      setNextTimestamp( currentTime );
    }
  }

}

void setDisplayPWM(uint32_t bright){
  HAL_DMA_Abort(&hdma_tim1_up);
  HAL_DMA_Abort(&hdma_tim7_up);
  HAL_DMA_Start(&hdma_tim1_up, (uint32_t)buffer_b, (uint32_t)&GPIOB->ODR, bright);
  HAL_DMA_Start(&hdma_tim7_up, (uint32_t)buffer_c, (uint32_t)&GPIOC->ODR, bright);
}

void parseConfigString(char const *key, char const *value) {

  if (strcasecmp(key, "brightness") == 0) {

    printf("found brightness\n");

  } else if (strcasecmp(key, "date_format") == 0) {

    printf("found date format\n");

  }


  printf("Parsed config key [%s] is value [%s]\n", key, value);
}

void readConfigFile(){

  FIL file;

   if (f_open(&file, "/CONFIG.TXT", FA_READ) != FR_OK)
     Error_Handler();

   char key[20], value[20], s[1];
   unsigned int rc;
   uint16_t col=0;


   while (1) {
     f_read(&file, s, 1, &rc);
     if (rc!=1) break; //EOF

     if (s[0]=='\r' || s[0]=='\n') { col=0; continue; } //EOL

     if (col==0 && (s[0]=='#' || s[0]==';')) { // comments
       while (rc && s[0]!='\n') f_read(&file, s, 1, &rc);
       continue;
     }

     if (s[0]!='=') {
       if (col<sizeof(key)-1 &&s[0]!=' ') key[col++] = s[0];
     } else {

       key[col]=0;

       col=0;
       while (s[0]!='\n') {
         f_read(&file, s, 1, &rc);
         if (rc!=1) break;
         if (col<sizeof(value)-1 &&s[0]!=' ' &&s[0]!='\r' &&s[0]!='\n') value[col++] = s[0];
       }
       value[col]=0;
       col=0;

       parseConfigString(key, value);

     }
   }

   f_close(&file);
}


void enablePPS(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

// PPS rising edge
void EXTI9_5_IRQHandler(void)
{
  SysTick->VAL = SysTick->LOAD;

  buffer_c[3].low=cLut[0];
  buffer_c[2].low=cLut[0];
  buffer_c[1].low=cLut[0];
  loadNextTimestamp();
  millisec=0;
  centisec=0;
  decisec=0;

  //int y =  SysTick->VAL;
  //printf("%d\n",x);

  // clear systick flag if set?

  had_pps = 1;
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);

}


void SysTick_CountUp(void)
{

  millisec++;
  if (millisec>=10) {
    millisec=0;
    centisec++;
    if (centisec>=10) {
      centisec=0;
      decisec++;
      if (decisec>=10) {
        decisec=0;

        loadNextTimestamp();

      }
    }
  }

  buffer_c[3].low=cLut[millisec];
  buffer_c[2].low=cLut[centisec];
  buffer_c[1].low=cLut[decisec];



  HAL_IncTick();

  // At the 0.900 mark, we calculate what the display should read at the next pulse
  if (decisec==9 && centisec==0 && millisec==0){
    // Calculating the next display from the unix timestamp takes about 32uS with -O2, -O3 or -Os
    // takes about 70uS on -O0 so I think it's fine to do this within systick
    // If needed, we should move this to a lower priority software-triggered interrupt
    currentTime++;
    setNextTimestamp( currentTime );
  }
}

void SysTick_CountDown(void)
{

  static int8_t i=9, j=9, k=9;

  i--;
  if (i<0) {
    i=9;
    j--;
    if (j<0) {
      j=9;
      k--;
      if (k<0) k=9;
    }
  }

  buffer_c[3].low=cLut[i];
  buffer_c[2].low=cLut[j];
  buffer_c[1].low=cLut[k];


  HAL_IncTick();

}

void SysTick_Dummy(void){
  HAL_IncTick();
}





uint8_t f_getzcmp(FIL* fp, char * str){
  unsigned int rc;
  char * a = str;
  char b[1] = {1};
  uint8_t ret = 0;

  while (b[0]!=0) {
    f_read(fp, &b, 1, &rc);
    if (b[0] != *a++) ret=-1;
  }
  return ret;
}
uint8_t findField( FIL* fp, char* str, uint8_t count, uint8_t padding ) {
  char buf[4];
  unsigned int rc;
  for (uint8_t i=0; i<count; i++) {
    if (f_getzcmp( fp, str ) ==0) return 1;
    f_read(fp, &buf, padding, &rc);
  }
  return 0;
}
uint8_t loadRules( char* cat, char* zo ) {
  FIL file;

  if (f_open(&file, "/TZRULES.BIN", FA_READ) != FR_OK) {
    //printf("Could not open rules file\n");
    return 1;
  }

  unsigned int rc;
  char buf[4];

  f_read(&file, &buf, 4, &rc);

  if(memcmp(&buf, "MTZ", 3)) {
    //printf("Error reading rules file\n");
    return 2;
  }
  if (buf[3]!=1) {
    //printf("version unknown\n");
    return 3;
  }

  uint8_t rowLength;
  f_read(&file, &rowLength, 1, &rc);

  uint8_t numCats;
  f_read(&file, &numCats, 1, &rc);

  if (findField( &file, cat, numCats, 3 ) ==0) {
    //printf("Could not find category\n");
    return 4;
  }
  uint16_t catAddr;
  f_read(&file, &catAddr, 2, &rc);

  uint8_t numZones;
  f_read(&file, &numZones, 1, &rc);

  //printf("Cat addr %04X, %d\n", catAddr, numZones);

  f_lseek(&file, catAddr);

  if (findField( &file, zo, numZones, 4 ) ==0) {
    //printf("Could not find zone\n");
    return 5;
  }

  //printf("found zone %04X\n", f_tell(&file));

  uint32_t zoAddr = 0;
  f_read(&file, &zoAddr, 3, &rc);

  uint8_t numEntries;
  f_read(&file, &numEntries, 1, &rc);

  //printf("zoAddr 0x%X\n",zoAddr);
  f_lseek(&file, zoAddr);

  int i;
  for (i=0;i<numEntries;i++) {
    f_read(&file, &rules[i], rowLength, &rc);
  }
  while (i< MAX_RULES ) {
    rules[i++].t=-1;
  }

  f_close(&file);

  return 0;
}

// loadRulesSingle modifies the input string, can't be used with const str
uint8_t loadRulesSingle(char * str){
  strcpy( loadedRulesString, str );

  char * zo = str;
  while (*zo && *zo != '/') zo++;
  *zo=0; zo++;
  return loadRules( str, zo );
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  extern uint32_t __VECTORS_FLASH[];
  extern uint32_t __VECTORS_RAM[];

  memcpy(__VECTORS_RAM, __VECTORS_FLASH, 0x188);
  SCB->VTOR = (uint32_t)&__VECTORS_RAM;

#define SetSysTick(x) __VECTORS_RAM[15] = (uint32_t)x

  SetSysTick( &SysTick_Dummy );


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  buffer_c[0].high=0b11011110;
  buffer_c[1].high=0b11011101;
  buffer_c[2].high=0b11011011;
  buffer_c[3].high=0b11010111;
  buffer_c[4].high=0b11001111;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_QUADSPI_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);


  // Configure display matrix
  if (HAL_DMA_Start(&hdma_tim1_up, (uint32_t)buffer_c, (uint32_t)&GPIOC->ODR, 5) != HAL_OK)
    Error_Handler();

  if (HAL_DMA_Start(&hdma_tim7_up, (uint32_t)buffer_b, (uint32_t)&GPIOB->ODR, 5) != HAL_OK)
    Error_Handler();

  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE(&htim1);

  __HAL_TIM_ENABLE_DMA(&htim7, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE(&htim7);


  doDateUpdate();


  enablePPS();

  // Configure UART1 for NMEA strings from GPS module
  USART1->CR1 |= USART_CR1_CMIE ;

  USART1->CR1 &= ~(USART_CR1_UE);
  USART1->CR2 |= '\n'<<24;
  USART1->CR1 |= USART_CR1_UE;

  HAL_UART_Receive_DMA(&huart1, nmea, sizeof(nmea));



  // Configure ADC and DAC DMA for display brightness
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_adc, ADC_BUFFER_SIZE) != HAL_OK)
    Error_Handler();

  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)buffer_dac, DAC_BUFFER_SIZE, DAC_ALIGN_12B_R) !=HAL_OK)
    Error_Handler();

  // Configure Colon Separators
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  TIM2->CCR1 = 10000-3500;

  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
  TIM2->CCR2 = 10000-3500;



  //Enable DP for subseconds
  buffer_c[0].high=0b11011110 | cSegDP;



  buffer_c[0].low=cSegDecode0;
  buffer_c[1].low=cSegDecode0;
  buffer_c[2].low=cSegDecode0;
  buffer_c[3].low=cSegDecode0;

  next7seg.c = buffer_c[0].low;

  next7seg.b[0] = buffer_b[0] = bCat0 | bSegDecode0;
  next7seg.b[1] = buffer_b[1] = bCat1 | bSegDecode0;
  next7seg.b[2] = buffer_b[2] = bCat2 | bSegDecode0;
  next7seg.b[3] = buffer_b[3] = bCat3 | bSegDecode0;
  next7seg.b[4] = buffer_b[4] = bCat4 | bSegDecode0;

  setDisplayPWM(5);

 // readConfigFile();









//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  HAL_UART_Init(&huart2);


//  char asdf[]="Europe/London";
//  loadRulesSingle(asdf);

//  while(1);


  if (RTC->ISR & RTC_ISR_INITS) //RTC contains non-zero data
  {
    RTC_DateTypeDef sdate;
    RTC_TimeTypeDef stime;

    char zone[32];
    memcpyword( (uint32_t*)zone,  (uint32_t*)&(RTC->BKP0R), 8 );

    if (loadRulesSingle(zone) != 0){ // takes 34ms -O0, 26ms -O2
      memcpyword( (uint32_t*)rules, (uint32_t*)&(RTC->BKP8R), 24 );
    }

    hrtc.Instance = RTC;
    HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);

    struct tm out;

    out.tm_isdst = 0;

    out.tm_sec = stime.Seconds;
    out.tm_min = stime.Minutes;
    out.tm_hour = stime.Hours;
    out.tm_mday = sdate.Date;
    out.tm_mon = sdate.Month -1;
    out.tm_year = sdate.Year + 100; //Years since 1900

    currentTime = mktime(&out);

    // subseconds ranges up to synchronous prescaler value of 255
    float fraction = (float)(255 - stime.SubSeconds) / 256.0;

    millisec = (uint32_t)(fraction*1000) % 10;
    centisec = (uint32_t)(fraction*100) % 10;
    decisec =  (uint32_t)(fraction*10) % 10;

    setNextTimestamp( currentTime );
    loadNextTimestamp();


  } else { // backup domain reset

    // The init process blanks the subsecond registers
    MX_RTC_Init();
  }

  SetSysTick( &SysTick_CountUp );


  while(1);

while (1){
  char s[10];
  printf("Enter target: ");
  scanf("%s", &s);

  dac_target=atof(s);
}


/*
  char cat[14], zo[29];

  temp:
  printf("Enter cat: ");
  scanf("%s", &cat);
  printf("Enter zone: ");
  scanf("%s", &zo);
  printf("%s\n",&zo);
*/
/*
  temp:
  printf("Enter tz: ");
  char str[48];
  scanf("%s", &str);

  printf("loading.. %d\n", loadRulesSingle(&str) );
  printf("rule 0: %lu, %d\n", rules[0].t, rules[0].offset);
  printf("rule 11: %lu, %d\n", rules[11].t, rules[11].offset);
  printf("rule 92: %lu, %d\n", rules[92].t, rules[11].offset);


goto temp;
*/
  //int dac = 0;
  while (1){

    TIM2->CCR1 = 5000;

    //HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac);

    //if (++dac>4095) dac=0;

    HAL_Delay(1000);
    TIM2->CCR1 = 10000;


    HAL_Delay(1000);
  }



  FIL file;

  if (f_open(&file, "/TZMAP.BIN", FA_READ) != FR_OK) {
    printf("Could not open tzmap\n");
    Error_Handler();
  }
  ZoneDetect *const cd = ZDOpenDatabase(&file);







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*
    char str[20];

    float bri;

    printf("Enter brightness: ");
    scanf("%s", &str);
    printf("%s\n",&str);
    bri = (float)atof(str);
    setDisplayPWM((int)bri);

*/

    if (data_valid && latitude>=-90.0 && latitude<=90.0 && longitude>=-180.0 && longitude<=180.0) {

      uint32_t start = HAL_GetTick();
      char* zone = ZDHelperSimpleLookupString(cd, latitude, longitude);

      printf("IANA Timezone is [%s]\n", zone);
      printf("Took %lu ms\n", (HAL_GetTick()-start));

      loadRulesSingle(zone);
      free(zone);
      HAL_Delay(2000);
    }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_CC2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,  DAC_ALIGN_12B_R, 4095);
  /* USER CODE END DAC1_Init 2 */

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
  hqspi.Init.FifoThreshold = 4;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 256;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 
                           PC3 PC4 PC5 PC6 
                           PC8 PC9 PC10 PC11 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
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

  __disable_irq();

  buffer_c[0].high=0b11011110;
  buffer_c[1].high=0b11011101;
  buffer_c[2].high=0b11011011;
  buffer_c[3].high=0b11010111;
  buffer_c[4].high=0b11001111;
  buffer_c[0].low=0b01010000;
  buffer_c[1].low=0b01010000;
  buffer_c[2].low=0b01011100;
  buffer_c[3].low=0b01010000;
  buffer_c[4].low=0;

  buffer_b[0] = bCat0;
  buffer_b[1] = bCat1;
  buffer_b[2] = bCat2;
  buffer_b[3] = bCat3;
  buffer_b[4] = bCat4 | 0b0111100100;

  //setDisplayPWM(5);




  while(1);
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
