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
#include <math.h>
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

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim5_ch2;
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
static void MX_LPTIM1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void tmToBcd(struct tm *in, bcdStamp_t *out );
uint8_t loadRulesSingle(char * str);
void nextMode(_Bool);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t cLut[]= { cSegDecode0, cSegDecode1, cSegDecode2, cSegDecode3, cSegDecode4, cSegDecode5, cSegDecode6, cSegDecode7, cSegDecode8, cSegDecode9 };
const uint16_t bLut[]={ bSegDecode0, bSegDecode1, bSegDecode2, bSegDecode3, bSegDecode4, bSegDecode5, bSegDecode6, bSegDecode7, bSegDecode8, bSegDecode9 };

buffer_c_t buffer_c[80] = {0};

uint16_t buffer_b[80] = {0};

uint8_t uart2_tx_buffer[32];

volatile uint16_t buffer_adc[ADC_BUFFER_SIZE] = {0};
uint16_t buffer_dac[DAC_BUFFER_SIZE] = {[0 ... DAC_BUFFER_SIZE-1] = 4095};
float dac_target=4095;

uint16_t buffer_colons_L[200] = {0};
uint16_t buffer_colons_R[200] = {0};

uint8_t nmea[NMEA_BUF_SIZE];
uint8_t GPS_sv = 255, GLONASS_sv = 255;

time_t currentTime;
bcdStamp_t nextBcd;
int tm_yday;
int iso_year;
int8_t wday;
uint8_t iso_week;
uint32_t countdown_days;
int32_t currentOffset=0;

struct {
  uint8_t c;
  uint16_t b[5];
} next7seg;

uint8_t decisec=0, centisec=0, millisec=0;

float longitude=-9999, latitude=-9999;
_Bool data_valid=0, had_pps=0, rtc_good=0;
#define rtc_last_write RTC->BKP30R
#define rtc_last_calibration RTC->BKP31R
uint32_t last_pps_time = 0;

struct {
  uint32_t t;
  int32_t offset;
} rules[162];
#define MAX_RULES (sizeof rules / sizeof rules[0])

char loadedRulesString[32];
char textDisplay[32];

uint32_t LPTIM1_high;

uint8_t displayMode = 0, countMode = 0, colonMode = 0;
uint8_t requestMode = 255;
uint8_t nmea_cdc_level=0;
int debug_rtc_val = 0;

struct {
  uint32_t tolerance_1ms;
  uint32_t tolerance_10ms;
  uint32_t tolerance_100ms;
  time_t countdown_to;
  float brightness_override;
  _Bool zone_override;
  _Bool modes_enabled[NUM_DISPLAY_MODES];

} config = {0};

struct {
  float in;
  float out;
} brightnessCurve[] = {
    {0,    4095-0},
    {1638, 4095-410},
    {2867, 4095-1229},
    {3686, 4095-2457},
    {4095, 4095-4095},
};

// memcpy() appears to move data by bytes, which doesn't work with the word-accessed backup registers
// here we explicitly move data a word at a time
void memcpyword(volatile uint32_t *dest, volatile uint32_t *src, size_t n){
  while (n--){
    dest[n] = src[n];
  }
}

void sendDate( _Bool now ){
  uint8_t i = 10;
  uart2_tx_buffer[0] = CMD_LOAD_TEXT;

  switch (displayMode) {
  default:
  case MODE_ISO8601_STD:
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
    break;
  case MODE_ISO_ORDINAL:
    uart2_tx_buffer[1] ='2' ;//-2+nextBcd.seconds;
    uart2_tx_buffer[2] ='0';
    uart2_tx_buffer[3] ='0'+nextBcd.tenYears;
    uart2_tx_buffer[4] ='0'+nextBcd.years;
    uart2_tx_buffer[5] ='-';
    i = 5 + sprintf((char*)&uart2_tx_buffer[6], "%d", tm_yday+1);
    break;
  case MODE_ISO_WEEK:
    i = sprintf((char*)&uart2_tx_buffer[1], "%d-W%d-%d", iso_year, iso_week, wday+1);
    break;
  case MODE_UNIX:
    i = sprintf((char*)&uart2_tx_buffer[1], "%010ld", (uint32_t)currentTime);
    break;
  case MODE_JULIAN_DATE:
    i = sprintf((char*)&uart2_tx_buffer[1], "%10f", (double)currentTime/86400.0 + 2440587.5 );
    break;
  case MODE_MODIFIED_JD:
    i = sprintf((char*)&uart2_tx_buffer[1], "%10f", (double)currentTime/86400.0 + 40587);
    break;
  case MODE_SHOW_OFFSET:
    // This probably isn't the best place to do it, but the data is static anyway

    if (currentOffset<0){
      buffer_b[0]=bCat0 | 0b0000000000;
      buffer_b[1]=bCat1 | 0b0100000000;
    } else {
      buffer_b[0]=bCat0 | 0b0100011000;
      buffer_b[1]=bCat1 | 0b0111000000;
    }
    int minutes = ((abs(currentOffset)/60) %60);
    int hours = (abs(currentOffset)/3600);

    buffer_b[2]=bCat2 | bLut[ hours/10 ];
    buffer_b[3]=bCat3 | bLut[ hours%10 ];
    buffer_b[4]=bCat4 | bLut[ minutes/10 ];

    buffer_c[0].low= cLut[ minutes%10 ];
    buffer_c[0].high=0b11001110;
    buffer_c[1].low=0;
    buffer_c[2].low=0;
    buffer_c[3].low=0;

    uart2_tx_buffer[1] ='u';
    uart2_tx_buffer[2] ='t';
    uart2_tx_buffer[3] ='c';
    uart2_tx_buffer[4] =' ';
    uart2_tx_buffer[5] ='o';
    uart2_tx_buffer[6] ='f';
    uart2_tx_buffer[7] ='f';
    uart2_tx_buffer[8] ='s';
    uart2_tx_buffer[9] ='e';
    uart2_tx_buffer[10]='t';
    break;
  case MODE_SHOW_TZ_NAME:
    if (loadedRulesString[0]) {
      char * zo = loadedRulesString;
      while (*zo && *zo != '/') zo++;
      if (currentTime%4 <2) {
        zo++;
        i = snprintf((char*)&uart2_tx_buffer[1], 11,"%s", zo);
      } else {
        i = zo-loadedRulesString;
        if (i>10) i=10;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
        snprintf((char*)&uart2_tx_buffer[1], i+1,"%s", loadedRulesString);
#pragma GCC diagnostic pop
      }
    } else {
      uart2_tx_buffer[1]='-';
      i=1;
    }
    break;
  case MODE_SATVIEW:
    if (GPS_sv==255) {
      i = sprintf((char*)&uart2_tx_buffer[1], "GPS - L-");
    } else if (GLONASS_sv==255) {
      i = sprintf((char*)&uart2_tx_buffer[1], "GPS %d. L-", GPS_sv);
    } else {
      i = sprintf((char*)&uart2_tx_buffer[1], "GPS %d. L%d", GPS_sv, GLONASS_sv);
    }
    break;
  case MODE_STANDBY:
     return;
  case MODE_COUNTDOWN:
    i = sprintf((char*)&uart2_tx_buffer[1], "t-%7ldd", countdown_days);
    break;
  case MODE_DEBUG_BRIGHTNESS:
    i = sprintf((char*)&uart2_tx_buffer[1], "%04d %04d", (int)ADC1->DR, 4095-(int)dac_target);
    break;
  case MODE_DEBUG_RTC:
    i = sprintf((char*)&uart2_tx_buffer[1], "rtc %d", debug_rtc_val);
    break;
  case MODE_TEXT:
    if (textDisplay[0]) {
      i = snprintf((char*)&uart2_tx_buffer[1], 30,"%s", textDisplay);
    } else {
      uart2_tx_buffer[1]='-';
      i=1;
    }
    break;
  }
  uart2_tx_buffer[++i]= now ? CMD_RELOAD_TEXT : '\n';
  HAL_UART_AbortTransmit(&huart2);
  HAL_UART_Transmit_DMA(&huart2, uart2_tx_buffer, i+1);

}

void setNextTimestamp(time_t nextTime){

  int32_t offset = 0;
  for (uint8_t i=0; i< MAX_RULES; i++) {
    if (rules[i].t <= nextTime) offset=rules[i].offset;
    else break;
  }
  // in case of the remote chance that we're interrupted while calculating,
  // don't assign to currentOffset until the end of the loop
  currentOffset = offset;
  nextTime += offset;

  struct tm * nextTm = gmtime( &nextTime );
  tmToBcd( nextTm, &nextBcd );
  tm_yday = nextTm->tm_yday;

  if (displayMode == MODE_ISO_WEEK){
    wday = (nextTm->tm_wday + 6) % 7;
    nextTm->tm_mday -= wday -3;
    mktime(nextTm);
    iso_year = nextTm->tm_year + 1900;
    iso_week = nextTm->tm_yday/7 + 1;
  }

  next7seg.c = cLut[nextBcd.seconds];

  next7seg.b[0] = bCat0 | cLut[nextBcd.tenHours]<<2;
  next7seg.b[1] = bCat1 | cLut[nextBcd.hours]<<2;
  next7seg.b[2] = bCat2 | cLut[nextBcd.tenMinutes]<<2;
  next7seg.b[3] = bCat3 | cLut[nextBcd.minutes]<<2;
  next7seg.b[4] = bCat4 | cLut[nextBcd.tenSeconds]<<2;

}

void setNextCountdown(time_t nextTime){

  int64_t remaining;
  if (config.countdown_to < nextTime) {
    remaining = 0;
    SetPPS( &PPS_NoUpdate ); // don't show 999 at the next pulse

  } else remaining = config.countdown_to - nextTime;

  uint64_t seconds = remaining % 60;
  uint64_t minutes = remaining / 60;
  uint64_t hours =   minutes / 60;
  minutes %= 60;
  countdown_days = hours / 24;
  hours %= 24;

  next7seg.b[0] = bCat0 | cLut[hours / 10]<<2;
  next7seg.b[1] = bCat1 | cLut[hours % 10]<<2;
  next7seg.b[2] = bCat2 | cLut[minutes / 10]<<2;
  next7seg.b[3] = bCat3 | cLut[minutes % 10]<<2;
  next7seg.b[4] = bCat4 | cLut[seconds / 10]<<2;
  next7seg.c = cLut[seconds % 10];
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
  // Next 22 words is a chunk of the ruleset in use, i.e. 11 years
  // Last two words are time of write, and time of last calibration

  uint8_t i;
  for (i=0; i< MAX_RULES; i++) {
    if (rules[i].t > currentTime) break;
  }
  if (i==0) return; //something has gone wrong, data invalid
  i--; //include currently active rule

  char numRulesToStore = (i+11>=MAX_RULES-1)? (MAX_RULES-i)*2 : 22;

  memcpyword( (uint32_t*)&(RTC->BKP0R), (uint32_t*)loadedRulesString, 8 );
  memcpyword( (uint32_t*)&(RTC->BKP8R), (uint32_t*)&rules[i], numRulesToStore );

  rtc_last_write = (uint32_t)currentTime;
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

  sprintf((char*)nmea, "%02X", sum);
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
    //if (*c!='0') printf("subseconds non-zero: %s\n", c);
  }
  nextField() // Navigation receiver warning
  data_valid = (*c=='A'?1:0);

  nextField() // Latitude deg
  if (*c){
    latitude =  (float)(*c++ -'0')*10.0;
    latitude += (float)(*c++ -'0');
    latitude += (float)atof((char*)c) / 60.0;
  }
  nextField() // Latitude N/S
  if (*c =='S') latitude =-latitude;

  nextField() // Longitude  deg
  if (*c){
    longitude =  (float)(*c++ -'0')*100.0;
    longitude += (float)(*c++ -'0')*10.0;
    longitude += (float)(*c++ -'0');
    longitude += (float)atof((char*)c) / 60.0;
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


  // Immediately after power-up, the GPS module does not know the GPS time/UTC leapsecond offset, and makes a guess
  // Even if it gets a fix and starts outputting PPS, the time can be off by a few seconds (usually 2 or 3 fast)
  // Only make use of this invalid data if there is nothing else to go on
  if ( data_valid || (!had_pps && !rtc_good) ) {
    currentTime = bcdToTm( &rmcBcd, &rmcTm );

    if (decisec >= 9) {
      currentTime++;
      // todo: check we're not <2ms away from rollover
      // Under normal conditions, we should only be parsing nmea at around .300 to .400
      // USART1 preemption priority is currently 1, so we could be interrupted by systick here
      setNextTimestamp( currentTime );
      sendDate(0);
    }
  }

}

void decodeGSV(void){
  uint8_t sv = (nmea[11]-'0')*10 + (nmea[12]-'0');
  if (nmea[2] == 'P') {
      GPS_sv = sv;
  } else if (nmea[2] == 'L') {
      GLONASS_sv = sv;
  }
}

void setDisplayPWM(uint32_t bright){
  HAL_DMA_Abort(&hdma_tim1_up);
  HAL_DMA_Abort(&hdma_tim7_up);
  HAL_DMA_Start(&hdma_tim1_up, (uint32_t)buffer_b, (uint32_t)&GPIOB->ODR, bright);
  HAL_DMA_Start(&hdma_tim7_up, (uint32_t)buffer_c, (uint32_t)&GPIOC->ODR, bright);
}

void displayOff(void){

  uart2_tx_buffer[0]=' '; //in case already waiting for latch
  uart2_tx_buffer[1]= CMD_LOAD_TEXT;
  uart2_tx_buffer[2]= CMD_RELOAD_TEXT;
  HAL_UART_AbortTransmit(&huart2);
  HAL_UART_Transmit_DMA(&huart2, uart2_tx_buffer, 3);

  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

  HAL_DMA_Abort(&hdma_tim1_up);
  HAL_DMA_Abort(&hdma_tim7_up);
  GPIOB->ODR=0;
  GPIOC->ODR=0;
}
void displayOn(void){
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  setDisplayPWM(5);
}

void setDisplayFreq(uint32_t freq){

  if (freq<1000 || freq>100000) return;

  uart2_tx_buffer[0]= CMD_SET_FREQUENCY;
  uart2_tx_buffer[1]= (freq>>14) & 0x7F;
  uart2_tx_buffer[2]= (freq>>7)  & 0x7F;
  uart2_tx_buffer[3]= (freq)     & 0x7F;
  HAL_UART_Transmit(&huart2, uart2_tx_buffer, 4, 2);

  uint32_t arr = round(16000000.0 / (float)freq) -1.0;

  TIM1->ARR = arr;
  TIM7->ARR = arr;
}

#define colonAnimationStart() \
  TIM5->CNT=0; \
  HAL_DMA_Start(&hdma_tim5_ch1, (uint32_t)buffer_colons_L, (uint32_t)&TIM2->CCR1, 200); \
  HAL_DMA_Start(&hdma_tim5_ch2, (uint32_t)buffer_colons_R, (uint32_t)&TIM2->CCR2, 200);

#define colonAnimationStop() \
  HAL_DMA_Abort(&hdma_tim5_ch1); \
  HAL_DMA_Abort(&hdma_tim5_ch2);

#define colonAnimationSync() \
  colonAnimationStop() \
  colonAnimationStart()

void loadColonAnimation(void){


  switch (colonMode) {
    case COLON_MODE_SLOWFADE:
      for (int k=0;k<100;k++) {
        buffer_colons_R[k] =
        buffer_colons_L[k] = k*2;
        buffer_colons_R[k+100] =
        buffer_colons_L[k+100] = 198-k*2;
      }
      break;
    case COLON_MODE_HEARTBEAT:
      for (int k=0;k<50;k++) {
        buffer_colons_L[k] = k*4;
      }
      for (int k=0;k<100;k++) {
        buffer_colons_L[k+50] = 200 - k*2;
      }
      for (int k=0;k<50;k++) {
        buffer_colons_L[k+150] = 0;
      }
      for (int k=0;k<200;k++) {
        buffer_colons_R[k] = buffer_colons_L[(k+175)%200];
      }

      break;
    case COLON_MODE_1PPS_SAWTOOTH:
      for (int k=0;k<100;k++) {
        buffer_colons_R[k] =
        buffer_colons_L[k] = 196-(k*k)/50;
        buffer_colons_R[k+100] =
        buffer_colons_L[k+100] = 196-(k*k)/50;
      }
      break;
    case COLON_MODE_ALT_SAWTOOTH:
      for (int k=0;k<100;k++) {
        buffer_colons_R[k]     = 0;
        buffer_colons_L[k+100] = 0;
        buffer_colons_L[k]     = 196-(k*k)/50;
        buffer_colons_R[k+100] = 196-(k*k)/50;
      }
      break;
    case COLON_MODE_TOGGLE:
      for (int k=0;k<100;k++) {
        buffer_colons_R[k] = 200;
        buffer_colons_L[k] = 200;
        buffer_colons_R[k+100] = 0;
        buffer_colons_L[k+100] = 0;
      }
      break;
    case COLON_MODE_SOLID:
      for (int k=0;k<200;k++) {
        buffer_colons_R[k] = 200;
        buffer_colons_L[k] = 200;
      }
      break;
  }

}

_Bool truthy(char const* str){
  if (strcasecmp(str, "on")==0) return 1;
  if (strcasecmp(str, "enabled")==0) return 1;
  if (strcasecmp(str, "1")==0) return 1;
  return 0;
}

_Bool falsey(char const* str){
  if (strcasecmp(str, "off")==0) return 1;
  if (strcasecmp(str, "disabled")==0) return 1;
  if (strcasecmp(str, "0")==0) return 1;
  if (strcasecmp(str, "none")==0) return 1;
  return 0;
}

// Accept a float between 0.0 and 1.0, or an int from 0 to 4096
float parseBrightness(char *v, _Bool invert){
  if (!v[0]) return -1;
  float b = strtof(v, NULL);
  if (!isfinite(b) || b<0.0) return -1;
  if (b<=1.0 && v[1]=='.')
    return invert? (1.0-b) * 4095 : b*4095;
  if (b<=4095)
    return invert? 4095-b : b;
  return -1;
}

#define set_mode_enabled(mode, value) \
  if ((config.modes_enabled[mode] = truthy(value))) requestMode=mode;

void parseConfigString(char *key, char *value) {

  if (strcasecmp(key, "text") == 0) {

     strcpy(textDisplay, value);

  } else if (strcasecmp(key, "MATRIX_FREQUENCY") == 0) {

    //todo: check not waiting for latch
    setDisplayFreq(atoi(value));

  } else if (strcasecmp(key, "zone_override") == 0) {

    if (!value[0]) return;
    config.zone_override = 1;
    if (loadRulesSingle(value) !=0) {
      config.zone_override = 0;
    }

  } else if (strcasecmp(key, "brightness") == 0) {

    config.brightness_override = parseBrightness(value, 1);

  } else if (strcasecmp(key, "countdown_to") == 0) {

    //  support fractional seconds??
    struct tm t = {0};
    if( sscanf(value, "%d-%d-%dT%d:%d:%dZ", &t.tm_year, &t.tm_mon, &t.tm_mday, &t.tm_hour, &t.tm_min, &t.tm_sec) >=3) {

      if (t.tm_year > 9999) return; // arbitrary cutoff, ~3e6 days
      t.tm_year -= 1900;
      t.tm_mon -= 1;

      config.countdown_to = mktime(&t) -1;

    }
  } else if (strcasecmp(key, "MODE_ISO8601_STD") == 0) {
    set_mode_enabled(MODE_ISO8601_STD, value);
  } else if (strcasecmp(key, "MODE_ISO_ORDINAL") == 0) {
    set_mode_enabled(MODE_ISO_ORDINAL, value);
  } else if (strcasecmp(key, "MODE_ISO_WEEK") == 0) {
    set_mode_enabled(MODE_ISO_WEEK, value);
  } else if (strcasecmp(key, "MODE_UNIX") == 0) {
    set_mode_enabled(MODE_UNIX, value);
  } else if (strcasecmp(key, "MODE_JULIAN_DATE") == 0) {
    set_mode_enabled(MODE_JULIAN_DATE, value);
  } else if (strcasecmp(key, "MODE_MODIFIED_JD") == 0) {
    set_mode_enabled(MODE_MODIFIED_JD, value);
  } else if (strcasecmp(key, "MODE_SHOW_OFFSET") == 0) {
    set_mode_enabled(MODE_SHOW_OFFSET, value);
  } else if (strcasecmp(key, "MODE_SHOW_TZ_NAME") == 0) {
    set_mode_enabled(MODE_SHOW_TZ_NAME, value);
  } else if (strcasecmp(key, "MODE_STANDBY") == 0) {
    set_mode_enabled(MODE_STANDBY, value);
  } else if (strcasecmp(key, "MODE_COUNTDOWN") == 0) {
    set_mode_enabled(MODE_COUNTDOWN, value);
  } else if (strcasecmp(key, "MODE_SATVIEW") == 0) {
    set_mode_enabled(MODE_SATVIEW, value);
  } else if (strcasecmp(key, "MODE_DEBUG_BRIGHTNESS") == 0) {
    set_mode_enabled(MODE_DEBUG_BRIGHTNESS, value);
  } else if (strcasecmp(key, "MODE_DEBUG_RTC") == 0) {
    set_mode_enabled(MODE_DEBUG_RTC, value);
  } else if (strcasecmp(key, "MODE_TEXT") == 0) {
    set_mode_enabled(MODE_TEXT, value);
  } else if (strcasecmp(key, "Tolerance_time_1ms") == 0) {
    config.tolerance_1ms = atoi(value);
  } else if (strcasecmp(key, "Tolerance_time_10ms") == 0) {
    config.tolerance_10ms = atoi(value);
  } else if (strcasecmp(key, "Tolerance_time_100ms") == 0) {
    config.tolerance_100ms = atoi(value);
  } else if (strcasecmp(key, "colon_mode") == 0) {

    if (strcasecmp(value, "solid") == 0) {
      colonMode = COLON_MODE_SOLID;
    } else if (strcasecmp(value, "heartbeat") == 0) {
      colonMode = COLON_MODE_HEARTBEAT;
    } else if (strcasecmp(value, "sawtooth") == 0) {
      colonMode = COLON_MODE_1PPS_SAWTOOTH;
    } else if (strcasecmp(value, "alt_sawtooth") == 0) {
      colonMode = COLON_MODE_ALT_SAWTOOTH;
    } else if (strcasecmp(value, "toggle") == 0) {
      colonMode = COLON_MODE_TOGGLE;
    } else colonMode = COLON_MODE_SLOWFADE;

  } else if (strcasecmp(key, "nmea") == 0) {

    if (falsey(value)) {
      nmea_cdc_level = NMEA_NONE;
    } else if (strcasecmp(value, "rmc") == 0) {
      nmea_cdc_level = NMEA_RMC;
    } else nmea_cdc_level = NMEA_ALL;

  } else if (key[0]=='B' && key[1]=='S' && key[3]==0) { //BS1, BS2, etc
    if (!key[2] || key[2]<'1' || key[2]>'0'+sizeof(brightnessCurve)/sizeof(brightnessCurve[0])) return;

    char *c = &value[0];
    while (*c++) if(*c==',') break;
    if (*c==0) return;
    *c=0; c++;

    float in  = parseBrightness(value,0);
    float out = parseBrightness(c,1);
    if (in<0 || out<0) return;

    brightnessCurve[key[2]-'1'].in = in;
    brightnessCurve[key[2]-'1'].out = out;

  }

}

void postConfigCleanup(void){
  loadColonAnimation();

  // check at least one mode is enabled
  uint8_t j = 0;
  for (uint8_t i=0; i<NUM_DISPLAY_MODES; i++)
    j+= config.modes_enabled[i];

  if (!j || (j==1 && config.modes_enabled[MODE_STANDBY])) config.modes_enabled[MODE_ISO8601_STD]=1;
  if (!config.modes_enabled[displayMode] || requestMode!=255) nextMode(0);

  // check tolerances
  if (config.tolerance_1ms == 0)   config.tolerance_1ms   = 0xFFFFFFFF;
  if (config.tolerance_10ms == 0)  config.tolerance_10ms  = 0xFFFFFFFF;
  if (config.tolerance_100ms == 0) config.tolerance_100ms = 0xFFFFFFFF;

  if (displayMode == MODE_COUNTDOWN) {
    setNextCountdown(currentTime);
    setPrecision();
    latchSegments();

    if (config.countdown_to < currentTime || decisec!=9 || centisec!=9 || millisec<7)
      sendDate(1);
  } else if (displayMode == MODE_TEXT) {
    if (decisec!=9 || centisec!=9 || millisec<7)
      sendDate(1);
  }
}

void rxConfigString(char c){
  static char key[32], value[32];
  static uint8_t k=0, v=0, state=0;

  if (c=='\n' || c=='\r') {
    if (k && (v || state>=2)) {
      value[v]=0;
      key[k]=0;
      parseConfigString(key, value);
      postConfigCleanup();
    }
    k=0;
    v=0;
    state=0;
    return;
  }

  switch (state) {
  case 0: // read key
    if (k) {
     if (c=='=') {state =2; break;}
     if (c==' ' || c=='\t') {state =1; break;}
    }
    key[k++] = c;
    if (k==31) k--;
    break;
  case 1: // whitespace
    if (c=='=') state=2;
    else if (c!=' ' && c!='\t') {state=0; k=0; key[k++]=c;}
    break;
  case 2: //second whitespace
    if (c!=' ' && c!='\t' && c!='=') {state=3; value[v++]=c;}
    break;
  case 3:
    value[v++]=c;
    if (v==31) v--;
  }
}

void readConfigFile(void){

  config.tolerance_1ms   = 1000;
  config.tolerance_10ms  = 10000;
  config.tolerance_100ms = 100000;
  config.zone_override = 0;
  config.brightness_override = -1.0;
  colonMode = 0;

  FIL file;

   if (f_open(&file, CONFIG_FILENAME, FA_READ) != FR_OK) {
     postConfigCleanup();
     return;
   }

   char key[32], value[32], s[1];
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

   requestMode=255;
   postConfigCleanup();
}

void calibrateRTC(void){
  // Called by PPS EXTI

  static uint32_t calibStart =0;
  // LPTIM period is 2 seconds
  // If calibrated well the overflow interrupt should collide with this one
  // Pick an odd number of seconds to calibrate against
#define CAL_PERIOD 63

  // No clear documentation on this but experimentally it appears to be 3 LSE cycles
#define LPTIM_START_DELAY 3

  if (currentTime - calibStart > CAL_PERIOD)  {

    LPTIM1_high=0;
    LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
    calibStart = currentTime;

  } else if ((uint32_t)currentTime - calibStart == CAL_PERIOD) {
    volatile uint16_t x = LPTIM1->CNT;
    volatile uint16_t y = LPTIM1->CNT;
    if (x!=y) goto skipRtcCal;

    int32_t error = ((LPTIM1_high<<16) + x) - 32768*CAL_PERIOD + LPTIM_START_DELAY;
    float e = (float)error * 32.0 / CAL_PERIOD;

    debug_rtc_val = error;//0x100 + round(e);

    if (e>255.0 || e< -255.0) goto skipRtcCal;

    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    RTC->CALR = 0x100 + (int)round(e);
    __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
    rtc_last_calibration = (uint32_t)currentTime;

skipRtcCal:
    // Prepare the counter for the next calibration
    // LPTIM1->CNT is read only, the only way to zero it is to disable and re-enable the timer.
    // There is a further delay associated with this, better to put it here than right at the moment we want to start the timer.
    LPTIM1->CR &= ~LPTIM_CR_ENABLE;
    LPTIM1->CR |= LPTIM_CR_ENABLE;
    LL_LPTIM_SetAutoReload(LPTIM1, 0xFFFF);
    LL_LPTIM_ClearFLAG_ARRM(LPTIM1); // just in case there's one pending
  }
}

void EXTI9_5_IRQHandler(void){__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);}

// PPS rising edge
void PPS(void)
{
  SysTick->VAL = SysTick->LOAD;

  buffer_c[3].low=cLut[0];
  buffer_c[2].low=cLut[0];
  buffer_c[1].low=cLut[0];
  loadNextTimestamp();
  millisec=0;
  centisec=0;
  decisec=0;

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);

  // clear systick flag if set?

  // During first power up PPS can be emitted before the GPS leapsecond offset is known
  // In this case, it is safest to pretend PPS hasn't happened
  if (!data_valid) return;

  calibrateRTC();

  if ((currentTime & 1) ==0) {colonAnimationSync()}

  had_pps = 1;
  last_pps_time = (uint32_t)currentTime;
}

void PPS_NoUpdate(void)
{
  SysTick->VAL = SysTick->LOAD;
  triggerPendSV();

  millisec=0;
  centisec=0;
  decisec=0;

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);

  if (!data_valid) return;

  calibrateRTC();

  had_pps = 1;
  last_pps_time = (uint32_t)currentTime;
}

void PPS_Countdown(void)
{
  SysTick->VAL = SysTick->LOAD;

  buffer_c[3].low=cLut[9];
  buffer_c[2].low=cLut[9];
  buffer_c[1].low=cLut[9];
  loadNextTimestamp();
  millisec=0;
  centisec=0;
  decisec=0;

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);

  if (!data_valid) return;
  calibrateRTC();
  if ((currentTime & 1) ==0) {colonAnimationSync()}

  had_pps = 1;
  last_pps_time = (uint32_t)currentTime;
}

void PPS_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  SetPPS( &PPS );
}

#define timetick() \
    millisec++; \
    if (millisec>=10) { \
      millisec=0; \
      centisec++; \
      if (centisec>=10) { \
        centisec=0; \
        decisec++; \
        if (decisec>=10) { \
          decisec=0; \
          loadNextTimestamp(); \
        } \
      } \
    }

void SysTick_CountUp_P3(void)
{
  timetick()

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
    sendDate(0);
  }
}

void SysTick_CountUp_P2(void) {
  timetick()

  buffer_c[2].low=cLut[centisec];
  buffer_c[1].low=cLut[decisec];

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextTimestamp( currentTime );
    sendDate(0);
  }
}
void SysTick_CountUp_P1(void) {

  timetick()

  buffer_c[1].low=cLut[decisec];

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextTimestamp( currentTime );
    sendDate(0);
  }
}

void SysTick_CountUp_P0(void) {

  timetick()

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextTimestamp( currentTime );
    sendDate(0);
  }
}

void SysTick_CountUp_NoUpdate(void) {
  millisec++;
  if (millisec>=10) {
    millisec=0;
    centisec++;
    if (centisec>=10) {
      centisec=0;
      decisec++;
      if (decisec>=10) {
        decisec=0;
        // write_rtc still needs to happen
        triggerPendSV();
      }
    }
  }

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextTimestamp( currentTime );
    //sendDate(0);
  }
}


void SysTick_CountDown_P3(void)
{
  timetick()

  buffer_c[3].low=cLut[9-millisec];
  buffer_c[2].low=cLut[9-centisec];
  buffer_c[1].low=cLut[9-decisec];


  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextCountdown( currentTime );
    sendDate(0);
  }
}

void SysTick_CountDown_P2(void)
{
  timetick()

  //buffer_c[3].low=cLut[9-millisec];
  buffer_c[2].low=cLut[9-centisec];
  buffer_c[1].low=cLut[9-decisec];


  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextCountdown( currentTime );
    sendDate(0);
  }
}

void SysTick_CountDown_P1(void)
{
  timetick()

  //buffer_c[3].low=cLut[9-millisec];
  //buffer_c[2].low=cLut[9-centisec];
  buffer_c[1].low=cLut[9-decisec];


  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextCountdown( currentTime );
    sendDate(0);
  }
}

// A no precision countdown is going to be really ambiguous, as it will hit zero a second before the target
// Then again it will only be used in situations where the tolerance is worse than a second
void SysTick_CountDown_P0(void)
{
  timetick()

  //buffer_c[3].low=cLut[9-millisec];
  //buffer_c[2].low=cLut[9-centisec];
  //buffer_c[1].low=cLut[9-decisec];

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextCountdown( currentTime );
    sendDate(0);
  }
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

  if (f_open(&file, RULES_FILENAME, FA_READ) != FR_OK) {
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

void setPrecision(void){
  if (countMode == COUNT_NORMAL) {

    // situations not covered:
    // - short poweroff - not had pps, but RTC calibrated only seconds ago
    // - last pps more than 100000 seconds ago (27 hours)
    if (currentTime - last_pps_time < config.tolerance_1ms){
      buffer_c[0].high= 0b11001110 | cSegDP;
      SetSysTick( &SysTick_CountUp_P3 );
    } else if (currentTime - last_pps_time < config.tolerance_10ms){
      buffer_c[3].low = 0b01000000;
      buffer_c[0].high= 0b11001110 | cSegDP;
      SetSysTick( &SysTick_CountUp_P2 );
    } else if (currentTime - rtc_last_calibration < config.tolerance_100ms){
      buffer_c[3].low = 0b01000000;
      buffer_c[2].low = 0b01000000;
      buffer_c[0].high= 0b11001110 | cSegDP;
      SetSysTick( &SysTick_CountUp_P1 );
    } else {
      buffer_c[3].low = 0b01000000;
      buffer_c[2].low = 0b01000000;
      buffer_c[1].low = 0b01000000;
      buffer_c[0].high= 0b11001110;
      SetSysTick( &SysTick_CountUp_P0 );
    }

  } else if (countMode == COUNT_DOWN) {

    if (config.countdown_to >= currentTime) {
      SetPPS( &PPS_Countdown );

      if (currentTime - last_pps_time < config.tolerance_1ms){
        buffer_c[0].high= 0b11001110 | cSegDP;
        SetSysTick( &SysTick_CountDown_P3 );
      } else if (currentTime - last_pps_time < config.tolerance_10ms){
        buffer_c[3].low = 0b01000000;
        buffer_c[0].high= 0b11001110 | cSegDP;
        SetSysTick( &SysTick_CountDown_P2 );
      } else if (currentTime - rtc_last_calibration < config.tolerance_100ms){
        buffer_c[3].low = 0b01000000;
        buffer_c[2].low = 0b01000000;
        buffer_c[0].high= 0b11001110 | cSegDP;
        SetSysTick( &SysTick_CountDown_P1 );
      } else {
        buffer_c[3].low = 0b01000000;
        buffer_c[2].low = 0b01000000;
        buffer_c[1].low = 0b01000000;
        buffer_c[0].high= 0b11001110;
        SetSysTick( &SysTick_CountDown_P0 );
      }

    } else {
      SetSysTick( &SysTick_CountUp_NoUpdate );
      SetPPS( &PPS_NoUpdate );
      buffer_c[0].low=cSegDecode0;
      buffer_c[1].low=cSegDecode0;
      buffer_c[2].low=cSegDecode0;
      buffer_c[3].low=cSegDecode0;

      next7seg.b[0] = bCat0 | cLut[0]<<2;
      next7seg.b[1] = bCat1 | cLut[0]<<2;
      next7seg.b[2] = bCat2 | cLut[0]<<2;
      next7seg.b[3] = bCat3 | cLut[0]<<2;
      next7seg.b[4] = bCat4 | cLut[0]<<2;
      next7seg.c = cLut[0];
    }

  }
}

void nextMode(_Bool reverse){

  _Bool wasOff = (displayMode==MODE_STANDBY);
  _Bool wasCountdown = (displayMode==MODE_COUNTDOWN);

  if (requestMode!=255){
    if (!config.modes_enabled[requestMode]) {
      requestMode=255;
      return;
    }
    displayMode=requestMode;
    requestMode=255;
  } else if (reverse) {
    do {
      if (--displayMode > NUM_DISPLAY_MODES) displayMode=NUM_DISPLAY_MODES;
    } while (!config.modes_enabled[displayMode]);
  } else {
    do {
      if (++displayMode >=NUM_DISPLAY_MODES) displayMode=0;
    } while (!config.modes_enabled[displayMode]);
  }

  if (wasOff && displayMode != MODE_STANDBY) displayOn();
  if ( displayMode == MODE_ISO_WEEK ||
       (wasCountdown && displayMode != MODE_COUNTDOWN)) {
    // It shouldn't be possible to exit countdown mode at .9 seconds
    // but if we did, it would show the wrong time for .1 seconds
    setNextTimestamp(currentTime);
  }

  if (displayMode == MODE_SHOW_OFFSET) {
    if (countMode != COUNT_HIDDEN) {
      countMode = COUNT_HIDDEN;
      SetSysTick( &SysTick_CountUp_NoUpdate );
      SetPPS( &PPS_NoUpdate );
      colonAnimationStop()
      TIM2->CCR1 = 0; // specific to show_offset
      TIM2->CCR2 = 300;
    }
  } else if (displayMode == MODE_COUNTDOWN) {
    if (countMode != COUNT_DOWN) {
      countMode = COUNT_DOWN;

      if (config.countdown_to >= currentTime) {
        setNextCountdown(currentTime);
      } else countdown_days = 0;

      setPrecision();
      TIM2->CCR1 = 0;
      TIM2->CCR2 = 0;
      latchSegments();
    }
  }
  else {
    if (countMode != COUNT_NORMAL) {
      countMode = COUNT_NORMAL;
      setPrecision();
      SetPPS( &PPS );
      TIM2->CCR1 = 0;
      TIM2->CCR2 = 0;
      latchSegments();
    }
  }

}
void button1pressed(void){

  // 12 bytes at 115200 8E1 is 1.14ms

  nextMode(0);

  // Normally this routine is only called when it is safe to do so
  // But if we have just left COUNT_HIDDEN, the button press could have been called at any time, so
  // avoid transmission if <3ms till rollover.
  // If we are still in COUNT_HIDDEN, always send.
  if (countMode == COUNT_HIDDEN || decisec!=9 || centisec!=9 || millisec<7)
    sendDate(1);
  // todo: detect this and resend date in pendsv

}
void button2pressed(void){
  nextMode(1);
  if (countMode == COUNT_HIDDEN || decisec!=9 || centisec!=9 || millisec<7)
    sendDate(1);
}

void generateDACbuffer(uint16_t * buf) {

  static float dac_last=4095;


  if (displayMode == MODE_STANDBY) {
    dac_target = dac_target*0.7 + 1.2*4095.0*0.3;
    if (dac_target>4094.0) {
      dac_target=4095.0;
      displayOff();
    }
  } else if (config.brightness_override >=0.0) {
    dac_target = config.brightness_override;
  } else {
    float adc = (float)ADC1->DR;

    uint8_t i;
    for (i=1; i< sizeof(brightnessCurve)/sizeof(brightnessCurve[0]) -1; i++){
      if (brightnessCurve[i].in > adc) break;
    }
    float factor = (adc - brightnessCurve[i-1].in) / (brightnessCurve[i].in - brightnessCurve[i-1].in);

    float out = brightnessCurve[i-1].out*(1.0-factor) + brightnessCurve[i].out*factor;

    if (out>4095.0 || !isfinite(out)) out=4095.0;
    else if (out<0.0) out=0.0;

    dac_target = dac_target*0.5 + out*0.5;
  }


  HAL_ADC_Start(&hadc1);



  float step = (dac_target-dac_last)/(DAC_BUFFER_SIZE*0.5);
  for (size_t i=0; i<DAC_BUFFER_SIZE/2; i++) {
    buf[i]= (uint16_t)(dac_last += step);
  }
  dac_last=dac_target;

  if (displayMode == MODE_DEBUG_BRIGHTNESS && decisec!=9) {
    sendDate(1);
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

  memcpy(__VECTORS_RAM, __VECTORS_FLASH, 0x188);
  SCB->VTOR = (uint32_t)&__VECTORS_RAM;

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

  buffer_c[0].high=0b11001110;
  buffer_c[1].high=0b11001101;
  buffer_c[2].high=0b11001011;
  buffer_c[3].high=0b11000111;
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
  MX_LPTIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */


  // Configure display matrix
  if (HAL_DMA_Start(&hdma_tim7_up, (uint32_t)buffer_c, (uint32_t)&GPIOC->ODR, 5) != HAL_OK)
    Error_Handler();

  if (HAL_DMA_Start(&hdma_tim1_up, (uint32_t)buffer_b, (uint32_t)&GPIOB->ODR, 5) != HAL_OK)
    Error_Handler();

  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE(&htim1);

  __HAL_TIM_ENABLE_DMA(&htim7, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE(&htim7);


  doDateUpdate();

  // Enable UART2 interrupt for button presses
  USART2->CR1 |= USART_CR1_RXNEIE;


  // Configure UART1 for NMEA strings from GPS module
  USART1->CR1 |= USART_CR1_CMIE ;

  USART1->CR1 &= ~(USART_CR1_UE);
  USART1->CR2 |= '\n'<<24;
  USART1->CR1 |= USART_CR1_UE;




  // Configure ADC and DAC DMA for display brightness
  HAL_ADC_Start(&hadc1);
  HAL_TIM_Base_Start(&htim6);

  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)buffer_dac, DAC_BUFFER_SIZE, DAC_ALIGN_12B_R) !=HAL_OK)
    Error_Handler();

  // Configure Colon Separators
  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;

  //loadColonAnimation();

  __HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2);
  __HAL_TIM_ENABLE(&htim5);

  //colonAnimationStart()


  //Enable DP for subseconds
  buffer_c[0].high=0b11001110 | cSegDP;



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

  //setDisplayPWM(5);
  displayOn();

  readConfigFile();




  if (RTC->ISR & RTC_ISR_INITS) //RTC contains non-zero data
  {
    RTC_DateTypeDef sdate;
    RTC_TimeTypeDef stime;

    if (!config.zone_override){
      char zone[32];
      memcpyword( (uint32_t*)zone,  (uint32_t*)&(RTC->BKP0R), 8 );
      zone[31]=0;

      if (loadRulesSingle(zone) != 0){ // takes 34ms -O0, 26ms -O2
        memcpyword( (uint32_t*)rules, (uint32_t*)&(RTC->BKP8R), 22 );
      }
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

    float fraction = (float)(32767 - stime.SubSeconds) / 32768.0;

    //  SysTick->VAL = SysTick->LOAD; ?
    millisec = (uint32_t)(fraction*1000) % 10;
    centisec = (uint32_t)(fraction*100) % 10;
    decisec =  (uint32_t)(fraction*10) % 10;

    if (decisec>=9) currentTime++;

    setNextTimestamp( currentTime );
    sendDate(1);
    latchSegments();
    rtc_good=1;

  } else { // backup domain reset

    currentTime=946684800; // 2000-01-01T00:00:00

    // The init process blanks the subsecond registers
    MX_RTC_Init();
  }

  setPrecision();
  PPS_Init();
  HAL_UART_Receive_DMA(&huart1, nmea, sizeof(nmea));




  // todo: reload mapfile each time, and abort zonedetect if USB write happens
  FIL mapfile;
  f_open(&mapfile, MAP_FILENAME, FA_READ);
  ZoneDetect *const cd = ZDOpenDatabase(&mapfile);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    if (!config.zone_override && cd && data_valid && latitude>=-90.0 && latitude<=90.0 && longitude>=-180.0 && longitude<=180.0) {

      char* zone = ZDHelperSimpleLookupString(cd, latitude, longitude);

      //printf("IANA Timezone is [%s]\n", zone);
      //printf("Took %lu ms\n", (HAL_GetTick()-start));

      if (!config.zone_override) loadRulesSingle(zone);
      free(zone);
      HAL_Delay(100);
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
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPTIM1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
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
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  /* LPTIM1 interrupt Init */
  NVIC_SetPriority(LPTIM1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(LPTIM1_IRQn);

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV1);
  LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
  LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);
  LL_LPTIM_TrigSw(LPTIM1);
  LL_LPTIM_SetInput1Src(LPTIM1, LL_LPTIM_INPUT1_SRC_GPIO);
  LL_LPTIM_SetInput2Src(LPTIM1, LL_LPTIM_INPUT2_SRC_GPIO);
  /* USER CODE BEGIN LPTIM1_Init 2 */

  LL_LPTIM_Enable(LPTIM1);
  LL_LPTIM_SetAutoReload(LPTIM1, 0xFFFF);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);

  /* USER CODE END LPTIM1_Init 2 */

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
  hrtc.Init.AsynchPrediv = 0;
  hrtc.Init.SynchPrediv = 32759;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  // RM page 1236
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  RTC->CALR = 0x100; // CALM to midpoint
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 7999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
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
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
