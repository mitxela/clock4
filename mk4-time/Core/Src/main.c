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
#include "astro.h"
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
ADC_HandleTypeDef hadc3;

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
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
void tmToBcd(struct tm *in, bcdStamp_t *out );
uint8_t loadRulesSingle(char * str);
void nextMode(_Bool);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t cLut[]= { cSegDecode0, cSegDecode1, cSegDecode2, cSegDecode3, cSegDecode4, cSegDecode5, cSegDecode6, cSegDecode7, cSegDecode8, cSegDecode9 };
const uint16_t bLut[]={ bSegDecode0, bSegDecode1, bSegDecode2, bSegDecode3, bSegDecode4, bSegDecode5, bSegDecode6, bSegDecode7, bSegDecode8, bSegDecode9 };

const char* wday_str[]={"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};

buffer_c_t buffer_c[80] = {0};

uint16_t buffer_b[80] = {0};

uint8_t uart2_tx_buffer[32];

volatile uint16_t buffer_adc[ADC_BUFFER_SIZE] = {0};
uint16_t buffer_dac[DAC_BUFFER_SIZE] = {[0 ... DAC_BUFFER_SIZE-1] = 4095};
float dac_target=4095;
float vbat = 0.0;

uint16_t buffer_colons_L[200] = {0};
uint16_t buffer_colons_R[200] = {0};

uint8_t nmea[NMEA_BUF_SIZE];
uint8_t satview[SV_COUNT];
uint8_t satview_stale = 0;

time_t currentTime;
bcdStamp_t nextBcd;
int tm_yday;
int8_t tm_wday;
int iso_year;
int8_t iso_wday;
uint8_t iso_week;
uint32_t countdown_days;
int32_t currentOffset=0;

struct {
  uint8_t c;
  uint16_t b[5];
} next7seg;

uint8_t decisec=0, centisec=0, millisec=0;

float longitude=-9999, latitude=-9999;
_Bool data_valid=0, had_pps=0, rtc_good=0, new_position=1;
// Last fix the ZoneDetect timezone lookup actually ran for. The NMEA parser sets new_position on
// EVERY 1 Hz fix, but that lookup reads TZMAP.BIN off the QSPI FATFS and costs ~300 ms — so on a
// stationary clock it re-ran every second on metre-scale GPS jitter, a 300 ms main-loop stall per
// second (starves the balance mirrors, widens the date-board byte-drop window). 999 = never yet.
double zone_lat=999.0, zone_lon=999.0;

// Astro pack — sun/moon/grid read-outs, computed once a second in the main loop
// (astro_update) and formatted by the sendDate cases, the same compute-in-loop /
// format-in-ISR split MODE_VBAT uses for vbat.
struct astro_cache_s {
  uint32_t epoch;          // currentTime this was computed for; 0 = never computed
  _Bool    have_pos;       // a usable lat/lon was available
  _Bool    sun_up_today;   // false = polar day/night (no rise/set this date)
  int16_t  rise_min, set_min, noon_min;  // local minutes-of-day [0,1440)
  int16_t  az, el;         // sun azimuth 0..359 / elevation, whole degrees
  uint8_t  moon_idx, moon_pct;           // phase index 0..7 / illuminated %
  char     grid[8];        // Maidenhead locator, or "----"
  float    lat_show, lon_show;           // the snapshot lat/lon, for MODE_LATLON
  // MODE_DARK twilight ladder — local minutes-of-day, -1 = not applicable (dashes).
  int16_t  civ_dusk_min, nau_dusk_min;   // civil (-6) / nautical (-12) evening twilight
  int16_t  ast_dusk_min, ast_dawn_min;   // astronomical (-18) darkness begins / ends
  _Bool    dark_tonight;                 // astronomical darkness occurs this date
  _Bool    always_dark;                  // polar night: sun below -18 now (dark round the clock)
} astro = {0};
#define rtc_last_write RTC->BKP30R
#define rtc_last_calibration RTC->BKP31R
uint32_t last_pps_time = 0;
uint32_t time_till_first_fix = 0;

struct {
  uint32_t t;
  int32_t offset;
} rules[162];
#define MAX_RULES (sizeof rules / sizeof rules[0])

char loadedRulesString[32];
char preloadRulesString[32];
char textDisplay[32];
_Bool delayedLoadRules = 0;
_Bool delayedReadConfigFile = 0;
_Bool delayedCheckOnEject = 0;
_Bool delayedPostConfigCleanup = 0;
// Set while the main loop is inside a (non-reentrant) FATFS operation, so the USB-ISR
// firmware-eject check defers instead of corrupting FATFS state. volatile: ISR-visible.
volatile uint8_t fatfs_busy = 0;
uint32_t delayedDisplayFreq = 0;

_Bool waitingForLatch = 0;
_Bool resendDate = 0;

uint32_t LPTIM1_high;

uint8_t displayMode = 0, countMode = 0, colonMode = 0;
// Civil vs alternate-timebase colon animation: colonMode is the ACTIVE selection that
// loadColonAnimation() renders; the per-context choices live here and applyColonForMode()
// swaps between them. The sidereal default must stay visually distinct from civil so
// MODE_LST/MODE_SOLAR can never masquerade as civil time.
uint8_t colonModeCivil = 0;
uint8_t colonModeAlt = COLON_MODE_ALT_SAWTOOTH;
_Bool colonAltExplicit = 0;    // user explicitly set colon_alt_mode
uint8_t requestMode = 255;
uint8_t nmea_cdc_level=0;
int debug_rtc_val = 0;

// --- PPS host timestamping ----------------------------------------------------------------
// Optional: emit one proprietary NMEA sentence ($PMTXTS) per PPS edge over the CDC port so a
// host can measure the clock's timing stability (phase jitter, oscillator drift, holdover) —
// things the plain NMEA stream cannot convey. Enabled by config "pps = on". Capture happens in
// the PPS ISR (cheap, just snapshots); the sentence is formatted + sent from the main loop.
volatile uint8_t pps_ts_enabled = 0;
volatile _Bool   pps_record_pending = 0;
int16_t die_temp_c = 0;  // latest STM32 die temperature (°C), a proxy for the crystal temperature
volatile struct {
  uint32_t seq;        // increments every PPS edge (32-bit: no practical wrap; host detects gaps)
  uint32_t systick;    // SysTick->VAL at the edge, captured BEFORE the reload (down-counter)
  uint16_t subms;      // 0..999 modelled ms-of-second at the edge, BEFORE the counters reset
  uint32_t epoch;      // currentTime at the edge (Unix seconds, UTC)
  int32_t  calerr;     // debug_rtc_val: signed LSE cycle error over CAL_PERIOD s (=> ppm on host)
  uint32_t sincecal;   // seconds since last successful RTC calibration (holdover age)
  int16_t  temp;       // die temperature (°C) — for host-side ppm-vs-temperature characterisation
  uint8_t  flags;      // bit0 data_valid, bit1 had_pps, bit2 rtc_good
  uint32_t dwt_pps;    // DWT->CYCCNT (free-running 12.5ns) latched at the edge — SOF-correlation timebase
} pps_cap;

// --- SOF correlation (experimental: sub-ms USB timestamping without a hardware PPS wire) -----------
// Latched by the USB SOF interrupt (PCD_SOFCallback, usbd_conf.c) every 1ms WHEN pps_ts_enabled: the
// 11-bit USB frame number and the DWT count at that Start-Of-Frame. Emitted in $PMTXTS so a host —
// which can read each USB frame's own arrival time in hardware — can place the PPS edge on its clock
// via the frame, immune to the ~6ms host-driven read jitter. Written in the SOF ISR, read at emit
// under __disable_irq. pps_sof_valid gates emission of the tail: it is 0 until the first SOF has
// latched a real anchor (so the first PPS after enumeration, or with pps just toggled on, or the
// emulator which has no SOF, emits the plain 9-field sentence rather than a stale (0,0) anchor).
volatile uint32_t pps_sof_dwt   = 0;   // DWT->CYCCNT at the most recent SOF
volatile uint16_t pps_sof_frame = 0;   // USB 11-bit frame number at that SOF (matches host frame mod 2048)
volatile uint8_t  pps_sof_valid = 0;   // 1 once a real SOF anchor has been latched; 0 = emit 9-field only

#define CHECK_CONFIG_MTIME

struct {
#ifdef CHECK_CONFIG_MTIME
  unsigned short fdate;
  unsigned short ftime;
#endif
  uint32_t tolerance_1ms;
  uint32_t tolerance_10ms;
  uint32_t tolerance_100ms;
  float fake_long;
  float fake_lat;
  time_t countdown_to;
  float brightness_override;
  volatile _Bool zone_override;
  uint16_t page_ms;             // sub-screen dwell, ms, for every mode that pages its date row on uwTick / page_ms()
  _Bool modes_enabled[NUM_DISPLAY_MODES];

} config = {0};

struct {
  float in;
  float out;
} brightnessCurve[] = {
    {0,    4095-0},
    {1425, 4095-737},
    {2566, 4095-1601},
    {3396, 4095-2725},
    {4095, 4095-4095},
};

// memcpy() appears to move data by bytes, which doesn't work with the word-accessed backup registers
// here we explicitly move data a word at a time
void memcpyword(volatile uint32_t *dest, volatile uint32_t *src, size_t n){
  while (n--){
    dest[n] = src[n];
  }
}

// 12 bytes at 115200 8E1 is 1.14ms, 32 bytes would be 3.06ms
// --- Astro pack helpers ----------------------------------------------------
// A usable position is held in latitude/longitude from either a GPS fix or the
// configured fake_latitude/fake_longitude; both sit at the -9999 sentinel until
// a position is known, so a simple range check is the "have we got a fix" test.
static _Bool astro_pos_ok(float lat, float lon){
  return lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f;
}
// Sub-screen dwell (ms) for every mode that pages its date row. Unset -> 5500 ms, a
// subjectively-tuned cadence found by feel. Floored at 250 ms so a tiny value can't
// flood the date-board UART.
static uint32_t page_ms(void){ uint32_t m = config.page_ms; return m == 0 ? 5500 : (m < 250 ? 250 : m); }
// Decimal UTC hour (sun_times may return <0 or >24) -> local minutes-of-day [0,1440).
static int astro_local_minutes(double utc_h){
  double h = fmod(utc_h + currentOffset / 3600.0, 24.0);
  if (h < 0) h += 24.0;
  int m = (int)(h * 60.0 + 0.5);
  if (m >= 1440) m -= 1440;
  return m;
}
// Recompute the astro cache (called from the main loop, never the ISR). The
// double soft-float maths runs here, then the small result struct is swapped in
// under a brief IRQ mask so sendDate() always reads a consistent snapshot.
static void astro_update(void){
  if (astro.epoch == (uint32_t)currentTime) return;     // at most once a second
  struct astro_cache_s c = {0};
  c.epoch = (uint32_t)currentTime;
  double ph = moon_phase((double)currentTime);          // moon needs no fix
  c.moon_idx = moon_phase_index(ph);
  c.moon_pct = (uint8_t)(moon_illuminated_fraction(ph) * 100.0 + 0.5);
  float lat = latitude, lon = longitude;                // one consistent snapshot of the fix
  c.have_pos = astro_pos_ok(lat, lon);
  if (c.have_pos) {
    c.lat_show = lat;
    c.lon_show = lon;
    double az, el, rise = 0, set = 0, noon = 0, civd = 0, naud = 0, astd = 0;
    sun_az_el(lat, lon, (double)currentTime, &az, &el);
    int ia = (int)(az + 0.5); if (ia >= 360) ia -= 360;
    c.az = (int16_t)ia;
    c.el = (int16_t)(el < 0 ? el - 0.5 : el + 0.5);
    c.civ_dusk_min = c.nau_dusk_min = c.ast_dusk_min = c.ast_dawn_min = -1;   // MODE_DARK: n/a unless computed below
    c.sun_up_today = (sun_times(lat, lon, (double)currentTime,
                                &rise, &set, &noon, &civd, &naud, 0, &astd) == 0);
    c.noon_min = (int16_t)astro_local_minutes(noon);     // noon is valid even at the poles
    if (c.sun_up_today) {
      c.rise_min = (int16_t)astro_local_minutes(rise);
      c.set_min  = (int16_t)astro_local_minutes(set);
      c.civ_dusk_min = (int16_t)astro_local_minutes(civd);
      c.nau_dusk_min = (int16_t)astro_local_minutes(naud);
      if (!isnan(astd)) {                                 // astronomical dark occurs -> dusk + symmetric dawn
        c.ast_dusk_min = (int16_t)astro_local_minutes(astd);
        c.ast_dawn_min = (int16_t)astro_local_minutes(2.0 * noon - astd);   // dawn = mirror of dusk about solar noon
        c.dark_tonight = 1;
      }
    } else {
      c.always_dark = (c.el < -18);                       // polar night: sun deep below the horizon now
    }
    maidenhead(lat, lon, c.grid);
  } else {
    strcpy(c.grid, "----");
  }
  __disable_irq();
  astro = c;
  __enable_irq();
}

void sendDate( _Bool now ){
  if (waitingForLatch) {
    if (countMode==COUNT_HIDDEN) {
      // if we've entered count_hidden while waiting for latch, it will never happen
      sendLatch()
      waitingForLatch=0;
    } else {
      resendDate=1;
      return;
    }
  }

  uint8_t i = 10;
  HAL_UART_AbortTransmit(&huart2);
  uart2_tx_buffer[0] = CMD_LOAD_TEXT;

  switch (displayMode) {
  default:
  case MODE_LST:       // alt-timebase modes keep the civil date on the date row —
  case MODE_SOLAR:   // the bottom row stays an unambiguous civil anchor
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
#ifdef NONCOMPLIANT_DATE_MODES
  case MODE_DDMMYYYY:
    uart2_tx_buffer[1] ='0'+nextBcd.tenDays;
    uart2_tx_buffer[2] ='0'+nextBcd.days;
    uart2_tx_buffer[3] ='-';
    uart2_tx_buffer[4] ='0'+nextBcd.tenMonths;
    uart2_tx_buffer[5] ='0'+nextBcd.months;
    uart2_tx_buffer[6] ='-';
    uart2_tx_buffer[7] ='2';
    uart2_tx_buffer[8] ='0';
    uart2_tx_buffer[9] ='0'+nextBcd.tenYears;
    uart2_tx_buffer[10]='0'+nextBcd.years;
    break;
#endif
  case MODE_ISO_ORDINAL:
    uart2_tx_buffer[1] ='2' ;//-2+nextBcd.seconds;
    uart2_tx_buffer[2] ='0';
    uart2_tx_buffer[3] ='0'+nextBcd.tenYears;
    uart2_tx_buffer[4] ='0'+nextBcd.years;
    uart2_tx_buffer[5] ='-';
    i = 5 + sprintf((char*)&uart2_tx_buffer[6], "%d", tm_yday+1);
    break;
  case MODE_ISO_WEEK:
    i = sprintf((char*)&uart2_tx_buffer[1], "%d-W%d-%d", iso_year, iso_week, iso_wday+1);
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
  case MODE_WEEKDAY:
    i = sprintf((char*)&uart2_tx_buffer[1], "%s", wday_str[tm_wday]);
    break;
  case MODE_WEEKDA_DD:
    sprintf((char*)&uart2_tx_buffer[1], "%-7.7s ", wday_str[tm_wday]);
    uart2_tx_buffer[9] ='0'+nextBcd.tenDays;
    uart2_tx_buffer[10]='0'+nextBcd.days;
    break;
  case MODE_WDY_MM_DD:
    sprintf((char*)&uart2_tx_buffer[1], "%.4s ", wday_str[tm_wday]);
    uart2_tx_buffer[6] ='0'+nextBcd.tenMonths;
    uart2_tx_buffer[7] ='0'+nextBcd.months;
    uart2_tx_buffer[8] ='-';
    uart2_tx_buffer[9] ='0'+nextBcd.tenDays;
    uart2_tx_buffer[10]='0'+nextBcd.days;
    break;
  case MODE_SATVIEW:
    if (satview[SV_GPS_L1]==255 && satview[SV_GPS_UNKNOWN]==255) {
      i = sprintf((char*)&uart2_tx_buffer[1], "GPS -");
    } else {
      uint8_t GPS_sv = 0, GLONASS_sv = 0, GALILEO_sv = 0, BEIDOU_sv = 0;
      if (satview[SV_GPS_L1]!=255) GPS_sv += satview[SV_GPS_L1];
      if (satview[SV_GPS_UNKNOWN]!=255) GPS_sv += satview[SV_GPS_UNKNOWN];
      if (satview[SV_GLONASS_L1]!=255) GLONASS_sv += satview[SV_GLONASS_L1];
      if (satview[SV_GLONASS_UNKNOWN]!=255) GLONASS_sv += satview[SV_GLONASS_UNKNOWN];
      if (satview[SV_GALILEO_E1]!=255) GALILEO_sv += satview[SV_GALILEO_E1];
      if (satview[SV_GALILEO_UNKNOWN]!=255) GALILEO_sv += satview[SV_GALILEO_UNKNOWN];
      if (satview[SV_BEIDOU_B1]!=255) BEIDOU_sv += satview[SV_BEIDOU_B1];
      if (satview[SV_BEIDOU_UNKNOWN]!=255)  BEIDOU_sv += satview[SV_BEIDOU_UNKNOWN];

      if (GLONASS_sv>0 && GLONASS_sv>=GALILEO_sv && GLONASS_sv>=BEIDOU_sv) {
        i = sprintf((char*)&uart2_tx_buffer[1], "GPS %d L%d", GPS_sv, GLONASS_sv);
      } else if (GALILEO_sv>0 && GALILEO_sv>=GLONASS_sv && GALILEO_sv>=BEIDOU_sv){
        i = sprintf((char*)&uart2_tx_buffer[1], "GPS %d A%d", GPS_sv, GALILEO_sv);
      } else if (BEIDOU_sv>0 && BEIDOU_sv>=GLONASS_sv && BEIDOU_sv>=GALILEO_sv){
        i = sprintf((char*)&uart2_tx_buffer[1], "GPS %d b%d", GPS_sv, BEIDOU_sv);
      } else {
        i = sprintf((char*)&uart2_tx_buffer[1], "GPS %d -", GPS_sv);
      }
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
      // snprintf returns the length it WOULD have written (newlib-nano follows C99),
      // not the truncated count; a >29-char TEXT= would otherwise push the ++i below
      // past uart2_tx_buffer[31]. Clamp to the bytes actually written.
      if (i > 29) i = 29;
    } else {
      uart2_tx_buffer[1]='-';
      i=1;
    }
    break;
  case MODE_VBAT:
    if (vbat == 0.0) {
      i = sprintf((char*)&uart2_tx_buffer[1], "bat -");
    } else {
      i = sprintf((char*)&uart2_tx_buffer[1], "bat %.4f", vbat);
    }
    break;
  case MODE_TTFF:
    // Our assumption is that uwTick is zero at power on
    if (!had_pps) time_till_first_fix = (int)(uwTick/1000);
    i = sprintf((char*)&uart2_tx_buffer[1], "ttff %3d.%02d", (int)(time_till_first_fix/60), (int)(time_till_first_fix%60));
    break;
  case MODE_DISPLAYTEST:
    int nn = currentTime%10;

    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    buffer_c[0].high &= ~cSegDP;
    buffer_c[1].high &= ~cSegDP;
    buffer_c[2].high &= ~cSegDP;
    buffer_c[3].high &= ~cSegDP;

    if ((currentTime%20)<10) {
      uart2_tx_buffer[1] =
      uart2_tx_buffer[2] =
      uart2_tx_buffer[3] =
      uart2_tx_buffer[4] =
      uart2_tx_buffer[5] =
      uart2_tx_buffer[6] =
      uart2_tx_buffer[7] =
      uart2_tx_buffer[8] =
      uart2_tx_buffer[9] =
      uart2_tx_buffer[10]= '0'+ nn;

      buffer_b[0]=bCat0 | bLut[ nn ];
      buffer_b[1]=bCat1 | bLut[ nn ];
      buffer_b[2]=bCat2 | bLut[ nn ];
      buffer_b[3]=bCat3 | bLut[ nn ];
      buffer_b[4]=bCat4 | bLut[ nn ];

      buffer_c[0].low= cLut[ nn ];
      buffer_c[1].low=cLut[ nn ];
      buffer_c[2].low=cLut[ nn ];
      buffer_c[3].low=cLut[ nn ];

      if ((currentTime%2) ==0) {
        TIM2->CCR2 = 300;
      } else {
        TIM2->CCR1 = 300;
      }
    } else {

      buffer_b[0]=bCat0 | (nn==0?bLut[8]:0);
      buffer_b[1]=bCat1 | (nn==1?bLut[8]:0);
      buffer_b[2]=bCat2 | (nn==2?bLut[8]:0);
      buffer_b[3]=bCat3 | (nn==3?bLut[8]:0);
      buffer_b[4]=bCat4 | (nn==4?bLut[8]:0);
      buffer_c[0].low=(nn==5?cLut[8]:0);
      buffer_c[1].low=(nn==6?cLut[8]:0);
      buffer_c[2].low=(nn==7?cLut[8]:0);
      buffer_c[3].low=(nn==8?cLut[8]:0);

      if (nn>=5) buffer_c[nn-5].high |= cSegDP;

      i = sprintf((char*)&uart2_tx_buffer[1], "%*s8.", nn, "");
    }

    break;
  case MODE_FIRMWARE_CRC_T:
  {
    extern uint32_t _app_crc[];
    uint32_t fwt = byteswap32(_app_crc[0]);
    i = sprintf((char*)&uart2_tx_buffer[1], "t %08lx", fwt);
  }
    break;
  case MODE_FIRMWARE_CRC_D:
    uart2_tx_buffer[0]=CMD_SHOW_CRC;
    break;

  // --- Astro pack: format the main-loop-computed cache onto the date row only,
  //     leaving the time row as the running clock (SATVIEW-style). --------------
  case MODE_SUN: {
    if (!astro.have_pos || !astro.epoch) { i = sprintf((char*)&uart2_tx_buffer[1], "RISE  ----"); break; }
    int page = (uwTick / page_ms()) % 3;               // rise -> set -> solar noon, page_ms each
    // labels padded to 4 chars in the literal ("SET "/"SOL ") so the time digits
    // line up under RISE without relying on the nano printf honouring "%-4s"
    const char *lbl = page == 0 ? "RISE" : page == 1 ? "SET " : "SOL ";
    int m           = page == 0 ? astro.rise_min : page == 1 ? astro.set_min : astro.noon_min;
    if (!astro.sun_up_today && page != 2) {            // sun never rises/sets today
      i = sprintf((char*)&uart2_tx_buffer[1], "%s ----", lbl);
    } else {
      i = sprintf((char*)&uart2_tx_buffer[1], "%s %02d.%02d", lbl, m / 60, m % 60);
    }
    break;
  }
  case MODE_SUN_AZEL:
    if (!astro.have_pos || !astro.epoch) { i = sprintf((char*)&uart2_tx_buffer[1], "AZ -- EL--"); }
    else if (astro.el < 0) i = sprintf((char*)&uart2_tx_buffer[1], "AZ%03dEL-%02d", astro.az, -astro.el);
    else                   i = sprintf((char*)&uart2_tx_buffer[1], "AZ%03dEL%02d",  astro.az,  astro.el);
    break;
  case MODE_MOON:                                      // UTC only; no fix needed
    if (!astro.epoch) i = sprintf((char*)&uart2_tx_buffer[1], "MOON -");
    else              i = sprintf((char*)&uart2_tx_buffer[1], "MOON %d %3d", astro.moon_idx, astro.moon_pct);
    break;
  case MODE_GRID:
    i = sprintf((char*)&uart2_tx_buffer[1], "%s", astro.epoch ? astro.grid : "----");
    break;
  case MODE_LATLON:
    // RISE/SET-style layout: label, separator space, a sign slot (space when positive), then
    // the digits — numbers align whether signed or not, and short values keep clear space at
    // the row's end. A 3-digit longitude can't fit both the separator and the sign slot in
    // 10 chars, so the separator is dropped just for that case ("LON 179.99" / "LON-179.99").
    if (!astro.have_pos || !astro.epoch) { i = sprintf((char*)&uart2_tx_buffer[1], "LAT  ----"); }
    else {
      _Bool lat = (uwTick / page_ms()) % 2 == 0;       // page latitude / longitude, page_ms each
      double v = lat ? astro.lat_show : astro.lon_show;
      long h = (long)(v * 100.0 + (v < 0 ? -0.5 : 0.5));  // hundredths, rounded
      long a2 = h < 0 ? -h : h;
      i = sprintf((char*)&uart2_tx_buffer[1], (a2 >= 10000) ? "%s%c%ld.%02ld" : "%s %c%ld.%02ld",
                  lat ? "LAT" : "LON", h < 0 ? '-' : ' ', a2 / 100, a2 % 100);
    }
    break;
  case MODE_DARK: {
    // The observing-session twilight ladder, paged: headline countdown to astronomical darkness, then
    // civil / nautical / astronomical dusk times and the astronomical dawn (dark ends). Honest at the
    // edges: no fix -> dashes; a white night that never reaches -18 -> "NO DARK"; polar night -> "DARK NOW".
    if (!astro.have_pos || !astro.epoch) { i = sprintf((char*)&uart2_tx_buffer[1], "%-4.4s ----", "DARK"); break; }
    int now_min = (int)((((long)currentTime + currentOffset) % 86400L + 86400L) % 86400L) / 60;
    // Every page uses MODE_SUN's RISE/SET layout: a 4-wide label + " HH.MM", so the digits sit in the
    // SAME columns as the ladder pages -> the numbers stay put as the mode pages. Countdown hours are
    // 2-digit too (05.57, not 5.57) so they line up with the times; no hyphen.
    switch ((int)((uwTick / page_ms()) % 5)) {
    case 0:                                              // headline: live countdown / status
      if (astro.always_dark)        i = sprintf((char*)&uart2_tx_buffer[1], "DARK  NOW");
      else if (!astro.dark_tonight) i = sprintf((char*)&uart2_tx_buffer[1], "NO DARK");
      else {
        int dusk = astro.ast_dusk_min, dawn = astro.ast_dawn_min;
        _Bool in_dark = (now_min >= dusk) || (now_min < dawn);   // dark window wraps midnight
        int togo = in_dark ? ((dawn - now_min + 1440) % 1440) : (dusk - now_min);   // minutes to the next edge
        // DAWN = counting to the end of dark (observing time left); DARK = counting down to its start.
        i = sprintf((char*)&uart2_tx_buffer[1], "%-4.4s %02d.%02d", in_dark ? "DAWN" : "DARK", togo / 60, togo % 60);
      }
      break;
    case 1:                                              // civil dusk (-6)
      i = astro.civ_dusk_min < 0 ? sprintf((char*)&uart2_tx_buffer[1], "%-4.4s ----", "CIV")
        : sprintf((char*)&uart2_tx_buffer[1], "%-4.4s %02d.%02d", "CIV", astro.civ_dusk_min/60, astro.civ_dusk_min%60);
      break;
    case 2:                                              // nautical dusk (-12)
      i = astro.nau_dusk_min < 0 ? sprintf((char*)&uart2_tx_buffer[1], "%-4.4s ----", "NAU")
        : sprintf((char*)&uart2_tx_buffer[1], "%-4.4s %02d.%02d", "NAU", astro.nau_dusk_min/60, astro.nau_dusk_min%60);
      break;
    case 3:                                              // astronomical dusk (-18): dark begins
      i = astro.ast_dusk_min < 0 ? sprintf((char*)&uart2_tx_buffer[1], "%-4.4s ----", "AST")
        : sprintf((char*)&uart2_tx_buffer[1], "%-4.4s %02d.%02d", "AST", astro.ast_dusk_min/60, astro.ast_dusk_min%60);
      break;
    default:                                             // astronomical dawn: dark ends
      i = astro.ast_dawn_min < 0 ? sprintf((char*)&uart2_tx_buffer[1], "%-4.4s ----", "End")
        : sprintf((char*)&uart2_tx_buffer[1], "%-4.4s %02d.%02d", "End", astro.ast_dawn_min/60, astro.ast_dawn_min%60);
      break;
    }
    break;
  }
  }
  if (now) {
    uart2_tx_buffer[++i]= CMD_RELOAD_TEXT;
  } else {
    uart2_tx_buffer[++i]= '\n';
    waitingForLatch=1;
  }
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
  tm_wday = nextTm->tm_wday;

  if (displayMode == MODE_ISO_WEEK){
    iso_wday = (nextTm->tm_wday + 6) % 7;
    nextTm->tm_mday -= iso_wday -3;
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
// --- Alternate timebase (MODE_LST / MODE_SOLAR) ------------------------------------------
// The TIME ROW ticks Local Sidereal Time or apparent solar time. Heavy double
// math runs in THREAD context once per second (alt_update), staging the reading for the
// coming civil boundary; the SysTick_Alt_* handlers latch it at the .900 prep mark. The
// display is quantized to civil second boundaries — value = floor(alt time at the boundary),
// reseeded every second — so GPS discipline and holdover honesty are inherited from
// currentTime for free. Sidereal runs 1.00273791x civil: the seconds display double-steps
// once every ~6 min 5 s. That skip is the authentic signature of a true sidereal clock.
static volatile struct {
  uint8_t hh, mm, ss;
  uint32_t for_time;            // civil epoch this reading is the floor of; 0 = invalid
} alt_stage;
static uint8_t alt_hh, alt_mm, alt_ss;   // ISR-owned: what the row currently shows
static volatile _Bool alt_have_pos = 0;
static volatile _Bool alt_seed_pending = 0;  // mode entered: thread must seed the row
static volatile uint8_t alt_gen = 0;         // bumped on mode entry; cancels in-flight staging

// Overlay an alternate HH:MM:SS onto the next7seg staging buffer. The stock
// setNextTimestamp() has just run (keeping nextBcd / DST / date-row bookkeeping fresh);
// only the six time-row digit patterns are replaced.
#define alt_render_next7seg(hh, mm, ss) do { \
    next7seg.c    = cLut[(ss) % 10]; \
    next7seg.b[0] = bCat0 | cLut[(hh) / 10] << 2; \
    next7seg.b[1] = bCat1 | cLut[(hh) % 10] << 2; \
    next7seg.b[2] = bCat2 | cLut[(mm) / 10] << 2; \
    next7seg.b[3] = bCat3 | cLut[(mm) % 10] << 2; \
    next7seg.b[4] = bCat4 | cLut[(ss) / 10] << 2; \
  } while (0)

// The .900 prep for the alternate modes: stock next-second bookkeeping first, then latch
// the staged reading — or, if the main loop was starved past the boundary, advance the last
// shown reading by one second. LST's fallback runs SLOW (2.74 ms/s; the reseed snap is
// always forward), SOLAR's runs fast by at most ~0.35 ms/s at the EoT extremes — a
// visible backwards reseed would need ~48+ minutes of continuous main-loop starvation.
#define alt_prep_next() do { \
    currentTime++; \
    setNextTimestamp( currentTime ); \
    if (alt_stage.for_time == (uint32_t)currentTime) { \
      alt_hh = alt_stage.hh; alt_mm = alt_stage.mm; alt_ss = alt_stage.ss; \
    } else if (++alt_ss >= 60) { \
      alt_ss = 0; \
      if (++alt_mm >= 60) { alt_mm = 0; if (++alt_hh >= 24) alt_hh = 0; } \
    } \
    alt_render_next7seg(alt_hh, alt_mm, alt_ss); \
    sendDate(0); \
  } while (0)

// Compute floor-HH:MM:SS of the alternate time at `when` (thread context only: doubles).
static _Bool alt_compute(uint32_t when, uint8_t *hh, uint8_t *mm, uint8_t *ss){
  float lat = latitude, lon = longitude;   // one consistent snapshot (astro_update pattern)
  if (!astro_pos_ok(lat, lon)) return 0;
  double hours = (displayMode == MODE_LST)
               ? local_sidereal_time((double)when, (double)lon)
               : local_solar_time((double)when, (double)lon);
  if (!(hours >= 0.0) || hours >= 24.0) hours = 0.0;  // NaN / float-residue guard
  int h2 = (int)hours;
  double fm = (hours - h2) * 60.0;
  int m2 = (int)fm;
  int s2 = (int)((fm - m2) * 60.0);
  if (h2 > 23) h2 = 23;
  if (m2 > 59) m2 = 59;
  if (s2 > 59) s2 = 59;
  *hh = (uint8_t)h2; *mm = (uint8_t)m2; *ss = (uint8_t)s2;
  return 1;
}

// Main-loop staging (thread context — ALL the double math for these modes lives here).
// Two jobs: (a) SEED after mode entry or position go-live — render + latch the current
// reading immediately and install the live handlers, so a PPS latch can never show civil
// digits under the alternate colon; (b) STAGE the reading for the coming civil boundary.
// A generation counter cancels any in-flight computation when the mode flips mid-pass, so
// a stale timebase can never be stamped as valid.
void alt_update(void){
  if (displayMode != MODE_LST && displayMode != MODE_SOLAR) return;

  uint8_t gen = alt_gen;                 // snapshot: mode flips abort the publish below

  if (alt_seed_pending || !alt_have_pos) {
    uint8_t hh, mm, ss;
    if (!alt_compute((uint32_t)currentTime, &hh, &mm, &ss)) {
      alt_have_pos = 0;                  // stay dashed; retried every pass
      return;
    }
    __disable_irq();
    if (gen == alt_gen) {
      alt_hh = hh; alt_mm = mm; alt_ss = ss;
      alt_render_next7seg(alt_hh, alt_mm, alt_ss);   // alt digits now staged: any latch is honest
      latchSegments()                                 // and shown immediately (countdown precedent)
      alt_have_pos = 1;
      alt_seed_pending = 0;
    }
    __enable_irq();
    if (gen == alt_gen) setPrecision();  // install Alt_Px/PPS now — don't wait for PendSV,
                                         // or the NoUpdate .900 prep could stage civil digits
    return;                              // stage the coming boundary on the next pass
  }

  uint32_t target = (uint32_t)currentTime + 1;
  if (alt_stage.for_time == target) return;
  uint8_t hh, mm, ss;
  if (!alt_compute(target, &hh, &mm, &ss)) {
    alt_have_pos = 0;                    // position lost: setPrecision dashes it this second
    alt_stage.for_time = 0;
    return;
  }
  __disable_irq();
  if (gen == alt_gen) {                  // publish only if no mode flip happened mid-compute
    alt_stage.hh = hh; alt_stage.mm = mm; alt_stage.ss = ss;
    alt_stage.for_time = target;         // IRQs masked: fields and stamp are one atomic unit
  }
  __enable_irq();
}

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

  float tempLatitude=-9999, tempLongitude=-9999;

  nextField() // Latitude deg
  if (*c){
    tempLatitude =  (float)(*c++ -'0')*10.0;
    tempLatitude += (float)(*c++ -'0');
    tempLatitude += (float)atof((char*)c) / 60.0;
  }
  nextField() // Latitude N/S
  if (*c =='S') tempLatitude =-tempLatitude;

  nextField() // Longitude  deg
  if (*c){
    tempLongitude =  (float)(*c++ -'0')*100.0;
    tempLongitude += (float)(*c++ -'0')*10.0;
    tempLongitude += (float)(*c++ -'0');
    tempLongitude += (float)atof((char*)c) / 60.0;
  }
  nextField() // Longitude  E/W
  if (*c == 'W') tempLongitude =-tempLongitude;

  if (!config.fake_long && !config.fake_lat) {
    longitude = tempLongitude;
    latitude = tempLatitude;
    new_position=1;
  }

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
      // check we're not <2ms away from rollover
      if (centisec==9 && millisec>7) return;

      // Under normal conditions, we should only be parsing nmea at around .300 to .400
      // USART1 preemption priority is currently 1, so we could be interrupted by systick here
      setNextTimestamp( currentTime );
      // In the alternate time-row modes the civil digits just staged must not reach the
      // display: restore the alt overlay so the boundary latch stays honest.
      if (countMode == COUNT_ALT) alt_render_next7seg(alt_hh, alt_mm, alt_ss);
      sendDate(0);
    }
  }

}

void decodeGSV(uint8_t rec){
  unsigned int sv = (nmea[11]-'0')*10 + (nmea[12]-'0');
  uint8_t constellation = nmea[2];
  uint8_t signal_id;

  // signal ID is not always present in GSV (on M8Q)

  unsigned int num_fields = 0, r=0;
  while (++r<rec) if (nmea[r]==',') num_fields++;

  if (num_fields % 4 != 0) {
    signal_id = '0';
  } else {
    signal_id = nmea[rec-6];
  }

  if (constellation == 'P') {
      if (signal_id == '0') {
        satview[SV_GPS_UNKNOWN] = sv;
      } else {
        satview[SV_GPS_L1] = sv;
      }
      satview_stale = 0;
  } else if (constellation == 'L') {
    if (signal_id == '0') {
      satview[SV_GLONASS_UNKNOWN] = sv;
    } else {
      satview[SV_GLONASS_L1] = sv;
    }
  } else if (constellation == 'A') {
    if (signal_id == '0') {
      satview[SV_GALILEO_UNKNOWN] = sv;
    } else {
      satview[SV_GALILEO_E1] = sv;
    }
  } else if (constellation == 'B') {
    if (signal_id == '0') {
      satview[SV_BEIDOU_UNKNOWN] = sv;
    } else {
      satview[SV_BEIDOU_B1] = sv;
    }
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
  if (waitingForLatch) {
    delayedDisplayFreq = freq;
    return;
  }

  if (freq<1000 || freq>100000) {delayedDisplayFreq=0; return;}

  uint8_t tx_buf[4];
  tx_buf[0]= CMD_SET_FREQUENCY;
  tx_buf[1]= (freq>>14) & 0x7F;
  tx_buf[2]= (freq>>7)  & 0x7F;
  tx_buf[3]= (freq)     & 0x7F;
  if (HAL_UART_Transmit(&huart2, tx_buf, 4, 2) == HAL_OK) {
    delayedDisplayFreq = 0;
  }

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

static uint8_t parseColonName(const char *value){
  if (strcasecmp(value, "solid") == 0)        return COLON_MODE_SOLID;
  if (strcasecmp(value, "heartbeat") == 0)    return COLON_MODE_HEARTBEAT;
  if (strcasecmp(value, "sawtooth") == 0)     return COLON_MODE_1PPS_SAWTOOTH;
  if (strcasecmp(value, "alt_sawtooth") == 0) return COLON_MODE_ALT_SAWTOOTH;
  if (strcasecmp(value, "toggle") == 0)       return COLON_MODE_TOGGLE;
  return COLON_MODE_SLOWFADE;
}

// Select the colon animation for the current display mode (idempotent, thread context).
// Alternate-timebase modes get their own animation so they read as "not civil" at a glance.
void applyColonForMode(void){
  uint8_t want = (displayMode == MODE_LST || displayMode == MODE_SOLAR)
               ? colonModeAlt : colonModeCivil;
  if (want != colonMode) {
    colonMode = want;
    loadColonAnimation();
  }
}

void parseConfigString(char *key, char *value) {

  if (strcasecmp(key, "text") == 0) {

    strcpy(textDisplay, value);

  } else if (strcasecmp(key, "MATRIX_FREQUENCY") == 0) {

    setDisplayFreq(atoi(value));

  } else if (strcasecmp(key, "zone_override") == 0) {

    if (!value[0] || delayedLoadRules) return;

    strcpy(preloadRulesString, value);
    delayedLoadRules=1;
    ZDAbort();

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
  } else if (strcasecmp(key, "MODE_WEEKDAY") == 0) {
    set_mode_enabled(MODE_WEEKDAY, value);
  } else if (strcasecmp(key, "MODE_WEEKDA_DD") == 0) {
    set_mode_enabled(MODE_WEEKDA_DD, value);
  } else if (strcasecmp(key, "MODE_WDY_MM_DD") == 0) {
    set_mode_enabled(MODE_WDY_MM_DD, value);
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
  } else if (strcasecmp(key, "MODE_VBAT") == 0) {
    set_mode_enabled(MODE_VBAT, value);
  } else if (strcasecmp(key, "MODE_DISPLAYTEST") == 0) {
    set_mode_enabled(MODE_DISPLAYTEST, value);
  } else if (strcasecmp(key, "MODE_TTFF") == 0) {
    set_mode_enabled(MODE_TTFF, value);
#ifdef NONCOMPLIANT_DATE_MODES
  } else if (strcasecmp(key, "MODE_DDMMYYYY") == 0) {
    set_mode_enabled(MODE_DDMMYYYY, value);
#endif
  } else if (strcasecmp(key, "MODE_FIRMWARE_CRC") == 0) {
    set_mode_enabled(MODE_FIRMWARE_CRC_D, value);
    set_mode_enabled(MODE_FIRMWARE_CRC_T, value);
  } else if (strcasecmp(key, "MODE_SUN") == 0) {
    set_mode_enabled(MODE_SUN, value);
  } else if (strcasecmp(key, "MODE_SUN_AZEL") == 0) {
    set_mode_enabled(MODE_SUN_AZEL, value);
  } else if (strcasecmp(key, "MODE_MOON") == 0) {
    set_mode_enabled(MODE_MOON, value);
  } else if (strcasecmp(key, "MODE_GRID") == 0) {
    set_mode_enabled(MODE_GRID, value);
  } else if (strcasecmp(key, "MODE_LATLON") == 0) {
    set_mode_enabled(MODE_LATLON, value);
  } else if (strcasecmp(key, "MODE_DARK") == 0) {
    set_mode_enabled(MODE_DARK, value);
  } else if (strcasecmp(key, "page_ms") == 0) {
    int v = atoi(value);
    config.page_ms = v < 0 ? 0 : (v > 65535 ? 65535 : v);   // fits uint16; 0 -> default
  } else if (strcasecmp(key, "MODE_LST") == 0) {
    set_mode_enabled(MODE_LST, value);
  } else if (strcasecmp(key, "MODE_SOLAR") == 0) {
    set_mode_enabled(MODE_SOLAR, value);
  } else if (strcasecmp(key, "Tolerance_time_1ms") == 0) {
    config.tolerance_1ms = atoi(value);
  } else if (strcasecmp(key, "Tolerance_time_10ms") == 0) {
    config.tolerance_10ms = atoi(value);
  } else if (strcasecmp(key, "Tolerance_time_100ms") == 0) {
    config.tolerance_100ms = atoi(value);
  } else if (strcasecmp(key, "fake_longitude") == 0) {
    config.fake_long = atof(value);
  } else if (strcasecmp(key, "fake_latitude") == 0) {
    config.fake_lat = atof(value);
  } else if (strcasecmp(key, "colon_mode") == 0) {

    colonModeCivil = parseColonName(value);

  } else if (strcasecmp(key, "colon_alt_mode") == 0) {

    colonModeAlt = parseColonName(value);   // shared by MODE_LST and MODE_SOLAR ("COLONALT" in the menu)
    colonAltExplicit = 1;

  } else if (strcasecmp(key, "nmea") == 0) {

    if (falsey(value)) {
      nmea_cdc_level = NMEA_NONE;
    } else if (strcasecmp(value, "rmc") == 0) {
      nmea_cdc_level = NMEA_RMC;
    } else nmea_cdc_level = NMEA_ALL;

  } else if (strcasecmp(key, "pps") == 0) {

    pps_ts_enabled = truthy(value);   // emit a $PMTXTS timing sentence on each PPS edge

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
  // Keep the alternate-timebase colon distinct unless the user EXPLICITLY matched them.
  if (!colonAltExplicit && colonModeAlt == colonModeCivil) {
    colonModeAlt = (colonModeCivil != COLON_MODE_ALT_SAWTOOTH) ? COLON_MODE_ALT_SAWTOOTH
                 : COLON_MODE_TOGGLE;
  }
  colonMode = 0xFF;             // force applyColonForMode to reload exactly once
  applyColonForMode();

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

  if (config.fake_long && config.fake_lat) {
    longitude = config.fake_long;
    latitude = config.fake_lat;
    new_position =1;
  }
}

void rxConfigString(char c){
  static char key[32], value[32];
  static uint8_t k=0, v=0, state=0;

  if (c=='\n' || c=='\r') {
    key[k]=0;
    value[v]=0;

    if (strcasecmp(key, "reboot") == 0) {
      MX_USB_Stop();
      NVIC_SystemReset();
    }
    if (k && (v || state>=2)) {
      parseConfigString(key, value);
      // rxConfigString runs in the USB OTG ISR; postConfigCleanup() calls nextMode()
      // and sendDate(), which are non-reentrant against the SysTick repaint. Defer it
      // to the main loop so it runs in thread context, like the file-config path does.
      delayedPostConfigCleanup=1;
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

#ifdef CHECK_CONFIG_MTIME
  FILINFO fno;
  if (f_stat(CONFIG_FILENAME, &fno) == FR_OK) {
    // if unchanged, exit early before touching any config
    // if the file doesn't exist, fall through and fail on the f_open
    // A zero FAT timestamp (the volume's RTC was unset when config.txt was written)
    // must not be used as a cache key: config={0} matches it on the very first boot,
    // so config is never loaded, no mode is enabled, and the first MODE button press
    // then spins nextMode() forever. Only short-circuit on a real, non-zero stamp.
    if ((fno.fdate || fno.ftime) && fno.fdate==config.fdate && fno.ftime==config.ftime) return;
    config.fdate=fno.fdate;
    config.ftime=fno.ftime;
  }
#endif

  config.tolerance_1ms   = 1000;
  config.tolerance_10ms  = 10000;
  config.tolerance_100ms = 100000;
  config.zone_override = 0;
  config.brightness_override = -1.0;
  colonModeCivil = 0;
  colonModeAlt = COLON_MODE_ALT_SAWTOOTH;
  colonAltExplicit = 0;

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

   // if enabled, always boot into ttff
   if (config.modes_enabled[MODE_TTFF]) requestMode=MODE_TTFF;
   else requestMode=255;

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

// Snapshot the timing state at the instant of the PPS edge. MUST run before SysTick->VAL is
// reloaded and before millisec/centisec/decisec are zeroed, so it captures the phase error
// between the firmware's modelled second and the true GPS edge.
#define capturePPS() do { \
    pps_cap.dwt_pps  = DWT->CYCCNT; \
    pps_cap.systick  = SysTick->VAL; \
    pps_cap.subms    = (uint16_t)decisec*100 + (uint16_t)centisec*10 + millisec; \
    pps_cap.epoch    = (uint32_t)currentTime; \
    pps_cap.calerr   = debug_rtc_val; \
    pps_cap.sincecal = (uint32_t)currentTime - (uint32_t)rtc_last_calibration; \
    pps_cap.temp     = die_temp_c; \
    pps_cap.flags    = (data_valid?1:0) | (had_pps?2:0) | (rtc_good?4:0); \
    pps_cap.seq++; \
    pps_record_pending = 1; \
  } while(0)

// PPS rising edge
void PPS(void)
{
  capturePPS();
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
  capturePPS();
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
  capturePPS();
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

// usbd_cdc_if.h isn't pulled into main.c; forward-declare the one symbol we need.
extern uint8_t CDC_Copy_Transmit(uint8_t* buf, uint16_t Len);
extern USBD_HandleTypeDef hUsbDeviceFS;

// Format + send one $PMTXTS sentence from the values captured at the last PPS edge.
// Runs in the main loop (snprintf is fine here, never in the ISR). Clears pps_record_pending
// on a successful send and for any undeliverable record (no host, formatting failure) — a
// fresh record arrives on the next edge, so only USBD_BUSY is worth retrying.
// Sentence: $PMTXTS,<seq>,<epoch>,<subms>,<systick>,<load>,<calerr>,<sincecal>,<temp>,<flags>,<dwt_pps>,<sof_frame>,<dwt_sof>*CC
//   subms+(load-systick)/(load+1) = modelled sub-second position at the edge (phase error);
//   ppm = calerr * 1e6 / (32768 * CAL_PERIOD)  [CAL_PERIOD=63];  temp = die °C;
//   flags: b0 valid, b1 pps, b2 rtc.
//   SOF-correlation tail (experimental): dwt_pps = DWT cycle count at the PPS edge; sof_frame = USB
//   11-bit frame number of the most recent SOF; dwt_sof = DWT at that SOF. A host that knows each USB
//   frame's own arrival time places the edge as hostTime(sof_frame) + (dwt_pps-dwt_sof)/f_dwt, immune
//   to USB read jitter. dwt_pps deltas (~80e6/s) self-calibrate f_dwt, so no core-clock assumption.
static uint8_t emitPPSTimestamp(void){
  // With no enumerated host (e.g. charger-only power) CDC can never accept the sentence;
  // drop the record before doing any formatting work, otherwise the pending flag would
  // re-run the whole format-and-fail cycle every main-loop pass until a host appears.
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
    pps_record_pending = 0;
    return USBD_FAIL;
  }

  __disable_irq();                       // atomic snapshot of the ISR-written capture
  uint32_t snap_seq = pps_cap.seq;
  uint32_t st       = pps_cap.systick;
  uint16_t subms    = pps_cap.subms;
  uint32_t epoch    = pps_cap.epoch;
  int32_t  calerr   = pps_cap.calerr;
  uint32_t sincecal = pps_cap.sincecal;
  int16_t  temp     = pps_cap.temp;
  uint8_t  flags    = pps_cap.flags;
  uint32_t dwt_pps  = pps_cap.dwt_pps;   // DWT at the PPS edge (SOF-correlation timebase)
  uint32_t sof_dwt  = pps_sof_dwt;       // DWT at the most recent SOF ...
  uint16_t sof_fr   = pps_sof_frame;     // ... and that SOF's 11-bit USB frame number ...
  uint8_t  sof_ok   = pps_sof_valid;     // ... valid only once a real SOF has latched an anchor
  __enable_irq();

  uint32_t load = SysTick->LOAD;         // constant; sent so the host needn't assume core clock

  char body[128];                        // everything between '$' and '*'
  int n = snprintf(body, sizeof body, "PMTXTS,%lu,%lu,%u,%lu,%lu,%ld,%lu,%d,%X",
                   (unsigned long)snap_seq, (unsigned long)epoch, (unsigned)subms,
                   (unsigned long)st, (unsigned long)load, (long)calerr,
                   (unsigned long)sincecal, (int)temp, (unsigned)flags);
  if (n < 0 || n >= (int)sizeof body) { pps_record_pending = 0; return USBD_FAIL; }
  // Append the SOF-correlation tail only when a real anchor exists — never a stale (0,0). Absent tail =
  // the plain sentence a 9-field parser expects (also the emulator's output, which has no USB SOF).
  if (sof_ok) {
    int t = snprintf(body + n, sizeof body - n, ",%lu,%u,%lu",
                     (unsigned long)dwt_pps, (unsigned)sof_fr, (unsigned long)sof_dwt);
    if (t < 0 || t >= (int)(sizeof body - n)) { pps_record_pending = 0; return USBD_FAIL; }
    n += t;
  }

  uint8_t cks = 0;                       // standard NMEA XOR checksum
  for (int i = 0; i < n; i++) cks ^= (uint8_t)body[i];

  char line[NMEA_BUF_SIZE];              // must fit the CDC txbuf[NMEA_BUF_SIZE] downstream
  int m = snprintf(line, sizeof line, "$%s*%02X\r\n", body, (unsigned)cks);
  if (m < 0 || m >= (int)sizeof line) { pps_record_pending = 0; return USBD_FAIL; }

  // The CDC IN endpoint is shared with the ISR NMEA passthrough; serialise the (tiny) submit,
  // and clear the pending flag only if no fresh PPS edge arrived since the snapshot (so a
  // record captured mid-send isn't silently dropped). FAIL also clears: the record is
  // undeliverable (USB de-inited under us), unlike BUSY where the host may drain the FIFO.
  __disable_irq();
  uint8_t r = CDC_Copy_Transmit((uint8_t*)line, (uint16_t)m);
  if (r != USBD_BUSY && pps_cap.seq == snap_seq) pps_record_pending = 0;
  __enable_irq();
  return r;
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

// Alternate-timebase handlers (MODE_LST / MODE_SOLAR): identical to the CountUp family —
// same cascade, same sub-second painting, same precision ladder — except the .900 prep
// overlays the staged alternate HH:MM:SS onto next7seg (see alt_prep_next).
void SysTick_Alt_P3(void)
{
  timetick()

  buffer_c[3].low=cLut[millisec];
  buffer_c[2].low=cLut[centisec];
  buffer_c[1].low=cLut[decisec];

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    alt_prep_next();
  }
}

void SysTick_Alt_P2(void) {
  timetick()

  buffer_c[2].low=cLut[centisec];
  buffer_c[1].low=cLut[decisec];

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    alt_prep_next();
  }
}

void SysTick_Alt_P1(void) {
  timetick()

  buffer_c[1].low=cLut[decisec];

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    alt_prep_next();
  }
}

void SysTick_Alt_P0(void) {
  timetick()

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    alt_prep_next();
  }
}

void SysTick_Dummy(void){
  HAL_IncTick();
}

// We cannot use hardware vbus monitoring since the pin is occupied by USART1 TX
// We can't use EXTI on PA8 as it's in the same group as PPS
void monitor_vbus(void){
  static _Bool vbus_state = 1; // power-on state is initialised, even if not connected

  _Bool vbus = (GPIOA->IDR & GPIO_PIN_8);

  if (vbus_state && !vbus) { // disconnected

    MX_USB_Stop();

  } else if (vbus && !vbus_state) { // connected

    MX_USB_DEVICE_Init();

  }
  vbus_state = vbus;
}

void measure_vbat(void){
  ADC123_COMMON->CCR |= ADC_CCR_VBATEN;
  HAL_Delay(5);
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 10);
  uint16_t adc = HAL_ADC_GetValue(&hadc3);
  ADC123_COMMON->CCR &= ~ADC_CCR_VBATEN;
  vbat = (float)adc *0.0024102564102564104;//3*3.29/4095.0;
}

// Read the STM32 internal die-temperature sensor on hadc3 (shared with VBAT) into die_temp_c.
// The die sits slightly above ambient on this low-power board, but it tracks the crystal well
// enough to characterise the oscillator's temperature dependence.
void measure_temp(void){
  ADC_ChannelConfTypeDef s = {0};
  s.Rank = ADC_REGULAR_RANK_1;
  s.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;   // temp sensor needs a long sampling time
  s.SingleDiff = ADC_SINGLE_ENDED;
  s.OffsetNumber = ADC_OFFSET_NONE;
  s.Offset = 0;

  s.Channel = ADC_CHANNEL_TEMPSENSOR;
  HAL_ADC_ConfigChannel(&hadc3, &s);
  ADC123_COMMON->CCR |= ADC_CCR_TSEN;
  HAL_Delay(1);                                  // tSTART for the temperature sensor (~120 us)
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 10);
  uint16_t raw = HAL_ADC_GetValue(&hadc3);
  ADC123_COMMON->CCR &= ~ADC_CCR_TSEN;

  // Factory-calibrated conversion (TS_CAL1/TS_CAL2 in flash). VREF taken as 3300 mV; absolute
  // accuracy isn't critical — the curve is fitted against GPS-measured ppm, not trusted raw.
  die_temp_c = (int16_t)__HAL_ADC_CALC_TEMPERATURE(3300, raw, ADC_RESOLUTION_12B);

  s.Channel = ADC_CHANNEL_VBAT;                  // restore so measure_vbat() keeps working
  HAL_ADC_ConfigChannel(&hadc3, &s);
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
    return RULES_NO_FILE;
  }

  unsigned int rc;
  char buf[4];

  f_read(&file, &buf, 4, &rc);

  if(memcmp(&buf, "MTZ", 3)) {
    return RULES_HEADER_ERR;
  }
  if (buf[3]!=1) {
    return RULES_VERSION_UNKNOWN;
  }

  uint8_t rowLength;
  f_read(&file, &rowLength, 1, &rc);

  uint8_t numCats;
  f_read(&file, &numCats, 1, &rc);

  if (findField( &file, cat, numCats, 3 ) ==0) {
    return RULES_CATEGORY_UNKNOWN;
  }
  uint16_t catAddr;
  f_read(&file, &catAddr, 2, &rc);

  uint8_t numZones;
  f_read(&file, &numZones, 1, &rc);

  f_lseek(&file, catAddr);

  if (findField( &file, zo, numZones, 4 ) ==0) {
    return RULES_ZONE_UNKNOWN;
  }

  uint32_t zoAddr = 0;
  f_read(&file, &zoAddr, 3, &rc);

  uint8_t numEntries;
  f_read(&file, &numEntries, 1, &rc);

  f_lseek(&file, zoAddr);

  // TZRULES.BIN is host-writable over the USB mass-storage volume, so its length
  // fields are untrusted: a rowLength larger than one rule slot, or numEntries larger
  // than the array, would overrun rules[] (global RAM corruption / HardFault). Reject.
  if (rowLength > sizeof rules[0] || numEntries > MAX_RULES) {
    f_close(&file);
    return RULES_HEADER_ERR;
  }

  int i;
  for (i=0;i<numEntries;i++) {
    f_read(&file, &rules[i], rowLength, &rc);
  }
  while (i< MAX_RULES ) {
    rules[i++].t=-1;
  }

  f_close(&file);

  return RULES_OK;
}

// loadRulesSingle modifies the input string, can't be used with const str
uint8_t loadRulesSingle(char * str){
  char * zo = str;
  while (*zo && *zo != '/') zo++;
  if (*zo!='/') return RULES_STR_ERR;
  *zo=0; zo++;
  uint8_t err = loadRules( str, zo );
  if (!err) {
    zo--;*zo='/';
    strcpy( loadedRulesString, str );
  }
  return err;
}

void checkDelayedLoadRules(){
  if (delayedLoadRules) {
    config.zone_override = 1;
    if (loadRulesSingle(preloadRulesString) !=RULES_OK) {
      config.zone_override = 0;
      if (data_valid) new_position=1;
    }
  }
  delayedLoadRules=0;
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

  } else if (countMode == COUNT_ALT) {

    if (!alt_have_pos) {
      // No usable position (no fix, no fake_longitude): dashes, digits not ticking —
      // never GMST-as-LST, never a guessed longitude. Re-evaluated every second; the
      // row goes live the moment a position appears. resendDate keeps the civil date
      // row refreshing (PendSV's resend check runs right after this) — without it the
      // date would freeze across midnight while dashed.
      resendDate = 1;
      SetPPS( &PPS_NoUpdate );
      SetSysTick( &SysTick_CountUp_NoUpdate );
      buffer_b[0] = bCat0 | 0b01000000 << 2;
      buffer_b[1] = bCat1 | 0b01000000 << 2;
      buffer_b[2] = bCat2 | 0b01000000 << 2;
      buffer_b[3] = bCat3 | 0b01000000 << 2;
      buffer_b[4] = bCat4 | 0b01000000 << 2;
      buffer_c[0].low = 0b01000000;
      buffer_c[3].low = 0b01000000;
      buffer_c[2].low = 0b01000000;
      buffer_c[1].low = 0b01000000;
      buffer_c[0].high= 0b11001110;
    } else if (currentTime - last_pps_time < config.tolerance_1ms){
      SetPPS( &PPS );
      buffer_c[0].high= 0b11001110 | cSegDP;
      SetSysTick( &SysTick_Alt_P3 );
    } else if (currentTime - last_pps_time < config.tolerance_10ms){
      SetPPS( &PPS );
      buffer_c[3].low = 0b01000000;
      buffer_c[0].high= 0b11001110 | cSegDP;
      SetSysTick( &SysTick_Alt_P2 );
    } else if (currentTime - rtc_last_calibration < config.tolerance_100ms){
      SetPPS( &PPS );
      buffer_c[3].low = 0b01000000;
      buffer_c[2].low = 0b01000000;
      buffer_c[0].high= 0b11001110 | cSegDP;
      SetSysTick( &SysTick_Alt_P1 );
    } else {
      SetPPS( &PPS );
      buffer_c[3].low = 0b01000000;
      buffer_c[2].low = 0b01000000;
      buffer_c[1].low = 0b01000000;
      buffer_c[0].high= 0b11001110;
      SetSysTick( &SysTick_Alt_P0 );
    }

  } else if (displayMode == MODE_COUNTDOWN) {

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
      countMode = COUNT_HIDDEN;
      SetSysTick( &SysTick_CountUp_NoUpdate );
      SetPPS( &PPS_NoUpdate );
      buffer_c[0].high= 0b11001110 | cSegDP;
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

#define justExited(x) ((oldMode==x) && (displayMode != x))
void nextMode(_Bool reverse){

  uint8_t oldMode = displayMode;

  if (requestMode!=255){
    if (!config.modes_enabled[requestMode]) {
      requestMode=255;
      return;
    }
    displayMode=requestMode;
    requestMode=255;
  } else if (reverse) {
    do {
      if (--displayMode >= NUM_DISPLAY_MODES) displayMode=NUM_DISPLAY_MODES-1;
    } while (!config.modes_enabled[displayMode]);
  } else {
    do {
      if (++displayMode >=NUM_DISPLAY_MODES) displayMode=0;
    } while (!config.modes_enabled[displayMode]);
  }

  if (justExited(MODE_VBAT)) vbat = 0.0;
  if (justExited(MODE_STANDBY)) displayOn();
  if (justExited(MODE_DISPLAYTEST)) {
    buffer_c[1].high &= ~cSegDP;
    buffer_c[2].high &= ~cSegDP;
    buffer_c[3].high &= ~cSegDP;
  }
  if ( displayMode == MODE_ISO_WEEK || justExited(MODE_COUNTDOWN)
       || justExited(MODE_LST) || justExited(MODE_SOLAR)) {
    // If we exit countdown/alt mode at .9 seconds
    // it will show the wrong time for .1 seconds
    setNextTimestamp(currentTime);
  }

  if (displayMode == MODE_SHOW_OFFSET || displayMode == MODE_DISPLAYTEST) {
    countMode = COUNT_HIDDEN;
    SetSysTick( &SysTick_CountUp_NoUpdate );
    SetPPS( &PPS_NoUpdate );
    colonAnimationStop()
    TIM2->CCR1 = 0; // specific to show_offset
    TIM2->CCR2 = 300;
  } else if (displayMode == MODE_COUNTDOWN) {

    if (config.countdown_to >= currentTime) {
      countMode = COUNT_DOWN;
      setNextCountdown(currentTime);
    } else {
      countMode = COUNT_HIDDEN;
      countdown_days = 0;
    }
    setPrecision();
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    latchSegments();

  } else if (displayMode == MODE_LST || displayMode == MODE_SOLAR) {

    countMode = COUNT_ALT;
    setNextTimestamp(currentTime);   // stock civil bookkeeping (integer path; countdown-arm cost)
    // NO double math here: nextMode can run in the USART2 button ISR (priority 0, which
    // blocks SysTick and the PPS EXTI), and the LST/solar computation is ~100 µs of
    // soft-double. Invalidate and let the main-loop alt_update() seed within one pass
    // (<100 ms); until then setPrecision shows the dashed state.
    alt_gen++;                       // cancels any in-flight staging for the previous timebase
    alt_stage.for_time = 0;
    alt_have_pos = 0;
    alt_seed_pending = 1;
    setPrecision();
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;

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
  applyColonForMode();   // idempotent: alt colon on entry, civil colon on exit
  sendDate(1);
}
void button1pressed(void){
  nextMode(0);
}
void button2pressed(void){
  nextMode(1);
}
void buttonsBothHeld(void){
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

  HAL_DMA_Abort(&hdma_tim1_up);
  HAL_DMA_Abort(&hdma_tim7_up);
  GPIOB->ODR=0;
  GPIOC->ODR=0;

  NVIC_SystemReset();
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

  // Enable the DWT cycle counter (free-running at the 80 MHz core clock, 12.5 ns/tick, wraps ~53.7 s):
  // the monotonic timebase the PPS edge and each USB SOF are both latched against for host correlation.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

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
  //MX_USB_DEVICE_Init();
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
  MX_USB_DEVICE_Init();

  // Enable UART2 interrupt for button presses
  USART2->CR1 |= USART_CR1_RXNEIE;


  // Configure UART1 for NMEA strings from GPS module
  USART1->CR1 |= USART_CR1_CMIE ;

  USART1->CR1 &= ~(USART_CR1_UE);
  USART1->CR2 |= '\n'<<24;
  USART1->CR1 |= USART_CR1_UE;


  MX_ADC3_Init();

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
  checkDelayedLoadRules();

  measure_vbat();

  if (RTC->ISR & RTC_ISR_INITS) //RTC contains non-zero data
  {
    RTC_DateTypeDef sdate;
    RTC_TimeTypeDef stime;

    if (!config.zone_override){
      char zone[32];
      memcpyword( (uint32_t*)zone,  (uint32_t*)&(RTC->BKP0R), 8 );
      zone[31]=0;

      if (loadRulesSingle(zone) != RULES_OK){ // takes ~8ms
        memcpyword( (uint32_t*)loadedRulesString,  (uint32_t*)&(RTC->BKP0R), 8 );
        loadedRulesString[31]=0;//paranoia
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

    // As the coin cell goes flat, the RTC stops ticking long before the backup registers die.
    // Powering on with a flat battery means the clock thinks no time has passed, and assumes it has good precision.
    // Explicitly stop this by checking the battery voltage.
    if (vbat > 2.70) {
      rtc_good=1;
    } else {
      // trash the calibration time to ensure lowest precision display
      if (currentTime - rtc_last_calibration < config.tolerance_100ms)
        rtc_last_calibration -= config.tolerance_100ms +1;
    }

  } else { // backup domain reset

    currentTime=946684800; // 2000-01-01T00:00:00

    // The init process blanks the subsecond registers
    MX_RTC_Init();
  }

  vbat = 0.0; // don't allow measurement to go stale

  setPrecision();
  PPS_Init();
  HAL_UART_Receive_DMA(&huart1, nmea, sizeof(nmea));

//#define MEASURE_LOOKUP_TIME

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Distance gate: skip the ~300 ms FATFS/ZoneDetect lookup unless the fix has actually moved far
    // enough to plausibly change zone. 0.005° ≈ 0.5 km — ~100× the metre-scale jitter of a stationary
    // clock, yet far finer than any timezone boundary, so a moving clock still re-detects its zone
    // within ~0.5 km of a crossing while a still one looks up exactly once. (dlat²+dlon² threshold
    // 2.5e-5 = 0.005²; the cos-lat foreshortening of longitude only makes the gate MORE conservative.)
    if (new_position && !qspi_write_time && !config.zone_override
        && (data_valid || (config.fake_long && config.fake_lat))
        && latitude>=-90.0 && latitude<=90.0 && longitude>=-180.0 && longitude<=180.0
        && (zone_lat>90.0 || (latitude-zone_lat)*(latitude-zone_lat) + (longitude-zone_lon)*(longitude-zone_lon) > 2.5e-5)) {

      new_position=0;
      zone_lat=latitude; zone_lon=longitude;
      fatfs_busy=1;   // map lookup + loadRulesSingle touch FATFS; block the eject-time check
      FIL mapfile;
      if (f_open(&mapfile, MAP_FILENAME, FA_READ) == FR_OK) {
#ifdef MEASURE_LOOKUP_TIME
        uint32_t start=uwTick;
#endif
        ZoneDetect *const zdb = ZDOpenDatabase(&mapfile);

        if (!zdb) {
          // mapfile error
        } else {
          char* zone = ZDHelperSimpleLookupString(zdb, latitude, longitude);
#ifdef MEASURE_LOOKUP_TIME
          uint32_t ztime=uwTick-start;
#endif
          if (zone && !delayedLoadRules) {
#ifdef MEASURE_LOOKUP_TIME
            start=uwTick;
#endif
            loadRulesSingle(zone);
#ifdef MEASURE_LOOKUP_TIME
            sprintf(textDisplay,"d%ld L%ld",ztime, uwTick-start);
#endif
          }
          free(zone);
          ZDCloseDatabase(zdb);
          //f_close(&mapfile);
        }
      }
      // else no_map = 1
      fatfs_busy=0;
    }

    if (delayedCheckOnEject) firmwareCheckOnEject();

    if (delayedPostConfigCleanup) {
      delayedPostConfigCleanup=0;
      postConfigCleanup();
    }

    fatfs_busy=1;   // FATFS_remount + readConfigFile + checkDelayedLoadRules touch FATFS
    if (delayedReadConfigFile) {
      FATFS_remount();
      readConfigFile();
      delayedReadConfigFile=0;
    }

    checkDelayedLoadRules();
    fatfs_busy=0;

    if (delayedDisplayFreq) setDisplayFreq(delayedDisplayFreq);

    monitor_vbus();

    if (pps_ts_enabled) {
      static uint32_t last_temp_read = 0;
      if ((uint32_t)currentTime - last_temp_read >= 4) {   // refresh die temp every ~4 s
        last_temp_read = (uint32_t)currentTime;
        measure_temp();
      }
      if (pps_record_pending) emitPPSTimestamp();           // emit clears pending itself on success
    }

    if (displayMode == MODE_VBAT)
      measure_vbat();

    if (displayMode == MODE_SUN  || displayMode == MODE_SUN_AZEL || displayMode == MODE_MOON
        || displayMode == MODE_GRID || displayMode == MODE_LATLON || displayMode == MODE_DARK) {
      astro_update();
      // honour the ms page dwell: the date row otherwise only repaints at 1 Hz, so repaint
      // the moment a paged mode flips sub-screen. Only with a fix (no-fix shows a
      // page-independent "----"), and never in the last decisecond -- there the SysTick ISR
      // runs its own (non-reentrant, shared-UART) sendDate(0), so we'd race it. Same
      // decisec!=9 guard the existing main-loop sendDate(1) calls use.
      if ((displayMode == MODE_SUN || displayMode == MODE_LATLON || displayMode == MODE_DARK) && astro.have_pos && astro.epoch) {
        static uint32_t last_pg = 0;
        uint32_t pg = uwTick / page_ms();
        if (pg != last_pg && decisec != 9) { last_pg = pg; sendDate(1); }
      }
    }

    // MODE_LST / MODE_SOLAR: stage the next civil boundary's alternate reading
    // (thread-context doubles; no-op in every other mode)
    alt_update();


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
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */


  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.NbrOfDiscConversion = 1;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = ENABLE;
  hadc3.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc3.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc3.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;

  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  ADC123_COMMON->CCR &= ~ADC_CCR_VBATEN;
  /* USER CODE END ADC3_Init 2 */

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

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
