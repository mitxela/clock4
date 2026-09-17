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

extern _Bool delayedReadConfigFile;
extern _Bool delayedCheckOnEject;
extern volatile uint8_t fatfs_busy;
extern volatile uint32_t settings_map_gen;   // bumped by STORAGE_Write_FS: host wrote the QSPI chip

extern _Bool waitingForLatch;
extern _Bool resendDate;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define RULES_FILENAME  "/TZRULES.BIN"
#define STARS_FILENAME  "/STARS.BIN"
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

// NMEA 0183 messages have a max length of 82 characters; the extended $PMTXTS (with the SOF-
// correlation tail: dwt_pps, sof_frame, dwt_sof) runs ~110, so this sizes the tx/rx buffers for it.
#define NMEA_BUF_SIZE 128

#define CMD_LOAD_TEXT          0x90
#define CMD_SET_FREQUENCY      0x91
#define CMD_RELOAD_TEXT        0x92
#define CMD_SHOW_CRC           0x9D

// ---- On-device 2-button menu ------------------------------------------------------------------
// Inbound date-board button events (date board -> this MCU over USART2). 0x91/0x92 are today's
// tap/hold-autorepeat, UNCHANGED. 0x93 was "both held -> reset"; it is re-tasked to "chord release"
// (fire the shown stage). 0x94/95/96 are NEW rolling chord-stage crossings. Note the same byte
// values are OUTBOUND display opcodes in the other direction (0x92 CMD_RELOAD_TEXT etc.) — no
// conflict, opposite wire direction. BACKWARD COMPAT: a stock date board emits only 0x91/0x92/0x93,
// so with menu_chord==0 a bare 0x93 at L0 still triggers the legacy reset and the menu stays dormant
// until a companion mk4-date firmware emits the 0x94/95/96 chord protocol.
#define EVT_BTN1        0x91
#define EVT_BTN2        0x92
#define EVT_CHORD_REL   0x93
#define EVT_CHORD_S1    0x94
#define EVT_CHORD_S2    0x95
#define EVT_CHORD_S3    0x96
#define MENU_IDLE_MS    15000u

typedef enum { MIT_TOGGLE, MIT_ENUM, MIT_STEP, MIT_ACTION, MIT_INFO } MItemType;   // MIT_ACTION: one-shot command (confirm at L3). MIT_INFO: read-only live readout (no editor; .lo = info kind)
// v2 sections: the setup ring is grouped; physical table order is UNCHANGED (menu_idx stays absolute,
// persistence keys off key_id) — the FSM just walks rows whose .section matches the entered section.
enum { SEC_CAL=0, SEC_ASTRO, SEC_DISP, SEC_DIAG, SEC_SYS, NSEC };
typedef struct MItem {
  uint8_t     key_id;          // STABLE persistence id (KID_*), never renumbered
  MItemType   type;
  const char *label;           // <=10 chars, shown on the date row
  int32_t     lo, hi, step;    // STEP bounds+increment; ENUM uses 0..hi (step ignored). i32: page_ms hi=60000 / matrix hi=100000 overflow i16
  const char *const *enums;    // ENUM label array [0..hi], else NULL
  int32_t   (*get)(const struct MItem *m);
  void      (*set)(const struct MItem *m, int32_t v);   // write global + apply live effect
  uint8_t     section;         // SEC_* — which setup section this row lives in
} MItem;
// Stable key ids (append-only; mode rows use KID_MODE_BASE + MODE_* ordinal).
enum { KID_BRIGHTNESS=1, KID_COLON=2, KID_COLON_ALT=3, KID_PAGE_MS=4,
       KID_SIG_FADE=5, KID_PPS=6, KID_NMEA=7, KID_MATRIX_FREQ=8, KID_TEMPCOMP=9,
       KID_BALANCE=10, KID_RESET=11, KID_CUCKOO=12, KID_MODE_BASE=64 };   // KID_RESET: the SYS factory-reset action (never persisted)

void menu_isr_event(uint8_t evt);   // called from the USART2 ISR — enqueue only, no work
void menu_poll(void);               // main-loop FSM tick
void ee_load(void);                 // boot: derive the EEPROM base, scan flash -> RAM override store
void menu_apply_overrides(void);    // merge the override store into live config (config.txt precedence)

//#define NONCOMPLIANT_DATE_MODES

enum {
  MODE_ISO8601_STD =0,
  MODE_ISO_ORDINAL,
  MODE_ISO_WEEK,
  MODE_UNIX,
  MODE_JULIAN_DATE,
  MODE_MODIFIED_JD,
  MODE_SHOW_OFFSET,
  MODE_SHOW_TZ_NAME,
  MODE_WEEKDAY,
  MODE_WEEKDA_DD,
  MODE_WDY_MM_DD,
  MODE_STANDBY,
  MODE_COUNTDOWN,
  MODE_SATVIEW,
  MODE_DEBUG_BRIGHTNESS,
  MODE_DEBUG_RTC,
  MODE_TEXT,
  MODE_FIRMWARE_CRC_T,
  MODE_FIRMWARE_CRC_D,
  MODE_VBAT,
  MODE_DISPLAYTEST,
  MODE_TTFF,
#ifdef NONCOMPLIANT_DATE_MODES
  MODE_DDMMYYYY,
#endif

  // Astro pack — GPS-derived astronomy read-outs. SATVIEW-style: the payload
  // shows on the 10-char date row while the live clock keeps running on the
  // time row. Enabled individually via the MODE_* config keys, like any mode.
  MODE_SUN,        // sunrise / sunset / solar noon (local), auto-paged
  MODE_SUN_AZEL,   // sun azimuth & elevation, now
  MODE_MOON,       // moon phase index + illuminated %
  MODE_GRID,       // Maidenhead grid locator
  MODE_LATLON,     // latitude / longitude, auto-paged

  // Observing-session twilight ladder: civil / nautical / astronomical dusk times and a live countdown
  // to astronomical darkness (sun -18 deg). Answers "when is it actually dark, and for how long." Honest:
  // high-latitude white nights that never reach -18 report NO DARK; polar night reports DARK NOW.
  MODE_DARK,

  // Temperature-compensation diagnostics: die temp / model offsets / sample count
  // paged on the date row (satview pattern). Values come from the tempcomp module.
  MODE_TEMPCOMP,

  // Alternate-timebase TIME-ROW modes: the big digits tick Local Sidereal Time or
  // apparent solar time, reseeded from the GPS-disciplined second; the
  // date row keeps the civil date and a dedicated colon animation marks the mode.
  MODE_LST,
  MODE_SOLAR,

  // Live Allan deviation of the free-running crystal, sigma_y(tau) paged across octave
  // taus (1,2,4,...,1024 s) on the date row. Date-row diagnostic (satview pattern); the
  // time row keeps live GPS time. Enabled via the MODE_ADEV config key like any mode.
  MODE_ADEV,

  // Bright-star meridian-transit predictor: the soonest bright stars to cross the local
  // meridian (culminate), paged as "<name> <h:mm>" countdowns on the date row. Uses the
  // GPS fix + local_sidereal_time (transit when LST == RA). Gated by the MODE_STAR key.
  MODE_STAR,

  NUM_DISPLAY_MODES
};

enum {
  COUNT_NORMAL =0,
  COUNT_HIDDEN,
  COUNT_DOWN,
  COUNT_ALT      // time row driven by the alternate timebase (MODE_LST / MODE_SOLAR)
};

enum {
  COLON_MODE_SLOWFADE = 0,
  COLON_MODE_HEARTBEAT,
  COLON_MODE_1PPS_SAWTOOTH,
  COLON_MODE_ALT_SAWTOOTH,
  COLON_MODE_TOGGLE,
  COLON_MODE_SOLID
};

enum {
  NMEA_ALL=0,
  NMEA_RMC,
  NMEA_NONE
};

enum {
  RULES_OK=0,
  RULES_STR_ERR,
  RULES_NO_FILE,
  RULES_HEADER_ERR,
  RULES_VERSION_UNKNOWN,
  RULES_CATEGORY_UNKNOWN,
  RULES_ZONE_UNKNOWN
};

enum {
  SV_GPS_L1=0,
  SV_GPS_UNKNOWN,
  SV_GLONASS_L1,
  SV_GLONASS_UNKNOWN,
  SV_GALILEO_E1,
  SV_GALILEO_UNKNOWN,
  SV_BEIDOU_B1,
  SV_BEIDOU_UNKNOWN,
  SV_COUNT
};
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define byteswap32(x) \
   ( ((x & 0xff000000) >> 24) | ((x & 0x00ff0000) >> 8) \
   | ((x & 0x0000ff00) <<  8) | ((x & 0x000000ff) << 24))

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void decodeRMC(void);
void decodeGSV(uint8_t rec);
void setDisplayPWM(uint32_t bright);
void write_rtc(void);
void displayOff(void);
void button1pressed(void);
void button2pressed(void);
void buttonsBothHeld(void);
void setPrecision(void);
void sendDate( _Bool now );
void generateDACbuffer(uint16_t * buf);
void PPS(void);
void PPS_NoUpdate(void);
void PPS_Countdown(void);
void rxConfigString(char c);
void monitor_vbus(void);

#define latchSegments() \
  buffer_c[0].low = next7seg.c; \
  buffer_b[0] = next7seg.b[0]; \
  buffer_b[1] = next7seg.b[1]; \
  buffer_b[2] = next7seg.b[2]; \
  buffer_b[3] = next7seg.b[3]; \
  buffer_b[4] = next7seg.b[4];

#define triggerPendSV() \
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

#define sendLatch() \
  huart2.Instance->TDR = 0xFE;

#define loadNextTimestamp() \
  latchSegments() \
  sendLatch() \
  waitingForLatch=0;\
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
