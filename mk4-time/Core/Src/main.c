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
static uint8_t adev_disp_noct(void);        // Allan-deviation display accessors (engine defined far below;
static float   adev_disp_sigma(uint8_t k);  // sendDate() lives above the engine, so read via these)
static volatile uint32_t adev_last_ms;      // uwTick of the last accepted sample (staleness/HOLD tell)
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

// --- Temperature compensation (opt-in) ------------------------------------------------------
// Learns ppm-vs-die-temperature for both oscillators while GPS-locked (tc_learn), then during
// GPS-loss holdover steers the SysTick timebase from the HSE model (tc_apply) and optionally
// trims RTC->CALR from the LSE model (tc_rtc) so the battery RTC hands over better time across
// a power loss. "tc_dump = on" over serial prints the learned coefficients as ready-to-paste
// config lines; non-NAN tc_hse_a/tc_lse_a in config freeze the model (config overrides learning).
// All defaults off: with none of the keys set, behaviour is identical to stock.
// Config-key scalars are written from the USB OTG ISR (parseConfigString) and read by the
// main loop: volatile, matching the pps_ts_enabled precedent.
volatile _Bool tc_learn = 0, tc_apply = 0, tc_rtc = 0;
// Holdover fade (opt-in). A sub-second digit whose accuracy can no longer be held — its time
// uncertainty during GPS-loss holdover has grown past that digit's place value — FADES to black by
// its remaining significance instead of dashing, overriding the fixed Tolerance_time_* ladder.
// digit_bright[] holds per-digit intensity 0..FADE_MAX for the [deciseconds, centiseconds,
// milliseconds, decimal-point] positions (FADE_MAX = fully lit, i.e. certainly significant).
// Default 0 = NOT significant: at power-on nothing is disciplined yet (had_pps=0, holdover age is
// huge), so with significance_fade the sub-second digits must start DASHED and only light once the
// first computeHoldoverFade() proves significance — else setPrecision() at boot reads a stale FADE_MAX
// and flashes ticking numbers for ~1 s before the first per-second recompute dashes them.
volatile _Bool significance_fade = 0;
#define FADE_MAX 16
uint8_t digit_bright[4] = { 0, 0, 0, 0 };
float holdover_u_us = 0.0f;                 // last computed 3σ time-interval-error bound U(τ), µs
volatile int16_t  tc_t0 = 40;              // model centre temperature (°C)
volatile uint16_t tc_engage_s = 2;         // seconds of PPS absence before steering engages (min 2)
volatile uint16_t tc_max_ppm = 100;        // hard clamp on the applied correction magnitude
// Frozen coefficients (ppm units at tc_t0). Elements are single-word (atomic) reads/writes;
// consumers snapshot each element once. HSE has NO 'a': its learned origin is arbitrary and
// steering uses temperature differences only, so freezing needs just b (and optionally c).
float tc_cfg_hse[3] = {NAN, NAN, NAN};     // [0] unused, [1] ppm/°C, [2] ppm/°C²
float tc_cfg_lse[3] = {NAN, NAN, NAN};     // absolute: ppm, ppm/°C, ppm/°C²
volatile _Bool tc_dump_pending = 0;        // set by the serial parser, serviced in the main loop
volatile _Bool tc_reset_pending = 0;
volatile _Bool adev_dump_pending = 0;      // "adev_dump = on" over serial -> emit one $PMADEV sentence
volatile _Bool hdev_dump_pending = 0;      // "hdev_dump = on" over serial -> emit one $PMHDEV sentence (Hadamard)

// Validated coefficient parse: garbage/'----'/empty leaves the value untouched (a pasted-back
// commented dump line must not freeze 0.0); an explicit "nan" parses and UNFREEZES the slot.
static void tc_parse_coeff(const char *v, float *out){
  char *end;
  float f = strtof(v, &end);
  if (end != v) *out = f;
}

// Steering handoff, governor (main loop) -> tick ISR. base/rem are written together under
// IRQ-off; the ISR Bresenham distributes `rem` one-tick-longer periods per 1000 ms so the
// average period is (tc_load_base+1) + rem/1000 ticks — fractional-ppm rate steering.
volatile uint8_t tc_steer_on = 0;
volatile int32_t tc_load_base = 0;         // SysTick->LOAD for the shorter of the two periods
volatile int32_t tc_rem = 0;               // extra-tick remainder, always in [0,1000)
volatile int32_t tc_acc = 0;               // Bresenham accumulator (ISR-owned)

// Learned state (main-loop only). 2 °C bins spanning die temp -8..71 °C; sums are bounded by
// the halving-at-32768 aging rule (max |sum| ~ 6400*32768 < 2^31), so int32 cannot overflow.
struct tc_bin { int32_t hse_sum, lse_sum; uint16_t hse_n, lse_n; };
struct tc_bin tc_bins[40];
float tc_hse_m[3], tc_lse_m[3];            // learned models (ppm at powers of T - tc_t0)
_Bool tc_hse_valid = 0, tc_lse_valid = 0;
int16_t tc_hse_tmin = 0, tc_hse_tmax = 0;  // learned coverage: model is clamped to this range
int16_t tc_lse_tmin = 0, tc_lse_tmax = 0;
uint32_t tc_n_hse = 0, tc_n_lse = 0;       // lifetime sample counts (display + dump)
// Weighted-RMS residual of each learned fit (model vs bin means): the model's OWN error, in its
// fit units (HSE: SysTick ticks/s; LSE: ppm). This is the holdover-fade uncertainty's σ_temp source
// — how far the temperature model actually is from the measured data, not a guess.
float tc_hse_resid = 0, tc_lse_resid = 0;

// Warm-start (seed-and-evolve). A previously-learned model — the last tc_dump, written back into
// config.txt by the host (the firmware never writes the filesystem; the QSPI drive is host-owned) —
// is reloaded at boot as an evolving PRIOR rather than a hard freeze. The clock is temperature-
// compensated from the first second and keeps refining: tc_hse_prior/tc_lse_prior hold the model
// ORDER (1 const, 2 linear, 3 quadratic) currently carried by the seed, and tc_fit keeps that model
// until real learning supports a fit at least as rich, then hands over. tc_seed = off leaves the
// existing freeze path (tc_hse_b/… as asserted constants) untouched.
volatile _Bool   tc_seed = 0;                 // config: warm-start from the seeded coefficients + evolve
volatile int16_t tc_seed_lo = 0, tc_seed_hi = 0;   // seed coverage (die °C): bounds the prior — never extrapolated
uint8_t tc_hse_prior = 0, tc_lse_prior = 0;   // seed model order still held (0 = handed over to real data)
_Bool tc_seed_done = 0;                        // one-shot: seed once per power-on (BSS-cleared at reset)
// Serial "tc_seed = on" arms this in the USB ISR; tc_housekeeping consumes it in the MAIN LOOP.
// The learned-state contract ("main-loop only") holds: the ISR only ever touches this one flag
// (plus the already-accepted single-word tc_cfg_*/tc_seed slots) — never the model itself.
volatile _Bool tc_seed_pending = 0;
static void tc_seed_apply(void);              // defined by the tempcomp block; called after the config load

// Display cache for MODE_TEMPCOMP. Written by the governor (main loop); read by sendDate,
// which ALSO runs from the SysTick ISRs — each field is a single 32-bit (atomic) access, so
// the worst case is a one-repaint-stale value pairing, never a torn read.
float tc_disp_hse = 0, tc_disp_lse = 0;
_Bool tc_disp_hse_ok = 0, tc_disp_lse_ok = 0;
char  tc_disp_state = '-';                 // A applying · F frozen (config) · L learning · - idle

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
    // Measured VTT9812FH (R11 = 470K) response, baked in so the clock needs no config.txt BS lines.
    // These are the same five stops as "BSn = in,out" (out is inverted, 4095-out, exactly as
    // parseBrightness does); a config.txt BS line still overrides its stop at load.
    {0,    4095-0},       // BS1 = 0,0
    {131,  4095-365},     // BS2 = 131,365
    {1076, 4095-1422},    // BS3 = 1076,1422
    {2774, 4095-2665},    // BS4 = 2774,2665
    {3849, 4095-4095},    // BS5 = 3849,4095
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
  case MODE_TEMPCOMP: {
    // Pages: die temp -> HSE model -> LSE model -> samples+state, page_ms dwell each.
    // Values are the governor's display cache (clamped so the row never overflows). Layout is
    // the RISE/SET style: label, separator space, a sign slot (space when positive), then the
    // digits — numbers align whether signed or not, and short values keep clear space at the
    // row's end beside the time row: "tC  32C" / "HSE -0.25" / "rtC  18.68" / "n 159 L".
    int tcp = (int)((uwTick / page_ms()) % 4);
    char num[12];
    if (tcp == 0) {
      int t2 = (int)die_temp_c;
      i = sprintf((char*)&uart2_tx_buffer[1], "tC %c%dC", t2 < 0 ? '-' : ' ', t2 < 0 ? -t2 : t2);
    } else if (tcp == 1 || tcp == 2) {
      _Bool ok = (tcp == 1) ? tc_disp_hse_ok : tc_disp_lse_ok;
      float v = (tcp == 1) ? tc_disp_hse : tc_disp_lse;
      if (!ok) i = sprintf((char*)&uart2_tx_buffer[1], "%s ----", (tcp == 1) ? "HSE" : "rtC");
      else {
        sprintf(num, "%.2f", (double)(v < 0 ? -v : v));
        i = sprintf((char*)&uart2_tx_buffer[1], "%s %c%s", (tcp == 1) ? "HSE" : "rtC", v < 0 ? '-' : ' ', num);
      }
    } else {
      unsigned long ns = tc_n_hse > 999999UL ? 999999UL : tc_n_hse;
      i = sprintf((char*)&uart2_tx_buffer[1], "n%6lu %c", ns, tc_disp_state);
    }
    break;
  }
  case MODE_ADEV: {
    // sigma_y(tau) paged one octave per page_ms dwell; tau = 2^page seconds. Until the ring has
    // enough contiguous 1 s samples for even tau=1 (needs 3), show a filling marker. The layout is
    // a compact scientific that always fits the 10-char row: "<tau>s <sigma>", mantissa dropped as
    // the tau label grows ("1s 3.2e-11" / "64s 3e-11" / "1024s3e-11"). Fractional-frequency values.
    uint8_t noct = adev_disp_noct();
    if (noct == 0) {
      i = sprintf((char*)&uart2_tx_buffer[1], "Adev ----");   // filling, or PPS not locked yet
    } else if ((uint32_t)(uwTick - adev_last_ms) > 2500u) {
      i = sprintf((char*)&uart2_tx_buffer[1], "Adev HOLd");   // holdover: no fresh samples — the curve would be a frozen replay
    } else {
      uint32_t k   = (uwTick / page_ms()) % noct;
      uint32_t tau = 1u << k;
      double   s   = (double)adev_disp_sigma((uint8_t)k);
      char sig[12];
      sprintf(sig, (tau < 10) ? "%.1e" : "%.0e", s);          // room for a mantissa digit only when tau is short
      i = sprintf((char*)&uart2_tx_buffer[1], "%lu %s",       // NO unit-s ('s' == the '5' glyph: "1024s3e-11" read as 10245...) and ALWAYS a separator
                  (unsigned long)tau, sig);
    }
    break;
  }
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

uint16_t display_scan_len = 5;   // last DMA transfer count set here — segbal_poll steers 5 <-> 80

void setDisplayPWM(uint32_t bright){
  display_scan_len = (uint16_t)bright;
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

// --- Per-segment brightness balance (seg_balance) --------------------------------------------
// The display is voltage-driven (buffer chips + 10R per segment; the DAC sets the rail), so all
// lit segments of a digit share the drop across that digit's common return path: FEWER lit
// segments leaves more voltage per LED, so '-', '1' and '7' glow visibly brighter than '8' —
// worst at low brightness where the rail sits just above the LED forward voltage. There is no
// per-digit analog knob, but there is headroom in the scan: buffer_b/buffer_c are sized [80] and
// the DMA is circular, so the 5-step scan can be replayed as 16 cycles of 5 slots. Slots 0..4
// stay the live "master" slots every existing writer already targets; segbal_poll() (main loop,
// at most 1 kHz) mirrors them into slots 5..79 with each digit lit in s of its 16 cycles,
//     s = 16 - strength * (16 - 2 * popcount(segments)) / 100
// so at full strength s = 2N and every lit segment averages the same duty-per-segment product:
// (1/N) * (2N/16) = 1/8, uniform across the display (an 8-segment '8.' keeps 16/16 — sparse
// digits dim DOWN to its level, dense digits are untouched). The cycle mask picks each digit's s
// lit cycles as a NESTED rank window: rank = bit-reversed k rotated by a fixed per-digit phase
// (cycle 0 — the master slot — pinned at rank 0, so masters are never rewritten and count toward
// the duty exactly). Two properties are load-bearing, and each was a hardware-visible artefact
// when missing:
//  - NESTED (s+1's lit set = s's plus exactly one cycle): s changes whenever a digit's segment
//    count does — every second boundary at least — and a spread that RESHUFFLES on s±1 (e.g.
//    (k*s) % 16 < s, the first ship) re-phases the whole column's light at that instant: a
//    once-per-second step in the ghost bleed of unlit segments + a beat on the sub-second digits.
//  - DECORRELATED (distinct phase per digit): un-rotated nested ranks light every digit on the
//    SAME low-rank cycles, so the shared rail sees a sawtooth (all-on cycles vs sparse cycles,
//    per-cycle load variance ~14 vs ~1.4 rotated in sim) whose shape steps with the time content
//    — the whole DATE row blipped at 1 Hz and per-digit brightness left calibration (the K curve
//    was eyeballed against decorrelated patterns).
// Positions stay near-evenly spread at every s (a rotated rank window bit-reverses to a van-der-
// Corput run; worst case s = 1 is the master alone, unchanged from stock 5-slot timing).
//
// Above 100 the map turns into a power law, s = 16*(N/8)^(strength/100) — continuous at 100
// (gamma 1 = the linear map). Near the LED forward-voltage knee (low rail) segment current is
// exponential in voltage, so the real bloom ratio between a '-' and an '8.' can exceed what any
// LINEAR duty map can counter; hardware A/B showed 100 under-correcting at low brightness. The
// exponent steepens the curve: at 200, a 1-segment digit runs 1/16 duty against the 8-segment
// digit's 16/16. Tune live over serial against the actual display.
// Off (0, the default) leaves the stock 5-slot scan and all timing untouched.
//
// The bloom is rail-dependent, and NOT monotonically: hardware calibration (eyeball-matched at
// four rail levels) found a valley — strong compensation needed at the dim end (the LED knee,
// where current is exponential in voltage), moderate again at full rail (maximum-current IR drop
// in the shared return), mild in between. "seg_balance = on" applies that measured curve,
// interpolated by the live rail (dac_target, 4095 = dimmest); it is the whole intended interface.
// A numeric value (2..300) instead applies a fixed manual strength for experiments.
// seg_balance = 0/off (the default) keeps the stock scan and stock timing untouched.
volatile uint16_t seg_balance = 0;     // 0 = off (stock) · 1/on = AUTO (calibrated curve) · 2..300 = fixed manual strength

#define SEGBAL_BSEG_MASK 0x01FCu       // GPIOB word: bits 2..8 are segments; everything else
                                       // (bCat column selects etc.) passes through unmasked

// rank(k) for D=16 (bit-reversed k). For D=8/4 shift right by 1/2 — dropping the low bits of the
// reversal is exactly the 3-/2-bit reversal, so one table serves every depth.
static const uint8_t SEGBAL_REV16[16] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};
// Per-digit phases for the mirror ranks (col x bank; DP rides its digit's phase so its cycles
// stay a subset of the digit's). Rank 0 = the master slot is pinned; the phase rotates ranks
// 1..D-1 among themselves (mod D-1), so every digit still lights the master and its mirrors sit
// in a distinct rank window. Values are spread over the 15-ring (idx*4 mod 15, all distinct).
static const uint8_t SEGBAL_PH_B[5] = {0, 4, 8, 12, 1};
static const uint8_t SEGBAL_PH_C[5] = {5, 9, 13, 2, 6};

// Mirror freshness across main-loop stalls. The sub-second MASTERS are written from the SysTick
// ISR (SysTick_CountUp_*), so the stock 5-slot scan never lags the main loop — but the mirrors
// carry (D-1)/D of the light when balancing, and a refill that lives only in the main loop goes
// stale for the length of any long pass (the once-per-second PendSV display prep, housekeeping,
// a flash commit): hardware showed the sub-second digits freezing once per second, present since
// the first segbal ship and independent of the dither order. segbal_poll() therefore EXPORTS the
// per-column lit-cycle bitmaps, and every SysTick tick re-copies the live master values through
// them (segbal_isr_refresh), pinning mirror VALUE freshness to the same 1 ms the masters get.
// The bitmaps may lag a main-loop pass behind a glyph change — a duty lag of one pass,
// imperceptible where a frozen digit was not.
static volatile uint16_t segbal_lit_b[5], segbal_lit_c[5], segbal_lit_dp[5];  // bit k = cycle k lit
static volatile uint8_t  segbal_mirror_live = 0;   // ISR refresh armed (D-cycle scan up, bitmaps valid)

// Lit cycles (of 16) for a digit with n lit segments at effective strength `eff` (0..300).
// n <= 8 always (7 segments + DP). Returns 16 (always lit) .. 1 (floor for any lit digit).
static uint32_t segbal_duty(uint32_t n, uint32_t eff){
  if (n == 0) return 0;
  if (eff <= 100)                                          // linear blend: 100 -> s = 2n exactly
    return 16u - eff * (16u - 2u*n) / 100u;
  float s = 16.0f * powf((float)n / 8.0f, (float)eff / 100.0f);   // power law, gamma > 1
  uint32_t si = (uint32_t)(s + 0.5f);
  return si < 1u ? 1u : (si > 16u ? 16u : si);
}

// Effective strength for THIS refill. AUTO (seg_balance = 1/on) follows the hardware calibration:
// a full eyeballed sweep across the whole rail on a production Mk IV (2026-07-11) landed the even
// point on a clean EXPONENTIAL — the round-number rails fell on a x3-per-half-brightness geometric
// progression (rail brightest -> 10, mid -> 30, dimmest -> 90), i.e. K = 10 * 9^(dac/4095). That is
// exactly the LED knee: near the bottom of the rail, segment current is exponential in forward
// voltage, so a sparse digit pulls away exponentially fast and needs exponentially more duty haircut.
// 4095 is the 12-bit DAC full-scale code (dac_target maxes there, matching the original firmware); the
// intermediate breakpoints below stay on clean powers of two and K evaluates to the measured anchors.
// Sampled to a 9-point LUT (cheaper than a per-refill powf, and exponentials interpolate linearly to
// well under 1 K). A numeric value (2..300) instead applies a fixed manual strength for experiments.
// seg_balance = 0/off (the default) keeps the stock scan and stock timing untouched.
static const uint16_t SEGBAL_AUTO_DAC[9] = { 0, 512, 1024, 1536, 2048, 2560, 3072, 3584, 4095 };
static const uint16_t SEGBAL_AUTO_K[9]   = { 10,  13,   17,   23,   30,   39,   52,   68,   90 };  // 10*9^(dac/4095)

static uint32_t segbal_strength(void){
  if (seg_balance != 1) return seg_balance;                // manual fixed strength, or 0 = off
  int32_t d = (int32_t)dac_target;
  if (d <= SEGBAL_AUTO_DAC[0]) return SEGBAL_AUTO_K[0];
  for (uint32_t i = 1; i < 9; i++) {
    if (d <= (int32_t)SEGBAL_AUTO_DAC[i]) {
      int32_t d0 = SEGBAL_AUTO_DAC[i-1], d1 = SEGBAL_AUTO_DAC[i];
      int32_t k0 = SEGBAL_AUTO_K[i-1],   k1 = SEGBAL_AUTO_K[i];
      return (uint32_t)(k0 + (k1 - k0) * (d - d0) / (d1 - d0));
    }
  }
  return SEGBAL_AUTO_K[8];
}

// Dither depth (cycles per column) for the CURRENT scan rate. display_frequency is a user config
// (1..100 kHz) that retunes TIM1, and a fixed 16-cycle dither at a slow scan would visibly strobe:
// the dither repeats at step/(5*D) Hz, so pick the deepest D of {16,8,4} that stays >= ~200 Hz,
// and return 0 (balancing unavailable) below ~4 kHz steps. Stock rate (~62 kHz steps) gives D=16.
static uint32_t segbal_depth(void){
  uint32_t step = 16000000u / (TIM1->ARR + 1u);            // TIM1 clock 16 MHz (see setDisplayFreq)
  if (step >= 16000u) return 16u;
  if (step >=  8000u) return 8u;
  if (step >=  4000u) return 4u;
  return 0u;
}

// Forward the duty table (ALWAYS on the 0..16 scale — the date board rescales to its own depth)
// to the DATE BOARD over the shared UART, so both rows equalise together from the one config key.
// Sent from the main loop only when the UART is idle and the date board is not inside its latch
// window (its RX is disabled there — bytes would be lost). The date board commits the 9 bytes
// atomically, so an interrupted frame is harmless; a periodic re-send makes delivery eventual.
#define CMD_SET_SEG_BALANCE 0x94
static void segbal_forward(uint32_t eff){
  static uint32_t eff_sent = 0xFFFFFFFFu;
  static uint16_t resend = 0;
  static uint8_t  tx[10];
  if (eff == eff_sent && ++resend < 2048u) return;         // re-assert every ~2 s (lost-frame heal)
  if (waitingForLatch || huart2.gState != HAL_UART_STATE_READY) return;   // retry a later poll
  tx[0] = CMD_SET_SEG_BALANCE;
  for (uint32_t n = 0; n <= 8u; n++) tx[1 + n] = (uint8_t)segbal_duty(n, eff);
  if (HAL_UART_Transmit_DMA(&huart2, tx, 10) == HAL_OK) { eff_sent = eff; resend = 0; }
}

void segbal_poll(void){
  static uint16_t last_ms = 0xFFFF;

  if (displayMode == MODE_STANDBY) { segbal_mirror_live = 0; return; }   // display is off — never (re)start its DMA here

  uint32_t D = segbal_depth();
  uint32_t eff = (seg_balance && D) ? segbal_strength() : 0;
  segbal_forward(eff);                              // keep the date row in step (0 = identity/off)

  // Gradual significance fade — the per-digit dimmer PR #9 anticipated. While any sub-second
  // digit is mid-fade, run the mirror even with seg_balance off (identity duty) so digit_bright
  // (0..FADE_MAX, recomputed each second from the live U(τ)) can scale each digit's cycles: the
  // real display renders the partial fade the emulator always could. Columns c1/c2/c3 = ds/cs/ms;
  // the decimal point (c0.high, cSegDP) follows digit_bright[3]. Big digits never fade.
  uint32_t fading = significance_fade && D && countMode == COUNT_NORMAL &&
                    (digit_bright[0] < FADE_MAX || digit_bright[1] < FADE_MAX ||
                     digit_bright[2] < FADE_MAX || digit_bright[3] < FADE_MAX);

  if (!eff && !fading) {
    segbal_mirror_live = 0;                         // stop the ISR refresh before dropping the scan
    if (display_scan_len != 5) setDisplayPWM(5);    // live-disable: back to the stock scan
    return;
  }

  // Refill at most once per ms: the masters change at most that fast (the sub-second digits
  // exactly that fast), and the mirror may lag a main-loop pass behind them harmlessly.
  uint16_t ms = (uint16_t)decisec*100 + (uint16_t)centisec*10 + millisec;
  uint16_t want_len = (uint16_t)(5u * D);
  if (ms == last_ms && display_scan_len == want_len) return;
  last_ms = ms;

  uint32_t revsh = (D == 16u) ? 0u : (D == 8u) ? 1u : 2u;  // SEGBAL_REV16 shift for this depth
  for (uint32_t col = 0; col < 5; col++) {
    uint16_t mb = buffer_b[col];
    uint8_t  ml = buffer_c[col].low, mh = buffer_c[col].high;
    uint32_t nb = (uint32_t)__builtin_popcount(mb & SEGBAL_BSEG_MASK);
    // The GPIOC digit is 7 segments in .low plus its decimal point in .high (cSegDP). The REST of
    // .high is that bank's one-cold column select + enables (set once in SysInit) — addressing,
    // not LEDs — and must survive in every mirror slot, exactly like GPIOB's bCat bits.
    uint32_t nc = (uint32_t)__builtin_popcount(ml) + ((mh >> 4) & 1u);
    // duty on the 0..16 scale, rescaled to this depth (D=16 is exact; lit digits keep >= 1 cycle)
    uint32_t sb = (segbal_duty(nb, eff) * D + 8u) / 16u;  if (nb && !sb) sb = 1u;
    uint32_t sc = (segbal_duty(nc, eff) * D + 8u) / 16u;  if (nc && !sc) sc = 1u;
    uint32_t sdp = sc;                                     // the DP rides its digit's cycles...
    if (fading) {
      if (col >= 1 && col <= 3) {                          // ds/cs/ms: scale by live significance
        uint32_t f = digit_bright[col - 1];
        sc = (sc * f + 8u) / 16u;
        if (f && ml && !sc) sc = 1u;                       // mid-fade digits never fully dark...
        if (!f) sc = 0u;                                   // ...but zero significance is zero
        sdp = sc;
      } else if (col == 0) {                               // ...except the seconds column's DP,
        uint32_t f = digit_bright[3];                      // which fades with the 0.1 s digit
        sdp = (sc * f + 8u) / 16u;
        if (f && !sdp) sdp = 1u;
        if (!f) sdp = 0u;
      }
    }
    uint16_t cat = mb & (uint16_t)~SEGBAL_BSEG_MASK;
    uint8_t  csel = mh & (uint8_t)~cSegDP;                 // GPIOC column select + enables
    uint32_t ring = D - 1u;                                // mirror ranks 1..D-1 rotate mod D-1
    uint32_t pb = SEGBAL_PH_B[col] % ring;
    uint32_t pc = SEGBAL_PH_C[col] % ring;
    uint16_t bm_b = 1u, bm_c = 1u, bm_dp = 1u;             // cycle 0 = the master, always lit
    for (uint32_t k = 1; k < D; k++) {
      uint32_t i = col + 5u*k;
      uint32_t r = (uint32_t)(SEGBAL_REV16[k] >> revsh);   // 1..D-1 for k >= 1
      uint32_t rb = r - 1u + pb; if (rb >= ring) rb -= ring;
      uint32_t rc = r - 1u + pc; if (rc >= ring) rc -= ring;
      uint32_t lb = (rb + 1u < sb), lc = (rc + 1u < sc), ldp = (rc + 1u < sdp);
      bm_b |= (uint16_t)(lb << k); bm_c |= (uint16_t)(lc << k); bm_dp |= (uint16_t)(ldp << k);
      buffer_b[i]      = lb ? mb : cat;                    // column select stays in every slot
      buffer_c[i].low  = lc ? ml : 0;
      buffer_c[i].high = csel | (ldp ? (mh & cSegDP) : 0);
    }
    segbal_lit_b[col] = bm_b; segbal_lit_c[col] = bm_c; segbal_lit_dp[col] = bm_dp;
  }
  if (display_scan_len != want_len) setDisplayPWM(want_len);   // extend to the D-cycle scan
  segbal_mirror_live = 1;                                  // bitmaps valid — arm the ISR refresh
}

// SysTick-side mirror refresh: re-copy the live master values through the exported bitmaps every
// millisecond, so the mirrors can never go staler than the masters (see the bitmap block above).
// Cost with D=16: 5 x 15 slot writes, ~10 us at 80 MHz — 1% of one SysTick period, only while
// the D-cycle scan is up. Bitmaps and masters are each written whole (halfword stores), so the
// worst race with the main-loop refill is one slot showing one frame of the other's value.
void segbal_isr_refresh(void){
  if (!segbal_mirror_live || display_scan_len <= 5) return;
  uint32_t D = (uint32_t)display_scan_len / 5u;
  for (uint32_t col = 0; col < 5; col++) {
    uint16_t mb = buffer_b[col];
    uint8_t  ml = buffer_c[col].low, mh = buffer_c[col].high;
    uint16_t cat  = mb & (uint16_t)~SEGBAL_BSEG_MASK;
    uint8_t  csel = mh & (uint8_t)~cSegDP;
    uint16_t lb = segbal_lit_b[col], lc = segbal_lit_c[col], ldp = segbal_lit_dp[col];
    for (uint32_t k = 1; k < D; k++) {
      uint32_t i = col + 5u*k;
      buffer_b[i]      = ((lb >> k) & 1u) ? mb : cat;
      buffer_c[i].low  = ((lc >> k) & 1u) ? ml : 0;
      buffer_c[i].high = csel | (((ldp >> k) & 1u) ? (mh & cSegDP) : 0);
    }
  }
}

// ---- $PMLOOP: main-loop latency diagnostic (`loop_diag = on` over serial or config) -----------
// Once per second, emit the WORST gap (ms of uwTick) between consecutive profiler marks and the
// tag of the section that produced it: $PMLOOP,<worst_ms>,<tag>. Marks bracket the main loop's
// sections; PendSV stamps tag 15 when it preempts, so the once-per-second display prep shows up
// under its own name. Emission is lossy on a busy USB endpoint by design (1 Hz diagnostic).
// Tags: 1 menu . 2 balance/colon . 3 tz-lookup . 4 delayed-housekeeping . 5 vbus/temp .
//       6 pps-emit/tempcomp/dumps . 7 vbat/astro . 8 mode-pages/star . 9 alt/loop-tail . 15 PendSV
volatile uint8_t loop_diag = 0;
volatile uint8_t pmloop_lasttag = 0;
static uint32_t  pmloop_last = 0, pmloop_max = 0, pmloop_win = 0;
static uint8_t   pmloop_maxtag = 0;
#define LP_MARK(n) do { uint32_t t_ = uwTick, g_ = t_ - pmloop_last; \
    if (g_ > pmloop_max) { pmloop_max = g_; pmloop_maxtag = pmloop_lasttag; } \
    pmloop_last = t_; pmloop_lasttag = (n); } while (0)

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

// Colon brightness tracking. The colons are TIM2 PWM (buffer_colons_L/R), NOT part of the segment
// scan and NOT coupled to dac_target — so out of the box they hold their animation brightness while
// the digits dim around them, blazing at low rail. colon_balance ties the colon PWM to the rail:
//   0/off (default) = stock (colons at full animation brightness)
//   1/on            = AUTO: scale the colon PWM by the calibrated rail curve (see colon_scale_for)
//   2..256          = fixed manual scale (of 256) for eyeball calibration across the rail
// The scale is folded into the animation buffer by loadColonAnimation and re-applied by
// colon_balance_poll() from the main loop when the rail moves — never from the DMA/scan ISR.
volatile uint16_t colon_balance = 0;
static   uint16_t colonScale    = 256;   // applied fixed-point scale, 256 = full animation brightness
static volatile uint8_t colonForce = 0;  // config set colon_balance — apply once even if within the AUTO hysteresis

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

  // Track the rail: fold the current colon brightness into the freshly-loaded animation (256 = full,
  // so colon_balance = off is exact identity). The DMA reads this buffer live — refilling it in place
  // re-brightens the colons without a restart or a phase jump.
  for (int k = 0; k < 200; k++) {
    buffer_colons_L[k] = (uint16_t)(((uint32_t)buffer_colons_L[k] * colonScale) >> 8);
    buffer_colons_R[k] = (uint16_t)(((uint32_t)buffer_colons_R[k] * colonScale) >> 8);
  }
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

// AUTO colon scale vs rail (dac_target 0 = brightest .. 4095 = dimmest). A full eyeballed sweep on a
// production Mk IV (2026-07-11) landed a clean straight line: the colons want full animation scale at
// the bright half of the rail and taper linearly to a dim floor at the darkest, so the separators stay
// present but recessed while the digits blaze at low rail. Two anchors define it (both baked here and
// overridable per-hardware from config.txt, exactly like the BS brightness curve):
//   colon_full_at — the dac at/below which colons run at full scale (256). Default 2048 (mid rail).
//   colon_floor   — the minimum scale, reached at the dimmest rail (dac 4095). Default 20.
// The line runs from (colon_full_at, 256) toward (4095, 0), clamped at the top to 256 and the bottom to
// colon_floor: scale = clamp( 256*(4095-dac) / (4095-colon_full_at), colon_floor, 256 ). With the
// defaults this is ~ (4095-dac)/8 and passes through the measured points (dac 3276->102, 2867->153).
volatile int32_t colon_full_at = 2048;   // baked default; config "colon_full_at = N" overrides
volatile int32_t colon_floor   = 20;     // baked default; config "colon_floor = N" overrides
static uint16_t colon_scale_for(int32_t d){
  int32_t span = 4095 - colon_full_at; if (span < 1) span = 1;   // guard div-by-zero / inverted anchor
  int32_t s = 256 * (4095 - d) / span;
  if (s > 256) s = 256;
  if (s < colon_floor) s = colon_floor;
  if (s < 0) s = 0;
  return (uint16_t)s;
}
// Re-mirror the colon scale into the animation buffer when the live rail (or the config) moves it.
// Main-loop only (loadColonAnimation touches 400 samples); throttled so it reloads on real change.
void colon_balance_poll(void){
  if (displayMode == MODE_STANDBY) return;   // colons off (displayOff stops TIM2) — match segbal_poll's guard
  uint16_t want = (colon_balance == 0) ? 256
                : (colon_balance == 1) ? colon_scale_for((int32_t)dac_target)
                : (colon_balance > 256) ? 256 : colon_balance;
  int diff = (int)want - (int)colonScale; if (diff < 0) diff = -diff;
  if (diff >= 4 || colonForce) {             // hysteresis throttles AUTO drift; colonForce lands an explicit set
    colonForce = 0;
    colonScale = want;
    // loadColonAnimation() also runs in the USART2 ISR (button -> nextMode -> applyColonForMode); mask
    // JUST that IRQ so a button press can't re-enter mid-refill and tear buffer_colons. A few us, only
    // on a real brightness step, and PPS (EXTI9_5) + GPS (USART1) are untouched.
    NVIC_DisableIRQ(USART2_IRQn);
    loadColonAnimation();          // refills buffer_colons at the new scale; DMA picks it up, no restart
    NVIC_EnableIRQ(USART2_IRQn);
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

// Accept a float between 0.0 and 1.0, or an int from 0 to 4095
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

void parseConfigString(char *key, char *value, _Bool from_serial) {

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

  } else if (strcasecmp(key, "MODE_TEMPCOMP") == 0) {
    set_mode_enabled(MODE_TEMPCOMP, value);
  } else if (strcasecmp(key, "MODE_LST") == 0) {
    set_mode_enabled(MODE_LST, value);
  } else if (strcasecmp(key, "MODE_SOLAR") == 0) {
    set_mode_enabled(MODE_SOLAR, value);
  } else if (strcasecmp(key, "MODE_ADEV") == 0) {
    set_mode_enabled(MODE_ADEV, value);
  } else if (strcasecmp(key, "tc_learn") == 0) {
    tc_learn = truthy(value);         // accumulate (die temp, ppm) samples while GPS-locked
  } else if (strcasecmp(key, "tc_apply") == 0) {
    tc_apply = truthy(value);         // steer the SysTick timebase during GPS-loss holdover
  } else if (strcasecmp(key, "significance_fade") == 0) {
    significance_fade = truthy(value);      // fade sub-second digits by significance in holdover, not dash
  } else if (strcasecmp(key, "seg_balance") == 0) {
    // Equalise per-segment brightness by duty (see segbal_poll). "on" (or 1) = AUTO, the
    // calibrated strength-vs-rail curve — the intended setting. A numeric 2..300 applies a
    // fixed manual strength (<=100 linear blend, >100 power-law) for tuning experiments.
    int v = truthy(value) ? 1 : atoi(value);
    seg_balance = (uint16_t)(v < 0 ? 0 : (v > 300 ? 300 : v));
  } else if (strcasecmp(key, "colon_balance") == 0) {
    // Dim the colons with the rail (see colon_balance_poll). "on"/1 = AUTO curve; a numeric 2..256
    // pins a fixed scale (of 256) for eyeball calibration. The main-loop poll re-applies it.
    int v = truthy(value) ? 1 : (falsey(value) ? 0 : atoi(value));
    colon_balance = (uint16_t)(v < 0 ? 0 : (v > 256 ? 256 : v));
    colonForce = 1;                   // land an explicit sweep step even if within the hysteresis band
  } else if (strcasecmp(key, "colon_full_at") == 0) {
    // Per-hardware anchor: the dac (0..4095) at/below which AUTO colons run at full scale. Overrides
    // the baked default. Kept below full-scale so 4095-colon_full_at stays a positive span.
    int v = atoi(value); colon_full_at = v < 0 ? 0 : (v > 4000 ? 4000 : v);
    colonForce = 1;
  } else if (strcasecmp(key, "colon_floor") == 0) {
    // Per-hardware anchor: the AUTO colon scale (of 256) at the dimmest rail. Overrides the baked default.
    int v = atoi(value); colon_floor = v < 0 ? 0 : (v > 256 ? 256 : v);
    colonForce = 1;
  } else if (strcasecmp(key, "tc_rtc") == 0) {
    tc_rtc = truthy(value);           // additionally trim RTC->CALR while GPS is absent
  } else if (strcasecmp(key, "tc_t0") == 0) {
    int v = atoi(value); tc_t0 = v < -30 ? -30 : (v > 80 ? 80 : v);
  } else if (strcasecmp(key, "tc_engage_s") == 0) {
    // Floor of 2: currentTime pre-increments at the modelled .900 mark, so "fresh" reads 1
    // for the last 100 ms of every LOCKED second — a floor of 1 would engage during lock.
    int v = atoi(value); tc_engage_s = v < 2 ? 2 : (v > 3600 ? 3600 : v);
  } else if (strcasecmp(key, "tc_max_ppm") == 0) {
    int v = atoi(value); tc_max_ppm = v < 1 ? 1 : (v > 200 ? 200 : v);
  } else if (strcasecmp(key, "tc_hse_b") == 0) { tc_parse_coeff(value, &tc_cfg_hse[1]);
  } else if (strcasecmp(key, "tc_hse_c") == 0) { tc_parse_coeff(value, &tc_cfg_hse[2]);
  } else if (strcasecmp(key, "tc_lse_a") == 0) { tc_parse_coeff(value, &tc_cfg_lse[0]);
  } else if (strcasecmp(key, "tc_lse_b") == 0) { tc_parse_coeff(value, &tc_cfg_lse[1]);
  } else if (strcasecmp(key, "tc_lse_c") == 0) { tc_parse_coeff(value, &tc_cfg_lse[2]);
  } else if (strcasecmp(key, "tc_seed") == 0) {
    tc_seed = truthy(value);          // load the coefficients above as an EVOLVING prior, not a freeze
    // Over serial, the "tc_seed = on" line is the TRIGGER: it arms a main-loop apply, so a paste
    // seeds exactly once, after all its coefficient lines have parsed (send it last — the order
    // tc_dump prints). ISR-safe by design: the apply itself always runs from tc_housekeeping.
    if (from_serial && tc_seed) tc_seed_pending = 1;
  } else if (strcasecmp(key, "tc_seed_lo") == 0) {
    tc_seed_lo = (int16_t)atoi(value);   // seed coverage low edge (die °C) — the prior is not extrapolated
  } else if (strcasecmp(key, "tc_seed_hi") == 0) {
    tc_seed_hi = (int16_t)atoi(value);
  } else if (strcasecmp(key, "tc_dump") == 0) {
    // Serial-only trigger: print the learned model as paste-ready config lines. A stray
    // tc_dump left in config.txt must not fire on every (re)load, hence the origin guard.
    if (from_serial && truthy(value)) tc_dump_pending = 1;
  } else if (strcasecmp(key, "adev_dump") == 0) {
    if (from_serial && truthy(value)) adev_dump_pending = 1;  // serial-only: emit one $PMADEV sentence
  } else if (strcasecmp(key, "hdev_dump") == 0) {
    if (from_serial && truthy(value)) hdev_dump_pending = 1;  // serial-only: emit one $PMHDEV sentence (drift-immune Hadamard)
  } else if (strcasecmp(key, "loop_diag") == 0) {
    loop_diag = truthy(value) ? 1 : 0;   // 1 Hz $PMLOOP main-loop latency diagnostic
  } else if (strcasecmp(key, "tc_reset") == 0) {
    if (from_serial && truthy(value)) tc_reset_pending = 1;   // serial-only, same guard

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
      parseConfigString(key, value, 1);   // serial origin: tc_dump/tc_reset may fire
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

       parseConfigString(key, value, 0);   // file origin: serial-only triggers inert

     }
   }

   // if enabled, always boot into ttff
   if (config.modes_enabled[MODE_TTFF]) requestMode=MODE_TTFF;
   else requestMode=255;

   postConfigCleanup();
   tc_seed_apply();   // warm-start the tempco model from a persisted seed (tc_seed = on), else no-op
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

// ---- Live Allan deviation of the FREE-RUNNING HSE (MODE_ADEV) ----------------------------------
// The honest oscillator-stability signal is the free-running DWT phase, NOT the SysTick residual:
// capturePPS() re-pins SysTick every second (an infinite-gain phase reset), so the residual's ADEV
// rolls off artificially past tau=1 s and measures the discipline loop, not the crystal. DWT->CYCCNT
// is never re-pinned; its per-second delta minus the 80 MHz core count is the bare fractional-
// frequency error. We integrate that to cumulative phase (int32 ticks, 12.5 ns) in a RAM2 ring and
// compute the overlapping Allan deviation (IEEE-1139) on demand. A host can reconstruct the
// disciplined-output curve from $PMTXTS. Verified vs a double-precision reference in the emulator.
#define ADEV_N     4096u          // 1 Hz phase samples (16 KB of RAM2); tau to 1024 s @ N-2m=2048 overlaps
#define ADEV_OCT   11u            // octave taus 1,2,4,...,1024 s
#define ADEV_FCPU  80000000       // core ticks per true GPS second
#ifdef __EMSCRIPTEN__
static int32_t adev_x[ADEV_N];                                     // emu: plain static (no RAM2 section)
#else
__attribute__((section(".ram2"))) static int32_t adev_x[ADEV_N];   // RAM2 @ 0x10000000, off the CRC path
#endif
static uint32_t adev_prev_dwt, adev_prev_epoch;
// Cumulative phase, ticks. UNSIGNED on purpose: with a static ppm-scale offset the phase ramps
// monotonically and an int32 accumulator hits signed-overflow UB after ~311 days of continuous lock
// at 1 ppm. Unsigned wrap is defined; the ring stores the (int32_t) view and the second difference
// is computed back in unsigned, so everything stays exact modulo 2^32 (true |d| << 2^31 always).
static uint32_t adev_phase_u;
static uint16_t adev_widx, adev_valid;
static uint8_t  adev_have_prev;
static float    adev_sigma_cache[ADEV_OCT];
static volatile uint8_t  adev_noct;
static volatile uint32_t adev_last_ms;   // uwTick of the last accepted sample: the display's staleness tell

static void adev_reset(void){
  memset((void*)adev_x, 0, sizeof adev_x);   // RAM2 is NOT zeroed at boot — clear explicitly
  adev_phase_u=0; adev_widx=0; adev_valid=0; adev_have_prev=0; adev_noct=0;
  for (uint32_t i=0;i<ADEV_OCT;i++) adev_sigma_cache[i]=0.0f;
}
// Append a phase sample directly (also the emu/test path — bypasses the DWT delta).
static void adev_push_x(int32_t x){
  adev_x[adev_widx]=x;
  adev_widx=(uint16_t)((adev_widx+1u)%ADEV_N);
  if (adev_valid<ADEV_N) adev_valid++;
  adev_last_ms=uwTick;
}
// One phase sample per locked second from the PPS edge (ISR context). ONE missed second (a receiver
// hiccup / USB stall) is tolerated with a linear midpoint so a 68-minute record isn't discarded over
// a single dropout; anything longer is a real gap — restart the chain (overlapping ADEV needs
// contiguous samples) AND zero adev_noct immediately so the display can't keep painting the stale
// pre-gap curve until the next page flip.
static void adev_push_dwt(uint32_t dwt, uint32_t epoch){
  uint32_t de = adev_have_prev ? (uint32_t)(epoch-adev_prev_epoch) : 0u;
  if (de==1u){
    adev_phase_u += (dwt-adev_prev_dwt) - (uint32_t)ADEV_FCPU;  // exact across one 53.7 s DWT wrap
    adev_push_x((int32_t)adev_phase_u);
  } else if (de==2u){                                           // single missed PPS: interpolate the midpoint
    int32_t inc = (int32_t)((dwt-adev_prev_dwt) - 2u*(uint32_t)ADEV_FCPU);
    adev_phase_u += (uint32_t)(inc/2);        adev_push_x((int32_t)adev_phase_u);
    adev_phase_u += (uint32_t)(inc - inc/2);  adev_push_x((int32_t)adev_phase_u);
  } else {                                                      // first sample or a real gap -> restart at 0
    adev_phase_u=0; adev_x[0]=0; adev_widx=1; adev_valid=1;
    adev_noct=0;                                                // honest NOW, not at the next page flip
  }
  adev_prev_dwt=dwt; adev_prev_epoch=epoch; adev_have_prev=1;
}
// Overlapping Allan deviation for averaging factor m (tau = m s), read time-ordered across the ring.
static float adev_sigma_for_m(uint32_t m){
  uint32_t N=adev_valid;
  if (N < 2u*m+1u) return 0.0f;
  uint16_t base=(adev_valid<ADEV_N)?0u:adev_widx;            // oldest sample
  uint32_t cnt=N-2u*m;
  int64_t S=0;
  for (uint32_t i=0;i<cnt;i++){
    uint32_t a=(uint32_t)adev_x[(uint16_t)((base+i)%ADEV_N)];
    uint32_t b=(uint32_t)adev_x[(uint16_t)((base+i+m)%ADEV_N)];
    uint32_t c=(uint32_t)adev_x[(uint16_t)((base+i+2u*m)%ADEV_N)];
    int32_t d=(int32_t)(c-2u*b+a);                            // second difference, ticks (mod-2^32 exact: the accumulator wraps unsigned)
    S += (int64_t)d*d;
  }
  double var=(double)S/(2.0*(double)cnt);                    // int64 kernel; one double sqrt/octave
  return (float)(sqrt(var)/((double)ADEV_FCPU*(double)m));   // -> fractional frequency (dimensionless)
}
// Recompute the octave cache (thread context). Publish adev_noct under IRQ mask (torn-read guard).
// PUBLICATION gate: valid >= 4m (>= 2m overlapping triplets), stricter than the mathematical minimum
// 2m+1 — at exactly 2m+1 an octave is a SINGLE second-difference shown at full display authority
// (~100% error bars). adev_sigma_for_m itself keeps the mathematical gate for the test/oracle path.
static void adev_reduce(void){
  uint8_t noct=0;
  for (uint8_t k=0;k<ADEV_OCT;k++){
    uint32_t m=1u<<k;
    if (adev_valid < 4u*m) break;
    adev_sigma_cache[k]=adev_sigma_for_m(m);
    noct=(uint8_t)(k+1);
  }
  __disable_irq(); adev_noct=noct; __enable_irq();
}
// Display-path refresh: the page flip needs ONE octave, not all eleven — a full reduce over a filled
// ring is ~41k MAC iterations (~4-6 ms) recomputed per flip. Compute noct arithmetically and only
// the sigma the current page shows (~0.2-0.4 ms worst case). The $PMADEV dump still full-reduces.
static void adev_display_update(void){
  uint8_t noct=0;
  for (uint8_t k=0;k<ADEV_OCT;k++){ if (adev_valid < 4u*(1u<<k)) break; noct=(uint8_t)(k+1); }
  if (noct){
    uint32_t k=(uwTick/page_ms())%noct;
    adev_sigma_cache[k]=adev_sigma_for_m(1u<<(uint8_t)k);
  }
  __disable_irq(); adev_noct=noct; __enable_irq();
}
// Overlapping HADAMARD deviation for averaging factor m — the third-difference kernel cancels
// LINEAR FREQUENCY DRIFT (temperature ramp / aging) that plain ADEV retains as a tau^+1 slope, so
// the long-tau octaves report the oscillator, not the ramp. Serial-only ($PMHDEV): reuses the same
// phase ring, no extra RAM, computed on demand. Same unsigned modulo arithmetic as the ADEV kernel.
static float hdev_sigma_for_m(uint32_t m){
  uint32_t N=adev_valid;
  if (N < 3u*m+1u) return 0.0f;
  uint16_t base=(adev_valid<ADEV_N)?0u:adev_widx;
  uint32_t cnt=N-3u*m;
  int64_t S=0;
  for (uint32_t i=0;i<cnt;i++){
    uint32_t a=(uint32_t)adev_x[(uint16_t)((base+i)%ADEV_N)];
    uint32_t b=(uint32_t)adev_x[(uint16_t)((base+i+m)%ADEV_N)];
    uint32_t c=(uint32_t)adev_x[(uint16_t)((base+i+2u*m)%ADEV_N)];
    uint32_t d4=(uint32_t)adev_x[(uint16_t)((base+i+3u*m)%ADEV_N)];
    int32_t d=(int32_t)(d4 - 3u*c + 3u*b - a);              // third difference, ticks
    S += (int64_t)d*d;
  }
  double var=(double)S/(6.0*(double)cnt);
  return (float)(sqrt(var)/((double)ADEV_FCPU*(double)m));
}
static uint8_t hdev_noct(void){                             // same 4m maturity gate as the ADEV publish path
  uint8_t n=0;
  for (uint8_t k=0;k<ADEV_OCT;k++){ if (adev_valid < 4u*(1u<<k)) break; n=(uint8_t)(k+1); }
  return n;
}
// Display accessors — sendDate() is defined above the engine, so it reads through these.
static uint8_t adev_disp_noct(void){ return adev_noct; }
static float   adev_disp_sigma(uint8_t k){ return (k < ADEV_OCT) ? adev_sigma_cache[k] : 0.0f; }

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
    adev_push_dwt(pps_cap.dwt_pps, pps_cap.epoch); /* free-running Allan-deviation sample */ \
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

// ==================== Temperature compensation (opt-in; state near pps_cap) ====================
// Everything below runs in the MAIN LOOP only (float allowed, calibrateRTC precedent). The tick
// ISRs see just three precomputed int32s via the tc_steer_* handoff in the timetick() hook.

static uint32_t tc_nom_load = 0;    // SysTick->LOAD captured before any steering (80 MHz: 79999)

// Ticks per ppm, derived from the captured nominal period so no core-clock assumption is baked
// in: one second is (LOAD+1)*1000 ticks, so 1 ppm = (LOAD+1)/1000 ticks (80 at 80 MHz).
// Verified against the live unit: $PMTXTS reports load=79999 (10 MHz TCXO -> PLL -> 80 MHz).
static int32_t tc_tpp(void){ return (int32_t)((tc_nom_load + 1) / 1000); }

static int tc_bin_i(int t){ int i = (t + 8) / 2; return i < 0 ? 0 : (i > 39 ? 39 : i); }

// One HSE sample per GPS-locked second. The PPS ISRs re-zero the ms cascade at every edge
// (SysTick->VAL reload + counter reset), so each capture is already a SELF-CONTAINED one-second
// accumulation: pos = const + tpp·ppm(T), where const is a fixed capture/reload offset and
// tpp = ticks per ppm. Measured on the live unit: pos = 79925.8 ± 0.67 ticks (~8 ns RMS) — the
// constant dominates and is unknowable from lock data alone, so the model is learned in ticks
// with an ARBITRARY ORIGIN, rebased to the first accepted sample to keep bin sums small. Its
// differences over temperature are exact, and holdover steering only ever applies
// model(T_now) − model(T_at_loss), from which the origin cancels. (An earlier draft differenced
// consecutive captures — but the per-edge cascade reset makes that identically ~0; verified on
// hardware: dpos = 0.05 ± 1.1 ticks.)
static int32_t tc_e0 = 0;                     // origin rebase: first accepted sample
static _Bool   tc_e0_set = 0;
static int32_t tc_ema = 0;                    // slow tracker for the glitch gate
static void tc_hse_learn(void){
  static uint32_t last_seq = 0;
  static uint8_t  warm = 0;

  __disable_irq();                            // tear-free copy (emitPPSTimestamp pattern)
  uint32_t seq   = pps_cap.seq;
  uint16_t subms = pps_cap.subms;
  uint32_t st    = pps_cap.systick;
  int16_t  temp  = pps_cap.temp;
  uint8_t  flags = pps_cap.flags;
  __enable_irq();

  if (seq == last_seq) return;                // no new edge since last pass
  _Bool contiguous = (seq == last_seq + 1);
  last_seq = seq;

  if ((flags & 0x3) != 0x3 || subms > 999) { warm = 0; return; }
  if (!contiguous) { warm = 0; return; }      // edges were missed: settle again
  if (warm < 10) { warm++; return; }          // settle after (re)acquisition

  int32_t half = (int32_t)(tc_nom_load + 1) * 500;   // half a second in ticks
  int32_t e = (int32_t)subms * (int32_t)(tc_nom_load + 1)
            + (int32_t)tc_nom_load - (int32_t)st;    // this second's accumulation (+ const)
  if (e >  half) e -= 2 * half;               // fold the origin into ±half a second
  if (e < -half) e += 2 * half;

  int32_t tpp = tc_tpp();                     // ticks per ppm (80 at 80 MHz)
  if (!tc_e0_set) { tc_e0 = e; tc_ema = 0; tc_e0_set = 1; }
  e -= tc_e0;                                 // arbitrary-origin rebase (keeps sums int32-safe)
  if (e - tc_ema > 100 * tpp || e - tc_ema < -100 * tpp) return; // >100 ppm step: glitch
  if (e > 30000 || e < -30000) return;        // hard cap so 32768·|e| can never overflow int32
  tc_ema += (e - tc_ema) / 16;

  struct tc_bin *b = &tc_bins[tc_bin_i(temp)];
  if (b->hse_n >= 8) {                        // per-bin outlier gate: 10 ppm off the mean
    int32_t d = e - b->hse_sum / (int32_t)b->hse_n;
    if (d > 10 * tpp || d < -10 * tpp) return;
  }
  if (b->hse_n >= 32768) { b->hse_sum /= 2; b->hse_n /= 2; }   // overflow-proof aging
  b->hse_sum += e; b->hse_n++;
  tc_n_hse++;
}

// One LSE sample per successful RTC calibration: calibrateRTC only advances the BKP31R stamp
// on an in-range 63 s measurement, so watching the stamp inherits its validity gate for free.
static void tc_lse_learn(void){
  static uint32_t seen = 0;
  uint32_t cal = rtc_last_calibration;
  if (cal == seen) return;
  _Bool first = (seen == 0);
  seen = cal;
  if (first) return;                          // boot-time stamp, not a fresh measurement
  int32_t v = debug_rtc_val;                  // raw LSE cycle error over CAL_PERIOD (63 s)
  if (v > 1000 || v < -1000) return;
  struct tc_bin *b = &tc_bins[tc_bin_i(die_temp_c)];
  if (b->lse_n >= 32768) { b->lse_sum /= 2; b->lse_n /= 2; }
  b->lse_sum += v; b->lse_n++;
  tc_n_lse++;
}

// Solve A·x = y for a 3x3 symmetric system by Gaussian elimination with partial pivoting.
static _Bool tc_gauss3(float A[3][3], float y[3], float x[3]){
  int p[3] = {0, 1, 2};
  for (int c = 0; c < 3; c++){
    int best = c;
    for (int r = c + 1; r < 3; r++)
      if (fabsf(A[p[r]][c]) > fabsf(A[p[best]][c])) best = r;
    int t = p[c]; p[c] = p[best]; p[best] = t;
    if (fabsf(A[p[c]][c]) < 1e-9f) return 0;
    for (int r = c + 1; r < 3; r++){
      float f = A[p[r]][c] / A[p[c]][c];
      for (int k = c; k < 3; k++) A[p[r]][k] -= f * A[p[c]][k];
      y[p[r]] -= f * y[p[c]];
    }
  }
  for (int c = 2; c >= 0; c--){
    float s = y[p[c]];
    for (int k = c + 1; k < 3; k++) s -= A[p[c]][k] * x[k];
    x[c] = s / A[p[c]][c];
  }
  return 1;
}

// Weighted least-squares fit of y(T) = a + b·x + c·x², x = T - tc_t0, over bin means.
// Falls back quadratic -> linear -> constant as temperature coverage thins. `scale` converts
// bin units (HSE: 1.0 — model stays in ticks, arbitrary origin; LSE: raw 63 s cal cycles → ppm).
// A fit is only accepted if every coefficient is finite: a near-singular system can pass the
// pivot threshold yet overflow to Inf/NaN, and NaN must never reach the steering or display.
static _Bool tc_fin3(const float m[3]){ return isfinite(m[0]) && isfinite(m[1]) && isfinite(m[2]); }

// Returns the ACHIEVED model order: 3 quadratic, 2 linear, 1 constant, 0 no fit. (The order lets the
// warm-start prior be preserved until real data supports a fit at least as rich — see tc_fit.)
static int tc_fit_one(_Bool lse, float scale, uint16_t n_quad, float m[3],
                        int16_t *tmin_out, int16_t *tmax_out){
  float S[5] = {0,0,0,0,0}, T[3] = {0,0,0};
  float S0a = 0, T0a = 0;                     // all-samples weighted mean (constant fallback)
  int nb = 0, tmin = 127, tmax = -128;
  int tmin_a = 127, tmax_a = -128;

  for (int i = 0; i < 40; i++){
    uint16_t n  = lse ? tc_bins[i].lse_n   : tc_bins[i].hse_n;
    if (!n) continue;
    int32_t sum = lse ? tc_bins[i].lse_sum : tc_bins[i].hse_sum;
    int   t = i * 2 - 8;                      // bin low edge; bin holds {t, t+1}
    float y = ((float)sum / (float)n) * scale;
    float w = (float)n;
    S0a += w; T0a += w * y;
    if (t < tmin_a) tmin_a = t;
    if (t > tmax_a) tmax_a = t;
    if (n < n_quad) continue;                 // curve terms only from well-filled bins
    float x = ((float)t + 0.5f) - (float)tc_t0;   // true bin centre: t + 0.5
    nb++;
    if (t < tmin) tmin = t;
    if (t > tmax) tmax = t;
    S[0] += w;         S[1] += w*x;       S[2] += w*x*x;
    S[3] += w*x*x*x;   S[4] += w*x*x*x*x;
    T[0] += w*y;       T[1] += w*x*y;     T[2] += w*x*x*y;
  }

  if (nb >= 3 && (tmax - tmin) >= 6) {        // quadratic
    float A[3][3] = {{S[0],S[1],S[2]},{S[1],S[2],S[3]},{S[2],S[3],S[4]}};
    float yv[3]   = {T[0],T[1],T[2]};
    if (tc_gauss3(A, yv, m) && tc_fin3(m)) { *tmin_out = tmin; *tmax_out = tmax; return 3; }
  }
  if (nb >= 2 && (tmax - tmin) >= 4) {        // linear
    float det = S[0]*S[2] - S[1]*S[1];
    if (fabsf(det) > 1e-9f){
      m[0] = (T[0]*S[2] - T[1]*S[1]) / det;
      m[1] = (S[0]*T[1] - S[1]*T[0]) / det;
      m[2] = 0;
      if (tc_fin3(m)) { *tmin_out = tmin; *tmax_out = tmax; return 2; }
    }
  }
  if (S0a >= (lse ? 8.0f : 60.0f)) {          // constant: the dominant fixed offset
    m[0] = T0a / S0a; m[1] = 0; m[2] = 0;
    if (tc_fin3(m)) { *tmin_out = tmin_a; *tmax_out = tmax_a; return 1; }
  }
  return 0;
}

static float tc_poly(const float m[3], float x);   // fwd: the residual pass evaluates the just-fit model

// Weighted-RMS residual of a fitted model over every populated bin (not only the curve-eligible
// ones): sqrt( Σ n·(bin_mean − model(T))² / Σ n ), in the fit's units. Runs as a second pass, so
// it never perturbs the fit itself; 40 bins, at most once per 5 min alongside tc_fit().
// `center` removes the weighted-mean of the residuals before the RMS — i.e. an ORIGIN-INVARIANT error.
// Needed for HSE: its model origin (m[0]) is arbitrary, so while a warm-start seed is held (m[0]=0) the
// real bins carry a different DC constant (the tc_e0 rebase); the raw offset would swamp the RMS and
// corrupt the holdover-fade σ_temp. Mean-centring measures only how well the SLOPE/CURVATURE match,
// which is all HSE cares about. LSE (absolute ppm, real m[0]) passes center=0 — its offset is real error.
static float tc_fit_resid(_Bool lse, float scale, const float m[3], _Bool center){
  float sw = 0, swr = 0, swr2 = 0;
  for (int i = 0; i < 40; i++){
    uint16_t n = lse ? tc_bins[i].lse_n : tc_bins[i].hse_n;
    if (!n) continue;
    int32_t sum = lse ? tc_bins[i].lse_sum : tc_bins[i].hse_sum;
    float y = ((float)sum / (float)n) * scale;
    float x = ((float)(i * 2 - 8) + 0.5f) - (float)tc_t0;   // bin centre − model origin
    float r = y - tc_poly(m, x);
    sw += (float)n; swr += (float)n * r; swr2 += (float)n * r * r;
  }
  if (!(sw > 0) || !isfinite(swr2)) return 0.0f;
  if (center) swr2 -= swr * swr / sw;                        // Σn(r−r̄)² = Σn·r² − (Σn·r)²/Σn
  return (swr2 > 0 && isfinite(swr2)) ? sqrtf(swr2 / sw) : 0.0f;
}

// Refit both models from the bins. A warm-start prior (tc_*_prior != 0) is PRESERVED until real data
// supports a fit at least as rich as the seed's order — then real data takes over (prior cleared).
// With no prior held (the normal cold-learn case) this is the original behaviour: fit -> adopt/invalidate.
// The residual is always re-measured against whatever model is held; while a seed is still held with no
// real samples yet, tc_fit_resid returns 0 (no data) and the seed's CARRIED residual is kept.
static void tc_fit(void){
  int16_t tmn, tmx; float m[3];

  int oh = tc_fit_one(0, 1.0f, 64, m, &tmn, &tmx);
  if (oh >= tc_hse_prior) {                    // real data at least as rich as the prior -> adopt it
    tc_hse_valid = (oh > 0);
    if (oh) { tc_hse_m[0]=m[0]; tc_hse_m[1]=m[1]; tc_hse_m[2]=m[2]; tc_hse_tmin=tmn; tc_hse_tmax=tmx; }
    tc_hse_prior = 0;
  }
  if (tc_hse_valid) { float r = tc_fit_resid(0, 1.0f, tc_hse_m, 1); if (r > 0 || !tc_hse_prior) tc_hse_resid = r; }
  else tc_hse_resid = 0;

  const float lse_scale = 1e6f/(32768.0f*63.0f);
  int ol = tc_fit_one(1, lse_scale, 4, m, &tmn, &tmx);
  if (ol >= tc_lse_prior) {
    tc_lse_valid = (ol > 0);
    if (ol) { tc_lse_m[0]=m[0]; tc_lse_m[1]=m[1]; tc_lse_m[2]=m[2]; tc_lse_tmin=tmn; tc_lse_tmax=tmx; }
    tc_lse_prior = 0;
  }
  if (tc_lse_valid) { float r = tc_fit_resid(1, lse_scale, tc_lse_m, 0); if (r > 0 || !tc_lse_prior) tc_lse_resid = r; }
  else tc_lse_resid = 0;
}

static float tc_poly(const float m[3], float x){ return m[0] + m[1]*x + m[2]*x*x; }

// Warm-start the tempco model from the seeded coefficients (tc_seed = on). Loads tc_hse_b/c and
// tc_lse_a/b/c — the last tc_dump, persisted in config.txt by the host — as the LIVE model so the
// clock is temperature-compensated from the first second, records the seed's order (kept by tc_fit
// until real data is at least as rich), and seeds a conservative residual so holdover-fade stays
// honest before real samples arrive. Clearing the frozen slots is what turns a freeze into an
// evolving prior. Runs once per boot; on a later live config reload it only re-clears the reparsed
// coefficients so the freeze path can't silently re-activate over the evolving model.
static void tc_seed_apply(void){
  if (!tc_seed) return;
  if (tc_seed_done) {                         // live reload: keep evolving — don't re-freeze or re-seed
    tc_cfg_hse[0]=tc_cfg_hse[1]=tc_cfg_hse[2]=NAN;
    tc_cfg_lse[0]=tc_cfg_lse[1]=tc_cfg_lse[2]=NAN;
    return;
  }
  _Bool have_hse = !isnan(tc_cfg_hse[1]) || !isnan(tc_cfg_hse[2]);
  _Bool have_lse = !isnan(tc_cfg_lse[0]) || !isnan(tc_cfg_lse[1]) || !isnan(tc_cfg_lse[2]);
  if (!have_hse && !have_lse) return;         // seed enabled but no coefficients yet — wait for a reload
  tc_seed_done = 1;

  int16_t lo = tc_seed_lo, hi = tc_seed_hi;
  if (hi - lo < 4) { lo = (int16_t)(tc_t0 - 6); hi = (int16_t)(tc_t0 + 6); }   // sane span if none given
  // readConfigFile (hence this seed) runs at boot BEFORE the first tc_housekeeping, so tc_nom_load is
  // still 0 and tc_tpp() would return 0 — which would silently zero the tick-domain HSE model. Capture
  // the nominal SysTick period here (SystemClock_Config set it before the config load) — the same
  // capture tc_housekeeping() also performs.
  if (!tc_nom_load) tc_nom_load = SysTick->LOAD;
  float tpp = (float)tc_tpp();

  if (have_hse) {
    tc_hse_m[0] = 0.0f;                        // arbitrary origin — steering uses temperature differences
    tc_hse_m[1] = isnan(tc_cfg_hse[1]) ? 0.0f : tc_cfg_hse[1] * tpp;
    tc_hse_m[2] = isnan(tc_cfg_hse[2]) ? 0.0f : tc_cfg_hse[2] * tpp;
    tc_hse_tmin = lo; tc_hse_tmax = hi; tc_hse_valid = 1;
    tc_hse_prior = (!isnan(tc_cfg_hse[2]) && tc_cfg_hse[2] != 0.0f) ? 3 : 2;
    tc_hse_resid = 2.0f * tpp;                 // conservative (~2 ppm) until real data measures it
    tc_cfg_hse[0] = tc_cfg_hse[1] = tc_cfg_hse[2] = NAN;   // seed replaces freeze -> free to evolve
  }
  if (have_lse) {
    tc_lse_m[0] = isnan(tc_cfg_lse[0]) ? 0.0f : tc_cfg_lse[0];
    tc_lse_m[1] = isnan(tc_cfg_lse[1]) ? 0.0f : tc_cfg_lse[1];
    tc_lse_m[2] = isnan(tc_cfg_lse[2]) ? 0.0f : tc_cfg_lse[2];
    tc_lse_tmin = lo; tc_lse_tmax = hi; tc_lse_valid = 1;
    tc_lse_prior = (!isnan(tc_cfg_lse[2]) && tc_cfg_lse[2] != 0.0f) ? 3
                 : (!isnan(tc_cfg_lse[1]) && tc_cfg_lse[1] != 0.0f) ? 2 : 1;
    tc_lse_resid = 2.0f;
    tc_cfg_lse[0] = tc_cfg_lse[1] = tc_cfg_lse[2] = NAN;
  }
}

// LSE model (absolute ppm): non-NAN config a freezes it (the user asserted the values);
// otherwise the learned fit, clamped to its observed temperature range (no extrapolation).
// Config values are USB-ISR-written; snapshot each element once (single-word reads are atomic).
static _Bool tc_model_lse(int t, float *ppm){
  float a = tc_cfg_lse[0], b = tc_cfg_lse[1], c = tc_cfg_lse[2];
  if (!isnan(a)) {
    if (isnan(b)) b = 0;
    if (isnan(c)) c = 0;
    float x = (float)t - (float)tc_t0;
    *ppm = a + b*x + c*x*x;
    return isfinite(*ppm);
  }
  if (!tc_lse_valid) return 0;
  if (t < tc_lse_tmin) t = tc_lse_tmin;
  if (t > tc_lse_tmax) t = tc_lse_tmax;
  *ppm = tc_poly(tc_lse_m, (float)t - (float)tc_t0);
  return 1;
}

// HSE steering delta in TICKS between two temperatures. The learned model's origin is
// arbitrary (see tc_hse_learn), so only differences are meaningful — which is exactly what
// holdover needs: at GPS loss the display is phase-true, and the error that then accrues is
// the temperature-driven CHANGE of the oscillator, model(T_now) − model(T_loss). Frozen config
// coefficients are in ppm; a cancels in the difference, so only tc_hse_b/c are required.
static _Bool tc_hse_delta(int t_now, int t_ref, int32_t *dticks){
  float b = tc_cfg_hse[1], c = tc_cfg_hse[2];
  if (!isnan(b)) {                            // frozen: b (and optionally c) from config
    if (isnan(c)) c = 0;
    float x1 = (float)t_now - (float)tc_t0, x0 = (float)t_ref - (float)tc_t0;
    float dppm = (b*x1 + c*x1*x1) - (b*x0 + c*x0*x0);
    if (!isfinite(dppm)) return 0;
    *dticks = (int32_t)lroundf(dppm * (float)tc_tpp());
    return 1;
  }
  if (!tc_hse_valid) return 0;
  if (t_now < tc_hse_tmin) t_now = tc_hse_tmin;   // no extrapolation past learned coverage
  if (t_now > tc_hse_tmax) t_now = tc_hse_tmax;
  if (t_ref < tc_hse_tmin) t_ref = tc_hse_tmin;
  if (t_ref > tc_hse_tmax) t_ref = tc_hse_tmax;
  float d = tc_poly(tc_hse_m, (float)t_now - (float)tc_t0)
          - tc_poly(tc_hse_m, (float)t_ref - (float)tc_t0);
  if (!isfinite(d)) return 0;
  *dticks = (int32_t)lroundf(d);              // learned model is already in ticks
  return 1;
}

// Once-per-second control: evaluate the models at the current die temperature, refresh the
// display cache, engage/disengage SysTick steering, and (optionally) trim RTC->CALR.
static void tc_governor(void){
  static uint32_t last_run = 0;
  static int32_t  applied_E = 0;              // ticks/second currently steered
  static _Bool    was_on = 0;
  static int16_t  t_loss = 0;                 // die temp captured when steering engaged
  static int32_t  last_steps = 0x7FFF;        // last CALR trim written (sentinel: none)
  static uint32_t last_calr = 0;

  // Snapshot the two ISR-written time variables together: currentTime increments at the
  // modelled .900 mark while last_pps_time updates at the edge, and reading them separately
  // can interleave with both ISRs and yield a wrapped-huge "fresh" that spuriously engages.
  __disable_irq();
  uint32_t now  = (uint32_t)currentTime;
  uint32_t lpps = last_pps_time;
  __enable_irq();
  if (now == last_run) return;
  last_run = now;

  uint32_t fresh = now - lpps;                // seconds since the last PPS edge
  if (fresh > 0x80000000u) fresh = 0;         // interleaved-read underflow: treat as fresh

  int t = die_temp_c;
  float lp = 0;
  _Bool have_l = tc_model_lse(t, &lp);
  int32_t tpp = tc_tpp();

  // Would-be steering delta at the current temperatures (also feeds the display)
  int32_t dt_now = 0;
  _Bool have_h = tc_hse_delta(t, was_on ? t_loss : t, &dt_now);

  // display cache (clamped so "HSE -99.99" never exceeds the 10-char row).
  // HSE page shows the ACTIVE steering correction in ppm (0.00 while locked — the PPS
  // discipline owns the phase then); LSE page shows the absolute model ppm.
  float dh = was_on ? (float)applied_E / (float)tpp : 0.0f;
  float dl = lp;
  if (dh >  99.99f) dh =  99.99f;
  if (dh < -99.99f) dh = -99.99f;
  if (dl >  99.99f) dl =  99.99f;
  if (dl < -99.99f) dl = -99.99f;
  tc_disp_hse = dh;  tc_disp_hse_ok = have_h;
  tc_disp_lse = dl;  tc_disp_lse_ok = have_l;
  tc_disp_state = tc_steer_on ? 'A'
                : (!isnan(tc_cfg_hse[1]) || !isnan(tc_cfg_lse[0])) ? 'F'
                : (tc_hse_prior || tc_lse_prior) ? 'S'          // running on the warm-start seed (evolving)
                : (tc_learn && fresh < 5) ? 'L' : '-';

  // --- HSE steering: engage only in holdover, after first-ever fix, with a usable model.
  // The correction is the temperature-driven CHANGE since GPS loss (origin cancels; see
  // tc_hse_delta). At the loss instant the delta is 0 by construction and grows only as the
  // die temperature moves, so engage is glitch-free and re-lock needs no unwinding beyond
  // the LOAD restore (the per-edge phase snap owns lock).
  if (tc_apply && had_pps && fresh >= tc_engage_s) {
    if (!was_on) t_loss = (int16_t)t;         // remember the temperature we lost GPS at
    int32_t target = 0;
    if (tc_hse_delta(t, t_loss, &target)) {
      int32_t lim = (int32_t)tc_max_ppm * tpp;
      if (target >  lim) target =  lim;
      if (target < -lim) target = -lim;
      if (!was_on) applied_E = target;        // 0 at engage by construction...
      else {                                  // ...then gentle slew (temp-quantisation steps)
        int32_t slew = tpp / 4;               // 0.25 ppm per second
        int32_t d = target - applied_E;
        if (d >  slew) d =  slew;
        if (d < -slew) d = -slew;
        applied_E += d;
      }
      int32_t base = applied_E >= 0 ? applied_E / 1000 : -((-applied_E + 999) / 1000);
      int32_t rem  = applied_E - base * 1000; // floor-division remainder, always [0,1000)
      __disable_irq();
      tc_load_base = (int32_t)tc_nom_load + base;
      tc_rem = rem;
      tc_steer_on = 1;
      __enable_irq();
      was_on = 1;
    } else if (was_on) {                      // model became unusable mid-holdover
      tc_steer_on = 0;
      SysTick->LOAD = tc_nom_load;
      tc_acc = 0;
      applied_E = 0;
      was_on = 0;
    }
  } else if (was_on) {
    tc_steer_on = 0;                          // flag first: the ISR stops writing LOAD...
    SysTick->LOAD = tc_nom_load;              // ...then restore the nominal period
    tc_acc = 0;
    applied_E = 0;
    was_on = 0;
  }

  // --- LSE -> RTC->CALR trim: power-loss insurance only (display time is HSE-driven) ---
  // calibrateRTC() owns CALR while locked (it runs from the PPS ISRs, which are silent now);
  // the first successful calibration after re-lock re-measures and overwrites this trim.
  // While PPS is fresh, forget our last write: calibrateRTC has since replaced CALR, so an
  // equal-valued model trim in the NEXT outage must not be skipped by the != guard.
  if (fresh <= 63) last_steps = 0x7FFF;
  if (tc_rtc && have_l && fresh > 63 && now - last_calr >= 60) {
    int32_t steps = (int32_t)lroundf(lp * (1048576.0f / 1000000.0f));  // ppm -> CALM steps
    if (steps >  255) steps =  255;
    if (steps < -255) steps = -255;
    if (steps != last_steps && !(RTC->ISR & RTC_ISR_RECALPF)) {
      // IRQ-off around the WPR unlock/write/relock triplet: PendSV's write_rtc() (runs each
      // second in holdover) does its own WPR sequence, and a preemption between our key
      // writes and the CALR store would leave the store silently ignored.
      __disable_irq();
      __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
      RTC->CALR = 0x100 + steps;              // same midpoint convention as calibrateRTC
      __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
      __enable_irq();
      last_steps = steps;
      last_calr = now;                        // note: BKP31R deliberately NOT updated
    }
  }
}

// "tc_dump = on" over serial: emit the learned model as ready-to-paste config.txt lines plus
// two checksummed $PMTXTC sentences (H and L — split so each fits NMEA_BUF_SIZE). One line per
// main-loop pass; each line is FORMATTED ONCE and only the CDC submit is retried on BUSY (float
// snprintf must not re-run thousands of times against the ISR's own float sprintf — newlib-nano
// shares one _reent). A stuck host aborts the dump after a bounded number of BUSY passes.
// HSE coefficients are printed in ppm/°C (per-degree slope b and curvature c, converted from
// the tick-domain model); the HSE 'a' term has an arbitrary instrument origin and is neither
// printed nor needed — steering uses temperature DIFFERENCES only (see tc_hse_delta).
static void tc_dump_step(void){
  static uint8_t  idx = 0;
  static int      dn = -1;                    // formatted length; -1 = line not built yet
  static uint16_t busy_ct = 0;
  static char     dline[NMEA_BUF_SIZE];
  if (!tc_dump_pending) return;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { tc_dump_pending = 0; idx = 0; dn = -1; return; }

  if (dn < 0) {                               // build the current line exactly once
    float tpp = (float)tc_tpp();
    int n = 0;
    switch (idx) {
      case 0:
        n = snprintf(dline, sizeof dline, "# tempcomp: hse n=%lu lse n=%lu, die %d..%d C, state %c\r\n",
                     (unsigned long)tc_n_hse, (unsigned long)tc_n_lse,
                     (int)tc_hse_tmin, (int)tc_hse_tmax, tc_disp_state);
        break;
      case 1: n = snprintf(dline, sizeof dline, "tc_t0 = %d\r\n", (int)tc_t0); break;
      case 2:                                 // HSE slope, ppm/degC (origin-free)
        if (tc_hse_valid) n = snprintf(dline, sizeof dline, "tc_hse_b = %.5f\r\n", (double)(tc_hse_m[1] / tpp));
        else              n = snprintf(dline, sizeof dline, "# tc_hse_b = ----\r\n");
        break;
      case 3:                                 // HSE curvature, ppm/degC^2
        if (tc_hse_valid) n = snprintf(dline, sizeof dline, "tc_hse_c = %.6f\r\n", (double)(tc_hse_m[2] / tpp));
        else              n = snprintf(dline, sizeof dline, "# tc_hse_c = ----\r\n");
        break;
      case 4: case 5: case 6: {               // LSE a/b/c, absolute ppm at tc_t0
        static const char nm[3] = {'a','b','c'};
        static const char *fm[3] = {"tc_lse_%c = %.4f\r\n", "tc_lse_%c = %.5f\r\n", "tc_lse_%c = %.6f\r\n"};
        int k = idx - 4;
        if (tc_lse_valid) n = snprintf(dline, sizeof dline, fm[k], nm[k], (double)tc_lse_m[k]);
        else              n = snprintf(dline, sizeof dline, "# tc_lse_%c = ----\r\n", nm[k]);
        break;
      }
      case 7: case 8: {                       // machine-parsable pair for the web app
        char body[72];
        int nb;
        if (idx == 7)
          nb = snprintf(body, sizeof body, "PMTXTC,H,%lu,%d,%d,%.5f,%.6f,%c",
                        (unsigned long)tc_n_hse, (int)tc_hse_tmin, (int)tc_hse_tmax,
                        (double)(tc_hse_valid ? tc_hse_m[1] / tpp : 0),
                        (double)(tc_hse_valid ? tc_hse_m[2] / tpp : 0), tc_hse_valid ? 'V' : '-');
        else
          nb = snprintf(body, sizeof body, "PMTXTC,L,%lu,%.4f,%.5f,%.6f,%c",
                        (unsigned long)tc_n_lse,
                        (double)(tc_lse_valid ? tc_lse_m[0] : 0), (double)(tc_lse_valid ? tc_lse_m[1] : 0),
                        (double)(tc_lse_valid ? tc_lse_m[2] : 0), tc_lse_valid ? 'V' : '-');
        if (nb < 0 || nb >= (int)sizeof body) { tc_dump_pending = 0; idx = 0; return; }
        uint8_t cks = 0;
        for (int i2 = 0; i2 < nb; i2++) cks ^= (uint8_t)body[i2];
        n = snprintf(dline, sizeof dline, "$%s*%02X\r\n", body, (unsigned)cks);
        break;
      }
      case 9: {                               // seed coverage — with "tc_seed = on" the paste warm-starts
        int lo = tc_hse_valid ? tc_hse_tmin : tc_lse_tmin;   // (and keeps evolving) instead of freezing
        int hi = tc_hse_valid ? tc_hse_tmax : tc_lse_tmax;
        if (tc_lse_valid) { if (tc_lse_tmin < lo) lo = tc_lse_tmin; if (tc_lse_tmax > hi) hi = tc_lse_tmax; }
        n = snprintf(dline, sizeof dline, "tc_seed_lo = %d\r\ntc_seed_hi = %d\r\n", lo, hi);
        break;
      }
    }
    if (n <= 0 || n >= (int)sizeof dline) { tc_dump_pending = 0; idx = 0; dn = -1; return; }
    dn = n;
    busy_ct = 0;
  }

  __disable_irq();                            // serialise against the ISR NMEA passthrough
  uint8_t r = CDC_Copy_Transmit((uint8_t*)dline, (uint16_t)dn);
  __enable_irq();
  if (r == USBD_BUSY) {                       // retry the SUBMIT only; the line stays built
    if (++busy_ct > 5000) { tc_dump_pending = 0; idx = 0; dn = -1; }  // host stopped reading
    return;
  }
  dn = -1;
  if (++idx > 9) { idx = 0; tc_dump_pending = 0; }
}

// "adev_dump = on" over serial: emit the whole live Allan-deviation curve as ONE checksummed
// sentence — $PMADEV,<valid>,<noct>,<sigma_0>,..,<sigma_{noct-1}>*CC — with sigma_k = sigma_y(tau)
// at tau = 2^k s (fractional frequency, %.2e). valid = contiguous phase samples behind the estimate
// (confidence). Fresh cache computed here (thread context) so the dump works in any display mode.
// Serviced from the main loop; retries only the CDC submit on USBD_BUSY, like tc_dump_step/emitPPS.
static void adev_dump_step(void){
  static int      dn = -1;                    // formatted length; -1 = not built yet
  static uint16_t busy_ct = 0;
  static char     dline[160];                 // epoch+tau0 header + 11 octaves outgrow NMEA_BUF_SIZE
  if (!adev_dump_pending) return;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { adev_dump_pending = 0; dn = -1; return; }

  if (dn < 0) {
    adev_reduce();                            // fresh octave cache (never in an ISR)
    uint8_t  noct  = adev_noct;
    uint16_t valid = adev_valid;
    char body[128];
    // Self-describing for machine consumers: epoch (unix s) + tau0 (s) lead the sentence, so tau_k =
    // tau0 * 2^k needs no out-of-band spec and stale sentences are detectable.
    int nb = snprintf(body, sizeof body, "PMADEV,%lu,1,%u,%u",
                      (unsigned long)(uint32_t)currentTime, (unsigned)valid, (unsigned)noct);
    if (nb < 0 || nb >= (int)sizeof body) { adev_dump_pending = 0; return; }
    for (uint8_t k = 0; k < noct; k++) {
      int t = snprintf(body + nb, sizeof body - nb, ",%.2e", (double)adev_sigma_cache[k]);
      if (t < 0 || t >= (int)(sizeof body - nb)) { adev_dump_pending = 0; return; }
      nb += t;
    }
    uint8_t cks = 0;
    for (int i = 0; i < nb; i++) cks ^= (uint8_t)body[i];
    int n = snprintf(dline, sizeof dline, "$%s*%02X\r\n", body, (unsigned)cks);
    if (n <= 0 || n >= (int)sizeof dline) { adev_dump_pending = 0; return; }
    dn = n; busy_ct = 0;
  }

  __disable_irq();                            // serialise against the ISR NMEA passthrough
  uint8_t r = CDC_Copy_Transmit((uint8_t*)dline, (uint16_t)dn);
  __enable_irq();
  if (r == USBD_BUSY) {                        // retry the SUBMIT only; the line stays built
    if (++busy_ct > 5000) { adev_dump_pending = 0; dn = -1; }
    return;
  }
  dn = -1; adev_dump_pending = 0;
}

// "hdev_dump = on": one $PMHDEV sentence — the Hadamard twin of $PMADEV (same shape: epoch, tau0,
// valid, noct, sigmas), computed on demand from the shared phase ring. Same CDC/BUSY-retry contract.
static void hdev_dump_step(void){
  static int      dn = -1;
  static uint16_t busy_ct = 0;
  static char     dline[160];
  if (!hdev_dump_pending) return;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { hdev_dump_pending = 0; dn = -1; return; }

  if (dn < 0) {
    uint8_t  noct  = hdev_noct();
    uint16_t valid = adev_valid;
    char body[128];
    int nb = snprintf(body, sizeof body, "PMHDEV,%lu,1,%u,%u",
                      (unsigned long)(uint32_t)currentTime, (unsigned)valid, (unsigned)noct);
    if (nb < 0 || nb >= (int)sizeof body) { hdev_dump_pending = 0; return; }
    for (uint8_t k = 0; k < noct; k++) {
      int t = snprintf(body + nb, sizeof body - nb, ",%.2e", (double)hdev_sigma_for_m(1u<<k));
      if (t < 0 || t >= (int)(sizeof body - nb)) { hdev_dump_pending = 0; return; }
      nb += t;
    }
    uint8_t cks = 0;
    for (int i = 0; i < nb; i++) cks ^= (uint8_t)body[i];
    int n = snprintf(dline, sizeof dline, "$%s*%02X\r\n", body, (unsigned)cks);
    if (n <= 0 || n >= (int)sizeof dline) { hdev_dump_pending = 0; return; }
    dn = n; busy_ct = 0;
  }

  __disable_irq();
  uint8_t r = CDC_Copy_Transmit((uint8_t*)dline, (uint16_t)dn);
  __enable_irq();
  if (r == USBD_BUSY) {
    if (++busy_ct > 5000) { hdev_dump_pending = 0; dn = -1; }
    return;
  }
  dn = -1; hdev_dump_pending = 0;
}

// Main-loop entry point, called every pass. With every tc key at its default this reduces to
// four flag checks — no measurable cost, no behaviour change.
// Holdover fade: from the residual 1σ time uncertainty during GPS-loss holdover, set each trailing
// sub-second digit's intensity by its remaining SIGNIFICANCE. U(τ) = k_σ·σ·τ (µs); σ = RSS of three
// independent ppm terms — how well the last cal pinned frequency, the MEASURED temp-model residual,
// and aging. A digit fades over its significance band and goes dark once U exceeds its place value.
static void computeHoldoverFade(void){
  uint32_t age = (uint32_t)currentTime - last_pps_time;              // holdover seconds

  // σ_cal — frequency knowledge from the last RTC calibration, decaying with its age.
  float cal_age = (float)((uint32_t)currentTime - (uint32_t)rtc_last_calibration);
  float sigma_cal;
  if (cal_age <= (float)CAL_PERIOD && tc_lse_valid) {
    float cal_ppm = (float)debug_rtc_val * (1e6f / (32768.0f * (float)CAL_PERIOD));
    sigma_cal = fabsf(cal_ppm) + 0.05f * sqrtf(cal_age / (float)CAL_PERIOD);
  } else {
    sigma_cal = 0.02f * sqrtf(cal_age);                             // stale cal: random-walk bound
  }

  // σ_temp — the MEASURED tempco-model residual (ppm), plus a penalty beyond the learned range.
  float sigma_temp;
  if (tc_hse_valid) {
    float tpp = (float)tc_tpp();
    sigma_temp = (tpp > 0.0f) ? tc_hse_resid / tpp : 10.0f;         // HSE residual (ticks/s) → ppm
    int t = die_temp_c;
    int over = t < tc_hse_tmin ? tc_hse_tmin - t : t > tc_hse_tmax ? t - tc_hse_tmax : 0;
    if (over > 0) sigma_temp += 0.3f * (float)over;                 // unvalidated beyond coverage
  } else {
    sigma_temp = 10.0f;                                            // no model yet — bare-crystal
  }

  // σ_age — long-term oscillator aging (negligible over minutes/hours, kept for completeness).
  float sigma_age = 0.1f * (float)age / 86400.0f;

  float sigma = sqrtf(sigma_cal*sigma_cal + sigma_temp*sigma_temp + sigma_age*sigma_age);
  float U_us  = 3.0f * sigma * (float)age;                          // k_σ = 3 ("certainly right")
  holdover_u_us = U_us;                                             // publish for read-back / display

  // Half place values (µs): 0.1 s, 0.01 s, 0.001 s. Fade band β. b_k = (h−U)/(β·h), clamped 0..1.
  static const float h_us[3] = { 50000.0f, 5000.0f, 500.0f };
  const float beta = 0.4f;
  for (int k = 0; k < 3; k++){
    float b = (h_us[k] - U_us) / (beta * h_us[k]);
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    digit_bright[k] = (uint8_t)(b * (float)FADE_MAX + 0.5f);
  }
  digit_bright[3] = digit_bright[0];   // the decimal point dies with the 0.1 s digit
}

void tc_housekeeping(void){
  if (!tc_nom_load) tc_nom_load = SysTick->LOAD;       // capture the nominal period once

  // Serial warm-start, deferred out of the USB ISR: "tc_seed = on" armed the flag; apply here,
  // serialized with tc_fit/tc_governor (the learned state stays main-loop-only). With the seed
  // already applied, the same call is the freeze guard — it just re-NANs any tc_hse_*/tc_lse_*
  // coefficients a later serial line reparsed, so the frozen path can't reactivate over the
  // evolving model.
  if (tc_seed_pending) { tc_seed_pending = 0; tc_seed_apply(); }
  else if (tc_seed_done) tc_seed_apply();

  if (tc_reset_pending) {
    memset(tc_bins, 0, sizeof tc_bins);
    tc_hse_valid = tc_lse_valid = 0;
    tc_hse_prior = tc_lse_prior = 0;          // drop any held warm-start prior: reset is a cold restart
    tc_n_hse = tc_n_lse = 0;
    tc_e0_set = 0; tc_ema = 0;                // new origin rebase with the next sample
    tc_reset_pending = 0;
  }

  if (tc_learn) {
    tc_hse_learn();
    tc_lse_learn();
    static uint32_t last_fit = 0;
    uint32_t now = (uint32_t)currentTime;
    if (now - last_fit >= 300) { last_fit = now; tc_fit(); }   // refit at most every 5 min
  }

  // tc_steer_on in the gate: the governor owns DISENGAGE, so it must stay reachable even if
  // the user turns every tc key off while steering is engaged mid-holdover — otherwise the
  // tick ISR would keep applying a stale frozen correction forever.
  if (tc_learn || tc_apply || tc_rtc || tc_steer_on || displayMode == MODE_TEMPCOMP) tc_governor();

  if (significance_fade) {   // recompute the per-digit fade once per second while enabled
    static uint32_t last_fade = 0;
    uint32_t now = (uint32_t)currentTime;
    if (now != last_fade) { last_fade = now; computeHoldoverFade(); }
  }

  tc_dump_step();
}

// tc_steer(): holdover rate steering (see tc_governor). Sets the length of the NEXT 1 ms
// period: LOAD writes take effect at the following reload, so distributing tc_rem longer
// periods per 1000 gives an average of base + rem/1000 extra ticks per ms — fractional-ppm
// rate control with three int32 ops. tc_steer_on is 0 unless tc_apply engaged in holdover,
// so the stock cost is one predicted-untaken branch per ms.
#define tc_steer() \
    if (tc_steer_on) { \
      tc_acc += tc_rem; \
      if (tc_acc >= 1000) { tc_acc -= 1000; SysTick->LOAD = (uint32_t)(tc_load_base + 1); } \
      else                { SysTick->LOAD = (uint32_t)tc_load_base; } \
    }

#define timetick() \
    tc_steer(); \
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

  segbal_isr_refresh();   // mirrors track the masters at ISR freshness (seg_balance)

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

  segbal_isr_refresh();

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

  segbal_isr_refresh();

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextTimestamp( currentTime );
    sendDate(0);
  }
}

void SysTick_CountUp_P0(void) {

  timetick()

  segbal_isr_refresh();

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    currentTime++;
    setNextTimestamp( currentTime );
    sendDate(0);
  }
}

void SysTick_CountUp_NoUpdate(void) {
  tc_steer();                         // this handler inlines its own cascade: hook it too
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

  segbal_isr_refresh();   // masters are main-loop-drawn here (TEXT etc.) — keep mirrors no staler

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

  segbal_isr_refresh();

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

  segbal_isr_refresh();

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

  segbal_isr_refresh();

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

  segbal_isr_refresh();

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

  segbal_isr_refresh();

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    alt_prep_next();
  }
}

void SysTick_Alt_P2(void) {
  timetick()

  buffer_c[2].low=cLut[centisec];
  buffer_c[1].low=cLut[decisec];

  segbal_isr_refresh();

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    alt_prep_next();
  }
}

void SysTick_Alt_P1(void) {
  timetick()

  buffer_c[1].low=cLut[decisec];

  segbal_isr_refresh();

  HAL_IncTick();

  if (decisec==9 && centisec==0 && millisec==0){
    alt_prep_next();
  }
}

void SysTick_Alt_P0(void) {
  timetick()

  segbal_isr_refresh();

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
  if (significance_fade && countMode == COUNT_NORMAL) {
    // Holdover fade replaces the FIXED Tolerance_time_* dash ladder with a SIGNIFICANCE-driven one:
    // computeHoldoverFade() sets each sub-second digit's intensity in digit_bright[] from the live
    // time-interval-error bound, and a digit goes BLACK once its significance reaches zero — it
    // FADES OUT rather than dashing: digits still significant keep ticking (P3/P2/P1) while the
    // seg-balance duty mirror scales each one's lit cycles by its digit_bright, so the display
    // renders the partial fade of the digit on its way out (the emulator reads the same levels).
    // At lock all are FADE_MAX, so this reduces to a plain P3. digit_bright = [ds, cs, ms, dp];
    // buffer_c[3]/[2]/[1] = ms/cs/ds; the decimal point dies with the 0.1 s digit.
    if (digit_bright[2]) {                                 // ms still significant
      buffer_c[0].high = 0b11001110 | cSegDP;
      SetSysTick( &SysTick_CountUp_P3 );
    } else if (digit_bright[1]) {                          // ms faded out, cs significant
      buffer_c[3].low = 0;
      buffer_c[0].high = 0b11001110 | cSegDP;
      SetSysTick( &SysTick_CountUp_P2 );
    } else if (digit_bright[0]) {                          // ds only
      buffer_c[3].low = 0;
      buffer_c[2].low = 0;
      buffer_c[0].high = 0b11001110 | cSegDP;
      SetSysTick( &SysTick_CountUp_P1 );
    } else {                                               // whole seconds
      buffer_c[3].low = 0;
      buffer_c[2].low = 0;
      buffer_c[1].low = 0;
      buffer_c[0].high = 0b11001110;
      SetSysTick( &SysTick_CountUp_P0 );
    }
    return;
  }
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
  adev_reset();   // clear the (un-zeroed) RAM2 Allan-deviation ring before the first PPS sample

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
    LP_MARK(1);      // $PMLOOP section marks — see the pmloop block above segbal_isr_refresh
    if (loop_diag && (uint32_t)(uwTick - pmloop_win) >= 1000u) {
      pmloop_win = uwTick;
      char pl[40];
      int pn = sprintf(pl, "$PMLOOP,%lu,%u\r\n", (unsigned long)pmloop_max, (unsigned)pmloop_maxtag);
      __disable_irq(); CDC_Copy_Transmit((uint8_t*)pl, (uint16_t)pn); __enable_irq();   // lossy: diag
      pmloop_max = 0;
    }
    LP_MARK(2);
    segbal_poll();   // per-segment brightness balance (seg_balance) — refills the mirror slots, ≤1 kHz
    colon_balance_poll();   // dim the colons with the rail (colon_balance) — reloads the anim buffer on change

    LP_MARK(3);
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

    LP_MARK(4);
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

    LP_MARK(5);
    monitor_vbus();

    // significance_fade is a die-temp consumer too: computeHoldoverFade charges an out-of-coverage
    // penalty from die_temp_c, which would otherwise stay at its init 0 with every other flag off.
    if (pps_ts_enabled || tc_learn || tc_apply || tc_rtc || significance_fade || displayMode == MODE_TEMPCOMP) {
      static uint32_t last_temp_read = 0;
      if ((uint32_t)currentTime - last_temp_read >= 4) {   // refresh die temp every ~4 s
        last_temp_read = (uint32_t)currentTime;
        measure_temp();
      }
    }
    LP_MARK(6);
    if (pps_ts_enabled && pps_record_pending) emitPPSTimestamp(); // emit clears pending itself on success

    tc_housekeeping();   // temp-comp learn/steer/dump; four flag checks when everything is off
    adev_dump_step();    // one-shot $PMADEV emit when adev_dump was set over serial (else 1 flag check)
    hdev_dump_step();    // one-shot $PMHDEV (Hadamard) twin

    LP_MARK(7);
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
      LP_MARK(8);
    if ((displayMode == MODE_SUN || displayMode == MODE_LATLON || displayMode == MODE_DARK) && astro.have_pos && astro.epoch) {
        static uint32_t last_pg = 0;
        uint32_t pg = uwTick / page_ms();
        if (pg != last_pg && decisec != 9) { last_pg = pg; sendDate(1); }
      }
    }

    // MODE_TEMPCOMP pages on the same dwell: repaint on the page flip (same guard as above)
    if (displayMode == MODE_TEMPCOMP) {
      static uint32_t tc_last_pg = 0;
      uint32_t pg = uwTick / page_ms();
      if (pg != tc_last_pg && decisec != 9) { tc_last_pg = pg; sendDate(1); }
    }

    // MODE_ADEV pages the octave taus on the same dwell. Recompute the octave cache on each flip
    // (thread context, ~sub-ms) so the shown sigma tracks the growing series, then repaint. The
    // 0xFFFFFFFF sentinel forces a reduce+paint on first entry rather than waiting a full dwell.
    if (displayMode == MODE_ADEV) {
      static uint32_t adev_last_pg = 0xFFFFFFFFu;
      uint32_t pg = uwTick / page_ms();
      if (pg != adev_last_pg && decisec != 9) { adev_last_pg = pg; adev_display_update(); sendDate(1); }
    }

    // MODE_LST / MODE_SOLAR: stage the next civil boundary's alternate reading
    // (thread-context doubles; no-op in every other mode)
    LP_MARK(9);
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
