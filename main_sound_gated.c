/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FOUR-MIC acoustic camera - GCC-PHAT azimuth + elevation
  *                   triggers on any sound louder than the room
  *
  *   m0  L1  PC3   L/R->GND   top-left
  *   m1  L2  PC3   L/R->3V3   bottom-left
  *   m2  R1  PC12  L/R->GND   top-right
  *   m3  R2  PC12  L/R->3V3   bottom-right
  *
  *   All four in ONE VERTICAL PLANE, facing the sound.
  *   D_X : m0-m2 horizontal  -> azimuth   (+ = source to your right)
  *   D_Y : m0-m1 vertical    -> elevation (+ = source above centre)
  *
  *   Measures the direction of ANY sound that rises clearly above the
  *   room's background: a clap, speech, a whistle, music, a door.
  *
  *   While the room is quiet the board says so and reports NO angle at
  *   all - a stale bearing is worse than none, because it looks like a
  *   measurement.  An angle is printed only for blocks that actually
  *   contain a sound.  A line marked '*' was a sharp onset.
  *
  *   Needs CMSIS-DSP:  Project > Manage > Run-Time Environment > CMSIS > DSP
  *
  *   Keys:
  *     v   verbose (show per-pair lags and agreement)
  *     h   also print a QUIET line each interval, or stay silent
  *     c   detail for the next measurement
  *     +/- sensitivity: how far above background a sound must be
  *     q/Q quality threshold (lower case raises it)
  *     z   set current direction as (0,0)      Z  clear calibration
  *     x   force I2S re-sync
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2s.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define BLK        1024
#define NFFT       1024
#define BUFLEN     (BLK * 2 * 2)

/* ---- MEASURED ARRAY GEOMETRY --------------------------------------------- */
#define D_X          0.112f      /* horizontal, m0 to m2, metres            */
#define D_Y          0.112f      /* vertical,   m0 to m1, metres            */
/* -------------------------------------------------------------------------- */

#define SOUND_C      343.0f
#define FS_HZ        31914.0f

/*  Band-pass, applied by zeroing FFT bins.
 *
 *  WIDE is what makes GCC-PHAT work.  The width of the correlation peak is
 *  roughly 1/bandwidth, so a narrow band gives a peak many samples across
 *  and the position of its maximum becomes meaningless.  With 300-6000 Hz
 *  the peak is only a few samples wide and lands where it should.
 *  A clap has energy across all of this.
 */
#define F_LO          300.0f
#define F_HI         6000.0f

/*  Largest delay the geometry allows:
 *  0.112 / 343 * 31914 = 10.4 samples.  Searching wider than the physics
 *  permits only invites false peaks, so allow a little margin and no more. */
#define MAXLAG       12

#define REPORT_MS    200
#define NSMOOTH       9          /* median length, continuous mode only     */

#define COARSE_STEP  0.05f
#define COARSE_N     20
#define FINE_STEP    0.004f
#define FINE_N       16

/* ---- orientation ----------------------------------------------------------
 *  Set these once by experiment, then leave them alone.
 *
 *    INVERT_AZ  sound on your RIGHT reads negative      -> set to 1
 *    INVERT_EL  sound ABOVE centre reads negative       -> set to 1
 *    SWAP_AXES  moving left/right moves EL not AZ       -> set to 1
 *               (the array is mounted rotated 90 degrees)
 *
 *  Work them out in this order: SWAP first, then the two inversions.
 */
#define INVERT_AZ     0
#define INVERT_EL     0
#define SWAP_AXES     0

/* ---- sound detection ------------------------------------------------------
 *  A block is measured when it is BOTH above an absolute floor AND clearly
 *  louder than the room's own background.  The second test is what stops
 *  the array chasing its own noise floor in a silent room.
 */
#define BG_ALPHA      0.02f      /* how fast the background estimate moves  */
#define TRIG_RATIO    2.0f       /* times above background to be "a sound"  */
#define TRIG_FLOOR   40.0f       /* absolute minimum rms worth measuring    */
#define ONSET_RATIO   4.0f       /* a jump this big is flagged as a transient */

/* ---- re-sync watchdog ---------------------------------------------------- */
#define SUSPECT_LEVEL   30000
#define SLIP_SAMPLES    40
#define SLIP_BLOCKS      3
#define RESYNC_COOLDOWN 50
/* USER CODE END PD */

/* USER CODE BEGIN PV */
int16_t bufA[BUFLEN];               /* m0 (left), m1 (right) */
int16_t bufB[BUFLEN];               /* m2 (left), m3 (right) */

volatile int      block_ready  = 0;
volatile uint32_t blocksA_seen = 0;
volatile uint32_t blocksB_seen = 0;
volatile uint32_t overruns     = 0;

static float sig[4][NFFT];
static float SP [4][NFFT];
static float X  [NFFT];
static float xc [NFFT];
static float win[NFFT];             /* Tukey - kinder to transients        */

static arm_rfft_fast_instance_f32 fft;

static int   kLo, kHi;

static float hist_az[NSMOOTH], hist_el[NSMOOTH];
static int   hist_n = 0;

static float bg[4]    = {0.0f, 0.0f, 0.0f, 0.0f};   /* background level    */
static uint32_t events = 0;                         /* measurements made   */

static uint32_t quiet_blocks = 0;   /* consecutive blocks with no sound    */

static float trig_ratio = TRIG_RATIO;
static float q_min      = 1.8f;
static float cal_az = 0.0f, cal_el = 0.0f;

static uint32_t slip_run = 0, cooldown = 0, resyncs = 0;

volatile uint8_t rx_byte     = 0;
volatile uint8_t want_cor    = 0;
volatile uint8_t want_zero   = 0;
volatile uint8_t want_resync = 0;
volatile uint8_t show_quiet  = 1;   /* announce the quiet periods too      */
volatile uint8_t verbose     = 0;

extern UART_HandleTypeDef huart2;
extern I2S_HandleTypeDef  hi2s2;
extern I2S_HandleTypeDef  hi2s3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
static void  prepare_block(int half, float *rms, uint32_t *suspect);
static void  spectra(void);
static float pair_delay(int ia, int ib, float *quality);
static float xc_at(float tau);
static float refine(int coarse);
static float median_of(const float *v, int n);
static void  resync_i2s(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int fputc(int c, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, HAL_MAX_DELAY);
  return c;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
  if (h->Instance == USART2) {
    switch (rx_byte) {
      case 'h': show_quiet = !show_quiet;
                printf("\r\nquiet lines %s\r\n", show_quiet ? "on" : "off");
                break;
      case 'v': verbose = !verbose; break;
      case 'c': want_cor = 1; break;
      case 'z': want_zero = 1; break;
      case 'Z': cal_az = cal_el = 0.0f;
                printf("\r\ncalibration cleared\r\n"); break;
      case 'x': want_resync = 1; break;
      case '+': trig_ratio *= 1.25f;
                printf("\r\nsensitivity: needs x%.2f background\r\n", trig_ratio);
                break;
      case '-': trig_ratio /= 1.25f; if (trig_ratio < 1.1f) trig_ratio = 1.1f;
                printf("\r\nsensitivity: needs x%.2f background\r\n", trig_ratio);
                break;
      case 'q': q_min += 0.2f; printf("\r\nq_min %.1f\r\n", q_min); break;
      case 'Q': q_min -= 0.2f; if (q_min < 0.0f) q_min = 0.0f;
                printf("\r\nq_min %.1f\r\n", q_min); break;
      default: break;
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1);
  }
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *h)
{
  if (h->Instance == SPI2) {
    if (block_ready) overruns++;
    block_ready = 1; blocksA_seen++;
  } else if (h->Instance == SPI3) blocksB_seen++;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *h)
{
  if (h->Instance == SPI2) {
    if (block_ready) overruns++;
    block_ready = 2; blocksA_seen++;
  } else if (h->Instance == SPI3) blocksB_seen++;
}

static void resync_i2s(void)
{
  HAL_I2S_DMAStop(&hi2s2);
  HAL_I2S_DMAStop(&hi2s3);
  HAL_Delay(2);
  block_ready = 0;
  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*)bufB, BUFLEN);
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)bufA, BUFLEN);
  resyncs++;
  slip_run = 0;
  cooldown = RESYNC_COOLDOWN;
}

static void prepare_block(int half, float *rms, uint32_t *suspect)
{
  int     off = (half == 1) ? 0 : BLK * 2;
  int     i, c;
  float   mean[4], pw[4];
  int16_t v;

  for (c = 0; c < 4; c++) { mean[c] = 0.0f; pw[c] = 0.0f; suspect[c] = 0; }

  for (i = 0; i < BLK; i++) {
    v = bufA[off + 2*i    ]; sig[0][i] = (float)v;
      if (v > SUSPECT_LEVEL || v < -SUSPECT_LEVEL) suspect[0]++;
    v = bufA[off + 2*i + 1]; sig[1][i] = (float)v;
      if (v > SUSPECT_LEVEL || v < -SUSPECT_LEVEL) suspect[1]++;
    v = bufB[off + 2*i    ]; sig[2][i] = (float)v;
      if (v > SUSPECT_LEVEL || v < -SUSPECT_LEVEL) suspect[2]++;
    v = bufB[off + 2*i + 1]; sig[3][i] = (float)v;
      if (v > SUSPECT_LEVEL || v < -SUSPECT_LEVEL) suspect[3]++;

    for (c = 0; c < 4; c++) mean[c] += sig[c][i];
  }

  for (c = 0; c < 4; c++) mean[c] /= (float)BLK;

  for (i = 0; i < BLK; i++)
    for (c = 0; c < 4; c++) {
      sig[c][i] -= mean[c];
      pw[c] += sig[c][i] * sig[c][i];
      sig[c][i] *= win[i];
    }

  for (c = 0; c < 4; c++) rms[c] = sqrtf(pw[c] / (float)BLK);
}

static void spectra(void)
{
  int c;
  for (c = 0; c < 4; c++)
    arm_rfft_fast_f32(&fft, sig[c], SP[c], 0);
}

static float xc_at(float tau)
{
  int   k;
  float th  = 2.0f * PI * tau / (float)NFFT;
  float cs  = cosf(th), sn = sinf(th);
  float c   = cosf(th * (float)kLo), s = sinf(th * (float)kLo);
  float acc = 0.0f, ct;

  for (k = kLo; k <= kHi; k++) {
    acc += X[2*k] * c - X[2*k+1] * s;
    ct = c * cs - s * sn;
    s  = s * cs + c * sn;
    c  = ct;
  }
  return acc * 2.0f / (float)NFFT;
}

static float refine(int coarse)
{
  float best_t = (float)coarse, best_v = -1e30f, t, v, centre;
  int   i;

  for (i = -COARSE_N; i <= COARSE_N; i++) {
    t = (float)coarse + COARSE_STEP * (float)i;
    v = xc_at(t);
    if (v > best_v) { best_v = v; best_t = t; }
  }
  centre = best_t;
  for (i = -FINE_N; i <= FINE_N; i++) {
    t = centre + FINE_STEP * (float)i;
    v = xc_at(t);
    if (v > best_v) { best_v = v; best_t = t; }
  }
  return best_t;
}

static float pair_delay(int ia, int ib, float *quality)
{
  int   k, n, idx, best = 0;
  float ar, ai, br, bi, xr, xi, mag;
  float bestv = -1e30f, sum = 0.0f;

  X[0] = 0.0f;
  X[1] = 0.0f;

  for (k = 1; k < NFFT/2; k++) {
    if (k < kLo || k > kHi) { X[2*k] = 0.0f; X[2*k+1] = 0.0f; continue; }

    ar = SP[ia][2*k];  ai = SP[ia][2*k+1];
    br = SP[ib][2*k];  bi = SP[ib][2*k+1];

    xr = ar*br + ai*bi;              /* conj(A) * B */
    xi = ar*bi - ai*br;

    mag = sqrtf(xr*xr + xi*xi);
    if (mag > 1e-12f) { xr /= mag; xi /= mag; }
    else              { xr = 0.0f;  xi = 0.0f; }

    X[2*k]   = xr;
    X[2*k+1] = xi;
  }

  arm_rfft_fast_f32(&fft, X, xc, 1);

  for (n = -MAXLAG; n <= MAXLAG; n++) {
    idx = (n >= 0) ? n : NFFT + n;
    sum += fabsf(xc[idx]);
    if (xc[idx] > bestv) { bestv = xc[idx]; best = n; }
  }

  {
    float mean = sum / (float)(2*MAXLAG + 1);
    *quality = (mean > 1e-12f) ? (bestv / mean) : 0.0f;
  }

  return refine(best);
}

static float median_of(const float *v, int n)
{
  float t[NSMOOTH], tmp;
  int i, j;
  for (i = 0; i < n; i++) t[i] = v[i];
  for (i = 0; i < n-1; i++)
    for (j = i+1; j < n; j++)
      if (t[j] < t[i]) { tmp = t[i]; t[i] = t[j]; t[j] = tmp; }
  return t[n/2];
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t last_ms = 0;
  float    rms[4];
  uint32_t sus[4];
  float    tx1, tx2, ty1, ty2, qx1, qx2, qy1, qy2;
  float    tau_x, tau_y, q_avg;
  float    ux, uy, uz, r2, az, el, saz, sel;
  float    lvl, bgl, ratio;
  int      i, fired, onset, taper;
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /*  Tukey window, 25% taper.  A Hann window would flatten a transient
   *  landing near a block edge; this leaves the middle 75% untouched.  */
  taper = (int)(0.25f * (float)(NFFT - 1) / 2.0f);
  for (i = 0; i < NFFT; i++) {
    if (i < taper)
      win[i] = 0.5f * (1.0f - cosf(PI * (float)i / (float)taper));
    else if (i > NFFT - 1 - taper)
      win[i] = 0.5f * (1.0f - cosf(PI * (float)(NFFT - 1 - i) / (float)taper));
    else
      win[i] = 1.0f;
  }

  kLo = (int)(F_LO * (float)NFFT / FS_HZ);
  kHi = (int)(F_HI * (float)NFFT / FS_HZ);
  if (kLo < 1)          kLo = 1;
  if (kHi > NFFT/2 - 1) kHi = NFFT/2 - 1;

  if (arm_rfft_fast_init_f32(&fft, NFFT) != ARM_MATH_SUCCESS)
    printf("!! FFT init failed\r\n");

  printf("\r\n=== acoustic camera : sound direction ===\r\n");
  printf("D_X %.1f mm   D_Y %.1f mm   fs %.0f Hz\r\n",
         D_X*1000.0f, D_Y*1000.0f, FS_HZ);
  printf("band %.0f-%.0f Hz (%d bins)   max delay %.1f samples\r\n",
         F_LO, F_HI, kHi - kLo + 1, D_X / SOUND_C * FS_HZ);
  printf("orientation: invert_az %d  invert_el %d  swap %d\r\n",
         INVERT_AZ, INVERT_EL, SWAP_AXES);
  printf("Make any sound louder than the room: clap, speak, whistle.\r\n");
  printf("An angle is printed ONLY while a sound is present.\r\n");
  printf("keys: v verbose  h quiet-lines  c detail  +/- sensitivity"
         "  z zero  x resync\r\n\r\n");

  HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1);

  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*)bufB, BUFLEN);
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)bufA, BUFLEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (block_ready)
    {
      prepare_block(block_ready, rms, sus);
      block_ready = 0;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

      /* ---- slave alignment watchdog ---------------------------------- */
      if (cooldown > 0) cooldown--;
      if (sus[2] > SLIP_SAMPLES || sus[3] > SLIP_SAMPLES) slip_run++;
      else slip_run = 0;

      if (want_resync || (slip_run >= SLIP_BLOCKS && cooldown == 0)) {
        want_resync = 0;
        printf("\r\n>> re-sync (total %lu)\r\n\r\n", (unsigned long)(resyncs+1));
        resync_i2s();
        continue;
      }

      /* ---- level and background --------------------------------------- */
      lvl = 0.25f * (rms[0] + rms[1] + rms[2] + rms[3]);
      bgl = 0.25f * (bg[0] + bg[1] + bg[2] + bg[3]);

      ratio = (bgl > 0.5f) ? (lvl / bgl) : 0.0f;

      /* ---- is there a sound worth measuring? --------------------------- *
       *  Loud in absolute terms AND clearly above the room's background.
       *  Any sound qualifies - a clap, a word, a whistle, a door.         */
      fired = (lvl > TRIG_FLOOR) && (bgl > 0.5f) && (ratio > trig_ratio);
      onset = fired && (ratio > ONSET_RATIO);

      /* the background tracks the quiet blocks only, so a loud sound
         cannot drag up the very threshold it has to beat                 */
      if (!fired)
        for (i = 0; i < 4; i++)
          bg[i] = (1.0f - BG_ALPHA) * bg[i] + BG_ALPHA * rms[i];

      if (!fired) {
        quiet_blocks++;
        /*  The history belongs to a sound that has now stopped.  Clearing
         *  it stops the next sound being median-mixed with the last one.  */
        hist_n = 0;

        if (show_quiet && HAL_GetTick() - last_ms >= REPORT_MS) {
          last_ms = HAL_GetTick();
          printf("QUIET   lvl %5.0f   bg %5.0f   needs %5.0f"
                 "   (%lu blocks)\r\n",
                 lvl, bgl, trig_ratio * bgl, (unsigned long)quiet_blocks);
          /*  An explicit "nothing to report" for the host: the Python
           *  overlay clears its marker on this instead of holding a
           *  bearing that is no longer being measured.                  */
          printf("ANGLE none\r\n");
        }
        continue;
      }

      if (quiet_blocks) {
        /* a new sound after silence - say so, and start the median fresh */
        quiet_blocks = 0;
      }

      /* ---- four cross-correlations ------------------------------------ */
      spectra();

      tx1 = pair_delay(0, 2, &qx1);   /* top    pair, left -> right */
      tx2 = pair_delay(1, 3, &qx2);   /* bottom pair, left -> right */
      ty1 = pair_delay(0, 1, &qy1);   /* left   pair, top  -> bottom */
      ty2 = pair_delay(2, 3, &qy2);   /* right  pair, top  -> bottom */

      tau_x = 0.5f * (tx1 + tx2);
      tau_y = 0.5f * (ty1 + ty2);
      q_avg = 0.25f * (qx1 + qx2 + qy1 + qy2);

      if (q_avg < q_min) {
        /*  Loud enough, but the four channels did not agree well enough
         *  for the bearing to mean anything.  Say so; report no angle.  */
        if (HAL_GetTick() - last_ms >= REPORT_MS) {
          last_ms = HAL_GetTick();
          printf("WEAK    q %4.2f < %4.2f   lvl %5.0f (x%.1f)\r\n",
                 q_avg, q_min, lvl, ratio);
          printf("ANGLE none\r\n");
        }
        continue;
      }

      /*  s . u = -tau * c
       *    horizontal s = (+D_X, 0)  ->  ux = -tau_x * c / D_X
       *    vertical   s = (0, -D_Y)  ->  uy = +tau_y * c / D_Y        */
      ux = -tau_x / FS_HZ * SOUND_C / D_X;
      uy =  tau_y / FS_HZ * SOUND_C / D_Y;

#if SWAP_AXES
      { float t_swap = ux; ux = uy; uy = t_swap; }
#endif
#if INVERT_AZ
      ux = -ux;
#endif
#if INVERT_EL
      uy = -uy;
#endif

      r2 = ux*ux + uy*uy;
      if (r2 > 1.0f) {
        float k = 1.0f / sqrtf(r2);
        ux *= k; uy *= k; r2 = 1.0f;
      }
      uz = sqrtf(1.0f - r2);
      if (uz < 1e-4f) uz = 1e-4f;

      az = atan2f(ux, uz) * 180.0f / PI - cal_az;
      el = atan2f(uy, uz) * 180.0f / PI - cal_el;

      /* ---- always smooth: one bad block cannot throw the reading ------ */
      for (i = NSMOOTH-1; i > 0; i--) {
        hist_az[i] = hist_az[i-1];
        hist_el[i] = hist_el[i-1];
      }
      hist_az[0] = az;
      hist_el[0] = el;
      if (hist_n < NSMOOTH) hist_n++;

      saz = median_of(hist_az, hist_n);
      sel = median_of(hist_el, hist_n);

      events++;

      if (HAL_GetTick() - last_ms >= REPORT_MS) {
        last_ms = HAL_GetTick();

        printf("%s AZ %+6.1f   EL %+6.1f   q %4.2f   lvl %5.0f (x%.1f)\r\n",
               onset ? "*" : " ", saz, sel, q_avg, lvl, ratio);

        if (verbose)
          printf("   lagX %+6.2f / %+6.2f   lagY %+6.2f / %+6.2f"
                 "   (agree X %+.2f  Y %+.2f)   n=%lu\r\n",
                 tx1, tx2, ty1, ty2, tx1 - tx2, ty1 - ty2,
                 (unsigned long)events);

        printf("ANGLE az=%.2f el=%.2f\r\n", saz, sel);
      }

      if (want_zero) {
        want_zero = 0;
        cal_az += saz;
        cal_el += sel;
        printf("\r\n>> zeroed here: az offset %+.2f  el offset %+.2f\r\n\r\n",
               cal_az, cal_el);
      }

      if (want_cor) {
        want_cor = 0;
        printf("\r\n   pair quality  X %4.2f %4.2f   Y %4.2f %4.2f\r\n",
               qx1, qx2, qy1, qy2);
        printf("   pair agreement X %+.3f   Y %+.3f  samples\r\n",
               tx1 - tx2, ty1 - ty2);
        printf("   tau_x %+.3f  tau_y %+.3f samples\r\n", tau_x, tau_y);
        printf("   ux %+.4f  uy %+.4f  uz %+.4f\r\n\r\n", ux, uy, uz);
      }
    }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    Error_Handler();
}

void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
