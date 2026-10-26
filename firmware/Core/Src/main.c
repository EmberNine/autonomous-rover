/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>    /* snprintf: boot line only; the loop uses telemetry_format */
#include "as5600.h"
#include "icm42688.h"
#include "telemetry_format.h"
#include "byte_ring.h"
#include "command_line.h"
#include "command_dispatch.h"
#include "motor_control.h"
#include "tb6612.h"
#include "steering_control.h"
#include "servo.h"
#include "loop_schedule.h"
#include "battery_monitor.h"
#include "speed_capture.h"     /* SPEED_CAPTURE: 0 in a normal build, which uses none of it */
#include "angle_correction.h"
#include "speed_estimator.h"
#include "speed_control.h"
#include "stall_detect.h"
#include "encoder_fault.h"     /* ENCODER_FAULT_TEST: 0 in a normal build, which uses none of it */
#include "stm32l4xx_ll_adc.h"  /* VREFINT_CAL_ADDR, LL_ADC_* (static inline: no extra code until used) */
#include <string.h>   /* memset: clears the loop timing profile window */
#include "link_codec.h"
#include "link_protocol.h"
#include "link_rx.h"
#include "tx_queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* IMU part of one telemetry line, copied when the window closes so it can be
 * formatted one iteration later without mixing in the next window's data. */
typedef struct
{
  bool               imu_ok;
  ICM42688_RawSample latest;
  int32_t            gyro_sum_x;
  int32_t            gyro_sum_y;
  int32_t            gyro_sum_z;
  uint32_t           sample_count;
  uint32_t           imu_error_count;
  uint32_t           spi_us_max;
} ImuReport;

/* Motor part of one telemetry line, copied when the window closes (same
 * reason as ImuReport): it is formatted two iterations later. */
typedef struct
{
  int32_t  requested_percent;
  int32_t  applied_ticks;
  uint32_t state;
  uint32_t watchdog_trip_count;
  uint32_t valid_command_count;
  uint32_t rejected_command_count;
  uint32_t clamped_command_count;
  uint32_t rx_error_count;
} MotorReport;

/* Reset cause + steering part of one telemetry line (telemetry v3), copied
 * when the window closes for the same reason: it is formatted three
 * iterations later. */
typedef struct
{
  uint32_t reset_cause_flags;
  int32_t  requested_percent;
  uint32_t pulse_us;
  uint32_t state;
  uint32_t valid_command_count;
  uint32_t timeout_count;
} SteeringReport;

/* Battery part of one telemetry line (telemetry v4), copied when the window
 * closes and formatted with the steering part. */
typedef struct
{
  uint32_t vbat_mv;   /* filtered pack voltage */
  uint32_t state;     /* BatteryState */
  uint32_t vdda_mv;
} BatteryReport;

/* Speed control part of one telemetry line (telemetry v5), copied when the
 * window closes and formatted in the line's 5th iteration. */
typedef struct
{
  int32_t  setpoint_units;    /* vset: last "v" as received */
  int32_t  estimate_units;    /* vest: the 20 ms estimate at window close */
  uint32_t state;             /* ctl: SpeedControlState */
  uint32_t estimate_valid;    /* vok */
  int32_t  integrator_ticks;  /* vi */
  uint32_t stall_count;       /* stall_n */
  uint32_t enc_bad_count;     /* enc_bad */
  uint32_t fault_reason;      /* mfault: MotorFaultReason, 0 unless mstate=3 */
} SpeedReport;

/* A telemetry line is built over five consecutive iterations, so no single
 * iteration formats more than ~16 fields (see the telemetry section). */
typedef enum
{
  LINE_IDLE = 0,             /* nothing pending: the next window close starts a line */
  LINE_IMU_PART_NEXT,        /* encoder part written; IMU part in the next iteration */
  LINE_MOTOR_PART_NEXT,      /* IMU part written; motor part in the next iteration */
  LINE_STEERING_PART_NEXT,   /* motor part written; steering + battery part in the next iteration */
  LINE_SPEED_PART_NEXT       /* steering + battery written; speed part + send in the next iteration */
} TelemetryLineStep;

/* ---- TEMPORARY: loop timing profile (see LOOP_TIMING_PROFILE) ---------- */
/* Which kind of iteration a timing belongs to. Normal iterations are split by
 * whether the UART was sending when the iteration started, because every byte
 * it sends fires an interrupt that runs in the middle of the loop's work. */
typedef enum
{
  PROFILE_NORMAL_UART_IDLE = 0,
  PROFILE_NORMAL_UART_BUSY,
  PROFILE_HEALTH,          /* health due (normally t % 100 == 50): extra STATUS + AGC reads */
  PROFILE_REPORT_HEAD,     /* window close: encoder part of the telemetry line */
  PROFILE_REPORT_TAIL,     /* next iteration: IMU part */
  PROFILE_REPORT_MOTOR,    /* the one after: motor part */
  PROFILE_REPORT_STEERING, /* the one after that: steering + battery part */
  PROFILE_REPORT_SPEED,    /* the one after that: speed control part (v5) + UART start */
  PROFILE_LINE_BUILD,      /* builds/sends the profile line itself (its own overhead) */
  PROFILE_KIND_COUNT
} ProfileIterationKind;

/* Largest value seen in one window, per section, in DWT cycles. "other" is
 * whatever the named sections don't cover (total minus their sum). ctl and
 * est are parts of motor and encoder (not added again). */
typedef struct
{
  uint32_t iteration_count;
  uint32_t motor_cycles_max;       /* command input + speed control + motor and steering state machines + pin writes + battery sample */
  uint32_t ctl_cycles_max;         /* part of motor: SpeedControl_Update + StallDetect_Update */
  uint32_t encoder_cycles_max;
  uint32_t est_cycles_max;         /* part of encoder: AngleCorrection + corrected unwrapper + SpeedEstimator_Update */
  uint32_t health_reads_cycles_max;
  uint32_t imu_cycles_max;
  uint32_t format_cycles_max;
  uint32_t uart_start_cycles_max;
  uint32_t other_cycles_max;
  uint32_t total_cycles_max;
} ProfileSectionMaxima;

typedef struct
{
  ProfileSectionMaxima kinds[PROFILE_KIND_COUNT];
} ProfileWindow;

/* What the Pi link's STATE frame says about the ms since the previous STATE
 * mark: kept every ms in both modes, restarted at every mark (sent only in
 * binary mode), so it never grows beyond one 10 ms period. */
typedef struct
{
  int32_t  gyro_sum[3];        /* raw x, y, z */
  int32_t  accel_sum[3];
  uint32_t imu_samples;        /* samples in the sums (at most LINK_STATE_MAX_SAMPLES) */
  bool     imu_read_failed;    /* a burst read failed */
  bool     estimate_invalid;   /* the speed estimate was invalid at some ms */
  bool     angle_read_failed;  /* an angle read failed */
} LinkPeriod;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOOP_PERIOD_MS        1     /* 1 kHz encoder sampling */
#define HEALTH_PERIOD_MS      100   /* magnet health changes slowly; no need to read it every ms */
#define HEALTH_PHASE_MS       50    /* mid-window, so the extra reads never share a ms with the report's formatting */
#define REPORT_PERIOD_MS      100   /* 10 Hz telemetry line */
/* Battery voltage: one ADC1 injected sequence (VREFINT, then the divider on
 * PA6) per 10 ms. Phase 5: the marks are t % 10 == 5, so never t % 100 == 0
 * (report head, then its four follow-on parts at 1-4) or == 50 (health),
 * the iterations that already take the longest. */
#define BATTERY_PERIOD_MS     10
#define BATTERY_PHASE_MS      5
/* The heartbeat LED steps through its pattern once per HEARTBEAT_TICK_MS
 * (loop_schedule.h): 500 ms on/off = loop alive, double blink = IMU init
 * failed, 100 ms on/off = motor FAULT (mstate=3, needs a reset). */
/* Running this late means something stalled (e.g. an I2C timeout). Catching
 * up would fire several iterations back-to-back with ~0 ms between samples,
 * which is useless data, so start a fresh schedule from "now" instead. */
#define RESYNC_THRESHOLD_MS   5
/* Gyro sums stop growing at this many samples. 65535 * 32768 < 2^31, so an
 * int32 sum can't overflow even if the UART stalls and windows pile up. */
#define IMU_SUM_MAX_SAMPLES   65535u
#define BOOT_LINE_TIMEOUT_MS  50    /* boot line is ~25 bytes = ~2 ms at 115200 */
/* 0: a positive command drives IN1 high / IN2 low. Stays 0: since 2026-10-06
 * the motor leads are placed to match the firmware (red -> AO2/BO2, black ->
 * AO1/BO1), verified by driving: "m +N" turns the wheels forward and d100 is
 * positive (README, Direction). Set to 1 only if a later motor or wiring
 * change can't be wired back to that convention. */
#define MOTOR_DIRECTION_INVERTED  0
/* 0: "s +N" (left, ROS REP 103) gives a LONGER pulse. Stays 0: the bench
 * (2026-10-09, and full lock 2026-10-10) showed "s 100" turns the wheels
 * left, seen from behind (README, Steering calibration). Set to 1 only if a
 * servo or linkage change reverses that. */
#define STEERING_DIRECTION_INVERTED  0

/* TEMPORARY instrumentation. 1 = time each section of each kind of iteration
 * and send one extra "prof ..." line per window with the window maxima (text
 * mode only: in binary mode it measures but never sends; HELLO_ACK reports
 * the build). 0 = firmware exactly as without it (none of the profile code is
 * compiled). Must be committed as 0: tools/test_firmware_sources.py fails
 * otherwise. Remove together with every "#if LOOP_TIMING_PROFILE" block once
 * measured. */
#define LOOP_TIMING_PROFILE   0
#if LOOP_TIMING_PROFILE
#warning "LOOP_TIMING_PROFILE build: an extra prof line per window (never sent in binary mode). Bench only."
/* The profile line (~520 bytes with telemetry v5, ~45 ms to send) is built in
 * small steps from 6 ms after the window closed (after the telemetry line's
 * five steps; building only formats, the UART is still sending the telemetry
 * line), then sent 51-53 ms in: after the health iteration (50), so health
 * still runs with the UART idle (the telemetry line, ~500 bytes from ms 4,
 * has ended by ~47 ms), and early enough that the send ends (~98 ms) before
 * the next window closes and needs the UART. A profile line that can't be
 * sent by ms 53 is dropped (missing from the log), never delayed. */
#define PROFILE_BUILD_START_MS     6
#define PROFILE_SEND_MS            51
#define PROFILE_SEND_DEADLINE_MS   53
#endif
#if SPEED_CAPTURE
#if LOOP_TIMING_PROFILE
#error "SPEED_CAPTURE and LOOP_TIMING_PROFILE both send an extra line at ms 51 of the window: use one at a time"
#endif
/* TEMPORARY (speed_capture.h). One capture line per telemetry window, built
 * a few pieces per iteration from ms 51 of the window (after health at 50,
 * like the profile line), then sent once the telemetry line has left. Not
 * sent later than CAPTURE_SEND_DEADLINE_MS after ms 51, so it (~150 bytes,
 * ~13 ms) is out before the next window's telemetry line needs the UART; a
 * line that misses it waits for the next window (nothing is lost). */
#define CAPTURE_LINE_PHASE_MS      51
#define CAPTURE_ITEMS_PER_STEP     4u
#define CAPTURE_SEND_DEADLINE_MS   20u
#endif
#if ENCODER_FAULT_TEST && SPEED_CAPTURE
#error "ENCODER_FAULT_TEST and SPEED_CAPTURE: a capture must record the real encoder, so use one at a time"
#endif

/* Pi link (link_rx.h, link_protocol.h), binary mode only: STATE every 10 ms
 * at t % 10 == 3, STATUS every 100 ms at t % 100 == 8. Neither ever lands on
 * health (50), a battery sample (t % 10 == 5) or the other one. STATE's
 * t % 100 == 3 is a text line step only in TEXT mode, where STATE isn't
 * sent (tools/test_firmware_sources.py checks all of it). */
#define LINK_STATE_PERIOD_MS    10
#define LINK_STATE_PHASE_MS     3
#define LINK_STATUS_PERIOD_MS   100
#define LINK_STATUS_PHASE_MS    8
/* The STATE field is a u8; 255 x 32768 can't overflow an int32 sum. */
#define LINK_STATE_MAX_SAMPLES  255u
/* Text commands over USB (m, v, s) in TEXT mode, before any HELLO. 0 = the
 * truck build: TEXT mode only waits for a HELLO, and every text line is
 * counted in cmd_bad and never acted on (the v5 telemetry line still goes
 * out). 1 = bench build: m, v and s work in TEXT mode as before the Pi link.
 * Bench driving without it goes through motor_console.py --binary. Test
 * builds (LINK_TEST_BUILD) enable text commands by themselves. Must be
 * committed as 0: tools/test_firmware_sources.py fails otherwise. */
#define BENCH_TEXT_COMMANDS   0
#if BENCH_TEXT_COMMANDS
#warning "BENCH_TEXT_COMMANDS build: m, v and s accepted over USB before a HELLO. Not for the truck."
#endif
/* Bench-only builds. They keep text commands in TEXT mode (the capture fitter
 * needs m steps, the profiler v under load) and their own letters (e, c, h)
 * in binary mode, so the tests can run against the Pi link; m, v and s never
 * work in binary mode (LinkRx_TextCommandAllowed). */
#define LINK_TEST_BUILD  ((IWDG_HANG_TEST != 0) || (SPEED_CAPTURE != 0) || (ENCODER_FAULT_TEST != 0) || \
                          (LOOP_TIMING_PROFILE != 0))
/* HELLO_ACK's build_flags: the Pi must never drive a build with any bit set. */
#define LINK_BUILD_FLAGS  (((IWDG_HANG_TEST != 0) ? LINK_BUILD_IWDG_HANG_TEST : 0u) |           \
                           ((SPEED_CAPTURE != 0) ? LINK_BUILD_SPEED_CAPTURE : 0u) |             \
                           ((ENCODER_FAULT_TEST != 0) ? LINK_BUILD_ENCODER_FAULT_TEST : 0u) |   \
                           ((LOOP_TIMING_PROFILE != 0) ? LINK_BUILD_LOOP_TIMING_PROFILE : 0u) | \
                           ((BENCH_TEXT_COMMANDS != 0) ? LINK_BUILD_BENCH_TEXT_COMMANDS : 0u))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* File scope, not a local: HAL_UART_Transmit_IT returns immediately and the
 * UART interrupt keeps reading this buffer for ~35 ms afterwards.
 * Worst-case line (every field at its widest) is 651 bytes: 420 for v1,
 * +128 for the motor fields (v2), +70 for rst and the steering fields (v3),
 * +33 for vbat (5 digits), bstate (1) and vdda (7: it is kept even when
 * implausible, up to 3000 x 1700 / 1) (v4), +135 for the speed control
 * fields counted at their full 32-bit width (v5): 786 bytes, ~68 ms at
 * 115200 baud. A bound that needs no knowledge of the values: the line with
 * EVERY field at its full 32-bit width is 956 bytes, which still fits
 * (tools/test_firmware_sources.py recomputes both from the field list). */
static char telemetry_line[1024];

/* ---- Shared between the USART2 interrupt and the main loop --------------
 * These are the firmware's own variables that an interrupt writes and the
 * main loop reads. Rules (see byte_ring.h for the details):
 *   - every one has exactly ONE writer, so no read-modify-write races and no
 *     need to disable interrupts
 *   - all are volatile, so the optimizer (-Og) can't cache them in registers
 *   - 32-bit aligned, so a read can never see half of an update
 * Received bytes: the interrupt pushes, the main loop pops. */
static ByteRing uart_rx_ring;
/* UART framing/noise/overrun errors. Written only by HAL_UART_ErrorCallback. */
static volatile uint32_t uart_rx_error_count;
/* HAL writes each received byte here, and HAL_UART_RxCpltCallback copies it
 * into the ring. Both happen inside the interrupt, so this one is not shared
 * with the main loop. */
static uint8_t uart_rx_byte;

#if LOOP_TIMING_PROFILE
/* Two windows: one collects while the other (the window that just closed) is
 * formatted. Swapping an index costs nothing at window close, where copying
 * ~200 bytes would add time to the very iteration being measured. Static for
 * the same reason as telemetry_line: the UART interrupt reads profile_line. */
static ProfileWindow profile_windows[2];
static char profile_line[768];
#endif

#if SPEED_CAPTURE
/* TEMPORARY (speed_capture.h): 12 KB of samples, static so it isn't on the
 * stack; the line is static for the same reason as telemetry_line (the
 * UART interrupt reads it after HAL_UART_Transmit_IT returns). */
static SpeedCaptureSample speed_capture_samples[SPEED_CAPTURE_SAMPLES];
static SpeedCapture speed_capture;
static char capture_line[320];
#endif

#if ENCODER_FAULT_TEST
/* TEST BUILD ONLY (encoder_fault.h): the "e" command's fault, file scope like
 * speed_capture so the command handler and the loop both reach it. */
static EncoderFault encoder_fault;
#endif

/* ---- Pi link ----------------------------------------------------------------
 * link_rx: the receive side, main loop only (Commands_ProcessReceived).
 * link_tx_queue: frames waiting to be sent, shared with the USART2
 * transmit-complete interrupt (tx_queue.h: the main loop pushes, the
 * interrupt pops). link_tx_queue_sending: a queued frame is on the wire, so
 * the interrupt pops only a frame it sent (the text, profile and capture
 * lines end in the same interrupt). It has two writers, but the main loop
 * writes it only with interrupts masked (LinkTx_Kick), so the two can never
 * interleave. The STATE and STATUS messages are filled in place (static, so
 * neither sits in main()'s frame for good). */
static LinkRx link_rx;
static TxQueue link_tx_queue;
static volatile bool link_tx_queue_sending;
static LinkStateMessage link_state_message;
static LinkStatusMessage link_status_message;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void CycleCounter_Enable(void);
static HAL_StatusTypeDef CommandUart_StartReceive(void);
static void CommandUart_KeepReceiving(uint32_t *rx_restart_count);
static uint32_t Commands_ProcessReceived(CommandLine *command_line, MotorController *motor,
                                         SpeedController *speed, SteeringController *steering,
                                         uint32_t *rx_trouble_count_seen, bool *hello_received, uint32_t now_ms);
static void LinkTx_Kick(void);
static void LinkTx_Send(uint8_t type, uint8_t *seq, const uint8_t *payload, size_t length);
static void LinkTx_SendHelloAck(uint32_t reset_flags, bool imu_ok, uint32_t imu_who, uint8_t *seq);
static void LinkTx_SendState(const LinkStateMessage *message, uint8_t *seq);
static void LinkTx_SendStatus(const LinkStatusMessage *message, uint8_t *seq);
static bool BatteryAdc_Start(void);
static MotorFaultReason BatteryAdc_MotorFaultReason(const BatteryMonitor *battery);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* DWT->CYCCNT counts CPU clock cycles (31.25 ns each at 32 MHz). It's the
 * finest-grained timer on the chip and costs nothing to read. It's off after
 * reset, so it has to be switched on once. */
static void CycleCounter_Enable(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* power up the trace/debug block DWT lives in */
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ---- Motor and steering commands over the VCP ------------------------------
 * USART2 has no receive FIFO on the L432: one byte waits in the data register
 * while the next one shifts in. At 115200 baud a byte arrives every 86.8 us,
 * so polling once per 1 ms loop would lose bytes. Instead each byte is
 * received by interrupt (HAL_UART_Receive_IT, one byte at a time), pushed
 * into uart_rx_ring by HAL_UART_RxCpltCallback, and the main loop drains the
 * ring once per iteration. Receiving uses huart2.RxState and transmitting
 * uses huart2.gState, so the telemetry send is unaffected. */

/* Arms the interrupt for the next received byte. */
static HAL_StatusTypeDef CommandUart_StartReceive(void)
{
  return HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1u);
}

/* Normally a no-op: the interrupt re-arms reception after every byte and
 * after every error. If that ever failed, reception would stay stopped (the
 * watchdog would keep the motor safe, but no command could get through), so
 * restart it here. Interrupts are masked for the check and the restart
 * (~2 us), so the interrupt can't start reception in between. */
static void CommandUart_KeepReceiving(uint32_t *rx_restart_count)
{
  if (huart2.RxState != HAL_UART_STATE_READY)
  {
    return;  /* receiving: the normal case */
  }
  uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  if (huart2.RxState == HAL_UART_STATE_READY && CommandUart_StartReceive() == HAL_OK)
  {
    (*rx_restart_count)++;
  }
  __set_PRIMASK(interrupt_mask);
}

/* Feeds every byte received since the last iteration to the Pi link first
 * (link_rx.c): frames stay there and are acted on there (HELLO, DRIVE,
 * ESTOP), text comes back. The text goes to the command line parser, and
 * each finished line is first checked by LinkRx_TextCommandAllowed: in TEXT
 * mode it acts only in a bench (BENCH_TEXT_COMMANDS) or test build, else it
 * is counted in cmd_bad; in BINARY mode only e, c and h, in a test build.
 * A line that may act goes to command_dispatch.c, which hands "m" to the
 * motor, "v" to the speed controller (and the motor's watchdog) and "s" to
 * the steering (and nothing else to any of them), exactly as before the link
 * existed. *hello_received is set when a HELLO was acted on (the caller
 * answers it). Returns how long to stall the loop: always 0 unless this is
 * an IWDG_HANG_TEST build and an "h" line arrived. */
static uint32_t Commands_ProcessReceived(CommandLine *command_line, MotorController *motor,
                                         SpeedController *speed, SteeringController *steering,
                                         uint32_t *rx_trouble_count_seen, bool *hello_received, uint32_t now_ms)
{
  uint32_t hang_ms = 0u;
  uint8_t received_byte;
  const LinkRxTargets link_targets = { .motor = motor, .speed = speed, .steering = steering };
  while (ByteRing_Pop(&uart_rx_ring, &received_byte))
  {
    /* Checked after each pop: the interrupt that delivered a corrupted byte
     * also counted the error before the main loop could run again, so the
     * count already shows it, and the line being assembled right now is
     * rejected instead of being acted on.
     * Known limitation: that is the damaged byte's own line only if no '\n'
     * was queued between the oldest unread byte and the damaged (or dropped)
     * one. With several bytes queued across a line end, the WRONG line is
     * rejected and the damaged one is parsed: worst case "m -20" losing its
     * '-' is accepted as "m 20". Bounded by the cap, ramp and reverse dwell,
     * and unlikely: the ring is drained every 1 ms and keepalives are ~6
     * bytes per 100 ms. Planned fix (README, open items): an in-band marker
     * pushed by the ISR at the error position, or a checksum in the Pi
     * protocol. The same applies to a corrupted letter that the UART does
     * not flag ("s 30" read as "m 30"; README, open items). */
    uint32_t rx_trouble_count = uart_rx_error_count + uart_rx_ring.dropped_count;
    if (rx_trouble_count != *rx_trouble_count_seen)
    {
      *rx_trouble_count_seen = rx_trouble_count;
      /* Inside a frame the frame is dropped at its end; outside, the text
       * line is (as before the link existed). */
      if (!LinkRx_Poison(&link_rx))
      {
        CommandLine_DiscardCurrent(command_line);
      }
    }

    LinkRxEvent link_event = LinkRx_FeedByte(&link_rx, received_byte, &link_targets, now_ms);
    if (link_event == LINK_RX_FRAME_START)
    {
      /* A 0x00 never belongs to a text line: drop the partial one before it
       * (in a test build's binary mode too, where text still reaches it). */
      CommandLine_Init(command_line);
      continue;
    }
    if (link_event == LINK_RX_HELLO)
    {
      *hello_received = true;
      continue;
    }
    if (link_event != LINK_RX_TEXT_BYTE)
    {
      continue;  /* part of a frame, or a frame link_rx acted on or dropped */
    }

    Command command = { .letter = 0, .value = 0 };
    CommandLineResult line_result = CommandLine_FeedByte(command_line, received_byte, &command);
    if (line_result == COMMAND_LINE_INCOMPLETE || line_result == COMMAND_LINE_EMPTY)
    {
      continue;
    }
    /* A finished line: may it act at all? (link_rx.c decides: TEXT mode only
     * in a bench or test build, BINARY mode only e, c, h in a test build.) */
    char letter = (line_result == COMMAND_LINE_VALID) ? command.letter : '\0';
    if (!LinkRx_TextCommandAllowed(link_rx.mode, letter, (BENCH_TEXT_COMMANDS != 0), (LINK_TEST_BUILD != 0)))
    {
      if (link_rx.mode == LINK_MODE_TEXT)
      {
        MotorController_CountRejected(motor);  /* truck build: counted in cmd_bad, never acted on */
      }
      continue;  /* binary mode: already counted by link_rx (rx_text_ignored) */
    }
#if SPEED_CAPTURE
    /* TEMPORARY: "c" exists only in a capture build (speed_capture.h). It
     * feeds no watchdog and moves nothing; a refused value counts as a
     * rejected line, like any other. */
    if (line_result == COMMAND_LINE_VALID && command.letter == 'c')
    {
      if (!SpeedCapture_Command(&speed_capture, command.value))
      {
        MotorController_CountRejected(motor);
      }
      continue;
    }
#endif
#if ENCODER_FAULT_TEST
    /* TEST BUILD ONLY: "e" exists only in an encoder fault test build
     * (encoder_fault.h). Like "c": feeds no watchdog, moves nothing by
     * itself; a refused value counts as a rejected line. */
    if (line_result == COMMAND_LINE_VALID && command.letter == 'e')
    {
      if (!EncoderFault_Command(&encoder_fault, command.value, now_ms))
      {
        MotorController_CountRejected(motor);
      }
      continue;
    }
#endif
    CommandDispatchResult dispatched = CommandDispatch_LineWithSpeed(line_result, &command, motor, steering, speed,
                                                                     (IWDG_HANG_TEST != 0), now_ms);
    hang_ms += dispatched.hang_ms;
  }
  return hang_ms;
}

/* ---- Pi link: sending frames ----------------------------------------------
 * Frames go through link_tx_queue: at 115200 baud a STATUS frame (124 bytes,
 * ~10.8 ms) outlasts the 10 ms between two STATE frames. The main loop
 * pushes and starts the first one (LinkTx_Kick); the transmit-complete
 * interrupt (HAL_UART_TxCpltCallback) pops it and starts the next. Nothing
 * here ever waits for the UART. */

/* Starts the oldest queued frame if the UART is idle and no queued frame is
 * on its way. Interrupts are masked for the check and the start (a few us),
 * so the transmit-complete interrupt can't start one in between. */
static void LinkTx_Kick(void)
{
  uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  if (!link_tx_queue_sending && huart2.gState == HAL_UART_STATE_READY)
  {
    size_t length = 0u;
    const volatile uint8_t *frame = TxQueue_Front(&link_tx_queue, &length);
    if (frame != NULL &&
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)(uintptr_t)frame, (uint16_t)length) == HAL_OK)
    {
      link_tx_queue_sending = true;
    }
  }
  __set_PRIMASK(interrupt_mask);
}

/* Frames one payload, queues it (a full queue counts in dropped_count) and
 * makes sure sending is under way. *seq counts this type's frames. */
static void LinkTx_Send(uint8_t type, uint8_t *seq, const uint8_t *payload, size_t length)
{
  uint8_t wire[LINK_MAX_WIRE];
  size_t wire_length = LinkFrame_Encode(type, *seq, payload, length, wire, sizeof(wire));
  (void)TxQueue_Push(&link_tx_queue, wire, wire_length);  /* 0 bytes (can't happen) is refused and counted too */
  *seq = (uint8_t)(*seq + 1u);
  LinkTx_Kick();
}

static void LinkTx_SendHelloAck(uint32_t reset_flags, bool imu_ok, uint32_t imu_who, uint8_t *seq)
{
  LinkHelloAckMessage message = {
    .build_flags = LINK_BUILD_FLAGS, .reset_flags = reset_flags, .imu_ok = imu_ok, .imu_who = imu_who,
    .state_period_ms = LINK_STATE_PERIOD_MS, .status_period_ms = LINK_STATUS_PERIOD_MS };
  uint8_t payload[LINK_HELLO_ACK_LENGTH];
  LinkTx_Send(LINK_MSG_HELLO_ACK, seq, payload, LinkProtocol_PackHelloAck(&message, payload));
}

static void LinkTx_SendState(const LinkStateMessage *message, uint8_t *seq)
{
  uint8_t payload[LINK_STATE_LENGTH];
  LinkTx_Send(LINK_MSG_STATE, seq, payload, LinkProtocol_PackState(message, payload));
}

static void LinkTx_SendStatus(const LinkStatusMessage *message, uint8_t *seq)
{
  uint8_t payload[LINK_STATUS_LENGTH];
  LinkTx_Send(LINK_MSG_STATUS, seq, payload, LinkProtocol_PackStatus(message, payload));
}

#if LOOP_TIMING_PROFILE
/* ---- TEMPORARY: loop timing profile ------------------------------------ */

static void Profile_KeepMax(uint32_t *maximum, uint32_t value)
{
  if (value > *maximum) { *maximum = value; }
}

/* Adds one iteration's section times (cycles) to its kind's window maxima.
 * ctl_cycles is part of motor_cycles and est_cycles part of encoder_cycles,
 * so neither counts towards "other". */
static void Profile_Record(ProfileSectionMaxima *maxima, uint32_t motor_cycles, uint32_t ctl_cycles,
                           uint32_t encoder_cycles, uint32_t est_cycles, uint32_t health_reads_cycles,
                           uint32_t imu_cycles, uint32_t format_cycles, uint32_t uart_start_cycles,
                           uint32_t total_cycles)
{
  uint32_t named_cycles = motor_cycles + encoder_cycles + health_reads_cycles + imu_cycles + format_cycles +
                          uart_start_cycles;
  uint32_t other_cycles = (total_cycles > named_cycles) ? (total_cycles - named_cycles) : 0u;

  maxima->iteration_count++;
  Profile_KeepMax(&maxima->motor_cycles_max, motor_cycles);
  Profile_KeepMax(&maxima->ctl_cycles_max, ctl_cycles);
  Profile_KeepMax(&maxima->encoder_cycles_max, encoder_cycles);
  Profile_KeepMax(&maxima->est_cycles_max, est_cycles);
  Profile_KeepMax(&maxima->health_reads_cycles_max, health_reads_cycles);
  Profile_KeepMax(&maxima->imu_cycles_max, imu_cycles);
  Profile_KeepMax(&maxima->format_cycles_max, format_cycles);
  Profile_KeepMax(&maxima->uart_start_cycles_max, uart_start_cycles);
  Profile_KeepMax(&maxima->other_cycles_max, other_cycles);
  Profile_KeepMax(&maxima->total_cycles_max, total_cycles);
}

/* One step of the profile line; one group of fields per loop iteration, so
 * no single iteration does much extra formatting. Returns the next step, or
 * 0 once the line is complete (including "\r\n"). Values are in us, divided
 * the same way as the telemetry line's *_us_max fields. */
static int Profile_BuildLineStep(TelemetryWriter *writer, int step, const ProfileWindow *window,
                                 uint32_t window_t_ms, uint32_t cycles_per_us)
{
  const ProfileSectionMaxima *idle   = &window->kinds[PROFILE_NORMAL_UART_IDLE];
  const ProfileSectionMaxima *busy   = &window->kinds[PROFILE_NORMAL_UART_BUSY];
  const ProfileSectionMaxima *health = &window->kinds[PROFILE_HEALTH];
  const ProfileSectionMaxima *head   = &window->kinds[PROFILE_REPORT_HEAD];
  const ProfileSectionMaxima *tail   = &window->kinds[PROFILE_REPORT_TAIL];
  const ProfileSectionMaxima *motor  = &window->kinds[PROFILE_REPORT_MOTOR];
  const ProfileSectionMaxima *steer  = &window->kinds[PROFILE_REPORT_STEERING];
  const ProfileSectionMaxima *speed  = &window->kinds[PROFILE_REPORT_SPEED];
  const ProfileSectionMaxima *build  = &window->kinds[PROFILE_LINE_BUILD];

  switch (step)
  {
    case 1:
      TelemetryWriter_AppendText(writer, "prof");
      TelemetryWriter_AppendUnsignedField(writer, "t_ms",     window_t_ms);  /* = the telemetry line's t_ms for this window */
      TelemetryWriter_AppendUnsignedField(writer, "idle_n",   idle->iteration_count);
      TelemetryWriter_AppendUnsignedField(writer, "idle_enc", idle->encoder_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "idle_imu", idle->imu_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "idle_oth", idle->other_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "idle_tot", idle->total_cycles_max / cycles_per_us);
      return 2;
    case 2:
      TelemetryWriter_AppendUnsignedField(writer, "busy_n",   busy->iteration_count);
      TelemetryWriter_AppendUnsignedField(writer, "busy_enc", busy->encoder_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "busy_imu", busy->imu_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "busy_oth", busy->other_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "busy_tot", busy->total_cycles_max / cycles_per_us);
      return 3;
    case 3:
      TelemetryWriter_AppendUnsignedField(writer, "hlth_enc", health->encoder_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "hlth_rd",  health->health_reads_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "hlth_imu", health->imu_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "hlth_oth", health->other_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "hlth_tot", health->total_cycles_max / cycles_per_us);
      return 4;
    case 4:
      TelemetryWriter_AppendUnsignedField(writer, "head_enc", head->encoder_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "head_imu", head->imu_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "head_fmt", head->format_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "head_oth", head->other_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "head_tot", head->total_cycles_max / cycles_per_us);
      return 5;
    case 5:
      TelemetryWriter_AppendUnsignedField(writer, "tail_enc", tail->encoder_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "tail_imu", tail->imu_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "tail_fmt", tail->format_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "tail_oth", tail->other_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "tail_tot", tail->total_cycles_max / cycles_per_us);
      return 6;
    case 6:
      TelemetryWriter_AppendUnsignedField(writer, "motr_enc", motor->encoder_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "motr_imu", motor->imu_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "motr_fmt", motor->format_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "motr_oth", motor->other_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "motr_tot", motor->total_cycles_max / cycles_per_us);
      return 7;
    case 7:
      TelemetryWriter_AppendUnsignedField(writer, "strg_enc", steer->encoder_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "strg_imu", steer->imu_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "strg_fmt", steer->format_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "strg_oth", steer->other_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "strg_tot", steer->total_cycles_max / cycles_per_us);
      return 8;
    case 8:
      TelemetryWriter_AppendUnsignedField(writer, "spd_enc",  speed->encoder_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "spd_imu",  speed->imu_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "spd_fmt",  speed->format_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "spd_tx",   speed->uart_start_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "spd_oth",  speed->other_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "spd_tot",  speed->total_cycles_max / cycles_per_us);
      return 9;
    default:
    {
      /* The motor section (command input + speed control + motor and
       * steering state machines + pin writes), and in it the speed control
       * (ctl), and the estimate in the encoder section (est), run in every
       * iteration: report their largest times over all kinds. */
      uint32_t motor_section_cycles_max = 0u;
      uint32_t ctl_section_cycles_max = 0u;
      uint32_t est_section_cycles_max = 0u;
      for (int kind = 0; kind < PROFILE_KIND_COUNT; kind++)
      {
        Profile_KeepMax(&motor_section_cycles_max, window->kinds[kind].motor_cycles_max);
        Profile_KeepMax(&ctl_section_cycles_max, window->kinds[kind].ctl_cycles_max);
        Profile_KeepMax(&est_section_cycles_max, window->kinds[kind].est_cycles_max);
      }
      TelemetryWriter_AppendUnsignedField(writer, "mot",      motor_section_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "ctl_us",   ctl_section_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "est_us",   est_section_cycles_max / cycles_per_us);
      TelemetryWriter_AppendUnsignedField(writer, "bld_tot",  build->total_cycles_max / cycles_per_us);
      TelemetryWriter_AppendText(writer, "\r\n");
      return 0;
    }
  }
}
#endif /* LOOP_TIMING_PROFILE */

/* ---- Battery voltage on ADC1 (injected group) ------------------------------
 * MX_ADC1_Init (CubeMX) set up two injected ranks, VREFINT then IN11 (PA6),
 * software trigger, 640.5 cycles each: ~41 us per sequence at 32 MHz. Boot
 * only (blocking is fine here, the loop never waits on the ADC):
 *   1. self-calibration, single-ended (needs the ADC disabled, as it is
 *      after MX_ADC1_Init; HAL_ADCEx_Calibration_Start waits for ADCAL to
 *      clear, ~116 ADC clocks = ~4 us);
 *   2. enable + start (HAL_ADCEx_InjectedStart: waits for ADRDY);
 *   3. that first sequence is waited for and thrown away: the filter is
 *      seeded with its first sample, and a VREFINT read before the
 *      reference has settled would give an implausible VDDA, i.e. a latched
 *      ADC fault. Waited for on JEOS (end of the whole sequence), not with
 *      HAL_ADCEx_InjectedPollForConversion: with EOCSelection =
 *      ADC_EOC_SINGLE_CONV (MX_ADC1_Init) that returns at JEOC, after rank
 *      1, while rank 2 is still converting;
 *   4. the next sequence is started; the loop collects it when it is due.
 * About 50 us in all (estimate, not measured). False on any HAL error or
 * if the sequence doesn't end within 2 ms. */
static bool BatteryAdc_Start(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) { return false; }
  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) { return false; }
  uint32_t wait_start_ms = HAL_GetTick();
  while (!LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    if (HAL_GetTick() - wait_start_ms > 2u) { return false; }
  }
  LL_ADC_ClearFlag_JEOC(ADC1);
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_INJ_StartConversion(ADC1);
  return true;
}

/* Why the battery monitor locks the motor out, as a motor FAULT reason
 * (MOTOR_FAULT_NONE if it doesn't). Same precedence as the battery state. */
static MotorFaultReason BatteryAdc_MotorFaultReason(const BatteryMonitor *battery)
{
  switch (battery->state)
  {
    case BATTERY_STATE_ADC_FAULT:             return MOTOR_FAULT_BATTERY_ADC;
    case BATTERY_STATE_CUTOFF:                return MOTOR_FAULT_LOW_BATTERY;
    case BATTERY_STATE_REMOVED_WHILE_DRIVING: return MOTOR_FAULT_BATTERY_REMOVED_WHILE_DRIVING;
    default:                                  return MOTOR_FAULT_NONE;
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
  /* Two unwrappers of the same reads: wheel_encoder follows the raw angle
   * (the capture build records it, and "raw" is that angle); corrected_encoder
   * follows the angle with the 4-per-revolution sensor error removed
   * (angle_correction.h). "count", "d100" and the speed estimate use the
   * corrected one. */
  AS5600_Unwrapper wheel_encoder;
  AS5600_Unwrapper corrected_encoder;
  AS5600_Status magnet_status = {0};
  uint8_t agc_value = 0;
  uint16_t latest_raw_angle = 0;
  int32_t count_at_last_report = 0;

  /* Per print window: reset each time a line is actually sent */
  uint16_t magnitude_min = UINT16_MAX;
  uint16_t magnitude_max = 0;
  uint32_t i2c_cycles_max = 0;
  uint32_t loop_cycles_max = 0;

  /* Totals since boot: never reset, so a single glitch is never hidden */
  uint32_t i2c_error_count = 0;
  uint32_t overrun_count = 0;
  uint32_t resync_count = 0;
  uint32_t skipped_report_count = 0;

  /* IMU on SPI1. imu_ok is the init result and never changes afterwards:
   * if init failed, the loop skips the IMU and the encoder runs as before. */
  ICM42688_Device imu = { .spi_handle = &hspi1, .cs_port = IMU_CS_GPIO_Port, .cs_pin = IMU_CS_Pin };
  bool imu_ok = false;
  ICM42688_RawSample latest_imu_sample = {0};

  /* IMU per print window: reset with the other window fields */
  int32_t gyro_sum_x = 0;
  int32_t gyro_sum_y = 0;
  int32_t gyro_sum_z = 0;
  uint32_t imu_sample_count = 0;
  uint32_t spi_cycles_max = 0;

  /* IMU total since boot */
  uint32_t imu_error_count = 0;

  /* Motor: TB6612FNG on TIM1_CH1 (PA8) + IN1/IN2/STBY, commanded over the VCP. */
  TB6612_Device motor_driver = {
    .pwm_timer = &htim1, .pwm_channel = TIM_CHANNEL_1,
    .in1_port = MOTOR_IN1_GPIO_Port, .in1_pin = MOTOR_IN1_Pin,
    .in2_port = MOTOR_IN2_GPIO_Port, .in2_pin = MOTOR_IN2_Pin,
    .standby_port = MOTOR_STBY_GPIO_Port, .standby_pin = MOTOR_STBY_Pin,
    .invert_direction = (MOTOR_DIRECTION_INVERTED != 0) };
  MotorController motor_controller;
  CommandLine command_line;
  uint32_t rx_trouble_count_seen = 0;  /* uart_rx_error_count + dropped bytes, as last seen */
  uint32_t rx_restart_count = 0;       /* receptions restarted by the main loop (normally 0) */
  bool pwm_period_ok = false;          /* TIM1 ARR + 1 == MOTOR_PWM_PERIOD_TICKS, checked at boot */

  /* Steering servo: TIM15_CH2 (PA3), commanded over the VCP ("s"). */
  Servo_Device servo = { .pwm_timer = &htim15, .pwm_channel = TIM_CHANNEL_2 };
  SteeringController steering_controller;
  bool servo_timer_ok = false;         /* TIM15 gives 1 us ticks and a 20 ms period, checked at boot */

  /* Battery voltage: ADC1 injected group (VREFINT + PA6), every 10 ms. */
  BatteryMonitor battery_monitor;

  /* Closed-loop wheel speed ("v"): the estimate from the corrected count,
   * the PI (its own 10 ms due-time schedule) and the stall check, every ms. */
  SpeedEstimator speed_estimator;
  SpeedController speed_controller;
  StallDetector stall_detector;

  /* RCC_CSR reset flags (bits 24..31) as found at boot, shifted down to
   * bits 0..7: 0 FW, 1 OBL, 2 PIN, 3 BOR, 4 SFT, 5 IWDG, 6 WWDG, 7 LPWR
   * (stm32l432xx.h, RCC_CSR_*RSTF). Reported as "rst" in every line. */
  uint32_t reset_cause_flags = 0;

  /* A line is formatted in five iterations (see the telemetry section). The
   * writer (which knows how much of the line is written) and the IMU, motor,
   * steering, battery and speed snapshots carry over to the later ones. */
  TelemetryLineStep line_step = LINE_IDLE;
  TelemetryWriter line_writer;
  ImuReport imu_report = {0};
  MotorReport motor_report = {0};
  SteeringReport steering_report = {0};
  BatteryReport battery_report = {0};
  SpeedReport speed_report = {0};

  uint32_t next_deadline_ms;
  uint32_t cycles_per_us;

  /* Periodic tasks, due by time rather than by "t % period == phase", so a
   * loop whose iterations take much longer than 1 ms (a stuck I2C bus: ~52
   * ms each) still reports and blinks about every period instead of never
   * (loop_schedule.h). Normally they fire on exactly the same ms as before. */
  PeriodicSchedule report_schedule;
  PeriodicSchedule health_schedule;
  PeriodicSchedule heartbeat_schedule;
  PeriodicSchedule battery_schedule;
  uint32_t heartbeat_tick = 0;   /* heartbeat due times since boot: position in the LED pattern */

  /* Pi link: STATE and STATUS (binary mode only), the current STATE period
   * (restarted at every STATE mark, in either mode), the odometer epoch (+1
   * at every valid -> invalid transition of the speed estimate: the Pi
   * treats a change as "distance uncertain"), and each frame type's seq. */
  PeriodicSchedule link_state_schedule;
  PeriodicSchedule link_status_schedule;
  LinkPeriod link_period = {0};
  uint32_t link_odometer_epoch = 0;
  bool link_estimate_was_valid = false;
  uint8_t link_ack_seq = 0;
  uint8_t link_state_seq = 0;
  uint8_t link_status_seq = 0;
#if SPEED_CAPTURE
  /* TEMPORARY (speed_capture.h): one capture line per window, built from ms 51. */
  PeriodicSchedule capture_schedule;
  TelemetryWriter capture_writer;
  bool capture_line_building = false;
  bool capture_line_ready = false;
  uint32_t capture_window_ms = 0;
#endif

#if LOOP_TIMING_PROFILE
  int profile_collecting_index = 0;   /* profile_windows[] entry that collects right now */
  int profile_build_step = 0;         /* 0 = no profile line being built, else next step */
  bool profile_line_ready = false;    /* built, waiting for its send slot (ms 51-53) */
  uint32_t profile_window_t_ms = 0;   /* t_ms of the window being reported */
  TelemetryWriter profile_writer;
#endif
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Why the MCU restarted. The flags stay set across resets until cleared
   * (RMVF), so read them once and clear them: the next boot then shows only
   * its own cause. Nothing before this point touches RCC_CSR. */
  reset_cause_flags = (RCC->CSR >> RCC_CSR_FWRSTF_Pos) & 0xFFu;
  SET_BIT(RCC->CSR, RCC_CSR_RMVF);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
 
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* Hardware watchdog, started first so it also covers everything slow at
   * boot (IMU init ~35 ms, boot line ~2 ms; ~40 ms to the first refresh).
   * 250 ms (LSI 32 kHz / 32, reload 249; 235-271 ms with the LSI's spread).
   * Once started it can't be stopped (stm32l4xx_hal_iwdg.c, "IWDG Generic
   * features"), so every path from here must refresh it in time: the only
   * refresh is the last statement of the main loop. Frozen at a debugger
   * halt (DBG_IWDG_STOP), set before the start so a breakpoint can't reset
   * the board. MX_IWDG_Init is generated by CubeMX with its call disabled,
   * so it runs here, after the freeze bit. */
  __HAL_DBGMCU_FREEZE_IWDG();
  MX_IWDG_Init();

  CycleCounter_Enable();
  cycles_per_us = SystemCoreClock / 1000000u;  /* read at runtime so a clock change can't silently break the µs math */
  AS5600_UnwrapperInit(&wheel_encoder);
  AS5600_UnwrapperInit(&corrected_encoder);

  /* The duty cap is in timer ticks (MOTOR_DUTY_CAP_TICKS), so it means 30 %
   * only if the timer period is MOTOR_PWM_PERIOD_TICKS. If ARR is ever
   * changed in CubeMX without that constant, the motor is never enabled
   * (FAULT, set below once the controller exists); encoder, IMU and
   * telemetry run as usual. ARR preload is on, so this reads the preload
   * register, which HAL_TIM_PWM_Init already copied to the active one. */
  pwm_period_ok = (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1u == (uint32_t)MOTOR_PWM_PERIOD_TICKS);

  /* Motor first, before anything slow: STBY low, IN1 = IN2 = low, duty 0,
   * then PWM on. Until here PA8 has floated (timer output not enabled yet),
   * held low by the TB6612's input pull-down. */
  if (TB6612_Start(&motor_driver) != HAL_OK)
  {
    Error_Handler();
  }
  /* At a debugger halt the CPU stops but TIM1 would keep making PWM, and the
   * software command watchdog can't run, so the motor would keep turning.
   * With this bit set TIM1 stops too, and its output is disabled as if MOE
   * were cleared: PA8 floats and the TB6612's pull-down makes PWM low (short
   * brake). This is a second layer only: the rule is still "no debug session
   * with the LiPo plugged in" (see the README). */
  __HAL_DBGMCU_FREEZE_TIM1();

  /* Servo: compare 0, then PWM on, so PA3 goes from undriven straight to a
   * driven low line with no pulses until the first "s" command. Frozen at
   * a debugger halt like TIM1 (output disabled, no pulses): every actuator
   * is passive at a breakpoint. */
  if (Servo_Start(&servo) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DBGMCU_FREEZE_TIM15();
  /* The pulse width is in timer ticks, so it means us only with a 1 MHz tick
   * and a 20 ms period. With anything else the steering is never driven
   * (FAULT, set below once the controller exists). Pulse 0 writes nothing
   * new: compare stays 0. */
  servo_timer_ok = !Servo_Apply(&servo, 0u).timer_refused;

  ByteRing_Init(&uart_rx_ring);
  CommandLine_Init(&command_line);
  LinkRx_Init(&link_rx);              /* TEXT mode until a HELLO */
  TxQueue_Init(&link_tx_queue);       /* before anything can end in the transmit-complete interrupt */
  link_tx_queue_sending = false;

  /* ~40 ms, mostly waiting for the gyro to start. Done before the schedule
   * starts so it doesn't show up as a resync. */
  uint8_t imu_who_am_i = 0;
  ICM42688_InitResult imu_init_result = ICM42688_Init(&imu, &imu_who_am_i);
  imu_ok = (imu_init_result == ICM42688_INIT_OK);

  /* One line at boot, blocking: nothing else is using the UART yet, and it
   * must be out before the first telemetry line reuses the buffer. */
  int boot_line_length = snprintf(telemetry_line, sizeof(telemetry_line), "imu_init=%s who=0x%02X\r\n",
                                  ICM42688_InitResultName(imu_init_result), (unsigned)imu_who_am_i);
  if (boot_line_length > 0 && boot_line_length < (int)sizeof(telemetry_line))
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)telemetry_line, (uint16_t)boot_line_length,
                            BOOT_LINE_TIMEOUT_MS);
  }

  /* Start receiving commands. Bytes that came in since reset (for example
   * keepalives from a console that was already running) left an overrun
   * flag and a stale byte; clear and flush them so they don't count as an
   * rx_err. The motor is latched anyway until an "m 0" arrives. */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
  __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
  if (CommandUart_StartReceive() != HAL_OK)
  {
    Error_Handler();
  }

  /* Battery monitor. VREFINT_CAL is the factory VREFINT reading at 3.0 V,
   * read from its fixed address (VREFINT_CAL_ADDR, 0x1FFF75AA,
   * stm32l4xx_ll_adc.h; the same word __LL_ADC_CALC_VREFANALOG_VOLTAGE
   * uses). A word outside the datasheet window is an ADC fault at once, and
   * so is an ADC that doesn't calibrate or start: either way the motor is
   * locked out (in the loop, below), and steering and telemetry run on.
   * Late in the boot, ~40 ms after MX_ADC1_Init switched the VREFINT path
   * on, so the reference has long settled. */
  BatteryMonitor_Init(&battery_monitor, *VREFINT_CAL_ADDR);
  if (!BatteryAdc_Start())
  {
    BatteryMonitor_Update(&battery_monitor, false, 0u, 0u, false, HAL_GetTick());
  }

  /* Seed the schedule right after a tick edge. Seeding at an arbitrary point
   * inside a ms gives a first deadline that may be only microseconds away;
   * if the tick then ticks over before the loop's first check (the boot line
   * alone takes ~1.99 ms, so it tends to end just before an edge), that
   * check counts a bogus overrun. Starting on a fresh edge leaves a full ms. */
  uint32_t seed_tick = HAL_GetTick();
  while (HAL_GetTick() == seed_tick)
  {
    /* wait for the next tick edge (< 1 ms) */
  }
  next_deadline_ms = HAL_GetTick() + LOOP_PERIOD_MS;
  /* Seeded with the first iteration's scheduled time: the first report is
   * at the first multiple of 100 ms from there, health at the first
   * ...50, as with the old "%" rule. */
  PeriodicSchedule_Init(&report_schedule, REPORT_PERIOD_MS, 0u, next_deadline_ms);
  PeriodicSchedule_Init(&health_schedule, HEALTH_PERIOD_MS, HEALTH_PHASE_MS, next_deadline_ms);
  PeriodicSchedule_Init(&heartbeat_schedule, HEARTBEAT_TICK_MS, 0u, next_deadline_ms);
  PeriodicSchedule_Init(&battery_schedule, BATTERY_PERIOD_MS, BATTERY_PHASE_MS, next_deadline_ms);
  PeriodicSchedule_Init(&link_state_schedule, LINK_STATE_PERIOD_MS, LINK_STATE_PHASE_MS, next_deadline_ms);
  PeriodicSchedule_Init(&link_status_schedule, LINK_STATUS_PERIOD_MS, LINK_STATUS_PHASE_MS, next_deadline_ms);
#if SPEED_CAPTURE
  SpeedCapture_Init(&speed_capture, speed_capture_samples, SPEED_CAPTURE_SAMPLES);
  PeriodicSchedule_Init(&capture_schedule, REPORT_PERIOD_MS, CAPTURE_LINE_PHASE_MS, next_deadline_ms);
#endif
  MotorController_Init(&motor_controller, HAL_GetTick());  /* starts latched */
  if (!pwm_period_ok)
  {
    MotorController_LockOut(&motor_controller, MOTOR_FAULT_PERIOD_MISMATCH);  /* mstate=3 + fast blink */
  }
  /* Open loop until the first "v"; the estimate is invalid (STARTING, then
   * NO_MAGNET until the first health read at ~50 ms) for its first ~71 ms,
   * while the motor is latched anyway. */
  SpeedEstimator_Init(&speed_estimator);
  SpeedControl_Init(&speed_controller);
  StallDetect_Init(&stall_detector);
#if ENCODER_FAULT_TEST
  EncoderFault_Init(&encoder_fault);
#endif
  SteeringController_Init(&steering_controller, HAL_GetTick(), (STEERING_DIRECTION_INVERTED != 0));  /* no pulses yet */
  if (!servo_timer_ok)
  {
    SteeringController_LockOut(&steering_controller);  /* sstate=3: never pulses */
  }
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- Fixed-rate schedule --------------------------------------------
     * Each deadline is the previous deadline + 1 ms, not "now + 1 ms", so
     * however long the work takes, iteration N always targets boot + N ms.
     * The signed casts keep the comparisons correct when the ms tick wraps
     * after 49.7 days. */

    /* Deadline already passed before we started waiting: the previous
     * iteration's work ran into this iteration's slot. */
    if ((int32_t)(HAL_GetTick() - next_deadline_ms) >= 0)
    {
      overrun_count++;
    }
    while ((int32_t)(HAL_GetTick() - next_deadline_ms) < 0)
    {
      /* busy-wait: nothing else to do yet, and it keeps timing simple */
    }

    uint32_t now_ms = HAL_GetTick();
    if (now_ms - next_deadline_ms >= RESYNC_THRESHOLD_MS)
    {
      resync_count++;
      next_deadline_ms = now_ms;
    }
    uint32_t scheduled_time_ms = next_deadline_ms;
    next_deadline_ms += LOOP_PERIOD_MS;

#if LOOP_TIMING_PROFILE
    /* Read before the timed region starts. Section times are filled in as
     * the iteration runs; those that don't happen this iteration stay 0. */
    bool profile_uart_busy_at_start = (huart2.gState != HAL_UART_STATE_READY);
    bool profile_was_health = false;
    bool profile_was_report_head = false;
    bool profile_was_report_tail = false;
    bool profile_was_report_motor = false;
    bool profile_was_report_steering = false;
    bool profile_was_report_speed = false;
    bool profile_was_line_build = false;
    uint32_t profile_motor_cycles = 0;
    uint32_t profile_ctl_cycles = 0;
    uint32_t profile_encoder_cycles = 0;
    uint32_t profile_est_cycles = 0;
    uint32_t profile_health_reads_cycles = 0;
    uint32_t profile_imu_cycles = 0;
    uint32_t profile_format_cycles = 0;
    uint32_t profile_uart_start_cycles = 0;
#endif

    uint32_t iteration_start_cycles = DWT->CYCCNT;

    /* ---- Every 1 ms: commands, speed control, motor, steering ------------ */
    /* First in the iteration, so the motor's timing doesn't depend on how
     * long the I2C reads took. now_ms (not scheduled_time_ms) is the clock,
     * so a stalled loop still ages the command watchdog and the steering
     * timeout. The ramp is in ticks per ms, so this has to run every
     * iteration. */
    bool hello_received = false;
    uint32_t hang_ms = Commands_ProcessReceived(&command_line, &motor_controller, &speed_controller,
                                                &steering_controller, &rx_trouble_count_seen, &hello_received,
                                                now_ms);
    if (hello_received)
    {
      /* link_rx already switched to binary mode and latched the motor. */
      LinkTx_SendHelloAck(reset_cause_flags, imu_ok, imu_who_am_i, &link_ack_seq);
    }
#if IWDG_HANG_TEST
    /* Deliberate stall ("h <n>", test builds only): busy-wait with interrupts
     * on, so only the IWDG can end a stall that's too long. The motor and the
     * servo keep their last outputs meanwhile, exactly as in a real hang. */
    if (hang_ms != 0u)
    {
      uint32_t stall_start_ms = HAL_GetTick();
      while (HAL_GetTick() - stall_start_ms < hang_ms)
      {
      }
    }
#else
    (void)hang_ms;  /* always 0: "h" doesn't exist in a normal build */
#endif
    CommandUart_KeepReceiving(&rx_restart_count);
    /* Speed control and the stall check act on the previous ms's estimate
     * (the encoder is read later in the iteration) and only through
     * motor_control's API, so the cap, ramp, dwell, watchdog, latch and FAULT
     * apply as for "m". Before MotorController_Update, so a new target or a
     * latch takes effect in this same ms. The PI runs on its own 10 ms
     * due-time schedule (speed_control.c); the stall check every ms, in "m"
     * as in "v" (plus its saturation rule, "v" only: stall_detect.h). */
#if LOOP_TIMING_PROFILE
    uint32_t profile_ctl_start_cycles = DWT->CYCCNT;
#endif
    SpeedControl_Update(&speed_controller, &motor_controller, &speed_estimator, now_ms);
    StallDetect_Update(&stall_detector, &motor_controller, &speed_controller, &speed_estimator, now_ms);
#if LOOP_TIMING_PROFILE
    profile_ctl_cycles = DWT->CYCCNT - profile_ctl_start_cycles;
#endif
    MotorOutput motor_output = MotorController_Update(&motor_controller, now_ms);
    TB6612_PinPlan motor_plan = TB6612_Apply(&motor_driver, &motor_output);
    /* A guard in the output stage fired: it already made this ms safe (cap,
     * coast or disable), but it means the state machine asked for something
     * it never should, so don't keep driving on it. The next ms is disabled. */
    MotorFaultReason motor_fault = TB6612_PlanFaultReason(&motor_plan);
    if (motor_fault != MOTOR_FAULT_NONE)
    {
      MotorController_LockOut(&motor_controller, motor_fault);
    }

    /* Steering: independent of the motor (its own timeout, fed only by "s";
     * unaffected by motor FAULT, so the truck can still be steered while it
     * coasts). Same pattern as the motor: if the output guard fired, the
     * state machine asked for something it never should (or the timer
     * changed), so no more pulses from the next ms on. */
    uint32_t steering_pulse_us = SteeringController_Update(&steering_controller, now_ms);
    ServoPlan servo_plan = Servo_Apply(&servo, steering_pulse_us);
    if (servo_plan.timer_refused || servo_plan.out_of_range)
    {
      SteeringController_LockOut(&steering_controller);
    }

    /* ---- Every 10 ms: battery voltage --------------------------------- */
    /* Collect the sequence started 10 ms ago (it took ~41 us) and start the
     * next one. Never waits: if it hasn't finished, that sample is missed
     * and counted (three in a row latch an ADC fault). Register-level LL
     * calls only (static inline), no HAL state machine. now_ms is the
     * clock, as for the motor and steering. */
    if (PeriodicSchedule_IsDue(&battery_schedule, scheduled_time_ms))
    {
      if (LL_ADC_IsActiveFlag_JEOS(ADC1))
      {
        uint32_t vrefint_raw = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        uint32_t battery_raw = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
        LL_ADC_ClearFlag_JEOC(ADC1);
        LL_ADC_ClearFlag_JEOS(ADC1);
        BatteryMonitor_Update(&battery_monitor, true, vrefint_raw, battery_raw,
                              motor_output.duty_ticks != 0u, now_ms);
      }
      else
      {
        BatteryMonitor_MissedSample(&battery_monitor);
      }
      if (!LL_ADC_INJ_IsConversionOngoing(ADC1))
      {
        LL_ADC_INJ_StartConversion(ADC1);
      }
    }
    /* Cutoff, ADC fault (also from boot) or removed while driving: motor
     * FAULT (mstate=3, fast blink, only a reset clears it), from the next
     * ms on. Checked every ms; LockOut keeps the first reason. Steering is
     * not touched: it still follows "s" and centres on link loss. */
    MotorFaultReason battery_fault = BatteryAdc_MotorFaultReason(&battery_monitor);
    if (battery_fault != MOTOR_FAULT_NONE)
    {
      MotorController_LockOut(&motor_controller, battery_fault);
    }

    /* The I2C timing (i2c_us_max) starts here, so it means the same as
     * before the motor section existed. */
    uint32_t encoder_start_cycles = DWT->CYCCNT;
#if LOOP_TIMING_PROFILE
    profile_motor_cycles = encoder_start_cycles - iteration_start_cycles;
#endif

    /* ---- Every 1 ms: angle + magnitude ---------------------------------- */
#if SPEED_CAPTURE
    uint32_t capture_i2c_errors_before = i2c_error_count;  /* the angle read failed if this grows */
#endif
    uint16_t raw_angle;
    bool angle_read_ok = (AS5600_ReadRawAngle(&hi2c1, &raw_angle) == HAL_OK);
    if (!angle_read_ok)
    {
      i2c_error_count++;
    }
#if ENCODER_FAULT_TEST
    /* TEST BUILD ONLY: after the bus's own error count (an injected failure
     * is not an I2C error), before anything uses the angle. */
    EncoderFault_Apply(&encoder_fault, &angle_read_ok, &raw_angle, now_ms);
#endif
#if LOOP_TIMING_PROFILE
    uint32_t profile_est_start_cycles = DWT->CYCCNT;   /* restarted below after the raw unwrap */
#endif
    if (angle_read_ok)
    {
      AS5600_UnwrapperUpdate(&wheel_encoder, raw_angle);
#if LOOP_TIMING_PROFILE
      profile_est_start_cycles = DWT->CYCCNT;
#endif
      AS5600_UnwrapperUpdate(&corrected_encoder, AngleCorrection_Apply(raw_angle));
      latest_raw_angle = raw_angle;
    }
    /* else: don't feed the unwrappers a bad value; the next good sample's
     * delta covers the gap. */
    /* Every ms, right after the read, failed or not (a failed read makes the
     * estimate invalid for the next 21 ms). MD is from the last health read. */
    {
      SpeedEstimatorInput estimator_input = {
        .now_ms = now_ms,
        .count = corrected_encoder.continuous_count,
        .read_ok = angle_read_ok,
        .magnet_detected = magnet_status.magnet_detected };
      SpeedEstimator_Update(&speed_estimator, &estimator_input);
    }
#if LOOP_TIMING_PROFILE
    profile_est_cycles = DWT->CYCCNT - profile_est_start_cycles;
#endif
    /* Pi link STATE: what the encoder did this period (both modes, ~1 us). */
    if (!speed_estimator.valid)
    {
      link_period.estimate_invalid = true;
      if (link_estimate_was_valid)
      {
        link_odometer_epoch++;  /* a new stretch of "distance uncertain" begins */
      }
    }
    link_estimate_was_valid = speed_estimator.valid;
    if (!angle_read_ok)
    {
      link_period.angle_read_failed = true;
    }
#if SPEED_CAPTURE
    /* TEMPORARY (speed_capture.h): the motor's target and applied duty and
     * the fresh count, every ms; a recording starts when the target changes
     * while armed. Before the magnitude read, which counts its own errors. */
    {
      SpeedCaptureInput capture_input = {
        .now_ms = now_ms,
        .target_ticks = motor_controller.target_ticks,
        .applied_ticks = motor_controller.applied_ticks,
        .count = wheel_encoder.continuous_count,
        .raw_angle = latest_raw_angle,
        .read_ok = (i2c_error_count == capture_i2c_errors_before),
        .vbat_mv = BatteryMonitor_FilteredMv(&battery_monitor) };
      SpeedCapture_Tick(&speed_capture, &capture_input);
    }
#endif

    uint16_t magnitude;
    if (AS5600_ReadMagnitude(&hi2c1, &magnitude) == HAL_OK)
    {
      if (magnitude < magnitude_min) { magnitude_min = magnitude; }
      if (magnitude > magnitude_max) { magnitude_max = magnitude; }
    }
    else
    {
      i2c_error_count++;
    }

#if LOOP_TIMING_PROFILE
    /* Encoder section = both reads plus the unwrap and min/max bookkeeping. */
    profile_encoder_cycles = DWT->CYCCNT - encoder_start_cycles;
#endif

    /* ---- Every 100 ms: magnet health ------------------------------------ */
    if (PeriodicSchedule_IsDue(&health_schedule, scheduled_time_ms))
    {
      if (AS5600_ReadStatus(&hi2c1, &magnet_status) != HAL_OK) { i2c_error_count++; }
      if (AS5600_ReadAgc(&hi2c1, &agc_value) != HAL_OK)        { i2c_error_count++; }
#if LOOP_TIMING_PROFILE
      profile_was_health = true;
#endif
    }

    /* Unsigned subtraction stays correct even if CYCCNT wrapped (every ~134 s at 32 MHz). */
    uint32_t i2c_cycles = DWT->CYCCNT - encoder_start_cycles;
    if (i2c_cycles > i2c_cycles_max) { i2c_cycles_max = i2c_cycles; }
#if LOOP_TIMING_PROFILE
    if (profile_was_health) { profile_health_reads_cycles = i2c_cycles - profile_encoder_cycles; }
#endif

    /* ---- Every 1 ms: IMU (only if init succeeded) ----------------------- */
    /* Not synchronized to the IMU's own 1 kHz ODR, so now and then a sample
     * is read twice or missed (see README). */
    if (imu_ok)
    {
      uint32_t spi_start_cycles = DWT->CYCCNT;
      ICM42688_RawSample imu_sample;
      HAL_StatusTypeDef imu_result = ICM42688_ReadRawSample(&imu, &imu_sample);
      uint32_t spi_cycles = DWT->CYCCNT - spi_start_cycles;
      if (spi_cycles > spi_cycles_max) { spi_cycles_max = spi_cycles; }
#if LOOP_TIMING_PROFILE
      profile_imu_cycles = spi_cycles;  /* the burst read call only, same span as spi_us_max */
#endif

      if (imu_result == HAL_OK)
      {
        latest_imu_sample = imu_sample;
        if (imu_sample_count < IMU_SUM_MAX_SAMPLES)
        {
          gyro_sum_x += imu_sample.gyro_x;
          gyro_sum_y += imu_sample.gyro_y;
          gyro_sum_z += imu_sample.gyro_z;
          imu_sample_count++;
        }
        /* Pi link STATE: its own 10 ms sums (the ones above are the text line's). */
        if (link_period.imu_samples < LINK_STATE_MAX_SAMPLES)
        {
          link_period.gyro_sum[0] += imu_sample.gyro_x;
          link_period.gyro_sum[1] += imu_sample.gyro_y;
          link_period.gyro_sum[2] += imu_sample.gyro_z;
          link_period.accel_sum[0] += imu_sample.accel_x;
          link_period.accel_sum[1] += imu_sample.accel_y;
          link_period.accel_sum[2] += imu_sample.accel_z;
          link_period.imu_samples++;
        }
      }
      else
      {
        link_period.imu_read_failed = true;
        imu_error_count++;
      }
    }

    /* ---- Later parts of a telemetry line -------------------------------- */
    /* The line used to be built with snprintf, which took ~450 us for the
     * first half alone, so it was split across iterations: the IMU fields are
     * formatted in the iteration after the window closed, the motor fields
     * (~14 us each) in the one after that, the reset cause, steering and
     * battery fields in the one after that, and the speed control fields
     * (v5) plus the send in the last one, all from the snapshots taken when
     * the window closed. That keeps every iteration below the
     * ~626 us the head and health iterations take. Keyed on line_step, not on
     * a phase, so a resync jump can't strand a half-built line. "else if":
     * each part gets an iteration of its own.
     * Binary mode (Pi link) has no text line: one half-built when the HELLO
     * came is dropped, so it can't take the UART from the frames. */
    if (link_rx.mode == LINK_MODE_BINARY)
    {
      line_step = LINE_IDLE;
    }
    if (line_step == LINE_IMU_PART_NEXT)
    {
      line_step = LINE_MOTOR_PART_NEXT;
#if LOOP_TIMING_PROFILE
      profile_was_report_tail = true;
      uint32_t profile_format_start_cycles = DWT->CYCCNT;
#endif
      TelemetryWriter_AppendUnsignedField(&line_writer, "imu_ok",  imu_report.imu_ok ? 1u : 0u);
      TelemetryWriter_AppendSignedField(&line_writer,   "ax",      imu_report.latest.accel_x);
      TelemetryWriter_AppendSignedField(&line_writer,   "ay",      imu_report.latest.accel_y);
      TelemetryWriter_AppendSignedField(&line_writer,   "az",      imu_report.latest.accel_z);
      TelemetryWriter_AppendSignedField(&line_writer,   "gx",      imu_report.latest.gyro_x);
      TelemetryWriter_AppendSignedField(&line_writer,   "gy",      imu_report.latest.gyro_y);
      TelemetryWriter_AppendSignedField(&line_writer,   "gz",      imu_report.latest.gyro_z);
      TelemetryWriter_AppendSignedField(&line_writer,   "temp",    imu_report.latest.temperature);
      TelemetryWriter_AppendSignedField(&line_writer,   "gsum_x",  imu_report.gyro_sum_x);
      TelemetryWriter_AppendSignedField(&line_writer,   "gsum_y",  imu_report.gyro_sum_y);
      TelemetryWriter_AppendSignedField(&line_writer,   "gsum_z",  imu_report.gyro_sum_z);
      TelemetryWriter_AppendUnsignedField(&line_writer, "n_imu",   imu_report.sample_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "imu_err", imu_report.imu_error_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "spi_us_max", imu_report.spi_us_max);
#if LOOP_TIMING_PROFILE
      profile_format_cycles = DWT->CYCCNT - profile_format_start_cycles;
#endif
    }
    else if (line_step == LINE_MOTOR_PART_NEXT)
    {
      line_step = LINE_STEERING_PART_NEXT;
#if LOOP_TIMING_PROFILE
      profile_was_report_motor = true;
      uint32_t profile_format_start_cycles = DWT->CYCCNT;
#endif
      TelemetryWriter_AppendSignedField(&line_writer,   "cmd_pct",   motor_report.requested_percent);
      TelemetryWriter_AppendSignedField(&line_writer,   "duty",      motor_report.applied_ticks);
      TelemetryWriter_AppendUnsignedField(&line_writer, "mstate",    motor_report.state);
      TelemetryWriter_AppendUnsignedField(&line_writer, "wd_trip",   motor_report.watchdog_trip_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "cmd_ok",    motor_report.valid_command_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "cmd_bad",   motor_report.rejected_command_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "cmd_clamp", motor_report.clamped_command_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "rx_err",    motor_report.rx_error_count);
#if LOOP_TIMING_PROFILE
      profile_format_cycles = DWT->CYCCNT - profile_format_start_cycles;
#endif
    }
    else if (line_step == LINE_STEERING_PART_NEXT)
    {
      line_step = LINE_SPEED_PART_NEXT;
#if LOOP_TIMING_PROFILE
      profile_was_report_steering = true;
      uint32_t profile_format_start_cycles = DWT->CYCCNT;
#endif
      /* Telemetry v3: appended after the motor fields, so v1/v2 positions
       * are unchanged. */
      TelemetryWriter_AppendUnsignedField(&line_writer, "rst",       steering_report.reset_cause_flags);
      TelemetryWriter_AppendSignedField(&line_writer,   "s_pct",     steering_report.requested_percent);
      TelemetryWriter_AppendUnsignedField(&line_writer, "s_us",      steering_report.pulse_us);
      TelemetryWriter_AppendUnsignedField(&line_writer, "sstate",    steering_report.state);
      TelemetryWriter_AppendUnsignedField(&line_writer, "s_ok",      steering_report.valid_command_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "s_wd",      steering_report.timeout_count);
      /* Telemetry v4: the battery, appended after the steering fields. */
      TelemetryWriter_AppendUnsignedField(&line_writer, "vbat",      battery_report.vbat_mv);
      TelemetryWriter_AppendUnsignedField(&line_writer, "bstate",    battery_report.state);
      TelemetryWriter_AppendUnsignedField(&line_writer, "vdda",      battery_report.vdda_mv);
#if LOOP_TIMING_PROFILE
      profile_format_cycles = DWT->CYCCNT - profile_format_start_cycles;
#endif
    }
    else if (line_step == LINE_SPEED_PART_NEXT)
    {
      line_step = LINE_IDLE;
#if LOOP_TIMING_PROFILE
      profile_was_report_speed = true;
      uint32_t profile_format_start_cycles = DWT->CYCCNT;
#endif
      /* Telemetry v5: the speed control, appended after the battery fields. */
      TelemetryWriter_AppendSignedField(&line_writer,   "vset",      speed_report.setpoint_units);
      TelemetryWriter_AppendSignedField(&line_writer,   "vest",      speed_report.estimate_units);
      TelemetryWriter_AppendUnsignedField(&line_writer, "ctl",       speed_report.state);
      TelemetryWriter_AppendUnsignedField(&line_writer, "vok",       speed_report.estimate_valid);
      TelemetryWriter_AppendSignedField(&line_writer,   "vi",        speed_report.integrator_ticks);
      TelemetryWriter_AppendUnsignedField(&line_writer, "stall_n",   speed_report.stall_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "enc_bad",   speed_report.enc_bad_count);
      TelemetryWriter_AppendUnsignedField(&line_writer, "mfault",    speed_report.fault_reason);
      TelemetryWriter_AppendText(&line_writer, "\r\n");
#if LOOP_TIMING_PROFILE
      uint32_t profile_format_end_cycles = DWT->CYCCNT;
      profile_format_cycles = profile_format_end_cycles - profile_format_start_cycles;
#endif

      /* Can't be truncated in practice (worst case 786 of 1024 bytes), but a
       * cut line would only confuse the parsers, so it isn't sent. The UART
       * was idle when the head was written and nothing else sends, so the
       * transmit can only fail on a HAL error. Either way the window is
       * already closed, so count it as a skipped line. */
      if (line_writer.truncated ||
          HAL_UART_Transmit_IT(&huart2, (uint8_t *)telemetry_line, (uint16_t)line_writer.length) != HAL_OK)
      {
        skipped_report_count++;
      }
#if LOOP_TIMING_PROFILE
      /* Includes any UART interrupt that fires as soon as sending starts. */
      profile_uart_start_cycles = DWT->CYCCNT - profile_format_end_cycles;
#endif
    }

    /* ---- Every 100 ms: telemetry line ----------------------------------- */
    /* TEXT mode only (byte for byte the v5 line); in binary mode the due
     * time is still consumed every iteration, and nothing is built or
     * counted as skipped. */
    if (PeriodicSchedule_IsDue(&report_schedule, scheduled_time_ms) && link_rx.mode == LINK_MODE_TEXT)
    {
      /* HAL sets gState back to READY in the UART interrupt once the last
       * byte of the previous line has left. Until then the interrupt is
       * still reading telemetry_line, so we must not overwrite it. */
      bool line_started = false;
      if (huart2.gState == HAL_UART_STATE_READY && line_step == LINE_IDLE)
      {
        /* Unsigned math: same odometer-wrap reasoning as the unwrapper. */
        int32_t counts_since_last_line =
            (int32_t)((uint32_t)corrected_encoder.continuous_count - (uint32_t)count_at_last_report);

#if LOOP_TIMING_PROFILE
        uint32_t profile_format_start_cycles = DWT->CYCCNT;
#endif
        TelemetryWriter_Init(&line_writer, telemetry_line, sizeof(telemetry_line));
        TelemetryWriter_AppendUnsignedField(&line_writer, "t_ms",        scheduled_time_ms);
        TelemetryWriter_AppendUnsignedField(&line_writer, "raw",         latest_raw_angle);
        TelemetryWriter_AppendSignedField(&line_writer,   "count",       corrected_encoder.continuous_count);
        TelemetryWriter_AppendSignedField(&line_writer,   "d100",        counts_since_last_line);
        TelemetryWriter_AppendUnsignedField(&line_writer, "MD",          magnet_status.magnet_detected);
        TelemetryWriter_AppendUnsignedField(&line_writer, "ML",          magnet_status.magnet_too_weak);
        TelemetryWriter_AppendUnsignedField(&line_writer, "MH",          magnet_status.magnet_too_strong);
        TelemetryWriter_AppendUnsignedField(&line_writer, "AGC",         agc_value);
        TelemetryWriter_AppendUnsignedField(&line_writer, "mag_min",     magnitude_min);
        TelemetryWriter_AppendUnsignedField(&line_writer, "mag_max",     magnitude_max);
        TelemetryWriter_AppendUnsignedField(&line_writer, "i2c_err",     i2c_error_count);
        TelemetryWriter_AppendUnsignedField(&line_writer, "overrun",     overrun_count);
        TelemetryWriter_AppendUnsignedField(&line_writer, "resync",      resync_count);
        TelemetryWriter_AppendUnsignedField(&line_writer, "skipped",     skipped_report_count);
        TelemetryWriter_AppendUnsignedField(&line_writer, "i2c_us_max",  i2c_cycles_max / cycles_per_us);
        TelemetryWriter_AppendUnsignedField(&line_writer, "loop_us_max", loop_cycles_max / cycles_per_us);
#if LOOP_TIMING_PROFILE
        profile_format_cycles = DWT->CYCCNT - profile_format_start_cycles;
#endif

        /* The head is at most 228 bytes, so it always fits; if it ever
         * didn't, there'd be no point adding the rest, so treat that as a
         * skipped line. */
        if (!line_writer.truncated)
        {
          /* Freeze the IMU window and the motor, steering, battery and speed state
           * now so every part of the line describes the same 100 ms. */
          imu_report.imu_ok = imu_ok;
          imu_report.latest = latest_imu_sample;
          imu_report.gyro_sum_x = gyro_sum_x;
          imu_report.gyro_sum_y = gyro_sum_y;
          imu_report.gyro_sum_z = gyro_sum_z;
          imu_report.sample_count = imu_sample_count;
          imu_report.imu_error_count = imu_error_count;
          imu_report.spi_us_max = spi_cycles_max / cycles_per_us;

          motor_report.requested_percent = motor_controller.requested_percent;
          motor_report.applied_ticks = motor_controller.applied_ticks;
          motor_report.state = (uint32_t)motor_controller.state;
          motor_report.watchdog_trip_count = motor_controller.watchdog_trip_count;
          motor_report.valid_command_count = motor_controller.valid_command_count;
          motor_report.rejected_command_count = motor_controller.rejected_command_count;
          motor_report.clamped_command_count = motor_controller.clamped_command_count;
          /* Every way a received byte can be lost or corrupted, since boot. */
          motor_report.rx_error_count = uart_rx_error_count + uart_rx_ring.dropped_count + rx_restart_count;

          steering_report.reset_cause_flags = reset_cause_flags;
          steering_report.requested_percent = steering_controller.requested_percent;
          steering_report.pulse_us = steering_controller.pulse_us;
          steering_report.state = (uint32_t)steering_controller.state;
          steering_report.valid_command_count = steering_controller.valid_command_count;
          steering_report.timeout_count = steering_controller.timeout_count;

          battery_report.vbat_mv = BatteryMonitor_FilteredMv(&battery_monitor);
          battery_report.state = (uint32_t)battery_monitor.state;
          battery_report.vdda_mv = battery_monitor.vdda_mv;

          speed_report.setpoint_units = speed_controller.setpoint_units;
          speed_report.estimate_units = speed_estimator.speed_units;
          speed_report.state = (uint32_t)SpeedControl_State(&speed_controller);
          speed_report.estimate_valid = speed_estimator.valid ? 1u : 0u;
          speed_report.integrator_ticks = SpeedControl_IntegratorTicks(&speed_controller);
          speed_report.stall_count = stall_detector.stall_count;
          speed_report.enc_bad_count = speed_controller.enc_bad_count;
          speed_report.fault_reason = (uint32_t)motor_controller.fault_reason;

          line_step = LINE_IMU_PART_NEXT;  /* the next four iterations format the rest and send it */
          line_started = true;
        }
      }

      if (line_started)
      {
        /* Start a new window only when a line is on its way, so nothing that
         * happened during a skipped window is lost from the next line. */
        count_at_last_report = corrected_encoder.continuous_count;
        magnitude_min = UINT16_MAX;
        magnitude_max = 0;
        i2c_cycles_max = 0;
        loop_cycles_max = 0;
        gyro_sum_x = 0;
        gyro_sum_y = 0;
        gyro_sum_z = 0;
        imu_sample_count = 0;
        spi_cycles_max = 0;
#if LOOP_TIMING_PROFILE
        profile_was_report_head = true;
        /* Same window boundary as loop_us_max: swap which profile window
         * collects. If the previous profile line is somehow still being
         * built (only possible after a resync), keep collecting into the
         * same window instead, so the next profile line covers both. */
        if (profile_build_step == 0 && !profile_line_ready)
        {
          profile_collecting_index = 1 - profile_collecting_index;
          profile_window_t_ms = scheduled_time_ms;
          profile_build_step = 1;
        }
#endif
      }
      else
      {
        skipped_report_count++;
      }
    }

    /* ---- Pi link: STATE every 10 ms, STATUS every 100 ms (binary mode) --- */
    /* Packed and queued in this iteration (no formatting, ~tens of us); the
     * transmit-complete interrupt sends them one after the other. Both due
     * times are consumed every iteration in either mode. */
    if (PeriodicSchedule_IsDue(&link_state_schedule, scheduled_time_ms))
    {
      if (link_rx.mode == LINK_MODE_BINARY)
      {
        link_state_message.t_ms = scheduled_time_ms;
        link_state_message.count = corrected_encoder.continuous_count;
        link_state_message.speed_cps = speed_estimator.speed_units * LINK_CPS_PER_UNIT;
        link_state_message.enc_flags = (speed_estimator.valid ? LINK_ENC_VALID_NOW : 0u) |
                                       (link_period.estimate_invalid ? LINK_ENC_INVALID_IN_PERIOD : 0u) |
                                       (link_period.angle_read_failed ? LINK_ENC_READ_FAILED : 0u);
        link_state_message.odometer_epoch = link_odometer_epoch;
        for (size_t axis = 0u; axis < 3u; axis++)
        {
          link_state_message.gyro_sum[axis] = link_period.gyro_sum[axis];
          link_state_message.accel_sum[axis] = link_period.accel_sum[axis];
        }
        link_state_message.imu_samples = link_period.imu_samples;
        link_state_message.imu_flags = (imu_ok ? LINK_IMU_OK : 0u) |
                                       (link_period.imu_read_failed ? LINK_IMU_READ_FAILED : 0u);
        link_state_message.duty_ticks = motor_controller.applied_ticks;
        link_state_message.motor_state = (uint32_t)motor_controller.state;
        link_state_message.control_state = (uint32_t)SpeedControl_State(&speed_controller);
        link_state_message.drive_seq_echo = link_rx.drive_seq_echo;
        LinkTx_SendState(&link_state_message, &link_state_seq);
      }
      link_period = (LinkPeriod){0};  /* the next period starts now */
    }
    if (PeriodicSchedule_IsDue(&link_status_schedule, scheduled_time_ms) && link_rx.mode == LINK_MODE_BINARY)
    {
      LinkStatusMessage *status = &link_status_message;
      status->t_ms = scheduled_time_ms;
      status->setpoint_cps = link_rx.drive_speed_cps;
      status->integrator_ticks = SpeedControl_IntegratorTicks(&speed_controller);
      status->steer_percent = steering_controller.requested_percent;
      status->steer_pulse_us = steering_controller.pulse_us;
      status->steer_state = (uint32_t)steering_controller.state;
      status->motor_state = (uint32_t)motor_controller.state;
      status->control_state = (uint32_t)SpeedControl_State(&speed_controller);
      status->fault_reason = (uint32_t)motor_controller.fault_reason;
      status->latch_reason = (uint32_t)motor_controller.latch_reason;
      status->battery_state = (uint32_t)battery_monitor.state;
      status->imu_ok = imu_ok;
      status->mag_flags = (magnet_status.magnet_detected ? LINK_MAG_DETECTED : 0u) |
                          (magnet_status.magnet_too_weak ? LINK_MAG_TOO_WEAK : 0u) |
                          (magnet_status.magnet_too_strong ? LINK_MAG_TOO_STRONG : 0u);
      status->agc = agc_value;
      status->reset_flags = reset_cause_flags;
      status->vbat_mv = BatteryMonitor_FilteredMv(&battery_monitor);
      status->vdda_mv = battery_monitor.vdda_mv;
      status->imu_temperature = latest_imu_sample.temperature;
      status->magnitude_min = magnitude_min;
      status->magnitude_max = magnitude_max;
      status->loop_us_max = loop_cycles_max / cycles_per_us;
      status->i2c_us_max = i2c_cycles_max / cycles_per_us;
      status->spi_us_max = spi_cycles_max / cycles_per_us;
      status->counters[LINK_COUNT_WATCHDOG_TRIPS] = motor_controller.watchdog_trip_count;
      status->counters[LINK_COUNT_LATCHES] = motor_controller.latch_count;
      status->counters[LINK_COUNT_LOCKED_STALLS] = stall_detector.locked_stall_count;
      status->counters[LINK_COUNT_SATURATION_STALLS] = stall_detector.saturation_stall_count;
      status->counters[LINK_COUNT_ENC_BAD] = speed_controller.enc_bad_count;
      status->counters[LINK_COUNT_I2C_ERRORS] = i2c_error_count;
      status->counters[LINK_COUNT_IMU_ERRORS] = imu_error_count;
      status->counters[LINK_COUNT_OVERRUNS] = overrun_count;
      status->counters[LINK_COUNT_RESYNCS] = resync_count;
      status->counters[LINK_COUNT_TX_DROPPED] = link_tx_queue.dropped_count;
      status->counters[LINK_COUNT_STEERING_TIMEOUTS] = steering_controller.timeout_count;
      status->counters[LINK_COUNT_RX_OK] = link_rx.ok_count;
      status->counters[LINK_COUNT_RX_CRC_ERRORS] = link_rx.crc_error_count;
      status->counters[LINK_COUNT_RX_FRAME_ERRORS] = link_rx.frame_error_count;
      status->counters[LINK_COUNT_RX_REJECTED] = link_rx.rejected_count;
      status->counters[LINK_COUNT_RX_SEQ_GAPS] = link_rx.seq_gap_count;
      status->counters[LINK_COUNT_RX_DUPLICATES] = link_rx.duplicate_count;
      /* UART errors and the (normally never needed) reception restarts after them. */
      status->counters[LINK_COUNT_RX_UART_ERRORS] = uart_rx_error_count + rx_restart_count;
      status->counters[LINK_COUNT_RX_RING_DROPS] = uart_rx_ring.dropped_count;
      status->counters[LINK_COUNT_RX_TEXT_IGNORED] = link_rx.text_ignored_count;
      LinkTx_SendStatus(status, &link_status_seq);
      /* No text line in binary mode, so STATUS closes the window maxima
       * instead: each STATUS reports the 100 ms before it. */
      magnitude_min = UINT16_MAX;
      magnitude_max = 0;
      i2c_cycles_max = 0;
      loop_cycles_max = 0;
      spi_cycles_max = 0;
    }

#if LOOP_TIMING_PROFILE
    /* ---- TEMPORARY: profile line, one step per iteration ---------------- */
    /* Built from ms 6 of the window (after the telemetry line's five steps),
     * sent at ms 51-53 once the UART is free (PROFILE_SEND_MS). The buffer is
     * never rewritten while it is being sent: a build starts only after a
     * telemetry line started, which needs the UART idle. If a profile line
     * ever runs late, the next telemetry line shows up as "skipped", not as a
     * corrupted line. */
    if (profile_build_step != 0 && !profile_was_report_head && line_step == LINE_IDLE &&
        scheduled_time_ms - profile_window_t_ms >= PROFILE_BUILD_START_MS)
    {
      profile_was_line_build = true;
      ProfileWindow *reported_window = &profile_windows[1 - profile_collecting_index];
      if (profile_build_step == 1)
      {
        TelemetryWriter_Init(&profile_writer, profile_line, sizeof(profile_line));
      }
      profile_build_step = Profile_BuildLineStep(&profile_writer, profile_build_step, reported_window,
                                                 profile_window_t_ms, cycles_per_us);
      if (profile_build_step == 0)
      {
        profile_line_ready = !profile_writer.truncated;   /* never send a cut line */
        memset(reported_window, 0, sizeof(*reported_window));  /* ready to collect again after the next swap */
      }
    }
    else if (profile_line_ready && !profile_was_report_head &&
             scheduled_time_ms - profile_window_t_ms >= PROFILE_SEND_MS)
    {
      /* Never wait for the UART: if it's still busy by the deadline, this
       * window's profile line is simply missing from the log. */
      if (scheduled_time_ms - profile_window_t_ms > PROFILE_SEND_DEADLINE_MS)
      {
        profile_line_ready = false;
      }
      else if (huart2.gState == HAL_UART_STATE_READY && line_step == LINE_IDLE)
      {
        profile_was_line_build = true;
        (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)profile_line, (uint16_t)profile_writer.length);
        profile_line_ready = false;
      }
    }
#endif

#if SPEED_CAPTURE
    /* ---- TEMPORARY: capture lines (speed_capture.h) ---------------------- */
    /* At ms 51 of a window: start a line if one is due. In the iterations
     * after: a few pieces each. Then send it once the UART is free and no
     * telemetry line is being built, at most CAPTURE_SEND_DEADLINE_MS after
     * ms 51; otherwise it waits for the next window. Never waits itself. */
    if (!SpeedCapture_HasLine(&speed_capture))
    {
      capture_line_building = false;  /* cancelled ("c 0") or nothing to send */
      capture_line_ready = false;
    }
    if (PeriodicSchedule_IsDue(&capture_schedule, scheduled_time_ms))
    {
      capture_window_ms = scheduled_time_ms;
      if (!capture_line_building && !capture_line_ready && SpeedCapture_HasLine(&speed_capture))
      {
        TelemetryWriter_Init(&capture_writer, capture_line, sizeof(capture_line));
        capture_line_building = true;
      }
    }
    else if (capture_line_building)
    {
      if (SpeedCapture_BuildLineStep(&speed_capture, &capture_writer, CAPTURE_ITEMS_PER_STEP))
      {
        capture_line_building = false;
        capture_line_ready = true;
      }
    }
    else if (capture_line_ready && scheduled_time_ms - capture_window_ms <= CAPTURE_SEND_DEADLINE_MS &&
             huart2.gState == HAL_UART_STATE_READY && line_step == LINE_IDLE)
    {
      /* A cut line is never sent (it can't happen: the longest possible
       * line fits, tests/test_speed_capture.c); it is dropped, and the
       * fitter reports the gap. A HAL refusal is retried. */
      if (capture_writer.truncated ||
          HAL_UART_Transmit_IT(&huart2, (uint8_t *)capture_line, (uint16_t)capture_writer.length) == HAL_OK)
      {
        capture_line_ready = false;
        SpeedCapture_LineSent(&speed_capture);
      }
    }
#endif

    /* ---- Heartbeat ------------------------------------------------------ */
    /* Shows without a PC: alive, IMU missing (the loop runs on without it)
     * or motor FAULT; FAULT wins (loop_schedule.h). */
    if (PeriodicSchedule_IsDue(&heartbeat_schedule, scheduled_time_ms))
    {
      HeartbeatMode heartbeat_mode = Heartbeat_Mode(motor_controller.state == MOTOR_STATE_FAULT, imu_ok);
      if (Heartbeat_LedOn(heartbeat_mode, heartbeat_tick))
      {
        BSP_LED_On(LED_GREEN);
      }
      else
      {
        BSP_LED_Off(LED_GREEN);
      }
      heartbeat_tick++;
    }

    /* Total work this iteration, including line formatting. The report iteration's
     * own time is counted in the next window. */
    uint32_t loop_cycles = DWT->CYCCNT - iteration_start_cycles;
    if (loop_cycles > loop_cycles_max) { loop_cycles_max = loop_cycles; }

#if LOOP_TIMING_PROFILE
    /* After the total is taken, so this bookkeeping isn't in any measurement.
     * A coincidence of kinds (possible only after a resync) is filed under
     * the first matching kind in this order. */
    ProfileIterationKind profile_kind;
    if (profile_was_report_head)       { profile_kind = PROFILE_REPORT_HEAD; }
    else if (profile_was_report_tail)  { profile_kind = PROFILE_REPORT_TAIL; }
    else if (profile_was_report_motor) { profile_kind = PROFILE_REPORT_MOTOR; }
    else if (profile_was_report_steering) { profile_kind = PROFILE_REPORT_STEERING; }
    else if (profile_was_report_speed) { profile_kind = PROFILE_REPORT_SPEED; }
    else if (profile_was_health)       { profile_kind = PROFILE_HEALTH; }
    else if (profile_was_line_build)   { profile_kind = PROFILE_LINE_BUILD; }
    else if (profile_uart_busy_at_start) { profile_kind = PROFILE_NORMAL_UART_BUSY; }
    else                               { profile_kind = PROFILE_NORMAL_UART_IDLE; }
    Profile_Record(&profile_windows[profile_collecting_index].kinds[profile_kind],
                   profile_motor_cycles, profile_ctl_cycles, profile_encoder_cycles, profile_est_cycles,
                   profile_health_reads_cycles, profile_imu_cycles, profile_format_cycles,
                   profile_uart_start_cycles, loop_cycles);
#endif

    /* ---- Hardware watchdog: the one and only refresh -------------------- */
    /* Last statement of the iteration, so it's reached only if every part of
     * the iteration returned, the deadline busy-wait included (that one
     * would spin forever if the ms tick stopped). Never in an interrupt:
     * SysTick and USART2 keep interrupting a main loop that is stuck, so a
     * refresh there would hide exactly the hang this is meant to catch.
     * tools/test_firmware_sources.py checks both. */
    (void)HAL_IWDG_Refresh(&hiwdg);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc1);

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VREFINT;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0060112F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 249;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 31;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 19999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_Pin|MOTOR_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU_CS_Pin MOTOR_IN1_Pin MOTOR_IN2_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin|MOTOR_IN1_Pin|MOTOR_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT1_Pin */
  GPIO_InitStruct.Pin = IMU_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IMU_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_STBY_Pin */
  GPIO_InitStruct.Pin = MOTOR_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_STBY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SB16_A5_KEEP_ANALOG_Pin */
  GPIO_InitStruct.Pin = SB16_A5_KEEP_ANALOG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SB16_A5_KEEP_ANALOG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ---- USART2 receive callbacks (run inside the USART2 interrupt) ---------- */

/* One byte arrived in uart_rx_byte: queue it and re-arm for the next one.
 * HAL calls this from HAL_UART_IRQHandler, i.e. in interrupt context. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }
  (void)ByteRing_Push(&uart_rx_ring, uart_rx_byte);  /* a full ring counts the drop itself */
  (void)CommandUart_StartReceive();                   /* the main loop retries if this fails */
}

/* Framing or noise error: HAL has already delivered the (bad) byte through
 * HAL_UART_RxCpltCallback and keeps receiving. Overrun (a byte arrived
 * before the previous one was read): HAL has stopped receiving
 * (RxState = READY). Either way: count it, which makes the main loop reject
 * the line the byte belonged to, and make sure reception runs again. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }
  uart_rx_error_count = uart_rx_error_count + 1u;  /* only this interrupt writes it */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    (void)CommandUart_StartReceive();
  }
}

/* ---- USART2 transmit-complete callback (runs inside the USART2 interrupt) --
 * The last byte of a transmission has left; HAL has already set gState back
 * to READY (UART_EndTransmit_IT). If it was a queued Pi link frame, its slot
 * is freed now and only now (the UART read it until here); then the next
 * queued frame, if any, starts. A text, profile or capture line ending here
 * frees nothing, but starts the queue too. Never touches the IWDG. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }
  if (link_tx_queue_sending)
  {
    TxQueue_Pop(&link_tx_queue);
    link_tx_queue_sending = false;
  }
  size_t length = 0u;
  const volatile uint8_t *frame = TxQueue_Front(&link_tx_queue, &length);
  if (frame != NULL && HAL_UART_Transmit_IT(huart, (uint8_t *)(uintptr_t)frame, (uint16_t)length) == HAL_OK)
  {
    link_tx_queue_sending = true;
  }
}

/* ---- Emergency stop (fault handlers) --------------------------------------
 * Called first thing from Error_Handler and the fault handlers in
 * stm32l4xx_it.c. Without it, a fault while driving would leave TIM1 making
 * PWM on its own while the CPU spins in the handler: the motor would keep
 * running at the last duty forever. (The IWDG then resets the MCU ~250 ms
 * later, and it boots with the motor latched and the servo without pulses.)
 *
 * Only direct register writes: no HAL calls and no RAM state, because after a
 * fault neither can be trusted.
 *
 * Motor first. Order: STBY low first (on its own it disables the bridge, and
 * if anything after it went wrong it would still be done), then IN1 = IN2 =
 * low (coast even if STBY were stuck), then PWM off: compare 0, main output
 * disabled, and PA8 switched from the timer to a plain GPIO output driven
 * low.
 *
 * Servo second, and only the compare register: with output-compare preload
 * on, compare 0 takes effect at the next 20 ms period, so the pulse in
 * progress ends normally and no pulse follows (the servo goes limp, or holds
 * if it is a type that holds without pulses; unverified for the AS-1).
 * Clearing MOE or re-muxing PA3 like PA8 would cut the current pulse short,
 * and a short pulse is itself a command toward one endpoint or beyond it.
 *
 * If this runs before the GPIO/TIM clocks are enabled (an early init error),
 * the writes are ignored; the pins are then still in their reset state
 * (floating): the TB6612's input pull-downs keep the bridge off and the
 * servo gets no pulses. */
void Board_EmergencyStop(void)
{
  MOTOR_STBY_GPIO_Port->BRR = MOTOR_STBY_Pin;
  MOTOR_IN1_GPIO_Port->BRR = MOTOR_IN1_Pin;
  MOTOR_IN2_GPIO_Port->BRR = MOTOR_IN2_Pin;
  TIM1->CCR1 = 0u;
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  MOTOR_PWM_GPIO_Port->BRR = MOTOR_PWM_Pin;
  uint32_t pwm_pin_position = POSITION_VAL(MOTOR_PWM_Pin);
  MOTOR_PWM_GPIO_Port->MODER = (MOTOR_PWM_GPIO_Port->MODER & ~(GPIO_MODER_MODE0 << (pwm_pin_position * 2u)))
                               | (GPIO_MODER_MODE0_0 << (pwm_pin_position * 2u));  /* 01 = general-purpose output */

  TIM15->CCR2 = 0u;  /* no more servo pulses after the current one */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Actuators off before anything else: the timers would keep making PWM
   * while the CPU spins below (until the IWDG resets the MCU, ~250 ms, if it
   * is already running). */
  Board_EmergencyStop();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
