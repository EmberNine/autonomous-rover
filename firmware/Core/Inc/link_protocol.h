/**
  ******************************************************************************
  * @file    link_protocol.h
  * @brief   The Pi link's messages (protocol version 1): their payload
  *          layouts, the checks on what the Pi sends, the cps -> speed-unit
  *          conversion, and packing what the Nucleo sends. Framing is
  *          link_codec.c; acting on a message is link_rx.c.
  *
  * All multi-byte fields little-endian. Payload lengths are fixed per type:
  * any other length is rejected.
  *
  * Pi -> Nucleo
  *   HELLO  0x01  1 byte   version u8 (must be LINK_PROTOCOL_VERSION).
  *                         Enters binary mode, latches the motor (unless in
  *                         FAULT), answered by HELLO_ACK. Also in binary
  *                         mode (a Pi bridge restart). Feeds nothing.
  *   DRIVE  0x10  4 bytes  speed_cps i16, steer_pct i8, flags u8:
  *                           bit 0 steer_valid: steer_pct is a steering command
  *                           bit 1 arm: re-arm a latched motor; only with speed_cps 0
  *                           bits 2-7: must be 0
  *                         |speed_cps| <= LINK_MAX_SPEED_CPS, |steer_pct| <= 100
  *                         (also when steer_valid is 0). The only message
  *                         that feeds the motor watchdog; steer_valid feeds
  *                         the steering timeout. Binary mode only.
  *   ESTOP  0x11  0 bytes  Latch the motor at once (coast). Feeds nothing;
  *                         only a DRIVE with arm re-arms. Binary mode only.
  *
  * Nucleo -> Pi
  *   HELLO_ACK 0x81  13 bytes  see LinkHelloAckMessage
  *   STATE     0x82  42 bytes  100 Hz, see LinkStateMessage
  *   STATUS    0x83 117 bytes  10 Hz, see LinkStatusMessage
  *
  * Packing saturates every field to its wire type (a u16 above 65535 is
  * sent as 65535, an i16 below -32768 as -32768), so a value out of range
  * is visibly pinned instead of wrapping into a plausible one.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_link_protocol.c against
  * firmware/tests/link_message_vectors.txt (shared with tools/). No globals.
  ******************************************************************************
  */
#ifndef LINK_PROTOCOL_H
#define LINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "link_codec.h"
#include "speed_control.h"
#include "speed_estimator.h"

#define LINK_PROTOCOL_VERSION   1u

/* Message types. */
#define LINK_MSG_HELLO          0x01u
#define LINK_MSG_DRIVE          0x10u
#define LINK_MSG_ESTOP          0x11u
#define LINK_MSG_HELLO_ACK      0x81u
#define LINK_MSG_STATE          0x82u
#define LINK_MSG_STATUS         0x83u

/* Payload lengths. */
#define LINK_HELLO_LENGTH       1u
#define LINK_DRIVE_LENGTH       4u
#define LINK_ESTOP_LENGTH       0u
#define LINK_HELLO_ACK_LENGTH   13u
#define LINK_STATE_LENGTH       42u
#define LINK_STATUS_LENGTH      117u

/* DRIVE flags. */
#define LINK_DRIVE_STEER_VALID  0x01u
#define LINK_DRIVE_ARM          0x02u
#define LINK_DRIVE_RESERVED     0xFCu

/* Speeds on the wire are encoder counts per second. The controller works in
 * speed units of 1 count per SPEED_WINDOW_MS (20 ms) = 50 counts/s. */
#define LINK_CPS_PER_UNIT       (1000 / (int32_t)SPEED_WINDOW_MS)                   /* 50 */
#define LINK_MIN_SPEED_CPS      (SPEED_MIN_SETPOINT_UNITS * LINK_CPS_PER_UNIT)    /* 1000 */
#define LINK_MAX_SPEED_CPS      5000   /* = the 100 units "v" allows */
#define LINK_MAX_STEER_PERCENT  100

/* HELLO_ACK build_flags: a test or bench build must never drive the truck,
 * so the Pi refuses to drive whenever any bit is set. */
#define LINK_BUILD_IWDG_HANG_TEST        0x01u
#define LINK_BUILD_SPEED_CAPTURE         0x02u
#define LINK_BUILD_ENCODER_FAULT_TEST    0x04u
#define LINK_BUILD_LOOP_TIMING_PROFILE   0x08u
#define LINK_BUILD_BENCH_TEXT_COMMANDS   0x10u   /* m, v, s accepted in text mode (not a truck build) */

/* STATE enc_flags. */
#define LINK_ENC_VALID_NOW          0x01u  /* the speed estimate is valid at frame close */
#define LINK_ENC_INVALID_IN_PERIOD  0x02u  /* it was invalid at some ms since the previous STATE */
#define LINK_ENC_READ_FAILED        0x04u  /* an angle read failed since the previous STATE */
/* STATE imu_flags. */
#define LINK_IMU_OK                 0x01u  /* IMU init succeeded (imu_ok) */
#define LINK_IMU_READ_FAILED        0x02u  /* a burst read failed since the previous STATE */
/* STATUS mag_flags. */
#define LINK_MAG_DETECTED           0x01u  /* MD */
#define LINK_MAG_TOO_WEAK           0x02u  /* ML */
#define LINK_MAG_TOO_STRONG         0x04u  /* MH */

typedef enum
{
  LINK_PARSE_OK = 0,
  LINK_PARSE_BAD_LENGTH,      /* not the type's payload length */
  LINK_PARSE_BAD_VERSION,     /* HELLO for another protocol version */
  LINK_PARSE_BAD_FLAGS,       /* a reserved DRIVE flag bit set */
  LINK_PARSE_OUT_OF_RANGE,    /* |speed_cps| > LINK_MAX_SPEED_CPS or |steer_pct| > 100 */
  LINK_PARSE_ARM_WITH_SPEED,  /* arm together with a non-zero speed */
  /* Decided by link_rx.c, not by the payload: */
  LINK_PARSE_UNKNOWN_TYPE,    /* not a Pi -> Nucleo type */
  LINK_PARSE_NOT_BINARY_MODE  /* DRIVE or ESTOP before a HELLO */
} LinkParseResult;

typedef struct
{
  int32_t speed_cps;
  int32_t steer_percent;
  bool    steer_valid;
  bool    arm;
} LinkDrive;

/* What HELLO_ACK carries. */
typedef struct
{
  uint32_t build_flags;       /* LINK_BUILD_* */
  uint32_t reset_flags;       /* rst: RCC_CSR >> 24 */
  bool     imu_ok;
  uint32_t imu_who;           /* WHO_AM_I read at boot */
  uint32_t state_period_ms;   /* 10 */
  uint32_t status_period_ms;  /* 100 */
} LinkHelloAckMessage;

/* What STATE carries (100 Hz), in wire order. The sums cover exactly the
 * IMU samples since the previous STATE, so sum / imu_samples is the mean
 * over the period. */
typedef struct
{
  uint32_t t_ms;              /* u32: the iteration's scheduled time at frame close */
  int32_t  count;             /* i32: corrected odometer (as "count" in v5) */
  int32_t  speed_cps;         /* i16: the 20 ms estimate in counts/s */
  uint32_t enc_flags;         /* u8:  LINK_ENC_* */
  uint32_t odometer_epoch;    /* u8:  +1 at every valid -> invalid transition of the estimate (wraps) */
  int32_t  gyro_sum[3];       /* i32 x3: raw gyro x, y, z summed over the period */
  int32_t  accel_sum[3];      /* i32 x3: raw accel x, y, z summed over the period */
  uint32_t imu_samples;       /* u8:  samples in the sums */
  uint32_t imu_flags;         /* u8:  LINK_IMU_* */
  int32_t  duty_ticks;        /* i16: applied duty, signed */
  uint32_t motor_state;       /* u4 (low nibble of one byte): mstate */
  uint32_t control_state;     /* u4 (high nibble): ctl */
  uint32_t drive_seq_echo;    /* u8:  seq of the last DRIVE acted on */
} LinkStateMessage;

/* The counters STATUS carries, in wire order (u32 each, since boot, wrap). */
#define LINK_STATUS_COUNTERS 20u
typedef enum
{
  LINK_COUNT_WATCHDOG_TRIPS = 0,
  LINK_COUNT_LATCHES,
  LINK_COUNT_LOCKED_STALLS,
  LINK_COUNT_SATURATION_STALLS,
  LINK_COUNT_ENC_BAD,
  LINK_COUNT_I2C_ERRORS,
  LINK_COUNT_IMU_ERRORS,
  LINK_COUNT_OVERRUNS,
  LINK_COUNT_RESYNCS,
  LINK_COUNT_TX_DROPPED,
  LINK_COUNT_STEERING_TIMEOUTS,
  LINK_COUNT_RX_OK,
  LINK_COUNT_RX_CRC_ERRORS,
  LINK_COUNT_RX_FRAME_ERRORS,
  LINK_COUNT_RX_REJECTED,
  LINK_COUNT_RX_SEQ_GAPS,
  LINK_COUNT_RX_DUPLICATES,
  LINK_COUNT_RX_UART_ERRORS,
  LINK_COUNT_RX_RING_DROPS,
  LINK_COUNT_RX_TEXT_IGNORED
} LinkStatusCounter;

/* What STATUS carries (10 Hz), in wire order. */
typedef struct
{
  uint32_t t_ms;              /* u32 */
  int32_t  setpoint_cps;      /* i16: the last DRIVE speed as received */
  int32_t  integrator_ticks;  /* i16: vi */
  int32_t  steer_percent;     /* i8:  s_pct */
  uint32_t steer_pulse_us;    /* u16: s_us */
  uint32_t steer_state;       /* u8:  sstate */
  uint32_t motor_state;       /* u8:  mstate */
  uint32_t control_state;     /* u8:  ctl */
  uint32_t fault_reason;      /* u8:  mfault (MotorFaultReason) */
  uint32_t latch_reason;      /* u8:  MotorLatchReason */
  uint32_t battery_state;     /* u8:  bstate */
  bool     imu_ok;            /* u8 */
  uint32_t mag_flags;         /* u8:  LINK_MAG_* */
  uint32_t agc;               /* u8 */
  uint32_t reset_flags;       /* u8:  rst */
  uint32_t vbat_mv;           /* u16 */
  uint32_t vdda_mv;           /* u16 (saturates: v5 keeps implausible values up to 5,100,000) */
  int32_t  imu_temperature;   /* i16: raw */
  uint32_t magnitude_min;     /* u16 */
  uint32_t magnitude_max;     /* u16 */
  uint32_t loop_us_max;       /* u16 */
  uint32_t i2c_us_max;        /* u16 */
  uint32_t spi_us_max;        /* u16 */
  uint32_t counters[LINK_STATUS_COUNTERS];  /* u32 x20, LinkStatusCounter order */
} LinkStatusMessage;

/* cps -> speed units: |cps| < LINK_MIN_SPEED_CPS (1000, the controller's own
 * minimum, as HELLO_ACK says) -> 0, checked on the raw cps; otherwise round
 * to nearest, ties toward zero, the same for both signs. So 999 cps is 0
 * and 1000..1025 cps are 20 units.
 * Defined for |cps| <= LINK_MAX_SPEED_CPS (larger values are rejected
 * before this is called). */
int32_t LinkProtocol_CpsToUnits(int32_t cps);

/* Checks of what the Pi sends. frame is a CRC-checked frame of the type in
 * the name; the outputs are set only on LINK_PARSE_OK. */
LinkParseResult LinkProtocol_ParseHello(const LinkFrame *frame);
LinkParseResult LinkProtocol_ParseDrive(const LinkFrame *frame, LinkDrive *drive);
LinkParseResult LinkProtocol_ParseEstop(const LinkFrame *frame);

/* Packing: out must hold the type's length; returns that length. */
size_t LinkProtocol_PackHelloAck(const LinkHelloAckMessage *message, uint8_t *out);
size_t LinkProtocol_PackState(const LinkStateMessage *message, uint8_t *out);
size_t LinkProtocol_PackStatus(const LinkStatusMessage *message, uint8_t *out);

/* For the Pi side and the tests: the payload of a DRIVE (4 bytes). */
size_t LinkProtocol_PackDrive(int32_t speed_cps, int32_t steer_percent, uint32_t flags, uint8_t *out);

#endif /* LINK_PROTOCOL_H */
