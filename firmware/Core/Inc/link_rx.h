/**
  ******************************************************************************
  * @file    link_rx.h
  * @brief   Receive side of the Pi link: splits the received bytes into
  *          frames and text, checks each frame and acts on it. The only
  *          place that decides what a link message does to the motor, the
  *          speed controller and the steering.
  *
  * Two modes, one byte stream:
  *   - TEXT (boot): the human grammar of command_line.c. A 0x00 can never
  *     be part of a text line, so a 0x00 starts a frame; the frame ends at
  *     the next 0x00, and the bytes after it are text again. Only HELLO is
  *     acted on in this mode; any other frame is rejected, and a frame that
  *     grows longer than a HELLO is given up at once (LINK_TEXT_MODE_MAX_COBS),
  *     so a stray 0x00 can't swallow the text that follows it.
  *   - BINARY: entered by a valid HELLO, left only by a reset. Bytes outside
  *     frames are ignored and counted (once per line, rx_text_ignored); the
  *     caller may still use them in a test build only (e, c, h).
  * Every frame needs a 0x00 before AND after it (link_codec.h). Two 0x00 in
  * a row are an empty frame and are skipped.
  *
  * Safety rules (tests/test_link_rx.c checks each one):
  *   - nothing acts on a frame until its closing 0x00 has arrived and it has
  *     passed COBS, length, CRC, type, mode and payload checks; a frame that
  *     fails any of them is dropped and counted, and changes nothing else
  *     (not even the watchdog);
  *   - only DRIVE feeds the motor watchdog (MotorController_AcceptDriveCommand),
  *     and only DRIVE with steer_valid feeds the steering timeout;
  *   - HELLO latches the motor (unless in FAULT), also when already in
  *     binary mode; ESTOP latches it; only DRIVE with arm (and speed 0)
  *     re-arms; nothing over the link clears FAULT;
  *   - a UART error or a dropped byte during a frame poisons that frame;
  *   - sequence numbers are counted (gaps, duplicates), never acted on.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_link_rx.c and test_link_fuzz.c. No globals.
  ******************************************************************************
  */
#ifndef LINK_RX_H
#define LINK_RX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "link_codec.h"
#include "link_protocol.h"
#include "motor_control.h"
#include "speed_control.h"
#include "steering_control.h"

typedef enum
{
  LINK_MODE_TEXT = 0,
  LINK_MODE_BINARY
} LinkMode;

/* In TEXT mode a frame longer than a HELLO's COBS bytes (type, seq, version,
 * CRC: 5 bytes, + 1 for COBS) is abandoned as soon as it grows past them and
 * the link is back to text: a stray 0x00 costs at most these bytes of text. */
#define LINK_TEXT_MODE_MAX_COBS  (LINK_HELLO_LENGTH + LINK_BODY_OVERHEAD + 1u)   /* 6 */

/* What one received byte led to. */
typedef enum
{
  LINK_RX_NONE = 0,     /* consumed: part of a frame, or an empty frame */
  LINK_RX_TEXT_BYTE,    /* outside a frame: TEXT mode -> the command line; BINARY -> ignored (counted) */
  LINK_RX_FRAME_START,  /* a 0x00 outside a frame: in TEXT mode, abandon the partial text line */
  LINK_RX_DROPPED,      /* a frame ended and was dropped (counted) */
  LINK_RX_HELLO,        /* a valid HELLO was acted on: send HELLO_ACK */
  LINK_RX_DRIVE,        /* a valid DRIVE was acted on */
  LINK_RX_ESTOP         /* a valid ESTOP was acted on */
} LinkRxEvent;

/* What the link acts on. */
typedef struct
{
  MotorController    *motor;
  SpeedController    *speed;
  SteeringController *steering;
} LinkRxTargets;

typedef struct
{
  LinkMode  mode;
  bool      in_frame;          /* between a frame's opening and closing 0x00 */
  bool      overlong;          /* the frame grew past LINK_MAX_COBS: dropped at its end */
  bool      poisoned;          /* a byte of the frame was lost or corrupted: dropped at its end */
  bool      outside_line;      /* BINARY: inside a counted run of text, until '\n' or a 0x00 */
  size_t    length;
  uint8_t   buffer[LINK_MAX_COBS];
  LinkFrame frame;             /* the last frame that passed the CRC (valid right after an event) */

  bool      seq_seen;
  uint8_t   last_seq;          /* of the last frame that passed the CRC */
  uint8_t   drive_seq_echo;    /* of the last DRIVE acted on (STATE) */
  int32_t   drive_speed_cps;   /* of the last DRIVE acted on (STATUS setpoint) */
  LinkParseResult last_reject; /* why the last CRC-valid frame was rejected */
  uint8_t   last_reject_type;

  /* Totals since boot (wrap). */
  uint32_t  ok_count;          /* frames acted on */
  uint32_t  crc_error_count;   /* COBS fine, CRC wrong */
  uint32_t  frame_error_count; /* COBS error, too short, too long, poisoned */
  uint32_t  rejected_count;    /* CRC fine, refused: type, mode, length, flags, range, arm */
  uint32_t  seq_gap_count;     /* seq != previous + 1 (mod 256), duplicates excluded */
  uint32_t  duplicate_count;   /* seq == previous */
  uint32_t  text_ignored_count;/* BINARY: lines of text outside frames */
} LinkRx;

/* TEXT mode, nothing received. */
void LinkRx_Init(LinkRx *link);

/* One received byte, in order. Acts on a finished frame through targets. */
LinkRxEvent LinkRx_FeedByte(LinkRx *link, uint8_t byte, const LinkRxTargets *targets, uint32_t now_ms);

/* A byte was lost or corrupted (UART error, full ring). Returns true if a
 * frame is being received (it will be dropped); false if the link is
 * outside a frame, so the caller discards the text line instead. */
bool LinkRx_Poison(LinkRx *link);

/* May a finished text line act? The one place that decides it (main.c asks
 * before anything acts on a text line). letter: the line's letter if it is a
 * well-formed command, '\0' for a rejected line (which then only counts).
 *   - TEXT mode: only with bench_text_commands (the BENCH_TEXT_COMMANDS
 *     switch) or in a test build; otherwise every line is ignored (the caller
 *     counts it in cmd_bad) and text mode only waits for a HELLO;
 *   - BINARY mode: only e, c and h, and only in a test build (never m, v, s).
 * test_build: any of IWDG_HANG_TEST, SPEED_CAPTURE, ENCODER_FAULT_TEST,
 * LOOP_TIMING_PROFILE (bench builds that HELLO_ACK reports). */
bool LinkRx_TextCommandAllowed(LinkMode mode, char letter, bool bench_text_commands, bool test_build);

#endif /* LINK_RX_H */
