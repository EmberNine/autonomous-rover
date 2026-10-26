/*
 * Host-side test for Core/Src/link_rx.c with the real motor_control,
 * speed_control and steering_control: modes, what each message does and
 * doesn't do, the explicit arm, what feeds which watchdog, dropped frames,
 * poison, sequence counting, text outside frames. Not part of the firmware
 * build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_link_rx.c Core/Src/link_rx.c Core/Src/link_protocol.c Core/Src/link_codec.c Core/Src/motor_control.c Core/Src/speed_control.c Core/Src/steering_control.c -o /tmp/test_link_rx && /tmp/test_link_rx
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link_rx.h"

static int failure_count = 0;
static int check_count = 0;

#define CHECK(condition, ...)                                   \
  do                                                            \
  {                                                             \
    check_count++;                                              \
    if (!(condition))                                           \
    {                                                           \
      failure_count++;                                          \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);               \
      printf(__VA_ARGS__);                                      \
      printf("\n");                                             \
    }                                                           \
  } while (0)

typedef struct
{
  MotorController    motor;
  SpeedController    speed;
  SteeringController steering;
  LinkRx             link;
  LinkRxTargets      targets;
  uint32_t           now_ms;
  uint8_t            seq;
  uint32_t           text_bytes;   /* LINK_RX_TEXT_BYTE events */
  uint32_t           frame_starts; /* LINK_RX_FRAME_START events */
} Rig;

static void rig_init(Rig *rig)
{
  memset(rig, 0, sizeof(*rig));
  rig->now_ms = 1000u;
  MotorController_Init(&rig->motor, rig->now_ms);
  SpeedControl_Init(&rig->speed);
  SteeringController_Init(&rig->steering, rig->now_ms, false);
  LinkRx_Init(&rig->link);
  rig->targets.motor = &rig->motor;
  rig->targets.speed = &rig->speed;
  rig->targets.steering = &rig->steering;
}

/* Feeds bytes; returns the last event other than NONE / TEXT_BYTE / FRAME_START. */
static LinkRxEvent feed(Rig *rig, const uint8_t *bytes, size_t length)
{
  LinkRxEvent last = LINK_RX_NONE;
  for (size_t i = 0u; i < length; i++)
  {
    LinkRxEvent event = LinkRx_FeedByte(&rig->link, bytes[i], &rig->targets, rig->now_ms);
    if (event == LINK_RX_TEXT_BYTE) { rig->text_bytes++; }
    else if (event == LINK_RX_FRAME_START) { rig->frame_starts++; }
    else if (event != LINK_RX_NONE) { last = event; }
  }
  return last;
}

static size_t wire_of(Rig *rig, uint8_t type, const uint8_t *payload, size_t length, uint8_t *wire)
{
  return LinkFrame_Encode(type, rig->seq++, payload, length, wire, LINK_MAX_WIRE);
}

static LinkRxEvent send(Rig *rig, uint8_t type, const uint8_t *payload, size_t length)
{
  uint8_t wire[LINK_MAX_WIRE];
  size_t n = wire_of(rig, type, payload, length, wire);
  return feed(rig, wire, n);
}

static LinkRxEvent send_hello(Rig *rig)
{
  uint8_t version = LINK_PROTOCOL_VERSION;
  return send(rig, LINK_MSG_HELLO, &version, 1u);
}

static LinkRxEvent send_drive(Rig *rig, int32_t speed_cps, int32_t steer, uint32_t flags)
{
  uint8_t payload[LINK_DRIVE_LENGTH];
  (void)LinkProtocol_PackDrive(speed_cps, steer, flags, payload);
  return send(rig, LINK_MSG_DRIVE, payload, sizeof(payload));
}

static LinkRxEvent send_estop(Rig *rig)
{
  return send(rig, LINK_MSG_ESTOP, NULL, 0u);
}

/* One ms of the motor: as main.c does after the commands. */
static void tick(Rig *rig, uint32_t ms)
{
  for (uint32_t i = 0u; i < ms; i++)
  {
    rig->now_ms++;
    (void)MotorController_Update(&rig->motor, rig->now_ms);
    (void)SteeringController_Update(&rig->steering, rig->now_ms);
  }
}

/* In binary mode, armed, closed loop at the given speed. */
static void rig_running(Rig *rig, int32_t speed_cps)
{
  rig_init(rig);
  (void)send_hello(rig);
  (void)send_drive(rig, 0, 0, LINK_DRIVE_ARM);
  (void)send_drive(rig, speed_cps, 0, 0u);
  MotorController_SetClosedLoopTarget(&rig->motor, 200);  /* stands in for the PI's output */
  tick(rig, 5u);
}

static void test_text_mode(void)
{
  Rig rig;
  rig_init(&rig);
  const char *text = "m 20\r\n";
  CHECK(feed(&rig, (const uint8_t *)text, strlen(text)) == LINK_RX_NONE && rig.text_bytes == strlen(text),
        "TEXT mode: text bytes are handed back for the command line");
  CHECK(rig.link.text_ignored_count == 0u, "TEXT mode: text isn't counted as ignored");

  /* DRIVE and ESTOP before HELLO: rejected, nothing touched, watchdog not fed. */
  uint32_t fed_before = rig.motor.last_command_ms;
  uint32_t valid_before = rig.motor.valid_command_count;
  rig.now_ms += 100u;
  /* A DRIVE (9 COBS bytes) is longer than a HELLO: given up as a stray 0x00
   * before it is ever decoded. An ESTOP (5 bytes) is decoded and rejected. */
  uint32_t frame_errors = rig.link.frame_error_count;
  CHECK(send_drive(&rig, 0, 0, LINK_DRIVE_ARM) != LINK_RX_DRIVE && rig.link.frame_error_count == frame_errors + 1u &&
        rig.link.rejected_count == 0u, "DRIVE in TEXT mode: given up (frame error), never decoded");
  CHECK(send_estop(&rig) == LINK_RX_DROPPED && rig.link.rejected_count == 1u &&
        rig.link.last_reject == LINK_PARSE_NOT_BINARY_MODE, "ESTOP in TEXT mode: rejected (NOT_BINARY_MODE)");
  CHECK(rig.motor.state == MOTOR_STATE_LATCHED && rig.motor.last_command_ms == fed_before &&
        rig.motor.valid_command_count == valid_before && rig.speed.mode == SPEED_MODE_OPEN_LOOP,
        "nothing acted on, watchdog not fed");
  CHECK(rig.link.mode == LINK_MODE_TEXT, "still TEXT");
  CHECK(rig.frame_starts == 2u, "each frame's opening 0x00 tells the caller to drop a partial text line");

  /* After a frame, text again. */
  rig.text_bytes = 0u;
  CHECK(feed(&rig, (const uint8_t *)"s 5\n", 4u) == LINK_RX_NONE && rig.text_bytes == 4u, "text again after a frame");
}

static void test_stray_zero_in_text_mode(void)
{
  /* A stray 0x00 in the text (noise, a NUL typed in a terminal) costs at
   * most LINK_TEXT_MODE_MAX_COBS bytes; the text after them is text again,
   * without any further 0x00. */
  Rig rig;
  rig_init(&rig);
  CHECK(LINK_TEXT_MODE_MAX_COBS == 6u, "a HELLO is 6 COBS bytes");
  (void)feed(&rig, (const uint8_t *)"\x00", 1u);
  const char *text = "v 20\nv 20\nv 20\n";   /* 15 bytes: the first 6 go, the other 9 are text */
  (void)feed(&rig, (const uint8_t *)text, strlen(text));
  CHECK(!rig.link.in_frame && rig.link.mode == LINK_MODE_TEXT && rig.link.frame_error_count == 1u &&
        rig.text_bytes == strlen(text) - LINK_TEXT_MODE_MAX_COBS, "stray 0x00: 6 bytes lost, %u text bytes handed back",
        (unsigned)rig.text_bytes);
  /* Exactly 6 bytes and a 0x00 are still checked as a frame (a HELLO's size). */
  static const uint8_t six[] = { 0x00u, 0x41u, 0x42u, 0x43u, 0x44u, 0x45u, 0x46u, 0x00u };
  uint32_t crc_errors = rig.link.crc_error_count, cobs_errors = rig.link.frame_error_count;
  (void)feed(&rig, six, sizeof(six));
  CHECK(rig.link.crc_error_count + rig.link.frame_error_count == crc_errors + cobs_errors + 1u && !rig.link.in_frame,
        "6 bytes between two 0x00: decoded and dropped once");
  /* HELLO still works afterwards, and in binary mode long frames are fine. */
  CHECK(send_hello(&rig) == LINK_RX_HELLO && send_drive(&rig, 0, 0, LINK_DRIVE_ARM) == LINK_RX_DRIVE,
        "then HELLO and a DRIVE act as usual");
}

static void test_hello(void)
{
  /* From text mode with the motor running open loop ("m 30"). */
  Rig rig;
  rig_init(&rig);
  MotorController_AcceptCommand(&rig.motor, 0, rig.now_ms);
  MotorController_AcceptCommand(&rig.motor, 30, rig.now_ms);
  tick(&rig, 300u);
  CHECK(rig.motor.state == MOTOR_STATE_RUNNING, "setup: running on m 30");
  uint32_t fed_before = rig.motor.last_command_ms;
  CHECK(send_hello(&rig) == LINK_RX_HELLO && rig.link.mode == LINK_MODE_BINARY, "HELLO: binary mode, HELLO event");
  CHECK(rig.motor.state == MOTOR_STATE_LATCHED && rig.motor.latch_reason == MOTOR_LATCH_LINK_HELLO,
        "HELLO latched the running motor (LINK_HELLO)");
  CHECK(rig.motor.last_command_ms == fed_before, "HELLO doesn't feed the watchdog");
  MotorOutput output = MotorController_Update(&rig.motor, rig.now_ms + 1u);
  CHECK(!output.driver_enabled, "driver off at once");

  /* HELLO again in binary mode (a bridge restart): latches again, ACKed again. */
  (void)send_drive(&rig, 0, 0, LINK_DRIVE_ARM);
  CHECK(rig.motor.state == MOTOR_STATE_OFF, "armed");
  CHECK(send_hello(&rig) == LINK_RX_HELLO && rig.motor.state == MOTOR_STATE_LATCHED &&
        rig.motor.latch_reason == MOTOR_LATCH_LINK_HELLO, "HELLO in binary mode: HELLO event, latched again");

  /* Wrong version or length: rejected, stays in text mode. */
  Rig other;
  rig_init(&other);
  uint8_t bad_version = 2u;
  CHECK(send(&other, LINK_MSG_HELLO, &bad_version, 1u) == LINK_RX_DROPPED && other.link.mode == LINK_MODE_TEXT &&
        other.link.last_reject == LINK_PARSE_BAD_VERSION, "HELLO version 2: rejected, still TEXT");
  uint8_t long_hello[2] = { 1u, 0u };
  CHECK(send(&other, LINK_MSG_HELLO, long_hello, 2u) != LINK_RX_HELLO && other.link.mode == LINK_MODE_TEXT,
        "HELLO with 2 bytes (7 COBS bytes, longer than a HELLO): not acted on, still TEXT");
  /* In binary mode the same frame is decoded and rejected for its length. */
  Rig binary;
  rig_init(&binary);
  (void)send_hello(&binary);
  CHECK(send(&binary, LINK_MSG_HELLO, long_hello, 2u) == LINK_RX_DROPPED &&
        binary.link.last_reject == LINK_PARSE_BAD_LENGTH, "HELLO with 2 bytes in binary mode: BAD_LENGTH");

  /* In FAULT: binary mode, ACKed, still FAULT. */
  Rig fault;
  rig_init(&fault);
  MotorController_LockOut(&fault.motor, MOTOR_FAULT_LOW_BATTERY);
  CHECK(send_hello(&fault) == LINK_RX_HELLO && fault.motor.state == MOTOR_STATE_FAULT &&
        fault.motor.fault_reason == MOTOR_FAULT_LOW_BATTERY && fault.motor.latch_reason == MOTOR_LATCH_NONE,
        "HELLO in FAULT: still FAULT (LOW_BATTERY)");
}

static void test_drive_and_arm(void)
{
  Rig rig;
  rig_init(&rig);
  (void)send_hello(&rig);

  /* DRIVE 0 without arm: feeds the watchdog, stays latched. */
  rig.now_ms += 50u;
  CHECK(send_drive(&rig, 0, 0, 0u) == LINK_RX_DRIVE && rig.motor.state == MOTOR_STATE_LATCHED &&
        rig.motor.last_command_ms == rig.now_ms, "DRIVE 0 without arm: fed, still latched");
  /* A non-zero DRIVE without arm: same. */
  CHECK(send_drive(&rig, 2500, 0, 0u) == LINK_RX_DRIVE && rig.motor.state == MOTOR_STATE_LATCHED,
        "DRIVE 2500 while latched: still latched");
  /* arm with a speed: rejected as a whole (no feed, no setpoint). */
  rig.now_ms += 50u;
  uint32_t fed_before = rig.motor.last_command_ms;
  int32_t setpoint_before = rig.speed.setpoint_units;
  CHECK(send_drive(&rig, 2500, 0, LINK_DRIVE_ARM) == LINK_RX_DROPPED &&
        rig.link.last_reject == LINK_PARSE_ARM_WITH_SPEED && rig.motor.state == MOTOR_STATE_LATCHED &&
        rig.motor.last_command_ms == fed_before && rig.speed.setpoint_units == setpoint_before,
        "arm with speed 2500: rejected, not fed, setpoint unchanged");
  /* DRIVE 0 with arm: OFF. */
  CHECK(send_drive(&rig, 0, 0, LINK_DRIVE_ARM) == LINK_RX_DRIVE && rig.motor.state == MOTOR_STATE_OFF,
        "DRIVE 0 + arm: re-armed");

  /* The setpoint: cps -> units, closed loop. */
  CHECK(send_drive(&rig, 2500, 0, 0u) == LINK_RX_DRIVE && rig.speed.mode == SPEED_MODE_CLOSED_LOOP &&
        rig.speed.setpoint_units == 50 && rig.speed.effective_units == 50 && rig.link.drive_speed_cps == 2500,
        "DRIVE 2500: closed loop, 50 units");
  CHECK(send_drive(&rig, -999, 0, 0u) == LINK_RX_DRIVE && rig.speed.setpoint_units == 0, "DRIVE -999: 0 units");
  CHECK(send_drive(&rig, -1000, 0, 0u) == LINK_RX_DRIVE && rig.speed.setpoint_units == -20, "DRIVE -1000: -20 units");
  CHECK(rig.link.drive_seq_echo == (uint8_t)(rig.seq - 1u), "seq echo is the last DRIVE's");

  /* Out of range and reserved flags: rejected, setpoint unchanged. */
  CHECK(send_drive(&rig, 5001, 0, 0u) == LINK_RX_DROPPED && rig.speed.setpoint_units == -20, "DRIVE 5001: rejected");
  CHECK(send_drive(&rig, 1000, 0, 0x04u) == LINK_RX_DROPPED && rig.link.last_reject == LINK_PARSE_BAD_FLAGS,
        "reserved flag bit: rejected");
  CHECK(send_drive(&rig, 1000, 101, LINK_DRIVE_STEER_VALID) == LINK_RX_DROPPED, "steer 101: rejected");
}

static void test_steering(void)
{
  Rig rig;
  rig_init(&rig);
  (void)send_hello(&rig);
  CHECK(send_drive(&rig, 0, -40, 0u) == LINK_RX_DRIVE && rig.steering.valid_command_count == 0u &&
        rig.steering.state == STEERING_STATE_OFF, "steer_valid 0: steering untouched");
  CHECK(send_drive(&rig, 0, -40, LINK_DRIVE_STEER_VALID) == LINK_RX_DRIVE && rig.steering.valid_command_count == 1u &&
        rig.steering.requested_percent == -40, "steer_valid 1: steering -40");
  /* Steering keepalives only through steer_valid DRIVEs: without them it times out. */
  for (int i = 0; i < 10; i++)
  {
    (void)send_drive(&rig, 0, 30, 0u);
    tick(&rig, 100u);
  }
  CHECK(rig.steering.state == STEERING_STATE_TIMED_OUT, "DRIVEs without steer_valid don't feed the steering timeout");
}

static void test_estop(void)
{
  Rig rig;
  rig_running(&rig, 2500);
  CHECK(rig.motor.state == MOTOR_STATE_RUNNING, "setup: running");
  uint32_t fed_before = rig.motor.last_command_ms;
  CHECK(send_estop(&rig) == LINK_RX_ESTOP && rig.motor.state == MOTOR_STATE_LATCHED &&
        rig.motor.latch_reason == MOTOR_LATCH_ESTOP && rig.motor.last_command_ms == fed_before,
        "ESTOP: latched at once (ESTOP), watchdog not fed");
  /* ESTOP then plain DRIVE 0: stays latched (the bench test from the plan). */
  for (int i = 0; i < 20; i++)
  {
    (void)send_drive(&rig, 0, 0, 0u);
    tick(&rig, 50u);
  }
  CHECK(rig.motor.state == MOTOR_STATE_LATCHED && rig.motor.latch_reason == MOTOR_LATCH_ESTOP,
        "ESTOP, then 1 s of plain DRIVE 0: still latched (ESTOP)");
  CHECK(send_drive(&rig, 0, 0, LINK_DRIVE_ARM) == LINK_RX_DRIVE && rig.motor.state == MOTOR_STATE_OFF,
        "an arming DRIVE re-arms after ESTOP");
  uint8_t extra = 0u;
  CHECK(send(&rig, LINK_MSG_ESTOP, &extra, 1u) == LINK_RX_DROPPED && rig.motor.state == MOTOR_STATE_OFF,
        "ESTOP with a payload byte: rejected");
}

static void test_only_drive_feeds_the_motor_watchdog(void)
{
  /* Running, then 1 s of everything except DRIVE: the watchdog trips. */
  Rig rig;
  rig_running(&rig, 2500);
  for (int i = 0; i < 100; i++)
  {
    uint8_t wire[LINK_MAX_WIRE];
    uint8_t payload[LINK_DRIVE_LENGTH];
    (void)LinkProtocol_PackDrive(2500, 0, 0u, payload);
    size_t n = wire_of(&rig, LINK_MSG_DRIVE, payload, sizeof(payload), wire);
    wire[n / 2u] ^= 0x10u;                                  /* a DRIVE with a corrupted byte */
    (void)feed(&rig, wire, n);
    (void)send(&rig, 0x55u, payload, 2u);                     /* an unknown type */
    (void)send_drive(&rig, 9999, 0, 0u);                      /* out of range */
    (void)feed(&rig, (const uint8_t *)"v 50\n", 5u);          /* text, ignored in binary mode */
    (void)send(&rig, LINK_MSG_HELLO_ACK, payload, 1u);        /* a Nucleo -> Pi type */
    tick(&rig, 10u);
    if (i == 49) { CHECK(rig.motor.state == MOTOR_STATE_LATCHED, "tripped within the first 500 ms"); }
  }
  CHECK(rig.motor.state == MOTOR_STATE_LATCHED && rig.motor.latch_reason == MOTOR_LATCH_WATCHDOG &&
        rig.motor.watchdog_trip_count == 1u, "no valid DRIVE for 500 ms: watchdog latch");

  /* The same 1 s with a valid DRIVE every 100 ms: no trip. */
  Rig keep;
  rig_running(&keep, 2500);
  for (int i = 0; i < 100; i++)
  {
    if (i % 10 == 0) { (void)send_drive(&keep, 2500, 0, 0u); }
    MotorController_SetClosedLoopTarget(&keep.motor, 200);
    tick(&keep, 10u);
  }
  CHECK(keep.motor.state == MOTOR_STATE_RUNNING && keep.motor.watchdog_trip_count == 0u,
        "a DRIVE every 100 ms keeps it running");
}

static void test_dropped_frames_count_and_change_nothing(void)
{
  Rig rig;
  rig_init(&rig);
  (void)send_hello(&rig);
  (void)send_drive(&rig, 0, 0, LINK_DRIVE_ARM);

  uint8_t payload[LINK_DRIVE_LENGTH];
  (void)LinkProtocol_PackDrive(3000, 20, LINK_DRIVE_STEER_VALID, payload);
  uint8_t wire[LINK_MAX_WIRE];
  size_t n = wire_of(&rig, LINK_MSG_DRIVE, payload, sizeof(payload), wire);
  wire[3] ^= 0x01u;
  rig.now_ms += 10u;
  uint32_t fed = rig.motor.last_command_ms;
  CHECK(feed(&rig, wire, n) == LINK_RX_DROPPED && rig.link.crc_error_count == 1u, "bit flip: CRC error");
  CHECK(rig.speed.setpoint_units == 0 && rig.steering.valid_command_count == 0u && rig.motor.last_command_ms == fed,
        "nothing changed");

  /* Truncated: the frame's tail lost, then the next frame's opening 0x00. */
  n = wire_of(&rig, LINK_MSG_DRIVE, payload, sizeof(payload), wire);
  (void)feed(&rig, wire, n - 3u);
  CHECK(feed(&rig, (const uint8_t *)"\x00", 1u) != LINK_RX_DRIVE, "truncated frame: not acted on");
  CHECK(rig.speed.setpoint_units == 0, "truncated frame changed nothing");

  /* Overlong piece: bounded, dropped once, back in step. */
  uint8_t noise[300];
  memset(noise, 0x41, sizeof(noise));
  uint32_t frame_errors = rig.link.frame_error_count;
  (void)feed(&rig, (const uint8_t *)"\x00", 1u);
  (void)feed(&rig, noise, sizeof(noise));
  CHECK(feed(&rig, (const uint8_t *)"\x00", 1u) == LINK_RX_DROPPED && rig.link.frame_error_count >= frame_errors + 1u &&
        rig.link.length == 0u, "300 bytes without a 0x00: one frame error, buffer bounded");
  CHECK(send_drive(&rig, 3000, 20, LINK_DRIVE_STEER_VALID) == LINK_RX_DRIVE && rig.speed.setpoint_units == 60 &&
        rig.steering.requested_percent == 20, "the next good frame acts");

  /* Empty frames (0x00 0x00 ...) are not errors. */
  uint32_t errors_before = rig.link.frame_error_count + rig.link.crc_error_count + rig.link.rejected_count;
  static const uint8_t zeros[8] = {0};
  (void)feed(&rig, zeros, sizeof(zeros));
  CHECK(rig.link.frame_error_count + rig.link.crc_error_count + rig.link.rejected_count == errors_before,
        "runs of 0x00: no errors");
}

static void test_poison(void)
{
  Rig rig;
  rig_init(&rig);
  (void)send_hello(&rig);
  CHECK(!LinkRx_Poison(&rig.link), "outside a frame: poison is the caller's (text line)");
  uint8_t payload[LINK_DRIVE_LENGTH];
  (void)LinkProtocol_PackDrive(0, 0, LINK_DRIVE_ARM, payload);
  uint8_t wire[LINK_MAX_WIRE];
  size_t n = wire_of(&rig, LINK_MSG_DRIVE, payload, sizeof(payload), wire);
  (void)feed(&rig, wire, 4u);
  CHECK(LinkRx_Poison(&rig.link), "inside a frame: poisoned");
  CHECK(feed(&rig, &wire[4], n - 4u) == LINK_RX_DROPPED && rig.motor.state == MOTOR_STATE_LATCHED &&
        rig.link.frame_error_count == 1u, "a perfect frame with a UART error in it: dropped, not acted on");
  /* A poison between two delimiters with nothing in between goes away. */
  (void)feed(&rig, (const uint8_t *)"\x00", 1u);
  (void)LinkRx_Poison(&rig.link);
  (void)feed(&rig, (const uint8_t *)"\x00", 1u);
  n = wire_of(&rig, LINK_MSG_DRIVE, payload, sizeof(payload), wire);
  CHECK(feed(&rig, &wire[1], n - 1u) == LINK_RX_DRIVE && rig.motor.state == MOTOR_STATE_OFF,
        "after an empty poisoned frame, the next frame acts");
}

static void test_sequence_numbers(void)
{
  Rig rig;
  rig_init(&rig);
  rig.seq = 250u;
  (void)send_hello(&rig);
  for (int i = 0; i < 10; i++) { (void)send_drive(&rig, 0, 0, 0u); }   /* 251..255, 0..4: wraps */
  CHECK(rig.link.seq_gap_count == 0u && rig.link.duplicate_count == 0u, "255 -> 0 is not a gap");
  rig.seq += 5u;
  (void)send_drive(&rig, 0, 0, 0u);
  CHECK(rig.link.seq_gap_count == 1u, "a jump: one gap");
  rig.seq--;
  CHECK(send_drive(&rig, 0, 0, 0u) == LINK_RX_DRIVE && rig.link.duplicate_count == 1u && rig.link.seq_gap_count == 1u,
        "the same seq again: a duplicate, still acted on (count only)");
  /* A frame that fails the CRC doesn't move the seq. */
  uint8_t wire[LINK_MAX_WIRE];
  uint8_t payload[LINK_DRIVE_LENGTH];
  (void)LinkProtocol_PackDrive(0, 0, 0u, payload);
  size_t n = wire_of(&rig, LINK_MSG_DRIVE, payload, sizeof(payload), wire);
  wire[2] ^= 0x80u;
  (void)feed(&rig, wire, n);
  (void)send_drive(&rig, 0, 0, 0u);
  CHECK(rig.link.seq_gap_count == 2u, "a lost frame shows as a gap at the next good one");
  CHECK(rig.link.ok_count == 1u + 10u + 1u + 1u + 1u, "ok_count %u", (unsigned)rig.link.ok_count);
}

static void test_text_in_binary_mode(void)
{
  Rig rig;
  rig_init(&rig);
  (void)send_hello(&rig);
  rig.text_bytes = 0u;
  const char *lines = "m 20\nm 20\r\n";
  CHECK(feed(&rig, (const uint8_t *)lines, strlen(lines)) == LINK_RX_NONE && rig.link.text_ignored_count == 2u &&
        rig.text_bytes == strlen(lines), "two text lines in binary mode: 2 ignored (bytes handed back for test builds)");
  CHECK(rig.motor.state == MOTOR_STATE_LATCHED && rig.motor.valid_command_count == 0u, "text moved nothing");
  /* Text without a newline, then a frame: one line counted, the frame acts. */
  (void)feed(&rig, (const uint8_t *)"garbage", 7u);
  CHECK(send_drive(&rig, 0, 0, LINK_DRIVE_ARM) == LINK_RX_DRIVE && rig.link.text_ignored_count == 3u,
        "text then a frame: the frame acts");
}

static void test_text_command_gate(void)
{
  /* Every mode x every letter (and the '\0' of a rejected line) x both
   * switches, against the rule written out once more. */
  int mismatches = 0, allowed_count = 0;
  for (int mode = LINK_MODE_TEXT; mode <= LINK_MODE_BINARY; mode++)
  {
    for (int value = 0; value < 128; value++)
    {
      for (int bench = 0; bench <= 1; bench++)
      {
        for (int test_build = 0; test_build <= 1; test_build++)
        {
          char letter = (char)value;
          bool expected = (mode == LINK_MODE_TEXT)
                            ? (bench || test_build)
                            : (test_build && (letter == 'e' || letter == 'c' || letter == 'h'));
          bool allowed = LinkRx_TextCommandAllowed((LinkMode)mode, letter, bench != 0, test_build != 0);
          if (allowed != expected) { mismatches++; }
          if (allowed) { allowed_count++; }
        }
      }
    }
  }
  /* Allowed: TEXT, any letter, in 3 of the 4 switch combinations (128 x 3);
   * BINARY, e/c/h with test_build, bench either way (3 x 2). */
  CHECK(mismatches == 0 && allowed_count == 128 * 3 + 3 * 2, "gate: every mode, letter and switch (%d mismatches, %d allowed)",
        mismatches, allowed_count);
  /* The cases that matter, spelled out. */
  CHECK(!LinkRx_TextCommandAllowed(LINK_MODE_TEXT, 'm', false, false), "truck build, TEXT: m ignored");
  CHECK(!LinkRx_TextCommandAllowed(LINK_MODE_TEXT, 'v', false, false), "truck build, TEXT: v ignored");
  CHECK(!LinkRx_TextCommandAllowed(LINK_MODE_TEXT, 's', false, false), "truck build, TEXT: s ignored");
  CHECK(!LinkRx_TextCommandAllowed(LINK_MODE_TEXT, '\0', false, false), "truck build, TEXT: a rejected line only counts");
  CHECK(LinkRx_TextCommandAllowed(LINK_MODE_TEXT, 'm', true, false), "bench build, TEXT: m acts");
  CHECK(LinkRx_TextCommandAllowed(LINK_MODE_TEXT, 'm', false, true), "test build, TEXT: m acts (capture steps)");
  CHECK(!LinkRx_TextCommandAllowed(LINK_MODE_BINARY, 'm', true, true), "any build, BINARY: m never acts");
  CHECK(!LinkRx_TextCommandAllowed(LINK_MODE_BINARY, 'v', true, true) &&
        !LinkRx_TextCommandAllowed(LINK_MODE_BINARY, 's', true, true), "any build, BINARY: v and s never act");
  CHECK(!LinkRx_TextCommandAllowed(LINK_MODE_BINARY, 'e', true, false), "bench (not test) build, BINARY: e doesn't act");
  CHECK(LinkRx_TextCommandAllowed(LINK_MODE_BINARY, 'h', false, true), "test build, BINARY: h acts");
}

int main(void)
{
  test_text_command_gate();
  test_text_mode();
  test_stray_zero_in_text_mode();
  test_hello();
  test_drive_and_arm();
  test_steering();
  test_estop();
  test_only_drive_feeds_the_motor_watchdog();
  test_dropped_frames_count_and_change_nothing();
  test_poison();
  test_sequence_numbers();
  test_text_in_binary_mode();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
