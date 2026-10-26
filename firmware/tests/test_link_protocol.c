/*
 * Host-side test for Core/Src/link_protocol.c: the cps -> units rule, the
 * checks on HELLO / DRIVE / ESTOP payloads, and the packing of HELLO_ACK,
 * STATE and STATUS, against tests/link_message_vectors.txt (generated from
 * Python struct; tools/test_rover_link.py reads the same file). Not part of
 * the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_link_protocol.c Core/Src/link_protocol.c Core/Src/link_codec.c -o /tmp/test_link_protocol && /tmp/test_link_protocol
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link_protocol.h"
#include "tx_queue.h"

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

#define VECTOR_FILE  "tests/link_message_vectors.txt"
#define MAX_FIELDS   64

static int parse_hex(const char *text, uint8_t *out, size_t capacity)
{
  if (strcmp(text, "-") == 0)
  {
    return 0;
  }
  size_t length = strlen(text);
  if (length % 2u != 0u || length / 2u > capacity)
  {
    return -1;
  }
  for (size_t i = 0u; i < length / 2u; i++)
  {
    unsigned int value;
    if (sscanf(&text[2u * i], "%2x", &value) != 1)
    {
      return -1;
    }
    out[i] = (uint8_t)value;
  }
  return (int)(length / 2u);
}

static const char *parse_result_name(LinkParseResult result)
{
  switch (result)
  {
    case LINK_PARSE_OK:              return "OK";
    case LINK_PARSE_BAD_LENGTH:      return "BAD_LENGTH";
    case LINK_PARSE_BAD_VERSION:     return "BAD_VERSION";
    case LINK_PARSE_BAD_FLAGS:       return "BAD_FLAGS";
    case LINK_PARSE_OUT_OF_RANGE:    return "OUT_OF_RANGE";
    case LINK_PARSE_ARM_WITH_SPEED:  return "ARM_WITH_SPEED";
    case LINK_PARSE_UNKNOWN_TYPE:    return "UNKNOWN_TYPE";
    case LINK_PARSE_NOT_BINARY_MODE: return "NOT_BINARY_MODE";
    default:                         return "?";
  }
}

static LinkFrame frame_with(uint8_t type, const uint8_t *payload, size_t length)
{
  LinkFrame frame;
  memset(&frame, 0, sizeof(frame));
  frame.type = type;
  frame.payload_length = length;
  if (length > 0u)
  {
    memcpy(frame.payload, payload, length);
  }
  return frame;
}

/* Splits a vector line into whitespace-separated tokens. */
static int tokenize(char *line, char **tokens, int capacity)
{
  int count = 0;
  for (char *token = strtok(line, " \t\r\n"); token != NULL && count < capacity; token = strtok(NULL, " \t\r\n"))
  {
    tokens[count++] = token;
  }
  return count;
}

static void check_payload(const char *what, const uint8_t *packed, size_t packed_length, const char *expected_hex)
{
  uint8_t expected[LINK_MAX_PAYLOAD];
  int n = parse_hex(expected_hex, expected, sizeof(expected));
  CHECK(n >= 0 && (size_t)n == packed_length && memcmp(packed, expected, packed_length) == 0,
        "%s: packed %u bytes, expected %d, bytes %s", what, (unsigned)packed_length, n,
        (n >= 0 && (size_t)n == packed_length && memcmp(packed, expected, packed_length) == 0) ? "equal" : "differ");
}

static void test_vectors(void)
{
  FILE *file = fopen(VECTOR_FILE, "r");
  CHECK(file != NULL, "open %s (run from firmware/)", VECTOR_FILE);
  if (file == NULL)
  {
    return;
  }
  char line[4096];
  int counts[5] = {0, 0, 0, 0, 0};  /* units, drive, hello_ack, state, status */
  while (fgets(line, sizeof(line), file) != NULL)
  {
    if (line[0] == '#' || line[0] == '\n')
    {
      continue;
    }
    char *t[MAX_FIELDS];
    int n = tokenize(line, t, MAX_FIELDS);
    uint8_t packed[LINK_MAX_PAYLOAD];
    if (strcmp(t[0], "units") == 0 && n == 3)
    {
      int32_t cps = (int32_t)strtol(t[1], NULL, 10);
      int32_t expected = (int32_t)strtol(t[2], NULL, 10);
      CHECK(LinkProtocol_CpsToUnits(cps) == expected, "%d cps -> %d units, got %d", (int)cps, (int)expected,
            (int)LinkProtocol_CpsToUnits(cps));
      counts[0]++;
    }
    else if (strcmp(t[0], "drive") == 0 && (n == 3 || n == 7))
    {
      uint8_t payload[16];
      int length = parse_hex(t[1], payload, sizeof(payload));
      LinkFrame frame = frame_with(LINK_MSG_DRIVE, payload, (size_t)length);
      LinkDrive drive = { .speed_cps = 12345, .steer_percent = 77, .steer_valid = true, .arm = true };
      LinkParseResult result = LinkProtocol_ParseDrive(&frame, &drive);
      CHECK(strcmp(parse_result_name(result), t[2]) == 0, "DRIVE %s: expected %s, got %s", t[1], t[2],
            parse_result_name(result));
      if (n == 7)
      {
        CHECK(drive.speed_cps == strtol(t[3], NULL, 10) && drive.steer_percent == strtol(t[4], NULL, 10) &&
              drive.steer_valid == (strtol(t[5], NULL, 10) != 0) && drive.arm == (strtol(t[6], NULL, 10) != 0),
              "DRIVE %s fields", t[1]);
        uint32_t flags = (drive.steer_valid ? LINK_DRIVE_STEER_VALID : 0u) | (drive.arm ? LINK_DRIVE_ARM : 0u);
        size_t packed_length = LinkProtocol_PackDrive(drive.speed_cps, drive.steer_percent, flags, packed);
        check_payload("DRIVE pack", packed, packed_length, t[1]);
      }
      else
      {
        CHECK(drive.speed_cps == 12345 && drive.steer_percent == 77 && drive.steer_valid && drive.arm,
              "DRIVE %s rejected: output untouched", t[1]);
      }
      counts[1]++;
    }
    else if (strcmp(t[0], "hello_ack") == 0 && n == 8)
    {
      LinkHelloAckMessage message = {
        .build_flags = (uint32_t)strtoul(t[1], NULL, 10), .reset_flags = (uint32_t)strtoul(t[2], NULL, 10),
        .imu_ok = strtoul(t[3], NULL, 10) != 0u, .imu_who = (uint32_t)strtoul(t[4], NULL, 10),
        .state_period_ms = (uint32_t)strtoul(t[5], NULL, 10), .status_period_ms = (uint32_t)strtoul(t[6], NULL, 10) };
      size_t packed_length = LinkProtocol_PackHelloAck(&message, packed);
      CHECK(packed_length == LINK_HELLO_ACK_LENGTH, "HELLO_ACK length %u", (unsigned)packed_length);
      check_payload("HELLO_ACK", packed, packed_length, t[7]);
      counts[2]++;
    }
    else if (strcmp(t[0], "state") == 0 && n == 19)
    {
      long long v[17];
      for (int i = 0; i < 17; i++) { v[i] = strtoll(t[1 + i], NULL, 10); }
      LinkStateMessage message = {
        .t_ms = (uint32_t)v[0], .count = (int32_t)v[1], .speed_cps = (int32_t)v[2], .enc_flags = (uint32_t)v[3],
        .odometer_epoch = (uint32_t)v[4], .gyro_sum = { (int32_t)v[5], (int32_t)v[6], (int32_t)v[7] },
        .accel_sum = { (int32_t)v[8], (int32_t)v[9], (int32_t)v[10] }, .imu_samples = (uint32_t)v[11],
        .imu_flags = (uint32_t)v[12], .duty_ticks = (int32_t)v[13], .motor_state = (uint32_t)v[14],
        .control_state = (uint32_t)v[15], .drive_seq_echo = (uint32_t)v[16] };
      size_t packed_length = LinkProtocol_PackState(&message, packed);
      CHECK(packed_length == LINK_STATE_LENGTH, "STATE length %u", (unsigned)packed_length);
      check_payload("STATE", packed, packed_length, t[18]);
      counts[3]++;
    }
    else if (strcmp(t[0], "status") == 0 && n == 45)
    {
      long long v[43];
      for (int i = 0; i < 43; i++) { v[i] = strtoll(t[1 + i], NULL, 10); }
      LinkStatusMessage message = {
        .t_ms = (uint32_t)v[0], .setpoint_cps = (int32_t)v[1], .integrator_ticks = (int32_t)v[2],
        .steer_percent = (int32_t)v[3], .steer_pulse_us = (uint32_t)v[4], .steer_state = (uint32_t)v[5],
        .motor_state = (uint32_t)v[6], .control_state = (uint32_t)v[7], .fault_reason = (uint32_t)v[8],
        .latch_reason = (uint32_t)v[9], .battery_state = (uint32_t)v[10], .imu_ok = v[11] != 0,
        .mag_flags = (uint32_t)v[12], .agc = (uint32_t)v[13], .reset_flags = (uint32_t)v[14],
        .vbat_mv = (uint32_t)v[15], .vdda_mv = (uint32_t)v[16], .imu_temperature = (int32_t)v[17],
        .magnitude_min = (uint32_t)v[18], .magnitude_max = (uint32_t)v[19], .loop_us_max = (uint32_t)v[20],
        .i2c_us_max = (uint32_t)v[21], .spi_us_max = (uint32_t)v[22] };
      for (int i = 0; i < (int)LINK_STATUS_COUNTERS; i++) { message.counters[i] = (uint32_t)v[23 + i]; }
      size_t packed_length = LinkProtocol_PackStatus(&message, packed);
      CHECK(packed_length == LINK_STATUS_LENGTH, "STATUS length %u", (unsigned)packed_length);
      check_payload("STATUS", packed, packed_length, t[44]);
      counts[4]++;
    }
    else
    {
      CHECK(false, "unknown vector line starting %s (%d tokens)", t[0], n);
    }
  }
  fclose(file);
  CHECK(counts[0] == 37 && counts[1] == 19 && counts[2] == 3 && counts[3] == 3 && counts[4] == 2,
        "vector counts units %d drive %d hello_ack %d state %d status %d", counts[0], counts[1], counts[2],
        counts[3], counts[4]);
}

static void test_cps_to_units(void)
{
  /* The cases the protocol pins, read straight from the rule. */
  static const struct { int32_t cps; int32_t units; } cases[] = {
    { 24, 0 }, { 25, 0 }, { 26, 0 }, { 976, 0 },  /* below the minimum */
    { 999, 0 }, { 1000, 20 }, { 1001, 20 },       /* the minimum is on the raw cps: 999 -> 0 */
    { 1024, 20 }, { 1025, 20 }, { 1026, 21 },     /* 20.5 ties toward zero */
    { 1075, 21 }, { 1076, 22 },
    { 2500, 50 }, { 5000, 100 },
  };
  for (size_t i = 0u; i < sizeof(cases) / sizeof(cases[0]); i++)
  {
    CHECK(LinkProtocol_CpsToUnits(cases[i].cps) == cases[i].units && LinkProtocol_CpsToUnits(-cases[i].cps) == -cases[i].units,
          "+-%d cps -> +-%d units (got %d, %d)", (int)cases[i].cps, (int)cases[i].units,
          (int)LinkProtocol_CpsToUnits(cases[i].cps), (int)LinkProtocol_CpsToUnits(-cases[i].cps));
  }
  /* Every value in range against a floating-point reference: the minimum on
   * the raw cps, then nearest, ties toward zero; and odd symmetry. */
  int mismatches = 0;
  for (int32_t cps = -LINK_MAX_SPEED_CPS; cps <= LINK_MAX_SPEED_CPS; cps++)
  {
    double exact = fabs((double)cps) / 50.0;
    double rounded = (exact - floor(exact) > 0.5) ? ceil(exact) : floor(exact);
    int32_t reference = (fabs((double)cps) < 1000.0) ? 0 : (int32_t)rounded;
    if (cps < 0) { reference = -reference; }
    if (LinkProtocol_CpsToUnits(cps) != reference || LinkProtocol_CpsToUnits(-cps) != -LinkProtocol_CpsToUnits(cps))
    {
      mismatches++;
    }
  }
  CHECK(mismatches == 0, "every cps in +-5000 matches the reference and is sign-symmetric (%d mismatches)", mismatches);
}

static void test_parse(void)
{
  uint8_t version = LINK_PROTOCOL_VERSION;
  LinkFrame frame = frame_with(LINK_MSG_HELLO, &version, 1u);
  CHECK(LinkProtocol_ParseHello(&frame) == LINK_PARSE_OK, "HELLO 1: OK");
  uint8_t other[2] = { 2u, 0u };
  frame = frame_with(LINK_MSG_HELLO, other, 1u);
  CHECK(LinkProtocol_ParseHello(&frame) == LINK_PARSE_BAD_VERSION, "HELLO 2: BAD_VERSION");
  other[0] = 0u;
  frame = frame_with(LINK_MSG_HELLO, other, 1u);
  CHECK(LinkProtocol_ParseHello(&frame) == LINK_PARSE_BAD_VERSION, "HELLO 0: BAD_VERSION");
  frame = frame_with(LINK_MSG_HELLO, other, 0u);
  CHECK(LinkProtocol_ParseHello(&frame) == LINK_PARSE_BAD_LENGTH, "HELLO, no payload: BAD_LENGTH");
  other[0] = 1u;
  frame = frame_with(LINK_MSG_HELLO, other, 2u);
  CHECK(LinkProtocol_ParseHello(&frame) == LINK_PARSE_BAD_LENGTH, "HELLO, 2 bytes: BAD_LENGTH");
  frame = frame_with(LINK_MSG_ESTOP, NULL, 0u);
  CHECK(LinkProtocol_ParseEstop(&frame) == LINK_PARSE_OK, "ESTOP: OK");
  frame = frame_with(LINK_MSG_ESTOP, other, 1u);
  CHECK(LinkProtocol_ParseEstop(&frame) == LINK_PARSE_BAD_LENGTH, "ESTOP with a byte: BAD_LENGTH");

  /* Every flag byte: only 0..3 pass, and arm only with speed 0. */
  int flag_failures = 0;
  for (uint32_t flags = 0u; flags < 256u; flags++)
  {
    for (int speed = -1; speed <= 1; speed++)
    {
      uint8_t payload[4];
      (void)LinkProtocol_PackDrive(speed * 1000, 10, flags, payload);
      frame = frame_with(LINK_MSG_DRIVE, payload, 4u);
      LinkDrive drive;
      LinkParseResult result = LinkProtocol_ParseDrive(&frame, &drive);
      LinkParseResult expected = (flags & LINK_DRIVE_RESERVED) ? LINK_PARSE_BAD_FLAGS
                               : ((flags & LINK_DRIVE_ARM) && speed != 0) ? LINK_PARSE_ARM_WITH_SPEED
                               : LINK_PARSE_OK;
      if (result != expected) { flag_failures++; }
    }
  }
  CHECK(flag_failures == 0, "every DRIVE flag byte x speed -1000/0/1000 (%d failures)", flag_failures);

  /* The whole i16 speed range and the whole i8 steering range. */
  int range_failures = 0;
  for (int32_t speed = INT16_MIN; speed <= INT16_MAX; speed++)
  {
    uint8_t payload[4];
    (void)LinkProtocol_PackDrive(speed, 0, 0u, payload);
    frame = frame_with(LINK_MSG_DRIVE, payload, 4u);
    LinkDrive drive;
    bool in_range = (speed >= -LINK_MAX_SPEED_CPS && speed <= LINK_MAX_SPEED_CPS);
    if ((LinkProtocol_ParseDrive(&frame, &drive) == LINK_PARSE_OK) != in_range) { range_failures++; }
  }
  for (int32_t steer = INT8_MIN; steer <= INT8_MAX; steer++)
  {
    uint8_t payload[4];
    (void)LinkProtocol_PackDrive(0, steer, LINK_DRIVE_STEER_VALID, payload);
    frame = frame_with(LINK_MSG_DRIVE, payload, 4u);
    LinkDrive drive;
    bool in_range = (steer >= -100 && steer <= 100);
    if ((LinkProtocol_ParseDrive(&frame, &drive) == LINK_PARSE_OK) != in_range) { range_failures++; }
  }
  CHECK(range_failures == 0, "DRIVE accepts exactly |speed| <= 5000 and |steer| <= 100 (%d failures)", range_failures);
}

static void test_constants_and_saturation(void)
{
  CHECK(LINK_PROTOCOL_VERSION == 1u && LINK_MSG_HELLO == 0x01u && LINK_MSG_DRIVE == 0x10u && LINK_MSG_ESTOP == 0x11u &&
        LINK_MSG_HELLO_ACK == 0x81u && LINK_MSG_STATE == 0x82u && LINK_MSG_STATUS == 0x83u, "types pinned");
  CHECK(LINK_CPS_PER_UNIT == 50 && LINK_MIN_SPEED_CPS == 1000 && LINK_MAX_SPEED_CPS == 5000,
        "50 cps per unit, 1000 min, 5000 max");
  CHECK(LINK_STATE_LENGTH + 7u <= TX_QUEUE_SLOT_BYTES && LINK_STATUS_LENGTH + 7u <= TX_QUEUE_SLOT_BYTES,
        "STATE (%u) and STATUS (%u) fit a 128-byte transmit slot", LINK_STATE_LENGTH + 7u, LINK_STATUS_LENGTH + 7u);

  /* Saturation, field by field where the type is narrower than the struct. */
  LinkStatusMessage status;
  memset(&status, 0, sizeof(status));
  status.vdda_mv = 5100000u;
  status.imu_temperature = -40000;
  status.setpoint_cps = 32768;
  status.steer_percent = -129;
  uint8_t out[LINK_MAX_PAYLOAD];
  (void)LinkProtocol_PackStatus(&status, out);
  CHECK(out[23] == 0xFFu && out[24] == 0xFFu, "vdda 5,100,000 -> 65535");
  CHECK(out[25] == 0x00u && out[26] == 0x80u, "temp -40000 -> -32768");
  CHECK(out[4] == 0xFFu && out[5] == 0x7Fu, "setpoint 32768 -> 32767");
  CHECK(out[8] == 0x80u, "steer -129 -> -128");
  LinkStateMessage state;
  memset(&state, 0, sizeof(state));
  state.odometer_epoch = 257u;
  state.drive_seq_echo = 258u;
  state.imu_samples = 300u;
  state.motor_state = 3u;
  state.control_state = 2u;
  (void)LinkProtocol_PackState(&state, out);
  CHECK(out[11] == 1u && out[41] == 2u, "epoch and seq echo wrap (counters), not saturate");
  CHECK(out[36] == 255u, "imu_samples 300 -> 255");
  CHECK(out[40] == 0x23u, "mstate 3 in the low nibble, ctl 2 in the high one");
}

int main(void)
{
  test_vectors();
  test_cps_to_units();
  test_parse();
  test_constants_and_saturation();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
