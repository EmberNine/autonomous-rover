/*
 * Host-side test for Core/Src/telemetry_format.c. Not part of the firmware
 * build or CI. It compiles the formatter with the PC's C compiler and checks
 * that its output is byte-for-byte what snprintf produces.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_telemetry_format.c Core/Src/telemetry_format.c -o /tmp/test_telemetry_format && /tmp/test_telemetry_format
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "telemetry_format.h"

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

/* ---- Single values against snprintf ------------------------------------- */

static void check_unsigned_value(uint32_t value)
{
  char expected[64];
  char actual[64];
  snprintf(expected, sizeof(expected), "k=%lu", (unsigned long)value);

  TelemetryWriter writer;
  TelemetryWriter_Init(&writer, actual, sizeof(actual));
  TelemetryWriter_AppendUnsignedField(&writer, "k", value);

  CHECK(!writer.truncated && strcmp(expected, actual) == 0 && writer.length == strlen(expected),
        "unsigned %" PRIu32 ": expected '%s', got '%s'", value, expected, actual);
}

static void check_signed_value(int32_t value)
{
  char expected[64];
  char actual[64];
  snprintf(expected, sizeof(expected), "k=%ld", (long)value);

  TelemetryWriter writer;
  TelemetryWriter_Init(&writer, actual, sizeof(actual));
  TelemetryWriter_AppendSignedField(&writer, "k", value);

  CHECK(!writer.truncated && strcmp(expected, actual) == 0 && writer.length == strlen(expected),
        "signed %" PRId32 ": expected '%s', got '%s'", value, expected, actual);
}

/* Deterministic pseudo-random numbers (xorshift32), so a failure reproduces. */
static uint32_t random_state = 0x12345678u;
static uint32_t next_random(void)
{
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

static void test_values(void)
{
  int32_t signed_edges[] = { 0, 1, -1, 9, -9, 10, -10, 99, -99, 100, -100,
                             INT16_MIN, INT16_MAX, INT32_MIN, INT32_MIN + 1, INT32_MAX, INT32_MAX - 1 };
  for (size_t i = 0; i < sizeof(signed_edges) / sizeof(signed_edges[0]); i++)
  {
    check_signed_value(signed_edges[i]);
  }

  uint32_t unsigned_edges[] = { 0u, 1u, 9u, 10u, 99u, 100u, 255u, 4095u, 65535u,
                                (uint32_t)INT32_MAX, (uint32_t)INT32_MAX + 1u, UINT32_MAX - 1u, UINT32_MAX };
  for (size_t i = 0; i < sizeof(unsigned_edges) / sizeof(unsigned_edges[0]); i++)
  {
    check_unsigned_value(unsigned_edges[i]);
  }

  /* Every power of ten, and one either side of it, up to the type limits. */
  for (uint64_t power = 1u; power <= UINT32_MAX; power *= 10u)
  {
    check_unsigned_value((uint32_t)power);
    check_unsigned_value((uint32_t)(power - 1u));
    if (power + 1u <= UINT32_MAX) { check_unsigned_value((uint32_t)(power + 1u)); }

    if (power <= (uint64_t)INT32_MAX)
    {
      int32_t signed_power = (int32_t)power;
      check_signed_value(signed_power);
      check_signed_value(-signed_power);
      check_signed_value(signed_power - 1);
      check_signed_value(-(signed_power - 1));
      if (power + 1u <= (uint64_t)INT32_MAX)
      {
        check_signed_value(signed_power + 1);
        check_signed_value(-(signed_power + 1));
      }
    }
  }

  /* Random values. Shifting right by a random amount gives every digit count
   * a fair share, instead of almost all values being 10 digits long. */
  for (int i = 0; i < 5000; i++)
  {
    uint32_t bits = next_random() >> (next_random() % 32u);
    check_unsigned_value(bits);
    check_signed_value((int32_t)bits);
    check_signed_value(-(int32_t)(bits >> 1));
  }
}

/* ---- Separators and literal text ---------------------------------------- */

static void test_separators_and_text(void)
{
  char buffer[64];
  TelemetryWriter writer;
  TelemetryWriter_Init(&writer, buffer, sizeof(buffer));
  CHECK(writer.length == 0 && buffer[0] == '\0', "init should leave an empty string");

  TelemetryWriter_AppendUnsignedField(&writer, "a", 1u);
  TelemetryWriter_AppendSignedField(&writer, "b", -2);
  TelemetryWriter_AppendText(&writer, "\r\n");
  CHECK(strcmp(buffer, "a=1 b=-2\r\n") == 0, "got '%s'", buffer);

  /* Empty text is a no-op and isn't truncation. */
  TelemetryWriter_AppendText(&writer, "");
  CHECK(!writer.truncated && strcmp(buffer, "a=1 b=-2\r\n") == 0, "empty text changed the line");
}

/* ---- Truncation and bounds ---------------------------------------------- */

#define GUARD_BYTE  ((char)0xA5)
#define GUARD_SIZE  16

/* Builds a fixed sequence of pieces into a buffer of the given capacity and
 * records the line length after each piece. With a large enough capacity
 * this also gives the reference line and the piece boundaries. */
static size_t build_sample_pieces(TelemetryWriter *writer, size_t *lengths_after_piece)
{
  size_t piece = 0;
  TelemetryWriter_AppendUnsignedField(writer, "t_ms", 65400u);      lengths_after_piece[piece++] = writer->length;
  TelemetryWriter_AppendSignedField(writer, "d100", -1);            lengths_after_piece[piece++] = writer->length;
  TelemetryWriter_AppendSignedField(writer, "count", INT32_MIN);    lengths_after_piece[piece++] = writer->length;
  TelemetryWriter_AppendUnsignedField(writer, "x", UINT32_MAX);     lengths_after_piece[piece++] = writer->length;
  TelemetryWriter_AppendUnsignedField(writer, "y", 7u);             lengths_after_piece[piece++] = writer->length;
  TelemetryWriter_AppendText(writer, "\r\n");                       lengths_after_piece[piece++] = writer->length;
  return piece;
}

static void test_truncation(void)
{
  /* Reference: everything fits. */
  char full_line[128];
  size_t full_boundaries[16];
  TelemetryWriter full_writer;
  TelemetryWriter_Init(&full_writer, full_line, sizeof(full_line));
  size_t piece_count = build_sample_pieces(&full_writer, full_boundaries);
  CHECK(!full_writer.truncated, "reference line should fit");
  CHECK(strcmp(full_line, "t_ms=65400 d100=-1 count=-2147483648 x=4294967295 y=7\r\n") == 0,
        "reference line was '%s'", full_line);

  /* Every capacity from 0 to a few bytes past the full length. */
  for (size_t capacity = 0; capacity <= full_writer.length + 3u; capacity++)
  {
    char storage[128 + GUARD_SIZE];
    memset(storage, GUARD_BYTE, sizeof(storage));

    TelemetryWriter writer;
    size_t boundaries[16];
    TelemetryWriter_Init(&writer, storage, capacity);
    build_sample_pieces(&writer, boundaries);

    /* Nothing written past the buffer. */
    int guard_intact = 1;
    for (size_t i = capacity; i < capacity + GUARD_SIZE; i++)
    {
      if (storage[i] != GUARD_BYTE) { guard_intact = 0; }
    }
    CHECK(guard_intact, "capacity %zu: wrote past the end of the buffer", capacity);

    /* The result is the longest run of whole pieces that fits. */
    size_t expected_length = 0;
    for (size_t piece = 0; piece < piece_count; piece++)
    {
      if (full_boundaries[piece] + 1u <= capacity) { expected_length = full_boundaries[piece]; }
      else { break; }
    }
    int expect_truncated = (full_writer.length + 1u > capacity);

    CHECK(writer.truncated == expect_truncated, "capacity %zu: truncated=%d, expected %d",
          capacity, writer.truncated, expect_truncated);
    CHECK(writer.length == expected_length, "capacity %zu: length %zu, expected %zu",
          capacity, writer.length, expected_length);
    if (capacity > 0u)
    {
      CHECK(storage[writer.length] == '\0', "capacity %zu: not NUL-terminated", capacity);
      CHECK(memcmp(storage, full_line, writer.length) == 0, "capacity %zu: content isn't a prefix", capacity);
    }
    else
    {
      CHECK(storage[0] == GUARD_BYTE, "capacity 0: buffer was touched");
    }
  }

  /* Exact fit: "ab" fills a 3-byte buffer (2 chars + NUL); "abc" doesn't. */
  char small[3 + GUARD_SIZE];
  memset(small, GUARD_BYTE, sizeof(small));
  TelemetryWriter writer;
  TelemetryWriter_Init(&writer, small, 3);
  TelemetryWriter_AppendText(&writer, "ab");
  CHECK(!writer.truncated && strcmp(small, "ab") == 0, "exact fit failed");
  TelemetryWriter_Init(&writer, small, 3);
  TelemetryWriter_AppendText(&writer, "abc");
  CHECK(writer.truncated && writer.length == 0 && small[0] == '\0', "one-too-many should truncate");
  CHECK(small[3] == GUARD_BYTE, "wrote past a 3-byte buffer");

  /* Sticky: after a piece fails, a later piece that would fit is ignored. */
  char sticky[16];
  TelemetryWriter_Init(&writer, sticky, sizeof(sticky));
  TelemetryWriter_AppendUnsignedField(&writer, "a", 1u);                 /* "a=1", 3 chars */
  TelemetryWriter_AppendUnsignedField(&writer, "long_key", UINT32_MAX);  /* doesn't fit */
  TelemetryWriter_AppendUnsignedField(&writer, "b", 2u);                 /* would fit, ignored */
  CHECK(writer.truncated && strcmp(sticky, "a=1") == 0, "sticky truncation broken: '%s'", sticky);
}

/* ---- Whole telemetry line, old snprintf code vs the formatter ----------- */

/* Every field of one telemetry line, typed exactly as in main.c. */
typedef struct
{
  uint32_t t_ms; uint16_t raw; int32_t count; int32_t d100;
  uint8_t md; uint8_t ml; uint8_t mh; uint8_t agc;
  uint16_t mag_min; uint16_t mag_max;
  uint32_t i2c_err; uint32_t overrun; uint32_t resync; uint32_t skipped;
  uint32_t i2c_us_max; uint32_t loop_us_max;
  _Bool imu_ok;
  int16_t ax; int16_t ay; int16_t az; int16_t gx; int16_t gy; int16_t gz; int16_t temp;
  int32_t gsum_x; int32_t gsum_y; int32_t gsum_z;
  uint32_t n_imu; uint32_t imu_err; uint32_t spi_us_max;
  int32_t cmd_pct; int32_t duty; uint32_t mstate; uint32_t wd_trip;
  uint32_t cmd_ok; uint32_t cmd_bad; uint32_t cmd_clamp; uint32_t rx_err;
} LineValues;

/* The two snprintf calls exactly as they were in main.c before the formatter
 * existed, head then tail, including the casts (the tail now without its
 * "\r\n"), plus a third one in the same style for the motor fields that
 * came later. */
static int format_line_with_snprintf(const LineValues *v, char *buffer, size_t size)
{
  int head_length = snprintf(buffer, size,
      "t_ms=%lu raw=%u count=%ld d100=%ld MD=%u ML=%u MH=%u AGC=%u "
      "mag_min=%u mag_max=%u i2c_err=%lu overrun=%lu resync=%lu skipped=%lu "
      "i2c_us_max=%lu loop_us_max=%lu",
      (unsigned long)v->t_ms, (unsigned)v->raw,
      (long)v->count, (long)v->d100,
      (unsigned)v->md, (unsigned)v->ml, (unsigned)v->mh, (unsigned)v->agc,
      (unsigned)v->mag_min, (unsigned)v->mag_max,
      (unsigned long)v->i2c_err, (unsigned long)v->overrun,
      (unsigned long)v->resync, (unsigned long)v->skipped,
      (unsigned long)v->i2c_us_max, (unsigned long)v->loop_us_max);
  int tail_length = snprintf(buffer + head_length, size - (size_t)head_length,
      " imu_ok=%u ax=%d ay=%d az=%d gx=%d gy=%d gz=%d temp=%d "
      "gsum_x=%ld gsum_y=%ld gsum_z=%ld n_imu=%lu imu_err=%lu spi_us_max=%lu",
      (unsigned)v->imu_ok,
      (int)v->ax, (int)v->ay, (int)v->az, (int)v->gx, (int)v->gy, (int)v->gz, (int)v->temp,
      (long)v->gsum_x, (long)v->gsum_y, (long)v->gsum_z,
      (unsigned long)v->n_imu, (unsigned long)v->imu_err, (unsigned long)v->spi_us_max);
  int motor_offset = head_length + tail_length;
  int motor_length = snprintf(buffer + motor_offset, size - (size_t)motor_offset,
      " cmd_pct=%ld duty=%ld mstate=%lu wd_trip=%lu cmd_ok=%lu cmd_bad=%lu cmd_clamp=%lu rx_err=%lu\r\n",
      (long)v->cmd_pct, (long)v->duty, (unsigned long)v->mstate, (unsigned long)v->wd_trip,
      (unsigned long)v->cmd_ok, (unsigned long)v->cmd_bad, (unsigned long)v->cmd_clamp, (unsigned long)v->rx_err);
  return motor_offset + motor_length;
}

/* The same line built the way main.c builds it now (same calls, same order,
 * head, IMU part and motor part as three separate runs on one writer). */
static size_t format_line_with_writer(const LineValues *v, char *buffer, size_t size, _Bool *truncated)
{
  TelemetryWriter line_writer;
  TelemetryWriter_Init(&line_writer, buffer, size);
  TelemetryWriter_AppendUnsignedField(&line_writer, "t_ms",        v->t_ms);
  TelemetryWriter_AppendUnsignedField(&line_writer, "raw",         v->raw);
  TelemetryWriter_AppendSignedField(&line_writer,   "count",       v->count);
  TelemetryWriter_AppendSignedField(&line_writer,   "d100",        v->d100);
  TelemetryWriter_AppendUnsignedField(&line_writer, "MD",          v->md);
  TelemetryWriter_AppendUnsignedField(&line_writer, "ML",          v->ml);
  TelemetryWriter_AppendUnsignedField(&line_writer, "MH",          v->mh);
  TelemetryWriter_AppendUnsignedField(&line_writer, "AGC",         v->agc);
  TelemetryWriter_AppendUnsignedField(&line_writer, "mag_min",     v->mag_min);
  TelemetryWriter_AppendUnsignedField(&line_writer, "mag_max",     v->mag_max);
  TelemetryWriter_AppendUnsignedField(&line_writer, "i2c_err",     v->i2c_err);
  TelemetryWriter_AppendUnsignedField(&line_writer, "overrun",     v->overrun);
  TelemetryWriter_AppendUnsignedField(&line_writer, "resync",      v->resync);
  TelemetryWriter_AppendUnsignedField(&line_writer, "skipped",     v->skipped);
  TelemetryWriter_AppendUnsignedField(&line_writer, "i2c_us_max",  v->i2c_us_max);
  TelemetryWriter_AppendUnsignedField(&line_writer, "loop_us_max", v->loop_us_max);

  TelemetryWriter_AppendUnsignedField(&line_writer, "imu_ok",  v->imu_ok ? 1u : 0u);
  TelemetryWriter_AppendSignedField(&line_writer,   "ax",      v->ax);
  TelemetryWriter_AppendSignedField(&line_writer,   "ay",      v->ay);
  TelemetryWriter_AppendSignedField(&line_writer,   "az",      v->az);
  TelemetryWriter_AppendSignedField(&line_writer,   "gx",      v->gx);
  TelemetryWriter_AppendSignedField(&line_writer,   "gy",      v->gy);
  TelemetryWriter_AppendSignedField(&line_writer,   "gz",      v->gz);
  TelemetryWriter_AppendSignedField(&line_writer,   "temp",    v->temp);
  TelemetryWriter_AppendSignedField(&line_writer,   "gsum_x",  v->gsum_x);
  TelemetryWriter_AppendSignedField(&line_writer,   "gsum_y",  v->gsum_y);
  TelemetryWriter_AppendSignedField(&line_writer,   "gsum_z",  v->gsum_z);
  TelemetryWriter_AppendUnsignedField(&line_writer, "n_imu",   v->n_imu);
  TelemetryWriter_AppendUnsignedField(&line_writer, "imu_err", v->imu_err);
  TelemetryWriter_AppendUnsignedField(&line_writer, "spi_us_max", v->spi_us_max);

  TelemetryWriter_AppendSignedField(&line_writer,   "cmd_pct",   v->cmd_pct);
  TelemetryWriter_AppendSignedField(&line_writer,   "duty",      v->duty);
  TelemetryWriter_AppendUnsignedField(&line_writer, "mstate",    v->mstate);
  TelemetryWriter_AppendUnsignedField(&line_writer, "wd_trip",   v->wd_trip);
  TelemetryWriter_AppendUnsignedField(&line_writer, "cmd_ok",    v->cmd_ok);
  TelemetryWriter_AppendUnsignedField(&line_writer, "cmd_bad",   v->cmd_bad);
  TelemetryWriter_AppendUnsignedField(&line_writer, "cmd_clamp", v->cmd_clamp);
  TelemetryWriter_AppendUnsignedField(&line_writer, "rx_err",    v->rx_err);
  TelemetryWriter_AppendText(&line_writer, "\r\n");

  *truncated = line_writer.truncated;
  return line_writer.length;
}

/* Buffers are 640 bytes, like main.c's telemetry_line. Returns the length. */
static size_t compare_whole_line(const char *label, const LineValues *values, const char *expected_literal)
{
  char with_snprintf[640];
  char with_writer[640];
  int snprintf_length = format_line_with_snprintf(values, with_snprintf, sizeof(with_snprintf));
  _Bool truncated;
  size_t writer_length = format_line_with_writer(values, with_writer, sizeof(with_writer), &truncated);

  CHECK(!truncated, "%s: writer truncated", label);
  CHECK((size_t)snprintf_length == writer_length && memcmp(with_snprintf, with_writer, writer_length) == 0,
        "%s: lines differ\n  snprintf: %s  writer:   %s", label, with_snprintf, with_writer);
  if (expected_literal != NULL)
  {
    CHECK(strcmp(with_writer, expected_literal) == 0, "%s: differs from the captured line\n  got: %s", label, with_writer);
  }
  printf("  %-12s %3zu bytes, identical: %s\n", label, writer_length,
         (memcmp(with_snprintf, with_writer, writer_length) == 0) ? "yes" : "NO");
  return writer_length;
}

static void test_whole_lines(void)
{
  /* Values from a line captured from the real board, plus the motor fields
   * as they read right after boot (latched, nothing received yet). */
  LineValues captured = {
    .t_ms = 65400, .raw = 1993, .count = 0, .d100 = -1, .md = 1, .ml = 1, .mh = 0, .agc = 128,
    .mag_min = 1540, .mag_max = 1544, .i2c_err = 0, .overrun = 0, .resync = 0, .skipped = 0,
    .i2c_us_max = 523, .loop_us_max = 899, .imu_ok = 1,
    .ax = -126, .ay = 215, .az = 8234, .gx = -9, .gy = -25, .gz = 1, .temp = -502,
    .gsum_x = -934, .gsum_y = -2932, .gsum_z = -295, .n_imu = 100, .imu_err = 0, .spi_us_max = 186,
    .cmd_pct = 0, .duty = 0, .mstate = 2, .wd_trip = 0, .cmd_ok = 0, .cmd_bad = 0, .cmd_clamp = 0, .rx_err = 0 };
  compare_whole_line("captured", &captured,
      "t_ms=65400 raw=1993 count=0 d100=-1 MD=1 ML=1 MH=0 AGC=128 mag_min=1540 mag_max=1544 "
      "i2c_err=0 overrun=0 resync=0 skipped=0 i2c_us_max=523 loop_us_max=899 imu_ok=1 "
      "ax=-126 ay=215 az=8234 gx=-9 gy=-25 gz=1 temp=-502 gsum_x=-934 gsum_y=-2932 gsum_z=-295 "
      "n_imu=100 imu_err=0 spi_us_max=186 "
      "cmd_pct=0 duty=0 mstate=2 wd_trip=0 cmd_ok=0 cmd_bad=0 cmd_clamp=0 rx_err=0\r\n");

  /* Every field at its widest possible text. *_us_max is at most
   * UINT32_MAX / 32 cycles per us = 134217727; gsum at most 65535 * -32768.
   * cmd_pct is at most -100 and duty at most -1600 (a whole PWM period). */
  LineValues widest = {
    .t_ms = UINT32_MAX, .raw = 4095, .count = INT32_MIN, .d100 = INT32_MIN, .md = 1, .ml = 1, .mh = 1,
    .agc = 255, .mag_min = UINT16_MAX, .mag_max = UINT16_MAX, .i2c_err = UINT32_MAX,
    .overrun = UINT32_MAX, .resync = UINT32_MAX, .skipped = UINT32_MAX,
    .i2c_us_max = UINT32_MAX / 32u, .loop_us_max = UINT32_MAX / 32u, .imu_ok = 1,
    .ax = INT16_MIN, .ay = INT16_MIN, .az = INT16_MIN, .gx = INT16_MIN, .gy = INT16_MIN, .gz = INT16_MIN,
    .temp = INT16_MIN, .gsum_x = -2147450880, .gsum_y = -2147450880, .gsum_z = -2147450880,
    .n_imu = 65535, .imu_err = UINT32_MAX, .spi_us_max = UINT32_MAX / 32u,
    .cmd_pct = -100, .duty = -1600, .mstate = 2, .wd_trip = UINT32_MAX, .cmd_ok = UINT32_MAX,
    .cmd_bad = UINT32_MAX, .cmd_clamp = UINT32_MAX, .rx_err = UINT32_MAX };
  size_t widest_length = compare_whole_line("widest", &widest, NULL);
  /* 420 bytes without the motor fields + 128 for them. main.c and the
   * README quote this number, and telemetry_line (640 bytes) must hold it. */
  CHECK(widest_length == 548u, "widest line is %zu bytes, expected 548", widest_length);

  /* Init failed: IMU fields all zero. */
  LineValues imu_off = captured;
  imu_off.imu_ok = 0;
  imu_off.ax = imu_off.ay = imu_off.az = imu_off.gx = imu_off.gy = imu_off.gz = imu_off.temp = 0;
  imu_off.gsum_x = imu_off.gsum_y = imu_off.gsum_z = 0;
  imu_off.n_imu = 0; imu_off.spi_us_max = 0;
  compare_whole_line("imu_off", &imu_off, NULL);

  /* Random lines: every field drawn across its whole type range. */
  int random_lines_identical = 1;
  for (int i = 0; i < 2000; i++)
  {
    LineValues v = {
      .t_ms = next_random(), .raw = (uint16_t)(next_random() & 0x0FFFu), .count = (int32_t)next_random(),
      .d100 = (int32_t)next_random() >> (next_random() % 32u), .md = (uint8_t)(next_random() & 1u),
      .ml = (uint8_t)(next_random() & 1u), .mh = (uint8_t)(next_random() & 1u), .agc = (uint8_t)next_random(),
      .mag_min = (uint16_t)next_random(), .mag_max = (uint16_t)next_random(),
      .i2c_err = next_random() >> (next_random() % 32u), .overrun = next_random() >> (next_random() % 32u),
      .resync = next_random() >> (next_random() % 32u), .skipped = next_random() >> (next_random() % 32u),
      .i2c_us_max = next_random() >> 5, .loop_us_max = next_random() >> 5, .imu_ok = (_Bool)(next_random() & 1u),
      .ax = (int16_t)next_random(), .ay = (int16_t)next_random(), .az = (int16_t)next_random(),
      .gx = (int16_t)next_random(), .gy = (int16_t)next_random(), .gz = (int16_t)next_random(),
      .temp = (int16_t)next_random(), .gsum_x = (int32_t)next_random(), .gsum_y = (int32_t)next_random(),
      .gsum_z = (int32_t)next_random(), .n_imu = next_random() & 0xFFFFu, .imu_err = next_random(),
      .spi_us_max = next_random() >> 5,
      .cmd_pct = (int32_t)(next_random() % 201u) - 100, .duty = (int32_t)(next_random() % 3201u) - 1600,
      .mstate = next_random() % 3u, .wd_trip = next_random() >> (next_random() % 32u),
      .cmd_ok = next_random() >> (next_random() % 32u), .cmd_bad = next_random() >> (next_random() % 32u),
      .cmd_clamp = next_random() >> (next_random() % 32u), .rx_err = next_random() >> (next_random() % 32u) };
    char with_snprintf[640];
    char with_writer[640];
    int snprintf_length = format_line_with_snprintf(&v, with_snprintf, sizeof(with_snprintf));
    _Bool truncated;
    size_t writer_length = format_line_with_writer(&v, with_writer, sizeof(with_writer), &truncated);
    int same = !truncated && (size_t)snprintf_length == writer_length &&
               memcmp(with_snprintf, with_writer, writer_length) == 0;
    CHECK(same, "random line %d differs\n  snprintf: %s  writer:   %s", i, with_snprintf, with_writer);
    if (!same) { random_lines_identical = 0; }
  }
  printf("  %-12s 2000 lines, identical: %s\n", "random", random_lines_identical ? "yes" : "NO");
}

int main(void)
{
  printf("whole lines:\n");
  test_whole_lines();
  test_values();
  test_separators_and_text();
  test_truncation();

  printf("%d checks, %d failed\n", check_count, failure_count);
  if (failure_count != 0)
  {
    return EXIT_FAILURE;
  }
  printf("ALL PASSED\n");
  return EXIT_SUCCESS;
}
