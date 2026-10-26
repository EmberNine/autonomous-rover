/*
 * Host-side test for Core/Src/speed_capture.c. Not part of the firmware
 * build or CI. Compiles it with the PC's C compiler.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_speed_capture.c Core/Src/speed_capture.c Core/Src/telemetry_format.c -o /tmp/test_speed_capture && /tmp/test_speed_capture
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "speed_capture.h"

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

#define LINE_CAPACITY 320u

static SpeedCaptureSample samples[SPEED_CAPTURE_SAMPLES];

static SpeedCaptureInput input_at(uint32_t now_ms, int32_t target, int32_t applied, int32_t count, bool read_ok)
{
  SpeedCaptureInput input = { .now_ms = now_ms, .target_ticks = target, .applied_ticks = applied,
                              .count = count, .raw_angle = (uint16_t)((uint32_t)count & 4095u),
                              .read_ok = read_ok, .vbat_mv = 7902u };
  return input;
}

/* Builds the current line with max_items pieces per step; returns the steps taken. */
static uint32_t build_line(SpeedCapture *capture, char *line, uint32_t max_items, bool *truncated)
{
  TelemetryWriter writer;
  TelemetryWriter_Init(&writer, line, LINE_CAPACITY);
  uint32_t steps = 0u;
  while (!SpeedCapture_BuildLineStep(capture, &writer, max_items))
  {
    steps++;
    if (steps > 1000u) { break; }
  }
  *truncated = writer.truncated;
  return steps + 1u;
}

/* ---- Commands ----------------------------------------------------------------- */

static void test_commands(void)
{
  SpeedCapture capture;
  SpeedCapture_Init(&capture, samples, SPEED_CAPTURE_SAMPLES);
  CHECK(capture.state == SPEED_CAPTURE_IDLE, "starts idle");
  CHECK(!SpeedCapture_Command(&capture, -1) && !SpeedCapture_Command(&capture, 11) &&
        !SpeedCapture_Command(&capture, 100), "values outside 0..10 are refused");
  CHECK(capture.state == SPEED_CAPTURE_IDLE, "and change nothing");
  CHECK(SpeedCapture_Command(&capture, 1) && capture.state == SPEED_CAPTURE_ARMED && capture.period_ms == 1u, "c 1 arms");
  CHECK(SpeedCapture_Command(&capture, 10) && capture.period_ms == 10u, "c 10 re-arms with a new period while armed");
  CHECK(SpeedCapture_Command(&capture, 0) && capture.state == SPEED_CAPTURE_IDLE, "c 0 disarms");

  /* Busy while recording: refused; c 0 cancels. */
  SpeedCapture_Command(&capture, 1);
  SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .target_ticks = 0 });
  SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .target_ticks = 320, .read_ok = true });
  CHECK(capture.state == SPEED_CAPTURE_RECORDING, "setup: recording");
  CHECK(!SpeedCapture_Command(&capture, 2) && capture.state == SPEED_CAPTURE_RECORDING, "c 2 while recording: refused");
  CHECK(SpeedCapture_Command(&capture, 0) && capture.state == SPEED_CAPTURE_IDLE, "c 0 cancels a recording");
}

/* ---- Trigger -------------------------------------------------------------------- */

static void test_trigger_only_on_a_target_change_while_armed(void)
{
  SpeedCapture capture;
  SpeedCapture_Init(&capture, samples, SPEED_CAPTURE_SAMPLES);
  /* Not armed: target changes start nothing. */
  for (uint32_t t = 0u; t < 100u; t++) { SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .now_ms = t, .target_ticks = (int32_t)(t / 10u) }); }
  CHECK(capture.state == SPEED_CAPTURE_IDLE && capture.capture_id == 0u, "not armed: nothing");

  /* Armed: the same target over and over (keepalives) starts nothing. */
  SpeedCapture_Command(&capture, 1);
  for (uint32_t t = 100u; t < 200u; t++) { SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .now_ms = t, .target_ticks = 9 }); }
  CHECK(capture.state == SPEED_CAPTURE_ARMED, "armed, unchanged target: still waiting");

  /* The change starts it, and the header remembers both targets. */
  SpeedCaptureInput trigger = input_at(200u, 320, 2, 1234, true);
  SpeedCapture_Tick(&capture, &trigger);
  CHECK(capture.state == SPEED_CAPTURE_RECORDING && capture.capture_id == 1u && capture.t0_ms == 200u &&
        capture.count0 == 1234 && capture.raw0 == (1234u & 4095u) && capture.vbat_mv == 7902u &&
        capture.previous_target_ticks == 9 && capture.trigger_target_ticks == 320,
        "trigger: header (state %d, id %u, t0 %u, count0 %d, prev %d, target %d)", (int)capture.state,
        (unsigned)capture.capture_id, (unsigned)capture.t0_ms, (int)capture.count0,
        (int)capture.previous_target_ticks, (int)capture.trigger_target_ticks);
  CHECK(capture.sample_count == 1u && samples[0].delta_counts == 0 && samples[0].duty_ticks == 2,
        "sample 0 is the trigger ms itself");

  /* The very first tick after Init can't trigger (no previous target to compare). */
  SpeedCapture_Init(&capture, samples, SPEED_CAPTURE_SAMPLES);
  SpeedCapture_Command(&capture, 1);
  SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .target_ticks = 320 });
  CHECK(capture.state == SPEED_CAPTURE_ARMED, "first tick: no previous target, no trigger");
}

/* ---- Recording ---------------------------------------------------------------------- */

static void test_recording_deltas_duty_and_missing_samples(void)
{
  SpeedCapture capture;
  SpeedCapture_Init(&capture, samples, 10u);
  SpeedCapture_Command(&capture, 1);
  SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .target_ticks = 0 });
  /* t: count, read ok?  The count stays put on a failed read, as in main.c. */
  static const struct { int32_t count; bool ok; int32_t applied; } ms[] = {
    { 100, true, 2 },    /* trigger: sample 0, d = 0 */
    { 101, true, 4 },    /* d = 1 */
    { 103, true, 6 },    /* d = 2 */
    { 103, false, 8 },   /* read failed: x */
    { 103, false, 10 },  /* x */
    { 109, true, 12 },   /* d = 6: since the last good sample (103) */
    { 110, true, 14 },
    { 110, true, 16 },   /* d = 0: really not moving */
    { 108, true, -18 },  /* d = -2, duty negative */
    { 108, true, 20 },
  };
  for (size_t i = 0; i < sizeof ms / sizeof ms[0]; i++)
  {
    SpeedCaptureInput input = input_at(1000u + (uint32_t)i, 320, ms[i].applied, ms[i].count, ms[i].ok);
    SpeedCapture_Tick(&capture, &input);
  }
  static const int16_t expected_d[] = { 0, 1, 2, SPEED_CAPTURE_MISSING, SPEED_CAPTURE_MISSING, 6, 1, 0, -2, 0 };
  int wrong = 0;
  for (size_t i = 0; i < 10u; i++)
  {
    if (samples[i].delta_counts != expected_d[i] || samples[i].duty_ticks != ms[i].applied) { wrong++; }
  }
  CHECK(wrong == 0, "%d samples differ from the expected deltas and duties", wrong);
  CHECK(capture.state == SPEED_CAPTURE_SENDING, "full: sending");

  /* Further ticks change nothing. */
  SpeedCaptureInput later = input_at(2000u, 0, 0, 5000, true);
  SpeedCapture_Tick(&capture, &later);
  CHECK(capture.sample_count == 10u && samples[9].delta_counts == 0, "nothing recorded after the end");
}

static void test_sample_period(void)
{
  SpeedCapture capture;
  SpeedCapture_Init(&capture, samples, 4u);
  SpeedCapture_Command(&capture, 4);
  SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .target_ticks = 0 });
  /* Count = 3 per ms; trigger at ms 0, then a sample every 4th ms. */
  uint32_t t = 0u;
  while (capture.state == SPEED_CAPTURE_RECORDING || t == 0u)
  {
    SpeedCaptureInput input = input_at(t, 100, (int32_t)t, (int32_t)(3u * t), true);
    SpeedCapture_Tick(&capture, &input);
    t++;
    if (t > 100u) { break; }
  }
  CHECK(capture.sample_count == 4u && t == 13u, "4 samples at period 4 take 13 ms (ms 0, 4, 8, 12), took %u", (unsigned)t);
  CHECK(samples[0].delta_counts == 0 && samples[1].delta_counts == 12 && samples[3].delta_counts == 12 &&
        samples[1].duty_ticks == 4 && samples[3].duty_ticks == 12, "deltas over 4 ms and the duty at the sample ms");
}

static void test_values_beyond_int16_are_clamped_and_never_read_as_missing(void)
{
  SpeedCapture capture;
  SpeedCapture_Init(&capture, samples, 3u);
  SpeedCapture_Command(&capture, 1);
  SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .target_ticks = 0 });
  SpeedCaptureInput a = input_at(0u, 1, 0, 0, true);
  SpeedCaptureInput b = input_at(1u, 1, 0, -100000, true);
  SpeedCaptureInput c = input_at(2u, 1, 0, 100000, true);
  SpeedCapture_Tick(&capture, &a);
  SpeedCapture_Tick(&capture, &b);
  SpeedCapture_Tick(&capture, &c);
  CHECK(samples[1].delta_counts == -INT16_MAX && samples[2].delta_counts == INT16_MAX,
        "clamped to +-32767 (got %d, %d), never INT16_MIN", samples[1].delta_counts, samples[2].delta_counts);
}

/* ---- Lines ---------------------------------------------------------------------------- */

static SpeedCapture filled_capture(uint32_t sample_total)
{
  SpeedCapture capture;
  SpeedCapture_Init(&capture, samples, sample_total);
  SpeedCapture_Command(&capture, 1);
  SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .target_ticks = 0 });
  for (uint32_t t = 0u; t < sample_total; t++)
  {
    bool ok = (t % 7u) != 3u;
    SpeedCaptureInput input = input_at(5000u + t, 320, (int32_t)(2u * t), (int32_t)(2u * t), ok);
    if (!ok) { input.count = (int32_t)(2u * (t - 1u)); }  /* stale on a failed read */
    SpeedCapture_Tick(&capture, &input);
  }
  return capture;
}

static void test_header_and_data_lines(void)
{
  SpeedCapture capture = filled_capture(25u);  /* one full data line and one of 5 */
  CHECK(SpeedCapture_HasLine(&capture), "lines to send");
  char line[LINE_CAPACITY];
  bool truncated;

  build_line(&capture, line, 3u, &truncated);
  CHECK(!truncated && strcmp(line, "caph id=1 per=1 n=25 t0=5000 count0=0 raw0=0 vbat=7902 prev=0 target=320\r\n") == 0,
        "header: [%s]", line);
  SpeedCapture_LineSent(&capture);

  build_line(&capture, line, 4u, &truncated);
  CHECK(!truncated && strcmp(line, "cap id=1 i=0 d=0,2,2,x,4,2,2,2,2,2,x,4,2,2,2,2,2,x,4,2"
                                   " u=0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38\r\n") == 0,
        "first data line: [%s]", line);
  SpeedCapture_LineSent(&capture);

  build_line(&capture, line, 1u, &truncated);
  CHECK(!truncated && strcmp(line, "cap id=1 i=20 d=2,2,2,2,x u=40,42,44,46,48\r\n") == 0, "last, short line: [%s]", line);
  SpeedCapture_LineSent(&capture);
  CHECK(!SpeedCapture_HasLine(&capture) && capture.state == SPEED_CAPTURE_IDLE, "after the last line: idle");
  CHECK(SpeedCapture_Command(&capture, 1) && capture.state == SPEED_CAPTURE_ARMED, "and can be armed again");
}

static void test_line_text_does_not_depend_on_the_step_size(void)
{
  char reference[LINE_CAPACITY];
  char line[LINE_CAPACITY];
  bool truncated;
  int differences = 0;
  int step_counts_wrong = 0;
  for (uint32_t line_number = 0u; line_number < 3u; line_number++)
  {
    SpeedCapture capture = filled_capture(45u);
    for (uint32_t skip = 0u; skip < line_number; skip++)
    {
      build_line(&capture, line, 100u, &truncated);
      SpeedCapture_LineSent(&capture);
    }
    SpeedCapture copy = capture;
    build_line(&copy, reference, 1000u, &truncated);
    for (uint32_t max_items = 1u; max_items <= 50u; max_items++)
    {
      copy = capture;
      uint32_t steps = build_line(&copy, line, max_items, &truncated);
      if (strcmp(line, reference) != 0 || truncated) { differences++; }
      uint32_t items = (line_number == 0u) ? 11u : 3u + 2u * 20u + 1u;
      if (steps != (items + max_items - 1u) / max_items) { step_counts_wrong++; }
    }
  }
  CHECK(differences == 0, "%d step sizes built a different line", differences);
  CHECK(step_counts_wrong == 0, "%d step sizes took a different number of steps than ceil(items / max)", step_counts_wrong);
}

static void test_full_size_capture_fits_the_line_buffer(void)
{
  /* Worst case per line: id and i at 10 digits, every d at -32767, every u at -32767. */
  SpeedCapture capture;
  SpeedCapture_Init(&capture, samples, SPEED_CAPTURE_SAMPLES);
  SpeedCapture_Command(&capture, 1);
  SpeedCapture_Tick(&capture, &(SpeedCaptureInput){ .target_ticks = 0 });
  for (uint32_t t = 0u; t < SPEED_CAPTURE_SAMPLES; t++)
  {
    SpeedCaptureInput input = input_at(t, 1, -100000, (t % 2u) ? -100000 : 100000, true);
    SpeedCapture_Tick(&capture, &input);
  }
  capture.capture_id = UINT32_MAX;
  char line[LINE_CAPACITY];
  bool truncated;
  uint32_t lines = 0u;
  size_t longest = 0u;
  int any_truncated = 0;
  while (SpeedCapture_HasLine(&capture))
  {
    build_line(&capture, line, 5u, &truncated);
    if (truncated) { any_truncated++; }
    if (strlen(line) > longest) { longest = strlen(line); }
    SpeedCapture_LineSent(&capture);
    lines++;
  }
  CHECK(lines == 1u + SPEED_CAPTURE_SAMPLES / SPEED_CAPTURE_SAMPLES_PER_LINE, "1 header + 150 data lines, got %u", (unsigned)lines);
  CHECK(any_truncated == 0 && longest < LINE_CAPACITY, "worst-case line %u bytes fits %u (truncated %d)",
        (unsigned)longest, LINE_CAPACITY, any_truncated);
}

static void test_switch_is_off(void)
{
  CHECK(SPEED_CAPTURE == 0, "SPEED_CAPTURE must be committed as 0");
}

int main(void)
{
  test_switch_is_off();
  test_commands();
  test_trigger_only_on_a_target_change_while_armed();
  test_recording_deltas_duty_and_missing_samples();
  test_sample_period();
  test_values_beyond_int16_are_clamped_and_never_read_as_missing();
  test_header_and_data_lines();
  test_line_text_does_not_depend_on_the_step_size();
  test_full_size_capture_fits_the_line_buffer();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
