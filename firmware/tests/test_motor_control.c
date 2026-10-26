/*
 * Host-side test for Core/Src/motor_control.c and Core/Src/byte_ring.c. Not
 * part of the firmware build or CI. Compiles both with the PC's C compiler.
 * Command-line parsing is tested in tests/test_command_line.c.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_motor_control.c Core/Src/motor_control.c Core/Src/byte_ring.c -o /tmp/test_motor_control && /tmp/test_motor_control
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "byte_ring.h"
#include "motor_control.h"

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

/* Deterministic pseudo-random numbers (xorshift32), so a failure reproduces. */
static uint32_t random_state = 0x2468ACE1u;
static uint32_t next_random(void)
{
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

/* ---- Byte ring ----------------------------------------------------------- */

static void test_byte_ring(void)
{
  ByteRing ring;
  uint8_t byte = 0;
  ByteRing_Init(&ring);
  CHECK(!ByteRing_Pop(&ring, &byte), "new ring should be empty");

  /* Fill exactly to capacity, one more is dropped and counted. */
  for (uint32_t i = 0; i < BYTE_RING_CAPACITY; i++)
  {
    CHECK(ByteRing_Push(&ring, (uint8_t)i), "push %u of %u failed", (unsigned)i, (unsigned)BYTE_RING_CAPACITY);
  }
  CHECK(!ByteRing_Push(&ring, 0xEEu), "push into a full ring should fail");
  CHECK(ring.dropped_count == 1u, "dropped_count %u, expected 1", (unsigned)ring.dropped_count);
  for (uint32_t i = 0; i < BYTE_RING_CAPACITY; i++)
  {
    CHECK(ByteRing_Pop(&ring, &byte) && byte == (uint8_t)i, "pop %u returned %u", (unsigned)i, (unsigned)byte);
  }
  CHECK(!ByteRing_Pop(&ring, &byte), "ring should be empty again");

  /* Order is kept across many wraps of the slot index, with a random mix of
   * pushes and pops. */
  ByteRing_Init(&ring);
  uint8_t next_pushed = 0;
  uint8_t next_expected = 0;
  int order_ok = 1;
  for (int i = 0; i < 200000; i++)
  {
    if (next_random() % 2u)
    {
      if (ByteRing_Push(&ring, next_pushed)) { next_pushed++; }
    }
    else if (ByteRing_Pop(&ring, &byte))
    {
      if (byte != next_expected) { order_ok = 0; }
      next_expected++;
    }
  }
  CHECK(order_ok, "bytes came out in the wrong order");

  /* The free-running counts wrap at 2^32 without losing or inventing bytes. */
  ByteRing_Init(&ring);
  ring.write_count = UINT32_MAX - 2u;
  ring.read_count = UINT32_MAX - 2u;
  for (uint8_t i = 0; i < 10u; i++)
  {
    CHECK(ByteRing_Push(&ring, (uint8_t)(100u + i)), "push across the wrap failed at %u", (unsigned)i);
  }
  CHECK(ring.write_count - ring.read_count == 10u, "queued count across the wrap is %u",
        (unsigned)(ring.write_count - ring.read_count));
  for (uint8_t i = 0; i < 10u; i++)
  {
    CHECK(ByteRing_Pop(&ring, &byte) && byte == (uint8_t)(100u + i), "pop across the wrap: %u", (unsigned)byte);
  }
  CHECK(!ByteRing_Pop(&ring, &byte), "ring should be empty after the wrap test");
}

/* ---- State machine: a simulated loop with invariant checks --------------- */

/* Runs the controller one ms at a time, like the firmware loop, and checks
 * after every step the rules that must never break, whatever the commands:
 *   1. duty never exceeds the cap
 *   2. driver disabled => coast and duty 0
 *   3. the direction pins change only between two ms that both have duty 0
 *      (going from disabled to enabled counts as a change, so the first
 *      enabled ms has duty 0)
 *   4. while enabled, the duty changes by at most 2 ticks per elapsed ms
 *      (at most MOTOR_RAMP_MAX_STEP_MS worth after a stall)
 *   5. a duty larger than one ramp step goes straight to "disabled" only on
 *      a watchdog trip, a lock-out to FAULT, or a MotorController_Latch
 *      (stall, encoder) since the last step (a normal stop ramps down first)
 *   6. enabled with duty 0 for two ms in a row, in a driving direction
 *      (= short brake), only while a reverse dwell is active */
typedef struct
{
  MotorController controller;
  uint32_t now_ms;
  uint32_t last_step_ms;
  MotorOutput previous_output;
  int invariant_failures;
  uint32_t brake_ms;            /* ms spent enabled with duty 0 after the first */
  uint32_t enabled_zero_run;    /* consecutive enabled ms with duty 0 */
  uint32_t latches_seen;        /* controller.latch_count at the last step */
} MotorBench;

static void bench_init(MotorBench *bench, uint32_t start_ms)
{
  memset(bench, 0, sizeof(*bench));
  bench->now_ms = start_ms;
  bench->last_step_ms = start_ms;
  MotorController_Init(&bench->controller, start_ms);
  bench->previous_output.driver_enabled = false;
  bench->previous_output.direction = MOTOR_DIRECTION_COAST;
}

static void bench_command(MotorBench *bench, int32_t percent)
{
  MotorController_AcceptCommand(&bench->controller, percent, bench->now_ms);
}

static int32_t signed_duty(const MotorOutput *output)
{
  return (output->direction == MOTOR_DIRECTION_REVERSE) ? -(int32_t)output->duty_ticks : (int32_t)output->duty_ticks;
}

#define BENCH_FAIL(...)                                          \
  do                                                             \
  {                                                              \
    if (bench->invariant_failures < 5)                           \
    {                                                            \
      printf("INVARIANT at t=%" PRIu32 ": ", bench->now_ms);     \
      printf(__VA_ARGS__);                                       \
      printf("\n");                                              \
    }                                                            \
    bench->invariant_failures++;                                 \
  } while (0)

static MotorOutput bench_step(MotorBench *bench)
{
  bench->now_ms++;
  uint32_t elapsed_ms = bench->now_ms - bench->last_step_ms;
  bench->last_step_ms = bench->now_ms;
  if (elapsed_ms > MOTOR_RAMP_MAX_STEP_MS) { elapsed_ms = MOTOR_RAMP_MAX_STEP_MS; }
  int32_t allowed_change = (int32_t)elapsed_ms * MOTOR_RAMP_TICKS_PER_MS;
  uint32_t trips_before = bench->controller.watchdog_trip_count;
  MotorOutput output = MotorController_Update(&bench->controller, bench->now_ms);
  const MotorOutput *previous = &bench->previous_output;

  if ((bench->controller.state == MOTOR_STATE_LATCHED) != (bench->controller.latch_reason != MOTOR_LATCH_NONE))
  {
    BENCH_FAIL("state %d with latch reason %d", (int)bench->controller.state, (int)bench->controller.latch_reason);
  }

  if (output.duty_ticks > (uint32_t)MOTOR_DUTY_CAP_TICKS)
  {
    BENCH_FAIL("duty %u above the cap", (unsigned)output.duty_ticks);
  }
  if (!output.driver_enabled && (output.direction != MOTOR_DIRECTION_COAST || output.duty_ticks != 0u))
  {
    BENCH_FAIL("disabled but direction %d duty %u", (int)output.direction, (unsigned)output.duty_ticks);
  }
  MotorDirection previous_pins = previous->driver_enabled ? previous->direction : MOTOR_DIRECTION_COAST;
  MotorDirection pins = output.driver_enabled ? output.direction : MOTOR_DIRECTION_COAST;
  if (output.driver_enabled && pins != previous_pins && (previous->duty_ticks != 0u || output.duty_ticks != 0u))
  {
    BENCH_FAIL("direction %d -> %d with duty %u -> %u", (int)previous_pins, (int)pins,
               (unsigned)previous->duty_ticks, (unsigned)output.duty_ticks);
  }
  if (output.driver_enabled && previous->driver_enabled)
  {
    int32_t change = signed_duty(&output) - signed_duty(previous);
    if (change > allowed_change || change < -allowed_change)
    {
      BENCH_FAIL("duty jumped by %d in %u ms", (int)change, (unsigned)elapsed_ms);
    }
  }
  bool latched_on_purpose = (bench->controller.latch_count != bench->latches_seen);
  bench->latches_seen = bench->controller.latch_count;
  if (!output.driver_enabled && previous->driver_enabled && previous->duty_ticks > (uint32_t)allowed_change &&
      bench->controller.watchdog_trip_count == trips_before && bench->controller.state != MOTOR_STATE_FAULT &&
      !latched_on_purpose)
  {
    BENCH_FAIL("disabled from duty %u without a watchdog trip or a latch", (unsigned)previous->duty_ticks);
  }
  if (output.driver_enabled && output.duty_ticks == 0u && output.direction != MOTOR_DIRECTION_COAST)
  {
    bench->enabled_zero_run++;
    if (bench->enabled_zero_run >= 2u)
    {
      bench->brake_ms++;
      /* Allowed only in a dwell, or on the ms the dwell ends and the pins switch. */
      if (!bench->controller.dwell_active && pins == previous_pins)
      {
        BENCH_FAIL("short brake (enabled, duty 0) outside a reverse dwell");
      }
    }
  }
  else
  {
    bench->enabled_zero_run = 0u;
  }

  bench->previous_output = output;
  return output;
}

static MotorOutput bench_run(MotorBench *bench, uint32_t milliseconds)
{
  MotorOutput output = bench->previous_output;
  for (uint32_t i = 0; i < milliseconds; i++)
  {
    output = bench_step(bench);
  }
  return output;
}

/* Keeps commanding percent every 100 ms (like the PC console) for duration_ms. */
static MotorOutput bench_drive(MotorBench *bench, int32_t percent, uint32_t duration_ms)
{
  MotorOutput output = bench->previous_output;
  for (uint32_t elapsed = 0; elapsed < duration_ms; elapsed++)
  {
    if (elapsed % 100u == 0u)
    {
      bench_command(bench, percent);
    }
    output = bench_step(bench);
  }
  return output;
}

static void test_latched_at_boot(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 1000u);
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED, "should start latched");

  /* Non-zero commands are valid but ignored while latched. */
  MotorOutput output = bench_drive(bench, 20, 1000u);
  CHECK(!output.driver_enabled && output.duty_ticks == 0u, "latched: driver must stay disabled");
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED, "still latched after m 20");
  CHECK(bench->controller.valid_command_count == 10u, "m 20 x10 should count as valid, got %u",
        (unsigned)bench->controller.valid_command_count);
  CHECK(bench->controller.requested_percent == 20, "requested_percent should show the ignored 20");
  CHECK(bench->controller.watchdog_trip_count == 0u, "no trip while latched");

  /* "m 0" re-arms but doesn't start anything. */
  bench_command(bench, 0);
  output = bench_step(bench);
  CHECK(bench->controller.state == MOTOR_STATE_OFF && !output.driver_enabled, "m 0 should give OFF, disabled");

  /* OFF with no commands for a long time: no trip, nothing happens. */
  output = bench_run(bench, 5000u);
  CHECK(bench->controller.watchdog_trip_count == 0u && !output.driver_enabled, "idle OFF should never trip");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_ramp_up_and_stop(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 0u);
  bench_command(bench, 0);
  bench_step(bench);

  /* m 20: one wake-up ms (enabled, forward, duty 0), then +2 ticks per ms
   * up to 20 % = 320 ticks, reached 160 ms after the wake-up. */
  bench_command(bench, 20);
  MotorOutput output = bench_step(bench);
  CHECK(output.driver_enabled && output.direction == MOTOR_DIRECTION_FORWARD && output.duty_ticks == 0u,
        "wake-up ms: enabled=%d dir=%d duty=%u", output.driver_enabled, (int)output.direction,
        (unsigned)output.duty_ticks);
  output = bench_step(bench);
  CHECK(output.duty_ticks == 2u, "first ramp ms should be 2 ticks, got %u", (unsigned)output.duty_ticks);
  output = bench_run(bench, 158u);
  CHECK(output.duty_ticks == 318u, "after 159 ramp ms: %u, expected 318", (unsigned)output.duty_ticks);
  bench_command(bench, 20);  /* keepalive */
  output = bench_step(bench);
  CHECK(output.duty_ticks == 320u && bench->controller.applied_ticks == 320, "after 160 ramp ms: %u",
        (unsigned)output.duty_ticks);
  output = bench_drive(bench, 20, 300u);
  CHECK(output.duty_ticks == 320u, "holds at 320, got %u", (unsigned)output.duty_ticks);

  /* m 0: ramps down at 2 ticks per ms, and on the ms it reaches 0 the
   * driver is DISABLED (STBY low, coast), not left in short brake. */
  bench_command(bench, 0);
  output = bench_run(bench, 159u);
  CHECK(output.driver_enabled && output.duty_ticks == 2u, "159 ms after m 0: enabled=%d duty=%u",
        output.driver_enabled, (unsigned)output.duty_ticks);
  output = bench_step(bench);
  CHECK(!output.driver_enabled && output.direction == MOTOR_DIRECTION_COAST && output.duty_ticks == 0u,
        "ramp reached 0: must be disabled + coast, got enabled=%d dir=%d", output.driver_enabled,
        (int)output.direction);
  CHECK(bench->controller.state == MOTOR_STATE_OFF, "state after stop: %d", (int)bench->controller.state);
  CHECK(bench->brake_ms == 0u, "no short brake expected without a reversal, got %u ms",
        (unsigned)bench->brake_ms);

  /* Reverse from rest: wake-up ms in reverse, then ramps negative. */
  bench_command(bench, -15);
  output = bench_step(bench);
  CHECK(output.driver_enabled && output.direction == MOTOR_DIRECTION_REVERSE && output.duty_ticks == 0u,
        "reverse wake-up ms");
  output = bench_drive(bench, -15, 200u);
  CHECK(output.direction == MOTOR_DIRECTION_REVERSE && output.duty_ticks == 240u &&
        bench->controller.applied_ticks == -240, "reverse 15 %%: dir=%d duty=%u", (int)output.direction,
        (unsigned)output.duty_ticks);

  /* m 0 while running in reverse also ends disabled. */
  bench_command(bench, 0);
  output = bench_run(bench, 120u);
  CHECK(!output.driver_enabled && bench->controller.state == MOTOR_STATE_OFF, "reverse stop: disabled");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_clamp(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 0u);
  bench_command(bench, 0);

  bench_command(bench, 30);
  CHECK(bench->controller.clamped_command_count == 0u && bench->controller.target_ticks == 480,
        "30 %% is at the cap, not clamped");
  bench_command(bench, 50);
  CHECK(bench->controller.clamped_command_count == 1u && bench->controller.target_ticks == 480,
        "50 %% should clamp to 480, got %d", (int)bench->controller.target_ticks);
  bench_command(bench, -100);
  CHECK(bench->controller.clamped_command_count == 2u && bench->controller.target_ticks == -480,
        "-100 %% should clamp to -480");
  CHECK(bench->controller.requested_percent == -100, "requested_percent keeps the asked value");
  CHECK(bench->controller.valid_command_count == 4u, "clamped commands are valid commands");

  bench_command(bench, 100);
  MotorOutput output = bench_drive(bench, 100, 600u);
  CHECK(output.duty_ticks == (uint32_t)MOTOR_DUTY_CAP_TICKS, "m 100 runs at the cap: %u",
        (unsigned)output.duty_ticks);
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_reversal(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 0u);
  bench_command(bench, 0);
  MotorOutput output = bench_drive(bench, 20, 400u);
  CHECK(output.duty_ticks == 320u, "setup: running at 320");

  /* m -20: 160 ms down to 0, then 100 ms short brake (enabled, forward,
   * duty 0), then 1 ms with the pins switched and duty 0, then up. */
  uint32_t reversal_start = bench->now_ms;
  bench_command(bench, -20);
  uint32_t ms_until_zero = 0;
  do
  {
    output = bench_step(bench);
    ms_until_zero++;
  } while (output.duty_ticks != 0u && ms_until_zero < 1000u);
  CHECK(ms_until_zero == 160u, "ramp down took %u ms, expected 160", (unsigned)ms_until_zero);
  CHECK(output.driver_enabled && output.direction == MOTOR_DIRECTION_FORWARD && bench->controller.dwell_active,
        "at zero: should brake (enabled, still forward, dwell)");

  uint32_t brake_ms = 1;  /* the ms that reached zero is the first brake ms */
  bench_command(bench, -20);  /* keepalives keep coming */
  while (output.direction == MOTOR_DIRECTION_FORWARD && brake_ms < 1000u)
  {
    output = bench_step(bench);
    if (output.direction == MOTOR_DIRECTION_FORWARD) { brake_ms++; }
    if ((bench->now_ms - reversal_start) % 100u == 0u) { bench_command(bench, -20); }
  }
  CHECK(brake_ms == (uint32_t)MOTOR_REVERSE_DWELL_MS, "short brake lasted %u ms, expected %d",
        (unsigned)brake_ms, MOTOR_REVERSE_DWELL_MS);
  CHECK(output.driver_enabled && output.direction == MOTOR_DIRECTION_REVERSE && output.duty_ticks == 0u,
        "switch ms: reverse with duty 0");
  output = bench_step(bench);
  CHECK(output.direction == MOTOR_DIRECTION_REVERSE && output.duty_ticks == 2u, "first reverse ramp ms");
  output = bench_drive(bench, -20, 300u);
  CHECK(output.duty_ticks == 320u && bench->controller.applied_ticks == -320, "reverse 20 %% reached");

  /* Changing back during the dwell cancels the reversal. */
  bench_command(bench, 20);
  output = bench_run(bench, 160u);  /* down to 0 */
  CHECK(bench->controller.dwell_active, "dwell should be active");
  output = bench_run(bench, 50u);
  bench_command(bench, -10);        /* back to the bridge's side, mid-dwell */
  output = bench_step(bench);
  CHECK(!bench->controller.dwell_active && output.direction == MOTOR_DIRECTION_REVERSE,
        "dwell cancelled, stays reverse");
  output = bench_drive(bench, -10, 200u);
  CHECK(output.direction == MOTOR_DIRECTION_REVERSE && output.duty_ticks == 160u, "reverse 10 %% reached: %u",
        (unsigned)output.duty_ticks);

  /* m 0 during a dwell: disabled at once (duty is already 0). */
  bench_command(bench, 10);
  output = bench_run(bench, 80u);  /* down to 0 at 2/ms from 160 */
  output = bench_run(bench, 20u);
  CHECK(bench->controller.dwell_active, "dwell should be active again");
  bench_command(bench, 0);
  output = bench_step(bench);
  CHECK(!output.driver_enabled && bench->controller.state == MOTOR_STATE_OFF, "m 0 in dwell: disabled, OFF");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_watchdog(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 0u);
  bench_command(bench, 0);
  MotorOutput output = bench_drive(bench, 20, 300u);
  CHECK(output.duty_ticks == 320u, "setup: running");

  /* Last command now; 499 ms later still running, at 500 ms stopped at
   * once (no ramp), latched and counted. */
  bench_command(bench, 20);
  uint32_t last_command_ms = bench->now_ms;
  output = bench_run(bench, 499u);
  CHECK(output.driver_enabled && output.duty_ticks == 320u, "499 ms after the last command: still running");
  output = bench_step(bench);
  CHECK(bench->now_ms - last_command_ms == 500u, "test arithmetic");
  CHECK(!output.driver_enabled && output.duty_ticks == 0u && output.direction == MOTOR_DIRECTION_COAST,
        "500 ms: must be disabled at once");
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED && bench->controller.watchdog_trip_count == 1u,
        "500 ms: latched, trips=%u", (unsigned)bench->controller.watchdog_trip_count);
  CHECK(bench->controller.applied_ticks == 0 && bench->controller.target_ticks == 0, "trip clears the duty");

  /* Commands coming back don't restart the motor until "m 0". */
  output = bench_drive(bench, 20, 500u);
  CHECK(!output.driver_enabled, "after a trip, m 20 must be ignored");
  CHECK(bench->controller.watchdog_trip_count == 1u, "no extra trips while latched");
  bench_command(bench, 0);
  output = bench_drive(bench, 20, 300u);
  CHECK(output.driver_enabled && output.duty_ticks == 320u, "after m 0, m 20 runs again");

  /* Rejected lines don't feed the watchdog. */
  for (int i = 0; i < 6; i++)
  {
    MotorController_CountRejected(&bench->controller);
    output = bench_run(bench, 100u);
  }
  CHECK(!output.driver_enabled && bench->controller.watchdog_trip_count == 2u,
        "only rejected lines for 600 ms: should trip, trips=%u", (unsigned)bench->controller.watchdog_trip_count);
  CHECK(bench->controller.rejected_command_count == 6u, "rejected count");

  /* The watchdog also covers the reverse dwell and the ramp-down. */
  bench_command(bench, 0);
  output = bench_drive(bench, 20, 300u);
  bench_command(bench, -20);
  output = bench_run(bench, 500u);  /* no keepalives: in the dwell at ~260 ms, trip at 500 */
  CHECK(!output.driver_enabled && bench->controller.watchdog_trip_count == 3u, "trip during a reversal");

  /* A loop stall longer than the timeout trips at the next update. */
  bench_command(bench, 0);
  output = bench_drive(bench, 20, 300u);
  bench_command(bench, 20);
  bench->now_ms += 600u;  /* the firmware loop was stuck for 600 ms */
  output = bench_step(bench);
  CHECK(!output.driver_enabled && bench->controller.watchdog_trip_count == 4u, "trip after a 600 ms stall");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_stall_step_limit(void)
{
  /* After a short loop stall, the ramp moves at most 10 ms worth (20 ticks). */
  MotorController controller;
  MotorController_Init(&controller, 0u);
  MotorController_AcceptCommand(&controller, 0, 0u);
  MotorController_AcceptCommand(&controller, 20, 0u);
  MotorOutput output = MotorController_Update(&controller, 1u);  /* wake-up ms */
  MotorController_AcceptCommand(&controller, 20, 50u);
  output = MotorController_Update(&controller, 50u);              /* 49 ms later */
  CHECK(output.duty_ticks == (uint32_t)(MOTOR_RAMP_MAX_STEP_MS * MOTOR_RAMP_TICKS_PER_MS),
        "after a 49 ms gap the duty should be 20, got %u", (unsigned)output.duty_ticks);
}

static void test_tick_wrap(void)
{
  /* Start 300 ms before HAL_GetTick() wraps (49.7 days) and run across it. */
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, UINT32_MAX - 300u);
  bench_command(bench, 0);
  MotorOutput output = bench_drive(bench, 20, 600u);
  CHECK(output.driver_enabled && output.duty_ticks == 320u, "running across the wrap: duty %u",
        (unsigned)output.duty_ticks);
  CHECK(bench->now_ms < 1000u, "the clock should have wrapped (now %" PRIu32 ")", bench->now_ms);

  /* Watchdog across the wrap: last command 100 ms before the wrap. */
  bench_init(bench, UINT32_MAX - 400u);
  bench_command(bench, 0);
  bench_drive(bench, 20, 300u);
  bench_command(bench, 20);
  output = bench_run(bench, 499u);
  CHECK(output.driver_enabled, "across the wrap: 499 ms, still running");
  output = bench_step(bench);
  CHECK(!output.driver_enabled && bench->controller.watchdog_trip_count == 1u, "across the wrap: trip at 500 ms");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

/* FAULT: entered at boot (period mismatch) or from RUNNING (an output-stage
 * guard fired). The driver stays disabled whatever is commanded, "m 0"
 * included; commands are still counted; the first reason is kept. */
static void test_fault_lockout(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;

  /* At boot, like main.c on a period mismatch. */
  bench_init(bench, 1000u);
  MotorController_LockOut(&bench->controller, MOTOR_FAULT_PERIOD_MISMATCH);
  CHECK(bench->controller.state == MOTOR_STATE_FAULT && bench->controller.fault_reason == MOTOR_FAULT_PERIOD_MISMATCH,
        "boot lock-out: state %d reason %d", (int)bench->controller.state, (int)bench->controller.fault_reason);
  uint32_t enabled_ms = 0;
  bench_command(bench, 0);
  for (int round = 0; round < 3; round++)
  {
    for (uint32_t ms = 0; ms < 1000u; ms++)
    {
      if (ms % 100u == 0u) { bench_command(bench, (round == 1) ? 0 : 50); }
      if (bench_step(bench).driver_enabled) { enabled_ms++; }
    }
  }
  CHECK(enabled_ms == 0u && bench->controller.state == MOTOR_STATE_FAULT,
        "FAULT: driver enabled for %u ms, state %d", (unsigned)enabled_ms, (int)bench->controller.state);
  CHECK(bench->controller.valid_command_count == 31u && bench->controller.requested_percent == 50 &&
        bench->controller.clamped_command_count == 20u,
        "FAULT still counts commands: ok=%u pct=%d clamp=%u", (unsigned)bench->controller.valid_command_count,
        (int)bench->controller.requested_percent, (unsigned)bench->controller.clamped_command_count);
  CHECK(bench->controller.watchdog_trip_count == 0u, "no watchdog trips in FAULT");

  /* Random commands, stalls and rejected lines for 100 s: never enabled. */
  for (uint32_t ms = 0; ms < 100000u; ms++)
  {
    uint32_t dice = next_random() % 100u;
    if (dice < 5u) { bench_command(bench, (int32_t)(next_random() % 201u) - 100); }
    else if (dice < 8u) { bench_command(bench, 0); }
    else if (dice == 8u) { MotorController_CountRejected(&bench->controller); }
    else if (dice == 9u) { bench->now_ms += next_random() % 700u; }
    if (bench_step(bench).driver_enabled) { enabled_ms++; }
  }
  CHECK(enabled_ms == 0u && bench->controller.fault_reason == MOTOR_FAULT_PERIOD_MISMATCH,
        "FAULT random run: enabled %u ms, reason %d", (unsigned)enabled_ms, (int)bench->controller.fault_reason);

  /* From RUNNING at 20 %: disabled on the very next update, no ramp. */
  bench_init(bench, 5000u);
  bench_command(bench, 0);
  MotorOutput output = bench_drive(bench, 20, 400);
  CHECK(output.driver_enabled && output.duty_ticks == 320u, "setup: running at 320, got %u",
        (unsigned)output.duty_ticks);
  MotorController_LockOut(&bench->controller, MOTOR_FAULT_CAP_GUARD);
  output = bench_step(bench);
  CHECK(!output.driver_enabled && output.duty_ticks == 0u && bench->controller.applied_ticks == 0,
        "lock-out from RUNNING: next update must be disabled");
  /* A second reason doesn't replace the first, and "m 0" doesn't clear it. */
  MotorController_LockOut(&bench->controller, MOTOR_FAULT_DIRECTION_GUARD);
  bench_command(bench, 0);
  output = bench_drive(bench, 20, 1000);
  CHECK(!output.driver_enabled && bench->controller.state == MOTOR_STATE_FAULT &&
        bench->controller.fault_reason == MOTOR_FAULT_CAP_GUARD,
        "after m 0 and m 20: enabled=%d state %d reason %d", (int)output.driver_enabled,
        (int)bench->controller.state, (int)bench->controller.fault_reason);
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

/* Random commands (valid, clamped, rejected, gaps) for a simulated hour; the
 * invariants in bench_step must hold the whole time. */
static void test_random_sequences(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 12345u);
  uint32_t enabled_ms = 0;
  for (uint32_t ms = 0; ms < 3600u * 1000u; ms++)
  {
    uint32_t dice = next_random() % 1000u;
    if (dice < 10u)
    {
      int32_t percent = (int32_t)(next_random() % 201u) - 100;  /* -100..100 */
      if (next_random() % 4u == 0u) { percent = 0; }
      bench_command(bench, percent);
    }
    else if (dice < 12u)
    {
      MotorController_CountRejected(&bench->controller);
    }
    else if (dice == 12u)
    {
      bench->now_ms += next_random() % 700u;  /* occasional loop stall */
    }
    MotorOutput output = bench_step(bench);
    if (output.driver_enabled) { enabled_ms++; }
  }
  CHECK(bench->invariant_failures == 0, "random run: %d invariant failures", bench->invariant_failures);
  /* Make sure the run actually exercised the interesting paths. */
  CHECK(enabled_ms > 100000u, "random run: driver enabled only %u ms", (unsigned)enabled_ms);
  CHECK(bench->controller.watchdog_trip_count > 10u, "random run: only %u trips",
        (unsigned)bench->controller.watchdog_trip_count);
  CHECK(bench->brake_ms > 1000u, "random run: only %u ms of reverse dwell", (unsigned)bench->brake_ms);
  printf("  random hour: enabled %u ms, %u trips, %u ms short brake, %u valid, %u clamped\n",
         (unsigned)enabled_ms, (unsigned)bench->controller.watchdog_trip_count, (unsigned)bench->brake_ms,
         (unsigned)bench->controller.valid_command_count, (unsigned)bench->controller.clamped_command_count);
}

/* ---- Closed loop: "v" and the speed controller's target ------------------- */

/* Like bench_drive, but a "v" keepalive every 100 ms and a closed-loop target
 * written every 10 ms (as the speed controller will). */
static MotorOutput bench_drive_closed_loop(MotorBench *bench, int32_t setpoint_units, int32_t target_ticks,
                                           uint32_t duration_ms, bool send_v)
{
  MotorOutput output = bench->previous_output;
  for (uint32_t elapsed = 0; elapsed < duration_ms; elapsed++)
  {
    if (send_v && elapsed % 100u == 0u) { MotorController_AcceptSpeedCommand(&bench->controller, setpoint_units, bench->now_ms); }
    if (elapsed % 10u == 0u) { MotorController_SetClosedLoopTarget(&bench->controller, target_ticks); }
    output = bench_step(bench);
  }
  return output;
}

static void test_speed_command_latch_and_watchdog(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 1000u);

  /* Latched at boot: "v 20" counts but neither re-arms nor sets a target;
   * a closed-loop target is ignored too. */
  MotorOutput output = bench_drive_closed_loop(bench, 20, 320, 1000u, true);
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED && !output.driver_enabled && bench->controller.target_ticks == 0,
        "latched: v 20 and a closed-loop target change nothing");
  CHECK(bench->controller.valid_command_count == 10u && bench->controller.requested_percent == 0,
        "v counts as a valid command (cmd_ok), but isn't an m percent (cmd_pct stays)");

  /* "v 0" re-arms, like "m 0". */
  MotorController_AcceptSpeedCommand(&bench->controller, 0, bench->now_ms);
  CHECK(bench->controller.state == MOTOR_STATE_OFF, "v 0 re-arms");

  /* A v stream keeps the watchdog fed while the target comes from the controller. */
  output = bench_drive_closed_loop(bench, 20, 320, 5000u, true);
  CHECK(output.driver_enabled && output.duty_ticks == 320u && bench->controller.watchdog_trip_count == 0u,
        "v every 100 ms + closed-loop target: running at 320, no trip (duty %u, trips %u)",
        (unsigned)output.duty_ticks, (unsigned)bench->controller.watchdog_trip_count);

  /* The closed-loop target alone does NOT feed the watchdog: trip exactly
   * MOTOR_COMMAND_TIMEOUT_MS after the last v. */
  uint32_t last_v_ms = bench->controller.last_command_ms;
  uint32_t trip_ms = 0u;
  for (uint32_t elapsed = 0; elapsed < 2000u && trip_ms == 0u; elapsed++)
  {
    if (elapsed % 10u == 0u) { MotorController_SetClosedLoopTarget(&bench->controller, 320); }
    bench_step(bench);
    if (bench->controller.watchdog_trip_count == 1u) { trip_ms = bench->now_ms; }
  }
  CHECK(trip_ms - last_v_ms == (uint32_t)MOTOR_COMMAND_TIMEOUT_MS && bench->controller.state == MOTOR_STATE_LATCHED,
        "SetClosedLoopTarget alone: trip %u ms after the last v (expected %d)", (unsigned)(trip_ms - last_v_ms),
        MOTOR_COMMAND_TIMEOUT_MS);

  /* Latched again: targets ignored until v 0 / m 0. */
  MotorController_SetClosedLoopTarget(&bench->controller, 200);
  CHECK(bench->controller.target_ticks == 0, "after the trip: target ignored while latched");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_closed_loop_target_clamp_and_states(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 0u);
  MotorController_AcceptSpeedCommand(&bench->controller, 0, bench->now_ms);
  MotorController_SetClosedLoopTarget(&bench->controller, 100000);
  CHECK(bench->controller.target_ticks == MOTOR_DUTY_CAP_TICKS, "clamped to +cap (got %d)", (int)bench->controller.target_ticks);
  MotorController_SetClosedLoopTarget(&bench->controller, -100000);
  CHECK(bench->controller.target_ticks == -MOTOR_DUTY_CAP_TICKS, "clamped to -cap");
  MotorController_SetClosedLoopTarget(&bench->controller, 0);

  /* AcceptSpeedCommand never sets a target, armed or not. */
  MotorController_AcceptSpeedCommand(&bench->controller, 80, bench->now_ms);
  CHECK(bench->controller.target_ticks == 0 && bench->controller.state == MOTOR_STATE_OFF, "v 80 alone: no target");

  /* A sign change through the closed-loop target still goes through the
   * ramp-down, the 100 ms short brake and a duty-0 pin switch (the bench
   * invariants check every ms). */
  MotorOutput output = bench_drive_closed_loop(bench, 50, 400, 2000u, true);
  CHECK(output.duty_ticks == 400u && output.direction == MOTOR_DIRECTION_FORWARD, "forward 400");
  output = bench_drive_closed_loop(bench, -50, -400, 2000u, true);
  CHECK(output.duty_ticks == 400u && output.direction == MOTOR_DIRECTION_REVERSE && bench->brake_ms >= 99u,
        "reversed to -400 through a short brake (%u ms)", (unsigned)bench->brake_ms);

  /* FAULT: v is counted, the target ignored, v 0 doesn't re-arm. */
  MotorController_LockOut(&bench->controller, MOTOR_FAULT_REPEATED_STALL);
  uint32_t valid_before = bench->controller.valid_command_count;
  output = bench_drive_closed_loop(bench, 0, 300, 1000u, true);
  CHECK(bench->controller.state == MOTOR_STATE_FAULT && !output.driver_enabled && bench->controller.target_ticks == 0 &&
        bench->controller.valid_command_count == valid_before + 10u &&
        bench->controller.fault_reason == MOTOR_FAULT_REPEATED_STALL,
        "FAULT: v counted, v 0 doesn't re-arm, target ignored");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_latch(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 0u);
  bench_command(bench, 0);
  MotorOutput output = bench_drive(bench, 30, 1000u);
  CHECK(output.duty_ticks == 480u, "setup: running at 480");

  /* Stop at once: the next ms is disabled, no ramp, latched, not a watchdog trip. */
  MotorController_Latch(&bench->controller, MOTOR_LATCH_STALL_LOCKED);
  output = bench_step(bench);
  CHECK(!output.driver_enabled && output.duty_ticks == 0u && bench->controller.state == MOTOR_STATE_LATCHED &&
        bench->controller.latch_count == 1u && bench->controller.watchdog_trip_count == 0u,
        "Latch: disabled the next ms, LATCHED, latch_count 1, no watchdog trip");
  /* Nothing pending is left behind: telemetry's duty (applied_ticks) shows 0,
   * not the duty before the latch, and no reversal is half done. */
  CHECK(bench->controller.applied_ticks == 0 && bench->controller.target_ticks == 0 && !bench->controller.dwell_active &&
        bench->controller.bridge_direction == MOTOR_DIRECTION_COAST,
        "Latch clears the motion (applied %d, target %d)", (int)bench->controller.applied_ticks,
        (int)bench->controller.target_ticks);

  /* Stays latched through m / v keepalives until m 0 or v 0. */
  output = bench_drive(bench, 30, 1000u);
  CHECK(!output.driver_enabled && bench->controller.state == MOTOR_STATE_LATCHED, "m 30 keepalives don't re-arm");
  MotorController_Latch(&bench->controller, MOTOR_LATCH_STALL_LOCKED);
  CHECK(bench->controller.latch_count == 1u, "Latch while latched: not counted again");
  MotorController_AcceptSpeedCommand(&bench->controller, 0, bench->now_ms);
  CHECK(bench->controller.state == MOTOR_STATE_OFF, "v 0 re-arms after a latch");
  MotorController_Latch(&bench->controller, MOTOR_LATCH_STALL_LOCKED);
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED && bench->controller.latch_count == 2u, "Latch from OFF: latched");
  bench_command(bench, 0);
  CHECK(bench->controller.state == MOTOR_STATE_OFF, "m 0 re-arms after a latch");

  /* FAULT wins: Latch can't turn a FAULT into a latch. */
  MotorController_LockOut(&bench->controller, MOTOR_FAULT_CAP_GUARD);
  MotorController_Latch(&bench->controller, MOTOR_LATCH_STALL_LOCKED);
  CHECK(bench->controller.state == MOTOR_STATE_FAULT && bench->controller.latch_count == 2u, "FAULT stays FAULT");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_latch_reason(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 0u);
  CHECK(bench->controller.latch_reason == MOTOR_LATCH_BOOT, "boot: BOOT");
  CHECK(MOTOR_LATCH_NONE == 0 && MOTOR_LATCH_BOOT == 1 && MOTOR_LATCH_WATCHDOG == 2 && MOTOR_LATCH_STALL_LOCKED == 3 &&
        MOTOR_LATCH_STALL_SATURATION == 4 && MOTOR_LATCH_ESTIMATE_INVALID == 5 && MOTOR_LATCH_ESTOP == 6 &&
        MOTOR_LATCH_LINK_HELLO == 7, "latch reason codes are pinned (Pi link STATUS format)");
  bench_command(bench, 0);
  CHECK(bench->controller.latch_reason == MOTOR_LATCH_NONE, "m 0: NONE");

  /* Watchdog trip while running. */
  (void)bench_drive(bench, 20, 500u);
  (void)bench_run(bench, MOTOR_COMMAND_TIMEOUT_MS + 10u);
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED && bench->controller.latch_reason == MOTOR_LATCH_WATCHDOG,
        "watchdog trip: WATCHDOG");
  /* A second latch request while latched keeps the first reason. */
  MotorController_Latch(&bench->controller, MOTOR_LATCH_ESTOP);
  CHECK(bench->controller.latch_reason == MOTOR_LATCH_WATCHDOG, "Latch while latched: first reason stays");
  MotorController_AcceptSpeedCommand(&bench->controller, 0, bench->now_ms);
  CHECK(bench->controller.state == MOTOR_STATE_OFF && bench->controller.latch_reason == MOTOR_LATCH_NONE, "v 0: NONE");

  /* Each reason passed to Latch is the one kept. */
  for (int reason = MOTOR_LATCH_STALL_LOCKED; reason <= MOTOR_LATCH_LINK_HELLO; reason++)
  {
    MotorController_Latch(&bench->controller, (MotorLatchReason)reason);
    CHECK(bench->controller.latch_reason == (MotorLatchReason)reason, "Latch(%d) kept", reason);
    bench_command(bench, 0);
  }

  /* FAULT: not a latch; no later Latch changes that. */
  MotorController_Latch(&bench->controller, MOTOR_LATCH_ESTOP);
  MotorController_LockOut(&bench->controller, MOTOR_FAULT_LOW_BATTERY);
  CHECK(bench->controller.latch_reason == MOTOR_LATCH_NONE, "LockOut from LATCHED: NONE");
  MotorController_Latch(&bench->controller, MOTOR_LATCH_ESTOP);
  CHECK(bench->controller.state == MOTOR_STATE_FAULT && bench->controller.latch_reason == MOTOR_LATCH_NONE,
        "Latch in FAULT: still FAULT, NONE");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_drive_command_arms_only_when_asked(void)
{
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 0u);

  /* DRIVE without arm: counted, feeds the watchdog, never re-arms. */
  uint32_t valid_before = bench->controller.valid_command_count;
  (void)bench_run(bench, 50u);
  MotorController_AcceptDriveCommand(&bench->controller, false, bench->now_ms);
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED && bench->controller.latch_reason == MOTOR_LATCH_BOOT &&
        bench->controller.valid_command_count == valid_before + 1u && bench->controller.last_command_ms == bench->now_ms,
        "DRIVE without arm: counted, watchdog fed, still LATCHED (BOOT)");
  MotorController_AcceptDriveCommand(&bench->controller, true, bench->now_ms);
  CHECK(bench->controller.state == MOTOR_STATE_OFF && bench->controller.latch_reason == MOTOR_LATCH_NONE &&
        bench->controller.target_ticks == 0, "DRIVE with arm: OFF, NONE, no target");

  /* Running on closed-loop targets with DRIVE keepalives: no watchdog trip. */
  for (int i = 0; i < 3000; i++)
  {
    if (i % 100 == 0) { MotorController_AcceptDriveCommand(&bench->controller, false, bench->now_ms); }
    MotorController_SetClosedLoopTarget(&bench->controller, 300);
    (void)bench_step(bench);
  }
  CHECK(bench->controller.state == MOTOR_STATE_RUNNING && bench->controller.applied_ticks == 300 &&
        bench->controller.watchdog_trip_count == 0u, "DRIVE keepalives keep it running at 300");
  /* Arm while running changes nothing. */
  MotorController_AcceptDriveCommand(&bench->controller, true, bench->now_ms);
  CHECK(bench->controller.state == MOTOR_STATE_RUNNING, "arm while RUNNING: no effect");

  /* ESTOP-style latch, then DRIVE 0 without arm: stays latched. */
  MotorController_Latch(&bench->controller, MOTOR_LATCH_ESTOP);
  (void)bench_step(bench);
  for (int i = 0; i < 10; i++)
  {
    MotorController_AcceptDriveCommand(&bench->controller, false, bench->now_ms);
    (void)bench_run(bench, 100u);
  }
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED && bench->controller.latch_reason == MOTOR_LATCH_ESTOP,
        "after ESTOP, plain DRIVEs keep it latched (ESTOP)");
  MotorController_AcceptDriveCommand(&bench->controller, true, bench->now_ms);
  CHECK(bench->controller.state == MOTOR_STATE_OFF, "after ESTOP, an arming DRIVE re-arms");

  /* Watchdog: no DRIVE for 500 ms while running trips it. */
  MotorController_SetClosedLoopTarget(&bench->controller, 200);
  MotorController_AcceptDriveCommand(&bench->controller, false, bench->now_ms);
  (void)bench_run(bench, MOTOR_COMMAND_TIMEOUT_MS + 5u);
  CHECK(bench->controller.state == MOTOR_STATE_LATCHED && bench->controller.latch_reason == MOTOR_LATCH_WATCHDOG,
        "no DRIVE for 500 ms: watchdog latch");

  /* FAULT: counted, never re-armed. */
  MotorController_LockOut(&bench->controller, MOTOR_FAULT_REPEATED_STALL);
  MotorController_AcceptDriveCommand(&bench->controller, true, bench->now_ms);
  CHECK(bench->controller.state == MOTOR_STATE_FAULT, "FAULT: an arming DRIVE changes nothing");
  CHECK(bench->invariant_failures == 0, "%d invariant failures", bench->invariant_failures);
}

static void test_random_sequences_with_closed_loop(void)
{
  /* The same invariants with m, v, closed-loop targets, latches and
   * rejected lines mixed at random. */
  MotorBench bench_storage;
  MotorBench *bench = &bench_storage;
  bench_init(bench, 777u);
  uint32_t enabled_ms = 0;
  for (uint32_t ms = 0; ms < 3600u * 1000u; ms++)
  {
    uint32_t dice = next_random() % 1000u;
    if (dice < 5u)
    {
      int32_t percent = (int32_t)(next_random() % 201u) - 100;
      if (next_random() % 4u == 0u) { percent = 0; }
      bench_command(bench, percent);
    }
    else if (dice < 10u)
    {
      int32_t setpoint = (int32_t)(next_random() % 201u) - 100;
      if (next_random() % 4u == 0u) { setpoint = 0; }
      MotorController_AcceptSpeedCommand(&bench->controller, setpoint, bench->now_ms);
    }
    else if (dice < 110u)
    {
      MotorController_SetClosedLoopTarget(&bench->controller, (int32_t)(next_random() % 1201u) - 600);
    }
    else if (dice == 110u)
    {
      MotorController_Latch(&bench->controller, MOTOR_LATCH_STALL_LOCKED);
    }
    else if (dice == 111u)
    {
      bench->now_ms += next_random() % 700u;
    }
    MotorOutput output = bench_step(bench);
    if (output.driver_enabled) { enabled_ms++; }
  }
  CHECK(bench->invariant_failures == 0, "random closed-loop run: %d invariant failures", bench->invariant_failures);
  CHECK(enabled_ms > 100000u && bench->controller.latch_count > 100u && bench->controller.watchdog_trip_count > 10u &&
        bench->brake_ms > 1000u, "random closed-loop run exercised it (enabled %u ms, %u latches, %u trips, %u ms brake)",
        (unsigned)enabled_ms, (unsigned)bench->controller.latch_count, (unsigned)bench->controller.watchdog_trip_count,
        (unsigned)bench->brake_ms);
}

int main(void)
{
  test_byte_ring();
  test_latched_at_boot();
  test_ramp_up_and_stop();
  test_clamp();
  test_reversal();
  test_watchdog();
  test_stall_step_limit();
  test_tick_wrap();
  test_fault_lockout();
  test_random_sequences();
  test_speed_command_latch_and_watchdog();
  test_closed_loop_target_clamp_and_states();
  test_latch();
  test_latch_reason();
  test_drive_command_arms_only_when_asked();
  test_random_sequences_with_closed_loop();

  printf("%d checks, %d failed\n", check_count, failure_count);
  if (failure_count != 0)
  {
    return EXIT_FAILURE;
  }
  printf("ALL PASSED\n");
  return EXIT_SUCCESS;
}
