/*
 * Host-side test for Core/Src/steering_control.c. Not part of the firmware
 * build or CI. Compiles it with the PC's C compiler.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_steering_control.c Core/Src/steering_control.c -o /tmp/test_steering_control && /tmp/test_steering_control
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "steering_control.h"

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
static uint32_t random_state = 0x13579BDFu;
static uint32_t next_random(void)
{
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

/* Width of each half of the mapping (s > 0 and s < 0), in us. */
#define LEFT_SPAN_US   (SERVO_PULSE_MAX_US - SERVO_PULSE_CENTER_US)
#define RIGHT_SPAN_US  (SERVO_PULSE_CENTER_US - SERVO_PULSE_MIN_US)

static int pulse_in_limits(uint32_t pulse_us)
{
  return pulse_us >= (uint32_t)SERVO_PULSE_MIN_US && pulse_us <= (uint32_t)SERVO_PULSE_MAX_US;
}

/* ---- Mapping ---------------------------------------------------------------- */

static void test_mapping_fixed_points(void)
{
  /* The defaults; if the limits are recalibrated, these follow from the
   * same formula, so they are written in terms of the constants. */
  CHECK(Steering_PulseForPercent(0, false) == (uint32_t)SERVO_PULSE_CENTER_US, "0 -> centre");
  CHECK(Steering_PulseForPercent(100, false) == (uint32_t)SERVO_PULSE_MAX_US, "+100 -> MAX");
  CHECK(Steering_PulseForPercent(-100, false) == (uint32_t)SERVO_PULSE_MIN_US, "-100 -> MIN");
  CHECK(Steering_PulseForPercent(0, true) == (uint32_t)SERVO_PULSE_CENTER_US, "inverted 0 -> centre");
  CHECK(Steering_PulseForPercent(100, true) == (uint32_t)SERVO_PULSE_MIN_US, "inverted +100 -> MIN");
  CHECK(Steering_PulseForPercent(-100, true) == (uint32_t)SERVO_PULSE_MAX_US, "inverted -100 -> MAX");

  /* +-50 is half of that side's span, +-1 one hundredth of it (rounded
   * toward the centre). With 1200/1500/1800: 1650 / 1350, 1503 / 1497. */
  CHECK(Steering_PulseForPercent(50, false) == (uint32_t)(SERVO_PULSE_CENTER_US + LEFT_SPAN_US / 2),
        "+50 -> %u", (unsigned)Steering_PulseForPercent(50, false));
  CHECK(Steering_PulseForPercent(-50, false) == (uint32_t)(SERVO_PULSE_CENTER_US - RIGHT_SPAN_US / 2),
        "-50 -> %u", (unsigned)Steering_PulseForPercent(-50, false));
  CHECK(Steering_PulseForPercent(1, false) == (uint32_t)(SERVO_PULSE_CENTER_US + LEFT_SPAN_US / 100),
        "+1 -> %u", (unsigned)Steering_PulseForPercent(1, false));
  CHECK(Steering_PulseForPercent(-1, false) == (uint32_t)(SERVO_PULSE_CENTER_US - RIGHT_SPAN_US / 100),
        "-1 -> %u", (unsigned)Steering_PulseForPercent(-1, false));

  /* Beyond +-100 (never produced by the parser): held at the endpoints. */
  static const int32_t beyond[] = { 101, 150, 1000, INT32_MAX, -101, -150, -1000, INT32_MIN };
  for (size_t i = 0; i < sizeof(beyond) / sizeof(beyond[0]); i++)
  {
    for (int invert = 0; invert <= 1; invert++)
    {
      uint32_t pulse_us = Steering_PulseForPercent(beyond[i], invert != 0);
      int expect_max = (beyond[i] > 0) != (invert != 0);
      CHECK(pulse_us == (uint32_t)(expect_max ? SERVO_PULSE_MAX_US : SERVO_PULSE_MIN_US),
            "percent %" PRId32 " invert %d -> %u", beyond[i], invert, (unsigned)pulse_us);
    }
  }
}

static void test_mapping_every_percent(void)
{
  /* Every command: inside the limits, monotonic, and inverting is the same as
   * negating the command. */
  int in_limits = 1;
  int monotonic = 1;
  int inversion_ok = 1;
  uint32_t previous_pulse_us = 0u;
  for (int32_t percent = -100; percent <= 100; percent++)
  {
    uint32_t pulse_us = Steering_PulseForPercent(percent, false);
    if (!pulse_in_limits(pulse_us)) { in_limits = 0; }
    if (percent > -100 && pulse_us < previous_pulse_us) { monotonic = 0; }
    if (Steering_PulseForPercent(percent, true) != Steering_PulseForPercent(-percent, false)) { inversion_ok = 0; }
    previous_pulse_us = pulse_us;
  }
  CHECK(in_limits, "a command mapped outside [MIN, MAX]");
  CHECK(monotonic, "mapping is not monotonic");
  CHECK(inversion_ok, "invert_direction is not the same as negating the command");
}

/* ---- State machine ------------------------------------------------------------ */

static void test_off_until_first_command(void)
{
  SteeringController controller;
  SteeringController_Init(&controller, 1000u, false);
  CHECK(controller.state == STEERING_STATE_OFF, "starts OFF");
  uint32_t pulsed_ms = 0u;
  for (uint32_t now_ms = 1001u; now_ms <= 11000u; now_ms++)
  {
    if (SteeringController_Update(&controller, now_ms) != 0u) { pulsed_ms++; }
  }
  CHECK(pulsed_ms == 0u, "OFF must not pulse, pulsed %u ms", (unsigned)pulsed_ms);
  CHECK(controller.state == STEERING_STATE_OFF && controller.timeout_count == 0u,
        "OFF never times out into centre (state %d, timeouts %u)", (int)controller.state,
        (unsigned)controller.timeout_count);

  SteeringController_AcceptCommand(&controller, 50, 11000u);
  CHECK(controller.state == STEERING_STATE_ACTIVE, "first s -> ACTIVE");
  CHECK(SteeringController_Update(&controller, 11001u) == (uint32_t)(SERVO_PULSE_CENTER_US + LEFT_SPAN_US / 2),
        "s 50 -> half of the left span");
  CHECK(controller.valid_command_count == 1u, "counted");
}

static void test_timeout_to_centre(void)
{
  SteeringController controller;
  SteeringController_Init(&controller, 0u, false);
  SteeringController_AcceptCommand(&controller, -100, 100u);

  /* 499 ms after the last command: still following it. 500 ms: centre. */
  CHECK(SteeringController_Update(&controller, 599u) == (uint32_t)SERVO_PULSE_MIN_US, "499 ms: still MIN");
  CHECK(controller.state == STEERING_STATE_ACTIVE && controller.timeout_count == 0u, "499 ms: ACTIVE");
  CHECK(SteeringController_Update(&controller, 600u) == (uint32_t)SERVO_PULSE_CENTER_US, "500 ms: centre");
  CHECK(controller.state == STEERING_STATE_TIMED_OUT && controller.timeout_count == 1u, "500 ms: TIMED_OUT, 1");

  /* Stays at centre, counted once. */
  uint32_t not_centre_ms = 0u;
  for (uint32_t now_ms = 601u; now_ms < 10601u; now_ms++)
  {
    if (SteeringController_Update(&controller, now_ms) != (uint32_t)SERVO_PULSE_CENTER_US) { not_centre_ms++; }
  }
  CHECK(not_centre_ms == 0u && controller.timeout_count == 1u, "TIMED_OUT: off centre %u ms, timeouts %u",
        (unsigned)not_centre_ms, (unsigned)controller.timeout_count);
  CHECK(controller.requested_percent == -100, "s_pct keeps the last command");

  /* The next command takes over at once. */
  SteeringController_AcceptCommand(&controller, 100, 10601u);
  CHECK(SteeringController_Update(&controller, 10601u) == (uint32_t)SERVO_PULSE_MAX_US, "after timeout, s 100 -> MAX");
  CHECK(controller.state == STEERING_STATE_ACTIVE, "back to ACTIVE");

  /* Keepalives every 100 ms for a minute: never times out. */
  for (uint32_t now_ms = 10602u; now_ms < 70602u; now_ms++)
  {
    if (now_ms % 100u == 0u) { SteeringController_AcceptCommand(&controller, 100, now_ms); }
    SteeringController_Update(&controller, now_ms);
  }
  CHECK(controller.timeout_count == 1u && controller.state == STEERING_STATE_ACTIVE,
        "keepalives every 100 ms: timeouts %u", (unsigned)controller.timeout_count);

  /* A loop stall longer than the timeout: centre at the next update. */
  SteeringController_AcceptCommand(&controller, 30, 80000u);
  CHECK(SteeringController_Update(&controller, 80700u) == (uint32_t)SERVO_PULSE_CENTER_US &&
        controller.timeout_count == 2u, "700 ms stall -> centre");
}

static void test_tick_wrap(void)
{
  SteeringController controller;
  SteeringController_Init(&controller, UINT32_MAX - 1000u, false);
  SteeringController_AcceptCommand(&controller, 100, UINT32_MAX - 100u);
  /* 499 ms later (across the wrap) still active, 500 ms centre. */
  CHECK(SteeringController_Update(&controller, UINT32_MAX - 100u + 499u) == (uint32_t)SERVO_PULSE_MAX_US,
        "across the wrap: 499 ms still MAX");
  CHECK(SteeringController_Update(&controller, UINT32_MAX - 100u + 500u) == (uint32_t)SERVO_PULSE_CENTER_US,
        "across the wrap: 500 ms centre");
}

static void test_fault(void)
{
  SteeringController controller;
  SteeringController_Init(&controller, 0u, false);
  SteeringController_AcceptCommand(&controller, 40, 0u);
  CHECK(SteeringController_Update(&controller, 1u) != 0u, "setup: pulsing");
  SteeringController_LockOut(&controller);
  CHECK(controller.state == STEERING_STATE_FAULT && controller.pulse_us == 0u, "lock-out: FAULT, no pulse");

  /* Commands, stalls and time: never pulses again, commands still counted. */
  uint32_t pulsed_ms = 0u;
  uint32_t now_ms = 1u;
  for (uint32_t i = 0; i < 100000u; i++)
  {
    uint32_t dice = next_random() % 100u;
    if (dice < 10u) { SteeringController_AcceptCommand(&controller, (int32_t)(next_random() % 201u) - 100, now_ms); }
    else if (dice == 10u) { now_ms += next_random() % 2000u; }
    if (SteeringController_Update(&controller, ++now_ms) != 0u) { pulsed_ms++; }
  }
  CHECK(pulsed_ms == 0u && controller.state == STEERING_STATE_FAULT, "FAULT: pulsed %u ms", (unsigned)pulsed_ms);
  CHECK(controller.valid_command_count > 1000u, "FAULT still counts commands (%u)",
        (unsigned)controller.valid_command_count);
  CHECK(controller.timeout_count == 0u, "no timeouts in FAULT");

  /* Lock-out from OFF too. */
  SteeringController_Init(&controller, 0u, false);
  SteeringController_LockOut(&controller);
  SteeringController_AcceptCommand(&controller, 10, 5u);
  CHECK(SteeringController_Update(&controller, 6u) == 0u && controller.state == STEERING_STATE_FAULT,
        "lock-out from OFF: a command doesn't start pulses");
}

/* ---- Output guard ------------------------------------------------------------- */

static void test_output_guard(void)
{
  /* Rule 1: wrong timer -> nothing driven, whatever the pulse. */
  static const struct { uint32_t tick_hz; uint32_t period; } wrong_timers[] = {
    { 999999u, SERVO_PWM_PERIOD_TICKS }, { 1000001u, SERVO_PWM_PERIOD_TICKS }, { 32000000u, SERVO_PWM_PERIOD_TICKS },
    { SERVO_TIMER_TICK_HZ, 19999u }, { SERVO_TIMER_TICK_HZ, 20001u }, { SERVO_TIMER_TICK_HZ, 1600u },
    { SERVO_TIMER_TICK_HZ, 0u }, { 0u, 0u },
  };
  for (size_t i = 0; i < sizeof(wrong_timers) / sizeof(wrong_timers[0]); i++)
  {
    ServoPlan plan = Steering_PlanOutput(1500u, wrong_timers[i].tick_hz, wrong_timers[i].period);
    CHECK(plan.compare_ticks == 0u && plan.timer_refused, "timer %u Hz / %u ticks should be refused",
          (unsigned)wrong_timers[i].tick_hz, (unsigned)wrong_timers[i].period);
  }

  /* Rule 2: no pulses asked -> none. */
  ServoPlan plan = Steering_PlanOutput(0u, SERVO_TIMER_TICK_HZ, SERVO_PWM_PERIOD_TICKS);
  CHECK(plan.compare_ticks == 0u && !plan.timer_refused && !plan.out_of_range, "pulse 0 -> compare 0, no flags");

  /* Rules 3 and 4, every pulse from 1 us to 3000 us plus the extremes. */
  int all_ok = 1;
  for (uint32_t pulse_us = 1u; pulse_us <= 3000u; pulse_us++)
  {
    plan = Steering_PlanOutput(pulse_us, SERVO_TIMER_TICK_HZ, SERVO_PWM_PERIOD_TICKS);
    int expected_out = !pulse_in_limits(pulse_us);
    uint32_t expected_compare = pulse_us < (uint32_t)SERVO_PULSE_MIN_US ? (uint32_t)SERVO_PULSE_MIN_US
                              : pulse_us > (uint32_t)SERVO_PULSE_MAX_US ? (uint32_t)SERVO_PULSE_MAX_US
                              : pulse_us;
    if (plan.compare_ticks != expected_compare || plan.out_of_range != expected_out || plan.timer_refused)
    {
      if (all_ok) { printf("  guard: pulse %u -> compare %u out %d\n", (unsigned)pulse_us,
                           (unsigned)plan.compare_ticks, (int)plan.out_of_range); }
      all_ok = 0;
    }
  }
  CHECK(all_ok, "guard: pulses 1..3000 us");
  plan = Steering_PlanOutput(UINT32_MAX, SERVO_TIMER_TICK_HZ, SERVO_PWM_PERIOD_TICKS);
  CHECK(plan.compare_ticks == (uint32_t)SERVO_PULSE_MAX_US && plan.out_of_range, "UINT32_MAX -> MAX, flagged");
  plan = Steering_PlanOutput(SERVO_PWM_PERIOD_TICKS, SERVO_TIMER_TICK_HZ, SERVO_PWM_PERIOD_TICKS);
  CHECK(plan.compare_ticks == (uint32_t)SERVO_PULSE_MAX_US && plan.out_of_range, "a 100 %% duty pulse -> MAX");
}

/* ---- Random hour against an independent model --------------------------------- */

/* The rules restated in the simplest way, to compare against ms by ms:
 * never commanded -> 0; commanded and the last command < 500 ms ago -> the
 * mapped pulse; otherwise centre. Timeouts = number of times the "< 500 ms"
 * condition went from true to false. */
static void test_random_hour(void)
{
  SteeringController controller;
  uint32_t now_ms = 777u;
  SteeringController_Init(&controller, now_ms, false);
  int commanded = 0;
  int was_fresh = 0;
  uint32_t model_last_ms = 0u;
  int32_t model_percent = 0;
  uint32_t model_timeouts = 0u;
  uint32_t mismatches = 0u;
  uint32_t centre_ms = 0u;
  uint32_t active_ms = 0u;

  for (uint32_t i = 0; i < 3600u * 1000u; i++)
  {
    now_ms++;
    uint32_t dice = next_random() % 1000u;
    if (dice < 8u)
    {
      int32_t percent = (int32_t)(next_random() % 201u) - 100;
      SteeringController_AcceptCommand(&controller, percent, now_ms);
      commanded = 1;
      model_last_ms = now_ms;
      model_percent = percent;
    }
    else if (dice == 8u)
    {
      now_ms += next_random() % 900u;  /* occasional loop stall */
    }

    uint32_t pulse_us = SteeringController_Update(&controller, now_ms);
    int fresh = commanded && (now_ms - model_last_ms < (uint32_t)STEERING_COMMAND_TIMEOUT_MS);
    if (was_fresh && !fresh) { model_timeouts++; }
    was_fresh = fresh;
    uint32_t expected_us = !commanded ? 0u
                         : fresh ? Steering_PulseForPercent(model_percent, false)
                         : (uint32_t)SERVO_PULSE_CENTER_US;
    if (pulse_us != expected_us)
    {
      if (mismatches < 5u)
      {
        printf("  model mismatch at t=%" PRIu32 ": got %u, expected %u\n", now_ms, (unsigned)pulse_us,
               (unsigned)expected_us);
      }
      mismatches++;
    }
    if (pulse_us != 0u && !pulse_in_limits(pulse_us)) { mismatches++; }
    if (pulse_us == (uint32_t)SERVO_PULSE_CENTER_US && !fresh) { centre_ms++; }
    if (fresh) { active_ms++; }
  }
  CHECK(mismatches == 0u, "random hour: %u mismatches with the model", (unsigned)mismatches);
  CHECK(controller.timeout_count == model_timeouts, "random hour: %u timeouts, model %u",
        (unsigned)controller.timeout_count, (unsigned)model_timeouts);
  /* Make sure the run exercised both states. */
  CHECK(model_timeouts > 100u && centre_ms > 100000u && active_ms > 100000u,
        "random hour too tame: %u timeouts, %u ms centred, %u ms active", (unsigned)model_timeouts,
        (unsigned)centre_ms, (unsigned)active_ms);
  printf("  random hour: %u commands, %u timeouts, %u ms active, %u ms centred\n",
         (unsigned)controller.valid_command_count, (unsigned)model_timeouts, (unsigned)active_ms,
         (unsigned)centre_ms);
}

int main(void)
{
  test_mapping_fixed_points();
  test_mapping_every_percent();
  test_off_until_first_command();
  test_timeout_to_centre();
  test_tick_wrap();
  test_fault();
  test_output_guard();
  test_random_hour();

  printf("%d checks, %d failed\n", check_count, failure_count);
  if (failure_count != 0)
  {
    return EXIT_FAILURE;
  }
  printf("ALL PASSED\n");
  return EXIT_SUCCESS;
}
