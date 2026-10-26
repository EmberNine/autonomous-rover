/*
 * Host-side test for the saturation-stall rule in Core/Src/stall_detect.c,
 * in the closed loop: the real motor_control, speed_control, speed_estimator
 * and stall_detect, in main.c's order every ms, with a model of the wheel on
 * the lift (fitted to the 2026-10-15 baseline and Group D bench logs) and of
 * what a hand does to it. Not part of the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_saturation_stall.c Core/Src/stall_detect.c Core/Src/speed_control.c Core/Src/speed_estimator.c Core/Src/motor_control.c -lm -o /tmp/test_saturation_stall && /tmp/test_saturation_stall
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stall_detect.h"

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

/* ---- The wheel on the lift ------------------------------------------------------------ */
/* 12.3 counts/s per tick above a 104.5-tick dead zone (two-point fit of the
 * baseline m 20 / m 30 logs), breakaway from rest at 140 ticks, tau 30 ms
 * driven; disabled it stops within ~0.1 s (bench: a watchdog trip stopped
 * it in under ~70 ms), modelled as tau 25 ms. 1 speed unit = 0.05 counts/ms. */
#define PLANT_GAIN        0.0123
#define PLANT_DEAD_ZONE   104.5
#define PLANT_BREAKAWAY   140.0
#define PLANT_TAU_MS      30.0
#define PLANT_COAST_TAU   25.0
#define UNITS_PER_COUNT_MS 20.0

typedef struct
{
  MotorController motor;
  SpeedController speed;
  SpeedEstimator  estimator;
  StallDetector   stall;
  uint32_t        now_ms;
  double          wheel;          /* counts/ms */
  double          position;
  double          load;           /* 1 = free; < 1: a drag that takes duty */
  double          tau_ms;         /* driven time constant: PLANT_TAU_MS on the lift, longer for a vehicle's inertia */
  double          hold_units;     /* < 0: no hand; else the hand lets the wheel slip at most this fast */
  bool            keepalive_is_v;
  int32_t         keepalive_value;
} Chain;

static void send_v(Chain *chain, int32_t units)
{
  MotorController_AcceptSpeedCommand(&chain->motor, units, chain->now_ms);
  SpeedControl_SetSetpoint(&chain->speed, units, chain->motor.applied_ticks);
  chain->keepalive_is_v = true;
  chain->keepalive_value = units;
}

static void send_m(Chain *chain, int32_t percent)
{
  MotorController_AcceptCommand(&chain->motor, percent, chain->now_ms);
  SpeedControl_SelectOpenLoop(&chain->speed);
  chain->keepalive_is_v = false;
  chain->keepalive_value = percent;
}

static void chain_init(Chain *chain, uint32_t start_ms)
{
  memset(chain, 0, sizeof(*chain));
  chain->now_ms = start_ms;
  MotorController_Init(&chain->motor, start_ms);
  SpeedControl_Init(&chain->speed);
  SpeedEstimator_Init(&chain->estimator);
  StallDetect_Init(&chain->stall);
  chain->load = 1.0;
  chain->tau_ms = PLANT_TAU_MS;
  chain->hold_units = -1.0;
  chain->position = 777.4;
}

/* One ms in main.c's order: keepalive, speed control, stall check, motor, wheel, read, estimate. */
static void chain_step(Chain *chain)
{
  chain->now_ms++;
  if (chain->now_ms % 100u == 0u)
  {
    if (chain->keepalive_is_v) { send_v(chain, chain->keepalive_value); } else { send_m(chain, chain->keepalive_value); }
  }
  SpeedControl_Update(&chain->speed, &chain->motor, &chain->estimator, chain->now_ms);
  StallDetect_Update(&chain->stall, &chain->motor, &chain->speed, &chain->estimator, chain->now_ms);
  MotorOutput output = MotorController_Update(&chain->motor, chain->now_ms);

  double duty = output.driver_enabled
                    ? ((output.direction == MOTOR_DIRECTION_REVERSE) ? -(double)output.duty_ticks : (double)output.duty_ticks)
                    : 0.0;
  if (!output.driver_enabled)
  {
    chain->wheel *= exp(-1.0 / PLANT_COAST_TAU);
  }
  else if (fabs(chain->wheel) < 0.02 && fabs(duty) < PLANT_BREAKAWAY)
  {
    chain->wheel = 0.0;
  }
  else
  {
    double target = copysign(fmax(fabs(duty) - PLANT_DEAD_ZONE, 0.0), duty) * PLANT_GAIN * chain->load;
    chain->wheel += (target - chain->wheel) * (1.0 - exp(-1.0 / chain->tau_ms));
  }
  if (chain->hold_units >= 0.0)
  {
    double limit = chain->hold_units / UNITS_PER_COUNT_MS;
    chain->wheel = fmax(-limit, fmin(limit, chain->wheel));
  }
  chain->position += chain->wheel;

  SpeedEstimatorInput input = { .now_ms = chain->now_ms, .count = (int32_t)floor(chain->position), .read_ok = true,
                                .magnet_detected = true };
  SpeedEstimator_Update(&chain->estimator, &input);
}

/* What a run of ms did. */
typedef struct
{
  uint32_t latched_after;     /* ms after the start the motor left RUNNING; 0 = never */
  uint32_t first_cap;         /* ms after the start the applied duty first reached the cap; 0 = never */
  uint32_t longest_cap_run;   /* longest stretch at the cap, ms */
  uint32_t at_cap;            /* ms at the cap */
  int32_t  min_speed;         /* smallest estimate in the setpoint's direction while running (after settle_ms) */
  double   mean_speed;        /* mean estimate over the last 1 s while running */
  int32_t  max_duty;          /* largest |applied| */
  uint32_t longest_sat_hold;  /* longest stretch the saturation rule counted (cap AND below half), ms */
} Run;

static Run chain_run(Chain *chain, uint32_t ms, int32_t sign, uint32_t settle_ms)
{
  Run run = { 0u, 0u, 0u, 0u, 100000, 0.0, 0, 0u };
  uint32_t cap_run = 0u;
  uint32_t sat_run = 0u;
  double sum = 0.0;
  int n = 0;
  for (uint32_t t = 1u; t <= ms; t++)
  {
    chain_step(chain);
    if (chain->motor.state != MOTOR_STATE_RUNNING)
    {
      if (run.latched_after == 0u && chain->motor.state != MOTOR_STATE_OFF) { run.latched_after = t; }
      continue;
    }
    sat_run = chain->stall.sat_holding ? sat_run + 1u : 0u;
    if (sat_run > run.longest_sat_hold) { run.longest_sat_hold = sat_run; }
    int32_t duty = abs(chain->motor.applied_ticks);
    if (duty > run.max_duty) { run.max_duty = duty; }
    if (duty == MOTOR_DUTY_CAP_TICKS)
    {
      if (run.first_cap == 0u) { run.first_cap = t; }
      run.at_cap++;
      cap_run++;
      if (cap_run > run.longest_cap_run) { run.longest_cap_run = cap_run; }
    }
    else
    {
      cap_run = 0u;
    }
    int32_t toward = sign * chain->estimator.speed_units;
    if (t > settle_ms && toward < run.min_speed) { run.min_speed = toward; }
    if (t + 1000u > ms) { sum += chain->estimator.speed_units; n++; }
  }
  run.mean_speed = (n > 0) ? sum / n : 0.0;
  return run;
}

/* Armed ("v 0"), then "v units" for settle_ms. */
static void chain_start_v(Chain *chain, int32_t units, uint32_t settle_ms)
{
  send_v(chain, 0);
  for (int i = 0; i < 200; i++) { chain_step(chain); }
  send_v(chain, units);
  for (uint32_t i = 0; i < settle_ms; i++) { chain_step(chain); }
}

/* ---- (a) the creep the first rule can't see ------------------------------------------- */

static void test_creeping_hold_at_v_20_latches_one_second_after_the_cap(void)
{
  static const double creeps[] = { 5.0, 6.0, 7.0, 9.0 };
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    for (size_t i = 0; i < sizeof creeps / sizeof creeps[0]; i++)
    {
      Chain chain;
      chain_init(&chain, 10000u);
      chain_start_v(&chain, sign * 20, 2000u);
      CHECK(chain.motor.state == MOTOR_STATE_RUNNING && abs(chain.estimator.speed_units - sign * 20) <= 3,
            "v %+d running before the hold (%d)", sign * 20, (int)chain.estimator.speed_units);
      chain.hold_units = creeps[i];                   /* a gloved hand, the wheel slipping slowly */
      Run run = chain_run(&chain, 5000u, sign, 100u);
      if (sign > 0 && i == 1)
      {
        printf("  v 20, hold slipping at %.0f units: duty at the cap %u ms after the hold, latch %u ms after the hold\n",
               creeps[i], (unsigned)run.first_cap, (unsigned)run.latched_after);
      }
      CHECK(run.first_cap > 0u && run.latched_after > run.first_cap, "creep %.0f, sign %+d: the PI winds up to the cap",
            creeps[i], sign);
      CHECK(run.latched_after - run.first_cap >= STALL_SAT_HOLD_MS && run.latched_after - run.first_cap <= STALL_SAT_HOLD_MS + 2u,
            "creep %.0f, sign %+d: latch 1 s after the cap (%u ms)", creeps[i], sign,
            (unsigned)(run.latched_after - run.first_cap));
      CHECK(run.min_speed >= STALL_MAX_SPEED_UNITS, "creep %.0f: never below 2 units (%d): the first rule can't fire",
            creeps[i], (int)run.min_speed);
      CHECK(chain.motor.state == MOTOR_STATE_LATCHED && chain.stall.stall_count == 1u &&
            chain.speed.enc_bad_count == 0u && chain.motor.applied_ticks == 0,
            "creep %.0f, sign %+d: LATCHED, stall_n 1, stopped", creeps[i], sign);
      send_v(&chain, 0);
      chain_step(&chain);
      CHECK(chain.motor.state == MOTOR_STATE_OFF, "v 0 re-arms");
    }
  }
}

/* ---- (b) v 100 saturates on the lift but stays above half ------------------------------ */

static void test_v_100_on_the_lift_never_latches(void)
{
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    Chain chain;
    chain_init(&chain, 10000u);
    chain_start_v(&chain, sign * 100, 0u);
    Run run = chain_run(&chain, 60000u, sign, 1000u);
    if (sign > 0)
    {
      printf("  v 100 for 60 s: %.1f %% at the cap, longest %u ms, speed %.1f units\n", 100.0 * run.at_cap / 60000.0,
             (unsigned)run.longest_cap_run, run.mean_speed);
    }
    CHECK(run.latched_after == 0u && chain.motor.state == MOTOR_STATE_RUNNING && chain.stall.stall_count == 0u,
          "v %+d for 60 s: never latches", sign * 100);
    CHECK(run.at_cap > 55000u && fabs(fabs(run.mean_speed) - 92.6) < 3.0,
          "and it is saturated (%u ms at the cap) at ~93 units (%.1f)", (unsigned)run.at_cap, run.mean_speed);
  }
}

/* ---- (c) drags: below the cap, at the cap above half, at the cap below half -------------- */

static void test_drags(void)
{
  /* load: d_drag_1 needed ~436 ticks for 50 units; 0.325 gives 30 units at the
   * cap (above half), 0.20 gives 18 (below half). */
  static const struct { double load; bool latches; const char *what; } cases[] = {
    { 0.61, false, "d_drag_1 drag (~436 ticks, not the cap)" },
    { 0.325, false, "a drag that saturates at ~30 units (above half)" },
    { 0.20, true, "a drag that saturates at ~18 units (below half)" },
  };
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
    {
      Chain chain;
      chain_init(&chain, 10000u);
      chain_start_v(&chain, sign * 50, 2000u);
      chain.load = cases[i].load;
      Run run = chain_run(&chain, 20000u, sign, 1500u);
      if (sign > 0)
      {
        printf("  v 50, %s: max duty %d, %s %u ms\n", cases[i].what, (int)run.max_duty,
               run.latched_after ? "latched after" : "no latch; at the cap", run.latched_after ? (unsigned)run.latched_after : (unsigned)run.at_cap);
      }
      CHECK((run.latched_after != 0u) == cases[i].latches, "v %+d, %s: %s", sign * 50, cases[i].what,
            cases[i].latches ? "latches" : "never latches");
      if (i == 0)
      {
        CHECK(run.max_duty < MOTOR_DUTY_CAP_TICKS && run.max_duty > 400, "d_drag_1 case: duty %d, below the cap",
              (int)run.max_duty);
      }
      if (cases[i].latches)
      {
        CHECK(run.latched_after - run.first_cap >= STALL_SAT_HOLD_MS && run.latched_after - run.first_cap <= STALL_SAT_HOLD_MS + 2u,
              "below half: latch 1 s after the cap (%u)", (unsigned)(run.latched_after - run.first_cap));
      }
    }
  }
}

/* ---- (d) a transient cap during a normal step --------------------------------------------- */

static void test_transient_cap_during_a_step_does_not_latch(void)
{
  /* A v 80 step, from rest and from v 20, where the output reaches the cap
   * while the speed comes up: on the lift at the edge of what the cap can do
   * (load 0.86: the steady duty is the cap, touched again and again), and
   * with a vehicle's inertia (tau 200 and 400 ms, load 0.95: the output
   * winds up to the cap until the speed catches up, then comes back down). */
  static const struct { double tau; double load; } plants[] = { { PLANT_TAU_MS, 0.86 }, { 200.0, 0.95 }, { 400.0, 0.95 } };
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    for (size_t p = 0; p < sizeof plants / sizeof plants[0]; p++)
    {
      for (int from = 0; from <= 20; from += 20)
      {
        Chain chain;
        chain_init(&chain, 10000u);
        chain.tau_ms = plants[p].tau;
        chain.load = plants[p].load;
        chain_start_v(&chain, sign * from, from ? 3000u : 0u);
        send_v(&chain, sign * 80);
        Run run = chain_run(&chain, 10000u, sign, 0u);
        if (sign > 0)
        {
          printf("  v %2d -> v 80 (tau %3.0f ms, load %.2f): at the cap from %4u ms, longest stretch %3u ms (counted by the rule %3u ms), final %.1f\n",
                 from, plants[p].tau, plants[p].load, (unsigned)run.first_cap, (unsigned)run.longest_cap_run,
                 (unsigned)run.longest_sat_hold, run.mean_speed);
        }
        CHECK(run.first_cap > 0u && run.longest_cap_run < STALL_SAT_HOLD_MS,
              "tau %.0f, v %+d -> v %+d: the cap is reached (at %u ms), never for 1 s (%u ms)", plants[p].tau,
              sign * from, sign * 80, (unsigned)run.first_cap, (unsigned)run.longest_cap_run);
        CHECK(run.longest_sat_hold < STALL_SAT_HOLD_MS / 2u,
              "tau %.0f, v %+d -> v %+d: the rule counts at most %u ms of it (margin over half the hold)",
              plants[p].tau, sign * from, sign * 80, (unsigned)run.longest_sat_hold);
        CHECK(run.latched_after == 0u && chain.stall.stall_count == 0u && fabs(run.mean_speed - sign * 80) < 3.0,
              "tau %.0f, v %+d -> v %+d: no latch, settles at 80 (%.1f)", plants[p].tau, sign * from, sign * 80,
              run.mean_speed);
      }
    }
  }
}

/* ---- m unchanged ---------------------------------------------------------------------- */

static void test_m_at_the_cap_with_a_creep_never_latches(void)
{
  Chain chain;
  chain_init(&chain, 10000u);
  send_m(&chain, 0);
  for (int i = 0; i < 200; i++) { chain_step(&chain); }
  send_m(&chain, 30);
  for (int i = 0; i < 2000; i++) { chain_step(&chain); }
  chain.hold_units = 6.0;
  Run run = chain_run(&chain, 5000u, 1, 100u);
  CHECK(run.latched_after == 0u && chain.motor.applied_ticks == MOTOR_DUTY_CAP_TICKS && chain.stall.stall_count == 0u,
        "m 30 (the cap), creeping at 6 units for 5 s: no latch (m is bench/diagnostic, unchanged)");
}

/* ---- (g) the 3-in-60 s escalation with both kinds ------------------------------------------ */

/* One stall of either kind, starting from "v 0"; returns when latched. */
static void stall_once(Chain *chain, bool saturation, int32_t sign)
{
  chain->hold_units = -1.0;
  chain_start_v(chain, sign * (saturation ? 20 : 50), 1500u);
  chain->hold_units = saturation ? 6.0 : 0.0;          /* slipping, or locked */
  for (int i = 0; i < 5000 && chain->motor.state == MOTOR_STATE_RUNNING; i++) { chain_step(chain); }
  chain->hold_units = -1.0;
}

static void test_both_kinds_count_towards_the_repeat_fault(void)
{
  static const struct { bool kinds[3]; const char *what; } mixes[] = {
    { { false, true, true }, "normal, saturation, saturation" },
    { { true, false, true }, "saturation, normal, saturation" },
    { { true, true, true }, "three saturation stalls" },
  };
  for (size_t m = 0; m < sizeof mixes / sizeof mixes[0]; m++)
  {
    Chain chain;
    chain_init(&chain, 10000u);
    MotorState states[3];
    uint32_t first = 0u;
    for (int n = 0; n < 3; n++)
    {
      stall_once(&chain, mixes[m].kinds[n], (n == 1) ? -1 : 1);
      if (n == 0) { first = chain.now_ms; }
      states[n] = chain.motor.state;
    }
    CHECK(states[0] == MOTOR_STATE_LATCHED && states[1] == MOTOR_STATE_LATCHED && chain.now_ms - first < 60000u,
          "%s: the first two latch", mixes[m].what);
    CHECK(chain.motor.state == MOTOR_STATE_FAULT && (int)chain.motor.fault_reason == 7 && chain.stall.stall_count == 3u,
          "%s, within 60 s: FAULT, mfault 7", mixes[m].what);
  }

  /* Spread out: the third more than 60 s after the first: only latched. */
  Chain chain;
  chain_init(&chain, 10000u);
  stall_once(&chain, true, 1);
  uint32_t first = chain.now_ms;
  for (int n = 1; n < 3; n++)
  {
    send_v(&chain, 0);
    while (chain.now_ms - first < (uint32_t)n * 31000u) { chain_step(&chain); }
    stall_once(&chain, n == 1, 1);
  }
  CHECK(chain.now_ms - first > 60000u && chain.motor.state == MOTOR_STATE_LATCHED && chain.stall.stall_count == 3u,
        "three stalls, the third %u ms after the first: latched, not FAULT", (unsigned)(chain.now_ms - first));
}

int main(void)
{
  test_creeping_hold_at_v_20_latches_one_second_after_the_cap();
  test_v_100_on_the_lift_never_latches();
  test_drags();
  test_transient_cap_during_a_step_does_not_latch();
  test_m_at_the_cap_with_a_creep_never_latches();
  test_both_kinds_count_towards_the_repeat_fault();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
