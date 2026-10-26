/*
 * Host-side test for Core/Src/stall_detect.c, with the real motor_control
 * and speed_control (only its setpoint and mode are used here; the closed
 * loop with a plant is tests/test_saturation_stall.c). The speed estimate is
 * set directly (valid / speed), so each rule can be placed exactly. Not part
 * of the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_stall_detect.c Core/Src/stall_detect.c Core/Src/motor_control.c Core/Src/speed_control.c -o /tmp/test_stall_detect && /tmp/test_stall_detect
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
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

typedef struct
{
  MotorController motor;
  SpeedController speed;        /* open loop unless a test selects "v" */
  StallDetector   detector;
  SpeedEstimator  estimate;     /* only valid and speed_units are used */
  uint32_t        now_ms;
  int32_t         keepalive_percent;
  bool            keepalive;
  int32_t         closed_loop_target;   /* != 0: written every ms instead of m keepalives */
} Rig;

static void rig_init(Rig *rig, uint32_t start_ms)
{
  memset(rig, 0, sizeof(*rig));
  rig->now_ms = start_ms;
  MotorController_Init(&rig->motor, start_ms);
  SpeedControl_Init(&rig->speed);
  StallDetect_Init(&rig->detector);
  rig->estimate.valid = true;
}

static void send_m(Rig *rig, int32_t percent)
{
  MotorController_AcceptCommand(&rig->motor, percent, rig->now_ms);
  rig->keepalive_percent = percent;
  rig->keepalive = true;
}

/* One ms in main.c's order: commands, stall detection, motor. */
static void rig_step(Rig *rig)
{
  rig->now_ms++;
  if (rig->keepalive && rig->now_ms % 100u == 0u)
  {
    if (rig->closed_loop_target != 0) { MotorController_AcceptSpeedCommand(&rig->motor, 50, rig->now_ms); }
    else { MotorController_AcceptCommand(&rig->motor, rig->keepalive_percent, rig->now_ms); }
  }
  if (rig->closed_loop_target != 0) { MotorController_SetClosedLoopTarget(&rig->motor, rig->closed_loop_target); }
  StallDetect_Update(&rig->detector, &rig->motor, &rig->speed, &rig->estimate, rig->now_ms);
  (void)MotorController_Update(&rig->motor, rig->now_ms);
}

static void rig_run(Rig *rig, uint32_t ms)
{
  for (uint32_t i = 0u; i < ms; i++) { rig_step(rig); }
}

/* Runs at percent with the wheel turning (estimate valid, speed 20) until the ramp is done. */
static void rig_drive(Rig *rig, int32_t percent)
{
  send_m(rig, 0);
  rig->estimate.speed_units = (percent >= 0) ? 30 : -30;
  send_m(rig, percent);
  rig_run(rig, 1000u);
}

/* From now, the wheel stops: returns the ms (since now) at which the motor latched, or limit. */
static uint32_t stall_after(Rig *rig, int32_t speed, uint32_t limit)
{
  rig->estimate.speed_units = speed;
  for (uint32_t t = 1u; t <= limit; t++)
  {
    rig_step(rig);
    if (rig->motor.state == MOTOR_STATE_LATCHED || rig->motor.state == MOTOR_STATE_FAULT) { return t; }
  }
  return limit;
}

static void test_constants(void)
{
  CHECK(STALL_MIN_DUTY_TICKS == 128 && STALL_MAX_SPEED_UNITS == 2 && STALL_HOLD_MS == 500u, "8 %%, 2 units, 500 ms");
  CHECK(STALL_REPEAT_COUNT == 3u && STALL_REPEAT_WINDOW_MS == 60000u, "3 stalls within 60 s -> FAULT");
}

static void test_stall_latches_after_exactly_500_ms(void)
{
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    Rig rig;
    rig_init(&rig, 1000u);
    rig_drive(&rig, sign * 20);
    uint32_t latched = stall_after(&rig, 0, 2000u);
    /* The condition first holds at the step after the wheel stops (t = 1);
     * 500 ms later is t = 501. */
    CHECK(latched == 1u + STALL_HOLD_MS, "m %+d, wheel stopped: latched after %u ms (expected %u)", sign * 20,
          (unsigned)latched, 1u + STALL_HOLD_MS);
    CHECK(rig.motor.state == MOTOR_STATE_LATCHED && rig.detector.stall_count == 1u && rig.motor.latch_count == 1u &&
          rig.motor.watchdog_trip_count == 0u, "a stall: LATCHED, stall_count 1, not a watchdog trip");
    MotorOutput output = MotorController_Update(&rig.motor, rig.now_ms + 1u);
    CHECK(!output.driver_enabled && output.duty_ticks == 0u, "driver off at once");

    /* Latched until m 0: keepalives don't restart it. */
    rig_run(&rig, 1000u);
    CHECK(rig.motor.state == MOTOR_STATE_LATCHED, "m %+d keepalives: still latched", sign * 20);
    rig_drive(&rig, sign * 20);
    CHECK(rig.motor.state == MOTOR_STATE_RUNNING, "m 0, m %+d: running again", sign * 20);
  }
}

static void test_thresholds(void)
{
  /* Duty exactly 128 counts, 127 doesn't (closed-loop target for odd ticks). */
  static const struct { int32_t target; int32_t speed; bool stall; const char *what; } cases[] = {
    { 128, 0, true, "128 ticks, speed 0" },    { 127, 0, false, "127 ticks, speed 0" },
    { -128, 0, true, "-128 ticks, speed 0" },  { -127, 0, false, "-127 ticks, speed 0" },
    { 300, 1, true, "speed 1" },               { 300, -1, true, "speed -1" },
    { 300, 2, false, "speed 2" },              { -300, -2, false, "speed -2" },
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
  {
    Rig rig;
    rig_init(&rig, 0u);
    MotorController_AcceptSpeedCommand(&rig.motor, 0, rig.now_ms);
    rig.keepalive = true;
    rig.closed_loop_target = cases[i].target;
    rig.estimate.speed_units = 30;
    rig_run(&rig, 1000u);
    uint32_t latched = stall_after(&rig, cases[i].speed, 3000u);
    bool stalled = (latched < 3000u);
    CHECK(stalled == cases[i].stall, "%s: stall %d, expected %d", cases[i].what, (int)stalled, (int)cases[i].stall);
  }
}

static void test_breaks_and_invalid_estimates_restart_the_500_ms(void)
{
  Rig rig;
  rig_init(&rig, 0u);
  rig_drive(&rig, 20);
  rig.estimate.speed_units = 0;
  rig_run(&rig, 450u);
  rig.estimate.speed_units = 3;     /* one ms of turning */
  rig_step(&rig);
  CHECK(stall_after(&rig, 0, 2000u) == 1u + STALL_HOLD_MS, "a 1 ms break: the full 500 ms again");

  rig_drive(&rig, 20);
  rig.estimate.speed_units = 0;
  rig_run(&rig, 300u);
  rig.estimate.valid = false;       /* can't tell: suspended, not "stopped" */
  rig_run(&rig, 1000u);
  CHECK(rig.motor.state == MOTOR_STATE_RUNNING, "invalid estimate for 1 s with the wheel 'stopped': no stall");
  rig.estimate.valid = true;
  CHECK(stall_after(&rig, 0, 2000u) == 1u + STALL_HOLD_MS, "valid again: the full 500 ms again");
}

static void test_no_stall_when_not_running_or_at_a_normal_start(void)
{
  Rig rig;
  rig_init(&rig, 0u);
  rig.estimate.speed_units = 0;
  send_m(&rig, 20);                 /* latched at boot: never runs */
  rig_run(&rig, 3000u);
  CHECK(rig.detector.stall_count == 0u, "latched: no stall");
  send_m(&rig, 0);                  /* OFF, stopped */
  rig_run(&rig, 3000u);
  CHECK(rig.detector.stall_count == 0u, "OFF: no stall");

  /* A normal start: the duty passes 128 ticks at 64 ms of ramp, the wheel
   * breaks away at ~172 ticks (86 ms) and the estimate follows ~20 ms later. */
  send_m(&rig, 30);
  for (uint32_t t = 0u; t < 1000u; t++)
  {
    if (rig.motor.applied_ticks >= 172) { rig.estimate.speed_units = 25; }
    rig_step(&rig);
  }
  CHECK(rig.detector.stall_count == 0u && rig.motor.state == MOTOR_STATE_RUNNING, "a normal start: no stall");
}

static void test_three_stalls_within_60_s_fault(void)
{
  /* Stalls at t, t + 30 s, t + 60 s (third exactly 60 s after the first):
   * FAULT. From t = 0 and across the tick wrap. */
  static const uint32_t starts[] = { 0u, UINT32_MAX - 20000u };
  Rig rig;
  for (size_t s = 0; s < sizeof starts / sizeof starts[0]; s++)
  {
    rig_init(&rig, starts[s]);
    uint32_t first_stall = 0u;
    uint32_t states_after[3];
    for (int n = 0; n < 3; n++)
    {
      rig_drive(&rig, 20);
      if (n > 0)
      {
        uint32_t target = first_stall + (uint32_t)n * 30000u - 501u;
        while (rig.now_ms != target) { rig_step(&rig); }
      }
      stall_after(&rig, 0, 2000u);
      if (n == 0) { first_stall = rig.now_ms; }
      states_after[n] = (uint32_t)rig.motor.state;
    }
    CHECK(rig.now_ms - first_stall == 60000u, "start %u: the third stall 60 s after the first (%u)", (unsigned)starts[s],
          (unsigned)(rig.now_ms - first_stall));
    CHECK(states_after[0] == MOTOR_STATE_LATCHED && states_after[1] == MOTOR_STATE_LATCHED,
          "start %u: the first two stalls only latch", (unsigned)starts[s]);
    CHECK(rig.motor.state == MOTOR_STATE_FAULT && rig.motor.fault_reason == MOTOR_FAULT_REPEATED_STALL &&
          rig.detector.stall_count == 3u, "start %u: 3 stalls within 60 s: FAULT (REPEATED_STALL)", (unsigned)starts[s]);
    rig_drive(&rig, 20);
    CHECK(rig.motor.state == MOTOR_STATE_FAULT, "m 0 doesn't clear it");
  }
  uint32_t first = 0u;

  /* The same, but the third 60.001 s after the first: only latched. */
  rig_init(&rig, 0u);
  first = 0u;
  for (int n = 0; n < 3; n++)
  {
    rig_drive(&rig, 20);
    if (n > 0)
    {
      uint32_t target = first + (uint32_t)n * 30000u + (n == 2 ? 1u : 0u) - 501u;
      while (rig.now_ms != target) { rig_step(&rig); }
    }
    stall_after(&rig, 0, 2000u);
    if (n == 0) { first = rig.now_ms; }
  }
  CHECK(rig.now_ms - first == 60001u && rig.motor.state == MOTOR_STATE_LATCHED && rig.detector.stall_count == 3u,
        "third stall 60.001 s after the first: latched, not FAULT (state %d)", (int)rig.motor.state);

  /* A 4th within 60 s of the 2nd: FAULT (the window slides). */
  rig_drive(&rig, 20);
  stall_after(&rig, 0, 2000u);
  CHECK(rig.motor.state == MOTOR_STATE_FAULT && rig.detector.stall_count == 4u, "4th stall soon after: FAULT");
}

static void test_fault_already(void)
{
  Rig rig;
  rig_init(&rig, 0u);
  rig_drive(&rig, 20);
  MotorController_LockOut(&rig.motor, MOTOR_FAULT_LOW_BATTERY);
  rig.estimate.speed_units = 0;
  rig_run(&rig, 2000u);
  CHECK(rig.detector.stall_count == 0u && rig.motor.fault_reason == MOTOR_FAULT_LOW_BATTERY, "FAULT: nothing more, first reason kept");
}

/* ---- Saturation stall ("v" only) ------------------------------------------------------ */

/* "v <setpoint>" with the controller's output pinned at the cap of the
 * setpoint's sign (the rig writes it every ms), the wheel at speed: runs
 * until the applied duty is at the cap. The condition holds from the next ms. */
static void rig_saturate(Rig *rig, int32_t setpoint, int32_t speed)
{
  MotorController_AcceptSpeedCommand(&rig->motor, 0, rig->now_ms);   /* "v 0" re-arms */
  SpeedControl_SetSetpoint(&rig->speed, setpoint, rig->motor.applied_ticks);
  MotorController_AcceptSpeedCommand(&rig->motor, setpoint, rig->now_ms);
  rig->keepalive = true;
  rig->closed_loop_target = (setpoint >= 0) ? MOTOR_DUTY_CAP_TICKS : -MOTOR_DUTY_CAP_TICKS;
  rig->estimate.valid = true;
  rig->estimate.speed_units = speed;
  for (int t = 0; t < 2000 && rig->motor.applied_ticks != rig->closed_loop_target; t++) { rig_step(rig); }
}

/* Steps until the motor leaves RUNNING: the step (1-based) it happened in, or 0 within limit. */
static uint32_t latch_within(Rig *rig, uint32_t limit)
{
  for (uint32_t t = 1u; t <= limit; t++)
  {
    rig_step(rig);
    if (rig->motor.state != MOTOR_STATE_RUNNING) { return t; }
  }
  return 0u;
}

static void test_saturation_constants(void)
{
  CHECK(STALL_SAT_HOLD_MS == 1000u && STALL_SAT_SPEED_NUMERATOR == 1 && STALL_SAT_SPEED_DENOMINATOR == 2,
        "saturation stall: at the cap, below 1/2 of the setpoint, 1000 ms");
  CHECK(STALL_SAT_HOLD_MS == 2u * STALL_HOLD_MS, "twice the first rule's hold");
}

static void test_saturation_latches_after_exactly_1000_ms(void)
{
  /* (setpoint, speed): forward, reverse, and driven the wrong way while at the cap. */
  static const struct { int32_t setpoint; int32_t speed; } cases[] = {
    { 20, 6 }, { -20, -6 }, { 20, -6 }, { -20, 6 }, { 20, -15 }, { -20, 15 }, { 100, 49 }, { -100, -49 }, { 20, 2 },
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
  {
    Rig rig;
    rig_init(&rig, 1000u);
    for (int n = 1; n <= 2; n++)   /* twice: the timer starts afresh after a latch */
    {
      rig_saturate(&rig, cases[i].setpoint, cases[i].speed);
      uint32_t t = latch_within(&rig, 3000u);
      CHECK(t == STALL_SAT_HOLD_MS + 1u && rig.motor.state == MOTOR_STATE_LATCHED &&
            rig.detector.stall_count == (uint32_t)n && rig.motor.latch_count == (uint32_t)n,
            "v %d at the cap, speed %d: latch #%d on the 1001st ms at the cap (%u), stall_n %u", (int)cases[i].setpoint,
            (int)cases[i].speed, n, (unsigned)t, (unsigned)rig.detector.stall_count);
      CHECK(rig.motor.applied_ticks == 0 && rig.motor.target_ticks == 0, "stopped at once");
    }
  }
}

static void test_saturation_thresholds(void)
{
  /* Half the setpoint: below latches, at or above never (3 s). Duty one step below the cap never. */
  static const struct { int32_t setpoint; int32_t speed; int32_t target; bool latches; } cases[] = {
    { 20, 9, 480, true },    { 20, 10, 480, false },   { 21, 10, 480, true },   { 21, 11, 480, false },
    { -20, -9, -480, true }, { -20, -10, -480, false }, { -21, -10, -480, true }, { -21, -11, -480, false },
    { 20, 6, 478, false },   { -20, -6, -478, false },  { 100, 50, 480, false },  { 100, 93, 480, false },
  };
  for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
  {
    Rig rig;
    rig_init(&rig, 1000u);
    rig_saturate(&rig, cases[i].setpoint, cases[i].speed);
    rig.closed_loop_target = cases[i].target;
    uint32_t t = latch_within(&rig, 3000u);
    CHECK((t != 0u) == cases[i].latches, "v %d, speed %d, target %d: %s (%u)", (int)cases[i].setpoint,
          (int)cases[i].speed, (int)cases[i].target, cases[i].latches ? "latches" : "never latches", (unsigned)t);
  }
}

typedef enum
{
  BREAK_DUTY_OFF_CAP, BREAK_SPEED_AT_HALF, BREAK_INVALID_ESTIMATE, BREAK_V_0, BREAK_BELOW_MINIMUM, BREAK_M,
  BREAK_COUNT
} SaturationBreak;

static void test_saturation_timer_resets(void)
{
  static const char *names[BREAK_COUNT] = { "duty off the cap", "speed at half the setpoint", "invalid estimate",
                                            "v 0", "a setpoint below the minimum", "an m command" };
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    for (int kind = 0; kind < BREAK_COUNT; kind++)
    {
      Rig rig;
      rig_init(&rig, 1000u);
      rig_saturate(&rig, sign * 20, sign * 6);
      CHECK(latch_within(&rig, 600u) == 0u, "no latch in the first 600 ms");
      /* One ms of the break, then the condition again. */
      switch ((SaturationBreak)kind)
      {
        case BREAK_DUTY_OFF_CAP:     rig.closed_loop_target = sign * (MOTOR_DUTY_CAP_TICKS - 2); break;
        case BREAK_SPEED_AT_HALF:    rig.estimate.speed_units = sign * 10; break;
        case BREAK_INVALID_ESTIMATE: rig.estimate.valid = false; break;
        case BREAK_V_0:              SpeedControl_SetSetpoint(&rig.speed, 0, rig.motor.applied_ticks); break;
        case BREAK_BELOW_MINIMUM:    SpeedControl_SetSetpoint(&rig.speed, sign * (SPEED_MIN_SETPOINT_UNITS - 1),
                                                              rig.motor.applied_ticks); break;
        case BREAK_M:                SpeedControl_SelectOpenLoop(&rig.speed); break;
        default: break;
      }
      rig_step(&rig);
      rig.closed_loop_target = sign * MOTOR_DUTY_CAP_TICKS;
      rig.estimate.speed_units = sign * 6;
      rig.estimate.valid = true;
      if (rig.speed.effective_units != sign * 20) { SpeedControl_SetSetpoint(&rig.speed, sign * 20, rig.motor.applied_ticks); }
      /* The 1000 ms start again (one more ms if the duty had to ramp back to the cap). */
      uint32_t t = latch_within(&rig, 3000u);
      CHECK(t >= STALL_SAT_HOLD_MS && t <= STALL_SAT_HOLD_MS + 2u && rig.detector.stall_count == 1u,
            "sign %+d, %s: the 1000 ms restart (latched %u ms after the break)", sign, names[kind], (unsigned)t);
    }
  }
}

static void test_saturation_sign_change_and_new_magnitude(void)
{
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    /* Sign change: the duty ramps through 0 (and the dwell) to the other
     * cap; the 1000 ms count from there, not from the first cap. */
    Rig rig;
    rig_init(&rig, 1000u);
    rig_saturate(&rig, sign * 20, sign * 6);
    CHECK(latch_within(&rig, 600u) == 0u, "no latch in the first 600 ms");
    SpeedControl_SetSetpoint(&rig.speed, -sign * 20, rig.motor.applied_ticks);
    rig.closed_loop_target = -sign * MOTOR_DUTY_CAP_TICKS;
    rig.estimate.speed_units = -sign * 6;
    uint32_t to_cap = 0u;
    while (rig.motor.applied_ticks != rig.closed_loop_target && to_cap < 2000u) { rig_step(&rig); to_cap++; }
    CHECK(rig.motor.state == MOTOR_STATE_RUNNING && to_cap > 400u, "through 0 to the other cap in %u ms, no latch",
          (unsigned)to_cap);
    uint32_t t = latch_within(&rig, 3000u);
    CHECK(t == STALL_SAT_HOLD_MS + 1u, "sign %+d -> %+d: 1000 ms from the new cap (%u)", sign, -sign, (unsigned)t);

    /* Same sign, new magnitude: not a reset (latch 1001 ms after the first cap). */
    rig_init(&rig, 1000u);
    rig_saturate(&rig, sign * 20, sign * 6);
    CHECK(latch_within(&rig, 600u) == 0u, "no latch in the first 600 ms");
    SpeedControl_SetSetpoint(&rig.speed, sign * 30, rig.motor.applied_ticks);
    t = latch_within(&rig, 3000u);
    CHECK(t == STALL_SAT_HOLD_MS + 1u - 600u, "sign %+d, v 20 -> v 30 at the cap: no restart (%u)", sign, (unsigned)t);
  }
}

static void test_saturation_keeps_counting_when_the_wheel_then_locks(void)
{
  /* Slipping at 6 units for 800 ms at the cap, then held still: both rules
   * hold now, and the earlier one (saturation, 1000 ms from the first cap)
   * fires, not the first rule 500 ms after the lock. */
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    Rig rig;
    rig_init(&rig, 1000u);
    rig_saturate(&rig, sign * 20, sign * 6);
    CHECK(latch_within(&rig, 800u) == 0u, "slipping: no latch in 800 ms");
    rig.estimate.speed_units = 0;
    uint32_t t = latch_within(&rig, 2000u);
    CHECK(t == STALL_SAT_HOLD_MS + 1u - 800u && rig.detector.stall_count == 1u,
          "sign %+d: then locked: the saturation rule's 1000 ms still end first (%u ms after the lock)", sign, (unsigned)t);
  }
}

static void test_saturation_rule_is_not_in_m(void)
{
  /* "m 30" is the cap too, and a slipping wheel at 6 units: in open loop only the first rule applies. */
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    Rig rig;
    rig_init(&rig, 1000u);
    rig_drive(&rig, sign * 30);
    rig.estimate.speed_units = sign * 6;
    CHECK(latch_within(&rig, 5000u) == 0u && rig.motor.applied_ticks == sign * MOTOR_DUTY_CAP_TICKS &&
          rig.detector.stall_count == 0u, "m %+d at the cap, 6 units for 5 s: no latch (m unchanged)", sign * 30);
  }
}

static void test_saturation_counts_towards_the_repeat_fault(void)
{
  /* A first-rule stall, then two saturation stalls, all within 60 s: FAULT 7. */
  Rig rig;
  rig_init(&rig, 1000u);
  rig_drive(&rig, 20);
  CHECK(stall_after(&rig, 0, 2000u) == STALL_HOLD_MS + 1u, "first-rule stall");
  rig.keepalive = false;
  rig_saturate(&rig, 20, 6);
  CHECK(latch_within(&rig, 3000u) == STALL_SAT_HOLD_MS + 1u && rig.motor.state == MOTOR_STATE_LATCHED,
        "saturation stall: latched");
  rig_saturate(&rig, -20, -6);
  CHECK(latch_within(&rig, 3000u) == STALL_SAT_HOLD_MS + 1u, "third stall");
  CHECK(rig.motor.state == MOTOR_STATE_FAULT && rig.motor.fault_reason == MOTOR_FAULT_REPEATED_STALL &&
        (int)rig.motor.fault_reason == 7 && rig.detector.stall_count == 3u, "1 + 2 within 60 s: FAULT, mfault 7");
  CHECK(rig.detector.locked_stall_count == 1u && rig.detector.saturation_stall_count == 2u &&
        rig.motor.latch_reason == MOTOR_LATCH_NONE, "per rule: 1 locked + 2 saturation; FAULT clears the latch reason");
}

/* The cause of each stall, for the Pi link: which rule latched, per-rule
 * counts that always add up to stall_count, and a latch reason that lasts
 * exactly as long as the latch. */
static void test_stall_cause(void)
{
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    Rig rig;
    rig_init(&rig, 1000u);
    CHECK(rig.motor.latch_reason == MOTOR_LATCH_BOOT, "boot: latch reason BOOT");
    rig_drive(&rig, sign * 20);
    CHECK(rig.motor.latch_reason == MOTOR_LATCH_NONE, "m 0 re-armed: no latch reason");
    CHECK(stall_after(&rig, 0, 2000u) == STALL_HOLD_MS + 1u, "sign %+d: locked stall", sign);
    CHECK(rig.motor.latch_reason == MOTOR_LATCH_STALL_LOCKED && rig.detector.locked_stall_count == 1u &&
          rig.detector.saturation_stall_count == 0u && rig.detector.stall_count == 1u,
          "sign %+d: rule 1 -> STALL_LOCKED, locked 1, saturation 0", sign);

    rig.keepalive = false;
    rig_saturate(&rig, sign * 20, sign * 6);
    CHECK(rig.motor.latch_reason == MOTOR_LATCH_NONE, "v 0 re-armed: no latch reason");
    CHECK(latch_within(&rig, 3000u) == STALL_SAT_HOLD_MS + 1u, "sign %+d: saturation stall", sign);
    CHECK(rig.motor.latch_reason == MOTOR_LATCH_STALL_SATURATION && rig.detector.locked_stall_count == 1u &&
          rig.detector.saturation_stall_count == 1u && rig.detector.stall_count == 2u,
          "sign %+d: rule 2 -> STALL_SATURATION, locked 1, saturation 1", sign);
    CHECK(rig.detector.locked_stall_count + rig.detector.saturation_stall_count == rig.detector.stall_count,
          "per-rule counts add up to stall_n");
  }
}

int main(void)
{
  test_constants();
  test_stall_latches_after_exactly_500_ms();
  test_thresholds();
  test_breaks_and_invalid_estimates_restart_the_500_ms();
  test_no_stall_when_not_running_or_at_a_normal_start();
  test_three_stalls_within_60_s_fault();
  test_fault_already();
  test_saturation_constants();
  test_saturation_latches_after_exactly_1000_ms();
  test_saturation_thresholds();
  test_saturation_timer_resets();
  test_saturation_sign_change_and_new_magnitude();
  test_saturation_keeps_counting_when_the_wheel_then_locks();
  test_saturation_rule_is_not_in_m();
  test_saturation_counts_towards_the_repeat_fault();
  test_stall_cause();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
