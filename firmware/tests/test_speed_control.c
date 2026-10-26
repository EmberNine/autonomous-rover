/*
 * Host-side test for Core/Src/speed_control.c, in closed loop with the REAL
 * motor_control and speed_estimator and a plant model of the lift (gain,
 * dead zone, breakaway, coast, adjustable tau and dead time). Not part of
 * the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_speed_control.c Core/Src/speed_control.c Core/Src/motor_control.c Core/Src/speed_estimator.c -lm -o /tmp/test_speed_control && /tmp/test_speed_control
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "speed_control.h"

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

/* ---- Plant: the wheel on the lift ------------------------------------------------ */
/* Static map 11.7 counts/s per tick above 66 ticks (2026-10-15 captures),
 * breakaway from rest at ~172 ticks (156-188 measured), coast tau 400 ms. */
#define PLANT_GAIN        0.0117   /* counts/ms per tick */
#define PLANT_DEAD_ZONE   66.0
#define PLANT_BREAKAWAY   172.0
#define PLANT_COAST_TAU   400.0
#define MAX_DELAY         64

typedef struct
{
  double tau_ms;           /* 0 = no lag */
  int    delay_ms;         /* dead time on the duty */
  double load;             /* 1 = free; < 1 = a load (gloved hand) */
  double forced_speed;     /* != 0: the wheel is driven at this speed from outside (a push, a slope) */
  double speed;            /* counts/ms */
  double position;
  double duty_history[MAX_DELAY];
  bool   enabled_history[MAX_DELAY];
  int    head;
} Plant;

typedef struct
{
  MotorController motor;
  SpeedController speed;
  SpeedEstimator  estimator;
  Plant           plant;
  uint32_t        now_ms;
  MotorOutput     output;
  bool            read_fails;      /* inject failed angle reads */
  bool            send_keepalive;
  int32_t         keepalive_v;
  bool            keepalive_is_v;
  int32_t         keepalive_m;
  /* Invariants, every ms */
  int             sign_violations;   /* target or integrator against the direction */
  int             integrator_out_of_range;
} Rig;

static void plant_step(Plant *plant, const MotorOutput *output)
{
  double duty = output->driver_enabled
                    ? ((output->direction == MOTOR_DIRECTION_REVERSE) ? -(double)output->duty_ticks : (double)output->duty_ticks)
                    : 0.0;
  plant->duty_history[plant->head] = duty;
  plant->enabled_history[plant->head] = output->driver_enabled;
  int index = (plant->head - plant->delay_ms + MAX_DELAY) % MAX_DELAY;
  plant->head = (plant->head + 1) % MAX_DELAY;
  double u = plant->duty_history[index];
  bool enabled = plant->enabled_history[index];
  double magnitude = fabs(u);
  if (!enabled)
  {
    plant->speed *= exp(-1.0 / PLANT_COAST_TAU);                       /* coast */
  }
  else if (fabs(plant->speed) < 0.02 && magnitude < PLANT_BREAKAWAY)
  {
    plant->speed = 0.0;                                                 /* stiction */
  }
  else
  {
    double target = copysign(fmax(magnitude - PLANT_DEAD_ZONE, 0.0), u) * PLANT_GAIN * plant->load;
    if (plant->tau_ms <= 0.0) { plant->speed = target; }
    else { plant->speed += (target - plant->speed) * (1.0 - exp(-1.0 / plant->tau_ms)); }
  }
  if (plant->forced_speed != 0.0) { plant->speed = plant->forced_speed; }
  plant->position += plant->speed;
}

static void rig_init(Rig *rig, double tau_ms, int delay_ms)
{
  memset(rig, 0, sizeof(*rig));
  rig->now_ms = 1000u;
  MotorController_Init(&rig->motor, rig->now_ms);
  SpeedControl_Init(&rig->speed);
  SpeedEstimator_Init(&rig->estimator);
  rig->plant.tau_ms = tau_ms;
  rig->plant.delay_ms = delay_ms;
  rig->plant.load = 1.0;
  rig->plant.position = 1234.6;
}

/* As command_dispatch.c will do for "v" and "m". */
static void send_v(Rig *rig, int32_t units)
{
  MotorController_AcceptSpeedCommand(&rig->motor, units, rig->now_ms);
  SpeedControl_SetSetpoint(&rig->speed, units, rig->motor.applied_ticks);
  rig->keepalive_is_v = true;
  rig->keepalive_v = units;
  rig->send_keepalive = true;
}

static void send_m(Rig *rig, int32_t percent)
{
  MotorController_AcceptCommand(&rig->motor, percent, rig->now_ms);
  SpeedControl_SelectOpenLoop(&rig->speed);
  rig->keepalive_is_v = false;
  rig->keepalive_m = percent;
  rig->send_keepalive = true;
}

/* One ms in main.c's order: commands, speed control, motor, plant, encoder, estimator. */
static void rig_step(Rig *rig)
{
  rig->now_ms++;
  if (rig->send_keepalive && rig->now_ms % 100u == 0u)
  {
    if (rig->keepalive_is_v) { send_v(rig, rig->keepalive_v); } else { send_m(rig, rig->keepalive_m); }
  }
  SpeedControl_Update(&rig->speed, &rig->motor, &rig->estimator, rig->now_ms);
  rig->output = MotorController_Update(&rig->motor, rig->now_ms);
  plant_step(&rig->plant, &rig->output);
  SpeedEstimatorInput input = { .now_ms = rig->now_ms, .count = (int32_t)floor(rig->plant.position),
                                .read_ok = !rig->read_fails, .magnet_detected = true };
  if (rig->read_fails) { input.count = rig->estimator.previous_count; }
  SpeedEstimator_Update(&rig->estimator, &input);

  int32_t direction = rig->speed.direction;
  if ((direction > 0 && (rig->motor.target_ticks < 0 || rig->speed.integrator_q8 < 0)) ||
      (direction < 0 && (rig->motor.target_ticks > 0 || rig->speed.integrator_q8 > 0)))
  {
    if (rig->speed.mode == SPEED_MODE_CLOSED_LOOP) { rig->sign_violations++; }
  }
  if (abs(rig->speed.integrator_q8) > MOTOR_DUTY_CAP_TICKS * SPEED_Q8_ONE) { rig->integrator_out_of_range++; }
}

static void rig_run(Rig *rig, uint32_t ms)
{
  for (uint32_t i = 0u; i < ms; i++) { rig_step(rig); }
}

/* Arms the motor ("v 0") and lets the estimator fill. */
static void rig_arm(Rig *rig)
{
  send_v(rig, 0);
  rig_run(rig, 100u);
}

typedef struct
{
  uint32_t settle_ms;      /* from the command to the last time the estimate was outside +-tolerance */
  int32_t  peak_units;     /* largest |estimate| in the setpoint's direction */
  double   final_mean;     /* mean estimate over the last 500 ms */
} Response;

static Response run_response(Rig *rig, int32_t setpoint, uint32_t duration_ms, int32_t tolerance)
{
  Response response = { 0u, 0, 0.0 };
  uint32_t start = rig->now_ms;
  double sum = 0.0;
  int n = 0;
  for (uint32_t t = 0u; t < duration_ms; t++)
  {
    rig_step(rig);
    int32_t speed = rig->estimator.speed_units;
    int32_t signed_speed = (setpoint >= 0) ? speed : -speed;
    if (signed_speed > response.peak_units) { response.peak_units = signed_speed; }
    if (!rig->estimator.valid || abs(speed - setpoint) > tolerance) { response.settle_ms = rig->now_ms - start; }
    if (t >= duration_ms - 500u) { sum += speed; n++; }
  }
  response.final_mean = sum / n;
  return response;
}

/* ---- Constants --------------------------------------------------------------------- */

static void test_pinned_constants(void)
{
  /* Pinned on purpose (gains from lambda tuning, see speed_control.h): change
   * only with a new derivation or a new identification. */
  CHECK(SPEED_KP_Q8 == 164 && SPEED_KI_Q8 == 55 && SPEED_CONTROL_PERIOD_MS == 10u, "Kp 164/256, Ki 55/256 per 10 ms");
  CHECK(SPEED_MIN_SETPOINT_UNITS == 20 && SPEED_GATE_UNITS == 2 && SPEED_INVALID_HOLD_MS == 50u &&
        SPEED_PRELOAD_TICKS == 160, "min 20 units, gate 2, hold 50 ms, preload 160 ticks");
  /* The derivation, recomputed: K = 0.234 units/tick, lambda 150, theta 50, tau 30. */
  double ki_per_ms = 1.0 / (0.234 * (150.0 + 50.0));
  CHECK(lround(ki_per_ms * 10.0 * 256.0) == SPEED_KI_Q8 && lround(ki_per_ms * 30.0 * 256.0) == SPEED_KP_Q8,
        "Q8 gains match the derivation (Ki %.4f/ms)", ki_per_ms);
}

/* ---- Step responses across the tau range --------------------------------------------- */

static void test_step_from_rest_for_tau_0_to_30_ms(void)
{
  static const struct { double tau; int delay; } plants[] = {
    { 0.0, 0 }, { 10.0, 0 }, { 20.0, 0 }, { 30.0, 0 }, { 0.0, 10 }, { 30.0, 10 },
  };
  for (size_t i = 0; i < sizeof plants / sizeof plants[0]; i++)
  {
    for (int sign = 1; sign >= -1; sign -= 2)
    {
      Rig rig;
      rig_init(&rig, plants[i].tau, plants[i].delay);
      rig_arm(&rig);
      send_v(&rig, sign * 50);
      Response r = run_response(&rig, sign * 50, 3000u, 3);
      printf("  tau %2.0f ms, delay %2d ms, v %+d: settled (+-3) in %4u ms, peak %3d, final %6.2f\n", plants[i].tau,
             plants[i].delay, sign * 50, (unsigned)r.settle_ms, (int)r.peak_units, r.final_mean);
      CHECK(r.settle_ms < 1000u, "tau %.0f delay %d v %+d: settles within 1 s (%u ms)", plants[i].tau, plants[i].delay,
            sign * 50, (unsigned)r.settle_ms);
      CHECK(r.peak_units <= 50 + 8, "tau %.0f delay %d v %+d: overshoot <= 8 units (peak %d)", plants[i].tau,
            plants[i].delay, sign * 50, (int)r.peak_units);
      CHECK(fabs(r.final_mean - sign * 50) < 1.0, "tau %.0f delay %d v %+d: no steady error (%.2f)", plants[i].tau,
            plants[i].delay, sign * 50, r.final_mean);
      CHECK(rig.sign_violations == 0 && rig.integrator_out_of_range == 0, "sign-symmetric clamps held");
    }
  }
}

/* ---- The first output: preload + Kp x error, nothing integrated -------------------------- */

static void test_first_output_after_a_start_from_rest(void)
{
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    Rig rig;
    rig_init(&rig, 20.0, 5);
    rig_arm(&rig);
    send_v(&rig, sign * 50);
    rig_step(&rig);
    /* Estimate 0, error 50: (160 x 256 + 164 x 50) / 256 = 192.03 -> 192.
     * The motor is OFF at this point, so nothing is integrated yet. */
    CHECK(rig.speed.output_ticks == sign * 192 && rig.motor.target_ticks == sign * 192 &&
          rig.speed.integrator_q8 == sign * SPEED_PRELOAD_TICKS * SPEED_Q8_ONE,
          "v %+d from rest: first target %d (expected %d), integrator %d", sign * 50, (int)rig.speed.output_ticks,
          sign * 192, (int)rig.speed.integrator_q8);
  }
}

/* ---- Wheel forced faster than the setpoint: the clamps hold ------------------------------ */

static void test_wheel_forced_faster_never_reverses(void)
{
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    Rig rig;
    rig_init(&rig, 20.0, 5);
    rig_arm(&rig);
    send_v(&rig, sign * 25);
    rig_run(&rig, 2000u);
    rig.plant.forced_speed = sign * 3.0;     /* 3 counts/ms = 60 units: pushed, or downhill */
    int wrong_way = 0;
    int32_t largest = 0;
    for (uint32_t t = 0u; t < 3000u; t++)
    {
      rig_step(&rig);
      if (sign * rig.motor.target_ticks < 0 || sign * rig.speed.integrator_q8 < 0) { wrong_way++; }
      if (t >= 1000u && abs(rig.motor.target_ticks) > largest) { largest = abs(rig.motor.target_ticks); }
    }
    /* The output falls to where P + I = 0 and dithers at 0 to a few ticks
     * (the anti-windup and the motor going OFF stop the integrator there):
     * never the other way, never a brake by reversing. */
    CHECK(wrong_way == 0 && largest <= 10 && sign * rig.speed.integrator_q8 >= 0,
          "v %+d, wheel forced to %+d units for 3 s: output 0..%d ticks, never the other way (%d wrong)",
          sign * 25, sign * 60, (int)largest, wrong_way);
    rig.plant.forced_speed = 0.0;
    Response r = run_response(&rig, sign * 25, 3000u, 3);
    CHECK(fabs(r.final_mean - sign * 25) < 1.0, "released: back to %+d (%.2f)", sign * 25, r.final_mean);
  }
}

/* ---- Saturation and windup ------------------------------------------------------------ */

static void test_saturation_does_not_wind_up(void)
{
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    Rig rig;
    rig_init(&rig, 20.0, 5);
    rig_arm(&rig);
    send_v(&rig, sign * 100);        /* above what 30 % reaches on the lift (~97 units) */
    rig_run(&rig, 3000u);
    CHECK(rig.motor.target_ticks == sign * MOTOR_DUTY_CAP_TICKS, "v %+d: output at the cap (%d)", sign * 100,
          (int)rig.motor.target_ticks);
    CHECK(abs(rig.speed.integrator_q8) <= MOTOR_DUTY_CAP_TICKS * SPEED_Q8_ONE, "integrator stays within the cap");
    send_v(&rig, sign * 50);
    Response r = run_response(&rig, sign * 50, 2000u, 3);
    CHECK(r.settle_ms < 700u, "v %+d -> %+d: back within +-3 in %u ms (no windup to unwind)", sign * 100, sign * 50,
          (unsigned)r.settle_ms);
    CHECK(rig.sign_violations == 0 && rig.integrator_out_of_range == 0, "clamps held");

    /* Far from reachable (half load: ~48 units at the cap, error ~52): the
     * output sits exactly at the cap, and the integrator stops where P + I
     * meets it; it doesn't climb on to the cap by itself (that would be
     * windup to unwind later). */
    Rig loaded;
    rig_init(&loaded, 20.0, 5);
    loaded.plant.load = 0.5;
    rig_arm(&loaded);
    send_v(&loaded, sign * 100);
    rig_run(&loaded, 2000u);
    int over = 0, against = 0;
    int32_t previous_integrator = loaded.speed.integrator_q8;
    for (uint32_t t = 0u; t < 2000u; t++)
    {
      rig_step(&loaded);
      /* Never moved against the error: with the error pushing out, I doesn't shrink. */
      if (sign * loaded.speed.error_units > 0 && sign * (loaded.speed.integrator_q8 - previous_integrator) < 0) { against++; }
      previous_integrator = loaded.speed.integrator_q8;
      int32_t error = sign * loaded.speed.error_units;
      /* I stops at cap - P as P was at its own step; the estimate's +-1 unit
       * jitter moves P by 164/256 tick per unit since then: allow 3 units.
       * Windup would put I at the cap, ~33 ticks (8,500/256) above this. */
      int32_t usable = MOTOR_DUTY_CAP_TICKS * SPEED_Q8_ONE - SPEED_KP_Q8 * error + 3 * SPEED_KP_Q8;
      if (error > 0 && sign * loaded.speed.integrator_q8 > usable) { over++; }
    }
    CHECK(over == 0 && loaded.motor.target_ticks == sign * MOTOR_DUTY_CAP_TICKS,
          "v %+d at half load: exactly at the cap (%d), integrator never beyond cap - P: %d ms over", sign * 100,
          (int)loaded.motor.target_ticks, over);
    CHECK(against == 0, "v %+d at half load: the integrator never moved against the error (%d times)", sign * 100, against);
  }
}

/* ---- Disturbance ------------------------------------------------------------------------- */

static void test_load_disturbance_is_rejected(void)
{
  Rig rig;
  rig_init(&rig, 20.0, 5);
  rig_arm(&rig);
  send_v(&rig, 40);
  rig_run(&rig, 2000u);
  int32_t duty_free = rig.motor.target_ticks;
  rig.plant.load = 0.7;                       /* a gloved hand on the tyre */
  Response r = run_response(&rig, 40, 2000u, 2);
  CHECK(r.settle_ms < 1000u && fabs(r.final_mean - 40.0) < 1.0, "under load: back to 40 (+-2) in %u ms, final %.2f",
        (unsigned)r.settle_ms, r.final_mean);
  CHECK(rig.motor.target_ticks > duty_free + 30, "duty rose under load (%d -> %d)", (int)duty_free,
        (int)rig.motor.target_ticks);
  /* Released all at once: the duty that was carrying the load briefly drives
   * the free wheel towards 40 / 0.7 = 57 units before the loop can react
   * (the plant, not windup); it must never go above that, and come back. */
  rig.plant.load = 1.0;
  r = run_response(&rig, 40, 2000u, 2);
  CHECK(r.settle_ms < 1000u && r.peak_units <= 40.0 / 0.7 + 2.0, "released: back in %u ms, peak %d (open-loop bound 57)",
        (unsigned)r.settle_ms, (int)r.peak_units);
}

/* ---- Minimum setpoint, v 0, sign change ---------------------------------------------------- */

static void test_minimum_setpoint_and_v_0(void)
{
  Rig rig;
  rig_init(&rig, 10.0, 0);
  rig_arm(&rig);
  send_v(&rig, SPEED_MIN_SETPOINT_UNITS - 1);
  rig_run(&rig, 2000u);
  CHECK(rig.speed.effective_units == 0 && rig.motor.target_ticks == 0 && !rig.output.driver_enabled &&
        rig.plant.speed == 0.0, "v 19: treated as 0, nothing moves");
  send_v(&rig, -(SPEED_MIN_SETPOINT_UNITS - 1));
  rig_run(&rig, 500u);
  CHECK(rig.motor.target_ticks == 0 && !rig.output.driver_enabled, "v -19: 0 as well");
  send_v(&rig, SPEED_MIN_SETPOINT_UNITS);
  Response r = run_response(&rig, SPEED_MIN_SETPOINT_UNITS, 3000u, 3);
  CHECK(fabs(r.final_mean - SPEED_MIN_SETPOINT_UNITS) < 1.0, "v 20 runs at 20 (%.2f)", r.final_mean);

  /* v 0 coasts: target 0, ramp down, driver off, integrator cleared. */
  send_v(&rig, 0);
  rig_run(&rig, 400u);
  CHECK(rig.motor.target_ticks == 0 && rig.motor.state == MOTOR_STATE_OFF && !rig.output.driver_enabled &&
        rig.speed.integrator_q8 == 0 && rig.speed.direction == 0, "v 0: coasting, driver off, integrator 0");
}

static void test_sign_change_waits_for_the_wheel_to_stop(void)
{
  Rig rig;
  rig_init(&rig, 20.0, 5);
  rig_arm(&rig);
  send_v(&rig, 50);
  rig_run(&rig, 2000u);
  send_v(&rig, -50);
  int reversed_while_moving = 0;
  uint32_t first_reverse_ms = 0u;
  for (uint32_t t = 0u; t < 4000u; t++)
  {
    rig_step(&rig);
    if (rig.motor.target_ticks < 0 && first_reverse_ms == 0u)
    {
      first_reverse_ms = t;
      if (rig.estimator.speed_units > SPEED_GATE_UNITS) { reversed_while_moving++; }
    }
  }
  CHECK(reversed_while_moving == 0 && first_reverse_ms > 0u, "no reverse target while the wheel still turned forward");
  CHECK(fabs((double)rig.estimator.speed_units + 50.0) <= 3.0, "reversed to -50 (%d) after %u ms", (int)rig.estimator.speed_units,
        (unsigned)first_reverse_ms);
  CHECK(rig.sign_violations == 0, "never a target or integrator against the direction (%d)", rig.sign_violations);
}

/* ---- m and v -------------------------------------------------------------------------------- */

static void test_m_switches_the_controller_off_and_back_bumplessly(void)
{
  Rig rig;
  rig_init(&rig, 20.0, 5);
  rig_arm(&rig);
  send_v(&rig, 50);
  rig_run(&rig, 2000u);
  send_m(&rig, 20);
  rig_run(&rig, 1500u);
  CHECK(rig.speed.mode == SPEED_MODE_OPEN_LOOP && rig.speed.integrator_q8 == 0 && rig.motor.target_ticks == 320 &&
        rig.output.duty_ticks == 320u, "m 20: open loop, integrator cleared, m's 320 applied and left alone");

  /* m 20 -> v at about the same speed: no bump. */
  int32_t speed_now = rig.estimator.speed_units;
  send_v(&rig, speed_now);
  int32_t worst_target_jump = 0, worst_speed_error = 0;
  for (uint32_t t = 0u; t < 1500u; t++)
  {
    rig_step(&rig);
    if (abs(rig.motor.target_ticks - 320) > worst_target_jump) { worst_target_jump = abs(rig.motor.target_ticks - 320); }
    if (abs(rig.estimator.speed_units - speed_now) > worst_speed_error)
    {
      worst_speed_error = abs(rig.estimator.speed_units - speed_now);
    }
  }
  CHECK(worst_target_jump <= 12 && worst_speed_error <= 3, "m -> v at %d units: target within +-%d ticks, speed within +-%d",
        (int)speed_now, (int)worst_target_jump, (int)worst_speed_error);
}

static void test_preload_starts_from_rest_faster(void)
{
  uint32_t settle[2];
  int32_t peak[2];
  for (int with_preload = 0; with_preload <= 1; with_preload++)
  {
    Rig rig;
    rig_init(&rig, 20.0, 5);
    rig.speed.preload_ticks = with_preload ? SPEED_PRELOAD_TICKS : 0;
    rig_arm(&rig);
    send_v(&rig, 25);
    Response r = run_response(&rig, 25, 3000u, 3);
    settle[with_preload] = r.settle_ms;
    peak[with_preload] = r.peak_units;
  }
  printf("  v 25 from rest: settle %u ms without the preload, %u ms with it (peak %d / %d)\n", (unsigned)settle[0],
         (unsigned)settle[1], (int)peak[0], (int)peak[1]);
  CHECK(settle[1] + 200u < settle[0], "the preload starts at least 200 ms sooner (%u vs %u ms)", (unsigned)settle[1],
        (unsigned)settle[0]);
  CHECK(peak[1] <= 25 + 8, "and doesn't overshoot more than 8 units (peak %d)", (int)peak[1]);

  /* While motor_control still ramps toward the target, the integrator stays put. */
  Rig rig;
  rig_init(&rig, 20.0, 5);
  rig_arm(&rig);
  send_v(&rig, 80);
  int changed_while_ramping = 0;
  for (uint32_t t = 0u; t < 300u; t++)
  {
    int32_t before = rig.speed.integrator_q8;
    bool ramping = (rig.motor.applied_ticks != rig.motor.target_ticks) || rig.motor.state != MOTOR_STATE_RUNNING;
    rig_step(&rig);
    if (ramping && rig.speed.integrator_q8 != before && t > 0u) { changed_while_ramping++; }
  }
  CHECK(changed_while_ramping == 0, "no integration while ramping or not RUNNING (%d)", changed_while_ramping);
}

/* ---- Invalid estimate, latch, FAULT, watchdog --------------------------------------------- */

static void test_invalid_estimate_holds_then_latches(void)
{
  Rig rig;
  rig_init(&rig, 20.0, 5);
  rig_arm(&rig);
  send_v(&rig, 40);
  rig_run(&rig, 2000u);
  /* 20 ms of failed reads. The controller sees each estimate one ms later
   * (main.c's order), so from the second failing ms on: target held,
   * integrator frozen, for as long as the estimate is invalid (the failures
   * plus the 21 ms the window takes to refill), and no latch. */
  rig.read_fails = true;
  rig_step(&rig);
  int32_t target = rig.motor.target_ticks;
  int32_t integrator = rig.speed.integrator_q8;
  int moved = 0;
  uint32_t invalid_ms_seen = 0u;
  for (uint32_t t = 1u; t < 200u && (t < 20u || !rig.estimator.valid); t++)
  {
    if (t == 20u) { rig.read_fails = false; }
    rig_step(&rig);
    if (!rig.estimator.valid && (rig.motor.target_ticks != target || rig.speed.integrator_q8 != integrator)) { moved++; }
    if (rig.estimator.invalid_ms > invalid_ms_seen) { invalid_ms_seen = rig.estimator.invalid_ms; }
  }
  CHECK(moved == 0 && rig.motor.state == MOTOR_STATE_RUNNING && rig.speed.enc_bad_count == 0u,
        "20 ms of failed reads (%u ms invalid): target and integrator held, no latch", (unsigned)invalid_ms_seen + 1u);
  rig_run(&rig, 1000u);
  CHECK(rig.motor.state == MOTOR_STATE_RUNNING && abs(rig.estimator.speed_units - 40) <= 3, "and it carries on");

  /* The boundary: a read outage of N ms keeps the estimate invalid for
   * N + 21 ms, so 28 ms is held and 30 ms latches. */
  for (uint32_t outage = 28u; outage <= 30u; outage += 2u)
  {
    Rig edge;
    rig_init(&edge, 20.0, 5);
    rig_arm(&edge);
    send_v(&edge, 40);
    rig_run(&edge, 2000u);
    edge.read_fails = true;
    rig_run(&edge, outage);
    edge.read_fails = false;
    rig_run(&edge, 100u);
    bool latched = (edge.motor.state == MOTOR_STATE_LATCHED);
    CHECK(latched == (outage == 30u), "a %u ms read outage: %s", (unsigned)outage, latched ? "latched" : "held");
  }

  /* Invalid for longer: latched at invalid_ms = 50, enc_bad 1, integrator cleared. */
  rig.read_fails = true;
  uint32_t latched_after = 0u;
  for (uint32_t t = 1u; t <= 200u && latched_after == 0u; t++)
  {
    rig_step(&rig);
    if (rig.motor.state == MOTOR_STATE_LATCHED) { latched_after = t; }
  }
  rig.read_fails = false;
  /* invalid_ms is 0 for the estimate of the first failing ms and 50 for the
   * 51st; the controller acts on it one ms later: the 52nd. */
  CHECK(latched_after == SPEED_INVALID_HOLD_MS + 2u && rig.speed.enc_bad_count == 1u && rig.speed.integrator_q8 == 0 &&
        rig.motor.latch_count == 1u, "latched %u ms after the reads started failing (invalid_ms 50 + 1 ms), enc_bad 1",
        (unsigned)latched_after);
  CHECK(rig.motor.latch_reason == MOTOR_LATCH_ESTIMATE_INVALID, "latch reason: ESTIMATE_INVALID");
  /* v 40 keepalives don't restart it; v 0 re-arms; v 40 runs again. */
  rig_run(&rig, 1000u);
  CHECK(rig.motor.state == MOTOR_STATE_LATCHED && !rig.output.driver_enabled, "v 40 keepalives: still latched");
  send_v(&rig, 0);
  rig_run(&rig, 200u);
  send_v(&rig, 40);
  Response r = run_response(&rig, 40, 3000u, 3);
  CHECK(fabs(r.final_mean - 40.0) < 1.0 && rig.speed.enc_bad_count == 1u, "after v 0, v 40 runs again (%.2f)", r.final_mean);
}

static void test_latch_fault_and_watchdog(void)
{
  Rig rig;
  rig_init(&rig, 20.0, 5);
  rig_arm(&rig);
  send_v(&rig, 40);
  rig_run(&rig, 2000u);

  /* The motor latched by someone else (a stall): integrator cleared, nothing written. */
  MotorController_Latch(&rig.motor, MOTOR_LATCH_STALL_LOCKED);
  rig_run(&rig, 300u);
  CHECK(rig.speed.integrator_q8 == 0 && rig.speed.direction == 0 && rig.motor.target_ticks == 0 && !rig.output.driver_enabled,
        "latched: integrator cleared, no target");

  /* FAULT: the same, for good. */
  send_v(&rig, 0);
  rig_run(&rig, 100u);
  send_v(&rig, 40);
  rig_run(&rig, 1500u);
  MotorController_LockOut(&rig.motor, MOTOR_FAULT_REPEATED_STALL);
  rig_run(&rig, 1000u);
  CHECK(rig.motor.state == MOTOR_STATE_FAULT && rig.speed.integrator_q8 == 0 && !rig.output.driver_enabled,
        "FAULT: integrator cleared, driver off");

  /* The controller never feeds the watchdog: v keepalives stop -> trip. */
  Rig quiet;
  rig_init(&quiet, 20.0, 5);
  rig_arm(&quiet);
  send_v(&quiet, 40);
  rig_run(&quiet, 2000u);
  quiet.send_keepalive = false;
  uint32_t last_v = quiet.motor.last_command_ms;
  while (quiet.motor.watchdog_trip_count == 0u && quiet.now_ms - last_v < 2000u) { rig_step(&quiet); }
  CHECK(quiet.motor.watchdog_trip_count == 1u && quiet.now_ms - last_v == (uint32_t)MOTOR_COMMAND_TIMEOUT_MS,
        "v stops: tripped %u ms after the last v although the controller kept writing", (unsigned)(quiet.now_ms - last_v));
  rig_step(&quiet);
  CHECK(quiet.speed.integrator_q8 == 0, "and the integrator is cleared");
}

/* ---- Random setpoints: the sign-symmetric clamps hold -------------------------------------- */

static uint32_t random_state = 0x5EEDu;
static uint32_t next_random(void)
{
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

static void test_random_setpoints_keep_the_clamps(void)
{
  Rig rig;
  rig_init(&rig, 25.0, 8);
  rig_arm(&rig);
  for (uint32_t t = 0u; t < 600000u; t++)
  {
    if (next_random() % 1500u == 0u)
    {
      int32_t v = (int32_t)(next_random() % 201u) - 100;
      if (next_random() % 5u == 0u) { send_m(&rig, (int32_t)(next_random() % 61u) - 30); }
      else { send_v(&rig, v); }
    }
    if (next_random() % 20000u == 0u) { rig.plant.load = 0.5 + (next_random() % 50u) / 100.0; }
    rig_step(&rig);
  }
  CHECK(rig.sign_violations == 0 && rig.integrator_out_of_range == 0,
        "10 random minutes: %d sign violations, %d integrator out of range", rig.sign_violations,
        rig.integrator_out_of_range);
  CHECK(rig.motor.watchdog_trip_count == 0u && rig.motor.state != MOTOR_STATE_FAULT, "and nothing tripped");
}

/* ---- Telemetry: ctl and vi -------------------------------------------------------------------- */

static void test_reported_state_and_integrator(void)
{
  /* The codes are part of the telemetry format. */
  CHECK(SPEED_CTL_OPEN_LOOP == 0 && SPEED_CTL_RUNNING == 1 && SPEED_CTL_STOPPED == 2 && SPEED_CTL_GATE == 3,
        "ctl codes 0 open loop, 1 running, 2 stopped, 3 gate");

  Rig rig;
  rig_init(&rig, 20.0, 5);
  CHECK(SpeedControl_State(&rig.speed) == SPEED_CTL_OPEN_LOOP && SpeedControl_IntegratorTicks(&rig.speed) == 0,
        "after Init: open loop, vi 0");
  rig_arm(&rig);
  CHECK(SpeedControl_State(&rig.speed) == SPEED_CTL_STOPPED, "v 0: stopped");
  send_v(&rig, SPEED_MIN_SETPOINT_UNITS - 1);
  rig_step(&rig);
  CHECK(SpeedControl_State(&rig.speed) == SPEED_CTL_STOPPED, "below the minimum setpoint: stopped");
  send_v(&rig, 50);
  rig_run(&rig, 2000u);
  CHECK(SpeedControl_State(&rig.speed) == SPEED_CTL_RUNNING, "v 50: running");
  int32_t ticks = SpeedControl_IntegratorTicks(&rig.speed);
  CHECK(abs(ticks * SPEED_Q8_ONE - rig.speed.integrator_q8) <= SPEED_Q8_ONE / 2 && ticks > 0,
        "vi is the integrator in whole ticks (%d, Q8 %d)", (int)ticks, (int)rig.speed.integrator_q8);

  /* Reverse while still turning forward: the gate, until the wheel stops. */
  send_v(&rig, -50);
  bool gate_seen = false;
  bool gate_while_stopped = false;
  for (uint32_t t = 0u; t < 4000u; t++)
  {
    /* The controller decides on the estimate from before this ms's encoder update. */
    int32_t seen_units = rig.estimator.speed_units;
    bool seen_valid = rig.estimator.valid;
    rig_step(&rig);
    if (SpeedControl_State(&rig.speed) == SPEED_CTL_GATE)
    {
      gate_seen = true;
      if (abs(seen_units) <= SPEED_GATE_UNITS && seen_valid) { gate_while_stopped = true; }
    }
  }
  CHECK(gate_seen && !gate_while_stopped, "reversal: gate while the wheel turns, never once it has stopped");
  CHECK(SpeedControl_State(&rig.speed) == SPEED_CTL_RUNNING && SpeedControl_IntegratorTicks(&rig.speed) < 0,
        "then running the other way, vi negative (%d)", (int)SpeedControl_IntegratorTicks(&rig.speed));

  /* Rounding: half away from zero, both signs. */
  rig.speed.integrator_q8 = 128;
  CHECK(SpeedControl_IntegratorTicks(&rig.speed) == 1, "0.5 tick -> 1");
  rig.speed.integrator_q8 = -128;
  CHECK(SpeedControl_IntegratorTicks(&rig.speed) == -1, "-0.5 tick -> -1");
  rig.speed.integrator_q8 = 127;
  CHECK(SpeedControl_IntegratorTicks(&rig.speed) == 0, "0.496 tick -> 0");

  send_m(&rig, 0);
  CHECK(SpeedControl_State(&rig.speed) == SPEED_CTL_OPEN_LOOP && SpeedControl_IntegratorTicks(&rig.speed) == 0,
        "m: open loop again, vi 0");
}

int main(void)
{
  test_pinned_constants();
  test_step_from_rest_for_tau_0_to_30_ms();
  test_first_output_after_a_start_from_rest();
  test_wheel_forced_faster_never_reverses();
  test_saturation_does_not_wind_up();
  test_load_disturbance_is_rejected();
  test_minimum_setpoint_and_v_0();
  test_sign_change_waits_for_the_wheel_to_stop();
  test_m_switches_the_controller_off_and_back_bumplessly();
  test_preload_starts_from_rest_faster();
  test_invalid_estimate_holds_then_latches();
  test_latch_fault_and_watchdog();
  test_random_setpoints_keep_the_clamps();
  test_reported_state_and_integrator();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
