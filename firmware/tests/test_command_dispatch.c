/*
 * Host-side test for Core/Src/command_dispatch.c, together with the real
 * line parser, motor state machine, speed controller and steering
 * controller it connects.
 * Not part of the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_command_dispatch.c Core/Src/command_dispatch.c Core/Src/command_line.c Core/Src/motor_control.c Core/Src/steering_control.c Core/Src/speed_control.c -o /tmp/test_command_dispatch && /tmp/test_command_dispatch
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "command_dispatch.h"

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

/* Like the firmware: the motor, the steering, and the line they share. */
typedef struct
{
  CommandLine        line;
  MotorController    motor;
  SteeringController steering;
  bool               hang_test_enabled;
  uint32_t           now_ms;
  uint32_t           hang_ms_total;
  uint32_t           targets[COMMAND_TARGET_SPEED + 1];
  SpeedController    speed;
  bool               with_speed;   /* CommandDispatch_LineWithSpeed; else the old CommandDispatch_Line (main.c today) */
} Rig;

static void rig_init(Rig *rig, uint32_t start_ms, bool hang_test_enabled)
{
  memset(rig, 0, sizeof(*rig));
  rig->now_ms = start_ms;
  rig->hang_test_enabled = hang_test_enabled;
  CommandLine_Init(&rig->line);
  MotorController_Init(&rig->motor, start_ms);
  SteeringController_Init(&rig->steering, start_ms, false);
  SpeedControl_Init(&rig->speed);
}

/* Sends text byte by byte through the parser and the dispatcher, as
 * main.c does with the received bytes. */
static void rig_send(Rig *rig, const char *text)
{
  for (size_t i = 0; text[i] != '\0'; i++)
  {
    Command command = { 0, 0 };
    CommandLineResult line_result = CommandLine_FeedByte(&rig->line, (uint8_t)text[i], &command);
    CommandDispatchResult result =
        rig->with_speed ? CommandDispatch_LineWithSpeed(line_result, &command, &rig->motor, &rig->steering, &rig->speed,
                                                        rig->hang_test_enabled, rig->now_ms)
                        : CommandDispatch_Line(line_result, &command, &rig->motor, &rig->steering,
                                               rig->hang_test_enabled, rig->now_ms);
    rig->targets[result.target]++;
    rig->hang_ms_total += result.hang_ms;
  }
}

/* One ms of the loop: both controllers are updated, as in main.c. */
static void rig_step(Rig *rig, MotorOutput *motor_output, uint32_t *pulse_us)
{
  rig->now_ms++;
  *motor_output = MotorController_Update(&rig->motor, rig->now_ms);
  *pulse_us = SteeringController_Update(&rig->steering, rig->now_ms);
}

static void test_hang_switch_is_off(void)
{
  /* The committed firmware must not contain the hang command. */
  CHECK(IWDG_HANG_TEST == 0, "IWDG_HANG_TEST is %d: a hang-test build must never be committed", IWDG_HANG_TEST);
}

static void test_routing(void)
{
  Rig rig;
  rig_init(&rig, 100u, false);

  rig_send(&rig, "m 0\n");
  CHECK(rig.targets[COMMAND_TARGET_MOTOR] == 1u && rig.motor.valid_command_count == 1u &&
        rig.steering.valid_command_count == 0u, "m goes to the motor only");
  CHECK(rig.motor.state == MOTOR_STATE_OFF, "m 0 re-arms the motor");

  uint32_t motor_reference_ms = rig.motor.last_command_ms;
  rig.now_ms += 50u;
  rig_send(&rig, "s -40\n");
  CHECK(rig.targets[COMMAND_TARGET_STEERING] == 1u && rig.steering.valid_command_count == 1u &&
        rig.steering.requested_percent == -40, "s goes to the steering");
  CHECK(rig.motor.valid_command_count == 1u && rig.motor.last_command_ms == motor_reference_ms &&
        rig.motor.rejected_command_count == 0u, "s must not touch the motor (count, watchdog reference)");

  /* Every other letter is rejected and counted in cmd_bad, with no effect. */
  int others_ok = 1;
  for (char letter = 'a'; letter <= 'z'; letter++)
  {
    if (letter == 'm' || letter == 's') { continue; }
    char text[8] = { letter, ' ', '5', '\n', '\0' };
    uint32_t rejected_before = rig.motor.rejected_command_count;
    rig_send(&rig, text);
    if (rig.motor.rejected_command_count != rejected_before + 1u) { others_ok = 0; }
  }
  CHECK(others_ok, "every letter other than m and s should be rejected (h included, in a normal build; v too "
                   "without a speed controller, as in main.c today)");
  CHECK(rig.motor.valid_command_count == 1u && rig.steering.valid_command_count == 1u &&
        rig.motor.last_command_ms == motor_reference_ms, "rejected letters change nothing");
  CHECK(rig.targets[COMMAND_TARGET_HANG] == 0u && rig.hang_ms_total == 0u, "no hang in a normal build");

  /* Malformed lines are counted the same way; empty lines aren't. */
  uint32_t rejected_before = rig.motor.rejected_command_count;
  rig_send(&rig, "s 101\nM 5\n\n\r\nm 20 \n");
  CHECK(rig.motor.rejected_command_count == rejected_before + 3u, "3 malformed lines -> cmd_bad +3, got +%u",
        (unsigned)(rig.motor.rejected_command_count - rejected_before));
  CHECK(rig.targets[COMMAND_TARGET_NONE] > 0u, "incomplete and empty lines -> no target");
}

static void test_hang_build(void)
{
  Rig rig;
  rig_init(&rig, 0u, true);
  rig_send(&rig, "h 30\n");
  CHECK(rig.targets[COMMAND_TARGET_HANG] == 1u && rig.hang_ms_total == 300u, "h 30 -> 300 ms stall, got %u",
        (unsigned)rig.hang_ms_total);
  rig_send(&rig, "h 1\nh 100\n");
  CHECK(rig.hang_ms_total == 300u + 10u + 1000u, "h 1 + h 100 -> 1010 ms more");
  uint32_t rejected_before = rig.motor.rejected_command_count;
  rig_send(&rig, "h 0\nh -5\n");
  CHECK(rig.targets[COMMAND_TARGET_HANG] == 3u && rig.motor.rejected_command_count == rejected_before + 2u,
        "h 0 and h -5 are rejected even in a hang-test build");
  CHECK(rig.motor.valid_command_count == 0u && rig.steering.valid_command_count == 0u,
        "h feeds neither controller");
}

/* The safety property: "m" and "v" feed the motor watchdog, "s" feeds the
 * steering. A sender that keeps sending "s" but has stopped sending "m" (and
 * "v") can't keep the motor running. */
static void test_steering_cannot_keep_motor_alive(void)
{
  Rig rig;
  rig_init(&rig, 1000u, false);
  MotorOutput motor_output;
  uint32_t pulse_us = 0u;
  /* Whatever the calibrated limits map "s 10" and "s -50" to. */
  const uint32_t s10_us = Steering_PulseForPercent(10, false);
  const uint32_t s_minus50_us = Steering_PulseForPercent(-50, false);

  rig_send(&rig, "m 0\n");
  for (uint32_t ms = 0; ms < 1000u; ms++)
  {
    if (ms % 100u == 0u) { rig_send(&rig, "m 20\ns 10\n"); }
    rig_step(&rig, &motor_output, &pulse_us);
  }
  CHECK(motor_output.driver_enabled && motor_output.duty_ticks == 320u && pulse_us == s10_us,
        "setup: motor at 320, steering at s 10 (got %u, %u)", (unsigned)motor_output.duty_ticks, (unsigned)pulse_us);

  /* From now on only "s". The last "m" was 100 ms before this point... */
  uint32_t last_motor_command_ms = rig.motor.last_command_ms;
  uint32_t trip_after_ms = 0u;
  for (uint32_t ms = 0; ms < 2000u; ms++)
  {
    if (ms % 100u == 0u) { rig_send(&rig, "s 10\n"); }
    rig_step(&rig, &motor_output, &pulse_us);
    if (trip_after_ms == 0u && !motor_output.driver_enabled)
    {
      trip_after_ms = rig.now_ms - last_motor_command_ms;
    }
  }
  /* ...so the motor trips exactly 500 ms after it, and stays latched. */
  CHECK(trip_after_ms == (uint32_t)MOTOR_COMMAND_TIMEOUT_MS, "motor tripped %u ms after the last m, expected %d",
        (unsigned)trip_after_ms, MOTOR_COMMAND_TIMEOUT_MS);
  CHECK(rig.motor.state == MOTOR_STATE_LATCHED && rig.motor.watchdog_trip_count == 1u && !motor_output.driver_enabled,
        "only s for 2 s: motor latched, trips %u", (unsigned)rig.motor.watchdog_trip_count);
  CHECK(rig.steering.state == STEERING_STATE_ACTIVE && pulse_us == s10_us && rig.steering.timeout_count == 0u,
        "steering kept following s (state %d, pulse %u)", (int)rig.steering.state, (unsigned)pulse_us);

  /* And the other way round: only "m", steering centres, motor runs on. */
  rig_send(&rig, "m 0\n");
  for (uint32_t ms = 0; ms < 600u; ms++)
  {
    if (ms % 100u == 0u) { rig_send(&rig, "m 20\ns -50\n"); }
    rig_step(&rig, &motor_output, &pulse_us);
  }
  CHECK(pulse_us == s_minus50_us && motor_output.driver_enabled, "setup 2: steering at s -50, motor running");
  for (uint32_t ms = 0; ms < 2000u; ms++)
  {
    if (ms % 100u == 0u) { rig_send(&rig, "m 20\n"); }
    rig_step(&rig, &motor_output, &pulse_us);
  }
  CHECK(pulse_us == (uint32_t)SERVO_PULSE_CENTER_US && rig.steering.timeout_count == 1u,
        "only m for 2 s: steering centred, timeouts %u", (unsigned)rig.steering.timeout_count);
  CHECK(motor_output.driver_enabled && motor_output.duty_ticks == 320u && rig.motor.watchdog_trip_count == 1u,
        "only m: motor keeps running, no extra trip");
}

/* ---- "v" and the speed controller ---------------------------------------------------- */

static void test_routing_with_the_speed_controller(void)
{
  Rig rig;
  rig_init(&rig, 100u, false);
  rig.with_speed = true;

  rig_send(&rig, "v 0\n");
  CHECK(rig.targets[COMMAND_TARGET_SPEED] == 1u && rig.motor.valid_command_count == 1u && rig.motor.last_command_ms == 100u &&
        rig.speed.mode == SPEED_MODE_CLOSED_LOOP && rig.steering.valid_command_count == 0u,
        "v goes to the motor watchdog and the speed controller, not the steering");
  CHECK(rig.motor.state == MOTOR_STATE_OFF && rig.motor.target_ticks == 0, "v 0 re-arms, sets no target");

  rig.now_ms += 40u;
  rig_send(&rig, "v -45\n");
  CHECK(rig.speed.setpoint_units == -45 && rig.speed.effective_units == -45 && rig.motor.last_command_ms == 140u &&
        rig.motor.target_ticks == 0, "v -45: setpoint set, watchdog fed, still no target (the controller's job)");

  rig.now_ms += 40u;
  rig_send(&rig, "s 30\n");
  CHECK(rig.speed.setpoint_units == -45 && rig.speed.mode == SPEED_MODE_CLOSED_LOOP && rig.motor.last_command_ms == 140u,
        "s touches neither the speed controller nor the motor watchdog");

  rig_send(&rig, "m 20\n");
  CHECK(rig.speed.mode == SPEED_MODE_OPEN_LOOP && rig.speed.integrator_q8 == 0 && rig.motor.target_ticks == 320,
        "m: open loop (controller off), m's target");
  /* Let m 20 reach its duty, then v: dispatch hands the applied duty over,
   * so the controller carries on from it (bumpless). */
  MotorOutput output;
  uint32_t pulse_us;
  for (uint32_t ms = 0; ms < 400u; ms++)
  {
    if (ms % 100u == 0u) { rig_send(&rig, "m 20\n"); }
    rig_step(&rig, &output, &pulse_us);
  }
  rig_send(&rig, "v 30\n");
  CHECK(rig.speed.mode == SPEED_MODE_CLOSED_LOOP && rig.speed.setpoint_units == 30 && rig.speed.direction == 1 &&
        rig.speed.integrator_q8 == 320 * SPEED_Q8_ONE, "m 20 -> v 30: closed loop, integrator starts at the applied 320");

  /* The grammar is the same for every letter: |v| <= 100. */
  uint32_t rejected_before = rig.motor.rejected_command_count;
  rig_send(&rig, "v 101\nv -101\nv 5x\n");
  CHECK(rig.motor.rejected_command_count == rejected_before + 3u && rig.speed.setpoint_units == 30,
        "v 101, v -101, v 5x: rejected, setpoint unchanged");

  /* Every letter other than m, s and v is still rejected. */
  int others_ok = 1;
  for (char letter = 'a'; letter <= 'z'; letter++)
  {
    if (letter == 'm' || letter == 's' || letter == 'v') { continue; }
    char text[8] = { letter, ' ', '5', '\n', '\0' };
    uint32_t before = rig.motor.rejected_command_count;
    rig_send(&rig, text);
    if (rig.motor.rejected_command_count != before + 1u) { others_ok = 0; }
  }
  CHECK(others_ok, "with the speed controller: every letter other than m, s and v rejected");
}

static void test_without_a_speed_controller_v_is_rejected(void)
{
  /* CommandDispatch_Line (what main.c calls until it is wired): "v" is an
   * unknown letter, exactly as before. */
  Rig rig;
  rig_init(&rig, 100u, false);
  rig_send(&rig, "v 0\nv 40\n");
  CHECK(rig.targets[COMMAND_TARGET_REJECTED] == 2u && rig.targets[COMMAND_TARGET_SPEED] == 0u &&
        rig.motor.rejected_command_count == 2u && rig.motor.valid_command_count == 0u &&
        rig.motor.state == MOTOR_STATE_LATCHED && rig.speed.mode == SPEED_MODE_OPEN_LOOP,
        "no speed controller: v rejected (cmd_bad), motor untouched and still latched");
}

static void test_v_keeps_the_motor_alive_and_s_still_cannot(void)
{
  Rig rig;
  rig_init(&rig, 1000u, false);
  rig.with_speed = true;
  MotorOutput motor_output;
  uint32_t pulse_us = 0u;

  /* v keepalives every 100 ms, the controller's target every 10 ms (as
   * SpeedControl_Update does through SetClosedLoopTarget). */
  rig_send(&rig, "v 0\n");
  for (uint32_t ms = 0; ms < 2000u; ms++)
  {
    if (ms % 100u == 0u) { rig_send(&rig, "v 40\ns 10\n"); }
    if (ms % 10u == 0u) { MotorController_SetClosedLoopTarget(&rig.motor, 300); }
    rig_step(&rig, &motor_output, &pulse_us);
  }
  CHECK(motor_output.driver_enabled && motor_output.duty_ticks == 300u && rig.motor.watchdog_trip_count == 0u,
        "a v stream keeps the motor running (duty %u, trips %u)", (unsigned)motor_output.duty_ticks,
        (unsigned)rig.motor.watchdog_trip_count);

  /* Only s from now on, while the controller keeps writing its target:
   * neither feeds the watchdog, so it trips 500 ms after the last v. */
  uint32_t last_v_ms = rig.motor.last_command_ms;
  uint32_t trip_after_ms = 0u;
  for (uint32_t ms = 0; ms < 2000u; ms++)
  {
    if (ms % 100u == 0u) { rig_send(&rig, "s 10\n"); }
    if (ms % 10u == 0u) { MotorController_SetClosedLoopTarget(&rig.motor, 300); }
    rig_step(&rig, &motor_output, &pulse_us);
    if (trip_after_ms == 0u && !motor_output.driver_enabled) { trip_after_ms = rig.now_ms - last_v_ms; }
  }
  CHECK(trip_after_ms == (uint32_t)MOTOR_COMMAND_TIMEOUT_MS && rig.motor.state == MOTOR_STATE_LATCHED,
        "only s (+ the controller's target) after the last v: tripped %u ms later, expected %d", (unsigned)trip_after_ms,
        MOTOR_COMMAND_TIMEOUT_MS);
}

int main(void)
{
  test_hang_switch_is_off();
  test_routing();
  test_hang_build();
  test_steering_cannot_keep_motor_alive();
  test_routing_with_the_speed_controller();
  test_without_a_speed_controller_v_is_rejected();
  test_v_keeps_the_motor_alive_and_s_still_cannot();

  printf("%d checks, %d failed\n", check_count, failure_count);
  if (failure_count != 0)
  {
    return EXIT_FAILURE;
  }
  printf("ALL PASSED\n");
  return EXIT_SUCCESS;
}
