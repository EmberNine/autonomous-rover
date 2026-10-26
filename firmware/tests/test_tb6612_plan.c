/*
 * Host-side test for Core/Src/tb6612_plan.c (the output stage's guards),
 * together with Core/Src/motor_control.c. Not part of the firmware build or
 * CI. Compiles both with the PC's C compiler.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_tb6612_plan.c Core/Src/tb6612_plan.c Core/Src/motor_control.c -o /tmp/test_tb6612_plan && /tmp/test_tb6612_plan
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 *
 * The registers are simulated by SimBridge, which TB6612_Plan reads and
 * writes exactly like tb6612.c does on the real pins. The key test
 * (test_same_as_old_output_stage) runs the state machine through every
 * normal sequence and requires the new output stage to write exactly what
 * the old TB6612_Apply wrote, every ms, with no guard ever firing.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor_control.h"
#include "tb6612_plan.h"

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

#define PERIOD  ((uint32_t)MOTOR_PWM_PERIOD_TICKS)
#define CAP     ((uint32_t)MOTOR_DUTY_CAP_TICKS)

/* ---- Simulated pins and compare register ---------------------------------- */

typedef struct
{
  bool     standby_high;
  bool     in1_high;
  bool     in2_high;
  uint32_t compare_ticks;  /* last value written (what a read returns) */
} SimBridge;

static const SimBridge SIM_RESET = { false, false, false, 0u };

static bool sim_equal(const SimBridge *a, const SimBridge *b)
{
  return a->standby_high == b->standby_high && a->in1_high == b->in1_high && a->in2_high == b->in2_high &&
         a->compare_ticks == b->compare_ticks;
}

/* What tb6612.c does: read, plan, write (a disabled plan is all low, 0). */
static TB6612_PinPlan sim_apply(SimBridge *bridge, const MotorOutput *output, bool invert, uint32_t period)
{
  TB6612_PinPlan plan = TB6612_Plan(output, invert, bridge->in1_high, bridge->in2_high, bridge->compare_ticks, period);
  bridge->standby_high = plan.standby_high;
  bridge->in1_high = plan.in1_high;
  bridge->in2_high = plan.in2_high;
  bridge->compare_ticks = plan.compare_ticks;
  return plan;
}

/* The output stage before this change (TB6612_Apply at commit 3349246),
 * with TIM1 ARR = 1599. */
static void sim_apply_old(SimBridge *bridge, const MotorOutput *output, bool invert)
{
  if (!output->driver_enabled)
  {
    *bridge = SIM_RESET;
    return;
  }
  bool in1_high = (output->direction == MOTOR_DIRECTION_FORWARD);
  bool in2_high = (output->direction == MOTOR_DIRECTION_REVERSE);
  if (invert)
  {
    bool swap = in1_high;
    in1_high = in2_high;
    in2_high = swap;
  }
  uint32_t duty_ticks = output->duty_ticks;
  if (in1_high != bridge->in1_high || in2_high != bridge->in2_high)
  {
    duty_ticks = 0u;
  }
  if (duty_ticks > PERIOD)
  {
    duty_ticks = PERIOD;
  }
  bridge->in1_high = in1_high;
  bridge->in2_high = in2_high;
  bridge->standby_high = true;
  bridge->compare_ticks = duty_ticks;
}

static MotorOutput request(bool enabled, MotorDirection direction, uint32_t duty_ticks)
{
  MotorOutput output = { .driver_enabled = enabled, .direction = direction, .duty_ticks = duty_ticks };
  return output;
}

static bool no_guard(const TB6612_PinPlan *plan)
{
  return !plan->period_refused && !plan->guard_coasted && !plan->drive_on_pin_change && !plan->capped;
}

/* ---- Single rules ----------------------------------------------------------- */

static void test_disabled(void)
{
  /* Rule 1, from every kind of previous state. */
  const SimBridge previous_states[] = {
    { false, false, false, 0u }, { true, true, false, 0u }, { true, true, false, 320u }, { true, false, true, 480u } };
  for (size_t i = 0; i < sizeof(previous_states) / sizeof(previous_states[0]); i++)
  {
    SimBridge bridge = previous_states[i];
    MotorOutput disabled = request(false, MOTOR_DIRECTION_COAST, 0u);
    TB6612_PinPlan plan = sim_apply(&bridge, &disabled, false, PERIOD);
    CHECK(sim_equal(&bridge, &SIM_RESET) && no_guard(&plan), "disabled from state %u: not all low / 0",
          (unsigned)i);
  }
  /* Disabled wins even with a wrong period (nothing to refuse). */
  SimBridge bridge = { true, true, false, 320u };
  MotorOutput disabled = request(false, MOTOR_DIRECTION_COAST, 0u);
  TB6612_PinPlan plan = sim_apply(&bridge, &disabled, false, 999u);
  CHECK(sim_equal(&bridge, &SIM_RESET) && !plan.period_refused, "disabled with a wrong period: no flag");
}

static void test_period_mismatch(void)
{
  /* Rule 2: any other period refuses to enable, and says so. */
  const uint32_t periods[] = { 0u, 800u, PERIOD - 1u, PERIOD + 1u, 3200u, 4000u, UINT32_MAX };
  for (size_t i = 0; i < sizeof(periods) / sizeof(periods[0]); i++)
  {
    SimBridge bridge = { true, true, false, 0u };
    MotorOutput forward = request(true, MOTOR_DIRECTION_FORWARD, 100u);
    TB6612_PinPlan plan = sim_apply(&bridge, &forward, false, periods[i]);
    CHECK(sim_equal(&bridge, &SIM_RESET) && plan.period_refused &&
          TB6612_PlanFaultReason(&plan) == MOTOR_FAULT_PERIOD_MISMATCH,
          "period %u: should be disabled and refused", (unsigned)periods[i]);
  }
  SimBridge bridge = { true, true, false, 0u };
  MotorOutput forward = request(true, MOTOR_DIRECTION_FORWARD, 100u);
  TB6612_PinPlan plan = sim_apply(&bridge, &forward, false, PERIOD);
  CHECK(!plan.period_refused && bridge.compare_ticks == 100u, "period %u should be accepted", (unsigned)PERIOD);
}

static void test_cap(void)
{
  const struct { uint32_t asked; uint32_t written; bool capped; } cases[] = {
    { 0u, 0u, false }, { CAP, CAP, false }, { CAP + 1u, CAP, true }, { PERIOD, CAP, true }, { UINT32_MAX, CAP, true } };
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
  {
    SimBridge bridge = { true, true, false, 100u };  /* already forward: rule 5 */
    MotorOutput forward = request(true, MOTOR_DIRECTION_FORWARD, cases[i].asked);
    TB6612_PinPlan plan = sim_apply(&bridge, &forward, false, PERIOD);
    CHECK(bridge.compare_ticks == cases[i].written && plan.capped == cases[i].capped,
          "duty %u: wrote %u capped=%d, expected %u capped=%d", (unsigned)cases[i].asked,
          (unsigned)bridge.compare_ticks, (int)plan.capped, (unsigned)cases[i].written, (int)cases[i].capped);
  }
  /* Reported even in a call that doesn't apply the duty (pins change). That
   * call is also a drive on a pin change, which takes precedence as reason. */
  SimBridge bridge = { false, false, false, 0u };
  MotorOutput forward = request(true, MOTOR_DIRECTION_FORWARD, 600u);
  TB6612_PinPlan plan = sim_apply(&bridge, &forward, false, PERIOD);
  CHECK(plan.capped && plan.drive_on_pin_change && bridge.compare_ticks == 0u &&
        TB6612_PlanFaultReason(&plan) == MOTOR_FAULT_DIRECTION_GUARD,
        "over-cap request on a pin-change call: capped=%d drive_on_pin_change=%d compare=%u", (int)plan.capped,
        (int)plan.drive_on_pin_change, (unsigned)bridge.compare_ticks);
}

static void test_pin_rules(void)
{
  /* Rule 4: coast -> forward with compare 0 and duty 0 asked (the wake-up
   * ms) -> new pins, compare 0, no flag. */
  SimBridge bridge = { true, false, false, 0u };
  MotorOutput forward = request(true, MOTOR_DIRECTION_FORWARD, 0u);
  TB6612_PinPlan plan = sim_apply(&bridge, &forward, false, PERIOD);
  CHECK(bridge.standby_high && bridge.in1_high && !bridge.in2_high && bridge.compare_ticks == 0u && no_guard(&plan),
        "rule 4 coast -> forward");

  /* Rule 4 with a non-zero duty asked (never done by the state machine):
   * pins change, compare stays 0, and it is reported as a direction fault. */
  const uint32_t pin_change_duties[] = { 1u, 2u, 320u, CAP };
  for (size_t i = 0; i < sizeof(pin_change_duties) / sizeof(pin_change_duties[0]); i++)
  {
    bridge = (SimBridge){ true, true, false, 0u };
    MotorOutput reverse = request(true, MOTOR_DIRECTION_REVERSE, pin_change_duties[i]);
    plan = sim_apply(&bridge, &reverse, false, PERIOD);
    CHECK(!bridge.in1_high && bridge.in2_high && bridge.compare_ticks == 0u && plan.drive_on_pin_change &&
          !plan.guard_coasted && !plan.capped && TB6612_PlanFaultReason(&plan) == MOTOR_FAULT_DIRECTION_GUARD,
          "rule 4 with duty %u: drive_on_pin_change=%d compare=%u reason=%d", (unsigned)pin_change_duties[i],
          (int)plan.drive_on_pin_change, (unsigned)bridge.compare_ticks, (int)TB6612_PlanFaultReason(&plan));
  }

  /* Rule 4: forward -> reverse with compare 0 (end of the reverse dwell). */
  bridge = (SimBridge){ true, true, false, 0u };
  MotorOutput reverse = request(true, MOTOR_DIRECTION_REVERSE, 0u);
  plan = sim_apply(&bridge, &reverse, false, PERIOD);
  CHECK(!bridge.in1_high && bridge.in2_high && bridge.compare_ticks == 0u && no_guard(&plan),
        "rule 4 forward -> reverse");

  /* Rule 5: same pins -> duty. */
  bridge = (SimBridge){ true, false, true, 0u };
  reverse = request(true, MOTOR_DIRECTION_REVERSE, 2u);
  plan = sim_apply(&bridge, &reverse, false, PERIOD);
  CHECK(!bridge.in1_high && bridge.in2_high && bridge.compare_ticks == 2u && no_guard(&plan), "rule 5 reverse");

  /* Rule 3: forward at 320 -> reverse: coast at once, STBY stays high. */
  bridge = (SimBridge){ true, true, false, 320u };
  reverse = request(true, MOTOR_DIRECTION_REVERSE, 320u);
  plan = sim_apply(&bridge, &reverse, false, PERIOD);
  CHECK(bridge.standby_high && !bridge.in1_high && !bridge.in2_high && bridge.compare_ticks == 0u &&
        plan.guard_coasted && TB6612_PlanFaultReason(&plan) == MOTOR_FAULT_DIRECTION_GUARD,
        "rule 3 forward(320) -> reverse should coast");

  /* Rule 3 also for forward -> coast-direction and with a duty of 0 asked. */
  bridge = (SimBridge){ true, true, false, 2u };
  MotorOutput coast = request(true, MOTOR_DIRECTION_COAST, 0u);
  plan = sim_apply(&bridge, &coast, false, PERIOD);
  CHECK(!bridge.in1_high && !bridge.in2_high && bridge.compare_ticks == 0u && plan.guard_coasted,
        "rule 3 forward(2) -> coast");

  /* Invert: forward = IN1 low / IN2 high. */
  bridge = (SimBridge){ true, false, true, 50u };
  forward = request(true, MOTOR_DIRECTION_FORWARD, 60u);
  plan = sim_apply(&bridge, &forward, true, PERIOD);
  CHECK(!bridge.in1_high && bridge.in2_high && bridge.compare_ticks == 60u && no_guard(&plan),
        "invert: forward should be IN1 low / IN2 high");
  bridge = (SimBridge){ true, false, true, 50u };
  reverse = request(true, MOTOR_DIRECTION_REVERSE, 60u);
  plan = sim_apply(&bridge, &reverse, true, PERIOD);
  CHECK(plan.guard_coasted && !bridge.in1_high && !bridge.in2_high, "invert: reverse from inverted forward coasts");

  /* Only one reason when several guards fire: direction before cap. */
  bridge = (SimBridge){ true, true, false, 300u };
  reverse = request(true, MOTOR_DIRECTION_REVERSE, 900u);
  plan = sim_apply(&bridge, &reverse, false, PERIOD);
  CHECK(plan.guard_coasted && plan.capped && TB6612_PlanFaultReason(&plan) == MOTOR_FAULT_DIRECTION_GUARD,
        "direction + cap: reason should be DIRECTION_GUARD");
  plan = TB6612_Plan(&reverse, false, true, false, 300u, PERIOD);
  CHECK(TB6612_PlanFaultReason(&plan) != MOTOR_FAULT_NONE, "a fired guard must give a reason");
}

/* The plug-brake case from the design review, ms by ms, without the FAULT
 * latch (the plan module on its own): coast, then new pins at 0, then duty. */
static void test_hazard_sequence_without_latch(void)
{
  SimBridge bridge = { true, true, false, 320u };  /* forward at 20 % */
  MotorOutput reverse = request(true, MOTOR_DIRECTION_REVERSE, 320u);

  TB6612_PinPlan plan = sim_apply(&bridge, &reverse, false, PERIOD);  /* ms h */
  CHECK(plan.guard_coasted && !bridge.in1_high && !bridge.in2_high && bridge.compare_ticks == 0u, "h: coast");
  plan = sim_apply(&bridge, &reverse, false, PERIOD);                 /* ms h+1 */
  CHECK(!plan.guard_coasted && plan.drive_on_pin_change && !bridge.in1_high && bridge.in2_high &&
        bridge.compare_ticks == 0u, "h+1: reverse pins with compare 0, no second coast, still reported");
  plan = sim_apply(&bridge, &reverse, false, PERIOD);                 /* ms h+2 */
  CHECK(no_guard(&plan) && bridge.in2_high && bridge.compare_ticks == 320u, "h+2: reverse at 320");
}

/* ---- With the state machine ------------------------------------------------- */

/* One firmware loop iteration: commands are given by the caller, then
 * Update, the output stage, and the FAULT latch exactly as in main.c. */
typedef struct
{
  MotorController controller;
  SimBridge       bridge;      /* new output stage */
  SimBridge       old_bridge;  /* old output stage, same requests */
  bool            invert;
  uint32_t        now_ms;
  uint32_t        mismatch_ms;   /* ms where new and old wrote different things */
  uint32_t        guard_ms;      /* ms where any guard fired */
  uint32_t        pin_changes;   /* enabled ms where the IN pins changed (coverage) */
  uint32_t        enabled_ms;
} LoopSim;

static void loop_init(LoopSim *sim, bool invert, uint32_t start_ms)
{
  memset(sim, 0, sizeof(*sim));
  sim->invert = invert;
  sim->now_ms = start_ms;
  sim->bridge = SIM_RESET;
  sim->old_bridge = SIM_RESET;
  MotorController_Init(&sim->controller, start_ms);
}

static MotorOutput loop_step(LoopSim *sim)
{
  sim->now_ms++;
  MotorOutput output = MotorController_Update(&sim->controller, sim->now_ms);
  SimBridge before = sim->bridge;
  TB6612_PinPlan plan = sim_apply(&sim->bridge, &output, sim->invert, PERIOD);
  sim_apply_old(&sim->old_bridge, &output, sim->invert);
  MotorFaultReason fault = TB6612_PlanFaultReason(&plan);
  if (fault != MOTOR_FAULT_NONE)
  {
    sim->guard_ms++;
    MotorController_LockOut(&sim->controller, fault);
  }
  if (!sim_equal(&sim->bridge, &sim->old_bridge))
  {
    if (sim->mismatch_ms < 3u)
    {
      printf("  mismatch at t=%u: new stby=%d in=%d%d cmp=%u, old stby=%d in=%d%d cmp=%u\n", (unsigned)sim->now_ms,
             sim->bridge.standby_high, sim->bridge.in1_high, sim->bridge.in2_high, (unsigned)sim->bridge.compare_ticks,
             sim->old_bridge.standby_high, sim->old_bridge.in1_high, sim->old_bridge.in2_high,
             (unsigned)sim->old_bridge.compare_ticks);
    }
    sim->mismatch_ms++;
  }
  if (sim->bridge.standby_high)
  {
    sim->enabled_ms++;
    if (sim->bridge.in1_high != before.in1_high || sim->bridge.in2_high != before.in2_high) { sim->pin_changes++; }
  }
  return output;
}

/* Commands percent every 100 ms (like the console) for duration_ms. */
static void loop_drive(LoopSim *sim, int32_t percent, uint32_t duration_ms)
{
  for (uint32_t elapsed = 0; elapsed < duration_ms; elapsed++)
  {
    if (elapsed % 100u == 0u) { MotorController_AcceptCommand(&sim->controller, percent, sim->now_ms); }
    (void)loop_step(sim);
  }
}

static void loop_silent(LoopSim *sim, uint32_t duration_ms)
{
  for (uint32_t elapsed = 0; elapsed < duration_ms; elapsed++) { (void)loop_step(sim); }
}

/* Every normal sequence, then a random hour: the new output stage must write
 * exactly what the old one wrote, and no guard may ever fire (so the FAULT
 * latch in main.c can't trip on a working state machine). */
static void test_same_as_old_output_stage(void)
{
  for (int invert = 0; invert <= 1; invert++)
  {
    LoopSim sim_storage;
    LoopSim *sim = &sim_storage;
    loop_init(sim, invert != 0, 100u);

    loop_drive(sim, 20, 300);    /* latched: ignored */
    loop_drive(sim, 0, 200);     /* arm */
    loop_drive(sim, 20, 600);    /* wake-up, ramp to 320, hold */
    loop_drive(sim, -20, 800);   /* reversal: ramp down, dwell, switch, ramp up */
    loop_drive(sim, 100, 900);   /* clamped command, reversal back */
    loop_drive(sim, -30, 170);   /* start a reversal ... */
    loop_drive(sim, 30, 400);    /* ... and cancel it (in the ramp or the dwell) */
    loop_drive(sim, 0, 400);     /* stop: ramp down, disabled */
    loop_drive(sim, -15, 300);   /* wake-up in reverse */
    loop_silent(sim, 700);       /* console gone: watchdog trip */
    loop_drive(sim, 25, 300);    /* latched again: ignored */
    loop_drive(sim, 0, 100);     /* re-arm */
    loop_drive(sim, 25, 500);
    loop_drive(sim, 0, 300);
    loop_drive(sim, 25, 1);      /* wake-up right after a stop */
    loop_drive(sim, 0, 300);

    /* Random hour, like test_motor_control.c's. */
    for (uint32_t ms = 0; ms < 3600u * 1000u; ms++)
    {
      uint32_t dice = next_random() % 1000u;
      if (dice < 10u)
      {
        int32_t percent = (int32_t)(next_random() % 201u) - 100;
        if (next_random() % 4u == 0u) { percent = 0; }
        MotorController_AcceptCommand(&sim->controller, percent, sim->now_ms);
      }
      else if (dice == 12u)
      {
        sim->now_ms += next_random() % 700u;  /* occasional loop stall */
      }
      (void)loop_step(sim);
    }

    CHECK(sim->mismatch_ms == 0u, "invert=%d: new output stage differs from the old one in %u ms", invert,
          (unsigned)sim->mismatch_ms);
    CHECK(sim->guard_ms == 0u && sim->controller.fault_reason == MOTOR_FAULT_NONE,
          "invert=%d: a guard fired %u times on normal sequences", invert, (unsigned)sim->guard_ms);
    CHECK(sim->pin_changes > 3000u &&sim->enabled_ms > 1000000u && sim->controller.watchdog_trip_count > 10u,
          "invert=%d: run too tame: %u pin changes, %u enabled ms, %u trips", invert, (unsigned)sim->pin_changes,
          (unsigned)sim->enabled_ms, (unsigned)sim->controller.watchdog_trip_count);
    printf("  invert=%d: %u enabled ms, %u pin changes, %u trips, 0 differences expected: %u\n", invert,
           (unsigned)sim->enabled_ms, (unsigned)sim->pin_changes, (unsigned)sim->controller.watchdog_trip_count,
           (unsigned)sim->mismatch_ms);
  }
}

/* A guard firing latches FAULT (main.c's loop, simulated by loop_step): the
 * next ms is disabled, "m 0" doesn't clear it, and the reason is recorded. */
static void test_guard_latches_fault(void)
{
  /* Direction guard: the state machine asks for reverse at 320 while forward
   * at 320 is set (a broken state machine; injected by hand). */
  LoopSim sim_storage;
  LoopSim *sim = &sim_storage;
  loop_init(sim, false, 0u);
  loop_drive(sim, 0, 100);
  loop_drive(sim, 20, 400);
  CHECK(sim->bridge.in1_high && sim->bridge.compare_ticks == 320u, "setup: forward at 320");

  MotorOutput broken = request(true, MOTOR_DIRECTION_REVERSE, 320u);
  TB6612_PinPlan plan = sim_apply(&sim->bridge, &broken, false, PERIOD);
  MotorFaultReason fault = TB6612_PlanFaultReason(&plan);
  if (fault != MOTOR_FAULT_NONE) { MotorController_LockOut(&sim->controller, fault); }
  CHECK(!sim->bridge.in1_high && !sim->bridge.in2_high && sim->bridge.compare_ticks == 0u,
        "direction guard: coast in the same ms");
  CHECK(sim->controller.state == MOTOR_STATE_FAULT && sim->controller.fault_reason == MOTOR_FAULT_DIRECTION_GUARD,
        "direction guard: state %d reason %d", (int)sim->controller.state, (int)sim->controller.fault_reason);
  sim->old_bridge = sim->bridge;  /* keep the comparison model in step */
  MotorController_AcceptCommand(&sim->controller, 20, sim->now_ms);
  (void)loop_step(sim);
  CHECK(sim_equal(&sim->bridge, &SIM_RESET), "direction guard: the next ms is disabled");
  loop_drive(sim, 0, 300);
  loop_drive(sim, 20, 1000);
  CHECK(!sim->bridge.standby_high && sim->controller.state == MOTOR_STATE_FAULT &&
        sim->controller.fault_reason == MOTOR_FAULT_DIRECTION_GUARD,
        "direction guard: m 0 then m 20 must not leave FAULT");

  /* Direction guard, rule-4 form: from stopped (compare 0, pins low), the
   * state machine asks for forward at 320 straight away instead of a 0-duty
   * wake-up ms. Pins change with compare 0, and FAULT is latched. */
  loop_init(sim, false, 0u);
  loop_drive(sim, 0, 100);
  CHECK(sim_equal(&sim->bridge, &SIM_RESET), "setup: stopped");
  broken = request(true, MOTOR_DIRECTION_FORWARD, 320u);
  plan = sim_apply(&sim->bridge, &broken, false, PERIOD);
  fault = TB6612_PlanFaultReason(&plan);
  if (fault != MOTOR_FAULT_NONE) { MotorController_LockOut(&sim->controller, fault); }
  CHECK(sim->bridge.in1_high && sim->bridge.compare_ticks == 0u && plan.drive_on_pin_change,
        "rule-4 guard: pins set with compare 0 in the same ms");
  CHECK(sim->controller.state == MOTOR_STATE_FAULT && sim->controller.fault_reason == MOTOR_FAULT_DIRECTION_GUARD,
        "rule-4 guard: state %d reason %d", (int)sim->controller.state, (int)sim->controller.fault_reason);
  sim->old_bridge = sim->bridge;
  (void)loop_step(sim);
  CHECK(sim_equal(&sim->bridge, &SIM_RESET), "rule-4 guard: the next ms is disabled (the 320 is never applied)");
  loop_drive(sim, 0, 300);
  loop_drive(sim, 20, 1000);
  CHECK(!sim->bridge.standby_high && sim->controller.state == MOTOR_STATE_FAULT &&
        sim->controller.fault_reason == MOTOR_FAULT_DIRECTION_GUARD,
        "rule-4 guard: m 0 then m 20 must not leave FAULT");

  /* Cap guard: a duty above the cap reaches the output stage. */
  loop_init(sim, false, 0u);
  loop_drive(sim, 0, 100);
  loop_drive(sim, 20, 400);
  broken = request(true, MOTOR_DIRECTION_FORWARD, 481u);
  plan = sim_apply(&sim->bridge, &broken, false, PERIOD);
  fault = TB6612_PlanFaultReason(&plan);
  if (fault != MOTOR_FAULT_NONE) { MotorController_LockOut(&sim->controller, fault); }
  CHECK(sim->bridge.compare_ticks == CAP, "cap guard: clamped to %u in the same ms, got %u", (unsigned)CAP,
        (unsigned)sim->bridge.compare_ticks);
  CHECK(sim->controller.state == MOTOR_STATE_FAULT && sim->controller.fault_reason == MOTOR_FAULT_CAP_GUARD,
        "cap guard: state %d reason %d", (int)sim->controller.state, (int)sim->controller.fault_reason);
  (void)loop_step(sim);
  CHECK(sim_equal(&sim->bridge, &SIM_RESET), "cap guard: the next ms is disabled");
  loop_drive(sim, 0, 300);
  loop_drive(sim, 20, 1000);
  CHECK(!sim->bridge.standby_high && sim->controller.fault_reason == MOTOR_FAULT_CAP_GUARD,
        "cap guard: m 0 then m 20 must not leave FAULT");
}

/* ---- Random requests: rules that must hold whatever is asked ------------------ */

static void test_random_requests(void)
{
  for (int invert = 0; invert <= 1; invert++)
  {
    SimBridge bridge = SIM_RESET;
    bool previous_coasted = false;
    uint32_t over_cap = 0, drive_on_pin_change = 0, coast_twice = 0, not_settled = 0, coasts = 0;
    for (uint32_t i = 0; i < 200000u; i++)
    {
      static const uint32_t duty_choices[] = { 0u, 1u, 2u, 320u, 480u, 481u, 1600u, 5000u, UINT32_MAX };
      MotorOutput asked = request(next_random() % 5u != 0u, (MotorDirection)(next_random() % 3u),
                                  duty_choices[next_random() % (sizeof(duty_choices) / sizeof(duty_choices[0]))]);
      if (!asked.driver_enabled) { asked.direction = MOTOR_DIRECTION_COAST; asked.duty_ticks = 0u; }
      /* Hold each request for 1 to 4 calls. */
      uint32_t hold = 1u + next_random() % 4u;
      for (uint32_t call = 0; call < hold; call++)
      {
        SimBridge before = bridge;
        TB6612_PinPlan plan = sim_apply(&bridge, &asked, invert != 0, PERIOD);
        bool pins_changed = (bridge.in1_high != before.in1_high) || (bridge.in2_high != before.in2_high);
        bool drives = bridge.in1_high || bridge.in2_high;

        if (bridge.compare_ticks > CAP) { over_cap++; }
        /* Never a drive direction set while a non-zero compare may still be active. */
        if (bridge.standby_high && pins_changed && drives && before.compare_ticks != 0u) { drive_on_pin_change++; }
        /* A non-zero compare only on pins that didn't change. */
        if (bridge.compare_ticks != 0u && pins_changed) { drive_on_pin_change++; }
        if (plan.guard_coasted && previous_coasted) { coast_twice++; }
        if (plan.guard_coasted) { coasts++; }
        previous_coasted = plan.guard_coasted;

        /* From any state, the request is reached by the third call. */
        if (call == 2u && asked.driver_enabled)
        {
          bool want_in1 = (asked.direction == MOTOR_DIRECTION_FORWARD);
          bool want_in2 = (asked.direction == MOTOR_DIRECTION_REVERSE);
          if (invert) { bool swap = want_in1; want_in1 = want_in2; want_in2 = swap; }
          uint32_t want_compare = (asked.duty_ticks > CAP) ? CAP : asked.duty_ticks;
          if (!bridge.standby_high || bridge.in1_high != want_in1 || bridge.in2_high != want_in2 ||
              bridge.compare_ticks != want_compare)
          {
            not_settled++;
          }
        }
      }
    }
    CHECK(over_cap == 0u, "invert=%d: compare above the cap %u times", invert, (unsigned)over_cap);
    CHECK(drive_on_pin_change == 0u, "invert=%d: drive on a pin change %u times", invert,
          (unsigned)drive_on_pin_change);
    CHECK(coast_twice == 0u, "invert=%d: coasted twice in a row %u times", invert, (unsigned)coast_twice);
    CHECK(not_settled == 0u, "invert=%d: request not reached by the third call %u times (stuck?)", invert,
          (unsigned)not_settled);
    CHECK(coasts > 1000u, "invert=%d: only %u guard coasts: the run didn't exercise rule 3", invert,
          (unsigned)coasts);
  }
}

int main(void)
{
  test_disabled();
  test_period_mismatch();
  test_cap();
  test_pin_rules();
  test_hazard_sequence_without_latch();
  test_same_as_old_output_stage();
  test_guard_latches_fault();
  test_random_requests();

  printf("%d checks, %d failed\n", check_count, failure_count);
  if (failure_count != 0)
  {
    return EXIT_FAILURE;
  }
  printf("ALL PASSED\n");
  return EXIT_SUCCESS;
}
