/**
  ******************************************************************************
  * @file    command_dispatch.c
  * @brief   Routes command lines to the motor, the speed controller or the
  *          steering (see the header).
  ******************************************************************************
  */
#include "command_dispatch.h"

CommandDispatchResult CommandDispatch_Line(CommandLineResult line_result, const Command *command,
                                           MotorController *motor, SteeringController *steering,
                                           bool hang_test_enabled, uint32_t now_ms)
{
  return CommandDispatch_LineWithSpeed(line_result, command, motor, steering, NULL, hang_test_enabled, now_ms);
}

CommandDispatchResult CommandDispatch_LineWithSpeed(CommandLineResult line_result, const Command *command,
                                                    MotorController *motor, SteeringController *steering,
                                                    SpeedController *speed, bool hang_test_enabled, uint32_t now_ms)
{
  CommandDispatchResult result = { .target = COMMAND_TARGET_NONE, .hang_ms = 0u };

  if (line_result == COMMAND_LINE_VALID)
  {
    switch (command->letter)
    {
      case 'm':
        MotorController_AcceptCommand(motor, command->value, now_ms);
        if (speed != NULL)
        {
          SpeedControl_SelectOpenLoop(speed);   /* last command wins: open loop */
        }
        result.target = COMMAND_TARGET_MOTOR;
        return result;

      case 'v':
        if (speed != NULL)
        {
          MotorController_AcceptSpeedCommand(motor, command->value, now_ms);
          SpeedControl_SetSetpoint(speed, command->value, motor->applied_ticks);
          result.target = COMMAND_TARGET_SPEED;
          return result;
        }
        break;  /* no speed controller: rejected below, like an unknown letter */

      case 's':
        SteeringController_AcceptCommand(steering, command->value, now_ms);
        result.target = COMMAND_TARGET_STEERING;
        return result;

      case 'h':
        if (hang_test_enabled && command->value > 0)
        {
          result.target = COMMAND_TARGET_HANG;
          result.hang_ms = (uint32_t)command->value * HANG_TEST_MS_PER_UNIT;
          return result;
        }
        break;  /* normal build, or h <= 0: rejected below */

      default:
        break;  /* unknown letter: rejected below */
    }
  }
  else if (line_result != COMMAND_LINE_REJECTED)
  {
    return result;  /* incomplete or empty: nothing to do */
  }

  MotorController_CountRejected(motor);
  result.target = COMMAND_TARGET_REJECTED;
  return result;
}
