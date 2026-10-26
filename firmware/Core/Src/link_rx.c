/**
  ******************************************************************************
  * @file    link_rx.c
  * @brief   Pi link receive side: frames, checks and actions (see the
  *          header).
  ******************************************************************************
  */
#include "link_rx.h"

void LinkRx_Init(LinkRx *link)
{
  *link = (LinkRx){0};
  link->mode = LINK_MODE_TEXT;
  link->last_reject = LINK_PARSE_OK;
}

bool LinkRx_Poison(LinkRx *link)
{
  if (!link->in_frame)
  {
    return false;
  }
  link->poisoned = true;
  return true;
}

bool LinkRx_TextCommandAllowed(LinkMode mode, char letter, bool bench_text_commands, bool test_build)
{
  if (mode == LINK_MODE_TEXT)
  {
    return bench_text_commands || test_build;
  }
  return test_build && (letter == 'e' || letter == 'c' || letter == 'h');
}

static LinkRxEvent LinkRx_Reject(LinkRx *link, LinkParseResult reason)
{
  link->rejected_count++;
  link->last_reject = reason;
  link->last_reject_type = link->frame.type;
  return LINK_RX_DROPPED;
}

/* A CRC-checked frame: check its type, mode and payload, then act. */
static LinkRxEvent LinkRx_Dispatch(LinkRx *link, const LinkRxTargets *targets, uint32_t now_ms)
{
  const LinkFrame *frame = &link->frame;
  LinkParseResult result;
  switch (frame->type)
  {
    case LINK_MSG_HELLO:
      result = LinkProtocol_ParseHello(frame);
      if (result != LINK_PARSE_OK)
      {
        return LinkRx_Reject(link, result);
      }
      /* Entering binary mode, or the Pi bridge restarted: either way the Pi
       * has to arm on purpose. No effect in FAULT. */
      link->mode = LINK_MODE_BINARY;
      MotorController_Latch(targets->motor, MOTOR_LATCH_LINK_HELLO);
      link->ok_count++;
      return LINK_RX_HELLO;

    case LINK_MSG_DRIVE:
    {
      if (link->mode != LINK_MODE_BINARY)
      {
        return LinkRx_Reject(link, LINK_PARSE_NOT_BINARY_MODE);
      }
      LinkDrive drive;
      result = LinkProtocol_ParseDrive(frame, &drive);
      if (result != LINK_PARSE_OK)
      {
        return LinkRx_Reject(link, result);
      }
      MotorController_AcceptDriveCommand(targets->motor, drive.arm, now_ms);
      SpeedControl_SetSetpoint(targets->speed, LinkProtocol_CpsToUnits(drive.speed_cps),
                               targets->motor->applied_ticks);
      if (drive.steer_valid)
      {
        SteeringController_AcceptCommand(targets->steering, drive.steer_percent, now_ms);
      }
      link->drive_seq_echo = frame->seq;
      link->drive_speed_cps = drive.speed_cps;
      link->ok_count++;
      return LINK_RX_DRIVE;
    }

    case LINK_MSG_ESTOP:
      if (link->mode != LINK_MODE_BINARY)
      {
        return LinkRx_Reject(link, LINK_PARSE_NOT_BINARY_MODE);
      }
      result = LinkProtocol_ParseEstop(frame);
      if (result != LINK_PARSE_OK)
      {
        return LinkRx_Reject(link, result);
      }
      MotorController_Latch(targets->motor, MOTOR_LATCH_ESTOP);
      link->ok_count++;
      return LINK_RX_ESTOP;

    default:
      return LinkRx_Reject(link, LINK_PARSE_UNKNOWN_TYPE);
  }
}

/* The closing 0x00 of a non-empty frame. */
static LinkRxEvent LinkRx_FinishFrame(LinkRx *link, const LinkRxTargets *targets, uint32_t now_ms)
{
  bool overlong = link->overlong;
  bool poisoned = link->poisoned;
  size_t length = link->length;
  link->in_frame = false;
  link->overlong = false;
  link->poisoned = false;
  link->length = 0u;

  if (overlong || poisoned)
  {
    link->frame_error_count++;
    return LINK_RX_DROPPED;
  }
  LinkFrameStatus status = LinkFrame_Decode(link->buffer, length, &link->frame);
  if (status == LINK_FRAME_CRC_ERROR)
  {
    link->crc_error_count++;
    return LINK_RX_DROPPED;
  }
  if (status != LINK_FRAME_OK)
  {
    link->frame_error_count++;
    return LINK_RX_DROPPED;
  }

  /* The frame is genuine (as far as a CRC can tell): count its seq. */
  uint8_t seq = link->frame.seq;
  if (link->seq_seen)
  {
    if (seq == link->last_seq)
    {
      link->duplicate_count++;
    }
    else if (seq != (uint8_t)(link->last_seq + 1u))
    {
      link->seq_gap_count++;
    }
  }
  link->seq_seen = true;
  link->last_seq = seq;

  return LinkRx_Dispatch(link, targets, now_ms);
}

LinkRxEvent LinkRx_FeedByte(LinkRx *link, uint8_t byte, const LinkRxTargets *targets, uint32_t now_ms)
{
  if (byte == 0u)
  {
    link->outside_line = false;
    if (!link->in_frame)
    {
      link->in_frame = true;
      link->length = 0u;
      link->overlong = false;
      link->poisoned = false;
      return LINK_RX_FRAME_START;
    }
    if (link->length == 0u && !link->overlong)
    {
      /* 0x00 0x00: an empty frame (a closing delimiter followed by the next
       * opening one). Still waiting for a frame; a poison on nothing goes. */
      link->poisoned = false;
      return LINK_RX_NONE;
    }
    return LinkRx_FinishFrame(link, targets, now_ms);
  }

  if (link->in_frame)
  {
    if (link->mode == LINK_MODE_TEXT && link->length >= LINK_TEXT_MODE_MAX_COBS)
    {
      /* In TEXT mode only a HELLO can act, and a HELLO is exactly
       * LINK_TEXT_MODE_MAX_COBS bytes. Anything longer was a stray 0x00 in
       * the text (noise, a NUL typed in a terminal): give up on it now and
       * hand this byte back as text, so the console isn't swallowed until a
       * 0x00 that a text console never sends. */
      link->in_frame = false;
      link->length = 0u;
      link->overlong = false;
      link->poisoned = false;
      link->frame_error_count++;
      return LINK_RX_TEXT_BYTE;
    }
    if (link->length < LINK_MAX_COBS)
    {
      link->buffer[link->length] = byte;
      link->length++;
    }
    else
    {
      link->overlong = true;  /* keep waiting for the 0x00, then drop it all */
    }
    return LINK_RX_NONE;
  }

  if (link->mode == LINK_MODE_BINARY)
  {
    if (!link->outside_line)
    {
      link->outside_line = true;
      link->text_ignored_count++;
    }
    if (byte == (uint8_t)'\n')
    {
      link->outside_line = false;
    }
  }
  return LINK_RX_TEXT_BYTE;
}
