/**
  ******************************************************************************
  * @file    speed_capture.c
  * @brief   TEMPORARY motor step capture for identification (see the header).
  ******************************************************************************
  */
#include "speed_capture.h"

#define HEADER_ITEMS  11u  /* "caph", 9 fields, "\r\n" */

static int16_t SpeedCapture_ToInt16(int32_t value)
{
  /* INT16_MIN is reserved for "missing". */
  if (value > INT16_MAX)  { return INT16_MAX; }
  if (value < -INT16_MAX) { return (int16_t)-INT16_MAX; }
  return (int16_t)value;
}

static uint32_t SpeedCapture_DataLineCount(const SpeedCapture *capture)
{
  return (capture->sample_count + SPEED_CAPTURE_SAMPLES_PER_LINE - 1u) / SPEED_CAPTURE_SAMPLES_PER_LINE;
}

static void SpeedCapture_Store(SpeedCapture *capture, const SpeedCaptureInput *input)
{
  SpeedCaptureSample *sample = &capture->samples[capture->sample_count];
  if (input->read_ok)
  {
    sample->delta_counts = SpeedCapture_ToInt16((int32_t)((uint32_t)input->count - (uint32_t)capture->last_good_count));
    capture->last_good_count = input->count;
  }
  else
  {
    /* The count is stale: no delta now; the next good sample's delta,
     * taken against last_good_count, covers the gap. */
    sample->delta_counts = SPEED_CAPTURE_MISSING;
  }
  sample->duty_ticks = SpeedCapture_ToInt16(input->applied_ticks);
  capture->sample_count++;
  if (capture->sample_count >= capture->capacity)
  {
    capture->state = SPEED_CAPTURE_SENDING;
    capture->line_index = 0u;
    capture->item_index = 0u;
  }
}

void SpeedCapture_Init(SpeedCapture *capture, SpeedCaptureSample *samples, uint32_t capacity)
{
  *capture = (SpeedCapture){0};
  capture->samples = samples;
  capture->capacity = capacity;
  capture->state = SPEED_CAPTURE_IDLE;
}

bool SpeedCapture_Command(SpeedCapture *capture, int32_t value)
{
  if (value == 0)
  {
    capture->state = SPEED_CAPTURE_IDLE;
    return true;
  }
  if (value < 1 || value > (int32_t)SPEED_CAPTURE_MAX_PERIOD_MS)
  {
    return false;
  }
  if (capture->state == SPEED_CAPTURE_RECORDING || capture->state == SPEED_CAPTURE_SENDING)
  {
    return false;  /* busy: cancel with "c 0" first */
  }
  capture->state = SPEED_CAPTURE_ARMED;
  capture->period_ms = (uint32_t)value;
  return true;
}

void SpeedCapture_Tick(SpeedCapture *capture, const SpeedCaptureInput *input)
{
  bool target_changed = capture->have_last_target && input->target_ticks != capture->last_target_ticks;
  int32_t previous_target = capture->last_target_ticks;
  capture->last_target_ticks = input->target_ticks;
  capture->have_last_target = true;

  if (capture->state == SPEED_CAPTURE_ARMED && target_changed && capture->capacity > 0u)
  {
    capture->state = SPEED_CAPTURE_RECORDING;
    capture->capture_id++;
    capture->t0_ms = input->now_ms;
    /* On a failed read, count is the last good count anyway (it isn't
     * updated without a good read), and raw_angle the last good angle. */
    capture->count0 = input->count;
    capture->last_good_count = input->count;
    capture->raw0 = input->raw_angle;
    capture->vbat_mv = input->vbat_mv;
    capture->previous_target_ticks = previous_target;
    capture->trigger_target_ticks = input->target_ticks;
    capture->sample_count = 0u;
    capture->ms_in_period = 0u;
    SpeedCapture_Store(capture, input);  /* sample 0: the trigger ms itself */
    return;
  }

  if (capture->state == SPEED_CAPTURE_RECORDING)
  {
    capture->ms_in_period++;
    if (capture->ms_in_period >= capture->period_ms)
    {
      capture->ms_in_period = 0u;
      SpeedCapture_Store(capture, input);
    }
  }
}

bool SpeedCapture_HasLine(const SpeedCapture *capture)
{
  return capture->state == SPEED_CAPTURE_SENDING;
}

/* Appends prefix + the decimal value as one piece (all or nothing). */
static void SpeedCapture_AppendNumber(TelemetryWriter *writer, const char *prefix, int32_t value)
{
  char text[16];
  size_t length = 0u;
  while (prefix[length] != '\0')
  {
    text[length] = prefix[length];
    length++;
  }
  if (value == SPEED_CAPTURE_MISSING)
  {
    text[length++] = 'x';
  }
  else
  {
    uint32_t magnitude = (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    if (value < 0) { text[length++] = '-'; }
    char digits[10];
    size_t digit_count = 0u;
    do
    {
      digits[digit_count++] = (char)('0' + (magnitude % 10u));
      magnitude /= 10u;
    } while (magnitude != 0u);
    while (digit_count > 0u) { text[length++] = digits[--digit_count]; }
  }
  text[length] = '\0';
  TelemetryWriter_AppendText(writer, text);
}

/* Appends piece number item of the header. False once there are no more. */
static bool SpeedCapture_HeaderItem(const SpeedCapture *capture, TelemetryWriter *writer, uint32_t item)
{
  switch (item)
  {
    case 0u:  TelemetryWriter_AppendText(writer, "caph"); return true;
    case 1u:  TelemetryWriter_AppendUnsignedField(writer, "id", capture->capture_id); return true;
    case 2u:  TelemetryWriter_AppendUnsignedField(writer, "per", capture->period_ms); return true;
    case 3u:  TelemetryWriter_AppendUnsignedField(writer, "n", capture->sample_count); return true;
    case 4u:  TelemetryWriter_AppendUnsignedField(writer, "t0", capture->t0_ms); return true;
    case 5u:  TelemetryWriter_AppendSignedField(writer, "count0", capture->count0); return true;
    case 6u:  TelemetryWriter_AppendUnsignedField(writer, "raw0", capture->raw0); return true;
    case 7u:  TelemetryWriter_AppendUnsignedField(writer, "vbat", capture->vbat_mv); return true;
    case 8u:  TelemetryWriter_AppendSignedField(writer, "prev", capture->previous_target_ticks); return true;
    case 9u:  TelemetryWriter_AppendSignedField(writer, "target", capture->trigger_target_ticks); return true;
    case 10u: TelemetryWriter_AppendText(writer, "\r\n"); return true;
    default:  return false;
  }
}

/* Appends piece number item of data line line_index (>= 1). */
static bool SpeedCapture_DataItem(const SpeedCapture *capture, TelemetryWriter *writer, uint32_t item)
{
  uint32_t first = (capture->line_index - 1u) * SPEED_CAPTURE_SAMPLES_PER_LINE;
  uint32_t count = capture->sample_count - first;
  if (count > SPEED_CAPTURE_SAMPLES_PER_LINE) { count = SPEED_CAPTURE_SAMPLES_PER_LINE; }

  if (item == 0u) { TelemetryWriter_AppendText(writer, "cap"); return true; }
  if (item == 1u) { TelemetryWriter_AppendUnsignedField(writer, "id", capture->capture_id); return true; }
  if (item == 2u) { TelemetryWriter_AppendUnsignedField(writer, "i", first); return true; }
  item -= 3u;
  if (item < count)
  {
    SpeedCapture_AppendNumber(writer, (item == 0u) ? " d=" : ",", capture->samples[first + item].delta_counts);
    return true;
  }
  item -= count;
  if (item < count)
  {
    SpeedCapture_AppendNumber(writer, (item == 0u) ? " u=" : ",", capture->samples[first + item].duty_ticks);
    return true;
  }
  if (item == count) { TelemetryWriter_AppendText(writer, "\r\n"); return true; }
  return false;
}

bool SpeedCapture_BuildLineStep(SpeedCapture *capture, TelemetryWriter *writer, uint32_t max_items)
{
  if (capture->state != SPEED_CAPTURE_SENDING)
  {
    return true;  /* nothing to build */
  }
  for (uint32_t done = 0u; done < max_items; done++)
  {
    bool appended = (capture->line_index == 0u) ? SpeedCapture_HeaderItem(capture, writer, capture->item_index)
                                                 : SpeedCapture_DataItem(capture, writer, capture->item_index);
    if (!appended)
    {
      return true;
    }
    capture->item_index++;
  }
  /* Complete if the next piece doesn't exist. */
  uint32_t total_items = (capture->line_index == 0u) ? HEADER_ITEMS : 0u;
  if (capture->line_index != 0u)
  {
    uint32_t first = (capture->line_index - 1u) * SPEED_CAPTURE_SAMPLES_PER_LINE;
    uint32_t count = capture->sample_count - first;
    if (count > SPEED_CAPTURE_SAMPLES_PER_LINE) { count = SPEED_CAPTURE_SAMPLES_PER_LINE; }
    total_items = 3u + 2u * count + 1u;
  }
  return capture->item_index >= total_items;
}

void SpeedCapture_LineSent(SpeedCapture *capture)
{
  if (capture->state != SPEED_CAPTURE_SENDING)
  {
    return;
  }
  capture->line_index++;
  capture->item_index = 0u;
  if (capture->line_index > SpeedCapture_DataLineCount(capture))
  {
    capture->state = SPEED_CAPTURE_IDLE;
  }
}
