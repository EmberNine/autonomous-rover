/**
  ******************************************************************************
  * @file    telemetry_format.c
  * @brief   Builds the telemetry text line without snprintf (see the header).
  ******************************************************************************
  */
#include "telemetry_format.h"

#include <string.h>

/* "-2147483648" is the longest value: 10 digits plus a sign. */
#define TELEMETRY_MAX_VALUE_CHARS  11

/* Writes the decimal digits of value into digits[] and returns how many.
 * Digits come out least significant first, so they're filled from the end
 * of the array backwards; the result starts at digits + (size - count). */
static size_t TelemetryFormat_UnsignedDigits(uint32_t value, char *digits, size_t digits_size)
{
  size_t count = 0;
  do
  {
    digits[digits_size - 1 - count] = (char)('0' + (value % 10u));
    value /= 10u;
    count++;
  } while (value != 0u);
  return count;
}

/* Copies length bytes and keeps the buffer NUL-terminated. Only called after
 * the caller has checked that they fit. */
static void TelemetryWriter_Put(TelemetryWriter *writer, const char *text, size_t length)
{
  memcpy(writer->buffer + writer->length, text, length);
  writer->length += length;
  writer->buffer[writer->length] = '\0';
}

/* True if length more characters still leave room for the NUL. Marks the
 * writer truncated otherwise. */
static bool TelemetryWriter_Fits(TelemetryWriter *writer, size_t length)
{
  if (writer->truncated)
  {
    return false;
  }
  /* length <= capacity - 1 - writer->length, written so it can't underflow. */
  if (writer->capacity == 0u || length >= writer->capacity - writer->length)
  {
    writer->truncated = true;
    return false;
  }
  return true;
}

/* Shared tail of both field functions: [space] key '=' [-] digits. */
static void TelemetryWriter_AppendField(TelemetryWriter *writer, const char *key,
                                        bool is_negative, uint32_t magnitude)
{
  char digits[TELEMETRY_MAX_VALUE_CHARS];
  size_t digit_count = TelemetryFormat_UnsignedDigits(magnitude, digits, sizeof(digits));

  size_t separator_length = (writer->length > 0u) ? 1u : 0u;
  size_t key_length = strlen(key);
  size_t sign_length = is_negative ? 1u : 0u;
  size_t total_length = separator_length + key_length + 1u + sign_length + digit_count;

  if (!TelemetryWriter_Fits(writer, total_length))
  {
    return;
  }
  if (separator_length != 0u)
  {
    TelemetryWriter_Put(writer, " ", 1u);
  }
  TelemetryWriter_Put(writer, key, key_length);
  TelemetryWriter_Put(writer, "=", 1u);
  if (is_negative)
  {
    TelemetryWriter_Put(writer, "-", 1u);
  }
  TelemetryWriter_Put(writer, digits + sizeof(digits) - digit_count, digit_count);
}

void TelemetryWriter_Init(TelemetryWriter *writer, char *buffer, size_t capacity)
{
  writer->buffer = buffer;
  writer->capacity = capacity;
  writer->length = 0u;
  writer->truncated = false;
  if (capacity > 0u)
  {
    buffer[0] = '\0';
  }
}

void TelemetryWriter_AppendText(TelemetryWriter *writer, const char *text)
{
  size_t text_length = strlen(text);
  if (TelemetryWriter_Fits(writer, text_length))
  {
    TelemetryWriter_Put(writer, text, text_length);
  }
}

void TelemetryWriter_AppendUnsignedField(TelemetryWriter *writer, const char *key, uint32_t value)
{
  TelemetryWriter_AppendField(writer, key, false, value);
}

void TelemetryWriter_AppendSignedField(TelemetryWriter *writer, const char *key, int32_t value)
{
  /* Negating INT32_MIN as an int32 overflows (undefined behaviour). Doing it
   * in uint32 is well defined: 0 - 0x80000000 wraps to 0x80000000, which is
   * exactly the magnitude 2147483648. */
  bool is_negative = (value < 0);
  uint32_t magnitude = is_negative ? (0u - (uint32_t)value) : (uint32_t)value;
  TelemetryWriter_AppendField(writer, key, is_negative, magnitude);
}
