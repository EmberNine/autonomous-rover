/**
  ******************************************************************************
  * @file    telemetry_format.h
  * @brief   Builds the telemetry text line without snprintf: appends literal
  *          text and key=value fields with integer values in plain decimal.
  *
  * newlib's snprintf takes roughly half of the 1 ms loop slot for one half of
  * the line, because it parses the format string and goes through its general
  * conversion code for every field. The telemetry line only ever needs signed
  * and unsigned 32-bit integers in plain decimal (no padding, no hex, no
  * floats), so this module does just that.
  *
  * The output is byte-for-byte what snprintf("%lu") / snprintf("%ld") would
  * produce; firmware/tests/test_telemetry_format.c checks that on a PC.
  *
  * Pure C with no HAL or MCU dependencies, so the same file compiles on a PC.
  * No globals: the writer state lives in a struct the caller owns.
  ******************************************************************************
  */
#ifndef TELEMETRY_FORMAT_H
#define TELEMETRY_FORMAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Appends text to a caller-owned buffer, one piece at a time.
 *
 * Bounds: at most capacity - 1 characters are ever written, and the buffer is
 * always NUL-terminated after each call (when capacity > 0).
 *
 * Truncation: a piece (a text, or a whole " key=value") is either written
 * completely or not at all. The first piece that doesn't fit sets truncated,
 * and every append after that is ignored, so the buffer only ever holds whole
 * pieces. The caller checks truncated once, after the last append. */
typedef struct
{
  char   *buffer;
  size_t  capacity;   /* size of buffer in bytes, including the NUL */
  size_t  length;     /* characters written so far, excluding the NUL */
  bool    truncated;  /* true once something didn't fit; sticky */
} TelemetryWriter;

/* Starts an empty line in buffer. */
void TelemetryWriter_Init(TelemetryWriter *writer, char *buffer, size_t capacity);

/* Appends text exactly as given (for example the "\r\n" line ending). */
void TelemetryWriter_AppendText(TelemetryWriter *writer, const char *text);

/* Append "key=value". If the line already has text, a single space goes
 * before the key, so fields come out as "a=1 b=2" with no special case for
 * the first one. Values are plain decimal, with a leading '-' for negatives. */
void TelemetryWriter_AppendUnsignedField(TelemetryWriter *writer, const char *key, uint32_t value);
void TelemetryWriter_AppendSignedField(TelemetryWriter *writer, const char *key, int32_t value);

#endif /* TELEMETRY_FORMAT_H */
