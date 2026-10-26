/**
  ******************************************************************************
  * @file    command_line.c
  * @brief   Command line collection and parsing (see the header).
  ******************************************************************************
  */
#include "command_line.h"

static void CommandLine_Reset(CommandLine *line)
{
  line->length = 0u;
  line->overlong = false;
  line->discarding = false;
}

void CommandLine_Init(CommandLine *line)
{
  CommandLine_Reset(line);
}

void CommandLine_DiscardCurrent(CommandLine *line)
{
  line->discarding = true;
}

bool Command_Parse(const char *text, size_t length, Command *command_out)
{
  /* Shortest valid command is "m 0". */
  if (length < 3u || text[0] < 'a' || text[0] > 'z' || text[1] != ' ')
  {
    return false;
  }
  size_t position = 2u;
  bool is_negative = false;
  if (text[position] == '-')
  {
    is_negative = true;
    position++;
  }
  /* 1 to 3 digits and nothing after them. Checked before the loop so the
   * value can't overflow, however long the line. */
  size_t digit_count = length - position;
  if (digit_count < 1u || digit_count > 3u)
  {
    return false;
  }
  int32_t value = 0;
  for (; position < length; position++)
  {
    char character = text[position];
    if (character < '0' || character > '9')
    {
      return false;
    }
    value = value * 10 + (character - '0');
  }
  if (value > COMMAND_MAX_VALUE)
  {
    return false;
  }
  command_out->letter = text[0];
  command_out->value = is_negative ? -value : value;
  return true;
}

CommandLineResult CommandLine_FeedByte(CommandLine *line, uint8_t byte, Command *command_out)
{
  if (byte == (uint8_t)'\n')
  {
    CommandLineResult result;
    size_t length = line->length;
    if (line->discarding || line->overlong)
    {
      result = COMMAND_LINE_REJECTED;
    }
    else
    {
      if (length > 0u && line->text[length - 1u] == '\r')
      {
        length--;  /* tolerate "\r\n" (one '\r' only) */
      }
      if (length == 0u)
      {
        result = COMMAND_LINE_EMPTY;
      }
      else
      {
        result = Command_Parse(line->text, length, command_out) ? COMMAND_LINE_VALID : COMMAND_LINE_REJECTED;
      }
    }
    CommandLine_Reset(line);
    return result;
  }

  /* A line already known to be bad: just wait for its '\n'. */
  if (line->discarding || line->overlong)
  {
    return COMMAND_LINE_INCOMPLETE;
  }
  if (line->length >= COMMAND_MAX_LINE_LENGTH)
  {
    line->overlong = true;
    return COMMAND_LINE_INCOMPLETE;
  }
  line->text[line->length] = (char)byte;
  line->length++;
  return COMMAND_LINE_INCOMPLETE;
}
