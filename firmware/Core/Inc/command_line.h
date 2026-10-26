/**
  ******************************************************************************
  * @file    command_line.h
  * @brief   Collects received bytes into lines and checks each line against
  *          the one command grammar shared by every command letter. What a
  *          letter means is decided elsewhere (command_dispatch.c).
  *
  * Grammar (from the PC, over the ST-LINK virtual COM port):
  *
  *   letter SPACE ["-"] 1 to 3 digits ["\r"] "\n"     for example "m 20\n", "s -15\r\n"
  *
  *   - letter is one lowercase ASCII letter, a..z
  *   - the value is a signed integer, -100..100 ("m -0" is 0)
  *   - anything else is rejected as a whole line: uppercase or no letter, no
  *     space or two spaces, "+", |value| > 100, extra characters, a line
  *     longer than COMMAND_MAX_LINE_LENGTH, or a line hit by a UART receive
  *     error or a dropped byte
  *   - an empty line ("\n" or "\r\n") is ignored and not counted
  *
  * A line that passes is only well-formed: "x 20" is a valid line with an
  * unknown letter, and command_dispatch.c rejects it.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_command_line.c. No globals: state lives in a struct
  * the caller owns.
  ******************************************************************************
  */
#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Longest line kept, counting a '\r' but not the '\n'. The longest valid
 * command, "m -100\r", is 7. Longer lines are rejected as a whole. */
#define COMMAND_MAX_LINE_LENGTH  16
/* |value| above this is not a valid command: rejected rather than clamped. */
#define COMMAND_MAX_VALUE        100

typedef enum
{
  COMMAND_LINE_INCOMPLETE = 0,  /* no '\n' yet */
  COMMAND_LINE_VALID,           /* a well-formed command; *command_out is set */
  COMMAND_LINE_REJECTED,        /* a complete line that is not a well-formed command */
  COMMAND_LINE_EMPTY            /* "\n" or "\r\n" alone: ignored, not counted */
} CommandLineResult;

/* One well-formed command line. */
typedef struct
{
  char    letter;  /* 'a'..'z' */
  int32_t value;   /* -COMMAND_MAX_VALUE..COMMAND_MAX_VALUE */
} Command;

/* Collects received bytes into one line. */
typedef struct
{
  char   text[COMMAND_MAX_LINE_LENGTH];
  size_t length;
  bool   overlong;    /* more than COMMAND_MAX_LINE_LENGTH bytes: reject at '\n' */
  bool   discarding;  /* a receive error hit this line: reject at '\n' */
} CommandLine;

void CommandLine_Init(CommandLine *line);

/* Adds one received byte. Returns COMMAND_LINE_INCOMPLETE until a '\n'
 * arrives, then the verdict for the whole line, and starts a new line. */
CommandLineResult CommandLine_FeedByte(CommandLine *line, uint8_t byte, Command *command_out);

/* A byte of the current line was lost or corrupted (UART error, full ring):
 * everything up to and including the next '\n' is rejected. */
void CommandLine_DiscardCurrent(CommandLine *line);

/* Checks one complete line (without its '\n' and '\r'). True and
 * *command_out set if it is well-formed. Exposed for the host test. */
bool Command_Parse(const char *text, size_t length, Command *command_out);

#endif /* COMMAND_LINE_H */
