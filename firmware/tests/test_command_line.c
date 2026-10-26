/*
 * Host-side test for Core/Src/command_line.c. Not part of the firmware build
 * or CI. Compiles it with the PC's C compiler.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc tests/test_command_line.c Core/Src/command_line.c -o /tmp/test_command_line && /tmp/test_command_line
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "command_line.h"

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

/* Feeds a whole string. Returns the result for the LAST byte, and how many
 * VALID / REJECTED / EMPTY lines were completed along the way. */
typedef struct
{
  CommandLineResult last_result;
  int valid_count;
  int rejected_count;
  int empty_count;
  Command last_command;
} FeedSummary;

static FeedSummary feed_bytes(CommandLine *line, const char *bytes, size_t length)
{
  FeedSummary summary = { COMMAND_LINE_INCOMPLETE, 0, 0, 0, { 0, 0 } };
  for (size_t i = 0; i < length; i++)
  {
    Command command = { '?', 9999 };
    summary.last_result = CommandLine_FeedByte(line, (uint8_t)bytes[i], &command);
    if (summary.last_result == COMMAND_LINE_VALID) { summary.valid_count++; summary.last_command = command; }
    if (summary.last_result == COMMAND_LINE_REJECTED) { summary.rejected_count++; }
    if (summary.last_result == COMMAND_LINE_EMPTY) { summary.empty_count++; }
  }
  return summary;
}

static FeedSummary feed_text(CommandLine *line, const char *text)
{
  return feed_bytes(line, text, strlen(text));
}

static void test_accepted(void)
{
  static const struct { const char *text; char letter; int32_t value; } accepted[] = {
    { "m 0\n", 'm', 0 }, { "m 20\n", 'm', 20 }, { "m -15\n", 'm', -15 }, { "m -15\r\n", 'm', -15 },
    { "m 100\n", 'm', 100 }, { "m -100\n", 'm', -100 }, { "m -0\n", 'm', 0 }, { "m 7\r\n", 'm', 7 },
    { "m 007\n", 'm', 7 }, { "m 30\n", 'm', 30 }, { "m 99\n", 'm', 99 }, { "m 1\n", 'm', 1 },
    /* Steering uses the same grammar. */
    { "s 0\n", 's', 0 }, { "s 100\n", 's', 100 }, { "s -100\r\n", 's', -100 }, { "s -35\n", 's', -35 },
    /* Any lowercase letter is a well-formed line; the dispatcher decides
     * whether the letter means anything. */
    { "h 30\n", 'h', 30 }, { "x 20\n", 'x', 20 }, { "a 1\n", 'a', 1 }, { "z -9\n", 'z', -9 },
  };
  for (size_t i = 0; i < sizeof(accepted) / sizeof(accepted[0]); i++)
  {
    CommandLine line;
    CommandLine_Init(&line);
    FeedSummary summary = feed_text(&line, accepted[i].text);
    CHECK(summary.last_result == COMMAND_LINE_VALID && summary.valid_count == 1 &&
          summary.last_command.letter == accepted[i].letter && summary.last_command.value == accepted[i].value,
          "'%s' should be valid %c %d, got result %d: %c %d", accepted[i].text, accepted[i].letter,
          (int)accepted[i].value, (int)summary.last_result, summary.last_command.letter,
          (int)summary.last_command.value);
  }
}

static void test_rejected(void)
{
  static const char *const rejected[] = {
    "m\n", "m \n", "m -\n", "m --5\n", "M 20\n", "m  20\n", "m +20\n", "m 101\n", "m -101\n", "m 1000\n",
    "m 0000\n", "m 20 \n", " m 20\n", "m20\n", "m 2a\n", "m 20\r\r\n", "m\t20\n", "mm 20\n",
    "m 2-0\n", "m 20x\n", "\r\r\n", "m 999\n", "s\n", "m -1000\n",
    /* Letter rules: one lowercase ASCII letter, nothing else. */
    "S 20\n", "s 101\n", "s -101\n", "s  5\n", "ss 5\n", "1 20\n", "- 20\n", "` 5\n", "{ 5\n", "@ 5\n",
    "[ 5\n", "s+5\n",
  };
  for (size_t i = 0; i < sizeof(rejected) / sizeof(rejected[0]); i++)
  {
    CommandLine line;
    CommandLine_Init(&line);
    FeedSummary summary = feed_text(&line, rejected[i]);
    CHECK(summary.last_result == COMMAND_LINE_REJECTED && summary.valid_count == 0,
          "line %zu ('%s') should be rejected, got %d", i, rejected[i], (int)summary.last_result);
  }

  /* Bytes that aren't text, including a NUL inside a command. */
  CommandLine line;
  CommandLine_Init(&line);
  static const char with_nul[] = { 'm', ' ', '2', '\0', '\n' };
  FeedSummary summary = feed_bytes(&line, with_nul, sizeof(with_nul));
  CHECK(summary.last_result == COMMAND_LINE_REJECTED, "embedded NUL should be rejected");
  static const char high_byte[] = { 'm', ' ', (char)0xB2, '\n' };
  summary = feed_bytes(&line, high_byte, sizeof(high_byte));
  CHECK(summary.last_result == COMMAND_LINE_REJECTED, "byte 0xB2 should be rejected");
  static const char high_letter[] = { (char)0xED, ' ', '5', '\n' };  /* 'm' | 0x80 */
  summary = feed_bytes(&line, high_letter, sizeof(high_letter));
  CHECK(summary.last_result == COMMAND_LINE_REJECTED, "letter byte 0xED should be rejected");
}

static void test_empty_lines(void)
{
  CommandLine line;
  CommandLine_Init(&line);
  FeedSummary summary = feed_text(&line, "\n\r\n\n");
  CHECK(summary.empty_count == 3 && summary.rejected_count == 0 && summary.valid_count == 0,
        "empty lines: empty=%d rejected=%d valid=%d", summary.empty_count, summary.rejected_count,
        summary.valid_count);
}

static void test_length_limit(void)
{
  /* 16 bytes before '\n' are kept (and here rejected as text), 17 make the
   * line overlong. Either way the next line is fine. */
  CommandLine line;
  CommandLine_Init(&line);
  FeedSummary summary = feed_text(&line, "m 2000000000000\n");  /* 15 bytes */
  CHECK(summary.rejected_count == 1, "15-byte line should be rejected as a bad command");
  summary = feed_text(&line, "m 20000000000000\n");               /* exactly 16 */
  CHECK(summary.rejected_count == 1 && !line.overlong, "16-byte line should be rejected, not overlong");
  summary = feed_text(&line, "m 200000000000000\n");              /* 17: overlong */
  CHECK(summary.rejected_count == 1, "17-byte line should be rejected");
  summary = feed_text(&line, "m 20\n");
  CHECK(summary.valid_count == 1 && summary.last_command.value == 20, "line after an overlong one should be valid");

  /* A very long line (1000 bytes) is one rejection, not several. */
  char long_line[1002];
  memset(long_line, '5', 1000);
  long_line[1000] = '\n';
  long_line[1001] = '\0';
  summary = feed_text(&line, long_line);
  CHECK(summary.rejected_count == 1 && summary.valid_count == 0, "1000-byte line: rejected=%d",
        summary.rejected_count);
  summary = feed_text(&line, "s -5\r\n");
  CHECK(summary.valid_count == 1 && summary.last_command.letter == 's' && summary.last_command.value == -5,
        "line after a 1000-byte line should be valid");
}

static void test_discard(void)
{
  /* A receive error in the middle of a line rejects that line only. */
  CommandLine line;
  CommandLine_Init(&line);
  FeedSummary summary = feed_text(&line, "m 2");
  CommandLine_DiscardCurrent(&line);
  summary = feed_text(&line, "0\n");
  CHECK(summary.rejected_count == 1 && summary.valid_count == 0, "discarded line should be rejected");
  summary = feed_text(&line, "m 25\n");
  CHECK(summary.valid_count == 1 && summary.last_command.value == 25, "line after a discarded one should be valid");

  /* Error between lines: the next line is the one that was hit. */
  CommandLine_DiscardCurrent(&line);
  summary = feed_text(&line, "s 30\nm 10\n");
  CHECK(summary.rejected_count == 1 && summary.valid_count == 1 && summary.last_command.letter == 'm' &&
        summary.last_command.value == 10,
        "error between lines: rejected=%d valid=%d", summary.rejected_count, summary.valid_count);
}

static void test_parse_bounds(void)
{
  /* Command_Parse never reads past length (ASan would catch it). */
  Command command = { 0, 0 };
  char exact[3] = { 's', ' ', '5' };  /* not NUL-terminated on purpose */
  CHECK(Command_Parse(exact, sizeof(exact), &command) && command.letter == 's' && command.value == 5,
        "unterminated 's 5'");
  char short_text[2] = { 'm', ' ' };
  CHECK(!Command_Parse(short_text, sizeof(short_text), &command), "'m ' should be rejected");
  char minus_only[3] = { 'm', ' ', '-' };
  CHECK(!Command_Parse(minus_only, sizeof(minus_only), &command), "'m -' should be rejected");

  /* A rejected line leaves the output untouched. */
  command.letter = 'q';
  command.value = 42;
  char bad[4] = { 's', ' ', '9', 'x' };
  CHECK(!Command_Parse(bad, sizeof(bad), &command) && command.letter == 'q' && command.value == 42,
        "rejected parse must not write the output");
}

int main(void)
{
  test_accepted();
  test_rejected();
  test_empty_lines();
  test_length_limit();
  test_discard();
  test_parse_bounds();

  printf("%d checks, %d failed\n", check_count, failure_count);
  if (failure_count != 0)
  {
    return EXIT_FAILURE;
  }
  printf("ALL PASSED\n");
  return EXIT_SUCCESS;
}
