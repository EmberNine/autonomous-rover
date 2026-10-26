/*
 * Host-side test for Core/Src/link_codec.c: CRC-16/CCITT-FALSE, COBS and
 * whole frames, against the shared vectors in tests/link_vectors.txt (which
 * tools/test_rover_link.py checks too) and against bit-by-bit / exhaustive
 * references. Not part of the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC; the vector file is read
 * relative to firmware/):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_link_codec.c Core/Src/link_codec.c -o /tmp/test_link_codec && /tmp/test_link_codec
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link_codec.h"

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

#define VECTOR_FILE  "tests/link_vectors.txt"
#define MAX_BYTES    512

static uint32_t random_state = 12345u;

static uint32_t next_random(void)
{
  /* xorshift32: deterministic, so a failure is reproducible. */
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

/* "-" is empty. Returns the byte count, or -1 on a malformed hex string. */
static int parse_hex(const char *text, uint8_t *out, size_t capacity)
{
  if (strcmp(text, "-") == 0)
  {
    return 0;
  }
  size_t length = strlen(text);
  if (length % 2u != 0u || length / 2u > capacity)
  {
    return -1;
  }
  for (size_t i = 0u; i < length / 2u; i++)
  {
    unsigned int value;
    if (sscanf(&text[2u * i], "%2x", &value) != 1)
    {
      return -1;
    }
    out[i] = (uint8_t)value;
  }
  return (int)(length / 2u);
}

static uint16_t crc_bitwise(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0u; i < length; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (int bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static const char *status_name(LinkFrameStatus status)
{
  switch (status)
  {
    case LINK_FRAME_OK:          return "OK";
    case LINK_FRAME_COBS_ERROR:  return "COBS_ERROR";
    case LINK_FRAME_TOO_SHORT:   return "TOO_SHORT";
    case LINK_FRAME_TOO_LONG:    return "TOO_LONG";
    case LINK_FRAME_CRC_ERROR:   return "CRC_ERROR";
    default:                     return "?";
  }
}

/* Splits wire bytes at every 0x00 and decodes each non-empty piece, as a
 * receiver would. Returns how many pieces decoded as a valid frame. */
static int frames_accepted_in(const uint8_t *wire, size_t length)
{
  int accepted = 0;
  size_t start = 0u;
  for (size_t i = 0u; i <= length; i++)
  {
    if (i == length || wire[i] == 0u)
    {
      if (i > start)
      {
        LinkFrame frame;
        if (LinkFrame_Decode(&wire[start], i - start, &frame) == LINK_FRAME_OK)
        {
          accepted++;
        }
      }
      start = i + 1u;
    }
  }
  return accepted;
}

static void test_vectors(void)
{
  FILE *file = fopen(VECTOR_FILE, "r");
  CHECK(file != NULL, "open %s (run from firmware/)", VECTOR_FILE);
  if (file == NULL)
  {
    return;
  }
  char line[2048];
  int counts[4] = {0, 0, 0, 0};  /* crc, cobs, frame, bad */
  while (fgets(line, sizeof(line), file) != NULL)
  {
    char kind[16], a[1100], b[1100], c[1100], d[1100];
    if (line[0] == '#' || line[0] == '\n')
    {
      continue;
    }
    uint8_t x[MAX_BYTES], y[MAX_BYTES], z[MAX_BYTES], out[MAX_BYTES];
    int fields = sscanf(line, "%15s %1099s %1099s %1099s %1099s", kind, a, b, c, d);
    if (strcmp(kind, "crc") == 0 && fields == 3)
    {
      int n = parse_hex(a, x, sizeof(x));
      unsigned int expected = 0u;
      CHECK(n >= 0 && sscanf(b, "%x", &expected) == 1, "crc vector parses: %s", line);
      CHECK(LinkCrc16(x, (size_t)n) == expected, "crc %s = %04x, got %04x", a, expected, LinkCrc16(x, (size_t)n));
      counts[0]++;
    }
    else if (strcmp(kind, "cobs") == 0 && fields == 3)
    {
      int n = parse_hex(a, x, sizeof(x));
      int m = parse_hex(b, y, sizeof(y));
      CHECK(n >= 0 && m > 0, "cobs vector parses");
      size_t encoded = LinkCobs_Encode(x, (size_t)n, out, sizeof(out));
      CHECK(encoded == (size_t)m && memcmp(out, y, (size_t)m) == 0, "cobs encode of %d bytes matches the vector", n);
      size_t decoded = 0u;
      CHECK(LinkCobs_Decode(y, (size_t)m, out, sizeof(out), &decoded) && decoded == (size_t)n &&
            memcmp(out, x, (size_t)n) == 0, "cobs decode of %d bytes matches the vector", m);
      counts[1]++;
    }
    else if (strcmp(kind, "frame") == 0 && fields == 5)
    {
      int t = parse_hex(a, x, 1u);
      int s = parse_hex(b, y, 1u);
      uint8_t type = x[0], seq = y[0];
      int n = parse_hex(c, z, sizeof(z));
      int m = parse_hex(d, y, sizeof(y));
      CHECK(t == 1 && s == 1 && n >= 0 && m > 2, "frame vector parses");
      size_t wire = LinkFrame_Encode(type, seq, z, (size_t)n, out, sizeof(out));
      CHECK(wire == (size_t)m && memcmp(out, y, (size_t)m) == 0, "frame %02x/%02x, %d payload bytes: wire matches",
            type, seq, n);
      LinkFrame frame;
      LinkFrameStatus status = LinkFrame_Decode(&y[1], (size_t)m - 2u, &frame);
      CHECK(status == LINK_FRAME_OK && frame.type == type && frame.seq == seq && frame.payload_length == (size_t)n &&
            memcmp(frame.payload, z, (size_t)n) == 0, "frame %02x/%02x decodes back (%s)", type, seq,
            status_name(status));
      counts[2]++;
    }
    else if (strcmp(kind, "bad") == 0 && fields == 3)
    {
      int n = parse_hex(b, x, sizeof(x));
      CHECK(n >= 0, "bad vector parses");
      LinkFrame frame;
      LinkFrameStatus status = LinkFrame_Decode(x, (size_t)n, &frame);
      CHECK(strcmp(status_name(status), a) == 0, "bad frame %s: expected %s, got %s", b, a, status_name(status));
      counts[3]++;
    }
    else
    {
      CHECK(false, "unknown vector line: %s", line);
    }
  }
  fclose(file);
  CHECK(counts[0] == 5 && counts[1] == 12 && counts[2] == 6 && counts[3] == 8,
        "vector counts crc %d cobs %d frame %d bad %d", counts[0], counts[1], counts[2], counts[3]);
}

static void test_crc(void)
{
  CHECK(LinkCrc16((const uint8_t *)"123456789", 9u) == 0x29B1u, "check value 0x29B1");
  CHECK(LinkCrc16(NULL, 0u) == 0xFFFFu, "empty: the initial value");
  int table_mismatches = 0;
  for (int value = 0; value < 256; value++)
  {
    uint8_t byte = (uint8_t)value;
    if (LinkCrc16(&byte, 1u) != crc_bitwise(&byte, 1u)) { table_mismatches++; }
  }
  CHECK(table_mismatches == 0, "every single byte matches the bit-by-bit CRC (%d mismatches)", table_mismatches);
  int random_mismatches = 0;
  int split_mismatches = 0;
  for (int trial = 0; trial < 2000; trial++)
  {
    uint8_t data[200];
    size_t length = next_random() % sizeof(data);
    for (size_t i = 0u; i < length; i++) { data[i] = (uint8_t)next_random(); }
    uint16_t whole = LinkCrc16(data, length);
    if (whole != crc_bitwise(data, length)) { random_mismatches++; }
    size_t split = (length == 0u) ? 0u : next_random() % length;
    if (LinkCrc16_Update(LinkCrc16(data, split), &data[split], length - split) != whole) { split_mismatches++; }
  }
  CHECK(random_mismatches == 0, "2000 random buffers match the bit-by-bit CRC (%d mismatches)", random_mismatches);
  CHECK(split_mismatches == 0, "Update continues a CRC across any split (%d mismatches)", split_mismatches);
}

/* Fills data with one of three patterns: all zero, no zero, or random with ~30 % zeros. */
static void fill_pattern(uint8_t *data, size_t length, int pattern)
{
  for (size_t i = 0u; i < length; i++)
  {
    switch (pattern)
    {
      case 0:  data[i] = 0u; break;
      case 1:  data[i] = (uint8_t)(1u + next_random() % 255u); break;
      default: data[i] = (next_random() % 10u < 3u) ? 0u : (uint8_t)next_random(); break;
    }
  }
}

static void test_cobs_round_trip_and_bounds(void)
{
  int round_trip_failures = 0, zero_failures = 0, length_failures = 0, capacity_failures = 0;
  for (size_t length = 0u; length <= 300u; length++)
  {
    for (int pattern = 0; pattern < 3; pattern++)
    {
      uint8_t data[300], encoded[310], decoded[300];
      fill_pattern(data, length, pattern);
      size_t n = LinkCobs_Encode(data, length, encoded, sizeof(encoded));
      size_t back = 0u;
      if (n == 0u || !LinkCobs_Decode(encoded, n, decoded, sizeof(decoded), &back) || back != length ||
          memcmp(decoded, data, length) != 0)
      {
        round_trip_failures++;
        continue;
      }
      if (memchr(encoded, 0, n) != NULL) { zero_failures++; }
      /* One extra byte up to 254 data bytes; at most one per 254 after that. */
      if ((length <= 254u && n != length + 1u) || n > length + 1u + length / 254u) { length_failures++; }
      /* The exact capacity works, one byte less doesn't (encode and decode). */
      uint8_t tight[310];
      if (LinkCobs_Encode(data, length, tight, n) != n || LinkCobs_Encode(data, length, tight, n - 1u) != 0u)
      {
        capacity_failures++;
      }
      if (length > 0u && LinkCobs_Decode(encoded, n, decoded, length - 1u, &back)) { capacity_failures++; }
    }
  }
  CHECK(round_trip_failures == 0, "COBS round trip, lengths 0..300 x 3 patterns (%d failures)", round_trip_failures);
  CHECK(zero_failures == 0, "COBS output never contains 0x00 (%d failures)", zero_failures);
  CHECK(length_failures == 0, "COBS output length (%d failures)", length_failures);
  CHECK(capacity_failures == 0, "COBS capacity limits exact (%d failures)", capacity_failures);
}

/* Every input that decodes is the canonical encoding of what it decodes to:
 * exhaustive for 1 and 2 bytes, random for 3..40 (below 255 bytes no code
 * byte can be 0xFF in a valid input, so there is no second encoding). */
static void test_cobs_decode_is_strict(void)
{
  int non_canonical = 0, decoded_count = 0;
  uint8_t input[40], decoded[64], again[64];
  for (uint32_t value = 0u; value < 65536u + 256u; value++)
  {
    size_t length = (value < 256u) ? 1u : 2u;
    uint32_t v = (value < 256u) ? value : value - 256u;
    input[0] = (uint8_t)v;
    input[1] = (uint8_t)(v >> 8);
    size_t back = 0u;
    if (LinkCobs_Decode(input, length, decoded, sizeof(decoded), &back))
    {
      decoded_count++;
      size_t n = LinkCobs_Encode(decoded, back, again, sizeof(again));
      if (n != length || memcmp(again, input, length) != 0) { non_canonical++; }
    }
  }
  for (int trial = 0; trial < 200000; trial++)
  {
    size_t length = 3u + next_random() % 38u;
    for (size_t i = 0u; i < length; i++)
    {
      uint32_t dice = next_random() % 8u;
      input[i] = (dice < 4u) ? (uint8_t)(1u + dice) : (dice == 4u) ? 0u : (uint8_t)next_random();
    }
    size_t back = 0u;
    if (LinkCobs_Decode(input, length, decoded, sizeof(decoded), &back))
    {
      decoded_count++;
      size_t n = LinkCobs_Encode(decoded, back, again, sizeof(again));
      if (n != length || memcmp(again, input, length) != 0) { non_canonical++; }
    }
  }
  CHECK(non_canonical == 0 && decoded_count > 1000, "%d inputs decoded, %d not canonical", decoded_count, non_canonical);
  size_t back = 0u;
  CHECK(!LinkCobs_Decode(input, 0u, decoded, sizeof(decoded), &back), "empty input is not COBS");
  static const uint8_t zero_code[] = { 0x02u, 0x11u, 0x00u };
  static const uint8_t past_end[] = { 0x04u, 0x11u, 0x22u };
  static const uint8_t full_block_short[] = { 0xFFu, 0x01u, 0x02u };
  CHECK(!LinkCobs_Decode(zero_code, sizeof(zero_code), decoded, sizeof(decoded), &back), "a 0x00 code is rejected");
  CHECK(!LinkCobs_Decode(past_end, sizeof(past_end), decoded, sizeof(decoded), &back), "a code past the end is rejected");
  CHECK(!LinkCobs_Decode(full_block_short, sizeof(full_block_short), decoded, sizeof(decoded), &back),
        "a 0xFF code without 254 bytes is rejected");
}

static void test_frame_round_trip_and_limits(void)
{
  int failures = 0;
  for (size_t length = 0u; length <= LINK_MAX_PAYLOAD; length++)
  {
    uint8_t payload[LINK_MAX_PAYLOAD], wire[LINK_MAX_WIRE + 8u];
    fill_pattern(payload, length, (int)(length % 3u));
    uint8_t type = (uint8_t)next_random(), seq = (uint8_t)next_random();
    size_t n = LinkFrame_Encode(type, seq, payload, length, wire, sizeof(wire));
    LinkFrame frame;
    if (n != length + 7u || wire[0] != 0u || wire[n - 1u] != 0u || memchr(&wire[1], 0, n - 2u) != NULL ||
        LinkFrame_Decode(&wire[1], n - 2u, &frame) != LINK_FRAME_OK || frame.type != type || frame.seq != seq ||
        frame.payload_length != length || memcmp(frame.payload, payload, length) != 0)
    {
      failures++;
    }
    /* Exactly n bytes of room works; n - 1 doesn't. */
    if (LinkFrame_Encode(type, seq, payload, length, wire, n) != n ||
        LinkFrame_Encode(type, seq, payload, length, wire, n - 1u) != 0u)
    {
      failures++;
    }
  }
  CHECK(failures == 0, "frames with 0..%u payload bytes: wire length payload + 7, delimiters, round trip, room (%d failures)",
        (unsigned)LINK_MAX_PAYLOAD, failures);
  CHECK(LINK_MAX_WIRE == 127u, "the largest frame is 127 bytes on the wire");

  uint8_t payload[LINK_MAX_PAYLOAD + 1u] = {0};
  uint8_t wire[LINK_MAX_WIRE + 8u];
  CHECK(LinkFrame_Encode(1u, 0u, payload, LINK_MAX_PAYLOAD + 1u, wire, sizeof(wire)) == 0u,
        "a payload above LINK_MAX_PAYLOAD is refused");
  /* Every type and seq value round-trips. */
  int field_failures = 0;
  for (int value = 0; value < 256; value++)
  {
    LinkFrame frame;
    size_t n = LinkFrame_Encode((uint8_t)value, (uint8_t)(255 - value), NULL, 0u, wire, sizeof(wire));
    if (n != 7u || LinkFrame_Decode(&wire[1], n - 2u, &frame) != LINK_FRAME_OK || frame.type != (uint8_t)value ||
        frame.seq != (uint8_t)(255 - value) || frame.payload_length != 0u)
    {
      field_failures++;
    }
  }
  CHECK(field_failures == 0, "all 256 type and seq values round-trip (%d failures)", field_failures);

  /* Bodies just around the limits. */
  uint8_t cobs[LINK_MAX_COBS + 2u];
  memset(cobs, 0x01, sizeof(cobs));
  LinkFrame frame;
  CHECK(LinkFrame_Decode(cobs, LINK_MAX_COBS + 1u, &frame) == LINK_FRAME_TOO_LONG, "126 COBS bytes: TOO_LONG");
  CHECK(LinkFrame_Decode(cobs, LINK_MAX_COBS, &frame) == LINK_FRAME_CRC_ERROR,
        "125 COBS bytes (a 124-byte body of zeros): checked, CRC error");
  /* n codes 0x01 decode to n - 1 zeros. */
  CHECK(LinkFrame_Decode(cobs, 5u, &frame) == LINK_FRAME_CRC_ERROR, "a 4-byte body is long enough to be checked");
  CHECK(LinkFrame_Decode(cobs, 4u, &frame) == LINK_FRAME_TOO_SHORT, "a 3-byte body is TOO_SHORT");
}

/* Corruption: no single bit flip anywhere in a frame (delimiters included),
 * and no pair of flips in a DRIVE-sized frame, yields a frame that decodes,
 * however the flips split it at 0x00 bytes. */
static void test_bit_flips(void)
{
  int single_accepts = 0, single_cases = 0;
  for (int trial = 0; trial < 300; trial++)
  {
    size_t length = (trial < 121) ? (size_t)trial : next_random() % (LINK_MAX_PAYLOAD + 1u);
    uint8_t payload[LINK_MAX_PAYLOAD], wire[LINK_MAX_WIRE];
    fill_pattern(payload, length, trial % 3);
    size_t n = LinkFrame_Encode((uint8_t)next_random(), (uint8_t)next_random(), payload, length, wire, sizeof(wire));
    for (size_t bit = 0u; bit < n * 8u; bit++)
    {
      wire[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
      single_accepts += frames_accepted_in(wire, n);
      single_cases++;
      wire[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
    }
  }
  CHECK(single_accepts == 0, "single bit flips: %d of %d corrupted frames accepted", single_accepts, single_cases);

  int double_accepts = 0, double_cases = 0;
  for (int trial = 0; trial < 20; trial++)
  {
    uint8_t payload[4], wire[16];
    fill_pattern(payload, sizeof(payload), trial % 3);
    size_t n = LinkFrame_Encode(0x10u, (uint8_t)trial, payload, sizeof(payload), wire, sizeof(wire));
    for (size_t first = 0u; first < n * 8u; first++)
    {
      for (size_t second = first + 1u; second < n * 8u; second++)
      {
        wire[first / 8u] ^= (uint8_t)(1u << (first % 8u));
        wire[second / 8u] ^= (uint8_t)(1u << (second % 8u));
        double_accepts += frames_accepted_in(wire, n);
        double_cases++;
        wire[first / 8u] ^= (uint8_t)(1u << (first % 8u));
        wire[second / 8u] ^= (uint8_t)(1u << (second % 8u));
      }
    }
  }
  CHECK(double_accepts == 0, "double bit flips in 11-byte frames: %d of %d accepted", double_accepts, double_cases);
  printf("  bit flips: %d single, %d double, all rejected\n", single_cases, double_cases);
}

int main(void)
{
  test_vectors();
  test_crc();
  test_cobs_round_trip_and_bounds();
  test_cobs_decode_is_strict();
  test_frame_round_trip_and_limits();
  test_bit_flips();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
