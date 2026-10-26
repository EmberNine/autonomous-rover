/**
  ******************************************************************************
  * @file    link_codec.h
  * @brief   Framing for the Pi link: CRC-16/CCITT-FALSE, COBS, and whole
  *          frames. Knows nothing about what the messages mean
  *          (link_protocol.c) or where the bytes come from (link_rx.c).
  *
  * A frame on the wire:
  *
  *   0x00  COBS( type | seq | payload | crc_lo | crc_hi )  0x00
  *
  *   - body = type (1 byte), seq (1 byte), payload (0..LINK_MAX_PAYLOAD
  *     bytes), then the CRC of type | seq | payload, low byte first
  *   - CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no
  *     reflection, no final XOR (check value 0x29B1 for "123456789"; Python:
  *     binascii.crc_hqx(data, 0xFFFF))
  *   - COBS (Cheshire and Baker, 1999) removes every 0x00 from the body, so
  *     0x00 only ever means "frame boundary". The body is at most 254 bytes,
  *     so COBS adds exactly one byte.
  *   - a 0x00 before AND after: a receiver that joins mid-frame, or loses a
  *     byte, is back in step at the next 0x00. Two 0x00 in a row are an
  *     empty frame, which receivers skip.
  *
  * The same rules, and the shared vectors in firmware/tests/link_vectors.txt,
  * are implemented by tools/rover_link.py.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_link_codec.c. No globals (the CRC table is const).
  ******************************************************************************
  */
#ifndef LINK_CODEC_H
#define LINK_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Largest payload of any message (the 10 Hz STATUS frame is the longest). */
#define LINK_MAX_PAYLOAD        120u
/* type + seq + payload + CRC. */
#define LINK_BODY_OVERHEAD      4u
#define LINK_MAX_BODY           (LINK_MAX_PAYLOAD + LINK_BODY_OVERHEAD)   /* 124 */
/* COBS of a body of at most 254 bytes: one byte longer. */
#define LINK_MAX_COBS           (LINK_MAX_BODY + 1u)                     /* 125 */
/* With the two 0x00 delimiters: the most one frame ever takes on the wire. */
#define LINK_MAX_WIRE           (LINK_MAX_COBS + 2u)                     /* 127 */

#define LINK_CRC_INITIAL        0xFFFFu

/* CRC-16/CCITT-FALSE of data (table-driven). */
uint16_t LinkCrc16(const uint8_t *data, size_t length);

/* Continues a CRC over more data: LinkCrc16_Update(LinkCrc16(a), b) equals
 * the CRC of a followed by b. Start from LINK_CRC_INITIAL. */
uint16_t LinkCrc16_Update(uint16_t crc, const uint8_t *data, size_t length);

/* COBS-encodes length bytes of in into out (no delimiters). Returns the
 * encoded length, or 0 if it doesn't fit in out_capacity (out is then
 * unspecified). in and out must not overlap. */
size_t LinkCobs_Encode(const uint8_t *in, size_t length, uint8_t *out, size_t out_capacity);

/* Decodes length COBS bytes (the bytes between two delimiters). False if
 * they aren't valid COBS (empty, a 0x00 inside, a code that runs past the
 * end) or the result doesn't fit in out_capacity; *out_length is set only
 * on success. in and out must not overlap. */
bool LinkCobs_Decode(const uint8_t *in, size_t length, uint8_t *out, size_t out_capacity, size_t *out_length);

/* One decoded, CRC-checked frame. */
typedef struct
{
  uint8_t type;
  uint8_t seq;
  uint8_t payload[LINK_MAX_PAYLOAD];
  size_t  payload_length;
} LinkFrame;

typedef enum
{
  LINK_FRAME_OK = 0,
  LINK_FRAME_COBS_ERROR,   /* not valid COBS */
  LINK_FRAME_TOO_SHORT,    /* body shorter than type + seq + CRC */
  LINK_FRAME_TOO_LONG,     /* more than LINK_MAX_COBS bytes, or a body above LINK_MAX_BODY */
  LINK_FRAME_CRC_ERROR     /* the CRC doesn't match */
} LinkFrameStatus;

/* Builds a whole frame, both delimiters included, into out. Returns its
 * length, or 0 if payload_length > LINK_MAX_PAYLOAD or it doesn't fit. */
size_t LinkFrame_Encode(uint8_t type, uint8_t seq, const uint8_t *payload, size_t payload_length,
                        uint8_t *out, size_t out_capacity);

/* Checks the bytes between two delimiters (no 0x00 included) and, if they
 * are a whole frame with the right CRC, fills *frame. *frame is unspecified
 * otherwise: nothing in it may be acted on. */
LinkFrameStatus LinkFrame_Decode(const uint8_t *cobs, size_t length, LinkFrame *frame);

#endif /* LINK_CODEC_H */
