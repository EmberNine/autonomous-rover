/**
  ******************************************************************************
  * @file    byte_ring.h
  * @brief   Fixed-size byte queue between ONE interrupt (the producer) and the
  *          main loop (the consumer), with no locks and no disabled interrupts.
  *
  * Why it is safe without locks: every variable has exactly one writer.
  *   - bytes[] slots and write_count:  written only by the producer (the ISR)
  *   - read_count:                     written only by the consumer (main loop)
  *   - dropped_count:                  written only by the producer
  * A 32-bit aligned load or store is a single instruction on the Cortex-M4,
  * so the other side can never see half of an update.
  *
  * Why everything is volatile: the main loop reads write_count in a loop while
  * an interrupt changes it. Without volatile the compiler may (and at -Og it
  * does) keep the value in a register and never see the interrupt's update.
  * volatile also keeps the order of the accesses as written: the producer
  * stores the byte BEFORE advancing write_count, and the consumer reads
  * write_count BEFORE reading the byte, so it never reads a slot that hasn't
  * been filled yet. On a single core no extra hardware barrier is needed.
  *
  * The counts run freely and wrap at 2^32. Because the capacity divides 2^32,
  * "write_count - read_count" (unsigned) is always the number of bytes queued,
  * even across the wrap.
  *
  * Pure C with no HAL or MCU dependencies, so the same file compiles on a PC
  * (firmware/tests/test_motor_control.c). No globals: the caller owns the
  * struct.
  ******************************************************************************
  */
#ifndef BYTE_RING_H
#define BYTE_RING_H

#include <stdbool.h>
#include <stdint.h>

/* Must be a power of two (so it divides 2^32). 64 bytes hold about 5.5 ms of
 * back-to-back bytes at 115200 baud (86.8 us each), so even a 5 ms loop stall
 * loses nothing. Commands are a few bytes every 100 ms. */
#define BYTE_RING_CAPACITY  64u

typedef struct
{
  volatile uint8_t  bytes[BYTE_RING_CAPACITY];
  volatile uint32_t write_count;    /* bytes ever pushed (producer only) */
  volatile uint32_t read_count;     /* bytes ever popped (consumer only) */
  volatile uint32_t dropped_count;  /* bytes lost because the ring was full (producer only) */
} ByteRing;

/* Empties the ring and zeroes the counters. Call before the producer starts. */
void ByteRing_Init(ByteRing *ring);

/* Producer side only (the UART receive interrupt). Returns false and counts
 * the byte in dropped_count if the ring is full. */
bool ByteRing_Push(ByteRing *ring, uint8_t byte);

/* Consumer side only (the main loop). Returns false if the ring is empty. */
bool ByteRing_Pop(ByteRing *ring, uint8_t *byte);

#endif /* BYTE_RING_H */
