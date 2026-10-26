/**
  ******************************************************************************
  * @file    tx_queue.h
  * @brief   Queue of whole outgoing frames between the main loop (producer)
  *          and the UART transmit-complete interrupt (consumer), with no
  *          locks: the same single-writer rules as byte_ring.h.
  *
  * Why a queue: at 115200 baud a 124-byte STATUS frame takes ~10.8 ms on the
  * wire, longer than the 10 ms between two STATE frames, so a frame can be
  * due while the previous one is still being sent. HAL_UART_Transmit_IT
  * takes one buffer at a time, so frames wait here, and the transmit-complete
  * callback starts the next one.
  *
  * Who writes what:
  *   - slot bytes, lengths and write_count: the producer only (TxQueue_Push)
  *   - read_count: the consumer only (TxQueue_Pop)
  *   - dropped_count: the producer only
  * The frame being sent stays in its slot (TxQueue_Front) until the
  * consumer pops it after its last byte has left, so the producer can never
  * overwrite bytes the UART is still reading.
  *
  * Pure C with no HAL or MCU dependencies; host-tested by
  * firmware/tests/test_tx_queue.c. No globals: the caller owns the struct.
  ******************************************************************************
  */
#ifndef TX_QUEUE_H
#define TX_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* A power of two (so it divides 2^32). Four 128-byte slots: a STATUS and a
 * STATE in flight or waiting, with room to spare. */
#define TX_QUEUE_SLOTS       4u
#define TX_QUEUE_SLOT_BYTES  128u

typedef struct
{
  volatile uint8_t  bytes[TX_QUEUE_SLOTS][TX_QUEUE_SLOT_BYTES];
  volatile uint16_t lengths[TX_QUEUE_SLOTS];
  volatile uint32_t write_count;    /* frames ever pushed (producer only) */
  volatile uint32_t read_count;     /* frames ever popped (consumer only) */
  volatile uint32_t dropped_count;  /* frames refused: full, empty or too long (producer only) */
} TxQueue;

void TxQueue_Init(TxQueue *queue);

/* Producer. Copies a frame (1..TX_QUEUE_SLOT_BYTES bytes) into the next
 * slot. False, and dropped_count + 1, if the queue is full or the length is
 * out of range. */
bool TxQueue_Push(TxQueue *queue, const uint8_t *frame, size_t length);

/* Frames pushed and not yet popped (either side may ask). */
uint32_t TxQueue_Count(const TxQueue *queue);

/* The oldest frame (the one to send, or being sent): its bytes and length,
 * or NULL if the queue is empty. */
const volatile uint8_t *TxQueue_Front(const TxQueue *queue, size_t *length);

/* Consumer only: the oldest frame has been sent; frees its slot. No effect
 * on an empty queue. */
void TxQueue_Pop(TxQueue *queue);

#endif /* TX_QUEUE_H */
