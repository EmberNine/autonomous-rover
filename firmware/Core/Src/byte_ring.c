/**
  ******************************************************************************
  * @file    byte_ring.c
  * @brief   Single-producer / single-consumer byte queue (see the header).
  ******************************************************************************
  */
#include "byte_ring.h"

#include <stddef.h>

/* Compile-time check of the header's power-of-two rule. */
typedef char byte_ring_capacity_is_power_of_two[((BYTE_RING_CAPACITY & (BYTE_RING_CAPACITY - 1u)) == 0u) ? 1 : -1];

void ByteRing_Init(ByteRing *ring)
{
  for (size_t i = 0; i < BYTE_RING_CAPACITY; i++)
  {
    ring->bytes[i] = 0u;
  }
  ring->write_count = 0u;
  ring->read_count = 0u;
  ring->dropped_count = 0u;
}

bool ByteRing_Push(ByteRing *ring, uint8_t byte)
{
  uint32_t write_count = ring->write_count;  /* our own variable: no one else changes it */
  if (write_count - ring->read_count >= BYTE_RING_CAPACITY)
  {
    ring->dropped_count = ring->dropped_count + 1u;
    return false;
  }
  ring->bytes[write_count % BYTE_RING_CAPACITY] = byte;
  ring->write_count = write_count + 1u;      /* published only after the byte is stored */
  return true;
}

bool ByteRing_Pop(ByteRing *ring, uint8_t *byte)
{
  uint32_t read_count = ring->read_count;    /* our own variable: no one else changes it */
  if (read_count == ring->write_count)
  {
    return false;
  }
  *byte = ring->bytes[read_count % BYTE_RING_CAPACITY];
  ring->read_count = read_count + 1u;        /* frees the slot only after the byte is read */
  return true;
}
