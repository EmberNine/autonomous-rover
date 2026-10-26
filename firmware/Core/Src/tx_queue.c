/**
  ******************************************************************************
  * @file    tx_queue.c
  * @brief   Single-producer / single-consumer queue of outgoing frames (see
  *          the header).
  ******************************************************************************
  */
#include "tx_queue.h"

/* Compile-time check of the header's power-of-two rule. */
typedef char tx_queue_slots_is_power_of_two[((TX_QUEUE_SLOTS & (TX_QUEUE_SLOTS - 1u)) == 0u) ? 1 : -1];

void TxQueue_Init(TxQueue *queue)
{
  for (size_t slot = 0u; slot < TX_QUEUE_SLOTS; slot++)
  {
    for (size_t i = 0u; i < TX_QUEUE_SLOT_BYTES; i++)
    {
      queue->bytes[slot][i] = 0u;
    }
    queue->lengths[slot] = 0u;
  }
  queue->write_count = 0u;
  queue->read_count = 0u;
  queue->dropped_count = 0u;
}

bool TxQueue_Push(TxQueue *queue, const uint8_t *frame, size_t length)
{
  uint32_t write_count = queue->write_count;  /* our own variable: no one else changes it */
  if (length == 0u || length > TX_QUEUE_SLOT_BYTES || write_count - queue->read_count >= TX_QUEUE_SLOTS)
  {
    queue->dropped_count = queue->dropped_count + 1u;
    return false;
  }
  uint32_t slot = write_count % TX_QUEUE_SLOTS;
  for (size_t i = 0u; i < length; i++)
  {
    queue->bytes[slot][i] = frame[i];
  }
  queue->lengths[slot] = (uint16_t)length;
  queue->write_count = write_count + 1u;      /* published only after the frame is stored */
  return true;
}

uint32_t TxQueue_Count(const TxQueue *queue)
{
  return queue->write_count - queue->read_count;
}

const volatile uint8_t *TxQueue_Front(const TxQueue *queue, size_t *length)
{
  uint32_t read_count = queue->read_count;
  if (read_count == queue->write_count)
  {
    *length = 0u;
    return NULL;
  }
  uint32_t slot = read_count % TX_QUEUE_SLOTS;
  *length = queue->lengths[slot];
  return queue->bytes[slot];
}

void TxQueue_Pop(TxQueue *queue)
{
  uint32_t read_count = queue->read_count;    /* our own variable: no one else changes it */
  if (read_count == queue->write_count)
  {
    return;
  }
  queue->read_count = read_count + 1u;        /* frees the slot only after its bytes were sent */
}
