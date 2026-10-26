/*
 * Host-side test for Core/Src/tx_queue.c: order, full and empty, length
 * limits, the frame being sent never overwritten, the count wrap at 2^32,
 * and a random million operations against a simple FIFO model. Not part of
 * the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_tx_queue.c Core/Src/tx_queue.c -o /tmp/test_tx_queue && /tmp/test_tx_queue
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tx_queue.h"

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

static uint32_t random_state = 99u;

static uint32_t next_random(void)
{
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

/* A frame whose bytes all say which frame it is and how long. */
static size_t make_frame(uint8_t *frame, uint32_t id, size_t length)
{
  for (size_t i = 0u; i < length; i++) { frame[i] = (uint8_t)(id * 31u + i); }
  return length;
}

static bool front_is(const TxQueue *queue, uint32_t id, size_t length)
{
  size_t front_length = 0u;
  const volatile uint8_t *front = TxQueue_Front(queue, &front_length);
  if (front == NULL || front_length != length)
  {
    return false;
  }
  for (size_t i = 0u; i < length; i++)
  {
    if (front[i] != (uint8_t)(id * 31u + i)) { return false; }
  }
  return true;
}

static void test_basics(void)
{
  TxQueue queue;
  TxQueue_Init(&queue);
  size_t length = 99u;
  CHECK(TxQueue_Front(&queue, &length) == NULL && length == 0u && TxQueue_Count(&queue) == 0u, "starts empty");
  TxQueue_Pop(&queue);
  CHECK(TxQueue_Count(&queue) == 0u && queue.read_count == 0u, "pop on empty: no effect");

  uint8_t frame[TX_QUEUE_SLOT_BYTES + 1u];
  for (uint32_t id = 0u; id < TX_QUEUE_SLOTS; id++)
  {
    CHECK(TxQueue_Push(&queue, frame, make_frame(frame, id, 10u + id)), "push %u", (unsigned)id);
  }
  CHECK(TxQueue_Count(&queue) == TX_QUEUE_SLOTS, "full at %u", (unsigned)TX_QUEUE_SLOTS);
  CHECK(!TxQueue_Push(&queue, frame, make_frame(frame, 99u, 5u)) && queue.dropped_count == 1u, "full: refused, counted");
  CHECK(front_is(&queue, 0u, 10u), "full: the first frame is untouched at the front");
  for (uint32_t id = 0u; id < TX_QUEUE_SLOTS; id++)
  {
    CHECK(front_is(&queue, id, 10u + id), "frame %u comes out in order", (unsigned)id);
    TxQueue_Pop(&queue);
  }
  CHECK(TxQueue_Count(&queue) == 0u, "empty again");

  CHECK(!TxQueue_Push(&queue, frame, 0u), "an empty frame is refused");
  CHECK(!TxQueue_Push(&queue, frame, TX_QUEUE_SLOT_BYTES + 1u), "129 bytes are refused");
  CHECK(queue.dropped_count == 3u && TxQueue_Count(&queue) == 0u, "both counted, nothing queued");
  CHECK(TxQueue_Push(&queue, frame, make_frame(frame, 7u, TX_QUEUE_SLOT_BYTES)) && front_is(&queue, 7u, TX_QUEUE_SLOT_BYTES),
        "exactly 128 bytes fit");
}

static void test_front_is_never_overwritten(void)
{
  /* The frame being sent stays intact while the producer fills every other
   * slot and is refused after that. */
  TxQueue queue;
  TxQueue_Init(&queue);
  uint8_t frame[TX_QUEUE_SLOT_BYTES];
  (void)TxQueue_Push(&queue, frame, make_frame(frame, 1000u, TX_QUEUE_SLOT_BYTES));
  bool intact = true;
  for (uint32_t id = 0u; id < 20u; id++)
  {
    (void)TxQueue_Push(&queue, frame, make_frame(frame, id, 1u + id % TX_QUEUE_SLOT_BYTES));
    if (!front_is(&queue, 1000u, TX_QUEUE_SLOT_BYTES)) { intact = false; }
  }
  CHECK(intact && TxQueue_Count(&queue) == TX_QUEUE_SLOTS && queue.dropped_count == 20u - (TX_QUEUE_SLOTS - 1u),
        "front intact through 20 pushes (%u dropped)", (unsigned)queue.dropped_count);
}

static void test_count_wrap(void)
{
  TxQueue queue;
  TxQueue_Init(&queue);
  queue.write_count = UINT32_MAX - 1u;
  queue.read_count = UINT32_MAX - 1u;
  uint8_t frame[16];
  bool ok = true;
  for (uint32_t id = 0u; id < 10u; id++)
  {
    ok = ok && TxQueue_Push(&queue, frame, make_frame(frame, id, 16u));
    ok = ok && TxQueue_Count(&queue) == 1u && front_is(&queue, id, 16u);
    TxQueue_Pop(&queue);
    ok = ok && TxQueue_Count(&queue) == 0u;
  }
  CHECK(ok && queue.write_count == 8u, "counts wrap at 2^32 without losing a frame");
}

static void test_random_against_model(void)
{
  TxQueue queue;
  TxQueue_Init(&queue);
  uint32_t model_ids[TX_QUEUE_SLOTS];
  size_t model_lengths[TX_QUEUE_SLOTS];
  uint32_t model_head = 0u, model_count = 0u, next_id = 0u, model_dropped = 0u;
  int mismatches = 0;
  for (int op = 0; op < 1000000; op++)
  {
    if (next_random() % 2u == 0u)
    {
      uint8_t frame[TX_QUEUE_SLOT_BYTES + 1u];
      size_t length = next_random() % (TX_QUEUE_SLOT_BYTES + 2u);   /* 0..129: both limits too */
      bool expected = (length >= 1u && length <= TX_QUEUE_SLOT_BYTES && model_count < TX_QUEUE_SLOTS);
      bool pushed = TxQueue_Push(&queue, frame, make_frame(frame, next_id, length));
      if (pushed != expected) { mismatches++; }
      if (expected)
      {
        uint32_t slot = (model_head + model_count) % TX_QUEUE_SLOTS;
        model_ids[slot] = next_id;
        model_lengths[slot] = length;
        model_count++;
      }
      else
      {
        model_dropped++;
      }
      next_id++;
    }
    else
    {
      if (model_count > 0u)
      {
        if (!front_is(&queue, model_ids[model_head], model_lengths[model_head])) { mismatches++; }
        model_head = (model_head + 1u) % TX_QUEUE_SLOTS;
        model_count--;
      }
      TxQueue_Pop(&queue);
    }
    if (TxQueue_Count(&queue) != model_count || queue.dropped_count != model_dropped) { mismatches++; }
  }
  CHECK(mismatches == 0, "a million random pushes and pops match a FIFO model (%d mismatches)", mismatches);
}

int main(void)
{
  test_basics();
  test_front_is_never_overwritten();
  test_count_wrap();
  test_random_against_model();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
