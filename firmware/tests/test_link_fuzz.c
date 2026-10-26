/*
 * Corruption / fuzz test for the Pi link receive path (link_rx.c over
 * link_protocol.c and link_codec.c), with the real motor, speed and steering
 * controllers, under AddressSanitizer and UBSan. Deterministic (fixed
 * seeds), so a failure is reproducible. Three streams:
 *   A. uniform random bytes: nothing is ever acted on
 *   B. random bodies wrapped in valid COBS, so every piece reaches the
 *      CRC: the few that pass it by chance are counted, and the motor
 *      watchdog moves only on a DRIVE event
 *   C. real frames, half of them damaged (bit flip, lost byte, extra byte,
 *      cut short, extra delimiter, noise): every frame acted on is
 *      byte-for-byte one that was sent, and every undamaged frame whose
 *      neighbours are intact is acted on
 * Not part of the firmware build or CI.
 *
 * Build and run from firmware/ (WSL / Linux, GCC):
 *   gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc tests/test_link_fuzz.c Core/Src/link_rx.c Core/Src/link_protocol.c Core/Src/link_codec.c Core/Src/motor_control.c Core/Src/speed_control.c Core/Src/steering_control.c -o /tmp/test_link_fuzz && /tmp/test_link_fuzz
 *
 * Exit code 0 and "ALL PASSED" means every check passed.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link_rx.h"

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

static uint32_t random_state;

static uint32_t next_random(void)
{
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

typedef struct
{
  MotorController    motor;
  SpeedController    speed;
  SteeringController steering;
  LinkRx             link;
  LinkRxTargets      targets;
  uint32_t           now_ms;
  /* What the byte loop saw. */
  uint32_t           acted;               /* HELLO, DRIVE, ESTOP events */
  uint32_t           feeds_without_drive; /* watchdog reference moved without a DRIVE event */
  uint32_t           steering_without_drive;
} Rig;

static void rig_init(Rig *rig, bool binary)
{
  memset(rig, 0, sizeof(*rig));
  rig->now_ms = 1000u;
  MotorController_Init(&rig->motor, rig->now_ms);
  SpeedControl_Init(&rig->speed);
  SteeringController_Init(&rig->steering, rig->now_ms, false);
  LinkRx_Init(&rig->link);
  rig->targets = (LinkRxTargets){ &rig->motor, &rig->speed, &rig->steering };
  if (binary)
  {
    uint8_t version = LINK_PROTOCOL_VERSION, wire[LINK_MAX_WIRE];
    size_t n = LinkFrame_Encode(LINK_MSG_HELLO, 0u, &version, 1u, wire, sizeof(wire));
    for (size_t i = 0u; i < n; i++) { (void)LinkRx_FeedByte(&rig->link, wire[i], &rig->targets, rig->now_ms); }
  }
}

/* One byte, with the "only DRIVE moves the watchdogs" property checked. */
static LinkRxEvent rig_byte(Rig *rig, uint8_t byte)
{
  uint32_t fed = rig->motor.last_command_ms;
  uint32_t steering = rig->steering.valid_command_count;
  rig->now_ms += (next_random() % 64u == 0u) ? 1u : 0u;
  LinkRxEvent event = LinkRx_FeedByte(&rig->link, byte, &rig->targets, rig->now_ms);
  if (event == LINK_RX_HELLO || event == LINK_RX_DRIVE || event == LINK_RX_ESTOP) { rig->acted++; }
  if (event != LINK_RX_DRIVE && rig->motor.last_command_ms != fed) { rig->feeds_without_drive++; }
  if (event != LINK_RX_DRIVE && rig->steering.valid_command_count != steering) { rig->steering_without_drive++; }
  if ((rig->now_ms & 0xFFu) == 0u) { (void)MotorController_Update(&rig->motor, rig->now_ms); }
  return event;
}

static void test_uniform_random_bytes(void)
{
  random_state = 0xC0FFEEu;
  Rig rig;
  rig_init(&rig, true);
  for (uint32_t i = 0u; i < 3000000u; i++) { (void)rig_byte(&rig, (uint8_t)next_random()); }
  CHECK(rig.acted == 0u && rig.link.ok_count == 1u, "3,000,000 random bytes: %u acted on", (unsigned)rig.acted);
  CHECK(rig.feeds_without_drive == 0u && rig.steering_without_drive == 0u, "no watchdog moved without a DRIVE");
  printf("  A: %u crc errors, %u frame errors, %u rejected, %u text lines ignored\n",
         (unsigned)rig.link.crc_error_count, (unsigned)rig.link.frame_error_count,
         (unsigned)rig.link.rejected_count, (unsigned)rig.link.text_ignored_count);
}

static void test_random_bodies_in_valid_cobs(void)
{
  /* Every piece is valid COBS around a random 4..12-byte body, so every one
   * reaches the CRC: about one in 65,536 passes it by chance, and then the
   * type, mode, length and range checks still apply. */
  random_state = 0xBADC0DEu;
  Rig rig;
  rig_init(&rig, true);
  uint32_t pieces = 0u;
  for (uint32_t i = 0u; i < 1000000u; i++)
  {
    uint8_t body[12], cobs[16];
    size_t length = 4u + next_random() % 9u;
    for (size_t k = 0u; k < length; k++)
    {
      uint32_t dice = next_random() % 8u;
      /* Bias the type byte to the Pi's types so chance passes reach the payload checks. */
      body[k] = (k == 0u && dice < 6u) ? ((dice < 4u) ? LINK_MSG_DRIVE : (dice == 4u) ? LINK_MSG_ESTOP : LINK_MSG_HELLO)
                                        : (uint8_t)next_random();
    }
    size_t n = LinkCobs_Encode(body, length, cobs, sizeof(cobs));
    (void)rig_byte(&rig, 0u);
    for (size_t k = 0u; k < n; k++) { (void)rig_byte(&rig, cobs[k]); }
    (void)rig_byte(&rig, 0u);
    pieces++;
  }
  uint32_t crc_valid = rig.link.ok_count - 1u + rig.link.rejected_count;
  CHECK(rig.feeds_without_drive == 0u && rig.steering_without_drive == 0u, "no watchdog moved without a DRIVE");
  CHECK(rig.link.crc_error_count + crc_valid == pieces && rig.link.frame_error_count == 0u,
        "every piece reached the CRC (%u CRC errors + %u passed = %u pieces)", (unsigned)rig.link.crc_error_count,
        (unsigned)crc_valid, (unsigned)pieces);
  CHECK(crc_valid < 60u, "CRC-valid by chance: %u of %u (expected ~15)", (unsigned)crc_valid, (unsigned)pieces);
  printf("  B: %u random bodies, %u passed the CRC by chance: %u then rejected, %u acted on\n", (unsigned)pieces,
         (unsigned)crc_valid, (unsigned)rig.link.rejected_count, (unsigned)rig.acted);
}

#define FRAME_COUNT 200000u

typedef struct
{
  uint8_t type;
  uint8_t seq;
  uint8_t payload[LINK_DRIVE_LENGTH];
  uint8_t length;
  bool    damaged;
} SentFrame;

static SentFrame sent[FRAME_COUNT];

static bool frame_equals(const LinkFrame *frame, const SentFrame *s)
{
  return frame->type == s->type && frame->seq == s->seq && frame->payload_length == s->length &&
         memcmp(frame->payload, s->payload, s->length) == 0;
}

static void test_damaged_real_frames(void)
{
  random_state = 0x5EED5u;
  Rig rig;
  rig_init(&rig, true);
  uint32_t not_sent = 0u, acted_ok = 0u;
  static bool acted_on[FRAME_COUNT];
  memset(acted_on, 0, sizeof(acted_on));
  for (uint32_t i = 0u; i < FRAME_COUNT; i++)
  {
    SentFrame *s = &sent[i];
    uint32_t kind = next_random() % 100u;
    s->seq = (uint8_t)i;
    if (kind < 1u)
    {
      s->type = LINK_MSG_HELLO;
      s->payload[0] = LINK_PROTOCOL_VERSION;
      s->length = 1u;
    }
    else if (kind < 3u)
    {
      s->type = LINK_MSG_ESTOP;
      s->length = 0u;
    }
    else
    {
      s->type = LINK_MSG_DRIVE;
      bool arm = (kind < 8u);
      int32_t speed = arm ? 0 : (int32_t)(i % 10001u) - 5000;
      (void)LinkProtocol_PackDrive(speed, (int32_t)(i % 201u) - 100,
                                   (arm ? LINK_DRIVE_ARM : 0u) | (next_random() % 2u), s->payload);
      s->length = LINK_DRIVE_LENGTH;
    }
    uint8_t wire[LINK_MAX_WIRE + 32u];
    size_t n = LinkFrame_Encode(s->type, s->seq, s->payload, s->length, wire, LINK_MAX_WIRE);
    s->damaged = (next_random() % 2u == 0u);
    if (s->damaged)
    {
      size_t at = next_random() % n;
      switch (next_random() % 6u)
      {
        case 0: wire[at] ^= (uint8_t)(1u << (next_random() % 8u)); break;               /* bit flip */
        case 1: memmove(&wire[at], &wire[at + 1u], n - at - 1u); n--; break;           /* lost byte */
        case 2: memmove(&wire[at + 1u], &wire[at], n - at); wire[at] = (uint8_t)next_random(); n++; break;
        case 3: n = 1u + next_random() % (n - 1u); break;                               /* cut short */
        case 4: memmove(&wire[at + 1u], &wire[at], n - at); wire[at] = 0u; n++; break;  /* extra 0x00 */
        default:                                                                        /* noise after */
          for (uint32_t k = 1u + next_random() % 20u; k > 0u; k--) { wire[n++] = (uint8_t)next_random(); }
          break;
      }
    }
    for (size_t b = 0u; b < n; b++)
    {
      LinkRxEvent event = rig_byte(&rig, wire[b]);
      if (event == LINK_RX_HELLO || event == LINK_RX_DRIVE || event == LINK_RX_ESTOP)
      {
        /* It must be this frame or one of the two before it, unchanged. */
        bool found = false;
        for (uint32_t back = 0u; back <= 2u && back <= i && !found; back++)
        {
          if (frame_equals(&rig.link.frame, &sent[i - back]))
          {
            found = true;
            acted_on[i - back] = true;
            if (!sent[i - back].damaged) { acted_ok++; }
          }
        }
        if (!found) { not_sent++; }
      }
    }
  }
  /* An undamaged frame is lost only if the frame before it damaged its own
   * closing delimiter or left noise behind; with an intact frame before it,
   * every undamaged frame must be acted on. */
  uint32_t must_act = 0u, missed = 0u;
  for (uint32_t i = 1u; i < FRAME_COUNT; i++)
  {
    if (!sent[i].damaged && !sent[i - 1u].damaged)
    {
      must_act++;
      if (!acted_on[i]) { missed++; }
    }
  }
  CHECK(not_sent == 0u, "every frame acted on was sent unchanged (%u were not)", (unsigned)not_sent);
  CHECK(missed == 0u && must_act > 40000u, "every undamaged frame after an undamaged one acted on (%u of %u missed)",
        (unsigned)missed, (unsigned)must_act);
  CHECK(rig.feeds_without_drive == 0u && rig.steering_without_drive == 0u, "no watchdog moved without a DRIVE");
  printf("  C: %u frames, %u acted on (%u undamaged), %u crc errors, %u frame errors, %u rejected, %u seq gaps\n",
         (unsigned)FRAME_COUNT, (unsigned)rig.acted, (unsigned)acted_ok, (unsigned)rig.link.crc_error_count,
         (unsigned)rig.link.frame_error_count, (unsigned)rig.link.rejected_count, (unsigned)rig.link.seq_gap_count);
}

int main(void)
{
  test_uniform_random_bytes();
  test_random_bodies_in_valid_cobs();
  test_damaged_real_frames();

  if (failure_count == 0)
  {
    printf("ALL PASSED (%d checks)\n", check_count);
    return EXIT_SUCCESS;
  }
  printf("%d of %d checks FAILED\n", failure_count, check_count);
  return EXIT_FAILURE;
}
