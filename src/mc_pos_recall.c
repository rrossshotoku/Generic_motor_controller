#include "mc_pos_recall.h"
#include "mc_pos_recall_port.h"
#include <string.h>

/** @file mc_pos_recall.c
 *  @brief Position-recall journal logic (HAL-free). See ADR-067 and mc_pos_recall.h.
 *
 *  Fixed 16-byte records are appended linearly across the POS_RECALL region; when the region
 *  fills, both pages are erased and the log restarts at slot 0 (the just-written record is the
 *  only history we need). The highest-`seq` good-CRC record is authoritative. All access is via
 *  MC_PosRecallPort_* so this file has no HAL dependency and stays host-compilable.
 */

#define MC_CRC32_POLY      0xEDB88320u
#define MC_POSREC_VALID    0x56414C44u   /* 'VALD' -- stored position is trustworthy */
#define MC_POSREC_INVALID  0x494E5644u   /* 'INVD' -- move in progress / stale        */
#define MC_POSREC_ERASED   0xFFFFFFFFu   /* erased flash reads as all-ones            */

typedef struct {
    uint32_t seq;            /* monotonic; wraps carry it forward across erase-and-restart */
    float    position_rad;   /* home-relative mechanical position */
    uint32_t marker;         /* MC_POSREC_VALID | MC_POSREC_INVALID */
    uint32_t crc;            /* CRC32 over the first 12 bytes (seq, position, marker) */
} pos_rec_t;                 /* 16 bytes = 2 doublewords */

/* Latched at Init(); reflect the authoritative (latest good-CRC) record. */
static uint32_t s_next_slot;     /* slot index the next append will use */
static uint32_t s_next_seq;      /* seq the next append will carry */
static uint32_t s_nslots;        /* region_size / 16 */
static bool     s_have_record;   /* any good-CRC record exists */
static uint32_t s_latest_marker; /* marker of the latest good-CRC record */
static float    s_latest_pos;    /* position of the latest good-CRC record */

/* CRC32 (reflected, poly 0xEDB88320) over the 12-byte record prefix -- matches
 * mc_persistent_store / boot_flash so tooling can reproduce it. */
static uint32_t rec_crc(const pos_rec_t *r)
{
    const uint8_t *p = (const uint8_t *)r;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < 12u; i++)
    {
        crc ^= p[i];
        for (uint8_t j = 0u; j < 8u; j++)
        {
            crc = (crc & 1u) ? ((crc >> 1) ^ MC_CRC32_POLY) : (crc >> 1);
        }
    }
    return ~crc;
}

static bool slot_good(const pos_rec_t *r)
{
    return (r->seq != MC_POSREC_ERASED) && (rec_crc(r) == r->crc);
}

void MC_PosRecall_Init(void)
{
    s_next_slot     = 0u;
    s_next_seq      = 0u;
    s_have_record   = false;
    s_latest_marker = MC_POSREC_INVALID;
    s_latest_pos    = 0.0f;
    s_nslots        = MC_PosRecallPort_Size() / (uint32_t)sizeof(pos_rec_t);

    uint32_t best_slot = 0u;
    uint32_t best_seq  = 0u;
    for (uint32_t i = 0u; i < s_nslots; i++)
    {
        pos_rec_t r;
        MC_PosRecallPort_Read(i * (uint32_t)sizeof(pos_rec_t), &r, sizeof r);
        if (!slot_good(&r)) { continue; }
        if (!s_have_record || (r.seq >= best_seq))
        {
            best_seq        = r.seq;
            best_slot       = i;
            s_latest_marker = r.marker;
            s_latest_pos    = r.position_rad;
            s_have_record   = true;
        }
    }

    if (s_have_record)
    {
        s_next_seq  = best_seq + 1u;
        s_next_slot = best_slot + 1u;   /* linear append; wraps handled at write time */
    }
}

bool  MC_PosRecall_HasValidStored(void) { return s_have_record && (s_latest_marker == MC_POSREC_VALID); }
bool  MC_PosRecall_HasAnyRecord(void)   { return s_have_record; }
float MC_PosRecall_StoredPosition(void) { return s_latest_pos; }

/* Append one record; erase-and-restart on wrap. Updates the latched latest state on success. */
static void append(float position_rad, uint32_t marker)
{
    if (s_nslots == 0u) { return; }

    if (s_next_slot >= s_nslots)
    {
        if (MC_PosRecallPort_EraseAll() != MC_OK) { return; }
        s_next_slot = 0u;
    }

    pos_rec_t r;
    r.seq          = s_next_seq;
    r.position_rad = position_rad;
    r.marker       = marker;
    r.crc          = rec_crc(&r);

    if (MC_PosRecallPort_Program(s_next_slot * (uint32_t)sizeof(pos_rec_t), &r, sizeof r) != MC_OK)
    {
        return;   /* leave latched state as-is; a failed write must not claim success */
    }

    s_next_slot++;
    s_next_seq++;
    s_have_record   = true;
    s_latest_marker = marker;
    s_latest_pos    = position_rad;
}

void MC_PosRecall_MarkMoving(void)
{
    /* Only write an INVALID marker if the store currently looks valid -- avoids piling up
     * duplicate INVALID records (and page erases) while the axis idles unhomed. */
    if (s_latest_marker != MC_POSREC_INVALID)
    {
        append(s_latest_pos, MC_POSREC_INVALID);
    }
}

void MC_PosRecall_Store(float position_home_rel)
{
    append(position_home_rel, MC_POSREC_VALID);
}
