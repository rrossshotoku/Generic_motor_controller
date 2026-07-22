#ifndef MC_POS_RECALL_H
#define MC_POS_RECALL_H
#include "mc_types.h"

/** @file mc_pos_recall.h
 *  @brief Persistent last-position journal for non-back-drivable incremental axes (ADR-067).
 *  @ingroup mc_persistence
 *
 *  A small wear-levelled flash journal in the dedicated POS_RECALL region (linker
 *  `_pos_recall_origin`/`_pos_recall_length`, bank2 pg124/125). Fixed 16-byte records
 *  {seq, position, marker(VALID/INVALID), crc} are appended; the highest-seq good-CRC record
 *  is authoritative. The stored position is the HOME-RELATIVE mechanical position.
 *
 *  Sequence per move (recall on): MarkMoving() at motion start (records INVALID -> a mid-move
 *  power loss stays "invalid"), Store() once settled (records VALID + the new position). On
 *  boot, a VALID latest record lets the axis skip homing. All flash ops are blocking -> call
 *  only in the slow/supervisory context.
 */

/** @brief Scan the journal at boot; latches the latest record's state. Call once at init. */
void  MC_PosRecall_Init(void);

/** @brief True iff the latest good-CRC record is VALID (a trustworthy stored position). */
bool  MC_PosRecall_HasValidStored(void);

/** @brief True iff any good-CRC record exists (distinguishes "stale/INVALID" from "empty"). */
bool  MC_PosRecall_HasAnyRecord(void);

/** @brief The stored home-relative position [rad] (meaningful only when HasValidStored). */
float MC_PosRecall_StoredPosition(void);

/** @brief Append an INVALID record (motion started -> stored position is now stale). Slow ctx. */
void  MC_PosRecall_MarkMoving(void);

/** @brief Append a VALID record with the current home-relative position [rad]. Slow ctx. */
void  MC_PosRecall_Store(float position_home_rel);

#endif /* MC_POS_RECALL_H */
