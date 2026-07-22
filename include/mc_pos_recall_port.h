#ifndef MC_POS_RECALL_PORT_H
#define MC_POS_RECALL_PORT_H
#include "mc_types.h"

/** @file mc_pos_recall_port.h
 *  @brief Flash backend for the position-recall journal (ADR-067).
 *  @ingroup mc_persistence
 *
 *  Abstracts the POS_RECALL flash region (linker `_pos_recall_origin`/`_pos_recall_length`,
 *  bank2 pg124/125) so the journal logic in mc_pos_recall.c stays HAL-free and host-compilable.
 *  The region is treated as one linear byte space [0 .. Size()); the two physical pages are only
 *  visible via EraseAll (the journal erases both together when it wraps). Blocking -> slow ctx.
 */

/** @brief Total journal region size in bytes (whole region, both pages). */
uint32_t    MC_PosRecallPort_Size(void);

/** @brief Copy @p len bytes from journal offset @p offset into @p dst. */
void        MC_PosRecallPort_Read(uint32_t offset, void *dst, uint32_t len);

/** @brief Program @p len bytes (must be a multiple of 8) at journal offset @p offset. */
MC_Status_t MC_PosRecallPort_Program(uint32_t offset, const void *src, uint32_t len);

/** @brief Erase the entire journal region (all pages) back to 0xFF. */
MC_Status_t MC_PosRecallPort_EraseAll(void);

#endif /* MC_POS_RECALL_PORT_H */
