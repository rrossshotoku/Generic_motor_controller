#ifndef MC_PERSISTENT_STORE_H
#define MC_PERSISTENT_STORE_H
#include "mc_types.h"

/** @file mc_persistent_store.h
 *  @brief Generic A/B non-volatile record store (header + CRC32-validated payload). See ADR-010.
 *  @ingroup mc_persistence
 *
 *  Stores one variable-size payload across two erase-slots (ping-pong) with a monotonic sequence
 *  counter, so a power loss mid-write always leaves the previous good copy intact. HAL-free: the
 *  actual flash access is delegated to mc_flash_port. The payload schema is the caller's (e.g.
 *  MC_CalibData_t). Writes happen only in the slow/background context.
 */
#define MC_PARAM_STORE_MAGIC       (0x4D435046u) /* 'MCPF' */
#define MC_PARAM_STORE_VERSION     (3u)   /* v3 (ADR-044). NOT bumped for the ADR-070 blob growth 448->640 B:
                                             MC_PersistentStore_Read now front-loads a shorter old record and
                                             zero-fills the grown tail, so old v3 records stay valid (no re-save). */
#define MC_PARAM_STORE_MAX_PAYLOAD (768u)  /* >= sizeof(MC_Params_t) (~668 B with a 640 B blob); slot is 2 KB (ADR-070) */

/** @brief On-flash record header (16 bytes; CRC32 covers the first 12 bytes + payload). */
typedef struct
{
    uint32_t magic;        /**< MC_PARAM_STORE_MAGIC. */
    uint16_t version;      /**< MC_PARAM_STORE_VERSION. */
    uint16_t payload_size; /**< Payload byte count. */
    uint32_t seq;          /**< Monotonic write counter (highest valid wins across slots). */
    uint32_t crc32;        /**< CRC32 over {magic, version, payload_size, seq} + payload. */
} MC_ParamStoreHeader_t;

/** @brief Scan both slots and latch the newest valid record into RAM. @return MC_OK if found. */
MC_Status_t MC_PersistentStore_Init(void);
/** @brief True if a valid record is currently held (after Init or a successful save). */
bool MC_PersistentStore_HasValid(void);
/** @brief True if a save has been requested but not yet written. */
bool MC_PersistentStore_SavePending(void);
/** @brief Copy the active payload out. @p size may be >= the stored size: the stored bytes are
 *  front-copied and any excess (a struct that grew since the record was written) is zero-filled,
 *  so a payload whose trailing field is a variable-length blob stays backward-compatible (ADR-070).
 *  A @p size smaller than the stored payload is rejected (MC_ERR_RANGE). */
MC_Status_t MC_PersistentStore_Read(void *dst, uint16_t size);
/** @brief Latch a payload to be written to the inactive slot by ServiceSlow. */
MC_Status_t MC_PersistentStore_RequestSave(const void *src, uint16_t size);
/** @brief Perform a pending write (slow/background context only; blocking flash erase+program). */
void MC_PersistentStore_ServiceSlow(void);
/** @brief Invalidate both slots so the next boot loads defaults. */
MC_Status_t MC_PersistentStore_FactoryReset(void);

#endif /* MC_PERSISTENT_STORE_H */
