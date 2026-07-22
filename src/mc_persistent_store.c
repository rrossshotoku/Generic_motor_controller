#include "mc_persistent_store.h"
#include "mc_flash_port.h"
#include <string.h>

/** @file mc_persistent_store.c
 *  @brief Generic A/B NV record store (HAL-free). See ADR-010.
 */

#define MC_CRC32_POLY 0xEDB88320u
#define MC_HDR_SIZE   ((uint32_t)sizeof(MC_ParamStoreHeader_t))           /* 16 */
#define MC_CRC_PREFIX (MC_HDR_SIZE - (uint32_t)sizeof(uint32_t))          /* 12: header minus crc32 */

/* Active record held in RAM. */
static uint8_t  s_payload[MC_PARAM_STORE_MAX_PAYLOAD];
static uint16_t s_size;
static uint32_t s_seq;
static uint8_t  s_active_slot;
static bool     s_has_valid;
static bool     s_dirty;

/* CRC32 (reflected, poly 0xEDB88320) over the header prefix (12 bytes) + payload. */
static uint32_t calc_crc(const MC_ParamStoreHeader_t *hdr, const uint8_t *payload, uint16_t size)
{
    const uint8_t *h = (const uint8_t *)hdr;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < MC_CRC_PREFIX; i++)
    {
        crc ^= h[i];
        for (uint8_t j = 0u; j < 8u; j++)
        {
            crc = (crc & 1u) ? ((crc >> 1) ^ MC_CRC32_POLY) : (crc >> 1);
        }
    }
    for (uint16_t i = 0u; i < size; i++)
    {
        crc ^= payload[i];
        for (uint8_t j = 0u; j < 8u; j++)
        {
            crc = (crc & 1u) ? ((crc >> 1) ^ MC_CRC32_POLY) : (crc >> 1);
        }
    }
    return ~crc;
}

/* Validate a slot; on success return its payload + seq via outputs. */
static bool slot_valid(uint8_t slot, uint8_t *payload_out, uint16_t *size_out, uint32_t *seq_out)
{
    MC_ParamStoreHeader_t hdr;
    MC_FlashPort_Read(slot, 0u, &hdr, MC_HDR_SIZE);

    if (hdr.magic != MC_PARAM_STORE_MAGIC)        { return false; }
    if (hdr.version != MC_PARAM_STORE_VERSION)    { return false; }
    if ((hdr.payload_size == 0u) ||
        (hdr.payload_size > MC_PARAM_STORE_MAX_PAYLOAD)) { return false; }

    uint8_t pl[MC_PARAM_STORE_MAX_PAYLOAD];
    MC_FlashPort_Read(slot, MC_HDR_SIZE, pl, hdr.payload_size);

    if (calc_crc(&hdr, pl, hdr.payload_size) != hdr.crc32) { return false; }

    memcpy(payload_out, pl, hdr.payload_size);
    *size_out = hdr.payload_size;
    *seq_out  = hdr.seq;
    return true;
}

MC_Status_t MC_PersistentStore_Init(void)
{
    s_has_valid   = false;
    s_dirty       = false;
    s_active_slot = 0u;
    s_seq         = 0u;
    s_size        = 0u;

    for (uint8_t slot = 0u; slot < MC_FLASH_NV_SLOTS; slot++)
    {
        uint8_t  pl[MC_PARAM_STORE_MAX_PAYLOAD];
        uint16_t size;
        uint32_t seq;
        if (slot_valid(slot, pl, &size, &seq))
        {
            if (!s_has_valid || (seq >= s_seq))
            {
                memcpy(s_payload, pl, size);
                s_size        = size;
                s_seq         = seq;
                s_active_slot = slot;
                s_has_valid   = true;
            }
        }
    }
    return s_has_valid ? MC_OK : MC_ERR_NOT_FOUND;
}

bool MC_PersistentStore_HasValid(void)   { return s_has_valid; }
bool MC_PersistentStore_SavePending(void){ return s_dirty; }

MC_Status_t MC_PersistentStore_Read(void *dst, uint16_t size)
{
    if (!s_has_valid)  { return MC_ERR_NOT_FOUND; }
    /* Backward-compatible read (ADR-070): accept a caller struct that has GROWN since the record was
       written (size > stored) -- front-copy the stored bytes and zero the grown tail. Valid only
       because the growable field (od_blob) is LAST in MC_Params_t, so earlier fields keep their
       offsets. A caller SMALLER than the stored payload is rejected (can't safely map / downgrade). */
    if (size < s_size) { return MC_ERR_RANGE; }
    memcpy(dst, s_payload, s_size);
    if (size > s_size) { memset((uint8_t *)dst + s_size, 0, (size_t)(size - s_size)); }
    return MC_OK;
}

MC_Status_t MC_PersistentStore_RequestSave(const void *src, uint16_t size)
{
    if ((size == 0u) || (size > MC_PARAM_STORE_MAX_PAYLOAD)) { return MC_ERR_RANGE; }
    memcpy(s_payload, src, size);
    s_size  = size;
    s_dirty = true;
    return MC_OK;
}

void MC_PersistentStore_ServiceSlow(void)
{
    if (!s_dirty) { return; }

    const uint8_t  target  = s_has_valid ? (uint8_t)(s_active_slot ^ 1u) : 0u;
    const uint32_t new_seq = s_seq + 1u;

    /* Build the record (header + payload) in RAM, padded to a multiple of 8. */
    static uint8_t rec[MC_PARAM_STORE_MAX_PAYLOAD + 16u + 8u];
    MC_ParamStoreHeader_t hdr;
    hdr.magic        = MC_PARAM_STORE_MAGIC;
    hdr.version      = MC_PARAM_STORE_VERSION;
    hdr.payload_size = s_size;
    hdr.seq          = new_seq;
    hdr.crc32        = calc_crc(&hdr, s_payload, s_size);

    memcpy(rec, &hdr, MC_HDR_SIZE);
    memcpy(rec + MC_HDR_SIZE, s_payload, s_size);
    uint32_t total = MC_HDR_SIZE + s_size;
    while ((total % 8u) != 0u) { rec[total++] = 0xFFu; }

    if (MC_FlashPort_EraseSlot(target) != MC_OK) { return; }
    if (MC_FlashPort_Program(target, 0u, rec, total) != MC_OK) { return; }

    /* Verify the written record before switching the active slot. */
    uint8_t  vpl[MC_PARAM_STORE_MAX_PAYLOAD];
    uint16_t vsize;
    uint32_t vseq;
    if (!slot_valid(target, vpl, &vsize, &vseq) || (vseq != new_seq)) { return; }

    s_active_slot = target;
    s_seq         = new_seq;
    s_has_valid   = true;
    s_dirty       = false;
}

MC_Status_t MC_PersistentStore_FactoryReset(void)
{
    MC_Status_t rc = MC_OK;
    for (uint8_t slot = 0u; slot < MC_FLASH_NV_SLOTS; slot++)
    {
        if (MC_FlashPort_EraseSlot(slot) != MC_OK) { rc = MC_ERR_FAULT; }
    }
    s_has_valid = false;
    s_dirty     = false;
    s_seq       = 0u;
    return rc;
}
