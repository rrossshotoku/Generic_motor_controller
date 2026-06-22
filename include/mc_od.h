#ifndef MC_OD_H
#define MC_OD_H
#include "mc_types.h"

/** @file mc_od.h
 *  @brief Static CiA 402-style object dictionary with typed access.
 *  @ingroup mc_od
 */

typedef enum
{
    MC_OD_TYPE_U8,
    MC_OD_TYPE_U16,
    MC_OD_TYPE_U32,
    MC_OD_TYPE_I8,
    MC_OD_TYPE_I16,
    MC_OD_TYPE_I32,
    MC_OD_TYPE_FLOAT32
} MC_OdType_t;

typedef enum
{
    MC_OD_ACCESS_RO = 1u,
    MC_OD_ACCESS_WO = 2u,
    MC_OD_ACCESS_RW = 3u
} MC_OdAccess_t;

typedef enum
{
    MC_OD_OK = 0,
    MC_OD_ERR_NOT_FOUND,   /* index not present */
    MC_OD_ERR_ACCESS,
    MC_OD_ERR_TYPE,
    MC_OD_ERR_RANGE,
    MC_OD_ERR_SIZE,
    MC_OD_ERR_CALLBACK,
    MC_OD_ERR_NO_SUB,      /* index present, subindex absent (-> wire MC_IF_OD_ERR_NO_SUB) */
    MC_OD_ERR_NOT_READY    /* owning module not ready (-> wire MC_IF_OD_ERR_NOT_READY) */
} MC_OdStatus_t;

typedef MC_OdStatus_t (*MC_OdReadCallback_t)(void *dst, uint32_t size_bytes);
typedef MC_OdStatus_t (*MC_OdWriteCallback_t)(const void *src, uint32_t size_bytes);

typedef struct
{
    uint16_t index;
    uint8_t subindex;
    MC_OdType_t type;
    MC_OdAccess_t access;
    void *data;
    uint32_t size_bytes;
    float min_value;
    float max_value;
    bool pdo_mappable;
    bool persistent;
    MC_OdReadCallback_t read_cb;
    MC_OdWriteCallback_t write_cb;
} MC_OdEntry_t;

void MC_Od_Init(void);
const MC_OdEntry_t *MC_Od_Find(uint16_t index, uint8_t subindex);
MC_OdStatus_t MC_Od_Read(uint16_t index, uint8_t subindex, void *dst, uint32_t size_bytes, MC_OdType_t expected_type);
MC_OdStatus_t MC_Od_Write(uint16_t index, uint8_t subindex, const void *src, uint32_t size_bytes, MC_OdType_t expected_type);
MC_OdStatus_t MC_Od_ReadU16(uint16_t index, uint8_t subindex, uint16_t *value);
MC_OdStatus_t MC_Od_WriteU16(uint16_t index, uint8_t subindex, uint16_t value);
MC_OdStatus_t MC_Od_ReadI32(uint16_t index, uint8_t subindex, int32_t *value);
MC_OdStatus_t MC_Od_WriteI32(uint16_t index, uint8_t subindex, int32_t value);
MC_OdStatus_t MC_Od_ReadFloat(uint16_t index, uint8_t subindex, float *value);
MC_OdStatus_t MC_Od_WriteFloat(uint16_t index, uint8_t subindex, float value);

/** @brief Raw read of an entry's bytes (for the SPI OD response and telemetry gather).
 *  Copies the entry's native size into @p dst (capacity @p cap); reports the actual type/len. */
MC_OdStatus_t MC_Od_ReadRaw(uint16_t index, uint8_t subindex, void *dst, uint32_t cap,
                            MC_OdType_t *out_type, uint32_t *out_len);

/** @brief Serialize every persistent (MC_IF_F_PERSIST) OD entry into @p buf as
 *  {index(LE16), subindex, len, value} records, for the flash params store (ADR-023).
 *  @return bytes written (stops early, without overflowing, if @p cap is reached). */
uint16_t MC_Od_GatherPersistent(uint8_t *buf, uint16_t cap);

/** @brief Restore persistent OD entries previously serialized by MC_Od_GatherPersistent.
 *  Records whose index/sub is unknown or whose size no longer matches are skipped. */
void MC_Od_RestorePersistent(const uint8_t *buf, uint16_t len);

#endif
