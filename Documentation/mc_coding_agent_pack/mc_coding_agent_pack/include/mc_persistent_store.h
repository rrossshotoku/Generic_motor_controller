#ifndef MC_PERSISTENT_STORE_H
#define MC_PERSISTENT_STORE_H
#include "mc_types.h"

/** @ingroup mc_persistence */
#define MC_PARAM_STORE_MAGIC (0x4D435046u) /* MCPF */
#define MC_PARAM_STORE_VERSION (1u)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size_bytes;
    uint32_t crc32;
} MC_ParamStoreHeader_t;

void MC_PersistentStore_Init(void);
MC_Status_t MC_PersistentStore_Load(void);
MC_Status_t MC_PersistentStore_RequestSave(void);
MC_Status_t MC_PersistentStore_FactoryReset(void);
void MC_PersistentStore_ServiceSlow(void);

#endif
