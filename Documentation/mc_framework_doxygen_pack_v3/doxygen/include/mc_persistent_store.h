#ifndef MC_PERSISTENT_STORE_H
#define MC_PERSISTENT_STORE_H
#include "mc_types.h"

typedef struct { uint32_t magic; uint16_t version; uint16_t length; uint32_t crc; } MC_ParamStoreHeader_t;
MC_Status_t MC_ParamStore_Load(void);
MC_Status_t MC_ParamStore_Save(void);
MC_Status_t MC_ParamStore_FactoryReset(void);
#endif
