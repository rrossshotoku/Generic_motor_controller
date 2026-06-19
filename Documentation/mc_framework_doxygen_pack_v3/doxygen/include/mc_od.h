#ifndef MC_OD_H
#define MC_OD_H
#include "mc_types.h"

typedef enum { MC_OD_U8, MC_OD_U16, MC_OD_U32, MC_OD_I8, MC_OD_I16, MC_OD_I32, MC_OD_F32 } MC_OdType_t;
typedef enum { MC_OD_RO = 1, MC_OD_WO = 2, MC_OD_RW = 3 } MC_OdAccess_t;
typedef enum { MC_OD_OK = 0, MC_OD_ERR_NOT_FOUND, MC_OD_ERR_ACCESS, MC_OD_ERR_TYPE, MC_OD_ERR_RANGE, MC_OD_ERR_CALLBACK } MC_OdStatus_t;

typedef MC_OdStatus_t (*MC_OdReadCallback_t)(void *dst, uint32_t size);
typedef MC_OdStatus_t (*MC_OdWriteCallback_t)(const void *src, uint32_t size);

typedef struct {
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

MC_OdStatus_t MC_Od_Read(uint16_t index, uint8_t subindex, MC_OdType_t type, void *dst, uint32_t size);
MC_OdStatus_t MC_Od_Write(uint16_t index, uint8_t subindex, MC_OdType_t type, const void *src, uint32_t size);
MC_OdStatus_t MC_Od_ReadU16(uint16_t index, uint8_t subindex, uint16_t *value);
MC_OdStatus_t MC_Od_WriteU16(uint16_t index, uint8_t subindex, uint16_t value);
MC_OdStatus_t MC_Od_ReadF32(uint16_t index, uint8_t subindex, float *value);
MC_OdStatus_t MC_Od_WriteF32(uint16_t index, uint8_t subindex, float value);
#endif
