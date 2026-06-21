#include "mc_od.h"
#include "mc_od_store.h"
#include <string.h>

/** @file mc_od.c
 *  @brief Static object-dictionary engine + table (tuning subset). See ADR-015.
 *
 *  Typed lookup/read/write with access, type, size and range checks and optional callbacks.
 *  The table is a hand-maintained subset of the shared canonical map
 *  (../Lightweight_CMC/Interface/mc_if_od.h, MC_IF_OD_OBJECTS) bound to @ref g_od; keep the two
 *  in sync (and log any contract change per the Interface CHANGELOG). The CiA-402 standard
 *  objects (0x6xxx) and the 0x2A00 telemetry map are added with the SPI transport + mode manager.
 */

MC_OdStore_t g_od;

/* Convenience for table entries. */
#define OD_F32(idx, sub, field, acc) \
    { (idx), (sub), MC_OD_TYPE_FLOAT32, (acc), &g_od.field, 4u, 0.0f, 0.0f, false, true, 0, 0 }
#define OD_F32_RO(idx, sub, field) \
    { (idx), (sub), MC_OD_TYPE_FLOAT32, MC_OD_ACCESS_RO, &g_od.field, 4u, 0.0f, 0.0f, true, false, 0, 0 }
#define OD_U8(idx, sub, field, lo, hi) \
    { (idx), (sub), MC_OD_TYPE_U8, MC_OD_ACCESS_RW, &g_od.field, 1u, (lo), (hi), false, true, 0, 0 }
#define OD_U16(idx, sub, field, lo, hi, persist) \
    { (idx), (sub), MC_OD_TYPE_U16, MC_OD_ACCESS_RW, &g_od.field, 2u, (lo), (hi), false, (persist), 0, 0 }

static const MC_OdEntry_t s_od_table[] =
{
    /* 0x2000 axis / motor model */
    OD_F32(0x2000, 1, motor_kt_nm_per_a,   MC_OD_ACCESS_RW),
    OD_F32(0x2000, 2, motor_inertia_kg_m2, MC_OD_ACCESS_RW),
    OD_U16(0x2000, 5, motor_pole_pairs, 1.0f, 50.0f, true),
    /* 0x2200 position controller */
    OD_F32(0x2200, 1, pos_kp, MC_OD_ACCESS_RW),
    OD_F32(0x2200, 2, pos_ki, MC_OD_ACCESS_RW),
    OD_F32(0x2200, 3, pos_kd, MC_OD_ACCESS_RW),
    /* 0x2300 velocity controller + telemetry */
    OD_F32(0x2300, 1, vel_kp, MC_OD_ACCESS_RW),
    OD_F32(0x2300, 2, vel_ki, MC_OD_ACCESS_RW),
    OD_F32(0x2300, 3, vel_kd, MC_OD_ACCESS_RW),
    OD_F32(0x2300, 4, vel_current_limit_a, MC_OD_ACCESS_RW),
    OD_F32_RO(0x2310, 1, tlm_vel_demand_rad_s),
    OD_F32_RO(0x2310, 2, tlm_vel_actual_rad_s),
    OD_F32_RO(0x2310, 3, tlm_vel_iq_cmd_a),
    /* 0x2400 current/FOC gains + telemetry */
    OD_F32(0x2400, 1, foc_id_kp, MC_OD_ACCESS_RW),
    OD_F32(0x2400, 2, foc_id_ki, MC_OD_ACCESS_RW),
    OD_F32(0x2400, 3, foc_iq_kp, MC_OD_ACCESS_RW),
    OD_F32(0x2400, 4, foc_iq_ki, MC_OD_ACCESS_RW),
    OD_F32(0x2400, 5, foc_voltage_limit_v, MC_OD_ACCESS_RW),
    OD_F32_RO(0x2410, 1, tlm_id_meas_a),
    OD_F32_RO(0x2410, 2, tlm_iq_meas_a),
    OD_F32_RO(0x2410, 3, tlm_vd_v),
    OD_F32_RO(0x2410, 4, tlm_vq_v),
    OD_F32_RO(0x2410, 5, tlm_electrical_angle_rad),
    /* 0x2500 encoder / estimator + telemetry */
    OD_F32(0x2500, 1, est_electrical_offset_rad, MC_OD_ACCESS_RW),
    OD_F32(0x2500, 2, est_velocity_filter_hz,    MC_OD_ACCESS_RW),
    OD_F32(0x2500, 3, est_obs_kp, MC_OD_ACCESS_RW),
    OD_F32(0x2500, 4, est_obs_ki, MC_OD_ACCESS_RW),
    OD_F32(0x2500, 5, est_obs_kv, MC_OD_ACCESS_RW),
    OD_U8 (0x2500, 6, est_use_observer, 0.0f, 1.0f),
    OD_F32_RO(0x2510, 1, tlm_mech_position_rad),
    OD_F32_RO(0x2510, 2, tlm_mech_velocity_rad_s),
    /* 0x2600 faults / limits */
    OD_F32(0x2600, 2, current_trip_a, MC_OD_ACCESS_RW),
    OD_F32_RO(0x2600, 3, tlm_bus_voltage_v),
    /* 0x2700 calibration / 0x2800 persistence (command + status) */
    OD_U16(0x2700, 1, cal_command, 0.0f, 0.0f, false),
    OD_U16(0x2800, 1, store_save_command, 0.0f, 0.0f, false),
    /* 0x2900 commissioning / test injection (placeholders until wired to the inject path) */
    OD_U8 (0x2900, 1, inject_enable, 0.0f, 1.0f),
    OD_U8 (0x2900, 2, inject_target, 0.0f, 3.0f),
    OD_F32(0x2900, 3, inject_step_amplitude, MC_OD_ACCESS_RW),
    OD_U8 (0x2900, 4, inject_step_trigger, 0.0f, 1.0f),
};
#define MC_OD_TABLE_COUNT (sizeof(s_od_table) / sizeof(s_od_table[0]))

static uint32_t type_size(MC_OdType_t t)
{
    switch (t)
    {
        case MC_OD_TYPE_U8:  case MC_OD_TYPE_I8:  return 1u;
        case MC_OD_TYPE_U16: case MC_OD_TYPE_I16: return 2u;
        default:                                  return 4u;
    }
}

static float to_float(MC_OdType_t t, const void *p)
{
    switch (t)
    {
        case MC_OD_TYPE_U8:      return (float)(*(const uint8_t *)p);
        case MC_OD_TYPE_U16:     return (float)(*(const uint16_t *)p);
        case MC_OD_TYPE_U32:     return (float)(*(const uint32_t *)p);
        case MC_OD_TYPE_I8:      return (float)(*(const int8_t *)p);
        case MC_OD_TYPE_I16:     return (float)(*(const int16_t *)p);
        case MC_OD_TYPE_I32:     return (float)(*(const int32_t *)p);
        default:                 return *(const float *)p;
    }
}

void MC_OdStore_LoadDefaults(void)
{
    memset(&g_od, 0, sizeof(g_od));

    /* Motor model (Maxon 500267) */
    g_od.motor_kt_nm_per_a   = 0.231f;
    g_od.motor_inertia_kg_m2 = 0.000506f;
    g_od.motor_pole_pairs    = 11u;

    /* Position controller (D3) */
    g_od.pos_kp = 5.0f;

    /* Velocity controller (proven gains, torque form: 150*Kt, 1000*Kt) */
    g_od.vel_kp = 150.0f * 0.231f;
    g_od.vel_ki = 1000.0f * 0.231f;
    g_od.vel_kd = 0.0f;
    g_od.vel_current_limit_a = 2.5f;

    /* Current/FOC loops */
    g_od.foc_id_kp = 1.7f;   g_od.foc_id_ki = 1700.0f;
    g_od.foc_iq_kp = 1.7f;   g_od.foc_iq_ki = 1700.0f;
    g_od.foc_voltage_limit_v = 13.8f;

    /* Encoder / estimator */
    g_od.est_electrical_offset_rad = 0.0f;   /* loaded from flash by the persistence store */
    g_od.est_velocity_filter_hz    = 20.0f;
    g_od.est_obs_kp = 40000.0f; g_od.est_obs_ki = 0.0f; g_od.est_obs_kv = 200.0f;
    g_od.est_use_observer = 1u;

    g_od.current_trip_a = 3.0f;
}

void MC_Od_Init(void)
{
    MC_OdStore_LoadDefaults();
}

const MC_OdEntry_t *MC_Od_Find(uint16_t index, uint8_t subindex)
{
    for (uint32_t i = 0u; i < MC_OD_TABLE_COUNT; i++)
    {
        if ((s_od_table[i].index == index) && (s_od_table[i].subindex == subindex))
        {
            return &s_od_table[i];
        }
    }
    return 0;
}

MC_OdStatus_t MC_Od_Read(uint16_t index, uint8_t subindex, void *dst,
                         uint32_t size_bytes, MC_OdType_t expected_type)
{
    const MC_OdEntry_t *e = MC_Od_Find(index, subindex);
    if (e == 0)                              { return MC_OD_ERR_NOT_FOUND; }
    if ((e->access & MC_OD_ACCESS_RO) == 0u) { return MC_OD_ERR_ACCESS; }   /* not readable */
    if (e->type != expected_type)            { return MC_OD_ERR_TYPE; }
    const uint32_t n = type_size(e->type);
    if (size_bytes < n)                      { return MC_OD_ERR_SIZE; }
    if (e->read_cb != 0)                     { return e->read_cb(dst, n); }
    memcpy(dst, e->data, n);
    return MC_OD_OK;
}

MC_OdStatus_t MC_Od_Write(uint16_t index, uint8_t subindex, const void *src,
                          uint32_t size_bytes, MC_OdType_t expected_type)
{
    const MC_OdEntry_t *e = MC_Od_Find(index, subindex);
    if (e == 0)                              { return MC_OD_ERR_NOT_FOUND; }
    if ((e->access & MC_OD_ACCESS_WO) == 0u) { return MC_OD_ERR_ACCESS; }   /* not writable */
    if (e->type != expected_type)            { return MC_OD_ERR_TYPE; }
    const uint32_t n = type_size(e->type);
    if (size_bytes < n)                      { return MC_OD_ERR_SIZE; }
    if (e->max_value > e->min_value)         /* range enforced only when a window is set */
    {
        const float v = to_float(e->type, src);
        if ((v < e->min_value) || (v > e->max_value)) { return MC_OD_ERR_RANGE; }
    }
    if (e->write_cb != 0)                    { return e->write_cb(src, n); }
    memcpy(e->data, src, n);
    return MC_OD_OK;
}

MC_OdStatus_t MC_Od_ReadRaw(uint16_t index, uint8_t subindex, void *dst, uint32_t cap,
                            MC_OdType_t *out_type, uint32_t *out_len)
{
    const MC_OdEntry_t *e = MC_Od_Find(index, subindex);
    if (e == 0)                              { return MC_OD_ERR_NOT_FOUND; }
    if ((e->access & MC_OD_ACCESS_RO) == 0u) { return MC_OD_ERR_ACCESS; }
    const uint32_t n = type_size(e->type);
    if (cap < n)                             { return MC_OD_ERR_SIZE; }
    if (e->read_cb != 0)
    {
        const MC_OdStatus_t s = e->read_cb(dst, n);
        if (s != MC_OD_OK) { return s; }
    }
    else
    {
        memcpy(dst, e->data, n);
    }
    if (out_type != 0) { *out_type = e->type; }
    if (out_len  != 0) { *out_len  = n; }
    return MC_OD_OK;
}

MC_OdStatus_t MC_Od_ReadU16(uint16_t index, uint8_t subindex, uint16_t *value)
{ return MC_Od_Read(index, subindex, value, sizeof(*value), MC_OD_TYPE_U16); }
MC_OdStatus_t MC_Od_WriteU16(uint16_t index, uint8_t subindex, uint16_t value)
{ return MC_Od_Write(index, subindex, &value, sizeof(value), MC_OD_TYPE_U16); }
MC_OdStatus_t MC_Od_ReadI32(uint16_t index, uint8_t subindex, int32_t *value)
{ return MC_Od_Read(index, subindex, value, sizeof(*value), MC_OD_TYPE_I32); }
MC_OdStatus_t MC_Od_WriteI32(uint16_t index, uint8_t subindex, int32_t value)
{ return MC_Od_Write(index, subindex, &value, sizeof(value), MC_OD_TYPE_I32); }
MC_OdStatus_t MC_Od_ReadFloat(uint16_t index, uint8_t subindex, float *value)
{ return MC_Od_Read(index, subindex, value, sizeof(*value), MC_OD_TYPE_FLOAT32); }
MC_OdStatus_t MC_Od_WriteFloat(uint16_t index, uint8_t subindex, float value)
{ return MC_Od_Write(index, subindex, &value, sizeof(value), MC_OD_TYPE_FLOAT32); }
