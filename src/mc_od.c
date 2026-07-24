#include "mc_od.h"
#include "mc_od_store.h"
#include "mc_if_od.h"        /* shared canonical map: MC_IF_OD_OBJECTS(X) + owner/type/access/flags */
#include <string.h>

/** @file mc_od.c
 *  @brief Static object-dictionary engine + table. See ADR-015 (engine), ADR-019 (generation).
 *
 *  Typed lookup/read/write with access, type, size and range checks and optional callbacks.
 *
 *  The table is GENERATED from the shared canonical map MC_IF_OD_OBJECTS(X)
 *  (../Generic_axis_controller/Generic_axis_controller/Interface/mc_if_od.h), filtered to MC_IF_OWNER_MOTOR entries and bound to
 *  @ref g_od by field name. The contract is therefore the single source of truth, and drift is
 *  caught at COMPILE TIME: a motor-owned entry whose `name` has no matching g_od field fails to
 *  compile (&g_od.<name>), and type/access/PDO/PERSIST follow the contract automatically. CMC-owned
 *  0x3xxx entries expand to nothing here (-> NO_OBJECT at runtime via od_notfound()).
 *
 *  Two things are motor policy, not in the contract, and live alongside the generator below:
 *  write-range windows (set in MC_Od_Init), and the handful of owner=MOTOR entries handled
 *  elsewhere (0x2A00 telemetry map -> mc_comms, REQ-0004 deferred), which are skipped.
 */

MC_OdStore_t g_od;

/* ===== Table generation from MC_IF_OD_OBJECTS(X) (ADR-019) =====
 * The generated rows cast the contract's MC_IF_T_* / MC_IF_A_* straight to the engine's
 * MC_OdType_t / MC_OdAccess_t. That is valid only while the enums stay value-aligned (mc_if_od.h
 * documents that they do); these guards make any future drift a compile error. */
_Static_assert((int)MC_IF_T_U8  == (int)MC_OD_TYPE_U8  && (int)MC_IF_T_U16 == (int)MC_OD_TYPE_U16 &&
               (int)MC_IF_T_U32 == (int)MC_OD_TYPE_U32 && (int)MC_IF_T_I8  == (int)MC_OD_TYPE_I8  &&
               (int)MC_IF_T_I16 == (int)MC_OD_TYPE_I16 && (int)MC_IF_T_I32 == (int)MC_OD_TYPE_I32 &&
               (int)MC_IF_T_F32 == (int)MC_OD_TYPE_FLOAT32, "OD type enum drift vs Interface");
_Static_assert((int)MC_IF_A_RO == (int)MC_OD_ACCESS_RO && (int)MC_IF_A_WO == (int)MC_OD_ACCESS_WO &&
               (int)MC_IF_A_RW == (int)MC_OD_ACCESS_RW, "OD access enum drift vs Interface");

/* Owner=MOTOR entries handled OUTSIDE this table are skipped during generation -- one marker per
 * field name. 0x2A00:0 tlm_map_count: the telemetry map lives in mc_comms (REQ-0004 deferred). */
#define OD_SKIP_tlm_map_count   ~, 1

/* PROBE: OD_IS_SKIP(name) -> 1 if OD_SKIP_<name> is defined (as "~, 1"), else 0. */
#define OD_SECOND_(a, b, ...)   b
#define OD_IS_SKIP_(...)        OD_SECOND_(__VA_ARGS__, 0)
#define OD_IS_SKIP(name)        OD_IS_SKIP_(OD_SKIP_##name)
#define OD_PASTE_(a, b)         a##b
#define OD_PASTE(a, b)          OD_PASTE_(a, b)

/* Byte size from the contract type (the engine recomputes from type; filled for completeness). */
#define OD_TSIZE(t) ( (((t)==MC_IF_T_U8)||((t)==MC_IF_T_I8))   ? 1u : \
                      (((t)==MC_IF_T_U16)||((t)==MC_IF_T_I16)) ? 2u : 4u )

/* Emit one row: bind to &g_od.<name>, cast type/access, derive pdo/persist from the contract
 * flags. Ranges start as 0/0 (no check), set per policy in MC_Od_Init; no entry uses a callback. */
#define OD_EMIT_1(idx, sub, name, type, acc, flags)   /* skipped: handled elsewhere */
#define OD_EMIT_0(idx, sub, name, type, acc, flags) \
    { (idx), (sub), (MC_OdType_t)(type), (MC_OdAccess_t)(acc), &g_od.name, OD_TSIZE(type), \
      0.0f, 0.0f, (((flags) & MC_IF_F_PDO) != 0), (((flags) & MC_IF_F_PERSIST) != 0), 0, 0 },

/* Owner dispatch: MOTOR entries emit a row (unless skipped); CMC (0x3xxx) and BOOTLOADER (0x1F5x)
   entries vanish -- the app serves neither. Bootloader-owned entries belong to the separate bootloader
   binary (contract v5 / REQ-0015); the running app just skips them. */
#define OD_ROW_MC_IF_OWNER_CMC(idx, sub, name, type, acc, flags)        /* not built on the motor app */
#define OD_ROW_MC_IF_OWNER_BOOTLOADER(idx, sub, name, type, acc, flags) /* served by the bootloader binary, not the app */
#define OD_ROW_MC_IF_OWNER_MOTOR(idx, sub, name, type, acc, flags) \
    OD_PASTE(OD_EMIT_, OD_IS_SKIP(name))(idx, sub, name, type, acc, flags)
#define OD_ROW(idx, sub, name, type, acc, flags, owner) \
    OD_ROW_##owner(idx, sub, name, type, acc, flags)

/* Non-const: MC_Od_Init patches the few write-range windows (below). */
static MC_OdEntry_t s_od_table[] =
{
    MC_IF_OD_OBJECTS(OD_ROW)
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
    g_od.motor_kt_nm_per_a    = 0.231f;
    g_od.motor_inertia_kg_m2  = 0.000506f;
    g_od.motor_resistance_ohm = 0.844f;
    g_od.motor_inductance_h   = 0.00107f;
    g_od.motor_pole_pairs     = 11u;
    g_od.device_type          = 0x00020192u;   /* CiA-402 servo-drive profile */

    /* Position controller (D3) */
    g_od.pos_kp = 5.0f;
    g_od.velocity_ff_gain = 1.0f;   /* 0x2200:4 -- full velocity feedforward by default (ADR-031) */
    g_od.position_deadband_rad = 0.0f;   /* 0x2200:5 -- position-error deadband off by default (ADR-071) */
    g_od.on_target_window_rad  = 0.02f;  /* 0x2200:6 -- ON_TARGET window; a de-energised axis settles at the deadband edge (ADR-078) */

    /* Velocity controller (proven gains, torque form: 150*Kt, 1000*Kt) */
    g_od.vel_kp = 150.0f * 0.231f;
    g_od.vel_ki = 1000.0f * 0.231f;
    g_od.vel_kd = 0.0f;
    g_od.vel_current_limit_a = 2.5f;

    /* Thermal model (0x2100, ADR-065): off by default. thermal_derate_factor must read 1.0
       (not the memset 0) so the current-limit scaling is a no-op until a motor is configured;
       thermal_derate_start defaults to 0.85 (must NOT be the memset 0, else derate would start
       at x=0 and choke output immediately once the model is enabled). */
    g_od.thermal_derate_factor = 1.0f;
    g_od.thermal_derate_start  = 0.85f;
    g_od.vel_load_factor = 1.0f;   /* 0x2300:5 -- no load scaling by default (REQ-0014/ADR-034) */
    g_od.vel_accel_up = 0.0f;  g_od.vel_accel_dn = 0.0f;   /* 0x2300:6,7 velocity accel ramp off by default (ADR-042) */
    g_od.vel_accel_jerk = 0.0f;                            /* 0x2300:8 accel ramp-up jerk off (= plain accel ramp) */
    g_od.holding_enable = 1u;                              /* 0x2300:9 ADVISORY ONLY since ADR-072/REQ-0016 (CMC owns idle policy via op_mode); default 1 */
    g_od.jog_position_mode = 0u;                           /* 0x2300:10 default 0 = direct velocity jog (unchanged); 1 = position-integrated jog (ADR-062) */
    g_od.vel_stop_bleed_v_th   = 0.5f;                     /* 0x2300:11 bleed threshold [rad/s] (inert until enabled) (ADR-074) */
    g_od.vel_stop_bleed_factor = 1.0f;                     /* 0x2300:12 bleed speed = 1×ki (inert until enabled) (ADR-074) */
    g_od.vel_stop_bleed_enable = 0u;                       /* 0x2300:13 stop-integrator bleed OFF by default (ADR-074) */
    g_od.vel_accel_scurve = 0u;                            /* 0x2300:14 anticipatory S-curve ramp OFF by default (ADR-075) */
    g_od.accel_ff_gain = 1.0f;                             /* 0x2300:15 full acceleration feedforward by default (ADR-079) */

    /* Current/FOC loops */
    g_od.foc_id_kp = 1.7f;   g_od.foc_id_ki = 1700.0f;
    g_od.foc_iq_kp = 1.7f;   g_od.foc_iq_ki = 1700.0f;
    g_od.hb_cur_kp = 5.55f;  g_od.hb_cur_ki = 6300.0f;  /* brushed current PI gains, set directly (ADR-049) */
    g_od.foc_voltage_limit_v = 13.8f;
    g_od.current_demand_limit_a = 0.0f;   /* 0x2400:8 soft current-demand ceiling; 0 = disabled (ADR-069) */

    /* Encoder / estimator */
    g_od.est_electrical_offset_rad = 0.0f;   /* loaded from flash by the persistence store */
    g_od.est_velocity_filter_hz    = 20.0f;
    g_od.est_obs_kp = 40000.0f; g_od.est_obs_ki = 0.0f; g_od.est_obs_kv = 200.0f;
    g_od.est_obs_filter_alpha = 0.3f;   /* observer output LPF coeff (~57 Hz at 1 kHz); was hardcoded (ADR-003) */
    g_od.quad_counts_per_rev = 4000.0f; /* incremental quad: 4x lines -- SET TO YOUR ENCODER; sign flips count direction (ADR-052) */
    /* Stepped-sine current sweep defaults (ADR-047): 5..200 Hz, 5 Hz steps, 0.5 s dwell, 0.2 A AC, no bias. */
    g_od.freq_sweep_start_hz = 5.0f;  g_od.freq_sweep_end_hz = 200.0f; g_od.freq_sweep_step_hz = 5.0f;
    g_od.freq_sweep_dwell_s  = 0.5f;  g_od.freq_sweep_bias_a = 0.0f;   g_od.freq_sweep_amplitude_a = 0.2f;
    /* Current-command notch (ADR-048): off by default, centred 55 Hz / 30 Hz wide (covers ~40-70 Hz). */
    g_od.notch_enable = 0u;  g_od.notch_freq_hz = 55.0f;  g_od.notch_bandwidth_hz = 30.0f;
    g_od.est_use_observer = 1u;

    g_od.current_trip_a = 3.0f;
    g_od.max_velocity_rad_s = 0.0f;   /* motor envelope ceilings (ADR-040): 0 = disabled; set per board */
    g_od.max_accel_rad_s2   = 0.0f;
    g_od.pos_limit_lo_rad = 0.0f;  g_od.pos_limit_hi_rad = 0.0f;   /* soft limits off (lo>=hi); set manually (ADR-040) */
    g_od.max_jerk_rad_s3 = 500.0f;  g_od.traj_use_scurve = 0u;     /* S-curve planner off by default -> trapezoidal (ADR-045) */

    /* Electrical-alignment routine defaults (ADR-024). */
    g_od.cal_align_current_a = 1.0f;     /* d-axis align current [A] */
    g_od.cal_align_hold_ms   = 1500u;    /* drive/hold duration [ms] */
    g_od.home_velocity_rad_s = -0.3f;    /* 0x2700:6 homing approach velocity [rad/s], PERSISTED; sign = direction -- SET FOR YOUR AXIS (ADR-057) */
    g_od.home_current_a      = 2.0f;     /* 0x2700:7 homing: stall-detect current [A] -- SET FOR YOUR AXIS (ADR-057) */
    g_od.home_command        = 0u;       /* idle */
    g_od.mech_zero_set_rad   = 0.0f;     /* 0x2700:10 mech-zero target for SET_MECH_ZERO_AT (ADR-022) */
    g_od.home_status         = 0u;       /* MC_IF_HOME_IDLE */
}

/* Set the write-range window [lo,hi] for one entry (motor policy; ranges aren't in the contract).
 * A zero-width window (lo==hi) disables the check -- see MC_Od_Write. */
static void od_set_range(uint16_t index, uint8_t subindex, float lo, float hi)
{
    for (uint32_t i = 0u; i < MC_OD_TABLE_COUNT; i++)
    {
        if ((s_od_table[i].index == index) && (s_od_table[i].subindex == subindex))
        {
            s_od_table[i].min_value = lo;
            s_od_table[i].max_value = hi;
            return;
        }
    }
}

void MC_Od_Init(void)
{
    MC_OdStore_LoadDefaults();

    /* Write-range windows (motor-side policy; the shared contract carries no range info). */
    od_set_range(0x2000u, 5u, 1.0f, 50.0f);   /* motor_pole_pairs    */
    od_set_range(0x2500u, 6u, 0.0f,  1.0f);   /* est_use_observer    */
    od_set_range(0x2900u, 1u, 0.0f,  1.0f);   /* inject_enable       */
    od_set_range(0x2900u, 2u, 0.0f,  3.0f);   /* inject_target       */
    od_set_range(0x2900u, 4u, 0.0f,  1.0f);   /* inject_step_trigger */
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

/* Distinguish "no such subindex" (index present) from "no such object" (REQ-0002). */
static MC_OdStatus_t od_notfound(uint16_t index)
{
    for (uint32_t i = 0u; i < MC_OD_TABLE_COUNT; i++)
    {
        if (s_od_table[i].index == index) { return MC_OD_ERR_NO_SUB; }
    }
    return MC_OD_ERR_NOT_FOUND;
}

MC_OdStatus_t MC_Od_Read(uint16_t index, uint8_t subindex, void *dst,
                         uint32_t size_bytes, MC_OdType_t expected_type)
{
    const MC_OdEntry_t *e = MC_Od_Find(index, subindex);
    if (e == 0)                              { return od_notfound(index); }
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
    if (e == 0)                              { return od_notfound(index); }
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
    if (e == 0)                              { return od_notfound(index); }
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

/* ===== Persistent-entry serialization (ADR-023): the params store gathers every MC_IF_F_PERSIST
   OD entry on save and restores them on boot, so the `persistent` flag finally means something. ===== */

/* Recurrence guard (ADR-070): true if the last gather ran out of buffer and DROPPED persistent
   entries -- the silent failure that hid the ADR-044 and ADR-067 truncations. Mirrored to the watch
   window so an overflow is visible instead of manifesting as "a setting won't persist". */
static bool s_persist_truncated = false;
bool MC_Od_PersistTruncated(void) { return s_persist_truncated; }

uint16_t MC_Od_GatherPersistent(uint8_t *buf, uint16_t cap)
{
    uint16_t n = 0u;
    s_persist_truncated = false;
    for (uint32_t i = 0u; i < MC_OD_TABLE_COUNT; i++)
    {
        const MC_OdEntry_t *e = &s_od_table[i];
        if (!e->persistent) { continue; }
        const uint32_t sz = type_size(e->type);
        if (((uint32_t)n + 4u + sz) > (uint32_t)cap) { s_persist_truncated = true; break; }   /* index(2)+sub(1)+len(1)+value */
        buf[n++] = (uint8_t)(e->index & 0xFFu);
        buf[n++] = (uint8_t)(e->index >> 8);
        buf[n++] = e->subindex;
        buf[n++] = (uint8_t)sz;
        memcpy(&buf[n], e->data, sz);
        n = (uint16_t)(n + sz);
    }
    return n;
}

void MC_Od_RestorePersistent(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0u;
    while (((uint32_t)i + 4u) <= (uint32_t)len)
    {
        const uint16_t index = (uint16_t)((uint16_t)buf[i] | ((uint16_t)buf[i + 1u] << 8));
        const uint8_t  sub   = buf[i + 2u];
        const uint8_t  sz    = buf[i + 3u];
        i = (uint16_t)(i + 4u);
        if (((uint32_t)i + sz) > (uint32_t)len) { break; }
        const MC_OdEntry_t *e = MC_Od_Find(index, sub);
        if ((e != 0) && (type_size(e->type) == sz))
        {
            (void)MC_Od_Write(index, sub, &buf[i], sz, e->type);   /* honours access/range checks */
        }
        i = (uint16_t)(i + sz);
    }
}
