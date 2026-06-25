#ifndef MC_OD_STORE_H
#define MC_OD_STORE_H
#include "mc_types.h"

/** @file mc_od_store.h
 *  @brief Backing storage for the object-dictionary entries (the live OD-exposed values).
 *  @ingroup mc_od
 *
 *  The OD table (mc_od.c) binds its entries to fields of @ref g_od. Gains/config/commands are RW
 *  (written via OD, seeded with defaults, applied to the live controllers by the scheduler);
 *  telemetry is RO (mirrored from the live control state each cycle). This is the first,
 *  tuning-focused subset of the shared map (../Lightweight_CMC/Interface/mc_if_od.h); the
 *  CiA-402 standard objects and the 0x2A00 telemetry map land with the SPI-slave transport and
 *  the mode manager. See ADR-015.
 */
typedef struct
{
    /* --- Gains / config (RW) --- */
    float    pos_kp, pos_ki, pos_kd;                 /* 0x2200:1-3 */
    float    velocity_ff_gain;                       /* 0x2200:4 -- position-cascade velocity FF ratio (ADR-031) */
    float    vel_kp, vel_ki, vel_kd;                 /* 0x2300 */
    float    vel_current_limit_a;                    /* 0x2300:4 */
    float    vel_load_factor;                        /* 0x2300:5 -- operator load multiplier on vel kp/ki (REQ-0014) */
    float    foc_id_kp, foc_id_ki, foc_iq_kp, foc_iq_ki, foc_voltage_limit_v; /* 0x2400 */
    float    est_electrical_offset_rad;              /* 0x2500:1 */
    float    est_velocity_filter_hz;                 /* 0x2500:2 */
    float    est_obs_kp, est_obs_ki, est_obs_kv;     /* 0x2500:3..5 */
    uint8_t  est_use_observer;                       /* 0x2500:6 */
    float    current_trip_a;                         /* 0x2600:2 */
    float    motor_kt_nm_per_a, motor_inertia_kg_m2; /* 0x2000:1,2 */
    uint16_t motor_pole_pairs;                       /* 0x2000:5 */

    /* --- Commands (RW; placeholders until wired to mode manager / inject path) --- */
    uint8_t  inject_enable, inject_target, inject_step_trigger; /* 0x2900 */
    float    inject_step_amplitude;
    /* --- 0x2910 loop-tuning test-signal overlay (ADR-030) --- */
    uint8_t  test_mode;              /* 0x2910:1 (MC_IF_TEST_MODE_*) */
    float    test_amplitude;         /* 0x2910:2 (rad/s or rad, per test_mode) */
    float    test_rate;              /* 0x2910:3 (rad/s^2 or rad/s; 0 = step) */
    float    test_dwell_s;           /* 0x2910:4 */
    uint8_t  test_continuous;        /* 0x2910:5 (0 one-shot / 1 alternating) */
    uint16_t test_trigger;           /* 0x2910:6 (write 1 to fire) */
    uint8_t  test_active;            /* 0x2910:7 RO */
    float    test_signal;            /* 0x2910:8 RO PDO -- raw signal-generator output (graphable) */
    float    test_pause_s;           /* 0x2910:9 -- inter-pulse pause [s] (continuous mode) */
    float    test_max_accel;         /* 0x2910:10 -- position-tuning accel limit [rad/s^2]; 0 = off (ADR-032) */
    uint16_t cal_command;                            /* 0x2700:1 */
    float    cal_align_current_a;                     /* 0x2700:3 electrical-align current [A] (PERSIST) */
    uint16_t cal_align_hold_ms;                       /* 0x2700:4 electrical-align hold [ms] (PERSIST) */
    uint16_t store_save_command;                     /* 0x2800:1 */

    /* --- Telemetry (RO; mirrored from the live control state) --- */
    float    tlm_vel_demand_rad_s, tlm_vel_actual_rad_s, tlm_vel_iq_cmd_a;   /* 0x2310 */
    float    tlm_id_meas_a, tlm_iq_meas_a, tlm_vd_v, tlm_vq_v, tlm_electrical_angle_rad; /* 0x2410 */
    float    tlm_mech_position_rad, tlm_mech_velocity_rad_s;                 /* 0x2510:1,2 */
    float    tlm_pos_demand_rad;                                            /* 0x2510:3 PDO -- absolute (home-relative) position demand */
    float    tlm_bus_voltage_v;                                              /* 0x2600:3 */
    uint16_t cal_status, store_status;
    uint16_t cal_done_flags;         /* 0x2700:5 RO -- calibration-completeness bitfield (ADR-026) */

    /* --- REQ-0003 manufacturer additions --- */
    float    motor_resistance_ohm;   /* 0x2000:3 */
    float    motor_inductance_h;     /* 0x2000:4 */
    uint32_t fault_flags;            /* 0x2600:1 */
    uint16_t store_factory_reset;    /* 0x2800:3 */

    /* --- CiA-402 standard objects (REQ-0001). RW = stored (mode manager applies later);
       RO = mirrored from live state (scaled) by the scheduler. --- */
    uint32_t device_type;            /* 0x1000 RO */
    uint8_t  error_register;         /* 0x1001 RO */
    uint16_t error_code;             /* 0x603F RO */
    uint16_t controlword;            /* 0x6040 RW */
    uint16_t statusword;             /* 0x6041 RO */
    int8_t   modes_of_operation;         /* 0x6060 RW */
    int8_t   modes_of_operation_display; /* 0x6061 RO (name matches the contract for OD generation) */
    int32_t  target_position;        /* 0x607A RW (scaled) */
    uint32_t target_position_time_ms;/* 0x607B RW (PROFILE_POSITION move duration, ms; 0 = ASAP) */
    int32_t  position_actual;        /* 0x6064 RO (scaled) */
    uint32_t profile_velocity;       /* 0x6081 RW */
    uint32_t profile_acceleration;   /* 0x6083 RW */
    uint32_t profile_deceleration;   /* 0x6084 RW */
    uint32_t quick_stop_deceleration;/* 0x6085 RW */
    int32_t  target_velocity;        /* 0x60FF RW (scaled) */
    int32_t  velocity_actual;        /* 0x606C RO (scaled) */
    int32_t  target_torque;          /* 0x6071 RW (scaled) */
    int32_t  torque_actual;          /* 0x6077 RO (scaled) */
} MC_OdStore_t;

/** @brief The single OD backing store instance (defined in mc_od.c). */
extern MC_OdStore_t g_od;

/** @brief Seed @ref g_od with built-in defaults (proven gains, default motor). */
void MC_OdStore_LoadDefaults(void);

#endif /* MC_OD_STORE_H */
