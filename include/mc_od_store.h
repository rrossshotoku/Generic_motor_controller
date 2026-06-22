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
    float    pos_kp, pos_ki, pos_kd;                 /* 0x2200 */
    float    vel_kp, vel_ki, vel_kd;                 /* 0x2300 */
    float    vel_current_limit_a;                    /* 0x2300:4 */
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
    uint16_t cal_command;                            /* 0x2700:1 */
    uint16_t store_save_command;                     /* 0x2800:1 */

    /* --- Telemetry (RO; mirrored from the live control state) --- */
    float    tlm_vel_demand_rad_s, tlm_vel_actual_rad_s, tlm_vel_iq_cmd_a;   /* 0x2310 */
    float    tlm_id_meas_a, tlm_iq_meas_a, tlm_vd_v, tlm_vq_v, tlm_electrical_angle_rad; /* 0x2410 */
    float    tlm_mech_position_rad, tlm_mech_velocity_rad_s;                 /* 0x2510 */
    float    tlm_bus_voltage_v;                                              /* 0x2600:3 */
    uint16_t cal_status, store_status;

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
