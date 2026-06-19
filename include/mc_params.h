#ifndef MC_PARAMS_H
#define MC_PARAMS_H

#include "mc_types.h"
#include "mc_board_config.h"
#include "mc_motor_model.h"
#include "mc_ssi_encoder.h"
#include "mc_state_estimator.h"
#include "mc_pid.h"

/** @file mc_params.h
 *  @brief Persistable configuration schema (the flash payload). See ADR-010.
 *  @ingroup mc_persistence
 *
 *  MC_Params_t is the single blob written to / read from non-volatile storage by
 *  mc_persistent_store. It embeds the pointer-free module config structs plus calibration
 *  results. It must NEVER contain runtime pointers/handles (those are bound at startup and are
 *  meaningless across boots). Any layout change requires bumping the store version.
 */

/** @brief Everything persisted to flash for one axis. */
typedef struct
{
    /* --- Board hardware scaling (data only; handles never persisted) --- */
    MC_CurrentSenseConfig_t  current_sense;   /**< shunt/gain/offset/vref/full-scale/signs. */
    MC_BusSenseConfig_t      bus_sense;
    MC_TempSenseConfig_t     temp_sense;
    MC_PwmConfig_t           pwm;
    MC_BoardEncoderConfig_t  board_encoder;

    /* --- Motor + feedback --- */
    MC_MotorModel_t          motor;           /**< Kt/R/L/pole-pairs/inertia/ratings/thermal. */
    MC_SsiEncoderConfig_t    ssi;             /**< frame layout, counts/rev, direction, zero. */
    MC_StateEstimatorConfig_t estimator;      /**< pole pairs, electrical offset, observer gains. */

    /* --- Calibration results --- */
    float  current_offset_a_counts;           /**< Measured zero-current ADC offset, phase A. */
    float  current_offset_c_counts;           /**< Measured zero-current ADC offset, phase C. */
    float  mechanical_zero_offset_rad;        /**< Home reference [rad]. */
    int8_t phase_order;                        /**< +1 / -1, from phase-order detection. */

    /* --- Controller gains --- */
    MC_PidConfig_t position_pid;
    MC_PidConfig_t velocity_pid;
    MC_PidConfig_t current_d_pi;
    MC_PidConfig_t current_q_pi;

    /* --- Trajectory + limits --- */
    float max_velocity_rad_s;
    float max_acceleration_rad_s2;
    float max_deceleration_rad_s2;
    float max_jerk_rad_s3;
    float current_limit_a;

    /* --- Soft limits --- */
    float soft_limit_cw_rad;
    float soft_limit_ccw_rad;
    bool  soft_limits_enabled;

    /* --- Fault thresholds (subset; extend with the fault manager) --- */
    float following_error_threshold_rad;
    float overvoltage_v;
    float undervoltage_v;
    float overtemp_trip_c;
} MC_Params_t;

/** @brief Fill @p p with compile-time defaults (board profile 0 + default motor + gains). */
void MC_Params_LoadDefaults(MC_Params_t *p);

/** @brief Push @p p into the live module configs (called after load / factory reset). */
void MC_Params_ApplyToLive(const MC_Params_t *p);

/** @brief Snapshot the live module configs into @p p (called before a save). */
void MC_Params_CaptureFromLive(MC_Params_t *p);

#endif /* MC_PARAMS_H */
