#ifndef MC_MOTOR_MODEL_H
#define MC_MOTOR_MODEL_H
#include "mc_types.h"

/** @file mc_motor_model.h
 *  @brief Generic, motor-type-agnostic electromechanical model for one axis.
 *  @ingroup mc_core
 *
 *  Holds the physical parameters the outer motion layers and the motor backend need,
 *  in SI units, independent of how the motor is driven. FOC-specific use (e.g. torque
 *  to iq via Kt) stays inside the BLDC/FOC backend; this struct only describes the motor.
 *  Values are runtime configuration: a built-in default is provided for first bring-up,
 *  and the persistent store / object dictionary may override them later. See ADR-004.
 */

/** @brief Electromechanical parameters for a single motor/axis (SI units). */
typedef struct
{
    /* --- Identification / topology --- */
    MC_MotorBackendType_t backend_type; /**< Control backend that drives this motor. */
    uint8_t pole_pairs;                 /**< Electrical pole pairs (1 for brushed DC). */
    uint8_t phase_count;                /**< Phases: 3 for BLDC/PMSM, 1 for brushed DC. */

    /* --- Electrical model (terminal, SI) --- */
    float resistance_ohm;               /**< Terminal/phase resistance R [Ohm]. */
    float inductance_h;                 /**< Terminal/phase inductance L [H]. */
    float kt_nm_per_a;                  /**< Torque constant Kt [Nm/A]. */
    float ke_v_per_rad_per_s;           /**< Back-EMF constant Ke [V/(rad/s)] (= Kt in SI). */

    /* --- Mechanical model (SI) --- */
    float rotor_inertia_kg_m2;          /**< Rotor inertia J [kg.m^2] (motor only; add load separately). */
    float friction_coulomb_nm;          /**< Coulomb friction torque [Nm], applied as sign(velocity). */
    float friction_viscous_nm_s_per_rad;/**< Viscous friction coefficient [Nm/(rad/s)]. */

    /* --- Ratings / limits (SI) --- */
    float nominal_voltage_v;            /**< Bus voltage the ratings assume [V]. */
    float nominal_current_a;            /**< Max continuous current [A]. */
    float nominal_torque_nm;            /**< Max continuous torque [Nm]. */
    float stall_current_a;              /**< Stall current [A]. */
    float stall_torque_nm;              /**< Stall torque [Nm]. */
    float max_speed_rad_per_s;          /**< Max mechanical speed [rad/s]. */

    /* --- Thermal model (first-order winding) --- */
    float thermal_time_constant_s;      /**< Winding thermal time constant [s]. */
    float thermal_rise_at_nominal_c;    /**< Steady-state winding rise at nominal current [degC]. */
    float winding_temp_max_c;           /**< Datasheet max winding temperature [degC]. */
} MC_MotorModel_t;

/**
 * @brief Fill @p model with the built-in default motor (Maxon EC 90 flat, part 500267).
 *
 * This is the "profile 0" motor used by the proven reference board for first bring-up.
 * Persistent storage / OD may overwrite these values at runtime.
 * @param model Destination (must be non-NULL).
 */
void MC_MotorModel_LoadDefault(MC_MotorModel_t *model);

/**
 * @brief Sanity-check a motor model (positive inertia, pole pairs, Kt, resistance, etc.).
 * @param model Model to validate (must be non-NULL).
 * @return MC_OK if usable; MC_ERR_INVALID_ARG or MC_ERR_RANGE otherwise.
 */
MC_Status_t MC_MotorModel_Validate(const MC_MotorModel_t *model);

#endif /* MC_MOTOR_MODEL_H */
