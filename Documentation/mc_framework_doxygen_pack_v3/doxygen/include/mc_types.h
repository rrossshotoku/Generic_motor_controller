#ifndef MC_TYPES_H
#define MC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/** @file mc_types.h @brief Shared SI-unit data contracts. */

typedef enum {
    MC_OK = 0,
    MC_ERR_INVALID_ARG,
    MC_ERR_INVALID_STATE,
    MC_ERR_TIMEOUT,
    MC_ERR_RANGE,
    MC_ERR_NOT_READY,
    MC_ERR_FAULT
} MC_Status_t;

typedef struct {
    float position_rad;
    float velocity_rad_per_s;
    float acceleration_rad_per_s2;
    uint32_t timestamp_ticks;
    bool position_valid;
    bool velocity_valid;
    bool absolute_position_valid;
} MC_MechanicalState_t;

typedef struct {
    float electrical_angle_rad;
    float electrical_velocity_rad_per_s;
    bool electrical_valid;
} MC_ElectricalState_t;

typedef struct {
    float position_rad;
    float velocity_rad_per_s;
    float acceleration_rad_per_s2;
    float jerk_rad_per_s3;
    bool valid;
    bool complete;
} MC_MotionSetpoint_t;

typedef struct {
    float torque_nm;
    float current_limit_a;
    bool enable;
} MC_MotorTorqueRequest_t;

typedef struct {
    float ia_a;
    float ib_a;
    float ic_a;
    bool valid;
    uint32_t timestamp_ticks;
} MC_PhaseCurrents_t;

typedef struct {
    float id_a;
    float iq_a;
    bool enable;
} MC_FocCurrentCommand_t;

typedef struct {
    float duty_a;
    float duty_b;
    float duty_c;
    bool enable;
} MC_PwmDuty_t;

#endif
