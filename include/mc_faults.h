#ifndef MC_FAULTS_H
#define MC_FAULTS_H
#include "mc_types.h"

/** @file mc_faults.h
 *  @brief Fault supervisor with severity-based actions.
 *  @ingroup mc_faults
 */

typedef enum
{
    MC_FAULT_SEV_WARNING = 0,
    MC_FAULT_SEV_RECOVERABLE,
    MC_FAULT_SEV_SEVERE
} MC_FaultSeverity_t;

typedef enum
{
    MC_FAULT_NONE = 0,
    MC_FAULT_SPI_TIMEOUT = 1,
    MC_FAULT_ENCODER_INVALID = 2,
    MC_FAULT_FOLLOWING_ERROR = 3,
    MC_FAULT_OVERCURRENT_FAST = 4,
    MC_FAULT_BUS_OVERVOLTAGE = 5,
    MC_FAULT_BUS_UNDERVOLTAGE = 6,
    MC_FAULT_OVERTEMP_WARNING = 7,
    MC_FAULT_OVERTEMP_FAULT = 8,
    MC_FAULT_SOFT_LIMIT = 9,
    MC_FAULT_FOC_SATURATION = 10,
    MC_FAULT_ADC_INVALID = 11,
    MC_FAULT_CALIBRATION_FAILED = 12
} MC_FaultId_t;

typedef struct
{
    uint32_t active_bits;
    uint16_t primary_error_code;
    bool severe_active;
    bool recoverable_active;
    bool warning_active;
} MC_FaultState_t;

void MC_Faults_Init(void);
void MC_Faults_UpdateFast(const MC_PhaseCurrents_t *currents);
void MC_Faults_UpdateMedium(const MC_MechanicalState_t *state);
void MC_Faults_UpdateSlow(const MC_PowerState_t *power);
void MC_Faults_Set(MC_FaultId_t id);
void MC_Faults_ResetRequest(void);
MC_FaultState_t MC_Faults_GetState(void);

#endif
