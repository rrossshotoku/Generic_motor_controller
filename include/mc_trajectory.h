#ifndef MC_TRAJECTORY_H
#define MC_TRAJECTORY_H
#include "mc_types.h"

/** @file mc_trajectory.h
 *  @brief Jerk-limited S-curve trajectory planner.
 *  @ingroup mc_motion
 */

typedef enum
{
    MC_TRAJ_OK = 0,
    MC_TRAJ_DONE,
    MC_TRAJ_TIME_STRETCHED,
    MC_TRAJ_ERR_INVALID_REQUEST,
    MC_TRAJ_ERR_INVALID_LIMITS,
    MC_TRAJ_ERR_UNSUPPORTED_BOUNDARY,
    MC_TRAJ_ERR_NUMERIC
} MC_TrajStatus_t;

typedef struct
{
    float max_velocity_rad_per_s;
    float max_acceleration_rad_per_s2;
    float max_deceleration_rad_per_s2;
    float max_jerk_rad_per_s3;
} MC_TrajLimits_t;

typedef struct
{
    MC_MechanicalState_t start;
    float target_position_rad;
    float target_velocity_rad_per_s;
    float target_acceleration_rad_per_s2;
    float requested_time_s;
    MC_TrajLimits_t limits;
} MC_TrajRequest_t;

typedef struct
{
    float requested_time_s;
    float planned_time_s;
    bool time_stretched;
    MC_TrajStatus_t status;
} MC_TrajPlanInfo_t;

typedef struct
{
    float duration_s;
    float jerk_rad_per_s3;
} MC_TrajSegment_t;

#define MC_TRAJ_MAX_SEGMENTS (7u)

typedef struct
{
    MC_TrajSegment_t segment[MC_TRAJ_MAX_SEGMENTS];
    uint8_t segment_count;
    float elapsed_time_s;
    MC_MechanicalState_t start;
    MC_TrajPlanInfo_t info;
    bool active;
} MC_TrajectoryPlanner_t;

void MC_Trajectory_Init(MC_TrajectoryPlanner_t *planner);
MC_TrajStatus_t MC_Trajectory_Start(MC_TrajectoryPlanner_t *planner, const MC_TrajRequest_t *req);
MC_TrajStatus_t MC_Trajectory_Replan(MC_TrajectoryPlanner_t *planner, const MC_TrajRequest_t *req);
MC_MotionSetpoint_t MC_Trajectory_Update(MC_TrajectoryPlanner_t *planner, float dt_s);
MC_MotionSetpoint_t MC_Trajectory_Evaluate(const MC_TrajectoryPlanner_t *planner, float elapsed_time_s);
MC_TrajPlanInfo_t MC_Trajectory_GetPlanInfo(const MC_TrajectoryPlanner_t *planner);

#endif
