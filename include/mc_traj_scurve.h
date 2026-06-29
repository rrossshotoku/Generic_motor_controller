#ifndef MC_TRAJ_SCURVE_H
#define MC_TRAJ_SCURVE_H
#include "mc_trajectory.h"

/** @file mc_traj_scurve.h
 *  @brief Jerk-limited S-curve trajectory planner (ADR-045) -- a parallel block to the trapezoidal
 *         planner (mc_trajectory.c, ADR-025), which is left unchanged. It fills the SAME
 *         MC_TrajectoryPlanner_t with a 7-segment jerk-limited profile and is sampled by the existing
 *         MC_Trajectory_Update / MC_Trajectory_Evaluate (already integrate the per-segment jerk).
 *
 *  Fixed jerk: limits.max_jerk_rad_per_s3 is the system-wide jerk (same for every move); peak
 *  acceleration and velocity vary per move within limits.max_acceleration / max_velocity. The move is
 *  planned rest-to-rest (target velocity/accel = 0) to honour requested_time_s, extending the time
 *  (MC_TRAJ_TIME_STRETCHED) only if the requested time is too short to stay within the limits.
 *  @ingroup mc_motion
 */

/** @brief Plan a jerk-limited S-curve into @p p (rest-to-rest). Same role as MC_Trajectory_Start but a
 *         fixed-jerk S-curve. Sample the result with MC_Trajectory_Update / MC_Trajectory_Evaluate. */
MC_TrajStatus_t MC_TrajScurve_Plan(MC_TrajectoryPlanner_t *p, const MC_TrajRequest_t *req);

#endif /* MC_TRAJ_SCURVE_H */
