#include "mc_trajectory.h"
#include <math.h>

/** @file mc_trajectory.c
 *  @brief Trapezoidal trajectory planner, fixed 1/6 : 2/3 : 1/6 split (ADR-025, spec 08).
 *
 *  Planned from rest to a target position (target velocity/accel = 0). With the fixed split
 *  (t_a = t_d = T/6, t_c = 2T/3) the move distance is D = v_cruise * 5T/6, so:
 *      v_cruise = 1.2 * D / T ,   a = v_cruise / t_a = 7.2 * D / T^2 .
 *  The minimum feasible time for the limits is T_min = max( 1.2 D / v_max , sqrt(7.2 D / a_lim) ).
 *  The move is stored as three constant-accel segments and integrated by MC_Trajectory_Evaluate.
 */

void MC_Trajectory_Init(MC_TrajectoryPlanner_t *p)
{
    if (p == 0) { return; }
    p->segment_count  = 0u;
    p->elapsed_time_s = 0.0f;
    p->active         = false;
    p->start.position_rad            = 0.0f;
    p->start.velocity_rad_per_s      = 0.0f;
    p->start.acceleration_rad_per_s2 = 0.0f;
    p->info.requested_time_s = 0.0f;
    p->info.planned_time_s   = 0.0f;
    p->info.time_stretched   = false;
    p->info.status           = MC_TRAJ_OK;
}

MC_TrajStatus_t MC_Trajectory_Start(MC_TrajectoryPlanner_t *p, const MC_TrajRequest_t *req)
{
    if ((p == 0) || (req == 0)) { return MC_TRAJ_ERR_INVALID_REQUEST; }

    const MC_TrajLimits_t *lim = &req->limits;
    if ((lim->max_velocity_rad_per_s     <= 0.0f) ||
        (lim->max_acceleration_rad_per_s2 <= 0.0f) ||
        (lim->max_deceleration_rad_per_s2 <= 0.0f))
    {
        p->active = false; p->info.status = MC_TRAJ_ERR_INVALID_LIMITS;
        return MC_TRAJ_ERR_INVALID_LIMITS;
    }
    /* First cut: only zero target velocity/acceleration are supported (spec 08 boundary scope). */
    if ((fabsf(req->target_velocity_rad_per_s)     > 1e-6f) ||
        (fabsf(req->target_acceleration_rad_per_s2) > 1e-6f))
    {
        p->active = false; p->info.status = MC_TRAJ_ERR_UNSUPPORTED_BOUNDARY;
        return MC_TRAJ_ERR_UNSUPPORTED_BOUNDARY;
    }

    const float p0  = req->start.position_rad;
    const float D   = req->target_position_rad - p0;
    const float d   = fabsf(D);
    const float dir = (D >= 0.0f) ? 1.0f : -1.0f;

    /* Planned from rest -- start velocity/accel accepted but treated as 0 in this cut. */
    p->start.position_rad            = p0;
    p->start.velocity_rad_per_s      = 0.0f;
    p->start.acceleration_rad_per_s2 = 0.0f;
    p->elapsed_time_s        = 0.0f;
    p->info.requested_time_s = req->requested_time_s;

    if (d < 1e-9f)   /* no move -> hold at the target */
    {
        p->segment_count       = 0u;
        p->info.planned_time_s = 0.0f;
        p->info.time_stretched = false;
        p->info.status         = MC_TRAJ_OK;
        p->active              = true;
        return MC_TRAJ_OK;
    }

    const float v_max = lim->max_velocity_rad_per_s;
    const float a_lim = (lim->max_acceleration_rad_per_s2 < lim->max_deceleration_rad_per_s2)
                      ?  lim->max_acceleration_rad_per_s2 :  lim->max_deceleration_rad_per_s2;
    const float t_vel = 1.2f * d / v_max;            /* time at which v_cruise hits v_max */
    const float t_acc = sqrtf(7.2f * d / a_lim);     /* time at which a hits a_lim */
    float T = (t_vel > t_acc) ? t_vel : t_acc;       /* T_min */
    bool  stretched = false;

    if (req->requested_time_s > T)         { T = req->requested_time_s; }  /* feasible -> honour it */
    else if (req->requested_time_s > 0.0f) { stretched = true; }           /* too short -> stretched */

    const float a = 7.2f * d / (T * T);

    p->segment[0].duration_s = T / 6.0f;
    p->segment[0].accel_rad_per_s2 =  dir * a; p->segment[0].jerk_rad_per_s3 = 0.0f;   /* accel  */
    p->segment[1].duration_s = 2.0f * T / 3.0f;
    p->segment[1].accel_rad_per_s2 =  0.0f;    p->segment[1].jerk_rad_per_s3 = 0.0f;   /* cruise */
    p->segment[2].duration_s = T / 6.0f;
    p->segment[2].accel_rad_per_s2 = -dir * a; p->segment[2].jerk_rad_per_s3 = 0.0f;   /* decel  */
    p->segment_count = 3u;

    p->info.planned_time_s = T;
    p->info.time_stretched = stretched;
    p->info.status         = stretched ? MC_TRAJ_TIME_STRETCHED : MC_TRAJ_OK;
    p->active              = true;
    return p->info.status;
}

/* Integrate the plan from the start state up to time @p t. Returns position/velocity/accel and the
   active jerk (0 except mid-jerk-segment, which the trapezoid never has). */
static void traj_integrate(const MC_TrajectoryPlanner_t *p, float t,
                           float *pos, float *vel, float *acc, float *jrk)
{
    float P = p->start.position_rad;
    float V = p->start.velocity_rad_per_s;
    float A = p->start.acceleration_rad_per_s2;
    float J = 0.0f;
    float rem = (t > 0.0f) ? t : 0.0f;

    for (uint8_t i = 0u; i < p->segment_count; i++)
    {
        const float dur = p->segment[i].duration_s;
        const float sa  = p->segment[i].accel_rad_per_s2;
        const float sj  = p->segment[i].jerk_rad_per_s3;
        const float dt  = (rem < dur) ? rem : dur;

        P += (V * dt) + (0.5f * sa * dt * dt) + ((1.0f / 6.0f) * sj * dt * dt * dt);
        V += (sa * dt) + (0.5f * sj * dt * dt);
        A  = sa + (sj * dt);
        J  = (dt < dur) ? sj : 0.0f;   /* mid-segment -> that segment's jerk; at a boundary -> 0 */
        rem -= dt;
        if (rem <= 0.0f) { break; }
    }
    *pos = P; *vel = V; *acc = A; *jrk = J;
}

MC_MotionSetpoint_t MC_Trajectory_Evaluate(const MC_TrajectoryPlanner_t *p, float elapsed_time_s)
{
    MC_MotionSetpoint_t sp;
    sp.position_rad = 0.0f; sp.velocity_rad_per_s = 0.0f;
    sp.acceleration_rad_per_s2 = 0.0f; sp.jerk_rad_per_s3 = 0.0f;
    sp.valid = false; sp.complete = false;

    if ((p == 0) || !p->active) { return sp; }
    sp.valid = true;

    const float total = p->info.planned_time_s;
    float P, V, A, J;

    if ((p->segment_count == 0u) || (elapsed_time_s >= total))
    {
        traj_integrate(p, total, &P, &V, &A, &J);   /* end-of-move state = target position */
        sp.position_rad = P;
        sp.velocity_rad_per_s = 0.0f;
        sp.acceleration_rad_per_s2 = 0.0f;
        sp.jerk_rad_per_s3 = 0.0f;
        sp.complete = true;
        return sp;
    }

    traj_integrate(p, elapsed_time_s, &P, &V, &A, &J);
    sp.position_rad = P;
    sp.velocity_rad_per_s = V;
    sp.acceleration_rad_per_s2 = A;
    sp.jerk_rad_per_s3 = J;
    sp.complete = false;
    return sp;
}

MC_MotionSetpoint_t MC_Trajectory_Update(MC_TrajectoryPlanner_t *p, float dt_s)
{
    if ((p != 0) && p->active) { p->elapsed_time_s += dt_s; }
    return MC_Trajectory_Evaluate(p, (p != 0) ? p->elapsed_time_s : 0.0f);
}

MC_TrajStatus_t MC_Trajectory_Replan(MC_TrajectoryPlanner_t *p, const MC_TrajRequest_t *req)
{
    if ((p == 0) || (req == 0)) { return MC_TRAJ_ERR_INVALID_REQUEST; }

    /* Evaluate the current planned state and use it as the new start (spec 08). This cut carries the
       position; velocity/accel are treated as 0 (the S-curve cut will carry them for a bumpless replan). */
    MC_TrajRequest_t r = *req;
    if (p->active)
    {
        const MC_MotionSetpoint_t cur = MC_Trajectory_Evaluate(p, p->elapsed_time_s);
        r.start.position_rad            = cur.position_rad;
        r.start.velocity_rad_per_s      = 0.0f;
        r.start.acceleration_rad_per_s2 = 0.0f;
    }
    return MC_Trajectory_Start(p, &r);
}

MC_TrajPlanInfo_t MC_Trajectory_GetPlanInfo(const MC_TrajectoryPlanner_t *p)
{
    MC_TrajPlanInfo_t info;
    if (p != 0)
    {
        info = p->info;
    }
    else
    {
        info.requested_time_s = 0.0f; info.planned_time_s = 0.0f;
        info.time_stretched = false;  info.status = MC_TRAJ_ERR_INVALID_REQUEST;
    }
    return info;
}
