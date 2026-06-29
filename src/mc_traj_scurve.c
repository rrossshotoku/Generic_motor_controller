#include "mc_traj_scurve.h"
#include <math.h>

/** @file mc_traj_scurve.c
 *  @brief Jerk-limited S-curve trajectory planner (ADR-045). Fixed jerk j; the single free knob is the
 *         peak velocity V, from which the whole 7-segment profile follows:
 *           a_peak  = min(a_lim, sqrt(j*V))            -- triangular accel below V=a_lim^2/j, else clamped
 *           d_acc   = (V/2)*t_acc                      -- accel-phase distance (antisymmetric ramp)
 *           T(V)    = t_acc(V) + D/V                   -- total time, MONOTONIC decreasing in V
 *         Solve T(V)=requested_time by bisection; clamp V to v_max and the distance-limited V; extend
 *         the time to T_min if the request is too short. Rest-to-rest only. Sampled by mc_trajectory.c.
 */

/* Accel-phase shape for a rest->V ramp at fixed jerk j with accel cap a_lim.
   t_j = jerk-segment duration, t_a = const-accel duration, a_peak = peak acceleration. */
static void scurve_accel_phase(float V, float j, float a_lim,
                               float *t_j, float *t_a, float *a_peak)
{
    const float v_tri = (a_lim * a_lim) / j;   /* velocity a pure triangular accel reaches a_lim at */
    if (V <= v_tri)
    {
        *a_peak = sqrtf(j * V);   /* < a_lim; no const-accel segment */
        *t_j    = *a_peak / j;    /* = sqrt(V/j) */
        *t_a    = 0.0f;
    }
    else
    {
        *a_peak = a_lim;                          /* clamped */
        *t_j    = a_lim / j;
        *t_a    = (V / a_lim) - (a_lim / j);      /* > 0 since V > a_lim^2/j */
    }
}

/* Accel-phase duration (t_acc) and distance (d_acc) for peak velocity V. */
static void scurve_accel_metrics(float V, float j, float a_lim, float *t_acc, float *d_acc)
{
    float t_j, t_a, a_peak;
    scurve_accel_phase(V, j, a_lim, &t_j, &t_a, &a_peak);
    *t_acc = (2.0f * t_j) + t_a;
    *d_acc = 0.5f * V * (*t_acc);   /* mean velocity V/2 over the antisymmetric ramp */
}

/* Total move time for peak velocity V over distance d (cruise clamped non-negative). */
static float scurve_time(float V, float j, float a_lim, float d)
{
    float t_acc, d_acc;
    scurve_accel_metrics(V, j, a_lim, &t_acc, &d_acc);
    const float d_cruise = d - (2.0f * d_acc);
    const float t_cruise = (d_cruise > 0.0f) ? (d_cruise / V) : 0.0f;
    return (2.0f * t_acc) + t_cruise;
}

MC_TrajStatus_t MC_TrajScurve_Plan(MC_TrajectoryPlanner_t *p, const MC_TrajRequest_t *req)
{
    if ((p == 0) || (req == 0)) { return MC_TRAJ_ERR_INVALID_REQUEST; }

    const MC_TrajLimits_t *lim = &req->limits;
    const float v_max = lim->max_velocity_rad_per_s;
    const float a_lim = (lim->max_acceleration_rad_per_s2 < lim->max_deceleration_rad_per_s2)
                      ?  lim->max_acceleration_rad_per_s2 :  lim->max_deceleration_rad_per_s2;
    const float j     = lim->max_jerk_rad_per_s3;
    if ((v_max <= 0.0f) || (a_lim <= 0.0f) || (j <= 0.0f))
    {
        p->active = false; p->info.status = MC_TRAJ_ERR_INVALID_LIMITS;
        return MC_TRAJ_ERR_INVALID_LIMITS;
    }
    /* Rest-to-rest only (spec/ADR-045 scope): zero target velocity/acceleration. */
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

    p->start.position_rad            = p0;     /* planned from rest */
    p->start.velocity_rad_per_s      = 0.0f;
    p->start.acceleration_rad_per_s2 = 0.0f;
    p->elapsed_time_s        = 0.0f;
    p->info.requested_time_s = req->requested_time_s;

    if (d < 1e-9f)   /* no move -> hold */
    {
        p->segment_count       = 0u;
        p->info.planned_time_s = 0.0f;
        p->info.time_stretched = false;
        p->info.status         = MC_TRAJ_OK;
        p->active              = true;
        return MC_TRAJ_OK;
    }

    /* --- V_ceil: fastest peak velocity within v_max AND the distance (where the cruise vanishes). --- */
    float t_acc_vm, d_acc_vm;
    scurve_accel_metrics(v_max, j, a_lim, &t_acc_vm, &d_acc_vm);
    float V_ceil;
    if ((2.0f * d_acc_vm) <= d)
    {
        V_ceil = v_max;   /* velocity-limited: a cruise fits at v_max */
    }
    else
    {
        /* distance-limited: bisect V in (0, v_max] for 2*d_acc(V) = d (d_acc monotonic in V). */
        float lo = 0.0f, hi = v_max;
        for (int i = 0; i < 60; i++)
        {
            const float mid = 0.5f * (lo + hi);
            float ta, da; scurve_accel_metrics(mid, j, a_lim, &ta, &da);
            if ((2.0f * da) < d) { lo = mid; } else { hi = mid; }
        }
        V_ceil = 0.5f * (lo + hi);
    }

    const float T_min = scurve_time(V_ceil, j, a_lim, d);

    /* --- Pick V to hit the requested time, else run at V_ceil and stretch. --- */
    float V; bool stretched = false;
    if (req->requested_time_s > (T_min + 1e-6f))
    {
        /* feasible + slower: bisect V in (eps, V_ceil] for T(V) = requested (T decreasing in V). */
        float lo = 1e-6f, hi = V_ceil;
        for (int i = 0; i < 60; i++)
        {
            const float mid = 0.5f * (lo + hi);
            if (scurve_time(mid, j, a_lim, d) > req->requested_time_s) { lo = mid; } else { hi = mid; }
        }
        V = 0.5f * (lo + hi);
    }
    else
    {
        V = V_ceil;
        if (req->requested_time_s > 0.0f) { stretched = true; }   /* too short -> stretched to T_min */
    }

    /* --- Build the 7 segments {duration, accel-at-start, jerk}; a(t) = accel + jerk*t. --- */
    float t_j, t_a, a_peak;
    scurve_accel_phase(V, j, a_lim, &t_j, &t_a, &a_peak);
    float t_acc, d_acc;
    scurve_accel_metrics(V, j, a_lim, &t_acc, &d_acc);
    float d_cruise = d - (2.0f * d_acc);
    if (d_cruise < 0.0f) { d_cruise = 0.0f; }
    const float t_cruise = (V > 0.0f) ? (d_cruise / V) : 0.0f;

    const float A  = dir * a_peak;   /* peak accel in the move direction */
    const float Jp = dir * j;        /* jerk up in the move direction */

    p->segment[0].duration_s = t_j;      p->segment[0].accel_rad_per_s2 = 0.0f; p->segment[0].jerk_rad_per_s3 =  Jp;  /* 0 -> +A  */
    p->segment[1].duration_s = t_a;      p->segment[1].accel_rad_per_s2 =  A;   p->segment[1].jerk_rad_per_s3 = 0.0f; /* const +A */
    p->segment[2].duration_s = t_j;      p->segment[2].accel_rad_per_s2 =  A;   p->segment[2].jerk_rad_per_s3 = -Jp;  /* +A -> 0, v=V */
    p->segment[3].duration_s = t_cruise; p->segment[3].accel_rad_per_s2 = 0.0f; p->segment[3].jerk_rad_per_s3 = 0.0f; /* cruise   */
    p->segment[4].duration_s = t_j;      p->segment[4].accel_rad_per_s2 = 0.0f; p->segment[4].jerk_rad_per_s3 = -Jp;  /* 0 -> -A  */
    p->segment[5].duration_s = t_a;      p->segment[5].accel_rad_per_s2 = -A;   p->segment[5].jerk_rad_per_s3 = 0.0f; /* const -A */
    p->segment[6].duration_s = t_j;      p->segment[6].accel_rad_per_s2 = -A;   p->segment[6].jerk_rad_per_s3 =  Jp;  /* -A -> 0, v=0 */
    p->segment_count = 7u;

    p->info.planned_time_s = (2.0f * t_acc) + t_cruise;
    p->info.time_stretched = stretched;
    p->info.status         = stretched ? MC_TRAJ_TIME_STRETCHED : MC_TRAJ_OK;
    p->active              = true;
    return p->info.status;
}
