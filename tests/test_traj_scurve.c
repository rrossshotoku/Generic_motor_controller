/* Host test for the jerk-limited S-curve planner (ADR-045). Plans moves, samples them with the shared
 * mc_trajectory.c integrator, and checks: lands at target at rest, stays within v/a/jerk limits, hits
 * the requested time when feasible, stretches when too short, no overshoot.
 *   gcc -std=c11 -I include tests/test_traj_scurve.c src/mc_traj_scurve.c src/mc_trajectory.c -lm -o t && ./t
 */
#include "mc_traj_scurve.h"
#include "mc_trajectory.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); g_fail++; } } while (0)

static void run_move(const char *name, float start, float target, float T_req,
                     float vmax, float amax, float jerk, bool expect_stretch)
{
    MC_TrajectoryPlanner_t p;
    MC_Trajectory_Init(&p);
    MC_TrajRequest_t req;
    req.start.position_rad            = start;
    req.start.velocity_rad_per_s      = 0.0f;
    req.start.acceleration_rad_per_s2 = 0.0f;
    req.target_position_rad            = target;
    req.target_velocity_rad_per_s      = 0.0f;
    req.target_acceleration_rad_per_s2 = 0.0f;
    req.requested_time_s               = T_req;
    req.limits.max_velocity_rad_per_s      = vmax;
    req.limits.max_acceleration_rad_per_s2 = amax;
    req.limits.max_deceleration_rad_per_s2 = amax;
    req.limits.max_jerk_rad_per_s3         = jerk;

    MC_TrajStatus_t st = MC_TrajScurve_Plan(&p, &req);
    CHECK(st == MC_TRAJ_OK || st == MC_TRAJ_TIME_STRETCHED, "[%s] status %d", name, st);

    const float D   = target - start;
    const float dir = (D >= 0.0f) ? 1.0f : -1.0f;
    const float Tp  = p.info.planned_time_s;

    float maxv = 0.0f, maxa = 0.0f, maxj = 0.0f, overshoot = 0.0f;
    for (float t = 0.0f; t <= Tp + 1e-4f; t += 1e-4f)
    {
        MC_MotionSetpoint_t sp = MC_Trajectory_Evaluate(&p, t);
        if (fabsf(sp.velocity_rad_per_s)     > maxv) maxv = fabsf(sp.velocity_rad_per_s);
        if (fabsf(sp.acceleration_rad_per_s2) > maxa) maxa = fabsf(sp.acceleration_rad_per_s2);
        if (fabsf(sp.jerk_rad_per_s3)        > maxj) maxj = fabsf(sp.jerk_rad_per_s3);
        float os = dir * (sp.position_rad - target);
        if (os > overshoot) overshoot = os;
    }
    MC_MotionSetpoint_t end = MC_Trajectory_Evaluate(&p, Tp);
    float pos_err = fabsf(end.position_rad - target);

    printf("%-16s D=%.3f Treq=%.3f -> planned=%.4f%s  maxV=%.4f/%.2f maxA=%.4f/%.2f maxJ=%.0f/%.0f  posErr=%.2e\n",
           name, D, T_req, Tp, p.info.time_stretched ? " [stretch]" : "",
           maxv, vmax, maxa, amax, maxj, jerk, pos_err);

    CHECK(pos_err < 1e-3f,                       "[%s] end pos off %.2e", name, pos_err);
    CHECK(fabsf(end.velocity_rad_per_s) < 1e-3f, "[%s] end vel %.2e", name, end.velocity_rad_per_s);
    CHECK(maxv <= vmax * 1.01f + 1e-4f,          "[%s] vel %.4f > vmax", name, maxv);
    CHECK(maxa <= amax * 1.01f + 1e-4f,          "[%s] accel %.4f > amax", name, maxa);
    CHECK(maxj <= jerk * 1.01f + 1e-2f,          "[%s] jerk %.1f > j", name, maxj);
    CHECK(overshoot < 1e-3f,                     "[%s] overshoot %.2e", name, overshoot);
    CHECK(p.info.time_stretched == expect_stretch, "[%s] stretch flag %d", name, p.info.time_stretched);
    if (!expect_stretch && T_req > 0.0f && fabsf(D) > 1e-6f)
        CHECK(fabsf(Tp - T_req) < 1e-2f,         "[%s] planned %.4f != req %.4f", name, Tp, T_req);
    if (expect_stretch)
        CHECK(Tp > T_req,                        "[%s] stretched but planned %.4f <= req", name, Tp);
}

int main(void)
{
    run_move("vel-limited",  0.0f, 10.0f, 5.0f, 5.0f, 20.0f, 100.0f, false); /* long, slow -> hits Treq */
    run_move("asap-long",    0.0f, 10.0f, 0.0f, 5.0f, 20.0f, 100.0f, false); /* fastest feasible       */
    run_move("too-short",    0.0f, 10.0f, 0.5f, 5.0f, 20.0f, 100.0f, true);  /* stretched to T_min     */
    run_move("short-tri",    0.0f, 0.05f, 0.0f, 5.0f, 20.0f, 100.0f, false); /* triangular, no cruise  */
    run_move("negative-dir", 2.0f, -3.0f, 0.0f, 5.0f, 20.0f, 100.0f, false); /* backward move          */
    run_move("slow-lowjerk", 0.0f, 1.0f,  4.0f, 2.0f, 10.0f, 20.0f,  false); /* low jerk, hits Treq    */
    run_move("no-move",      1.0f, 1.0f,  1.0f, 5.0f, 20.0f, 100.0f, false); /* zero distance          */

    printf(g_fail ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
