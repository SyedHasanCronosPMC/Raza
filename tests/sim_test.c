// tests/sim_test.c
// Regression tests for the simulation. Anchored values were captured from
// the working CLI binary on 2026-05-05 commit d7b0f2d with seed=42.
#include <stdio.h>
#include <math.h>
#include "../sim.h"

static int near(double a, double b, double tol) { return fabs(a - b) <= tol; }

static int test_standard_hover_settles_near_target(void) {
    SimParams p = {
        .kp = 2.0, .ki = 0.5, .kd = 1.0, .target_alt = 10.0,
        .mass = 1.0, .gravity = 9.81,
        .has_wind = 0, .wind_strength = 0.0,
        .deriv_on_meas = 0, .integrator = 0,
        .motor_tau = 0.0, .sensor_sigma = 0.0,
        .integral_max = 50.0, .thrust_max = 150.0,
        .duration = 10.0, .dt = 0.1, .seed = 42,
    };
    SimSample out[256];
    int n = simulate(&p, out, 256);
    if (n != 100) { printf("FAIL: expected 100 samples, got %d\n", n); return 1; }
    double final = out[n - 1].altitude;
    if (!near(final, 10.0, 0.5)) {
        printf("FAIL: expected final altitude near 10.0m, got %.3f\n", final);
        return 1;
    }
    return 0;
}

static int test_motor_lag_delays_thrust(void) {
    SimParams base = {
        .kp = 5.0, .ki = 0.0, .kd = 0.0, .target_alt = 10.0,
        .mass = 1.0, .gravity = 9.81,
        .has_wind = 0, .wind_strength = 0.0,
        .deriv_on_meas = 0, .integrator = 0,
        .motor_tau = 0.0, .sensor_sigma = 0.0,
        .integral_max = 50.0, .thrust_max = 150.0,
        .duration = 1.0, .dt = 0.1, .seed = 42,
    };
    SimSample instant[16], lagged[16];
    simulate(&base, instant, 16);
    base.motor_tau = 0.2;
    simulate(&base, lagged, 16);
    if (lagged[0].thrust >= instant[0].thrust) {
        printf("FAIL: lagged thrust[0] %.3f not < instant thrust[0] %.3f\n",
               lagged[0].thrust, instant[0].thrust);
        return 1;
    }
    return 0;
}

static int test_seed_is_deterministic(void) {
    SimParams p = {
        .kp = 2.0, .ki = 0.5, .kd = 1.0, .target_alt = 10.0,
        .mass = 1.0, .gravity = 9.81,
        .has_wind = 1, .wind_strength = 4.0,
        .deriv_on_meas = 0, .integrator = 0,
        .motor_tau = 0.0, .sensor_sigma = 0.0,
        .integral_max = 50.0, .thrust_max = 150.0,
        .duration = 5.0, .dt = 0.1, .seed = 7,
    };
    SimSample a[64], b[64];
    int na = simulate(&p, a, 64);
    int nb = simulate(&p, b, 64);
    if (na != nb) { printf("FAIL: sample count mismatch\n"); return 1; }
    for (int i = 0; i < na; i++) {
        if (!near(a[i].altitude, b[i].altitude, 1e-12)) {
            printf("FAIL: sample %d altitude diverges (%.6f vs %.6f)\n",
                   i, a[i].altitude, b[i].altitude);
            return 1;
        }
    }
    return 0;
}

static int test_rk4_settles_near_target(void) {
    SimParams p = {
        .kp = 2.0, .ki = 0.5, .kd = 1.0, .target_alt = 10.0,
        .mass = 1.0, .gravity = 9.81,
        .has_wind = 0, .wind_strength = 0.0,
        .deriv_on_meas = 0, .integrator = 1,
        .motor_tau = 0.0, .sensor_sigma = 0.0,
        .integral_max = 50.0, .thrust_max = 150.0,
        .duration = 10.0, .dt = 0.1, .seed = 42,
    };
    SimSample out[256];
    int n = simulate(&p, out, 256);
    if (n <= 0) { printf("FAIL: rk4 returned 0\n"); return 1; }
    if (!near(out[n - 1].altitude, 10.0, 0.5)) {
        printf("FAIL: rk4 final altitude %.3f not near 10.0\n", out[n - 1].altitude);
        return 1;
    }
    return 0;
}

int main(void) {
    int failures = 0;
    failures += test_standard_hover_settles_near_target();
    failures += test_motor_lag_delays_thrust();
    failures += test_seed_is_deterministic();
    failures += test_rk4_settles_near_target();
    if (failures == 0) { printf("All tests passed.\n"); return 0; }
    printf("%d test(s) failed.\n", failures);
    return 1;
}
