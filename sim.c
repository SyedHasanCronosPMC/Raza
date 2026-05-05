// sim.c - Pure simulation. No I/O, no UI. Safe for WASM.
#include "sim.h"
#include <math.h>
#include <stdlib.h>

// Local seeded RNG (xorshift32) so behavior is reproducible from SimParams.seed
// and so we don't share state with the host's rand().
static unsigned int rng_state;
static void rng_seed(int seed) { rng_state = (unsigned int)(seed ? seed : 1); }
static double rng_uniform(void) {
    unsigned int x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return (double)x / 4294967296.0;
}
// Box-Muller for gaussian samples (mean 0, stddev 1)
static double rng_normal(void) {
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(6.2831853071795864769 * u2);
}

int simulate(const SimParams* p, SimSample* out, int max_samples) {
    if (!p || !out || max_samples <= 0) return 0;
    if (p->dt <= 0.0 || p->duration <= 0.0) return 0;

    int steps = (int)(p->duration / p->dt);
    if (steps > max_samples) steps = max_samples;
    if (steps <= 0) return 0;

    rng_seed(p->seed);

    double altitude = 0.0;
    double velocity = 0.0;
    double integral = 0.0;
    double prev_error = p->target_alt - altitude;
    double prev_alt = altitude;
    double thrust_applied = 0.0;
    double wind_force = 0.0;

    for (int i = 0; i < steps; i++) {
        // Sensor reading (control loop sees noisy altitude when sigma > 0)
        double measured = altitude;
        if (p->sensor_sigma > 0.0) measured += p->sensor_sigma * rng_normal();

        double error = p->target_alt - measured;
        integral += error * p->dt;
        if (integral >  p->integral_max) integral =  p->integral_max;
        if (integral < -p->integral_max) integral = -p->integral_max;

        double derivative;
        if (p->deriv_on_meas) {
            derivative = -(measured - prev_alt) / p->dt;
        } else {
            derivative = (error - prev_error) / p->dt;
        }

        double p_term = p->kp * error;
        double i_term = p->ki * integral;
        double d_term = p->kd * derivative;
        double thrust_cmd = p_term + i_term + d_term;
        if (thrust_cmd < 0.0)            thrust_cmd = 0.0;
        if (thrust_cmd > p->thrust_max)  thrust_cmd = p->thrust_max;

        // First-order motor lag (or instant if tau == 0)
        if (p->motor_tau > 0.0) {
            double alpha = p->dt / (p->motor_tau + p->dt);
            thrust_applied = thrust_applied + alpha * (thrust_cmd - thrust_applied);
        } else {
            thrust_applied = thrust_cmd;
        }

        // Wind: re-randomize every 2 simulated seconds (20 steps at dt=0.1)
        int gust_period = (int)(2.0 / p->dt);
        if (gust_period < 1) gust_period = 1;
        if (p->has_wind && (i % gust_period == 0)) {
            wind_force = (rng_uniform() * 2.0 - 1.0) * p->wind_strength;
        }
        if (!p->has_wind) wind_force = 0.0;

        double net_force = thrust_applied - p->mass * p->gravity + wind_force;
        double accel = net_force / p->mass;

        if (p->integrator == 1) {
            // RK4 on the {altitude, velocity} ODE with constant accel over the step
            double k1v = accel;
            double k1x = velocity;
            double k2v = accel;
            double k2x = velocity + 0.5 * p->dt * k1v;
            double k3v = accel;
            double k3x = velocity + 0.5 * p->dt * k2v;
            double k4v = accel;
            double k4x = velocity + p->dt * k3v;
            velocity += p->dt * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0;
            altitude += p->dt * (k1x + 2.0 * k2x + 2.0 * k3x + k4x) / 6.0;
        } else {
            velocity += accel * p->dt;
            altitude += velocity * p->dt;
        }

        if (altitude < 0.0) { altitude = 0.0; velocity = 0.0; }

        out[i].time     = i * p->dt;
        out[i].altitude = altitude;
        out[i].error    = error;
        out[i].thrust   = thrust_applied;
        out[i].p_term   = p_term;
        out[i].i_term   = i_term;
        out[i].d_term   = d_term;

        prev_error = error;
        prev_alt   = altitude;
    }
    return steps;
}
