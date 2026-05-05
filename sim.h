// sim.h - Pure simulation module. Compiles for both desktop CLI and WASM.
#ifndef SIM_H
#define SIM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double kp, ki, kd, target_alt;
    double mass, gravity;
    int    has_wind;
    double wind_strength;
    int    deriv_on_meas;     // 0 = derivative on error, 1 = on measurement
    int    integrator;        // 0 = forward Euler, 1 = RK4
    double motor_tau;         // 0 = instant; >0 = first-order lag (seconds)
    double sensor_sigma;      // 0 = perfect; >0 = gaussian altitude noise stddev
    double integral_max;      // anti-windup clamp
    double thrust_max;        // saturation ceiling
    double duration;          // seconds
    double dt;                // seconds
    int    seed;              // RNG seed for wind + sensor noise
} SimParams;

typedef struct {
    double time;
    double altitude;
    double error;
    double thrust;
    double p_term;
    double i_term;
    double d_term;
} SimSample;

// Runs the simulation. Caller allocates `out_samples` of length >= ceil(duration/dt).
// Returns the number of samples written, or 0 on invalid params.
int simulate(const SimParams* params, SimSample* out_samples, int max_samples);

#ifdef __cplusplus
}
#endif

#endif // SIM_H
