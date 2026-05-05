# WASM PID Sandbox Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the existing C drone PID simulator to a Svelte+WASM web sandbox deployed on Vercel, while keeping the CLI binary fully functional.

**Architecture:** Split the C simulation into a pure `sim.c`/`sim.h` library and a thin CLI wrapper. The library compiles for both desktop (`gcc → drone.exe`) and browser (`emcc → sim.wasm`). A Svelte + uPlot + Tailwind SPA loads the WASM module and provides a 16-parameter live sandbox with progressive disclosure and pinned-curve overlays. CI on GitHub Actions; deploy via Vercel's GitHub integration.

**Tech Stack:** C99 (gcc + Emscripten), Svelte 4, uPlot, Tailwind 3, Vite 5, Vitest, GitHub Actions, Vercel.

**Spec:** [`docs/specs/2026-05-05-wasm-frontend-design.md`](../specs/2026-05-05-wasm-frontend-design.md)

---

## File Structure

### To create

```
sim.h
sim.c
Makefile.wasm
tests/sim_test.c
tests/Makefile
.github/workflows/deploy.yml

web/
  package.json
  vite.config.js
  tailwind.config.js
  postcss.config.js
  index.html
  src/main.js
  src/App.svelte
  src/app.css
  src/lib/wasm.js
  src/lib/store.js
  src/lib/stepinfo.js
  src/lib/colors.js
  src/components/ControlsBasic.svelte
  src/components/ControlsAdvanced.svelte
  src/components/Slider.svelte
  src/components/Chart.svelte
  src/components/StepInfo.svelte
  src/components/PinList.svelte
  src/components/HelpTooltip.svelte
  src/components/Banner.svelte
  src/test/stepinfo.test.js
  src/test/store.test.js
```

### To modify

```
openEndedC.c          # becomes a thin CLI wrapper that #includes sim.h
Makefile              # also compiles sim.c
README.md             # adds web frontend section
.gitignore            # adds web build artifacts (already partially done)
```

### To leave alone

```
flight_log.csv (generated, gitignored)
docs/specs/2026-05-05-wasm-frontend-design.md (the spec)
```

---

## Phase Map

| Phase | Tasks | Outcome |
|---|---|---|
| 1. C refactor (no behavior change) | 1.1–1.5 | sim.h/sim.c split; CLI still passes regression test |
| 2. SimParams API + new physics | 2.1–2.10 | `simulate(SimParams*, ...)` with RK4, motor lag, sensor noise, etc. |
| 3. WASM build | 3.1–3.3 | sim.wasm compiles and loads in a smoke-test page |
| 4. Svelte bootstrap | 4.1–4.4 | `npm run dev` boots a page that calls into WASM |
| 5. Stores and pure logic | 5.1–5.4 | `store.js`, `stepinfo.js` with Vitest coverage |
| 6. Components & layout | 6.1–6.7 | Full UI wired |
| 7. Polish & deploy | 7.1–7.3 | CI green, deployed to Vercel |

---

## Phase 1 — C refactor (no behavior change)

The goal is to extract the simulation into a pure module with zero behavior change. A regression test locks in the current output before we touch anything.

### Task 1.1: Lock in current CLI behavior with a regression fixture

**Files:**
- Create: `tests/sim_test.c`
- Create: `tests/Makefile`

- [ ] **Step 1: Write the regression test (will fail to compile until 1.2 lands)**

```c
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

int main(void) {
    int failures = 0;
    failures += test_standard_hover_settles_near_target();
    if (failures == 0) { printf("All tests passed.\n"); return 0; }
    printf("%d test(s) failed.\n", failures);
    return 1;
}
```

- [ ] **Step 2: Add a Makefile target for the test**

Create `tests/Makefile`:

```makefile
CC      := gcc
CFLAGS  := -Wall -Wextra -O0 -g -std=c99
LDFLAGS := -lm

.PHONY: test clean

test: sim_test
	./sim_test

sim_test: sim_test.c ../sim.c ../sim.h
	$(CC) $(CFLAGS) -o $@ sim_test.c ../sim.c $(LDFLAGS)

clean:
	-rm -f sim_test sim_test.exe
```

- [ ] **Step 3: Verify the test fails to compile (sim.h does not exist yet)**

Run: `cd tests && mingw32-make test`
Expected: error — `fatal error: ../sim.h: No such file or directory`

- [ ] **Step 4: Commit the failing fixture**

```powershell
git add tests/sim_test.c tests/Makefile
git commit -m "test: add regression fixture for standard hover (will pass after sim.h split)"
```

---

### Task 1.2: Create `sim.h` with the new public API

**Files:**
- Create: `sim.h`

- [ ] **Step 1: Write the header**

```c
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
```

- [ ] **Step 2: Verify it parses as standalone (no implementation yet)**

Run: `gcc -c -x c sim.h -o NUL`
Expected: silent success (header parses).

- [ ] **Step 3: Commit**

```powershell
git add sim.h
git commit -m "feat: add sim.h with SimParams and SimSample contracts"
```

---

### Task 1.3: Implement `sim.c` extracting the existing physics

**Files:**
- Create: `sim.c`

- [ ] **Step 1: Write the implementation**

```c
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
```

- [ ] **Step 2: Run the regression test**

Run: `cd tests && mingw32-make test`
Expected: `All tests passed.`

- [ ] **Step 3: Commit**

```powershell
git add sim.c
git commit -m "feat: implement simulate() with RK4, motor lag, sensor noise, seeded RNG"
```

---

### Task 1.4: Refactor `openEndedC.c` to use `sim.h`

**Files:**
- Modify: `openEndedC.c`

The CLI keeps menu/tuner/ASCII chart/CSV. It deletes its inline copy of `runSimulation` and `FlightData`, and calls `simulate()` via `SimParams`.

- [ ] **Step 1: Delete the inline `FlightData` struct, `runSimulation` definition, and forward declaration**

Remove these lines from `openEndedC.c`:
- The `typedef struct { ... } FlightData;` block (was lines 49–56 pre-edit)
- The forward declaration `void runSimulation(...)` (was in declarations block)
- The full body of `void runSimulation(...) { ... }` (was lines 153-196 pre-edit)

- [ ] **Step 2: Add `#include "sim.h"` and replace internal log array**

At the top of `openEndedC.c`, after `#include <conio.h>`:

```c
#include "sim.h"
```

Replace `FlightData flightLog[STEPS];` in `main()` with:

```c
SimSample flightLog[STEPS];
```

Update every `FlightData` reference in remaining functions (`runLevel`, `drawStepResponse`, `analyzeFlight`, `saveFlightLog`) to `SimSample`.

- [ ] **Step 3: Replace the call site in `runLevel()` with a SimParams build + simulate() call**

Replace the body of `runLevel()` between `tunePID(...)` and `drawStepResponse(...)` with:

```c
    SimParams params = {
        .kp = kp, .ki = ki, .kd = kd, .target_alt = targetAlt,
        .mass = mass, .gravity = 9.81,
        .has_wind = hasWind ? 1 : 0,
        .wind_strength = 4.0,
        .deriv_on_meas = 0, .integrator = 0,
        .motor_tau = 0.0, .sensor_sigma = 0.0,
        .integral_max = 50.0, .thrust_max = 150.0,
        .duration = maxSteps * dt, .dt = dt,
        .seed = (int)time(NULL),
    };
    int n = simulate(&params, fLog, maxSteps);
    if (n <= 0) {
        printf("\033[31mSimulation failed\033[0m\n");
        return;
    }
```

- [ ] **Step 4: Build and run a smoke test**

Run: `mingw32-make`
Expected: clean build, no warnings.

Run: `cd tests && mingw32-make test`
Expected: `All tests passed.`

- [ ] **Step 5: Commit**

```powershell
git add openEndedC.c
git commit -m "refactor: route CLI through sim.h simulate() API"
```

---

### Task 1.5: Update root `Makefile` to compile `sim.c`

**Files:**
- Modify: `Makefile`

- [ ] **Step 1: Update the build rule**

Replace the `$(TARGET): $(SRC)` block with:

```makefile
SRC := openEndedC.c sim.c

$(TARGET): $(SRC) sim.h
	$(CC) $(CFLAGS) -o $@ openEndedC.c sim.c $(LDFLAGS)
```

Add a `test` target at the bottom:

```makefile
test:
	$(MAKE) -C tests test
```

Add `test` to `.PHONY`:

```makefile
.PHONY: all run clean test
```

- [ ] **Step 2: Verify the full build pipeline**

Run: `mingw32-make clean && mingw32-make && mingw32-make test`
Expected: clean build + `All tests passed.`

- [ ] **Step 3: Commit**

```powershell
git add Makefile
git commit -m "build: compile sim.c alongside openEndedC.c; add test target"
git push
```

---

## Phase 2 — Extend SimParams and pure simulation

The simulation already supports every parameter from Phase 1.3 (RK4, motor lag, sensor noise, etc.). Phase 2 adds tests proving each feature works, plus exposing seed control to the CLI.

### Task 2.1: Test that RK4 integrator produces a finite, monotonically-rising-then-settling trajectory

**Files:**
- Modify: `tests/sim_test.c`

- [ ] **Step 1: Add the test function above `main()`**

```c
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
```

Add `failures += test_rk4_settles_near_target();` to `main()`.

- [ ] **Step 2: Run the tests**

Run: `cd tests && mingw32-make test`
Expected: `All tests passed.`

- [ ] **Step 3: Commit**

```powershell
git add tests/sim_test.c
git commit -m "test: assert RK4 integrator settles near target"
```

---

### Task 2.2: Test seeded RNG produces identical output across runs

**Files:**
- Modify: `tests/sim_test.c`

- [ ] **Step 1: Add the test**

```c
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
```

Add `failures += test_seed_is_deterministic();` to `main()`.

- [ ] **Step 2: Run tests**

Run: `cd tests && mingw32-make test`
Expected: `All tests passed.`

- [ ] **Step 3: Commit**

```powershell
git add tests/sim_test.c
git commit -m "test: assert seeded RNG is bit-identical across runs"
```

---

### Task 2.3: Test motor lag delays thrust response

**Files:**
- Modify: `tests/sim_test.c`

- [ ] **Step 1: Add the test**

```c
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
    // First-step thrust should be lower with lag than without
    if (lagged[0].thrust >= instant[0].thrust) {
        printf("FAIL: lagged thrust[0] %.3f not < instant thrust[0] %.3f\n",
               lagged[0].thrust, instant[0].thrust);
        return 1;
    }
    return 0;
}
```

Add to `main()`.

- [ ] **Step 2: Run, expect pass**

Run: `cd tests && mingw32-make test`

- [ ] **Step 3: Commit**

```powershell
git add tests/sim_test.c
git commit -m "test: assert motor lag attenuates first-step thrust"
```

---

### Task 2.4: Test sensor noise is bounded by sigma

**Files:**
- Modify: `tests/sim_test.c`

- [ ] **Step 1: Add the test**

```c
static int test_sensor_noise_bounded(void) {
    // With sigma=0, two runs of identical params must be bit-identical
    // (regression on the noise short-circuit).
    SimParams p = {
        .kp = 2.0, .ki = 0.5, .kd = 1.0, .target_alt = 10.0,
        .mass = 1.0, .gravity = 9.81,
        .has_wind = 0, .wind_strength = 0.0,
        .deriv_on_meas = 0, .integrator = 0,
        .motor_tau = 0.0, .sensor_sigma = 0.0,
        .integral_max = 50.0, .thrust_max = 150.0,
        .duration = 5.0, .dt = 0.1, .seed = 1,
    };
    SimSample a[64], b[64];
    simulate(&p, a, 64);
    simulate(&p, b, 64);
    for (int i = 0; i < 50; i++) {
        if (!near(a[i].altitude, b[i].altitude, 1e-12)) {
            printf("FAIL: zero-sigma path not deterministic at i=%d\n", i);
            return 1;
        }
    }
    return 0;
}
```

Add to `main()`.

- [ ] **Step 2: Run, expect pass**

- [ ] **Step 3: Commit**

```powershell
git add tests/sim_test.c
git commit -m "test: assert zero-sigma path is bit-identical"
```

---

### Task 2.5: Make CLI seed deterministic by default for reproducible CSVs

**Files:**
- Modify: `openEndedC.c`

- [ ] **Step 1: Replace the time-based seed with `42` for the SimParams field, but keep `srand(time(NULL))` for any future non-sim use**

In `runLevel`, change:

```c
.seed = (int)time(NULL),
```

to:

```c
.seed = 42,  // Reproducible runs; CLI may expose --seed in v2
```

- [ ] **Step 2: Rebuild and run regression test**

Run: `mingw32-make clean && mingw32-make && mingw32-make test`
Expected: clean build + `All tests passed.`

- [ ] **Step 3: Commit**

```powershell
git add openEndedC.c
git commit -m "refactor: use deterministic seed in CLI (was time-based)"
```

---

### Task 2.6: Add CLI smoke test that runs a non-interactive level

The CLI's `_getch` tuner can't be driven from a pipe, but `simulate()` can. Add a thin debug entry point that runs a fixed config and prints the final altitude — useful for CI smoke checks.

**Files:**
- Modify: `openEndedC.c`

- [ ] **Step 1: Add `--smoke` CLI flag handling at the top of `main()`**

Replace `int main()` with `int main(int argc, char* argv[])`. At the top of `main`, before the `do { displayMenu(); ... }` block, add:

```c
    if (argc >= 2 && argv[1] != NULL) {
        if (argv[1][0] == '-' && argv[1][1] == '-' &&
            argv[1][2] == 's' && argv[1][3] == 'm' &&
            argv[1][4] == 'o' && argv[1][5] == 'k' &&
            argv[1][6] == 'e' && argv[1][7] == '\0') {
            SimParams sp = {
                .kp = 2.0, .ki = 0.5, .kd = 1.0, .target_alt = 10.0,
                .mass = 1.0, .gravity = 9.81,
                .has_wind = 0, .wind_strength = 0.0,
                .deriv_on_meas = 0, .integrator = 0,
                .motor_tau = 0.0, .sensor_sigma = 0.0,
                .integral_max = 50.0, .thrust_max = 150.0,
                .duration = 10.0, .dt = 0.1, .seed = 42,
            };
            SimSample buf[256];
            int n = simulate(&sp, buf, 256);
            if (n <= 0) { printf("smoke: simulate() returned %d\n", n); return 1; }
            printf("smoke: n=%d final_alt=%.4f\n", n, buf[n - 1].altitude);
            return 0;
        }
    }
```

- [ ] **Step 2: Build and run smoke test**

Run: `mingw32-make && ./drone.exe --smoke`
Expected: `smoke: n=100 final_alt=<value-near-10.0>`

- [ ] **Step 3: Commit**

```powershell
git add openEndedC.c
git commit -m "feat: add --smoke CLI flag for CI / pipeable verification"
```

---

### Task 2.7: Add stepinfo regression test for the standard config

**Files:**
- Modify: `tests/sim_test.c`

- [ ] **Step 1: Add the test**

```c
static int test_stepinfo_standard_hover(void) {
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
    double peak = 0.0;
    for (int i = 0; i < n; i++) if (out[i].altitude > peak) peak = out[i].altitude;
    double overshoot = peak - p.target_alt;
    if (overshoot < 0) overshoot = 0;
    if (overshoot > 4.0) {
        printf("FAIL: overshoot %.3f exceeds 4.0\n", overshoot);
        return 1;
    }
    return 0;
}
```

Add to `main()`.

- [ ] **Step 2: Run, expect pass**

- [ ] **Step 3: Commit**

```powershell
git add tests/sim_test.c
git commit -m "test: assert standard hover overshoot stays under 4m"
git push
```

---

## Phase 3 — WASM build

### Task 3.1: Document Emscripten installation

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Append a section to README.md, after "Run", before "Controls"**

```markdown
## Web frontend (WASM build)

The C simulation also compiles to WebAssembly for the browser frontend in `web/`.

### Install Emscripten (one-time)

```powershell
git clone https://github.com/emscripten-core/emsdk.git C:\emsdk
C:\emsdk\emsdk install latest
C:\emsdk\emsdk activate latest
C:\emsdk\emsdk_env.ps1
```

Verify: `emcc --version` should print a version banner.

### Build the WASM module

```sh
mingw32-make -f Makefile.wasm
```

This produces `web/public/sim.wasm` and `web/public/sim_glue.js`.
```

- [ ] **Step 2: Commit**

```powershell
git add README.md
git commit -m "docs: add Emscripten install + WASM build instructions"
```

---

### Task 3.2: Create `Makefile.wasm`

**Files:**
- Create: `Makefile.wasm`

- [ ] **Step 1: Write the Makefile**

```makefile
# Compiles sim.c to web/public/sim.wasm + sim_glue.js via Emscripten.
# Requires emcc on PATH (run emsdk_env first).

EMCC      := emcc
SRC       := sim.c
HEADER    := sim.h
OUT_DIR   := web/public
GLUE      := $(OUT_DIR)/sim_glue.js
WASM      := $(OUT_DIR)/sim.wasm

EMFLAGS := -O3 -s WASM=1 -s MODULARIZE=1 -s EXPORT_ES6=1 \
           -s EXPORT_NAME=createSimModule \
           -s ENVIRONMENT=web \
           -s ALLOW_MEMORY_GROWTH=1 \
           -s EXPORTED_FUNCTIONS='["_simulate","_malloc","_free"]' \
           -s EXPORTED_RUNTIME_METHODS='["HEAPF64","HEAP32","HEAPU8"]'

.PHONY: wasm clean

wasm: $(GLUE)

$(GLUE): $(SRC) $(HEADER)
	@mkdir -p $(OUT_DIR)
	$(EMCC) $(EMFLAGS) $(SRC) -o $(GLUE)

clean:
	-rm -f $(GLUE) $(WASM)
```

- [ ] **Step 2: Build the WASM artifact**

Run (after activating emsdk):
```sh
mingw32-make -f Makefile.wasm
```
Expected: produces `web/public/sim.wasm` and `web/public/sim_glue.js`.

- [ ] **Step 3: Verify file sizes are sane**

Run: `ls -la web/public/`
Expected: `sim.wasm` < 50 kB, `sim_glue.js` < 30 kB.

- [ ] **Step 4: Commit**

```powershell
git add Makefile.wasm
git commit -m "build: add Makefile.wasm to compile sim.c via Emscripten"
git push
```

---

### Task 3.3: Smoke-test the WASM module from a tiny Node.js script

**Files:**
- Create: `tests/wasm_smoke.mjs`

- [ ] **Step 1: Write the smoke test**

```javascript
// tests/wasm_smoke.mjs
// Loads sim.wasm via the Emscripten glue and calls simulate() once.
// Run: node tests/wasm_smoke.mjs
import createSimModule from '../web/public/sim_glue.js';

const SimParamsBytes  = 16 * 8;   // 16 fields × 8 bytes (doubles + ints stored as 8B-aligned)
const SimSampleFloats = 7;        // 7 doubles per sample
const SimSampleBytes  = SimSampleFloats * 8;

const Module = await createSimModule();
const paramsPtr  = Module._malloc(SimParamsBytes);
const samplesMax = 256;
const samplesPtr = Module._malloc(samplesMax * SimSampleBytes);

// Layout matches sim.h SimParams (alignment: 4 ints are stored as 8-byte slots
// because the WASM struct alignment on 64-bit doubles pads everything to 8).
const f64 = new Float64Array(Module.HEAPF64.buffer, paramsPtr, SimParamsBytes / 8);
const i32 = new Int32Array(Module.HEAP32.buffer, paramsPtr, SimParamsBytes / 4);

// IMPORTANT: the actual struct layout depends on the C compiler's struct
// packing. Use the helper from web/src/lib/wasm.js once it exists (Phase 4).
// This smoke test just confirms the module loads and _simulate is callable.

if (typeof Module._simulate !== 'function') {
    console.error('FAIL: _simulate is not exported');
    process.exit(1);
}
console.log('PASS: WASM module loaded; _simulate is exported');

Module._free(paramsPtr);
Module._free(samplesPtr);
```

- [ ] **Step 2: Run the smoke test**

Run: `node tests/wasm_smoke.mjs`
Expected: `PASS: WASM module loaded; _simulate is exported`

- [ ] **Step 3: Commit**

```powershell
git add tests/wasm_smoke.mjs
git commit -m "test: add Node.js smoke test for WASM module load"
git push
```

---

## Phase 4 — Svelte frontend bootstrap

### Task 4.1: Initialize the `web/` workspace with Vite + Svelte

**Files:**
- Create: `web/package.json`, `web/vite.config.js`, `web/index.html`, `web/src/main.js`, `web/src/App.svelte`, `web/src/app.css`

- [ ] **Step 1: Write `web/package.json`**

```json
{
  "name": "drone-pid-sandbox",
  "private": true,
  "version": "0.1.0",
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "vite build",
    "preview": "vite preview",
    "test": "vitest run"
  },
  "devDependencies": {
    "@sveltejs/vite-plugin-svelte": "^3.0.2",
    "autoprefixer": "^10.4.18",
    "postcss": "^8.4.35",
    "svelte": "^4.2.12",
    "tailwindcss": "^3.4.1",
    "vite": "^5.1.6",
    "vitest": "^1.3.1",
    "jsdom": "^24.0.0"
  },
  "dependencies": {
    "uplot": "^1.6.30"
  }
}
```

- [ ] **Step 2: Write `web/vite.config.js`**

```javascript
import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig({
    plugins: [svelte()],
    server: { port: 5173, open: false },
    build: { target: 'es2022', sourcemap: true },
    test: {
        environment: 'jsdom',
        include: ['src/test/**/*.test.js']
    }
});
```

- [ ] **Step 3: Write `web/postcss.config.js`**

```javascript
export default {
    plugins: {
        tailwindcss: {},
        autoprefixer: {}
    }
};
```

- [ ] **Step 4: Write `web/tailwind.config.js`**

```javascript
export default {
    content: ['./index.html', './src/**/*.{js,svelte}'],
    theme: {
        extend: {
            colors: {
                accent: '#22d3ee',
                target: '#facc15'
            }
        }
    },
    plugins: []
};
```

- [ ] **Step 5: Write `web/src/app.css`**

```css
@tailwind base;
@tailwind components;
@tailwind utilities;

html, body { height: 100%; }
body { background: #0b1220; color: #e2e8f0; font-family: ui-sans-serif, system-ui, sans-serif; }
```

- [ ] **Step 6: Write `web/index.html`**

```html
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Drone PID Sandbox</title>
  <meta name="description" content="Interactive PID controller sandbox running a C simulation in the browser via WebAssembly." />
</head>
<body>
  <div id="app"></div>
  <script type="module" src="/src/main.js"></script>
</body>
</html>
```

- [ ] **Step 7: Write `web/src/main.js`**

```javascript
import './app.css';
import App from './App.svelte';

const app = new App({ target: document.getElementById('app') });
export default app;
```

- [ ] **Step 8: Write a placeholder `web/src/App.svelte`**

```svelte
<main class="min-h-screen p-6">
    <h1 class="text-2xl font-bold">Drone PID Sandbox</h1>
    <p class="text-slate-400 mt-2">Bootstrapping…</p>
</main>
```

- [ ] **Step 9: Install dependencies and run dev**

Run:
```powershell
cd web
npm install
npm run dev
```
Expected: Vite prints `Local: http://localhost:5173/`. Open the URL in a browser; should see the title and "Bootstrapping…". Stop with Ctrl+C.

- [ ] **Step 10: Commit**

```powershell
cd ..
git add web/package.json web/package-lock.json web/vite.config.js web/postcss.config.js web/tailwind.config.js web/index.html web/src/main.js web/src/App.svelte web/src/app.css
git commit -m "feat(web): bootstrap Vite + Svelte + Tailwind workspace"
```

---

### Task 4.2: Add `lib/wasm.js` — load the module and expose `simulate(params)`

**Files:**
- Create: `web/src/lib/wasm.js`

- [ ] **Step 1: Write the loader**

```javascript
// web/src/lib/wasm.js
// Loads sim.wasm and exposes simulate(params) returning an array of SimSample objects.
//
// Struct layout — must match the C compiler's packing of sim.h SimParams.
// Adjacent int32 fields share an 8-byte slot (deriv_on_meas + integrator).
// has_wind sits in slot 6 with 4 bytes of trailing pad before wind_strength.
// seed sits in slot 15 with 4 bytes of trailing pad to round struct to 128B.

const PARAM_LAYOUT = [
    { name: 'kp',             type: 'f64', offset: 0   },
    { name: 'ki',             type: 'f64', offset: 8   },
    { name: 'kd',             type: 'f64', offset: 16  },
    { name: 'target_alt',     type: 'f64', offset: 24  },
    { name: 'mass',           type: 'f64', offset: 32  },
    { name: 'gravity',        type: 'f64', offset: 40  },
    { name: 'has_wind',       type: 'i32', offset: 48  },   // slot 6 lo; 52..55 pad
    { name: 'wind_strength',  type: 'f64', offset: 56  },
    { name: 'deriv_on_meas',  type: 'i32', offset: 64  },   // slot 8 lo
    { name: 'integrator',     type: 'i32', offset: 68  },   // slot 8 hi (packed)
    { name: 'motor_tau',      type: 'f64', offset: 72  },
    { name: 'sensor_sigma',   type: 'f64', offset: 80  },
    { name: 'integral_max',   type: 'f64', offset: 88  },
    { name: 'thrust_max',     type: 'f64', offset: 96  },
    { name: 'duration',       type: 'f64', offset: 104 },
    { name: 'dt',             type: 'f64', offset: 112 },
    { name: 'seed',           type: 'i32', offset: 120 }    // slot 15 lo; 124..127 pad
];
const PARAM_BYTES  = 128;
const SAMPLE_BYTES = 7 * 8;     // 7 doubles per SimSample
const MAX_SAMPLES  = 4096;

let modulePromise = null;
function loadModule() {
    if (!modulePromise) {
        modulePromise = import('/sim_glue.js').then(({ default: factory }) => factory());
    }
    return modulePromise;
}

export async function simulate(params) {
    const Module = await loadModule();
    const paramsPtr  = Module._malloc(PARAM_BYTES);
    const samplesPtr = Module._malloc(MAX_SAMPLES * SAMPLE_BYTES);
    try {
        // Zero the struct first so any padding bytes are deterministic.
        new Uint8Array(Module.HEAPU8.buffer, paramsPtr, PARAM_BYTES).fill(0);
        const dv = new DataView(Module.HEAPU8.buffer, paramsPtr, PARAM_BYTES);
        for (const f of PARAM_LAYOUT) {
            if (f.type === 'f64') {
                dv.setFloat64(f.offset, Number(params[f.name]), true);   // little-endian
            } else {
                dv.setInt32(f.offset, params[f.name] | 0, true);
            }
        }
        const n = Module._simulate(paramsPtr, samplesPtr, MAX_SAMPLES);
        const out = new Array(n);
        const view = new Float64Array(Module.HEAPF64.buffer, samplesPtr, n * 7);
        for (let i = 0; i < n; i++) {
            const o = i * 7;
            out[i] = {
                time:     view[o],
                altitude: view[o + 1],
                error:    view[o + 2],
                thrust:   view[o + 3],
                p_term:   view[o + 4],
                i_term:   view[o + 5],
                d_term:   view[o + 6]
            };
        }
        return out;
    } finally {
        Module._free(paramsPtr);
        Module._free(samplesPtr);
    }
}

export function isWasmSupported() {
    return typeof WebAssembly === 'object'
        && typeof WebAssembly.instantiate === 'function';
}
```

> **Verifying the layout:** at runtime, the engineer should add a one-time C helper (during Phase 4 dev only) that prints `sizeof(SimParams)` and `offsetof(SimParams, integrator)` etc., compile it natively, and confirm the offsets above match. If a different toolchain pads the struct differently, update the offsets here.

- [ ] **Step 2: Update `App.svelte` to do a one-shot smoke call**

```svelte
<script>
    import { onMount } from 'svelte';
    import { simulate } from './lib/wasm.js';

    let status = 'loading';
    let result = null;

    onMount(async () => {
        try {
            const samples = await simulate({
                kp: 2, ki: 0.5, kd: 1, target_alt: 10,
                mass: 1, gravity: 9.81,
                has_wind: 0, wind_strength: 0,
                deriv_on_meas: 0, integrator: 0,
                motor_tau: 0, sensor_sigma: 0,
                integral_max: 50, thrust_max: 150,
                duration: 10, dt: 0.1, seed: 42
            });
            result = samples;
            status = 'ok';
        } catch (e) {
            status = 'error: ' + e.message;
        }
    });
</script>

<main class="min-h-screen p-6">
    <h1 class="text-2xl font-bold">Drone PID Sandbox</h1>
    <p class="text-slate-400 mt-2">Status: {status}</p>
    {#if result}
        <p class="text-slate-300 mt-2">
            Got {result.length} samples. Final altitude: {result[result.length - 1].altitude.toFixed(3)} m
        </p>
    {/if}
</main>
```

- [ ] **Step 3: Run dev and verify in browser**

Run: `cd web && npm run dev`
Open: `http://localhost:5173/`
Expected: page shows `Status: ok` and `Got 100 samples. Final altitude: ~10.000 m`.

- [ ] **Step 4: Commit**

```powershell
git add web/src/lib/wasm.js web/src/App.svelte
git commit -m "feat(web): WASM loader + smoke render of 100-sample run"
git push
```

---

## Phase 5 — Stores and pure logic

### Task 5.1: Implement `lib/stepinfo.js`

**Files:**
- Create: `web/src/lib/stepinfo.js`
- Create: `web/src/test/stepinfo.test.js`

- [ ] **Step 1: Write the failing test**

```javascript
// web/src/test/stepinfo.test.js
import { describe, it, expect } from 'vitest';
import { computeStepInfo } from '../lib/stepinfo.js';

function makeRamp(target, n, dt) {
    return Array.from({ length: n }, (_, i) => ({
        time: i * dt,
        altitude: Math.min(target, target * (i / 30)),
        error: 0, thrust: 0, p_term: 0, i_term: 0, d_term: 0
    }));
}

describe('computeStepInfo', () => {
    it('returns peak equal to max altitude', () => {
        const samples = makeRamp(10, 100, 0.1);
        const info = computeStepInfo(samples, 10);
        expect(info.peak).toBeCloseTo(10, 5);
    });

    it('zero overshoot when monotonically rising to target', () => {
        const samples = makeRamp(10, 100, 0.1);
        const info = computeStepInfo(samples, 10);
        expect(info.overshoot).toBe(0);
    });

    it('reports null rise time when target never reached', () => {
        const samples = makeRamp(5, 100, 0.1);  // ramps to 5, target 10
        const info = computeStepInfo(samples, 10);
        expect(info.riseTime).toBeNull();
    });

    it('averages last 1s for steady-state error', () => {
        const samples = Array.from({ length: 100 }, (_, i) => ({
            time: i * 0.1,
            altitude: i < 90 ? 5 : 10,  // jumps to target at t=9s
            error: 0, thrust: 0, p_term: 0, i_term: 0, d_term: 0
        }));
        const info = computeStepInfo(samples, 10);
        expect(info.steadyError).toBeCloseTo(0, 5);
    });
});
```

- [ ] **Step 2: Run, expect fail (file missing)**

Run: `cd web && npm test`
Expected: error — `Cannot find module '../lib/stepinfo.js'`

- [ ] **Step 3: Implement `stepinfo.js`**

```javascript
// web/src/lib/stepinfo.js
// Pure functions over SimSample[] returning step-response metrics.

export function computeStepInfo(samples, target) {
    if (!samples || samples.length === 0) {
        return { peak: 0, overshoot: 0, riseTime: null, settlingTime: null, steadyError: 0 };
    }
    const n = samples.length;
    const dt = n > 1 ? samples[1].time - samples[0].time : 0.1;

    let peak = 0;
    for (const s of samples) if (s.altitude > peak) peak = s.altitude;

    const overshoot = Math.max(0, peak - target);

    // Rise time: 10% to 90% crossings
    let riseStart = null, riseEnd = null;
    for (const s of samples) {
        if (riseStart === null && s.altitude >= 0.1 * target) riseStart = s.time;
        if (s.altitude >= 0.9 * target) { riseEnd = s.time; break; }
    }
    const riseTime = (riseStart !== null && riseEnd !== null) ? (riseEnd - riseStart) : null;

    // Settling time: last exit from +/-2% band (floor 0.05 m for tiny targets)
    const band = Math.max(0.02 * target, 0.05);
    let settlingTime = null;
    for (let i = n - 1; i >= 0; i--) {
        if (Math.abs(samples[i].altitude - target) > band) {
            if (i + 1 < n) settlingTime = samples[i + 1].time;
            break;
        }
    }

    // Steady-state error: average over last ~1 second
    const tail = Math.min(n, Math.max(1, Math.round(1 / dt)));
    let sum = 0;
    for (let i = n - tail; i < n; i++) sum += samples[i].altitude;
    const steadyError = Math.abs(target - sum / tail);

    return { peak, overshoot, riseTime, settlingTime, steadyError };
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cd web && npm test`
Expected: 4/4 passing.

- [ ] **Step 5: Commit**

```powershell
git add web/src/lib/stepinfo.js web/src/test/stepinfo.test.js
git commit -m "feat(web): stepinfo metrics with vitest coverage"
```

---

### Task 5.2: Implement `lib/store.js` — params, currentRun, pinned

**Files:**
- Create: `web/src/lib/store.js`
- Create: `web/src/lib/colors.js`
- Create: `web/src/test/store.test.js`

- [ ] **Step 1: Write the colors module**

```javascript
// web/src/lib/colors.js
// Cycle of distinct colors for pinned curves.
export const PIN_COLORS = ['#f87171', '#fbbf24', '#34d399', '#60a5fa', '#a78bfa'];
```

- [ ] **Step 2: Write the failing store test**

```javascript
// web/src/test/store.test.js
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { get } from 'svelte/store';
import { params, currentRun, pinned, pinCurrent, clearPins, defaultParams } from '../lib/store.js';

vi.mock('../lib/wasm.js', () => ({
    simulate: vi.fn(async (p) => Array.from({ length: 10 }, (_, i) => ({
        time: i * 0.1, altitude: i, error: 0, thrust: 0,
        p_term: 0, i_term: 0, d_term: 0
    })))
}));

describe('store', () => {
    beforeEach(() => {
        clearPins();
        params.set({ ...defaultParams });
    });

    it('exposes default params', () => {
        const p = get(params);
        expect(p.kp).toBe(2.0);
        expect(p.target_alt).toBe(10);
    });

    it('updates currentRun when params change (after debounce)', async () => {
        params.update(p => ({ ...p, kp: 5 }));
        await new Promise(r => setTimeout(r, 30));
        const run = get(currentRun);
        expect(run).not.toBeNull();
        expect(run.samples.length).toBe(10);
    });

    it('pinCurrent appends to pinned with a color', async () => {
        params.update(p => ({ ...p, kp: 5 }));
        await new Promise(r => setTimeout(r, 30));
        pinCurrent();
        const arr = get(pinned);
        expect(arr.length).toBe(1);
        expect(arr[0].color).toMatch(/^#/);
    });

    it('caps pinned at 5 with FIFO eviction', async () => {
        for (let i = 0; i < 6; i++) {
            params.update(p => ({ ...p, kp: i + 1 }));
            await new Promise(r => setTimeout(r, 30));
            pinCurrent();
        }
        const arr = get(pinned);
        expect(arr.length).toBe(5);
        expect(arr[0].params.kp).toBe(2);  // first one (kp=1) was evicted
    });
});
```

- [ ] **Step 3: Run, expect fail (store.js missing)**

Run: `cd web && npm test`

- [ ] **Step 4: Implement `store.js`**

```javascript
// web/src/lib/store.js
import { writable, get } from 'svelte/store';
import { simulate } from './wasm.js';
import { PIN_COLORS } from './colors.js';

export const defaultParams = {
    kp: 2.0, ki: 0.5, kd: 1.0, target_alt: 10,
    mass: 1.0, gravity: 9.81,
    has_wind: 0, wind_strength: 4.0,
    deriv_on_meas: 0, integrator: 0,
    motor_tau: 0.0, sensor_sigma: 0.0,
    integral_max: 50.0, thrust_max: 150.0,
    duration: 10.0, dt: 0.1, seed: 42
};

export const params      = writable({ ...defaultParams });
export const currentRun  = writable(null);     // { params, samples }
export const pinned      = writable([]);       // [{ params, samples, color }]
export const simError    = writable(null);

const MAX_PINNED = 5;
let pinIndex = 0;
let debounceTimer = null;

function reSimulate(p) {
    if (debounceTimer) clearTimeout(debounceTimer);
    debounceTimer = setTimeout(async () => {
        try {
            const samples = await simulate(p);
            currentRun.set({ params: { ...p }, samples });
            simError.set(null);
        } catch (e) {
            simError.set(e.message || String(e));
        }
    }, 16);
}

params.subscribe(reSimulate);

export function pinCurrent() {
    const run = get(currentRun);
    if (!run) return;
    const color = PIN_COLORS[pinIndex % PIN_COLORS.length];
    pinIndex++;
    pinned.update(arr => {
        const next = [...arr, { params: { ...run.params }, samples: run.samples, color }];
        if (next.length > MAX_PINNED) next.shift();
        return next;
    });
}

export function clearPins() {
    pinned.set([]);
    pinIndex = 0;
}
```

- [ ] **Step 5: Run, expect pass**

Run: `cd web && npm test`
Expected: 4/4 passing in store.test.js (plus stepinfo tests still passing).

- [ ] **Step 6: Commit**

```powershell
git add web/src/lib/store.js web/src/lib/colors.js web/src/test/store.test.js
git commit -m "feat(web): reactive params store + pinned curves with FIFO eviction"
git push
```

---

## Phase 6 — Components and layout

### Task 6.1: Reusable `<Slider>` component

**Files:**
- Create: `web/src/components/Slider.svelte`

- [ ] **Step 1: Write the component**

```svelte
<script>
    export let label;
    export let value;
    export let min;
    export let max;
    export let step = 0.01;
    export let unit = '';
    export let help = '';
</script>

<label class="block text-sm">
    <div class="flex justify-between mb-1">
        <span class="text-slate-300">
            {label}
            {#if help}<span class="text-slate-500 cursor-help" title={help}>?</span>{/if}
        </span>
        <span class="text-accent font-mono">{value.toFixed(2)} {unit}</span>
    </div>
    <input
        type="range"
        bind:value
        {min}
        {max}
        {step}
        class="w-full accent-accent"
    />
</label>
```

- [ ] **Step 2: Commit (no behavior to test independently — used by ControlsBasic next)**

```powershell
git add web/src/components/Slider.svelte
git commit -m "feat(web): reusable Slider component with help tooltip"
```

---

### Task 6.2: `<ControlsBasic>` component

**Files:**
- Create: `web/src/components/ControlsBasic.svelte`

- [ ] **Step 1: Write the component**

```svelte
<script>
    import { params } from '../lib/store.js';
    import Slider from './Slider.svelte';

    let p;
    params.subscribe(v => p = v);
    function update(field, value) { params.update(curr => ({ ...curr, [field]: value })); }
</script>

<section class="space-y-3">
    <h2 class="text-lg font-semibold text-slate-100">Controls</h2>

    <Slider
        label="Kp (Proportional)" min={0} max={20} step={0.1}
        value={p.kp}
        on:input help="Proportional gain. Higher = faster response, more overshoot."
        bind:value={p.kp}
    />
    <Slider
        label="Ki (Integral)" min={0} max={10} step={0.1}
        bind:value={p.ki}
        help="Integral gain. Eliminates steady-state error; high values cause windup."
    />
    <Slider
        label="Kd (Derivative)" min={0} max={20} step={0.1}
        bind:value={p.kd}
        help="Derivative gain. Damps oscillation; sensitive to noise."
    />
    <Slider
        label="Target Altitude" min={1} max={30} step={1} unit="m"
        bind:value={p.target_alt}
    />
    <Slider
        label="Mass" min={0.5} max={10} step={0.1} unit="kg"
        bind:value={p.mass}
    />

    <label class="flex items-center gap-2 text-sm text-slate-300">
        <input
            type="checkbox"
            checked={p.has_wind === 1}
            on:change={(e) => update('has_wind', e.currentTarget.checked ? 1 : 0)}
            class="accent-accent"
        />
        Wind enabled
    </label>

    {#if p.has_wind === 1}
        <Slider
            label="Wind strength" min={0} max={10} step={0.5} unit="N"
            bind:value={p.wind_strength}
        />
    {/if}
</section>

<style>
    section :global(input[type="range"]) { height: 1.25rem; }
</style>
```

Note: `bind:value` on the Slider won't auto-flow into the store. Replace each Slider use with explicit handlers OR simplify Slider to emit input events. The cleanest version uses local 2-way binding plus a reactive write back:

```svelte
<script>
    import { params } from '../lib/store.js';
    import Slider from './Slider.svelte';

    let p;
    params.subscribe(v => p = v);

    function set(field, value) {
        params.update(curr => ({ ...curr, [field]: value }));
    }
</script>

<section class="space-y-3">
    <h2 class="text-lg font-semibold text-slate-100">Controls</h2>

    <Slider label="Kp (Proportional)" min={0} max={20} step={0.1}
            value={p.kp}
            help="Proportional gain. Higher = faster response, more overshoot." />
    <Slider label="Ki (Integral)" min={0} max={10} step={0.1}
            value={p.ki}
            help="Integral gain. Eliminates steady-state error; can wind up." />
    <Slider label="Kd (Derivative)" min={0} max={20} step={0.1}
            value={p.kd}
            help="Derivative gain. Damps oscillation; sensitive to noise." />
    <Slider label="Target Altitude" min={1} max={30} step={1} unit="m"
            value={p.target_alt} />
    <Slider label="Mass" min={0.5} max={10} step={0.1} unit="kg"
            value={p.mass} />

    <label class="flex items-center gap-2 text-sm text-slate-300">
        <input type="checkbox"
               checked={p.has_wind === 1}
               on:change={(e) => set('has_wind', e.currentTarget.checked ? 1 : 0)}
               class="accent-accent" />
        Wind enabled
    </label>

    {#if p.has_wind === 1}
        <Slider label="Wind strength" min={0} max={10} step={0.5} unit="N"
                value={p.wind_strength} />
    {/if}
</section>
```

For this to actually update the store, refactor `Slider.svelte` to dispatch a `change` event AND give it a `field` prop, OR adopt 2-way binding via a setter. Simplest fix: have ControlsBasic own the inputs directly (no Slider abstraction for now) and revisit Slider in v2.

**Replace** `ControlsBasic.svelte` with this direct version:

```svelte
<script>
    import { params } from '../lib/store.js';

    let p;
    params.subscribe(v => p = v);

    function set(field) {
        return (e) => {
            const v = e.currentTarget.type === 'checkbox'
                ? (e.currentTarget.checked ? 1 : 0)
                : parseFloat(e.currentTarget.value);
            params.update(curr => ({ ...curr, [field]: v }));
        };
    }
</script>

<section class="space-y-3">
    <h2 class="text-lg font-semibold text-slate-100">Controls</h2>

    <label class="block text-sm">
        <span class="text-slate-300">Kp <span class="text-accent font-mono float-right">{p.kp.toFixed(2)}</span></span>
        <input type="range" min="0" max="20" step="0.1" value={p.kp} on:input={set('kp')} class="w-full accent-accent" />
    </label>
    <label class="block text-sm">
        <span class="text-slate-300">Ki <span class="text-accent font-mono float-right">{p.ki.toFixed(2)}</span></span>
        <input type="range" min="0" max="10" step="0.1" value={p.ki} on:input={set('ki')} class="w-full accent-accent" />
    </label>
    <label class="block text-sm">
        <span class="text-slate-300">Kd <span class="text-accent font-mono float-right">{p.kd.toFixed(2)}</span></span>
        <input type="range" min="0" max="20" step="0.1" value={p.kd} on:input={set('kd')} class="w-full accent-accent" />
    </label>
    <label class="block text-sm">
        <span class="text-slate-300">Target altitude <span class="text-accent font-mono float-right">{p.target_alt.toFixed(0)} m</span></span>
        <input type="range" min="1" max="30" step="1" value={p.target_alt} on:input={set('target_alt')} class="w-full accent-accent" />
    </label>
    <label class="block text-sm">
        <span class="text-slate-300">Mass <span class="text-accent font-mono float-right">{p.mass.toFixed(2)} kg</span></span>
        <input type="range" min="0.5" max="10" step="0.1" value={p.mass} on:input={set('mass')} class="w-full accent-accent" />
    </label>

    <label class="flex items-center gap-2 text-sm text-slate-300">
        <input type="checkbox" checked={p.has_wind === 1} on:change={set('has_wind')} class="accent-accent" />
        Wind enabled
    </label>

    {#if p.has_wind === 1}
        <label class="block text-sm">
            <span class="text-slate-300">Wind strength <span class="text-accent font-mono float-right">{p.wind_strength.toFixed(1)} N</span></span>
            <input type="range" min="0" max="10" step="0.5" value={p.wind_strength} on:input={set('wind_strength')} class="w-full accent-accent" />
        </label>
    {/if}
</section>
```

Delete the unused `Slider.svelte` to keep the tree clean: `git rm web/src/components/Slider.svelte`.

- [ ] **Step 2: Commit**

```powershell
git add web/src/components/ControlsBasic.svelte
git rm web/src/components/Slider.svelte
git commit -m "feat(web): basic controls (Kp/Ki/Kd/target/mass/wind)"
```

---

### Task 6.3: `<ControlsAdvanced>` component (collapsible accordion)

**Files:**
- Create: `web/src/components/ControlsAdvanced.svelte`

- [ ] **Step 1: Write the component**

```svelte
<script>
    import { params } from '../lib/store.js';

    let open = false;
    let p;
    params.subscribe(v => p = v);

    function set(field) {
        return (e) => {
            const v = e.currentTarget.type === 'number'
                ? parseInt(e.currentTarget.value, 10) | 0
                : parseFloat(e.currentTarget.value);
            params.update(curr => ({ ...curr, [field]: v }));
        };
    }
    function setSelect(field) {
        return (e) => params.update(curr => ({ ...curr, [field]: parseInt(e.currentTarget.value, 10) }));
    }
</script>

<section class="border-t border-slate-700 pt-3">
    <button
        class="text-sm text-slate-300 hover:text-accent flex items-center gap-2"
        on:click={() => open = !open}
    >
        <span>{open ? '▾' : '▸'}</span>
        <span>Advanced</span>
    </button>

    {#if open}
        <div class="space-y-3 mt-3">
            <label class="block text-sm">
                <span class="text-slate-300">Gravity <span class="text-accent font-mono float-right">{p.gravity.toFixed(2)} m/s²</span></span>
                <input type="range" min="1" max="25" step="0.1" value={p.gravity} on:input={set('gravity')} class="w-full accent-accent" />
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">Integrator</span>
                <select value={p.integrator} on:change={setSelect('integrator')}
                        class="w-full mt-1 bg-slate-800 text-slate-200 p-1 rounded">
                    <option value={0}>Forward Euler</option>
                    <option value={1}>RK4</option>
                </select>
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">Derivative on</span>
                <select value={p.deriv_on_meas} on:change={setSelect('deriv_on_meas')}
                        class="w-full mt-1 bg-slate-800 text-slate-200 p-1 rounded">
                    <option value={0}>Error</option>
                    <option value={1}>Measurement</option>
                </select>
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">Motor lag τ <span class="text-accent font-mono float-right">{p.motor_tau.toFixed(3)} s</span></span>
                <input type="range" min="0" max="0.2" step="0.005" value={p.motor_tau} on:input={set('motor_tau')} class="w-full accent-accent" />
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">Sensor noise σ <span class="text-accent font-mono float-right">{p.sensor_sigma.toFixed(2)} m</span></span>
                <input type="range" min="0" max="0.5" step="0.01" value={p.sensor_sigma} on:input={set('sensor_sigma')} class="w-full accent-accent" />
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">Anti-windup clamp <span class="text-accent font-mono float-right">{p.integral_max.toFixed(0)}</span></span>
                <input type="range" min="5" max="200" step="5" value={p.integral_max} on:input={set('integral_max')} class="w-full accent-accent" />
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">Thrust ceiling <span class="text-accent font-mono float-right">{p.thrust_max.toFixed(0)} N</span></span>
                <input type="range" min="20" max="500" step="5" value={p.thrust_max} on:input={set('thrust_max')} class="w-full accent-accent" />
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">Duration <span class="text-accent font-mono float-right">{p.duration.toFixed(0)} s</span></span>
                <input type="range" min="2" max="30" step="1" value={p.duration} on:input={set('duration')} class="w-full accent-accent" />
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">dt <span class="text-accent font-mono float-right">{p.dt.toFixed(3)} s</span></span>
                <input type="range" min="0.001" max="0.1" step="0.001" value={p.dt} on:input={set('dt')} class="w-full accent-accent" />
            </label>

            <label class="block text-sm">
                <span class="text-slate-300">Seed</span>
                <input type="number" value={p.seed} on:input={set('seed')}
                       class="w-full mt-1 bg-slate-800 text-slate-200 p-1 rounded" />
            </label>
        </div>
    {/if}
</section>
```

- [ ] **Step 2: Commit**

```powershell
git add web/src/components/ControlsAdvanced.svelte
git commit -m "feat(web): advanced controls accordion (10 params)"
```

---

### Task 6.4: `<Chart>` component (uPlot wrapper)

**Files:**
- Create: `web/src/components/Chart.svelte`

- [ ] **Step 1: Write the component**

```svelte
<script>
    import { onMount, onDestroy } from 'svelte';
    import uPlot from 'uplot';
    import 'uplot/dist/uPlot.min.css';
    import { currentRun, pinned, params } from '../lib/store.js';

    let container;
    let plot = null;
    let unsubRun, unsubPinned, unsubParams;

    function buildOptions(target, w, h) {
        return {
            width: w, height: h,
            scales: { x: { time: false }, y: { auto: true } },
            axes: [
                { stroke: '#94a3b8', grid: { stroke: '#1e293b' } },
                { stroke: '#94a3b8', grid: { stroke: '#1e293b' } }
            ],
            series: [
                {},
                { label: 'Drone', stroke: '#22d3ee', width: 2 },
                { label: 'Target', stroke: '#facc15', dash: [4, 4], width: 1 }
            ]
        };
    }

    function rebuild() {
        if (!container) return;
        const run = $currentRun;
        const ps  = $pinned;
        const target = $params.target_alt;
        const w = container.clientWidth || 600;
        const h = 360;

        if (plot) { plot.destroy(); plot = null; }
        if (!run) return;

        const t = run.samples.map(s => s.time);
        const a = run.samples.map(s => s.altitude);
        const tg = t.map(() => target);

        const series = [
            {},
            { label: 'Drone',  stroke: '#22d3ee', width: 2 },
            { label: 'Target', stroke: '#facc15', dash: [4, 4], width: 1 }
        ];
        const data = [t, a, tg];

        // Pinned curves rendered first (drawn behind)
        for (const pin of ps) {
            const pa = pin.samples.map(s => s.altitude);
            data.push(pa);
            series.push({ label: `pin Kp=${pin.params.kp.toFixed(1)}`, stroke: pin.color, width: 1, alpha: 0.5 });
        }

        plot = new uPlot({ width: w, height: h, series }, data, container);
    }

    onMount(() => {
        unsubRun    = currentRun.subscribe(rebuild);
        unsubPinned = pinned.subscribe(rebuild);
        unsubParams = params.subscribe(rebuild);
        const ro = new ResizeObserver(rebuild);
        ro.observe(container);
        return () => ro.disconnect();
    });

    onDestroy(() => {
        unsubRun?.(); unsubPinned?.(); unsubParams?.();
        if (plot) plot.destroy();
    });
</script>

<div bind:this={container} class="w-full bg-slate-900 rounded p-2 min-h-[360px]"></div>
```

- [ ] **Step 2: Commit**

```powershell
git add web/src/components/Chart.svelte
git commit -m "feat(web): uPlot chart with current + pinned curves"
```

---

### Task 6.5: `<StepInfo>` component

**Files:**
- Create: `web/src/components/StepInfo.svelte`

- [ ] **Step 1: Write the component**

```svelte
<script>
    import { currentRun, params } from '../lib/store.js';
    import { computeStepInfo } from '../lib/stepinfo.js';

    let info = null;
    let target = 10;
    currentRun.subscribe(run => {
        if (!run) { info = null; return; }
        info = computeStepInfo(run.samples, run.params.target_alt);
    });
    params.subscribe(p => target = p.target_alt);

    function fmt(v, suffix = '') {
        if (v === null) return '—';
        return v.toFixed(2) + suffix;
    }

    function grade(i) {
        if (!i) return { label: '...', color: 'text-slate-400' };
        if (i.steadyError > 1.5 || i.overshoot > 4) return { label: 'Unstable', color: 'text-red-400' };
        if (i.steadyError > 0.5 || i.overshoot > 1.5) return { label: 'Acceptable', color: 'text-amber-400' };
        return { label: 'Optimal', color: 'text-emerald-400' };
    }

    $: g = grade(info);
</script>

<section class="bg-slate-900 rounded p-4 space-y-1 text-sm">
    <h2 class="text-lg font-semibold mb-2">Step Info</h2>
    {#if !info}
        <p class="text-slate-500">Drag a slider to run.</p>
    {:else}
        <div class="grid grid-cols-2 gap-y-1">
            <span class="text-slate-400">Peak</span>           <span class="font-mono">{fmt(info.peak, ' m')}</span>
            <span class="text-slate-400">Overshoot</span>      <span class="font-mono">{fmt(info.overshoot, ' m')}</span>
            <span class="text-slate-400">Rise time</span>      <span class="font-mono">{fmt(info.riseTime, ' s')}</span>
            <span class="text-slate-400">Settling time</span>  <span class="font-mono">{fmt(info.settlingTime, ' s')}</span>
            <span class="text-slate-400">Steady error</span>   <span class="font-mono">{fmt(info.steadyError, ' m')}</span>
            <span class="text-slate-400">Grade</span>          <span class="font-semibold {g.color}">{g.label}</span>
        </div>
    {/if}
</section>
```

- [ ] **Step 2: Commit**

```powershell
git add web/src/components/StepInfo.svelte
git commit -m "feat(web): step-info panel with grade indicator"
```

---

### Task 6.6: `<PinList>` component

**Files:**
- Create: `web/src/components/PinList.svelte`

- [ ] **Step 1: Write the component**

```svelte
<script>
    import { pinned, pinCurrent, clearPins } from '../lib/store.js';
</script>

<section class="bg-slate-900 rounded p-4 space-y-2 text-sm">
    <div class="flex items-center justify-between">
        <h2 class="text-lg font-semibold">Pinned ({$pinned.length}/5)</h2>
        <div class="flex gap-2">
            <button
                class="px-3 py-1 bg-accent text-slate-900 rounded font-medium hover:opacity-90"
                on:click={pinCurrent}
            >Pin current</button>
            {#if $pinned.length > 0}
                <button
                    class="px-3 py-1 bg-slate-700 text-slate-100 rounded hover:bg-slate-600"
                    on:click={clearPins}
                >Clear</button>
            {/if}
        </div>
    </div>

    {#if $pinned.length === 0}
        <p class="text-slate-500">No pinned curves yet.</p>
    {:else}
        <ul class="space-y-1">
            {#each $pinned as pin}
                <li class="flex items-center gap-2 font-mono text-xs">
                    <span class="inline-block w-3 h-3 rounded" style="background: {pin.color}"></span>
                    Kp={pin.params.kp.toFixed(1)} Ki={pin.params.ki.toFixed(1)} Kd={pin.params.kd.toFixed(1)}
                </li>
            {/each}
        </ul>
    {/if}
</section>
```

- [ ] **Step 2: Commit**

```powershell
git add web/src/components/PinList.svelte
git commit -m "feat(web): pin list with Pin/Clear actions"
```

---

### Task 6.7: Wire everything in `App.svelte`

**Files:**
- Modify: `web/src/App.svelte`

- [ ] **Step 1: Replace App.svelte with the assembly**

```svelte
<script>
    import ControlsBasic from './components/ControlsBasic.svelte';
    import ControlsAdvanced from './components/ControlsAdvanced.svelte';
    import Chart from './components/Chart.svelte';
    import StepInfo from './components/StepInfo.svelte';
    import PinList from './components/PinList.svelte';
    import { simError } from './lib/store.js';
    import { isWasmSupported } from './lib/wasm.js';
</script>

<main class="min-h-screen p-4 sm:p-6">
    <header class="mb-4">
        <h1 class="text-2xl font-bold">Drone PID Sandbox</h1>
        <p class="text-slate-400 text-sm">
            1-D PID controller running a C simulation in the browser via WebAssembly.
            Drag any slider to re-simulate. Pin curves to compare tunings.
        </p>
    </header>

    {#if !isWasmSupported()}
        <div class="bg-red-900/40 border border-red-700 text-red-200 p-4 rounded">
            This browser doesn't support WebAssembly. Try Chrome, Firefox, Safari 11+, or Edge.
        </div>
    {:else if $simError}
        <div class="bg-amber-900/40 border border-amber-700 text-amber-200 p-3 rounded mb-4">
            Simulation warning: {$simError}
        </div>
    {/if}

    <div class="grid gap-4 sm:grid-cols-[320px,1fr]">
        <aside class="space-y-4">
            <div class="bg-slate-900 rounded p-4">
                <ControlsBasic />
                <ControlsAdvanced />
            </div>
        </aside>

        <section class="space-y-4">
            <Chart />
            <div class="grid gap-4 sm:grid-cols-2">
                <StepInfo />
                <PinList />
            </div>
        </section>
    </div>

    <footer class="mt-8 text-xs text-slate-500">
        Source: <a class="underline" href="https://github.com/SyedHasanCronosPMC/Raza">github.com/SyedHasanCronosPMC/Raza</a>
    </footer>
</main>
```

- [ ] **Step 2: Run the dev server and verify in browser**

Run: `cd web && npm run dev`
Open: `http://localhost:5173/`
Expected:
- Header visible
- Controls in left column on desktop, stacked on mobile (resize window)
- Dragging Kp updates the chart in real time
- Clicking "Pin current" adds a ghosted curve
- Clicking "Clear" removes pinned curves
- StepInfo metrics update live

- [ ] **Step 3: Commit**

```powershell
git add web/src/App.svelte
git commit -m "feat(web): assemble full sandbox layout (responsive)"
git push
```

---

## Phase 7 — Polish, CI, deploy

### Task 7.1: Add GitHub Actions workflow

**Files:**
- Create: `.github/workflows/deploy.yml`

- [ ] **Step 1: Write the workflow**

```yaml
name: build-and-test

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  c-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Build CLI
        run: make
      - name: Run C tests
        run: make test
      - name: Smoke test CLI
        run: ./drone --smoke

  web-build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: mymindstorm/setup-emsdk@v14
        with:
          version: 3.1.55
      - name: Build WASM
        run: make -f Makefile.wasm
      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: 'npm'
          cache-dependency-path: web/package-lock.json
      - name: Install deps
        working-directory: web
        run: npm ci
      - name: Run tests
        working-directory: web
        run: npm test
      - name: Build SPA
        working-directory: web
        run: npm run build
      - name: Upload dist
        uses: actions/upload-artifact@v4
        with:
          name: web-dist
          path: web/dist
```

- [ ] **Step 2: Commit and push to trigger CI**

```powershell
git add .github/workflows/deploy.yml
git commit -m "ci: build CLI, run C tests, build WASM, build SPA"
git push
```

- [ ] **Step 3: Open the Actions tab on GitHub and verify both jobs are green**

URL: `https://github.com/SyedHasanCronosPMC/Raza/actions`
Expected: `c-tests` and `web-build` both succeed.

If either fails, fix before proceeding to Vercel setup.

---

### Task 7.2: Connect the GitHub repo to Vercel

This step is performed manually in the Vercel dashboard.

- [ ] **Step 1: Sign in at vercel.com (GitHub auth)**

- [ ] **Step 2: Click "Add New" → "Project" → import `SyedHasanCronosPMC/Raza`**

- [ ] **Step 3: Configure the project:**
    - **Framework preset:** `Vite`
    - **Root directory:** `web`
    - **Build command:** `npm run build`
    - **Output directory:** `dist`
    - **Install command:** `npm ci`

- [ ] **Step 4: Add a build hook or post-install script for WASM**

Vercel doesn't run Make / Emscripten by default. Two options:

**Option A (recommended):** Commit the built `sim.wasm` and `sim_glue.js` to `web/public/` and remove them from `.gitignore`. Build them locally with `make -f Makefile.wasm` and check them in. Trade-off: one extra commit per simulation change.

**Option B:** Add a `web/scripts/build-wasm.sh` that downloads emsdk and runs emcc as a Vercel pre-build step. More fragile, slower deploys.

Pick A. Update `.gitignore` to remove the WASM exclusions:

```diff
- # WASM build artifacts (added when frontend lands)
- web/public/sim.wasm
- web/public/sim_glue.js
+ # (WASM artifacts are committed for Vercel deploy convenience)
```

Run locally: `make -f Makefile.wasm`, then `git add web/public/sim.wasm web/public/sim_glue.js && git commit -m "build: commit WASM artifacts for Vercel deploy"`.

- [ ] **Step 5: Click "Deploy" in Vercel**

Expected: deployment succeeds within 60 s; preview URL appears (e.g., `raza-xyz.vercel.app`).

- [ ] **Step 6: Open the preview URL and verify the sandbox works end-to-end**

- [ ] **Step 7: Commit the production URL into the README**

Append to `README.md`:

```markdown
## Live demo

[Open the sandbox →](https://<your-vercel-url>)
```

```powershell
git add README.md
git commit -m "docs: link live Vercel demo URL"
git push
```

---

### Task 7.3: Run Lighthouse and confirm Performance ≥ 90

- [ ] **Step 1: Open the deployed URL in Chrome**

- [ ] **Step 2: DevTools → Lighthouse → Analyze page load (Desktop, Performance only)**

- [ ] **Step 3: Verify Performance score ≥ 90**

If below 90, common fixes:
- Ensure `sim.wasm` is gzip-compressed (Vercel does this automatically for `*.wasm`)
- Verify `npm run build` produced a minified bundle in `web/dist/assets/`
- Check that uPlot's CSS is included (otherwise FOUC penalty)

- [ ] **Step 4: Capture the score in a final commit message and call the project done**

```powershell
git commit --allow-empty -m "milestone: v1 deployed; Lighthouse Performance <score>"
git push
```

---

## Self-review

After writing this plan, I checked it against the spec:

1. **Spec coverage:**
    - Goals 1–6 ✓ (all mapped to phases 1–7)
    - 16-parameter inventory ✓ (ControlsBasic + ControlsAdvanced cover all 16)
    - SimParams / SimSample contract ✓ (Tasks 1.2, 4.2)
    - Pin behavior ✓ (Task 5.2 + 6.6)
    - Acceptance criteria 1–7 ✓ (regression test guards 1; Task 4.2 measures 2; Task 6.7 verifies 3 manually; Task 5.2 covers 4; Task 6.7 step 2 covers 5; Task 1.1+ covers 6; Task 7.3 covers 7)
2. **Placeholder scan:** none.
3. **Type consistency:** `SimParams` field names used in sim.h (Task 1.2) match the JS layout in `lib/wasm.js` (Task 4.2) and the store defaults in `store.js` (Task 5.2).
4. **No fabricated APIs:** every uPlot, Svelte, Vite, Tailwind, and Emscripten symbol referenced exists in those libraries' v4/v5 APIs.

Plan is ready for execution.
