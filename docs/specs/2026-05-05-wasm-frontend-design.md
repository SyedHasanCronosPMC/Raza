# Spec: WASM PID Sandbox

**Date:** 2026-05-05
**Authors:** Syed Naqvi, Claude Opus 4.7
**Status:** Approved (brainstorming complete; ready for implementation plan)
**Repo:** [github.com/SyedHasanCronosPMC/Raza](https://github.com/SyedHasanCronosPMC/Raza)

## Summary

A live-interactive 1-D drone PID simulator running in the browser via WebAssembly, deployed to Vercel. The existing `runSimulation` C function becomes the single source of truth for the physics; a Svelte + uPlot frontend exposes 16 parameters as a power-user "sandbox" with progressive disclosure, live re-simulation on every input change, and a Pin feature for ghost-overlay comparisons.

## Audience

Public-facing PID learning / lab tool. Target users are control-systems learners, hobbyist drone builders, and engineering students who land on the page from search engines. The design prioritizes immediate engagement ("drag Kp, watch oscillations grow") over hand-holding tutorials.

## Goals

1. Compile the existing `runSimulation()` to WebAssembly via Emscripten — the C remains the single source of truth for the physics.
2. Live re-run on slider drag (debounced ~16 ms) for 16 parameters.
3. Render with uPlot; **Pin** button captures the current curve as a ghost overlay (max 5, oldest evicts).
4. Progressive disclosure: 6 basic controls always visible, ▸ Advanced accordion holds the other 10.
5. Step-info panel updates live alongside the chart: peak, overshoot, rise time, settling time, steady-state error.
6. Static SPA on Vercel, auto-deployed from GitHub `main`.

## Non-goals (v1)

- Multi-axis (2-D / 3-D) flight, lateral wind
- URL-state sharing of a tuning
- User accounts, persistence, leaderboards
- Server-side functionality
- Native iOS / Android apps

## Architecture

```
Browser
 ├─ index.html + app.js     (~50 kB Svelte + uPlot + Tailwind)
 ├─ sim.wasm                (~10 kB, compiled from sim.c via Emscripten)
 └─ sim_glue.js             (~5 kB, Emscripten output — JS ↔ WASM bridge)
```

Three artifacts ship; Vercel serves all three statically. No server, no API, no database.

## C-side rework

### File split

```
sim.h          interface — public sim functions + data structs
sim.c          pure simulation (no I/O, no UI). Compiles for CLI and WASM.
openEndedC.c   thin CLI wrapper — main, menu, _getch tuner, ASCII chart.
               #include "sim.h", calls into sim.c.
```

The simulation function moves to `sim.c` along with `FlightData` and physics constants; `openEndedC.c` keeps `main`, the menu loop, the interactive `_getch` tuner, the ASCII chart renderer, and the CSV writer.

### Public API (`sim.h`)

```c
typedef struct {
    double kp, ki, kd, target_alt;
    double mass, gravity;
    int    has_wind;
    double wind_strength;
    int    deriv_on_meas;     // 0 = error, 1 = measurement
    int    integrator;        // 0 = Euler, 1 = RK4
    double motor_tau;         // 0 = instant, >0 = first-order lag
    double sensor_sigma;      // gaussian noise stddev on altitude
    double integral_max;      // anti-windup
    double thrust_max;        // saturation
    double duration;          // seconds
    double dt;                // seconds
    int    seed;              // RNG seed for reproducibility
} SimParams;

typedef struct {
    double time, altitude, error, thrust;
    double p_term, i_term, d_term;
} SimSample;

// JS allocates out_samples of length max_samples; C fills it; returns count.
int simulate(const SimParams* params, SimSample* out_samples, int max_samples);
```

Single struct in / array out is the cleanest Emscripten interop pattern.

### New simulation features (driven by parameter inventory)

- **Integrator:** existing forward-Euler stays as `integrator=0`. RK4 added as `integrator=1`.
- **Derivative on measurement:** when `deriv_on_meas=1`, `derivative = -(altitude - prev_altitude) / dt`.
- **Motor lag:** when `motor_tau > 0`, applied thrust = first-order filter over commanded thrust with τ.
- **Sensor noise:** when `sensor_sigma > 0`, control loop reads `altitude + N(0, σ)`; the truth altitude is still recorded in the sample log.
- **Per-step PID breakdown:** `p_term`, `i_term`, `d_term` recorded per sample for future visualization.
- **Seedable RNG:** both wind and sensor noise use a seeded Mersenne-Twister-equivalent (or `srand(seed)`) so identical params reproduce identical curves. The CLI is updated to expose `--seed` (default `time(NULL)` to preserve current behavior).

## Parameter inventory

**Tier 1 — always visible (6 controls):**

| Param | Range | Default |
|---|---|---|
| Kp | 0–20 | 2.0 |
| Ki | 0–10 | 0.5 |
| Kd | 0–20 | 1.0 |
| Target altitude | 1–30 m | 10 |
| Mass | 0.5–10 kg | 1.0 |
| Wind | on/off + strength 0–10 N | off, 4 |

**Tier 2 — `▸ Advanced` accordion (10 controls):**

| Param | Range | Default |
|---|---|---|
| Gravity | 1–25 m/s² | 9.81 |
| Integrator | Euler / RK4 | Euler |
| Derivative on | error / measurement | error |
| Motor lag τ | 0–0.2 s | 0 |
| Sensor noise σ | 0–0.5 m | 0 |
| Anti-windup clamp | 5–200 | 50 |
| Thrust ceiling | 20–500 N | 150 |
| Duration | 2–30 s | 10 |
| dt | 0.001–0.1 s | 0.1 |
| Seed | integer | 42 |

## Repo layout

```
/                          # repo root
├── sim.h, sim.c           # pure simulation (CLI + WASM)
├── openEndedC.c           # CLI wrapper
├── Makefile               # builds CLI binary
├── Makefile.wasm          # builds web/public/sim.wasm via emcc
├── tests/sim_test.c       # C unit tests
├── README.md
├── docs/specs/            # design docs
├── .github/workflows/
│   └── deploy.yml
└── web/
    ├── index.html
    ├── package.json
    ├── vite.config.js
    ├── tailwind.config.js
    ├── public/
    │   ├── sim.wasm          (gitignored — produced by Makefile.wasm)
    │   └── sim_glue.js       (gitignored)
    ├── src/
    │   ├── main.js
    │   ├── App.svelte
    │   ├── lib/
    │   │   ├── wasm.js          # loads sim.wasm, exposes simulate(params)
    │   │   ├── store.js         # params store + pinned curves
    │   │   └── stepinfo.js      # peak / overshoot / rise / settling
    │   └── components/
    │       ├── ControlsBasic.svelte
    │       ├── ControlsAdvanced.svelte
    │       ├── Chart.svelte         # uPlot wrapper
    │       ├── StepInfo.svelte
    │       ├── PinList.svelte
    │       └── HelpTooltip.svelte
    └── tests/                       # vitest
```

## Frontend module responsibilities

| Module | Responsibility | Key dependencies |
|---|---|---|
| `lib/wasm.js` | Load sim.wasm + sim_glue.js; expose `simulate(params): SimSample[]` | sim_glue.js |
| `lib/store.js` | Reactive params store; pinned-curves array (max 5); current run; debounced re-sim trigger | svelte/store, lib/wasm |
| `lib/stepinfo.js` | Pure functions: peak, overshoot, rise time, settling time, steady error | none |
| `App.svelte` | Top-level layout, viewport breakpoint logic | all components |
| `ControlsBasic.svelte` | Tier 1 sliders/toggles | store |
| `ControlsAdvanced.svelte` | Tier 2 in collapsible accordion | store |
| `Chart.svelte` | uPlot wrapper; plots current run + pinned curves | uPlot, store |
| `StepInfo.svelte` | Renders metrics from `lib/stepinfo` | store, lib/stepinfo |
| `PinList.svelte` | Shows pinned tunings with delete buttons; **Clear all** action | store |
| `HelpTooltip.svelte` | Inline `?` icons with explanations of each PID term | none |

## Data flow

```
slider drag
  → store.params.set(...)
  → debounce 16 ms
  → wasm.simulate(params) → SimSample[]
  → store.currentRun = result
  → <Chart/> reactively pushes into uPlot
  → <StepInfo/> recomputes metrics
```

**Pin button:** `store.pinned.push({ params, samples, color })`, capped at 5, FIFO eviction. **Clear pins** wipes the array. Each pinned curve is rendered as a faded ghost line in a distinct color.

## Layout (Tailwind)

- **< 640 px:** single column, controls above chart, Advanced accordion collapsed by default.
- **≥ 640 px:** two columns — controls left (~320 px fixed), chart + StepInfo right (fluid).

## Build & deploy pipeline

| Stage | Command |
|---|---|
| Build CLI binary | `make` |
| Build WASM | `make -f Makefile.wasm` (requires emsdk on PATH) |
| Local dev | `cd web && npm run dev` (Vite HMR) |
| Production build | `cd web && npm run build` → `web/dist/` |
| Deploy | Vercel auto-detects Vite project; CI on push to `main` |

GitHub Actions (`.github/workflows/deploy.yml`):
1. Checkout
2. `mymindstorm/setup-emsdk@v14`
3. `make -f Makefile.wasm`
4. `cd web && npm ci && npm run build`
5. `make test` (C unit tests) and `cd web && npm test` (Vitest)
6. Vercel GitHub integration deploys from `main` automatically.

## Error handling

- **WASM load fails:** banner *"Failed to load simulation engine — refresh"* with retry button.
- **No WebAssembly support:** static fallback message with browser-upgrade hint.
- **Pathological params (NaN, ∞):** C clamps internally; UI shows a yellow warning banner and keeps the last good chart.
- No network calls, no auth, no server errors to handle.

## Testing strategy

| Layer | Tool | What it verifies |
|---|---|---|
| C unit | `tests/sim_test.c` + Make target | Fixed-seed determinism: known `SimParams` → expected final altitude (±0.1 m); rise-time and overshoot regression checks for three known-good configs |
| JS unit | Vitest | stepinfo math, store reactivity, pin eviction, debounce timing |
| E2E | manual on Vercel preview URL | Slider → chart change; Pin/Clear behavior; mobile layout |

E2E automation (Playwright) deferred to v2.

## Acceptance criteria

1. CLI binary continues to build and run identically to today (no regression in `openEndedC.c` behavior).
2. WASM module loads in < 500 ms on a typical broadband connection.
3. Dragging any slider updates the chart within 32 ms (≥ 30 fps perceived).
4. Pin captures the current curve as a faded ghost; up to 5 pins; oldest evicts FIFO.
5. Mobile layout (< 640 px viewport) shows controls stacked above the chart with no horizontal scroll.
6. C unit tests pass for at least three known-good configs.
7. Lighthouse Performance ≥ 90 on the deployed Vercel URL.

## Deferred to v2

- URL state sync (deep-linkable tunings): ~30 lines, low priority.
- A/B compare mode separate from Pin: probably unnecessary if Pin works well.
- Educational walkthrough overlay: rejected in scope decision (audience is power-user lab); worth revisiting if usage data shows confusion.
- Playwright E2E automation.
