# Drone PID Simulator

A 1-D vertical drone flight simulator with an interactive PID tuner, ASCII step-response graph, and CSV log export. Written in C as a course assignment.

**Author:** Syed Naqvi (250210060)
**Date:** 02/05/2026

## Features

- Three flight scenarios: standard hover, wind disturbance, and a heavy 5 kg drone
- Arrow-key live tuner for `Kp`, `Ki`, `Kd`, and target altitude
- ASCII step-response graph rendered after each run
- Step-info report: peak altitude, overshoot, rise time (10% → 90%), settling time (±2% band), steady-state error
- Anti-windup integral clamp and motor-saturation thrust ceiling
- CSV log export (`flight_log.csv`) for analysis in Excel, pandas, or MATLAB

## Build

Requires `gcc` with C99 support (MinGW-w64 on Windows, or any modern Linux/macOS toolchain).

```sh
make
```

Or directly:

```sh
gcc openEndedC.c -lm -Wall -Wextra -O2 -o drone.exe
```

> **Platform note:** The interactive tuner uses `<conio.h>` and Windows-style arrow-key codes, so the program is **Windows-only** as written. ANSI colors require a VT100-capable terminal (Windows Terminal or Windows 10+ console).

## Run

```sh
make run
```

Or:

```sh
./drone.exe
```

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

## Controls

| Screen | Keys |
|---|---|
| Main menu | `1`–`6` then Enter |
| PID tuner | ↑/↓ to select slider, ←/→ to adjust, Enter on **[ Run Simulation ]** to launch |
| Tutorial / math / post-run | Enter to return |

## CSV log format

After every simulation run the program writes `flight_log.csv` to the working directory:

```
# Level 1: Standard Hover (Mass: 1kg, No Wind)
# Kp=2.00 Ki=0.50 Kd=1.00 Target=10.00m Mass=1.00kg
time_s,altitude_m,error_m,thrust_N
0.00,0.0000,10.0000,20.0000
0.10,0.1019,9.8981,...
```

Quick plot in Python:

```python
import pandas as pd, matplotlib.pyplot as plt
df = pd.read_csv("flight_log.csv", comment="#")
df.plot(x="time_s", y=["altitude_m", "thrust_N"], subplots=True)
plt.show()
```

## Math reference

```
error      = target_altitude - current_altitude
integral  += error * dt           (clamped to +/- 50)
derivative = (error - prev_error) / dt
thrust     = Kp*error + Ki*integral + Kd*derivative   (clamped to [0, 150])
net_force  = thrust - mass*gravity + wind
accel      = net_force / mass
velocity  += accel * dt
altitude  += velocity * dt
```

Wind (Level 2 only) is a uniform random force in [−4, +4] N, re-sampled every 2 s and held between updates.

## Project layout

```
.
├── openEndedC.c     # single-file source
├── Makefile         # build / run / clean targets
├── README.md        # this file
└── flight_log.csv   # generated after the first run
```

## Academic note

Gen AI was used to assist with brainstorming and formatting the ASCII graph; the final implementation and structure are the author's own work.
