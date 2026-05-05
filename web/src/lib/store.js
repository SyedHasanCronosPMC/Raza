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
export const currentRun  = writable(null);
export const pinned      = writable([]);
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
