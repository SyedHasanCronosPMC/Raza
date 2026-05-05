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
