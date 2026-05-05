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
