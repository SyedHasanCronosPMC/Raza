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
