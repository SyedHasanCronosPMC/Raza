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
