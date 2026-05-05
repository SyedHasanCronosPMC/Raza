<script>
    import { onMount, onDestroy } from 'svelte';
    import uPlot from 'uplot';
    import 'uplot/dist/uPlot.min.css';
    import { currentRun, pinned, params } from '../lib/store.js';

    let container;
    let plot = null;
    let unsubRun, unsubPinned, unsubParams;

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
