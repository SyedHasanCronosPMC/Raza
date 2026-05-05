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
