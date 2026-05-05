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
