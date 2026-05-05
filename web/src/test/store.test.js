import { describe, it, expect, vi, beforeEach } from 'vitest';
import { get } from 'svelte/store';
import { params, currentRun, pinned, pinCurrent, clearPins, defaultParams } from '../lib/store.js';

vi.mock('../lib/wasm.js', () => ({
    simulate: vi.fn(async (p) => Array.from({ length: 10 }, (_, i) => ({
        time: i * 0.1, altitude: i, error: 0, thrust: 0,
        p_term: 0, i_term: 0, d_term: 0
    })))
}));

describe('store', () => {
    beforeEach(() => {
        clearPins();
        params.set({ ...defaultParams });
    });

    it('exposes default params', () => {
        const p = get(params);
        expect(p.kp).toBe(2.0);
        expect(p.target_alt).toBe(10);
    });

    it('updates currentRun when params change (after debounce)', async () => {
        params.update(p => ({ ...p, kp: 5 }));
        await new Promise(r => setTimeout(r, 30));
        const run = get(currentRun);
        expect(run).not.toBeNull();
        expect(run.samples.length).toBe(10);
    });

    it('pinCurrent appends to pinned with a color', async () => {
        params.update(p => ({ ...p, kp: 5 }));
        await new Promise(r => setTimeout(r, 30));
        pinCurrent();
        const arr = get(pinned);
        expect(arr.length).toBe(1);
        expect(arr[0].color).toMatch(/^#/);
    });

    it('caps pinned at 5 with FIFO eviction', async () => {
        for (let i = 0; i < 6; i++) {
            params.update(p => ({ ...p, kp: i + 1 }));
            await new Promise(r => setTimeout(r, 30));
            pinCurrent();
        }
        const arr = get(pinned);
        expect(arr.length).toBe(5);
        expect(arr[0].params.kp).toBe(2);
    });
});
