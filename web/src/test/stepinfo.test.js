import { describe, it, expect } from 'vitest';
import { computeStepInfo } from '../lib/stepinfo.js';

function makeRamp(target, n, dt) {
    return Array.from({ length: n }, (_, i) => ({
        time: i * dt,
        altitude: Math.min(target, target * (i / 30)),
        error: 0, thrust: 0, p_term: 0, i_term: 0, d_term: 0
    }));
}

describe('computeStepInfo', () => {
    it('returns peak equal to max altitude', () => {
        const samples = makeRamp(10, 100, 0.1);
        const info = computeStepInfo(samples, 10);
        expect(info.peak).toBeCloseTo(10, 5);
    });

    it('zero overshoot when monotonically rising to target', () => {
        const samples = makeRamp(10, 100, 0.1);
        const info = computeStepInfo(samples, 10);
        expect(info.overshoot).toBe(0);
    });

    it('reports null rise time when target never reached', () => {
        const samples = makeRamp(5, 100, 0.1);
        const info = computeStepInfo(samples, 10);
        expect(info.riseTime).toBeNull();
    });

    it('averages last 1s for steady-state error', () => {
        const samples = Array.from({ length: 100 }, (_, i) => ({
            time: i * 0.1,
            altitude: i < 90 ? 5 : 10,
            error: 0, thrust: 0, p_term: 0, i_term: 0, d_term: 0
        }));
        const info = computeStepInfo(samples, 10);
        expect(info.steadyError).toBeCloseTo(0, 5);
    });
});
