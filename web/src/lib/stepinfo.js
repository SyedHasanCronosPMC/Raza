// web/src/lib/stepinfo.js
// Pure functions over SimSample[] returning step-response metrics.

export function computeStepInfo(samples, target) {
    if (!samples || samples.length === 0) {
        return { peak: 0, overshoot: 0, riseTime: null, settlingTime: null, steadyError: 0 };
    }
    const n = samples.length;
    const dt = n > 1 ? samples[1].time - samples[0].time : 0.1;

    let peak = 0;
    for (const s of samples) if (s.altitude > peak) peak = s.altitude;

    const overshoot = Math.max(0, peak - target);

    let riseStart = null, riseEnd = null;
    for (const s of samples) {
        if (riseStart === null && s.altitude >= 0.1 * target) riseStart = s.time;
        if (s.altitude >= 0.9 * target) { riseEnd = s.time; break; }
    }
    const riseTime = (riseStart !== null && riseEnd !== null) ? (riseEnd - riseStart) : null;

    const band = Math.max(0.02 * target, 0.05);
    let settlingTime = null;
    for (let i = n - 1; i >= 0; i--) {
        if (Math.abs(samples[i].altitude - target) > band) {
            if (i + 1 < n) settlingTime = samples[i + 1].time;
            break;
        }
    }

    const tail = Math.min(n, Math.max(1, Math.round(1 / dt)));
    let sum = 0;
    for (let i = n - tail; i < n; i++) sum += samples[i].altitude;
    const steadyError = Math.abs(target - sum / tail);

    return { peak, overshoot, riseTime, settlingTime, steadyError };
}
