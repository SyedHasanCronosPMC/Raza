// web/src/lib/wasm.js
// Loads sim.wasm and exposes simulate(params) returning an array of SimSample objects.
//
// Struct layout — must match the C compiler's packing of sim.h SimParams.
// Adjacent int32 fields share an 8-byte slot (deriv_on_meas + integrator).
// has_wind sits in slot 6 with 4 bytes of trailing pad before wind_strength.
// seed sits in slot 15 with 4 bytes of trailing pad to round struct to 128B.

const PARAM_LAYOUT = [
    { name: 'kp',             type: 'f64', offset: 0   },
    { name: 'ki',             type: 'f64', offset: 8   },
    { name: 'kd',             type: 'f64', offset: 16  },
    { name: 'target_alt',     type: 'f64', offset: 24  },
    { name: 'mass',           type: 'f64', offset: 32  },
    { name: 'gravity',        type: 'f64', offset: 40  },
    { name: 'has_wind',       type: 'i32', offset: 48  },
    { name: 'wind_strength',  type: 'f64', offset: 56  },
    { name: 'deriv_on_meas',  type: 'i32', offset: 64  },
    { name: 'integrator',     type: 'i32', offset: 68  },
    { name: 'motor_tau',      type: 'f64', offset: 72  },
    { name: 'sensor_sigma',   type: 'f64', offset: 80  },
    { name: 'integral_max',   type: 'f64', offset: 88  },
    { name: 'thrust_max',     type: 'f64', offset: 96  },
    { name: 'duration',       type: 'f64', offset: 104 },
    { name: 'dt',             type: 'f64', offset: 112 },
    { name: 'seed',           type: 'i32', offset: 120 }
];
const PARAM_BYTES  = 128;
const SAMPLE_BYTES = 7 * 8;
const MAX_SAMPLES  = 4096;

let modulePromise = null;
function loadModule() {
    if (!modulePromise) {
        modulePromise = import('/sim_glue.js').then(({ default: factory }) => factory());
    }
    return modulePromise;
}

export async function simulate(params) {
    const Module = await loadModule();
    const paramsPtr  = Module._malloc(PARAM_BYTES);
    const samplesPtr = Module._malloc(MAX_SAMPLES * SAMPLE_BYTES);
    try {
        new Uint8Array(Module.HEAPU8.buffer, paramsPtr, PARAM_BYTES).fill(0);
        const dv = new DataView(Module.HEAPU8.buffer, paramsPtr, PARAM_BYTES);
        for (const f of PARAM_LAYOUT) {
            if (f.type === 'f64') {
                dv.setFloat64(f.offset, Number(params[f.name]), true);
            } else {
                dv.setInt32(f.offset, params[f.name] | 0, true);
            }
        }
        const n = Module._simulate(paramsPtr, samplesPtr, MAX_SAMPLES);
        const out = new Array(n);
        const view = new Float64Array(Module.HEAPF64.buffer, samplesPtr, n * 7);
        for (let i = 0; i < n; i++) {
            const o = i * 7;
            out[i] = {
                time:     view[o],
                altitude: view[o + 1],
                error:    view[o + 2],
                thrust:   view[o + 3],
                p_term:   view[o + 4],
                i_term:   view[o + 5],
                d_term:   view[o + 6]
            };
        }
        return out;
    } finally {
        Module._free(paramsPtr);
        Module._free(samplesPtr);
    }
}

export function isWasmSupported() {
    return typeof WebAssembly === 'object'
        && typeof WebAssembly.instantiate === 'function';
}
