// tests/wasm_smoke.mjs
// Loads sim.wasm via the Emscripten glue and confirms _simulate is callable.
// Run: node tests/wasm_smoke.mjs

import createSimModule from '../web/public/sim_glue.js';

const Module = await createSimModule();

if (typeof Module._simulate !== 'function') {
    console.error('FAIL: _simulate is not exported');
    process.exit(1);
}
if (typeof Module._malloc !== 'function' || typeof Module._free !== 'function') {
    console.error('FAIL: _malloc/_free not exported');
    process.exit(1);
}
console.log('PASS: WASM module loaded; _simulate, _malloc, _free are exported');
