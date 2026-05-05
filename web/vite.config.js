import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig({
    plugins: [svelte()],
    server: { port: 5173, open: false },
    build: { target: 'es2022', sourcemap: true },
    test: {
        environment: 'jsdom',
        include: ['src/test/**/*.test.js']
    }
});
