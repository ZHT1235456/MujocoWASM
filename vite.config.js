import { defineConfig } from 'vite';
import path from 'node:path';

export default defineConfig({
  assetsInclude: ['**/*.wasm'],
  optimizeDeps: {
    exclude: ['@mujoco/mujoco'],
  },
  resolve: {
    alias: {
      '@drone': path.resolve(__dirname, 'reference/drone'),
    },
  },
  server: {
    port: 5173,
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
  preview: {
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
});
