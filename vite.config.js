import { defineConfig } from 'vite';

export default defineConfig({
  base: '/ros2-turtle-school/',
  build: {
    outDir: 'dist',
    sourcemap: true,
    rollupOptions: {
      output: {
        manualChunks: {
          // Split large vendor libraries into separate chunks
          'vendor-xterm': ['xterm', '@xterm/addon-fit'],
          'vendor-cytoscape': ['cytoscape']
        }
      }
    }
  },
  server: {
    port: 3000,
    open: true
  }
});
