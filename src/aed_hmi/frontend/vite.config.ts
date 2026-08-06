import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// 개발 서버는 5173, 백엔드는 8000 이다. API_BASE 를 절대 주소로 두므로
// 프록시는 쓰지 않는다. MJPEG 는 프록시를 거치면 버퍼링이 끼어들어
// 프레임이 뭉쳐 나오는 일이 있어, 직접 붙이는 편이 안전하다.
export default defineConfig({
  plugins: [react()],
  server: { port: 5173, host: true },
  build: { outDir: 'dist', sourcemap: true },
});
