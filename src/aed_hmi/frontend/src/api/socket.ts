/**
 * 실시간 상태 통로. WebSocket 하나로 화면 전체 상태를 받는다.
 *
 * 관제 화면은 사람이 자리를 비운 사이에도 계속 붙어 있어야 하므로,
 * 끊기면 스스로 다시 붙는다. 재접속 간격은 점점 늘린다. 서버가 죽어 있는데
 * 초당 여러 번 두드리면 서버가 살아날 때 접속 폭주가 생긴다.
 */

import type { SystemSnapshot } from '../types/telemetry';

export type ConnectionState = 'connecting' | 'open' | 'closed';

interface Handlers {
  onSnapshot: (snapshot: SystemSnapshot) => void;
  onStateChange: (state: ConnectionState) => void;
}

const RECONNECT_MIN_MS = 500;
const RECONNECT_MAX_MS = 8000;
/** 이 시간 동안 아무 메시지도 없으면 죽은 연결로 보고 새로 붙는다. */
const SILENCE_TIMEOUT_MS = 6000;

export class LiveSocket {
  private socket: WebSocket | null = null;
  private reconnectDelay = RECONNECT_MIN_MS;
  private silenceTimer: number | null = null;
  private closedByUser = false;

  constructor(private url: string, private handlers: Handlers) {}

  connect(): void {
    this.closedByUser = false;
    this.handlers.onStateChange('connecting');
    const socket = new WebSocket(this.url);
    this.socket = socket;

    socket.onopen = () => {
      this.reconnectDelay = RECONNECT_MIN_MS;
      this.handlers.onStateChange('open');
      this.armSilenceTimer();
    };

    socket.onmessage = (event) => {
      this.armSilenceTimer();
      try {
        const payload = JSON.parse(event.data as string);
        if (payload.type === 'snapshot') {
          this.handlers.onSnapshot(payload.data as SystemSnapshot);
        }
      } catch {
        // 깨진 프레임 하나 때문에 화면 전체를 멈추지 않는다.
      }
    };

    socket.onclose = () => {
      this.clearSilenceTimer();
      this.handlers.onStateChange('closed');
      if (!this.closedByUser) this.scheduleReconnect();
    };

    socket.onerror = () => socket.close();
  }

  disconnect(): void {
    this.closedByUser = true;
    this.clearSilenceTimer();
    this.socket?.close();
    this.socket = null;
  }

  private scheduleReconnect(): void {
    window.setTimeout(() => this.connect(), this.reconnectDelay);
    this.reconnectDelay = Math.min(this.reconnectDelay * 2, RECONNECT_MAX_MS);
  }

  private armSilenceTimer(): void {
    this.clearSilenceTimer();
    this.silenceTimer = window.setTimeout(() => {
      // 서버가 살아 있어도 중간 장비가 연결을 붙잡고 있을 수 있다.
      // 그 경우 onclose 가 안 오므로 직접 끊는다.
      this.socket?.close();
    }, SILENCE_TIMEOUT_MS);
  }

  private clearSilenceTimer(): void {
    if (this.silenceTimer !== null) {
      window.clearTimeout(this.silenceTimer);
      this.silenceTimer = null;
    }
  }
}
