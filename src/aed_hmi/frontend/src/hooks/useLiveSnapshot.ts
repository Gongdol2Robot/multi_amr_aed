/**
 * WebSocket 으로 오는 상태를 리액트 상태로 옮긴다.
 *
 * 화면 조각들이 각자 소켓을 열지 않도록 여기 한 곳에서만 연결한다.
 * 4분할 영상까지 붙는 화면이라 연결이 늘어나면 브라우저가 먼저 지친다.
 */

import { useEffect, useRef, useState } from 'react';

import { LiveSocket, type ConnectionState } from '../api/socket';
import { WS_URL } from '../api/http';
import type { SystemSnapshot } from '../types/telemetry';

export interface LiveSnapshot {
  snapshot: SystemSnapshot | null;
  connection: ConnectionState;
  /** 마지막으로 상태를 받은 시각(ms). 화면 상단의 지연 표시에 쓴다. */
  receivedAt: number | null;
}

export function useLiveSnapshot(): LiveSnapshot {
  const [snapshot, setSnapshot] = useState<SystemSnapshot | null>(null);
  const [connection, setConnection] = useState<ConnectionState>('connecting');
  const [receivedAt, setReceivedAt] = useState<number | null>(null);
  const socketRef = useRef<LiveSocket | null>(null);

  useEffect(() => {
    const socket = new LiveSocket(WS_URL, {
      onSnapshot: (next) => {
        setSnapshot(next);
        setReceivedAt(Date.now());
      },
      onStateChange: setConnection,
    });
    socketRef.current = socket;
    socket.connect();
    return () => socket.disconnect();
  }, []);

  return { snapshot, connection, receivedAt };
}
