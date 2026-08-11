/**
 * 이력 조회용 HTTP 클라이언트.
 *
 * 실시간 상태는 WebSocket 으로 오므로 여기서 다루지 않는다. 여기는 지나간
 * 것만 본다.
 *
 * 개발 중에는 Vite(5173)와 API(8000)의 출처가 달라서 절대 주소를 쓴다.
 * 빌드해서 한 서버에서 내보낼 때를 대비해 환경변수로 바꿀 수 있게 둔다.
 */

import type {
  EtaAccuracyStats,
  MapMeta,
  MissionEvent,
  MissionSummary,
  ResponseTimeStats,
} from '../types/telemetry';

/**
 * 별도 설정이 없으면 화면을 연 호스트의 8000번 포트에 붙는다.
 *
 * 127.0.0.1로 고정하면 관제 PC의 IP로 화면을 열었을 때 API 요청이 브라우저를
 * 실행한 장비 자신을 향한다. localhost로 열든 LAN IP로 열든 같은 중앙 PC를
 * 가리키도록 현재 페이지의 hostname을 사용한다.
 */
export const API_BASE = import.meta.env.VITE_API_BASE
  ?? `${window.location.protocol}//${window.location.hostname}:8000`;

export const WS_URL = API_BASE.replace(/^http/, 'ws') + '/ws/live';

/** 영상 타일이 그대로 <img src> 에 넣는 주소. */
export function videoUrl(streamId: string): string {
  return `${API_BASE}/api/video/${streamId}`;
}

async function getJson<T>(path: string): Promise<T> {
  const response = await fetch(`${API_BASE}${path}`);
  if (!response.ok) {
    throw new Error(`${path} 요청 실패: HTTP ${response.status}`);
  }
  return (await response.json()) as T;
}

export async function fetchMissions(limit = 50): Promise<MissionSummary[]> {
  const data = await getJson<{ items: MissionSummary[] }>(
    `/api/missions?limit=${limit}`,
  );
  return data.items;
}

export async function fetchMissionTimeline(
  missionId: string,
): Promise<MissionEvent[]> {
  const data = await getJson<{ timeline: MissionEvent[] }>(
    `/api/missions/${encodeURIComponent(missionId)}`,
  );
  return data.timeline;
}

export async function fetchResponseStats(): Promise<ResponseTimeStats> {
  return getJson<ResponseTimeStats>('/api/stats/response-time');
}

/** 운영자가 지도에서 찍은 자리를 보낸다. 관제에서 유일하게 나가는 통로다. */
export async function postOperatorReport(
  x: number,
  y: number,
): Promise<{ accepted: boolean; event_id: string; via: string }> {
  const response = await fetch(`${API_BASE}/api/report`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ x, y }),
  });
  if (!response.ok) {
    throw new Error(`신고 실패: HTTP ${response.status}`);
  }
  return response.json();
}

export async function fetchMapMeta(): Promise<MapMeta> {
  return getJson<MapMeta>('/api/map/meta');
}

export async function fetchEtaAccuracy(
  limit = 20,
): Promise<EtaAccuracyStats> {
  return getJson<EtaAccuracyStats>(
    `/api/stats/eta-accuracy?limit=${limit}`,
  );
}
