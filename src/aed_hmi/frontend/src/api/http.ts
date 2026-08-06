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
  MissionEvent,
  MissionSummary,
  ResponseTimeStats,
} from '../types/telemetry';

export const API_BASE =
  import.meta.env.VITE_API_BASE ?? 'http://127.0.0.1:8000';

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
