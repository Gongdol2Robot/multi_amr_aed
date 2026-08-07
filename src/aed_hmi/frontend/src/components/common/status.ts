/**
 * 상태값을 화면 표현(색·라벨)으로 바꾼다.
 *
 * 이 매핑을 컴포넌트 안에 흩어 두면, 같은 상태가 화면마다 다른 색으로
 * 나오는 일이 생긴다. 한 곳에 모아 두고 전부 여기서 가져다 쓴다.
 *
 * 라벨은 한국어로 둔다. 운영자가 보는 화면이고, 영어 상태명을 그대로
 * 노출하면 급할 때 읽는 속도가 떨어진다.
 */

import type {
  EventStatus,
  MissionState,
  RobotAvailability,
  RobotRole,
} from '../../types/telemetry';

export type Tone = 'ok' | 'warn' | 'danger' | 'info' | 'idle';

interface Display {
  label: string;
  tone: Tone;
}

const MISSION: Record<MissionState, Display> = {
  assigned: { label: '배정됨', tone: 'info' },
  dispatching: { label: '출동 준비', tone: 'info' },
  en_route: { label: '이동 중', tone: 'info' },
  arrived: { label: 'AED 도착', tone: 'ok' },
  completed: { label: '완료', tone: 'ok' },
  canceled: { label: '취소', tone: 'idle' },
  blocked: { label: '경로 막힘', tone: 'danger' },
  network_lost: { label: '통신 두절', tone: 'danger' },
  navigation_error: { label: '주행 오류', tone: 'danger' },
  recovery_wait: { label: '복구 대기', tone: 'warn' },
  recovery_resumed: { label: '복구 후 재출동', tone: 'info' },
  helper_requested: { label: '인력 호출', tone: 'warn' },
  helper_en_route: { label: '인력 이동 중', tone: 'info' },
  helper_arrived: { label: '인력 도착', tone: 'ok' },
};

const AVAILABILITY: Record<RobotAvailability, Display> = {
  available: { label: '대기', tone: 'ok' },
  busy: { label: '임무 수행', tone: 'info' },
  blocked: { label: '경로 막힘', tone: 'danger' },
  network_lost: { label: '통신 두절', tone: 'danger' },
  navigation_error: { label: '주행 오류', tone: 'danger' },
  localization_error: { label: '위치추정 실패', tone: 'danger' },
  low_battery: { label: '배터리 부족', tone: 'warn' },
  emergency_stop: { label: '비상 정지', tone: 'danger' },
  unavailable: { label: '사용 불가', tone: 'idle' },
};

const ROLE: Record<RobotRole, string> = {
  none: '—',
  aed_delivery: 'AED 전달',
  helper_request: '인력 호출',
  guide: '현장 안내',
  return: '복귀',
};

const EVENT: Record<EventStatus, Display> = {
  detected: { label: '검출', tone: 'warn' },
  confirmed: { label: '확정', tone: 'danger' },
  dispatched: { label: '출동', tone: 'info' },
  resolved: { label: '종료', tone: 'ok' },
  canceled: { label: '취소', tone: 'idle' },
};

export const missionDisplay = (state: MissionState): Display =>
  MISSION[state] ?? { label: state, tone: 'idle' };

export const availabilityDisplay = (value: RobotAvailability): Display =>
  AVAILABILITY[value] ?? { label: value, tone: 'idle' };

export const roleLabel = (value: RobotRole): string => ROLE[value] ?? value;

export const eventDisplay = (value: EventStatus): Display =>
  EVENT[value] ?? { label: value, tone: 'idle' };

/** 배터리는 값 자체보다 "언제 걱정해야 하나"가 중요하다. */
export function batteryTone(percentage: number): Tone {
  if (percentage <= 20) return 'danger';
  if (percentage <= 40) return 'warn';
  return 'ok';
}

/** epoch 초 -> 화면용 시:분:초. 서버는 UTC 로 주고 표시만 로컬로 한다. */
export function clockText(epochSeconds: number | null): string {
  if (!epochSeconds) return '--:--:--';
  return new Date(epochSeconds * 1000).toLocaleTimeString('ko-KR', {
    hour12: false,
  });
}

/** 경과 시간을 사람이 읽는 형태로. 응답 시간 표시에 쓴다. */
export function durationText(seconds: number | null): string {
  if (seconds === null || Number.isNaN(seconds)) return '—';
  if (seconds < 60) return `${seconds.toFixed(1)}초`;
  const minutes = Math.floor(seconds / 60);
  return `${minutes}분 ${Math.round(seconds - minutes * 60)}초`;
}
