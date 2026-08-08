/**
 * 도착 예상 표시.
 *
 * 관제실에서 이 숫자는 무전으로 그대로 읽히는 값이다("2분 뒤 도착").
 * 중앙제어가 로봇 선정과 재배정에 사용하는 ETA를 그대로 표시한다. HMI가
 * 별도 계산식을 갖지 않아 화면과 실제 배정 판단이 어긋나지 않는다.
 *
 * 남은 시간은 서버가 보낸 도착 예상 시각(eta_at)에서 계산한다. 서버가 준
 * eta_seconds 를 그대로 쓰면 다음 갱신까지 값이 멈춰 있어서, 사람이 보기에
 * 시간이 흐르지 않는 것처럼 느껴진다.
 */

import type { MissionSummary } from '../../types/telemetry';
import { durationText } from '../common/status';

function remainingSeconds(mission: MissionSummary, now: number): number | null {
  if (mission.eta_at === null) return mission.eta_seconds;
  return Math.max(mission.eta_at - now / 1000, 0);
}

function initialRemainingSeconds(
  mission: MissionSummary,
  now: number,
): number | null {
  if (mission.initial_eta_at === null) return mission.initial_eta_seconds;
  return Math.max(mission.initial_eta_at - now / 1000, 0);
}

function projectedDelaySeconds(mission: MissionSummary): number | null {
  if (mission.eta_at === null || mission.initial_eta_at === null) return null;
  return mission.eta_at - mission.initial_eta_at;
}

function signedDuration(seconds: number): string {
  const rounded = Math.round(seconds);
  if (rounded === 0) return '±0초';
  return `${rounded > 0 ? '+' : '-'}${durationText(Math.abs(rounded))}`;
}

export function EtaBadge({
  mission,
  now,
}: {
  mission: MissionSummary;
  now: number;
}) {
  const remaining = remainingSeconds(mission, now);
  if (remaining === null) return null;
  const delayed = (projectedDelaySeconds(mission) ?? 0) > 1;

  return (
    <span
      className={`eta ${delayed ? 'eta--delayed' : 'eta--sure'}`}
      title="중앙제어의 Nav2 경로·회전·혼잡도 ETA"
    >
      <span className="eta__label">도착까지</span>
      <span className="eta__value mono">{durationText(remaining)}</span>
    </span>
  );
}

/** 경보 배너 오른쪽에 크게 붙는 판. 운영자가 가장 자주 보는 숫자다. */
export function EtaPanel({
  missions,
  now,
}: {
  missions: MissionSummary[];
  now: number;
}) {
  // 여러 임무가 동시에 돌 수 있다. 가장 빨리 도착하는 것을 대표로 보여준다.
  const candidates = missions
    .map((mission) => ({
      mission,
      remaining: remainingSeconds(mission, now)
        ?? initialRemainingSeconds(mission, now),
    }))
    .filter((item) => item.remaining !== null)
    .sort((a, b) => (a.remaining ?? 0) - (b.remaining ?? 0));

  if (candidates.length === 0) {
    return (
      <div className="etapanel etapanel--none">
        <div className="alert__label">도착 예상</div>
        <div className="etapanel__none-text">이동 중 아님</div>
      </div>
    );
  }

  const { mission, remaining } = candidates[0];
  const delay = projectedDelaySeconds(mission);
  return (
    <div className="etapanel">
      <div className="alert__label">현재 남은 시간</div>
      <div
        className={`etapanel__value mono ${
          delay !== null && delay > 1 ? 'etapanel__value--delayed' : ''
        }`}
      >
        {durationText(remaining)}
      </div>
      <div className="etapanel__meta">
        <b>{mission.robot_id}</b>
        <span> · 중앙 ETA</span>
      </div>
      <div className="etapanel__comparison">
        <span>최초 예상</span>
        <b className="mono">
          {mission.initial_eta_seconds === null
            ? '—'
            : durationText(mission.initial_eta_seconds)}
        </b>
        <span>예상 지연</span>
        <b
          className={`mono ${
            delay !== null && delay > 1 ? 'etapanel__delay--late' : ''
          }`}
        >
          {delay === null ? '—' : signedDuration(delay)}
        </b>
      </div>
    </div>
  );
}
