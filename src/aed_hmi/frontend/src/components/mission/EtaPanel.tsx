/**
 * 도착 예상 표시.
 *
 * 관제실에서 이 숫자는 무전으로 그대로 읽히는 값이다("2분 뒤 도착").
 * 그래서 추정이라는 사실을 감추면 안 된다. 거리와 속도가 모두 실측일 때와
 * 가정값을 섞었을 때를 다르게 표시한다.
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

export function EtaBadge({
  mission,
  now,
}: {
  mission: MissionSummary;
  now: number;
}) {
  const remaining = remainingSeconds(mission, now);
  if (remaining === null) return null;

  return (
    <span
      className={`eta ${mission.eta_confident ? 'eta--sure' : 'eta--rough'}`}
      title={
        mission.eta_confident
          ? 'Nav2 경로 길이와 실측 속도로 계산'
          : '경로 또는 속도가 아직 없어 가정값을 섞음'
      }
    >
      <span className="eta__label">도착까지</span>
      <span className="eta__value mono">{durationText(remaining)}</span>
      {!mission.eta_confident && <span className="eta__rough">추정</span>}
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
    .map((mission) => ({ mission, remaining: remainingSeconds(mission, now) }))
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
  return (
    <div className="etapanel">
      <div className="alert__label">도착 예상</div>
      <div
        className={`etapanel__value mono ${
          mission.eta_confident ? '' : 'etapanel__value--rough'
        }`}
      >
        {durationText(remaining)}
      </div>
      <div className="etapanel__meta">
        <b>{mission.robot_id}</b>
        {mission.eta_distance_m !== null && (
          <span className="mono"> · {mission.eta_distance_m.toFixed(1)}m</span>
        )}
        {!mission.eta_confident && <span className="eta__rough">추정</span>}
      </div>
    </div>
  );
}
