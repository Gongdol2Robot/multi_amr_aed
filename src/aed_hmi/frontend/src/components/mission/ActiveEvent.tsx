/**
 * 진행 중인 응급 상황 배너. 화면에서 가장 눈에 띄어야 하는 자리.
 *
 * 이벤트가 없을 때도 자리를 비우지 않는다. 자리가 사라지면 배치가 흔들려서
 * 다른 정보의 위치까지 바뀐다. 관제 화면에서 그건 피해야 한다.
 */

import type {
  EmergencyEventSnapshot,
  MissionSummary,
  RobotSnapshot,
} from '../../types/telemetry';
import { clockText, eventDisplay, missionDisplay } from '../common/status';
import { Badge } from '../common/Indicators';
import { EtaBadge, EtaPanel } from './EtaPanel';

interface Props {
  event: EmergencyEventSnapshot | null;
  missions: MissionSummary[];
  robots: RobotSnapshot[];
  now: number;
}

export function ActiveEvent({ event, missions, robots, now }: Props) {
  if (!event) {
    return (
      <section className="alert alert--idle">
        <div className="alert__idle-text">진행 중인 응급 상황 없음</div>
        <div className="alert__idle-sub">대기 중 · 검출 및 신고 수신 대기</div>
      </section>
    );
  }

  const relevantMissions = missions.filter(
    (mission) => (
      mission.event_id === event.event_id && mission.role === 'aed_delivery'
    ),
  );
  const movingMissions = relevantMissions.filter((mission) => {
    if (mission.final_state !== 'en_route' || mission.dispatched_at === null) {
      return false;
    }
    const robot = robots.find((item) => item.robot_id === mission.robot_id);
    return (
      robot !== undefined
      && !robot.is_docked
      && robot.role === 'aed_delivery'
      && robot.network_ok
      && robot.nav2_ok
      && !robot.emergency_stop
    );
  });
  const startedMissions = relevantMissions.filter(
    (mission) => mission.dispatched_at !== null,
  );
  const firstDispatch = startedMissions.reduce<number | null>(
    (earliest, mission) => (
      earliest === null || (mission.dispatched_at ?? Infinity) < earliest
        ? mission.dispatched_at
        : earliest
    ),
    null,
  );
  const elapsed = firstDispatch === null
    ? null
    : Math.max(now / 1000 - firstDispatch, 0);
  const failureMission = relevantMissions.find((mission) => (
    mission.final_state === 'navigation_error'
    || mission.final_state === 'network_lost'
    || mission.final_state === 'blocked'
    || mission.final_state === 'recovery_wait'
  ));
  const status = movingMissions.length > 0
    ? { label: '출동', tone: 'info' as const }
    : failureMission
      ? missionDisplay(failureMission.final_state)
      : relevantMissions.length > 0
        ? { label: '출동 준비', tone: 'warn' as const }
        : eventDisplay(event.status);

  return (
    <section className="alert alert--active">
      <div className="alert__left">
        <div className="alert__title">
          <Badge tone={status.tone}>{status.label}</Badge>
          <span className="alert__id mono">{event.event_id}</span>
        </div>
        <div className="alert__meta">
          <span>
            출처 <b>{event.source_id || '—'}</b>
          </span>
          <span>
            카메라 <b>{event.camera_id || '—'}</b>
          </span>
          <span>
            구역 <b>{event.zone_id || '—'}</b>
          </span>
          <span>
            신뢰도 <b className="mono">{(event.confidence * 100).toFixed(0)}%</b>
          </span>
          <span>
            확정근거 <b className="mono">{event.consecutive_detections}</b>
          </span>
        </div>
      </div>

      <div className="alert__center">
        <div className="alert__label">
          {elapsed === null ? '출동 상태' : '출동 경과'}
        </div>
        <div className="alert__elapsed mono">
          {elapsed === null ? '대기' : `${elapsed.toFixed(0)}초`}
        </div>
        <div className="alert__label">
          접수 {clockText(event.detected_at)}
        </div>
      </div>

      <EtaPanel missions={movingMissions} now={now} />

      <div className="alert__right">
        <div className="alert__label">목표 좌표</div>
        <div className="alert__coord mono">
          {event.location.x.toFixed(2)}, {event.location.y.toFixed(2)}
        </div>
        <div className="alert__missions">
          {relevantMissions.length === 0 ? (
            <span className="alert__label">배정 대기</span>
          ) : (
            relevantMissions.map((mission) => {
              const actuallyMoving = movingMissions.some(
                (item) => item.mission_id === mission.mission_id,
              );
              const robot = robots.find(
                (item) => item.robot_id === mission.robot_id,
              );
              let display = missionDisplay(mission.final_state);
              if (mission.final_state === 'en_route' && !actuallyMoving) {
                if (!robot) {
                  display = { label: '상태 미수신', tone: 'danger' };
                } else if (robot.is_docked) {
                  display = { label: '출동 대기', tone: 'warn' };
                } else if (!robot.network_ok) {
                  display = { label: '통신 두절', tone: 'danger' };
                } else if (!robot.nav2_ok) {
                  display = { label: '주행 오류', tone: 'danger' };
                } else if (robot.emergency_stop) {
                  display = { label: '비상 정지', tone: 'danger' };
                } else {
                  display = { label: '상태 확인', tone: 'warn' };
                }
              }
              return (
                <span key={mission.mission_id} className="alert__mission">
                  <Badge tone={display.tone}>{display.label}</Badge>
                  <b>{mission.robot_id}</b>
                  {mission.reassignment_count > 0 && (
                    <span className="alert__retry">
                      재할당 {mission.reassignment_count}
                    </span>
                  )}
                  {actuallyMoving && <EtaBadge mission={mission} now={now} />}
                </span>
              );
            })
          )}
        </div>
      </div>
    </section>
  );
}
