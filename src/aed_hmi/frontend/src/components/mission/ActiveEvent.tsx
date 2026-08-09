/**
 * 진행 중인 응급 상황 배너. 화면에서 가장 눈에 띄어야 하는 자리.
 *
 * 이벤트가 없을 때도 자리를 비우지 않는다. 자리가 사라지면 배치가 흔들려서
 * 다른 정보의 위치까지 바뀐다. 관제 화면에서 그건 피해야 한다.
 */

import type {
  EmergencyEventSnapshot,
  MissionSummary,
} from '../../types/telemetry';
import { clockText, eventDisplay, missionDisplay } from '../common/status';
import { Badge } from '../common/Indicators';
import { EtaBadge, EtaPanel } from './EtaPanel';

interface Props {
  event: EmergencyEventSnapshot | null;
  missions: MissionSummary[];
  now: number;
}

export function ActiveEvent({ event, missions, now }: Props) {
  if (!event) {
    return (
      <section className="alert alert--idle">
        <div className="alert__idle-text">진행 중인 응급 상황 없음</div>
        <div className="alert__idle-sub">대기 중 · 검출 및 신고 수신 대기</div>
      </section>
    );
  }

  const status = eventDisplay(event.status);
  // 신고 시각부터 지금까지. 운영자가 가장 자주 보는 숫자라 크게 둔다.
  const elapsed = Math.max(now / 1000 - event.detected_at, 0);

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
            연속검출 <b className="mono">{event.consecutive_detections}</b>
          </span>
        </div>
      </div>

      <div className="alert__center">
        <div className="alert__label">경과</div>
        <div className="alert__elapsed mono">{elapsed.toFixed(0)}초</div>
        <div className="alert__label">
          접수 {clockText(event.detected_at)}
        </div>
      </div>

      <EtaPanel missions={missions} now={now} />

      <div className="alert__right">
        <div className="alert__label">목표 좌표</div>
        <div className="alert__coord mono">
          {event.location.x.toFixed(2)}, {event.location.y.toFixed(2)}
        </div>
        <div className="alert__missions">
          {missions.length === 0 ? (
            <span className="alert__label">배정 대기</span>
          ) : (
            missions.map((mission) => {
              const display = missionDisplay(mission.final_state);
              return (
                <span key={mission.mission_id} className="alert__mission">
                  <Badge tone={display.tone}>{display.label}</Badge>
                  <b>{mission.robot_id}</b>
                  {mission.reassignment_count > 0 && (
                    <span className="alert__retry">
                      재할당 {mission.reassignment_count}
                    </span>
                  )}
                  <EtaBadge mission={mission} now={now} />
                </span>
              );
            })
          )}
        </div>
      </div>
    </section>
  );
}
