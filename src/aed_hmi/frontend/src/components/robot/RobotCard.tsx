/**
 * 로봇 한 대의 상태 카드.
 *
 * RobotState.msg 가 싣는 것 중 운영자가 판단에 쓰는 것만 올린다.
 * 전부 늘어놓으면 급할 때 읽히지 않는다.
 *
 * 맨 위 한 줄로 "지금 쓸 수 있나"가 끝나야 한다. 나머지는 왜 그런지에 대한
 * 근거다.
 */

import type { RobotSnapshot } from '../../types/telemetry';
import {
  availabilityDisplay,
  batteryTone,
  roleLabel,
} from '../common/status';
import { Badge, FlagChip, Metric, StatusDot } from '../common/Indicators';

function recoveryNotice(robot: RobotSnapshot): {
  label: string;
  tone: 'ok' | 'warn' | 'danger' | 'info';
} | null {
  switch (robot.fallback_state) {
    case 'STARTING':
      return { label: 'Depth·cmd_vel 전환 중', tone: 'warn' };
    case 'ACTIVE':
      return { label: 'Depth·cmd_vel 주행 중', tone: 'info' };
    case 'BLOCKED':
      return { label: 'Depth 장애물 감지 · 정지', tone: 'danger' };
    case 'SUCCEEDED':
      return { label: 'Depth fallback 도착', tone: 'ok' };
    case 'FAILED':
      return { label: 'Fallback 실패 · 대체 필요', tone: 'danger' };
    default:
      break;
  }
  switch (robot.lidar_state) {
    case 'STARTING':
      return { label: 'LiDAR 시작 확인 중', tone: 'warn' };
    case 'FAULT':
      return { label: 'LiDAR 장애 · fallback 대기', tone: 'danger' };
    case 'RECOVERING':
      return { label: 'LiDAR 복구 확인 중', tone: 'warn' };
    default:
      return null;
  }
}

export function RobotCard({ robot }: { robot: RobotSnapshot }) {
  const availability = availabilityDisplay(robot.availability);
  // 가용성과 별개로 하부 신호가 하나라도 죽었으면 그것부터 보여준다.
  // availability 는 mission_manager 의 판단이고, 이건 원인이다.
  const tone = robot.healthy ? availability.tone : 'danger';
  const recovery = recoveryNotice(robot);

  return (
    <article className={`robot ${robot.healthy ? '' : 'robot--fault'}`}>
      <header className="robot__head">
        <div className="robot__id">
          <StatusDot tone={tone} />
          <strong>{robot.robot_id}</strong>
          {robot.is_docked && <span className="robot__dock">도킹</span>}
        </div>
        <Badge tone={availability.tone}>{availability.label}</Badge>
      </header>

      <div className="robot__metrics">
        <Metric
          label="속도"
          value={robot.speed_mps.toFixed(2)}
          unit="m/s"
        />
        <Metric
          label="배터리"
          value={robot.battery_percentage.toFixed(0)}
          unit="%"
          tone={batteryTone(robot.battery_percentage)}
        />
        <Metric
          label="위치"
          value={`${robot.position.x.toFixed(2)}, ${robot.position.y.toFixed(2)}`}
        />
        <Metric label="방위" value={robot.yaw_deg.toFixed(0)} unit="°" />
      </div>

      <div className="robot__flags">
        <FlagChip label="통신" ok={robot.network_ok} />
        <FlagChip label="위치추정" ok={robot.localization_ok} />
        <FlagChip label="Nav2" ok={robot.nav2_ok} />
        <FlagChip label="경로" ok={robot.path_valid} />
        {robot.lidar_ok === null ? (
          <Badge tone="idle">LiDAR 감시 미연결</Badge>
        ) : (
          <FlagChip label="LiDAR" ok={robot.lidar_ok} />
        )}
        {robot.emergency_stop && <FlagChip label="비상정지" ok={false} />}
      </div>

      {recovery && (
        <div className="robot__recovery">
          <Badge tone={recovery.tone}>{recovery.label}</Badge>
          <span className="mono">
            {robot.lidar_state} / {robot.fallback_state}
          </span>
        </div>
      )}

      <footer className="robot__foot">
        <span className="robot__role">{roleLabel(robot.role)}</span>
        {robot.mission_id && (
          <span className="robot__mission mono">{robot.mission_id}</span>
        )}
        {/* 하트비트가 오래됐다는 것은 통신 두절의 첫 신호다. 값 자체보다
            "몇 초째"가 중요하므로 초 단위로만 보여준다. */}
        {robot.heartbeat_age_s > 2 && (
          <span className="robot__stale mono">
            HB {robot.heartbeat_age_s.toFixed(0)}s
          </span>
        )}
      </footer>

      {robot.detail && <p className="robot__detail">{robot.detail}</p>}
    </article>
  );
}
