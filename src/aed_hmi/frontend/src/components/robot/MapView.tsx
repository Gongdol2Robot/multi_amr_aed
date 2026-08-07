/**
 * 공용 지도 위에 로봇과 목표를 찍는다.
 *
 * 로봇 카드는 좌표를 숫자로 보여주지만, 숫자만으로는 두 로봇이 서로 어디
 * 있는지, 목표가 어느 쪽인지가 안 잡힌다. 급할 때 필요한 것은 그 관계다.
 *
 * 좌표 변환은 지도가 준 계수로 한다. 서버가 /api/map/meta 로 해상도와
 * 원점을 주므로 화면은 그것만 쓰면 된다. 값을 여기에 적어 두면 지도를
 * 다시 만들 때마다 화면 코드를 고쳐야 한다.
 *
 *   px = (x - origin_x) / resolution
 *   py = height - (y - origin_y) / resolution      y 축이 뒤집힌다
 *
 * ROS 지도는 왼쪽 아래가 원점이고 위로 갈수록 y 가 커진다. 그림은 왼쪽
 * 위가 원점이라 y 를 뒤집어야 한다.
 */

import { useEffect, useState } from 'react';

import { API_BASE, fetchMapMeta } from '../../api/http';
import type { MapMeta, MissionSummary, RobotSnapshot } from '../../types/telemetry';

interface Props {
  robots: RobotSnapshot[];
  missions: MissionSummary[];
}

/** 지도 좌표(m) → 그림 안의 백분율 위치. CSS 로 그대로 쓴다. */
function toPercent(meta: MapMeta, x: number, y: number) {
  const px = (x - meta.origin_x) / meta.resolution;
  const py = meta.height - (y - meta.origin_y) / meta.resolution;
  return {
    left: `${(px / meta.width) * 100}%`,
    top: `${(py / meta.height) * 100}%`,
  };
}

export function MapView({ robots, missions }: Props) {
  const [meta, setMeta] = useState<MapMeta | null>(null);
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    let cancelled = false;
    // 지도는 시연 중에 바뀌지 않는다. 한 번만 받는다.
    fetchMapMeta()
      .then((value) => !cancelled && setMeta(value))
      .catch(() => !cancelled && setFailed(true));
    return () => {
      cancelled = true;
    };
  }, []);

  if (failed) {
    return (
      <section className="panel map">
        <header className="panel__title">
          <span>지도</span>
        </header>
        <div className="map__empty">지도 파일 없음 (maps/map.yaml)</div>
      </section>
    );
  }
  if (!meta) return null;

  const target = missions.find((mission) => mission.target.x || mission.target.y);

  return (
    <section className="panel map">
      <header className="panel__title">
        <span>지도</span>
        <span className="map__scale mono">
          {(meta.width * meta.resolution).toFixed(1)} ×{' '}
          {(meta.height * meta.resolution).toFixed(1)} m
        </span>
      </header>

      <div className="map__frame">
        {/* 표시는 그림 크기를 기준으로 %로 놓는다. 바깥 칸을 기준으로
            두면 그림이 칸보다 작아질 때 표시가 그림 밖으로 밀린다. */}
        <div className="map__inner">
        <img
          className="map__image"
          src={`${API_BASE}/api/map/image`}
          alt="공용 지도"
          style={{ aspectRatio: `${meta.width} / ${meta.height}` }}
        />

        {/* 목표부터 그린다. 로봇 표시가 그 위에 오게 하려는 것이다. */}
        {target && (
          <span
            className="map__target"
            style={toPercent(meta, target.target.x, target.target.y)}
            title={`목표 ${target.target.x.toFixed(2)}, ${target.target.y.toFixed(2)}`}
          />
        )}

        {robots.map((robot) => (
          <span
            key={robot.robot_id}
            className={
              robot.availability === 'busy' ? 'map__robot map__robot--busy'
                : 'map__robot'
            }
            style={{
              ...toPercent(meta, robot.position.x, robot.position.y),
              // 로봇이 바라보는 쪽을 삼각형 머리로 알린다. 화면 y 축이
              // 아래로 자라므로 각도의 부호를 뒤집는다.
              transform: `translate(-50%, -50%) rotate(${-robot.yaw_deg}deg)`,
            }}
            title={`${robot.robot_id} ${robot.position.x.toFixed(2)}, ${robot.position.y.toFixed(2)}`}
          >
            <b>{robot.robot_id.replace('robot', '')}</b>
          </span>
        ))}
        </div>
      </div>
    </section>
  );
}
