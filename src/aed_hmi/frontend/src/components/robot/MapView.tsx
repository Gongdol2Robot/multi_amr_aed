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

import { API_BASE, fetchMapMeta, postOperatorReport } from '../../api/http';
import type {
  CrowdZoneSnapshot,
  MapMeta,
  MissionSummary,
  RobotSnapshot,
} from '../../types/telemetry';

interface Props {
  robots: RobotSnapshot[];
  missions: MissionSummary[];
  crowdZone: CrowdZoneSnapshot | null;
}

/** 그림 안의 백분율 위치 → 지도 좌표(m). 클릭한 자리를 되돌린다. */
function toMap(meta: MapMeta, ratioX: number, ratioY: number) {
  const px = ratioX * meta.width;
  const py = ratioY * meta.height;
  return {
    x: px * meta.resolution + meta.origin_x,
    y: (meta.height - py) * meta.resolution + meta.origin_y,
  };
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

function toPixel(meta: MapMeta, x: number, y: number) {
  return {
    x: (x - meta.origin_x) / meta.resolution,
    y: meta.height - (y - meta.origin_y) / meta.resolution,
  };
}

export function MapView({ robots, missions, crowdZone }: Props) {
  const [meta, setMeta] = useState<MapMeta | null>(null);
  const [failed, setFailed] = useState(false);
  // 방금 찍은 자리와 그 결과. 보낸 뒤 잠깐 보여주고 지운다.
  const [picked, setPicked] = useState<{ x: number; y: number } | null>(null);
  const [notice, setNotice] = useState<string | null>(null);

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

  const onPick = async (event: React.MouseEvent<HTMLDivElement>) => {
    if (!meta) return;
    // 클릭 지점을 그림 안의 비율로 바꾼다. 그림이 확대·축소돼 있어도
    // 비율은 그대로라 좌표 계산이 흔들리지 않는다.
    const box = event.currentTarget.getBoundingClientRect();
    const point = toMap(
      meta,
      (event.clientX - box.left) / box.width,
      (event.clientY - box.top) / box.height,
    );
    setPicked(point);
    setNotice('보내는 중…');
    try {
      const result = await postOperatorReport(point.x, point.y);
      setNotice(`신고 접수 ${result.event_id}`);
    } catch (error) {
      setNotice((error as Error).message);
    }
    window.setTimeout(() => setNotice(null), 4000);
  };

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
        <div
          className="map__inner map__inner--pickable"
          style={{ aspectRatio: `${meta.width} / ${meta.height}` }}
          onClick={onPick}
          title="누르면 그 자리를 신고로 내보냅니다"
        >
        <img
          className="map__image"
          src={`${API_BASE}/api/map/image`}
          alt="공용 지도"
        />

        {crowdZone && crowdZone.polygon.length >= 3 && (
          <>
            <svg
              className={`map__crowd map__crowd--${
                crowdZone.fresh ? crowdZone.level : 'unknown'
              }`}
              viewBox={`0 0 ${meta.width} ${meta.height}`}
              preserveAspectRatio="none"
              aria-label={`혼잡 구역 ${crowdZone.level_name}`}
            >
              <polygon
                points={crowdZone.polygon
                  .map((point) => {
                    const pixel = toPixel(meta, point.x, point.y);
                    return `${pixel.x},${pixel.y}`;
                  })
                  .join(' ')}
                vectorEffect="non-scaling-stroke"
              />
            </svg>
            <span
              className="map__crowd-label"
              style={toPercent(
                meta,
                crowdZone.polygon.reduce((sum, point) => sum + point.x, 0)
                  / crowdZone.polygon.length,
                crowdZone.polygon.reduce((sum, point) => sum + point.y, 0)
                  / crowdZone.polygon.length,
              )}
            >
              골목 {crowdZone.fresh ? crowdZone.level_name : 'UNKNOWN'} ·{' '}
              {crowdZone.person_count}명
            </span>
          </>
        )}

        {/* 목표부터 그린다. 로봇 표시가 그 위에 오게 하려는 것이다. */}
        {target && (
          <span
            className="map__target"
            style={toPercent(meta, target.target.x, target.target.y)}
            title={`목표 ${target.target.x.toFixed(2)}, ${target.target.y.toFixed(2)}`}
          />
        )}
        {target && (
          <span
            className="map__label map__label--target mono"
            style={toPercent(meta, target.target.x, target.target.y)}
          >
            목표 {target.target.x.toFixed(2)}, {target.target.y.toFixed(2)}
          </span>
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

        {picked && (
          <>
            <span
              className="map__picked"
              style={toPercent(meta, picked.x, picked.y)}
            />
            <span
              className="map__label map__label--picked mono"
              style={toPercent(meta, picked.x, picked.y)}
            >
              찍음 {picked.x.toFixed(2)}, {picked.y.toFixed(2)}
            </span>
          </>
        )}

        {/* 좌표는 표시 옆에 글로 찍는다. 지도만으로는 "대략 저쯤"까지고,
            숫자가 있어야 다른 화면·로그와 맞춰볼 수 있다. */}
        {robots.map((robot) => (
          <span
            key={`${robot.robot_id}-label`}
            className="map__label mono"
            style={toPercent(meta, robot.position.x, robot.position.y)}
          >
            {robot.robot_id} {robot.position.x.toFixed(2)},{' '}
            {robot.position.y.toFixed(2)}
          </span>
        ))}
        </div>
      </div>

      {notice && <div className="map__notice">{notice}</div>}
    </section>
  );
}
