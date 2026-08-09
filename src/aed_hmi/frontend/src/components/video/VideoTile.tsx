/**
 * 영상 한 갈래. MJPEG 를 <img> 로 받는다.
 *
 * 브라우저가 multipart/x-mixed-replace 를 알아서 갱신해 주므로 별도
 * 플레이어가 필요 없다. 대신 두 가지를 직접 챙겨야 한다.
 *
 * 1) 끊긴 것을 img 만으로는 알 수 없다. 그래서 fps 는 백엔드가 준 값을 쓴다.
 * 2) 오래 켜 두면 연결이 죽은 채 마지막 프레임이 남는다. 그래서 끊긴 것이
 *    확인되면 주소에 시각을 붙여 강제로 다시 연결한다.
 */

import { useEffect, useRef, useState } from 'react';

import { videoUrl } from '../../api/http';
import type { StreamHealth } from '../../types/telemetry';
import { StatusDot } from '../common/Indicators';

interface Props {
  health: StreamHealth;
  /** 화면에서 이 타일의 자리 번호. 그대로 단축키 숫자가 된다. */
  seat: number;
  focused: boolean;
  onToggle: () => void;
}

const RETRY_INITIAL_MS = 4000;
const RETRY_MAX_MS = 30000;

export function VideoTile({ health, seat, focused, onToggle }: Props) {
  const [reloadKey, setReloadKey] = useState(0);
  const retryDelay = useRef(RETRY_INITIAL_MS);

  useEffect(() => {
    if (health.online) {
      retryDelay.current = RETRY_INITIAL_MS;
      return;
    }
    // 장시간 끊긴 카메라를 4초마다 계속 두드리지 않는다. 처음에는 빠르게
    // 복구하고, 실패가 이어지면 최대 30초까지 간격을 늘린다.
    const delay = retryDelay.current;
    const timer = window.setTimeout(() => {
      retryDelay.current = Math.min(delay * 2, RETRY_MAX_MS);
      setReloadKey((n) => n + 1);
    }, delay);
    return () => window.clearTimeout(timer);
  }, [health.online, reloadKey]);

  const source = `${videoUrl(health.stream_id)}?k=${reloadKey}`;
  const tone = health.online ? 'ok' : 'danger';

  return (
    <figure
      className={focused ? 'tile tile--focus' : 'tile'}
      onClick={onToggle}
      title={focused ? '눌러서 4분할로 (Esc)' : `눌러서 크게 (숫자키 ${seat})`}
    >
      <img
        className="tile__image"
        src={source}
        alt={health.label}
        // 화면이 4개라 지연 로딩이 더 헷갈린다. 항상 즉시 붙인다.
        loading="eager"
      />

      {/* 네 갈래 모두 YOLO 를 돌린다. 검출이 잡힌 타일은 테두리로 즉시
          드러나야 한다. 숫자만으로는 4분할에서 눈에 안 들어온다. */}
      {health.detections > 0 && <div className="tile__alarm" />}

      {/* 어느 숫자키가 이 타일인지 화면에 있어야 외우지 않아도 쓴다. */}
      <span className="tile__seat mono">{seat}</span>

      <figcaption className="tile__bar">
        <span className="tile__name">
          <StatusDot tone={tone} />
          {health.label}
        </span>
        <span className="tile__stats mono">
          {health.online ? `${health.fps.toFixed(1)} fps` : 'NO SIGNAL'}
          {health.detections > 0 && (
            <span className="tile__detect">
              검출 {health.detections}
            </span>
          )}
        </span>
      </figcaption>

      {!health.online && (
        <div className="tile__offline">
          <div className="tile__offline-text">신호 없음</div>
          <div className="tile__offline-sub mono">{health.stream_id}</div>
        </div>
      )}
    </figure>
  );
}
