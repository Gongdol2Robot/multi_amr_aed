/**
 * 상단 표시줄. 시스템이 살아 있는지를 한 줄로 말한다.
 *
 * 관제 화면에서 가장 위험한 상태는 "화면은 멀쩡한데 데이터가 멈춘 것"이다.
 * 그래서 마지막 수신 후 몇 초가 지났는지를 항상 보여준다. 값이 멈추면
 * 사람이 알아채기 어렵지만, 초 카운터가 올라가면 바로 보인다.
 */

import type { ConnectionState } from '../../api/socket';
import { StatusDot } from '../common/Indicators';

interface Props {
  connection: ConnectionState;
  rosConnected: boolean;
  receivedAt: number | null;
  now: number;
  robotCount: number;
}

export function TopBar({
  connection,
  rosConnected,
  receivedAt,
  now,
  robotCount,
}: Props) {
  const staleSeconds = receivedAt === null ? null : (now - receivedAt) / 1000;
  const linkTone =
    connection === 'open' ? (staleSeconds ?? 0) < 2 ? 'ok' : 'warn' : 'danger';

  return (
    <header className="topbar">
      <div className="topbar__brand">
        <span className="topbar__mark">AED</span>
        <span className="topbar__title">Multi-AMR 관제</span>
      </div>

      <div className="topbar__status">
        <span className="topbar__item">
          <StatusDot tone={linkTone} />
          관제 서버{' '}
          <b>
            {connection === 'open'
              ? '연결'
              : connection === 'connecting'
                ? '연결 중'
                : '끊김'}
          </b>
        </span>

        <span className="topbar__item">
          <StatusDot tone={rosConnected ? 'ok' : 'danger'} />
          ROS <b>{rosConnected ? '수신' : '없음'}</b>
        </span>

        <span className="topbar__item">
          로봇 <b className="mono">{robotCount}</b>대
        </span>

        <span className="topbar__item">
          최종 수신{' '}
          <b className="mono">
            {staleSeconds === null ? '—' : `${staleSeconds.toFixed(1)}초 전`}
          </b>
        </span>
      </div>

      <div className="topbar__clock mono">
        {new Date(now).toLocaleTimeString('ko-KR', { hour12: false })}
      </div>
    </header>
  );
}
