/**
 * 화면 배치만 담당한다. 데이터는 훅에서, 표현은 각 컴포넌트에서 한다.
 *
 * 배치 원칙은 "위험한 것일수록 위로, 자주 보는 것일수록 왼쪽으로"다.
 *   1행: 시스템 상태 (죽었는지 살았는지)
 *   2행: 진행 중인 응급 상황 (지금 무슨 일인가)
 *   3행: 영상 4분할 + 로봇 상태 (현장이 어떤가)
 *   4행: 출동 이력 (지나간 일)
 */

import { useEffect, useState } from 'react';

import { useLiveSnapshot } from './hooks/useLiveSnapshot';
import { TopBar } from './components/layout/TopBar';
import { ActiveEvent } from './components/mission/ActiveEvent';
import { MissionTable } from './components/mission/MissionTable';
import { RobotCard } from './components/robot/RobotCard';
import { VideoWall } from './components/video/VideoWall';
import './styles/app.css';

/** 경과 시간 표시를 위해 1초마다 다시 그린다. 상태 수신과는 무관하다. */
function useTicker(intervalMs = 1000): number {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), intervalMs);
    return () => window.clearInterval(timer);
  }, [intervalMs]);
  return now;
}

export default function App() {
  const { snapshot, connection, receivedAt } = useLiveSnapshot();
  const now = useTicker();

  const robots = snapshot?.robots ?? [];
  const streams = snapshot?.streams ?? [];

  return (
    <div className="app">
      <TopBar
        connection={connection}
        rosConnected={snapshot?.ros_connected ?? false}
        receivedAt={receivedAt}
        now={now}
        robotCount={robots.length}
      />

      <ActiveEvent
        event={snapshot?.active_event ?? null}
        missions={snapshot?.active_missions ?? []}
        now={now}
      />

      <div className="app__main">
        <VideoWall streams={streams} />

        <section className="panel fleet">
          <header className="panel__title">
            <span>로봇 상태</span>
            <span className="fleet__count">
              {robots.filter((item) => item.healthy).length}/{robots.length} 정상
            </span>
          </header>
          <div className="panel__body fleet__body">
            {robots.length === 0 && (
              <p className="fleet__empty">
                로봇 상태 수신 없음
                <br />
                <span>
                  /aed/robot_state 를 발행하는 노드가 있는지 확인하세요.
                </span>
              </p>
            )}
            {robots.map((robot) => (
              <RobotCard key={robot.robot_id} robot={robot} />
            ))}
          </div>
        </section>
      </div>

      <MissionTable />
    </div>
  );
}
