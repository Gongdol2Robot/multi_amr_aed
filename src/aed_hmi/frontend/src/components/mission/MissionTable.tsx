/**
 * 지난 임무 표. SQLite 에 쌓인 이력을 읽어 온다.
 *
 * 실시간 상태와 달리 스스로 갱신되지 않으므로 주기적으로 다시 받는다.
 * 주기를 짧게 잡을 이유는 없다. 이력은 초 단위로 바뀌지 않는다.
 */

import { useEffect, useState } from 'react';

import { fetchMissions, fetchResponseStats } from '../../api/http';
import type { MissionSummary, ResponseTimeStats } from '../../types/telemetry';
import { clockText, durationText, missionDisplay } from '../common/status';
import { Badge, Metric } from '../common/Indicators';

const REFRESH_MS = 5000;

export function MissionTable() {
  const [missions, setMissions] = useState<MissionSummary[]>([]);
  const [stats, setStats] = useState<ResponseTimeStats | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;

    const load = async () => {
      try {
        const [items, summary] = await Promise.all([
          fetchMissions(30),
          fetchResponseStats(),
        ]);
        if (cancelled) return;
        setMissions(items);
        setStats(summary);
        setError(null);
      } catch (cause) {
        if (!cancelled) setError((cause as Error).message);
      }
    };

    load();
    const timer = window.setInterval(load, REFRESH_MS);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, []);

  return (
    <section className="panel history">
      <header className="panel__title">
        <span>출동 이력</span>
        {error && <span className="history__error">{error}</span>}
      </header>

      {stats && (
        <div className="history__stats">
          <Metric label="총 출동" value={stats.total} unit="건" />
          <Metric
            label="도착 완료"
            value={stats.arrived}
            unit="건"
            tone={stats.arrived === stats.total ? 'ok' : 'warn'}
          />
          <Metric
            label="평균 응답"
            value={durationText(stats.avg_seconds)}
          />
          <Metric label="최단" value={durationText(stats.min_seconds)} />
          <Metric label="최장" value={durationText(stats.max_seconds)} />
        </div>
      )}

      <div className="panel__body history__body">
        <table className="table">
          <thead>
            <tr>
              <th>임무</th>
              <th>로봇</th>
              <th>상태</th>
              <th>접수</th>
              <th>도착</th>
              <th>응답시간</th>
              <th>재할당</th>
              <th>사유</th>
            </tr>
          </thead>
          <tbody>
            {missions.length === 0 && (
              <tr>
                <td colSpan={8} className="table__empty">
                  기록 없음
                </td>
              </tr>
            )}
            {missions.map((mission) => {
              const display = missionDisplay(mission.final_state);
              return (
                <tr key={mission.mission_id}>
                  <td className="mono">{mission.mission_id}</td>
                  <td>{mission.robot_id || '—'}</td>
                  <td>
                    <Badge tone={display.tone}>{display.label}</Badge>
                  </td>
                  <td className="mono">{clockText(mission.called_at)}</td>
                  <td className="mono">{clockText(mission.arrived_at)}</td>
                  <td className="mono">
                    {durationText(mission.response_seconds)}
                  </td>
                  <td className="mono">
                    {mission.reassignment_count > 0
                      ? mission.reassignment_count
                      : '—'}
                  </td>
                  <td className="table__reason">
                    {mission.failure_reasons.join(' / ') || '—'}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </section>
  );
}
