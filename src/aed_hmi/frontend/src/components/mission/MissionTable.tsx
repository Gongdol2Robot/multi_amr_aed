/**
 * 지난 임무 표. SQLite 에 쌓인 이력을 읽어 온다.
 *
 * 실시간 상태와 달리 스스로 갱신되지 않으므로 주기적으로 다시 받는다.
 * 주기를 짧게 잡을 이유는 없다. 이력은 초 단위로 바뀌지 않는다.
 */

import { useEffect, useState } from 'react';

import {
  fetchEtaAccuracy,
  fetchMissions,
  fetchResponseStats,
} from '../../api/http';
import type {
  EtaAccuracyStats,
  MissionSummary,
  ResponseTimeStats,
} from '../../types/telemetry';
import { clockText, durationText, missionDisplay } from '../common/status';
import { Badge, Metric } from '../common/Indicators';

const REFRESH_MS = 5000;

export function MissionTable() {
  const [missions, setMissions] = useState<MissionSummary[]>([]);
  const [stats, setStats] = useState<ResponseTimeStats | null>(null);
  const [eta, setEta] = useState<EtaAccuracyStats | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;

    const load = async () => {
      try {
        const [items, summary, accuracy] = await Promise.all([
          fetchMissions(30),
          fetchResponseStats(),
          fetchEtaAccuracy(1),
        ]);
        if (cancelled) return;
        setMissions(items);
        setStats(summary);
        setEta(accuracy);
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

          {/* 예상이 얼마나 맞았나. 평균 오차는 늦은 건과 빠른 건이 서로
              지워버리므로, 정확도로 보여줄 값은 절대 오차 쪽이다. */}
          {eta && eta.total > 0 && (
            <>
              <Metric
                label="예상 오차"
                value={durationText(eta.avg_abs_error_sec)}
                tone={
                  (eta.avg_abs_error_sec ?? 0) <= 5 ? 'ok' : 'warn'
                }
              />
              <Metric
                label="예상보다 늦음"
                value={`${eta.late_count}/${eta.total}`}
                unit="건"
                tone={
                  eta.late_count * 2 <= eta.total ? 'ok' : 'warn'
                }
              />
            </>
          )}
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
              <th>예상시간</th>
              <th>오차율</th>
              <th>재할당</th>
              <th>사유</th>
            </tr>
          </thead>
          <tbody>
            {missions.length === 0 && (
              <tr>
                <td colSpan={10} className="table__empty">
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
                  <td
                    className="mono"
                    title={
                      mission.actual_travel_seconds == null
                        ? undefined
                        : `실제 주행 ${durationText(mission.actual_travel_seconds)}`
                    }
                  >
                    {durationText(mission.predicted_eta_seconds)}
                  </td>
                  <td
                    className={
                      mission.eta_error_rate_percent == null
                        ? 'mono'
                        : mission.eta_error_rate_percent <= 15
                          ? 'mono table__eta-ok'
                          : 'mono table__eta-warn'
                    }
                  >
                    {mission.eta_error_rate_percent == null
                      ? '—'
                      : `${mission.eta_error_rate_percent.toFixed(1)}%`}
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
