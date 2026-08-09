/** 여러 화면에서 반복되는 작은 표시들. 색과 모양을 한 곳에 묶는다. */

import type { Tone } from './status';

export function StatusDot({ tone }: { tone: Tone }) {
  const className =
    tone === 'ok'
      ? 'dot dot--ok'
      : tone === 'warn'
        ? 'dot dot--warn'
        : tone === 'danger'
          ? 'dot dot--danger'
          : 'dot dot--idle';
  return <span className={className} />;
}

export function Badge({
  tone,
  children,
}: {
  tone: Tone;
  children: React.ReactNode;
}) {
  return <span className={`badge badge--${tone}`}>{children}</span>;
}

/**
 * 숫자 한 칸. 라벨은 작게, 값은 크고 고정폭으로 둔다.
 * 관제 화면에서 눈이 값만 훑고 지나갈 수 있어야 한다.
 */
export function Metric({
  label,
  value,
  unit,
  tone = 'idle',
}: {
  label: string;
  value: string | number;
  unit?: string;
  tone?: Tone;
}) {
  const color =
    tone === 'ok'
      ? 'var(--ok)'
      : tone === 'warn'
        ? 'var(--warn)'
        : tone === 'danger'
          ? 'var(--danger)'
          : 'var(--text)';
  return (
    <div className="metric">
      <div className="metric__label">{label}</div>
      <div className="metric__value mono" style={{ color }}>
        {value}
        {unit && <span className="metric__unit">{unit}</span>}
      </div>
    </div>
  );
}

/** 켜짐/꺼짐만 보여주는 칩. 통신·위치추정·Nav2 처럼 참/거짓인 것들. */
export function FlagChip({ label, ok }: { label: string; ok: boolean }) {
  return (
    <span className={`flag ${ok ? 'flag--ok' : 'flag--bad'}`}>
      {label}
    </span>
  );
}
