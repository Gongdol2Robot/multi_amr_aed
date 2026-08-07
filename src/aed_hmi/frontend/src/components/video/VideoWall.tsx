/**
 * 4분할 영상 벽. 로봇 2대 시점 + 고정 웹캠 2대.
 *
 * 순서를 고정한다(로봇 먼저, 웹캠 다음). 관제 화면에서 타일이 매번 다른
 * 자리에 있으면 운영자가 위치를 외울 수 없다. 백엔드가 주는 순서에
 * 의존하지 않고 여기서 정렬한다.
 *
 * 한 갈래를 크게 봐야 할 때가 있다. 4분할로는 멀리 있는 검출 상자가 안
 * 보인다. 숫자키 1~4 로 그 자리를 키우고, 같은 키나 Esc 로 되돌린다.
 * 자리 번호는 위 정렬 규칙 덕분에 늘 같은 갈래를 가리킨다. 급할 때
 * 마우스로 타일을 찾는 것보다 손이 먼저 간다.
 */

import { useEffect, useMemo, useState } from 'react';

import type { StreamHealth } from '../../types/telemetry';
import { VideoTile } from './VideoTile';

const KIND_ORDER: Record<string, number> = { robot: 0, webcam: 1 };

export function VideoWall({ streams }: { streams: StreamHealth[] }) {
  const [focused, setFocused] = useState<string | null>(null);

  const ordered = useMemo(
    () =>
      [...streams].sort((a, b) => {
        const byKind = (KIND_ORDER[a.kind] ?? 9) - (KIND_ORDER[b.kind] ?? 9);
        return byKind !== 0 ? byKind : a.stream_id.localeCompare(b.stream_id);
      }),
    [streams],
  );

  // 순서가 그대로면 핸들러를 다시 달 이유가 없다. 스냅샷은 0.25초마다
  // 오므로, streams 자체를 의존성으로 두면 초당 네 번 붙였다 뗀다.
  const orderKey = ordered.map((item) => item.stream_id).join('|');

  useEffect(() => {
    const ids = orderKey.split('|');

    const onKey = (event: KeyboardEvent) => {
      if (event.altKey || event.ctrlKey || event.metaKey) return;
      const target = event.target as HTMLElement | null;
      // 나중에 검색창이 생겨도 타자가 화면을 바꾸지 않게 한다.
      if (target && /^(INPUT|TEXTAREA|SELECT)$/.test(target.tagName)) return;

      if (event.key === 'Escape' || event.key === '0') {
        setFocused(null);
        return;
      }
      const seat = Number(event.key);
      if (!Number.isInteger(seat) || seat < 1 || seat > ids.length) return;
      const id = ids[seat - 1];
      setFocused((current) => (current === id ? null : id));
    };

    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [orderKey]);

  // 갈래가 사라졌는데 그 자리를 키운 채로 두면 빈 화면이 남는다.
  useEffect(() => {
    if (focused && !orderKey.split('|').includes(focused)) setFocused(null);
  }, [focused, orderKey]);

  const offline = ordered.filter((item) => !item.online).length;
  const shown = focused
    ? ordered.filter((item) => item.stream_id === focused)
    : ordered;

  return (
    <section className="panel wall">
      <header className="panel__title">
        <span>현장 영상</span>
        <span className="wall__tools">
          <span className="wall__keys">
            {ordered.map((item, index) => (
              <button
                key={item.stream_id}
                type="button"
                className={
                  focused === item.stream_id
                    ? 'wall__key wall__key--on'
                    : 'wall__key'
                }
                onClick={() =>
                  setFocused((current) =>
                    current === item.stream_id ? null : item.stream_id,
                  )
                }
                title={item.label}
              >
                {index + 1}
              </button>
            ))}
            <button
              type="button"
              className={focused ? 'wall__key' : 'wall__key wall__key--on'}
              onClick={() => setFocused(null)}
              title="4분할로 되돌리기 (Esc)"
            >
              전체
            </button>
          </span>
          <span className={offline > 0 ? 'wall__warn' : 'wall__ok'}>
            {ordered.length - offline}/{ordered.length} 수신
          </span>
        </span>
      </header>
      <div className={focused ? 'wall__grid wall__grid--one' : 'wall__grid'}>
        {shown.map((item) => (
          <VideoTile
            key={item.stream_id}
            health={item}
            seat={ordered.indexOf(item) + 1}
            focused={focused === item.stream_id}
            onToggle={() =>
              setFocused((current) =>
                current === item.stream_id ? null : item.stream_id,
              )
            }
          />
        ))}
      </div>
    </section>
  );
}
