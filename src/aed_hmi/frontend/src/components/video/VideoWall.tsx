/**
 * 4분할 영상 벽. 로봇 2대 시점 + 고정 웹캠 2대.
 *
 * 순서를 고정한다(로봇 먼저, 웹캠 다음). 관제 화면에서 타일이 매번 다른
 * 자리에 있으면 운영자가 위치를 외울 수 없다. 백엔드가 주는 순서에
 * 의존하지 않고 여기서 정렬한다.
 */

import type { StreamHealth } from '../../types/telemetry';
import { VideoTile } from './VideoTile';

const KIND_ORDER: Record<string, number> = { robot: 0, webcam: 1 };

export function VideoWall({ streams }: { streams: StreamHealth[] }) {
  const ordered = [...streams].sort((a, b) => {
    const byKind = (KIND_ORDER[a.kind] ?? 9) - (KIND_ORDER[b.kind] ?? 9);
    return byKind !== 0 ? byKind : a.stream_id.localeCompare(b.stream_id);
  });

  const offline = ordered.filter((item) => !item.online).length;

  return (
    <section className="panel wall">
      <header className="panel__title">
        <span>현장 영상</span>
        <span className={offline > 0 ? 'wall__warn' : 'wall__ok'}>
          {ordered.length - offline}/{ordered.length} 수신
        </span>
      </header>
      <div className="wall__grid">
        {ordered.map((item) => (
          <VideoTile key={item.stream_id} health={item} />
        ))}
      </div>
    </section>
  );
}
