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
  const [recording, setRecording] = useState(false);
  const [recordError, setRecordError] = useState('');
  const retryDelay = useRef(RETRY_INITIAL_MS);
  const imageRef = useRef<HTMLImageElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const recorderRef = useRef<MediaRecorder | null>(null);
  const captureStreamRef = useRef<MediaStream | null>(null);
  const drawTimerRef = useRef<number | null>(null);
  const chunksRef = useRef<Blob[]>([]);

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

  const releaseCapture = () => {
    if (drawTimerRef.current !== null) {
      window.clearInterval(drawTimerRef.current);
      drawTimerRef.current = null;
    }
    captureStreamRef.current?.getTracks().forEach((track) => track.stop());
    captureStreamRef.current = null;
  };

  const stopRecording = () => {
    const recorder = recorderRef.current;
    if (recorder && recorder.state !== 'inactive') recorder.stop();
  };

  useEffect(() => {
    if (!health.online && recording) stopRecording();
  }, [health.online, recording]);

  useEffect(
    () => () => {
      const recorder = recorderRef.current;
      if (recorder && recorder.state !== 'inactive') recorder.stop();
      releaseCapture();
    },
    [],
  );

  const startRecording = () => {
    setRecordError('');
    const image = imageRef.current;
    const canvas = canvasRef.current;
    if (!health.online || !image || !canvas || image.naturalWidth === 0) {
      setRecordError('영상 준비 안 됨');
      return;
    }
    if (typeof MediaRecorder === 'undefined' || !canvas.captureStream) {
      setRecordError('브라우저 녹화 미지원');
      return;
    }

    canvas.width = image.naturalWidth;
    canvas.height = image.naturalHeight;
    const context = canvas.getContext('2d');
    if (!context) {
      setRecordError('캔버스 생성 실패');
      return;
    }

    const draw = () => {
      try {
        context.drawImage(image, 0, 0, canvas.width, canvas.height);
      } catch {
        setRecordError('영상 캡처 실패');
        stopRecording();
      }
    };
    draw();
    // 이미 받은 MJPEG 프레임을 브라우저에서만 다시 그린다. ROS 구독이나
    // 로봇 네트워크 연결은 추가하지 않는다.
    const captureFps = Math.max(1, Math.min(10, Math.ceil(health.fps || 7)));
    drawTimerRef.current = window.setInterval(draw, 1000 / captureFps);
    try {
      const stream = canvas.captureStream(captureFps);
      captureStreamRef.current = stream;

      const mimeCandidates = [
        'video/webm;codecs=vp9',
        'video/webm;codecs=vp8',
        'video/webm',
      ];
      const mimeType = mimeCandidates.find((value) =>
        MediaRecorder.isTypeSupported(value),
      );
      const recorder = new MediaRecorder(
        stream,
        mimeType ? { mimeType, videoBitsPerSecond: 2_000_000 } : undefined,
      );
      chunksRef.current = [];
      recorder.ondataavailable = (event) => {
        if (event.data.size > 0) chunksRef.current.push(event.data);
      };
      recorder.onerror = () => setRecordError('녹화 중 오류');
      recorder.onstop = () => {
        releaseCapture();
        recorderRef.current = null;
        setRecording(false);
        if (chunksRef.current.length === 0) return;
        const blob = new Blob(chunksRef.current, {
          type: recorder.mimeType || 'video/webm',
        });
        chunksRef.current = [];
        const url = URL.createObjectURL(blob);
        const anchor = document.createElement('a');
        const stamp = new Date().toISOString().replace(/[:.]/g, '-');
        anchor.href = url;
        anchor.download = `${health.stream_id}-${stamp}.webm`;
        anchor.click();
        window.setTimeout(() => URL.revokeObjectURL(url), 1000);
      };
      recorderRef.current = recorder;
      recorder.start(1000);
      setRecording(true);
    } catch {
      releaseCapture();
      setRecordError('브라우저 녹화 시작 실패');
    }
  };

  const toggleRecording = (event: React.MouseEvent<HTMLButtonElement>) => {
    event.stopPropagation();
    if (recording) stopRecording();
    else startRecording();
  };

  const source = `${videoUrl(health.stream_id)}?k=${reloadKey}`;
  const tone = health.online ? 'ok' : 'danger';

  return (
    <figure
      className={focused ? 'tile tile--focus' : 'tile'}
      onClick={onToggle}
      title={focused ? '눌러서 4분할로 (Esc)' : `눌러서 크게 (숫자키 ${seat})`}
    >
      <img
        ref={imageRef}
        className="tile__image"
        src={source}
        alt={health.label}
        crossOrigin="anonymous"
        // 화면이 4개라 지연 로딩이 더 헷갈린다. 항상 즉시 붙인다.
        loading="eager"
      />

      {/* 네 갈래 모두 YOLO 를 돌린다. 검출이 잡힌 타일은 테두리로 즉시
          드러나야 한다. 숫자만으로는 4분할에서 눈에 안 들어온다. */}
      {health.detections > 0 && <div className="tile__alarm" />}

      {/* 어느 숫자키가 이 타일인지 화면에 있어야 외우지 않아도 쓴다. */}
      <span className="tile__seat mono">{seat}</span>

      <button
        type="button"
        className={recording ? 'tile__record tile__record--on' : 'tile__record'}
        onClick={toggleRecording}
        disabled={!health.online && !recording}
        title={recording ? '녹화를 끝내고 저장' : '이 영상만 브라우저에서 녹화'}
      >
        {recording ? '■ 저장' : '● 녹화'}
      </button>
      {recordError && <span className="tile__record-error">{recordError}</span>}

      <canvas ref={canvasRef} className="tile__capture" aria-hidden="true" />

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
