#!/usr/bin/env python3
"""관제 화면을 이 PC 안에서 사진·영상으로 남긴다.

왜 필요한가
-----------
원격으로 접속해 쓰면 Print 키는 **접속한 쪽 PC**에 저장된다. 그렇다고 이
PC 에서 화면을 찍으려 해도 Wayland 라 막혀 있다. GNOME 42 부터 D-Bus
스크린샷이 AccessDenied 이고, ImageMagick 의 import 나 xdotool 은 X11
전용이라 Wayland 화면을 못 본다.

그래서 화면을 거치지 않는다. 브라우저를 창 없이 띄워 페이지를 직접 그리고
그 그림을 파일로 받는다. 접속 방식과 무관하게 이 PC 에 저장된다.

영상은 그 그림을 일정 간격으로 여러 장 받아 이어 붙인다. 화면 녹화가
아니라 연속 촬영이라, 마우스 커서나 다른 창이 안 들어온다. 시연 자료로는
그 편이 깨끗하다.

사용:
  python3 tools/capture_hmi.py                       한 장
  python3 tools/capture_hmi.py --seconds 40          40초 영상
  python3 tools/capture_hmi.py --url http://... --out docs/videos/hmi.mp4
"""
import argparse
import base64
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

CHROME = "google-chrome"
DEBUG_PORT = 9455


def find_chrome() -> str:
    for name in (CHROME, "google-chrome-stable", "chromium", "chromium-browser"):
        path = shutil.which(name)
        if path:
            return path
    raise SystemExit("실패: 크롬을 못 찾았습니다")


def wait_for_devtools(port: int, timeout: float = 20.0):
    """브라우저가 뜰 때까지 기다린 뒤, 페이지 탭의 주소를 준다."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            tabs = json.load(urllib.request.urlopen(
                f"http://127.0.0.1:{port}/json", timeout=1.5
            ))
            pages = [t for t in tabs if t.get("type") == "page"]
            if pages:
                return pages[0]["webSocketDebuggerUrl"]
        except Exception:
            time.sleep(0.3)
    raise SystemExit("실패: 브라우저가 안 떴습니다")


def capture(url: str, out: str, seconds: float, fps: float,
            width: int, height: int, settle: float) -> int:
    import websockets  # 여기서만 필요하다

    import asyncio

    chrome = find_chrome()
    profile = tempfile.mkdtemp(prefix="hmi-capture-")
    process = subprocess.Popen(
        [chrome, "--headless=new", "--disable-gpu", "--no-sandbox",
         f"--remote-debugging-port={DEBUG_PORT}",
         f"--user-data-dir={profile}",
         f"--window-size={width},{height}", url],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    frames_dir = tempfile.mkdtemp(prefix="hmi-frames-")
    count = 0
    try:
        socket_url = wait_for_devtools(DEBUG_PORT)

        async def run():
            nonlocal count
            message_id = 0

            async with websockets.connect(socket_url, max_size=None) as ws:
                async def send(method, params=None):
                    nonlocal message_id
                    message_id += 1
                    await ws.send(json.dumps({
                        "id": message_id, "method": method,
                        "params": params or {},
                    }))
                    while True:
                        reply = json.loads(await ws.recv())
                        if reply.get("id") == message_id:
                            return reply.get("result", {})

                # 첫 그림이 다 그려질 때까지 기다린다. 영상 타일은 붙는 데
                # 시간이 걸려서, 바로 찍으면 빈 칸이 나온다.
                await asyncio.sleep(settle)

                if seconds <= 0:
                    result = await send("Page.captureScreenshot",
                                        {"format": "png"})
                    with open(out, "wb") as handle:
                        handle.write(base64.b64decode(result["data"]))
                    count = 1
                    return

                interval = 1.0 / fps
                end = time.time() + seconds
                while time.time() < end:
                    started = time.time()
                    result = await send("Page.captureScreenshot",
                                        {"format": "png"})
                    path = os.path.join(frames_dir, f"{count:05d}.png")
                    with open(path, "wb") as handle:
                        handle.write(base64.b64decode(result["data"]))
                    count += 1
                    if count % 20 == 0:
                        print(f"  {count}장", flush=True)
                    rest = interval - (time.time() - started)
                    if rest > 0:
                        await asyncio.sleep(rest)

        asyncio.run(run())

        if seconds > 0:
            if count == 0:
                raise SystemExit("실패: 한 장도 못 찍었습니다")
            subprocess.run(
                ["ffmpeg", "-y", "-loglevel", "error",
                 "-framerate", str(fps), "-i",
                 os.path.join(frames_dir, "%05d.png"),
                 # h.264 는 가로·세로가 짝수여야 한다. 페이지 높이는
                 # 내용에 따라 정해져 홀수가 나오므로 여기서 깎는다.
                 "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                 "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
                 "-pix_fmt", "yuv420p", out],
                check=True,
            )
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
        shutil.rmtree(profile, ignore_errors=True)
        shutil.rmtree(frames_dir, ignore_errors=True)

    size = os.path.getsize(out) / 1e6
    if seconds > 0:
        print(f"저장: {out}  {count}장 @ {fps:.0f}fps "
              f"= {count / fps:.0f}초 · {size:.1f}MB")
    else:
        print(f"저장: {out}  {size:.1f}MB")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="관제 화면을 이 PC 에 사진·영상으로 남긴다"
    )
    parser.add_argument("--url", default="http://localhost:5173/")
    parser.add_argument("--out", default=None)
    parser.add_argument("--seconds", type=float, default=0,
                        help="0 이면 사진 한 장. 크면 그 길이의 영상")
    parser.add_argument("--fps", type=float, default=4.0)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1000)
    parser.add_argument("--settle", type=float, default=12.0,
                        help="첫 그림을 기다리는 초. 영상 타일이 붙을 시간")
    args = parser.parse_args()

    out = args.out or (
        "docs/images/hmi_demo.mp4" if args.seconds > 0
        else "docs/images/hmi_shot.png"
    )
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if not os.path.isabs(out):
        out = os.path.join(root, out)
    os.makedirs(os.path.dirname(out), exist_ok=True)

    return capture(args.url, out, args.seconds, args.fps,
                   args.width, args.height, args.settle)


if __name__ == "__main__":
    sys.exit(main())
