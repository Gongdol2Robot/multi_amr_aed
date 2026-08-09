#!/usr/bin/env python3
"""rosbag 에 담긴 CompressedImage 를 mp4 로 뽑는다.

시연 영상을 짜깁기하려면 먼저 통짜 영상이 있어야 한다. rosbag 을 그대로
재생하면 ROS 가 떠 있어야 하고 편집도 안 되므로, 파일로 떨궈 둔다.

프레임 간격이 일정하지 않으므로(카메라가 밀리거나 끊긴다) 실제 수신 시각을
보고 목표 fps 에 맞춰 프레임을 고르거나 반복한다. 그냥 이어 붙이면 재생
속도가 실제와 달라져서, 나중에 "몇 초에 무슨 일이 있었나"를 못 맞춘다.

사용:
  python3 tools/bag_to_video.py bags/robot1_map_0806_1846
  python3 tools/bag_to_video.py <bag> --topic /robot1/oakd/rgb/image_raw/compressed
  python3 tools/bag_to_video.py <bag> --fps 15 --out docs/images/demo.mp4

  --start / --end 로 구간만 자를 수 있다(초, 녹화 시작 기준).
"""
import argparse
import os
import sys

import cv2
import numpy as np


def read_frames(bag_path: str, topic: str):
    """(수신시각초, jpeg bytes) 를 순서대로 내놓는다."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    # 압축 여부와 방식은 metadata.yaml 에 적혀 있다. StorageOptions 에
    # 직접 넘길 수 없으므로, 압축본이면 SequentialCompressionReader 를 쓴다.
    import yaml

    with open(os.path.join(bag_path, "metadata.yaml"), encoding="utf-8") as f:
        meta = yaml.safe_load(f)["rosbag2_bagfile_information"]
    compressed = bool(meta.get("compression_format"))

    if compressed:
        reader = rosbag2_py.SequentialCompressionReader()
    else:
        reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    while reader.has_next():
        _, data, stamp_ns = reader.read_next()
        message = deserialize_message(data, CompressedImage)
        yield stamp_ns * 1e-9, bytes(message.data)


def main() -> int:
    parser = argparse.ArgumentParser(description="rosbag 영상 -> mp4")
    parser.add_argument("bag")
    parser.add_argument(
        "--topic", default="/robot1/oakd/rgb/image_raw/compressed"
    )
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--out", default=None)
    parser.add_argument("--start", type=float, default=0.0, help="시작 초")
    parser.add_argument("--end", type=float, default=None, help="끝 초")
    args = parser.parse_args()

    if not os.path.isdir(args.bag):
        print(f"실패: {args.bag} 폴더가 없습니다")
        return 1
    out_path = args.out or (args.bag.rstrip("/") + ".mp4")

    print(f"bag   : {args.bag}")
    print(f"topic : {args.topic}")

    frames = []
    first = None
    for stamp, jpeg in read_frames(args.bag, args.topic):
        if first is None:
            first = stamp
        offset = stamp - first
        if offset < args.start:
            continue
        if args.end is not None and offset > args.end:
            break
        frames.append((offset, jpeg))

    if not frames:
        print("실패: 해당 토픽의 프레임이 없습니다")
        return 1

    span = frames[-1][0] - frames[0][0]
    print(f"프레임: {len(frames)}개, {span:.1f}초 "
          f"(평균 {len(frames)/max(span,0.001):.1f} fps)")

    sample = cv2.imdecode(np.frombuffer(frames[0][1], np.uint8),
                          cv2.IMREAD_COLOR)
    if sample is None:
        print("실패: 첫 프레임을 디코드하지 못했습니다")
        return 1
    height, width = sample.shape[:2]

    writer = cv2.VideoWriter(
        out_path, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (width, height)
    )
    if not writer.isOpened():
        print(f"실패: {out_path} 를 열지 못했습니다")
        return 1

    # 목표 fps 의 격자에 맞춰, 그 시점에 가장 가까운 프레임을 쓴다.
    # 카메라가 잠깐 끊긴 구간에서는 같은 프레임이 반복되지만, 그 편이
    # 시간축이 어긋나는 것보다 낫다.
    base = frames[0][0]
    total = frames[-1][0] - base
    index = 0
    written = 0
    step = 1.0 / args.fps
    target = 0.0
    while target <= total:
        while index + 1 < len(frames) and (frames[index + 1][0] - base) <= target:
            index += 1
        image = cv2.imdecode(
            np.frombuffer(frames[index][1], np.uint8), cv2.IMREAD_COLOR
        )
        if image is not None:
            if image.shape[:2] != (height, width):
                image = cv2.resize(image, (width, height))
            writer.write(image)
            written += 1
        target += step

    writer.release()
    size_mb = os.path.getsize(out_path) / 1e6
    print(f"저장  : {out_path}")
    print(f"        {written}프레임 @ {args.fps:.0f}fps "
          f"= {written/args.fps:.1f}초, {size_mb:.1f}MB, {width}x{height}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
