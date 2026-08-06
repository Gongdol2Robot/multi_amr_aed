#!/usr/bin/env python3
"""Roboflow의 Caterpillar 낙상 Pose 데이터셋을 안전하게 다운로드한다."""

from __future__ import annotations

import argparse
import getpass
import json
import os
import shutil
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "datasets" / "caterpillar_fall_pose_v3"
API_ENDPOINT_TEMPLATE = (
    "https://api.roboflow.com/caterpillar-dlcux/"
    "fall-detection-apsso/3/{format_name}"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--api-key-env",
        default="ROBOFLOW_API_KEY",
        help="API 키를 읽을 환경변수 이름",
    )
    return parser.parse_args()


def read_api_key(environment_name: str) -> str:
    key = os.environ.get(environment_name, "").strip()
    if not key:
        key = getpass.getpass("Roboflow API key (입력 내용은 표시되지 않음): ").strip()
    if not key:
        raise SystemExit("API 키가 비어 있습니다.")
    return key


def _error_message(payload: dict, api_key: str) -> str | None:
    for field in ("error", "message", "detail"):
        value = payload.get(field)
        if value:
            text = value if isinstance(value, str) else json.dumps(value)
            return text.replace(api_key, "***")[:500]
    return None


def request_export_link(api_key: str, format_name: str) -> str:
    endpoint = API_ENDPOINT_TEMPLATE.format(format_name=format_name)
    url = f"{endpoint}?{urllib.parse.urlencode({'api_key': api_key})}"
    for attempt in range(1, 61):
        try:
            with urllib.request.urlopen(url, timeout=30) as response:
                payload = json.load(response)
        except urllib.error.HTTPError as error:
            if error.code in (401, 403):
                raise SystemExit("Roboflow API 키가 유효하지 않거나 접근 권한이 없습니다.")
            if error.code == 400:
                try:
                    error_payload = json.load(error)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    error_payload = {}
                message = _error_message(error_payload, api_key)
                raise RuntimeError(message or "HTTP 400")
            raise SystemExit(f"Roboflow API 요청 실패: HTTP {error.code}")
        except urllib.error.URLError as error:
            raise SystemExit(f"Roboflow API 연결 실패: {error.reason}")

        message = _error_message(payload, api_key)
        if message:
            raise RuntimeError(message)
        export = payload.get("export") or {}
        link = export.get("link") if isinstance(export, dict) else export
        if link:
            return str(link)
        if attempt == 1:
            print("Roboflow에서 YOLOv11 export를 생성 중입니다...")
        time.sleep(5)
    raise SystemExit("5분 안에 데이터셋 export가 준비되지 않았습니다. 다시 실행하세요.")


def resolve_export_link(api_key: str) -> tuple[str, str]:
    errors = []
    for format_name in ("yolov11", "yolov8"):
        try:
            return request_export_link(api_key, format_name), format_name
        except RuntimeError as error:
            errors.append(f"{format_name}: {error}")
    raise SystemExit("Roboflow export 요청 실패:\n- " + "\n- ".join(errors))


def safe_extract(archive: Path, destination: Path) -> None:
    with zipfile.ZipFile(archive) as zip_file:
        destination_root = destination.resolve()
        for member in zip_file.infolist():
            member_path = (destination / member.filename).resolve()
            if destination_root not in member_path.parents and member_path != destination_root:
                raise SystemExit(f"안전하지 않은 ZIP 경로가 있습니다: {member.filename}")
        zip_file.extractall(destination)


def find_data_yaml(dataset_dir: Path) -> Path:
    candidates = sorted(dataset_dir.rglob("data.yaml"))
    if len(candidates) != 1:
        raise SystemExit(
            f"data.yaml이 정확히 하나여야 합니다. 발견: {len(candidates)}개"
        )
    return candidates[0]


def inspect_dataset(dataset_dir: Path) -> None:
    data_yaml = find_data_yaml(dataset_dir)
    with data_yaml.open(encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}
    names = config.get("names")
    keypoint_shape = config.get("kpt_shape")
    field_counts: dict[int, int] = {}
    objects = 0
    label_files = list(dataset_dir.rglob("labels/*.txt"))
    for label_path in label_files:
        for line in label_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            field_count = len(line.split())
            field_counts[field_count] = field_counts.get(field_count, 0) + 1
            objects += 1

    print(f"data.yaml: {data_yaml}")
    print(f"classes: {names}")
    print(f"kpt_shape: {keypoint_shape}")
    print(f"label files: {len(label_files)}, objects: {objects}")
    print(f"label field counts: {dict(sorted(field_counts.items()))}")
    if not keypoint_shape or not field_counts:
        raise SystemExit("Pose 관절 정보가 없는 데이터셋입니다.")
    expected = 5 + int(keypoint_shape[0]) * int(keypoint_shape[1])
    unexpected = [count for count in field_counts if count != expected]
    if unexpected:
        raise SystemExit(
            f"Pose 라벨은 {expected}필드여야 합니다. 비정상 필드: {unexpected}"
        )
    print("Pose 데이터 구조 검사 통과")


def main() -> int:
    args = parse_args()
    output = args.output.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"출력 폴더가 비어 있지 않습니다: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    api_key = read_api_key(args.api_key_env)
    export_link, format_name = resolve_export_link(api_key)
    print(f"Roboflow export 형식: {format_name}")

    with tempfile.TemporaryDirectory(prefix="roboflow-pose-") as temporary:
        temporary_dir = Path(temporary)
        archive = temporary_dir / "dataset.zip"
        extracted = temporary_dir / "dataset"
        extracted.mkdir()
        print("데이터셋 ZIP 다운로드 중...")
        try:
            urllib.request.urlretrieve(export_link, archive)
        except urllib.error.URLError as error:
            raise SystemExit(f"ZIP 다운로드 실패: {error.reason}")
        safe_extract(archive, extracted)
        inspect_dataset(extracted)
        if output.exists():
            output.rmdir()
        shutil.move(str(extracted), str(output))

    print(f"다운로드 완료: {output}")
    print(
        "학습 명령:\n"
        f"python3 vision_training/training/yolo_train_pose.py "
        f"--data {find_data_yaml(output)} "
        "--device 0 --epochs 100 --batch 8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
