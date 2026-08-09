# AED 비전 모델 학습

구조 상황에서 `fallen_person`과 `helper_rc_car`를 검출하고, 사람의 자세와
낙상 상태를 시험하기 위한 데이터 준비·YOLO 학습·평가 도구입니다.

이 디렉터리는 **모델 개발용**입니다. 학습이 끝난 가중치로 ROS 추론 노드를
실행하는 방법은 프로젝트 루트 문서를 참고하세요.

## 구성

```text
vision_training/
├── data.yaml                  # Detection 데이터셋 설정
├── pose_data.example.yaml     # Pose 데이터셋 설정 예시
├── requirements.txt           # 학습·로컬 테스트 의존성
├── models/                    # 사전학습 가중치
├── training/                  # Detection, Pose 학습과 파인튜닝
├── testing/                   # 웹캠·영상·TurtleBot 추론 테스트
├── tools/                     # 촬영과 데이터 준비
├── scripts/                   # 지표 수집과 비교 그래프
└── runs/, finetune_runs/      # 실행 시 생성되는 학습 결과
```

## 환경 준비

저장소 루트에서 실행합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r vision_training/requirements.txt
```

GPU 학습 전에는 PyTorch가 CUDA를 인식하는지 확인하세요.

```bash
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

## Detection 데이터

클래스 번호는 모든 이미지와 라벨에서 다음 순서를 유지해야 합니다.

```text
0: fallen_person
1: helper_rc_car
```

기본 데이터셋 구조는 다음과 같습니다.

```text
vision_training/datasets/rescue/
├── images/
│   ├── train/
│   ├── val/
│   └── test/
└── labels/
    ├── train/
    ├── val/
    └── test/
```

각 라벨은 YOLO Detection 형식입니다.

```text
class_id center_x center_y width height
```

경로와 클래스 정의는 [`data.yaml`](data.yaml)에 있습니다. 다른 데이터셋을
사용할 때는 같은 형식의 YAML 파일을 만들고 학습 명령에 `--data`로
지정하세요.

### 이미지 촬영

```bash
python3 vision_training/tools/webcam_capture.py --camera 2
```

기본 저장 위치는 `vision_training/captures/raw`입니다. 촬영 창에서 왼쪽
클릭이나 `Space`로 저장하고, `Q` 또는 `Esc`로 종료합니다.

### Hard negative 추가

서 있는 사람, 앉은 사람, 빈 바닥처럼 오검출하기 쉬운 음성 이미지를 다음
위치에 넣습니다.

```text
vision_training/hard_negatives/images/
```

빈 라벨을 만들고 기본 데이터셋에 분할하여 추가합니다.

```bash
python3 vision_training/tools/prepare_hard_negatives.py
```

이 작업은 `datasets/rescue`를 수정하므로 원본 데이터 백업 여부를 먼저
확인하세요.

## Detection 학습

기본 명령은 로컬의 YOLOv8, YOLO11, YOLO26 `n/s` 가중치 6개를 동일한
조건으로 학습하고 비교합니다.

```bash
python3 vision_training/training/yolo_train.py --device 0
```

한 모델만 빠르게 확인하려면:

```bash
python3 vision_training/training/yolo_train.py \
  --models vision_training/models/yolo11n.pt \
  --epochs 1 \
  --device 0
```

주요 기본값은 `epochs=100`, `imgsz=640`, `batch=8`이며 결과는
`vision_training/runs`에 저장됩니다. 전체 옵션은 다음 명령으로 확인합니다.

```bash
python3 vision_training/training/yolo_train.py --help
```

CUDA 메모리가 부족하면 먼저 `--batch 4` 또는 `--batch 2`를 사용하세요.
학습률과 증강 설정의 배경은 [`HYPERPARAMETERS.md`](HYPERPARAMETERS.md)를
참고하세요.

## Detection 파인튜닝

Hard negative를 추가한 뒤 1차 학습 결과를 낮은 학습률로 파인튜닝합니다.
인자를 생략하면 `runs`에서 6개 모델별 최신 `best.pt`를 찾습니다.

```bash
python3 vision_training/training/yolo_finetune.py --device 0
```

특정 가중치만 사용하려면:

```bash
python3 vision_training/training/yolo_finetune.py \
  --weights vision_training/runs/<run>/weights/best.pt \
  --device 0
```

기본값은 `epochs=25`, `lr0=0.0001`이고 결과는
`vision_training/finetune_runs`에 저장됩니다. 학습 후 `test` split 평가도
자동으로 실행됩니다.

## Pose 학습

Pose 학습에는 Detection 라벨을 사용할 수 없습니다. COCO 17관절 기준으로
객체 한 개의 라벨은 다음 형식이어야 합니다.

```text
class cx cy width height (keypoint_x keypoint_y visibility) x 17
```

설정 예시를 복사하고 실제 데이터셋 경로를 수정합니다.

```bash
cp vision_training/pose_data.example.yaml vision_training/pose_data.yaml
python3 vision_training/training/yolo_train_pose.py \
  --data vision_training/pose_data.yaml \
  --device 0
```

스크립트는 학습 전에 `kpt_shape`, `flip_idx`, train/val 경로와 라벨 필드
수를 검사합니다. 결과는 `vision_training/runs/pose`에 저장됩니다.

Colab에서 OmniFall 기반 Pose+LSTM 모델을 학습하려면
`training/omnifall_pose_colab.ipynb`를 사용합니다. API 키나 개인 데이터 경로는
노트북에 커밋하지 마세요.

## 모델 테스트

### 파인튜닝 모델과 COCO person 모델 비교

```bash
python3 vision_training/testing/model_test.py --source 2 --device 0
```

기본적으로 최신 YOLO11n 파인튜닝 가중치를 찾습니다. `Q` 또는 `Esc`로
종료하고, `S`로 현재 화면을 저장합니다.

### Detection 후 Pose 자세 판정

실제 사람:

```bash
python3 vision_training/testing/detect_then_pose_webcam_test.py \
  --source 0 --target person --device auto
```

목각인형 검출 모델:

```bash
python3 vision_training/testing/detect_then_pose_webcam_test.py \
  --source 0 --target mannequin --device auto
```

### Pose+LSTM 낙상 분류

```bash
python3 vision_training/testing/omnifall_pose_lstm_webcam_test.py \
  --source 0 --device 0
```

### TurtleBot OAK-D 영상 테스트

ROS 2 환경을 먼저 source한 뒤 실행합니다.

```bash
python3 vision_training/testing/turtlebot_yolo_test.py \
  --topic /robot2/oakd/rgb/image_raw/compressed \
  --device 0
```

각 테스트의 가중치·confidence·입력 소스 옵션은 `--help`로 확인할 수
있습니다.

## 결과물 관리

학습 결과, 촬영 이미지, 데이터셋은 용량이 크므로 Git에 넣지 않는 것을
원칙으로 합니다. 배포할 모델을 선택할 때는 최소한 별도 test split의
precision, recall, mAP50-95와 실제 카메라 오검출을 함께 확인하세요.

재현 가능한 실험을 위해 실행에 사용한 다음 정보를 남겨두는 것을 권장합니다.

- 데이터셋 버전과 클래스 순서
- 초기 가중치와 최종 `best.pt`
- 실행 명령과 주요 하이퍼파라미터
- Ultralytics 및 PyTorch 버전
- 검증·테스트 지표
