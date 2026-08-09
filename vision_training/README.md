# 구조 상황 YOLO 학습

이 폴더는 기존 `yolo_training`과 연결되지 않는 독립 학습 공간입니다.

Python 파일은 역할별로 정리되어 있습니다.

```text
training/  모델 학습과 파인튜닝
testing/   이미지·웹캠·TurtleBot 추론 테스트
tools/     촬영, 데이터 준비, Roboflow 다운로드
unit_tests/ 단위 테스트
scripts/   학습 지표와 그래프 내부 유틸리티
notebooks/ Google Colab GPU 학습 노트북
```

## Google Colab GPU 학습

`training/caterpillar_fall_pose_colab.ipynb`를 Google Drive에 업로드하거나
Colab에서 직접 연 뒤 위 셀부터 순서대로 실행합니다.

```text
런타임 → 런타임 유형 변경 → T4 GPU
```

노트북에는 Roboflow 데이터 다운로드, 12관절 Pose 라벨 검사, YOLO11n Pose
학습, test split 검증, 이미지·영상 업로드 추론, `best.pt`와 전체 결과 ZIP
다운로드 과정이 포함되어 있습니다. API 키는 코드에 적지 않고 Colab
Secrets의 `ROBOFLOW_API_KEY` 또는 숨김 입력을 사용합니다.

클래스 순서는 다음과 같습니다.

```text
0: fallen_person  # 쓰러진 사람
1: helper_rc_car  # 도와주는 사람 역할의 빨간 RC카
```

## 촬영

모든 원본 사진은 `vision_training/captures/raw`에 함께 저장됩니다.

```bash
cd /home/rokey/rokey_ws/multi_amr_aed
python3 vision_training/tools/webcam_capture.py --camera 2
```

- 촬영 창에서 마우스 왼쪽 클릭: 현재 프레임 저장
- `Space`: 현재 프레임 저장
- `Q` 또는 `Esc`: 종료

사진에 두 클래스가 함께 있어도 됩니다. 객체 탐지 학습에는 사진 속 객체마다
바운딩 박스 라벨이 필요합니다. 쓰러지지 않은 사람은 `fallen_person`으로
라벨링하지 말고, 빨간 RC카 전체를 `helper_rc_car`로 라벨링하세요. 폴더명이
아닌 바운딩 박스의 클래스 번호로 두 객체를 구분합니다.

## 데이터 배치

Roboflow 등의 라벨링 도구에서 YOLO Detection 형식으로 내보내 아래와 같이
놓습니다. `captures`, `datasets`, `runs`는 `.gitignore` 대상이라 기존
프로젝트 데이터와 섞이지 않습니다.

```text
vision_training/datasets/rescue/
├── images/{train,val,test}/
└── labels/{train,val,test}/
```

라벨 파일의 클래스 번호도 반드시 `0=fallen_person`, `1=helper_rc_car`여야
합니다.

기존 `yolo_training/datasets/car_dummy`의 빨간 RC카 데이터만 가져오려면
다음 명령을 실행합니다. 기존 `dummy` 데이터는 제외되고 RC카 라벨 번호는
자동으로 `0`에서 `1`로 변환됩니다.

```bash
python3 vision_training/import_legacy_rc_car.py
```

현재 라벨링 COCO 데이터와 기존 RC카 데이터를 한 번에 다시 합치려면 다음
명령을 사용합니다. 연속 프레임은 10초 촬영 묶음 단위로 train/val/test에
분리됩니다.

```bash
python3 vision_training/prepare_combined_dataset.py
```

## 학습

```bash
python3 -m pip install -r vision_training/requirements.txt
python3 vision_training/training/yolo_train.py --device 0
```

기본적으로 기존 코드에서 가져온 로컬 가중치로 YOLOv8, YOLO11, YOLO26의
n/s 모델 6개를 각각 학습·검증합니다. 가중치는 `vision_training/models`에
있으므로 별도 다운로드가 필요하지 않습니다. 한 모델만 학습하려면 다음처럼
실행합니다.

```bash
python3 vision_training/training/yolo_train.py \
  --models vision_training/models/yolo11n.pt \
  --device 0
```

결과는 `vision_training/runs`에 저장됩니다. 각 모델의 학습·검증 플롯과
mAP50, mAP50-95, 정밀도, 재현율, F1, 추론속도, 평균 confidence가 CSV와
JSON으로 저장되며 6개 모델 비교 그래프도 자동 생성됩니다.

## 실제 사람 Pose 학습

`models/yolo11n-pose.pt`는 COCO 실제 사람의 17개 관절로 사전학습된
YOLO11n Pose 모델입니다. 기존 `data.yaml`의 5필드 바운딩 박스 라벨은 Pose
학습에 사용할 수 없습니다. Pose 데이터는 객체마다 다음 56개 필드가
필요합니다.

```text
class cx cy width height (keypoint_x keypoint_y visibility) × 17
```

실제 사람의 서기·앉기·숙이기·눕기 장면을 COCO Keypoints 형식으로
라벨링하고 `pose_data.example.yaml`을 복사해 경로를 맞춘 뒤 학습합니다.

```bash
cp vision_training/pose_data.example.yaml vision_training/pose_data.yaml
python3 vision_training/training/yolo_train_pose.py \
  --data vision_training/pose_data.yaml \
  --device 0 --epochs 100 --batch 8
```

RTX 3060 6GB에서 CUDA 메모리가 부족하면 `--batch 4` 또는 `--batch 2`로
낮춥니다. 결과 가중치는 `vision_training/runs/pose/<실행명>/weights/best.pt`에
저장됩니다. 실제 사람 일반화를 유지하려면 인형 프레임만으로 Pose 모델을
파인튜닝하지 말고 다양한 체형·옷·방향·조명의 실제 사람 데이터를 포함해야
합니다.

### 라벨링 전 Pose 테스트

웹캠의 실제 사람을 먼저 검출하고 bbox 내부에서 17개 관절과
`STANDING`, `SITTING`, `FALLEN`, `UNKNOWN` 자세 판정을 확인합니다.

```bash
python3 vision_training/testing/detect_then_pose_webcam_test.py \
  --source 0 --target person --device auto
```

목각인형용 검출 모델을 시험하려면 다음처럼 실행합니다.

```bash
python3 vision_training/testing/detect_then_pose_webcam_test.py \
  --source 0 --target mannequin --device auto
```

카메라 번호는 USB 웹캠 장치에 맞춰 `--source 0` 또는 `--source 2`로
변경합니다. 창에서 `Q` 또는 `Esc`를 누르면 종료합니다.

이 테스트에서 실제 사람의 자세별 오판 사례를 모은 다음에만 관절 라벨을
검수하고 Pose 파인튜닝 여부를 결정합니다.

### Roboflow 낙상 Pose 데이터 다운로드

Caterpillar의 공개 `Fall/Non-Fall` Keypoint Detection v3 데이터셋을 받으려면
다음 명령을 실행합니다. API 키는 화면과 셸 히스토리에 표시되지 않도록 숨김
입력으로 받습니다.

```bash
python3 vision_training/tools/download_roboflow_pose.py
```

자동 실행 환경에서는 API 키를 파일에 저장하지 말고 해당 프로세스에만
환경변수로 전달할 수 있습니다.

```bash
read -rsp "Roboflow API key: " ROBOFLOW_API_KEY
export ROBOFLOW_API_KEY
python3 vision_training/tools/download_roboflow_pose.py
unset ROBOFLOW_API_KEY
```

다운로드 후 `kpt_shape`, 클래스, 라벨 필드 수를 자동 검사하고 통과한 경우에만
`datasets/caterpillar_fall_pose_v3`에 저장합니다. 마지막에 실제 `data.yaml`을
사용한 학습 명령도 출력합니다.

다운로드한 Caterpillar 데이터의 12관절 `Fall/Non-Fall` 시험 모델은 다음처럼
웹캠에서 확인합니다. 본 학습 모델이 있으면 이를 우선 사용하고, 없으면
1 epoch smoke 모델을 자동으로 선택합니다.

```bash
python3 vision_training/testing/fall_pose_model_test.py \
  --source 2 --device 0
```

이미지 파일은 결과를 `runs/pose/fall_pose_test`에 저장하고 종료합니다.

```bash
python3 vision_training/testing/fall_pose_model_test.py \
  --source /path/to/person.jpg --device 0 --no-show
```

## 어려운 음성 데이터 파인튜닝

1차 학습 후 서기·앉기·숙이기·빈 바닥 사진을 추가합니다. 해당 사진에
`fallen_person`이나 RC카가 없다면 같은 이름의 빈 `.txt` 라벨 파일을
만듭니다. 기존 양성 데이터는 제거하지 않고 함께 학습해야 합니다.

사진을 다음 폴더에 넣습니다.

```text
vision_training/hard_negatives/images/
```

빈 라벨 생성과 기존 데이터셋 추가를 자동으로 실행합니다.

```bash
python3 vision_training/tools/prepare_hard_negatives.py
```

빈 라벨은 `vision_training/hard_negatives/labels`에도 보관되고, 사진은 촬영
시간 10초 묶음을 유지하며 train/val/test로 나뉩니다.

`vision_training/runs` 아래 1차 학습 모델 6종의 최신 `best.pt`를 모두
순차 파인튜닝:

```bash
python3 vision_training/training/yolo_finetune.py --device 0
```

일부 가중치만 직접 지정하려면:

```bash
python3 vision_training/training/yolo_finetune.py \
  --weights \
    vision_training/runs/yolo11n_<시간>/weights/best.pt \
    vision_training/runs/yolo11s_<시간>/weights/best.pt \
  --device 0
```

기본값은 25 epoch, 초기 학습률 `0.0001`이며 결과는
`vision_training/finetune_runs`에 저장됩니다. 파인튜닝 완료 후 `test`
분할로 최종 평가합니다.

## 학습 모델 추적 테스트

같은 웹캠 프레임을 최신 파인튜닝 구조 모델과 COCO 사전학습 YOLO11n에
동시에 입력해 두 개 창에서 비교합니다. 구조 창은 `fallen_person`과
`helper_rc_car`, COCO 창은 실제 `person`만 표시합니다.

```bash
python3 vision_training/testing/model_test.py --source 2 --device 0
```

- `Q` 또는 `Esc`: 종료
- `S`: 두 창의 바운딩 박스 화면을 각각 저장

저장 화면은 `vision_training/test_captures`에 들어갑니다.

최신 1차 학습 모델을 카메라 2에서 ByteTrack으로 테스트합니다.

```bash
python3 vision_training/testing/yolo_track_test.py --source 2 --device 0
```

특정 모델이나 BoT-SORT를 사용하려면:

```bash
python3 vision_training/testing/yolo_track_test.py \
  --weights vision_training/runs/<실행폴더>/weights/best.pt \
  --source 2 \
  --tracker botsort \
  --device 0
```

추적 결과 영상은 `vision_training/runs/tracking`에 저장됩니다.
