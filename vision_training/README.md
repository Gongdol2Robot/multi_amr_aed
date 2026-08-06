# 구조 상황 YOLO 학습

이 폴더는 기존 `yolo_training`과 연결되지 않는 독립 학습 공간입니다.

클래스 순서는 다음과 같습니다.

```text
0: fallen_person  # 쓰러진 사람
1: helper_rc_car  # 도와주는 사람 역할의 빨간 RC카
```

## 촬영

모든 원본 사진은 `vision_training/captures/raw`에 함께 저장됩니다.

```bash
cd /home/rokey/rokey_ws/multi_amr_aed
python3 vision_training/webcam_capture.py --camera 2
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
python3 vision_training/yolo_train.py --device 0
```

기본적으로 기존 코드에서 가져온 로컬 가중치로 YOLOv8, YOLO11, YOLO26의
n/s 모델 6개를 각각 학습·검증합니다. 가중치는 `vision_training/models`에
있으므로 별도 다운로드가 필요하지 않습니다. 한 모델만 학습하려면 다음처럼
실행합니다.

```bash
python3 vision_training/yolo_train.py \
  --models vision_training/models/yolo11n.pt \
  --device 0
```

결과는 `vision_training/runs`에 저장됩니다. 각 모델의 학습·검증 플롯과
mAP50, mAP50-95, 정밀도, 재현율, F1, 추론속도, 평균 confidence가 CSV와
JSON으로 저장되며 6개 모델 비교 그래프도 자동 생성됩니다.

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
python3 vision_training/prepare_hard_negatives.py
```

빈 라벨은 `vision_training/hard_negatives/labels`에도 보관되고, 사진은 촬영
시간 10초 묶음을 유지하며 train/val/test로 나뉩니다.

`vision_training/runs` 아래 1차 학습 모델 6종의 최신 `best.pt`를 모두
순차 파인튜닝:

```bash
python3 vision_training/yolo_finetune.py --device 0
```

일부 가중치만 직접 지정하려면:

```bash
python3 vision_training/yolo_finetune.py \
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
python3 vision_training/model_test.py --source 2 --device 0
```

- `Q` 또는 `Esc`: 종료
- `S`: 두 창의 바운딩 박스 화면을 각각 저장

저장 화면은 `vision_training/test_captures`에 들어갑니다.

최신 1차 학습 모델을 카메라 2에서 ByteTrack으로 테스트합니다.

```bash
python3 vision_training/yolo_track_test.py --source 2 --device 0
```

특정 모델이나 BoT-SORT를 사용하려면:

```bash
python3 vision_training/yolo_track_test.py \
  --weights vision_training/runs/<실행폴더>/weights/best.pt \
  --source 2 \
  --tracker botsort \
  --device 0
```

추적 결과 영상은 `vision_training/runs/tracking`에 저장됩니다.
