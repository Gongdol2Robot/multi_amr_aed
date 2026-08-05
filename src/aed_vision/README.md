# aed_vision

응급상황 비전 파이프라인의 공통 입력과 좌표 변환 기능입니다.

담당: 김지훈(호모그래피·위치 검증), 이현민(목각인형·사람 검출 및 통합)

## Nodes

- `webcam_publisher`: 로컬 웹캠을 JPEG 압축 ROS 2 이미지 토픽으로 발행
- `emergency_detector` 예정: 쓰러진 목각인형 연속 검출과 이벤트 생성
- `helper_presence_detector` 예정: AED 도착 후 주변 도움 인력 존재 여부 판별

## Library

- `aed_vision.homography.Homography`: 검출 박스의 바닥 접점을 map 좌표로 변환

`config/homography.example.yaml`은 형식 예제입니다. 실제 설치 장소에서 측량한
픽셀-map 대응점을 사용해 행렬을 다시 계산해야 합니다.
