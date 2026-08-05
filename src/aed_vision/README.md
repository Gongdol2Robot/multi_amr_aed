# aed_vision

응급상황 비전 파이프라인의 공통 입력과 좌표 변환 기능입니다.

## Nodes

- `webcam_publisher`: 로컬 웹캠을 JPEG 압축 ROS 2 이미지 토픽으로 발행

## Library

- `aed_vision.homography.Homography`: 검출 박스의 바닥 접점을 map 좌표로 변환

`config/homography.example.yaml`은 형식 예제입니다. 실제 설치 장소에서 측량한
픽셀-map 대응점을 사용해 행렬을 다시 계산해야 합니다.

