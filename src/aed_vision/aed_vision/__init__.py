"""AED 카메라 영상에서 낙상·조력자·골목 혼잡도를 판정하는 ROS 2 패키지.

처음 코드를 읽을 때는 다음 순서로 보면 된다.

1. ``vision_detector``: ROS 노드의 시작점. 영상 수신과 결과 발행을 담당한다.
2. ``camera_source`` / ``qos``: 영상이 들어오는 방법과 통신 정책을 담당한다.
3. ``inference_pipeline``: 모델을 실행하고 한 프레임의 검출 결과를 조립한다.
4. ``pose_posture`` / ``posture_classifier``: 자세를 FALLEN 등으로 분류한다.
5. ``detection_logic``: IoU, 시간 확정, 혼잡도 같은 순수 후처리를 담당한다.
6. ``homography``: 검출된 영상 좌표를 ROS map 좌표로 변환한다.

핵심 원칙은 모델 추론과 ROS 통신을 분리하는 것이다. 그래서 계산 규칙은 가능한
한 ``detection_logic``에 두어 카메라나 GPU 없이도 단위 테스트할 수 있게 한다.
"""
