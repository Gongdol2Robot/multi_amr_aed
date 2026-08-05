# emergency_alert

TurtleBot4 스피커로 긴급 알림음을 재생합니다. 토픽이 파라미터이므로 로봇별
namespace에 맞게 실행할 수 있습니다.

```bash
ros2 run emergency_alert siren --ros-args \
  -p audio_topic:=/robot1/cmd_audio
```

