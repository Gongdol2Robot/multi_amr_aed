cmake_minimum_required(VERSION 3.8)
project(aed_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GuideHelper.action"
  "msg/EmergencyEvent.msg"
  "msg/HelperPresence.msg"
  "msg/RobotState.msg"
  "msg/MissionAssignment.msg"
  "msg/MissionStatus.msg"
  "msg/Heartbeat.msg"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
