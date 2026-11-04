cmake_minimum_required(VERSION 3.8)
project(construction_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LayoutPoint.msg" "msg/MarkResult.msg" "msg/MissionStatus.msg"
  "srv/LoadLayout.srv" "srv/TriggerMark.srv" "srv/ResetMission.srv"
  "action/ExecuteMission.action"
  DEPENDENCIES geometry_msgs builtin_interfaces)
ament_package()
