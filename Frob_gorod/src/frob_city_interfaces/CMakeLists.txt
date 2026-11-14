cmake_minimum_required(VERSION 3.8)
project(frob_city_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
  "msg/SignDetection.msg"
  "msg/SignDetectionArray.msg"
  "msg/ApproachState.msg"
  "msg/KnownSignState.msg"
  "msg/TargetState.msg"
  "msg/EdgeState.msg"
  "msg/CoverageState.msg"
  "msg/MapState.msg"
  "msg/RouteSegment.msg"
  "msg/MotionPermit.msg"
  "msg/GuardState.msg"
  "msg/MissionState.msg"
)

set(srv_files
  "srv/PlanRoute.srv"
  "srv/ValidateSegment.srv"
  "srv/ReportExecution.srv"
  "srv/ResetMap.srv"
  "srv/ControlMission.srv"
)

set(action_files
  "action/Observe.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(interface_manifest test/test_interface_manifest.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
