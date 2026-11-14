cmake_minimum_required(VERSION 3.8)
project(rover_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LEDState.msg"
  "msg/LEDStateArray.msg"
  "msg/LedStripState.msg"
  "msg/OctolinerReading.msg"
  "msg/WheelCommand.msg"
  "msg/WheelEncoders.msg"
  "srv/GetFrame.srv"
  "srv/SetLEDEffect.srv"
  "srv/SetLEDs.srv"
  "srv/SetLedStripState.srv"
  "srv/SetSensitivity.srv"
  "srv/SpeakText.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
