cmake_minimum_required(VERSION 3.8)
project(krsbi_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPositionBasedOnCamera.msg"
  "msg/BoundingBox.msg"
  "msg/ControlOutput.msg"
  "msg/Coordinate.msg"
  "msg/DribbleModule.msg"
  "msg/DribbleSetting.msg"
  "msg/Encoder.msg"
  "msg/KickerModule.msg"
  "msg/KickerSetting.msg"
  "msg/NavigationSetting.msg"
  "msg/ObjectDetection.msg"
  "msg/RobotMode.msg"
  "msg/RobotStatus.msg"
  "msg/SetPoint.msg"
  "msg/Wheel.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
