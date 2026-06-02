cmake_minimum_required(VERSION 3.15)
project(gh_twin_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Force CMake to use Pixi's environment Python exclusively
set(Python3_FIND_VIRTUALENV ONLY)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Flower.msg"
  "msg/Pest.msg"
  "msg/Sensor.msg"
  "msg/SensorData.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()