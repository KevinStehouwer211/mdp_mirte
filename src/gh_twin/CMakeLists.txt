cmake_minimum_required(VERSION 3.8)
project(gh_twin)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mirte_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
#find_package(action_msgs REQUIRED)
find_package(mirte_gazebo REQUIRED)
#find_package(rosidl_default_generators REQUIRED)


#ament_target_dependencies(rclcpp std_msgs sensor_msgs mirte_msgs std_srvs)



foreach(X IN ITEMS "config" "launch" "models" "worlds" "rviz" "maps" "action")
    message(STATUS "X=${X}")
    install(DIRECTORY ${X}
      DESTINATION share/${PROJECT_NAME}/)
endforeach()

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "action/Nav.action"
#  DEPENDENCIES geometry_msgs std_msgs
#)

ament_python_install_package(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
