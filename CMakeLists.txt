cmake_minimum_required(VERSION 3.5)
project(mi_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(serial REQUIRED)
find_package(realtime_tools REQUIRED)

set(
  DEPENDENCIES
  "rclcpp"
  "tf2"
  "geometry_msgs"
  "sensor_msgs"
  "std_srvs"
  "serial"
  "realtime_tools"
)

add_executable(
  mi_driver_node 
  src/mi_driver.cpp
  src/parser.cpp
)
target_include_directories(mi_driver_node PRIVATE include)
ament_target_dependencies(mi_driver_node ${DEPENDENCIES})

install(
  DIRECTORY launch rules
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS mi_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_package()