cmake_minimum_required(VERSION 3.16.3)
project(vcu_analytics)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ix_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(vcu_can_msgs_yourcar17 REQUIRED)
find_package(vcu_shared_lib REQUIRED)

set(vcu
	include/Analytics_Data.hpp
	src/vcu_analytics.cpp
)

include_directories(include)

add_executable(vcu_analytics src/vcu_analytics.cpp)

ament_target_dependencies(vcu_analytics
    rclcpp
ament_index_cpp
    std_msgs
    geometry_msgs
    ix_msgs
    vcu_msgs
    std_srvs
    vcu_can_msgs_yourcar17
    vcu_shared_lib
)

install(TARGETS vcu_analytics
	DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
