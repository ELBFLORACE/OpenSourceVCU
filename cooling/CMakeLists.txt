cmake_minimum_required(VERSION 3.16.3)
project(cooling)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ix_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)


set(cooling   
	include/CoolingNode.hpp
	src/cooling.cpp
)

include_directories(include)


add_executable(cooling_vcu src/cooling.cpp)

ament_target_dependencies(cooling_vcu
rclcpp
std_msgs
geometry_msgs
ix_msgs
vcu_msgs
std_srvs
)

install(TARGETS cooling_vcu
	DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
