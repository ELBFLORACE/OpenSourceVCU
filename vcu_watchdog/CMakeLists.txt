cmake_minimum_required(VERSION 3.16.3)
project(vcu_watchdog)

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
find_package(vcu_shared_lib REQUIRED)



set(vcu_watchdog
	include/Watchdog_Data.hpp
	src/vcu_watchdog.cpp
)

include_directories(include)

if(BUILD_TESTING)
  # Integration tests
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_integration.py)
endif()


add_executable(vcu_watchdog src/vcu_watchdog.cpp)

ament_target_dependencies(vcu_watchdog
    rclcpp
    std_msgs
    geometry_msgs
    ix_msgs
    vcu_msgs
    std_srvs
    vcu_shared_lib
)

install(TARGETS vcu_watchdog
	DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
