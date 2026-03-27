cmake_minimum_required(VERSION 3.16.3)
project(vcu)

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
find_package(vcu_can_msgs_yourcar17 REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(vcu_shared_lib REQUIRED)


set(vcu
	include/VCU_Data.hpp
	src/vcu.cpp
)

include_directories(include)

if(BUILD_TESTING)
  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_integration.py)
endif()

add_executable(vcu src/vcu.cpp)

ament_target_dependencies(vcu
    rclcpp
    std_msgs
    geometry_msgs
    ix_msgs
    vcu_msgs
    vcu_can_msgs_yourcar17
    std_srvs
    vcu_shared_lib
)


install(TARGETS vcu
	DESTINATION lib/${PROJECT_NAME})


ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
