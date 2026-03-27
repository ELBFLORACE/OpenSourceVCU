cmake_minimum_required(VERSION 3.16.3)
project(mailman)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ix_msgs REQUIRED)
find_package(vcu_shared_lib REQUIRED)
find_package(vcu_can_msgs_yourcar17 REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(MAILMAN
  include/CANReader.hpp
  include/MailmanData.hpp
  include/motor_driver/CANInterface.hpp
  include/motor_driver/MotorDriver.hpp
  src/CANReader.cpp
  src/generated/Can1Decoding.cpp
  src/generated/Can1Packing.cpp
  src/generated/Can2Decoding.cpp
  src/generated/Can2Packing.cpp
  src/generated/Can3Decoding.cpp
  src/generated/Can3Packing.cpp
  src/generated/Can4Decoding.cpp
  src/generated/Can4Packing.cpp
  src/generated/FsgDecoding.cpp
  src/generated/FsgPacking.cpp
  src/motor_driver/CANInterface.cpp
  src/motor_driver/MotorDriver.cpp

)

include_directories(include)
add_executable(mailman src/mailman.cpp ${MAILMAN})

ament_target_dependencies(mailman
rclcpp
std_msgs
geometry_msgs
std_srvs
ix_msgs
vcu_can_msgs_yourcar17
vcu_shared_lib
vcu_msgs
)

install(TARGETS mailman
DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
