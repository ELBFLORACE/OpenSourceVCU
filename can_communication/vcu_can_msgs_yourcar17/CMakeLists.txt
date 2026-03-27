cmake_minimum_required(VERSION 3.8)
project(vcu_can_msgs_yourcar17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

file(GLOB ROS_MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)

rosidl_generate_interfaces(
   ${PROJECT_NAME}

   ${ROS_MSG_FILES}

   DEPENDENCIES
   geometry_msgs
   visualization_msgs
   std_msgs
   sensor_msgs
)
ament_export_dependencies(std_msgs geometry_msgs rosidl_default_runtime visualization_msgs)
ament_export_include_directories(
  include
)

ament_package()
