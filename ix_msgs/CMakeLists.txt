cmake_minimum_required(VERSION 3.8)
project(ix_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(generic_msgs
  "msg/generic/Int8Stamped.msg"
  "msg/generic/Int8StampedArray.msg"
  "msg/generic/Int32Stamped.msg"
  "msg/generic/Int32StampedArray.msg"
  "msg/generic/Float32Stamped.msg"
  "msg/generic/Float32StampedArray.msg"
  )

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${generic_msgs}
  "msg/Wheels.msg"
  "msg/InverterSetpoints.msg"
  "msg/Pedals.msg"
  "msg/GnssData.msg"
  "msg/Inverter.msg"
  "msg/Energymeter.msg"
  "msg/Mission.msg"
  "msg/LapCounter.msg"
  "msg/Accumulator.msg"
  "msg/DvObserver.msg"

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
