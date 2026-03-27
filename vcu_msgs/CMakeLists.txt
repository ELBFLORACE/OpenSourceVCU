cmake_minimum_required(VERSION 3.8)
project(vcu_msgs)

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
find_package(ix_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${generic_msgs}
  "msg/AccelBrakeSteer.msg"
  "msg/DISOutput.msg"
  "msg/InverterOutput.msg"
  "msg/RESInput.msg"
  "msg/VehicleState.msg"
  "msg/Watchdog.msg"
  "msg/SteeringActuator.msg"
  "msg/DriverInput.msg"
  "msg/ASSM.msg"
  "msg/SSBRear.msg"
  "msg/PDUOutput.msg"
  "msg/AccumulatorOutput.msg"
  "msg/ASSMOutput.msg"
  "msg/VehicleStateDebug.msg"
  "msg/CoolingRequest.msg"
  "msg/VehicleStateReadable.msg"
  "msg/DataLogger.msg"
  "msg/Temps.msg"
  "msg/WatchdogDebug.msg"
  "msg/HardwareAnalytics.msg"
  "msg/Analytics.msg"
   DEPENDENCIES
   geometry_msgs
   ix_msgs
   visualization_msgs
   std_msgs
   sensor_msgs

)


ament_export_dependencies(std_msgs geometry_msgs rosidl_default_runtime visualization_msgs)
ament_export_include_directories(
  include
)

ament_package()
