cmake_minimum_required(VERSION 3.8)
project(dummy_ev)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(ix_msgs)
find_package(vcu_msgs)


set(${PEDAL_POSITIONS}
  "${PROJECT_SOURCE_DIR}/include/pedal_positions/*.cpp"
  "${PROJECT_SOURCE_DIR}/include/pedal_positions/*.c"
)

add_executable(dummy_ev src/dummy_ev.cpp)
ament_target_dependencies(dummy_ev
  rclcpp
  std_msgs
  geometry_msgs
  ix_msgs
  vcu_msgs
)

target_include_directories(dummy_ev PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include>")


install(TARGETS dummy_ev
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include
)

ament_export_include_directories(
  include
)

ament_package()
