cmake_minimum_required(VERSION 3.8)
project(dummy_dv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(ix_msgs)
find_package(std_msgs)
find_package(vcu_msgs)
find_package(vcu_shared_lib REQUIRED)

add_executable(dummy_dv_node src/dummyNode.cpp)
ament_target_dependencies(dummy_dv_node
  rclcpp
  std_msgs
  geometry_msgs
  ix_msgs
  vcu_msgs
  std_msgs
  vcu_shared_lib
)

target_include_directories(dummy_dv_node PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include>")


install(TARGETS dummy_dv_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include
)

ament_export_include_directories(
  include
)

ament_package()
