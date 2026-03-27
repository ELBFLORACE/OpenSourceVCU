cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ix_msgs REQUIRED)
find_package(vcu_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(vcu_shared_lib REQUIRED)

set(${PROJECT_NAME}_node
  include/stateMachineNode.hpp
  src/stateMachineNode.cpp
)

include_directories(include)

add_executable(${PROJECT_NAME}_node src/stateMachineNode.cpp)

# Specify the source files for the library
set(STATE_MACHINE_LIBRARY_SRC
  include/StateMachineData.hpp
  include/EBSStateMachine.hpp
  src/stateMachine.cpp
  src/ebsStateMachine.cpp
)

# Create a static or shared library
add_library(state_machine_lib STATIC ${STATE_MACHINE_LIBRARY_SRC})
ament_target_dependencies(state_machine_lib PUBLIC vcu_shared_lib)
target_link_libraries(state_machine_lib PUBLIC yaml-cpp::yaml-cpp) # The library or executable that require yaml-cpp library

# Optionally, specify the include directories (for headers)
target_include_directories(state_machine_lib PUBLIC include)

# Link any required dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  ix_msgs
  std_srvs
  vcu_msgs
  vcu_shared_lib
)

target_link_libraries(${PROJECT_NAME}_node state_machine_lib)

# Test Integration
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/testStateMachine.cpp)

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    std_msgs
    rclcpp
  )

  target_link_libraries(${PROJECT_NAME}_test
    state_machine_lib
  )
endif()

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
