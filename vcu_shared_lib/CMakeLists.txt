cmake_minimum_required(VERSION 3.16)
project(vcu_shared_lib)

find_package(rclcpp REQUIRED)
find_package(ix_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

# Install headers to include/<project_name>
install(DIRECTORY include/
  DESTINATION include
)

# Sources to compile into the library
set(SOURCES
  src/enums.cpp
  src/params.cpp
  src/wheels.cpp
)

# Build a compiled library (static preferred for faster link times across builds)
include_directories(include)
add_library(vcu_shared_lib STATIC ${SOURCES})

target_include_directories(vcu_shared_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link any dependencies your sources need
target_link_libraries(vcu_shared_lib PUBLIC
    rclcpp::rclcpp
    ix_msgs::ix_msgs__rosidl_typesupport_cpp
)


# Export/install targets and headers
ament_export_targets(vcu_shared_lib HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

install(TARGETS vcu_shared_lib
  EXPORT vcu_shared_lib
  ARCHIVE DESTINATION lib    # for static libs
  LIBRARY DESTINATION lib    # for shared libs (if you change to SHARED)
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(EXPORT vcu_shared_lib
  FILE vcu_shared_libTargets.cmake
  NAMESPACE vcu_shared_lib::
  DESTINATION lib/cmake/vcu_shared_lib
)

ament_package()
