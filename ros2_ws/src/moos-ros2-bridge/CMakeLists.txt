cmake_minimum_required(VERSION 3.8)
project(moos-ros2-bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(MOOS REQUIRED)

# Source files
set(SRC_FILES
  src/bridge_handler.cpp
  src/float64_bridge_handler.cpp
  src/float32_bridge_handler.cpp
  src/int32_bridge_handler.cpp
  src/string_bridge_handler.cpp
  src/bridge_factory.cpp
  src/Bridge.cpp
  src/MOOSNode.cpp
)

# Library
add_library(moos_ros2_bridge_lib ${SRC_FILES})
ament_target_dependencies(moos_ros2_bridge_lib
  rclcpp std_msgs geometry_msgs nav_msgs
)

target_include_directories(moos_ros2_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${MOOS_INCLUDE_DIRS}
)

target_link_libraries(moos_ros2_bridge_lib
  ${MOOS_LIBRARIES}
)

# Executable
add_executable(moos_ros2_bridge_main src/main.cpp)
target_link_libraries(moos_ros2_bridge_main
  moos_ros2_bridge_lib
)

target_include_directories(moos_ros2_bridge_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(moos_ros2_bridge_main
  rclcpp std_msgs geometry_msgs nav_msgs
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install targets
install(TARGETS 
  moos_ros2_bridge_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(counter_node src/counter.cpp)
ament_target_dependencies(counter_node
  rclcpp std_msgs
)
install(TARGETS counter_node DESTINATION lib/${PROJECT_NAME})

# Tests and linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
