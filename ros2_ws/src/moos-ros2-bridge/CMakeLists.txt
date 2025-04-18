cmake_minimum_required(VERSION 3.8)
project(moos-ros2-bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(MOOS REQUIRED)
include_directories(
  $(MOOS_INCLUDE_DIRS)
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
link_directories($(MOOS_LIBRARY_DIRS))

# include directories
include_directories(
  include/moos-ros2-bridge
)

# list of source files
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

add_library(moos_ros2_bridge_lib ${SRC_FILES})

ament_target_dependencies(moos_ros2_bridge_lib rclcpp std_msgs geometry_msgs nav_msgs)

target_include_directories(moos_ros2_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $(MOOS_INCLUDE_DIRS)
)

add_executable(moos_ros2_bridge_main src/main.cpp)
target_link_libraries(moos_ros2_bridge_main moos_ros2_bridge_lib)

# create a static library (if using this in other nodes)

target_include_directories(moos_ros2_bridge_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# install headers
install(DIRECTORY include/
  DESTINATION include/
)

# install library
install(TARGETS 
  moos_ros2_bridge_lib
  moos_ros2_bridge_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
