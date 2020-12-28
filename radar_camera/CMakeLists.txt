cmake_minimum_required(VERSION 3.5)
project(radar_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lane_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories (
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
)

add_library(lane_detection_projection_component SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/lane_detection_projection_component.cpp)
target_compile_definitions(lane_detection_projection_component PRIVATE "LANE_DETECTION_PROJECTION_BUILDING_DLL")
target_link_libraries(lane_detection_projection_component dl -lboost_system
  ${rclcpp_lifecycle_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
)

ament_target_dependencies(
  lane_detection_projection_component
  lane_msgs
  sensor_msgs
  rclcpp_lifecycle
  rclcpp
  std_msgs
  rclcpp_components
  ament_index_cpp
  Boost
  OpenCV
)

rclcpp_components_register_nodes(lane_detection_projection_component "CAMERA::lane_detection_projection")
set(node_plugins "${node_plugins}CAMERA::lane_detection_projection;$<TARGET_FILE:lane_detection_projection_component>\n")


add_executable(lane_detection_projection_composition ${CMAKE_CURRENT_SOURCE_DIR}/src/lane_detection_projection_composition.cpp)

target_link_libraries(lane_detection_projection_composition
  lane_detection_projection_component
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  -dl -lboost_system -lavformat -lavcodec -lswresample -lswscale -lavutil -lm -lz -lpthread
)

ament_target_dependencies(lane_detection_projection_composition
  rclcpp
)

install(
  TARGETS lane_detection_projection_composition
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
