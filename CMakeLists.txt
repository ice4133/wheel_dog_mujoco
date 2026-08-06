cmake_minimum_required(VERSION 3.8)
project(wheel_dog_mujoco)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)

add_library(${PROJECT_NAME}_driver src/driver/drv_dds.cpp)
target_include_directories(${PROJECT_NAME}_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_driver rclcpp unitree_go)

add_library(${PROJECT_NAME}_control src/stand_controller.cpp)
target_include_directories(${PROJECT_NAME}_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(stand_node
  src/stand_node.cpp
)
target_link_libraries(stand_node ${PROJECT_NAME}_control ${PROJECT_NAME}_driver)
ament_target_dependencies(stand_node geometry_msgs rclcpp unitree_go)

add_executable(keyboard_controller
  src/keyboard_controller.cpp
)
target_link_libraries(keyboard_controller ${PROJECT_NAME}_control)
ament_target_dependencies(keyboard_controller geometry_msgs rclcpp)

install(
  TARGETS ${PROJECT_NAME}_driver ${PROJECT_NAME}_control stand_node keyboard_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp unitree_go)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
