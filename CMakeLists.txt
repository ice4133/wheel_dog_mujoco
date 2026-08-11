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
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_driver src/driver/drv_dds.cpp)
target_include_directories(${PROJECT_NAME}_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_driver rclcpp unitree_go)

add_library(${PROJECT_NAME}_actuator_model src/actuator/actuator_model.cpp)
target_include_directories(${PROJECT_NAME}_actuator_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_actuator_model PUBLIC yaml-cpp)

add_library(${PROJECT_NAME}_actuator_controller src/actuator/actuator_controller.cpp)
target_include_directories(${PROJECT_NAME}_actuator_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_actuator_safety src/actuator/actuator_safety.cpp)
target_include_directories(${PROJECT_NAME}_actuator_safety PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_actuator_manager src/actuator/actuator_manager.cpp)
target_include_directories(${PROJECT_NAME}_actuator_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_actuator_manager PUBLIC
  ${PROJECT_NAME}_actuator_model
  ${PROJECT_NAME}_actuator_controller
  ${PROJECT_NAME}_actuator_safety
)

add_library(${PROJECT_NAME}_control src/stand_controller.cpp)
target_include_directories(${PROJECT_NAME}_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(stand_node
  src/stand_node.cpp
)
target_link_libraries(stand_node
  ${PROJECT_NAME}_control
  ${PROJECT_NAME}_driver
  ${PROJECT_NAME}_actuator_manager
)
ament_target_dependencies(stand_node ament_index_cpp geometry_msgs rclcpp unitree_go)

add_executable(keyboard_controller
  src/keyboard_controller.cpp
)
target_link_libraries(keyboard_controller ${PROJECT_NAME}_control)
ament_target_dependencies(keyboard_controller geometry_msgs rclcpp)

install(
  TARGETS
    ${PROJECT_NAME}_driver
    ${PROJECT_NAME}_actuator_model
    ${PROJECT_NAME}_actuator_controller
    ${PROJECT_NAME}_actuator_safety
    ${PROJECT_NAME}_actuator_manager
    ${PROJECT_NAME}_control
    stand_node
    keyboard_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(FILES config.yaml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp rclcpp unitree_go yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(actuator_manager_test test/actuator_manager_test.cpp)
  target_link_libraries(actuator_manager_test ${PROJECT_NAME}_actuator_manager)
  target_compile_definitions(actuator_manager_test PRIVATE
    ACTUATOR_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config.yaml"
  )
endif()

ament_package()
