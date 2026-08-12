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
find_package(std_msgs REQUIRED)
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

add_library(${PROJECT_NAME}_body_model src/body/body_model.cpp)
target_include_directories(${PROJECT_NAME}_body_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_body_model PUBLIC yaml-cpp)

add_library(${PROJECT_NAME}_body_state_estimator src/body/body_state_estimator.cpp)
target_include_directories(${PROJECT_NAME}_body_state_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_body_state_estimator PUBLIC
  ${PROJECT_NAME}_body_model
)

add_library(${PROJECT_NAME}_body_trajectory src/body/body_trajectory.cpp)
target_include_directories(${PROJECT_NAME}_body_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_body_controller src/body/body_controller.cpp)
target_include_directories(${PROJECT_NAME}_body_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_body_controller PUBLIC
  ${PROJECT_NAME}_body_trajectory
)

add_library(${PROJECT_NAME}_wheel_leg_coordinator src/body/wheel_leg_coordinator.cpp)
target_include_directories(${PROJECT_NAME}_wheel_leg_coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_wheel_leg_coordinator PUBLIC
  ${PROJECT_NAME}_body_controller
  ${PROJECT_NAME}_body_model
)

add_library(${PROJECT_NAME}_body_manager src/body/body_manager.cpp)
target_include_directories(${PROJECT_NAME}_body_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_body_manager PUBLIC
  ${PROJECT_NAME}_body_state_estimator
  ${PROJECT_NAME}_body_trajectory
  ${PROJECT_NAME}_body_controller
  ${PROJECT_NAME}_wheel_leg_coordinator
)

add_library(${PROJECT_NAME}_stand_skill src/skill/stand_skill.cpp)
target_include_directories(${PROJECT_NAME}_stand_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_skill_task src/skill/skill_task.cpp)
target_include_directories(${PROJECT_NAME}_skill_task PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(skill_task
  src/skill/skill_task_node.cpp
  src/skill/skill_task_main.cpp
)
target_link_libraries(skill_task
  ${PROJECT_NAME}_driver
  ${PROJECT_NAME}_actuator_manager
  ${PROJECT_NAME}_stand_skill
  ${PROJECT_NAME}_skill_task
  yaml-cpp
)
ament_target_dependencies(
  skill_task ament_index_cpp geometry_msgs rclcpp std_msgs unitree_go)

add_executable(keyboard_controller
  src/keyboard_controller.cpp
)
target_include_directories(keyboard_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(keyboard_controller geometry_msgs rclcpp)

install(
  TARGETS
    ${PROJECT_NAME}_driver
    ${PROJECT_NAME}_actuator_model
    ${PROJECT_NAME}_actuator_controller
    ${PROJECT_NAME}_actuator_safety
    ${PROJECT_NAME}_actuator_manager
    ${PROJECT_NAME}_body_model
    ${PROJECT_NAME}_body_state_estimator
    ${PROJECT_NAME}_body_trajectory
    ${PROJECT_NAME}_body_controller
    ${PROJECT_NAME}_wheel_leg_coordinator
    ${PROJECT_NAME}_body_manager
    ${PROJECT_NAME}_stand_skill
    ${PROJECT_NAME}_skill_task
    skill_task
    keyboard_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp geometry_msgs rclcpp std_msgs unitree_go yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(actuator_manager_test test/actuator_manager_test.cpp)
  target_link_libraries(actuator_manager_test ${PROJECT_NAME}_actuator_manager)
  target_compile_definitions(actuator_manager_test PRIVATE
    ACTUATOR_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/actuator.yaml"
  )

  ament_add_gtest(skill_task_test test/skill_task_test.cpp)
  target_link_libraries(skill_task_test ${PROJECT_NAME}_skill_task)

  ament_add_gtest(stand_skill_test test/stand_skill_test.cpp)
  target_link_libraries(stand_skill_test ${PROJECT_NAME}_stand_skill)

  ament_add_gtest(body_model_test test/body_model_test.cpp)
  target_link_libraries(body_model_test ${PROJECT_NAME}_body_model)
  target_compile_definitions(body_model_test PRIVATE
    BODY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/body.yaml"
  )

  ament_add_gtest(body_state_estimator_test test/body_state_estimator_test.cpp)
  target_link_libraries(body_state_estimator_test
    ${PROJECT_NAME}_body_state_estimator
  )
  target_compile_definitions(body_state_estimator_test PRIVATE
    BODY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/body.yaml"
  )

  ament_add_gtest(body_trajectory_test test/body_trajectory_test.cpp)
  target_link_libraries(body_trajectory_test
    ${PROJECT_NAME}_body_trajectory
    ${PROJECT_NAME}_body_model
  )
  target_compile_definitions(body_trajectory_test PRIVATE
    BODY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/body.yaml"
  )

  ament_add_gtest(body_controller_test test/body_controller_test.cpp)
  target_link_libraries(body_controller_test
    ${PROJECT_NAME}_body_controller
    ${PROJECT_NAME}_body_model
  )
  target_compile_definitions(body_controller_test PRIVATE
    BODY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/body.yaml"
  )

  ament_add_gtest(wheel_leg_coordinator_test test/wheel_leg_coordinator_test.cpp)
  target_link_libraries(wheel_leg_coordinator_test
    ${PROJECT_NAME}_wheel_leg_coordinator
  )
  target_compile_definitions(wheel_leg_coordinator_test PRIVATE
    BODY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/body.yaml"
  )

  ament_add_gtest(body_manager_test test/body_manager_test.cpp)
  target_link_libraries(body_manager_test
    ${PROJECT_NAME}_body_manager
  )
  target_compile_definitions(body_manager_test PRIVATE
    BODY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/body.yaml"
  )
endif()

ament_package()
