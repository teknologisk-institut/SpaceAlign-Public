cmake_minimum_required(VERSION 3.8)
project(isaac_sim_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit REQUIRED)
find_package(custom_srvs_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(sim_bootstrap src/sim_bootstrap.cpp)
add_executable(sim_ctrl
  src/SimCtrl.cpp
  src/MovePoseActionHandler.cpp
  src/MoveJointActionHandler.cpp
  src/sim_ctrl_main.cpp
  src/USDA_writer.cpp
)
add_executable(test_srvs test/test_srvs.cpp)
add_executable(test_act test/test_act.cpp)
add_executable(all_in_one_test test/all_in_one_test.cpp)

target_include_directories(sim_bootstrap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(sim_ctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(test_srvs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(test_act PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(all_in_one_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(sim_bootstrap PUBLIC c_std_99 cxx_std_17)
target_compile_features(sim_ctrl PUBLIC c_std_99 cxx_std_23)
target_compile_features(test_srvs PUBLIC c_std_99 cxx_std_17)
target_compile_features(test_act PUBLIC c_std_99 cxx_std_17)
target_compile_features(all_in_one_test PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(sim_bootstrap
  rclcpp
  simulation_interfaces
)
ament_target_dependencies(sim_ctrl
  rclcpp
  rclcpp_action
  simulation_interfaces
  std_srvs
  custom_srvs_msgs
  controller_manager_msgs
  moveit_ros_planning_interface
  nlohmann_json
)
ament_target_dependencies(test_srvs
  rclcpp
  custom_srvs_msgs
)
ament_target_dependencies(test_act
  rclcpp
  rclcpp_action
  custom_srvs_msgs
)
ament_target_dependencies(all_in_one_test
  rclcpp
  rclcpp_action
  custom_srvs_msgs
  simulation_interfaces
)

install(TARGETS sim_bootstrap DESTINATION lib/${PROJECT_NAME})
install(TARGETS sim_ctrl DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_srvs DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_act DESTINATION lib/${PROJECT_NAME})
install(TARGETS all_in_one_test DESTINATION lib/${PROJECT_NAME})


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
