cmake_minimum_required(VERSION 3.8)
project(spacealign_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Flags
set(config_file_path "${CMAKE_CURRENT_SOURCE_DIR}/config/default-config_ver2.xml")

set(DEBUG ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit REQUIRED)
find_package(custom_srvs_msgs REQUIRED)

set(MAPLE_K_DIR "${CMAKE_CURRENT_SOURCE_DIR}/MAPLE-lib/lib/MAPLE_K-1.0")
find_package(MAPLE_K REQUIRED)


# Norminal 
add_executable(lin_move_norminal src/MainController.cpp src/RobotProgram.cpp src/linear_move_norminal.cpp)
target_include_directories(lin_move_norminal PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(lin_move_norminal MAPLE_K::MAPLE_K)
target_compile_features(lin_move_norminal PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(lin_move_norminal PRIVATE 
config_file_path="${config_file_path}")

ament_target_dependencies(lin_move_norminal
rclcpp
rclcpp_action
custom_srvs_msgs
)
install(TARGETS lin_move_norminal DESTINATION lib/${PROJECT_NAME})

# Release_failure
add_executable(lin_move_release_failure  src/MainController.cpp src/RobotProgram.cpp src/linear_move_release_failure.cpp)
target_include_directories(lin_move_release_failure PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(lin_move_release_failure MAPLE_K::MAPLE_K)
target_compile_features(lin_move_release_failure PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(lin_move_release_failure PRIVATE 
config_file_path="${config_file_path}")

ament_target_dependencies(lin_move_release_failure
rclcpp
rclcpp_action
custom_srvs_msgs
)
install(TARGETS lin_move_release_failure DESTINATION lib/${PROJECT_NAME})



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
