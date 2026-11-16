cmake_minimum_required(VERSION 3.8)
project(robe_roll_out_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_srvs_msgs REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Development NumPy)

# Automatically download matplotlib-cpp header-only library
# include(FetchContent)
# FetchContent_Declare(
#   matplotlib_cpp
#   GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
#   GIT_TAG        master
# )
# FetchContent_MakeAvailable(matplotlib_cpp)

# -----------------------------------------------------------------------------
# Executable Build Target
# -----------------------------------------------------------------------------
add_executable(robe_roll_out_test src/robe_roll_out_test.cpp)

# Link ROS 2 and Python libraries
target_link_libraries(robe_roll_out_test
  rclcpp::rclcpp
  Python3::Python
  Python3::NumPy
)

target_include_directories(robe_roll_out_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${matplotlib_cpp_SOURCE_DIR}       # Path to downloaded matplotlibcpp.h
  ${Python3_NumPy_INCLUDE_DIRS}      # Path to NumPy headers
)
target_compile_features(robe_roll_out_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(robe_roll_out_test
  rclcpp
  rclcpp_action
  custom_srvs_msgs
)


# -----------------------------------------------------------------------------
# Install & Export
# -----------------------------------------------------------------------------
install(TARGETS robe_roll_out_test
  DESTINATION lib/${PROJECT_NAME})

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
