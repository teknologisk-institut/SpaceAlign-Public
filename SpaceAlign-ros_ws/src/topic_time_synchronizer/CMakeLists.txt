cmake_minimum_required(VERSION 3.8)
project(topic_time_synchronizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_srvs_msgs REQUIRED)


find_package(message_filters REQUIRED)

add_executable(time_synchronizer_node src/time_synchronizer_node.cpp)
ament_target_dependencies(time_synchronizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  custom_srvs_msgs

)

target_compile_features(time_synchronizer_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  time_synchronizer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
