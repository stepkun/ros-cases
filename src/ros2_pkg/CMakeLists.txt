cmake_minimum_required(VERSION 3.8)
project(ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(test_interfaces REQUIRED)

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

add_executable(talker src/ros2_publisher.cpp)
ament_target_dependencies(talker rclcpp test_interfaces)

add_executable(listener src/ros2_subscriber.cpp)
ament_target_dependencies(listener rclcpp test_interfaces)

add_executable(server src/ros2_server.cpp)
ament_target_dependencies(server rclcpp test_interfaces)

add_executable(client src/ros2_client.cpp)
ament_target_dependencies(client rclcpp test_interfaces)

install(TARGETS
  talker
  listener
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
