cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(attach_shelf_interface REQUIRED)

# include header files
include_directories(include)

# components
add_library(pre_approach_node SHARED src/pre_approach.cpp)
target_compile_definitions(pre_approach_node PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach_node rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs)
rclcpp_components_register_nodes(pre_approach_node "my_components::PreApproach")

add_library(attach_client_node SHARED src/attach_client.cpp)
target_compile_definitions(attach_client_node PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_client_node rclcpp rclcpp_components attach_shelf_interface)
rclcpp_components_register_nodes(attach_client_node "my_components::AttachClient")

# compiler components target location
install(TARGETS
  pre_approach_node
  attach_client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

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
