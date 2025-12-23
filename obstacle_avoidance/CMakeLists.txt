cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters segmentation kdtree)
find_package(pcl_conversions REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(lasersensor 
  src/main.cpp 
  src/processPointClouds.cpp
  src/quiz/cluster/cluster_utils.cpp)

ament_target_dependencies(lasersensor rclcpp sensor_msgs geometry_msgs tf2_ros tf2 tf2_sensor_msgs  tf2_geometry_msgs nav_msgs visualization_msgs pcl_conversions)


install(TARGETS
 lasersensor
 DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(lasersensor
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
