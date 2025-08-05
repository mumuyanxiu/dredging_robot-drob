cmake_minimum_required(VERSION 3.8)
project(drob_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

# 添加可执行文件
add_executable(four_wheel_controller src/four_wheel_controller.cpp)
ament_target_dependencies(four_wheel_controller 
  rclcpp 
  geometry_msgs 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

# 安装可执行文件
install(TARGETS four_wheel_controller
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  src/test_data_flow.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config meshes world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
