cmake_minimum_required(VERSION 3.8)
project(spider_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(spider_warehouse REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(scan_filter src/scan_filter.cpp)
ament_target_dependencies(scan_filter rclcpp std_msgs sensor_msgs)
install(PROGRAMS
  src/test.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/test1.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/test2.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/gui.py
  src/gait.py
  src/circle_gait.py
  src/ike_gait.py
  DESTINATION lib/${PROJECT_NAME}
)
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
install(
  DIRECTORY
    launch
    worlds
    urdf
    meshes
    plugins
    config
    
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
