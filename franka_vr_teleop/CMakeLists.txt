cmake_minimum_required(VERSION 3.8)
project(franka_vr_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED) # For Eigen conversions
find_package(Eigen3 REQUIRED COMPONENTS geometry) # For Eigen itself

add_executable(wrist_pose_control_cpp_node src/wrist_pose_control.cpp)

ament_target_dependencies(wrist_pose_control_cpp_node
  rclcpp
  geometry_msgs
  moveit_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3 # Only linking against Eigen3::Eigen or Eigen3::Relocatable is needed typically
)

target_include_directories(wrist_pose_control_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Eigen3_INCLUDE_DIRS}
)

# If Eigen is header-only, linking might not be strictly needed but often included.
# For Eigen, it's typically sufficient to ensure include directories are set.
# If Eigen provided actual libraries to link against (less common for core modules):
# target_link_libraries(wrist_pose_control_cpp_node Eigen3::Eigen)


install(TARGETS
  wrist_pose_control_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
