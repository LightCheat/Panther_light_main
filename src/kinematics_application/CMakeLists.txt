cmake_minimum_required(VERSION 3.8)
project(kinematics_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(forward_differential_kinematics
  src/forward_differential_kinematics.cpp
)

add_library(ik_solver_dls
  src/ik_solver_dls.cpp
)

target_include_directories(forward_differential_kinematics PUBLIC
  include
)

target_include_directories(ik_solver_dls PUBLIC
  include
)

target_link_libraries(
  forward_differential_kinematics
  Eigen3::Eigen
)

target_link_libraries(ik_solver_dls PUBLIC
  pinocchio::pinocchio
  Eigen3::Eigen
)

ament_target_dependencies(
  forward_differential_kinematics
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(
  forward_differential_kinematics_demo1
  src/forward_differential_kinematics_demo1.cpp
)

target_link_libraries(
  forward_differential_kinematics_demo1
  forward_differential_kinematics
  Eigen3::Eigen
)

ament_target_dependencies(
  forward_differential_kinematics_demo1
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(
  joint_velocity_pub_node
  src/joint_velocity_pub_node.cpp
)

add_executable(
  ik_dls_demo
  src/ik_dls_demo.cpp
)

target_link_libraries(
  ik_dls_demo
  ik_solver_dls
)

ament_target_dependencies(
  ik_dls_demo
  rclcpp
  ament_index_cpp
  std_msgs
)

ament_target_dependencies(
  joint_velocity_pub_node
  rclcpp
  std_msgs
)

install(
  TARGETS
  forward_differential_kinematics
  ik_solver_dls
  forward_differential_kinematics_demo1
  joint_velocity_pub_node
  ik_dls_demo
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

ament_package()
