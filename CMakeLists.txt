cmake_minimum_required(VERSION 3.8)
project(rebet_frog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies We import external dependencies for both Cpp and Python at the same time.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rebet_msgs REQUIRED)
find_package(rebet REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(wavefront_frontier_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aal_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_lifecycle 
    behaviortree_ros2
    behaviortree_cpp
    rebet_msgs
    lifecycle_msgs
    diagnostic_msgs 
    geometry_msgs
    nav_msgs
    nav2_msgs
    sensor_msgs
    rebet
    slam_toolbox
    wavefront_frontier_msgs
    aal_msgs)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(frog_arborist src/frog_arborist.cpp)
ament_target_dependencies(frog_arborist ${THIS_PACKAGE_DEPS})

add_library(find_frontier_plugin SHARED src/find_frontier_service.cpp)
target_compile_definitions(find_frontier_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(find_frontier_plugin ${THIS_PACKAGE_DEPS})

add_library(get_map_plugin SHARED src/get_map_service.cpp)
target_compile_definitions(get_map_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(get_map_plugin ${THIS_PACKAGE_DEPS})

add_library(gotopose_plugin SHARED src/gotopose_action.cpp)
target_compile_definitions(gotopose_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(gotopose_plugin ${THIS_PACKAGE_DEPS})

add_library(detect_plugin SHARED src/detect_service.cpp)
target_compile_definitions(detect_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(detect_plugin ${THIS_PACKAGE_DEPS})

add_library(initial_pose_plugin SHARED src/initial_pose_pub.cpp)
target_compile_definitions(initial_pose_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(initial_pose_plugin ${THIS_PACKAGE_DEPS})

add_library(visit_obstacle_plugin SHARED src/visitobstacle_action.cpp)
target_compile_definitions(visit_obstacle_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(visit_obstacle_plugin ${THIS_PACKAGE_DEPS})

add_library(robot_pose_plugin SHARED src/robot_pose_sub.cpp)
target_compile_definitions(robot_pose_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(robot_pose_plugin ${THIS_PACKAGE_DEPS})




# Install Cpp executables
install(TARGETS
  frog_arborist
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/blackboard_set_client.py
  scripts/blackboard_get_client.py
  scripts/system_reflection.py
  scripts/tree_action_client.py
  scripts/weights_set_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  find_frontier_plugin
  get_map_plugin
  gotopose_plugin
  detect_plugin
  initial_pose_plugin
  visit_obstacle_plugin
  robot_pose_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  find_frontier_plugin
  get_map_plugin
  gotopose_plugin
  detect_plugin
  initial_pose_plugin
  visit_obstacle_plugin
  robot_pose_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

install(DIRECTORY
  launch
  config
  trees
  models
  worlds
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
