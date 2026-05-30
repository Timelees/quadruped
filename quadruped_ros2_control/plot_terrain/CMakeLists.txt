cmake_minimum_required(VERSION 3.8)
project(plot_terrain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(convex_plane_decomposition REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(grid_map_filters_rsl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIRS})

# Declare C++ executable
add_executable(${PROJECT_NAME}_ConvexPlaneDecomposition
  src/ConvexPlaneDecomposition.cpp
  # src/elevation_map_visualization.cpp
)


# Link libraries
ament_target_dependencies(${PROJECT_NAME}_ConvexPlaneDecomposition
  rclcpp
  convex_plane_decomposition
  convex_plane_decomposition_msgs
  convex_plane_decomposition_ros
  grid_map_filters_rsl
  grid_map_ros
  grid_map_sdf
  ocs2_ros_interfaces
  sensor_msgs
  visualization_msgs
  ocs2_robotic_tools
)
target_link_libraries(${PROJECT_NAME}_ConvexPlaneDecomposition
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

# Install executable
install(TARGETS ${PROJECT_NAME}_ConvexPlaneDecomposition
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
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
