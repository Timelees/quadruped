cmake_minimum_required(VERSION 3.8)
project(test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_input_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ocs2_quadruped_controller REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(qpoases_colcon REQUIRED)

# 2. 定义可执行文件
add_executable(imu_sanity_check src/imu_sanity_check.cpp)
add_executable(clf_cbf_test src/clf_cbf_test.cpp)
add_executable(bspline_clf_cbf_test src/bspline_clf_cbf_test.cpp src/BSpline.cpp)
add_executable(traject_build src/traject_build.cpp)

# 3. 公共头文件目录
set(TEST_COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
  /opt/ros/humble/include/grid_map_core
)

target_include_directories(imu_sanity_check PUBLIC ${TEST_COMMON_INCLUDE_DIRS})
target_include_directories(clf_cbf_test PUBLIC ${TEST_COMMON_INCLUDE_DIRS})
target_include_directories(bspline_clf_cbf_test PUBLIC ${TEST_COMMON_INCLUDE_DIRS})
target_include_directories(traject_build PUBLIC ${TEST_COMMON_INCLUDE_DIRS})

# 4. 链接 ROS 2 库
ament_target_dependencies(imu_sanity_check
  rclcpp
  sensor_msgs
  grid_map_core
)

ament_target_dependencies(clf_cbf_test
  ocs2_quadruped_controller
  qpoases_colcon
  grid_map_core
)

ament_target_dependencies(bspline_clf_cbf_test
  ocs2_quadruped_controller
  qpoases_colcon
  grid_map_core
)

ament_target_dependencies(traject_build
  rclcpp
  control_input_msgs
)

# Prefer linking the imported Eigen target if available. Otherwise fall back to include directories.
if (TARGET Eigen3::Eigen)
  target_link_libraries(imu_sanity_check Eigen3::Eigen)
  target_link_libraries(clf_cbf_test Eigen3::Eigen)
  target_link_libraries(bspline_clf_cbf_test Eigen3::Eigen)
else()
  if (DEFINED EIGEN3_INCLUDE_DIR)
    target_include_directories(imu_sanity_check PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_include_directories(clf_cbf_test PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_include_directories(bspline_clf_cbf_test PUBLIC ${EIGEN3_INCLUDE_DIR})
  elseif(DEFINED EIGEN3_INCLUDE_DIRS)
    target_include_directories(imu_sanity_check PUBLIC ${EIGEN3_INCLUDE_DIRS})
    target_include_directories(clf_cbf_test PUBLIC ${EIGEN3_INCLUDE_DIRS})
    target_include_directories(bspline_clf_cbf_test PUBLIC ${EIGEN3_INCLUDE_DIR})
  endif()
endif()

target_link_libraries(clf_cbf_test
  ocs2_quadruped_controller::ocs2_quadruped_controller
  ${qpoases_colcon_LIBRARIES}
)

target_link_libraries(bspline_clf_cbf_test
  ocs2_quadruped_controller::ocs2_quadruped_controller
  ${qpoases_colcon_LIBRARIES}
)

# 5. 安装规则 (让 ros2 run 能找到它)
install(TARGETS imu_sanity_check clf_cbf_test bspline_clf_cbf_test traject_build
  DESTINATION lib/${PROJECT_NAME}) 

ament_package()
