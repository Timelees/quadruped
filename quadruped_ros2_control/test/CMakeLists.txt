cmake_minimum_required(VERSION 3.8)
project(test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 定义可执行文件
add_executable(imu_sanity_check src/imu_sanity_check.cpp)

# 3. 包含头文件目录 (Eigen 通常需要显式 include)
target_include_directories(imu_sanity_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # Modern CMake provides an imported target Eigen3::Eigen. Prefer linking the target below.
  # Keep this variable in case the FindEigen3 module sets it (some systems use EIGEN3_INCLUDE_DIR/Eigen3_INCLUDE_DIRS).
  ${Eigen3_INCLUDE_DIRS}
)

# 4. 链接 ROS 2 库
ament_target_dependencies(imu_sanity_check
  rclcpp
  sensor_msgs
)

# Prefer linking the imported Eigen target if available. Otherwise fall back to include directories.
if (TARGET Eigen3::Eigen)
  # Use plain signature to match ament_target_dependencies (which uses plain signature internally)
  target_link_libraries(imu_sanity_check Eigen3::Eigen)
else()
  if (DEFINED EIGEN3_INCLUDE_DIR)
    target_include_directories(imu_sanity_check PUBLIC ${EIGEN3_INCLUDE_DIR})
  elseif(DEFINED EIGEN3_INCLUDE_DIRS)
    target_include_directories(imu_sanity_check PUBLIC ${EIGEN3_INCLUDE_DIRS})
  endif()
endif()

# 5. 安装规则 (让 ros2 run 能找到它)
install(TARGETS imu_sanity_check
  DESTINATION lib/${PROJECT_NAME})

ament_package()