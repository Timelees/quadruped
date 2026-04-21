cmake_minimum_required(VERSION 3.5)

project(logs)

find_package(ament_cmake REQUIRED)

find_package(ocs2_core REQUIRED)
find_package(Eigen3 REQUIRED)



add_executable(logs_test
  # test/logs_test.cpp
  test/logs_saveCsv_test.cpp
)

target_include_directories(logs_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Allow finding ocs2_core headers from workspace install or source tree
target_include_directories(logs_test PRIVATE
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ocs2_ros2/core/ocs2_core/include
)
target_link_libraries(logs_test PRIVATE Eigen3::Eigen)

set_target_properties(logs_test PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
)

install(TARGETS logs_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Build library from source to link into test
add_library(log_save STATIC
  src/log_save.cpp
)

target_include_directories(log_save PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add ocs2_core include for library target as well
target_include_directories(log_save PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ocs2_ros2/core/ocs2_core/include
)
target_link_libraries(log_save PUBLIC Eigen3::Eigen)

set_target_properties(log_save PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
)

target_link_libraries(logs_test PRIVATE log_save)
target_link_libraries(log_save PRIVATE ocs2_core::ocs2_core)

# Install headers (copy include/ into install include/)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()