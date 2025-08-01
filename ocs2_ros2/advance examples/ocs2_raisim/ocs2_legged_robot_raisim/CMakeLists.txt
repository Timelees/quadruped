cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_legged_robot_raisim)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_PREFIX_PATH "~/raisimLib/raisim/linux" ${CMAKE_PREFIX_PATH})

set(dependencies
        ocs2_legged_robot_ros
        ocs2_raisim_core
        ament_index_cpp
        grid_map_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_raisim_core REQUIRED)
find_package(grid_map_msgs REQUIRED)

###########
## Build ##
###########

# main library
add_library(${PROJECT_NAME}
        src/RaiSimConversions.cpp
        src/RaiSimVisualizer.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# RaiSim dummy
add_executable(legged_robot_raisim_dummy src/RaiSimDummy.cpp)
ament_target_dependencies(legged_robot_raisim_dummy ${dependencies})
target_link_libraries(legged_robot_raisim_dummy ${PROJECT_NAME})
target_compile_options(legged_robot_raisim_dummy PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS
        legged_robot_raisim_dummy
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest)

ament_add_gtest(${PROJECT_NAME}_test
        test/testLeggedRobotRaisimConversions.cpp
)
ament_target_dependencies(${PROJECT_NAME}_test
        ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()