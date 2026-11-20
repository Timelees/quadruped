cmake_minimum_required(VERSION 3.8)
project(ocs2_quadruped_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CONTROLLER_INCLUDE_DEPENDS
        pluginlib
        rcpputils
        controller_interface
        controller_common
        gz_quadruped_hardware

        ocs2_legged_robot_ros
        ocs2_self_collision
        control_input_msgs
        angles
        nav_msgs
        qpoases_colcon
        ament_index_cpp

        convex_plane_decomposition_msgs
        convex_plane_decomposition_ros
        grid_map_filters_rsl
        grid_map_sdf
        ocs2_sphere_approximation
        sensor_msgs

        elevation_mapping

        logs

        Sophus
)



# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED
        src/Ocs2QuadrupedController.cpp
        src/FSM/StateOCS2.cpp

        src/estimator/GroundTruth.cpp
        src/estimator/LinearKalmanFilter.cpp
        src/estimator/StateEstimateBase.cpp
        src/estimator/FromOdomTopic.cpp
        src/estimator/ErrorStateKalmanFilter.cpp
        src/estimator/ContactEstimator.cpp
        src/estimator/STAObserver.cpp


        src/wbc/HierarchicalWbc.cpp
        src/wbc/HoQp.cpp
        src/wbc/WbcBase.cpp
        src/wbc/WeightedWbc.cpp

        src/interface/constraint/EndEffectorLinearConstraint.cpp
        src/interface/constraint/FrictionConeConstraint.cpp
        src/interface/constraint/ZeroForceConstraint.cpp
        src/interface/constraint/NormalVelocityConstraintCppAd.cpp
        src/interface/constraint/ZeroVelocityConstraintCppAd.cpp
        src/interface/constraint/QuinticSpline.cpp
        src/interface/constraint/QuinticSplineCpg.cpp
        src/interface/constraint/BSpline.cpp
        src/interface/constraint/SwingTrajectoryPlanner.cpp
        src/interface/constraint/SwingOnlineReplanner.cpp
        src/interface/constraint/SwingSafetyFilter.cpp
        src/interface/constraint/PolynomialHull.cpp
        src/interface/constraint/TerrainGeometry.cpp
        src/interface/constraint/TerrainSafetyCorridor.cpp
        src/interface/constraint/SwingCollisionCertifier.cpp
        src/interface/constraint/SwingTrajectoryQPOptimizer.cpp
        src/interface/initialization/LeggedRobotInitializer.cpp
        src/interface/SwitchedModelReferenceManager.cpp
        src/interface/LeggedRobotPreComputation.cpp
        src/interface/LeggedInterface.cpp


        src/control/GaitManager.cpp
        src/control/TargetManager.cpp
        src/control/CtrlComponent.cpp

        src/perceptive/constraint/FootCollisionConstraint.cpp
        src/perceptive/constraint/FootPlacementCBFConstraint.cpp
        src/perceptive/constraint/FootPlacementConstraint.cpp
        src/perceptive/constraint/SphereSdfConstraint.cpp
        src/perceptive/common/FootPlacementSafetySet.cpp
        src/perceptive/interface/ConvexRegionSelector.cpp
        src/perceptive/interface/PerceptiveLeggedInterface.cpp
        src/perceptive/interface/PerceptiveLeggedPrecomputation.cpp
        src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp
        src/perceptive/visualize/FootPlacementVisualization.cpp
        src/perceptive/visualize/SphereVisualization.cpp
        src/perceptive/visualize/TerrainCorridorVisualization.cpp
        src/perceptive/synchronize/PlanarTerrainReceiver.cpp
        # include logs implementation so LogSave symbols are available at link time
        ../../logs/src/log_save.cpp


)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${qpOASES_INCLUDE_DIR}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        # Also add logs package include directory (local workspace path) for build only
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../logs/include>"

        
        PRIVATE
        src)
ament_target_dependencies(${PROJECT_NAME}
        ${CONTROLLER_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(controller_interface ocs2_quadruped_controller.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)


ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Runnable self-check for the section 3.1 terrain safety corridor builder.
# Plain executable so it can be run without a test harness or a terrain map.
add_executable(terrain_safety_corridor_selfcheck test/terrain_safety_corridor_selfcheck.cpp)
target_include_directories(terrain_safety_corridor_selfcheck PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(terrain_safety_corridor_selfcheck
        ${PROJECT_NAME}
        Eigen3::Eigen)

# Runnable self-check for the section 4 online replanning admission test and the
# section 4.1-4.3 acceleration safety filter.
add_executable(swing_replan_selfcheck test/swing_replan_selfcheck.cpp)
target_include_directories(swing_replan_selfcheck PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(swing_replan_selfcheck
        ${PROJECT_NAME}
        Eigen3::Eigen)

add_executable(swing_update_experiment test/swing_update_experiment.cpp)
target_link_libraries(swing_update_experiment ${PROJECT_NAME} Eigen3::Eigen)

add_executable(execution_contract_selfcheck test/execution_contract_selfcheck.cpp)
target_link_libraries(execution_contract_selfcheck ${PROJECT_NAME} Eigen3::Eigen)

if (BUILD_TESTING)
    add_test(NAME swing_replan_selfcheck COMMAND swing_replan_selfcheck)
    add_test(NAME execution_contract_selfcheck COMMAND execution_contract_selfcheck)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
