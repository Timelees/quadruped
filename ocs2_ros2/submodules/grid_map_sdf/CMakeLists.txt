cmake_minimum_required(VERSION 3.8)
project(grid_map_sdf)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)
# Solution to find Eigen3 with Saucy.
find_package(Eigen3 QUIET)
if (NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif ()

grid_map_package()

set(dependencies
        grid_map_core
        PCL
)

###########
## Build ##
###########

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/SignedDistanceField.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        SYSTEM
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} SYSTEM
        ${dependencies}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.hpp"
)

#############
## Testing ##
#############

if (BUILD_TESTING)
    # Linting is setup this way to add a filter
    # to ament_cpplint to ignore the lack of
    # copyright messages at the top of files.
    # Copyright messages are being checked for by both
    # ament_cmake_cpplint & ament_cmake_copyright.

    find_package(ament_lint_auto REQUIRED)
    find_package(ament_lint_auto QUIET)
    if (ament_lint_auto_FOUND)
        # exclude copyright checks
        list(APPEND AMENT_LINT_AUTO_EXCLUDE
                ament_cmake_cpplint
                ament_cmake_copyright
        )
        ament_lint_auto_find_test_dependencies()

        # run cpplint without copyright filter
        find_package(ament_cmake_cpplint)
        ament_cpplint(
                FILTERS -legal/copyright -build/include_order
        )
    endif ()
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    ament_add_gtest(${PROJECT_NAME}-test
            test/SignedDistanceFieldTest.cpp
            test/test_grid_map_sdf.cpp)

    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

endif ()

if (TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif ()

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
ament_package()
