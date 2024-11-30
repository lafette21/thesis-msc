cmake_minimum_required(VERSION 3.16)

project(thesis-msc
    VERSION 0.0.1
    DESCRIPTION ""
    HOMEPAGE_URL "https://github.com/lafette21/thesis-msc"
    LANGUAGES CXX C
)

option(ENABLE_ROSBAG2_OUTPUT "Enable output into ROS2 bag format" OFF)

if(ENABLE_ROSBAG2_OUTPUT)
    add_definitions(-DENABLE_ROSBAG2_OUTPUT=1)
else()
    add_definitions(-DENABLE_ROSBAG2_OUTPUT=0)
endif()

include(cmake/compiler-warnings.cmake)
include(cmake/dependencies.cmake)
include(cmake/settings.cmake)

enable_testing()

add_subdirectory(benchmark)
add_subdirectory(deps)
add_subdirectory(src)
