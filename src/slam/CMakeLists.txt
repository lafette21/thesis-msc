set(EXEC_NAME slam)

add_executable(${EXEC_NAME}
    main.cc
)

target_include_directories(${EXEC_NAME} PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${EXEC_NAME} PUBLIC
    Boost::program_options
    Ceres::ceres
    fmt::fmt-header-only
    nova
    range-v3::range-v3
    ${PCL_LIBRARIES}
    yaml-cpp::yaml-cpp
)

code_analysis(${EXEC_NAME} PRIVATE)
