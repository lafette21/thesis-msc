set(EXEC_NAME perception)

add_executable(${EXEC_NAME}
    main.cc
)

target_include_directories(${EXEC_NAME} PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${EXEC_NAME} PUBLIC
    Boost::program_options
    fmt::fmt-header-only
    spdlog::spdlog_header_only
    nova
    range-v3::range-v3
    ${PCL_LIBRARIES}
    yaml-cpp::yaml-cpp
)

code_analysis(${EXEC_NAME} PRIVATE)
