include_directories(
    ${CMAKE_SOURCE_DIR}/src
)

add_executable(test-thesis
    utils.cc
)

target_link_libraries(test-thesis PRIVATE
    nova

    Eigen3::Eigen
    fmt::fmt-header-only

    GTest::gmock
    GTest::gtest
    GTest::gtest_main
)

code_analysis(test-thesis PRIVATE)

include(CTest)
include(GoogleTest)

gtest_discover_tests(
    test-thesis
    DISCOVERY_MODE PRE_TEST
)
