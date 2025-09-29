# Tests CMakeLists.txt

# Test executable
add_executable(
    cpu_test
    cpu_test.cpp
)

target_include_directories(cpu_test PRIVATE "${CMAKE_SOURCE_DIR}/OpenTimer")

# Link test executable with Google Test and buffering library
target_link_libraries(
    cpu_test
    GTest::gtest_main
    buffering_lib
    OpenTimer
)

# Discover and register tests
include(GoogleTest)
gtest_discover_tests(cpu_test)
