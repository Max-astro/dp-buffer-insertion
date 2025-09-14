# Tests CMakeLists.txt

# Test executable
add_executable(
    cpu_test
    cpu_test.cpp
)

# Link test executable with Google Test and buffering library
target_link_libraries(
    cpu_test
    GTest::gtest_main
    liberty_helper
    buffering_lib
)

# Discover and register tests
include(GoogleTest)
gtest_discover_tests(cpu_test)
