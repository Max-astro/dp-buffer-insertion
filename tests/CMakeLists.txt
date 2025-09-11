# Tests CMakeLists.txt

# Test executable
add_executable(
    cpu_test
    cpu_test.cpp
)

# Link test executable with Google Test
target_link_libraries(
    cpu_test
    GTest::gtest_main
)

# Include directories for tests (so they can access src headers)
target_include_directories(cpu_test PRIVATE ../src)

# Discover and register tests
include(GoogleTest)
gtest_discover_tests(cpu_test)
