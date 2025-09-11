cmake_minimum_required(VERSION 3.31.1)
project(buffering LANGUAGES CXX)

# # Set cmake policy to use the new behavior for project version variables
# cmake_policy(SET CMP0048 NEW)


# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find and configure Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()

# Test executable
add_executable(
    cpu_test
    tests/cpu_test.cpp
)

# Link test executable with Google Test
target_link_libraries(
    cpu_test
    GTest::gtest_main
)

# Include directories for tests (so they can access src headers)
target_include_directories(cpu_test PRIVATE src)

# Discover and register tests
include(GoogleTest)
gtest_discover_tests(cpu_test)
# Main executable
add_executable(cpu src/cpu.cpp)