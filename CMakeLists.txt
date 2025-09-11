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

# Main executable
add_library(buffering_lib src/cpu.cpp)
target_include_directories(buffering_lib PUBLIC src)

add_executable(cpu src/main.cpp)
target_link_libraries(cpu buffering_lib)

# Enable testing and add tests subdirectory
enable_testing()
add_subdirectory(tests)