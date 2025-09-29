cmake_minimum_required(VERSION 3.13.1)
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

# For debugging
set(ENABLE_ASSERT 1)

set(TASKFLOW_DIR "${CMAKE_SOURCE_DIR}/taskflow")

# Bring in OpenTimer submodule (library target: OpenTimer)
add_subdirectory(OpenTimer)

add_library(buffering_lib src/cpu.cpp)
target_include_directories(buffering_lib PUBLIC src PRIVATE ${TASKFLOW_DIR} "${CMAKE_SOURCE_DIR}/OpenTimer")
target_link_libraries(buffering_lib PRIVATE OpenTimer)

# Set debug flags for Debug build type
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG -DENABLE_ASSERT=${ENABLE_ASSERT}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()


# Benchmark executable
add_executable(benchmark src/benchmark.cpp)
target_include_directories(benchmark PRIVATE "${CMAKE_SOURCE_DIR}/OpenTimer")
target_link_libraries(benchmark PRIVATE buffering_lib OpenTimer)

# Example executable
add_executable(example src/example.cpp)
target_include_directories(example PRIVATE "${CMAKE_SOURCE_DIR}/OpenTimer")
target_link_libraries(example PRIVATE buffering_lib OpenTimer)

add_executable(opt src/main.cpp)
target_include_directories(opt PRIVATE "${CMAKE_SOURCE_DIR}/OpenTimer")
target_link_libraries(opt PRIVATE buffering_lib OpenTimer)

# # Add debug-specific settings for the executable
# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
# target_compile_definitions(cpu PRIVATE DEBUG_BUILD)
# target_compile_options(cpu PRIVATE -Wall -Wextra -Wpedantic)
# endif()

# Enable testing and add tests subdirectory
enable_testing()
add_subdirectory(tests)
