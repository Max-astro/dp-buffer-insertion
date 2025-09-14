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


# Path to the Rust crate
set(RUST_CRATE_DIR "${CMAKE_SOURCE_DIR}/liberty-helper")  # adjust as needed

set(TASKFLOW_DIR "${CMAKE_SOURCE_DIR}/taskflow")

# Build type → Cargo profile/args
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(RUST_PROFILE "release")
  set(CARGO_BUILD_ARGS --release)
else()
  set(RUST_PROFILE "debug")
  set(CARGO_BUILD_ARGS)
endif()

# Artifacts we consume
set(RUST_TARGET_DIR "${RUST_CRATE_DIR}/target/${RUST_PROFILE}")
set(RUST_STATIC     "${RUST_TARGET_DIR}/libliberty_helper.a")

# cxx bridge files copied by build.rs into a stable include dir
set(RUST_INCLUDE_DIR "${RUST_CRATE_DIR}/include")
set(RUST_CXX_HEADER  "${RUST_INCLUDE_DIR}/liberty_helper.rs.h")
set(RUST_CXX_UMBRELLA "${RUST_INCLUDE_DIR}/liberty_helper.hpp")
set(RUST_CXX_SHIM    "${RUST_INCLUDE_DIR}/liberty_helper.rs.cc")

# Build the Rust crate (produces staticlib + copies cxx shim/header to include/)
add_custom_command(
  OUTPUT "${RUST_STATIC}" "${RUST_CXX_HEADER}" "${RUST_CXX_UMBRELLA}" "${RUST_CXX_SHIM}"
  COMMAND cargo build ${CARGO_BUILD_ARGS}
  WORKING_DIRECTORY "${RUST_CRATE_DIR}"
  COMMENT "Building Rust crate via Cargo (${RUST_PROFILE})"
  VERBATIM
)

add_custom_target(liberty_helper_rust ALL
  DEPENDS "${RUST_STATIC}" "${RUST_CXX_HEADER}" "${RUST_CXX_UMBRELLA}" "${RUST_CXX_SHIM}"
)

# Import the Rust static library
add_library(liberty_helper STATIC IMPORTED GLOBAL)
set_target_properties(liberty_helper PROPERTIES
  IMPORTED_LOCATION "${RUST_STATIC}"
  INTERFACE_INCLUDE_DIRECTORIES "${RUST_INCLUDE_DIR}"
)

add_library(buffering_lib src/cpu.cpp)
add_dependencies(buffering_lib liberty_helper_rust)
target_include_directories(buffering_lib PUBLIC src PRIVATE ${RUST_INCLUDE_DIR} ${TASKFLOW_DIR})
target_link_libraries(buffering_lib PRIVATE liberty_helper)

# Set debug flags for Debug build type
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Main executable
add_executable(cpu src/main.cpp "${RUST_CXX_SHIM}")
add_dependencies(cpu liberty_helper_rust)
target_include_directories(cpu PRIVATE "${RUST_INCLUDE_DIR}")
target_link_libraries(cpu PRIVATE liberty_helper buffering_lib)


# # Add debug-specific settings for the executable
# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#     target_compile_definitions(cpu PRIVATE DEBUG_BUILD)
#     target_compile_options(cpu PRIVATE -Wall -Wextra -Wpedantic)
# endif()

# Enable testing and add tests subdirectory
enable_testing()
add_subdirectory(tests)