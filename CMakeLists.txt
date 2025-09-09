cmake_minimum_required(VERSION 3.31.1)
project(buffering LANGUAGES CXX)

# # Set cmake policy to use the new behavior for project version variables
# cmake_policy(SET CMP0048 NEW)


# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(cpu src/cpu.cpp)