cmake_minimum_required(VERSION 3.10)

project(preslamlib VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(preslamlib INTERFACE)

target_include_directories(preslamlib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/eigen>
)
