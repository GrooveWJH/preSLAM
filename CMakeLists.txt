cmake_minimum_required(VERSION 3.10)
project(Groove-preSLAM)

# find_package(Eigen3 REQUIRED)
# message(STATUS "Found Eigen3: headers at ${Eigen3_INCLUDE_DIRS}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(APPLE)
    set(CMAKE_C_COMPILER "/usr/bin/clang")
    set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_SUPPRESS_REGENERATION ON)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/output)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(lib)

set(EXPERIMENTS_LIST)

file(GLOB_RECURSE experiments_sources "${CMAKE_SOURCE_DIR}/src/*.cxx" "${CMAKE_SOURCE_DIR}/src/*.cpp")

foreach(src_file ${experiments_sources})
    file(RELATIVE_PATH relative_path ${CMAKE_SOURCE_DIR}/src ${src_file})
    get_filename_component(base_name ${relative_path} NAME_WE)
    get_filename_component(target_dir ${relative_path} DIRECTORY)

    if ("${base_name}" MATCHES "^mid-")
        message(STATUS "Skipping target creation for intermediate file: ${src_file}")
        continue()
    endif()

    if (target_dir)
        string(REPLACE "/" "-" target_name_prefix ${target_dir})
        set(unique_target_name "${target_name_prefix}-${base_name}")
    else()
        set(unique_target_name "${base_name}")
    endif()

    add_executable(${unique_target_name} ${src_file})

    target_link_libraries(${unique_target_name} PRIVATE preslamlib)
    set_target_properties(${unique_target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}/${target_dir}")

    list(APPEND EXPERIMENTS_LIST ${unique_target_name})
endforeach()

add_custom_target(all_exps DEPENDS ${EXPERIMENTS_LIST})