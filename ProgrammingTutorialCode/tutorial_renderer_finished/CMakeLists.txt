cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(renderer)

find_package(PCL 1.2 REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -pthread")
set(sources src/main.cpp src/Renderer.cpp)

add_executable (main ${sources})
target_link_libraries (main ${PCL_LIBRARIES} )


