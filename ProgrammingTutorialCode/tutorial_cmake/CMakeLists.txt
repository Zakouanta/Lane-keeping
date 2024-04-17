cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(tutorial_cmake)

include_directories(include)

set(sources src/hellomake.c src/hellofunc.c)

add_executable (main ${sources})


