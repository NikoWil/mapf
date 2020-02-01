cmake_minimum_required(VERSION 3.15)
project(mapf)

set(CMAKE_CXX_STANDARD 17)

add_executable(mapf main.cpp pathfinding.cpp pathfinding.h)

TARGET_COMPILE_OPTIONS(mapf PUBLIC -pedantic -Wall -Wextra -Werror)