cmake_minimum_required(VERSION 3.22.1)
project(communication_master_example)

set(CMAKE_CXX_STANDARD 14)

add_executable(communication_master_example main.cpp)

include_directories(openFrameworksArduino/src)