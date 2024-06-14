cmake_minimum_required(VERSION 3.22.1)
project(communication_master_example)

set(CMAKE_CXX_STANDARD 14)

# Setup Boost
find_package(Boost REQUIRED COMPONENTS log log_setup)
include_directories(${Boost_INCLUDE_DIRS})

# Setup openFrameworksArduino
add_subdirectory(openFrameworksArduino)

add_executable(communication_master_example main.cpp StepperController.cpp StepperController.h Utils.h)

#target_link_libraries(communication_master_example -static)
target_link_libraries(communication_master_example ${Boost_LIBRARIES})

target_link_libraries(communication_master_example openFrameworksArduino)
#include_directories(openFrameworksArduino/src)
target_include_directories(communication_master_example PUBLIC
        "${PROJECT_BINARY_DIR}"
        "${PROJECT_SOURCE_DIR}/openFrameworksArduino/src"
        )

