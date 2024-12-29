cmake_minimum_required(VERSION 3.22.1)
project(communication_master_example)

set(CMAKE_CXX_STANDARD 14)

# Setup Boost
find_package(Boost REQUIRED COMPONENTS log log_setup)
include_directories(${Boost_INCLUDE_DIRS})

# Setup openFrameworksArduino
add_subdirectory(include/openFrameworksArduino)

# Setup arm_firmware_lib library

add_library(arm_firmware_lib SHARED src/StepperController.cpp)

target_link_libraries(arm_firmware_lib ${Boost_LIBRARIES})
target_link_libraries(arm_firmware_lib openFrameworksArduino)
target_include_directories(arm_firmware_lib PRIVATE
        "${PROJECT_BINARY_DIR}"
        "${PROJECT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/include/openFrameworksArduino/openFrameworksArduino/src"
        )

set_target_properties(arm_firmware_lib PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(arm_firmware_lib PROPERTIES PUBLIC_HEADER "
        include/StepperController.h;
        include/SYSEX_COMMANDS.h;
        include/Utils.h;
        ")

# Setup communication_test_script executable
add_executable(communication_test_script src/CommunicationTestScript.cpp)
target_link_libraries(communication_test_script arm_firmware_lib)
target_include_directories(communication_test_script PRIVATE
        "${PROJECT_BINARY_DIR}"
        "${PROJECT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/include/openFrameworksArduino/openFrameworksArduino/src"
        )

# Setup packages