cmake_minimum_required(VERSION 3.22.1)
project(umrt-arm-firmware-lib)

# ********** Setup CMake **********

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        # enables building a static library but later link it into a dynamic library
        add_compile_options(-fPIC)
endif()
if(NOT WIN32)
        # About -Wno-sign-conversion: With Clang, -Wconversion implies -Wsign-conversion. There are a number of
        # implicit sign conversions in gtest.cc, see https://ci.ros2.org/job/ci_osx/9381/clang/.
        # Hence disabling -Wsign-conversion for now until all those have eventually been fixed.
        # (from https://github.com/ros2/rcutils/pull/263#issuecomment-663252537)
        add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic)
endif()

# Setup Boost
find_package(Boost REQUIRED COMPONENTS log log_setup)
include_directories(${Boost_INCLUDE_DIRS})

# Setup openFrameworksArduino
find_package(openFrameworksArduino CONFIG REQUIRED)

# ********** Setup umrt_arm_firmware_lib library **********

set(umrt_arm_firmware_lib_sources
        src/StepperController.cpp
        )
set_source_files_properties(
        ${umrt_arm_firmware_lib_sources}
        PROPERTIES language "CXX")
add_library(
        ${PROJECT_NAME}
        ${umrt_arm_firmware_lib_sources})
target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


target_link_libraries(${PROJECT_NAME} PRIVATE
        ${Boost_LIBRARIES}
        openFrameworksArduino::openFrameworksArduino
        )

set(public_headers
        include/umrt-arm-firmware-lib/StepperController.h
        include/umrt-arm-firmware-lib/SYSEX_COMMANDS.h
        include/umrt-arm-firmware-lib/Utils.h
        )

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${public_headers}")

# ********** Setup communication_test_script executable **********

set(communication_test_target communication_test_script)
set(communication_test_script_sources
        src/CommunicationTestScript.cpp
        )
set_source_files_properties(
        ${communication_test_script_sources}
        PROPERTIES language "CXX")
add_executable(
        ${communication_test_target}
        ${communication_test_script_sources})
target_include_directories(${communication_test_target} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/openFrameworksArduino/openFrameworksArduino/src>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/openFrameworksArduino/openFrameworksArduino/src>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        "$<INSTALL_INTERFACE:include/openFrameworksArduino/openFrameworksArduino/src>")

target_link_libraries(${communication_test_target}
        ${PROJECT_NAME}
        )

set_target_properties(${communication_test_target} PROPERTIES LINKER_LANGUAGE CXX)

# ********** Setup packaging **********

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
        EXPORT "${PROJECT_NAME}Targets"
        COMPONENT ${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )

install(TARGETS ${communication_test_target}
        EXPORT "${communication_test_target}Targets"
        COMPONENT ${communication_test_target}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )

message(STATUS "Components to pack: ${CPACK_COMPONENTS_ALL}")
include(cpack.cmake)