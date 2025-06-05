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
find_package(Boost REQUIRED COMPONENTS log log_setup filesystem program_options)
include_directories(${Boost_INCLUDE_DIRS})

# Setup openFrameworksArduino
find_package(openFrameworksArduino CONFIG REQUIRED)

find_package(ros2_socketcan CONFIG REQUIRED)

# Fixes intellisense not finding ros2 headers https://youtrack.jetbrains.com/issue/CPP-29747/Certain-ROS2-package-headers-missing-from-Intellisense-when-using-a-Docker-toolchain
# (since it doesn't seem transfer target_include_directories paths for remote toolchains)
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include)

# ********** Setup umrt_arm_firmware_lib library **********

set(lib_target ${PROJECT_NAME})

add_library(${lib_target})

target_sources(${lib_target} PRIVATE # These files will only be available during building
        src/arduino_stepper_controller.cpp
        src/mks_stepper_controller.cpp
        )

# Locate headers
# Using FILE_SET would be much cleaner, but needs CMake 3.23+ and ROS Humble ships with 3.22
set(public_headers # These files will be installed with the library
        include/umrt-arm-firmware-lib/arduino_stepper_controller.hpp
        include/umrt-arm-firmware-lib/mks_stepper_controller.hpp
        include/umrt-arm-firmware-lib/SYSEX_COMMANDS.hpp
        include/umrt-arm-firmware-lib/utils.hpp
        )
set_property(TARGET ${lib_target} PROPERTY PUBLIC_HEADER ${public_headers})
target_include_directories(${lib_target} PUBLIC # Everything in this folder will be available during building
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}>"
        "$<INSTALL_INTERFACE:include/>")

target_link_libraries(${lib_target} PUBLIC
        # Consumers need to know about openFrameworksArduino since we reference it in ArduinoStepperController.h
        openFrameworksArduino::openFrameworksArduino
        Boost::log
        Boost::log_setup
        )

target_link_libraries(${lib_target} PRIVATE
        ${ros2_socketcan_LIBRARIES}
        )
target_include_directories(${lib_target} PRIVATE
        ${ros2_socketcan_INCLUDE_DIRS}
        )

# ********** Setup arduino_communication_test_script executable **********

set(arduino_communication_test_target arduino_communication_test_script)

add_executable(${arduino_communication_test_target})

target_sources(${arduino_communication_test_target} PRIVATE
        src/arduino_communication_test_script.cpp
        src/arduino_communication_test.cpp
        )

target_link_libraries(${arduino_communication_test_target} PRIVATE
        Boost::log_setup
        Boost::log
        Boost::filesystem
        ${lib_target}
        )

# ********** Setup mks_test_script executable **********

set(mks_test_target mks_test_script)

add_executable(${mks_test_target})

target_sources(${mks_test_target} PRIVATE
        src/mks_test_script.cpp
        src/mks_test.cpp
)

target_link_libraries(${mks_test_target} PRIVATE
        Boost::log_setup
        Boost::log
        Boost::filesystem
        Boost::program_options
        ${lib_target}
)

# ********** Setup packaging **********

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Parse version number from the file
file(READ "version" version_input)
string(REGEX MATCH "([0-9]+)\.([0-9]+)\.([0-9]+)" _ ${version_input})

# Fill in cmake variables in config template
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/config.in.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        @ONLY
)

write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        VERSION ${version_input}
        COMPATIBILITY AnyNewerVersion
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
        )

install(
        TARGETS ${lib_target} ${arduino_communication_test_target} ${mks_test_target} # These targets will be packaged
        EXPORT ${PROJECT_NAME}Targets
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(
        EXPORT ${PROJECT_NAME}Targets
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

message(STATUS "Components to pack: ${CPACK_COMPONENTS_ALL}")
include(cpack.cmake)

